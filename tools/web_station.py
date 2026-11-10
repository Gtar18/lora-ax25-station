"""
LoRa AX.25 Web Station - ควบคุมทุกบอร์ดผ่านหน้าเว็บ

  python tools/web_station.py                    # หาบอร์ด CP210x/CH340 อัตโนมัติ
  python tools/web_station.py --ports COM5,COM6  # ระบุพอร์ตเอง
  แล้วเปิด http://localhost:8000/web/

- มี terminal ของแต่ละบอร์ดในหน้าเว็บ (พิมพ์ /help)
- ส่งรูป / ping / ตั้งค่า / อัปโหลดรูป ได้จากหน้าเว็บ
- คลิกช่อง packet สีแดง (หาย) เพื่อส่งใหม่เฉพาะ packet นั้น
"""

import argparse
import json
import math
import queue
import random
import re
import struct
import sys
import threading
import time
import zlib
from datetime import datetime
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

sys.path.insert(0, str(Path(__file__).resolve().parent))
from station import (CHUNK, D_HDR, IMG_DIR, LOG_DIR, MAGIC, RECV_DIR, ROOT,  # noqa: E402
                     log_event, parse_rx_line, prepare_image)

import serial  # noqa: E402
import serial.tools.list_ports  # noqa: E402

OUTBOX = LOG_DIR / "outbox"   # ข้อมูลไฟล์ที่ส่งไป (ไว้ส่งซ้ำทีหลัง)
INBOX = LOG_DIR / "inbox"     # สถานะไฟล์ที่รับ (ไว้เติม packet ที่ส่งซ้ำมาทีหลัง)
for d in (OUTBOX, INBOX):
    d.mkdir(exist_ok=True)

IMG_EXT = {".jpg", ".jpeg", ".png", ".bmp", ".gif", ".webp"}

HELP = """── ส่ง / ทดสอบ
  <ข้อความ>                         ส่งข้อความไปยัง dest
  /ping [n]                         ping ไปยัง dest (ค่าเริ่มต้น 4 ครั้ง)
  /send <ไฟล์> [opts]               ส่งรูปจาก images_to_send
      loss=0.2                      ทดสอบ: จงใจทิ้ง packet รอบแรก
      rounds=5                      รอบส่งซ้ำอัตโนมัติ (1 = ไม่ส่งซ้ำเอง)
      px=320  q=60  raw             ย่อรูป / คุณภาพ JPEG / ส่งต้นฉบับ
  /images                           รายการรูปใน images_to_send
  /resend <transfer> <1,5,9|all>    ส่ง packet ที่หายใหม่ (หรือคลิกช่องสีแดง)
  /beacon on|off                    บีคอนอัตโนมัติทุก 10 วินาที
── สถานี
  /mycall CALL-SSID                 ตั้งชื่อบอร์ดนี้
  /dest CALL-SSID                   ตั้งปลายทาง
  /status                           ค่าปัจจุบัน
── วิทยุ (ทุกบอร์ดต้องตั้งให้ตรงกัน)
  /modem lora|fsk|gfsk              เลือก modem
  /bitrate <kbps>                   FSK/GFSK 0.5 - 300 kbps  เช่น /bitrate 9.6
  /dev <kHz>|auto                   FSK/GFSK frequency deviation
  /rxbw <kHz>|auto                  FSK/GFSK receiver bandwidth
  /shaping 0.3|0.5|0.7|1.0          GFSK Gaussian BT
  /sf <6-12>    /bw <kHz>           LoRa spreading factor / bandwidth
  /freq <MHz>                       ความถี่ 410 - 525 MHz
  /power <dBm>                      กำลังส่ง 2 - 17 (20 = PA boost)
  /radio                            แสดงค่าวิทยุ
  /radio reset                      คืนค่าเริ่มต้น (GFSK 9.6 kbps)
  /radioall <คำสั่งวิทยุ>             ใช้กับทุกบอร์ดพร้อมกัน  เช่น /radioall /bitrate 19.2
── อื่นๆ
  /clear                            ล้างหน้าจอ
  /help                             แสดงคำสั่งนี้
  ↑ / ↓                             เรียกคำสั่งเก่า"""

RADIO_CMDS = ("/modem", "/bitrate", "/br", "/dev", "/rxbw", "/shaping", "/freq", "/power", "/sf", "/bw", "/radio")


def safe_name(s):
    return re.sub(r"[^\w.\-]", "_", s)


# ====================================================================== terminal buffer

class Term:
    def __init__(self, keep=3000):
        self.lines, self.next_id, self.keep = [], 0, keep
        self.lock = threading.Lock()

    def add(self, text, kind="out", key=None):
        with self.lock:
            for s in str(text).split("\n"):
                self.lines.append({"id": self.next_id, "t": datetime.now().strftime("%H:%M:%S"),
                                   "k": kind, "s": s, "key": key})
                self.next_id += 1
            if len(self.lines) > self.keep:
                self.lines = self.lines[-self.keep:]

    def since(self, i):
        with self.lock:
            return [l for l in self.lines if l["id"] >= i], self.next_id


# ====================================================================== receive-side transfer

class Transfer:
    def __init__(self, src, fid, key=None):
        self.src, self.fid = src, fid
        self.key = key or f"{src}#{fid:02X}@{datetime.now():%H%M%S}"
        self.name = f"unknown_{fid:02X}.jpg"
        self.total = self.size = self.crc = self.chunk = None
        self.chunks = {}
        self.first_missing = None
        self.nacks = 0
        self.done = False
        self.saved = None
        self.finalized_count = -1
        self.t0 = self.last = time.time()

    def missing(self):
        return [s for s in range(self.total or 0) if s not in self.chunks]

    def to_json(self):
        return {"src": self.src, "fid": self.fid, "key": self.key, "name": self.name, "total": self.total,
                "size": self.size, "crc": self.crc, "chunk": self.chunk, "first_missing": self.first_missing, "nacks": self.nacks,
                "saved": self.saved, "t0": self.t0, "chunks": {str(k): v.hex() for k, v in self.chunks.items()}}

    @classmethod
    def from_json(cls, d):
        t = cls(d["src"], d["fid"], d["key"])
        t.name, t.total, t.size, t.crc = d["name"], d["total"], d["size"], d["crc"]
        t.chunk = d.get("chunk") or CHUNK
        t.first_missing, t.nacks, t.saved, t.t0 = d["first_missing"], d["nacks"], d["saved"], d["t0"]
        t.chunks = {int(k): bytes.fromhex(v) for k, v in d["chunks"].items()}
        t.done, t.finalized_count, t.last = True, len(t.chunks), t.t0
        return t


# ====================================================================== node

class Node:
    def __init__(self, port):
        self.port = port
        self.term = Term()
        self.jobs = queue.Queue()
        self.resp = queue.Queue()     # คำตอบของคำสั่ง (#TX #OK #ERR #STATUS #PING ...)
        self.ctrl = queue.Queue()     # เฟรม K/N ที่ส่งมาหาเรา (ใช้ตอนส่งไฟล์)
        self.mycall = self.dest = "?"
        self.modem, self.radio, self.maxinfo = "?", "", 237
        self.ready = False
        self.busy = None
        self.transfers = {}
        self.rx_lock = threading.Lock()
        self._load_inbox()

        s = serial.Serial()
        s.port, s.baudrate, s.timeout = port, 115200, 0.1
        s.dtr = False
        s.rts = False
        s.open()
        self.ser = s
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._worker, daemon=True).start()
        self.submit(self._init, "init")

    # ---------- infra
    def info(self):
        return {"id": self.port, "mycall": self.mycall, "dest": self.dest, "ready": self.ready,
                "modem": self.modem, "radio": self.radio, "maxinfo": self.maxinfo,
                "busy": self.busy, "queued": self.jobs.qsize()}

    def write(self, line):
        self.ser.write((line + "\r").encode())

    def submit(self, fn, label):
        self.jobs.put((fn, label))

    def _worker(self):
        while True:
            fn, label = self.jobs.get()
            self.busy = label
            try:
                fn()
            except Exception as e:  # ไม่ให้ worker ตาย
                self.term.add(f"[ERR] {label}: {e!r}", "err")
            self.busy = None

    def _reader(self):
        buf = b""
        while True:
            try:
                buf += self.ser.read(512)
            except serial.SerialException as e:
                self.term.add(f"[ERR] พอร์ตหลุด: {e}", "err")
                self.ready = False
                return
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").strip()
                if line:
                    try:
                        self._on_line(line)
                    except Exception as e:
                        self.term.add(f"[ERR] {e!r}", "err")

    def request(self, line, prefixes, timeout):
        while not self.resp.empty():
            self.resp.get_nowait()
        self.write(line)
        end = time.time() + timeout
        while time.time() < end:
            try:
                r = self.resp.get(timeout=max(0.01, end - time.time()))
            except queue.Empty:
                break
            if r.startswith(prefixes):
                return r
        return None

    def send_bin(self, dest, data):
        r = self.request(f"/hexto {dest} {data.hex()}", ("#TX,", "#ERR,"), 15)
        if not r or not r.startswith("#TX,"):
            return False, 0
        p = r.split(",")
        return p[1] == "OK", int(p[3]) if len(p) > 3 else 0

    def _init(self):
        for _ in range(6):
            self.write("")
            if self.request("/machine on", ("#OK,machine",), 1.5):
                break
        else:
            self.term.add(f"[ERR] บอร์ดที่ {self.port} ไม่ตอบ (อัปโหลด LoRa_Node แล้วหรือยัง?)", "err")
            return
        self._refresh_status()
        self.ready = True
        self.term.add(f"เชื่อมต่อ {self.port} แล้ว: mycall={self.mycall} dest={self.dest}   พิมพ์ /help", "sys")
        self.term.add(f"[RADIO] {self.radio}", "sys")

    def _refresh_status(self):
        r = self.request("/status", ("#STATUS,",), 2)
        if r:
            p = r.split(",")
            self.mycall, self.dest, beacon = p[1], p[2], p[3]
            if len(p) >= 7:
                self.modem, self.maxinfo, self.radio = p[4], int(p[5]), ",".join(p[6:])
            return beacon
        return None

    # ---------- incoming lines
    def _on_line(self, line):
        if line.startswith(("#TX,", "#OK,", "#ERR,", "#STATUS,", "#PING,", "#PINGDONE")):
            self.resp.put(line)
        elif line.startswith("#RX,"):
            self._on_rx(line)
        elif line.startswith("#PINGED,"):
            _, src, rssi, snr = line.split(",")
            log_event("receiver", "ping_reply", node=self.port, src=src, rssi=float(rssi), snr=float(snr))
            self.term.add(f"[PING] {src} ping มา  RSSI {rssi} dBm  SNR {snr} dB → ตอบ PONG แล้ว", "rx")
        elif line.startswith("#RXERR"):
            log_event("receiver", "rx_error", node=self.port, detail=line[7:])
            self.term.add(f"[RX] packet เสีย ({line[7:]})", "err")
        elif "=== LoRa AX.25 Node ===" in line:
            self.term.add("[SYS] บอร์ดรีสตาร์ท กำลังเชื่อมต่อใหม่...", "sys")
            self.ready = False
            self.submit(self._init, "init")

    def _on_rx(self, line):
        rssi, snr, fr, err = parse_rx_line(line)
        if not fr:
            log_event("receiver", "rx_bad", node=self.port, reason=err, rssi=rssi, snr=snr)
            self.term.add(f"[RX] เฟรมเสีย ({err})  RSSI {rssi}", "err")
            return
        info = fr["info"]
        for_me = fr["dst"] == self.mycall

        if info[:1] == MAGIC and len(info) >= 3:
            kind, body = info[1:2], info[2:]
            if kind in (b"K", b"N"):
                if for_me:
                    self.ctrl.put((fr["src"], kind, body))
                return
            if for_me:
                with self.rx_lock:
                    self._rx_file(fr["src"], kind, body, rssi, snr)
            return

        text = info.decode("utf-8", "replace")
        log_event("receiver", "rx_frame", node=self.port, src=fr["src"], dst=fr["dst"],
                  rssi=rssi, snr=snr, info=text[:200])
        if text.startswith(("PING ", "PONG ")):
            return
        self.term.add(f"[RX] {fr['src']}>{fr['dst']}: {text}   (RSSI {rssi:.0f} dBm, SNR {snr:.1f} dB)", "rx")

    # ---------- receiving files
    def _rx_file(self, src, kind, body, rssi, snr):
        fid = body[0]
        k = (src, fid)
        tr = self.transfers.get(k)
        if tr is None or (tr.done and kind == b"S"):
            tr = self.transfers[k] = Transfer(src, fid)
        tr.last = time.time()

        if kind == b"S" and len(body) >= 12:
            _, tr.total, tr.size, tr.crc, tr.chunk = struct.unpack(">BHIIB", body[:12])
            tr.name = safe_name(body[12:].decode("utf-8", "replace")) or tr.name
            log_event("receiver", "transfer_start", node=self.port, transfer=tr.key, src=src, file=tr.name,
                      total=tr.total, size=tr.size, rssi=rssi, snr=snr)
            self.term.add(f"[FILE] เริ่มรับ {tr.name} จาก {src}: {tr.size} bytes / {tr.total} packets", "rx")

        elif kind == b"D" and len(body) >= 5:
            _, seq, total = struct.unpack(">BHH", body[:5])
            tr.total = tr.total or total
            late = tr.done
            dup = seq in tr.chunks
            tr.chunks[seq] = body[5:]
            if seq < total - 1:
                tr.chunk = tr.chunk or len(body) - 5
            log_event("receiver", "rx_chunk", node=self.port, transfer=tr.key, seq=seq, total=total,
                      rssi=rssi, snr=snr, dup=dup, after_nack=tr.first_missing is not None)
            label = "ส่งซ้ำ" if late or tr.first_missing is not None else "รับ"
            self.term.add(f"[FILE] {tr.name}: {label} {len(tr.chunks)}/{tr.total}  (#{seq}, RSSI {rssi:.0f})",
                          "prog", key=f"rx-{tr.key}")

        elif kind == b"E" and len(body) >= 3:
            tr.total = tr.total or struct.unpack(">H", body[1:3])[0]
            missing = tr.missing()
            if tr.first_missing is None:
                tr.first_missing = missing
            if tr.done and len(tr.chunks) != tr.finalized_count:
                self._finalize(tr, "resend")
            if not missing:
                self.submit(lambda d=src, f=fid: self.send_bin(d, MAGIC + b"K" + bytes([f])), "reply K")
                if not tr.done:
                    self._finalize(tr, "complete")
            else:
                m = missing[:max(1, (self.maxinfo - 5) // 2)]
                payload = MAGIC + b"N" + bytes([fid]) + struct.pack(f">H{len(m)}H", len(m), *m)
                self.submit(lambda d=src, p=payload: self.send_bin(d, p), "reply N")
                tr.nacks += 1
                log_event("receiver", "nack", node=self.port, transfer=tr.key, missing=missing)
                self.term.add(f"[FILE] {tr.name}: ขาด {len(missing)} packet → ขอส่งซ้ำ {missing[:20]}"
                              f"{' ...' if len(missing) > 20 else ''}", "err")

        elif kind == b"F":
            if not tr.done:
                tr.first_missing = tr.first_missing if tr.first_missing is not None else tr.missing()
                self._finalize(tr, "sender_gave_up")

    def _finalize(self, tr, reason):
        missing = tr.missing()
        data = b"".join(tr.chunks.get(s, b"\x00" * (tr.chunk or CHUNK)) for s in range(tr.total or 0))
        if tr.size:
            data = data[:tr.size]
        crc_ok = tr.crc is not None and not missing and zlib.crc32(data) == tr.crc
        if tr.saved:                                   # ลบไฟล์เก่า (เช่น _INCOMPLETE) ก่อนบันทึกใหม่
            try:
                (RECV_DIR / tr.saved).unlink()
            except OSError:
                pass
        stem, ext = Path(tr.name).stem, Path(tr.name).suffix or ".bin"
        fname = f"{datetime.fromtimestamp(tr.t0):%Y%m%d_%H%M%S}_{stem}{'' if not missing else '_INCOMPLETE'}{ext}"
        if data:
            (RECV_DIR / fname).write_bytes(data)
            tr.saved = fname
        tr.done = True
        tr.finalized_count = len(tr.chunks)
        (INBOX / f"{safe_name(tr.key)}.json").write_text(json.dumps(tr.to_json()), encoding="utf-8")
        log_event("receiver", "transfer_end", node=self.port, transfer=tr.key, src=tr.src, file=tr.name,
                  total=tr.total, received=len(tr.chunks), missing_first=tr.first_missing or [],
                  missing_final=missing, nacks=tr.nacks, crc_ok=crc_ok, saved=tr.saved, reason=reason,
                  duration_s=round(time.time() - tr.t0, 1))
        kind = "rx" if crc_ok else "err"
        self.term.add(f"[FILE] {tr.name}: {'ครบ · CRC OK' if crc_ok else f'ไม่ครบ (หาย {len(missing)})'}"
                      f"  → received_images/{tr.saved}", kind)

    def check_timeouts(self, timeout):
        with self.rx_lock:
            for tr in list(self.transfers.values()):
                if not tr.done and tr.total and time.time() - tr.last > timeout:
                    if tr.first_missing is None:
                        tr.first_missing = tr.missing()
                    self._finalize(tr, "timeout")

    def _load_inbox(self):
        for f in sorted(INBOX.glob("*.json"), key=lambda p: p.stat().st_mtime):
            try:
                t = Transfer.from_json(json.loads(f.read_text(encoding="utf-8")))
                self.transfers[(t.src, t.fid)] = t
            except Exception:
                pass

    def find_transfer(self, key):
        for t in self.transfers.values():
            if t.key == key:
                return t
        return None

    # ---------- commands from web
    def command(self, line):
        line = line.strip()
        if not line:
            return
        self.term.add(f"> {line}", "in")
        if not line.startswith("/"):
            self.submit(lambda: self._job_text(line), "send text")
            return
        parts = line.split()
        cmd, args = parts[0].lower(), parts[1:]

        if cmd == "/help":
            self.term.add(HELP, "help")
        elif cmd == "/images":
            files = list_images()
            self.term.add("\n".join(f"  {f['name']}  ({f['size']} bytes)" for f in files)
                          or "  (ว่าง) วางรูปไว้ในโฟลเดอร์ images_to_send หรืออัปโหลดจากหน้าเว็บ", "sys")
        elif cmd == "/send":
            if not args:
                self.term.add("รูปแบบ: /send <ไฟล์> [loss=0.2] [rounds=5] [px=320] [q=60] [raw]", "err")
                return
            opt = {"loss": 0.0, "rounds": 5, "px": 320, "q": 60, "raw": False}
            names = []
            for a in args:
                if "=" in a:
                    k, v = a.split("=", 1)
                    if k in opt:
                        opt[k] = float(v) if k == "loss" else int(v)
                elif a == "raw":
                    opt["raw"] = True
                else:
                    names.append(a)
            name = " ".join(names)
            path = IMG_DIR / name
            if not path.is_file():
                self.term.add(f"ไม่พบ {name} ใน images_to_send (พิมพ์ /images ดูรายการ)", "err")
                return
            self.submit(lambda: self._job_send_file(path, opt), f"send {name}")
        elif cmd == "/resend":
            if len(args) < 2:
                self.term.add("รูปแบบ: /resend <transfer> <1,5,9|all>", "err")
                return
            seqs = "all" if args[1] == "all" else [int(x) for x in args[1].split(",") if x.strip()]
            msg = STATION.resend(args[0], seqs)
            self.term.add(msg, "sys")
        elif cmd == "/ping":
            n = int(args[0]) if args and args[0].isdigit() else 4
            self.submit(lambda: self._job_ping(n), f"ping x{n}")
        elif cmd in ("/mycall", "/dest", "/beacon"):
            self.submit(lambda: self._job_set(line), cmd)
        elif cmd == "/radio" and not args:
            self.submit(self._job_status, "status")
        elif cmd in RADIO_CMDS:
            self.submit(lambda: self._job_set(line), cmd)
        elif cmd == "/radioall":
            sub = " ".join(args)
            if not sub.split() or sub.split()[0].lower() not in RADIO_CMDS:
                self.term.add("รูปแบบ: /radioall /bitrate 19.2   (ใช้ได้กับคำสั่งวิทยุเท่านั้น)", "err")
                return
            for n in STATION.nodes.values():
                if n is not self:
                    n.term.add(f"> {sub}   (จาก /radioall ที่ {self.port})", "in")
                n.submit(lambda n=n: n._job_set(sub), sub.split()[0])
            self.term.add(f"[RADIO] ส่งคำสั่ง {sub} ไปทุกบอร์ด ({len(STATION.nodes)})", "sys")
        elif cmd == "/status":
            self.submit(self._job_status, "status")
        elif cmd == "/clear":
            pass  # หน้าเว็บล้างเอง
        else:
            self.term.add("ไม่รู้จักคำสั่ง พิมพ์ /help", "err")

    # ---------- jobs (worker thread)
    def _job_text(self, text):
        r = self.request(text[:237], ("#TX,", "#ERR,"), 10)
        if r and r.startswith("#TX,OK"):
            self.term.add(f"[TX] {self.mycall}>{self.dest}: {text}   ({r.split(',')[3]} ms)", "tx")
        else:
            self.term.add(f"[TX] ส่งไม่สำเร็จ ({r})", "err")

    def _job_status(self):
        b = self._refresh_status()
        self.term.add(f"[INFO] mycall={self.mycall}  dest={self.dest}  beacon={b}  port={self.port}", "sys")
        self.term.add(f"[RADIO] {self.radio}   (ข้อมูลสูงสุด {self.maxinfo} bytes/เฟรม)", "sys")

    def _job_set(self, line):
        r = self.request(line, ("#OK,", "#ERR,"), 4)
        if r and r.startswith("#OK"):
            self._job_status()
        else:
            self.term.add(f"[ERR] {r[5:] if r else 'บอร์ดไม่ตอบ'}", "err")

    def _job_ping(self, n):
        while not self.resp.empty():
            self.resp.get_nowait()
        self.write(f"/ping {n}")
        self.term.add(f"[PING] {self.mycall} → {self.dest}  x{n}", "tx")
        end = time.time() + n * 6 + 5
        while time.time() < end:
            try:
                line = self.resp.get(timeout=1)
            except queue.Empty:
                continue
            if line.startswith("#PING,"):
                p = line.split(",")
                if p[2] == "OK":
                    log_event("sender", "ping", node=self.port, seq=int(p[1]), ok=True, rtt_ms=int(p[3]),
                              rssi_remote=float(p[4]), snr_remote=float(p[5]),
                              rssi_local=float(p[6]), snr_local=float(p[7]), dest=p[8])
                    self.term.add(f"[PING] #{p[1]} ตอบกลับ  rtt={p[3]} ms   ปลายทางได้ยินเรา {p[4]} dBm/{p[5]} dB"
                                  f"   เราได้ยิน {p[6]} dBm/{p[7]} dB", "rx")
                else:
                    log_event("sender", "ping", node=self.port, seq=int(p[1]), ok=False, reason=p[2], dest=p[-1])
                    self.term.add(f"[PING] #{p[1]} {p[2]}", "err")
            elif line.startswith("#PINGDONE,"):
                _, ok, cnt = line.split(",")
                self.term.add(f"[PING] สรุป ตอบกลับ {ok}/{cnt}", "sys")
                return
        self.term.add("[PING] หมดเวลา", "err")

    def _wait_reply(self, dest, fid, timeout):
        end = time.time() + timeout
        while time.time() < end:
            try:
                src, kind, body = self.ctrl.get(timeout=max(0.05, end - time.time()))
            except queue.Empty:
                break
            if src != dest or not body or body[0] != fid:
                continue
            if kind == b"K":
                return ("K", [])
            n = struct.unpack(">H", body[1:3])[0]
            return ("N", list(struct.unpack(f">{n}H", body[3:3 + 2 * n])))
        return None

    def _send_end(self, dest, fid, total):
        for attempt in range(3):
            self.send_bin(dest, MAGIC + b"E" + struct.pack(">BH", fid, total))
            reply = self._wait_reply(dest, fid, 8)
            if reply:
                return reply
            self.term.add("[FILE] ตัวรับไม่ตอบ ส่ง END ซ้ำ...", "err")
        return None

    def _job_send_file(self, path, opt):
        dest = self.dest
        data, name = prepare_image(path, opt["px"], opt["q"], opt["raw"])
        C = self.maxinfo - D_HDR
        total = math.ceil(len(data) / C)
        fid = random.randint(0, 255)
        crc = zlib.crc32(data)
        xfer = f"{self.mycall}#{fid:02X}@{datetime.now():%H%M%S}"
        meta = {"transfer": xfer, "src": self.mycall, "dest": dest, "fid": fid, "file": name,
                "total": total, "chunk": C, "size": len(data), "crc": crc}
        (OUTBOX / f"{safe_name(self.mycall)}_{fid:02X}.bin").write_bytes(data)
        (OUTBOX / f"{safe_name(self.mycall)}_{fid:02X}.json").write_text(json.dumps(meta), encoding="utf-8")

        self.term.add(f"[FILE] ส่ง {path.name} → {name}  {len(data)} bytes, {total} packets → {dest}"
                      f"  [{self.radio}]{'  [ทดสอบ loss=' + str(opt['loss']) + ']' if opt['loss'] else ''}", "tx")
        log_event("sender", "transfer_start", node=self.port, transfer=xfer, fid=fid, file=name,
                  src_file=path.name, size=len(data), total=total, dest=dest, chunk=C, radio=self.radio)
        while not self.ctrl.empty():
            self.ctrl.get_nowait()
        t0 = time.time()

        def chunk(seq):
            return MAGIC + b"D" + struct.pack(">BHH", fid, seq, total) + data[seq * C:(seq + 1) * C]

        self.send_bin(dest, MAGIC + b"S" + struct.pack(">BHIIB", fid, total, len(data), crc, C)
                      + name.encode()[:self.maxinfo - 15])
        pending, rnd, first_missing, result = list(range(total)), 1, None, "fail"
        while True:
            for i, seq in enumerate(pending):
                if rnd == 1 and opt["loss"] and random.random() < opt["loss"]:
                    log_event("sender", "tx_chunk", node=self.port, transfer=xfer, seq=seq, total=total,
                              round=rnd, ok=True, airtime_ms=0, simulated_drop=True)
                else:
                    ok, air = self.send_bin(dest, chunk(seq))
                    log_event("sender", "tx_chunk", node=self.port, transfer=xfer, seq=seq, total=total,
                              round=rnd, ok=ok, airtime_ms=air)
                self.term.add(f"[FILE] รอบ {rnd}: ส่ง {i + 1}/{len(pending)}", "prog", key=f"tx-{xfer}-{rnd}")
            reply = self._send_end(dest, fid, total)
            if reply is None:
                result = "no_reply"
                self.term.add("[FILE] ตัวรับไม่ตอบ ยกเลิก", "err")
                break
            if reply[0] == "K":
                result = "ok"
                first_missing = first_missing if first_missing is not None else []
                break
            missing = reply[1]
            if first_missing is None:
                first_missing = missing
            log_event("sender", "nack", node=self.port, transfer=xfer, round=rnd, missing=missing)
            self.term.add(f"[FILE] ตัวรับขาด {len(missing)} packet: {missing[:20]}"
                          f"{' ...' if len(missing) > 20 else ''}", "err")
            rnd += 1
            if rnd > opt["rounds"]:
                result = "gave_up"
                self.send_bin(dest, MAGIC + b"F" + bytes([fid]))   # บอกตัวรับให้ปิดไฟล์
                self.term.add("[FILE] ครบจำนวนรอบแล้ว หยุดส่ง  → คลิกช่องสีแดงในหน้าเว็บเพื่อส่งใหม่", "err")
                break
            pending = missing

        dur = time.time() - t0
        log_event("sender", "transfer_end", node=self.port, transfer=xfer, file=name, total=total, result=result,
                  rounds=rnd, missing_first=first_missing or [], duration_s=round(dur, 1),
                  bytes_per_s=round(len(data) / dur, 1))
        self.term.add(f"[FILE] ผล: {result}  ({dur:.1f} s, {len(data) / dur:.0f} B/s)",
                      "rx" if result == "ok" else "err")

    def _job_resend(self, meta, data, seqs, rx_key):
        dest, fid, total = meta["dest"], meta["fid"], meta["total"]
        C = meta.get("chunk", CHUNK)
        if C + D_HDR > self.maxinfo:
            self.term.add(f"[RESEND] ไฟล์นี้ส่งด้วย packet {C + D_HDR} bytes แต่ modem ตอนนี้รับได้ {self.maxinfo}"
                          f" → เปลี่ยน modem กลับเป็นแบบเดิมก่อน", "err")
            return
        self.term.add(f"[RESEND] {meta['file']}: ส่งใหม่ {len(seqs)} packet {seqs[:20]}"
                      f"{' ...' if len(seqs) > 20 else ''} → {dest}", "tx")
        while not self.ctrl.empty():
            self.ctrl.get_nowait()
        pkey = f"rs-{rx_key}-{time.time():.0f}"
        for i, seq in enumerate(seqs):
            pkt = MAGIC + b"D" + struct.pack(">BHH", fid, seq, total) + data[seq * C:(seq + 1) * C]
            ok, air = self.send_bin(dest, pkt)
            log_event("sender", "tx_chunk", node=self.port, transfer=meta["transfer"], seq=seq, total=total,
                      round="manual", ok=ok, airtime_ms=air)
            self.term.add(f"[RESEND] ส่ง {i + 1}/{len(seqs)}", "prog", key=pkey)
        reply = self._send_end(dest, fid, total)
        if reply is None:
            res = "no_reply"
            self.term.add("[RESEND] ตัวรับไม่ตอบ", "err")
        elif reply[0] == "K":
            res = "ok"
            self.term.add("[RESEND] ตัวรับได้ครบแล้ว ✓", "rx")
        else:
            res = "still_missing"
            self.term.add(f"[RESEND] ตัวรับยังขาด {len(reply[1])} packet: {reply[1][:20]}", "err")
        log_event("sender", "resend", node=self.port, transfer=meta["transfer"], rx_transfer=rx_key,
                  seqs=seqs, result=res)


# ====================================================================== station (all nodes)

def list_images():
    return [{"name": p.name, "size": p.stat().st_size}
            for p in sorted(IMG_DIR.iterdir()) if p.is_file() and p.suffix.lower() in IMG_EXT]


class Station:
    def __init__(self, ports, timeout):
        self.nodes = {}
        for p in ports:
            try:
                self.nodes[p] = Node(p)
                print(f"เปิด {p} แล้ว")
            except serial.SerialException as e:
                print(f"เปิด {p} ไม่ได้: {e}  (ปิด PuTTY / Serial Monitor ก่อน)")
        self.timeout = timeout
        threading.Thread(target=self._tick, daemon=True).start()

    def _tick(self):
        while True:
            time.sleep(1)
            for n in self.nodes.values():
                n.check_timeouts(self.timeout)

    def resend(self, rx_key, seqs):
        try:
            src, rest = rx_key.split("#", 1)
            fid = int(rest[:2], 16)
        except ValueError:
            return f"[ERR] transfer ไม่ถูกต้อง: {rx_key}"
        sender = next((n for n in self.nodes.values() if n.mycall == src), None)
        if not sender:
            return f"[ERR] ไม่มีบอร์ด {src} ต่ออยู่กับเครื่องนี้ จึงส่งใหม่ไม่ได้"
        mf, df = OUTBOX / f"{safe_name(src)}_{fid:02X}.json", OUTBOX / f"{safe_name(src)}_{fid:02X}.bin"
        if not mf.exists():
            return f"[ERR] ไม่พบข้อมูลไฟล์ต้นฉบับของ {rx_key} ในเครื่อง"
        meta, data = json.loads(mf.read_text(encoding="utf-8")), df.read_bytes()
        if seqs == "all":
            tr = next((t for n in self.nodes.values() for t in [n.find_transfer(rx_key)] if t), None)
            if not tr:
                return "[ERR] ไม่พบสถานะการรับของไฟล์นี้"
            seqs = tr.missing()
        seqs = sorted({s for s in seqs if 0 <= s < meta["total"]})
        if not seqs:
            return "ไม่มี packet ที่ต้องส่งใหม่"
        sender.submit(lambda: sender._job_resend(meta, data, seqs, rx_key), f"resend {len(seqs)}")
        if sender.busy:
            sender.term.add(f"[RESEND] เข้าคิวแล้ว (กำลังทำ: {sender.busy})", "sys")
        return f"สั่ง {sender.port} ({src}) ส่งใหม่ {len(seqs)} packet: {seqs[:20]}{' ...' if len(seqs) > 20 else ''}"


STATION: Station = None


# ====================================================================== HTTP

class Handler(SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def _json(self, obj, code=200):
        b = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _body(self):
        n = int(self.headers.get("Content-Length", 0))
        return self.rfile.read(n) if n else b""

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            self.send_response(302)
            self.send_header("Location", "/web/")
            self.end_headers()
            return
        if not u.path.startswith("/api/"):
            return super().do_GET()
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        if u.path == "/api/nodes":
            return self._json([n.info() for n in STATION.nodes.values()])
        if u.path == "/api/term":
            n = STATION.nodes.get(q.get("node"))
            if not n:
                return self._json({"error": "no node"}, 404)
            lines, nxt = n.term.since(int(q.get("since", 0)))
            return self._json({"lines": lines, "next": nxt, "node": n.info()})
        if u.path == "/api/images":
            return self._json(list_images())
        self._json({"error": "not found"}, 404)

    def do_POST(self):
        u = urlparse(self.path)
        if u.path == "/api/cmd":
            d = json.loads(self._body() or b"{}")
            n = STATION.nodes.get(d.get("node"))
            if not n:
                return self._json({"error": "no node"}, 404)
            n.command(d.get("line", ""))
            return self._json({"ok": True})
        if u.path == "/api/resend":
            d = json.loads(self._body() or b"{}")
            return self._json({"message": STATION.resend(d.get("transfer", ""), d.get("seqs", []))})
        if u.path == "/api/upload":
            name = safe_name(unquote(parse_qs(u.query).get("name", ["upload.jpg"])[0]))
            if Path(name).suffix.lower() not in IMG_EXT:
                return self._json({"error": "รองรับเฉพาะไฟล์รูป"}, 400)
            data = self._body()
            if len(data) > 20 * 1024 * 1024:
                return self._json({"error": "ไฟล์ใหญ่เกิน 20MB"}, 400)
            (IMG_DIR / name).write_bytes(data)
            return self._json({"ok": True, "name": name})
        self._json({"error": "not found"}, 404)


def autodetect():
    return [p.device for p in serial.tools.list_ports.comports() if p.vid in (0x10C4, 0x1A86)]


def main():
    global STATION
    ap = argparse.ArgumentParser(description="LoRa AX.25 Web Station")
    ap.add_argument("--ports", help="เช่น COM5,COM6 (ไม่ใส่ = หาอัตโนมัติ)")
    ap.add_argument("--web-port", type=int, default=8000)
    ap.add_argument("--timeout", type=float, default=30, help="วินาทีที่ไม่มีข้อมูลก่อนปิดไฟล์ฝั่งรับ")
    args = ap.parse_args()
    ports = args.ports.split(",") if args.ports else autodetect()
    if not ports:
        sys.exit("ไม่พบบอร์ด (เสียบ USB แล้วหรือยัง?)")
    STATION = Station(ports, args.timeout)
    if not STATION.nodes:
        sys.exit("เปิดพอร์ตไม่ได้เลย")
    srv = ThreadingHTTPServer(("127.0.0.1", args.web_port), partial(Handler, directory=str(ROOT)))
    print(f"เปิดหน้าเว็บ: http://localhost:{args.web_port}/web/   (Ctrl+C เพื่อหยุด)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("หยุด")


if __name__ == "__main__":
    main()
