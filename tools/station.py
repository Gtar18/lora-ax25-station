"""
LoRa AX.25 station - ส่งรูป / ping / รับรูป / หน้าเว็บดู log

ใช้งาน (รันจากโฟลเดอร์ sx1278):
  python tools/station.py rx   --port COM6            # ตัวรับ: รับรูป + เปิดหน้าเว็บ http://localhost:8000/web/
  python tools/station.py send --port COM5 cat.jpg    # ส่งรูปจากโฟลเดอร์ images_to_send
  python tools/station.py send --port COM5 --all      # ส่งทุกรูปในโฟลเดอร์ images_to_send
  python tools/station.py ping --port COM5 -n 10      # ping 10 ครั้ง
  python tools/station.py web                         # เปิดหน้าเว็บอย่างเดียว

โปรโตคอลรูป (อยู่ใน info field ของเฟรม AX.25 ทุกแพ็กเก็ตขึ้นต้นด้วยไบต์ 0xA5):
  S fid total(2) size(4) crc32(4) chunk(1) name   เริ่มไฟล์
  D fid seq(2) total(2) data(<=chunk)              ข้อมูลแต่ละชิ้น (LoRa 230 / FSK 39 bytes)
  E fid total(2)                           จบรอบ -> ตัวรับตอบ K (ครบ) หรือ N (รายการที่หาย)
  N fid count(2) seq(2)...                 ขอชิ้นที่หาย  -> ตัวส่งส่งซ้ำเฉพาะชิ้นนั้น
  K fid                                    ได้ครบแล้ว
"""

import argparse
import http.server
import io
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
from pathlib import Path

try:
    import serial
except ImportError:
    sys.exit("ต้องติดตั้ง pyserial ก่อน:  python -m pip install pyserial")

for _s in (sys.stdout, sys.stderr):   # ให้พิมพ์ภาษาไทยได้แม้ terminal / pipe เป็น cp1252
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

ROOT = Path(__file__).resolve().parent.parent
IMG_DIR = ROOT / "images_to_send"
RECV_DIR = ROOT / "received_images"
LOG_DIR = ROOT / "logs"
LOG_FILE = LOG_DIR / "events.jsonl"
CHUNK = 200       # ค่าสำรอง (ขนาดจริงขึ้นกับ modem: maxinfo - 7)
D_HDR = 7         # MAGIC + "D" + fid + seq(2) + total(2)
MAGIC = b"\xa5"   # ไบต์นำหน้าแพ็กเก็ตรูป (ไม่ใช่ตัวอักษร จึงไม่ชนกับข้อความที่พิมพ์)
for d in (IMG_DIR, RECV_DIR, LOG_DIR):
    d.mkdir(exist_ok=True)

_log_lock = threading.Lock()


def log_event(station, event, **kw):
    rec = {"t": datetime.now().isoformat(timespec="milliseconds"), "station": station, "event": event}
    rec.update(kw)
    with _log_lock, open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    return rec


# ---------------------------------------------------------------- AX.25

def ax25_fcs(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return ~crc & 0xFFFF


def _call(a: bytes) -> str:
    c = "".join(chr(x >> 1) for x in a[:6]).strip()
    ssid = (a[6] >> 1) & 0x0F
    return f"{c}-{ssid}" if ssid else c


def parse_ax25(f: bytes):
    if len(f) < 18:
        return None, "short"
    if ax25_fcs(f[:-2]) != (f[-2] | f[-1] << 8):
        return None, "fcs"
    p, addrs = 0, []
    while p + 7 <= len(f) - 2:
        addrs.append(_call(f[p:p + 7]))
        last = f[p + 6] & 1
        p += 7
        if last:
            break
    else:
        return None, "addr"
    if len(addrs) < 2 or p + 2 > len(f) - 2:
        return None, "addr"
    return {"dst": addrs[0], "src": addrs[1], "path": addrs[2:],
            "ctrl": f[p], "pid": f[p + 1], "info": f[p + 2:-2]}, None


# ---------------------------------------------------------------- serial node

class Node:
    """คุยกับบอร์ด LoRa_Node ในโหมด machine"""

    def __init__(self, port):
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = port, 115200, 0.1
        s.dtr = False
        s.rts = False
        try:
            s.open()
        except serial.SerialException as e:
            sys.exit(f"เปิด {port} ไม่ได้: {e}\n(ถ้าเปิด PuTTY / Serial Monitor ค้างไว้ ให้ปิดก่อน)")
        self.ser = s
        self.q = queue.Queue()
        self.stash = []
        threading.Thread(target=self._reader, daemon=True).start()
        self.mycall = None
        self.maxinfo = 237
        for _ in range(5):
            self.write("")
            self.write("/machine on")
            if self.wait_for("#OK,machine", 1.5):
                break
        else:
            sys.exit(f"บอร์ดที่ {port} ไม่ตอบ (อัปโหลด LoRa_Node แล้วหรือยัง?)")
        self.write("/status")
        st = self.wait_for("#STATUS,", 2)
        if st:
            p = st.split(",")
            self.mycall, self.dest = p[1], p[2]
            if len(p) > 5:
                self.maxinfo = int(p[5])

    def _reader(self):
        buf = b""
        while True:
            try:
                buf += self.ser.read(512)
            except serial.SerialException:
                self.q.put("#PORTCLOSED")
                return
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.q.put(line.decode("utf-8", "replace").strip())

    def write(self, line):
        self.ser.write((line + "\r").encode())

    def get(self, timeout):
        if self.stash:
            return self.stash.pop(0)
        try:
            return self.q.get(timeout=timeout)
        except queue.Empty:
            return None

    def wait_for(self, prefix, timeout):
        """รอบรรทัดที่ขึ้นต้นด้วย prefix; บรรทัดอื่นเก็บไว้ใน stash ให้คนอื่นอ่านต่อ"""
        end, other = time.time() + timeout, []
        try:
            while time.time() < end:
                try:
                    line = self.q.get(timeout=max(0.01, end - time.time()))
                except queue.Empty:
                    break
                if line.startswith(prefix):
                    return line
                if line.startswith("#"):
                    other.append(line)
            return None
        finally:
            self.stash.extend(other)

    def send_bin(self, dest, data: bytes, timeout=15):
        self.write(f"/hexto {dest} {data.hex()}")
        r = self.wait_for("#TX,", timeout)
        if not r:
            return False, 0
        parts = r.split(",")
        return parts[1] == "OK", int(parts[3]) if len(parts) > 3 else 0


def parse_rx_line(line):
    """'#RX,rssi,snr,HEX' -> (rssi, snr, frame dict | None, err)"""
    try:
        _, rssi, snr, hx = line.split(",", 3)
        raw = bytes.fromhex(hx)
    except ValueError:
        return None, None, None, "parse"
    fr, err = parse_ax25(raw)
    return float(rssi), float(snr), fr, err


# ---------------------------------------------------------------- image prep

def prepare_image(path: Path, max_px, quality, raw):
    data = path.read_bytes()
    if raw:
        return data, path.name
    try:
        from PIL import Image
    except ImportError:
        print("  (ไม่มี Pillow ส่งไฟล์ตามต้นฉบับ)")
        return data, path.name
    try:
        im = Image.open(io.BytesIO(data))
    except Exception:
        return data, path.name  # ไม่ใช่รูป ส่งตามเดิม
    im = im.convert("RGB")
    im.thumbnail((max_px, max_px))
    out = io.BytesIO()
    im.save(out, "JPEG", quality=quality, optimize=True)
    return out.getvalue(), path.stem + ".jpg"


# ---------------------------------------------------------------- SEND

def cmd_send(args):
    files = []
    if args.all:
        files = sorted(p for p in IMG_DIR.iterdir() if p.is_file() and not p.name.startswith("."))
    for f in args.files:
        p = Path(f)
        if not p.exists():
            p = IMG_DIR / f
        if not p.exists():
            sys.exit(f"ไม่พบไฟล์ {f} (วางรูปไว้ใน {IMG_DIR})")
        files.append(p)
    if not files:
        sys.exit(f"ไม่มีไฟล์จะส่ง วางรูปไว้ใน {IMG_DIR} แล้วใช้ --all หรือระบุชื่อไฟล์")

    node = Node(args.port)
    dest = args.dest or node.dest
    print(f"[{node.mycall}] -> {dest}")
    for p in files:
        send_one(node, dest, p, args)


def send_one(node, dest, path, args):
    data, name = prepare_image(path, args.max_px, args.quality, args.raw)
    C = node.maxinfo - D_HDR
    total = math.ceil(len(data) / C)
    fid = random.randint(0, 255)
    crc = zlib.crc32(data)
    xfer = f"{node.mycall}#{fid:02X}@{datetime.now():%H%M%S}"
    est = total * 1.6
    print(f"\n=== ส่ง {path.name} -> {name}  {len(data)} bytes, {total} packets (~{est:.0f} s) ===")
    log_event("sender", "transfer_start", transfer=xfer, fid=fid, file=name, src_file=path.name,
              size=len(data), total=total, dest=dest)
    t0 = time.time()

    def tx(payload, kind, seq=None, rnd=None):
        ok, air = node.send_bin(dest, payload)
        if kind == "D":
            log_event("sender", "tx_chunk", transfer=xfer, seq=seq, total=total, round=rnd, ok=ok, airtime_ms=air)
        return ok

    def chunk(seq):
        return MAGIC + b"D" + struct.pack(">BHH", fid, seq, total) + data[seq * C:(seq + 1) * C]

    tx(MAGIC + b"S" + struct.pack(">BHIIB", fid, total, len(data), crc, C) + name.encode()[:node.maxinfo - 15], "S")

    pending = list(range(total))
    first_missing = None
    rnd = 1
    result = "fail"
    while True:
        for i, seq in enumerate(pending):
            if rnd == 1 and args.simulate_loss and random.random() < args.simulate_loss:
                log_event("sender", "tx_chunk", transfer=xfer, seq=seq, total=total, round=rnd,
                          ok=True, airtime_ms=0, simulated_drop=True)
                continue   # ทดสอบ: จงใจไม่ส่ง เพื่อดูการส่งซ้ำ
            tx(chunk(seq), "D", seq, rnd)
            print(f"\r  round {rnd}: {i + 1}/{len(pending)}", end="", flush=True)
        print()

        reply = None
        for attempt in range(3):
            tx(MAGIC + b"E" + struct.pack(">BH", fid, total), "E")
            reply = wait_reply(node, dest, fid, 8)
            if reply:
                break
            print("  ไม่มีคำตอบ ส่ง END ซ้ำ...")
        if reply is None:
            print("  ตัวรับไม่ตอบ ยกเลิก")
            result = "no_reply"
            break
        if reply[0] == "K":
            result = "ok"
            if first_missing is None:
                first_missing = []
            break
        missing = reply[1]
        if first_missing is None:
            first_missing = missing
        log_event("sender", "nack", transfer=xfer, round=rnd, missing=missing)
        print(f"  ตัวรับขาด {len(missing)} packet: {missing[:20]}{' ...' if len(missing) > 20 else ''}")
        rnd += 1
        if rnd > args.max_rounds:
            result = "gave_up"
            break
        pending = missing

    dur = time.time() - t0
    log_event("sender", "transfer_end", transfer=xfer, file=name, total=total, result=result,
              rounds=rnd, missing_first=first_missing or [], duration_s=round(dur, 1),
              bytes_per_s=round(len(data) / dur, 1))
    print(f"  ผล: {result}  ({dur:.1f} s, {len(data) / dur:.0f} B/s)")


def wait_reply(node, dest, fid, timeout):
    end = time.time() + timeout
    while time.time() < end:
        line = node.get(max(0.05, end - time.time()))
        if not line or not line.startswith("#RX,"):
            continue
        rssi, snr, fr, err = parse_rx_line(line)
        if not fr or fr["src"] != dest or fr["info"][:1] != MAGIC or fr["info"][2:3] != bytes([fid]):
            continue
        kind, body = fr["info"][1:2], fr["info"][2:]
        if kind == b"K":
            return ("K", [])
        if kind == b"N":
            n = struct.unpack(">H", body[1:3])[0]
            return ("N", list(struct.unpack(f">{n}H", body[3:3 + 2 * n])))
    return None


# ---------------------------------------------------------------- PING

def cmd_ping(args):
    node = Node(args.port)
    if args.dest:
        node.write(f"/dest {args.dest}")
        node.wait_for("#OK,dest", 2)
        node.dest = args.dest
    print(f"PING {node.mycall} -> {node.dest}  x{args.n}")
    node.write(f"/ping {args.n}")
    end = time.time() + args.n * 6 + 5
    while time.time() < end:
        line = node.get(1)
        if not line:
            continue
        if line.startswith("#PING,"):
            p = line.split(",")
            if p[2] == "OK":
                rec = log_event("sender", "ping", seq=int(p[1]), ok=True, rtt_ms=int(p[3]),
                                rssi_remote=float(p[4]), snr_remote=float(p[5]),
                                rssi_local=float(p[6]), snr_local=float(p[7]), dest=p[8])
                print(f"  #{p[1]} rtt={p[3]} ms  ปลายทางได้ยิน {p[4]} dBm/{p[5]} dB  เราได้ยิน {p[6]} dBm/{p[7]} dB")
            else:
                log_event("sender", "ping", seq=int(p[1]), ok=False, reason=p[2], dest=p[-1])
                print(f"  #{p[1]} {p[2]}")
        elif line.startswith("#PINGDONE,"):
            _, okc, cnt = line.split(",")
            print(f"สรุป: ตอบกลับ {okc}/{cnt}")
            return


# ---------------------------------------------------------------- RX daemon

class Transfer:
    def __init__(self, src, fid):
        self.src, self.fid = src, fid
        self.key = f"{src}#{fid:02X}@{datetime.now():%H%M%S}"
        self.name = f"unknown_{fid:02X}.jpg"
        self.total = None
        self.chunk = None
        self.size = None
        self.crc = None
        self.chunks = {}
        self.first_missing = None
        self.nacks = 0
        self.done = False
        self.saved = None
        self.t0 = self.last = time.time()


def cmd_rx(args):
    if not args.no_web:
        start_web(args.web_port)
    node = Node(args.port)
    print(f"[{node.mycall}] กำลังฟัง...  รูปที่ได้จะอยู่ใน {RECV_DIR}")
    transfers = {}

    def reply(dst, payload):
        ok, _ = node.send_bin(dst, payload)
        return ok

    def finalize(tr, reason):
        missing = [s for s in range(tr.total or 0) if s not in tr.chunks]
        data = b"".join(tr.chunks.get(s, b"\x00" * (tr.chunk or CHUNK)) for s in range(tr.total or 0))
        if tr.size:
            data = data[:tr.size]
        crc_ok = tr.crc is not None and not missing and zlib.crc32(data) == tr.crc
        stem, ext = Path(tr.name).stem, Path(tr.name).suffix or ".bin"
        fname = f"{datetime.now():%Y%m%d_%H%M%S}_{stem}{'' if not missing else '_INCOMPLETE'}{ext}"
        if data:
            (RECV_DIR / fname).write_bytes(data)
            tr.saved = fname
        tr.done = True
        log_event("receiver", "transfer_end", transfer=tr.key, src=tr.src, file=tr.name, total=tr.total,
                  received=len(tr.chunks), missing_first=tr.first_missing or [], missing_final=missing,
                  nacks=tr.nacks, crc_ok=crc_ok, saved=tr.saved, reason=reason,
                  duration_s=round(time.time() - tr.t0, 1))
        print(f"\n  [{tr.name}] {reason}: ได้ {len(tr.chunks)}/{tr.total}  "
              f"หายรอบแรก {len(tr.first_missing or [])}  CRC {'OK' if crc_ok else 'FAIL'}  -> {tr.saved}")

    while True:
        line = node.get(0.5)
        now = time.time()
        for tr in list(transfers.values()):
            if not tr.done and tr.total and now - tr.last > args.timeout:
                finalize(tr, "timeout")
        if not line:
            continue
        if line == "#PORTCLOSED":
            sys.exit("พอร์ตถูกปิด")
        if line.startswith("#PINGED,"):
            _, src, rssi, snr = line.split(",")
            log_event("receiver", "ping_reply", src=src, rssi=float(rssi), snr=float(snr))
            print(f"  ping จาก {src}  RSSI {rssi}")
            continue
        if line.startswith("#RXERR"):
            log_event("receiver", "rx_error", detail=line[7:])
            continue
        if not line.startswith("#RX,"):
            continue

        rssi, snr, fr, err = parse_rx_line(line)
        if not fr:
            log_event("receiver", "rx_bad", reason=err, rssi=rssi, snr=snr)
            continue
        info = fr["info"]
        if fr["dst"] != node.mycall:
            log_event("receiver", "rx_frame", src=fr["src"], dst=fr["dst"], rssi=rssi, snr=snr,
                      info=info.decode("latin-1")[:80], note="not for me")
            continue
        kind, body = info[1:2], info[2:]

        if info[:1] == MAGIC and kind in (b"S", b"D", b"E") and len(body) >= 3:
            fid = body[0]
            k = (fr["src"], fid)
            tr = transfers.get(k)
            if tr is None or (tr.done and kind == b"S"):
                tr = transfers[k] = Transfer(fr["src"], fid)
            tr.last = now

            if kind == b"S":
                _, tr.total, tr.size, tr.crc, tr.chunk = struct.unpack(">BHIIB", body[:12])
                tr.name = re.sub(r"[^\w.\-]", "_", body[12:].decode("utf-8", "replace")) or tr.name
                log_event("receiver", "transfer_start", transfer=tr.key, src=tr.src, file=tr.name,
                          total=tr.total, size=tr.size, rssi=rssi, snr=snr)
                print(f"\n>> เริ่มรับ {tr.name} จาก {tr.src}: {tr.size} bytes / {tr.total} packets")

            elif kind == b"D":
                _, seq, total = struct.unpack(">BHH", body[:5])
                tr.total = tr.total or total
                dup = seq in tr.chunks
                tr.chunks[seq] = body[5:]
                if seq < total - 1:
                    tr.chunk = tr.chunk or len(body) - 5
                log_event("receiver", "rx_chunk", transfer=tr.key, seq=seq, total=total, rssi=rssi,
                          snr=snr, dup=dup, after_nack=tr.first_missing is not None)
                print(f"\r  {tr.name}: {len(tr.chunks)}/{tr.total}  RSSI {rssi:.0f}", end="", flush=True)

            elif kind == b"E":
                tr.total = tr.total or struct.unpack(">H", body[1:3])[0]
                missing = [s for s in range(tr.total) if s not in tr.chunks]
                if tr.first_missing is None:
                    tr.first_missing = missing
                if not missing:
                    reply(tr.src, MAGIC + b"K" + bytes([fid]))
                    if not tr.done:
                        finalize(tr, "complete")
                else:
                    m = missing[:max(1, (node.maxinfo - 5) // 2)]
                    reply(tr.src, MAGIC + b"N" + bytes([fid]) + struct.pack(f">H{len(m)}H", len(m), *m))
                    tr.nacks += 1
                    log_event("receiver", "nack", transfer=tr.key, missing=missing)
                    print(f"\n  ขอส่งซ้ำ {len(missing)} packet: {missing[:20]}")
        else:
            txt = info.decode("utf-8", "replace")
            log_event("receiver", "rx_frame", src=fr["src"], dst=fr["dst"], rssi=rssi, snr=snr, info=txt[:200])
            if not txt.startswith(("PING", "PONG")):
                print(f"\n  {fr['src']}>{fr['dst']}: {txt}  (RSSI {rssi})")


# ---------------------------------------------------------------- web

class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def start_web(port):
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", port), partial(QuietHandler, directory=str(ROOT)))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"หน้าเว็บ: http://localhost:{port}/web/")
    return srv


def cmd_web(args):
    start_web(args.web_port)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="LoRa AX.25 station")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("send", help="ส่งรูป")
    s.add_argument("--port", required=True)
    s.add_argument("--dest", help="ปลายทาง เช่น N0CALL-2 (ค่าเริ่มต้น = dest ที่ตั้งในบอร์ด)")
    s.add_argument("--all", action="store_true", help="ส่งทุกไฟล์ใน images_to_send")
    s.add_argument("--max-px", type=int, default=320, help="ย่อรูปให้ด้านยาวสุดไม่เกินนี้ (ค่าเริ่มต้น 320)")
    s.add_argument("--quality", type=int, default=60, help="คุณภาพ JPEG (ค่าเริ่มต้น 60)")
    s.add_argument("--raw", action="store_true", help="ส่งไฟล์ตามต้นฉบับ ไม่ย่อ")
    s.add_argument("--max-rounds", type=int, default=5, help="จำนวนรอบส่งซ้ำสูงสุด")
    s.add_argument("--simulate-loss", type=float, default=0, metavar="P",
                   help="ทดสอบ: จงใจข้าม packet รอบแรกด้วยความน่าจะเป็น P (เช่น 0.1)")
    s.add_argument("files", nargs="*")
    s.set_defaults(fn=cmd_send)

    p = sub.add_parser("ping", help="ping")
    p.add_argument("--port", required=True)
    p.add_argument("--dest")
    p.add_argument("-n", type=int, default=4)
    p.set_defaults(fn=cmd_ping)

    r = sub.add_parser("rx", help="ตัวรับ (รับรูป + หน้าเว็บ)")
    r.add_argument("--port", required=True)
    r.add_argument("--timeout", type=float, default=30, help="วินาทีที่ไม่มีข้อมูลก่อนปิดไฟล์")
    r.add_argument("--web-port", type=int, default=8000)
    r.add_argument("--no-web", action="store_true")
    r.set_defaults(fn=cmd_rx)

    w = sub.add_parser("web", help="เปิดหน้าเว็บดู log")
    w.add_argument("--web-port", type=int, default=8000)
    w.set_defaults(fn=cmd_web)

    args = ap.parse_args()
    try:
        args.fn(args)
    except KeyboardInterrupt:
        print("\nหยุด")


if __name__ == "__main__":
    main()
