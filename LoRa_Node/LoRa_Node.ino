/*
  LoRa AX.25 Node (ใช้ firmware เดียวกันทั้งสองบอร์ด)
  บอร์ด : ESP32 DevKit V1 (ESP32-WROOM-32) + Ra-01 (SX1278)
  ไลบรารี: RadioLib v8.x

  - ฟังตลอดเวลา ส่งเฉพาะเมื่อสั่ง (บีคอนปิดเป็นค่าเริ่มต้น)
  - เฟรม AX.25 UI (ctrl 0x03, PID 0xF0, FCS CRC-16/X.25) ผ่าน LoRa
  - ตอบ PING อัตโนมัติ (ส่ง PONG กลับพร้อม RSSI/SNR ที่วัดได้)
  - ใช้กับ PuTTY (คนพิมพ์) หรือกับ tools/station.py (โหมด machine) ก็ได้

  คำสั่ง (พิมพ์แล้ว Enter):
    <ข้อความ>                 ส่งข้อความไปยัง dest
    /ping [n]                 ping ไปยัง dest n ครั้ง (ค่าเริ่มต้น 4)
    /mycall CALL-SSID         ตั้งนามเรียกขานของบอร์ดนี้ (บันทึกถาวร)
    /dest CALL-SSID           ตั้งปลายทาง (บันทึกถาวร)
    /beacon on|off            บีคอนอัตโนมัติ
    /status                   แสดงค่าปัจจุบัน
    /hexto CALL-SSID <hex>    ส่งข้อมูลไบนารี (ใช้โดย station.py)
    /machine on|off           โหมดสำหรับโปรแกรม (ไม่มี echo, ผลลัพธ์ขึ้นต้นด้วย #)

  การต่อสาย (Ra-01 -> ESP32, D.. หรือ G.. เลขเดียวกัน)
    3.3V -> 3V3 | GND -> GND
    SCK -> 18 | MISO -> 19 | MOSI -> 23 | NSS -> 5 | RST -> 14 | DIO0 -> 26
  ** ต้องต่อเสาอากาศก่อนส่งทุกครั้ง **
*/

#include <RadioLib.h>
#include <Preferences.h>

// ---------------- ขา ----------------
#define LORA_NSS   5
#define LORA_DIO0  26
#define LORA_RST   14
#define LORA_DIO1  RADIOLIB_NC
#define LED_PIN    2

// ---------------- ค่าวิทยุ (ต้องเหมือนกันทุกบอร์ด) ----------------
#define LORA_FREQ       RADIOLIB_UNIT_MEGA(433)
#define LORA_BW         RADIOLIB_UNIT_KILO(125)
#define LORA_SF         9
#define LORA_CR         7
#define LORA_SYNC_WORD  0x12
#define LORA_POWER      17
#define LORA_PREAMBLE   8

// ---------------- อื่นๆ ----------------
#define MAX_INFO_LEN        237     // 255 - 16 (address) - 2 (FCS)
#define MAX_LINE            600
#define PING_TIMEOUT_MS     4000
#define PING_GAP_MS         500
#define BEACON_INTERVAL_MS  10000
#define LED_ON_MS           300

// เฟรม AX.25 ที่ถอดแล้ว (ต้องประกาศก่อนฟังก์ชัน เพราะ Arduino สร้าง prototype ให้อัตโนมัติ)
struct Ax25 {
  char dst[7]; uint8_t dstSsid;
  char src[7]; uint8_t srcSsid;
  String path;
  uint8_t ctrl, pid;
  const uint8_t *info; size_t infoLen;
};

SX1278 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1);
Preferences prefs;

char     myCall[7]   = "N0CALL";
uint8_t  mySsid      = 1;
char     destCall[7] = "N0CALL";
uint8_t  destSsid    = 2;
bool     beaconOn    = false;
bool     machine     = false;

volatile bool irqFlag = false;
void IRAM_ATTR onIrq() { irqFlag = true; }

String   inputLine;
uint32_t lastBeacon = 0, beaconSeq = 0;
uint32_t ledOnAt = 0;
bool     ledOn = false;

// ping state
bool     pingWaiting = false;
uint16_t pingId = 0;
uint32_t pingSentAt = 0;
int      pingRemoteRssi = 0;
float    pingRemoteSnr = 0;
uint32_t pingRtt = 0;
float    pingLocalRssi = 0, pingLocalSnr = 0;

// ====================== utils ======================

void out(const char *s) { Serial.print(s); }

void outf(const char *fmt, ...) {
  char buf[640];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
}

String fmtCall(const char *call, uint8_t ssid) {
  String s(call);
  if (ssid) s += "-" + String(ssid);
  return s;
}

bool parseCall(String s, char *call, uint8_t &ssid) {
  s.trim();
  s.toUpperCase();
  if (s.length() == 0) return false;
  int dash = s.indexOf('-');
  String c = dash > 0 ? s.substring(0, dash) : s;
  if (c.length() == 0 || c.length() > 6) return false;
  ssid = dash > 0 ? constrain(s.substring(dash + 1).toInt(), 0, 15) : 0;
  strncpy(call, c.c_str(), 6);
  call[6] = '\0';
  return true;
}

void ledFlash() {
  digitalWrite(LED_PIN, HIGH);
  ledOn = true;
  ledOnAt = millis();
}

void prompt() {
  if (!machine) { out("> "); Serial.print(inputLine); }
}

// ====================== AX.25 ======================

uint16_t ax25Fcs(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
  }
  return ~crc;
}

uint8_t *putAddr(uint8_t *p, const char *call, uint8_t ssid, bool cBit, bool last) {
  size_t n = strlen(call);
  for (size_t i = 0; i < 6; i++) *p++ = (uint8_t)((i < n) ? toupper(call[i]) : ' ') << 1;
  *p++ = (cBit ? 0x80 : 0) | 0x60 | ((ssid & 0x0F) << 1) | (last ? 1 : 0);
  return p;
}

size_t buildUI(uint8_t *buf, const char *dst, uint8_t dSsid, const uint8_t *info, size_t n) {
  uint8_t *p = buf;
  p = putAddr(p, dst, dSsid, true, false);
  p = putAddr(p, myCall, mySsid, false, true);
  *p++ = 0x03;
  *p++ = 0xF0;
  memcpy(p, info, n);
  p += n;
  uint16_t f = ax25Fcs(buf, p - buf);
  *p++ = f & 0xFF;
  *p++ = f >> 8;
  return p - buf;
}

void decodeCall(const uint8_t *a, char *call, uint8_t &ssid) {
  int k = 0;
  for (int i = 0; i < 6; i++) { char c = a[i] >> 1; if (c != ' ') call[k++] = c; }
  call[k] = '\0';
  ssid = (a[6] >> 1) & 0x0F;
}

// คืนค่า 0 = OK, 1 = สั้นเกิน, 2 = FCS ผิด, 3 = address ผิด
int parseAx25(const uint8_t *f, size_t len, Ax25 &x) {
  if (len < 18) return 1;
  uint16_t fc = f[len - 2] | (f[len - 1] << 8);
  if (ax25Fcs(f, len - 2) != fc) return 2;
  size_t p = 0; int n = 0; bool end = false;
  x.path = "";
  while (p + 7 <= len - 2) {
    const uint8_t *a = f + p;
    char c[7]; uint8_t s;
    decodeCall(a, c, s);
    if (n == 0)      { strcpy(x.dst, c); x.dstSsid = s; }
    else if (n == 1) { strcpy(x.src, c); x.srcSsid = s; }
    else             { x.path += "," + fmtCall(c, s) + ((a[6] & 0x80) ? "*" : ""); }
    n++; p += 7;
    if (a[6] & 1) { end = true; break; }
  }
  if (!end || n < 2 || p + 2 > len - 2) return 3;
  x.ctrl = f[p++];
  x.pid = f[p++];
  x.info = f + p;
  x.infoLen = len - 2 - p;
  return 0;
}

// ====================== radio ======================

int sendInfo(const char *dst, uint8_t dSsid, const uint8_t *info, size_t n, uint32_t &airtime) {
  if (n > MAX_INFO_LEN) n = MAX_INFO_LEN;
  uint8_t frame[16 + MAX_INFO_LEN + 2];
  size_t len = buildUI(frame, dst, dSsid, info, n);
  ledFlash();
  uint32_t t0 = millis();
  int st = radio.transmit(frame, len);
  airtime = millis() - t0;
  irqFlag = false;          // TX-done ก็ทำให้ DIO0 ขึ้น จึงต้องล้างก่อนกลับไปรับ
  radio.startReceive();
  return st;
}

int sendText(const char *dst, uint8_t dSsid, const char *text, uint32_t &airtime) {
  return sendInfo(dst, dSsid, (const uint8_t *)text, strlen(text), airtime);
}

void printInfoText(const uint8_t *d, size_t n) {
  for (size_t i = 0; i < n; i++) Serial.print((d[i] >= 0x20 && d[i] < 0x7F) ? (char)d[i] : '.');
}

void handleFrame(const uint8_t *f, size_t len, float rssi, float snr) {
  if (machine) {
    outf("#RX,%.1f,%.1f,", rssi, snr);
    for (size_t i = 0; i < len; i++) outf("%02X", f[i]);
    out("\r\n");
  }

  Ax25 x;
  int r = parseAx25(f, len, x);
  if (r != 0) {
    if (!machine) outf("\r\n[RX] bad frame (%s) %u bytes RSSI=%.1f\r\n",
                       r == 2 ? "FCS" : (r == 1 ? "short" : "addr"), (unsigned)len, rssi);
    return;
  }

  bool toMe = strcmp(x.dst, myCall) == 0 && x.dstSsid == mySsid;
  bool binary = false;
  for (size_t i = 0; i < x.infoLen; i++) if (x.info[i] < 0x20 || x.info[i] >= 0x7F) { binary = true; break; }

  if (!machine) {
    outf("\r\n[RX] %s>%s%s:", fmtCall(x.src, x.srcSsid).c_str(), fmtCall(x.dst, x.dstSsid).c_str(), x.path.c_str());
    if (binary) outf("<binary %u bytes>", (unsigned)x.infoLen);
    else printInfoText(x.info, x.infoLen);
    outf("  RSSI=%.1f SNR=%.1f\r\n", rssi, snr);
  }

  if (!toMe || binary) return;

  String info;
  for (size_t i = 0; i < x.infoLen; i++) info += (char)x.info[i];

  // ---- ตอบ PING ----
  if (info.startsWith("PING ")) {
    String id = info.substring(5);
    id.trim();
    char reply[48];
    snprintf(reply, sizeof(reply), "PONG %s %d %.1f", id.c_str(), (int)rssi, snr);
    delay(30);
    uint32_t at;
    sendText(x.src, x.srcSsid, reply, at);
    if (machine) outf("#PINGED,%s,%.1f,%.1f\r\n", fmtCall(x.src, x.srcSsid).c_str(), rssi, snr);
    else outf("[PING] ตอบ PONG ไปที่ %s\r\n", fmtCall(x.src, x.srcSsid).c_str());
    return;
  }

  // ---- รับ PONG ----
  if (info.startsWith("PONG ") && pingWaiting &&
      strcmp(x.src, destCall) == 0 && x.srcSsid == destSsid) {
    int a = 5, b = info.indexOf(' ', a), c = info.indexOf(' ', b + 1);
    if (b > 0 && c > 0 && (uint16_t)info.substring(a, b).toInt() == pingId) {
      pingRtt = millis() - pingSentAt;
      pingRemoteRssi = info.substring(b + 1, c).toInt();
      pingRemoteSnr = info.substring(c + 1).toFloat();
      pingLocalRssi = rssi;
      pingLocalSnr = snr;
      pingWaiting = false;
    }
  }
}

void pollRadio() {
  if (!irqFlag) return;
  irqFlag = false;
  uint8_t buf[256];
  size_t len = radio.getPacketLength();
  if (len > sizeof(buf)) len = sizeof(buf);
  int st = radio.readData(buf, len);
  float rssi = radio.getRSSI(), snr = radio.getSNR();
  radio.startReceive();

  if (st == RADIOLIB_ERR_NONE) {
    ledFlash();
    handleFrame(buf, len, rssi, snr);
  } else if (st == RADIOLIB_ERR_CRC_MISMATCH) {
    if (machine) outf("#RXERR,CRC,%.1f\r\n", rssi);
    else outf("\r\n[RX] LoRa CRC error RSSI=%.1f\r\n", rssi);
  } else {
    if (machine) outf("#RXERR,%d\r\n", st);
    else outf("\r\n[RX] readData failed %d\r\n", st);
  }
}

// ====================== ping ======================

void doPing(int count) {
  String dst = fmtCall(destCall, destSsid);
  if (!machine) outf("[PING] %s  %d ครั้ง\r\n", dst.c_str(), count);
  int ok = 0;
  uint32_t sum = 0, mn = 0xFFFFFFFF, mx = 0;
  for (int i = 1; i <= count; i++) {
    pingId++;
    char msg[24];
    snprintf(msg, sizeof(msg), "PING %u", pingId);
    uint32_t at;
    pingSentAt = millis();
    int st = sendText(destCall, destSsid, msg, at);
    if (st != RADIOLIB_ERR_NONE) {
      if (machine) outf("#PING,%d,TXERR,%d,%s\r\n", i, st, dst.c_str());
      else outf("[PING] #%d ส่งไม่ได้ code %d\r\n", i, st);
      continue;
    }
    pingWaiting = true;
    while (pingWaiting && millis() - pingSentAt < PING_TIMEOUT_MS) {
      pollRadio();
      delay(1);
    }
    if (!pingWaiting) {
      ok++; sum += pingRtt; mn = min(mn, pingRtt); mx = max(mx, pingRtt);
      if (machine) outf("#PING,%d,OK,%lu,%d,%.1f,%.1f,%.1f,%s\r\n", i, (unsigned long)pingRtt,
                        pingRemoteRssi, pingRemoteSnr, pingLocalRssi, pingLocalSnr, dst.c_str());
      else outf("[PING] #%d ตอบกลับจาก %s  rtt=%lu ms  ปลายทางได้ยินเรา RSSI=%d SNR=%.1f | เราได้ยิน RSSI=%.1f SNR=%.1f\r\n",
                i, dst.c_str(), (unsigned long)pingRtt, pingRemoteRssi, pingRemoteSnr, pingLocalRssi, pingLocalSnr);
    } else {
      pingWaiting = false;
      if (machine) outf("#PING,%d,TIMEOUT,%s\r\n", i, dst.c_str());
      else outf("[PING] #%d timeout (ไม่มีการตอบกลับใน %d ms)\r\n", i, PING_TIMEOUT_MS);
    }
    if (i < count) {
      uint32_t t = millis();
      while (millis() - t < PING_GAP_MS) { pollRadio(); delay(1); }
    }
  }
  if (machine) outf("#PINGDONE,%d,%d\r\n", ok, count);
  else outf("[PING] สรุป: ตอบกลับ %d/%d (%.0f%%)  rtt min/avg/max = %lu/%lu/%lu ms\r\n",
            ok, count, 100.0 * ok / count, ok ? (unsigned long)mn : 0UL,
            ok ? (unsigned long)(sum / ok) : 0UL, (unsigned long)mx);
}

// ====================== commands ======================

void saveConfig() {
  prefs.putString("call", myCall);
  prefs.putUChar("ssid", mySsid);
  prefs.putString("dcall", destCall);
  prefs.putUChar("dssid", destSsid);
}

void printStatus() {
  if (machine) outf("#STATUS,%s,%s,%s\r\n", fmtCall(myCall, mySsid).c_str(),
                    fmtCall(destCall, destSsid).c_str(), beaconOn ? "on" : "off");
  else outf("[INFO] mycall=%s  dest=%s  beacon=%s  (433 MHz SF%d BW125)\r\n",
            fmtCall(myCall, mySsid).c_str(), fmtCall(destCall, destSsid).c_str(),
            beaconOn ? "on" : "off", LORA_SF);
}

void printHelp() {
  out("\r\nคำสั่ง:\r\n"
      "  <ข้อความ>             ส่งข้อความไปยัง dest\r\n"
      "  /ping [n]            ping ไปยัง dest (ค่าเริ่มต้น 4 ครั้ง)\r\n"
      "  /mycall CALL-SSID    ตั้งชื่อบอร์ดนี้\r\n"
      "  /dest CALL-SSID      ตั้งปลายทาง\r\n"
      "  /beacon on|off       บีคอนอัตโนมัติ\r\n"
      "  /status  /help\r\n");
}

int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

void ok(const char *what) { if (machine) outf("#OK,%s\r\n", what); }
void err(const char *what) {
  if (machine) outf("#ERR,%s\r\n", what);
  else outf("[INFO] %s\r\n", what);
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (!line.startsWith("/")) {
    uint32_t at;
    if (line.length() > MAX_INFO_LEN) line = line.substring(0, MAX_INFO_LEN);
    int st = sendText(destCall, destSsid, line.c_str(), at);
    if (machine) outf("#TX,%s,%u,%lu\r\n", st == 0 ? "OK" : "ERR", line.length(), (unsigned long)at);
    else if (st == 0) outf("[TX] %s>%s:%s  (%lu ms)\r\n", fmtCall(myCall, mySsid).c_str(),
                           fmtCall(destCall, destSsid).c_str(), line.c_str(), (unsigned long)at);
    else outf("[TX] failed, code %d\r\n", st);
    return;
  }

  String cmd = line, arg = "";
  int sp = line.indexOf(' ');
  if (sp > 0) { cmd = line.substring(0, sp); arg = line.substring(sp + 1); arg.trim(); }
  cmd.toLowerCase();

  if (cmd == "/help") {
    printHelp();
  } else if (cmd == "/status") {
    printStatus();
  } else if (cmd == "/machine") {
    machine = (arg == "on");
    ok(machine ? "machine" : "human");
    if (!machine) { printStatus(); }
  } else if (cmd == "/mycall") {
    if (parseCall(arg, myCall, mySsid)) { saveConfig(); ok("mycall"); if (!machine) printStatus(); }
    else err("รูปแบบ: /mycall CALL-SSID");
  } else if (cmd == "/dest") {
    if (parseCall(arg, destCall, destSsid)) { saveConfig(); ok("dest"); if (!machine) printStatus(); }
    else err("รูปแบบ: /dest CALL-SSID");
  } else if (cmd == "/beacon") {
    arg.toLowerCase();
    beaconOn = (arg == "on");
    ok("beacon");
    if (!machine) printStatus();
  } else if (cmd == "/ping") {
    int n = arg.length() ? constrain(arg.toInt(), 1, 100) : 4;
    doPing(n);
  } else if (cmd == "/hexto") {
    int sp2 = arg.indexOf(' ');
    char c[7]; uint8_t s;
    if (sp2 <= 0 || !parseCall(arg.substring(0, sp2), c, s)) { err("hexto args"); return; }
    String hex = arg.substring(sp2 + 1);
    hex.trim();
    size_t n = hex.length() / 2;
    if (n == 0 || n > MAX_INFO_LEN || hex.length() % 2) { err("hexto length"); return; }
    uint8_t data[MAX_INFO_LEN];
    for (size_t i = 0; i < n; i++) {
      int h = hexVal(hex[2 * i]), l = hexVal(hex[2 * i + 1]);
      if (h < 0 || l < 0) { err("hexto hex"); return; }
      data[i] = (h << 4) | l;
    }
    uint32_t at;
    int st = sendInfo(c, s, data, n, at);
    if (machine) outf("#TX,%s,%u,%lu\r\n", st == 0 ? "OK" : "ERR", (unsigned)n, (unsigned long)at);
    else outf("[TX] binary %u bytes -> %s (%lu ms) %s\r\n", (unsigned)n, fmtCall(c, s).c_str(),
              (unsigned long)at, st == 0 ? "OK" : "FAILED");
  } else {
    err("ไม่รู้จักคำสั่ง พิมพ์ /help");
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (inputLine.length()) {
        if (!machine) out("\r\n");
        String l = inputLine;
        inputLine = "";
        handleLine(l);
        prompt();
      }
    } else if (c == 0x08 || c == 0x7F) {
      if (inputLine.length()) {
        inputLine.remove(inputLine.length() - 1);
        if (!machine) out("\b \b");
      }
    } else if (c >= 0x20 && inputLine.length() < MAX_LINE) {
      inputLine += c;
      if (!machine) Serial.print(c);
    }
  }
}

// ====================== setup / loop ======================

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(115200);
  delay(300);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  prefs.begin("node", false);
  strncpy(myCall, prefs.getString("call", myCall).c_str(), 6);
  mySsid = prefs.getUChar("ssid", mySsid);
  strncpy(destCall, prefs.getString("dcall", destCall).c_str(), 6);
  destSsid = prefs.getUChar("dssid", destSsid);

  out("\r\n=== LoRa AX.25 Node ===\r\n");
  SPI.begin(18, 19, 23, LORA_NSS);

  ConfigLoRa_t cfg;
  cfg.frequency = LORA_FREQ;
  cfg.bandwidth = LORA_BW;
  cfg.spreadingFactor = LORA_SF;
  cfg.codingRate = LORA_CR;
  cfg.syncWord = LORA_SYNC_WORD;
  cfg.power = LORA_POWER;
  cfg.preambleLength = LORA_PREAMBLE;
  int st = radio.begin(cfg);
  if (st != RADIOLIB_ERR_NONE) {
    outf("[ERR] radio.begin failed, code %d  -> ตรวจสายไฟ / 3.3V / NSS / RST\r\n", st);
    while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  }
  radio.setCRC(true);
  radio.setIrqAction(onIrq);
  radio.startReceive();

  out("[OK] Radio ready, listening\r\n");
  printStatus();
  printHelp();
  prompt();
}

void loop() {
  readSerial();
  pollRadio();

  if (beaconOn && millis() - lastBeacon >= BEACON_INTERVAL_MS) {
    lastBeacon = millis();
    char msg[48];
    snprintf(msg, sizeof(msg), "Beacon #%lu uptime=%lus", (unsigned long)beaconSeq++,
             (unsigned long)(millis() / 1000));
    uint32_t at;
    sendText(destCall, destSsid, msg, at);
    if (!machine) { outf("\r\n[TX] beacon: %s\r\n", msg); prompt(); }
  }

  if (ledOn && millis() - ledOnAt >= LED_ON_MS) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }
}
