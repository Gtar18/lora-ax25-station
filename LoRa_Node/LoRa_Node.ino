/*
  LoRa / GFSK AX.25 Node (ใช้ firmware เดียวกันทุกบอร์ด)
  บอร์ด : ESP32 DevKit V1 (ESP32-WROOM-32) + Ra-01 (SX1278)
  ไลบรารี: RadioLib v8.x

  - เลือก modem ได้: LoRa / FSK / GFSK  (ค่าเริ่มต้น GFSK 9600 bps)
  - ปรับ bitrate ได้ 0.5 - 300 kbps, ความถี่, กำลังส่ง, deviation, RX bandwidth, shaping, SF/BW
    ค่าทั้งหมดบันทึกถาวรในบอร์ด (ทุกบอร์ดต้องตั้งให้ตรงกัน)
  - เฟรม AX.25 UI (ctrl 0x03, PID 0xF0, FCS CRC-16/X.25)
  - ตอบ PING อัตโนมัติ (ส่ง PONG กลับพร้อม RSSI/SNR ที่วัดได้)
  - ใช้กับ PuTTY (มีสีให้อ่านง่าย) หรือกับ tools/web_station.py (โหมด machine)

  ขนาด packet สูงสุด: LoRa 255 bytes, FSK/GFSK 64 bytes (FIFO ของ SX1278)
  -> ข้อความ/ข้อมูลต่อเฟรม: LoRa 237 bytes, FSK/GFSK 46 bytes

  พิมพ์ /help ดูคำสั่งทั้งหมด

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

// ---------------- อื่นๆ ----------------
#define AX25_OVERHEAD       18      // address 14 + ctrl + pid + FCS 2
#define MAX_INFO_ABS        (255 - AX25_OVERHEAD)
#define MAX_LINE            600
#define PING_TIMEOUT_MS     4000
#define PING_GAP_MS         300
#define BEACON_INTERVAL_MS  10000
#define LED_ON_MS           300
#define CFG_VERSION         2

// ---------------- สีสำหรับ PuTTY (ANSI) ----------------
#define C_RST  "\033[0m"
#define C_DIM  "\033[90m"
#define C_CMD  "\033[1;36m"
#define C_TX   "\033[94m"
#define C_RX   "\033[92m"
#define C_ERR  "\033[91m"
#define C_HEAD "\033[1;33m"
#define C_VAL  "\033[1;37m"

// ---------------- ค่าวิทยุ ----------------
enum { MODEM_LORA = 0, MODEM_FSK = 1, MODEM_GFSK = 2 };
const char *MODEM_NAME[] = {"LoRa", "FSK", "GFSK"};

struct RadioCfg {
  uint8_t  version;
  uint8_t  modem;
  uint32_t freq;      // Hz
  int8_t   power;     // dBm
  // LoRa
  uint32_t bw;        // Hz
  uint8_t  sf;
  uint8_t  cr;
  // FSK / GFSK
  uint32_t bitrate;   // bps
  uint32_t dev;       // Hz (0 = auto)
  uint32_t rxbw;      // Hz (0 = auto)
  uint8_t  shaping;   // RADIOLIB_SHAPING_x (ใช้กับ GFSK)
};

const RadioCfg DEFAULT_CFG = {
  CFG_VERSION, MODEM_GFSK, RADIOLIB_UNIT_MEGA(433), 17,
  RADIOLIB_UNIT_KILO(125), 9, 7,
  9600, 0, 0, RADIOLIB_SHAPING_0_5,
};

// เฟรม AX.25 ที่ถอดแล้ว (ต้องประกาศก่อนฟังก์ชัน เพราะ Arduino สร้าง prototype ให้อัตโนมัติ)
struct Ax25 {
  char dst[7]; uint8_t dstSsid;
  char src[7]; uint8_t srcSsid;
  String path;
  uint8_t ctrl, pid;
  const uint8_t *info; size_t infoLen;
};

Module loraMod(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1);
SX1278 radio(&loraMod);
Preferences prefs;
RadioCfg rc = DEFAULT_CFG;
bool radioOk = false;

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
uint32_t pingSentAt = 0, pingRtt = 0;
int      pingRemoteRssi = 0;
float    pingRemoteSnr = 0, pingLocalRssi = 0, pingLocalSnr = 0;

// ====================== utils ======================

void out(const char *s) { Serial.print(s); }

void outf(const char *fmt, ...) {
  char buf[700];
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
  String cs = dash > 0 ? s.substring(0, dash) : s;
  if (cs.length() == 0 || cs.length() > 6) return false;
  ssid = dash > 0 ? constrain(s.substring(dash + 1).toInt(), 0, 15) : 0;
  strncpy(call, cs.c_str(), 6);
  call[6] = '\0';
  return true;
}

void ledFlash() {
  digitalWrite(LED_PIN, HIGH);
  ledOn = true;
  ledOnAt = millis();
}

void prompt() {
  if (!machine) { outf("%s>%s ", C_CMD, C_RST); Serial.print(inputLine); }
}

size_t maxInfo() {
  return (rc.modem == MODEM_LORA ? 255 : RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK) - AX25_OVERHEAD;
}

// ====================== radio config ======================

uint32_t effDev() {
  if (rc.dev) return rc.dev;
  // auto: modulation index ~1 (dev = bitrate/2) ภายในข้อจำกัด dev + br/2 <= 250 kHz
  uint32_t d = rc.bitrate / 2;
  uint32_t lim = RADIOLIB_UNIT_KILO(250) - rc.bitrate / 2;
  if (d > lim) d = lim;
  if (d > RADIOLIB_UNIT_KILO(200)) d = RADIOLIB_UNIT_KILO(200);
  if (d < 600) d = 600;
  return d;
}

uint32_t effRxBw() {
  if (rc.rxbw) return rc.rxbw;
  // auto: Carson bandwidth 2*(dev + br/2) + 25% margin
  uint32_t b = (uint32_t)(2.5f * (effDev() + rc.bitrate / 2.0f));
  if (b < 5200) b = 5200;
  if (b > 500000) b = 500000;
  return b;
}

const char *shapingName(uint8_t s) {
  switch (s) {
    case RADIOLIB_SHAPING_0_3: return "0.3";
    case RADIOLIB_SHAPING_0_5: return "0.5";
    case RADIOLIB_SHAPING_0_7: return "0.7";
    case RADIOLIB_SHAPING_1_0: return "1.0";
    default: return "none";
  }
}

String radioSummary() {
  char b[160];
  if (rc.modem == MODEM_LORA) {
    snprintf(b, sizeof(b), "LoRa %.3f MHz SF%d BW%.1fk CR4/%d %ddBm",
             rc.freq / 1e6, rc.sf, rc.bw / 1e3, rc.cr, rc.power);
  } else {
    snprintf(b, sizeof(b), "%s %.3f MHz %.1fkbps dev%.1fk rxbw%.1fk%s%s %ddBm",
             MODEM_NAME[rc.modem], rc.freq / 1e6, rc.bitrate / 1e3, effDev() / 1e3, effRxBw() / 1e3,
             rc.modem == MODEM_GFSK ? " BT" : "", rc.modem == MODEM_GFSK ? shapingName(rc.shaping) : "",
             rc.power);
  }
  return String(b);
}

int applyRadio() {
  int st;
  if (rc.modem == MODEM_LORA) {
    ConfigLoRa_t cfg;
    cfg.frequency = rc.freq;
    cfg.bandwidth = rc.bw;
    cfg.spreadingFactor = rc.sf;
    cfg.codingRate = rc.cr;
    cfg.syncWord = 0x12;
    cfg.power = rc.power;
    cfg.preambleLength = 8;
    st = radio.begin(cfg);
    if (st == RADIOLIB_ERR_NONE) st = radio.setCRC(true);
  } else {
    ConfigFSK_t cfg;
    cfg.frequency = rc.freq;
    cfg.bitRate = rc.bitrate;
    cfg.frequencyDeviation = 600;           // ตั้งจริงหลังตั้ง bitrate แล้ว (ช่วงที่ใช้ได้ขึ้นกับ bitrate)
    cfg.receiverBandwidth = effRxBw();
    cfg.power = rc.power;
    cfg.preambleLength = 32;                // bits
    st = radio.beginFSK(cfg);
    if (st == RADIOLIB_ERR_NONE) st = radio.setFrequencyDeviation(effDev());
    if (st == RADIOLIB_ERR_NONE) st = radio.setRxBandwidth(effRxBw());
    if (st == RADIOLIB_ERR_NONE)
      st = radio.setDataShaping(rc.modem == MODEM_GFSK ? rc.shaping : RADIOLIB_SHAPING_NONE);
    if (st == RADIOLIB_ERR_NONE) st = radio.variablePacketLengthMode(RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK);
    if (st == RADIOLIB_ERR_NONE) st = radio.setCRC(true);
  }
  radioOk = (st == RADIOLIB_ERR_NONE);
  if (radioOk) {
    radio.setIrqAction(onIrq);
    irqFlag = false;
    radio.startReceive();
  }
  return st;
}

void loadConfig() {
  RadioCfg tmp;
  if (prefs.getBytes("radio", &tmp, sizeof(tmp)) == sizeof(tmp) && tmp.version == CFG_VERSION) rc = tmp;
  strncpy(myCall, prefs.getString("call", myCall).c_str(), 6);
  mySsid = prefs.getUChar("ssid", mySsid);
  strncpy(destCall, prefs.getString("dcall", destCall).c_str(), 6);
  destSsid = prefs.getUChar("dssid", destSsid);
}

void saveConfig() {
  prefs.putBytes("radio", &rc, sizeof(rc));
  prefs.putString("call", myCall);
  prefs.putUChar("ssid", mySsid);
  prefs.putString("dcall", destCall);
  prefs.putUChar("dssid", destSsid);
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
  for (int i = 0; i < 6; i++) { char ch = a[i] >> 1; if (ch != ' ') call[k++] = ch; }
  call[k] = '\0';
  ssid = (a[6] >> 1) & 0x0F;
}

// คืนค่า 0 = OK, 1 = สั้นเกิน, 2 = FCS ผิด, 3 = address ผิด
int parseAx25(const uint8_t *f, size_t len, Ax25 &x) {
  if (len < AX25_OVERHEAD) return 1;
  uint16_t fc = f[len - 2] | (f[len - 1] << 8);
  if (ax25Fcs(f, len - 2) != fc) return 2;
  size_t p = 0; int n = 0; bool end = false;
  x.path = "";
  while (p + 7 <= len - 2) {
    const uint8_t *a = f + p;
    char cs[7]; uint8_t s;
    decodeCall(a, cs, s);
    if (n == 0)      { strcpy(x.dst, cs); x.dstSsid = s; }
    else if (n == 1) { strcpy(x.src, cs); x.srcSsid = s; }
    else             { x.path += "," + fmtCall(cs, s) + ((a[6] & 0x80) ? "*" : ""); }
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

// ====================== radio TX / RX ======================

int sendInfo(const char *dst, uint8_t dSsid, const uint8_t *info, size_t n, uint32_t &airtime) {
  airtime = 0;
  if (!radioOk) return RADIOLIB_ERR_UNKNOWN;
  if (n > maxInfo()) return RADIOLIB_ERR_PACKET_TOO_LONG;
  uint8_t frame[AX25_OVERHEAD + MAX_INFO_ABS];
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
    if (!machine) outf("\r\n%s[RX] เฟรมเสีย (%s) %u bytes  RSSI %.1f%s\r\n", C_ERR,
                       r == 2 ? "FCS" : (r == 1 ? "สั้นเกิน" : "address"), (unsigned)len, rssi, C_RST);
    return;
  }

  bool toMe = strcmp(x.dst, myCall) == 0 && x.dstSsid == mySsid;
  bool binary = false;
  for (size_t i = 0; i < x.infoLen; i++) if (x.info[i] < 0x20 || x.info[i] >= 0x7F) { binary = true; break; }

  if (!machine) {
    outf("\r\n%s[RX]%s %s>%s%s: ", C_RX, C_RST, fmtCall(x.src, x.srcSsid).c_str(),
         fmtCall(x.dst, x.dstSsid).c_str(), x.path.c_str());
    if (binary) outf("%s<binary %u bytes>%s", C_DIM, (unsigned)x.infoLen, C_RST);
    else { out(C_VAL); printInfoText(x.info, x.infoLen); out(C_RST); }
    outf("  %sRSSI %.1f dBm  SNR %.1f dB%s\r\n", C_DIM, rssi, snr, C_RST);
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
    else outf("%s[PING]%s ตอบ PONG ไปที่ %s\r\n", C_RX, C_RST, fmtCall(x.src, x.srcSsid).c_str());
    return;
  }

  // ---- รับ PONG ----
  if (info.startsWith("PONG ") && pingWaiting &&
      strcmp(x.src, destCall) == 0 && x.srcSsid == destSsid) {
    int a = 5, b = info.indexOf(' ', a), cc = info.indexOf(' ', b + 1);
    if (b > 0 && cc > 0 && (uint16_t)info.substring(a, b).toInt() == pingId) {
      pingRtt = millis() - pingSentAt;
      pingRemoteRssi = info.substring(b + 1, cc).toInt();
      pingRemoteSnr = info.substring(cc + 1).toFloat();
      pingLocalRssi = rssi;
      pingLocalSnr = snr;
      pingWaiting = false;
    }
  }
}

void pollRadio() {
  if (!irqFlag || !radioOk) return;
  irqFlag = false;
  uint8_t buf[256];
  size_t len = radio.getPacketLength();
  if (len > sizeof(buf)) len = sizeof(buf);
  int st = radio.readData(buf, len);
  float rssi = radio.getRSSI();
  float snr = rc.modem == MODEM_LORA ? radio.getSNR() : 0;
  radio.startReceive();

  if (st == RADIOLIB_ERR_NONE) {
    ledFlash();
    handleFrame(buf, len, rssi, snr);
  } else if (st == RADIOLIB_ERR_CRC_MISMATCH) {
    if (machine) outf("#RXERR,CRC,%.1f\r\n", rssi);
    else outf("\r\n%s[RX] CRC error  RSSI %.1f%s\r\n", C_ERR, rssi, C_RST);
  } else {
    if (machine) outf("#RXERR,%d\r\n", st);
    else outf("\r\n%s[RX] readData failed %d%s\r\n", C_ERR, st, C_RST);
  }
}

// ====================== ping ======================

void doPing(int count) {
  String dst = fmtCall(destCall, destSsid);
  if (!machine) outf("%s[PING]%s %s  x%d  (%s)\r\n", C_TX, C_RST, dst.c_str(), count, radioSummary().c_str());
  int okc = 0;
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
      else outf("%s[PING] #%d ส่งไม่ได้ code %d%s\r\n", C_ERR, i, st, C_RST);
      continue;
    }
    pingWaiting = true;
    while (pingWaiting && millis() - pingSentAt < PING_TIMEOUT_MS) { pollRadio(); delay(1); }
    if (!pingWaiting) {
      okc++; sum += pingRtt; mn = min(mn, pingRtt); mx = max(mx, pingRtt);
      if (machine) outf("#PING,%d,OK,%lu,%d,%.1f,%.1f,%.1f,%s\r\n", i, (unsigned long)pingRtt,
                        pingRemoteRssi, pingRemoteSnr, pingLocalRssi, pingLocalSnr, dst.c_str());
      else outf("%s[PING]%s #%d ตอบกลับ  rtt=%s%lu ms%s   ปลายทางได้ยินเรา %d dBm   เราได้ยิน %.1f dBm\r\n",
                C_RX, C_RST, i, C_VAL, (unsigned long)pingRtt, C_RST, pingRemoteRssi, pingLocalRssi);
    } else {
      pingWaiting = false;
      if (machine) outf("#PING,%d,TIMEOUT,%s\r\n", i, dst.c_str());
      else outf("%s[PING] #%d timeout (ไม่มีการตอบกลับใน %d ms)%s\r\n", C_ERR, i, PING_TIMEOUT_MS, C_RST);
    }
    if (i < count) {
      uint32_t t = millis();
      while (millis() - t < PING_GAP_MS) { pollRadio(); delay(1); }
    }
  }
  if (machine) outf("#PINGDONE,%d,%d\r\n", okc, count);
  else outf("%s[PING] สรุป: ตอบกลับ %d/%d (%.0f%%)  rtt min/avg/max = %lu/%lu/%lu ms%s\r\n",
            C_HEAD, okc, count, 100.0 * okc / count, okc ? (unsigned long)mn : 0UL,
            okc ? (unsigned long)(sum / okc) : 0UL, (unsigned long)mx, C_RST);
}

// ====================== commands ======================

void printStatus() {
  if (machine) {
    outf("#STATUS,%s,%s,%s,%s,%u,%s\r\n", fmtCall(myCall, mySsid).c_str(), fmtCall(destCall, destSsid).c_str(),
         beaconOn ? "on" : "off", MODEM_NAME[rc.modem], (unsigned)maxInfo(), radioSummary().c_str());
  } else {
    outf("%s[INFO]%s mycall=%s%s%s  dest=%s%s%s  beacon=%s\r\n", C_HEAD, C_RST,
         C_VAL, fmtCall(myCall, mySsid).c_str(), C_RST, C_VAL, fmtCall(destCall, destSsid).c_str(), C_RST,
         beaconOn ? "on" : "off");
    outf("%s[RADIO]%s %s%s%s  (ข้อความสูงสุด %u bytes)%s\r\n", C_HEAD, C_RST, C_VAL, radioSummary().c_str(),
         C_RST, (unsigned)maxInfo(), radioOk ? "" : "  \033[91m<radio error>\033[0m");
  }
}

// ความกว้างที่แสดงบนจอของสตริง UTF-8 (ไม่นับสระ/วรรณยุกต์ไทยที่อยู่บน-ล่าง)
int displayWidth(const char *s) {
  int w = 0;
  const uint8_t *p = (const uint8_t *)s;
  while (*p) {
    uint32_t cp;
    if (*p < 0x80) { cp = *p++; }
    else if ((*p & 0xE0) == 0xC0) { cp = ((p[0] & 0x1F) << 6) | (p[1] & 0x3F); p += 2; }
    else if ((*p & 0xF0) == 0xE0) { cp = ((p[0] & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F); p += 3; }
    else { cp = 0; p += 4; }
    bool thaiMark = cp == 0x0E31 || (cp >= 0x0E34 && cp <= 0x0E3A) || (cp >= 0x0E47 && cp <= 0x0E4E);
    if (!thaiMark) w++;
  }
  return w;
}

void helpLine(const char *cmd, const char *desc) {
  int pad = 26 - displayWidth(cmd);
  outf("  %s%s%s%*s %s\r\n", C_CMD, cmd, C_RST, pad > 0 ? pad : 0, "", desc);
}

void printHelp() {
  outf("\r\n%s── ส่ง / ทดสอบ ─────────────────────────────%s\r\n", C_HEAD, C_RST);
  helpLine("<ข้อความ>", "ส่งข้อความไปยัง dest");
  helpLine("/ping [n]", "ping ไปยัง dest (ค่าเริ่มต้น 4 ครั้ง)");
  helpLine("/beacon on|off", "บีคอนอัตโนมัติทุก 10 วินาที");
  outf("%s── สถานี ───────────────────────────────────%s\r\n", C_HEAD, C_RST);
  helpLine("/mycall CALL-SSID", "ตั้งชื่อบอร์ดนี้");
  helpLine("/dest CALL-SSID", "ตั้งปลายทาง");
  helpLine("/status", "แสดงค่าปัจจุบัน");
  outf("%s── วิทยุ (ทุกบอร์ดต้องตั้งให้ตรงกัน) ─────────────%s\r\n", C_HEAD, C_RST);
  helpLine("/modem lora|fsk|gfsk", "เลือก modem");
  helpLine("/bitrate <kbps>", "FSK/GFSK: 0.5 - 300 kbps (เช่น 9.6)");
  helpLine("/dev <kHz>|auto", "FSK/GFSK: frequency deviation");
  helpLine("/rxbw <kHz>|auto", "FSK/GFSK: receiver bandwidth");
  helpLine("/shaping 0.3|0.5|0.7|1.0", "GFSK: Gaussian BT");
  helpLine("/sf <6-12>  /bw <kHz>", "LoRa: spreading factor, bandwidth");
  helpLine("/freq <MHz>", "ความถี่ 410 - 525 MHz");
  helpLine("/power <dBm>", "กำลังส่ง 2 - 17 (20 = PA boost)");
  helpLine("/radio", "แสดงค่าวิทยุ");
  helpLine("/radio reset", "คืนค่าเริ่มต้น (GFSK 9600)");
}

int hexVal(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  return -1;
}

void okMsg(const char *what) { if (machine) outf("#OK,%s\r\n", what); }

void errMsg(const char *what) {
  if (machine) outf("#ERR,%s\r\n", what);
  else outf("%s[ERR] %s%s\r\n", C_ERR, what, C_RST);
}

// เปลี่ยนค่าวิทยุ: ถ้าใช้ไม่ได้ให้กลับค่าเดิม
void radioChange(const RadioCfg &next, const char *what) {
  RadioCfg prev = rc;
  rc = next;
  int st = applyRadio();
  if (st != RADIOLIB_ERR_NONE) {
    rc = prev;
    applyRadio();
    char m[80];
    snprintf(m, sizeof(m), "ตั้ง %s ไม่ได้ (code %d) ใช้ค่าเดิม", what, st);
    errMsg(m);
    return;
  }
  saveConfig();
  okMsg("radio");
  if (machine) printStatus();
  else outf("%s[RADIO]%s %s%s%s\r\n", C_HEAD, C_RST, C_VAL, radioSummary().c_str(), C_RST);
}

bool needFsk() {
  if (rc.modem == MODEM_LORA) { errMsg("คำสั่งนี้ใช้กับ FSK/GFSK (พิมพ์ /modem gfsk ก่อน)"); return false; }
  return true;
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (!line.startsWith("/")) {
    uint32_t at;
    if (line.length() > maxInfo()) line = line.substring(0, maxInfo());
    int st = sendText(destCall, destSsid, line.c_str(), at);
    if (machine) outf("#TX,%s,%u,%lu\r\n", st == 0 ? "OK" : "ERR", line.length(), (unsigned long)at);
    else if (st == 0) outf("%s[TX]%s %s>%s: %s%s%s  %s(%lu ms)%s\r\n", C_TX, C_RST, fmtCall(myCall, mySsid).c_str(),
                           fmtCall(destCall, destSsid).c_str(), C_VAL, line.c_str(), C_RST, C_DIM,
                           (unsigned long)at, C_RST);
    else outf("%s[TX] ส่งไม่ได้ code %d%s\r\n", C_ERR, st, C_RST);
    return;
  }

  String cmd = line, arg = "";
  int sp = line.indexOf(' ');
  if (sp > 0) { cmd = line.substring(0, sp); arg = line.substring(sp + 1); arg.trim(); }
  cmd.toLowerCase();
  String argl = arg;
  argl.toLowerCase();
  RadioCfg n = rc;

  if (cmd == "/help") {
    printHelp();
  } else if (cmd == "/status") {
    printStatus();
  } else if (cmd == "/machine") {
    machine = (argl == "on");
    okMsg(machine ? "machine" : "human");
    if (!machine) printStatus();
  } else if (cmd == "/mycall") {
    if (parseCall(arg, myCall, mySsid)) { saveConfig(); okMsg("mycall"); if (!machine) printStatus(); }
    else errMsg("รูปแบบ: /mycall CALL-SSID");
  } else if (cmd == "/dest") {
    if (parseCall(arg, destCall, destSsid)) { saveConfig(); okMsg("dest"); if (!machine) printStatus(); }
    else errMsg("รูปแบบ: /dest CALL-SSID");
  } else if (cmd == "/beacon") {
    beaconOn = (argl == "on");
    okMsg("beacon");
    if (!machine) printStatus();
  } else if (cmd == "/ping") {
    int cnt = arg.length() ? constrain(arg.toInt(), 1, 100) : 4;
    doPing(cnt);
  } else if (cmd == "/diag") {
    Module *m = &loraMod;
    outf("[DIAG] opmode=0x%02X irq1=0x%02X irq2=0x%02X dio1map=0x%02X dio0pin=%d irqFlag=%d rssi=%.1f\r\n",
         m->SPIreadRegister(0x01), m->SPIreadRegister(0x3E), m->SPIreadRegister(0x3F),
         m->SPIreadRegister(0x40), digitalRead(LORA_DIO0), (int)irqFlag, radio.getRSSI());

  // ---------- radio ----------
  } else if (cmd == "/radio") {
    if (argl == "reset") radioChange(DEFAULT_CFG, "ค่าเริ่มต้น");
    else printStatus();
  } else if (cmd == "/modem") {
    if (argl == "lora") n.modem = MODEM_LORA;
    else if (argl == "fsk") n.modem = MODEM_FSK;
    else if (argl == "gfsk") n.modem = MODEM_GFSK;
    else { errMsg("รูปแบบ: /modem lora|fsk|gfsk"); return; }
    radioChange(n, "modem");
  } else if (cmd == "/bitrate" || cmd == "/br") {
    if (!needFsk()) return;
    float k = arg.toFloat();
    if (k < 0.5f || k > 300.0f) { errMsg("bitrate ต้องอยู่ระหว่าง 0.5 - 300 kbps"); return; }
    n.bitrate = (uint32_t)(k * 1000.0f + 0.5f);
    radioChange(n, "bitrate");
  } else if (cmd == "/dev") {
    if (!needFsk()) return;
    n.dev = (argl == "auto" || argl == "0") ? 0 : (uint32_t)(arg.toFloat() * 1000.0f);
    radioChange(n, "deviation");
  } else if (cmd == "/rxbw") {
    if (!needFsk()) return;
    n.rxbw = (argl == "auto" || argl == "0") ? 0 : (uint32_t)(arg.toFloat() * 1000.0f);
    radioChange(n, "rx bandwidth");
  } else if (cmd == "/shaping") {
    if (argl == "0.3") n.shaping = RADIOLIB_SHAPING_0_3;
    else if (argl == "0.5") n.shaping = RADIOLIB_SHAPING_0_5;
    else if (argl == "0.7") n.shaping = RADIOLIB_SHAPING_0_7;
    else if (argl == "1.0" || argl == "1") n.shaping = RADIOLIB_SHAPING_1_0;
    else { errMsg("รูปแบบ: /shaping 0.3|0.5|0.7|1.0"); return; }
    radioChange(n, "shaping");
  } else if (cmd == "/freq") {
    float m = arg.toFloat();
    if (m < 410 || m > 525) { errMsg("ความถี่ต้องอยู่ระหว่าง 410 - 525 MHz"); return; }
    n.freq = (uint32_t)(m * 1e6 + 0.5);
    radioChange(n, "ความถี่");
  } else if (cmd == "/power") {
    int p = arg.toInt();
    if (p < 2 || p > 20) { errMsg("กำลังส่งต้องอยู่ระหว่าง 2 - 20 dBm"); return; }
    n.power = p;
    radioChange(n, "กำลังส่ง");
  } else if (cmd == "/sf") {
    int s = arg.toInt();
    if (s < 6 || s > 12) { errMsg("SF ต้องอยู่ระหว่าง 6 - 12"); return; }
    n.sf = s;
    radioChange(n, "SF");
  } else if (cmd == "/bw") {
    n.bw = (uint32_t)(arg.toFloat() * 1000.0f);
    radioChange(n, "BW");

  // ---------- binary (ใช้โดย web_station.py) ----------
  } else if (cmd == "/hexto") {
    int sp2 = arg.indexOf(' ');
    char cs[7]; uint8_t s;
    if (sp2 <= 0 || !parseCall(arg.substring(0, sp2), cs, s)) { errMsg("hexto args"); return; }
    String hex = arg.substring(sp2 + 1);
    hex.trim();
    size_t nb = hex.length() / 2;
    if (nb == 0 || nb > maxInfo() || hex.length() % 2) { errMsg("hexto length"); return; }
    uint8_t data[MAX_INFO_ABS];
    for (size_t i = 0; i < nb; i++) {
      int h = hexVal(hex[2 * i]), l = hexVal(hex[2 * i + 1]);
      if (h < 0 || l < 0) { errMsg("hexto hex"); return; }
      data[i] = (h << 4) | l;
    }
    uint32_t at;
    int st = sendInfo(cs, s, data, nb, at);
    if (machine) outf("#TX,%s,%u,%lu\r\n", st == 0 ? "OK" : "ERR", (unsigned)nb, (unsigned long)at);
    else outf("%s[TX]%s binary %u bytes -> %s (%lu ms) %s\r\n", C_TX, C_RST, (unsigned)nb,
              fmtCall(cs, s).c_str(), (unsigned long)at, st == 0 ? "OK" : "FAILED");
  } else {
    errMsg("ไม่รู้จักคำสั่ง พิมพ์ /help");
  }
}

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (inputLine.length()) {
        if (!machine) out("\r\n");
        String l = inputLine;
        inputLine = "";
        handleLine(l);
        prompt();
      }
    } else if (ch == 0x08 || ch == 0x7F) {
      if (inputLine.length()) {
        inputLine.remove(inputLine.length() - 1);
        if (!machine) out("\b \b");
      }
    } else if ((uint8_t)ch >= 0x20 && inputLine.length() < MAX_LINE) {
      inputLine += ch;
      if (!machine) Serial.print(ch);
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
  loadConfig();

  outf("\r\n%s=== LoRa AX.25 Node ===%s\r\n", C_HEAD, C_RST);
  SPI.begin(18, 19, 23, LORA_NSS);

  int st = applyRadio();
  if (st != RADIOLIB_ERR_NONE) {
    outf("%s[ERR] radio init failed (code %d) ลองค่าเริ่มต้น...%s\r\n", C_ERR, st, C_RST);
    rc = DEFAULT_CFG;
    st = applyRadio();
  }
  if (st != RADIOLIB_ERR_NONE) {
    outf("%s[ERR] radio.begin failed, code %d  -> ตรวจสายไฟ / 3.3V / NSS / RST%s\r\n", C_ERR, st, C_RST);
    while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  }

  outf("%s[OK]%s Radio ready, listening   พิมพ์ %s/help%s ดูคำสั่ง\r\n", C_RX, C_RST, C_CMD, C_RST);
  printStatus();
  prompt();
}

void loop() {
  readSerial();
  pollRadio();

  if (beaconOn && millis() - lastBeacon >= BEACON_INTERVAL_MS) {
    lastBeacon = millis();
    char msg[48];
    snprintf(msg, sizeof(msg), "Beacon #%lu up %lus", (unsigned long)beaconSeq++,
             (unsigned long)(millis() / 1000));
    uint32_t at;
    sendText(destCall, destSsid, msg, at);
    if (!machine) { outf("\r\n%s[TX]%s beacon: %s\r\n", C_TX, C_RST, msg); prompt(); }
  }

  if (ledOn && millis() - ledOnAt >= LED_ON_MS) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }

  // ให้ IDLE task ได้ทำงาน (จำเป็นเมื่อ loop รันบน core 0 ไม่งั้น watchdog จะรีบูตบอร์ด)
  if (!Serial.available() && !irqFlag) delay(1);
}
