/*
  SX1278 LoRa AX.25 Transmitter (ตัวส่ง AX.25 + terminal)
  บอร์ด : ESP32 DevKit V1 (ESP32-WROOM-32)
  ไลบรารี: RadioLib v8.x

  ส่งเฟรม AX.25 UI (control 0x03, PID 0xF0, FCS CRC-16/X.25) ผ่าน LoRa
  - บีคอนอัตโนมัติทุก BEACON_INTERVAL_MS
  - พิมพ์ข้อความใน terminal (PuTTY / Serial Monitor 115200) แล้วกด Enter = ส่งข้อความนั้น
  คำสั่งใน terminal:
    /help                 แสดงคำสั่ง
    /dest CALL[-SSID]     ตั้งปลายทาง  เช่น /dest CQ  หรือ /dest HS0ABC-7
    /beacon on|off        เปิด/ปิดบีคอนอัตโนมัติ
    /status               แสดงค่าปัจจุบัน

  ** เปลี่ยน MY_CALL เป็นนามเรียกขานของคุณ (ถ้ามีใบอนุญาตวิทยุสมัครเล่น) **

  การต่อสาย (Ra-01 -> ESP32)
    3.3V -> 3V3 | GND -> GND
    SCK -> D18 | MISO -> D19 | MOSI -> D23
    NSS -> D5  | RST  -> D14 | DIO0 -> D26 | DIO1 -> ไม่ต่อ
  ** ต้องต่อเสาอากาศก่อนส่งทุกครั้ง **
*/

#include <RadioLib.h>

// ---------------- ขา ----------------
#define LORA_NSS   5
#define LORA_DIO0  26
#define LORA_RST   14
#define LORA_DIO1  RADIOLIB_NC   // ไม่ได้ต่อ DIO1
#define LED_PIN    2

// ---------------- ค่าวิทยุ (ต้องตรงกับตัวรับ) ----------------
#define LORA_FREQ       RADIOLIB_UNIT_MEGA(433)   // 433 MHz
#define LORA_BW         RADIOLIB_UNIT_KILO(125)   // 125 kHz
#define LORA_SF         9
#define LORA_CR         7
#define LORA_SYNC_WORD  0x12
#define LORA_POWER      17      // dBm
#define LORA_PREAMBLE   8

// ---------------- AX.25 ----------------
#define MY_CALL       "N0CALL"   // นามเรียกขานต้นทาง (สูงสุด 6 ตัวอักษร)
#define MY_SSID       1
#define DEFAULT_DEST  "BEACON"
#define MAX_INFO_LEN  200

// ---------------- บีคอน / LED ----------------
#define BEACON_INTERVAL_MS  5000
#define LED_ON_MS           500

SX1278 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1);

char     destCall[7] = DEFAULT_DEST;
uint8_t  destSsid    = 0;
bool     beaconOn    = true;
uint32_t seq         = 0;
uint32_t lastBeacon  = 0;
uint32_t ledOnAt     = 0;
bool     ledOn       = false;
String   inputLine;

// ====================== AX.25 encoder ======================

// CRC-16/X.25 (FCS ของ AX.25)
uint16_t ax25Fcs(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
  }
  return ~crc;
}

// เขียน address field 7 ไบต์ (callsign 6 ตัว shift ซ้าย 1 บิต + SSID byte)
uint8_t *ax25PutAddr(uint8_t *p, const char *call, uint8_t ssid, bool cBit, bool last) {
  size_t n = strlen(call);
  for (size_t i = 0; i < 6; i++) {
    char c = (i < n) ? toupper(call[i]) : ' ';
    *p++ = (uint8_t)c << 1;
  }
  *p++ = (cBit ? 0x80 : 0x00) | 0x60 | ((ssid & 0x0F) << 1) | (last ? 0x01 : 0x00);
  return p;
}

// สร้างเฟรม UI: [DEST][SRC][0x03][0xF0][INFO][FCS lo][FCS hi]
size_t ax25BuildUI(uint8_t *buf, const char *dst, uint8_t dstSsid,
                   const char *src, uint8_t srcSsid, const char *info) {
  uint8_t *p = buf;
  p = ax25PutAddr(p, dst, dstSsid, true,  false);  // command: C-bit ที่ปลายทาง = 1
  p = ax25PutAddr(p, src, srcSsid, false, true);
  *p++ = 0x03;   // control = UI
  *p++ = 0xF0;   // PID = no layer 3
  size_t n = strnlen(info, MAX_INFO_LEN);
  memcpy(p, info, n);
  p += n;
  uint16_t fcs = ax25Fcs(buf, p - buf);
  *p++ = fcs & 0xFF;
  *p++ = fcs >> 8;
  return p - buf;
}

// ====================== ส่ง ======================

void printHex(const uint8_t *d, size_t n) {
  for (size_t i = 0; i < n; i++) Serial.printf("%02X ", d[i]);
  Serial.print("\r\n");
}

void sendAx25(const char *info) {
  uint8_t frame[16 + MAX_INFO_LEN + 2];
  size_t len = ax25BuildUI(frame, destCall, destSsid, MY_CALL, MY_SSID, info);

  digitalWrite(LED_PIN, HIGH);
  ledOn = true;
  ledOnAt = millis();

  uint32_t t0 = millis();
  int state = radio.transmit(frame, len);
  uint32_t airtime = millis() - t0;

  char src[10], dst[10];
  if (MY_SSID) snprintf(src, sizeof(src), "%s-%d", MY_CALL, MY_SSID);
  else         snprintf(src, sizeof(src), "%s", MY_CALL);
  if (destSsid) snprintf(dst, sizeof(dst), "%s-%d", destCall, destSsid);
  else          snprintf(dst, sizeof(dst), "%s", destCall);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("\r\n[TX] %s>%s:%s  (%u bytes, %lu ms)\r\n",
                  src, dst, info, (unsigned)len, (unsigned long)airtime);
    Serial.print("     ");
    printHex(frame, len);
  } else {
    Serial.printf("\r\n[TX] failed, code %d\r\n", state);
  }
  radio.sleep();
}

// ====================== terminal ======================

void printPrompt() {
  Serial.print("> ");
  Serial.print(inputLine);
}

void printStatus() {
  Serial.printf("[INFO] SRC=%s-%d  DEST=%s-%d  beacon=%s (%d s)\r\n",
                MY_CALL, MY_SSID, destCall, destSsid,
                beaconOn ? "on" : "off", BEACON_INTERVAL_MS / 1000);
}

void printHelp() {
  Serial.print("\r\nคำสั่ง:\r\n"
               "  <ข้อความ> + Enter    ส่งข้อความเป็นเฟรม AX.25\r\n"
               "  /dest CALL[-SSID]   ตั้งปลายทาง\r\n"
               "  /beacon on|off      เปิด/ปิดบีคอนอัตโนมัติ\r\n"
               "  /status             แสดงค่าปัจจุบัน\r\n"
               "  /help               แสดงคำสั่ง\r\n");
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("/")) {
    String cmd = line, arg = "";
    int sp = line.indexOf(' ');
    if (sp > 0) { cmd = line.substring(0, sp); arg = line.substring(sp + 1); arg.trim(); }
    cmd.toLowerCase();

    if (cmd == "/help") {
      printHelp();
    } else if (cmd == "/status") {
      printStatus();
    } else if (cmd == "/beacon") {
      arg.toLowerCase();
      if (arg == "on") beaconOn = true;
      else if (arg == "off") beaconOn = false;
      printStatus();
    } else if (cmd == "/dest" && arg.length() > 0) {
      arg.toUpperCase();
      int dash = arg.indexOf('-');
      String call = dash > 0 ? arg.substring(0, dash) : arg;
      destSsid = dash > 0 ? constrain(arg.substring(dash + 1).toInt(), 0, 15) : 0;
      strncpy(destCall, call.c_str(), 6);
      destCall[6] = '\0';
      printStatus();
    } else {
      Serial.print("[INFO] ไม่รู้จักคำสั่ง พิมพ์ /help\r\n");
    }
    return;
  }

  if (line.length() > MAX_INFO_LEN) line = line.substring(0, MAX_INFO_LEN);
  sendAx25(line.c_str());
}

void readTerminal() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (inputLine.length()) {
        Serial.print("\r\n");
        handleLine(inputLine);
        inputLine = "";
        printPrompt();
      }
    } else if (c == 0x08 || c == 0x7F) {          // backspace
      if (inputLine.length()) {
        inputLine.remove(inputLine.length() - 1);
        Serial.print("\b \b");
      }
    } else if (c >= 0x20 && inputLine.length() < MAX_INFO_LEN) {
      inputLine += c;
      Serial.print(c);                            // echo (PuTTY ไม่ echo เอง)
    }
  }
}

// ====================== setup / loop ======================

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.print("\r\n=== SX1278 AX.25 Transmitter ===\r\n");

  SPI.begin(18, 19, 23, LORA_NSS);

  ConfigLoRa_t config;
  config.frequency       = LORA_FREQ;
  config.bandwidth       = LORA_BW;
  config.spreadingFactor = LORA_SF;
  config.codingRate      = LORA_CR;
  config.syncWord        = LORA_SYNC_WORD;
  config.power           = LORA_POWER;
  config.preambleLength  = LORA_PREAMBLE;
  int state = radio.begin(config);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[TX] radio.begin failed, code %d\r\n", state);
    Serial.print("ตรวจสอบสายไฟ / ไฟเลี้ยง 3.3V / ขา NSS, RST\r\n");
    while (true) {                                // กะพริบเร็ว = error
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }
  }
  radio.setCRC(true);

  Serial.print("[TX] Radio OK\r\n");
  printStatus();
  printHelp();
  printPrompt();
}

void loop() {
  readTerminal();

  if (beaconOn && (seq == 0 || millis() - lastBeacon >= BEACON_INTERVAL_MS)) {
    lastBeacon = millis();
    char info[64];
    snprintf(info, sizeof(info), "Beacon #%lu uptime=%lus",
             (unsigned long)seq, (unsigned long)(millis() / 1000));
    seq++;
    sendAx25(info);
    printPrompt();
  }

  if (ledOn && millis() - ledOnAt >= LED_ON_MS) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }
}
