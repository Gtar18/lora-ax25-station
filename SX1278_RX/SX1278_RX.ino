/*
  SX1278 LoRa AX.25 Receiver (ตัวรับ AX.25 + terminal) - แบบ Interrupt
  บอร์ด : ESP32 DevKit V1 (ESP32-WROOM-32)
  ไลบรารี: RadioLib v8.x

  รับแพ็กเก็ต LoRa แล้วถอดเป็นเฟรม AX.25 ตรวจ FCS
  แสดงผลแบบ TNC2 monitor:  SRC>DEST,DIGI*:ข้อความ  พร้อม RSSI / SNR
  เปิดดูด้วย PuTTY / Serial Monitor ที่ 115200
  คำสั่งใน terminal (กดปุ่มเดียว):
    h  เปิด/ปิดการแสดง hex dump ของเฟรม
    s  แสดงสถิติ

  การต่อสาย (Ra-01 -> ESP32, บอร์ดที่พิมพ์ G.. ใช้เลขเดียวกัน)
    3.3V -> 3V3 | GND -> GND
    SCK -> G18 | MISO -> G19 | MOSI -> G23
    NSS -> G5  | RST  -> G14 | DIO0 -> G26 | DIO1 -> ไม่ต่อ
*/

#include <RadioLib.h>

// ---------------- ขา ----------------
#define LORA_NSS   5
#define LORA_DIO0  26
#define LORA_RST   14
#define LORA_DIO1  RADIOLIB_NC   // ไม่ได้ต่อ DIO1
#define LED_PIN    2

// ---------------- ค่าวิทยุ (ต้องตรงกับตัวส่ง) ----------------
#define LORA_FREQ       RADIOLIB_UNIT_MEGA(433)
#define LORA_BW         RADIOLIB_UNIT_KILO(125)
#define LORA_SF         9
#define LORA_CR         7
#define LORA_SYNC_WORD  0x12
#define LORA_POWER      17
#define LORA_PREAMBLE   8

#define LED_ON_MS  500

SX1278 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1);

volatile bool receivedFlag = false;
void IRAM_ATTR setFlag(void) { receivedFlag = true; }

bool     showHex  = true;
uint32_t rxOk = 0, rxBadFcs = 0, rxNotAx25 = 0, rxCrcErr = 0;
uint32_t ledOnAt = 0;
bool     ledOn   = false;

// ====================== AX.25 decoder ======================

uint16_t ax25Fcs(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
  }
  return ~crc;
}

// แปลง address 7 ไบต์ -> "CALL-SSID"
String ax25Addr(const uint8_t *a) {
  String s;
  for (int i = 0; i < 6; i++) {
    char c = a[i] >> 1;
    if (c != ' ') s += c;
  }
  uint8_t ssid = (a[6] >> 1) & 0x0F;
  if (ssid) s += "-" + String(ssid);
  return s;
}

void printHex(const uint8_t *d, size_t n) {
  Serial.print("     ");
  for (size_t i = 0; i < n; i++) Serial.printf("%02X ", d[i]);
  Serial.print("\r\n");
}

void handleFrame(const uint8_t *f, size_t len, float rssi, float snr) {
  // ขั้นต่ำ: dest(7) + src(7) + control + PID + FCS(2) = 18
  if (len < 18) {
    rxNotAx25++;
    Serial.printf("[RX] ไม่ใช่ AX.25 (%u bytes)  RSSI=%.1f\r\n", (unsigned)len, rssi);
    printHex(f, len);
    return;
  }

  uint16_t fcsCalc = ax25Fcs(f, len - 2);
  uint16_t fcsRecv = f[len - 2] | (f[len - 1] << 8);
  if (fcsCalc != fcsRecv) {
    rxBadFcs++;
    Serial.printf("[RX] FCS ผิด (calc %04X, recv %04X)  RSSI=%.1f\r\n", fcsCalc, fcsRecv, rssi);
    printHex(f, len);
    return;
  }

  // อ่าน address field จนเจอ extension bit (bit0 = 1)
  size_t p = 0;
  int nAddr = 0;
  bool endFound = false;
  String dst, src, path;
  while (p + 7 <= len - 2) {
    const uint8_t *a = f + p;
    String call = ax25Addr(a);
    if (nAddr == 0)      dst = call;
    else if (nAddr == 1) src = call;
    else                 path += "," + call + ((a[6] & 0x80) ? "*" : "");  // H-bit = ผ่าน digi แล้ว
    nAddr++;
    p += 7;
    if (a[6] & 0x01) { endFound = true; break; }
  }
  if (!endFound || nAddr < 2 || p + 2 > len - 2) {
    rxNotAx25++;
    Serial.print("[RX] address field ไม่ถูกต้อง\r\n");
    printHex(f, len);
    return;
  }

  uint8_t control = f[p++];
  uint8_t pid     = f[p++];
  String info;
  for (; p < len - 2; p++) info += (char)f[p];

  rxOk++;
  Serial.printf("[RX] %s>%s%s:%s\r\n", src.c_str(), dst.c_str(), path.c_str(), info.c_str());
  Serial.printf("     ctrl=0x%02X pid=0x%02X  %u bytes  RSSI=%.1f dBm  SNR=%.1f dB  FCS OK\r\n",
                control, pid, (unsigned)len, rssi, snr);
  if (showHex) printHex(f, len);
}

// ====================== setup / loop ======================

void printStats() {
  Serial.printf("[STAT] ok=%lu  fcs_bad=%lu  not_ax25=%lu  lora_crc_err=%lu  hex=%s\r\n",
                (unsigned long)rxOk, (unsigned long)rxBadFcs, (unsigned long)rxNotAx25,
                (unsigned long)rxCrcErr, showHex ? "on" : "off");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.print("\r\n=== SX1278 AX.25 Receiver ===\r\n");

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
    Serial.printf("[RX] radio.begin failed, code %d\r\n", state);
    while (true) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }
  }

  radio.setCRC(true);
  radio.setIrqAction(setFlag);

  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[RX] startReceive failed, code %d\r\n", state);
    while (true) { delay(1000); }
  }
  Serial.print("[RX] Listening for AX.25 frames...  (กด h = hex on/off, s = สถิติ)\r\n");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'h' || c == 'H') {
      showHex = !showHex;
      Serial.printf("[INFO] hex dump %s\r\n", showHex ? "on" : "off");
    } else if (c == 's' || c == 'S') {
      printStats();
    }
  }

  if (ledOn && millis() - ledOnAt >= LED_ON_MS) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }

  if (!receivedFlag) return;
  receivedFlag = false;

  uint8_t buf[256];
  size_t len = radio.getPacketLength();
  if (len > sizeof(buf)) len = sizeof(buf);
  int state = radio.readData(buf, len);

  if (state == RADIOLIB_ERR_NONE) {
    digitalWrite(LED_PIN, HIGH);
    ledOn = true;
    ledOnAt = millis();
    handleFrame(buf, len, radio.getRSSI(), radio.getSNR());
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    rxCrcErr++;
    Serial.print("[RX] LoRa CRC error\r\n");
  } else {
    Serial.printf("[RX] readData failed, code %d\r\n", state);
  }

  radio.startReceive();
}
