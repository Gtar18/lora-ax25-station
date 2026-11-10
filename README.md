# LoRa AX.25 Station

ESP32 DevKit V1 + Ra-01 (SX1278, 433 MHz) ส่งเฟรม **AX.25** ผ่าน **GFSK / FSK / LoRa** ด้วย [RadioLib](https://github.com/jgromes/RadioLib)
พร้อมหน้าเว็บควบคุมบอร์ด ส่งรูป ping และดู log ว่า packet ไหนรับได้ / รับไม่ได้

- เลือก modem ได้: **GFSK** (ค่าเริ่มต้น 9.6 kbps), FSK, LoRa ปรับ bitrate 0.5 - 300 kbps ได้จากหน้าเว็บหรือคำสั่ง
- เฟรม AX.25 UI ตามมาตรฐาน (address, control `0x03`, PID `0xF0`, FCS CRC-16/X.25)
- Ping ไป-กลับ พร้อม RTT และ RSSI/SNR ทั้งสองฝั่ง
- ส่งรูปแบ่งเป็น packet ตัวรับขอส่งซ้ำเฉพาะ packet ที่หาย (selective repeat) ตรวจ CRC32 ทั้งไฟล์
- หน้าเว็บ: terminal ของแต่ละบอร์ด, อัปโหลด/ส่งรูป, ตาราง packet (คลิก packet ที่หายเพื่อส่งใหม่), ping, log ทั้งหมด

> ส่งผ่านโมดูล SX1278 โดยตรง (ไม่ใช่ AFSK 1200 ผ่านวิทยุเสียง) จึงคุยกันได้ระหว่างบอร์ดชุดนี้ด้วยกัน
> ไม่สามารถรับด้วยวิทยุ packet VHF ทั่วไป

## ฮาร์ดแวร์

| Ra-01 (SX1278) | ESP32 DevKit V1 (D.. / G..) |
|---|---|
| 3.3V | **3V3** (ห้ามต่อ 5V / VIN) |
| GND | GND |
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |
| NSS | 5 |
| RST | 14 |
| DIO0 | 26 |
| DIO1 | ไม่ต้องต่อ |

**ต้องต่อเสาอากาศก่อนส่งทุกครั้ง**

## ค่าวิทยุ

ทุกบอร์ดต้องตั้งให้ตรงกัน (ใช้ปุ่ม **ใช้กับทุกบอร์ด** ในหน้าเว็บ หรือคำสั่ง `/radioall`) ค่าจะบันทึกถาวรในบอร์ด

| Modem | ค่าเริ่มต้น | ปรับได้ | ข้อมูลต่อเฟรม |
|---|---|---|---|
| **GFSK** (ค่าเริ่มต้น) | 9.6 kbps, BT 0.5, deviation/RX BW = auto | bitrate 0.5 - 300 kbps, deviation, RX BW, BT 0.3 - 1.0 | 46 bytes (FIFO 64) |
| FSK | เหมือน GFSK แต่ไม่มี shaping | เหมือน GFSK | 46 bytes |
| LoRa | SF9, BW 125 kHz, CR 4/7 | SF 6 - 12, BW | 237 bytes |

ทั้งหมด: 433 MHz (410 - 525), 17 dBm (2 - 20)

deviation แบบ auto = bitrate/2 (ไม่เกินข้อจำกัด dev + bitrate/2 ≤ 250 kHz), RX BW แบบ auto = 2.5 × (dev + bitrate/2)

## โครงสร้าง

```
LoRa_Node/          firmware (ใช้ตัวเดียวกันทุกบอร์ด)
tools/web_station.py  เว็บเซิร์ฟเวอร์ ควบคุมทุกบอร์ดผ่านหน้าเว็บ
tools/station.py      เวอร์ชัน command line (send / ping / rx / web)
web/index.html      หน้าเว็บ
images_to_send/     วางรูปที่จะส่ง
received_images/    รูปที่รับได้
logs/               events.jsonl (log ทุก packet), outbox/, inbox/
SX1278_TX, SX1278_RX  ตัวอย่างแยกส่ง/รับ แบบง่าย (ไม่ใช้กับเว็บ)
```

## ติดตั้ง

### Firmware
1. Arduino IDE / arduino-cli + ESP32 core (`esp32:esp32`) + ไลบรารี **RadioLib 8.x**
2. อัปโหลด `LoRa_Node` ลงทุกบอร์ด
   ```
   arduino-cli compile --fqbn "esp32:esp32:esp32:LoopCore=0,EventsCore=0,FlashFreq=40" -u -p COM5 LoRa_Node
   ```
   (Arduino IDE: บอร์ด *ESP32 Dev Module*, Arduino Runs On = Core 0, Events Run On = Core 0, Flash Frequency = 40MHz)
   ถ้าค้างที่ `Connecting...` ให้กดปุ่ม BOOT ค้าง

   > ใช้ core 0 และ flash 40 MHz เพราะบอร์ดหนึ่งในชุดทดสอบมีบิตเสียใน instruction cache ของ core 1
   > (crash แบบ IllegalInstruction) ถ้าบอร์ดของคุณปกติ จะใช้ค่าเริ่มต้นของ *DOIT ESP32 DEVKIT V1* ก็ได้
3. ตั้งชื่อแต่ละบอร์ด (เปิด PuTTY 115200 หรือใช้ terminal ในหน้าเว็บ) ค่าจะบันทึกถาวรในบอร์ด
   ```
   /mycall N0CALL-1      (อีกบอร์ด: N0CALL-2)
   /dest   N0CALL-2      (อีกบอร์ด: N0CALL-1)
   ```
   ถ้ามีใบอนุญาตวิทยุสมัครเล่น ให้ใช้นามเรียกขานของตัวเอง

### PC
```
python -m pip install pyserial pillow
python tools/web_station.py              # หาบอร์ดอัตโนมัติ หรือ --ports COM5,COM6
```
เปิด http://localhost:8000/web/

> ขณะที่ web_station.py ทำงาน พอร์ต COM จะถูกใช้อยู่ ต้องหยุดก่อนอัปโหลด firmware หรือเปิด PuTTY

## คำสั่ง (terminal ในหน้าเว็บ)

```
<ข้อความ>                          ส่งข้อความไปยัง dest
/ping [n]                          ping
/send <ไฟล์> [loss=0.2] [rounds=5] [px=320] [q=60] [raw]
/images                            รายการรูปใน images_to_send
/resend <transfer> <1,5,9|all>     ส่ง packet ที่หายใหม่
/mycall CALL-SSID   /dest CALL-SSID   /beacon on|off   /status   /clear   /help

/modem lora|fsk|gfsk               เลือก modem
/bitrate <kbps>                    FSK/GFSK 0.5 - 300 kbps
/dev <kHz>|auto   /rxbw <kHz>|auto   /shaping 0.3|0.5|0.7|1.0
/sf <6-12>   /bw <kHz>             LoRa
/freq <MHz>   /power <dBm>
/radio   /radio reset              ดูค่า / คืนค่าเริ่มต้น (GFSK 9.6 kbps)
/radioall <คำสั่งวิทยุ>              ใช้กับทุกบอร์ดพร้อมกัน เช่น /radioall /bitrate 19.2
```

ใน PuTTY (115200, แนะนำตั้ง Translation เป็น UTF-8) ใช้ได้ทุกคำสั่งที่เป็นของบอร์ด รวมถึงคำสั่งวิทยุ มีสีให้อ่านง่าย
(ยกเว้น `/send`, `/images`, `/resend`, `/radioall` ที่เป็นของโปรแกรมบน PC)

### Command line (ไม่ใช้เว็บ)
```
python tools/station.py rx   --port COM6
python tools/station.py send --port COM5 test_pattern.jpg
python tools/station.py ping --port COM5 -n 10
```

## โปรโตคอลส่งรูป

อยู่ใน info field ของเฟรม AX.25 ทุกแพ็กเก็ตขึ้นต้นด้วยไบต์ `0xA5`

| ชนิด | รูปแบบ | ความหมาย |
|---|---|---|
| `S` | fid, total(2), size(4), crc32(4), chunk(1), name | เริ่มไฟล์ |
| `D` | fid, seq(2), total(2), data(≤chunk) | ข้อมูลแต่ละชิ้น (LoRa 230 / GFSK 39 bytes) |
| `E` | fid, total(2) | จบรอบ → ตัวรับตอบ `K` หรือ `N` |
| `N` | fid, count(2), seq(2)… | รายการ packet ที่หาย |
| `K` | fid | ได้ครบแล้ว |
| `F` | fid | ตัวส่งหยุดส่ง (ตัวรับปิดไฟล์) |

ที่ LoRa SF9 ได้ประมาณ 120 B/s (รูป 5 KB ≈ 40 วินาที) โปรแกรมจึงย่อรูปเป็น JPEG ด้านยาว 320 px ก่อนส่ง

## สถานะการทดสอบ

- LoRa SF9: ping, ข้อความ, ส่งรูป, ส่ง packet ที่หายใหม่ — ทดสอบกับบอร์ดจริงแล้ว ใช้งานได้
- GFSK / FSK: บอร์ดส่งได้ (9.6 kbps ≈ 30 ms/เฟรม) แต่**ยังไม่ได้ทดสอบรับระหว่างสองบอร์ด**
  ถ้า ping ไม่ตอบ ให้กดปุ่ม **LoRa SF9** ในหน้าเว็บเพื่อกลับไปใช้ค่าที่ทดสอบแล้ว
