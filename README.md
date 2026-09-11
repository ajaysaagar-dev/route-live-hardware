# Route-Live — ESP8266 GPS & RFID Telemetry Node

Embedded C++ / Arduino firmware for **ESP8266** (NodeMCU / D1 Mini) interfaced with **NEO-6M GPS** and **RC522 RFID Reader** transmitting telemetry and student scans via HTTP REST to the Route-Live server.

---

## 1. Hardware Pinout Table (ESP8266 ↔ Peripherals)

| ESP8266 Pin | GPIO | Connected to | Purpose / Behavior |
| :--- | :---: | :--- | :--- |
| **D0** | `GPIO16` | **WiFi Connection LED** | Solid `HIGH` when connected, blinks while connecting, `LOW` if offline. |
| **D1** | `GPIO5` | **Server / Data TX LED** | Pulses `HIGH` during HTTP POST payload transmission. |
| **D2** | `GPIO4` | **RC522 SDA / SS** | RFID SPI Chip Select. |
| **D3** | `GPIO0` | **RC522 RST** | RFID Reset line. |
| **D4** | `GPIO2` | **NEO-6M GPS TX** | GPS NMEA serial stream input (SoftwareSerial at 9600 baud). |
| **D5** | `GPIO14` | **RC522 SCK** | SPI Clock. |
| **D6** | `GPIO12` | **RC522 MISO** | SPI Data Input (RC522 → ESP8266). |
| **D7** | `GPIO13` | **RC522 MOSI** | SPI Data Output (ESP8266 → RC522). |
| **D8** | `GPIO15` | **Unused** | Available. |

---

## 2. Configuration in `sketch.ino`

```cpp
// 1. Hardware Identity
const char* HARDWARE_ID  = "12345"; // Custom Hardware Identifier configured in Admin Web

// 2. Server Endpoints
const char* GPS_API_URL  = "http://10.242.210.183:5050/api/gps";  // GPS Telemetry Endpoint
const char* RFID_API_URL = "http://10.242.210.183:5050/api/rfid"; // RFID Attendance Endpoint

// 3. Wi-Fi Settings
const char* WIFI_SSID    = "bgps_u24";
const char* WIFI_PASS    = "bgps_p24";
```

---

## 3. Endpoints & Data Payloads

### A. Real-Time GPS Telemetry (`POST /api/gps`)
*Sent only when latitude or longitude changes from the previous reading.*

```json
{
  "hardware_id": "12345",
  "latitude": 11.234567,
  "longitude": 77.891234,
  "speed": 38.5,
  "heading": 180.0,
  "altitude": 220.0,
  "satellites": 9,
  "hdop": 1.1,
  "battery_level": 95,
  "signal_strength": -65
}
```

### B. Real-Time RFID Student Attendance (`POST /api/rfid`)
*Sent only when an active card tap is detected.*

```json
{
  "hardware_id": "12345",
  "rfid": "31C55855"
}
```

---

## 4. Arduino IDE Upload Settings
- **Board:** NodeMCU 1.0 (ESP-12E Module) / LOLIN(WEMOS) D1 R2 & mini
- **Upload Speed:** 115200 (or 921600)
- **CPU Frequency:** 80 MHz (or 160 MHz)
- **Flash Size:** 4MB (FS:2MB OTA:~1019KB)
- **Serial Monitor Baud Rate:** `115200 baud`

