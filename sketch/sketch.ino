#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <MFRC522.h>

// ============================================================
// ROUTE-LIVE TRANSIT FLEET SYSTEM — ESP8266 GPS & RFID NODE
// ============================================================

// ============================================================
// 1. PIN CONFIGURATION (ESP8266 NodeMCU / D1 Mini)
// ============================================================
// D0 (GPIO16) -> WiFi Connection LED
// D1 (GPIO5)  -> Server / Data TX LED
// D2 (GPIO4)  -> RC522 SDA / SS
// D3 (GPIO0)  -> RC522 RST
// D4 (GPIO2)  -> NEO-6M GPS TX -> ESP8266 RX (SoftwareSerial)
// D5 (GPIO14) -> RC522 SCK
// D6 (GPIO12) -> RC522 MISO
// D7 (GPIO13) -> RC522 MOSI
// D8 (GPIO15) -> Card Scan Feedback LED & Buzzer

#define PIN_LED_WIFI    D0  // GPIO16: WiFi Status LED
#define PIN_LED_DATA    D1  // GPIO5:  Data Transmission LED
#define PIN_BUZZER_LED  D8  // GPIO15: Card Scan Feedback LED & Buzzer
#define RFID_SS_PIN     D2  // GPIO4:  RC522 Chip Select (SDA/SS)
#define RFID_RST_PIN    D3  // GPIO0:  RC522 Reset (RST)
#define GPS_RX_PIN      D4  // GPIO2:  NEO-6M GPS Module TX -> ESP RX

// ============================================================
// 2. HARDWARE & SERVER ENDPOINTS CONFIGURATION
// ============================================================
const char* HARDWARE_CODE   = "HW-AA26CB10";                         // Hardware Code configured in Admin Portal
const char* GPS_API_URL     = "http://10.242.210.183:5050/api/gps";  // GPS Telemetry Endpoint
const char* RFID_API_URL    = "http://10.242.210.183:5050/api/rfid"; // RFID Attendance Endpoint

const char* WIFI_SSID       = "bgps_u24";
const char* WIFI_PASS       = "bgps_p24";

// Real-Time Intervals
const unsigned long GPS_CHECK_INTERVAL = 1000;   // Send GPS telemetry every 1000ms (1 second)
const unsigned long RFID_DEBOUNCE_MS   = 2000;   // 2 seconds debounce for same card

// ============================================================
// 3. HARDWARE PERIPHERALS INITIALIZATION
// ============================================================
MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
SoftwareSerial gpsSerial(GPS_RX_PIN, -1); // RX on D4, TX unused

// ============================================================
// 4. GPS & TELEMETRY DATA STRUCTURES
// ============================================================
struct GpsData {
  double  latitude       = 0.0;
  double  longitude      = 0.0;
  float   speedKmh       = 0.0;
  float   headingDeg     = 0.0;
  float   altitudeMeters = 0.0;
  int     satellites     = 0;
  float   hdop           = 1.0;
  bool    isValid        = false;
};

GpsData currentGps;

// Track Last Sent Coordinates (to send only on position change)
double lastSentLat = 0.0;
double lastSentLng = 0.0;
bool   hasSentInitialFix = false;

// RFID State
unsigned long lastRfidScanTime = 0;
String        lastScannedRfid   = "";
unsigned long feedbackUntil     = 0;

// Data Success LED Timer
unsigned long dataLedUntil      = 0;

// GPS Check Timer
unsigned long lastGpsCheckTime = 0;

// ============================================================
// 5. LIGHTWEIGHT NMEA GPS PARSER
// ============================================================
double parseNmeaCoord(const String& raw, const String& dir) {
  if (raw.length() < 4) return 0.0;
  int dotIdx = raw.indexOf('.');
  if (dotIdx < 2) return 0.0;

  int degLen = (dotIdx == 4) ? 2 : 3;
  double degrees = raw.substring(0, degLen).toDouble();
  double minutes = raw.substring(degLen).toDouble();
  double decimal = degrees + (minutes / 60.0);

  if (dir == "S" || dir == "W") {
    decimal = -decimal;
  }
  return decimal;
}

void parseNmeaSentence(const String& sentence) {
  if (sentence.indexOf("RMC") != -1) {
    int idx = 0;
    String tokens[13];
    int tokenIdx = 0;
    int nextIdx;

    while ((nextIdx = sentence.indexOf(',', idx)) != -1 && tokenIdx < 13) {
      tokens[tokenIdx++] = sentence.substring(idx, nextIdx);
      idx = nextIdx + 1;
    }
    if (tokenIdx < 13) {
      tokens[tokenIdx++] = sentence.substring(idx);
    }

    if (tokenIdx >= 8 && tokens[2] == "A") { // Status A = Valid fix
      currentGps.isValid   = true;
      currentGps.latitude  = parseNmeaCoord(tokens[3], tokens[4]);
      currentGps.longitude = parseNmeaCoord(tokens[5], tokens[6]);
      currentGps.speedKmh  = tokens[7].toFloat() * 1.852; // Knots to km/h
      if (tokenIdx >= 9 && tokens[8].length() > 0) {
        float parsedHeading = tokens[8].toFloat();
        if (parsedHeading >= 0.0 && parsedHeading <= 360.0) {
          currentGps.headingDeg = parsedHeading;
        } else {
          currentGps.headingDeg = 0.0;
        }
      }
    }
  } else if (sentence.indexOf("GGA") != -1) {
    int idx = 0;
    String tokens[15];
    int tokenIdx = 0;
    int nextIdx;

    while ((nextIdx = sentence.indexOf(',', idx)) != -1 && tokenIdx < 15) {
      tokens[tokenIdx++] = sentence.substring(idx, nextIdx);
      idx = nextIdx + 1;
    }
    if (tokenIdx < 15) {
      tokens[tokenIdx++] = sentence.substring(idx);
    }

    if (tokenIdx >= 10 && tokens[6].toInt() > 0) {
      currentGps.isValid        = true;
      currentGps.latitude       = parseNmeaCoord(tokens[2], tokens[3]);
      currentGps.longitude      = parseNmeaCoord(tokens[4], tokens[5]);
      currentGps.satellites     = tokens[7].toInt();
      currentGps.hdop           = tokens[8].toFloat();
      currentGps.altitudeMeters = tokens[9].toFloat();
    }
  }
}

void updateGpsFeed() {
  while (gpsSerial.available()) {
    char c = (char)gpsSerial.read();
    if (c == '\r') continue;
    static String nmeaBuffer = "";
    if (c == '\n') {
      if (nmeaBuffer.length() > 6 && nmeaBuffer.startsWith("$")) {
        parseNmeaSentence(nmeaBuffer);
      }
      nmeaBuffer = "";
    } else {
      if (nmeaBuffer.length() < 120) {
        nmeaBuffer += c;
      }
    }
  }
}

// ============================================================
// 6. WI-FI & ALERT ROUTINE WITH LED & BUZZER INDICATORS
// ============================================================
void setWifiLed(bool state) {
  digitalWrite(PIN_LED_WIFI, state ? HIGH : LOW);
}

void setDataLed(bool state) {
  digitalWrite(PIN_LED_DATA, state ? HIGH : LOW);
}

void blinkDataLed(unsigned long durationMs = 200) {
  setDataLed(true);
  dataLedUntil = millis() + durationMs;
}

void setFeedbackAlert(bool state) {
  digitalWrite(PIN_BUZZER_LED, state ? HIGH : LOW);
}

void triggerFeedback(unsigned long durationMs = 1000) {
  setFeedbackAlert(true);
  feedbackUntil = millis() + durationMs;
}

void setupWiFi() {
  Serial.println();
  Serial.println("========================================");
  Serial.print("🌐 [WiFi] Connecting to: ");
  Serial.println(WIFI_SSID);
  Serial.println("========================================");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 35) {
    setWifiLed(attempts % 2 == 0);
    delay(200);
    Serial.print(".");
    attempts++;
  }

  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    setWifiLed(true);
    Serial.println("✅ [WiFi] Connected!");
    Serial.print("📍 [WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    setWifiLed(false);
    Serial.println("❌ [WiFi] Connection Failed! Will retry in background.");
  }
}

// ============================================================
// 7. SEPARATE GPS DISPATCH -> /api/gps
// (Continuously transmits GPS telemetry to server)
// ============================================================
bool sendGpsUpdate() {
  if (!currentGps.isValid || (currentGps.latitude == 0.0 && currentGps.longitude == 0.0)) {
    static unsigned long lastFixWarning = 0;
    if (millis() - lastFixWarning >= 4000) {
      Serial.println("⏳ [GPS] Waiting for GPS satellite lock (ensure antenna is near window/outdoors)...");
      lastFixWarning = millis();
    }
    return false; // No valid GPS fix with real coordinates yet
  }

  if (WiFi.status() != WL_CONNECTED) {
    setWifiLed(false);
    setupWiFi();
    if (WiFi.status() != WL_CONNECTED) return false;
  }
  setWifiLed(true);

  WiFiClient client;
  HTTPClient http;

  if (!http.begin(client, GPS_API_URL)) {
    return false;
  }

  http.setTimeout(2000);
  http.addHeader("Content-Type", "application/json");

  int rssi         = WiFi.RSSI();
  int batteryLevel = 95;

  // Format GPS JSON Payload using hardware_code
  String json = "{";
  json += "\"hardware_code\":\"" + String(HARDWARE_CODE) + "\",";
  json += "\"latitude\":" + String(currentGps.latitude, 6) + ",";
  json += "\"longitude\":" + String(currentGps.longitude, 6) + ",";
  json += "\"speed\":" + String(currentGps.speedKmh, 1) + ",";
  json += "\"heading\":" + String(currentGps.headingDeg, 1) + ",";
  json += "\"altitude\":" + String(currentGps.altitudeMeters, 1) + ",";
  json += "\"satellites\":" + String(currentGps.satellites) + ",";
  json += "\"hdop\":" + String(currentGps.hdop, 2) + ",";
  json += "\"battery_level\":" + String(batteryLevel) + ",";
  json += "\"signal_strength\":" + String(rssi);
  json += "}";

  Serial.println();
  Serial.println("🛰️  >>> SENDING RAW GPS FIX TO /api/gps <<<");
  Serial.print("📦 Payload: ");
  Serial.println(json);

  int httpCode = http.POST(json);

  if (httpCode >= 200 && httpCode < 300) {
    Serial.print("✅ [GPS Server Response] Code: ");
    Serial.println(httpCode);
    // Blink D1 LED only on successful server transmission
    blinkDataLed(200);

    lastSentLat = currentGps.latitude;
    lastSentLng = currentGps.longitude;
    hasSentInitialFix = true;
  } else {
    Serial.print("❌ [GPS Error]: ");
    Serial.println(httpCode > 0 ? String(httpCode) : http.errorToString(httpCode));
  }

  http.end();
  return (httpCode >= 200 && httpCode < 300);
}

// ============================================================
// 8. SEPARATE RFID DISPATCH -> /api/rfid
// (Transmits only when an RFID card is detected/scanned)
// ============================================================
bool sendRfidScan(const String& uid) {
  if (uid.length() == 0) return false;

  if (WiFi.status() != WL_CONNECTED) {
    setWifiLed(false);
    setupWiFi();
    if (WiFi.status() != WL_CONNECTED) return false;
  }
  setWifiLed(true);

  WiFiClient client;
  HTTPClient http;

  if (!http.begin(client, RFID_API_URL)) {
    return false;
  }

  http.setTimeout(2500);
  http.addHeader("Content-Type", "application/json");

  // Format RFID JSON Payload using hardware_code
  String json = "{";
  json += "\"hardware_code\":\"" + String(HARDWARE_CODE) + "\",";
  json += "\"rfid\":\"" + uid + "\"";
  json += "}";

  Serial.println();
  Serial.println("💳 ========================================");
  Serial.println("⚡ [RFID TAP DETECTED] SENDING TO /api/rfid");
  Serial.print("📦 Payload: ");
  Serial.println(json);

  int httpCode = http.POST(json);

  if (httpCode >= 200 && httpCode < 300) {
    Serial.print("✅ [RFID Server Response] Code: ");
    Serial.println(httpCode);
    String response = http.getString();
    Serial.print("📩 [Server Data]    : ");
    Serial.println(response);

    // Blink D1 LED only on successful server transmission
    blinkDataLed(200);
  } else {
    Serial.print("❌ [RFID Error]: ");
    Serial.println(httpCode > 0 ? String(httpCode) : http.errorToString(httpCode));
  }
  Serial.println("========================================");

  http.end();
  return (httpCode >= 200 && httpCode < 300);
}

// ============================================================
// 9. SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Configure LEDs & Buzzer
  pinMode(PIN_LED_WIFI, OUTPUT);
  pinMode(PIN_LED_DATA, OUTPUT);
  pinMode(PIN_BUZZER_LED, OUTPUT);
  setWifiLed(false);
  setDataLed(false);
  setFeedbackAlert(false);

  Serial.println();
  Serial.println("========================================");
  Serial.println("   ROUTE-LIVE ESP8266 GPS & RFID NODE   ");
  Serial.println("========================================");
  Serial.print("🏷️  Hardware Code: ");
  Serial.println(HARDWARE_CODE);
  Serial.print("🛰️  GPS Endpoint : ");
  Serial.println(GPS_API_URL);
  Serial.print("💳 RFID Endpoint: ");
  Serial.println(RFID_API_URL);
  Serial.print("🔊 Feedback Pin : D8 (LED + Buzzer, 1s pulse)");
  Serial.println();
  Serial.println("========================================");

  // Initialize GPS Serial
  gpsSerial.begin(9600);
  Serial.println("🛰️  [GPS] NEO-6M Serial Active on D4 (9600 baud)");

  // Initialize SPI & RC522 RFID
  SPI.begin();
  rfid.PCD_Init();
  delay(100);
  rfid.PCD_AntennaOn();
  delay(100);

  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print("💳 [RFID] RC522 Version: 0x");
  Serial.println(version, HEX);
  if (version == 0x00 || version == 0xFF) {
    Serial.println("⚠️  [RFID] WARNING: RC522 reader not detected! Check SPI wiring.");
  } else {
    Serial.println("✅ [RFID] RC522 Reader Ready.");
  }

  // Connect to Wi-Fi
  setupWiFi();

  Serial.println();
  Serial.println("✨ System Ready — Listening for GPS movement & RFID taps ✨");
  Serial.println();
}

// ============================================================
// 10. MAIN LOOP
// ============================================================
void loop() {
  // 1. Continuously process incoming GPS NMEA Stream
  updateGpsFeed();

  // 2. Maintain Wi-Fi LED Status
  if (WiFi.status() == WL_CONNECTED) {
    setWifiLed(true);
  } else {
    setWifiLed(false);
  }

  unsigned long now = millis();

  // 3. Handle 1s Feedback Alert (Non-blocking turn-off for D8 Buzzer & LED)
  if (feedbackUntil > 0 && now >= feedbackUntil) {
    setFeedbackAlert(false);
    feedbackUntil = 0;
  }

  // 4. Handle Data Success LED Blink (Non-blocking turn-off for D1 LED)
  if (dataLedUntil > 0 && now >= dataLedUntil) {
    setDataLed(false);
    dataLedUntil = 0;
  }

  // 5. Check for RFID Card Tap (SEND ONLY IF RFID IS DETECTED)
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    String uid = "";
    for (byte i = 0; i < rfid.uid.size; i++) {
      if (rfid.uid.uidByte[i] < 0x10) uid += "0";
      uid += String(rfid.uid.uidByte[i], HEX);
    }
    uid.toUpperCase();

    // Debounce check
    if (uid.length() > 0 && (uid != lastScannedRfid || (now - lastRfidScanTime >= RFID_DEBOUNCE_MS))) {
      lastScannedRfid  = uid;
      lastRfidScanTime = now;

      // Trigger 1-second active alert on D8 (LED + Buzzer)
      triggerFeedback(1000);

      // Send immediately to /api/rfid
      sendRfidScan(uid);
    }

    // Halt card and stop crypto
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }

  // 5. Check GPS Position (SEND TO /api/gps ONLY IF LAT/LNG IS DIFFERENT FROM PREVIOUS)
  if (now - lastGpsCheckTime >= GPS_CHECK_INTERVAL) {
    lastGpsCheckTime = now;
    sendGpsUpdate();
  }

  delay(20);
}