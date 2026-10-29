
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <SPI.h>
#include <SD.h>
#include <U8g2lib.h>
#include <HardwareSerial.h>
#include <math.h>

// ---------------- PINS ----------------
#define BUZZER_PIN 0
#define GSM_RX_PIN 16
#define GSM_TX_PIN 17

#define OLED_SDA_PIN 21
#define OLED_SCL_PIN 22

#define SD_CS_PIN   5
#define SD_SCK_PIN  18
#define SD_MISO_PIN 19
#define SD_MOSI_PIN 23

// ---------------- CONFIGURATION ----------------
const uint8_t ESPNOW_CHANNEL = 1;
const uint32_t PACKET_MAGIC = 0x50414E49;
const uint8_t PACKET_VERSION = 1;

// Replace with your caregiver's number.
const char PHONE_NUMBER[] = "+918660308610";

// Set true if your buzzer driver turns ON when GPIO 0 is LOW.
const bool BUZZER_ACTIVE_LOW = false;

enum PacketType : uint8_t {
  TELEMETRY = 1,
  EMERGENCY = 2,
  ACK = 3
};

enum EmergencyReason : uint8_t {
  NONE = 0,
  PANIC_BUTTON_EVENT = 1,
  VOICE_HELP_EVENT = 2,
  MPU_MOTION_EVENT = 3,
  ECG_ABNORMAL_EVENT = 4,
  SPO2_LOW_EVENT = 5,
  TEMP_HIGH_EVENT = 6
};

enum ValidFlags : uint8_t {
  ECG_VALID  = 1,
  HR_VALID   = 2,
  SPO2_VALID = 4,
  TEMP_VALID = 8,
  MPU_VALID  = 16
};

// MUST match Sensor Node exactly.
struct __attribute__((packed)) Packet {
  uint32_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t sequence;
  uint32_t uptimeMs;
  float ecgBpm;
  float heartRateBpm;
  float spo2Percent;
  float temperatureC;
  uint8_t validFlags;
  uint8_t risk;
  uint8_t reason;
  uint16_t ackSequence;
};

static_assert(sizeof(Packet) == 33, "Packet size mismatch");

// ---------------- HARDWARE ----------------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(
  U8G2_R0, U8X8_PIN_NONE
);

HardwareSerial GSM(2);
SPIClass sdSPI(VSPI);

// ---------------- STATE ----------------
QueueHandle_t packetQueue;

Packet latestPacket = {};
bool haveTelemetry = false;
bool sdOK = false;

uint32_t lastDisplayMs = 0;
uint32_t lastLogMs = 0;
uint32_t buzzerUntilMs = 0;
uint32_t lastEventMs[7] = {};

const uint32_t DISPLAY_INTERVAL_MS = 250;
const uint32_t LOG_INTERVAL_MS = 10000;
const uint32_t EVENT_COOLDOWN_MS = 30000;
const uint32_t BUZZER_DURATION_MS = 15000;

bool gsmReady = false;

// ---------------- BUZZER ----------------
void setBuzzer(bool on) {
  bool outputHigh = BUZZER_ACTIVE_LOW ? !on : on;
  digitalWrite(BUZZER_PIN, outputHigh ? HIGH : LOW);
}

void startBuzzer() {
  buzzerUntilMs = millis() + BUZZER_DURATION_MS;
}

void updateBuzzer() {
  if ((int32_t)(buzzerUntilMs - millis()) <= 0) {
    setBuzzer(false);
    return;
  }

  // Repeating siren pattern.
  bool on = ((millis() / 250) % 2) == 0;
  setBuzzer(on);
}

// ---------------- ESP-NOW RECEIVE ----------------
void enqueuePacket(const uint8_t *data, int len) {
  if (len != sizeof(Packet)) return;

  Packet p;
  memcpy(&p, data, sizeof(p));

  if (p.magic != PACKET_MAGIC ||
      p.version != PACKET_VERSION) {
    return;
  }

  if (packetQueue != nullptr) {
    // Keep the callback short so ESP-NOW stays responsive.
    xQueueSend(packetQueue, &p, 0);
  }
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {
  (void)info;
  enqueuePacket(data, len);
}
#else
void onDataReceived(
  const uint8_t *mac,
  const uint8_t *data,
  int len
) {
  (void)mac;
  enqueuePacket(data, len);
}
#endif

// ---------------- GSM HELPERS ----------------
String readGSM(uint32_t timeoutMs) {
  String response;
  uint32_t start = millis();

  while ((uint32_t)(millis() - start) < timeoutMs) {
    while (GSM.available()) {
      char c = (char)GSM.read();
      response += c;
    }

    updateBuzzer();
    delay(2);
  }

  return response;
}

String sendAT(const String &command, uint32_t timeoutMs = 1500) {
  while (GSM.available()) GSM.read();

  GSM.println(command);
  return readGSM(timeoutMs);
}

bool phoneConfigured() {
  return String(PHONE_NUMBER) != "+91XXXXXXXXXX" &&
         strlen(PHONE_NUMBER) >= 10;
}

bool initializeGSM() {
  Serial.println("Initializing SIM800L...");

  GSM.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(500);

  bool found = false;

  for (int i = 0; i < 5; i++) {
    String r = sendAT("AT", 1000);
    if (r.indexOf("OK") >= 0) {
      found = true;
      break;
    }
  }

  if (!found) {
    Serial.println("SIM800L did not respond");
    return false;
  }

  sendAT("ATE0");
  sendAT("AT+CMGF=1");
  sendAT("AT+CNMI=2,1,0,0,0");

  String sim = sendAT("AT+CPIN?", 1500);
  Serial.println(sim);

  String network = sendAT("AT+CREG?", 1500);
  Serial.println(network);

  String signal = sendAT("AT+CSQ", 1500);
  Serial.println(signal);

  Serial.println("GSM AT interface initialized");
  return true;
}

bool sendSMS(const String &message) {
  if (!phoneConfigured()) {
    Serial.println("Set PHONE_NUMBER before sending SMS");
    return false;
  }

  Serial.println("Sending SMS...");

  while (GSM.available()) GSM.read();

  GSM.println("AT+CMGF=1");
  readGSM(500);

  GSM.print("AT+CMGS=\"");
  GSM.print(PHONE_NUMBER);
  GSM.println("\"");

  String prompt = readGSM(3000);

  if (prompt.indexOf(">") < 0) {
    Serial.println("SMS prompt not received");
    Serial.println(prompt);
    return false;
  }

  GSM.print(message);
  GSM.write(26); // Ctrl+Z

  String result = readGSM(10000);
  Serial.println(result);

  return result.indexOf("OK") >= 0;
}

bool makeCall() {
  if (!phoneConfigured()) {
    Serial.println("Set PHONE_NUMBER before calling");
    return false;
  }

  GSM.print("ATD");
  GSM.print(PHONE_NUMBER);
  GSM.println(";");

  Serial.println("Calling caregiver...");

  uint32_t start = millis();

  while ((uint32_t)(millis() - start) < 15000) {
    updateBuzzer();
    delay(20);
  }

  GSM.println("ATH");
  readGSM(1000);

  return true;
}

// ---------------- EVENT TEXT ----------------
String reasonText(uint8_t reason) {
  switch (reason) {
    case PANIC_BUTTON_EVENT: return "PANIC BUTTON";
    case VOICE_HELP_EVENT:   return "VOICE HELP";
    case MPU_MOTION_EVENT:   return "SUDDEN MOTION/FALL";
    case ECG_ABNORMAL_EVENT: return "ABNORMAL ECG/HEART RATE";
    case SPO2_LOW_EVENT:     return "LOW SPO2";
    case TEMP_HIGH_EVENT:    return "TEMPERATURE OUT OF RANGE";
    default:                 return "UNKNOWN EVENT";
  }
}

bool readingValid(uint8_t flag) {
  return (latestPacket.validFlags & flag) != 0;
}

String readingText(float value, bool valid, int decimals = 1) {
  if (!valid || !isfinite(value)) return "--";
  return String(value, decimals);
}

// Count currently abnormal categories.
// ECG and HR are counted as one category.
int countAbnormalConditions(const Packet &p) {
  int count = 0;

  bool heartAbnormal = false;

  if (p.validFlags & ECG_VALID) {
    if (p.ecgBpm < 50 || p.ecgBpm > 120) {
      heartAbnormal = true;
    }
  }

  if (p.validFlags & HR_VALID) {
    if (p.heartRateBpm < 50 || p.heartRateBpm > 120) {
      heartAbnormal = true;
    }
  }

  if (heartAbnormal) count++;

  if ((p.validFlags & SPO2_VALID) &&
      p.spo2Percent < 92) {
    count++;
  }

  if ((p.validFlags & TEMP_VALID) &&
      (p.temperatureC < 35 || p.temperatureC > 38)) {
    count++;
  }

  return count;
}

// ---------------- SD CARD ----------------
void logCSV(const String &event) {
  if (!sdOK) return;

  File file = SD.open("/patient.csv", FILE_APPEND);

  if (!file) {
    Serial.println("Unable to open patient.csv");
    return;
  }

  // Use GSM network clock if available; otherwise log uptime.
  String timestamp = "uptime_" + String(millis() / 1000);

  String clockResponse = sendAT("AT+CCLK?", 500);
  int q1 = clockResponse.indexOf('"');
  int q2 = clockResponse.indexOf('"', q1 + 1);

  if (q1 >= 0 && q2 > q1) {
    timestamp = clockResponse.substring(q1 + 1, q2);
  }

  file.print(timestamp);
  file.print(',');
  file.print(millis());
  file.print(',');
  file.print(readingText(latestPacket.ecgBpm,
                         readingValid(ECG_VALID)));
  file.print(',');
  file.print(readingText(latestPacket.heartRateBpm,
                         readingValid(HR_VALID)));
  file.print(',');
  file.print(readingText(latestPacket.spo2Percent,
                         readingValid(SPO2_VALID)));
  file.print(',');
  file.print(readingText(latestPacket.temperatureC,
                         readingValid(TEMP_VALID)));
  file.print(',');
  file.println(event);

  file.close();
}

// ---------------- OLED ----------------
void displayReadings() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);

  oled.drawStr(0, 10, "PATIENT MONITOR");

  if (!haveTelemetry) {
    oled.drawStr(0, 27, "Waiting for Sensor");
    oled.drawStr(0, 41, "Node via ESP-NOW...");
    oled.sendBuffer();
    return;
  }

  String line1 = "ECG:" +
    readingText(latestPacket.ecgBpm,
                readingValid(ECG_VALID)) +
    " HR:" +
    readingText(latestPacket.heartRateBpm,
                readingValid(HR_VALID));

  String line2 = "SpO2:" +
    readingText(latestPacket.spo2Percent,
                readingValid(SPO2_VALID)) + "%";

  String line3 = "Temp:" +
    readingText(latestPacket.temperatureC,
                readingValid(TEMP_VALID)) + " C";

  oled.drawStr(0, 24, line1.c_str());
  oled.drawStr(0, 36, line2.c_str());
  oled.drawStr(0, 48, line3.c_str());

  if ((uint32_t)(millis() - latestPacket.uptimeMs) < 0) {
    // No action; keeps all displayed fields simple.
  }

  oled.drawStr(0, 62, "ESP-NOW: RECEIVING");
  oled.sendBuffer();
}

void displayEmergency(uint8_t reason) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 12, "!!! EMERGENCY !!!");

  String event = reasonText(reason);
  oled.drawStr(0, 29, event.c_str());

  String temp = "Temp: " +
    readingText(latestPacket.temperatureC,
                readingValid(TEMP_VALID)) + " C";
  oled.drawStr(0, 43, temp.c_str());

  oled.drawStr(0, 60, "Alert processing...");
  oled.sendBuffer();
}

// ---------------- EMERGENCY PROCESSING ----------------
void processEmergency(const Packet &p) {
  uint8_t reason = p.reason;

  if (reason < PANIC_BUTTON_EVENT ||
      reason > TEMP_HIGH_EVENT) {
    return;
  }

  uint32_t now = millis();

  if ((uint32_t)(now - lastEventMs[reason]) <
      EVENT_COOLDOWN_MS) {
    Serial.println("Duplicate emergency suppressed");
    return;
  }

  lastEventMs[reason] = now;

  latestPacket = p;
  haveTelemetry = true;

  String event = reasonText(reason);

  Serial.println();
  Serial.println("!!! EMERGENCY RECEIVED !!!");
  Serial.println(event);

  displayEmergency(reason);
  startBuzzer();
  logCSV(event);

  int abnormalCount = countAbnormalConditions(p);

  // Panic, voice help, ECG/heart event always call.
  // Other event types escalate if 3+ categories are abnormal.
  bool callRequired =
    reason == PANIC_BUTTON_EVENT ||
    reason == VOICE_HELP_EVENT ||
    reason == ECG_ABNORMAL_EVENT ||
    abnormalCount >= 3;

  String message = "PATIENT EMERGENCY: " + event;
  message += "\nECG: " +
    readingText(p.ecgBpm, p.validFlags & ECG_VALID) + " BPM";
  message += "\nHR: " +
    readingText(p.heartRateBpm, p.validFlags & HR_VALID) + " BPM";
  message += "\nSpO2: " +
    readingText(p.spo2Percent, p.validFlags & SPO2_VALID) + "%";
  message += "\nTemp: " +
    readingText(p.temperatureC, p.validFlags & TEMP_VALID) + " C";
  message += "\nAbnormal categories: " + String(abnormalCount);

  sendSMS(message);

  if (callRequired) {
    makeCall();
  }

  Serial.println("Emergency processing complete");
}

// ---------------- INCOMING REPORT SMS ----------------
void sendRecentReport() {
  if (!sdOK) {
    sendSMS("Patient report unavailable: SD card not initialized.");
    return;
  }

  File file = SD.open("/patient.csv", FILE_READ);

  if (!file) {
    sendSMS("Patient report file not found.");
    return;
  }

  String rows[8];
  int count = 0;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      if (count < 8) {
        rows[count++] = line;
      } else {
        for (int i = 0; i < 7; i++) rows[i] = rows[i + 1];
        rows[7] = line;
      }
    }
  }

  file.close();

  String report = "PATIENT RECENT LOG\n";

  for (int i = 0; i < count; i++) {
    if (report.length() + rows[i].length() > 130) {
      sendSMS(report);
      report = "PATIENT LOG (continued)\n";
    }
    report += rows[i] + "\n";
  }

  if (count == 0) report += "No readings recorded.";

  sendSMS(report);
}

void checkIncomingSMS() {
  // SIM800L reports unread SMS indexes with +CMTI.
  static String inputLine;

  while (GSM.available()) {
    char c = (char)GSM.read();

    if (c == '\n') {
      inputLine.trim();

      if (inputLine.startsWith("+CMTI:")) {
        int comma = inputLine.lastIndexOf(',');
        if (comma >= 0) {
          int index = inputLine.substring(comma + 1).toInt();

          String response = sendAT(
            "AT+CMGR=" + String(index), 2500
          );

          Serial.println(response);

          // Only accept REPORT from the configured number.
          // CMGR response contains sender number and message body.
          if (response.indexOf(PHONE_NUMBER) >= 0 &&
              response.indexOf("REPORT") >= 0) {
            sendRecentReport();
          }

          sendAT("AT+CMGD=" + String(index), 1000);
        }
      }

      inputLine = "";
    } else if (inputLine.length() < 300) {
      inputLine += c;
    }
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BUZZER_PIN, OUTPUT);
  setBuzzer(false);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(100000);

  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 15, "WALL NODE STARTING");
  oled.sendBuffer();

  packetQueue = xQueueCreate(20, sizeof(Packet));

  if (packetQueue == nullptr) {
    Serial.println("Packet queue creation failed");
    while (true) delay(1000);
  }

  // Initialize SD card.
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  sdOK = SD.begin(SD_CS_PIN, sdSPI, 1000000);

  if (sdOK) {
    Serial.println("SD card initialized");

    if (!SD.exists("/patient.csv")) {
      File file = SD.open("/patient.csv", FILE_WRITE);

      if (file) {
        file.println(
          "timestamp,uptime_ms,ecg_bpm,heart_rate_bpm,"
          "spo2_percent,temperature_c,event"
        );
        file.close();
      }
    }
  } else {
    Serial.println("SD card initialization failed");
  }

  gsmReady = initializeGSM();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataReceived);

  Serial.print("Wall Node MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println("Wall Node ready");
}

// ---------------- MAIN LOOP ----------------
void loop() {
  updateBuzzer();
  checkIncomingSMS();

  Packet p;

  // Process received packets without waiting for another packet.
  while (xQueueReceive(packetQueue, &p, 0) == pdTRUE) {
    latestPacket = p;
    haveTelemetry = true;

    if (p.type == EMERGENCY) {
      processEmergency(p);
    }
  }

  uint32_t now = millis();

  if ((uint32_t)(now - lastDisplayMs) >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs = now;
    displayReadings();
  }

  if (haveTelemetry &&
      (uint32_t)(now - lastLogMs) >= LOG_INTERVAL_MS) {
    lastLogMs = now;
    logCSV("TELEMETRY");
  }

  delay(1);
}
