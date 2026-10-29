
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <DFRobot_DF2301Q.h>

// ---------------- PINS ----------------
#define SDA_PIN       21
#define SCL_PIN       22
#define BUTTON_PIN    23
#define ECG_LO_PLUS   32
#define ECG_LO_MINUS  33
#define ECG_OUT       34
#define VOICE_RX      16
#define VOICE_TX      17

// Replace with the actual Wall Node MAC address.
uint8_t WALL_MAC[] = {0x30, 0x76, 0xF5, 0xE5, 0xA6, 0x40};

const uint8_t ESPNOW_CHANNEL = 1;
const uint32_t PACKET_MAGIC = 0x50414E49;
const uint8_t PACKET_VERSION = 1;

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

// Keep this structure identical on the Wall Node.
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

// ---------------- SENSORS ----------------
Adafruit_MPU6050 mpu;
Adafruit_TMP117 tmp117;
MAX30105 maxSensor;
DFRobot_DF2301Q_UART voice(&Serial1);

bool mpuOK = false;
bool tempOK = false;
bool maxOK = false;
bool voiceOK = false;

float ecgBpm = NAN;
float heartRateBpm = NAN;
float spo2Percent = NAN;
float temperatureC = NAN;

uint8_t validFlags = 0;
uint16_t sequenceNumber = 0;

// ---------------- TIMING ----------------
const uint32_t TELEMETRY_INTERVAL_MS = 1000;
const uint32_t SENSOR_READ_INTERVAL_MS = 100;
const uint32_t ECG_ABNORMAL_HOLD_MS = 5000;
const uint32_t SPO2_ABNORMAL_HOLD_MS = 5000;
const uint32_t TEMP_ABNORMAL_HOLD_MS = 90000; // 1.5 minutes
const uint32_t FALL_COOLDOWN_MS = 10000;
const uint32_t EVENT_COOLDOWN_MS = 30000;

// ---------------- THRESHOLDS ----------------
const float ECG_LOW_BPM = 50.0;
const float ECG_HIGH_BPM = 120.0;

const float HR_LOW_BPM = 50.0;
const float HR_HIGH_BPM = 120.0;

const float SPO2_LOW_PERCENT = 92.0;

// Prototype thresholds; adjust with your supervisor.
const float TEMP_LOW_C = 35.0;
const float TEMP_HIGH_C = 38.0;

const float FALL_ACCEL_THRESHOLD = 25.0; // m/s^2

uint32_t lastTelemetryMs = 0;
uint32_t lastSensorReadMs = 0;
uint32_t lastFallMs = 0;
uint32_t lastButtonMs = 0;
uint32_t lastVoiceMs = 0;

uint32_t ecgAbnormalSince = 0;
uint32_t spo2AbnormalSince = 0;
uint32_t tempAbnormalSince = 0;

uint32_t lastECGEventMs = 0;
uint32_t lastSpO2EventMs = 0;

bool tempAlertSentForCurrentEpisode = false;
bool ecgAlertSentForCurrentEpisode = false;
bool spo2AlertSentForCurrentEpisode = false;

bool lastButtonState = HIGH;
uint32_t lastBeatMs = 0;
float beatIntervals[4] = {0, 0, 0, 0};
uint8_t beatIndex = 0;
long lastIR = 0;
long lastRed = 0;

float previousAccelMagnitude = 9.8;
uint32_t lastAccelSampleMs = 0;

// ---------------- ESP-NOW ----------------
void sendPacket(uint8_t type, uint8_t reason) {
  Packet p = {};
  p.magic = PACKET_MAGIC;
  p.version = PACKET_VERSION;
  p.type = type;
  p.sequence = ++sequenceNumber;
  p.uptimeMs = millis();
  p.ecgBpm = ecgBpm;
  p.heartRateBpm = heartRateBpm;
  p.spo2Percent = spo2Percent;
  p.temperatureC = temperatureC;
  p.validFlags = validFlags;
  p.risk = 0;
  p.reason = reason;
  p.ackSequence = 0;

  // Emergency packets are repeated quickly for reliability.
  int copies = (type == EMERGENCY) ? 3 : 1;

  for (int i = 0; i < copies; i++) {
    esp_now_send(WALL_MAC, (uint8_t *)&p, sizeof(p));
    if (i < copies - 1) delay(20);
  }
}

void triggerEmergency(uint8_t reason) {
  sendPacket(EMERGENCY, reason);
}

// ---------------- SENSOR READING ----------------
void readECG() {
  bool leadsOff =
    digitalRead(ECG_LO_PLUS) == HIGH ||
    digitalRead(ECG_LO_MINUS) == HIGH;

  if (leadsOff) {
    ecgBpm = NAN;
    validFlags &= ~ECG_VALID;
    return;
  }

  static int previousValue = 0;
  static uint32_t lastCrossingMs = 0;

  int value = analogRead(ECG_OUT);
  uint32_t now = millis();

  // Simple threshold-crossing estimate; not a clinical ECG algorithm.
  if (previousValue < 2000 && value >= 2000) {
    if (lastCrossingMs != 0) {
      uint32_t interval = now - lastCrossingMs;
      if (interval >= 300 && interval <= 2000) {
        float bpm = 60000.0 / interval;
        if (bpm >= 30 && bpm <= 200) {
          ecgBpm = bpm;
          validFlags |= ECG_VALID;
        }
      }
    }
    lastCrossingMs = now;
  }

  previousValue = value;
}

void readMAX30102() {
  if (!maxOK) return;

  long ir = maxSensor.getIR();
  long red = maxSensor.getRed();
  lastIR = ir;
  lastRed = red;

  // No finger: invalidate rather than reporting stale readings.
  if (ir < 50000 || red <= 0) {
    heartRateBpm = NAN;
    spo2Percent = NAN;
    validFlags &= ~(HR_VALID | SPO2_VALID);
    return;
  }

  if (checkForBeat(ir)) {
    uint32_t now = millis();

    if (lastBeatMs != 0) {
      float bpm = 60000.0 / (now - lastBeatMs);

      if (bpm >= 30 && bpm <= 220) {
        beatIntervals[beatIndex] = bpm;
        beatIndex = (beatIndex + 1) % 4;

        float sum = 0;
        int count = 0;

        for (int i = 0; i < 4; i++) {
          if (beatIntervals[i] > 0) {
            sum += beatIntervals[i];
            count++;
          }
        }

        if (count > 0) {
          heartRateBpm = sum / count;
          validFlags |= HR_VALID;
        }
      }
    }

    lastBeatMs = now;
  }

  // Rough experimental estimate, not a medical SpO2 algorithm.
  float ratio = (float)red / (float)ir;
  float estimate = 110.0 - 25.0 * ratio;

  if (estimate < 70.0) estimate = 70.0;
  if (estimate > 100.0) estimate = 100.0;

  spo2Percent = estimate;
  validFlags |= SPO2_VALID;
}

void readTemperature() {
  if (!tempOK) return;

  sensors_event_t tempEvent;

  if (tmp117.getEvent(&tempEvent)) {
    temperatureC = tempEvent.temperature;

    if (isfinite(temperatureC) &&
        temperatureC > 0 && temperatureC < 60) {
      validFlags |= TEMP_VALID;
    } else {
      validFlags &= ~TEMP_VALID;
    }
  } else {
    validFlags &= ~TEMP_VALID;
  }
}

void checkTemperaturePersistence() {
  uint32_t now = millis();

  if (!(validFlags & TEMP_VALID)) {
    // Invalid readings break continuity.
    tempAbnormalSince = 0;
    tempAlertSentForCurrentEpisode = false;
    return;
  }

  bool abnormal =
    temperatureC < TEMP_LOW_C ||
    temperatureC > TEMP_HIGH_C;

  if (!abnormal) {
    tempAbnormalSince = 0;
    tempAlertSentForCurrentEpisode = false;
    return;
  }

  if (tempAbnormalSince == 0) {
    tempAbnormalSince = now;
  }

  if (!tempAlertSentForCurrentEpisode &&
      (uint32_t)(now - tempAbnormalSince) >= TEMP_ABNORMAL_HOLD_MS) {
    tempAlertSentForCurrentEpisode = true;
    triggerEmergency(TEMP_HIGH_EVENT);
  }
}

void checkHeartAbnormality() {
  uint32_t now = millis();

  bool ecgBad = (validFlags & ECG_VALID) &&
    (ecgBpm < ECG_LOW_BPM || ecgBpm > ECG_HIGH_BPM);

  bool hrBad = (validFlags & HR_VALID) &&
    (heartRateBpm < HR_LOW_BPM || heartRateBpm > HR_HIGH_BPM);

  bool abnormal = ecgBad || hrBad;

  if (!abnormal) {
    ecgAbnormalSince = 0;
    ecgAlertSentForCurrentEpisode = false;
    return;
  }

  if (ecgAbnormalSince == 0) ecgAbnormalSince = now;

  if (!ecgAlertSentForCurrentEpisode &&
      (uint32_t)(now - ecgAbnormalSince) >= ECG_ABNORMAL_HOLD_MS &&
      (uint32_t)(now - lastECGEventMs) >= EVENT_COOLDOWN_MS) {
    ecgAlertSentForCurrentEpisode = true;
    lastECGEventMs = now;
    triggerEmergency(ECG_ABNORMAL_EVENT);
  }
}

void checkSpO2Abnormality() {
  uint32_t now = millis();

  bool abnormal = (validFlags & SPO2_VALID) &&
                  (spo2Percent < SPO2_LOW_PERCENT);

  if (!abnormal) {
    spo2AbnormalSince = 0;
    spo2AlertSentForCurrentEpisode = false;
    return;
  }

  if (spo2AbnormalSince == 0) spo2AbnormalSince = now;

  if (!spo2AlertSentForCurrentEpisode &&
      (uint32_t)(now - spo2AbnormalSince) >= SPO2_ABNORMAL_HOLD_MS &&
      (uint32_t)(now - lastSpO2EventMs) >= EVENT_COOLDOWN_MS) {
    spo2AlertSentForCurrentEpisode = true;
    lastSpO2EventMs = now;
    triggerEmergency(SPO2_LOW_EVENT);
  }
}

void checkFall() {
  if (!mpuOK) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float magnitude = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );

  validFlags |= MPU_VALID;

  uint32_t now = millis();

  // Sudden acceleration heuristic, not validated fall detection.
  if (magnitude > FALL_ACCEL_THRESHOLD &&
      (uint32_t)(now - lastFallMs) >= FALL_COOLDOWN_MS) {
    lastFallMs = now;
    triggerEmergency(MPU_MOTION_EVENT);
  }

  previousAccelMagnitude = magnitude;
}

void checkButton() {
  bool current = digitalRead(BUTTON_PIN);
  uint32_t now = millis();

  if (lastButtonState == HIGH && current == LOW &&
      (uint32_t)(now - lastButtonMs) > 500) {
    lastButtonMs = now;
    triggerEmergency(PANIC_BUTTON_EVENT);
  }

  lastButtonState = current;
}

void checkVoice() {
  if (!voiceOK) return;

  uint8_t commandID = voice.getCMDID();
  uint32_t now = millis();

  // Command ID 1 is configured as the help command.
  if (commandID == 1 &&
      (uint32_t)(now - lastVoiceMs) > 2000) {
    lastVoiceMs = now;
    triggerEmergency(VOICE_HELP_EVENT);
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(ECG_LO_PLUS, INPUT);
  pinMode(ECG_LO_MINUS, INPUT);
  analogReadResolution(12);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("\nSensor Node starting...");

  mpuOK = mpu.begin();
  if (mpuOK) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 OK");
  } else {
    Serial.println("MPU6050 NOT FOUND");
  }

  tempOK = tmp117.begin();
  Serial.println(tempOK ? "TMP117 OK" : "TMP117 NOT FOUND");

  maxOK = maxSensor.begin(Wire, I2C_SPEED_FAST);
  if (maxOK) {
    maxSensor.setup();
    maxSensor.setPulseAmplitudeRed(0x1F);
    maxSensor.setPulseAmplitudeIR(0x1F);
    maxSensor.setPulseAmplitudeGreen(0);
    Serial.println("MAX30102 OK");
  } else {
    Serial.println("MAX30102 NOT FOUND");
  }

  Serial1.begin(9600, SERIAL_8N1, VOICE_RX, VOICE_TX);
  voiceOK = voice.begin();
  Serial.println(voiceOK ? "Voice module OK" : "Voice module not detected");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, WALL_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;

  if (!esp_now_is_peer_exist(WALL_MAC)) {
    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("Failed to add Wall Node peer");
    }
  }

  Serial.print("Sensor Node MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Sensor Node ready");
}

// ---------------- MAIN LOOP ----------------
void loop() {
  uint32_t now = millis();

  checkButton();
  checkVoice();

  if ((uint32_t)(now - lastSensorReadMs) >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadMs = now;

    readECG();
    readMAX30102();
    readTemperature();
    checkTemperaturePersistence();
    checkHeartAbnormality();
    checkSpO2Abnormality();
    checkFall();
  }

  if ((uint32_t)(now - lastTelemetryMs) >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    sendPacket(TELEMETRY, NONE);

    Serial.print("ECG=");
    Serial.print(ecgBpm);
    Serial.print(" HR=");
    Serial.print(heartRateBpm);
    Serial.print(" SpO2=");
    Serial.print(spo2Percent);
    Serial.print(" Temp=");
    Serial.println(temperatureC);
  }

  delay(1);
}
