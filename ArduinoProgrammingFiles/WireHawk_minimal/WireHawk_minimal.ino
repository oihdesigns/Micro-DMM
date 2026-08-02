/*
 * WireHawk_minimal.ino
 * Minimal proof-of-concept firmware for Arduino Nano ESP32.
 *
 * Protocol, 115200 baud, newline terminated:
 *   Host -> board: ON, OFF, PING, STATUS
 *   Board -> host:
 *     DATA,<millis>,<relay>,<mA>,<mV>,<d4>,<d3>,<state>
 *     OK,<command>
 *     INFO,<text>
 *
 * The relay opens if PING is not received for HOST_TIMEOUT_MS while on.
 */

#include <Arduino.h>
#include <math.h>

constexpr int CURRENT_PIN = A0;
constexpr int RELAY_PIN   = D2;
constexpr int SENSE_LO    = D3;
constexpr int SENSE_HI    = D4;

constexpr float ZERO_MV = 1590.0f;       // calibrate for your hardware
constexpr float MV_PER_AMP = 500.0f;     // calibrate for your hardware
constexpr float CURRENT_THRESHOLD_MA = 20.0f;

constexpr uint32_t REPORT_INTERVAL_MS = 100;
constexpr uint32_t HOST_TIMEOUT_MS = 1500;
constexpr uint8_t ADC_SAMPLES = 8;

bool relayOn = false;
uint32_t lastReportMs = 0;
uint32_t lastPingMs = 0;

char commandBuffer[32];
size_t commandLength = 0;

void setRelay(bool on) {
  relayOn = on;
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  if (on) lastPingMs = millis();
}

float readCurrentMv() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
    sum += (analogRead(CURRENT_PIN)/1000);
  }
  return static_cast<float>(sum) / ADC_SAMPLES;
}

float currentMaFromMv(float mv) {
  return (mv - ZERO_MV) * 1000.0f / MV_PER_AMP;
}

const char* classify(bool relay, float currentMa, int d4, int d3) {
  const bool currentFlowing = fabsf(currentMa) >= CURRENT_THRESHOLD_MA;
  const int sense = (d4 << 1) | d3;

  if (!relay) {
    if (sense == 2) return "READY";
    if (sense == 1) return "NO_MOTOR";
    if (sense == 3) return "BACKFEED";
    return "IDLE";
  }

  if (currentFlowing) return "RUNNING";
  if (sense == 3) return "NO_CURRENT";
  if (sense == 2) return "NO_VOLTAGE";
  if (sense == 1) return "NO_MOTOR";
  return "UNKNOWN";
}

void sendData() {
  const float mv = readCurrentMv();
  const float ma = currentMaFromMv(mv);
  const int d3 = digitalRead(SENSE_LO) ? 1 : 0;
  const int d4 = digitalRead(SENSE_HI) ? 1 : 0;
  const char* state = classify(relayOn, ma, d4, d3);

  // One short, fixed-format line. At 10 Hz this is far below USB CDC capacity.
  Serial.print("DATA,%lu,%d,%.1f,%.1f,%d,%d,%s\n",
                static_cast<unsigned long>(millis()), relayOn ? 1 : 0,
                ma, mv, d4, d3, state);
}

void handleCommand(char* command) {
  // Trim leading/trailing spaces and accept either legacy !ON or simple ON.
  while (*command == ' ' || *command == '\t') ++command;
  if (*command == '!') ++command;
  for (char* p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p -= ('a' - 'A');
  }

  if (strcmp(command, "ON") == 0) {
    setRelay(true);
    Serial.println("OK,ON");
  } else if (strcmp(command, "OFF") == 0 || strcmp(command, "STOP") == 0) {
    setRelay(false);
    Serial.println("OK,OFF");
  } else if (strcmp(command, "PING") == 0) {
    lastPingMs = millis();
    Serial.println("OK,PING");
  } else if (strcmp(command, "STATUS") == 0) {
    sendData();
  } else if (*command != '\0') {
    Serial.print("INFO,unknown command: ");
    Serial.println(command);
  }
}

void readCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      commandBuffer[commandLength] = '\0';
      handleCommand(commandBuffer);
      commandLength = 0;
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0; // discard overlong command cleanly
    }
  }
}

void setup() {
  digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(SENSE_LO, INPUT_PULLDOWN);
  pinMode(SENSE_HI, INPUT_PULLDOWN);

  analogReadResolution(12);
  Serial.begin(115200);

  setRelay(false);
  delay(100);
  Serial.println("INFO,WireHawk minimal ready");
}

void loop() {
  readCommands();

  if (relayOn && millis() - lastPingMs > HOST_TIMEOUT_MS) {
    setRelay(false);
    Serial.println("INFO,relay opened: heartbeat timeout");
  }

  if (millis() - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = millis();
    sendData();
  }
}
