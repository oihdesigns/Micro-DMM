/*
 * Generic_DMM_Headless.ino
 *
 * Stripped-down, headless DMM sketch that runs on any Arduino or similar
 * dev board using standard analogRead(). Streams $DMM readings over serial
 * and responds to the same command set used by pc_gui.py (DMM tab).
 *
 * Compatible with: Uno, Nano, Mega, Due, Zero, ESP32, RP2040, GIGA, etc.
 * Just set ADC_BITS and VREF for your board.
 */

// ======================= User Configuration =======================
#define ADC_BITS        14        // ADC resolution: 10 (Uno/Nano), 12 (Due/ESP32), 14 (R4), 16 (GIGA)
#define ADC_PIN         A0        // Analog input pin
#define VREF            3.307       // Reference voltage (3.3 or 5.0)
#define ANALOG_OFFSET   1.65718       // DC offset at 0V input (e.g. 1.65692 for AMC0330)
#define PROBE_SCALE_INIT 16.7658      // Voltage divider ratio (1.0 = no divider)
#define DMM_INTERVAL_MS 250       // Streaming rate (250ms = 4Hz)
#define BAUD_RATE       115200

// ======================= Derived Constants ========================
static const float ADC_MAX_F = (float)((1 << ADC_BITS) - 1);

// ======================= Calibration State ========================
static float analogOffset = ANALOG_OFFSET;
static float probeScale   = PROBE_SCALE_INIT;

// ======================= DMM State ================================

static float dmmAvg    = 0;
static float dmmMin    = 9999;
static float dmmMax    = -9999;
static float dmmVrms   = 0;
static float dmmRefV   = 0;
static bool  dmmRefSet = false;
static bool  dmmShowVac = false;

// ======================= Serial Command Buffer ====================
static char  serialCmdBuf[32];
static uint8_t serialCmdLen = 0;

// ======================= Timing ===================================
static unsigned long lastDmmSend = 0;

// ======================= Voltage Conversion =======================
static inline float adcToVolts(uint16_t raw) {
  float rawV = ((float)raw / ADC_MAX_F) * VREF;
  return (rawV - analogOffset) * probeScale;
}

// ======================= Serial Output ============================

void sendDmmReading() {
  Serial.print("$DMM,");
  Serial.print(dmmAvg, 5);
  Serial.print(",");
  Serial.print(dmmMin, 5);
  Serial.print(",");
  Serial.print(dmmMax, 5);
  Serial.print(",");
  Serial.print(dmmVrms, 5);
  Serial.print(",");
  Serial.print(dmmRefV, 5);
  Serial.print(",");
  Serial.print(dmmRefSet ? 1 : 0);
  Serial.print(",");
  Serial.println(dmmShowVac ? 1 : 0);
}

void sendConfig() {
  Serial.print("$CFG,");
  Serial.print(probeScale, 4);
  Serial.print(",");
  Serial.print(analogOffset, 5);
  Serial.print(",");
  Serial.print(VREF, 2);
  Serial.print(",");
  Serial.print((int)ADC_MAX_F);
  Serial.print(",");
  Serial.println(0);   // plotW = 0 (no scope)
}

void sendCalValues() {
  Serial.println("------ Copy these into your #defines ------");
  Serial.print("#define ANALOG_OFFSET   ");
  Serial.println(analogOffset, 5);
  Serial.print("#define PROBE_SCALE_INIT ");
  Serial.println(probeScale, 5);
  Serial.println("--------------------------------------------");
}

void sendMode() {
  Serial.println("$MODE,DMM");
}

// ======================= Serial Command Handler ===================

void handleSerialCmd(const char* cmd) {
  // Skip leading '!'
  const char* c = cmd;
  if (*c == '!') c++;

  if (strcmp(c, "CFG") == 0) {
    sendConfig();
    sendMode();
  }
  else if (strcmp(c, "RST") == 0) {
    dmmMin = dmmAvg;
    dmmMax = dmmAvg;
  }
  else if (strcmp(c, "REF") == 0) {
    if (dmmRefSet) {
      dmmRefSet = false;
      dmmRefV = 0;
    } else {
      dmmRefSet = true;
      dmmRefV = dmmAvg;
    }
  }
  else if (strcmp(c, "VAC") == 0) {
    dmmShowVac = !dmmShowVac;
  }
  else if (strcmp(c, "ZERO") == 0) {
    // Set zero: current raw ADC voltage becomes the new offset
    analogOffset = dmmAvg / probeScale + analogOffset;
    dmmAvg = 0;
    dmmMin = 0;
    dmmMax = 0;
    sendConfig();
    sendCalValues();
  }
  else if (strncmp(c, "CAL,", 4) == 0) {
    float target = atof(c + 4);
    if (fabsf(dmmAvg) > 0.01f && target > 0) {
      probeScale = probeScale * (target / dmmAvg);
      dmmMin = dmmAvg;
      dmmMax = dmmAvg;
      sendConfig();
      sendCalValues();
    }
  }
  // Scope commands (!TB, !VD, !OFS, !RUN, !AUTO, !TRIG, !MODE) are
  // intentionally not handled — this sketch is DMM-only.
}

void processSerialCommand() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serialCmdLen > 0) {
        serialCmdBuf[serialCmdLen] = '\0';
        handleSerialCmd(serialCmdBuf);
        serialCmdLen = 0;
      }
    } else {
      if (serialCmdLen < sizeof(serialCmdBuf) - 1) {
        serialCmdBuf[serialCmdLen++] = ch;
      }
    }
  }
}

// ======================= Setup ====================================

void setup() {
  Serial.begin(BAUD_RATE);

  // Set ADC resolution on boards that support it
#if defined(ARDUINO_ARCH_SAM)   || defined(ARDUINO_ARCH_SAMD)    || defined(ESP32)       || \
    defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_MBED) || \
    defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_RENESAS_UNO)
  analogReadResolution(ADC_BITS);
#endif

analogReference(AR_EXTERNAL);

  // Initialize DMM state
  dmmAvg    = 0;
  dmmMin    = 9999;
  dmmMax    = -9999;
  dmmVrms   = 0;
  dmmRefV   = 0;
  dmmRefSet = false;
  dmmShowVac = false;

  sendConfig();
  sendMode();
}

// ======================= Main Loop ================================

void loop() {
  processSerialCommand();

  // --- DMM acquisition (every iteration for smooth EMA) ---
  uint16_t raw = analogRead(ADC_PIN);
  float v = adcToVolts(raw);

  // EMA smoothing (alpha = 0.1)
  const float alpha = 0.1f;
  dmmAvg  = dmmAvg * (1.0f - alpha) + v * alpha;

  // Running min / max
  if (v < dmmMin) dmmMin = v;
  if (v > dmmMax) dmmMax = v;

  // AC-coupled RMS (EMA of squared AC component)
  float ac = v - dmmAvg;
  static float acSqEma = 0;
  acSqEma = acSqEma * (1.0f - alpha) + (ac * ac) * alpha;
  dmmVrms = sqrtf(acSqEma);

  // --- Send DMM reading at configured interval ---
  unsigned long now = millis();
  if (now - lastDmmSend >= DMM_INTERVAL_MS) {
    lastDmmSend = now;
    sendDmmReading();
  }
}
