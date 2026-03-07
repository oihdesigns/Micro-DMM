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
#define ANALOG_OFFSET   1.65159       // DC offset at 0V input (e.g. 1.65692 for AMC0330)
#define PROBE_SCALE_INIT 35.5681      // Voltage divider ratio (1.0 = no divider)
// ---- Current channel (AMC1200B differential) ----
#define CUR_PIN_POS     A1        // AMC1200B positive output
#define CUR_PIN_NEG     A2        // AMC1200B negative output
#define AMC1200B_GAIN   8.0       // AMC1200B voltage gain (V/V)
#define SHUNT_R         0.1       // Shunt resistor value (ohms)

#define DMM_INTERVAL_MS 250       // Streaming rate (250ms = 4Hz)
#define BAUD_RATE       115200

// ======================= Derived Constants ========================
static const float ADC_MAX_F = (float)((1 << ADC_BITS) - 1);

// ======================= Calibration State ========================
static float analogOffset = ANALOG_OFFSET;
static float probeScale   = PROBE_SCALE_INIT;

// ======================= Voltage DMM State ========================

static float dmmAvg    = 0;
static float dmmMin    = 9999;
static float dmmMax    = -9999;
static float dmmVrms   = 0;
static float dmmRefV   = 0;
static bool  dmmRefSet = false;
static bool  dmmShowVac = false;

// Windowed RMS accumulators (reset each reporting interval)
static double rmsVSum   = 0.0;
static double rmsVSqSum = 0.0;
static uint32_t rmsSampleCount = 0;

// ======================= Smoothing State ===========================
static bool  dmmSmooth = false;

// ======================= Current Channel State ====================

static float curAvg    = 0;
static float curMin    = 9999;
static float curMax    = -9999;

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

// LSB at the probe tip — smallest voltage change the ADC can resolve
static inline float probeLsb() {
  return (VREF / ADC_MAX_F) * probeScale;
}

// Round a value to the nearest LSB
static inline float roundToLsb(float v, float lsb) {
  return roundf(v / lsb) * lsb;
}

// Number of decimal places needed to show the LSB digit
static uint8_t decimalsForLsb(float lsb) {
  uint8_t d = 0;
  while (lsb < 1.0f && d < 6) { lsb *= 10.0f; d++; }
  return d;
}

// ======================= Current Conversion =======================

// Read differential AMC1200B outputs and convert to amps
static float readCurrent() {
  float vPos = ((float)analogRead(CUR_PIN_POS) / ADC_MAX_F) * VREF;
  float vNeg = ((float)analogRead(CUR_PIN_NEG) / ADC_MAX_F) * VREF;
  float diffV = vPos - vNeg;
  return diffV / AMC1200B_GAIN / SHUNT_R;
}

// Current LSB — smallest current change the ADC can resolve
static inline float currentLsb() {
  return (VREF / ADC_MAX_F) / AMC1200B_GAIN / SHUNT_R;
}

// ======================= Serial Output ============================

void sendDmmReading() {
  float vLsb = probeLsb();
  uint8_t vDp = decimalsForLsb(vLsb);
  float cLsb = currentLsb();
  uint8_t cDp = decimalsForLsb(cLsb);

  Serial.print("$DMM,");
  Serial.print(roundToLsb(dmmAvg, vLsb), vDp);
  Serial.print(",");
  Serial.print(roundToLsb(dmmMin, vLsb), vDp);
  Serial.print(",");
  Serial.print(roundToLsb(dmmMax, vLsb), vDp);
  Serial.print(",");
  Serial.print(roundToLsb(dmmVrms, vLsb), vDp);
  Serial.print(",");
  Serial.print(roundToLsb(dmmRefV, vLsb), vDp);
  Serial.print(",");
  Serial.print(dmmRefSet ? 1 : 0);
  Serial.print(",");
  Serial.print(dmmShowVac ? 1 : 0);
  Serial.print(",");
  Serial.print(roundToLsb(curAvg, cLsb), cDp);
  Serial.print(",");
  Serial.print(roundToLsb(curMin, cLsb), cDp);
  Serial.print(",");
  Serial.println(roundToLsb(curMax, cLsb), cDp);
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
  Serial.print(0);   // plotW = 0 (no scope)
  Serial.print(",");
  Serial.print(SHUNT_R, 4);
  Serial.print(",");
  Serial.println(AMC1200B_GAIN, 1);
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
    curMin = curAvg;
    curMax = curAvg;
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
  else if (strcmp(c, "SMOOTH") == 0) {
    dmmSmooth = !dmmSmooth;
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



//Space for attention...




 










//The below line must be uncommented for proper function on the R4 Nano when using a 3.3V rail as the analog reference. 
//analogReference(AR_EXTERNAL);

  // Initialize DMM state
  dmmAvg    = 0;
  dmmMin    = 9999;
  dmmMax    = -9999;
  dmmVrms   = 0;
  dmmRefV   = 0;
  dmmRefSet = false;
  dmmShowVac = false;
  rmsVSum   = 0.0;
  rmsVSqSum = 0.0;
  rmsSampleCount = 0;

  curAvg    = 0;
  curMin    = 9999;
  curMax    = -9999;

  sendConfig();
  sendMode();

    // Configure LED_BUILTIN pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize LEDR, LEDG and LEDB as outputs
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  
  // Turn off all LEDs initially
  analogWrite(LEDR, 64);
  analogWrite(LEDG, 255);
  analogWrite(LEDB, 64);


}

// ======================= Main Loop ================================

void loop() {
  processSerialCommand();

  // --- DMM acquisition (every iteration for smooth EMA) ---
  uint16_t raw = analogRead(ADC_PIN);
  float v = adcToVolts(raw);

  // Time-based EMA for DC average display (tau ≈ 50 ms)
  // Using micros() keeps the time-constant correct regardless of loop speed.
  static unsigned long lastUs = 0;
  unsigned long nowUs = micros();
  float dtSec = (float)(nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dtSec < 0.0f || dtSec > 0.5f) dtSec = 0.001f; // clamp on first call / wrap

  const float tau_dc = dmmSmooth ? 0.5f : 0.05f; // 500 ms smooth / 50 ms normal
  float alpha = dtSec / (tau_dc + dtSec);
  if (alpha < 1e-6f) alpha = 1e-6f;
  if (alpha > 1.0f)  alpha = 1.0f;
  dmmAvg = dmmAvg * (1.0f - alpha) + v * alpha;

  // Running min / max
  if (v < dmmMin) dmmMin = v;
  if (v > dmmMax) dmmMax = v;

  // Windowed RMS: accumulate sum(v) and sum(v²) over the reporting interval.
  // At send time: Vrms = sqrt(mean(v²) - mean(v)²)  →  true AC-RMS (DC-rejected).
  rmsVSum   += (double)v;
  rmsVSqSum += (double)v * (double)v;
  rmsSampleCount++;

  // --- Current channel acquisition ---
  float cur = readCurrent();
  static float curAcSqEma = 0;

  if (dmmSmooth) {
    float cLsb = currentLsb();
    float curRms = sqrtf(curAcSqEma);
    float ca = alpha * cLsb / (cLsb + curRms);
    if (ca < 0.001f) ca = 0.001f;
    curAvg = curAvg * (1.0f - ca) + cur * ca;
  } else {
    curAvg = curAvg * (1.0f - alpha) + cur * alpha;
  }

  // Current RMS (always tracked so adaptive alpha is ready)
  float curAc = cur - curAvg;
  curAcSqEma = curAcSqEma * (1.0f - alpha) + (curAc * curAc) * alpha;

  if (cur < curMin) curMin = cur;
  if (cur > curMax) curMax = cur;

  // --- Send DMM reading at configured interval ---
  unsigned long now = millis();

  if (now - lastDmmSend >= DMM_INTERVAL_MS) {
    lastDmmSend = now;

    // Compute true AC-RMS from the window accumulated since last send.
    // sqrt(E[v²] - E[v]²) removes any DC bias automatically.
    if (rmsSampleCount > 0) {
      double meanV   = rmsVSum   / rmsSampleCount;
      double meanVSq = rmsVSqSum / rmsSampleCount;
      double variance = meanVSq - meanV * meanV;
      dmmVrms = (variance > 0.0) ? sqrtf((float)variance) : 0.0f;
    }
    rmsVSum        = 0.0;
    rmsVSqSum      = 0.0;
    rmsSampleCount = 0;

    analogWrite(LEDR, 255);
    analogWrite(LEDG, 0);
    analogWrite(LEDB, 255);

    sendDmmReading();
    
    analogWrite(LEDR, 64);
    analogWrite(LEDG, 255);
    analogWrite(LEDB, 64);
  }
}
