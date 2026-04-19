/*
  AMC_OLED.ino
  Isolated precision current + voltage meter — OLED variant.

  Hardware
  ─────────────────────────────────────────────────────────────────────────────
  Feather RP2350
  Adafruit 128×64 OLED FeatherWing  (SSD1306, I2C address 0x3C)
  ADS1015 12-bit ADC                (I2C address 0x48, ADDR pin = GND)

  Channel assignments (ADS1015)
  ─────────────────────────────────────────────────────────────────────────────
  AIN0 − AIN1  →  AMC3301 output  (current via 0.01 Ω shunt)
  AIN2 − AIN3  →  AMC3330 output  (voltage via 50.5:1 resistor divider)

  Scaling
  ─────────────────────────────────────────────────────────────────────────────
  AMC3301 internal gain ≈ 8.2 V/V:
    I (A) = V_ADS (V) ÷ (AMC3301_GAIN × SHUNT_R)

  AMC3330 unity-gain isolation amp + 50.5:1 divider:
    V (V) = V_ADS (V) × DIVIDER_RATIO

  FeatherWing buttons (active LOW, internal pull-up)
  ─────────────────────────────────────────────────────────────────────────────
  Button A  (pin 9)  →  Reset min/max
  Button B  (pin 6)  →  Zero current (tare)
  Button C  (pin 5)  →  Un-zero current

  Display layout  (SH1107 64×128 rotated → 128×64 landscape)
  ─────────────────────────────────────────────────────────────────────────────
  y= 0  size-1  "CURRENT              [gain]"
  y= 8  size-2  "+X.XXXX A"    ← large live reading (16 px tall)
  y=24  size-1  "mn:±X.XXX  mx:±X.XXX"
  y=32  size-1  "VOLTAGE              [gain]"
  y=40  size-2  "+XXX.XX V"   ← large live reading (16 px tall)
  y=56  size-1  "mn:±XXX.X  mx:±XXX.X"

  Serial protocol  (same as AMC_testBoard — Python GUI / Android app compatible)
  ─────────────────────────────────────────────────────────────────────────────
  Outgoing (10 Hz):
    $AMC,<I>,<V>,<Imin>,<Imax>,<Vmin>,<Vmax>,<gainI>,<gainV>
  Commands in:
    !RST              reset min/max to current live values
    !ZERO             tare current to zero
    !UNZERO           clear tare offset
    !STATUS           send one $AMC line immediately
    !CAPTURE,N[,sm]   trigger N-sample capture (sm = smooth override)
    !ABORT            cancel in-progress capture

  Libraries required
  ─────────────────────────────────────────────────────────────────────────────
  Adafruit SH110X
  Adafruit GFX Library
  Adafruit ADS1X15
  Adafruit BusIO
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_ADS1X15.h>

// ── OLED ─────────────────────────────────────────────────────────────────────
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
#define OLED_RESET   -1   // shared with Arduino reset

// ── FeatherWing buttons (active LOW, INPUT_PULLUP) ────────────────────────────
#define BTN_A  9    // Reset min/max
#define BTN_B  6    // Zero current (tare)
#define BTN_C  5    // Un-zero current

#define BTN_DEBOUNCE_MS 150

// ── ADS1015 ───────────────────────────────────────────────────────────────────
#define ADS_ADDR    0x48

// ── Physical scaling ──────────────────────────────────────────────────────────
#define AMC3301_GAIN  8.2f
#define SHUNT_R       0.01f
#define DIVIDER_RATIO 50.5f

// ── Digital smoothing ─────────────────────────────────────────────────────────
#define SMOOTH_N 7

// ── ADS1015 gain table ────────────────────────────────────────────────────────
static const adsGain_t kGain[] = {
  GAIN_TWOTHIRDS,  // ±6.144 V  →  3.000 mV/count
  GAIN_ONE,        // ±4.096 V  →  2.000 mV/count
  GAIN_TWO,        // ±2.048 V  →  1.000 mV/count
  GAIN_FOUR,       // ±1.024 V  →  0.500 mV/count
  GAIN_EIGHT,      // ±0.512 V  →  0.250 mV/count
  GAIN_SIXTEEN     // ±0.256 V  →  0.125 mV/count
};
static const float      kMvPerCount[] = { 3.000f, 2.000f, 1.000f, 0.500f, 0.250f, 0.125f };
static const char*      kGainLabel[]  = { "6.1V","4.1V","2.0V","1.0V","512m","256m" };
static const uint8_t    kNumGain      = 6;

static const int16_t GAIN_HI = 1900;
static const int16_t GAIN_LO =  200;

// ── Peripheral objects ────────────────────────────────────────────────────────
Adafruit_ADS1015 ads;

// ── Measurement state ─────────────────────────────────────────────────────────
float current_A  = 0.0f;
float voltage_V  = 0.0f;
float currentMin = 0.0f, currentMax = 0.0f;
float voltageMin = 0.0f, voltageMax = 0.0f;
bool  firstReading = true;

size_t gainIdxI = 5;   // start at GAIN_SIXTEEN (most sensitive)
size_t gainIdxV = 0;   // start at GAIN_TWOTHIRDS (widest range)

// ── Serial / timing ───────────────────────────────────────────────────────────
String        serialBuf    = "";
unsigned long lastSerialMs  = 0;
unsigned long lastDisplayMs = 0;
float         currentOffset = 0.0f;

static const uint16_t DISP_RATE_MS = 500;  // 10 Hz

// ── Button state ──────────────────────────────────────────────────────────────
unsigned long lastBtnMs = 0;

// ── Capture mode ─────────────────────────────────────────────────────────────
struct CapSample { float I; float V; uint32_t t_us; };
static const uint16_t CAP_MAX = 2000;
static CapSample capBuf[CAP_MAX];
uint16_t capN      = 0;
uint16_t capIdx    = 0;
uint8_t  capSmooth = 0;
bool     capturing = false;
bool     capReady  = false;

// ═══════════════════════════════════════════════════════════════════════════
//  MEASUREMENT
// ═══════════════════════════════════════════════════════════════════════════

float readSmoothedDiff01(size_t& gainIdx, uint8_t smoothN = SMOOTH_N) {
  size_t g = gainIdx;
  ads.setGain(kGain[g]);
  (void)ads.readADC_Differential_0_1();  // discard: settle after gain change

  int32_t sum = 0;
  for (uint8_t i = 0; i < smoothN; i++)
    sum += ads.readADC_Differential_0_1();
  float avg_counts = (float)sum / (float)smoothN;
  float volts      = (avg_counts * kMvPerCount[g]) / 1000.0f;

  int16_t absAvg = (int16_t)fabsf(avg_counts);
  if      (absAvg > GAIN_HI && gainIdx > 0)                    gainIdx--;
  else if (absAvg < GAIN_LO && gainIdx < (size_t)(kNumGain-1)) gainIdx++;

  return volts;
}

float readSmoothedDiff23(size_t& gainIdx, uint8_t smoothN = SMOOTH_N) {
  size_t g = gainIdx;
  ads.setGain(kGain[g]);
  (void)ads.readADC_Differential_2_3();

  int32_t sum = 0;
  for (uint8_t i = 0; i < smoothN; i++)
    sum += ads.readADC_Differential_2_3();
  float avg_counts = (float)sum / (float)smoothN;
  float volts      = (avg_counts * kMvPerCount[g]) / 1000.0f;

  int16_t absAvg = (int16_t)fabsf(avg_counts);
  if      (absAvg > GAIN_HI && gainIdx > 0)                    gainIdx--;
  else if (absAvg < GAIN_LO && gainIdx < (size_t)(kNumGain-1)) gainIdx++;

  return volts;
}

void takeMeasurements() {
  float v01 = readSmoothedDiff01(gainIdxI);
  float v23 = readSmoothedDiff23(gainIdxV);

  current_A = (v01 / (AMC3301_GAIN * SHUNT_R)) - currentOffset;
  voltage_V = v23 * DIVIDER_RATIO;

  if (firstReading) {
    currentMin = currentMax = current_A;
    voltageMin = voltageMax = voltage_V;
    firstReading = false;
  } else {
    if (current_A < currentMin) currentMin = current_A;
    if (current_A > currentMax) currentMax = current_A;
    if (voltage_V < voltageMin) voltageMin = voltage_V;
    if (voltage_V > voltageMax) voltageMax = voltage_V;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  CAPTURE
// ═══════════════════════════════════════════════════════════════════════════

void runCapture() {
  uint8_t sm = capSmooth ? capSmooth : SMOOTH_N;
  float v01  = readSmoothedDiff01(gainIdxI, sm);
  float v23  = readSmoothedDiff23(gainIdxV, sm);
  capBuf[capIdx++] = {
    (v01 / (AMC3301_GAIN * SHUNT_R)) - currentOffset,
    v23 * DIVIDER_RATIO,
    micros()
  };
  if (capIdx >= capN) { capturing = false; capReady = true; }
}

void dumpCapture() {
  uint32_t t0 = capBuf[0].t_us;
  for (uint16_t i = 0; i < capIdx; i++) {
    Serial.print(F("$S,"));
    Serial.print(i);                    Serial.print(',');
    Serial.print(capBuf[i].t_us - t0); Serial.print(',');
    Serial.print(capBuf[i].I, 5);      Serial.print(',');
    Serial.println(capBuf[i].V, 4);
  }
  uint8_t sm = capSmooth ? capSmooth : SMOOTH_N;
  Serial.print(F("$CAPDONE,"));
  Serial.print(capIdx);                       Serial.print(',');
  Serial.print(capBuf[capIdx-1].t_us - t0);  Serial.print(',');
  Serial.print(sm);                           Serial.print(',');
  Serial.print(kGainLabel[gainIdxI]);         Serial.print(',');
  Serial.println(kGainLabel[gainIdxV]);
  capReady = false;
}

// ═══════════════════════════════════════════════════════════════════════════
//  SERIAL PROTOCOL
// ═══════════════════════════════════════════════════════════════════════════

void sendData() {
  Serial.print(F("$AMC,"));
  Serial.print(current_A,  5); Serial.print(',');
  Serial.print(voltage_V,  4); Serial.print(',');
  Serial.print(currentMin, 5); Serial.print(',');
  Serial.print(currentMax, 5); Serial.print(',');
  Serial.print(voltageMin, 4); Serial.print(',');
  Serial.print(voltageMax, 4); Serial.print(',');
  Serial.print(kGainLabel[gainIdxI]); Serial.print(',');
  Serial.println(kGainLabel[gainIdxV]);
}

void doReset() {
  currentMin = currentMax = current_A;
  voltageMin = voltageMax = voltage_V;
  Serial.println(F("$RST,OK"));
}

void processSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      String cmd = serialBuf; cmd.trim(); serialBuf = "";
      if (cmd.length() == 0) continue;
      if (cmd == F("!RST")) {
        doReset();
      } else if (cmd == F("!ZERO")) {
        currentOffset += current_A;
        currentMin = currentMax = 0.0f;
        Serial.println(F("$ZERO,OK"));
      } else if (cmd == F("!UNZERO")) {
        currentOffset = 0.0f;
        currentMin = currentMax = current_A;
        Serial.println(F("$UNZERO,OK"));
      } else if (cmd == F("!STATUS")) {
        sendData();
      } else if (cmd.startsWith(F("!CAPTURE,"))) {
        String args  = cmd.substring(9);
        int    comma = args.indexOf(',');
        capN      = (uint16_t)constrain(args.toInt(), 10, CAP_MAX);
        capSmooth = (comma >= 0) ? (uint8_t)args.substring(comma + 1).toInt() : 0;
        capSmooth = constrain(capSmooth, 0, 16);
        capIdx    = 0;
        capturing = true;
        capReady  = false;
        Serial.print(F("$CAPTURE_START,")); Serial.println(capN);
      } else if (cmd == F("!ABORT")) {
        capturing = false; capReady = false; capIdx = 0;
        Serial.println(F("$ABORT,OK"));
      }
    } else {
      if (serialBuf.length() < 20) serialBuf += c;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  BUTTONS
// ═══════════════════════════════════════════════════════════════════════════

void processButtons() {
  unsigned long now = millis();
  if (now - lastBtnMs < BTN_DEBOUNCE_MS) return;

  if (!digitalRead(BTN_A)) {
    lastBtnMs = now;
    doReset();
  } else if (!digitalRead(BTN_B)) {
    lastBtnMs = now;
    currentOffset += current_A;
    currentMin = currentMax = 0.0f;
    Serial.println(F("$ZERO,OK"));
  } else if (!digitalRead(BTN_C)) {
    lastBtnMs = now;
    currentOffset = 0.0f;
    currentMin = currentMax = current_A;
    Serial.println(F("$UNZERO,OK"));
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  DISPLAY
// ═══════════════════════════════════════════════════════════════════════════
//
//  128×64 layout — size-2 (12×16 px) for live values, size-1 (6×8 px) for
//  headers and min/max.  Fills all 64 rows exactly:
//
//  y= 0  size-1  "CURRENT              [gain]"
//  y= 8  size-2  "+X.XXXX A"           (16 px tall)
//  y=24  size-1  "mn:±X.XXX  mx:±X.XXX"
//  y=32  size-1  "VOLTAGE              [gain]"
//  y=40  size-2  "+XXX.XX V"           (16 px tall)
//  y=56  size-1  "mn:±XXX.X  mx:±XXX.X"

// Large format for live current — always Amps, guaranteed ≤10 chars (120 px at size-2)
// Field width 8 gives "+X.XXXXX" (8 chars) + " A" = 10 chars for |a| < 10.
// Drop one decimal for |a| ≥ 10 to keep "+XX.XXXX A" = 10 chars.
static void fmtCurrentLarge(float a, char* buf, uint8_t bufLen) {
  if (fabsf(a) < 10.0f)
    snprintf(buf, bufLen, "%+8.5f A", a);  // "+X.XXXXX A" = 10 chars
  else
    snprintf(buf, bufLen, "%+8.4f A", a);  // "+XX.XXXX A" = 10 chars
}

// Large format for live voltage — always Volts, guaranteed ≤10 chars
// Field width 8: "+XX.XXXX V" = 10 chars for |v| < 100.
// Drop one decimal for |v| ≥ 100 → "+XXX.XXX V" = 10 chars.
static void fmtVoltageLarge(float v, char* buf, uint8_t bufLen) {
  if (fabsf(v) < 100.0f)
    snprintf(buf, bufLen, "%+8.4f V", v);  // "+XX.XXXX V" = 10 chars
  else
    snprintf(buf, bufLen, "%+8.3f V", v);  // "+XXX.XXX V" = 10 chars
}

// Compact min/max line — always A or V, fits 21 chars at size-1 (126 px)
// Output: "mn:±X.XXXX mx:±X.XXXX"
static void fmtMinMax(float mn, float mx, bool isCurrent, char* buf, uint8_t bufLen) {
  char smn[9], smx[9];
  if (isCurrent) {
    snprintf(smn, sizeof(smn), "%+7.4f", mn);   // "+X.XXXX" 7 chars
    snprintf(smx, sizeof(smx), "%+7.4f", mx);
  } else {
    snprintf(smn, sizeof(smn), "%+7.3f", mn);   // "+XX.XXX" or "+XXX.XX" 7 chars
    snprintf(smx, sizeof(smx), "%+7.3f", mx);
  }
  snprintf(buf, bufLen, "mn:%s mx:%s", smn, smx);
}

void updateDisplay() {
  char buf[24];

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  // ── CURRENT ───────────────────────────────────────────────────────────────
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("CURRENT"));
  display.setCursor(104, 0);                 // right-align gain (4 chars × 6 px = 24 px)
  display.print(kGainLabel[gainIdxI]);

  display.setTextSize(2);
  display.setCursor(0, 8);
  fmtCurrentLarge(current_A, buf, sizeof(buf));
  display.print(buf);

  display.setTextSize(1);
  display.setCursor(0, 24);
  fmtMinMax(currentMin, currentMax, true, buf, sizeof(buf));
  display.print(buf);

  // ── VOLTAGE ───────────────────────────────────────────────────────────────
  display.setTextSize(1);
  display.setCursor(0, 32);
  display.print(F("VOLTAGE"));
  display.setCursor(104, 32);
  display.print(kGainLabel[gainIdxV]);

  display.setTextSize(2);
  display.setCursor(0, 40);
  fmtVoltageLarge(voltage_V, buf, sizeof(buf));
  display.print(buf);

  display.setTextSize(1);
  display.setCursor(0, 56);
  fmtMinMax(voltageMin, voltageMax, false, buf, sizeof(buf));
  display.print(buf);

  display.display();
}

// Show a brief splash while initialising — useful for spotting I2C failures
void showSplash(const char* msg) {
  display.clearDisplay();
  display.setCursor(0, 24);
  display.setTextSize(1);
  display.print(msg);
  display.display();
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(F("AMC_OLED starting"));

  Wire.begin();
  Wire.setClock(400000);

  // ── Buttons ───────────────────────────────────────────────────────────────
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_C, INPUT_PULLUP);

  // ── OLED ──────────────────────────────────────────────────────────────────
  if (!display.begin(0x3C, true)) {
    Serial.println(F("display failed — check I2C wiring and address"));
    while (1) delay(100);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setRotation(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 4);
  display.print(F("AMC Precision Meter"));
  display.setCursor(22, 16);
  display.print(F("Initialising..."));
  display.display();
  Serial.println(F("SSD1306 OK (0x3C)"));

  // ── ADS1015 ───────────────────────────────────────────────────────────────
  ads.setDataRate(RATE_ADS1015_3300SPS);
  if (!ads.begin(ADS_ADDR)) {
    showSplash("ADS1015 FAIL!");
    Serial.println(F("ADS1015 init failed — check I2C wiring and ADDR pin"));
    while (1) delay(100);
  }
  Serial.println(F("ADS1015 OK (0x48)"));

  delay(400);
  Serial.println(F("AMC_OLED ready"));
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  processSerial();

  if (capturing) {
    runCapture();
    return;
  }

  if (capReady) {
    dumpCapture();
  }

  takeMeasurements();
  processButtons();

  unsigned long now = millis();
  if (now - lastSerialMs  >= DISP_RATE_MS) { lastSerialMs  = now; sendData();       }
  if (now - lastDisplayMs >= DISP_RATE_MS) { lastDisplayMs = now; updateDisplay();  }
}
