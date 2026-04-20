/*
  AMC_CAN.ino
  Isolated precision current + voltage meter — CAN bus variant.

  Hardware
  ─────────────────────────────────────────────────────────────────────────────
  Adafruit Feather RP2040 CAN Bus  (#5724)
  Adafruit 128×64 OLED FeatherWing (SH1107, I2C 0x3C)
  ADS1015 12-bit ADC               (I2C 0x48, ADDR pin = GND)

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

  CAN bus pin conflicts — Feather RP2040 CAN Bus
  ─────────────────────────────────────────────────────────────────────────────
  The onboard MCP2515 occupies GPIO9 (SPI CS) and GPIO6 (INT), which are the
  same pins that the OLED FeatherWing uses for buttons A and B.  These two
  buttons CANNOT be used on this board.  Only Button C (GPIO5) is wired and
  is reassigned here to Reset min/max.  Use serial commands or the GUI for
  ZERO / UNZERO.

  OLED FeatherWing button wiring on this board
  ─────────────────────────────────────────────────────────────────────────────
  Button C  (GPIO5)  →  Reset min/max          ← only usable button
  Button A  (GPIO9)  →  CONFLICTS with CAN CS  ← do not use
  Button B  (GPIO6)  →  CONFLICTS with CAN INT ← do not use

  CAN messages  (see AMC_CAN.dbc for signal definitions)
  ─────────────────────────────────────────────────────────────────────────────
  0x100  AMC_LIVE    8 bytes  live current (µA, int32 LE) + voltage (0.1mV, int32 LE)
  0x101  AMC_MINMAX  8 bytes  min/max current (1mA, int16 LE) + voltage (10mV, int16 LE)
  Both messages transmitted at DISP_RATE_MS intervals (default 500 ms).

  Serial protocol  (identical to AMC_OLED — Python GUI / Android app compatible)
  ─────────────────────────────────────────────────────────────────────────────
  Outgoing (at DISP_RATE_MS):
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
  Adafruit MCP2515   (CAN controller)
  Adafruit SH110X    (OLED display)
  Adafruit GFX Library
  Adafruit ADS1X15
  Adafruit BusIO
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP2515.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_ADS1X15.h>

// ── CAN (MCP2515 on SPI1, hardwired on Feather RP2040 CAN Bus) ───────────────
// Verify these against the board schematic if behaviour is unexpected.
#define CAN_CS       19       // MCP2515 chip-select (SPI1)
#define CAN_BAUDRATE 500000  // 500 kbps — change to 250000 if your network needs it

// Message IDs
#define CAN_ID_LIVE   0x100
#define CAN_ID_MINMAX 0x101

Adafruit_MCP2515 CAN(CAN_CS, &SPI1);

// ── OLED (SH1107 64×128, rotated to 128×64 landscape) ────────────────────────
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// ── Button — only C (GPIO5) is available; A and B conflict with CAN pins ──────
#define BTN_C           5    // Reset min/max
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
static const float   kMvPerCount[] = { 3.000f, 2.000f, 1.000f, 0.500f, 0.250f, 0.125f };
static const char*   kGainLabel[]  = { "6.1V","4.1V","2.0V","1.0V","512m","256m" };
static const uint8_t kNumGain      = 6;

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

// ── Timing ────────────────────────────────────────────────────────────────────
String        serialBuf    = "";
unsigned long lastSerialMs  = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastCANMs     = 0;
float         currentOffset = 0.0f;

static const uint16_t DISP_RATE_MS = 500;  // display + serial rate
static const uint16_t CAN_RATE_MS  = 100;  // CAN transmit rate (10 Hz)

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
  (void)ads.readADC_Differential_0_1();

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
//  CAN TRANSMIT
// ═══════════════════════════════════════════════════════════════════════════
//
//  0x100  AMC_LIVE  (8 bytes)
//  ┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
//  │ byte 0 │ byte 1 │ byte 2 │ byte 3 │ byte 4 │ byte 5 │ byte 6 │ byte 7 │
//  ├──────────────────────────┼─────────────────────────────────────────────┤
//  │    Current_A  (int32 LE, scale 1e-6 A/count, µA resolution)            │
//  │                          │    Voltage_V  (int32 LE, scale 1e-4 V/count)│
//  └──────────────────────────┴─────────────────────────────────────────────┘
//
//  0x101  AMC_MINMAX  (8 bytes)
//  ┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
//  │ byte 0 │ byte 1 │ byte 2 │ byte 3 │ byte 4 │ byte 5 │ byte 6 │ byte 7 │
//  ├─────────────────┼─────────────────┼─────────────────┼─────────────────┤
//  │ Current_Min     │ Current_Max     │ Voltage_Min     │ Voltage_Max     │
//  │ (int16 LE,      │ (int16 LE,      │ (int16 LE,      │ (int16 LE,      │
//  │  scale 1e-3 A)  │  scale 1e-3 A)  │  scale 1e-2 V)  │  scale 1e-2 V)  │
//  └─────────────────┴─────────────────┴─────────────────┴─────────────────┘

static inline void pack16LE(uint8_t* buf, int16_t val) {
  buf[0] = (uint8_t)(val & 0xFF);
  buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

static inline void pack32LE(uint8_t* buf, int32_t val) {
  buf[0] = (uint8_t)(val & 0xFF);
  buf[1] = (uint8_t)((val >>  8) & 0xFF);
  buf[2] = (uint8_t)((val >> 16) & 0xFF);
  buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

void sendCAN() {
  uint8_t buf[8];

  // ── 0x100  AMC_LIVE ───────────────────────────────────────────────────────
  // Current: int32, 1 µA per count  →  multiply A by 1e6
  // Voltage: int32, 0.1 mV per count → multiply V by 1e4
  pack32LE(buf + 0, (int32_t)(current_A * 1e6f));
  pack32LE(buf + 4, (int32_t)(voltage_V * 1e4f));

  CAN.beginPacket(CAN_ID_LIVE);
  CAN.write(buf, 8);
  CAN.endPacket();

  // ── 0x101  AMC_MINMAX ─────────────────────────────────────────────────────
  // Current min/max: int16, 1 mA per count  → multiply A by 1e3, clamp to int16
  // Voltage min/max: int16, 10 mV per count → multiply V by 1e2, clamp to int16
  pack16LE(buf + 0, (int16_t)constrain(currentMin * 1e3f, -32768.0f, 32767.0f));
  pack16LE(buf + 2, (int16_t)constrain(currentMax * 1e3f, -32768.0f, 32767.0f));
  pack16LE(buf + 4, (int16_t)constrain(voltageMin * 1e2f, -32768.0f, 32767.0f));
  pack16LE(buf + 6, (int16_t)constrain(voltageMax * 1e2f, -32768.0f, 32767.0f));

  CAN.beginPacket(CAN_ID_MINMAX);
  CAN.write(buf, 8);
  CAN.endPacket();
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
//  BUTTON
// ═══════════════════════════════════════════════════════════════════════════

void processButtons() {
  unsigned long now = millis();
  if (now - lastBtnMs < BTN_DEBOUNCE_MS) return;

  // Only Button C is available — GPIO9 (A) and GPIO6 (B) are used by the CAN chip
  if (!digitalRead(BTN_C)) {
    lastBtnMs = now;
    doReset();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  DISPLAY
// ═══════════════════════════════════════════════════════════════════════════

static void fmtCurrentLarge(float a, char* buf, uint8_t bufLen) {
  if (fabsf(a) < 10.0f)
    snprintf(buf, bufLen, "%+8.5f A", a);  // "+X.XXXXX A" = 10 chars
  else
    snprintf(buf, bufLen, "%+8.4f A", a);  // "+XX.XXXX A" = 10 chars
}

static void fmtVoltageLarge(float v, char* buf, uint8_t bufLen) {
  if (fabsf(v) < 100.0f)
    snprintf(buf, bufLen, "%+8.4f V", v);  // "+XX.XXXX V" = 10 chars
  else
    snprintf(buf, bufLen, "%+8.3f V", v);  // "+XXX.XXX V" = 10 chars
}

static void fmtMinMax(float mn, float mx, bool isCurrent, char* buf, uint8_t bufLen) {
  char smn[9], smx[9];
  if (isCurrent) {
    snprintf(smn, sizeof(smn), "%+7.4f", mn);
    snprintf(smx, sizeof(smx), "%+7.4f", mx);
  } else {
    snprintf(smn, sizeof(smn), "%+7.3f", mn);
    snprintf(smx, sizeof(smx), "%+7.3f", mx);
  }
  snprintf(buf, bufLen, "mn:%s mx:%s", smn, smx);
}

void updateDisplay() {
  char buf[24];

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);   display.print(F("CURRENT"));
  display.setCursor(104, 0); display.print(kGainLabel[gainIdxI]);

  display.setTextSize(2);
  display.setCursor(0, 8);
  fmtCurrentLarge(current_A, buf, sizeof(buf));
  display.print(buf);

  display.setTextSize(1);
  display.setCursor(0, 24);
  fmtMinMax(currentMin, currentMax, true, buf, sizeof(buf));
  display.print(buf);

  display.setTextSize(1);
  display.setCursor(0, 32);   display.print(F("VOLTAGE"));
  display.setCursor(104, 32); display.print(kGainLabel[gainIdxV]);

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

void showSplash(const char* msg) {
  display.clearDisplay();
  display.setCursor(0, 4);
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
  Serial.println(F("AMC_CAN starting"));

  Wire.begin();
  Wire.setClock(400000);

  // ── Button C only — A and B pins belong to MCP2515 ────────────────────────
  pinMode(BTN_C, INPUT_PULLUP);

  // ── OLED ──────────────────────────────────────────────────────────────────
  if (!display.begin(0x3C, true)) {
    Serial.println(F("OLED init failed"));
    while (1) delay(100);
  }
  display.setRotation(1);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 4);  display.print(F("AMC Precision Meter"));
  display.setCursor(28, 16); display.print(F("CAN Bus variant"));
  display.setCursor(22, 28); display.print(F("Initialising..."));
  display.display();
  Serial.println(F("OLED OK (0x3C)"));

  // ── CAN (MCP2515 on SPI1) ─────────────────────────────────────────────────
  SPI1.begin();
  if (!CAN.begin(CAN_BAUDRATE)) {
    showSplash("MCP2515 FAIL!\nCheck SPI/CS pins");
    Serial.println(F("MCP2515 init failed — check SPI1 wiring and CS pin"));
    while (1) delay(100);
  }
  Serial.print(F("MCP2515 OK — CAN bus @ "));
  Serial.print(CAN_BAUDRATE / 1000); Serial.println(F(" kbps"));

  // ── ADS1015 ───────────────────────────────────────────────────────────────
  ads.setDataRate(RATE_ADS1015_3300SPS);
  if (!ads.begin(ADS_ADDR)) {
    showSplash("ADS1015 FAIL!");
    Serial.println(F("ADS1015 init failed — check I2C wiring"));
    while (1) delay(100);
  }
  Serial.println(F("ADS1015 OK (0x48)"));

  delay(300);
  Serial.println(F("AMC_CAN ready"));
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
  if (now - lastCANMs     >= CAN_RATE_MS)  { lastCANMs     = now; sendCAN();      }
  if (now - lastSerialMs  >= DISP_RATE_MS) { lastSerialMs  = now; sendData();     }
  if (now - lastDisplayMs >= DISP_RATE_MS) { lastDisplayMs = now; updateDisplay();}
}
