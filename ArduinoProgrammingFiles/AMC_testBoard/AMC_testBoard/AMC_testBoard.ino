/*
  AMC_testBoard.ino
  Isolated precision current + voltage meter.

  Hardware
  ─────────────────────────────────────────────────────────────────────────────
  Feather RP2350 + Adafruit 2.4" TFT FeatherWing v2 (#3315)
  ADS1015 12-bit ADC — single chip at I2C address 0x48 (ADDR pin = GND)
  TSC2007 resistive touch controller — I2C address 0x4B (A0 + A1 jumpers
    bridged on the FeatherWing back to avoid the default 0x48 collision)

  Channel assignments (ADS1015)
  ─────────────────────────────────────────────────────────────────────────────
  AIN0 − AIN1  →  AMC3301 output  (current via 0.1 Ω shunt)
  AIN2 − AIN3  →  AMC3330 output  (voltage via 50.5:1 resistor divider)

  Scaling
  ─────────────────────────────────────────────────────────────────────────────
  AMC3301 internal gain ≈ 8.2 V/V (fixed).  With a 0.01 Ω shunt:
    I (A) = V_ADS (V) ÷ (AMC3301_GAIN × SHUNT_R)
    Max current at ±250 mV ADS full-scale ≈ ±0.30 A

  AMC3330 is a unity-gain isolation amplifier; the 50.5:1 external divider
  scales the bus voltage down to the IC's input range:
    V (V) = V_ADS (V) × DIVIDER_RATIO

  Adjust AMC3301_GAIN, SHUNT_R, and DIVIDER_RATIO below if your component
  values or calibration differ.

  Digital smoothing
  ─────────────────────────────────────────────────────────────────────────────
  SMOOTH_N consecutive single-shot ADS1015 reads are averaged to form each
  displayed measurement.  At 3300 SPS each conversion takes ≈ 0.30 ms, so
  the default SMOOTH_N = 7 gives a ≈ 2.1 ms integration window per channel.
  A one-sample discard follows every gain change to let the input settle.

  Dynamic auto-range
  ─────────────────────────────────────────────────────────────────────────────
  After each smoothed burst the ADS1015 gain is stepped up or down so the
  averaged count stays between GAIN_LO (200) and GAIN_HI (1900) out of ±2047.
  The new gain takes effect on the next measurement cycle.

  Libraries required
  ─────────────────────────────────────────────────────────────────────────────
  Adafruit ILI9341
  Adafruit TSC2007
  Adafruit GFX Library
  Adafruit ADS1X15
  Adafruit NeoPixel
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_TSC2007.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_NeoPixel.h>

// ── Pin assignments (FeatherWing v2 hardwired) ───────────────────────────────
#define TFT_CS    9
#define TFT_DC   10
#define TS_IRQ    6
#define PIXEL_PIN 17

// ── I2C addresses ────────────────────────────────────────────────────────────
#define ADS_ADDR    0x48   // ADS1015, ADDR = GND
#define TS_I2C_ADDR 0x4B   // TSC2007, A0 + A1 jumpers bridged on FeatherWing

// ── Touch calibration (landscape rotation 1) ─────────────────────────────────
// rawY maps to screen X;  rawX maps to screen Y (inverted).
// Re-calibrate by printing raw values with TOUCH_DEBUG enabled below.
#define TS_RAW_Y_LEFT   578    // rawY when finger is at screen left edge  (X=0)
#define TS_RAW_Y_RIGHT 3320    // rawY when finger is at screen right edge (X=319)
#define TS_RAW_X_TOP   3636    // rawX when finger is at screen top edge   (Y=0)
#define TS_RAW_X_BOT    492    // rawX when finger is at screen bottom edge(Y=239)
#define TS_MIN_Z           10  // minimum pressure to register a touch
#define TOUCH_DEBOUNCE_MS 180  // ignore re-triggers within this many ms

// Uncomment to print raw touch coords to Serial for calibration
// #define TOUCH_DEBUG

// ── Display geometry (landscape 320 × 240) ───────────────────────────────────
#define DISP_W 320
#define DISP_H 240
#define HDR_H   28

// ── Colour palette (RGB565) ──────────────────────────────────────────────────
#define COL_BG      0x1082
#define COL_HDR     0x2124
#define COL_GREEN   0x07E0
#define COL_RED     0xF800
#define COL_YELLOW  0xFFE0
#define COL_WHITE   0xFFFF
#define COL_LTGRAY  0xC618
#define COL_DKGRAY  0x4208
#define COL_CYAN    0x07FF
#define COL_BTN_ON  0x0440
#define COL_BTN_OFF 0x2945

// ── Physical scaling — adjust to match your components / calibration ──────────
#define AMC3301_GAIN  8.2f    // AMC3301 internal amplifier gain (V/V, typical)
#define SHUNT_R       0.01f    // shunt resistor value (Ohms)
#define DIVIDER_RATIO 50.5f   // external voltage divider ratio for AMC3330

// ── Digital smoothing ────────────────────────────────────────────────────────
// Number of single-shot ADS1015 samples averaged per channel per reading.
// At 3300 SPS: SMOOTH_N = 7 → ≈ 2.1 ms integration window per channel.
#define SMOOTH_N 7

// ── ADS1015 gain table ───────────────────────────────────────────────────────
static const adsGain_t kGain[] = {
  GAIN_TWOTHIRDS,  // ±6.144 V  →  3.000 mV/count
  GAIN_ONE,        // ±4.096 V  →  2.000 mV/count
  GAIN_TWO,        // ±2.048 V  →  1.000 mV/count
  GAIN_FOUR,       // ±1.024 V  →  0.500 mV/count
  GAIN_EIGHT,      // ±0.512 V  →  0.250 mV/count
  GAIN_SIXTEEN     // ±0.256 V  →  0.125 mV/count
};
static const float kMvPerCount[] = { 3.000f, 2.000f, 1.000f, 0.500f, 0.250f, 0.125f };
static const char* kGainLabel[]  = { "6.1V","4.1V","2.0V","1.0V","512m","256m" };
static const uint8_t kNumGain    = 6;

// Auto-range hysteresis thresholds (absolute ADC counts, max = 2047)
static const int16_t GAIN_HI = 1900;  // step to lower gain (wider FS range)
static const int16_t GAIN_LO =  200;  // step to higher gain (better resolution)

// ── Peripheral objects ───────────────────────────────────────────────────────
Adafruit_ILI9341  tft(TFT_CS, TFT_DC);
Adafruit_TSC2007  ts;
Adafruit_ADS1015  ads;
Adafruit_NeoPixel pixel(1, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool tsOK = false;

// ── Measurement state ────────────────────────────────────────────────────────
float current_A  = 0.0f;
float voltage_V  = 0.0f;

float currentMin = 0.0f, currentMax = 0.0f;
float voltageMin = 0.0f, voltageMax = 0.0f;
bool  firstReading = true;

// Gain indices — start current at highest sensitivity, voltage at widest range
size_t gainIdxI = 5;  // GAIN_SIXTEEN → good start for small shunt voltages
size_t gainIdxV = 0;  // GAIN_TWOTHIRDS → widest range for divided bus voltage

// ── Touch state ──────────────────────────────────────────────────────────────
bool          touchActive  = false;
unsigned long touchStartMs = 0;

// ── Display timing ───────────────────────────────────────────────────────────
unsigned long lastDisplayMs      = 0;
static const uint16_t DISP_RATE_MS = 100;  // 10 Hz — fast enough, no flicker

// ── Serial protocol ──────────────────────────────────────────────────────────
// Outgoing (every 100 ms):
//   $AMC,<I>,<V>,<Imin>,<Imax>,<Vmin>,<Vmax>,<gainI_label>,<gainV_label>
// Incoming commands:
//   !RST      reset min/max to current live values
//   !ZERO     set a current offset so present reading = 0 A (tare)
//   !UNZERO   clear the current offset
String serialBuf           = "";
unsigned long lastSerialMs = 0;
float currentOffset        = 0.0f;  // subtracted from current_A after scaling

// ── Capture mode ─────────────────────────────────────────────────────────────
// Non-blocking: one sample is filled per loop() call while capturing=true.
// On completion the buffer is streamed to serial then normal meter resumes.
struct CapSample { float I; float V; uint32_t t_us; };
static const uint16_t CAP_MAX = 2000;
static CapSample capBuf[CAP_MAX];
uint16_t capN      = 0;     // requested count
uint16_t capIdx    = 0;     // samples collected so far
uint8_t  capSmooth = 0;     // 0 = use SMOOTH_N, 1-16 = override
bool     capturing = false;
bool     capReady  = false; // buffer full, dump pending

// ── UI geometry ──────────────────────────────────────────────────────────────
// Current section: y = HDR_H … 103
#define SEC_I_Y   HDR_H          // section top
#define SEC_I_LBL (SEC_I_Y + 2)  // label row y
#define SEC_I_VAL (SEC_I_Y + 12) // live value y  (size-3 text, 24 px tall → ends at +36)
#define SEC_I_MIN (SEC_I_Y + 52) // Min row y     (size-2 text, 16 px tall → ends at +68)
#define SEC_I_MAX (SEC_I_Y + 72) // Max row y     (size-2 text → ends at +88)

// Divider between sections
#define DIV1_Y  (HDR_H + 96)     // 124

// Voltage section: y = 128 … 220
#define SEC_V_Y   (DIV1_Y + 4)   // 128
#define SEC_V_LBL (SEC_V_Y + 2)  // label row
#define SEC_V_VAL (SEC_V_Y + 12) // live value (size-3, ends at +36)
#define SEC_V_MIN (SEC_V_Y + 52) // Min row
#define SEC_V_MAX (SEC_V_Y + 72) // Max row

// Divider + Reset button
#define DIV2_Y   (SEC_V_Y + 94)  // 222 — pushed to just above button
#define RST_X    80
#define RST_Y    (DIV2_Y - 34)   // 188
#define RST_W    160
#define RST_H    28

// Gain label right-side X (size-1 text, ≤ 6 chars × 6 px = 36 px)
#define GAIN_LBL_X 276

// ═══════════════════════════════════════════════════════════════════════════
//  MEASUREMENT
// ═══════════════════════════════════════════════════════════════════════════

// Reads smoothN single-shot samples from AIN0-AIN1, returns the averaged
// voltage in Volts.  One discard sample is taken after any gain change.
// gainIdx is updated for the NEXT call.
// smoothN defaults to the global SMOOTH_N; capture mode passes its own value.
float readSmoothedDiff01(size_t& gainIdx, uint8_t smoothN = SMOOTH_N) {
  size_t g = gainIdx;
  ads.setGain(kGain[g]);
  (void)ads.readADC_Differential_0_1();   // discard: lets input settle after gain set

  int32_t sum = 0;
  for (uint8_t i = 0; i < smoothN; i++) {
    sum += ads.readADC_Differential_0_1();
  }
  float avg_counts = (float)sum / (float)smoothN;
  float volts      = (avg_counts * kMvPerCount[g]) / 1000.0f;

  // Step gain for next call
  int16_t absAvg = (int16_t)fabsf(avg_counts);
  if      (absAvg > GAIN_HI && gainIdx > 0)                    gainIdx--;
  else if (absAvg < GAIN_LO && gainIdx < (size_t)(kNumGain-1)) gainIdx++;

  return volts;
}

// Same as above but for AIN2-AIN3.
float readSmoothedDiff23(size_t& gainIdx, uint8_t smoothN = SMOOTH_N) {
  size_t g = gainIdx;
  ads.setGain(kGain[g]);
  (void)ads.readADC_Differential_2_3();

  int32_t sum = 0;
  for (uint8_t i = 0; i < smoothN; i++) {
    sum += ads.readADC_Differential_2_3();
  }
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

  // Convert to physical units
  current_A = (v01 / (AMC3301_GAIN * SHUNT_R)) - currentOffset;
  voltage_V = v23 * DIVIDER_RATIO;

  // Update min/max
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
//  CAPTURE HELPERS
// ═══════════════════════════════════════════════════════════════════════════

// Fill one capture slot.  Called every loop() iteration while capturing=true.
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

// Stream the completed buffer to serial.
// Format: $S,<idx>,<t_us_rel>,<I_A>,<V_V>  then $CAPDONE summary.
void dumpCapture() {
  uint32_t t0 = capBuf[0].t_us;
  for (uint16_t i = 0; i < capIdx; i++) {
    Serial.print(F("$S,"));
    Serial.print(i);                    Serial.print(',');
    Serial.print(capBuf[i].t_us - t0); Serial.print(',');
    Serial.print(capBuf[i].I, 5);       Serial.print(',');
    Serial.println(capBuf[i].V, 4);
  }
  uint8_t sm = capSmooth ? capSmooth : SMOOTH_N;
  Serial.print(F("$CAPDONE,"));
  Serial.print(capIdx);                        Serial.print(',');
  Serial.print(capBuf[capIdx-1].t_us - t0);   Serial.print(',');
  Serial.print(sm);                            Serial.print(',');
  Serial.print(kGainLabel[gainIdxI]);          Serial.print(',');
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
        // Tare: add the current live reading to the offset so display reads 0
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
        // !CAPTURE,N          — use current SMOOTH_N
        // !CAPTURE,N,smooth   — override smooth count (1 = raw, no averaging)
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
//  DISPLAY
// ═══════════════════════════════════════════════════════════════════════════

void drawBtn(int16_t x, int16_t y, int16_t w, int16_t h,
             const char* label, bool pressed = false) {
  uint16_t fill   = pressed ? COL_BTN_ON  : COL_BTN_OFF;
  uint16_t border = pressed ? COL_GREEN   : COL_DKGRAY;
  uint16_t fg     = pressed ? COL_GREEN   : COL_LTGRAY;
  tft.fillRoundRect(x, y, w, h, 5, fill);
  tft.drawRoundRect(x, y, w, h, 5, border);
  tft.setTextColor(fg);
  tft.setTextSize(1);
  tft.setCursor(x + (w - (int16_t)strlen(label) * 6) / 2, y + (h - 8) / 2);
  tft.print(label);
}

inline bool inRect(int16_t px, int16_t py,
                   int16_t x,  int16_t y, int16_t w, int16_t h) {
  return (px >= x && px < x+w && py >= y && py < y+h);
}

// Paint the static chrome — called once at startup.
void drawFrame() {
  tft.fillScreen(COL_BG);

  // Header bar
  tft.fillRect(0, 0, DISP_W, HDR_H, COL_HDR);
  tft.setTextColor(COL_WHITE, COL_HDR);
  tft.setTextSize(1);
  tft.setCursor(92, 10); tft.print(F("AMC Precision Meter"));

  // Section labels (static, drawn once)
  tft.setTextColor(COL_DKGRAY, COL_BG);
  tft.setTextSize(1);
  tft.setCursor(4, SEC_I_LBL); tft.print(F("CURRENT  (AMC3301, 0.1 Ohm shunt)"));
  tft.setCursor(4, SEC_V_LBL); tft.print(F("VOLTAGE  (AMC3330, 50.5:1 divider)"));

  // Section dividers
  tft.drawFastHLine(4, DIV1_Y, DISP_W - 8, COL_DKGRAY);

  // Reset button
  drawBtn(RST_X, RST_Y, RST_W, RST_H, "RESET MIN/MAX");
}

// Overdraw only the changing numbers — no full-screen clear, no blank-frame flicker.
void updateDisplay() {
  char buf[20];

  // ── CURRENT: live reading (size 3, green) ──────────────────────────────────
  tft.setTextSize(3);
  tft.setTextColor(COL_GREEN, COL_BG);
  tft.setCursor(4, SEC_I_VAL);
  dtostrf(current_A, 9, 4, buf);   // e.g. "   0.1234"
  tft.print(buf);
  tft.print(F(" A "));

  // ── CURRENT: Min / Max (size 2, cyan) ─────────────────────────────────────
  tft.setTextSize(2);
  tft.setTextColor(COL_CYAN, COL_BG);
  tft.setCursor(4, SEC_I_MIN);
  tft.print(F("Min:"));
  dtostrf(currentMin, 8, 4, buf); tft.print(buf); tft.print(F("A "));

  tft.setCursor(4, SEC_I_MAX);
  tft.print(F("Max:"));
  dtostrf(currentMax, 8, 4, buf); tft.print(buf); tft.print(F("A "));

  // ── CURRENT: active gain (size 1, right side of label row) ────────────────
  tft.setTextSize(1);
  tft.setTextColor(COL_DKGRAY, COL_BG);
  tft.setCursor(GAIN_LBL_X, SEC_I_LBL);
  tft.print(kGainLabel[gainIdxI]); tft.print(F("  "));

  // ── VOLTAGE: live reading (size 3, yellow) ─────────────────────────────────
  tft.setTextSize(3);
  tft.setTextColor(COL_YELLOW, COL_BG);
  tft.setCursor(4, SEC_V_VAL);
  dtostrf(voltage_V, 9, 3, buf);
  tft.print(buf);
  tft.print(F(" V "));

  // ── VOLTAGE: Min / Max (size 2, cyan) ─────────────────────────────────────
  tft.setTextSize(2);
  tft.setTextColor(COL_CYAN, COL_BG);
  tft.setCursor(4, SEC_V_MIN);
  tft.print(F("Min:"));
  dtostrf(voltageMin, 8, 3, buf); tft.print(buf); tft.print(F("V "));

  tft.setCursor(4, SEC_V_MAX);
  tft.print(F("Max:"));
  dtostrf(voltageMax, 8, 3, buf); tft.print(buf); tft.print(F("V "));

  // ── VOLTAGE: active gain ───────────────────────────────────────────────────
  tft.setTextSize(1);
  tft.setTextColor(COL_DKGRAY, COL_BG);
  tft.setCursor(GAIN_LBL_X, SEC_V_LBL);
  tft.print(kGainLabel[gainIdxV]); tft.print(F("  "));
}

// ═══════════════════════════════════════════════════════════════════════════
//  TOUCH
// ═══════════════════════════════════════════════════════════════════════════

void mapTouch(uint16_t rawX, uint16_t rawY, int16_t* sx, int16_t* sy) {
  *sx = (int16_t)map((long)rawY, TS_RAW_Y_LEFT, TS_RAW_Y_RIGHT, 0, DISP_W);
  *sy = (int16_t)map((long)rawX, TS_RAW_X_TOP,  TS_RAW_X_BOT,  0, DISP_H);
  *sx = constrain(*sx, 0, DISP_W - 1);
  *sy = constrain(*sy, 0, DISP_H - 1);
}

void processTouches() {
  if (!tsOK) return;

  // IRQ high = no finger; clear active flag so next press is a fresh tap
  if (digitalRead(TS_IRQ)) { touchActive = false; return; }

  unsigned long now = millis();
  if (touchActive) return;                          // wait for release
  if (now - touchStartMs < TOUCH_DEBOUNCE_MS) return;

  TS_Point p = ts.getPoint();
  if ((p.x == 0 && p.y == 0) || p.z < TS_MIN_Z) return;

  touchActive  = true;
  touchStartMs = now;

  int16_t sx, sy;
  mapTouch((uint16_t)p.x, (uint16_t)p.y, &sx, &sy);

#ifdef TOUCH_DEBUG
  Serial.print(F("Touch raw ")); Serial.print(p.x);
  Serial.print(','); Serial.print(p.y);
  Serial.print(F(" z=")); Serial.print(p.z);
  Serial.print(F(" → screen ")); Serial.print(sx);
  Serial.print(','); Serial.println(sy);
#endif

  if (inRect(sx, sy, RST_X, RST_Y, RST_W, RST_H)) {
    doReset();
    drawBtn(RST_X, RST_Y, RST_W, RST_H, "RESET MIN/MAX", true);
    delay(120);
    drawBtn(RST_X, RST_Y, RST_W, RST_H, "RESET MIN/MAX", false);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("AMC_testBoard starting"));

  Wire.begin();
  Wire.setClock(400000);   // 400 kHz I2C for faster ADS conversions
  SPI.begin();
  pinMode(TS_IRQ, INPUT);  // TSC2007 IRQ: HIGH = idle, LOW = touched (no pull-up)

  // ── TFT (SPI) ────────────────────────────────────────────────────────────
  tft.begin();
  tft.setRotation(1);      // landscape
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_WHITE); tft.setTextSize(2);
  tft.setCursor(20, 80);   tft.print(F("AMC Precision Meter"));
  tft.setTextSize(1); tft.setTextColor(COL_LTGRAY);
  tft.setCursor(20, 110);  tft.print(F("Feather RP2350 + 2.4\" TFT FeatherWing v2"));
  tft.setCursor(20, 125);  tft.print(F("Initialising..."));

  // ── TSC2007 (I2C at 0x4B) ────────────────────────────────────────────────
  if (ts.begin(TS_I2C_ADDR, &Wire)) {
    tsOK = true;
    Serial.println(F("TSC2007 OK (0x4B)"));
  } else {
    Serial.println(F("TSC2007 not found — touch disabled"));
    tft.setTextColor(COL_YELLOW);
    tft.setCursor(20, 140); tft.print(F("Touch not found (check A0/A1 jumpers)"));
  }

  // ── ADS1015 (I2C at 0x48) ────────────────────────────────────────────────
  ads.setDataRate(RATE_ADS1015_3300SPS);   // fastest rate → shortest integration window
  if (!ads.begin(ADS_ADDR)) {
    tft.setTextColor(COL_RED);
    tft.setCursor(20, 155); tft.print(F("ADS1015 init failed!"));
    Serial.println(F("ADS1015 init failed — check I2C wiring and ADDR pin"));
    while (1) delay(100);
  }
  Serial.println(F("ADS1015 OK (0x48)"));
  Serial.print(F("Smooth window: ")); Serial.print(SMOOTH_N);
  Serial.print(F(" samples @ 3300 SPS = ~"));
  Serial.print((float)SMOOTH_N / 3.3f, 1); Serial.println(F(" ms per channel"));

  // ── NeoPixel ─────────────────────────────────────────────────────────────
  pixel.begin(); pixel.clear(); pixel.show();

  delay(400);
  drawFrame();
  Serial.println(F("AMC_testBoard ready"));
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  // Serial commands always run first so !ABORT works during capture
  processSerial();

  if (capturing) {
    // Capture mode: fill one slot per iteration, skip all other tasks
    runCapture();
    return;
  }

  if (capReady) {
    // Buffer just finished — stream to serial, then fall through to normal meter
    dumpCapture();
  }

  // Normal meter operation
  takeMeasurements();
  processTouches();

  unsigned long now = millis();
  if (now - lastSerialMs  >= DISP_RATE_MS) { lastSerialMs  = now; sendData();      }
  if (now - lastDisplayMs >= DISP_RATE_MS) { lastDisplayMs = now; updateDisplay(); }
}
