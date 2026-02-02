/*
 * OpenLeadDetect_Feather_Voltmeter.ino
 *
 * Differential voltmeter with open/closed lead detection.
 * Runs on Adafruit ESP32-S2/S3 TFT Feather (ST7789 240x135).
 * Ported from OpenLeadDetect_MinimalistProof_R4Nano.
 *
 * Hardware:
 *   - Adafruit ESP32-S2 or S3 TFT Feather
 *   - ADS1115 16-bit ADC on I2C (addr 0x48)
 *   - Voltage divider bridge with MOSFET on VbridgePin
 *   - VRef enable switch on VEnableControl (active-high, pulled up)
 *   - Pot on clsdThreshold for closed-detect sensitivity  (optional)
 *   - Pot on zeroThreshold for zero-voltage threshold      (optional)
 *
 * Serial commands (115200 baud):
 *   S  Force Stop    G  Go    U  Toggle updates
 *   M  Manual mode   R  Range toggle   D  Debug   ?  Help
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MAX1704X.h>

Adafruit_ADS1115 ads;

// TFT -- pin macros are predefined by the Feather board package
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// NeoPixel -- onboard LED (power shared with TFT_I2C_POWER)
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 33
#endif
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Battery fuel gauge (MAX17048 on ESP32-S3 TFT Feather, addr 0x36)
Adafruit_MAX17048 battMonitor;
bool  battFound    = false;
float battPct      = 0.0;
float battVoltage  = 0.0;
unsigned long lastBattRead = 0;
const unsigned long battReadInterval = 10000;  // 10 s between reads

// ── Pin assignments (adjust for your wiring) ─────────────────────
const int VbridgePin     = 5;    // MOSFET gate for bridge measurement
const int VEnablePin     = 6;    // Vref enable output
const int VEnableControl = 9;    // Vref enable switch (INPUT_PULLUP)
const int clsdThreshold  = A1;   // Pot: closed-detect sensitivity
const int zeroThreshold  = A0;   // Pot: zero-voltage threshold
const int cfSuppressPin  = 13;   // Pull low to suppress closed/float detection

// ── Timing ────────────────────────────────────────────────────────
unsigned long currentMillis   = 0;
unsigned long lastSerialTime  = 0;
unsigned long lastStatusTime  = 0;
unsigned long lastDisplayTime = 0;

const unsigned long serialInterval  = 1;
const unsigned long statusInterval  = 2000;
const unsigned long displayInterval = 1000;   // 4 Hz TFT refresh

// ── Voltage measurement ───────────────────────────────────────────
float medianVoltage     = 0.0;
float newVoltageReading = 0.0;
float prevOutVoltage    = 0.0;
float vActual           = 0.0;
int16_t countV          = 0;

const float VOLTAGE_SCALE_full = 69.4486;
const float VOLTAGE_SCALE_low  = 3.51108;
float vScale = 0.0;

bool vClimb      = false;
bool firstVoltRun = true;
bool vRefEnable  = false;
bool manual      = false;
bool range       = false;
bool updates     = true;
bool debug       = false;
bool forceStop   = false;
bool cfSuppress  = false;    // closed/float detection suppressed
bool alarm1       = false;
bool alarmFlag   = false;

// ── Lead-detection state ──────────────────────────────────────────
bool  vFloating        = false;
float bridgeV          = 0.0;
bool  Vzero            = true;
bool  VzeroFlag        = true;
bool  vClosed          = false;
bool  vUndefined       = true;
float ClosedConfidence = 10.0;
float vClosedThres     = 0.1;
float closedBias       = 0.0;
float CorFTrig         = 1.0;

bool vClosedtrig    = false;
bool vFloattrig     = false;
bool vUndefinedtrig = false;
bool prevVzero      = false;

// ── NeoPixel alert state ──────────────────────────────────────────
const float VOLTAGE_ALARM_THRESH     = 3.2;
const unsigned long BLINK_DURATION_MS    = 40;
const unsigned long CLOSED_BLINK_MIN_MS  = 500;  // max 2 green blinks/sec
bool prevVoltageHigh    = false;
bool prevClosedForBlink = false;
unsigned long lastClosedBlinkTime = 0;

// ── ADS1115 auto-ranging ──────────────────────────────────────────
const float GAIN_FACTOR_TWOTHIRDS = 0.1875;   // +/-6.144 V
const float GAIN_FACTOR_1         = 0.125;     // +/-4.096 V
const float GAIN_FACTOR_2         = 0.0625;    // +/-2.048 V
const float GAIN_FACTOR_4         = 0.03125;   // +/-1.024 V
const float GAIN_FACTOR_8         = 0.015625;  // +/-0.512 V
const float GAIN_FACTOR_16        = 0.0078125; // +/-0.256 V

static const adsGain_t kGainLevels[] = {
  GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO,
  GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN
};
static const float kGainFactors[] = {
  GAIN_FACTOR_TWOTHIRDS, GAIN_FACTOR_1, GAIN_FACTOR_2,
  GAIN_FACTOR_4, GAIN_FACTOR_8, GAIN_FACTOR_16
};
static const int kNumGainLevels       = sizeof(kGainLevels) / sizeof(kGainLevels[0]);
static const int ADC_COUNT_LOW_THRESH  = 10000;
static const int ADC_COUNT_HIGH_THRESH = 30000;
static size_t gainIndexVolt;

// ── TFT colour palette ───────────────────────────────────────────
#define COL_BG         ST77XX_BLACK
#define COL_HEADER     0x1082          // dark blue-grey
#define COL_TEXT       ST77XX_WHITE
#define COL_LABEL      0xBDF7          // light grey
#define COL_VOLTAGE    ST77XX_CYAN
#define COL_CLOSED_FG  ST77XX_WHITE
#define COL_CLOSED_BG  0x0400          // dark green
#define COL_FLOAT_FG   ST77XX_WHITE
#define COL_FLOAT_BG   0x8200          // dark orange
#define COL_VNONZERO   0xFFE0          // yellow
#define COL_DISABLED   0x7BEF          // grey
#define COL_DIVIDER    0x4208          // dim grey
#define COL_BAR_FILL   0x07E0          // bright green
#define COL_BAR_EMPTY  0x2104          // very dark grey

// ── Forward declarations ──────────────────────────────────────────
void drawStaticUI();
void measureVoltage();
void ClosedOrFloat();
void updateDisplay();
void statusUpdate();
void handleSerialCommands(char cmd);
void blinkPixel(uint8_t r, uint8_t g, uint8_t b, unsigned long duration = BLINK_DURATION_MS);

// ==================================================================
//  SETUP
// ==================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // ── Power on TFT + I2C ──
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // ── Initialise TFT ──
  tft.init(135, 240);
  tft.setRotation(3);              // landscape, USB on right
  tft.fillScreen(COL_BG);

  // Splash screen
  tft.setTextSize(3);
  tft.setTextColor(COL_VOLTAGE);
  tft.setCursor(16, 20);
  tft.print("OPEN LEAD");
  tft.setCursor(16, 50);
  tft.print("DETECT");
  tft.setTextSize(1);
  tft.setTextColor(COL_LABEL);
  tft.setCursor(16, 90);
  tft.print("Feather Voltmeter  v1.0");

  // ── I2C + ADC ──
  Wire.begin();
  analogReadResolution(12);        // ESP32 12-bit ADC (0-4095)

  while (!ads.begin(0x48, &Wire)) {
    Serial.println("ADS1115 not found - retrying...");
    delay(1000);
  }
  Serial.println("ADS1115 connected.");
  ads.setGain(GAIN_SIXTEEN);
  ads.setDataRate(RATE_ADS1115_860SPS);

  // ── GPIO ──
  pinMode(VbridgePin,     OUTPUT);
  pinMode(VEnablePin,     OUTPUT);
  pinMode(VEnableControl, INPUT_PULLUP);
  pinMode(clsdThreshold,  INPUT);
  pinMode(zeroThreshold,  INPUT);
  pinMode(cfSuppressPin,  INPUT_PULLUP);
  digitalWrite(VbridgePin, LOW);    // bridge disconnected (resting state)

  // ── NeoPixel ──
  pixel.begin();
  pixel.clear();
  pixel.show();

  // ── Battery monitor ──
  if (battMonitor.begin()) {
    battFound = true;
    battPct     = battMonitor.cellPercent();
    battVoltage = battMonitor.cellVoltage();
    Serial.print("MAX17048 found  Batt: ");
    Serial.print(battVoltage, 2);
    Serial.print("V  ");
    Serial.print(battPct, 0);
    Serial.println("%");
  } else {
    Serial.println("MAX17048 not found - battery info unavailable.");
  }

  delay(1200);                     // hold splash visible
  tft.fillScreen(COL_BG);
  drawStaticUI();
}

// ── Draw elements that never change ──────────────────────────────
void drawStaticUI() {
  // Header bar
  tft.fillRect(0, 0, 240, 16, COL_HEADER);
  tft.setTextSize(1);
  tft.setTextColor(COL_VOLTAGE, COL_HEADER);
  tft.setCursor(4, 4);
  tft.print("OPEN LEAD DETECT");

  // Dividers
  tft.drawFastHLine(0, 17, 240, COL_DIVIDER);
  tft.drawFastHLine(0, 68, 240, COL_DIVIDER);

  // Voltage section label
  tft.setTextSize(1);
  tft.setTextColor(COL_LABEL, COL_BG);
  tft.setCursor(4, 20);
  tft.print("VOLTAGE");
}

// ==================================================================
//  MAIN LOOP
// ==================================================================
void loop() {
  currentMillis = millis();

  // ── VRef switch ──
  vRefEnable = digitalRead(VEnableControl);
  digitalWrite(VEnablePin, vRefEnable ? HIGH : LOW);
  cfSuppress = !digitalRead(cfSuppressPin);   // active-low

  // ── Serial commands ──
  if (Serial.available()) {
    handleSerialCommands(Serial.read());
  }

  // ── Measure ──
  float prev = newVoltageReading;
  measureVoltage();
  vClimb = (fabs(prev) < fabs(newVoltageReading));

  // ── NeoPixel alerts (immediate blinks on state change) ──
  bool voltageHigh = (fabs(medianVoltage) > VOLTAGE_ALARM_THRESH);
  bool closedNow   = (vRefEnable && Vzero && ClosedConfidence > 10);

  // Red blink the instant voltage first crosses above threshold
  if (voltageHigh && !prevVoltageHigh) {
    blinkPixel(250, 0, 0);
  }
  prevVoltageHigh = voltageHigh;

  // Green blink the instant circuit first detected closed (rate-limited)
  if (closedNow && !prevClosedForBlink) {
    if (currentMillis - lastClosedBlinkTime >= CLOSED_BLINK_MIN_MS) {
      blinkPixel(0, 250, 0);
      lastClosedBlinkTime = currentMillis;
    }
  }
  prevClosedForBlink = closedNow;

  float vThreshold = vClimb ? 0.25f : 1.0f;

  // Serial output on significant voltage change
  if ((currentMillis - lastSerialTime >= serialInterval) &&
      (fabs(medianVoltage) + vThreshold < fabs(prevOutVoltage) ||
       fabs(medianVoltage) - vThreshold > fabs(prevOutVoltage))) {
    lastSerialTime = currentMillis;
    prevOutVoltage = medianVoltage;
    VzeroFlag = false;
    Serial.print("Vtrig: VDC:");
    Serial.print(medianVoltage, 4);
    Serial.print(" T(s):");
    Serial.println(currentMillis / 1000.0, 3);
  }

  // Periodic status
  if ((currentMillis - lastStatusTime >= statusInterval && updates) ||
      (Vzero && VzeroFlag)) {
    statusUpdate();
  }

  if ( alarm && !alarmFlag) { Serial.println("ALARM"); alarmFlag = true;  }
  if (!alarm &&  alarmFlag) { alarmFlag = false; }

  // ── Battery (slow read, every 10 s) ──
  if (battFound && (currentMillis - lastBattRead >= battReadInterval)) {
    lastBattRead = currentMillis;
    battPct     = constrain(battMonitor.cellPercent(), 0.0f, 100.0f);
    battVoltage = battMonitor.cellVoltage();
  }

  // ── Display ──
  if (currentMillis - lastDisplayTime >= displayInterval) {
    lastDisplayTime = currentMillis;
    updateDisplay();

    // Periodic blinks at screen refresh rate (1 Hz)
    if (voltageHigh) {
      blinkPixel(200, 0, 0);
    } else if (closedNow &&
               (currentMillis - lastClosedBlinkTime >= CLOSED_BLINK_MIN_MS)) {
      blinkPixel(0, 200, 0);
      lastClosedBlinkTime = currentMillis;
    }
  }
}

// ==================================================================
//  VOLTAGE MEASUREMENT  +  AUTO-RANGE  +  ZERO DETECT
// ==================================================================
void measureVoltage() {
  if (manual) {
    if (!range) {
      digitalWrite(VbridgePin, HIGH);
      vScale = VOLTAGE_SCALE_full;
    } else {
      digitalWrite(VbridgePin, LOW);
      vScale = VOLTAGE_SCALE_low;
      ads.setDataRate(RATE_ADS1115_64SPS);
    }
  } else {
    digitalWrite(VbridgePin, LOW);          // ensure bridge disconnected
    vScale = VOLTAGE_SCALE_full;
    ads.setDataRate(RATE_ADS1115_860SPS);
  }

  if (firstVoltRun) {
    gainIndexVolt = kNumGainLevels - 1;   // start at highest gain
    firstVoltRun = false;
  }

  ads.setGain(kGainLevels[gainIndexVolt]);
  countV = ads.readADC_Differential_0_1();

  if (abs(countV) > ADC_COUNT_HIGH_THRESH && gainIndexVolt > 0) {
    --gainIndexVolt;
    ads.setGain(kGainLevels[gainIndexVolt]);
    countV = ads.readADC_Differential_0_1();
  } else if (abs(countV) < ADC_COUNT_LOW_THRESH &&
             gainIndexVolt < (size_t)(kNumGainLevels - 1)) {
    ++gainIndexVolt;
    ads.setGain(kGainLevels[gainIndexVolt]);
    countV = ads.readADC_Differential_0_1();
  }

  vActual           = countV * kGainFactors[gainIndexVolt] / 1000.0f;
  newVoltageReading = (vActual * vScale)-0.023;
  medianVoltage     = newVoltageReading;

  // Zero-threshold pot  (ESP32 12-bit: 0-4095)
  CorFTrig = 0.2;
  //CorFTrig = (analogRead(zeroThreshold) / 4095.0f) * 0.2f;

  prevVzero = Vzero;

  if (fabs(medianVoltage) < CorFTrig && !manual && vRefEnable && !cfSuppress) {
    Vzero = true;
    VzeroFlag = (prevVzero != Vzero);
    ClosedOrFloat();
  } else {
    Vzero     = false;
    vFloating = false;
    VzeroFlag = false;
  }
}

// ==================================================================
//  BRIDGE MEASUREMENT  -  CLOSED / FLOAT CLASSIFICATION
// ==================================================================
void ClosedOrFloat() {
  bool prevClosed = vClosed;

  // Closed-threshold pot (12-bit)
  //closedBias   = (analogRead(clsdThreshold) / 4095.0f) * 2.0f;
  //vClosedThres = 0.1f * closedBias;

  vClosedThres = 0.1f;

  vClosed = vFloating = vUndefined = false;

  // First bridge sample
  digitalWrite(VbridgePin, HIGH);
  delay(5);
  ads.setDataRate(RATE_ADS1115_860SPS);
  ads.setGain(GAIN_SIXTEEN);
  float bv1 = ads.readADC_Differential_0_1()
              * (0.0078125f / 1000.0f) * VOLTAGE_SCALE_full;
  digitalWrite(VbridgePin, LOW);
/*
  // Second bridge sample
  digitalWrite(VbridgePin, HIGH);
  float bv2 = ads.readADC_Differential_0_1()
              * (0.0078125f / 1000.0f) * VOLTAGE_SCALE_full;
  digitalWrite(VbridgePin, LOW);

  bridgeV = (fabs(bv1) + fabs(bv2)) / 2.0f;
*/
  bridgeV = bv1;
  // ── Classification ──
  if (fabs(bridgeV) < vClosedThres) {
    if (vClosedtrig) vClosed = true;
    vClosedtrig = true;
    vFloattrig = vUndefinedtrig = false;
    ClosedConfidence = min(ClosedConfidence + 1.0f, 20.0f);
    if (debug) {
      Serial.print("Bridge:");
      Serial.print(bridgeV, 4);
      Serial.println(" -> Closed");
    }

  } else if (fabs(bridgeV) > vClosedThres) {
    if (vFloattrig) vFloating = true;
    vFloattrig = true;
    vClosedtrig = vUndefinedtrig = false;
    ClosedConfidence = max(ClosedConfidence - 1.0f, 0.0f);
    if (debug) {
      Serial.print("Bridge:");
      Serial.print(bridgeV, 4);
      Serial.println(" -> Float");
    }

  } else {
    // Exactly equal - hold previous state
    if (prevClosed) vClosed  = true;
    else            vFloating = true;
  }
}

// ==================================================================
//  NEOPIXEL BLINK
// ==================================================================
void blinkPixel(uint8_t r, uint8_t g, uint8_t b, unsigned long duration) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
  delay(duration);
  pixel.clear();
  pixel.show();
}

// ==================================================================
//  TFT DISPLAY UPDATE
// ==================================================================
void updateDisplay() {
  // Previous-state tracking (static persists across calls)
  static int  prevSt      = -1;     // forces first-call draw
  static bool prevVRef    = true;
  static bool prevVzDisp  = false;
  static bool firstUpdate = true;

  // ── Determine current states ──
  int st;
  if      (!vRefEnable)            st = 3;   // disabled
  else if (cfSuppress)             st = 4;   // closed/float suppressed
  else if (!Vzero)                 st = 2;   // V != 0
  else if (ClosedConfidence > 10)  st = 0;   // closed
  else                             st = 1;   // floating

  bool statusChanged = (st != prevSt)           || firstUpdate;
  bool vzeroChanged  = (Vzero != prevVzDisp)    || firstUpdate;
  bool vrefChanged   = (vRefEnable != prevVRef) || firstUpdate;

  // ── VRef indicator (header) -- only on change ──
  if (vrefChanged) {
    tft.fillRect(160, 0, 80, 16, COL_HEADER);
    tft.setTextSize(1);
    tft.setCursor(164, 4);
    if (vRefEnable) {
      tft.setTextColor(COL_BAR_FILL, COL_HEADER);
      tft.print("VRef: ON ");
    } else {
      tft.setTextColor(0xFDA0, COL_HEADER);   // orange
      tft.print("VRef: OFF");
    }
  }

  // ── Voltage value ──
  // Clear once when Vzero state toggles (format/colour changes)
  if (vzeroChanged) {
    tft.fillRect(0, 30, 240, 36, COL_BG);
  }
  // Overwrite in place -- setTextColor(fg, bg) paints background
  // behind each character, so no fillRect needed between refreshes.
  tft.setTextSize(3);
  char vBuf[14];
  if (!Vzero) {
    tft.setTextColor(COL_VOLTAGE, COL_BG);
    dtostrf(medianVoltage, 10, 3, vBuf);       // fixed 10-char width
  } else {
    tft.setTextColor(COL_LABEL, COL_BG);
    char tmp[8];
    dtostrf(CorFTrig, 5, 3, tmp);
    snprintf(vBuf, sizeof(vBuf), "  <%s    ", tmp);  // pad to ~10 chars
  }
  tft.setCursor(4, 34);
  tft.print(vBuf);

  // Unit label (constant position, bg-colour overwrites itself)
  tft.setTextSize(2);
  tft.setTextColor(COL_LABEL, COL_BG);
  tft.setCursor(222, 38);
  tft.print("V");

  // ── Lead-status badge -- only on change ──
  if (statusChanged) {
    tft.fillRect(0, 70, 168, 26, COL_BG);      // clear badge zone
    tft.setTextSize(2);
    switch (st) {
      case 0:
        tft.fillRoundRect(4, 72, 130, 22, 4, COL_CLOSED_BG);
        tft.setTextColor(COL_CLOSED_FG, COL_CLOSED_BG);
        tft.setCursor(20, 75);
        tft.print("CLOSED");
        break;
      case 1:
        tft.fillRoundRect(4, 72, 154, 22, 4, COL_FLOAT_BG);
        tft.setTextColor(COL_FLOAT_FG, COL_FLOAT_BG);
        tft.setCursor(14, 75);
        tft.print("FLOATING");
        break;
      case 2:
        tft.setTextColor(COL_VNONZERO, COL_BG);
        tft.setCursor(8, 75);
        tft.print("(V != 0)");
        break;
      case 3:
        tft.setTextColor(COL_DISABLED, COL_BG);
        tft.setCursor(8, 75);
        tft.print("DISABLED");
        break;
      case 4:
        tft.setTextColor(COL_VOLTAGE, COL_BG);
        tft.setCursor(8, 75);
        tft.print("VOLT ONLY");
        break;
    }
    // Clear confidence / bar areas when leaving a state that shows them
    if (!(vRefEnable && Vzero && !cfSuppress)) {
      tft.fillRect(170, 72, 70, 22, COL_BG);
      tft.fillRect(3, 97, 204, 8, COL_BG);
    }
  }

  // ── Confidence percentage (dynamic, overwrites in place) ──
  if (vRefEnable && Vzero && !cfSuppress) {
    float pct = ClosedConfidence * 5.0f;
    tft.setTextSize(2);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.setCursor(174, 75);
    char cBuf[8];
    snprintf(cBuf, sizeof(cBuf), "%3d%%", (int)pct);   // fixed 4-char
    tft.print(cBuf);
  }

  // ── Confidence bar (fills with colour, no black flash) ──
  if (vRefEnable && Vzero && !cfSuppress) {
    float pct = ClosedConfidence * 5.0f;
    int barW  = (int)(pct / 100.0f * 200);
    tft.fillRect(4,        98, barW,       6, COL_BAR_FILL);
    tft.fillRect(4 + barW, 98, 200 - barW, 6, COL_BAR_EMPTY);
    tft.drawRect(3, 97, 202, 8, COL_DIVIDER);
  }

  // ── Bottom info (overwrite in place with bg colour, no fillRect) ──
  char buf[12];
  tft.setTextSize(1);
  tft.setTextColor(COL_LABEL, COL_BG);

  tft.setCursor(4, 112);
  tft.print("Thres:");
  dtostrf(vClosedThres, 5, 3, buf);  tft.print(buf);

  tft.setCursor(90, 112);
  tft.print("Zero:");
  dtostrf(CorFTrig, 5, 3, buf);      tft.print(buf);

  tft.setCursor(180, 112);
  tft.print("Gain:");
  tft.print((int)gainIndexVolt);
  tft.print(" ");                     // trailing space covers shrinking digit

  tft.setCursor(4, 124);
  tft.print("Bridge:");
  dtostrf(bridgeV, 8, 4, buf);       tft.print(buf);

  tft.setCursor(120, 124);
  tft.print("T:");
  dtostrf(currentMillis / 1000.0f, 7, 1, buf);
  tft.print(buf);
  tft.print("s");

  // Battery in bottom-right corner
  tft.setCursor(198, 124);
  if (battFound) {
    char bBuf[8];
    snprintf(bBuf, sizeof(bBuf), "B:%3d%%", (int)battPct);
    tft.print(bBuf);
  } else {
    tft.print("B: --");
  }

  // ── Update tracked state ──
  prevSt     = st;
  prevVRef   = vRefEnable;
  prevVzDisp = Vzero;
  firstUpdate = false;
}

// ==================================================================
//  SERIAL STATUS
// ==================================================================
void statusUpdate() {
  lastStatusTime = currentMillis;
  VzeroFlag = false;

  Serial.print("Status:");
  if (alarm)          Serial.print(" ALARM! ");
  else if (forceStop) Serial.print(" Force Stop ");
  else                Serial.print(" Good ");

  if (Vzero) {
    Serial.print(" Bridge:");    Serial.print(bridgeV, 4);
    Serial.print(" / Thres:");   Serial.print(vClosedThres, 4);
    Serial.print(" / ");         Serial.print(vClosed ? "Closed" : "Floating");
  } else {
    Serial.print(" / V !0");
  }

  Serial.print(" / VDC:");   Serial.print(medianVoltage, 4);
  Serial.print(" Actual:");   Serial.print(vActual, 4);
  Serial.print(" Count:");    Serial.print(countV);
  if (battFound) {
    Serial.print(" Batt:");   Serial.print(battVoltage, 2);
    Serial.print("V ");       Serial.print((int)battPct);
    Serial.print("%");
  }
  Serial.print(" T(s):");    Serial.println(currentMillis / 1000.0, 3);
}

// ==================================================================
//  SERIAL COMMANDS
// ==================================================================
void handleSerialCommands(char command) {
  switch (command) {
    case 'S': Serial.println("Force Stop On");    forceStop = true;    break;
    case 'G': Serial.println("Going");            forceStop = false;   break;
    case 'U': Serial.println("Updates toggled");   updates = !updates;  break;
    case 'M': Serial.println("Manual toggled");    manual  = !manual;   break;
    case 'R': Serial.println("Range toggled");     range   = !range;    break;
    case 'D': Serial.println("Debug toggled");     debug   = !debug;    break;
    case '?':
      Serial.println("Commands: S(Stop) G(Go) U(Updates) M(Manual) R(Range) D(Debug)");
      break;
    default: break;
  }
}
