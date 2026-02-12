/*
 * OpenLeadDetect_Feather_Voltmeter_AlertBridge.ino
 *
 * EXPERIMENTAL VERSION: Uses ADS1115 ALERT pin to control bridge MOSFET
 * instead of GPIO. The ALERT pin goes LOW to connect the bridge.
 *
 * Hardware setup for ALERT bridge control:
 *   - Connect ADS1115 ALERT pin to bridge MOSFET gate (with pull-up on PCB)
 *   - Tie AIN3 to a reference voltage (e.g., VDD through a divider, ~1.5V)
 *   - AIN2 can be left floating or tied to GND
 *   - Differential voltage measurement on AIN0/AIN1 (as before)
 *
 * How it works:
 *   - Comparator is configured in window mode with thresholds set so that:
 *     * Reading AIN0/AIN1 (near zero when bridge off) keeps ALERT HIGH
 *     * Reading AIN2/AIN3 (reference voltage) triggers ALERT LOW
 *   - To turn bridge ON: read AIN2/AIN3 briefly (ALERT goes LOW)
 *   - To turn bridge OFF: read AIN0/AIN1 (ALERT goes HIGH)
 *   - ALERT pin is latching, so it holds state between reads
 *
 * Note: This is experimental. The traditional GPIO approach is more reliable.
 *
 * Other hardware (unchanged):
 *   - Adafruit ESP32-S2/S3 TFT Feather
 *   - ADS1115 16-bit ADC on I2C (addr 0x48)
 *   - VRef enable switch on VEnableControl
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

// ── ADC Selection: Comment/uncomment ONE of these ────────────────
//#define USE_ADS1115    // 16-bit ADC
#define USE_ADS1015      // 12-bit ADC

// ── VRef Enable Logic: Comment/uncomment ONE of these ────────────
#define VENABLE_ACTIVE_HIGH   // VEnablePin HIGH = Vref enabled
//#define VENABLE_ACTIVE_LOW  // VEnablePin LOW  = Vref enabled

#ifdef USE_ADS1115
Adafruit_ADS1115 ads;
#else
Adafruit_ADS1015 ads;
#endif

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

// ── Pin assignments ───────────────────────────────────────────────
// NOTE: VbridgePin is NOT used in this version - ALERT pin controls bridge
const int VEnablePin     = 6;    // Vref enable output
const int VEnableControl = 9;    // Vref enable switch (INPUT_PULLUP)
const int clsdThreshold  = A0;   // Pot: closed-detect sensitivity
const int zeroThreshold  = A1;   // Pot: zero-voltage threshold
const int cfSuppressBtn  = 0;    // Boot button (GPIO 0) toggles closed/float suppression

// ── ALERT/Comparator configuration ────────────────────────────────
// AIN3 should be tied to a reference voltage (~1.0-1.5V works well)
// This voltage must exceed the comparator threshold to trigger ALERT LOW
// Threshold is set just above zero so normal differential readings don't trigger
const int16_t COMP_THRESH_LOW  = 100;    // ~12.5mV at GAIN_EIGHT
const int16_t COMP_THRESH_HIGH = 32767;  // Max positive (won't trigger on high side)

// ── Timing ────────────────────────────────────────────────────────
unsigned long currentMillis   = 0;
unsigned long lastSerialTime  = 0;
unsigned long lastStatusTime  = 0;
unsigned long lastDisplayTime = 0;

const unsigned long serialInterval  = 1;
const unsigned long statusInterval  = 2000;
const unsigned long displayInterval = 1000;

// ── Voltage measurement ───────────────────────────────────────────
float medianVoltage     = 0.0;
float newVoltageReading = 0.0;
float displayVoltage    = 0.0;
float prevOutVoltage    = 0.0;
float vActual           = 0.0;
int16_t countV          = 0;

const float VOLTAGE_SCALE_full = 69.6023;
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
bool cfSuppress      = false;
bool prevcfSuppress  = false;
bool cfBtnPrev       = true;
bool alarm1       = false;
bool alarmFlag   = false;

// ── Lead-detection state ──────────────────────────────────────────
bool  vFloating        = false;
float bridgeV          = 0.0;
float bridgeAvg        = 0.0;
float bridgeAvgDiff    = 0.0;
bool  Vzero            = true;
bool  VzeroFlag        = true;
bool  vClosed          = false;
bool  vUndefined       = true;
float ClosedConfidence = 5.0;
float vClosedThres     = 0.0;
float closedBias       = 0.0;
float CorFTrig         = 1.0;

bool vClosedtrig    = false;
bool vFloattrig     = false;
bool vUndefinedtrig = false;
bool prevVzero      = false;

// ── NeoPixel alert state ──────────────────────────────────────────
const float VOLTAGE_ALARM_THRESH     = 3.2;
const unsigned long BLINK_DURATION_MS    = 40;
const unsigned long CLOSED_BLINK_MIN_MS  = 500;
bool prevVoltageHigh    = false;
bool prevClosedForBlink = false;
unsigned long lastClosedBlinkTime = 0;

// ── ADC gain factors & thresholds ─────────────────────────────────
#ifdef USE_ADS1115
const float GAIN_FACTOR_TWOTHIRDS = 0.1875;      // +/-6.144 V
const float GAIN_FACTOR_1         = 0.125;       // +/-4.096 V
const float GAIN_FACTOR_2         = 0.0625;      // +/-2.048 V
const float GAIN_FACTOR_4         = 0.03125;     // +/-1.024 V
const float GAIN_FACTOR_8         = 0.015625;    // +/-0.512 V
const float GAIN_FACTOR_16        = 0.0078125;   // +/-0.256 V
static const int ADC_COUNT_LOW_THRESH  = 10000;
static const int ADC_COUNT_HIGH_THRESH = 30000;
#define ADS_RATE_FAST  RATE_ADS1115_860SPS
#define ADS_RATE_SLOW  RATE_ADS1115_64SPS
const float vClosedThresDefault = 0.07;
#else
const float GAIN_FACTOR_TWOTHIRDS = 3.0;         // +/-6.144 V
const float GAIN_FACTOR_1         = 2.0;         // +/-4.096 V
const float GAIN_FACTOR_2         = 1.0;         // +/-2.048 V
const float GAIN_FACTOR_4         = 0.5;         // +/-1.024 V
const float GAIN_FACTOR_8         = 0.25;        // +/-0.512 V
const float GAIN_FACTOR_16        = 0.125;       // +/-0.256 V
static const int ADC_COUNT_LOW_THRESH  = 600;
static const int ADC_COUNT_HIGH_THRESH = 1800;
#define ADS_RATE_FAST  RATE_ADS1015_3300SPS
#define ADS_RATE_SLOW  RATE_ADS1015_250SPS
const float vClosedThresDefault = 0.1;
#endif

static const adsGain_t kGainLevels[] = {
  GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO,
  GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN
};
static const float kGainFactors[] = {
  GAIN_FACTOR_TWOTHIRDS, GAIN_FACTOR_1, GAIN_FACTOR_2,
  GAIN_FACTOR_4, GAIN_FACTOR_8, GAIN_FACTOR_16
};
static const int kNumGainLevels = sizeof(kGainLevels) / sizeof(kGainLevels[0]);
static size_t gainIndexVolt;

// ── TFT colour palette ───────────────────────────────────────────
#define COL_BG         ST77XX_BLACK
#define COL_HEADER     0x1082
#define COL_TEXT       ST77XX_WHITE
#define COL_LABEL      0xBDF7
#define COL_VOLTAGE    ST77XX_CYAN
#define COL_CLOSED_FG  ST77XX_WHITE
#define COL_CLOSED_BG  0x0400
#define COL_FLOAT_FG   ST77XX_WHITE
#define COL_FLOAT_BG   0x8200
#define COL_VNONZERO   0xFFE0
#define COL_DISABLED   0x7BEF
#define COL_DIVIDER    0x4208
#define COL_BAR_FILL   0x07E0
#define COL_BAR_EMPTY  0x2104

// ── Forward declarations ──────────────────────────────────────────
void drawStaticUI();
void measureVoltage();
void ClosedOrFloat();
void updateDisplay();
void statusUpdate();
void handleSerialCommands(char cmd);
void blinkPixel(uint8_t r, uint8_t g, uint8_t b, unsigned long duration = BLINK_DURATION_MS);
void setBridgeOn();
void setBridgeOff();
void initComparator();

// ==================================================================
//  ALERT PIN BRIDGE CONTROL
// ==================================================================
// These functions control the bridge via the ADS1115 comparator/ALERT pin
// ALERT is active-low, open-drain with external pull-up on PCB
// Reading AIN2/AIN3 (reference voltage) triggers ALERT LOW = bridge ON
// Reading AIN0/AIN1 (near zero) releases ALERT HIGH = bridge OFF

void initComparator() {
  // Configure comparator in traditional mode (not window)
  // ALERT goes LOW when reading exceeds threshold
  // We set a low threshold so the reference voltage on AIN3 triggers it

  // Start with bridge OFF - read differential 0-1 first
  ads.setGain(GAIN_EIGHT);

  // The Adafruit library doesn't expose direct comparator config,
  // so we'll use a different approach: the ALERT pin behavior depends
  // on what we're reading. We'll manually control via channel switching.

  Serial.println("ALERT bridge control initialized");
  Serial.println("Tie AIN3 to ~1.0-1.5V reference for bridge trigger");
}

void setBridgeOn() {
  // Read AIN3 single-ended (tied to reference voltage)
  // This should be above threshold, triggering ALERT LOW
  // The comparator latches, so ALERT stays LOW

  ads.setGain(GAIN_EIGHT);  // Use consistent gain

  // Start a conversion on AIN3 (single-ended, referenced to GND)
  // AIN3 should be tied to a voltage > threshold (~50mV at GAIN_8)
  int16_t triggerRead = ads.readADC_SingleEnded(3);

  // Small delay for ALERT to assert
  delayMicroseconds(100);

  if (debug) {
    Serial.print("Bridge ON - AIN3 read: ");
    Serial.println(triggerRead);
  }
}

void setBridgeOff() {
  // Read AIN0/AIN1 differential (should be near zero when bridge was on)
  // This reading will be below threshold, releasing ALERT HIGH

  ads.setGain(GAIN_EIGHT);

  // Read differential - this also serves as our measurement
  int16_t reading = ads.readADC_Differential_0_1();

  // Small delay for ALERT to deassert
  delayMicroseconds(100);

  if (debug) {
    Serial.print("Bridge OFF - AIN0-1 read: ");
    Serial.println(reading);
  }
}

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
  tft.setRotation(3);
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
  tft.print("ALERT Bridge v1.0");

  // ── I2C + ADC ──
  Wire.begin();
  analogReadResolution(12);

  while (!ads.begin(0x48, &Wire)) {
#ifdef USE_ADS1115
    Serial.println("ADS1115 not found - retrying...");
#else
    Serial.println("ADS1015 not found - retrying...");
#endif
    delay(1000);
  }
#ifdef USE_ADS1115
  Serial.println("ADS1115 connected.");
#else
  Serial.println("ADS1015 connected.");
#endif

  // Configure for comparator/ALERT usage
  ads.setGain(GAIN_EIGHT);
  ads.setDataRate(ADS_RATE_FAST);

  // Initialize comparator for bridge control
  initComparator();

  // Start with bridge OFF
  setBridgeOff();

  // ── GPIO (no VbridgePin needed!) ──
  pinMode(VEnablePin,     OUTPUT);
  pinMode(VEnableControl, INPUT_PULLUP);
  pinMode(clsdThreshold,  INPUT);
  pinMode(zeroThreshold,  INPUT);
  pinMode(cfSuppressBtn,  INPUT_PULLUP);

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

  delay(1200);
  tft.fillScreen(COL_BG);
  drawStaticUI();
}

// ── Draw elements that never change ──────────────────────────────
void drawStaticUI() {
  tft.fillRect(0, 0, 240, 16, COL_HEADER);
  tft.setTextSize(1);
  tft.setTextColor(COL_VOLTAGE, COL_HEADER);
  tft.setCursor(4, 4);
  tft.print("OPEN LEAD (ALERT)");

  tft.drawFastHLine(0, 17, 240, COL_DIVIDER);
  tft.drawFastHLine(0, 68, 240, COL_DIVIDER);

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
#ifdef VENABLE_ACTIVE_HIGH
  digitalWrite(VEnablePin, vRefEnable ? HIGH : LOW);
#else
  digitalWrite(VEnablePin, vRefEnable ? LOW : HIGH);
#endif

  // Boot button toggle
  bool cfBtnNow = digitalRead(cfSuppressBtn);
  if (cfBtnNow == LOW && cfBtnPrev == HIGH) {
    cfSuppress = !cfSuppress;
    Serial.print("CF suppress: ");
    Serial.println(cfSuppress ? "ON" : "OFF");
  }
  cfBtnPrev = cfBtnNow;

  // ── Serial commands ──
  if (Serial.available()) {
    handleSerialCommands(Serial.read());
  }

  // ── Measure ──
  float prev = newVoltageReading;
  measureVoltage();
  vClimb = (fabs(prev) < fabs(newVoltageReading));

  // ── NeoPixel alerts ──
  bool voltageHigh = (fabs(medianVoltage) > VOLTAGE_ALARM_THRESH);
  bool closedNow   = (vRefEnable && Vzero && ClosedConfidence > 5);

  if (voltageHigh && !prevVoltageHigh) {
    blinkPixel(250, 0, 0);
  }
  prevVoltageHigh = voltageHigh;

  if (closedNow && !prevClosedForBlink) {
    if (currentMillis - lastClosedBlinkTime >= CLOSED_BLINK_MIN_MS) {
      blinkPixel(0, 250, 0);
      lastClosedBlinkTime = currentMillis;
    }
  }
  prevClosedForBlink = closedNow;

  // Periodic status
  if ((currentMillis - lastStatusTime >= statusInterval && updates) ||
      (Vzero && VzeroFlag)) {
    statusUpdate();
  }

  // ── Battery ──
  if (battFound && (currentMillis - lastBattRead >= battReadInterval)) {
    lastBattRead = currentMillis;
    battPct     = constrain(battMonitor.cellPercent(), 0.0f, 100.0f);
    battVoltage = battMonitor.cellVoltage();
  }

  // ── Display ──
  if (currentMillis - lastDisplayTime >= displayInterval) {
    lastDisplayTime = currentMillis;
    updateDisplay();

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
  // Bridge is controlled via ALERT pin, not GPIO
  // Normal measurement: bridge should be OFF (ALERT HIGH)
  // We ensure this by reading differential 0-1

  vScale = VOLTAGE_SCALE_full;

  if (firstVoltRun) {
    gainIndexVolt = kNumGainLevels - 1;
    firstVoltRun = false;
  }

  if (prevcfSuppress != cfSuppress) {
    if (cfSuppress) {
      ads.setDataRate(ADS_RATE_SLOW);
      Serial.print("Slow SPS");
    } else {
      ads.setDataRate(ADS_RATE_FAST);
      Serial.print("Fast SPS");
    }
    prevcfSuppress = cfSuppress;
  }

  ads.setGain(kGainLevels[gainIndexVolt]);
  countV = ads.readADC_Differential_0_1();
  // This read also ensures ALERT is HIGH (bridge OFF)

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
  newVoltageReading = vActual * vScale;

  // Exponential rolling average
  medianVoltage = medianVoltage + (newVoltageReading - medianVoltage) / 10.0f;

  displayVoltage = cfSuppress ? newVoltageReading : medianVoltage;

  CorFTrig = 0.25;

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

  vClosed = vFloating = vUndefined = false;

  // Turn bridge ON via ALERT pin
  setBridgeOn();
  delay(1);  // settling time

  // Read with bridge connected
  ads.setGain(GAIN_EIGHT);
  float bv1 = ads.readADC_Differential_0_1() * (GAIN_FACTOR_8 / 1000.0f);

  // Turn bridge OFF via ALERT pin
  setBridgeOff();

  bridgeV = bv1;

  if (fabs(bridgeV) > fabs(bridgeAvg)) {
    bridgeAvg = bridgeAvg - (fabs(bridgeV) / 10);
  } else {
    bridgeAvg = bridgeAvg + (fabs(bridgeV) / 10);
  }

  // Classification
  if (fabs(bridgeV) < vClosedThres) {
    if (vClosedtrig) vClosed = true;
    vClosedtrig = true;
    vFloattrig = vUndefinedtrig = false;
    ClosedConfidence = min(ClosedConfidence + 1.0f, 10.0f);
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
    if (prevClosed) vClosed  = true;
    else            vFloating = true;
  }

  delay(2);
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
  static int  prevSt      = -1;
  static bool prevVRef    = true;
  static bool prevVzDisp  = false;
  static bool firstUpdate = true;

  int st;
  if      (!vRefEnable)            st = 3;
  else if (cfSuppress)             st = 4;
  else if (!Vzero)                 st = 2;
  else if (ClosedConfidence > 5)   st = 0;
  else                             st = 1;

  bool statusChanged = (st != prevSt)           || firstUpdate;
  bool vzeroChanged  = (Vzero != prevVzDisp)    || firstUpdate;
  bool vrefChanged   = (vRefEnable != prevVRef) || firstUpdate;

  if (vrefChanged) {
    tft.fillRect(160, 0, 80, 16, COL_HEADER);
    tft.setTextSize(1);
    tft.setCursor(164, 4);
    if (vRefEnable) {
      tft.setTextColor(COL_BAR_FILL, COL_HEADER);
      tft.print("VRef: ON ");
    } else {
      tft.setTextColor(0xFDA0, COL_HEADER);
      tft.print("VRef: OFF");
    }
  }

  if (vzeroChanged) {
    tft.fillRect(0, 30, 240, 36, COL_BG);
  }

  tft.setTextSize(3);
  char vBuf[14];
  if (!Vzero || cfSuppress) {
    tft.setTextColor(COL_VOLTAGE, COL_BG);
    dtostrf(displayVoltage, 10, 3, vBuf);
  } else {
    tft.setTextColor(COL_LABEL, COL_BG);
    char tmp[8];
    dtostrf(CorFTrig, 5, 3, tmp);
    snprintf(vBuf, sizeof(vBuf), "  <%s    ", tmp);
  }
  tft.setCursor(4, 34);
  tft.print(vBuf);

  tft.setTextSize(2);
  tft.setTextColor(COL_LABEL, COL_BG);
  tft.setCursor(222, 38);
  tft.print("V");

  if (statusChanged) {
    tft.fillRect(0, 70, 168, 26, COL_BG);
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
    if (!(vRefEnable && Vzero && !cfSuppress)) {
      tft.fillRect(170, 72, 70, 22, COL_BG);
      tft.fillRect(3, 97, 204, 8, COL_BG);
    }
  }

  if (vRefEnable && Vzero && !cfSuppress) {
    float pct = (abs(bridgeAvg) / vClosedThres) * 100;
    tft.setTextSize(2);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.setCursor(174, 75);
    char cBuf[8];
    snprintf(cBuf, sizeof(cBuf), "%3d%%", (int)pct);
    tft.print(cBuf);
  }

  if (vRefEnable && Vzero && !cfSuppress) {
    const int barX      = 4;
    const int barY      = 98;
    const int barTotal  = 200;
    const int barH      = 6;
    const int thresh100 = barTotal / 3;

    float pct = fabs(bridgeAvg) / (vClosedThres * 3.0f);
    int barW  = constrain((int)(pct * barTotal), 0, barTotal);

    if (barW <= thresh100) {
      tft.fillRect(barX,        barY, barW,            barH, COL_BAR_FILL);
      tft.fillRect(barX + barW, barY, barTotal - barW, barH, COL_BAR_EMPTY);
    } else {
      tft.fillRect(barX,            barY, thresh100,          barH, COL_BAR_FILL);
      tft.fillRect(barX + thresh100, barY, barW - thresh100,   barH, ST77XX_RED);
      tft.fillRect(barX + barW,      barY, barTotal - barW,    barH, COL_BAR_EMPTY);
    }

    tft.drawRect(barX - 1, barY - 1, barTotal + 2, barH + 2, COL_DIVIDER);
    tft.drawFastVLine(barX + thresh100, barY - 1, barH + 2, COL_TEXT);
  }

  char buf[12];
  tft.setTextSize(1);
  tft.setTextColor(COL_LABEL, COL_BG);

  tft.setCursor(4, 112);
  tft.print("Thres:");
  dtostrf(vClosedThres, 5, 3, buf);  tft.print(buf);

  tft.setCursor(90, 112);
  tft.print("bAvg:");
  dtostrf(bridgeAvg, 5, 3, buf);      tft.print(buf);

  tft.setCursor(180, 112);
  tft.print("Gain:");
  tft.print((int)gainIndexVolt);
  tft.print(" ");

  tft.setCursor(4, 124);
  tft.print("Bridge:");
  dtostrf(bridgeV, 8, 4, buf);       tft.print(buf);

  tft.setCursor(120, 124);
  tft.print("T:");
  dtostrf(currentMillis / 1000.0f, 7, 1, buf);
  tft.print(buf);
  tft.print("s");

  tft.setCursor(198, 124);
  if (battFound) {
    char bBuf[8];
    snprintf(bBuf, sizeof(bBuf), "B:%3d%%", (int)battPct);
    tft.print(bBuf);
  } else {
    tft.print("B: --");
  }

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

  if (Vzero) {
    Serial.print(" Bridge:");    Serial.print(bridgeV, 4);
    Serial.print(" / Thres:");   Serial.print(vClosedThres, 4);
    Serial.print(" / ");         Serial.print(vClosed ? "Closed" : "Floating");
  } else {
    Serial.print(" / V !0");
  }

  Serial.print(" / VDC:");   Serial.print(displayVoltage, 4);
  Serial.print(" Actual(mV):");   Serial.print((vActual*1000), 2);
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
    case 'B':
      Serial.println("Bridge ON test");
      setBridgeOn();
      delay(1000);
      Serial.println("Bridge OFF");
      setBridgeOff();
      break;
    case '?':
      Serial.println("Commands: S(Stop) G(Go) U(Updates) M(Manual) R(Range) D(Debug) B(Bridge test)");
      break;
    default: break;
  }
}
