/*
  DataLogger_TFT.ino
  DataLogger for Adafruit Feather RP2350 + Adafruit 2.4" TFT FeatherWing v2 (#3315).

  All settings that were previously controlled by DIP switches and a potentiometer
  are now handled via touchscreen GUI elements.  SD card logging uses the
  FeatherWing's built-in microSD slot.

  Full compatibility with the DataLogger Android app remote-control protocol is
  maintained.  Sending "!REMOTE" puts the logger into remote mode; "!LOCAL" returns
  control to the touchscreen.

  ── Libraries required ──────────────────────────────────────────────────────────
  Adafruit ILI9341         (TFT driver)
  Adafruit TSC2007         (I2C resistive touchscreen — v2 FeatherWing)
  Adafruit GFX Library     (graphics primitives)
  SdFat                    (SD card — Adafruit fork recommended)
  Adafruit ADS1X15         (ADS1015 12-bit ADC)
  RTClib                   (PCF8523 RTC)
  Adafruit ADXL343         (accelerometer)
  Adafruit NeoPixel        (status LED)

  ── Pin assignments (Feather RP2350 + 2.4" TFT FeatherWing v2) ─────────────────
  TFT CS    →  9   (FeatherWing hardwired)
  TFT DC    → 10   (FeatherWing hardwired)
  TS IRQ    →  6   (TSC2007 touch interrupt, FeatherWing hardwired)
  SD CS     →  5   (FeatherWing built-in microSD slot)
  NeoPixel  → 17   (Feather RP2350 onboard NeoPixel)
  Battery   → A2   (2× 4.7 K divider on custom PCB, restored since A0-A3 are free)
  NTC Temp  → A3   (10 K NTC, restored since A0-A3 are free)
  ADS1015   → I2C 0x48 (ch1) / 0x49 (ch2)
  TSC2007   → I2C — see address note below
  PCF8523   → I2C
  ADXL343   → I2C

  ── ⚠ I2C address conflict ──────────────────────────────────────────────────────
  The TSC2007 defaults to I2C address 0x48, which collides with ADS1015 #1.
  You MUST resolve this before use.  Two options:

  Option A (recommended): Change ADS1015 #1 ADDR pin from GND to VDD → address
    becomes 0x49, but that collides with ADS1015 #2.  So instead wire ADDR to SDA
    → 0x4A, and update ads.begin(0x4A) below.

  Option B: Change the TSC2007 address on the FeatherWing by soldering the A0/A1
    jumpers on the back of the board.  A0=1 → 0x49, A1=1 → 0x4A, both → 0x4B.
    Update TS_I2C_ADDR below to match.

  This sketch uses Option B default: TSC2007 at 0x4B (A0+A1 bridged),
  ADS1015 #1 at 0x48, ADS1015 #2 at 0x49.  Adjust if your hardware differs.

  ── Touch calibration ───────────────────────────────────────────────────────────
  TSC2007 returns raw 12-bit ADC values (0–4095).  Adjust TS_MINX/MAXX/MINY/MAXY
  if touch registration is off.  Orientation is landscape (setRotation(1)).

  ── Commands (host → device) ────────────────────────────────────────────────────
  !REMOTE           Enter remote control mode (Android app takes over)
  !LOCAL            Return to local touchscreen control
  !STATUS           Request immediate $STATUS + $LOG reply
  !SD,0/1           SD logging enable / disable
  !CH2,0/1          Channel-2 voltage enable / disable
  !VOLT,0/1         Voltage channel 1 enable / disable
  !ACCEL,0/1        Accelerometer enable / disable
  !CUR,0/1          Current channel enable / disable
  !AUTO,0/1         Autorange enable / disable
  !SLOW,0/1         Slow-log (60 s) / fast-log (1 s) interval
  !SCREEN,0/1       (accepted for app compat; no-op — TFT is always on)
  !VTHRES,<float>   Voltage trigger threshold (V)
  !ITHRES,<float>   Current trigger threshold (A)
  !ATHRES,<float>   Accelerometer trigger threshold (g)
  !SHUNT,<float>    Shunt resistor value (Ohms)
  !MARK             Insert a manual mark in the next log flush

  ── Replies (device → host) ─────────────────────────────────────────────────────
  $SDSTATUS,0/1
  $STATUS,MODE,SD,CH2,VOLT,ACCEL,CUR,AUTO,SLOW,SCREEN,VTHRES,ITHRES,ATHRES,SHUNTR,VSCALE
  $LOG,V01,V23,I,AX,AY,AZ,AXPK,AYPK,AZPK,TEMP,BATT,LOGS
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_TSC2007.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>
#include <RTClib.h>
#include "SdFat.h"

// ─── Display / touch / SD pins (FeatherWing v2 hardwired) ───────────────────
#define TFT_CS    9
#define TFT_DC   10
#define TS_IRQ    6
#define SD_CS     5

// ─── TSC2007 I2C address ─────────────────────────────────────────────────────
// Default is 0x48.  ⚠ This collides with ADS1015 #1 (also 0x48 when ADDR=GND).
// If you have an address conflict, move ADS1015 #1 ADDR pin to VDD/SDA/SCL and
// update ads.begin() below accordingly.
#define TS_I2C_ADDR  0x4B

// ─── Uncomment to print raw + mapped touch coords to Serial for calibration ──
// #define TOUCH_DEBUG

// ─── Touch calibration (landscape rotation 1) ───────────────────────────────
// In landscape rotation=1 the sensor axes are SWAPPED vs portrait:
//   rawY → screenX (horizontal 0-319)
//   rawX → screenY (vertical   0-239, inverted: high rawX = top)
//
// Calibration measured at corners:
//   Top-left  (screen   0,  0): rawX=3636, rawY=578
//   Bot-right (screen 319,239): rawX=492,  rawY=3320
#define TS_RAW_Y_LEFT    578   // rawY at screen left  edge (screenX=0)
#define TS_RAW_Y_RIGHT  3320   // rawY at screen right edge (screenX=DISP_W)
#define TS_RAW_X_TOP    3636   // rawX at screen top   edge (screenY=0)
#define TS_RAW_X_BOT     492   // rawX at screen bot   edge (screenY=DISP_H)
// Minimum pressure (Z) to register a touch — match example (uses 10)
#define TS_MIN_Z  10

// ─── Other pins ──────────────────────────────────────────────────────────────
#define PIXEL_PIN  17
#define BATT_PIN   A2
#define TEMP_PIN   A3

// ─── Display geometry (landscape 320×240) ────────────────────────────────────
#define DISP_W   320
#define DISP_H   240
#define HDR_H     28
#define TAB_H     28
#define CONTENT_Y HDR_H
#define CONTENT_H (DISP_H - HDR_H - TAB_H)

// ─── Colour palette (RGB565) ─────────────────────────────────────────────────
#define COL_BG       0x1082
#define COL_HDR      0x2124
#define COL_GREEN    0x07E0
#define COL_RED      0xF800
#define COL_YELLOW   0xFFE0
#define COL_ORANGE   0xFC00
#define COL_WHITE    0xFFFF
#define COL_LTGRAY   0xC618
#define COL_DKGRAY   0x4208
#define COL_BLUE     0x021F
#define COL_CYAN     0x07FF
#define COL_BTN_ON   0x0440
#define COL_BTN_OFF  0x2945

// ─── Pages ───────────────────────────────────────────────────────────────────
#define PAGE_DATA     0
#define PAGE_SETTINGS 1

// ─── Data page fixed row positions (px from top, landscape 320×240) ───────────
// CONTENT_Y=28, CONTENT_H=184  → rows must end before Y=212
#define DP_V1_Y      32   // size-3 row  (24 px tall) ends at 56
#define DP_V2_Y      57   // size-2 row  (16 px tall) ends at 73
#define DP_I_Y       74   // size-2 row  (16 px tall) ends at 90
#define DP_AX_Y      92   // size-1 accel live        ends at 100
#define DP_AP_Y     102   // size-1 accel peak        ends at 110
#define DP_ST1_Y    114   // size-1 status row 1      ends at 122
#define DP_ST2_Y    124   // size-1 status row 2      ends at 132
#define DP_PIL_Y    137   // pills (14 px tall)       ends at 151

// ─── Peripheral objects ──────────────────────────────────────────────────────
Adafruit_ILI9341  tft(TFT_CS, TFT_DC);
Adafruit_TSC2007  ts;
Adafruit_ADS1015  ads;
Adafruit_ADS1015  ads2;
Adafruit_ADXL343  accel(12345);
Adafruit_NeoPixel pixel(1, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
RTC_PCF8523       rtc;
SdFat             SD;
File32            logfile;

SdSpiConfig sdCfg(SD_CS, SHARED_SPI, SD_SCK_MHZ(12));
char filename[40];

// ─── Hardware availability ────────────────────────────────────────────────────
bool sdHwOK    = false;
bool accelHwOK = false;
bool tsOK      = false;

// ─── UI state (needed by handleCommand before display functions are defined) ──
bool    needFullRedraw    = true;
uint8_t currentPage       = PAGE_DATA;
uint8_t selectedThreshold = 0;

// ─── ADS gain tables ──────────────────────────────────────────────────────────
static const adsGain_t kGainLevels[] = {
  GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN
};
static const float kGainFactors[] = { 3.0f, 2.0f, 1.0f, 0.5f, 0.25f, 0.125f };
static const int kNumGainLevels = 6;

static size_t gainIndexVolt        = 2;
static size_t gainIndexCurrent     = 5;
static adxl34x_range_t currentAccelRange = ADXL343_RANGE_4_G;
static size_t thresholdSuggestedGainVolt = 255;
static const int ADC_COUNT_LOW_THRESHOLD  = 250;
static const int ADC_COUNT_HIGH_THRESHOLD = 1800;

// ─── Measurement state ────────────────────────────────────────────────────────
float multiplier       = 1.0f;
int16_t ads0_results23 = 0, ads1_results23 = 0, results23 = 0;
float voltage01        = 0.0f, voltage01actual = 0.0f, voltage23 = 0.0f;
float vScale           = 14.319f;
float shuntR           = 0.1f;
float current          = 0.0f;
int   tempF            = 0;
float battV            = 0.0f;

bool  ch2on     = false;
bool  logON     = false;
bool  manualLog = false;
bool  logError  = false;
float logInterval = 1000.0f;

float vStep     = 0.25f;
float iStep     = 0.10f;
float accelThres = 4.0f;
float noiseThreshold = 0.5f;
float lastLoggedV01 = 0.25f, vMax01 = 0.0f, vMin01 = 0.0f;
float lastLoggedI   = 0.25f, iMax   = 0.0f, iMin   = 0.0f;
float lastLoggedV23 = 0.25f, vMax23 = 0.0f, vMin23 = 0.0f;

bool writeTrigger = false, trigFlag = false, triggerPrev = false;
float prevTriggerT = 0.0f, deltaT = 0.0f, triggerT = 0.0f, timemS = 0.0f;
int   logCount = 0;

bool firstVoltRunAuto = true, firstVoltRunMan = true, autorange = true;

// ─── Accelerometer ───────────────────────────────────────────────────────────
float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
float accelXMax = 0.0f, accelYMax = 0.0f, accelZMax = 0.0f;
float accelLastLoggedX = 0.0f, accelLastLoggedY = 0.0f, accelLastLoggedZ = 0.0f;
int   accelCount = 0, accelZMaxcount = 0;

// ─── RAM sample buffer ────────────────────────────────────────────────────────
const size_t MAX_SAMPLES = 2000;
struct Sample { float d1, d2, d3, d4, d5, d6, td, t; };
Sample buffer[MAX_SAMPLES];
volatile size_t sampleCount = 0;

// ─── Timing ──────────────────────────────────────────────────────────────────
unsigned long lastLog           = 0;
unsigned long lastLogSend       = 0;
unsigned long lastStatusSend    = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastTouchCheck    = 0;

// ═══════════════════════════════════════════════════════════════════════════════
//  LOCAL TOUCHSCREEN CONTROL STATE
//  Replaces DIP switches and potentiometer from DataLogger_Remote.
// ═══════════════════════════════════════════════════════════════════════════════

bool localSD        = true;
bool localCH2       = false;
bool localVolt      = true;
bool localAccel     = false;
bool localCurrent   = false;
bool localAutorange = true;
bool localSlowLog   = false;
bool localMark      = false;

// ─── Remote control state ────────────────────────────────────────────────────
bool  remoteMode      = false;
bool  remoteSD        = true;
bool  remoteCH2       = false;
bool  remoteVolt      = true;
bool  remoteAccel     = false;
bool  remoteCurrent   = false;
bool  remoteAutorange = true;
bool  remoteSlowLog   = false;
bool  remoteMark      = false;
float remoteVThres    = 0.25f;
float remoteIThres    = 0.10f;
float remoteAThres    = 4.00f;

String serialInputBuf = "";

// ─── Effective-setting getters ────────────────────────────────────────────────
inline bool effSD()        { return remoteMode ? remoteSD        : localSD;        }
inline bool effCH2()       { return remoteMode ? remoteCH2       : localCH2;       }
inline bool effVolt()      { return remoteMode ? remoteVolt      : localVolt;      }
inline bool effAccelEn()   { return remoteMode ? remoteAccel     : localAccel;     }
inline bool effCurrent()   { return remoteMode ? remoteCurrent   : localCurrent;   }
inline bool effAutorange() { return remoteMode ? remoteAutorange : localAutorange; }
inline bool effSlowLog()   { return remoteMode ? remoteSlowLog   : localSlowLog;   }
inline bool effScreen()    { return true; }

inline bool effMark() {
  if (remoteMode) { bool m = remoteMark; remoteMark = false; return m; }
  bool m = localMark; localMark = false; return m;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SERIAL PROTOCOL  (identical to DataLogger_Remote for Android app compat)
// ═══════════════════════════════════════════════════════════════════════════════

void sendStatus() {
  Serial.print(F("$SDSTATUS,")); Serial.println(sdHwOK ? "1" : "0");
  Serial.print(F("$STATUS,"));
  Serial.print(remoteMode ? "1" : "0");
  Serial.print(','); Serial.print(effSD()        ? "1" : "0");
  Serial.print(','); Serial.print(effCH2()       ? "1" : "0");
  Serial.print(','); Serial.print(effVolt()      ? "1" : "0");
  Serial.print(','); Serial.print(effAccelEn()   ? "1" : "0");
  Serial.print(','); Serial.print(effCurrent()   ? "1" : "0");
  Serial.print(','); Serial.print(effAutorange() ? "1" : "0");
  Serial.print(','); Serial.print(effSlowLog()   ? "1" : "0");
  Serial.print(','); Serial.print(effScreen()    ? "1" : "0");
  Serial.print(','); Serial.print(vStep,    3);
  Serial.print(','); Serial.print(iStep,    3);
  Serial.print(','); Serial.print(accelThres, 2);
  Serial.print(','); Serial.print(shuntR,    4);
  Serial.print(','); Serial.println(vScale,  4);
}

void sendLog() {
  Serial.print(F("$LOG,"));
  Serial.print(voltage01, 3);
  Serial.print(','); Serial.print(voltage23, 3);
  Serial.print(','); Serial.print(current, 4);
  Serial.print(','); Serial.print(accelX, 2);
  Serial.print(','); Serial.print(accelY, 2);
  Serial.print(','); Serial.print(accelZ, 2);
  Serial.print(','); Serial.print(accelXMax, 2);
  Serial.print(','); Serial.print(accelYMax, 2);
  Serial.print(','); Serial.print(accelZMax, 2);
  Serial.print(','); Serial.print(tempF);
  Serial.print(','); Serial.print(battV, 2);
  Serial.print(','); Serial.println(logCount);
}

void handleCommand(const String& cmd) {
  if (cmd == F("!REMOTE")) {
    remoteMode      = true;
    remoteSD        = localSD;        remoteCH2       = localCH2;
    remoteVolt      = localVolt;      remoteAccel     = localAccel;
    remoteCurrent   = localCurrent;   remoteAutorange = localAutorange;
    remoteSlowLog   = localSlowLog;
    remoteVThres    = vStep;          remoteIThres    = iStep;
    remoteAThres    = accelThres;
    Serial.println(F("$MODE,REMOTE")); sendStatus();
    needFullRedraw = true; return;
  }
  if (cmd == F("!LOCAL"))  { remoteMode = false; Serial.println(F("$MODE,LOCAL")); sendStatus(); needFullRedraw = true; return; }
  if (cmd == F("!STATUS")) { sendStatus(); sendLog(); return; }
  if (cmd == F("!MARK"))   { if (remoteMode) remoteMark = true; return; }

  if      (cmd.startsWith(F("!SD,")))     { remoteSD        = (bool)cmd.substring(4).toInt();  sendStatus(); }
  else if (cmd.startsWith(F("!CH2,")))    { remoteCH2       = (bool)cmd.substring(5).toInt();  if (remoteMode) ch2on = remoteCH2; sendStatus(); }
  else if (cmd.startsWith(F("!VOLT,")))   { remoteVolt      = (bool)cmd.substring(6).toInt();  sendStatus(); }
  else if (cmd.startsWith(F("!ACCEL,")))  { remoteAccel     = (bool)cmd.substring(7).toInt();  sendStatus(); }
  else if (cmd.startsWith(F("!CUR,")))    { remoteCurrent   = (bool)cmd.substring(5).toInt();  sendStatus(); }
  else if (cmd.startsWith(F("!AUTO,")))   { remoteAutorange = (bool)cmd.substring(6).toInt();  firstVoltRunAuto = true; firstVoltRunMan = true; sendStatus(); }
  else if (cmd.startsWith(F("!SLOW,")))   { remoteSlowLog   = (bool)cmd.substring(6).toInt();  sendStatus(); }
  else if (cmd.startsWith(F("!SCREEN,"))) { /* no-op */                                          sendStatus(); }
  else if (cmd.startsWith(F("!VTHRES,"))) { remoteVThres = cmd.substring(8).toFloat(); vStep = remoteVThres; thresholdSuggestedGainVolt = 255; sendStatus(); }
  else if (cmd.startsWith(F("!ITHRES,"))) { remoteIThres = cmd.substring(8).toFloat(); iStep = remoteIThres; sendStatus(); }
  else if (cmd.startsWith(F("!ATHRES,"))) { remoteAThres = cmd.substring(8).toFloat(); accelThres = remoteAThres; sendStatus(); }
  else if (cmd.startsWith(F("!SHUNT,")))  { float v = cmd.substring(7).toFloat(); if (v >= 0.0001f) shuntR = v; sendStatus(); }
}

void processSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      String cmd = serialInputBuf; cmd.trim(); serialInputBuf = "";
      if (cmd.length() > 0) handleCommand(cmd);
    } else {
      if (serialInputBuf.length() < 64) serialInputBuf += c;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MEASUREMENT HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

float readNTCTemperatureC(uint8_t pin) {
  uint32_t adcRaw = analogRead(pin);
  float vNTC = (adcRaw * 3.3f) / 4095.0f;
  if (vNTC <= 0.0f || vNTC >= 3.3f) return NAN;
  float rNTC = 10000.0f * (vNTC / (3.3f - vNTC));
  float t0K  = 298.15f;  // 25°C in Kelvin
  float invT = (1.0f / t0K) + (1.0f / 3950.0f) * log(rNTC / 10000.0f);
  return (1.0f / invT) - 273.15f;
}

int16_t readRaw23()     { return ads.readADC_Differential_2_3();  }
int16_t ads2ReadRaw23() { return ads2.readADC_Differential_2_3(); }

void updateGainsFromThresholds() {
  if (effVolt() && autorange) {
    size_t sg;
    if      (vStep > 6.0f)   sg = 0;
    else if (vStep > 3.0f)   sg = 1;
    else if (vStep > 1.5f)   sg = 2;
    else if (vStep > 0.75f)  sg = 3;
    else if (vStep > 0.375f) sg = 4;
    else                      sg = 5;
    if (sg != thresholdSuggestedGainVolt) {
      thresholdSuggestedGainVolt = sg; gainIndexVolt = sg; firstVoltRunAuto = true;
    }
  }
  if (effCurrent()) {
    size_t ng;
    if      (iStep > 1.0f)  ng = 0; else if (iStep > 0.5f)  ng = 1;
    else if (iStep > 0.25f) ng = 2; else if (iStep > 0.1f)  ng = 3;
    else                     ng = 4;
    if (ng != gainIndexCurrent) {
      gainIndexCurrent = ng; ads2.setGain(kGainLevels[gainIndexCurrent]);
      (void)ads2ReadRaw23(); ads2.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, true);
    }
  }
  if (effAccelEn() && accelHwOK) {
    adxl34x_range_t nr;
    if      (accelThres > 8.0f) nr = ADXL343_RANGE_16_G;
    else if (accelThres > 4.0f) nr = ADXL343_RANGE_8_G;
    else if (accelThres > 2.0f) nr = ADXL343_RANGE_4_G;
    else                         nr = ADXL343_RANGE_2_G;
    if (nr != currentAccelRange) { currentAccelRange = nr; accel.setRange(nr); }
  }
}

void measureVoltage() {
  if (ch2on) {
    ads0_results23  = ads.readADC_Differential_2_3();
    results23       = ads.readADC_Differential_0_1();
    voltage01actual = (ads0_results23 * multiplier) / 1000.0f;
    voltage01       = voltage01actual * vScale;
    voltage23       = ((results23 * multiplier) / 1000.0f) * vScale;
    return;
  }
  if (autorange) {
    if (firstVoltRunAuto) {
      gainIndexVolt = (thresholdSuggestedGainVolt < (size_t)kNumGainLevels)
                        ? thresholdSuggestedGainVolt : (kNumGainLevels - 1);
      firstVoltRunAuto = false; firstVoltRunMan = true;
      ads.setGain(kGainLevels[gainIndexVolt]); (void)readRaw23();
    }
    ads0_results23 = ads.getLastConversionResults();
    ads.setGain(kGainLevels[gainIndexVolt]);
    ads0_results23 = readRaw23();
    if (abs(ads0_results23) > ADC_COUNT_HIGH_THRESHOLD && gainIndexVolt > 0) {
      --gainIndexVolt; ads.setGain(kGainLevels[gainIndexVolt]); (void)readRaw23(); return;
    }
    if (abs(ads0_results23) < ADC_COUNT_LOW_THRESHOLD && gainIndexVolt < (size_t)(kNumGainLevels - 1)) {
      ++gainIndexVolt; ads.setGain(kGainLevels[gainIndexVolt]); (void)readRaw23(); return;
    }
    voltage01actual = (ads0_results23 * kGainFactors[gainIndexVolt]) / 1000.0f;
    voltage01       = voltage01actual * vScale;
  } else {
    if (firstVoltRunMan) {
      firstVoltRunMan = false; firstVoltRunAuto = true;
      ads.setGain(GAIN_ONE); ads2.setGain(kGainLevels[gainIndexCurrent]);
      (void)readRaw23(); (void)ads2ReadRaw23();
      ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, true);
      ads2.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, true);
    }
    multiplier      = 2.0f;
    ads0_results23  = ads.getLastConversionResults();
    voltage01actual = (ads0_results23 * multiplier) / 1000.0f;
    voltage01       = voltage01actual * vScale;
  }
}

void readAccel() {
  sensors_event_t event; accel.getEvent(&event);
  accelX = event.acceleration.x; accelY = event.acceleration.y; accelZ = event.acceleration.z;
  accelCount++;
  if (abs(accelX) - accelThres > abs(accelLastLoggedX) || abs(accelX) + accelThres < abs(accelLastLoggedX)) { accelXMax = accelX; trigFlag = true; }
  if (abs(accelY) - accelThres > abs(accelLastLoggedY) || abs(accelY) + accelThres < abs(accelLastLoggedY)) { accelYMax = accelY; trigFlag = true; }
  if (abs(accelZ) - accelThres > abs(accelLastLoggedZ) || abs(accelZ) + accelThres < abs(accelLastLoggedZ)) { accelZMax = accelZ; accelZMaxcount++; trigFlag = true; }
}

// ─── SD helpers ──────────────────────────────────────────────────────────────
void captureSample(float d1, float d2, float d3, float d4, float d5, float d6) {
  if (sampleCount >= MAX_SAMPLES) return;
  prevTriggerT = triggerT; triggerT = micros(); timemS = millis(); deltaT = triggerT - prevTriggerT;
  buffer[sampleCount] = { d1, d2, d3, d4, d5, d6, deltaT, timemS + (triggerT / 1000) };
  sampleCount++;
}

void flushBufferToSD() {
  logfile = SD.open(filename, FILE_WRITE);
  if (!logfile) { Serial.println(F("SD open failed")); logError = true; return; }
  logError = false;
  DateTime now = rtc.now();
  for (size_t i = 0; i < sampleCount; i++) {
    logfile.print(buffer[i].td, 1);  logfile.print(',');
    logfile.print(buffer[i].d1, 3);  logfile.print(',');
    logfile.print(buffer[i].d2, 1);  logfile.print(',');
    logfile.print(buffer[i].d3, 4);  logfile.print(',');
    logfile.print(buffer[i].d4, 2);  logfile.print(',');
    logfile.print(buffer[i].d5, 2);  logfile.print(',');
    logfile.print(buffer[i].d6, 2);  logfile.print(',');
    logfile.print(tempF);             logfile.print(',');
    if (manualLog) { logfile.print(now.timestamp()); logfile.println(F(",MARK")); }
    else           { logfile.println(now.timestamp()); }
  }
  logfile.flush(); logfile.close();
  pixel.setPixelColor(0, pixel.Color(0, 255, 0)); pixel.show(); delay(5);
  pixel.clear(); pixel.show();
  sampleCount = 0;
}

void blinkPixel(uint8_t r, uint8_t g, uint8_t b, int dur = 5) {
  pixel.setPixelColor(0, pixel.Color(r, g, b)); pixel.show(); delay(dur);
  pixel.clear(); pixel.show();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DISPLAY — DRAWING PRIMITIVES
// ═══════════════════════════════════════════════════════════════════════════════

void drawToggleBtn(int16_t x, int16_t y, int16_t w, int16_t h,
                   const char* label, bool state, bool highlight = false) {
  uint16_t fill   = highlight ? 0x3200 : (state ? COL_BTN_ON  : COL_BTN_OFF);
  uint16_t border = highlight ? COL_YELLOW : (state ? COL_GREEN   : COL_DKGRAY);
  uint16_t text   = highlight ? COL_YELLOW : (state ? COL_GREEN   : COL_LTGRAY);
  tft.fillRoundRect(x, y, w, h, 4, fill);
  tft.drawRoundRect(x, y, w, h, 4, border);
  tft.setTextColor(text);
  tft.setTextSize(1);
  tft.setCursor(x + (w - (int16_t)strlen(label) * 6) / 2, y + (h - 8) / 2);
  tft.print(label);
}

void drawThreshRow(int16_t y, const char* label, float value, const char* unit, bool selected) {
  uint16_t bc = selected ? COL_YELLOW : COL_DKGRAY;
  tft.setTextColor(selected ? COL_YELLOW : COL_LTGRAY, COL_BG);
  tft.setTextSize(1);
  tft.setCursor(4, y + 5); tft.print(label); tft.print(F("          ")); // clear stale text

  tft.fillRoundRect(100, y, 24, 20, 3, COL_BTN_OFF);
  tft.drawRoundRect(100, y, 24, 20, 3, bc);
  tft.setTextColor(bc); tft.setCursor(109, y + 6); tft.print('-');

  char buf[12]; dtostrf(value, 7, 3, buf);
  tft.setTextColor(selected ? COL_WHITE : COL_LTGRAY, COL_BG);
  tft.setCursor(128, y + 6);
  tft.print(buf); tft.print(' '); tft.print(unit); tft.print(F("   ")); // clear old unit

  tft.fillRoundRect(192, y, 24, 20, 3, COL_BTN_OFF);
  tft.drawRoundRect(192, y, 24, 20, 3, bc);
  tft.setTextColor(bc); tft.setCursor(200, y + 6); tft.print('+');
}

// drawHeaderFrame: static parts — fill + title + MARK button. Call once per full redraw.
void drawHeaderFrame() {
  tft.fillRect(0, 0, DISP_W, HDR_H, COL_HDR);
  tft.setTextColor(COL_WHITE, COL_HDR);
  tft.setTextSize(1);
  tft.setCursor(90, 10); tft.print(F("DataLogger TFT"));
  drawToggleBtn(282, 4, 34, 20, "MARK", false);
}

// updateHeaderDynamics: overdraw only the two changing fields (mode badge, SD status).
// Uses setTextColor(fg, COL_HDR) — no fillRect needed, no blank-frame flicker.
void updateHeaderDynamics() {
  tft.setTextSize(1);
  if (remoteMode) { tft.setTextColor(COL_ORANGE, COL_HDR); tft.setCursor(4, 10); tft.print(F("REMOTE")); }
  else            { tft.setTextColor(COL_GREEN,  COL_HDR); tft.setCursor(4, 10); tft.print(F("LOCAL ")); }
  tft.setCursor(234, 10);
  if      (!sdHwOK)  { tft.setTextColor(COL_RED,    COL_HDR); tft.print(F("SD:N/A")); }
  else if (logError) { tft.setTextColor(COL_RED,    COL_HDR); tft.print(F("SD:ERR")); }
  else if (logON)    { tft.setTextColor(COL_GREEN,  COL_HDR); tft.print(F("SD:LOG")); }
  else               { tft.setTextColor(COL_DKGRAY, COL_HDR); tft.print(F("SD:OFF")); }
}

void drawHeader() { drawHeaderFrame(); updateHeaderDynamics(); }

void drawTabBar() {
  tft.fillRect(0, DISP_H - TAB_H, DISP_W, TAB_H, COL_HDR);
  drawToggleBtn(2,   DISP_H - TAB_H + 2, 154, TAB_H - 4, "DATA",     currentPage == PAGE_DATA);
  drawToggleBtn(162, DISP_H - TAB_H + 2, 156, TAB_H - 4, "SETTINGS", currentPage == PAGE_SETTINGS);
}

// drawDataPageFrame: initial fill — call once on page entry (or full redraw).
void drawDataPageFrame() {
  tft.fillRect(0, CONTENT_Y, DISP_W, CONTENT_H, COL_BG);
}

// updateDataDynamics: overdraw all changing values using setTextColor(fg, COL_BG).
// No fillRect of the whole content area → zero blank-frame flicker.
void updateDataDynamics() {
  char buf[16];

  // ── V1 (size 3, fixed-width) ────────────────────────────────────────────────
  tft.setTextSize(3);
  tft.setTextColor(effVolt() ? COL_GREEN : COL_DKGRAY, COL_BG);
  tft.setCursor(4, DP_V1_Y);
  tft.print(F("V1:"));
  dtostrf(voltage01, 7, 3, buf); tft.print(buf); tft.print(F("V "));

  // ── V2 (size 2) ─────────────────────────────────────────────────────────────
  tft.setTextSize(2);
  tft.setTextColor(effCH2() ? COL_CYAN : COL_DKGRAY, COL_BG);
  tft.setCursor(4, DP_V2_Y);
  tft.print(F("V2:"));
  if (effCH2()) { dtostrf(voltage23, 7, 3, buf); tft.print(buf); tft.print(F("V       ")); }
  else          { tft.print(F("        (off)  ")); }

  // ── Current (size 2) ────────────────────────────────────────────────────────
  tft.setTextColor(effCurrent() ? COL_YELLOW : COL_DKGRAY, COL_BG);
  tft.setCursor(4, DP_I_Y);
  tft.print(F("I: "));
  if (effCurrent()) { dtostrf(current * 1000.0f, 7, 1, buf); tft.print(buf); tft.print(F("mA ")); }
  else              { tft.print(F("        (off)  ")); }

  // ── Accelerometer (size 1) ──────────────────────────────────────────────────
  tft.setTextSize(1);
  tft.setTextColor((effAccelEn() && accelHwOK) ? COL_CYAN : COL_DKGRAY, COL_BG);
  tft.setCursor(4, DP_AX_Y);
  tft.print(F("Accel X:"));
  dtostrf(accelX, 6, 2, buf); tft.print(buf);
  tft.print(F(" Y:")); dtostrf(accelY, 6, 2, buf); tft.print(buf);
  tft.print(F(" Z:")); dtostrf(accelZ, 6, 2, buf); tft.print(buf); tft.print(F("g "));

  tft.setTextColor(COL_DKGRAY, COL_BG);
  tft.setCursor(4, DP_AP_Y);
  tft.print(F("Peak  X:"));
  dtostrf(accelXMax, 6, 2, buf); tft.print(buf);
  tft.print(F(" Y:")); dtostrf(accelYMax, 6, 2, buf); tft.print(buf);
  tft.print(F(" Z:")); dtostrf(accelZMax, 6, 2, buf); tft.print(buf); tft.print(F("  "));

  // ── Status row 1: Temp / Batt / Logs ────────────────────────────────────────
  tft.setTextColor(COL_LTGRAY, COL_BG);
  tft.setCursor(4, DP_ST1_Y);
  tft.print(F("Temp:")); tft.print(tempF); tft.print(F("F  "));
  tft.print(F("Batt:")); dtostrf(battV, 4, 1, buf); tft.print(buf); tft.print(F("V  "));
  tft.print(F("Logs:")); tft.print(logCount); tft.print(F("    "));

  // ── Status row 2: Runtime / autorange / vThreshold ──────────────────────────
  tft.setCursor(4, DP_ST2_Y);
  tft.print(F("Runtime:")); tft.print(millis() / 1000UL); tft.print(F("s   "));
  tft.print(effAutorange() ? F("Auto ") : F("Fixed"));
  tft.print(F("  vT:")); dtostrf(vStep, 5, 2, buf); tft.print(buf); tft.print(' ');

  // ── Channel status pills (small clear + redraw) ──────────────────────────────
  tft.fillRect(0, DP_PIL_Y, DISP_W, 16, COL_BG);
  const char* labels[] = {"SD","CH2","VOLT","ACCEL","CUR","AUTO","SLOW"};
  bool states[] = { effSD(), effCH2(), effVolt(), effAccelEn(), effCurrent(), effAutorange(), effSlowLog() };
  int16_t bx = 4;
  for (int i = 0; i < 7; i++) {
    uint16_t bg = states[i] ? COL_BTN_ON : COL_BTN_OFF;
    uint16_t fg = states[i] ? COL_GREEN  : COL_DKGRAY;
    int16_t  bw = (int16_t)strlen(labels[i]) * 6 + 6;
    tft.fillRoundRect(bx, DP_PIL_Y, bw, 14, 3, bg);
    tft.drawRoundRect(bx, DP_PIL_Y, bw, 14, 3, fg);
    tft.setTextColor(fg); tft.setCursor(bx + 3, DP_PIL_Y + 3); tft.print(labels[i]);
    bx += bw + 3;
  }
}

void drawDataPage() { drawDataPageFrame(); updateDataDynamics(); }

void drawSettingsPage() {
  tft.fillRect(0, CONTENT_Y, DISP_W, CONTENT_H, COL_BG);

  int16_t row1y = CONTENT_Y + 4, row2y = CONTENT_Y + 32;
  int16_t bw = 72, bh = 24, gap = 4;
  int16_t c0 = 4, c1 = c0+bw+gap, c2 = c1+bw+gap, c3 = c2+bw+gap;

  drawToggleBtn(c0, row1y, bw, bh, "SD",    effSD());
  drawToggleBtn(c1, row1y, bw, bh, "CH2",   effCH2());
  drawToggleBtn(c2, row1y, bw, bh, "VOLT",  effVolt());
  drawToggleBtn(c3, row1y, bw, bh, "ACCEL", effAccelEn());
  drawToggleBtn(c0, row2y, bw, bh, "CUR",   effCurrent());
  drawToggleBtn(c1, row2y, bw, bh, "AUTO",  effAutorange());
  drawToggleBtn(c2, row2y, bw, bh, "SLOW",  effSlowLog());
  drawToggleBtn(c3, row2y, bw, bh, "MARK",  false);

  int16_t divY = CONTENT_Y + 62;
  tft.drawFastHLine(4, divY, DISP_W - 8, COL_DKGRAY);

  int16_t ty = divY + 6;
  drawThreshRow(ty,      "V Threshold", vStep,     "V",   selectedThreshold == 0);
  drawThreshRow(ty + 26, "I Threshold", iStep,     "A",   selectedThreshold == 1);
  drawThreshRow(ty + 52, "A Threshold", accelThres,"g",   selectedThreshold == 2);
  drawThreshRow(ty + 78, "Shunt R",     shuntR,    "Ohm", selectedThreshold == 3);

  if (remoteMode) {
    tft.setTextColor(COL_ORANGE, COL_BG); tft.setTextSize(1);
    tft.setCursor(4, CONTENT_Y + CONTENT_H - 14);
    tft.print(F("REMOTE MODE: settings reflect app state"));
  }
}

void drawFullScreen() {
  tft.fillScreen(COL_BG);
  drawHeaderFrame(); updateHeaderDynamics();
  drawTabBar();
  if (currentPage == PAGE_DATA) drawDataPage();  // drawDataPageFrame + updateDataDynamics
  else                           drawSettingsPage();
  needFullRedraw = false;
}

// Called every 500 ms on the data page — no full clear, no flicker.
void refreshDataArea() {
  updateHeaderDynamics();
  updateDataDynamics();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TOUCH HANDLING  (TSC2007 over I2C)
// ═══════════════════════════════════════════════════════════════════════════════

// TSC2007 raw → screen pixels (landscape rotation 1).
// rawY → screenX  (portrait-Y becomes landscape-X)
// rawX → screenY  inverted: high rawX = top of landscape screen
void mapTouch(uint16_t rawX, uint16_t rawY, int16_t* sx, int16_t* sy) {
  *sx = (int16_t)map((long)rawY, TS_RAW_Y_LEFT, TS_RAW_Y_RIGHT, 0, DISP_W);
  *sy = (int16_t)map((long)rawX, TS_RAW_X_TOP,  TS_RAW_X_BOT,  0, DISP_H);
  *sx = constrain(*sx, 0, DISP_W - 1);
  *sy = constrain(*sy, 0, DISP_H - 1);
}

inline bool inRect(int16_t px, int16_t py, int16_t x, int16_t y, int16_t w, int16_t h) {
  return (px >= x && px < x+w && py >= y && py < y+h);
}

bool  touchActive   = false;
unsigned long touchStartMs = 0;
#define TOUCH_DEBOUNCE_MS 180

void handleTouch(int16_t sx, int16_t sy) {
  // Tab bar
  if (sy >= DISP_H - TAB_H) {
    uint8_t newPage = (sx < 160) ? PAGE_DATA : PAGE_SETTINGS;
    if (newPage != currentPage) { currentPage = newPage; needFullRedraw = true; }
    return;
  }

  // Header MARK button
  if (inRect(sx, sy, 282, 4, 34, 20)) {
    localMark = true; drawToggleBtn(282, 4, 34, 20, "MARK", true); return;
  }

  if (currentPage == PAGE_SETTINGS) {
    int16_t row1y = CONTENT_Y + 4, row2y = CONTENT_Y + 32;
    int16_t bw = 72, bh = 24, gap = 4;
    int16_t c0 = 4, c1 = c0+bw+gap, c2 = c1+bw+gap, c3 = c2+bw+gap;

    // Toggle buttons (local mode only)
    if (!remoteMode) {
      if (inRect(sx, sy, c0, row1y, bw, bh)) { localSD        = !localSD;        needFullRedraw = true; return; }
      if (inRect(sx, sy, c1, row1y, bw, bh)) { localCH2       = !localCH2;       needFullRedraw = true; return; }
      if (inRect(sx, sy, c2, row1y, bw, bh)) { localVolt      = !localVolt;      needFullRedraw = true; return; }
      if (inRect(sx, sy, c3, row1y, bw, bh)) { localAccel     = !localAccel;     needFullRedraw = true; return; }
      if (inRect(sx, sy, c0, row2y, bw, bh)) { localCurrent   = !localCurrent;   needFullRedraw = true; return; }
      if (inRect(sx, sy, c1, row2y, bw, bh)) { localAutorange = !localAutorange; firstVoltRunAuto = true; firstVoltRunMan = true; needFullRedraw = true; return; }
      if (inRect(sx, sy, c2, row2y, bw, bh)) { localSlowLog   = !localSlowLog;   needFullRedraw = true; return; }
    }
    if (inRect(sx, sy, c3, row2y, bw, bh)) {
      localMark = true; drawToggleBtn(c3, row2y, bw, bh, "MARK", true); return;
    }

    // Threshold ± rows
    int16_t divY = CONTENT_Y + 62;
    int16_t ty   = divY + 6;

    struct ThreshRow {
      int16_t ry; uint8_t idx; float* val; float minV; float maxV; float step;
    };
    static const float kVStep[]     = {0.01f, 0.05f, 0.25f, 0.5f, 1.0f};
    static const float kIStep[]     = {0.001f, 0.01f, 0.05f, 0.1f};
    static const float kAStep[]     = {0.25f, 0.5f, 1.0f, 2.0f};
    static const float kShuntStep[] = {0.001f, 0.01f, 0.1f};
    static uint8_t vIdx = 2, iIdx = 1, aIdx = 0, sIdx = 1;

    ThreshRow rows[4] = {
      { ty,                    0, &vStep,     0.005f, 10.0f, kVStep[vIdx]     },
      { (int16_t)(ty + 26),   1, &iStep,     0.001f,  5.0f, kIStep[iIdx]     },
      { (int16_t)(ty + 52),   2, &accelThres,0.25f,  16.0f, kAStep[aIdx]     },
      { (int16_t)(ty + 78),   3, &shuntR,    0.0001f, 1.0f, kShuntStep[sIdx] },
    };

    for (auto& r : rows) {
      if (sy < r.ry || sy >= r.ry + 22) continue;
      selectedThreshold = r.idx;
      if (!remoteMode) {
        if (inRect(sx, sy, 100, r.ry, 24, 20)) { *r.val = max(r.minV, *r.val - r.step); updateGainsFromThresholds(); }
        if (inRect(sx, sy, 192, r.ry, 24, 20)) { *r.val = min(r.maxV, *r.val + r.step); updateGainsFromThresholds(); }
      }
      needFullRedraw = true;
      return;
    }
  }
}

void processTouches() {
  if (!tsOK) return;

  // IRQ HIGH = no touch; clear active flag so next press is treated as new
  if (digitalRead(TS_IRQ)) {
    touchActive = false;
    return;
  }

  // Rate-limit I2C reads
  unsigned long now = millis();
  if (now - lastTouchCheck < 20) return;
  lastTouchCheck = now;

  // Finger is still held from a previously handled tap — wait for release
  if (touchActive) return;

  TS_Point p = ts.getPoint();
  if ((p.x == 0 && p.y == 0) || p.z < TS_MIN_Z) return;

  // Cooldown: ignore spurious re-triggers within TOUCH_DEBOUNCE_MS of last tap
  if (now - touchStartMs < TOUCH_DEBOUNCE_MS) return;

  // Valid new tap — fire once and mark active until IRQ goes HIGH (release)
  touchActive  = true;
  touchStartMs = now;
  int16_t sx, sy;
  mapTouch((uint16_t)p.x, (uint16_t)p.y, &sx, &sy);

#ifdef TOUCH_DEBUG
  Serial.print(F("Touch raw ")); Serial.print(p.x); Serial.print(','); Serial.print(p.y);
  Serial.print(F(" z=")); Serial.print(p.z);
  Serial.print(F(" → screen ")); Serial.print(sx); Serial.print(','); Serial.println(sy);
#endif

  handleTouch(sx, sy);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("DataLogger_TFT starting"));

  Wire.begin();
  Wire.setClock(400000);
  analogReadResolution(12);
  SPI.begin();

  pinMode(TS_IRQ, INPUT);   // TSC2007 IRQ: HIGH=idle, LOW=touched — no pullup

  // ── TFT ──
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_WHITE); tft.setTextSize(2);
  tft.setCursor(20, 80);  tft.print(F("DataLogger TFT"));
  tft.setTextSize(1); tft.setTextColor(COL_LTGRAY);
  tft.setCursor(20, 110); tft.print(F("Feather RP2350 + 2.4\" TFT FeatherWing v2"));
  tft.setCursor(20, 125); tft.print(F("Initialising..."));

  // ── TSC2007 touch ──
  if (ts.begin(TS_I2C_ADDR, &Wire)) {
    tsOK = true;
    Serial.println(F("TSC2007 touch OK"));
  } else {
    Serial.println(F("TSC2007 not found — touch disabled"));
    tft.setTextColor(COL_YELLOW);
    tft.setCursor(20, 140); tft.print(F("Touch not found — check TS_I2C_ADDR"));
  }

  // ── RTC ──
  if (!rtc.begin()) {
    tft.setTextColor(COL_RED); tft.setCursor(20, 155); tft.print(F("RTC not found!"));
    Serial.println(F("RTC not found — halting"));
    while (1) delay(100);
  }
  DateTime now = rtc.now();
  Serial.print(F("RTC: ")); Serial.println(now.timestamp());

  // ── ADS ──
  ads.setDataRate(RATE_ADS1015_3300SPS);
  if (!ads.begin(0x48)) {
    tft.setTextColor(COL_RED); tft.setCursor(20, 155); tft.print(F("ADS1 (0x48) init failed!"));
    Serial.println(F("ADS1 failed — check I2C address conflict with TSC2007"));
    while (1) delay(100);
  }
  if (!ads2.begin(0x49)) {
    tft.setTextColor(COL_RED); tft.setCursor(20, 155); tft.print(F("ADS2 (0x49) init failed!"));
    Serial.println(F("ADS2 failed"));
    while (1) delay(100);
  }
  Serial.println(F("ADS1015 OK"));

  // ── NeoPixel ──
  pixel.begin(); pixel.clear(); pixel.show();

  // ── SD (FeatherWing slot, pin 5) ──
  if (SD.begin(sdCfg)) {
    sprintf(filename, "log_%04d%02d%02d_%02d%02d%02d.csv",
            now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    logfile = SD.open(filename, FILE_WRITE);
    if (logfile) {
      logfile.print(F("Log Start Time,")); logfile.println(now.timestamp());
      logfile.println(F("Delta uS,V Ch1,V Ch2,I,X,Y,Z,Temp,RTC Stamp"));
      logfile.flush(); logfile.close();
      sdHwOK = true;
      Serial.print(F("SD OK: ")); Serial.println(filename);
    } else { Serial.println(F("SD: could not create log file")); }
  } else {
    Serial.println(F("SD init failed"));
    for (int i = 0; i < 5; i++) blinkPixel(128, 0, 0, 100);
  }

  // ── Accelerometer ──
  if (accel.begin()) {
    accelHwOK = true;
    accel.setRange(ADXL343_RANGE_4_G);
    accel.setDataRate(ADXL343_DATARATE_1600_HZ);
    Serial.println(F("ADXL343 OK"));
  } else { Serial.println(F("ADXL343 not found")); }

  // ── Channel 2 default — off ──
  ch2on = false;
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, true);
  ads2.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, true);

  Serial.print(F("$SDSTATUS,")); Serial.println(sdHwOK ? "1" : "0");
  sendStatus();

  needFullRedraw = true;
  drawFullScreen();
  Serial.println(F("DataLogger_TFT ready. Send !REMOTE for Android app control."));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {

  // 1. Serial commands
  processSerial();

  // 2. Touch
  processTouches();

  // 3. Effective settings
  autorange   = effAutorange();
  logInterval = effSlowLog() ? 60000.0f : 1000.0f;
  ch2on       = effCH2();
  logON       = sdHwOK && effSD();

  // 4. Battery (A2 — 2× 4.7K divider)
  {
    uint16_t raw = analogRead(BATT_PIN);
    battV = (raw / 4095.0f) * 3.3f * 2.0f;
  }

  // 5. Temperature (A3 — NTC)
  {
    float tempC = readNTCTemperatureC(TEMP_PIN);
    if (!isnan(tempC)) tempF = (int)((tempC * 9.0f / 5.0f) + 32.0f);
  }

  // 6. Gains
  updateGainsFromThresholds();

  // 7. Mark
  bool markPressed = effMark();
  if (markPressed) { manualLog = true; logCount++; } else { manualLog = false; }

  // 8. Accelerometer
  if (effAccelEn() && accelHwOK) readAccel();
  else { accelX = 0.0f; accelY = 0.0f; accelZ = 0.0f; }

  // 9. Voltage
  if (effVolt()) measureVoltage(); else voltage01 = 0.0f;

  // 10. Current
  if (effCurrent()) {
    ads1_results23 = ads2.getLastConversionResults();
    current = ((ads1_results23 * -kGainFactors[gainIndexCurrent]) / 1000.0f) / shuntR;
  } else { current = 0.0f; }

  // 11. Trigger evaluation
  if (voltage01 > vMax01) { vMax01 = voltage01; trigFlag = true; }
  if (voltage01 < vMin01) { vMin01 = voltage01; trigFlag = true; }
  if (current   > iMax)   { iMax   = current;   trigFlag = true; }
  if (current   < iMin)   { iMin   = current;   trigFlag = true; }
  if (abs(voltage01) > noiseThreshold) {
    if (voltage01 > lastLoggedV01 + vStep) trigFlag = true;
    if (voltage01 < lastLoggedV01 - vStep) trigFlag = true;
  }
  if (abs(current) > 0.025f) {
    if (current > lastLoggedI + iStep) trigFlag = true;
    if (current < lastLoggedI - iStep) trigFlag = true;
  }
  if (ch2on) {
    if (voltage23 > vMax23) { vMax23 = voltage23; trigFlag = true; }
    if (voltage23 < vMin23) { vMin23 = voltage23; trigFlag = true; }
    if (abs(voltage23) > noiseThreshold) {
      if (voltage23 > lastLoggedV23 + 0.1f) trigFlag = true;
      if (voltage23 < lastLoggedV23 - 0.1f) trigFlag = true;
    }
  }

  if (trigFlag) {
    writeTrigger = true;
    lastLoggedV01 = voltage01; lastLoggedV23 = voltage23; lastLoggedI = current;
    accelLastLoggedX = accelX; accelLastLoggedY = accelY; accelLastLoggedZ = accelZ;
    trigFlag = false;
  } else { writeTrigger = false; }

  // 12. Capture / periodic log
  if (writeTrigger && logON) {
    captureSample(voltage01, voltage23, current, accelX, accelY, accelZ);
    logCount++;
  } else {
    if ((millis() - lastLog > (unsigned long)logInterval) || manualLog) {
      if (!manualLog) lastLog = millis();
      captureSample(voltage01, voltage23, current, accelX, accelY, accelZ);
      if (logON) { flushBufferToSD(); logCount++; manualLog = false; }
    }
    writeTrigger = false;
  }

  // 13. Flush on trigger falling edge
  if (((triggerPrev && !writeTrigger) || manualLog) && logON) flushBufferToSD();
  triggerPrev = writeTrigger;

  // 14. Periodic $LOG (~500 ms)
  unsigned long nowMs = millis();
  if (nowMs - lastLogSend >= 500)    { lastLogSend    = nowMs; sendLog(); }

  // 15. Periodic $STATUS (~10 s)
  if (nowMs - lastStatusSend >= 10000) { lastStatusSend = nowMs; sendStatus(); }

  // 16. Display update (~500 ms or on interaction)
  if (needFullRedraw) {
    drawFullScreen();
  } else if (nowMs - lastDisplayUpdate >= 500) {
    lastDisplayUpdate = nowMs;
    if (currentPage == PAGE_DATA) refreshDataArea();
  }
}
