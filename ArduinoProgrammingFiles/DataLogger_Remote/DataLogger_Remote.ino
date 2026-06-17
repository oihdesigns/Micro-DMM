/*
  DataLogger_Remote.ino
  Based on DataLogger_Claude.ino — same hardware, same SD/ADS/RTC/accel logic.

  Adds a serial remote-control protocol so every setting that was previously
  controlled by DIP switches and a potentiometer can be driven from a host
  (e.g. the companion Android app) over USB serial.

  Sending "!REMOTE" puts the logger in remote-control mode; "!LOCAL" returns
  control to the physical switches and pot.  In remote mode the physical
  hardware is ignored for settings purposes (but data is still sampled from
  the ADC / accelerometer as directed by the remote settings).

  ── Commands (host → Arduino) ──────────────────────────────────────────────
  !REMOTE           Enter remote control mode
  !LOCAL            Return to local (switch/pot) control
  !STATUS           Request an immediate $STATUS + $LOG reply
  !SD,0/1           SD logging enable / disable
  !CH2,0/1          Channel-2 voltage enable / disable
  !VOLT,0/1         Voltage channel 1 enable / disable
  !ACCEL,0/1        Accelerometer enable / disable
  !CUR,0/1          Current channel enable / disable
  !AUTO,0/1         Autorange enable / disable
  !SLOW,0/1         Slow-log (60 s) / fast-log (1 s) interval
  !SCREEN,0/1       OLED screen enable / disable
  !VTHRES,<float>   Voltage trigger threshold (V)
  !ITHRES,<float>   Current trigger threshold (A)
  !ATHRES,<float>   Accelerometer trigger threshold (g)
  !SHUNT,<float>    Shunt resistor value (Ohms, default 0.1)
  !VGAIN,<n>        Force voltage ADC gain index (0=±6.1V … 5=±0.25V); -1 = auto (threshold-based)
  !IGAIN,<n>        Force current ADC gain index (0=±6.1V … 5=±0.25V); -1 = auto (threshold-based)
  !MARK             Insert a manual mark in the next log flush

  ── Replies (Arduino → host) ───────────────────────────────────────────────
  $MODE,REMOTE / $MODE,LOCAL
  $STATUS,MODE,SD,CH2,VOLT,ACCEL,CUR,AUTO,SLOW,SCREEN,VTHRES,ITHRES,ATHRES,SHUNTR,VSCALE,ACMODE,VGAIN,IGAIN
  $LOG,V01,V23,I,AX,AY,AZ,AXPK,AYPK,AZPK,TEMP,BATT,LOGS
    (sent ~every 500 ms while connected)
  $SDSTATUS,0/1       1 = SD hardware present and initialised at startup

  All boolean fields in $STATUS are 0 or 1.
  MODE: 0=LOCAL, 1=REMOTE.
*/

#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

// ─── Peripheral objects ───────────────────────────────────────────────────────
RTC_PCF8523 rtc;
Adafruit_ADS1015 ads;
Adafruit_ADS1015 ads2;

// ─── Screen ───────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ─── Pin assignments (solder-version PCB) ────────────────────────────────────
#define SD_CS_PIN       23
const int PIXEL_PIN     = 17;
const int ch2Enable     = 25;   // INPUT_PULLUP; LOW = CH2 on
const int voltReadPin   = 24;   // INPUT_PULLUP; HIGH = volt on
const int accelPin      = 13;   // INPUT_PULLUP; HIGH = accel on  (DIP 3)
const int currentEnable = 12;   // INPUT_PULLUP; HIGH = current on (DIP 4)
const int autorangePin  = 11;   // INPUT_PULLUP; HIGH = autorange  (DIP 5)
const int slowLogPin    = 10;   // INPUT_PULLUP; HIGH = 60-s log    (DIP 6)
const int screenEnable  =  9;   // INPUT_PULLUP; HIGH = screen on   (DIP 7)
const int sdEnable      =  6;   // INPUT_PULLUP; HIGH = SD on        (DIP 8)
const int markButton    =  5;   // INPUT_PULLUP; LOW  = pressed
const int trigThresPin  = A0;   // 10 K pot
const int battPin       = A2;   // 2× 4.7 K divider

// ─── Accelerometer ────────────────────────────────────────────────────────────
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);
float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
float accelThres  = 4.0f;
float accelXMax   = 0.0f, accelYMax   = 0.0f, accelZMax   = 0.0f;
float accelLastLoggedX = 0.0f, accelLastLoggedY = 0.0f, accelLastLoggedZ = 0.0f;
unsigned long accelWindowTime = 0;
int accelCount = 0, accelZMaxcount = 0;

// ─── RAM sample buffer ────────────────────────────────────────────────────────
const size_t MAX_SAMPLES = 2000;
struct Sample { float d1, d2, d3, d4, d5, d6, td, t; };
Sample buffer[MAX_SAMPLES];
volatile size_t sampleCount = 0;
bool triggerPrev = false;
int  logCount    = 0;

// ─── ADC / voltage vars ───────────────────────────────────────────────────────
float multiplier = 1.0F;
int16_t ads0_results23 = 0, ads1_results23 = 0, results23 = 0;
float voltage01 = 0.0f, voltage01actual = 0.0f, voltage23 = 0.0f;
float vScale = 14.319f;
float shuntR  = 0.1f;    // shunt resistor for current measurement (Ohms)
float current = 0.0f;
float voltageTrigRaw = 0.0f, accelThresRaw = 0.0f;
int   tempF = 0;
float battV = 0.0f, battRaw = 0.0f;

bool  ch2on       = false;
bool  logON       = false;
bool  manualLog   = false;
bool  logError    = false;
bool  screenInhibit         = false;
bool  screenInhibitPrevious = false;
float logInterval = 1000.0f;

bool  writeTrigger = false, trigFlag = false;
float prevVoltage  = 0.0f;
float vStep        = 0.25f;
float iStep        = 0.10f;
float iStepRaw     = 0.0f;
float noiseThreshold = 0.5f;
float lastLoggedV01 = 0.25f, vMax01 = 0.0f, vMin01 = 0.0f;
float lastLoggedI   = 0.25f, iMax   = 0.0f, iMin   = 0.0f;
float prevTriggerT  = 0.0f,  deltaT = 0.0f, triggerT = 0.0f, timemS = 0.0f;
float lastLoggedV23 = 0.25f, vMax23 = 0.0f, vMin23  = 0.0f;

bool firstVoltRunAuto = true, firstVoltRunMan = true, autorange = true;

// ─── ADS gain tables ──────────────────────────────────────────────────────────
const float GAIN_FACTOR_TWOTHIRDS = 3.0f;
const float GAIN_FACTOR_1  = 2.0f;
const float GAIN_FACTOR_2  = 1.0f;
const float GAIN_FACTOR_4  = 0.5f;
const float GAIN_FACTOR_8  = 0.25f;
const float GAIN_FACTOR_16 = 0.125f;

static const adsGain_t kGainLevels[] = {
  GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN
};
static const float kGainFactors[] = {
  GAIN_FACTOR_TWOTHIRDS, GAIN_FACTOR_1, GAIN_FACTOR_2,
  GAIN_FACTOR_4, GAIN_FACTOR_8, GAIN_FACTOR_16
};
static const int kNumGainLevels = sizeof(kGainLevels) / sizeof(kGainLevels[0]);

static size_t gainIndexVolt         = 2;
static size_t gainIndexCurrent      = 5;
static size_t gainIndexCurrentPrev  = 5;
static adxl34x_range_t currentAccelRange = ADXL343_RANGE_4_G;
static size_t thresholdSuggestedGainVolt = 255;
static const int ADC_COUNT_LOW_THRESHOLD  = 250;
static const int ADC_COUNT_HIGH_THRESHOLD = 1800;
static const int ADC_GUARD_BAND = 100;

// ─── NeoPixel / SD ────────────────────────────────────────────────────────────
const int PIXEL_COUNT = 1;
Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
SdFat SD;
File32 logfile;
char filename[40];
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

// ─── Timing ───────────────────────────────────────────────────────────────────
unsigned long lastStatus     = 0;
unsigned long lastLog        = 0;
unsigned long lastLogMillies = 0;
unsigned long lastLogSend    = 0;   // $LOG serial output rate
unsigned long lastStatusSend = 0;   // periodic $STATUS output rate

// ─── Hardware availability ────────────────────────────────────────────────────
bool sdHwOK    = false;   // SD card present & initialised at startup
bool accelHwOK = false;   // ADXL343 responded at startup

// ═══════════════════════════════════════════════════════════════════════════════
//  REMOTE CONTROL STATE
// ═══════════════════════════════════════════════════════════════════════════════

bool  remoteMode      = false;
bool  remoteSD        = true;
bool  remoteCH2       = false;
bool  remoteVolt      = true;
bool  remoteAccel     = false;
bool  remoteCurrent   = false;
bool  remoteAutorange = true;
bool  remoteSlowLog   = false;
bool  remoteScreen    = true;
float remoteVThres    = 0.25f;
float remoteIThres    = 0.10f;
float remoteAThres    = 4.00f;
bool  remoteMark      = false;
bool  remoteACMode    = false;   // !ACMODE command
int   remoteVGainOverride = -1;  // -1 = threshold-based auto; 0-5 = fixed gain index
int   remoteIGainOverride = -1;  // -1 = threshold-based auto; 0-5 = fixed gain index
float acBlipThresh    = 0.15f;   // blip threshold (fraction of peak amplitude)

// ─── AC mode state ────────────────────────────────────────────────────────────
// Rolling ring buffer — 512 × 12 B = 6 KB, captures ~9 cycles at 60 Hz / 3300 SPS
const uint16_t AC_RING = 512;
struct ACSample { float v, i; uint32_t t_us; };
ACSample  acRing[AC_RING];
uint16_t  acRingHead  = 0;
uint16_t  acRingCount = 0;

// Per-cycle accumulators (reset at each rising zero crossing)
bool     acLastSignV    = false;
uint32_t acLastRiseUS   = 0;
float    acSumSqV       = 0.0f, acSumSqI      = 0.0f;
float    acCyclePeakV   = 0.0f, acCyclePeakI  = 0.0f;
int      acCycleSamples = 0;

// Results (updated each full cycle)
float    acFreq   = 0.0f;
float    acRMS_V  = 0.0f, acPeak_V = 0.0f;
float    acRMS_I  = 0.0f, acPeak_I = 0.0f;
bool     acBlipPending   = false;
int      acSettlingCycles = 0;   // cycles to suppress blip detection after a large step change

// Rolling 4-cycle average for change detection
float    acFreqHist[4] = {}, acRMSHist[4] = {};
uint8_t  acHistIdx     = 0;
float    acAvgFreq     = 0.0f, acAvgRMS = 0.0f;

unsigned long lastACStatusSend = 0;
char     acFilename[44];

String serialInputBuf = "";

// ─── Effective-setting getters ────────────────────────────────────────────────
// In local mode these read the physical hardware; in remote mode they return
// the values set by the last serial command.

inline bool effSD()        { return remoteMode ? remoteSD        : (bool)digitalRead(sdEnable);       }
inline bool effCH2()       { return remoteMode ? remoteCH2       : !(bool)digitalRead(ch2Enable);     }
inline bool effVolt()      { return remoteMode ? remoteVolt      : (bool)digitalRead(voltReadPin);    }
inline bool effAccelEn()   { return remoteMode ? remoteAccel     : (bool)digitalRead(accelPin);       }
inline bool effCurrent()   { return remoteMode ? remoteCurrent   : (bool)digitalRead(currentEnable);  }
inline bool effAutorange() { return remoteMode ? remoteAutorange : (bool)digitalRead(autorangePin);   }
inline bool effSlowLog()   { return remoteMode ? remoteSlowLog   : (bool)digitalRead(slowLogPin);     }
inline bool effScreen()    { return remoteMode ? remoteScreen    : (bool)digitalRead(screenEnable);   }

inline bool effMark() {
  if (remoteMode) {
    bool m = remoteMark;
    remoteMark = false;   // auto-clear after one read
    return m;
  }
  return !(bool)digitalRead(markButton);
}

// AC mode is independent of remote/local: only activatable via !ACMODE command.
// In local (switch) mode there is no DIP for AC, so local AC is always false.
inline bool effACMode() { return remoteACMode; }

// ═══════════════════════════════════════════════════════════════════════════════
//  SERIAL OUTPUT HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

void sendStatus() {
  // Re-send $SDSTATUS alongside every $STATUS so the host always has the
  // correct hardware-availability flag, even if it connected after the
  // one-shot message emitted in setup().
  Serial.print(F("$SDSTATUS,"));
  Serial.println(sdHwOK ? "1" : "0");
  Serial.print(F("$STATUS,"));
  Serial.print(remoteMode ? "1" : "0");      // MODE: 0=LOCAL, 1=REMOTE
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
  Serial.print(','); Serial.print(vScale,    4);
  Serial.print(','); Serial.print(remoteACMode ? "1" : "0");
  Serial.print(','); Serial.print(remoteVGainOverride);
  Serial.print(','); Serial.println(remoteIGainOverride);
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

// ═══════════════════════════════════════════════════════════════════════════════
//  SERIAL COMMAND HANDLER
// ═══════════════════════════════════════════════════════════════════════════════

void handleCommand(const String& cmd) {

  if (cmd == F("!REMOTE")) {
    remoteMode = true;
    // Snapshot current physical state into remote variables so the app starts
    // in sync with whatever the switches/pot were doing.
    remoteSD        = (bool)digitalRead(sdEnable);
    remoteCH2       = !(bool)digitalRead(ch2Enable);
    remoteVolt      = (bool)digitalRead(voltReadPin);
    remoteAccel     = (bool)digitalRead(accelPin);
    remoteCurrent   = (bool)digitalRead(currentEnable);
    remoteAutorange = (bool)digitalRead(autorangePin);
    remoteSlowLog   = (bool)digitalRead(slowLogPin);
    remoteScreen    = (bool)digitalRead(screenEnable);
    remoteVThres    = vStep;
    remoteIThres    = iStep;
    remoteAThres    = accelThres;
    Serial.println(F("$MODE,REMOTE"));
    sendStatus();
    return;
  }

  if (cmd == F("!LOCAL")) {
    remoteMode = false;
    remoteVGainOverride = -1;   // restore auto gain on local mode
    remoteIGainOverride = -1;
    Serial.println(F("$MODE,LOCAL"));
    sendStatus();
    return;
  }

  if (cmd == F("!STATUS")) {
    sendStatus();
    sendLog();
    return;
  }

  if (cmd == F("!MARK")) {
    if (remoteMode) remoteMark = true;
    return;
  }

  // All remaining commands only meaningfully change remote state,
  // but we parse them regardless and reply with updated status.

  if (cmd.startsWith(F("!SD,"))) {
    remoteSD = (bool)cmd.substring(4).toInt();
    sendStatus();
  } else if (cmd.startsWith(F("!CH2,"))) {
    remoteCH2 = (bool)cmd.substring(5).toInt();
    if (remoteMode) ch2on = remoteCH2;
    sendStatus();
  } else if (cmd.startsWith(F("!VOLT,"))) {
    remoteVolt = (bool)cmd.substring(6).toInt();
    sendStatus();
  } else if (cmd.startsWith(F("!ACCEL,"))) {
    remoteAccel = (bool)cmd.substring(7).toInt();
    sendStatus();
  } else if (cmd.startsWith(F("!CUR,"))) {
    remoteCurrent = (bool)cmd.substring(5).toInt();
    sendStatus();
  } else if (cmd.startsWith(F("!AUTO,"))) {
    remoteAutorange = (bool)cmd.substring(6).toInt();
    firstVoltRunAuto = true;
    firstVoltRunMan  = true;
    sendStatus();
  } else if (cmd.startsWith(F("!SLOW,"))) {
    remoteSlowLog = (bool)cmd.substring(6).toInt();
    sendStatus();
  } else if (cmd.startsWith(F("!SCREEN,"))) {
    remoteScreen = (bool)cmd.substring(8).toInt();
    sendStatus();
  } else if (cmd.startsWith(F("!VTHRES,"))) {
    remoteVThres = cmd.substring(8).toFloat();
    vStep = remoteVThres;
    thresholdSuggestedGainVolt = 255;   // force gain re-evaluation
    sendStatus();
  } else if (cmd.startsWith(F("!ITHRES,"))) {
    remoteIThres = cmd.substring(8).toFloat();
    iStep = remoteIThres;
    sendStatus();
  } else if (cmd.startsWith(F("!ATHRES,"))) {
    remoteAThres = cmd.substring(8).toFloat();
    accelThres = remoteAThres;
    sendStatus();
  } else if (cmd.startsWith(F("!SHUNT,"))) {
    float v = cmd.substring(7).toFloat();
    if (v >= 0.0001f) shuntR = v;   // reject nonsensical values
    sendStatus();
  } else if (cmd.startsWith(F("!ACMODE,"))) {
    remoteACMode = (bool)cmd.substring(8).toInt();
    // Reset all AC state so the next signal starts fresh
    acRingHead = 0; acRingCount = 0; acLastRiseUS = 0; acCycleSamples = 0;
    acLastSignV = false; acBlipPending = false; acHistIdx = 0; acSettlingCycles = 0;
    acFreq = 0.0f; acRMS_V = 0.0f; acPeak_V = 0.0f;
    acAvgFreq = 0.0f; acAvgRMS = 0.0f;
    memset(acFreqHist, 0, sizeof(acFreqHist));
    memset(acRMSHist,  0, sizeof(acRMSHist));
    if (remoteACMode) {
      // Fixed gain GAIN_ONE (±4.096V) for AC — no autorange during waveform capture
      ads.setGain(GAIN_ONE);
      ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, true);
      if (effCurrent()) {
        ads2.setGain(kGainLevels[gainIndexCurrent]);
        ads2.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, true);
      }
    }
    sendStatus();
  } else if (cmd.startsWith(F("!ABLIP,"))) {
    float v = cmd.substring(7).toFloat();
    if (v >= 0.01f && v <= 0.9f) acBlipThresh = v;
    sendStatus();
  } else if (cmd.startsWith(F("!VGAIN,"))) {
    int n = cmd.substring(7).toInt();
    remoteVGainOverride = (n >= 0 && n < kNumGainLevels) ? n : -1;
    thresholdSuggestedGainVolt = 255;   // force gain re-evaluation
    sendStatus();
  } else if (cmd.startsWith(F("!IGAIN,"))) {
    int n = cmd.substring(7).toInt();
    remoteIGainOverride = (n >= 0 && n < kNumGainLevels) ? n : -1;
    sendStatus();
  }
}

void processSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      String cmd = serialInputBuf;
      cmd.trim();
      serialInputBuf = "";
      if (cmd.length() > 0) handleCommand(cmd);
    } else {
      if (serialInputBuf.length() < 64) serialInputBuf += c;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  UTILITY FUNCTIONS  (unchanged from DataLogger_Claude except noted)
// ═══════════════════════════════════════════════════════════════════════════════

float readNTCTemperatureC(
    uint8_t analogPin, uint8_t adcBits, float vRef,
    float rFixed, float rNominal, float tNominalC, float beta)
{
  uint32_t adcMax = (1UL << adcBits) - 1;
  uint32_t adcRaw = analogRead(analogPin);
  float vNTC = (adcRaw * vRef) / adcMax;
  if (vNTC <= 0.0f || vNTC >= vRef) return NAN;
  float rNTC = rFixed * (vNTC / (vRef - vNTC));
  float t0K  = tNominalC + 273.15f;
  float invT = (1.0f / t0K) + (1.0f / beta) * log(rNTC / rNominal);
  return (1.0f / invT) - 273.15f;
}

// Updated to use effXxx() so it works correctly in both modes
void readPotAndSetThresholds() {
  // In remote mode skip the pot — thresholds come from serial commands.
  if (remoteMode) {
    vStep     = remoteVThres;
    iStep     = remoteIThres;
    accelThres = remoteAThres;
    return;
  }

  float raw  = analogRead(trigThresPin);
  float norm = raw / 4095.0f;
  float cubic = norm * norm * norm;

  if (effAccelEn()) {
    accelThresRaw = raw;
    accelThres = 0.25f + cubic * 16.0f;
  } else if (effCurrent()) {
    iStepRaw = raw;
    iStep = 0.01f + cubic * 2.0f;
  } else if (effVolt()) {
    voltageTrigRaw = raw;
    vStep = 0.01f + cubic * 8.0f;
  }
}

void updateGainsFromThresholds() {
  // Voltage gain
  if (effVolt() && autorange) {
    size_t suggestedGain;
    if (remoteVGainOverride >= 0) {
      // Fixed gain override from remote command
      suggestedGain = (size_t)constrain(remoteVGainOverride, 0, kNumGainLevels - 1);
    } else if (vStep > 6.0f)   suggestedGain = 0;
    else if (vStep > 3.0f)   suggestedGain = 1;
    else if (vStep > 1.5f)   suggestedGain = 2;
    else if (vStep > 0.75f)  suggestedGain = 3;
    else if (vStep > 0.375f) suggestedGain = 4;
    else                      suggestedGain = 5;

    if (suggestedGain != thresholdSuggestedGainVolt) {
      thresholdSuggestedGainVolt = suggestedGain;
      gainIndexVolt = suggestedGain;
      firstVoltRunAuto = true;
    }
  }

  // Current gain
  if (effCurrent()) {
    size_t newCurrentGain;
    if (remoteIGainOverride >= 0) {
      // Fixed gain override from remote command
      newCurrentGain = (size_t)constrain(remoteIGainOverride, 0, kNumGainLevels - 1);
    } else if (iStep > 1.0f)  newCurrentGain = 0;
    else if (iStep > 0.5f)  newCurrentGain = 1;
    else if (iStep > 0.25f) newCurrentGain = 2;
    else if (iStep > 0.1f)  newCurrentGain = 3;
    else                     newCurrentGain = 4;

    if (newCurrentGain != gainIndexCurrent) {
      gainIndexCurrent = newCurrentGain;
      ads2.setGain(kGainLevels[gainIndexCurrent]);
      (void)ads2ReadRaw23();
      ads2.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, true);
    }
  }

  // Accel range
  if (effAccelEn() && accelHwOK) {
    adxl34x_range_t newRange;
    if      (accelThres > 8.0f) newRange = ADXL343_RANGE_16_G;
    else if (accelThres > 4.0f) newRange = ADXL343_RANGE_8_G;
    else if (accelThres > 2.0f) newRange = ADXL343_RANGE_4_G;
    else                         newRange = ADXL343_RANGE_2_G;

    if (newRange != currentAccelRange) {
      currentAccelRange = newRange;
      accel.setRange(currentAccelRange);
    }
  }
}

void readAccel() {
  sensors_event_t event;
  accel.getEvent(&event);
  accelX = event.acceleration.x;
  accelY = event.acceleration.y;
  accelZ = event.acceleration.z;
  accelCount++;

  if (abs(accelX) - accelThres > abs(accelLastLoggedX) || abs(accelX) + accelThres < abs(accelLastLoggedX)) {
    accelXMax = accelX; trigFlag = true;
  }
  if (abs(accelY) - accelThres > abs(accelLastLoggedY) || abs(accelY) + accelThres < abs(accelLastLoggedY)) {
    accelYMax = accelY; trigFlag = true;
  }
  if (abs(accelZ) - accelThres > abs(accelLastLoggedZ) || abs(accelZ) + accelThres < abs(accelLastLoggedZ)) {
    accelZMax = accelZ; accelZMaxcount++; trigFlag = true;
  }
}

int16_t readRaw23()     { return ads.readADC_Differential_2_3();  }
int16_t ads2ReadRaw23() { return ads2.readADC_Differential_2_3(); }

void printField(Print* pr, char sep, uint8_t v) {
  if (sep) pr->write(sep);
  if (v < 10) pr->write('0');
  pr->print(v);
}

void printNow(Print* pr) {
  DateTime now = rtc.now();
  pr->print(now.year());
  printField(pr, '-', now.month());
  printField(pr, '-', now.day());
  printField(pr, ' ', now.hour());
  printField(pr, ':', now.minute());
  printField(pr, ':', now.second());
}

void blinkPixel(uint8_t r, uint8_t g, uint8_t b, int duration = 5) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
  delay(duration);
  pixel.clear();
  pixel.show();
}

void makeUniqueFilename(const char* base, const char* suffix, char* outBuf, size_t outSize) {
  snprintf(outBuf, outSize, "/%s%s", base, suffix);
  if (!SD.exists(outBuf)) return;
  for (int i = 1; i < 10000; i++) {
    snprintf(outBuf, outSize, "/%s%d%s", base, i, suffix);
    if (!SD.exists(outBuf)) return;
  }
  snprintf(outBuf, outSize, "/%s9999%s", base, suffix);
}

void captureSample(float data1, float data2, float data3,
                   float data4, float data5, float data6)
{
  if (sampleCount >= MAX_SAMPLES) return;
  prevTriggerT = triggerT;
  triggerT     = micros();
  timemS       = millis();
  deltaT       = triggerT - prevTriggerT;

  buffer[sampleCount].d1 = data1;
  buffer[sampleCount].d2 = data2;
  buffer[sampleCount].d3 = data3;
  buffer[sampleCount].d4 = data4;
  buffer[sampleCount].d5 = data5;
  buffer[sampleCount].d6 = data6;
  buffer[sampleCount].t  = (timemS + (triggerT / 1000));
  buffer[sampleCount].td = deltaT;
  sampleCount++;
}

void flushBufferToSD() {
  logfile = SD.open(filename, FILE_WRITE);
  if (!logfile) {
    Serial.println(F("ERROR: Could not open capture file"));
    return;
  }
  DateTime now = rtc.now();
  for (size_t i = 0; i < sampleCount; i++) {
    logfile.print(buffer[i].td, 1);   logfile.print(',');
    logfile.print(buffer[i].d1, 3);   logfile.print(',');
    logfile.print(buffer[i].d2, 1);   logfile.print(',');
    logfile.print(buffer[i].d3, 4);   logfile.print(',');
    logfile.print(buffer[i].d4, 2);   logfile.print(',');
    logfile.print(buffer[i].d5, 2);   logfile.print(',');
    logfile.print(buffer[i].d6, 2);   logfile.print(',');
    logfile.print(tempF);             logfile.print(',');
    if (manualLog) {
      logfile.print(now.timestamp());
      logfile.println(F(", MARK"));
    } else {
      logfile.println(now.timestamp());
    }
  }
  logfile.flush();
  logfile.close();
  blinkPixel(0, 255, 0);   // green = write OK
  sampleCount = 0;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  AC MODE — FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

void sendACStatus() {
  Serial.print(F("$ACSTATUS,"));
  Serial.print(acFreq,   3); Serial.print(',');
  Serial.print(acRMS_V,  4); Serial.print(',');
  Serial.print(acPeak_V, 4); Serial.print(',');
  Serial.print(acRMS_I * 1000.0f, 2); Serial.print(',');
  Serial.println(acPeak_I * 1000.0f, 2);
}

// Write an AC event (header + raw sample window) to the AC log file.
// evtType: 0 = FREQ_CHANGE, 1 = AMP_CHANGE, 2 = BLIP
void flushACEvent(uint8_t evtType) {
  File32 f = SD.open(acFilename, FILE_WRITE);
  if (!f) { logError = true; return; }
  logError = false;

  DateTime now = rtc.now();
  const char* typeStr = (evtType == 2) ? "BLIP"
                      : (evtType == 0) ? "FREQ_CHANGE" : "AMP_CHANGE";

  f.print(F("AC_EVENT,")); f.print(now.timestamp());
  f.print(','); f.print(typeStr);
  f.print(F(",Hz="));     f.print(acFreq,   3);
  f.print(F(",Vrms="));   f.print(acRMS_V,  4);
  f.print(F(",Vpeak="));  f.print(acPeak_V, 4);
  f.print(F(",mArms="));  f.print(acRMS_I  * 1000.0f, 2);
  f.print(F(",mApeak=")); f.println(acPeak_I * 1000.0f, 2);

  // Write last 3 cycles of raw samples from ring buffer
  uint16_t sampsPerCycle = (acFreq > 0) ? (uint16_t)(3300.0f / acFreq + 0.5f) : 165;
  uint16_t nWrite = min(acRingCount, (uint16_t)min((uint16_t)(sampsPerCycle * 3), AC_RING));
  uint16_t startIdx = (acRingHead + AC_RING - nWrite) % AC_RING;
  uint32_t t0 = acRing[startIdx].t_us;
  f.println(F("t_us_rel,V,I"));
  for (uint16_t k = 0; k < nWrite; k++) {
    uint16_t idx = (startIdx + k) % AC_RING;
    f.print(acRing[idx].t_us - t0); f.print(',');
    f.print(acRing[idx].v,  4);     f.print(',');
    f.println(acRing[idx].i, 5);
  }
  f.flush(); f.close();
  blinkPixel(0, 0, 255, 10);   // blue = AC event written
  logCount++;

  Serial.print(F("$ACEVENT,"));
  Serial.print(typeStr);     Serial.print(',');
  Serial.print(acFreq,   3); Serial.print(',');
  Serial.print(acRMS_V,  4); Serial.print(',');
  Serial.println(acPeak_V, 4);
}

// Called every loop() iteration when AC mode is active.
// Grabs the latest ADS1015 result, feeds the ring buffer, runs per-cycle analysis,
// and fires flushACEvent() whenever a meaningful change or blip is detected.
void runACMode() {
  uint32_t now_us = micros();

  // ADS1015 is in continuous mode with GAIN_ONE (factor = 2.0 mV/count)
  int16_t rawV = ads.getLastConversionResults();
  float v = (rawV * 2.0f / 1000.0f) * vScale;   // real volts after divider

  float i_val = 0.0f;
  if (effCurrent()) {
    int16_t rawI = ads2.getLastConversionResults();
    i_val = ((rawI * -kGainFactors[gainIndexCurrent]) / 1000.0f) / shuntR;
  }

  // Store in ring buffer
  acRing[acRingHead] = { v, i_val, now_us };
  acRingHead  = (acRingHead + 1) % AC_RING;
  if (acRingCount < AC_RING) acRingCount++;

  // Cycle accumulation
  acSumSqV += v * v;
  acSumSqI += i_val * i_val;
  if (fabsf(v)     > acCyclePeakV) acCyclePeakV = fabsf(v);
  if (fabsf(i_val) > acCyclePeakI) acCyclePeakI = fabsf(i_val);
  acCycleSamples++;

  // ── Rising zero crossing ──────────────────────────────────────────────────
  bool signV = (v >= 0.0f);
  if (signV && !acLastSignV && acCycleSamples > 5) {
    if (acLastRiseUS > 0) {
      uint32_t period_us = now_us - acLastRiseUS;
      if (period_us > 4000UL && period_us < 500000UL) {  // 2–250 Hz
        float newFreq  = 1e6f / (float)period_us;
        float newRMS_V = sqrtf(acSumSqV / (float)acCycleSamples);
        float newRMS_I = effCurrent() ? sqrtf(acSumSqI / (float)acCycleSamples) : 0.0f;

        acFreq   = newFreq;
        acRMS_V  = newRMS_V;  acPeak_V = acCyclePeakV;
        acRMS_I  = newRMS_I;  acPeak_I = acCyclePeakI;

        // Update $LOG values with per-cycle RMS
        voltage01 = newRMS_V;
        current   = newRMS_I;

        // Change detection against rolling average
        bool freqEvent = false, ampEvent = false, bigStep = false;
        if (acAvgFreq > 0.0f) {
          float ampDiff = fabsf(newRMS_V - acAvgRMS);
          freqEvent = fabsf(newFreq - acAvgFreq) > acAvgFreq * 0.02f;    // >2% freq drift
          ampEvent  = ampDiff > acAvgRMS * 0.05f + 0.01f;                // >5% amp shift
          bigStep   = ampDiff > acAvgRMS * 0.5f || freqEvent;            // >50% = step change
        }

        if ((freqEvent || ampEvent || acBlipPending) && logON) {
          uint8_t evtType = acBlipPending ? 2 : (freqEvent ? 0 : 1);
          flushACEvent(evtType);
          acBlipPending = false;
        }

        if (bigStep) {
          // Large step: instantly re-baseline so the next cycle measures against the
          // new steady-state and does not re-trigger the change detector.
          for (int k = 0; k < 4; k++) { acFreqHist[k] = newFreq; acRMSHist[k] = newRMS_V; }
          acAvgFreq = newFreq;  acAvgRMS = newRMS_V;  acHistIdx = 0;
          acSettlingCycles = 3;   // blip detection suppressed while acPeak_V catches up
        } else {
          // Normal rolling average update
          acFreqHist[acHistIdx] = newFreq;
          acRMSHist[acHistIdx]  = newRMS_V;
          acHistIdx = (acHistIdx + 1) % 4;
          float sf = 0.0f, sr = 0.0f;
          for (int k = 0; k < 4; k++) { sf += acFreqHist[k]; sr += acRMSHist[k]; }
          acAvgFreq = sf / 4.0f;
          acAvgRMS  = sr / 4.0f;
        }
      }
    }
    // Reset cycle accumulators on each rising zero crossing
    acLastRiseUS   = now_us;
    acSumSqV       = 0.0f;  acSumSqI      = 0.0f;
    acCyclePeakV   = 0.0f;  acCyclePeakI  = 0.0f;
    acCycleSamples = 0;
  }
  acLastSignV = signV;

  // ── Blip detection ────────────────────────────────────────────────────────
  // Suppressed for acSettlingCycles cycles after a large step change, while
  // acPeak_V (updated each zero-crossing) converges to the new amplitude.
  if (acSettlingCycles > 0) {
    acSettlingCycles--;
  } else if (acPeak_V > 0.05f && acFreq > 0.0f && acLastRiseUS > 0) {
    float t_s    = (float)(now_us - acLastRiseUS) / 1e6f;
    float expect = acPeak_V * sinf(2.0f * PI * acFreq * t_s);
    if (fabsf(v - expect) > acBlipThresh * acPeak_V) {
      acBlipPending = true;
    }
  }
}

// ─── Local display + human-readable serial output ────────────────────────────
void outputs(float data1, float data2) {
  unsigned long currentmillis = millis();
  if (currentmillis - lastStatus < 2000) return;
  lastStatus = currentmillis;

  DateTime now = rtc.now();

  if (effScreen()) {
    updateDisplay();
    // Human-readable serial (useful for debugging over local monitor)
    Serial.print(F("Time:")); Serial.print(now.timestamp());
    Serial.print(F("/ V0-1:")); Serial.print(voltage01, 3);
    Serial.print(F("V / I:"));  Serial.print(current, 4);
    Serial.print(F("/ Temp:")); Serial.print(tempF);
    Serial.print(F("F Batt:")); Serial.print(battV, 2);
    Serial.print(F("V Logs:"));  Serial.println(logCount);
    if (remoteMode) Serial.println(F("[REMOTE MODE]"));

    Serial.print(F("Accel X:"));   Serial.print(accelXMax);
    Serial.print(F("  Y:"));       Serial.print(accelYMax);
    Serial.print(F("  Z:"));       Serial.print(accelZMax);
    Serial.print(F("  count:"));   Serial.println(accelCount);

    accelXMax = 0; accelYMax = 0; accelZMax = 0;
    accelWindowTime = 0; accelCount = 0; accelZMaxcount = 0;

    blinkPixel(0, 128, 128);
  }
}

// ─── Voltage measurement (unchanged logic; uses ch2on / autorange vars) ──────
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
      firstVoltRunAuto = false;
      firstVoltRunMan  = true;
      ads.setGain(kGainLevels[gainIndexVolt]);
      (void)readRaw23();
    }

    ads0_results23 = ads.getLastConversionResults();
    ads.setGain(kGainLevels[gainIndexVolt]);
    ads0_results23 = readRaw23();

    if (abs(ads0_results23) > ADC_COUNT_HIGH_THRESHOLD && gainIndexVolt > 0) {
      --gainIndexVolt;
      ads.setGain(kGainLevels[gainIndexVolt]);
      (void)readRaw23();
      return;
    }
    if (abs(ads0_results23) < ADC_COUNT_LOW_THRESHOLD && gainIndexVolt < (size_t)(kNumGainLevels - 1)) {
      ++gainIndexVolt;
      ads.setGain(kGainLevels[gainIndexVolt]);
      (void)readRaw23();
      return;
    }

    voltage01actual = (ads0_results23 * kGainFactors[gainIndexVolt]) / 1000.0f;
    voltage01       = voltage01actual * vScale;

  } else {
    if (firstVoltRunMan) {
      firstVoltRunMan  = false;
      firstVoltRunAuto = true;
      ads.setGain(GAIN_ONE);
      ads2.setGain(kGainLevels[gainIndexCurrent]);
      (void)readRaw23();
      (void)ads2ReadRaw23();
      ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, true);
      ads2.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, true);
    }
    multiplier      = 2.0F;
    ads0_results23  = ads.getLastConversionResults();
    voltage01actual = (ads0_results23 * multiplier) / 1000.0f;
    voltage01       = voltage01actual * vScale;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DISPLAY
// ═══════════════════════════════════════════════════════════════════════════════

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(2);

  if (effACMode()) {
    // AC mode: show frequency and RMS voltage prominently
    display.setCursor(0, 0);
    display.print(acFreq, 1); display.print(F("Hz"));
    display.setCursor(0, 16);
    display.print(acRMS_V, 3); display.print(F("Vr"));
    display.setTextSize(1);
    display.setCursor(0, 32);
    display.print(F("Vrms I:")); display.print(acRMS_I * 1000.0f, 1); display.print(F("mA"));
    display.setCursor(0, 40);
    display.print(F("Vpk:")); display.print(acPeak_V, 3);
    display.setCursor(64, 40);
    display.print(F("Evt:")); display.print(logCount);
    display.setCursor(0, 48);
    display.print(F("AC blipT:")); display.print(acBlipThresh, 2);
    display.setCursor(0, 56);
    display.print(remoteMode ? F("REM ") : F("LOC "));
    display.print(F("AC MODE"));
    display.display();
    return;
  }

  display.setCursor(0, 0);
  display.print(F("V:"));
  display.print(voltage01, 2);

  display.setCursor(0, 16);
  display.print(F("mA:"));
  if (effCurrent()) {
    display.print((current * 1000), 0);
  } else {
    display.print(F("- #4"));
  }

  display.setTextSize(1);

  if (effCH2()) {
    display.setCursor(0, 16);
    display.print(F("V2:"));
    display.print(voltage23, 2);
  }

  if (effAccelEn()) {
    display.setCursor(108, 0);
    display.print(F("g1"));
    display.setCursor(100, 8);
    display.print(accelThres);
  }

  display.setTextSize(1);
  display.setCursor(0, 32);
  display.print(F("#logs:"));
  display.print(logCount);

  display.setCursor(64, 32);
  if (effAccelEn()) {
    display.print(F("aT:")); display.print(accelThres, 1);
  } else if (effCurrent()) {
    display.print(F("iT:")); display.print(iStep, 2);
  } else {
    display.print(F("vT:")); display.print(vStep, 2);
  }

  timemS = millis();
  display.setCursor(0, 40);
  display.print(F("Runtime(s):"));
  display.print((timemS / 1000), 0);

  if (logError) {
    display.setCursor(0, 48);
    display.print(F("SD ERROR"));
  } else if (!logON) {
    display.setCursor(0, 48);
    display.print(F("SD OFF #8"));
  } else {
    display.setCursor(0, 48);
    display.print(F("SD GOOD"));
  }

  display.setCursor(64, 48);
  display.print(F("Temp"));
  display.print(tempF);
  display.print('F');

  display.setCursor(0, 56);
  if (remoteMode) {
    display.print(F("REM "));
  } else {
    display.print(F("LOC "));
  }
  if (autorange) {
    display.print(F("Auto"));
  } else {
    display.print(F("Fix #5"));
  }

  display.setCursor(72, 56);
  display.print(F("B:"));
  display.print(battV, 1);

  display.display();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("DataLogger_Remote ready"));

  Wire.begin();
  Wire.setClock(1000000);
  analogReadResolution(12);

  // ── OLED ──
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED init failed"));
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setRotation(0);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Datalogger"));
  display.setCursor(0, 20);
  display.setTextSize(1);
  display.print(F("Remote v1.0"));
  display.display();
  delay(200);

  // ── RTC ──
  if (!rtc.begin()) {
    Serial.println(F("RTC not found!"));
    while (1);
  }
  DateTime now = rtc.now();
  Serial.print(F("RTC Time: "));
  Serial.println(now.timestamp());

  // ── Pin modes ──
  pinMode(sdEnable,      INPUT_PULLUP);
  pinMode(ch2Enable,     INPUT_PULLUP);
  pinMode(accelPin,      INPUT_PULLUP);
  pinMode(currentEnable, INPUT_PULLUP);
  pinMode(trigThresPin,  INPUT);
  pinMode(autorangePin,  INPUT_PULLUP);
  pinMode(voltReadPin,   INPUT_PULLUP);
  pinMode(markButton,    INPUT_PULLUP);
  pinMode(battPin,       INPUT);
  pinMode(slowLogPin,    INPUT_PULLUP);
  pinMode(screenEnable,  INPUT_PULLUP);

  // ── ADS ──
  ads.setDataRate(RATE_ADS1015_3300SPS);
  if (!ads.begin(0x48)) { Serial.println(F("ADS1 init failed")); while (1); }
  if (!ads2.begin(0x49)) { Serial.println(F("ADS2 init failed")); while (1); }
  Serial.println(F("Both ADS started"));

  // ── NeoPixel ──
  pixel.begin(); pixel.clear(); pixel.show();

  // ── SD — always attempt; do not halt on failure ──
  if (SD.begin(config)) {
    sdHwOK = true;
    sprintf(filename, "log_%04d%02d%02d_%02d%02d%02d.csv",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
    logfile = SD.open(filename, FILE_WRITE);
    if (logfile) {
      logfile.print(F("Log Start Time,"));
      logfile.println(now.timestamp());
      logfile.println(F("Delta uS,V Ch1,V Ch2,I,X,Y,Z,Temp,RTC Stamp"));
      logfile.flush();
      logfile.close();
      // Create a companion AC log file (written only when AC events occur)
      sprintf(acFilename, "ac_%04d%02d%02d_%02d%02d%02d.csv",
              now.year(), now.month(), now.day(),
              now.hour(), now.minute(), now.second());
      Serial.print(F("SD OK, logging to: "));
      Serial.println(filename);
    } else {
      sdHwOK = false;
      Serial.println(F("SD: could not create log file"));
    }
  } else {
    sdHwOK = false;
    Serial.println(F("SD init failed (no card or wiring issue)"));
    // Blink red to indicate no SD, but don't halt
    for (int i = 0; i < 5; i++) blinkPixel(128, 0, 0, 100);
  }

  // Inform the host whether SD is available
  Serial.print(F("$SDSTATUS,"));
  Serial.println(sdHwOK ? "1" : "0");

  // ── Accelerometer — always attempt; do not halt on failure ──
  if (accel.begin()) {
    accelHwOK = true;
    accel.setRange(ADXL343_RANGE_4_G);
    accel.setDataRate(ADXL343_DATARATE_1600_HZ);
    Serial.println(F("ADXL343 OK"));
  } else {
    accelHwOK = false;
    Serial.println(F("ADXL343 not found — accel channel disabled"));
  }

  // ── Channel 2 ──
  if (!digitalRead(ch2Enable)) {
    ch2on = true;
  } else {
    ch2on = false;
    voltage23 = 0.0f;
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, true);
    ads2.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, true);
  }

  Serial.println(F("Setup complete — send !REMOTE to enter remote mode"));
  // Send initial status so host can sync immediately
  sendStatus();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {

  // ── 1. Parse any incoming serial commands ──
  processSerial();

  // ── 2. Read effective settings (local or remote) ──
  autorange     = effAutorange();
  logInterval   = effSlowLog() ? 60000.0f : 1000.0f;
  ch2on         = effCH2();
  logON         = sdHwOK && effSD();

  // ── 3. Screen inhibit ──
  bool newScreenInhibit = !effScreen();
  if (newScreenInhibit != screenInhibit) {
    screenInhibit = newScreenInhibit;
    if (screenInhibit) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.print(F("Inhibit"));
      display.display();
    }
  }

  // ── 4. Battery ──
  battRaw = analogRead(battPin);
  battV   = (battRaw / 4095.0f) * 3.3f * 2.0f;

  // ── 5. Thresholds ──
  readPotAndSetThresholds();   // no-op in remote mode; updates vStep/iStep/accelThres in local mode
  updateGainsFromThresholds();

  // ── 6. Manual mark / mark button ──
  bool markPressed = effMark();
  if (!markPressed) {
    manualLog = false;
  } else {
    manualLog = true;
    logCount++;
  }

  // ── 7. Temperature ──
  float tempC = readNTCTemperatureC(A3, 12, 3.3f, 10000.0f, 10000.0f, 25.0f, 3950.0f);
  tempF = (int)((tempC * 9.0f / 5.0f) + 32.0f);

  // ── 8. Accelerometer ──
  if (effAccelEn() && accelHwOK) {
    readAccel();
  } else {
    accelX = 0.0f; accelY = 0.0f; accelZ = 0.0f;
  }

  if (effACMode()) {
    // ── 9–13. AC mode: continuous waveform sampling and analysis ──────────────
    // runACMode() grabs the latest ADS1015 result, accumulates per-cycle stats,
    // detects freq/amplitude changes and blips, and calls flushACEvent() as needed.
    runACMode();

  } else {
    // ── 9. Voltage (DC) ───────────────────────────────────────────────────────
    prevVoltage = voltage01;
    if (effVolt()) {
      measureVoltage();
    } else {
      voltage01 = 0.0f;
    }

    // ── 10. Current (DC) ──────────────────────────────────────────────────────
    if (effCurrent()) {
      ads1_results23 = ads2.getLastConversionResults();
      current = ((ads1_results23 * -kGainFactors[gainIndexCurrent]) / 1000.0f) / shuntR;
    } else {
      current = 0.0f;
    }

    // ── 11. Trigger evaluation ────────────────────────────────────────────────
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
      writeTrigger     = true;
      lastLoggedV01    = voltage01;  lastLoggedV23 = voltage23;
      lastLoggedI      = current;
      accelLastLoggedX = accelX;     accelLastLoggedY = accelY;  accelLastLoggedZ = accelZ;
      trigFlag         = false;
    } else {
      writeTrigger = false;
    }

    // ── 12. Capture / periodic log ────────────────────────────────────────────
    if (writeTrigger && logON) {
      captureSample(voltage01, voltage23, current, accelX, accelY, accelZ);
      logCount++;
    } else {
      outputs(voltage01, voltage23);
      if ((millis() - lastLog > (unsigned long)logInterval) || manualLog) {
        if (!manualLog) lastLog = millis();
        captureSample(voltage01, voltage23, current, accelX, accelY, accelZ);
        if (logON) {
          flushBufferToSD();
          logCount++;
          manualLog = false;
        }
      }
      writeTrigger = false;
    }

    // ── 13. Flush on trigger falling edge ─────────────────────────────────────
    if (((triggerPrev && !writeTrigger) || manualLog) && logON) {
      flushBufferToSD();
    }
    triggerPrev = writeTrigger;
  }

  // ── 14. Periodic $LOG output (~500 ms) ──
  unsigned long now = millis();
  if (now - lastLogSend >= 500) {
    lastLogSend = now;
    sendLog();
  }

  // ── 15. Periodic $STATUS output (~10 s) ──
  if (now - lastStatusSend >= 10000) {
    lastStatusSend = now;
    sendStatus();
  }

  // ── 16. Periodic $ACSTATUS output (~500 ms, AC mode only) ──
  if (effACMode() && now - lastACStatusSend >= 500) {
    lastACStatusSend = now;
    sendACStatus();
  }
}
