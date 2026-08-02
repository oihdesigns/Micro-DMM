/*
 * OpenLeadDetect_Feather_Headless.ino
 *
 * Differential voltmeter with open/closed lead detection.
 * Headless: no TFT.  All output is over serial + the NeoPixel and the
 * 2-bit lead-status pins.  Derived from OpenLeadDetect_Feather_Voltmeter.
 *
 * Hardware:
 *   - Adafruit ESP32-S2 or S3 Feather
 *   - ADS1115 (16-bit) or ADS1015 (12-bit) ADC on I2C (addr 0x48)
 *     Select via USE_ADS1115 / USE_ADS1015 define below
 *   - Voltage divider bridge with MOSFET on VbridgePin
 *   - 2-bit lead-status output on StatusHiPin / StatusLoPin (see below)
 *
 * Serial commands (115200 baud):
 *   Legacy single-char (send with newline): S G U M R D ?
 *
 * ── Diagnostic protocol (compatible with diagnostic_gui.py) ───────
 * Commands in (each terminated with newline):
 *   !DIAG[,0|1]      enter/exit diagnostic mode (bare = toggle)
 *   !STREAM[,0|1]    continuous raw streaming on/off
 *   !RATE,<ms>       stream interval in ms
 *   !VMODE,<0|1|2>   voltage mode: 0=auto  1=lock ON  2=disable
 *   !MOSFET,<-1|0|1> bridge: -1=auto(run detection) 0=hold off 1=hold on
 *   !CAP[,<ms>]      capture ADC across the bridge-MOSFET toggle, then dump
 *   !STATUS  / !?    print current status
 * Data out:
 *   $STATUS,diag=..,vmode=..,mosfet=..,stream=..,rate=..,capms=..,res=..,vref=..
 *   $DIAG,<ms>,<rawDiff>,<diffV>                           (streaming)
 *   $CAPSTART,<n>,<toggleUs>,<durMs>,<fullScale>,<vref>    (capture header)
 *   $CAP,<t_us>,<rawDiff>                                  (capture rows)
 *   $CAPEND
 * This board only reads the ADC's internal differential (A0-A1); there are
 * no separate single-ended readings.  Capture reports counts with a
 * fixed-gain scale (fullScale=1000, vref=gain factor in mV/bit) so the host
 * recovers volts as count/1000*gain.  ADS conversions are slow, so a few-ms
 * capture yields only a handful of samples; lengthen the window for more.
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MAX1704X.h>
#include <ctype.h>

// ── ADC Selection: Comment/uncomment ONE of these ────────────────
//#define USE_ADS1115    // 16-bit ADC
#define USE_ADS1015  // 12-bit ADC

// ── Bridge MOSFET Logic: Comment/uncomment ONE of these ──────────
//#define VBRIDGE_ACTIVE_HIGH   // VbridgePin HIGH = bridge connected
#define VBRIDGE_ACTIVE_LOW  // VbridgePin LOW  = bridge connected

// Bridge pin states (computed from above selection)
#ifdef VBRIDGE_ACTIVE_HIGH
#define BRIDGE_ON   HIGH
#define BRIDGE_OFF  LOW
#else
#define BRIDGE_ON   LOW
#define BRIDGE_OFF  HIGH
#endif

#ifdef USE_ADS1115
Adafruit_ADS1115 ads;
#else
Adafruit_ADS1015 ads;
#endif

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
const unsigned long battReadInterval = 5000;  // 5 s between reads

// ── Pin assignments (adjust for your wiring) ─────────────────────
const int VbridgePin     = 5;    // MOSFET gate for bridge measurement
const int StatusHiPin    = 6;    // lead-status output, high-order bit
const int StatusLoPin    = 9;    // lead-status output, low-order bit
const int cfSuppressBtn  = 0;    // Boot button (GPIO 0) toggles closed/float suppression

// ── Lead-status digital output (2-bit code on StatusHiPin / StatusLoPin) ──
// Written as <StatusHiPin><StatusLoPin>, refreshed every loop() pass:
//   00  not used  (detection suppressed -- volt-only mode)
//   01  floating  (StatusHiPin LOW,  StatusLoPin HIGH)
//   10  closed    (StatusHiPin HIGH, StatusLoPin LOW)
//   11  voltage present
#define STATUS_NONE     0x0
#define STATUS_FLOAT    0x1
#define STATUS_CLOSED   0x2
#define STATUS_VOLTAGE  0x3

// ── Timing ────────────────────────────────────────────────────────
unsigned long currentMillis   = 0;
unsigned long lastSerialTime  = 0;
unsigned long lastStatusTime  = 0;
unsigned long lastBlinkTick   = 0;

const unsigned long serialInterval  = 1;
const unsigned long statusInterval  = 2000;
const unsigned long blinkInterval   = 1000;   // 1 Hz periodic NeoPixel tick

// ── Voltage measurement ───────────────────────────────────────────
float medianVoltage     = 0.0;
float newVoltageReading = 0.0;
float displayVoltage    = 0.0;   // smoothed or raw depending on cfSuppress
float prevOutVoltage    = 0.0;
float vActual           = 0.0;
int16_t countV          = 0;

const float VOLTAGE_SCALE_full = -69.95; // seperate boards version:69.6023;
const float VOLTAGE_SCALE_low  = -3.51108;
float vScale = 0.0;

bool vClimb      = false;
bool firstVoltRun = true;
uint8_t statusCode = STATUS_NONE;   // current 2-bit lead-status code (see above)
bool manual      = false;
bool range       = false;
bool updates     = true;
bool debug       = false;
bool forceStop   = false;
bool cfSuppress      = false;   // closed/float detection suppressed
bool prevcfSuppress  = false;   // previous suppression state
bool cfBtnPrev       = true;    // previous boot button state (HIGH = released) 
bool alarm1       = false;
bool alarmFlag   = false;

// ── AC mode ───────────────────────────────────────────────────────
bool  acMode              = false;  // long-press boot btn toggles AC/DC
float vrmsVoltage         = 0.0;   // most recent computed VRMS

// VRMS accumulation window (reset each time window fills)
float acSumSq             = 0.0;
int   acSampleCount       = 0;
const int AC_VRMS_SAMPLES = 32;    // ~2 cycles at 60 Hz (est. ~16 samples/cycle)

// Zero-crossing debounce: signal must stay below threshold for this long
// before ClosedOrFloat() is called in AC mode.  A genuine mains zero crossing
// at 120 Vrms passes through ±0.5 V in ≈8 µs; 4 ms catches only true zero.
unsigned long acBelowThreshStart  = 0;
bool          acBelowThreshActive = false;
const unsigned long AC_ZERO_DEBOUNCE_MS = 4;  // ≈ quarter cycle at 60 Hz

// Long-press timing for boot button
unsigned long btnPressStart = 0;
const unsigned long LONG_PRESS_MS = 600;

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
unsigned long lastRedBlinkTime    = 0;
const unsigned long RED_BLINK_MIN_MS_AC = 500;  // max 2 Hz red blink in AC mode

// ── Diagnostic mode ───────────────────────────────────────────────
bool diagMode = false;                 // suppresses human serial, enables $ protocol
bool streamOn = false;                 // continuous raw streaming
unsigned long streamIntervalMs = 50;   // streaming period (ADS is slow; ~20 Hz default)
unsigned long lastStreamMs = 0;

enum VoltOverride { VOLT_AUTO, VOLT_FORCE_ON, VOLT_DISABLED };
VoltOverride voltOverride = VOLT_AUTO; // applies wherever Vzero is decided

int mosfetHold = -1;                    // -1 auto (run detection), 0 hold off, 1 hold on

// Transient capture buffer (raw differential counts; volts recovered by host)
const int CAP_MAX_SAMPLES   = 600;
const unsigned long CAP_PRE_US = 500;   // baseline sampled before the toggle
unsigned long capDurationMs = 5;        // total capture window
uint32_t capT[CAP_MAX_SAMPLES];
int16_t  capDiff[CAP_MAX_SAMPLES];
int capCount = 0;

// Serial command line buffer
char cmdBuf[48];
int  cmdLen = 0;

// ── ADC auto-ranging (values differ for ADS1115 vs ADS1015) ───────
#ifdef USE_ADS1115
// ADS1115: 16-bit, gain factors in mV/bit
const float GAIN_FACTOR_TWOTHIRDS = 0.1875;    // +/-6.144 V
const float GAIN_FACTOR_1         = 0.125;     // +/-4.096 V
const float GAIN_FACTOR_2         = 0.0625;    // +/-2.048 V
const float GAIN_FACTOR_4         = 0.03125;   // +/-1.024 V
const float GAIN_FACTOR_8         = 0.015625;  // +/-0.512 V
const float GAIN_FACTOR_16        = 0.0078125; // +/-0.256 V
static const int ADC_COUNT_LOW_THRESH  = 10000;
static const int ADC_COUNT_HIGH_THRESH = 30000;
float vClosedThres     = 0.07;
#define ADS_RATE_FAST  RATE_ADS1115_860SPS
#define ADS_RATE_MID  RATE_ADS1115_250SPS
#define ADS_RATE_SLOW  RATE_ADS1115_64SPS
#else
// ADS1015: 12-bit, gain factors in mV/bit (16x larger than ADS1115)
const float GAIN_FACTOR_TWOTHIRDS = 3.0;       // +/-6.144 V
const float GAIN_FACTOR_1         = 2.0;       // +/-4.096 V
const float GAIN_FACTOR_2         = 1.0;       // +/-2.048 V
const float GAIN_FACTOR_4         = 0.5;       // +/-1.024 V
const float GAIN_FACTOR_8         = 0.25;      // +/-0.512 V
const float GAIN_FACTOR_16        = 0.125;     // +/-0.256 V
static const int ADC_COUNT_LOW_THRESH  = 600;
static const int ADC_COUNT_HIGH_THRESH = 1800;
float vClosedThres     = 1.55;  //red=1.34
#define ADS_RATE_FAST  RATE_ADS1015_3300SPS
#define ADS_RATE_MID  RATE_ADS1015_490SPS
#define ADS_RATE_SLOW  RATE_ADS1015_250SPS
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

// ── Forward declarations ──────────────────────────────────────────
void measureVoltage();
void ClosedOrFloat();
const char* bridgeResistanceLabel(float v);
void statusUpdate();
void handleSerialCommands(char cmd);
void blinkPixel(uint8_t r, uint8_t g, uint8_t b, unsigned long duration = BLINK_DURATION_MS);
void pollSerial();
void handleLine(char *line);
void printStatus();
void streamSample();
void runCapture(unsigned long durationMs);
int  leadDisplayState();
uint8_t leadStatusCode();
void writeStatusOutputs(uint8_t code);

// ==================================================================
//  SETUP
// ==================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // ── Power rail for I2C + NeoPixel ──
  // On the TFT Feather this rail also feeds the display; it must be on for the
  // ADC/fuel gauge/NeoPixel even though nothing is drawn.  Backlight stays off.
#ifdef TFT_I2C_POWER
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif
#ifdef TFT_BACKLITE
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, LOW);
#endif
  delay(10);

  Serial.println("OPEN LEAD DETECT - Feather Headless v1.0");

  // ── I2C + ADC ──
  Wire.begin();
  Wire.setClock(1000000);           // 400 kHz fast-mode I2C: each ADS read is
                                   // several register transactions, so the bus
                                   // clock dominates throughput. ADS1015 also
                                   // supports 1 MHz if the bus wiring is clean.
  analogReadResolution(12);        // ESP32 12-bit ADC (0-4095)

  while (!ads.begin(0x48, &Wire)) {
    Serial.println("ADS1115 not found - retrying...");
    delay(1000);
  }
  Serial.println("ADS1115 connected.");
  ads.setGain(GAIN_SIXTEEN);
  ads.setDataRate(ADS_RATE_FAST);

  // ── GPIO ──
  pinMode(VbridgePin,     OUTPUT);
  pinMode(StatusHiPin,    OUTPUT);
  pinMode(StatusLoPin,    OUTPUT);
  pinMode(cfSuppressBtn,  INPUT_PULLUP);
  digitalWrite(VbridgePin, BRIDGE_ON);  // bridge connected (resting state)
  writeStatusOutputs(STATUS_NONE);      // 00 until the first classification

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
}

// ==================================================================
//  LEAD STATUS
// ==================================================================
// Single source of truth for "what state are the leads in":
//   0 = closed   1 = floating   2 = V != 0   4 = closed/float suppressed
int leadDisplayState() {
  if      (cfSuppress)           return 4;   // volt-only mode, no lead test
  else if (!Vzero)               return 2;   // voltage present
  else if (ClosedConfidence > 5) return 0;   // closed
  else                           return 1;   // floating
}

// Map the display state onto the 2-bit output code.  Suppressed detection has
// no lead result to report, so it emits the "not used" code.
uint8_t leadStatusCode() {
  switch (leadDisplayState()) {
    case 0:  return STATUS_CLOSED;
    case 1:  return STATUS_FLOAT;
    case 2:  return STATUS_VOLTAGE;
    default: return STATUS_NONE;
  }
}

// Drive the two status pins and record the code for the serial status line.
void writeStatusOutputs(uint8_t code) {
  statusCode = code;
  digitalWrite(StatusHiPin, (code & 0x2) ? HIGH : LOW);
  digitalWrite(StatusLoPin, (code & 0x1) ? HIGH : LOW);
}

// ==================================================================
//  MAIN LOOP
// ==================================================================
void loop() {
  currentMillis = millis();

  // ── Lead-status digital output ──
  // Refreshed every pass (including diagnostic mode) from the most recent
  // classification, so an external device sees the current state immediately.
  // No rate limiting: the code follows the detection result directly.
  writeStatusOutputs(leadStatusCode());

  // ── Serial (! diagnostic lines + legacy single chars) ──
  pollSerial();

  // ── Diagnostic mode takes over the loop ──
  if (diagMode) {
    if (mosfetHold >= 0) {
      // Manual bridge hold: detection paused, MOSFET parked for observation.
      digitalWrite(VbridgePin, mosfetHold ? BRIDGE_ON : BRIDGE_OFF);
    } else {
      measureVoltage();        // keep detection alive (honours voltOverride)
    }
    if (streamOn && (currentMillis - lastStreamMs >= streamIntervalMs)) {
      lastStreamMs = currentMillis;
      streamSample();
    }
    return;                    // skip human-readable status
  }

  // Boot button:
  //   Short press (<600 ms release) → toggle cfSuppress (volt-only mode)
  //   Long press  (≥600 ms release) → toggle AC mode (VRMS + debounced lead detect)
  bool cfBtnNow = digitalRead(cfSuppressBtn);
  if (cfBtnNow == LOW && cfBtnPrev == HIGH) {       // falling edge: press start
    btnPressStart = currentMillis;
  }
  if (cfBtnNow == HIGH && cfBtnPrev == LOW) {       // rising edge: evaluate hold
    unsigned long heldMs = currentMillis - btnPressStart;
    if (heldMs >= LONG_PRESS_MS) {
      acMode = !acMode;
      acSumSq = 0.0;  acSampleCount = 0;
      vrmsVoltage = 0.0;
      acBelowThreshActive = false;
      firstVoltRun = true;
      Serial.print("AC mode: ");
      Serial.println(acMode ? "ON" : "OFF");
    } else {
      cfSuppress = !cfSuppress;
      Serial.print("CF suppress: ");
      Serial.println(cfSuppress ? "ON" : "OFF");
    }
  }
  cfBtnPrev = cfBtnNow;

  // ── Measure ──
  float prev = newVoltageReading;
  measureVoltage();
  vClimb = (fabs(prev) < fabs(newVoltageReading));

  // ── NeoPixel alerts (immediate blinks on state change) ──
  bool voltageHigh = (fabs(medianVoltage) > VOLTAGE_ALARM_THRESH);
  bool closedNow   = (Vzero && ClosedConfidence > 5);

  // Red blink the instant voltage first crosses above threshold.
  // In AC mode, rate-limited to 2 Hz to avoid rapid flicker.
  if (voltageHigh && !prevVoltageHigh) {
    if (!acMode || (currentMillis - lastRedBlinkTime >= RED_BLINK_MIN_MS_AC)) {
      blinkPixel(250, 0, 0);
      lastRedBlinkTime = currentMillis;
    }
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

  // Periodic status
  if ((currentMillis - lastStatusTime >= statusInterval && updates) ||
      (Vzero && VzeroFlag)) {
    statusUpdate();
  }

  // ── Battery (slow read, every 10 s) ──
  if (battFound && (currentMillis - lastBattRead >= battReadInterval)) {
    lastBattRead = currentMillis;
    battPct     = constrain(battMonitor.cellPercent(), 0.0f, 100.0f);
    battVoltage = battMonitor.cellVoltage();
  }

  // ── Periodic NeoPixel tick (1 Hz) ──
  if (currentMillis - lastBlinkTick >= blinkInterval) {
    lastBlinkTick = currentMillis;

    // In AC mode the red blink is additionally rate-limited to 2 Hz.
    if (voltageHigh &&
        (!acMode || (currentMillis - lastRedBlinkTime >= RED_BLINK_MIN_MS_AC))) {
      blinkPixel(200, 0, 0);
      lastRedBlinkTime = currentMillis;
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
      digitalWrite(VbridgePin, BRIDGE_OFF);
      vScale = VOLTAGE_SCALE_full;
    } else {
      digitalWrite(VbridgePin, BRIDGE_ON);
      vScale = VOLTAGE_SCALE_low;
      ads.setDataRate(ADS_RATE_SLOW);
    }
  } else {
    digitalWrite(VbridgePin, BRIDGE_OFF);   // ensure bridge disconnected
    vScale = VOLTAGE_SCALE_full;
  }

  if (firstVoltRun) {
    gainIndexVolt = kNumGainLevels - 1;   // start at highest gain
    firstVoltRun = false;
  }

  if(prevcfSuppress != cfSuppress && !acMode){  // AC mode always uses FAST rate

    if(cfSuppress){
      ads.setDataRate(ADS_RATE_SLOW);
      //Serial.print("64sps / 250 (1015)");
    }else{
      ads.setDataRate(ADS_RATE_MID);
      //Serial.print("860sps / 3300 (1015)"); //Out of date
    }
    prevcfSuppress = cfSuppress;
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
  newVoltageReading = (vActual * vScale);// - 0.023;

  //The below line tests using the straight reading instead of an EMA
  //medianVoltage = newVoltageReading;
  medianVoltage += (newVoltageReading - medianVoltage) / 10.0f;

  // Zero-threshold (fixed)
  CorFTrig = 0.3; //this was 0.25 for the unit that has distinct boards

  prevVzero = Vzero;

  if (acMode) {
    // ── AC mode: accumulate VRMS ──────────────────────────────────
    acSumSq += newVoltageReading * newVoltageReading;
    if (++acSampleCount >= AC_VRMS_SAMPLES) {
      vrmsVoltage   = sqrt(acSumSq / (float)acSampleCount);
      acSumSq       = 0.0;
      acSampleCount = 0;
    }
    displayVoltage = vrmsVoltage;

    // Zero-crossing debounce for open-lead detect.
    // The instantaneous reading must stay below CorFTrig for a full quarter
    // cycle (AC_ZERO_DEBOUNCE_MS ≈ 4 ms at 60 Hz) before ClosedOrFloat() is
    // called.  A real mains zero crossing passes the threshold in microseconds,
    // so it will never accumulate enough time to trigger.
    bool acZeroCond = (fabs(newVoltageReading) < CorFTrig);
    if (voltOverride == VOLT_DISABLED) acZeroCond = true;    // force lead test
    if (voltOverride == VOLT_FORCE_ON) acZeroCond = false;   // force voltage-present
    if (!manual && !cfSuppress && acZeroCond) {
      if (!acBelowThreshActive) {
        acBelowThreshActive = true;
        acBelowThreshStart  = currentMillis;
      }
      if (currentMillis - acBelowThreshStart >= AC_ZERO_DEBOUNCE_MS) {
        Vzero     = true;
        VzeroFlag = (prevVzero != Vzero);
        ClosedOrFloat();        
      } else {
        Vzero     = false;
        vFloating = false;
        VzeroFlag = false;
      }
    } else {
      acBelowThreshActive = false;
      Vzero     = false;
      vFloating = false;
      VzeroFlag = false;
    }

  } else {
    // ── DC mode: original behaviour ───────────────────────────────
    displayVoltage = cfSuppress ? newVoltageReading : medianVoltage;

    bool dcZeroCond = (fabs(medianVoltage) < CorFTrig);
    if (voltOverride == VOLT_DISABLED) dcZeroCond = true;     // force lead test
    if (voltOverride == VOLT_FORCE_ON) dcZeroCond = false;    // force voltage-present
    if (dcZeroCond && !manual && !cfSuppress) {
      Vzero     = true;
      VzeroFlag = (prevVzero != Vzero);
      ClosedOrFloat();
    } else {
      Vzero     = false;
      vFloating = false;
      VzeroFlag = false;
    }
  }
}

// ==================================================================
//  BRIDGE MEASUREMENT  -  CLOSED / FLOAT CLASSIFICATION
// ==================================================================
void ClosedOrFloat() {
  bool prevClosed = vClosed;

  vClosed = vFloating = vUndefined = false;

  // First bridge sample
  digitalWrite(VbridgePin, BRIDGE_ON);
  //delay(1); //I don't think I need this, or it should be in micro seconds, not millis.
  ads.setDataRate(ADS_RATE_FAST);
  ads.setGain(GAIN_TWO);
  float bv1 = ads.readADC_Differential_0_1()
              * (GAIN_FACTOR_2 / 1000.0f);
  digitalWrite(VbridgePin, BRIDGE_OFF);
  ads.setDataRate(ADS_RATE_MID);
  /*
    // Second bridge sample
    digitalWrite(VbridgePin, BRIDGE_ON);
    float bv2 = ads.readADC_Differential_0_1()
                * (0.0078125f / 1000.0f);
    digitalWrite(VbridgePin, BRIDGE_OFF);

    bridgeV = (fabs(bv1) + fabs(bv2)) / 2.0f;
  */
    bridgeV = bv1;
    bridgeAvg += (fabs(bridgeV) - bridgeAvg) / 10.0f;

  /*
    if(fabs(bridgeV)-fabs(bridgeAvg)>bridgeAvgDiff){
      bridgeAvgDiff=bridgeAvgDiff-((fabs(bridgeV)-fabs(bridgeAvg))/10);
    }else{
      bridgeAvgDiff=bridgeAvgDiff+((fabs(bridgeV)-fabs(bridgeAvg))/10);
    }
  */

  // ── Classification ──
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
    // Exactly equal - hold previous state
    if (prevClosed) vClosed  = true;
    else            vFloating = true;
  }

  delay(1); //this allows settling before we resume  

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
//  DIAGNOSTIC: streaming, capture, command handling
// ==================================================================
void printStatus() {
  Serial.print("$STATUS,diag=");  Serial.print(diagMode ? 1 : 0);
  Serial.print(",vmode=");        Serial.print((int)voltOverride);
  Serial.print(",mosfet=");       Serial.print(mosfetHold);
  Serial.print(",stream=");       Serial.print(streamOn ? 1 : 0);
  Serial.print(",rate=");         Serial.print(streamIntervalMs);
  Serial.print(",capms=");        Serial.print(capDurationMs);
#ifdef USE_ADS1115
  Serial.print(",res=");          Serial.print(16);
#else
  Serial.print(",res=");          Serial.print(12);
#endif
  Serial.print(",vref=");         Serial.println(4.096, 3);  // GAIN_ONE full-scale
}

// One streamed sample: the ADC's internal differential (A0-A1) only.
void streamSample() {
  ads.setGain(GAIN_TWO);                 // fixed gain -> constant host scaling
  int16_t rawDif = ads.readADC_Differential_0_1();
  float dv = rawDif * GAIN_FACTOR_2 / 1000.0f;
  Serial.print("$DIAG,");
  Serial.print(millis()); Serial.print(",");
  Serial.print(rawDif);   Serial.print(",");
  Serial.println(dv, 4);
}

// Capture the ADC differential (A0-A1) across the bridge-MOSFET toggle.
// Holds the bridge OFF (resting voltage-measure state), samples a baseline,
// engages the bridge (BRIDGE_ON, the lead-test edge) at CAP_PRE_US, keeps
// sampling until durationMs elapses or the buffer fills, then restores OFF.
// Fixed gain (GAIN_ONE) so the host's count/1000*gain conversion is constant.
void runCapture(unsigned long durationMs) {
  ads.setGain(GAIN_TWO);
  ads.setDataRate(ADS_RATE_FAST);
  digitalWrite(VbridgePin, BRIDGE_OFF);  // resting (bridge disconnected)
  delay(2);

  // Start ONE continuous conversion stream on the A0-A1 differential. In
  // continuous mode each loop iteration is a single conversion-register read
  // (getLastConversionResults) instead of the single-shot sequence of
  // start-conversion + busy-poll + read that readADC_Differential_0_1() does.
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);

  capCount = 0;
  unsigned long durUs    = durationMs * 1000UL;
  unsigned long toggleUs = 0;
  bool toggled = false;
  unsigned long t0 = micros();
  unsigned long nextSampleUs = 0;
  // ADS1015 @ 3300 SPS converts every ~303 us. Pace the loop to that so every
  // stored sample is a fresh conversion rather than a re-read of the same one.
  const unsigned long sampleStepUs = 305;

  while (capCount < CAP_MAX_SAMPLES) {
    unsigned long t = micros() - t0;
    if (!toggled && t >= CAP_PRE_US) {
      digitalWrite(VbridgePin, BRIDGE_ON);   // engage bridge (lead-test edge)
      toggleUs = micros() - t0;
      toggled = true;
    }
    if (t >= durUs) break;
    if (t < nextSampleUs) continue;          // pace to the conversion rate
    nextSampleUs = t + sampleStepUs;
    capT[capCount]    = t;
    capDiff[capCount] = ads.getLastConversionResults();
    capCount++;
  }

  digitalWrite(VbridgePin, BRIDGE_OFF);  // restore resting state

  Serial.print("$CAPSTART,");
  Serial.print(capCount);      Serial.print(",");
  Serial.print(toggleUs);      Serial.print(",");
  Serial.print(durationMs);    Serial.print(",");
  Serial.print(1000.0, 0);     Serial.print(",");      // fullScale
  Serial.println(GAIN_FACTOR_2, 6);                    // vref = gain factor (mV/bit)
  for (int i = 0; i < capCount; i++) {
    Serial.print("$CAP,");
    Serial.print(capT[i]);    Serial.print(",");
    Serial.println(capDiff[i]);
  }
  Serial.println("$CAPEND");

  ads.setDataRate(ADS_RATE_MID);         // restore a sane rate for detection
}

// Parse one received command line.  '!' lines are diagnostic commands;
// any other line is treated as a sequence of legacy single-char commands.
void handleLine(char *line) {
  if (line[0] != '!') {
    for (char *p = line; *p; ++p) handleSerialCommands(*p);
    return;
  }
  char *cmd = line + 1;
  char *arg = strchr(cmd, ',');
  if (arg) { *arg = '\0'; arg++; }
  for (char *p = cmd; *p; ++p) *p = toupper(*p);

  if (strcmp(cmd, "DIAG") == 0) {
    diagMode = arg ? (atoi(arg) != 0) : !diagMode;
    if (!diagMode) { streamOn = false; mosfetHold = -1; }
    printStatus();
  } else if (strcmp(cmd, "STREAM") == 0) {
    streamOn = arg ? (atoi(arg) != 0) : !streamOn;
    printStatus();
  } else if (strcmp(cmd, "RATE") == 0) {
    if (arg) { long r = atol(arg); streamIntervalMs = (r < 1) ? 1 : r; }
    printStatus();
  } else if (strcmp(cmd, "VMODE") == 0) {
    int v = arg ? atoi(arg) : 0;
    voltOverride = (VoltOverride)constrain(v, 0, 2);
    printStatus();
  } else if (strcmp(cmd, "MOSFET") == 0) {
    int m = arg ? atoi(arg) : -1;
    mosfetHold = (m < 0) ? -1 : (m ? 1 : 0);
    printStatus();
  } else if (strcmp(cmd, "CAP") == 0) {
    unsigned long d = arg ? atol(arg) : capDurationMs;
    if (d < 1) d = 1;
    capDurationMs = d;
    runCapture(d);
  } else if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "?") == 0) {
    printStatus();
  } else {
    Serial.print("$ERR,unknown,"); Serial.println(cmd);
  }
}

// Accumulate serial bytes into cmdBuf; dispatch on newline.
void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) { cmdBuf[cmdLen] = '\0'; handleLine(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// Baseline bridge voltage at the lowest characterized point (~10K).  The
// confidence percent is expressed relative to this baseline, so a near-short
// (<10K) reads ~0% and the open/closed threshold (vClosedThres, ~330K) reads
// 100%.  Must match the ~10K entry (kBinV[0]) in bridgeResistanceLabel().
const float BRIDGE_BASELINE_V = 1.262f;

// Confidence percent derived from the smoothed bridge voltage (see above).
float bridgeConfidencePct() {
  float span = vClosedThres - BRIDGE_BASELINE_V;
  float pct  = (span > 0.0f)
               ? ((fabs(bridgeAvg) - BRIDGE_BASELINE_V) / span) * 100.0f
               : 0.0f;
  return (pct < 0.0f) ? 0.0f : pct;
}

// ==================================================================
//  BRIDGE RESISTANCE ESTIMATE
//  Maps a bridge voltage reading (as shown on the "Bridge:" line) to
//  an estimated resistance bin using nearest-neighbour matching against
//  the measured calibration points below.  Readings below the 10K point
//  read "<10K"; readings above the 11.2M point read ">11.2M".
// ==================================================================
const char* bridgeResistanceLabel(float v) {
  v = fabs(v);
  if (v < 1.34f) return "<33K";
  if (v > 1.706f)  return ">10M";

/*Red
  static const float kBinV[]    = { 1.273f, 1.288f,  1.339f,  1.431f, 1.578f, 1.633f  };
  static const char* kBinName[] = { "~33K", "~100K", "~330K", "~1M", "~10M", "~50M" };
*/
  static const float kBinV[]    = { 1.35f, 1.37f,  1.425f,  1.521f, 1.679f};
  static const char* kBinName[] = { "~33K", "~100K", "~330K", "~1M", "~10M"};

  const int n = sizeof(kBinV) / sizeof(kBinV[0]);

  int   best     = 0;
  float bestDiff = fabs(v - kBinV[0]);
  for (int i = 1; i < n; i++) {
    float d = fabs(v - kBinV[i]);
    if (d < bestDiff) { bestDiff = d; best = i; }
  }
  return kBinName[best];
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
    Serial.print(" / R:");       Serial.print(bridgeResistanceLabel(bridgeAvg));
    Serial.print(" / Conf:");    Serial.print((int)bridgeConfidencePct());
    Serial.print("%");
    Serial.print(" / Avg Diff");         Serial.print(bridgeAvgDiff);
  } else {
    Serial.print(" / V !0");
  }

  Serial.print(" / DO:");
  Serial.print((statusCode & 0x2) ? '1' : '0');
  Serial.print((statusCode & 0x1) ? '1' : '0');
  if (cfSuppress) Serial.print(" (VOLT ONLY)");

  Serial.print(acMode ? " / VRMS:" : " / VDC:");
  Serial.print(displayVoltage, 4);
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
    case '?':
      Serial.println("Commands: S(Stop) G(Go) U(Updates) M(Manual) R(Range) D(Debug)");
      break;
    default: break;
  }
}
