/*
 * microDMM -- Seeed XIAO RA4M1 pocket multimeter, PCB v5.
 *
 * Resistance, DC and AC voltage, and current on an ADS1115, an SSD1306 OLED,
 * two buttons, a buzzer/LED, and USB HID for typing readings into a
 * spreadsheet.  Boards from revision 5 also carry a bridge MOSFET across the
 * inputs that tells an open test lead from one closed onto a dead circuit.
 *
 * ── Layout ────────────────────────────────────────────────────────
 *   this file      pins, shared state, setup(), loop()
 *   Config.ino     EEPROM config, the key table, legacy calibration
 *   Measure.ino    ADS1115 acquisition and the bridge test
 *   Display.ino    OLED rendering and the value formatters
 *   Alerts.ino     buzzer / LED
 *   SerialCmd.ino  the !CMD / $LINE host protocol
 *   Util.ino       buttons, min/max, the current log, small helpers
 *   DMMTypes.h     shared types -- see the note at the top of that file for
 *                  why they cannot live in an .ino
 *
 * ── Configuration ─────────────────────────────────────────────────
 * Everything tunable is an EEPROM key, editable over serial and never
 * recompiled: !CFG dumps them, !SET,<key>,<value> changes one, !SAVE persists.
 * The six hand-tuned per-unit calibration sets that used to be a compiled-in
 * if/else ladder are seeded automatically on first boot from the unit id in
 * EEPROM byte 1, and can be re-applied with !SEEDCAL,<n>.
 *
 * ── Modes ─────────────────────────────────────────────────────────
 * MODE_BUTTON cycles the Mode enum (see DMMTypes.h).  Default auto-selects
 * voltage or resistance from what is connected; the rest force a behaviour.
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Keyboard.h>
#include <EEPROM.h>
#include "DMMTypes.h"

// ==================================================================
//  HARDWARE
// ==================================================================
Adafruit_ADS1115 ads;

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int logPin          = 1;   // pull low to arm the current log
const int TYPE_PIN        = 3;   // action button: type / set reference / reset
const int CONTINUITY_PIN  = 6;   // buzzer or alert LED
const int SETRANGE_PIN    = 7;   // high / low resistance range select
const int VbridgePin      = 8;   // bridge MOSFET for the open-lead test
const int OHMPWMPIN       = 9;   // ohms source PWM, parked high in power save
#define   MODE_BUTTON     10     // cycles the mode

// Battery sense.  BAT_DET_PIN / BAT_READ_EN are the XIAO RA4M1 variant's own
// names for the divider and its enable; the enable must be driven high before
// a reading means anything.
#define   BATT_PIN        BAT_DET_PIN
#define   enablePin       BAT_READ_EN

// ADS1115 mV per bit at each gain setting.
const float GAIN_FACTOR_TWOTHIRDS = 0.1875f;    // +/-6.144 V
const float GAIN_FACTOR_1         = 0.125f;     // +/-4.096 V
const float GAIN_FACTOR_2         = 0.0625f;    // +/-2.048 V
const float GAIN_FACTOR_4         = 0.03125f;   // +/-1.024 V
const float GAIN_FACTOR_8         = 0.015625f;  // +/-0.512 V
const float GAIN_FACTOR_16        = 0.0078125f; // +/-0.256 V

// ==================================================================
//  SHARED STATE
// ==================================================================
// All state reachable from more than one tab lives here.  The Arduino
// preprocessor concatenates this file first and the other tabs after it in
// name order, so anything defined in a later tab is invisible to setup() and
// loop() -- keeping the shared globals here avoids needing extern for each.

// ---- Mode ----
Mode currentMode = Default;
#define DEBOUNCE_DELAY 50
unsigned long lastDebounceTime      = 0;
bool          buttonPreviouslyPressed = false;

// ---- Scheduling ----
unsigned long previousAdcMillis  = 0;
unsigned long previousBattMillis = 0;
unsigned long previousLcdMillis  = 0;
unsigned long deepSleepStart     = 0;
unsigned long lcdInterval        = 1000;   // live value; reset from cfg each refresh
unsigned int  seconds            = 0;      // written by formatTime(); see Util.ino

// ---- Raw ADC ----
int16_t  adcCount           = 0;   // resistance channel counts
int16_t  countV             = 0;   // voltage differential counts
int32_t  countI             = 0;   // current channel counts
int16_t  adcReadingCurrent  = 0;   // boot-time current baseline
uint8_t  gainIndex          = 0;   // resistance channel gain index

// ---- Voltage ----
float newVoltageReading = 0.0f;
float averageVoltage    = 0.0f;
float VAC               = 0.0f;
float medianVoltage     = 0.0f;
float medianVoltageStep = 0.0f;

float voltageSamples[VOLT_SAMPLE_MAX]        = {0.0f};
float voltageSquaredSamples[VOLT_SAMPLE_MAX] = {0.0f};
int   voltageSampleIndex = 0;
float voltageSum         = 0.0f;
float squaredVoltageSum  = 0.0f;

// ---- Resistance ----
float rawResistance        = 0.0f;
float calibratedResistance = 0.0f;
float currentResistance    = 0.0f;
float displayResistance    = 0.0f;
float zeroOffsetRes        = 0.0f;   // lead null, set by !ZERO or auto at boot
float ohmsVoltage          = 0.0f;
// The reference actually in force.  Normally cfg.zenerMaxV; drops to
// cfg.sleepV while power save has the ohms source parked.
float zenerActiveV         = 5.0f;

// ---- Current ----
#define I_HIGH_RESET (-6.0f)
#define I_LOW_RESET  ( 6.0f)
float currentShuntVoltage = 0.0f;
float Ireading            = 0.0f;
bool  Irange              = false;   // false = low range, true = high range
bool  currentOnOff        = false;   // a sensor was detected at boot
float IHigh               = I_HIGH_RESET;
float ILow                = I_LOW_RESET;

// ---- Battery ----
float batteryVoltage = 0.0f;

// ---- Min / max ----
#define TIME_BUF_LEN 6               // "MM:SS" plus terminator
float highV = -100.0f, lowV = 100.0f;
float highR = 0.0f,    lowR = 3000000.0f;
char  timeAtMaxV[TIME_BUF_LEN] = "", timeAtMinV[TIME_BUF_LEN] = "";
char  timeAtMaxI[TIME_BUF_LEN] = "", timeAtMinI[TIME_BUF_LEN] = "";
float voltageAtMaxI = 0.0f, voltageAtMinI = 0.0f;
float currentAtMaxV = 0.0f, currentAtMinV = 0.0f;

// ---- Formatted display values ----
float       roundedV = 0.0f, roundedVlow = 0.0f, roundedVhigh = 0.0f;
int         vDigits  = 0,    vDigitslow  = 0,    vDigitshigh  = 0;
const char *vSuffix  = "",  *vSuffixlow  = "",  *vSuffixhigh  = "";
float       roundedR = 0.0f, roundedRlow = 0.0f, roundedRhigh = 0.0f;
int         rDigits  = 0,    rDigitslow  = 0,    rDigitshigh  = 0;
const char *rSuffix  = "",  *rSuffixlow  = "",  *rSuffixhigh  = "";

// ---- Flags ----
bool ohmsHighRange    = true;
bool ohmsAutoRange    = true;
bool voltageDisplay   = false;
bool MinMaxDisplay    = false;
bool screenRefreshFast = false;
bool ampsMode         = false;
bool debugMode        = false;
bool initialZeroSet   = false;
bool serialMode       = false;   // $LIVE streaming
bool powerSave        = false;
bool timeHighset      = false;
bool altUnits         = false;
bool preciseMode      = false;
bool deepSleepTrigger = false;
bool screenSleep      = false;
bool continuity       = false;
bool vFlag            = false;   // voltage alert is sounding
bool rFlag            = false;   // continuity alert is sounding
bool VACPresense      = false;
bool buttonPressed    = false;
unsigned long buttonPressTime = 0;
unsigned long timeHigh        = 0;   // when the ohms rail first pegged

// ---- Delta reference ----
float deltaV       = 0.0f;
int   deltaVdigits = 0;

// ---- Bridge test ----
bool  vFloating = false;
float bridgeV   = 0.0f;
bool  Vzero     = true;

// ---- Current log ----
const int LOG_SIZE     = 120;
const int SAMPLE_COUNT = 120;
bool  takeLog      = false;
int   samplesTaken = 0;
float tLogStart    = 0.0f;
float tLogEnd      = 0.0f;
float loggedCurrents[LOG_SIZE]    = {0.0f};
float loggedVoltagesAtI[LOG_SIZE] = {0.0f};
float loggedTimeStamps[LOG_SIZE]  = {0.0f};

// Alert pulses issued since the last screen refresh, so a fast loop cannot
// machine-gun the buzzer between updates.
int blinkLimit = 0;

// ==================================================================
//  SETUP
// ==================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Keyboard.begin();

  pinMode(TYPE_PIN,       INPUT_PULLUP);
  pinMode(MODE_BUTTON,    INPUT_PULLUP);
  pinMode(logPin,         INPUT_PULLUP);
  pinMode(BATT_PIN,       INPUT);
  pinMode(CONTINUITY_PIN, OUTPUT);
  pinMode(SETRANGE_PIN,   OUTPUT);
  pinMode(OHMPWMPIN,      OUTPUT);
  pinMode(VbridgePin,     OUTPUT);
  pinMode(enablePin,      OUTPUT);
  digitalWrite(enablePin, HIGH);        // enable the battery divider

  // Config before anything reads a calibration constant.
  configSetup();
  zenerActiveV = cfg.zenerMaxV;
  resetVoltageFilter();

  // ---- OLED ----
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("$ERR,boot,OLED init failed"));
    analogWrite(CONTINUITY_PIN, 10);    // audible cue with no screen to use
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("uMeter #");
  display.print(cfg.hwRev);
  display.setCursor(0, 48);
  display.println("XIAO RA4M1");
  display.display();

  Serial.print(F("$INFO,boot,uMeter,"));
  Serial.print(cfg.hwRev);
  Serial.print(',');
  Serial.println(unitSN);
  delay(200);

  // ---- ADS1115 ----
  if (!ads.begin()) {
    Serial.println(F("$ERR,boot,ADS1115 not found"));
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("ADS1115 not found");
    display.display();
    while (1);
  }
  ads.setDataRate(RATE_ADS1115_16SPS);  // resistance is the resting mode
  delay(300);

  // ---- Current sensor detection ----
  // Once, here, and never again while running: if nothing is fitted the
  // channel stays suppressed until the next reboot.  See detectCurrentSensor
  // in Measure.ino for what it looks at.
  detectCurrentSensor();

  display.clearDisplay();
  Serial.println(F("$INFO,boot,ready"));
}

// ==================================================================
//  LOOP
// ==================================================================
void loop() {
  unsigned long currentMillis = millis();

  checkModeButton();

  // Modes that are simply a pair of flags elsewhere.
  altUnits    = (currentMode == AltUnitsMode);
  preciseMode = (currentMode == Low);
  // Typing and high-resistance work have no use for a delta reference, and a
  // stale one would be typed into a spreadsheet alongside the reading.
  if (currentMode == Type || currentMode == HighRMode) deltaV = 0.0f;

  if (digitalRead(logPin) == 0) takeLog = true;

  if (takeLog) {
    if (samplesTaken < SAMPLE_COUNT + 1) {
      logCurrentData(newVoltageReading, millis() / 1000.0f, Ireading);
      samplesTaken++;
    } else {
      takeLog      = false;
      tLogEnd      = millis() / 1000.0f;
      samplesTaken = 0;
      analogWrite(CONTINUITY_PIN, 100);      // audible "log finished"
      Serial.println(F("$INFO,log,complete"));
    }
  }

  handleButtonInput();

  if (currentMillis - previousBattMillis >= cfg.battMs) {
    previousBattMillis = currentMillis;
    int raw = analogRead(BATT_PIN);
    batteryVoltage = raw * (3.3f / 1023.0f) * cfg.battScale;
  }

  if (currentMillis - previousAdcMillis >= cfg.adcMs) {
    previousAdcMillis = currentMillis;

    if (!takeLog) measureResistance();     // logging wants the fastest V/I rate
    if (currentMode != HighRMode) {
      measureVoltage();
      measureCurrent();
    }

    // ---- Power save ----
    // The ohms source is the meter's biggest continuous draw.  Once the rail
    // has sat pegged (nothing connected) for psHoldMs, park it and watch for
    // the rail to sag, which means a resistance is across the leads again.
    if (!powerSave) {
      if (ohmsVoltage > cfg.zenerMaxV - cfg.psMargin && !timeHighset &&
          currentMode != HighRMode) {
        timeHigh    = millis();
        timeHighset = true;
        if (cfg.psDebug) Serial.println(F("$PS,armed"));
      }
      if (timeHighset && currentResistance < cfg.psCancelR) {
        timeHighset = false;
        if (cfg.psDebug) Serial.println(F("$PS,cancel"));
      }
      if (timeHighset && (millis() - timeHigh) > cfg.psHoldMs) {
        powerSave = true;
        if (cfg.psDebug) Serial.println(F("$PS,start"));
      }
    }

    if (powerSave || currentMode == Charging) {
      analogWrite(OHMPWMPIN, cfg.psPwm);
      if (ohmsVoltage < cfg.sleepV - cfg.psHyst) {
        powerSave   = false;
        timeHighset = false;
        if (cfg.psDebug) {
          Serial.print(F("$PS,end,"));
          Serial.println(ohmsVoltage, 3);
        }
        analogWrite(OHMPWMPIN, 0);
      }
    }

    // ---- Which measurement to show ----
    // Voltage wins if there is any; resistance takes over once the reading
    // falls into a plausible ohms window.  HighRMode pins it to resistance.
    if ((abs(countV) > (int)cfg.vDispCount || currentMode == Voltmeter) &&
        currentMode != HighRMode) {
      voltageDisplay = true;
      ads.setDataRate(preciseMode ? RATE_ADS1115_16SPS : RATE_ADS1115_860SPS);
    }
    if ((isBetween(currentResistance, cfg.rDispMin, cfg.rDispMax) &&
         currentMode != Voltmeter) || currentMode == HighRMode) {
      voltageDisplay = false;
      ads.setDataRate(RATE_ADS1115_32SPS);
    }

    displayResistance = currentResistance - zeroOffsetRes;

    // Track resistance extremes only once the lead null has been settled --
    // before that every reading is offset by the leads themselves.
    if (initialZeroSet) {
      if (displayResistance > highR && displayResistance < cfg.mmRMax)
        highR = displayResistance;
      if (displayResistance < lowR && displayResistance > cfg.mmRMin)
        lowR = displayResistance;
    }
  }

  serialPoll();
  serialStreamTick(currentMillis);

  updateAlerts();

  if ((currentMillis - previousLcdMillis >= lcdInterval) && !takeLog) {
    previousLcdMillis = currentMillis;
    // Reset the period here rather than where it was bumped: measureVoltage
    // and measureResistance shorten it to lcdBumpMs for one refresh when a
    // reading jumps, and it has to come back on its own afterwards.
    lcdInterval = screenRefreshFast ? cfg.lcdFastMs : cfg.lcdMs;
    updateDisplay();
  }

  // ---- Screen saver ----
  // Only once the meter is in power save AND genuinely idle: no voltage, no
  // AC, no current.  Any activity resets the countdown.
  if (powerSave && fabs(newVoltageReading) < cfg.sleepVMax &&
      !VACPresense && Ireading == 0.0f) {
    if (!deepSleepTrigger) {
      deepSleepStart   = millis();
      deepSleepTrigger = true;
    } else if (millis() - deepSleepStart > (unsigned long)cfg.sleepSec * 1000UL) {
      screenSleep = true;
    }
  } else {
    screenSleep      = false;
    deepSleepTrigger = false;
  }
}
