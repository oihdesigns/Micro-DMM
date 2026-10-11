/*
 * GigaMeter -- Arduino GIGA R1 + GIGA Display multimeter, SMD V5 board.
 *
 * Resistance, DC and AC voltage, and current on an ADS1115 (Wire1), an
 * 800x480 touch display, a mode button, a type/ref button, a mic-jack button
 * ladder on A7, a buzzer/LED, the RGB LED, USB HID typing, and USB-stick CSV
 * logging.  The M4 core (microDMM_Giga_M4) makes the audio, steered over RPC
 * and by the buzzer pin.
 *
 * ONE BUILD FOR EVERY UNIT.  Calibration is no longer compiled in: each
 * meter keeps its own config in QSPI flash.  A unit with nothing stored runs
 * on defaults (unit 1's calibration) and reports HWREV 0 until assigned.
 *
 * ── Layout ────────────────────────────────────────────────────────
 *   this file      pins, shared state, setup(), loop()
 *   Config.ino     flash config, the key table, per-unit starting points
 *   Measure.ino    ADS1115 acquisition and the bridge test
 *   Display.ino    screen rendering, touch buttons, plots, formatters
 *   Alerts.ino     buzzer / RGB LED / M4 flags
 *   SerialCmd.ino  the !CMD / $LINE host protocol (same as the XIAO rewrite,
 *                  so microdmm_gui.py drives either meter)
 *   Util.ino       buttons, mic ladder, min/max, logs, USB-stick CSV
 *   DMMTypes.h     shared types
 *
 * ── Bringing up a unit ────────────────────────────────────────────
 *   !SEEDCAL,<n>   load that unit's compiled-in starting point (RAM only)
 *   !SET,K,V       adjust any key; !CAL / !CALV / !CALI capture against a reference
 *   !SAVE          persist (read back and verified)
 *   !SN,<value>    write the serial number (persists immediately)
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino_GigaDisplay.h>
#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>
#include "PluggableUSBHID.h"
#include "USBKeyboard.h"
#include <Arduino_USBHostMbed5.h>
#include <DigitalOut.h>
#include <FATFileSystem.h>
#include "BlockDevice.h"
#include "RPC.h"
#include <ArduinoBLE.h>
#include "DMMTypes.h"

// ==================================================================
//  HOST OUTPUT
// ==================================================================
// Every host-facing line goes through Host, which writes it to USB serial and,
// while a BLE client is subscribed, to the BLE UART as well (Ble.ino).  Use
// Host.print where you would have used Serial.print.  Declared here because
// this tab is concatenated first and every other tab prints.
class HostOut : public Print {
public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buf, size_t n) override;
  using Print::write;
};
HostOut Host;

// ==================================================================
//  HARDWARE
// ==================================================================
Adafruit_ADS1115         ads;
GigaDisplay_GFX          display;
Arduino_GigaDisplayTouch touch;
GigaDisplayRGB           rgb;
USBKeyboard              Keyboard;

USBHostMSD          msd;
mbed::FATFileSystem usb("usb");

const int OHMPWMPIN      = 2;    // ohms source PWM, parked high when unused
const int TYPE_PIN       = 3;    // type / ref button (held at boot = flashlight)
#define   BUTTON_PIN       4     // mode button
const int VbridgePin     = 5;    // bridge MOSFET for the open-lead test
const int CONTINUITY_PIN = 6;    // buzzer / LED; the M4 also watches it
const int SETRANGE_PIN   = 7;    // high / low resistance range select
const int cycleTrack     = 52;   // toggles every loop, for timing on a scope
const int backlightRef         = 64;
const int backlightControlHigh = 65;
const int backlightControlLow  = 63;
const int logButton      = A1;   // pull low to take a manual log
const int micPin         = A7;   // mic-jack button ladder -- idle should read HIGH

#define BLACK  0x0000
#define WHITE  0xFFFF
#define RED    0xF800
#define GREEN  0x07E0
#define BLUE   0x001F
#define YELLOW 0xFFE0

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
// ---- Mode ----
Mode currentMode = Voltmeter;
#define DEBOUNCE_DELAY 300
unsigned long lastDebounceTime        = 0;
bool          buttonPreviouslyPressed = false;

// ---- Inputs ----
bool redPress = false, greenPress = false, bluePress = false, yellowPress = false;
int  micLevel = 1023;                 // analogRead(micPin), once per loop

// ---- Scheduling ----
unsigned long previousAdcMillis = 0;
unsigned long previousLcdMillis = 0;
unsigned long deepSleepStart    = 0;
unsigned long lcdInterval       = 1000;   // live value; reset from cfg each refresh
unsigned int  secondsTime           = 0;      // written by formatTime(); see Util.ino

// ---- Raw ADC ----
int16_t  adcCount          = 0;
int16_t  countV            = 0;
int32_t  countI            = 0;
int16_t  adcReadingCurrent = 0;
uint8_t  gainIndex         = 0;

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
float currentResistance    = 1000001.0f;
float displayResistance    = 0.0f;
float zeroOffsetRes        = 0.0f;
float ohmsVoltage          = 0.0f;
float ohmsVoltageRaw       = 0.0f;   // before the reference clamp
float zenerActiveV         = 5.0f;   // cfg.zenerMaxV, or cfg.sleepV while parked
// At or past the reference: open leads, or more than the divider can resolve.
bool  ohmsOpen             = false;
const int NUM_RESISTANCE_SAMPLES = 100;
float resistanceSamples[NUM_RESISTANCE_SAMPLES] = {0.0f};   // ohms rail, for the plot
int   resistanceSampleIndex = 0;

// ---- Current ----
#define I_HIGH_RESET (-6.0f)
#define I_LOW_RESET  ( 6.0f)
float currentShuntVoltage = 0.0f;
float Ireading            = 0.0f;
bool  Irange              = false;
bool  currentOnOff        = false;
float IHigh               = I_HIGH_RESET;
float ILow                = I_LOW_RESET;

// No battery sense on the Giga; kept so $LIVE has the same shape as the XIAO's.
float batteryVoltage = 0.0f;

// ---- Min / max ----
#define TIME_BUF_LEN 12           // "MM:SS", room for minutes past 99
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
bool ohmsHighRange      = true;
bool ohmsAutoRange      = true;
bool voltageDisplay     = false;
bool resistanceMeasured = false;  // this pass read the ohms channel, settled
bool ohmsParked         = false;  // the ohms source is switched off
bool forceLowPower      = false;  // !LOWPWR: hold power save for bench work
unsigned long ohmsUnparkMs = 0;
#define OHMS_UNPARK_SETTLE_MS 25UL
bool MinMaxDisplay      = false;
bool screenRefreshFast  = false;
bool ampsMode           = false;
bool debugMode          = false;
bool initialZeroSet     = false;
bool serialMode         = false;  // $LIVE streaming
bool powerSave          = false;
bool timeHighset        = false;
bool altUnits           = false;
bool preciseMode        = false;
bool deepSleepTrigger   = false;
bool screenSleep        = false;
bool continuity         = false;
bool vFlag              = false;
bool rFlag              = false;
bool VACPresense        = false;
bool flashlightMode     = false;  // TYPE_PIN held at boot
bool buttonPressed      = false;
unsigned long buttonPressTime = 0;
unsigned long timeHigh        = 0;

// ---- Delta reference ----
float deltaV       = 0.0f;
int   deltaVdigits = 0;

// ---- Bridge test ----
bool  vFloating   = false;
bool  vUndefined  = true;
bool  vClosed     = false;
float bridgeV     = 0.0f;
bool  Vzero       = true;

// ---- RPC to the M4 ----
bool    txFlag   = false;     // tone pitch: false = continuity, true = voltage
bool    humFlag  = false;     // a resistance is across the leads
uint8_t lastSent = 0xFF;

// ---- Logs ----
const int LOG_SIZE     = 100;
const int SAMPLE_COUNT = 100;
bool  takeLog      = false;
int   samplesTaken = 0;
float tLogStart    = 0.0f;
float tLogEnd      = 0.0f;
float loggedCurrents[LOG_SIZE]    = {0.0f};
float loggedVoltagesAtI[LOG_SIZE] = {0.0f};
float loggedTimeStamps[LOG_SIZE]  = {0.0f};
bool  logMode = false;            // a USB stick is mounted
int   autologCount   = 0;
int   manuallogCount = 0;

int blinkLimit = 0;

// ==================================================================
//  SETUP
// ==================================================================
void setup() {
  Wire1.begin();                  // ADS1115 on SDA1/SCL1
  Wire1.setClock(400000);

  Serial.begin(115200);
  delay(100);

  display.begin();
  display.setRotation(1);
  display.fillScreen(BLUE);
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.setCursor(0, 0);
  display.println("GIGA METER");

  // Config before anything reads a calibration constant.
  configSetup();
  zenerActiveV = cfg.zenerMaxV;
  resetVoltageFilter();

  display.setTextSize(3);
  if (cfg.hwRev) {
    display.print("Unit #");
    display.println(cfg.hwRev);
  } else {
    display.setTextColor(YELLOW);
    display.println("UNIT NOT ASSIGNED");
    display.setTextSize(2);
    display.println("Send !SEEDCAL,<n> then !SAVE");
    display.setTextColor(WHITE);
  }
  display.setTextSize(2);
  if (unitSN[0]) { display.print("SN "); display.println(unitSN); }
  if (!flashAvailable()) display.println("Config flash unavailable");
  Host.print(F("$INFO,boot,GigaMeter,"));
  Host.print(cfg.hwRev);
  Host.print(',');
  Host.println(unitSN);

  if (!touch.begin()) {
    Host.println(F("$ERR,boot,touch init failed"));
    display.println("Touch init failed");
    while (1) {}
  }

  // ---- USB stick ----
  pinMode(PA_15, OUTPUT);
  digitalWrite(PA_15, HIGH);            // enable the USB-A port
  mbed::DigitalOut vbusEnable(PB_8, 1);
  display.println("Waiting for USB drive");
  for (int i = 0; i < 2 && !msd.connect(); i++) delay(200);
  if (msd.connect()) {
    int err = usb.mount(&msd);
    if (err) {
      Host.print(F("$ERR,boot,USB mount failed,")); Host.println(err);
      display.print("USB mount failed: "); display.println(err);
      while (true);
    }
    logMode = true;
    Host.println(F("$INFO,boot,usb,mounted"));
    display.println("USB drive mounted");
  } else {
    logMode = false;
    Host.println(F("$INFO,boot,usb,none"));
    display.println("No USB drive");
  }

  pinMode(TYPE_PIN,       INPUT_PULLUP);
  pinMode(BUTTON_PIN,     INPUT_PULLUP);
  pinMode(CONTINUITY_PIN, OUTPUT);
  pinMode(SETRANGE_PIN,   OUTPUT);
  pinMode(OHMPWMPIN,      OUTPUT);
  pinMode(VbridgePin,     OUTPUT);
  pinMode(logButton,      INPUT_PULLUP);
  pinMode(cycleTrack,     OUTPUT);
  pinMode(micPin,         INPUT);
  pinMode(backlightRef,         OUTPUT);
  pinMode(backlightControlHigh, INPUT_PULLUP);
  pinMode(backlightControlLow,  INPUT_PULLUP);

  flashlightMode = (digitalRead(TYPE_PIN) == LOW);

  // ---- ADS1115 ----
  if (!ads.begin(0x48, &Wire1)) {
    Host.println(F("$ERR,boot,ADS1115 not found"));
    display.println("ADS1115 not found");
    while (1);
  }
  display.println("ADS1115 found");
  ads.setDataRate(RATE_ADS1115_16SPS);
  delay(100);

  // ---- Current sensor detection (once per boot) ----
  detectCurrentSensor();
  display.print("Ammeter: ");
  display.println(currentOnOff ? (Irange ? "hall sensor" : "shunt") : "none");

  // ---- M4 core ----
  display.println("Booting Core 2");
  RPC.begin();
  uint8_t pack = (txFlag ? 1 : 0) | (humFlag ? 2 : 0);
  RPC.write(pack);
  lastSent = pack;

  // ---- Bluetooth ----
  display.println("Starting Bluetooth");
  bleSetup();
  display.print("Bluetooth: ");
  display.println(bleState() ? bleDeviceName() : "unavailable");

  rgb.begin();
  delay(1000);
  display.fillScreen(BLACK);
  drawButtons();

  emitHelp();
  Host.println(F("$INFO,boot,ready"));
}

// ==================================================================
//  LOOP
// ==================================================================
void loop() {
  unsigned long currentMillis = millis();

  static bool cycleTracking = false;
  cycleTracking = !cycleTracking;
  digitalWrite(cycleTrack, cycleTracking ? HIGH : LOW);

  pollTouch(currentMillis);
  pollMicLadder();
  checkModeButton();

  altUnits    = (currentMode == AltUnitsMode);
  preciseMode = (currentMode == Low);
  if (currentMode == Type) deltaV = 0.0f;

  if (digitalRead(logButton) == 0 || greenPress) takeLog = true;
  serviceManualLog();

  handleButtonInput();

  if (currentMillis - previousAdcMillis >= cfg.adcMs) {
    previousAdcMillis = currentMillis;

    // Does this mode use the ohms channel?  Voltmeter/VAC only want the input,
    // logging wants the V/I rate, and Charging has nothing reading anything.
    bool ohmsMode = !takeLog &&
                    currentMode != Voltmeter &&
                    currentMode != VACmanual &&
                    currentMode != Charging;
    // A reading taken while the source is still coming back up is a transient.
    bool rFresh = ohmsMode && (millis() - ohmsUnparkMs >= OHMS_UNPARK_SETTLE_MS);
    resistanceMeasured = rFresh;

    // Free-running conversion only pays off on a single channel whose config
    // is not rewritten every pass -- in practice Voltmeter/VAC/logging with no
    // ammeter fitted.  See ADS1115 ACCESS in Measure.ino.
    adsPlanPass((uint8_t)ohmsMode + 1 + (uint8_t)currentOnOff, ohmsMode);

    if (ohmsMode) measureResistance();
    measureVoltage();
    measureCurrent();                    // self-suppresses with no sensor

    // ---- Power save ----
    // Stepped only on a fresh ohms reading: ohmsVoltage is stale in modes that
    // skip the channel.
    if (rFresh) {
      if (forceLowPower) {
        powerSave   = true;
        timeHighset = false;
      } else if (!powerSave) {
        if (ohmsVoltage > cfg.zenerMaxV - cfg.psMargin && !timeHighset &&
            currentMode != HighRMode) {
          timeHigh    = millis();
          timeHighset = true;
          if (cfg.psDebug) Host.println(F("$PS,armed"));
        }
        if (timeHighset && currentResistance < cfg.psCancelR) {
          timeHighset = false;
          if (cfg.psDebug) Host.println(F("$PS,cancel"));
        }
        if (timeHighset && (millis() - timeHigh) > cfg.psHoldMs) {
          powerSave = true;
          if (cfg.psDebug) Host.println(F("$PS,start"));
        }
      } else if (ohmsVoltage < cfg.sleepV - cfg.psHyst) {
        powerSave   = false;
        timeHighset = false;
        if (cfg.psDebug) { Host.print(F("$PS,end,")); Host.println(ohmsVoltage, 3); }
      }
    } else if (!ohmsMode) {
      powerSave   = false;
      timeHighset = false;
    }

    // ---- Ohms source parking ----
    // Derived from the state every pass and written only on a change.  The
    // old code parked it in Voltmeter mode and only released it when the
    // parked rail sagged, so Voltmeter -> High R with the leads open left it
    // parked and the reading wrong.
    static int ohmsPwmApplied = -1;
    int ohmsPwmWant = (!ohmsMode || powerSave) ? cfg.psPwm : 0;
    if (ohmsPwmWant != ohmsPwmApplied) {
      if (ohmsPwmWant == 0) ohmsUnparkMs = millis();
      ohmsPwmApplied = ohmsPwmWant;
      analogWrite(OHMPWMPIN, ohmsPwmWant);
    }
    ohmsParked = (ohmsPwmWant != 0);

    // ---- Which measurement to show ----
    if ((abs(countV) > (int)cfg.vDispCount ||
         currentMode == Voltmeter || currentMode == VACmanual) &&
        currentMode != HighRMode && currentMode != rPlotMode) {
      voltageDisplay = true;
      ads.setDataRate(preciseMode ? RATE_ADS1115_16SPS : RATE_ADS1115_860SPS);
    }
    if ((isBetween(currentResistance, cfg.rDispMin, cfg.rDispMax) &&
         currentMode != Voltmeter && currentMode != VACmanual) ||
        currentMode == HighRMode || currentMode == rPlotMode) {
      voltageDisplay = false;
      ads.setDataRate(RATE_ADS1115_32SPS);
    }

    if (rFresh) {
      displayResistance = currentResistance - zeroOffsetRes;
      if (initialZeroSet) {
        if (displayResistance > highR && displayResistance < cfg.mmRMax)
          highR = displayResistance;
        if (displayResistance < lowR && displayResistance > cfg.mmRMin)
          lowR = displayResistance;
      }
    }
  }

  serialPoll();
  bleTick();
  serialStreamTick(currentMillis);

  updateAlerts();
  updateM4();

  // ---- USB-stick autolog ----
  // A sample is captured on a new extreme or a current step; the CSV write
  // happens on the pass after, so the screen is not redrawn in the same pass
  // as a file write.
  if (serviceAutoLog()) return;

  if ((currentMillis - previousLcdMillis >= lcdInterval) && !takeLog) {
    previousLcdMillis = currentMillis;
    lcdInterval = screenRefreshFast ? cfg.lcdFastMs : cfg.lcdMs;
    updateDisplay();
  }

  // ---- Screen saver ----
  if (ohmsParked && fabs(newVoltageReading) < cfg.sleepVMax &&
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
