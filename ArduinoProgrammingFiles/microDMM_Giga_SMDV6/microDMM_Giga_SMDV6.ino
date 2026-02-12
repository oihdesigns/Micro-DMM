// microDMM_Giga_SMDV6 - GUI Redesign
// Based on SMDV5 measurement logic with Oscilloscope_Giga-style dark-theme GUI
// All measurement, calibration, serial, RPC, USB logging code unchanged from SMDV5

// ======================= INCLUDES =======================
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino_GigaDisplay.h>

// GIGA HID Related
#include "PluggableUSBHID.h"
#include "USBKeyboard.h"
USBKeyboard Keyboard;

// Giga USB Related
#include <Arduino_USBHostMbed5.h>
#include <DigitalOut.h>
#include <FATFileSystem.h>

GigaDisplayRGB rgb;
GigaDisplayBacklight backlight;

#include "RPC.h"

// USB mass-storage objects
USBHostMSD        msd;
mbed::FATFileSystem usb("usb");

uint16_t lut[] = {
    0x0800,0x08c8,0x098f,0x0a52,0x0b0f,0x0bc5,0x0c71,0x0d12,0x0da7,0x0e2e,0x0ea6,0x0f0d,0x0f63,0x0fa7,0x0fd8,0x0ff5,
    0x0fff,0x0ff5,0x0fd8,0x0fa7,0x0f63,0x0f0d,0x0ea6,0x0e2e,0x0da7,0x0d12,0x0c71,0x0bc5,0x0b0f,0x0a52,0x098f,0x08c8,
    0x0800,0x0737,0x0670,0x05ad,0x04f0,0x043a,0x038e,0x02ed,0x0258,0x01d1,0x0159,0x00f2,0x009c,0x0058,0x0027,0x000a,
    0x0000,0x000a,0x0027,0x0058,0x009c,0x00f2,0x0159,0x01d1,0x0258,0x02ed,0x038e,0x043a,0x04f0,0x05ad,0x0670,0x0737
};
static size_t lut_size = sizeof(lut) / sizeof(lut[0]);

// ======================= DISPLAY & TOUCH =======================
Adafruit_ADS1115 ads;

#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>

GigaDisplay_GFX display;
Arduino_GigaDisplayTouch touch;

// ======================= SCREEN CONSTANTS =======================
static const int16_t SCREEN_W = 800;
static const int16_t SCREEN_H = 480;

// ======================= RGB565 HELPER =======================
static inline uint16_t RGB(uint8_t r, uint8_t g, uint8_t b) {
  return display.color565(r, g, b);
}

// ======================= UI BUTTON STRUCT =======================
struct UIButton {
  int16_t x, y, w, h;
  const char* label;
  uint16_t color;
  bool latching;
  bool state;
  bool pressed;
  uint32_t lastChangeMs;
  bool drawnState;
};

#define inRect(tx, ty, b) \
  ((tx) >= (b).x && (tx) < (b).x + (b).w && (ty) >= (b).y && (ty) < (b).y + (b).h)

// ======================= COLORS =======================
static uint16_t COL_BG;
static uint16_t COL_PANEL;
static uint16_t COL_FRAME;
static uint16_t COL_TEXT;
static uint16_t COL_SUBTEXT;
static uint16_t COL_DATA;
static uint16_t COL_OFF;

static uint16_t COL_BTN_MODE;
static uint16_t COL_BTN_ZERO;
static uint16_t COL_BTN_MINMAX;
static uint16_t COL_BTN_LOG;
static uint16_t COL_BTN_RANGE;
static uint16_t COL_BTN_SMOOTH;
static uint16_t COL_BTN_AMPS;
static uint16_t COL_BTN_FAST;

void initColors() {
  COL_BG       = RGB(0, 0, 0);
  COL_PANEL    = RGB(20, 20, 28);
  COL_FRAME    = RGB(60, 60, 70);
  COL_TEXT     = RGB(220, 220, 220);
  COL_SUBTEXT  = RGB(140, 140, 150);
  COL_DATA     = RGB(0, 255, 50);
  COL_OFF      = RGB(60, 60, 60);

  COL_BTN_MODE   = RGB(200, 100, 255);  // Purple
  COL_BTN_ZERO   = RGB(255, 220, 0);    // Yellow
  COL_BTN_MINMAX = RGB(0, 200, 255);    // Cyan
  COL_BTN_LOG    = RGB(50, 200, 90);    // Green
  COL_BTN_RANGE  = RGB(70, 140, 255);   // Blue
  COL_BTN_SMOOTH = RGB(50, 200, 90);    // Green
  COL_BTN_AMPS   = RGB(255, 165, 0);    // Orange
  COL_BTN_FAST   = RGB(220, 60, 60);    // Red
}

// ======================= LAYOUT =======================
static const int16_t INFO_H      = 36;
static const int16_t BTN_PANEL_X = 630;
static const int16_t BTN_PANEL_W = SCREEN_W - BTN_PANEL_X - 6; // 164

// ======================= PLOT CONFIG =======================
extern const float samples[];
extern const int SAMPLE_COUNT_PLOT = 100;

static const int PLOT_SIZE = 400;
static const int PLOT_X    = 0;
static const int PLOT_Y    = 80;

#define COLOR_BG     0x0000
#define COLOR_DATA   0xFFFF
#define COLOR_MIN    0x07E0
#define COLOR_MEAN   0xFFE0
#define COLOR_MAX    0xF800

// ======================= BUTTON DEFINITIONS =======================
enum BtnIdx {
  BTN_MODE = 0,
  BTN_ZERO,
  BTN_MINMAX,
  BTN_LOG,
  BTN_RANGE,
  BTN_SMOOTH,
  BTN_AMPS,
  BTN_FAST
};

static const int MAX_BUTTONS = 8;
static UIButton g_buttons[MAX_BUTTONS];
static const uint32_t TOUCH_DEBOUNCE_MS = 300;

// Mode popup state
static bool modePopupActive = false;

// ======================= MINI SCHEDULER =======================
struct Task { void (*fn)(); uint32_t interval; uint32_t last; };
static const uint8_t MAX_TASKS = 8;
static Task g_tasks[MAX_TASKS];
static uint8_t g_taskCount = 0;

void addTask(void (*fn)(), uint32_t intervalMs) {
  if (g_taskCount < MAX_TASKS) {
    g_tasks[g_taskCount++] = {fn, intervalMs, 0};
  }
}

void runTasks() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < g_taskCount; ++i) {
    Task &t = g_tasks[i];
    if (now - t.last >= t.interval) {
      t.last = now;
      t.fn();
    }
  }
}

// ======================= HARDWARE PIN DEFINITIONS (unchanged) =======================
const int OHMPWMPIN = 2;
const int TYPE_PIN      = 3;
#define BUTTON_PIN 4
const int VbridgePin = 5;
const int CONTINUITY_PIN = 6;
const int SETRANGE_PIN = 7;

const int cycleTrack = 52;
const int backlightRef =  64;
const int backlightControlHigh =  65;
const int backlightControlLow =  63;

const int logButton = A1;
const int micPin = A7;

// ======================= RPC VARIABLES =======================
static bool txFlag = false;
static uint8_t lastSent = 0xFF;
static bool humFlag = false;

// ======================= ADC GAIN FACTORS (unchanged) =======================
const float GAIN_FACTOR_TWOTHIRDS = 0.1875;
const float GAIN_FACTOR_1   = 0.125;
const float GAIN_FACTOR_2   = 0.0625;
const float GAIN_FACTOR_4   = 0.03125;
const float GAIN_FACTOR_8   = 0.015625;
const float GAIN_FACTOR_16  = 0.0078125;

static const adsGain_t kGainLevels[] = {
  GAIN_TWOTHIRDS,
  GAIN_ONE,
  GAIN_TWO,
  GAIN_FOUR,
  GAIN_EIGHT,
  GAIN_SIXTEEN
};

// ======================= CALIBRATION VALUES (unchanged) =======================
float constantI            = 0.020138f;
const float constantR            = 330.0f;
const float dividerR             = 22000.0f;
float ZENER_MAX_V          = 4.9854f;
float EEPROM_MAXV = 4.972f;
float EEPROM_SleepV = 0.6117;

float VOLTAGE_SCALE = 68.5289;
float VOLTAGE_SCALE_Negative = 68.6692;
float giga_absfactor = 0.0;

// ======================= MODE ENUM (unchanged) =======================
#define DEBOUNCE_DELAY 300
enum Mode {
  Voltmeter,
  HighRMode,
  Default,
  VACmanual,
  Type,
  Low,
  AltUnitsMode,
  rPlotMode,
  Charging,
  NUM_MODES
};

Mode currentMode = Voltmeter;
Mode previousMode = Voltmeter;

unsigned long lastDebounceTime = 0;
bool lastButtonState = HIGH;
bool buttonPreviouslyPressed = false;

// ======================= TIMING VARIABLES (unchanged) =======================
const unsigned long ADC_INTERVAL    = 1;
const unsigned long BATT_INTERVAL   = 1000;
unsigned long LCD_INTERVAL         = 1000;
const unsigned long SERIAL_INTERVAL = 200;
const unsigned long IR_DEBOUNCE_INTERVAL = 300;
unsigned long previousAdcMillis   = 0;
unsigned long previousBattMillis  = 0;
unsigned long previousLcdMillis   = 0;
unsigned long previousSerialMillis= 0;
unsigned long lastIrReceiveMillis = 0;
unsigned long deepSleepStart = 0;
unsigned int secondsTime = 0;
unsigned long previousTouchTime;

// ======================= MEASUREMENT VARIABLES (unchanged) =======================
int16_t adcReadingVoltage = 0;
int16_t adcReadingOhms    = 0;
int16_t adcReadingCurrent = 0;
float batteryVoltage = 0.0;
int16_t countV;
int16_t adcCount;
static uint8_t gainIndex;
int32_t countI;

float newVoltageReading = 0.0;
float averageVoltage = 0.0;
float VAC = 0.0;
float previousVoltage = 0.0;
float medianVoltage = 0.0;
float medianVoltageStep = 0.0;
float previousmeanY = 0.0;
float meanY = 0.0;

float rawResistance = 0.0;
float calibratedResistance = 0.0;
float currentResistance = 1000001.0;
float prevResistance = 0.0;
float zeroOffsetRes = 0.0;
float displayResistance = 0.0;
float ohmsVoltage = 0.0;
float timeHigh = 0.0;

float currentShuntVoltage = 0.0;
float Ireading = 0.0;
float Izero = 0.0;
bool Irange = false;
bool currentOnOff = false;
float IHigh = -6.0;
float ILow  = 6.0;
float IMedian = 0.0;

// Rolling buffers (unchanged)
const int NUM_VOLTAGE_SAMPLES = 100;
float voltageSamples[NUM_VOLTAGE_SAMPLES] = {0.0};
float voltageSquaredSamples[NUM_VOLTAGE_SAMPLES] = {0.0};
int voltageSampleIndex = 0;
float voltageSum = 0.0;
float squaredVoltageSum = 0.0;

const int NUM_RESISTANCE_SAMPLES = 100;
float resistanceSamples[NUM_VOLTAGE_SAMPLES] = {0.0};
int resistanceSampleIndex = 0;

const int NUM_CURRENT_SAMPLES = 100;
float currentSamples[NUM_VOLTAGE_SAMPLES] = {0.0};
int currentSampleIndex = 0;

// Min/Max tracking (unchanged)
float highV = -100.0, lowV = 100.0;
float highR = 0.0, lowR = 3000000.0;
String timeAtMaxV = "", timeAtMinV = "";
String timeAtMaxI = "", timeAtMinI = "";
float voltageAtMaxI = 0.0, voltageAtMinI = 0.0;
float currentAtMaxV = 0.0, currentAtMinV = 0.0;

// Formatted display values (unchanged)
float roundedV = 0.0, roundedVlow = 0.0, roundedVhigh = 0.0;
int vDigits = 0, vDigitslow = 0, vDigitshigh = 0;
String vSuffix = "", vSuffixlow = "", vSuffixhigh = "";
float roundedR = 0.0, roundedRlow = 0.0, roundedRhigh = 0.0;
int rDigits = 0, rDigitslow = 0, rDigitshigh = 0;
String rSuffix = "", rSuffixlow = "", rSuffixhigh = "";

// IR remote codes (kept for compatibility)
#define SET_ZERO_CODE       0x16
#define CLEAR_ZERO_CODE     0x0C
#define SERIAL_SEND_CODE    0x40
#define R_RANGE_CODE        0x47
#define V_SMOOTH_CODE       0x5E
#define TYPE_VOLTAGE_CODE   0x07
#define TYPE_RESISTANCE_CODE 0x09
#define KEY_DELETE_CODE     0x45
#define KEY_UP_CODE         0x46
#define KEY_RIGHT_CODE      0x43
#define KEY_LEFT_CODE       0x44
#define KEY_DOWN_CODE       0x15
#define TYPE_MOVE_CODE      0x19
#define AUTO_RANGE_CODE     0x1C
#define CLEAR_MINMAX_CODE   0x4A
#define RESET_MINMAX_CODE   0x52
#define CURRENT_MODE_CODE   0x18

// Custom bitmaps (unchanged)
const uint8_t PROGMEM OHM_8x8[] = {
  0b00111100,
  0b01000010,
  0b10000001,
  0b10000001,
  0b10000001,
  0b10011001,
  0b01000010,
  0b00111100
};

const uint8_t MICRO_5x7[] PROGMEM = {
  0b00000,
  0b10010,
  0b10010,
  0b10010,
  0b10010,
  0b10110,
  0b10000
};

// Calibration correction factors (unchanged)
float CF_A = 0.9607;
float CF_B = 0.9848;
float CF_C = 0.997;
float CF_D = 0.9958;
float CF_E = 0.9934;
float CF_F = 0.9965;
float CF_G = 0.9982;
float CF_H = 1.0028;
float CF_I = 1.0012;
float CF_J = 1.0005;
float CF_K = 1.0017;
float CF_L = 1.0035;
float CF_M = 1.0075;
float CF_N = 1.0237;
float CF_O = 1.1226;

// ======================= FLAGS AND MODES (unchanged) =======================
bool ohmsHighRange = true;
bool ohmsAutoRange = true;
bool voltageDisplay = false;
bool smoothVmode = true;
bool vDisplayManual = false;
bool MinMaxDisplay = false;
bool screenRefreshFast = false;
bool ampsMode = false;
bool debugMode = false;
bool initialZeroSet = false;
bool serialMode = false;
bool useKeyboard = false;
bool powerSave;
bool timeHighset;
bool altUnits = false;
bool preciseMode = false;
bool deepSleepTrigger = false;
bool screenSleep = false;
bool deltaMode = false;
bool deltaTrigger = false;
float deltaV = 0.0;
int deltaVdigits = 0;

bool continuity = false;
bool logMode = false;
int usbDelay = 0;

bool flashlightMode = false;
bool flashlightChecked = false;
bool buttonPressed = false;
unsigned long buttonPressTime = 0;
bool vFlag = false;
bool rFlag = false;
bool VACPresense = false;
bool cycleTracking = false;

// Logging buffers (unchanged)
const int LOG_SIZE = 100;
const int SAMPLE_COUNT = 100;
bool takeLog = false;
int samplesTaken = 0;
float tLogStart = 0;
float tLogEnd = 0;
float loggedCurrents[LOG_SIZE] = {0};
float loggedVoltagesAtI[LOG_SIZE] = {0};
float loggedTimeStamps[LOG_SIZE] = {0};

// V Float Detect (unchanged)
bool vFloating = false;
float bridgeV = 0.0;
bool Vzero = true;
bool vClosed = false;
bool vUndefined = true;
bool vClosedflag = false;
bool vClosedflagPrevious = false;

int blinkLimit = 0;

// Auto Log (unchanged)
#define NUM_AUTO_VALUES 32
float VAutoArray[NUM_AUTO_VALUES] = {0};
float TAutoArray[NUM_AUTO_VALUES] = {0};
float IAutoArray[NUM_AUTO_VALUES] = {0};
float Istep = 0.1;
bool AutologTriggered = 0;
bool writePrimed = 0;
float lastLoggedI = 0;
float ItHighm;
int autologCount = 0;
int manuallogCount = 0;

// Legacy press flags (kept but no longer set by touch - only used with micPin fallback)
bool redPress    = false;
bool greenPress  = false;
bool bluePress   = false;
bool yellowPress = false;

// ======================= FORWARD DECLARATIONS =======================
void drawStaticChrome();
void drawInfoBar();
void initButtons();
void drawButton(const UIButton& b);
void drawButtons(bool full);
void handleTouch();
void onButtonAction(int idx);
void drawModePopup();
void handleModePopupTouch(int16_t tx, int16_t ty);

void handleSerialCommands(char command);
void handleButtonInput();
void measureVoltage();
void measureResistance();
void measureCurrent();
void updateDisplay();
void updateAlerts();
void logCurrentData(float voltage, float timeSec, float current);
void formatResistanceValue(float value, float &outValue, String &outSuffix, int &outDigits);
void formatVoltageValue(float value, float &outValue, String &outSuffix, int &outDigits);
bool isBetween(float value, float low, float high);
String formatTime(unsigned long milliseconds);
void ReZero();
void checkModeButton();
void drawPlot(const float data[], int n);
void drawStatLine(float value, float plotMin, float yScale, uint16_t color);
void drawTwoPlots(const float data[], const float data2[], int n);
void ClosedOrFloat();
void manuallogArraysToCSV();
void autologArraysToCSV();
void logValues(float VatIHigh, float ItHighm, float IHigh);
void shiftAndStore(float* array, float newValue);

// ======================= GUI HELPER FUNCTIONS =======================

void drawRoundedPanel(int16_t x, int16_t y, int16_t w, int16_t h,
                      uint16_t frame, uint16_t fill) {
  display.fillRoundRect(x, y, w, h, 8, fill);
  display.drawRoundRect(x, y, w, h, 8, frame);
}

void drawStaticChrome() {
  display.fillScreen(COL_BG);

  // Info bar background
  display.fillRect(0, 0, SCREEN_W, INFO_H, COL_PANEL);
  display.drawFastHLine(0, INFO_H - 1, SCREEN_W, COL_FRAME);
}

void drawInfoBar() {
  // Clear info bar
  display.fillRect(0, 0, SCREEN_W, INFO_H - 1, COL_PANEL);

  display.setTextSize(2);
  display.setTextColor(COL_BTN_MODE);
  display.setCursor(8, 10);
  display.print(modeToString(currentMode));

  // Time display
  display.setTextColor(COL_TEXT);
  display.setCursor(540, 10);
  display.print(formatTime(millis()));

  // USB/Log status
  if (logMode) {
    display.setTextColor(COL_BTN_LOG);
    display.setCursor(300, 10);
    display.print("USB");
    display.setTextColor(COL_SUBTEXT);
    display.print(" A:");
    display.print(autologCount);
    display.print(" M:");
    display.print(manuallogCount);
  }
}

// ======================= BUTTON INITIALIZATION =======================
void initButtons() {
  const int16_t btnH = 51;
  const int16_t gap  = 4;
  int16_t y = INFO_H + 4;

  struct BtnDef {
    const char* label;
    uint16_t color;
    bool latching;
  };

  BtnDef defs[8] = {
    {"MODE",    COL_BTN_MODE,   false},
    {"ZERO",    COL_BTN_ZERO,   false},
    {"MIN/MAX", COL_BTN_MINMAX, true},
    {"LOG",     COL_BTN_LOG,    false},
    {"RANGE",   COL_BTN_RANGE,  true},
    {"SMOOTH",  COL_BTN_SMOOTH, true},
    {"AMPS",    COL_BTN_AMPS,   true},
    {"FAST",    COL_BTN_FAST,   true},
  };

  for (int i = 0; i < MAX_BUTTONS; ++i) {
    UIButton &b = g_buttons[i];
    b.x = BTN_PANEL_X;
    b.y = y;
    b.w = BTN_PANEL_W;
    b.h = btnH;
    b.label = defs[i].label;
    b.color = defs[i].color;
    b.latching = defs[i].latching;
    b.state = false;
    b.pressed = false;
    b.lastChangeMs = 0;
    b.drawnState = !b.state; // force first draw
    y += btnH + gap;
  }

  // Initialize latching button states from current flags
  g_buttons[BTN_MINMAX].state = MinMaxDisplay;
  g_buttons[BTN_RANGE].state  = !ohmsAutoRange; // RANGE on = manual
  g_buttons[BTN_SMOOTH].state = smoothVmode;
  g_buttons[BTN_AMPS].state   = ampsMode;
  g_buttons[BTN_FAST].state   = screenRefreshFast;
}

// ======================= BUTTON DRAWING =======================
void drawButton(const UIButton& b) {
  const bool on = b.latching ? b.state : b.pressed;
  const uint16_t fill = on ? b.color : COL_OFF;
  const uint16_t frame = on ? COL_TEXT : COL_FRAME;

  display.fillRoundRect(b.x, b.y, b.w, b.h, 8, fill);
  display.drawRoundRect(b.x, b.y, b.w, b.h, 8, frame);

  // Center label
  display.setTextColor(COL_TEXT);
  display.setTextSize(2);
  int16_t textW = strlen(b.label) * 12;
  int16_t tx = b.x + (b.w - textW) / 2;
  int16_t ty = b.y + (b.h - 16) / 2;
  display.setCursor(tx, ty);
  display.print(b.label);
}

void drawButtons(bool full) {
  for (int i = 0; i < MAX_BUTTONS; ++i) {
    UIButton &b = g_buttons[i];
    const bool visState = b.latching ? b.state : b.pressed;
    if (full || visState != b.drawnState) {
      drawButton(b);
      b.drawnState = visState;
    }
  }
}

// ======================= TOUCH HANDLING =======================
void handleTouch() {
  uint8_t contacts = 0;
  GDTpoint_t pts[5];
  contacts = touch.getTouchPoints(pts);

  // Handle mode popup touches - intercept main area touches
  if (modePopupActive && contacts > 0) {
    static uint32_t lastPopupTouch = 0;
    int16_t tx = pts[0].y;
    int16_t ty = (SCREEN_H - 1) - pts[0].x;
    if (tx < BTN_PANEL_X && (millis() - lastPopupTouch >= TOUCH_DEBOUNCE_MS)) {
      lastPopupTouch = millis();
      handleModePopupTouch(tx, ty);
      return;
    }
    // If touch is on button panel, fall through to button handling
    if (tx < BTN_PANEL_X) return;
  }

  bool touched[MAX_BUTTONS] = {};
  for (uint8_t k = 0; k < contacts; ++k) {
    // Touch controller returns native portrait coords (480x800)
    // Remap for GFX rotation(1) landscape (800x480):
    int16_t tx = pts[k].y;
    int16_t ty = (SCREEN_H - 1) - pts[k].x;
    for (int i = 0; i < MAX_BUTTONS; ++i) {
      if (inRect(tx, ty, g_buttons[i])) touched[i] = true;
    }
  }

  const uint32_t now = millis();

  for (int i = 0; i < MAX_BUTTONS; ++i) {
    UIButton &b = g_buttons[i];

    if (b.latching) {
      bool entering = touched[i] && !b.pressed;
      if (entering && (now - b.lastChangeMs >= TOUCH_DEBOUNCE_MS)) {
        b.state = !b.state;
        b.lastChangeMs = now;
        onButtonAction(i);
      }
      b.pressed = touched[i];
    } else {
      bool entering = touched[i] && !b.pressed;
      if (entering && (now - b.lastChangeMs >= TOUCH_DEBOUNCE_MS)) {
        b.lastChangeMs = now;
        onButtonAction(i);
      }
      b.pressed = touched[i];
    }
  }
}

// ======================= BUTTON ACTIONS =======================
void onButtonAction(int idx) {
  switch (idx) {
    case BTN_MODE:
      modePopupActive = true;
      drawModePopup();
      Serial.println("Mode popup opened");
      break;

    case BTN_ZERO:
      // Same logic as old yellowPress handler
      if (currentMode == Type) {
        if (voltageDisplay) {
          Keyboard.printf("%.*f", vDigits, newVoltageReading);
        } else {
          if (displayResistance < 1) {
            Keyboard.printf("%.*f", rDigits + 2, displayResistance);
          } else {
            Keyboard.printf("%.*f", rDigits, displayResistance);
          }
        }
        Keyboard.key_code(RIGHT_ARROW);
      } else {
        if (voltageDisplay) {
          deltaVdigits = vDigits - 1;
          if (preciseMode) {
            deltaV = newVoltageReading;
          } else {
            deltaV = averageVoltage;
          }
        } else {
          if (zeroOffsetRes == 0) {
            zeroOffsetRes = currentResistance;
          } else {
            zeroOffsetRes = 0;
          }
        }
      }
      break;

    case BTN_MINMAX:
      MinMaxDisplay = g_buttons[BTN_MINMAX].state;
      if (MinMaxDisplay) ReZero();
      break;

    case BTN_LOG:
      takeLog = true;
      break;

    case BTN_RANGE:
      ohmsAutoRange = !g_buttons[BTN_RANGE].state; // RANGE on = manual (auto off)
      Serial.print("Auto-range: ");
      Serial.println(ohmsAutoRange ? "On" : "Off");
      break;

    case BTN_SMOOTH:
      smoothVmode = g_buttons[BTN_SMOOTH].state;
      Serial.print("Smooth mode: ");
      Serial.println(smoothVmode ? "On" : "Off");
      break;

    case BTN_AMPS:
      ampsMode = g_buttons[BTN_AMPS].state;
      Serial.print("Amps mode: ");
      Serial.println(ampsMode ? "On" : "Off");
      break;

    case BTN_FAST:
      screenRefreshFast = g_buttons[BTN_FAST].state;
      Serial.print("Fast refresh: ");
      Serial.println(screenRefreshFast ? "On" : "Off");
      break;
  }
}

// ======================= MODE POPUP =======================
void drawModePopup() {
  // Dark overlay on main area
  display.fillRect(0, INFO_H, BTN_PANEL_X, SCREEN_H - INFO_H, RGB(10, 10, 15));

  // 3x3 grid
  const int16_t gridW = 190;
  const int16_t gridH = 120;
  const int16_t gap = 15;
  const int16_t totalW = 3 * gridW + 2 * gap;
  const int16_t totalH = 3 * gridH + 2 * gap;
  const int16_t startX = (BTN_PANEL_X - totalW) / 2;
  const int16_t startY = INFO_H + ((SCREEN_H - INFO_H) - totalH) / 2;

  const char* modeNames[] = {
    "Voltmeter", "High R", "Default",
    "VAC", "Type", "Low",
    "Alt Units", "R Plot", "Charging"
  };

  for (int i = 0; i < (int)NUM_MODES; i++) {
    int col = i % 3;
    int row = i / 3;
    int16_t x = startX + col * (gridW + gap);
    int16_t y = startY + row * (gridH + gap);

    uint16_t fillColor = (i == (int)currentMode) ? COL_BTN_MODE : RGB(40, 40, 50);
    display.fillRoundRect(x, y, gridW, gridH, 10, fillColor);
    display.drawRoundRect(x, y, gridW, gridH, 10, COL_FRAME);

    // Center text
    display.setTextSize(2);
    display.setTextColor(COL_TEXT);
    int16_t textW = strlen(modeNames[i]) * 12;
    int16_t tx = x + (gridW - textW) / 2;
    int16_t ty = y + (gridH - 16) / 2;
    display.setCursor(tx, ty);
    display.print(modeNames[i]);
  }
}

void handleModePopupTouch(int16_t tx, int16_t ty) {
  const int16_t gridW = 190;
  const int16_t gridH = 120;
  const int16_t gap = 15;
  const int16_t totalW = 3 * gridW + 2 * gap;
  const int16_t totalH = 3 * gridH + 2 * gap;
  const int16_t startX = (BTN_PANEL_X - totalW) / 2;
  const int16_t startY = INFO_H + ((SCREEN_H - INFO_H) - totalH) / 2;

  for (int i = 0; i < (int)NUM_MODES; i++) {
    int col = i % 3;
    int row = i / 3;
    int16_t x = startX + col * (gridW + gap);
    int16_t y = startY + row * (gridH + gap);

    if (tx >= x && tx < x + gridW && ty >= y && ty < y + gridH) {
      currentMode = static_cast<Mode>(i);
      modePopupActive = false;
      Serial.print("Mode selected: ");
      Serial.println(modeToString(currentMode));
      // Redraw main area
      display.fillRect(0, INFO_H, BTN_PANEL_X, SCREEN_H - INFO_H, COL_BG);
      return;
    }
  }
}

// ======================= SCHEDULED TASK =======================
void task_touch() {
  handleTouch();
  drawButtons(false);
}

// ======================= SETUP =======================
void setup() {
  Wire1.begin();
  Wire1.setClock(400000);

  Serial.begin(115200);
  delay(100);
  Serial.println("Setup Start - SMDV6");

  // Initialize display
  display.begin();
  display.setRotation(1); // landscape mode

  initColors();

  display.setTextSize(4);
  display.setTextColor(COL_TEXT);
  display.fillScreen(COL_BG);
  display.setCursor(0, 0);
  display.println("GIGA METER V6");

  // Initialize touch
  if (!touch.begin()) {
    while (1) {}
  }

  // USB drive init
  display.setTextSize(2);
  display.setTextColor(COL_SUBTEXT);
  pinMode(PA_15, OUTPUT);
  digitalWrite(PA_15, HIGH);
  mbed::DigitalOut vbusEnable(PB_8, 1);
  Serial.print("Waiting for USB drive");
  display.println("Waiting 400ms for USB drive");
  while (!msd.connect() && usbDelay < 2) {
    delay(200);
    usbDelay++;
  }
  if (msd.connect()) {
    logMode = true;
    Serial.print('\n');
    Serial.println("Drive connected.");
    display.println("Drive connected");
    int err = usb.mount(&msd);
    if (err) {
      Serial.print("Mount failed: ");
      display.println(err);
      while (true);
    }
    Serial.println("Filesystem mounted.");
  } else {
    logMode = false;
    display.println("No USB Drive.");
    Serial.print("No Drive");
  }

  // Configure pins
  pinMode(TYPE_PIN, INPUT_PULLUP);
  pinMode(CONTINUITY_PIN, OUTPUT);
  pinMode(SETRANGE_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(OHMPWMPIN, OUTPUT);
  pinMode(logButton, INPUT_PULLUP);
  pinMode(cycleTrack, OUTPUT);
  pinMode(VbridgePin, OUTPUT);
  pinMode(micPin, INPUT);
  pinMode(backlightRef, OUTPUT);
  pinMode(backlightControlHigh, INPUT_PULLUP);
  pinMode(backlightControlLow, INPUT_PULLUP);

  // Initialize ADS1115 ADC
  if (!ads.begin(0x48, &Wire1)) {
    display.setCursor(0, 0);
    display.setTextSize(1);
    Serial.println("ADS1115 not found");
    display.println("ADS1115 not found");
    while (1);
  }
  display.println("ADS1115 found");
  Serial.println("ADS1115 good");

  ads.setDataRate(RATE_ADS1115_16SPS);
  ads.setGain(GAIN_TWOTHIRDS);
  delay(100);

  Serial.println(F("Commands: Q,q,Z,C,R,S,V,A,F,M,B,D,e,E,I,L,?"));

  // Ammeter Auto Detect
  delay(100);
  adcReadingCurrent = ads.readADC_SingleEnded(3);
  currentShuntVoltage = adcReadingCurrent * GAIN_FACTOR_TWOTHIRDS / 1000.0;
  if (adcReadingCurrent < 300 || isBetween(currentShuntVoltage, 1.5, 3.5)) {
    if (isBetween(currentShuntVoltage, 1.5, 3.5)) {
      Irange = true;
      Izero = currentShuntVoltage;
    } else {
      Irange = false;
      Izero = 0.0;
    }
    currentOnOff = true;
    Serial.print("Ammeter Enabled:");
    Serial.println(currentShuntVoltage);
    display.print("Ammeter Enabled:");
    display.println(currentShuntVoltage);
  } else {
    currentOnOff = false;
    Serial.print("Ammeter Disabled:");
    Serial.println(currentShuntVoltage);
    display.print("Ammeter Disabled:");
    display.println(currentShuntVoltage);
    Ireading = 0.0;
  }

  // RPC init
  display.println("Booting Core 2");
  RPC.begin();
  uint8_t pack = (txFlag ? 1 : 0) | (humFlag ? 2 : 0);
  RPC.write(pack);
  lastSent = pack;
  display.println("Core 2 Booted");

  delay(1000);

  // Draw the new dark-theme GUI
  drawStaticChrome();
  initButtons();
  drawButtons(true);
  drawInfoBar();

  rgb.begin();

  // Set up scheduler for touch polling
  addTask(task_touch, 30); // touch at ~33 Hz

  Serial.println("Setup End - SMDV6");
}

// ======================= MAIN LOOP =======================
void loop() {
  unsigned long currentMillis = millis();

  cycleTracking = !cycleTracking;
  if (cycleTracking) {
    digitalWrite(cycleTrack, HIGH);
  } else {
    digitalWrite(cycleTrack, LOW);
  }

  // Run scheduled tasks (touch polling)
  runTasks();

  // Legacy analog button inputs (micPin) - unchanged behavior
  if (isBetween(analogRead(micPin), 100, 275)) {
    MinMaxDisplay = true;
    ReZero();
    g_buttons[BTN_MINMAX].state = true; // sync button state
  }

  if (analogRead(micPin) < 60) {
    if (currentMode == Type) {
      if (voltageDisplay) {
        Keyboard.printf("%.*f", vDigits, newVoltageReading);
      } else {
        if (displayResistance < 1) {
          Keyboard.printf("%.*f", rDigits + 2, displayResistance);
        } else {
          Keyboard.printf("%.*f", rDigits, displayResistance);
        }
      }
      Keyboard.key_code(RIGHT_ARROW);
    } else {
      if (voltageDisplay) {
        deltaVdigits = vDigits - 1;
        if (preciseMode) {
          deltaV = newVoltageReading;
        } else {
          deltaV = averageVoltage;
        }
      } else {
        if (zeroOffsetRes == 0) {
          zeroOffsetRes = currentResistance;
        } else {
          zeroOffsetRes = 0;
        }
      }
    }
  }

  previousMode = currentMode;

  checkModeButton(); // Check physical mode button - opens popup

  if (currentMode == AltUnitsMode) {
    altUnits = true;
  } else {
    altUnits = false;
  }

  if (currentMode == Low) {
    preciseMode = true;
  } else {
    preciseMode = false;
  }

  if (currentMode == Type) {
    deltaMode = false;
    deltaV = 0;
  } else {
    deltaMode = true;
  }

  // Flashlight mode check
  if (!flashlightChecked) {
    if (digitalRead(TYPE_PIN) == LOW) {
      flashlightMode = true;
    }
    flashlightChecked = true;
  }

  // Log button (physical pin)
  if (digitalRead(logButton) == 0) {
    takeLog = true;
  }

  // Handle log sampling (unchanged)
  if (takeLog == true) {
    if (samplesTaken < (SAMPLE_COUNT + 1)) {
      float nowSec = millis() / 1000.0;
      logCurrentData(newVoltageReading, nowSec, Ireading);
      samplesTaken++;
    } else {
      takeLog = false;
      tLogEnd  = millis() / 1000.0;
      samplesTaken = 0;
      analogWrite(CONTINUITY_PIN, 100);
      rgb.on(0, 255, 0);
      Serial.println("Samples logged!");

      if (logMode) {
        manuallogArraysToCSV();
        manuallogCount++;
      }
    }
  }

  handleButtonInput();

  // Periodic ADC measurements (unchanged)
  if (currentMillis - previousAdcMillis >= ADC_INTERVAL) {
    previousAdcMillis = currentMillis;
    if (!takeLog && currentMode != Voltmeter && currentMode != VACmanual) {
      measureResistance();
    }
    measureVoltage();
    if (currentOnOff) {
      measureCurrent();
    }

    // PowerSave Related
    if (!powerSave) {
      if (ohmsVoltage > EEPROM_MAXV - 0.002 && !timeHighset && currentMode != HighRMode) {
        timeHigh = millis();
        timeHighset = true;
      }
      if (timeHighset && currentResistance < 20000) {
        timeHighset = false;
      }
      if (timeHighset && timeHigh + 5000 < millis()) {
        powerSave = true;
      }
    }

    if (powerSave || currentMode == Charging || currentMode == Voltmeter) {
      analogWrite(OHMPWMPIN, 254);
      if (ohmsVoltage < EEPROM_SleepV - 0.01 && currentMode != Voltmeter) {
        powerSave = false;
        timeHighset = false;
        analogWrite(OHMPWMPIN, 0);
      }
    }

    // Auto-switch display mode
    if (fabs(countV) > 500 || currentMode == Voltmeter) {
      voltageDisplay = true;
      if (preciseMode) {
        ads.setDataRate(RATE_ADS1115_16SPS);
      } else {
        ads.setDataRate(RATE_ADS1115_860SPS);
      }
    }
    if (isBetween(currentResistance, -10.0, 100.0) && currentMode != Voltmeter || currentMode == HighRMode || currentMode == rPlotMode) {
      voltageDisplay = false;
      ads.setDataRate(RATE_ADS1115_32SPS);
    }

    displayResistance = currentResistance - zeroOffsetRes;

    if (initialZeroSet) {
      if (displayResistance > highR && displayResistance < 10000000.0) {
        highR = displayResistance;
      }
      if (displayResistance < lowR && displayResistance > -1000.0) {
        lowR = displayResistance;
      }
    }

    if (Ireading != 0.0) {
      if (Ireading > IMedian) IMedian += 0.02;
      else IMedian -= 0.02;
    }
  }

  // Serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    handleSerialCommands(cmd);
  }

  // Continuous serial output (unchanged)
  if (serialMode && currentMillis - previousSerialMillis >= SERIAL_INTERVAL) {
    previousSerialMillis = currentMillis;
    if (ampsMode) {
      Serial.print("A: ");
      Serial.print(Ireading, 3);
      Serial.print("BREAK");
      Serial.print("I Low: ");
      Serial.print(ILow, 3);
      Serial.print(" (Vdc:");
      Serial.print(voltageAtMinI);
      Serial.print(") T:");
      Serial.print(timeAtMinI);
      Serial.print("BREAK");
      Serial.print("I High: ");
      Serial.print(IHigh, 3);
      Serial.print(" (Vdc:");
      Serial.print(voltageAtMaxI);
      Serial.print(") T:");
      Serial.print(timeAtMaxI);
      Serial.print("BREAK");
      Serial.print(" V:");
      Serial.print(roundedV, vDigits);
      Serial.print(vSuffix);
      Serial.print("VDC");
      Serial.print("BREAK");
      if (MinMaxDisplay) {
        Serial.print("Min:");
        Serial.print(roundedVlow, vDigitslow);
        Serial.print(vSuffixlow);
        Serial.print(" Max:");
        Serial.print(roundedVhigh, vDigitshigh);
        Serial.print(vSuffixhigh);
        Serial.print(" Range:");
        Serial.print(highV - lowV, 3);
      } else {
        Serial.print("Min Max Off");
      }
      Serial.print("BREAK");
      Serial.print("I@max:");
      Serial.print(IHigh, 3);
      Serial.print("A@");
      Serial.print(timeAtMaxI);
      Serial.print(" I@min:");
      Serial.print(ILow, 3);
      Serial.print("A@");
      Serial.print(timeAtMinI);
      Serial.println();
    } else if (voltageDisplay) {
      if (fabs(averageVoltage) < 0.001 && VAC < 0.2) {
        Serial.print("absV <1mV");
      } else {
        Serial.print(roundedV, vDigits);
        Serial.print(vSuffix);
        Serial.print("VDC");
      }
      Serial.print("BREAK");
      if (MinMaxDisplay) {
        Serial.print("Min:");
        Serial.print(roundedVlow, vDigitslow);
        Serial.print(vSuffixlow);
        Serial.print(" Max:");
        Serial.print(roundedVhigh, vDigitshigh);
        Serial.print(vSuffixhigh);
        Serial.print(" Range:");
        Serial.print(highV - lowV, 3);
      } else {
        Serial.print("Min Max Off");
      }
      Serial.print("BREAK");
      Serial.print("VAC_RMS:");
      Serial.print(VAC, 1);
      Serial.println();
    } else {
      if (ohmsAutoRange || ohmsVoltage < (ZENER_MAX_V - 0.005)) {
        Serial.print(displayResistance, rDigits);
        Serial.print(rSuffix);
        Serial.print(" Ohms");
      } else {
        Serial.print("R Open");
      }
      Serial.print("BREAK");
      if (MinMaxDisplay) {
        Serial.print("Min:");
        Serial.print(roundedRlow, rDigitslow);
        Serial.print(rSuffixlow);
        Serial.print(" Max:");
        Serial.print(roundedRhigh, rDigitshigh);
        Serial.print(rSuffixhigh);
      } else {
        Serial.print("Min Max Off");
      }
      Serial.print("BREAK");
      Serial.print("vR:");
      Serial.print(ohmsVoltage, 4);
      Serial.println();
    }
  }

  // Alerts (unchanged)
  updateAlerts();

  // RPC logic (unchanged)
  if (!voltageDisplay && isBetween(currentResistance, 20, 1000000)) {
    humFlag = 1;
  } else {
    humFlag = 0;
  }

  uint8_t pack = (txFlag ? 1 : 0) | (humFlag ? 2 : 0);
  if (pack != lastSent) {
    RPC.write(pack);
    lastSent = pack;
  }

  // Autolog (unchanged)
  if (!AutologTriggered && writePrimed) {
    autologArraysToCSV();
    analogWrite(CONTINUITY_PIN, 100);
    rgb.on(0, 255, 0);
    autologCount++;
    Serial.println("Values Autologged:");
    Serial.println(autologCount);
    writePrimed = 0;
  }

  if (
    (logMode && !VACPresense) && (
    (IHigh == Ireading && IHigh > 0.05) || (ILow == Ireading && ILow < -0.05) || (newVoltageReading == highV && highV > 0.05) || (newVoltageReading == lowV && lowV < -0.05) ||
    (abs(lastLoggedI) > (abs(Ireading) + Istep) || abs(lastLoggedI) < (abs(Ireading) - Istep))
    )
  )
  {
    float TimeS = (millis());
    tLogEnd  = millis() / 1000.0;
    logValues(newVoltageReading, (TimeS / 1000), Ireading);
    AutologTriggered = 1;
    writePrimed = 1;
    lastLoggedI = Ireading;
  } else {
    AutologTriggered = 0;

    // Update display at interval (skip if popup active)
    if ((currentMillis - previousLcdMillis >= LCD_INTERVAL) && !takeLog) {
      previousLcdMillis = currentMillis;
      LCD_INTERVAL = screenRefreshFast ? 500 : 1000;
      if (!modePopupActive) {
        updateDisplay();
      }
    }

    if (powerSave && newVoltageReading < 0.1 && !VACPresense && Ireading == 0) {
      if (!deepSleepTrigger) {
        deepSleepStart = millis();
        deepSleepTrigger = true;
      } else {
        if (millis() - deepSleepStart > 300000) {
          screenSleep = true;
        }
      }
    } else {
      screenSleep = false;
      deepSleepTrigger = false;
    }
  }
}

// ======================= INPUT HANDLING (unchanged) =======================

void handleSerialCommands(char command) {
  switch (command) {
    case 'Q':
      Serial.print("Res: ");
      if (displayResistance > -100.0 && displayResistance < 8000000.0) {
        Serial.print(displayResistance);
      } else {
        Serial.print("OPEN");
      }
      if (debugMode) {
        Serial.print(" Vr: ");
        Serial.print(ohmsVoltage, 4);
      }
      Serial.print(" Vdc: ");
      Serial.print(roundedV, vDigits);
      Serial.print(vSuffix);
      Serial.print(" I: ");
      Serial.print(Ireading * 1000.0, 0);
      Serial.println(" mA");
      break;
    case 'q':
      serialMode = !serialMode;
      if (serialMode) Serial.println("Serial mode ON");
      break;
    case 'Z':
      if (!initialZeroSet || fabs(zeroOffsetRes) < 1e-6) {
        zeroOffsetRes = currentResistance;
        initialZeroSet = true;
        Serial.print("Zero set to ");
        Serial.println(zeroOffsetRes);
      } else {
        zeroOffsetRes = 0.0;
        Serial.println("Zero offset cleared");
      }
      break;
    case 'C':
      zeroOffsetRes = 0.0;
      Serial.println("Zero offset cleared");
      break;
    case 'R':
      ohmsHighRange = !ohmsHighRange;
      Serial.print("Manual Range now: ");
      Serial.println(ohmsHighRange ? "High" : "Low");
      break;
    case 'S':
      smoothVmode = !smoothVmode;
      g_buttons[BTN_SMOOTH].state = smoothVmode;
      Serial.print("Smooth mode: ");
      Serial.println(smoothVmode ? "On" : "Off");
      break;
    case 'V':
      voltageDisplay = !voltageDisplay;
      Serial.println("Display mode toggled");
      break;
    case 'A':
      ohmsAutoRange = !ohmsAutoRange;
      g_buttons[BTN_RANGE].state = !ohmsAutoRange;
      Serial.print("Auto-range: ");
      Serial.println(ohmsAutoRange ? "On" : "Off");
      break;
    case 'F':
      screenRefreshFast = !screenRefreshFast;
      g_buttons[BTN_FAST].state = screenRefreshFast;
      Serial.print("Screen refresh: ");
      Serial.println(screenRefreshFast ? "Fast" : "Normal");
      break;
    case 'M':
      vDisplayManual = !vDisplayManual;
      Serial.print("Manual voltage display: ");
      Serial.println(vDisplayManual ? "On" : "Off");
      break;
    case 'B':
      Serial.print("Battery: ");
      Serial.print(batteryVoltage, 2);
      Serial.println(" V");
      break;
    case 'D':
      debugMode = !debugMode;
      Serial.print("Debug mode: ");
      Serial.println(debugMode ? "On" : "Off");
      break;
    case 'e':
      MinMaxDisplay = !MinMaxDisplay;
      g_buttons[BTN_MINMAX].state = MinMaxDisplay;
      Serial.print("Min/Max display: ");
      Serial.println(MinMaxDisplay ? "On" : "Off");
      break;
    case 'E':
      ReZero();
      Serial.println("Min/Max reset");
      break;
    case 'I':
      ampsMode = !ampsMode;
      g_buttons[BTN_AMPS].state = ampsMode;
      Serial.print("Current mode: ");
      Serial.println(ampsMode ? "On" : "Off");
      break;
    case 'L':
      Serial.print("LOG: I, V@I, T");
      Serial.print("BREAK");
      Serial.print("I:");
      for (int i = 0; i < LOG_SIZE; ++i) {
        Serial.print(loggedCurrents[i], 2);
        if (i < LOG_SIZE - 1) Serial.print(", ");
      }
      Serial.print("/");
      Serial.print("BREAK");
      Serial.print("V:");
      for (int i = 0; i < LOG_SIZE; ++i) {
        Serial.print(loggedVoltagesAtI[i], 3);
        if (i < LOG_SIZE - 1) Serial.print(", ");
      }
      Serial.print("/");
      Serial.print("BREAK");
      Serial.print("T:");
      for (int i = 0; i < LOG_SIZE; ++i) {
        Serial.print(loggedTimeStamps[i], 3);
        if (i < LOG_SIZE - 1) Serial.print(", ");
      }
      Serial.print("/");
      Serial.println();
      break;
    case 'l':
      takeLog = true;
      tLogStart  = millis() / 1000.0;
      break;
    case 'k':
      Serial.print("Time Start (s):");
      Serial.println(tLogStart, 3);
      Serial.println("Voltages");
      for (int i = 0; i < LOG_SIZE; ++i) {
        Serial.print(loggedVoltagesAtI[i], 3);
        if (i < LOG_SIZE - 1) Serial.print(", ");
      }
      Serial.println("");
      Serial.println("Times");
      for (int i = 0; i < LOG_SIZE; ++i) {
        Serial.print(loggedTimeStamps[i], 3);
        if (i < LOG_SIZE - 1) Serial.print(", ");
      }
      Serial.println("");
      Serial.print("TimeEnd (s):");
      Serial.println(tLogEnd, 3);
      break;
    case 'j':
      Serial.print("Time Start (s):");
      Serial.println(tLogStart, 3);
      Serial.println("Voltages");
      for (int i = 0; i < LOG_SIZE; ++i) {
        Serial.print(loggedVoltagesAtI[i], 3);
        if (i < LOG_SIZE - 1) Serial.print(", ");
      }
      Serial.println("");
      Serial.println("Currents");
      for (int i = 0; i < LOG_SIZE; ++i) {
        Serial.print(loggedCurrents[i], 3);
        if (i < LOG_SIZE - 1) Serial.print(", ");
      }
      Serial.println("");
      Serial.println("Times");
      for (int i = 0; i < LOG_SIZE; ++i) {
        Serial.print(loggedTimeStamps[i], 3);
        if (i < LOG_SIZE - 1) Serial.print(", ");
      }
      Serial.println("");
      Serial.print("TimeEnd (s):");
      Serial.println(tLogEnd, 3);
      break;
    case '?':
      Serial.print("Commands: Q (query once), q (toggle continuous), Z (zero on/off),");
      Serial.print("BREAK");
      Serial.print("C (clear zero), R (range toggle), S (smooth toggle), V (display toggle),");
      Serial.print("BREAK");
      Serial.print("A (auto-range toggle), F (fast refresh), M (manual V mode),");
      Serial.print("BREAK");
      Serial.print("B (battery), D (debug), e (min/max toggle), E (reset min/max),");
      Serial.print("BREAK");
      Serial.println("I (current mode toggle), L (print log).");
      break;
    default:
      break;
  }
}

void handleButtonInput() {
  bool isPressed = (digitalRead(TYPE_PIN) == LOW);
  if (isPressed && !buttonPressed) {
    buttonPressed = true;
    buttonPressTime = millis();
  }
  if (!isPressed && buttonPressed) {
    unsigned long pressDuration = millis() - buttonPressTime;
    buttonPressed = false;
    analogWrite(CONTINUITY_PIN, 0);
    if (pressDuration > 400) {
      ReZero();
      MinMaxDisplay = true;
      g_buttons[BTN_MINMAX].state = true;
    } else if (pressDuration > 50) {
      if (currentMode == Type) {
        if (voltageDisplay) {
          Keyboard.printf("%.*f", vDigits, newVoltageReading);
        } else {
          if (displayResistance < 1) {
            Keyboard.printf("%.*f", rDigits + 2, displayResistance);
          } else {
            Keyboard.printf("%.*f", rDigits, displayResistance);
          }
        }
        Keyboard.key_code(RIGHT_ARROW);
      } else {
        if (voltageDisplay) {
          deltaVdigits = vDigits - 1;
          if (preciseMode) {
            deltaV = newVoltageReading;
          } else {
            deltaV = averageVoltage;
          }
        } else {
          if (zeroOffsetRes == 0) {
            zeroOffsetRes = currentResistance;
          } else {
            zeroOffsetRes = 0;
          }
        }
      }
    }
  }
  if (isPressed && flashlightMode && (millis() - buttonPressTime) > 500) {
    // flashlight feedback handled by updateAlerts
  }
}

// ======================= MEASUREMENT FUNCTIONS (unchanged) =======================

float OHMS_RANGE_THRESHOLD   = 400.0f;
static const float OHMS_RANGE_DEADBAND    = 0.05f;
static const float OHMS_LOW_THRESHOLD     = OHMS_RANGE_THRESHOLD * (1.0f - OHMS_RANGE_DEADBAND);
static const float OHMS_HIGH_THRESHOLD    = OHMS_RANGE_THRESHOLD * (1.0f + OHMS_RANGE_DEADBAND);

static const int ADC_COUNT_LOW_THRESHOLD  = 10000;
static const int ADC_COUNT_HIGH_THRESHOLD = 30000;

static const float kGainFactors[] = {GAIN_FACTOR_TWOTHIRDS, GAIN_FACTOR_1, GAIN_FACTOR_2, GAIN_FACTOR_4, GAIN_FACTOR_8, GAIN_FACTOR_16};
static const int   kNumGainLevels = sizeof(kGainLevels) / sizeof(kGainLevels[0]);

void measureResistance() {
  static float   prevResistance = 0.0f;
  static bool    firstRun       = true;
  static bool    currentRangeHigh;

  prevResistance = rawResistance;

  if (firstRun) {
    currentRangeHigh = (digitalRead(SETRANGE_PIN) == LOW);
    gainIndex        = currentRangeHigh ? 0 : (kNumGainLevels - 1);
    firstRun         = false;
  }

  if (currentMode != rPlotMode || currentMode != HighRMode) {
    float OHMS_RANGE_THRESHOLD   = 400.0f;
  } else {
    float OHMS_RANGE_THRESHOLD   = 40.0f;
  }

  if (ohmsAutoRange && !powerSave) {
    if (!currentRangeHigh && prevResistance > OHMS_HIGH_THRESHOLD) {
      currentRangeHigh = true;
      digitalWrite(SETRANGE_PIN, HIGH);
    } else if (currentRangeHigh && prevResistance < OHMS_LOW_THRESHOLD) {
      currentRangeHigh = false;
      digitalWrite(SETRANGE_PIN, LOW);
    }
  } else if (powerSave) {
    currentRangeHigh = true;
    digitalWrite(SETRANGE_PIN, HIGH);
  } else {
    if (ohmsHighRange && !currentRangeHigh) {
      currentRangeHigh = true;
      digitalWrite(SETRANGE_PIN, HIGH);
    } else if (!ohmsHighRange && currentRangeHigh) {
      currentRangeHigh = false;
      digitalWrite(SETRANGE_PIN, LOW);
    }
  }

  ads.setGain(kGainLevels[gainIndex]);
  adcCount = ads.readADC_SingleEnded(2);

  if (adcCount > ADC_COUNT_HIGH_THRESHOLD && gainIndex > 0) {
    gainIndex--;
    ads.setGain(kGainLevels[gainIndex]);
    adcCount = ads.readADC_SingleEnded(2);
  } else if (adcCount < ADC_COUNT_LOW_THRESHOLD && gainIndex < kNumGainLevels - 1) {
    gainIndex++;
    ads.setGain(kGainLevels[gainIndex]);
    adcCount = ads.readADC_SingleEnded(2);
  }

  ohmsVoltage = adcCount * kGainFactors[gainIndex] / 1000.0f;

  if (powerSave) {
    ZENER_MAX_V = EEPROM_SleepV;
  } else {
    if (ohmsVoltage > ZENER_MAX_V) {
      ohmsVoltage = ZENER_MAX_V - 0.0001;
    } else {
      ZENER_MAX_V = EEPROM_MAXV;
    }
  }

  if (currentRangeHigh) {
    rawResistance = dividerR * (ohmsVoltage / (ZENER_MAX_V - ohmsVoltage));
  } else {
    rawResistance = ohmsVoltage / (constantI - (ohmsVoltage / constantR));
  }

  if (!initialZeroSet) {
    analogWrite(OHMPWMPIN, 0);
    if (currentResistance > 0.001f && currentResistance < 10.0f) {
      zeroOffsetRes    = currentResistance;
      initialZeroSet   = true;
      Serial.print("Auto-zero set: ");
      Serial.println(zeroOffsetRes);
    } else if (currentResistance > 10.0f && currentResistance < 1e6f) {
      initialZeroSet = true;
      Serial.println("No auto zero applied");
    }
  }

  if (!isBetween(rawResistance, prevResistance * 0.2f, prevResistance * 5.0f) && ohmsVoltage < 4.995) {
    ads.setDataRate(RATE_ADS1115_475SPS);
    if (!powerSave) {
      LCD_INTERVAL = 200;
    }
  } else if (!voltageDisplay && preciseMode && rawResistance < 3e6f
            && isBetween(rawResistance, prevResistance * 0.98f, prevResistance * 1.02f)) {
    ads.setDataRate(RATE_ADS1115_16SPS);
  } else if (!voltageDisplay && (ohmsVoltage < (ZENER_MAX_V - 0.02f))) {
    ads.setDataRate(RATE_ADS1115_128SPS);
  }

  if      (rawResistance < 0.75f)    calibratedResistance = rawResistance * CF_A;
  else if (rawResistance < 3.0f)     calibratedResistance = rawResistance * CF_B;
  else if (rawResistance < 7.0f)     calibratedResistance = rawResistance * CF_C;
  else if (rawResistance < 20.0f)    calibratedResistance = rawResistance * CF_D;
  else if (rawResistance < 70.0f)    calibratedResistance = rawResistance * CF_E;
  else if (rawResistance < 170.0f)   calibratedResistance = rawResistance * CF_F;
  else if (rawResistance < 700.0f)   calibratedResistance = rawResistance * CF_G;
  else if (rawResistance < 1700.0f)  calibratedResistance = rawResistance * CF_H;
  else if (rawResistance < 7000.0f)  calibratedResistance = rawResistance * CF_I;
  else if (rawResistance < 17000.0f) calibratedResistance = rawResistance * CF_J;
  else if (rawResistance < 70000.0f) calibratedResistance = rawResistance * CF_K;
  else if (rawResistance < 170000.0f) calibratedResistance = rawResistance * CF_L;
  else if (rawResistance < 700000.0f) calibratedResistance = rawResistance * CF_M;
  else if (rawResistance < 1700000.0f) calibratedResistance = rawResistance * CF_N;
  else calibratedResistance = rawResistance * CF_O;

  currentResistance = calibratedResistance;
  if (altUnits) {
    currentResistance = (1.0f / ((1.0f / 298.15f) + (log(currentResistance / 10000.0f) / 3694.0f)) - 273.15f) * 1.8f + 32.0f;
  }

  resistanceSamples[resistanceSampleIndex] = ohmsVoltage;
  resistanceSampleIndex = (resistanceSampleIndex + 1) % NUM_RESISTANCE_SAMPLES;

  if (currentMode != HighRMode) {
    if (ohmsVoltage > (ZENER_MAX_V - 0.007f)) {
      currentResistance = 8e6f;
      if (!voltageDisplay) {
        ads.setDataRate(RATE_ADS1115_475SPS);
      }
    }
  }
}

void measureVoltage() {
  static float   prevVoltage     = 0.0f;
  static bool    firstVoltRun    = true;
  static size_t  gainIndexVolt;

  prevVoltage = newVoltageReading;

  if (firstVoltRun) {
    gainIndexVolt = kNumGainLevels - 1;
    firstVoltRun  = false;
  }

  if (VACPresense || currentMode == VACmanual) {
    ads.setGain(GAIN_ONE);
    countV = ads.readADC_Differential_0_1();
    newVoltageReading = ((countV * GAIN_FACTOR_1 / 1000.0f) * VOLTAGE_SCALE) - giga_absfactor;
  } else if (!altUnits) {
    ads.setGain(kGainLevels[gainIndexVolt]);
    countV = ads.readADC_Differential_0_1();
    if (abs(countV) > ADC_COUNT_HIGH_THRESHOLD && gainIndexVolt > 0) {
      --gainIndexVolt;
      ads.setGain(kGainLevels[gainIndexVolt]);
      countV = ads.readADC_Differential_0_1();
    } else if (abs(countV) < ADC_COUNT_LOW_THRESHOLD && gainIndexVolt < kNumGainLevels - 1) {
      ++gainIndexVolt;
      ads.setGain(kGainLevels[gainIndexVolt]);
      countV = ads.readADC_Differential_0_1();
    }
    if (countV > 0) {
      newVoltageReading = ((countV * kGainFactors[gainIndexVolt] / 1000.0f) * VOLTAGE_SCALE) - giga_absfactor;
    } else {
      newVoltageReading = ((countV * kGainFactors[gainIndexVolt] / 1000.0f) * VOLTAGE_SCALE_Negative) - giga_absfactor;
    }
  } else {
    ads.setGain(GAIN_EIGHT);
    countV = ads.readADC_Differential_0_1();
    newVoltageReading = ((countV * GAIN_FACTOR_8 / 1000.0f) * VOLTAGE_SCALE) - giga_absfactor;
  }

  if (altUnits) {
    newVoltageReading = (newVoltageReading * 50);
  }

  if (newVoltageReading > highV) {
    highV = newVoltageReading;
    timeAtMaxV = formatTime(millis());
    currentAtMaxV = Ireading;
    LCD_INTERVAL = 200;
    if (MinMaxDisplay && voltageDisplay) {
      analogWrite(CONTINUITY_PIN, 200);
      rgb.on(0, 255, 0);
    }
  }
  if (newVoltageReading < lowV) {
    lowV = newVoltageReading;
    timeAtMinV = formatTime(millis());
    currentAtMinV = Ireading;
    LCD_INTERVAL = 200;
  }

  medianVoltageStep = (newVoltageReading - medianVoltage) * 0.2;
  medianVoltage += medianVoltageStep;

  if (!VACPresense && fabs(medianVoltageStep) > 0.3 && !preciseMode && newVoltageReading > 0.5) {
    LCD_INTERVAL = 200;
  } else if (preciseMode && voltageDisplay) {
    LCD_INTERVAL = 1000;
  }

  voltageSum -= voltageSamples[voltageSampleIndex];
  squaredVoltageSum -= voltageSquaredSamples[voltageSampleIndex];
  voltageSamples[voltageSampleIndex] = newVoltageReading;
  float diff = newVoltageReading - averageVoltage;
  voltageSquaredSamples[voltageSampleIndex] = diff * diff;
  voltageSum += newVoltageReading;
  squaredVoltageSum += voltageSquaredSamples[voltageSampleIndex];
  voltageSampleIndex = (voltageSampleIndex + 1) % NUM_VOLTAGE_SAMPLES;
  averageVoltage = voltageSum / NUM_VOLTAGE_SAMPLES;
  VAC = sqrt(squaredVoltageSum / NUM_VOLTAGE_SAMPLES);
  if (preciseMode) {
    VAC = 0;
  }

  if ((VAC > 4.9 && (!altUnits && averageVoltage < 0.1) || (altUnits && VAC > 10)) || currentMode == VACmanual) {
    VACPresense = true;
  } else {
    VACPresense = false;
  }

  if (((fabs(averageVoltage) < 0.03 && currentMode != VACmanual && fabs(newVoltageReading) < 0.2) || (currentMode == VACmanual && VAC < 5)) && !preciseMode && voltageDisplay) {
    Vzero = true;
    ClosedOrFloat();
  } else {
    Vzero = false;
    vFloating = false;
  }
}

void measureCurrent() {
  static float prevCurrent        = 0.0f;
  static bool  firstCurrentRun    = true;
  static size_t gainIndexCurrent;

  prevCurrent = Ireading;

  if (firstCurrentRun) {
    gainIndexCurrent  = kNumGainLevels - 1;
    firstCurrentRun   = false;
  }

  if (IHigh) {
    ads.setGain(GAIN_TWOTHIRDS);
    countI = ads.readADC_SingleEnded(3);
    currentShuntVoltage = (countI * GAIN_FACTOR_TWOTHIRDS / 1000.0f);
    Ireading = (currentShuntVoltage - Izero) / 0.185f;
  } else {
    ads.setGain(kGainLevels[gainIndexCurrent]);
    countI = ads.readADC_SingleEnded(3);

    if (abs(countI) > ADC_COUNT_HIGH_THRESHOLD && gainIndexCurrent > 0) {
      --gainIndexCurrent;
      ads.setGain(kGainLevels[gainIndexCurrent]);
      countI = ads.readADC_SingleEnded(3);
    } else if (abs(countI) < ADC_COUNT_LOW_THRESHOLD
               && gainIndexCurrent < kNumGainLevels - 1) {
      ++gainIndexCurrent;
      ads.setGain(kGainLevels[gainIndexCurrent]);
      countI = ads.readADC_SingleEnded(3);
    }

    Ireading = currentShuntVoltage / 1.0f;
    currentShuntVoltage = (countI * kGainFactors[gainIndexCurrent] / 1000.0f);
    Ireading = currentShuntVoltage / 1;
  }

  if ((Irange  && isBetween(Ireading, -0.01f,  0.01f)) ||
      (!Irange  && Ireading < 0.0005f)) {
    Ireading = 0.0f;
  } else {
    if (Ireading > IHigh || Ireading < ILow) {
      float nowSec = millis() / 1000.0f;
      logCurrentData(newVoltageReading, nowSec, Ireading);
      if (Ireading > IHigh) {
        IHigh          = Ireading;
        voltageAtMaxI  = newVoltageReading;
        timeAtMaxI     = formatTime(millis());
      }
      if (Ireading < ILow) {
        ILow           = Ireading;
        voltageAtMinI  = newVoltageReading;
        timeAtMinI     = formatTime(millis());
      }
    }
  }

  currentSamples[currentSampleIndex] = Ireading;
  currentSampleIndex = (currentSampleIndex + 1) % NUM_CURRENT_SAMPLES;
}

// ======================= DISPLAY UPDATE (rewritten for dark theme) =======================
void updateDisplay() {
  // Update info bar
  drawInfoBar();

  blinkLimit = 0;

  // Clear main text area (above plot, left of buttons)
  display.fillRect(0, INFO_H, BTN_PANEL_X, PLOT_Y - INFO_H, COL_BG);

  int gfxLine = 8;
  int PrimeX = 10;
  int PrimeY = INFO_H + 2; // 38

  // Right column positions (for min/max and current)
  int mmHomeX = 200;
  int mmHomeY = INFO_H + 2;
  int IHomeX = 420;
  int IHomeY = INFO_H + 2;

  if (!screenSleep && currentMode != Charging) {

    // Determine voltage to display
    float voltageToDisplay;
    if (!preciseMode) {
      voltageToDisplay = (fabs(medianVoltageStep) > 0.5) ? newVoltageReading : medianVoltage;
    } else {
      voltageToDisplay = newVoltageReading;
    }

    // Format values
    formatVoltageValue(voltageToDisplay, roundedV, vSuffix, vDigits);
    formatVoltageValue(lowV, roundedVlow, vSuffixlow, vDigitslow);
    formatVoltageValue(highV, roundedVhigh, vSuffixhigh, vDigitshigh);
    formatResistanceValue(displayResistance, roundedR, rSuffix, rDigits);
    formatResistanceValue(lowR, roundedRlow, rSuffixlow, rDigitslow);
    formatResistanceValue(highR, roundedRhigh, rSuffixhigh, rDigitshigh);

    // Current display
    if (currentOnOff) {
      display.fillRect(IHomeX, IHomeY, 200, 40, COL_BG);
      display.setTextSize(2);
      display.setTextColor(COL_BTN_AMPS);
      display.setCursor(IHomeX, IHomeY);
      if (Irange) {
        display.print("A:");
        display.print(Ireading, 2);
      } else {
        display.print("mA:");
        if (Ireading > 0.100) {
          display.print(Ireading * 1000.0, 2);
        } else if (Ireading > 0.010) {
          display.print(Ireading * 1000.0, 3);
        } else {
          display.print(Ireading * 1000.0, 4);
        }
      }

      if (IHigh != -6.0) {
        display.setTextColor(COL_SUBTEXT);
        display.setTextSize(1);
        display.setCursor(IHomeX, IHomeY + 18);
        display.print("Lo:");
        display.print(ILow, 3);
        display.print(" Hi:");
        display.print(IHigh, 3);
      }
    }

    // Primary measurement
    display.setTextColor(COL_DATA);
    display.setTextSize(3);
    display.setCursor(PrimeX, PrimeY);

    if (voltageDisplay) {
      if (Vzero) {
        display.setTextColor(COL_BTN_ZERO);
        if (vFloating) {
          display.print("V FLT:");
        } else if (vUndefined) {
          display.print("V UNDF:");
        } else {
          display.print("V CLD:");
        }
        display.print(bridgeV * 1000, 0);
        display.print("m");
      } else {
        if (currentMode == VACmanual) {
          display.print("VAC:");
          if (altUnits) {
            display.print(VAC, 1);
          } else {
            display.print(VAC, 3);
          }
        } else {
          display.print("VDC:");
          if (preciseMode) {
            display.print(roundedV, (vDigits + 1));
            display.println(vSuffix);
          } else {
            display.print(roundedV, vDigits);
            display.println(vSuffix);
          }
        }

        // Delta/Reference info
        if (deltaV != 0) {
          display.setTextColor(COL_BTN_ZERO);
          display.setTextSize(2);
          display.setCursor(PrimeX, PrimeY + 26);
          display.print("ref:");
          display.print(deltaV, deltaVdigits);
          display.print(" d:");
          display.print(newVoltageReading - deltaV, 4);
        }
      }

      // Min/Max display
      if (MinMaxDisplay) {
        display.fillRect(mmHomeX, mmHomeY, 210, 40, COL_BG);
        display.setTextColor(COL_SUBTEXT);
        display.setTextSize(2);
        display.setCursor(mmHomeX, mmHomeY);
        display.print("Lo:");
        display.print(roundedVlow, vDigitslow);
        display.print(vSuffixlow);
        display.setCursor(mmHomeX, mmHomeY + 18);
        display.print("Hi:");
        display.print(roundedVhigh, vDigitshigh);
        display.print(vSuffixhigh);
      }

      // VAC/VDC secondary (below primary, in plot overlay area)
      if (currentMode != Low) {
        display.setTextColor(COL_SUBTEXT);
        display.setTextSize(2);
        if (currentMode == VACmanual) {
          display.setCursor(PrimeX, PLOT_Y);
          display.print("VDC: ");
          display.print(roundedV, vDigits);
        } else {
          display.setCursor(PrimeX, PLOT_Y);
          display.print("VAC: ");
          display.print(VAC, vDigits);
        }
      }

      // Plot
      if (!vFloating) {
        display.setTextSize(2);
        drawPlot(voltageSamples, SAMPLE_COUNT_PLOT);
      }

    } else {
      // Resistance display
      if (ohmsVoltage < (ZENER_MAX_V - 0.007) || currentMode == HighRMode) {
        if (altUnits) {
          display.print("F:");
        } else {
          display.print("R:");
        }
        display.print(roundedR, rDigits);
        display.print(rSuffix);

        if (MinMaxDisplay) {
          display.fillRect(mmHomeX, mmHomeY, 210, 40, COL_BG);
          display.setTextColor(COL_SUBTEXT);
          display.setTextSize(2);
          display.setCursor(mmHomeX, mmHomeY);
          display.print("Lo:");
          display.print(roundedRlow, rDigitslow);
          display.print(rSuffixlow);
          display.setCursor(mmHomeX, mmHomeY + 18);
          display.print("Hi:");
          display.print(roundedRhigh, rDigitshigh);
          display.print(rSuffixhigh);
        }

        // Voltage drop info
        display.setTextColor(COL_SUBTEXT);
        display.setTextSize(2);
        display.setCursor(PrimeX, PrimeY + 26);
        display.print("mVR:");
        if (ohmsVoltage < 1) {
          display.print(ohmsVoltage * 1000.0, 1);
        } else {
          display.print(ohmsVoltage * 1000.0, 0);
        }

        if (zeroOffsetRes != 0) {
          display.setCursor(PrimeX + 120, PrimeY + 26);
          display.print("null:");
          display.print(zeroOffsetRes * 1000, 1);
          display.print("m");
        }
      } else {
        // Open circuit
        if (!altUnits) {
          display.print("R:OPEN");
          display.setTextColor(COL_SUBTEXT);
          display.setTextSize(2);
          display.setCursor(PrimeX, PrimeY + 26);
          display.print("mV:");
          display.print(ohmsVoltage * 1000.0, 2);
        } else {
          display.print("F:OPEN");
        }
        if (MinMaxDisplay) {
          display.fillRect(mmHomeX, mmHomeY, 210, 40, COL_BG);
          display.setTextColor(COL_SUBTEXT);
          display.setTextSize(2);
          display.setCursor(mmHomeX, mmHomeY);
          display.print("Lo:");
          display.print(roundedRlow, rDigitslow);
          display.print(rSuffixlow);
          display.setCursor(mmHomeX, mmHomeY + 18);
          display.print("Hi:");
          display.print(roundedRhigh, rDigitshigh);
          display.print(rSuffixhigh);
        }
      }

      if (currentMode == rPlotMode || currentMode == HighRMode) {
        display.setTextSize(2);
        drawPlot(resistanceSamples, SAMPLE_COUNT_PLOT);
      }
    }
  } else {
    // Screen sleep / Charging mode - minimal display
    display.fillRect(0, INFO_H, BTN_PANEL_X, SCREEN_H - INFO_H, COL_BG);
    formatTime(millis());
    int step = secondsTime % 10;
    display.setCursor((8 + (4 * step)), INFO_H + 20);
    display.setTextColor(COL_SUBTEXT);
    display.setTextSize(1);
    display.print(".");
  }
}

// ======================= ALERTS (unchanged) =======================
void updateAlerts() {
  unsigned long now = millis();
  if (!flashlightMode) {
    continuity = (isBetween(currentResistance, -20.0, 1.0) ||
                      (isBetween(currentResistance, -20.0, 20.0) && ohmsHighRange) ||
                      (Vzero && vClosed) && blinkLimit < 2);

    bool logicVoltage = ((!altUnits && (fabs(newVoltageReading) > 3.2 || (VACPresense && VAC > 1.0))) ||
                         (altUnits && (fabs(newVoltageReading) > 30.2 || (VACPresense && VAC > 15.0))) &&
                         blinkLimit < 2);
    if (continuity) {
      txFlag = 0;
      if (!rFlag) {
        analogWrite(CONTINUITY_PIN, 200);
        rgb.on(255, 255, 255);
        blinkLimit++;
        rFlag = true;
      } else if ((now % 1000 <= 100 || isBetween(now % 1000, 300, 400)) && rFlag) {
        analogWrite(CONTINUITY_PIN, 50);
        rgb.on(0, 0, 255);
      } else {
        analogWrite(CONTINUITY_PIN, 0);
        rgb.off();
      }
    } else if (logicVoltage) {
      txFlag = 1;
      if (!vFlag) {
        analogWrite(CONTINUITY_PIN, 200);
        rgb.on(255, 255, 255);
        blinkLimit++;
        vFlag = true;
      } else if ((now % 1000 <= 100) && vFlag && (!VACPresense)) {
        analogWrite(CONTINUITY_PIN, 50);
        rgb.on(255, 0, 0);
      } else {
        analogWrite(CONTINUITY_PIN, 0);
        rgb.off();
      }
    } else {
      analogWrite(CONTINUITY_PIN, 0);
      rgb.off();
      vFlag = false;
      rFlag = false;
    }
  } else {
    float pwmValue;
    if (currentResistance >= 0.0 && currentResistance <= 10000.0) {
      pwmValue = currentResistance;
    } else if (currentResistance < 0.0 || currentResistance > 3000000.0) {
      pwmValue = 0.0;
    } else {
      pwmValue = 10000.0;
    }
    int brightness = 255 - (int)round(255 * (pwmValue / 10000.0));
    analogWrite(CONTINUITY_PIN, brightness);
  }
}

// ======================= UTILITY FUNCTIONS (unchanged) =======================

void logCurrentData(float voltage, float timeSec, float current) {
  for (int i = LOG_SIZE - 1; i > 0; --i) {
    loggedCurrents[i] = loggedCurrents[i - 1];
    loggedVoltagesAtI[i] = loggedVoltagesAtI[i - 1];
    loggedTimeStamps[i] = loggedTimeStamps[i - 1];
  }
  loggedCurrents[0] = current;
  loggedVoltagesAtI[0] = voltage;
  loggedTimeStamps[0] = timeSec - tLogStart;
}

void formatResistanceValue(float value, float &outValue, String &outSuffix, int &outDigits) {
  if (value > 90000000.0) {
    outValue = value / 1000000.0;
    outSuffix = "M";
    outDigits = 1;
  } else if (value > 9000000.0) {
    outValue = value / 1000000.0;
    outSuffix = "M";
    outDigits = 2;
  } else if (value > 900000.0) {
    outValue = value / 1000000.0;
    outSuffix = "M";
    outDigits = 3;
  } else if (value > 900.0) {
    outValue = value / 1000.0;
    outSuffix = "k";
    outDigits = (value > 90000.0) ? 3 : 4;
  } else {
    if (value < 0.9) {
      outValue = value * 1000.0;
      outSuffix = "m";
      if (value < 0.09) {
        outDigits = 1;
      } else {
        outDigits = 2;
      }
    } else {
      outValue = value;
      outSuffix = "";
      if (value >= 100.0) {
        outDigits = 2;
      } else if (value < 9.5) {
        outDigits = 4;
      } else {
        outDigits = 3;
      }
    }
  }
}

void formatVoltageValue(float value, float &outValue, String &outSuffix, int &outDigits) {
  if (fabs(value) < 0.9) {
    outValue = value * 1000.0;
    outSuffix = "m";
    outDigits = 0;
  } else {
    outValue = value;
    outSuffix = "";
    outDigits = (fabs(value) > 11.0) ? 2 : 3;
  }
}

bool isBetween(float value, float low, float high) {
  return (value >= low && value <= high);
}

String formatTime(unsigned long milliseconds) {
  unsigned long totalSeconds = milliseconds / 1000;
  unsigned int minutes = totalSeconds / 60;
  secondsTime = totalSeconds % 60;
  char buf[6];
  snprintf(buf, sizeof(buf), "%02u:%02u", minutes, secondsTime);
  return String(buf);
}

void ReZero() {
  highR = 0.0;
  lowR = 7000000.0;
  highV = -100.0;
  lowV = 100.0;
  IHigh = -6.0;
  ILow = 6.0;
  IMedian = 0.0;
  deltaV = 0;
}

void checkModeButton() {
  bool buttonState = digitalRead(BUTTON_PIN);
  unsigned long currentTime = millis();

  // Physical button or micPin opens popup instead of cycling
  if ((buttonState == LOW || isBetween(analogRead(micPin), 300, 450)) && !buttonPreviouslyPressed && (currentTime - lastDebounceTime + 100 > DEBOUNCE_DELAY)) {
    lastDebounceTime = currentTime;
    buttonPreviouslyPressed = true;

    modePopupActive = true;
    drawModePopup();
    Serial.println("Mode popup opened (physical)");
  }

  if (buttonState == HIGH && buttonPreviouslyPressed) {
    buttonPreviouslyPressed = false;
  }
}

const char* modeToString(Mode m) {
  switch (m) {
    case Default:       return "Default";
    case Voltmeter:     return "Voltmeter";
    case VACmanual:     return "VAC";
    case Type:          return "Type";
    case Low:           return "Low";
    case AltUnitsMode:  return "AltUnits";
    case HighRMode:     return "High R";
    case rPlotMode:     return "rPlotMode";
    case Charging:      return "Charging";
    default:            return "Unknown";
  }
}

// ======================= PLOT FUNCTIONS (unchanged) =======================

void drawPlot(const float data[], int n) {
  float minY = data[0], maxY = data[0], sumY = data[0];
  for (int i = 1; i < n; i++) {
    float v = data[i];
    sumY += v;
    if (v < minY) minY = v;
    if (v > maxY) maxY = v;
  }
  previousmeanY = meanY;
  meanY = sumY / n;
  float rangeY = 1;

  if (!MinMaxDisplay) {
    if (maxY - minY < 0.05 && voltageDisplay && currentMode == Low) {
      rangeY = 0.1;
    } else if (maxY - minY < 0.2 && voltageDisplay && currentMode != Low) {
      rangeY = 0.2;
    } else {
      rangeY = maxY - minY;
    }
  } else {
    minY = lowV;
    maxY = highV;
    rangeY = maxY - minY;
  }

  float margin = rangeY * 0.05f;
  float plotMin = minY - margin * 0.5f;
  float plotMax = maxY + margin * 0.5f;
  float yScale  = float(PLOT_SIZE) / (plotMax - plotMin);

  display.fillRect(PLOT_X, PLOT_Y, PLOT_SIZE + 100, PLOT_SIZE, COLOR_BG);

  for (int i = 0; i < n - 1; i++) {
    int x1 = PLOT_X + (i     * PLOT_SIZE) / (n - 1);
    int y1 = PLOT_Y + PLOT_SIZE - int((data[i]     - plotMin) * yScale);
    int x2 = PLOT_X + ((i + 1) * PLOT_SIZE) / (n - 1);
    int y2 = PLOT_Y + PLOT_SIZE - int((data[i + 1] - plotMin) * yScale);
    display.drawLine(x1, y1, x2, y2, COLOR_DATA);
  }

  drawStatLine(minY, plotMin, yScale, COLOR_MIN);
  drawStatLine(meanY, plotMin, yScale, COLOR_MEAN);
  drawStatLine(maxY, plotMin, yScale, COLOR_MAX);
}

void drawStatLine(float value, float plotMin, float yScale, uint16_t color) {
  int y = PLOT_Y + PLOT_SIZE - int((value - plotMin) * yScale);
  const int dashLen = 6;
  for (int x = PLOT_X; x < PLOT_X + PLOT_SIZE; x += dashLen) {
    display.drawFastHLine(x, y, dashLen / 2, color);
  }
  display.setCursor(PLOT_X + PLOT_SIZE + 5, y - 4);
  display.setTextColor(color);
  display.print(value, 3);
}

void drawTwoPlots(const float data[], const float data2[], int n) {
  float minY = data[0], maxY = data[0], sumY = data[0];
  for (int i = 1; i < n; i++) {
    float v = data[i];
    sumY += v;
    if (v < minY) minY = v;
    if (v > maxY) maxY = v;
  }
  float meanY = sumY / n;
  float rangeY = 1;

  if (maxY - minY < 0.05 && voltageDisplay && currentMode == Low) {
    rangeY = 0.1;
  } else if (maxY - minY < 0.2 && voltageDisplay && currentMode != Low) {
    rangeY = 0.4;
  } else {
    rangeY = maxY - minY;
  }

  float margin = rangeY * 0.10f;
  float plotMin = minY - margin * 0.5f;
  float plotMax = maxY + margin * 0.5f;
  float yScale  = float(PLOT_SIZE) / (plotMax - plotMin);

  display.fillRect(PLOT_X, PLOT_Y, PLOT_SIZE + 100, PLOT_SIZE, COLOR_BG);

  for (int i = 0; i < n - 1; i++) {
    int x1 = PLOT_X + (i     * PLOT_SIZE) / (n - 1);
    int y1 = PLOT_Y + PLOT_SIZE - int((data[i]     - plotMin) * yScale);
    int x2 = PLOT_X + ((i + 1) * PLOT_SIZE) / (n - 1);
    int y2 = PLOT_Y + PLOT_SIZE - int((data[i + 1] - plotMin) * yScale);
    display.drawLine(x1, y1, x2, y2, COLOR_DATA);
  }

  drawStatLine(minY, plotMin, yScale, COLOR_MIN);
  drawStatLine(meanY, plotMin, yScale, COLOR_MEAN);
  drawStatLine(maxY, plotMin, yScale, COLOR_MAX);
}

// ======================= USB LOGGING (unchanged) =======================

template<typename T>
void printArrayCSV(FILE* f, const T* arr, size_t len, const char* fmt) {
  for (size_t i = 0; i < len; ++i) {
    fprintf(f, fmt, arr[i]);
    if (i + 1 < len) fputc(',', f);
  }
  fputc('\n', f);
}

void manuallogArraysToCSV() {
  FILE* f = fopen("/usb/data.csv", "a");
  if (!f) {
    Serial.println("Error opening data.csv");
    return;
  }
  fprintf(f, "DataLog End Time: ");
  fprintf(f, "%.2f\n", tLogEnd);
  printArrayCSV(f,
    loggedVoltagesAtI,
    sizeof(loggedVoltagesAtI) / sizeof(loggedVoltagesAtI[0]),
    "%.3f");
  if (currentOnOff) {
    printArrayCSV(f,
      loggedCurrents,
      sizeof(loggedCurrents) / sizeof(loggedCurrents[0]),
      "%.3f");
  }
  printArrayCSV(f,
    loggedTimeStamps,
    sizeof(loggedTimeStamps) / sizeof(loggedTimeStamps[0]),
    "%.3f");
  fclose(f);
  Serial.println("All arrays logged as separate CSV rows.");
}

void shiftAndStore(float* array, float newValue) {
  for (int i = NUM_AUTO_VALUES - 1; i > 0; i--) {
    array[i] = array[i - 1];
  }
  array[0] = newValue;
}

void logValues(float VatIHigh, float ItHighm, float IHigh) {
  shiftAndStore(VAutoArray, VatIHigh);
  shiftAndStore(TAutoArray, ItHighm);
  shiftAndStore(IAutoArray, IHigh);
}

void autologArraysToCSV() {
  FILE* f = fopen("/usb/data_autologged.csv", "a");
  if (!f) {
    Serial.println("Error opening data.csv");
    return;
  }
  fprintf(f, "DataLogged End Time: ");
  fprintf(f, "%.2f\n", tLogEnd);
  printArrayCSV(f,
    VAutoArray,
    sizeof(VAutoArray) / sizeof(VAutoArray[0]),
    "%.3f");
  if (currentOnOff) {
    printArrayCSV(f,
      IAutoArray,
      sizeof(IAutoArray) / sizeof(IAutoArray[0]),
      "%.3f");
  }
  printArrayCSV(f,
    TAutoArray,
    sizeof(TAutoArray) / sizeof(TAutoArray[0]),
    "%.3f");
  fclose(f);
}

// ======================= VOLTAGE BRIDGE DETECTION (unchanged) =======================

void ClosedOrFloat() {
  vClosedflagPrevious = vClosedflag;

  digitalWrite(VbridgePin, HIGH);
  delay(2);

  ads.setGain(GAIN_SIXTEEN);
  bridgeV = (ads.readADC_Differential_0_1() * GAIN_FACTOR_16 / 1000.0);

  vClosedflag = false;
  vFloating = false;
  vUndefined = false;

  if (fabs(bridgeV) < 0.03) {
    vClosedflag = true;
  } else if (fabs(bridgeV) > 0.05) {
    vFloating = true;
  } else {
    vUndefined = true;
  }
  if (vClosedflag && vClosedflagPrevious) {
    vClosed = true;
  } else {
    vClosed = false;
  }

  digitalWrite(VbridgePin, 0);
}
