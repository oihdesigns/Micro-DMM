#include <Wire.h>
#include <Adafruit_ADS1X15.h>

//GIGA HID Related
#include "PluggableUSBHID.h"
#include "USBKeyboard.h"
USBKeyboard Keyboard;
//#include <FlashStorage.h>
//#include <array>

#include "Arduino_H7_Video.h"
#include "Arduino_GigaDisplayTouch.h"
#include "lvgl.h"

Arduino_H7_Video Display(800, 480, GigaDisplayShield);  // 800x480 resolution
Arduino_GigaDisplayTouch Touch;



// 2) Your working RAM arrays
//float calib[19];
const float defaultCalib[19] = {
  // initial calibration list (19 entries)
  0.9828, 0.9958, 0.9999, 0.9979, 0.9954,
  0.9965, 0.9980, 1.0034, 1.0021, 1.0012,
  1.0031, 1.0073, 1.0176, 1.0611, 1.1130,
  0.020073, 0.6120, 4.9980, 46.4680
};

// ===== Hardware Setup Constants =====
Adafruit_ADS1115 ads;



// Pin definitions
const int CONTINUITY_PIN = 6;   // Buzzer or LED for continuity/alerts
const int SETRANGE_PIN = 7;   // Controls high/low resistance range
//#define IR_RECEIVE_PIN   8      // IR receiver input pin
const int OHMPWMPIN = 9;
//const int BATT_PIN      = A2;   // Battery voltage analog input
//#define enablePin  BAT_READ_EN  // Pin for enabling battery voltage reading
//#define BATT_PIN BAT_DET_PIN
const int TYPE_PIN      = 3;    // Mode button (also triggers flashlight mode if held at boot)
const int KBPin = 8;   // Toggle: if LOW -> keyboard mode; if HIGH -> serial mode
const int logPin = 2; // take log pin
//const int LowerButton = 10;   // 


// ADS1115 gain factors (mV per bit) for each gain setting
const float GAIN_FACTOR_TWOTHIRDS = 0.1875;  // 2/3x (±6.144V range)
const float GAIN_FACTOR_1   = 0.125;   // ±4.096V
const float GAIN_FACTOR_2   = 0.0625;  // ±2.048V
const float GAIN_FACTOR_4   = 0.03125; // ±1.024V
const float GAIN_FACTOR_8   = 0.015625;// ±0.512V
const float GAIN_FACTOR_16  = 0.0078125;// ±0.256V


// instead of “static const int kGainLevels[] = { … };”
static const adsGain_t kGainLevels[] = {
  GAIN_TWOTHIRDS,
  GAIN_ONE,
  GAIN_TWO,
  GAIN_FOUR,
  GAIN_EIGHT,
  GAIN_SIXTEEN
};



// Measurement constants (calibration values)
float constantI            = 0.02016f;   // A for low-resistance constant-current source
const float constantR            = 330.0f;     // Ω internal resistor in constant-current circuit
const float dividerR             = 22000.0f;   // Ω series resistor for high-resistance divider
float ZENER_MAX_V          = 5.0f;       // V reference in high-range mode
float EEPROM_MAXV = 5.0f;
float EEPROM_SleepV = 0.615;

//Mode Rotate Related
#define DEBOUNCE_DELAY 50    // debounce time in milliseconds
enum Mode {
  Default,
  Voltmeter,
  VACmanual,
  Type,
  Low,
  AltUnitsMode,
  HighRMode,
  //Impedance,
  //RelayControl,
  Charging,
  NUM_MODES
};
#define BUTTON_PIN 10         // Set your button input pin

Mode currentMode = Default;
Mode previousMode = Default;

unsigned long lastDebounceTime = 0;
bool lastButtonState = HIGH;
bool buttonPreviouslyPressed = false;

/*
// Measurement constants (calibration values)
const float constantI = 0.02016;  // Constant current source (A) for low resistance
const float constantR = 330.0;//EEPROM2:240    // Internal resistor (Ω) in constant current circuit
const float dividerR = 22000.0;   // Series resistor (Ω) for high resistance divider
const float ZENER_MAX_V =5.0; //EEPROM2:4.353;  // Zener/reference voltage in high-range mode (V)
*/
float VOLTAGE_SCALE = 63.539; // Calibration scale factor for voltage input

// Timing intervals (ms)
const unsigned long ADC_INTERVAL    = 1;     // ADC sampling interval
const unsigned long BATT_INTERVAL   = 1000;  // Battery check interval
unsigned long LCD_INTERVAL         = 1000;   // Display refresh interval (can change)
const unsigned long SERIAL_INTERVAL = 200;   // Serial output interval (continuous mode)
const unsigned long IR_DEBOUNCE_INTERVAL = 300;
unsigned long previousAdcMillis   = 0;
unsigned long previousBattMillis  = 0;
unsigned long previousLcdMillis   = 0;
unsigned long previousSerialMillis= 0;
unsigned long lastIrReceiveMillis = 0;
unsigned long deepSleepStart = 0;
unsigned int seconds = 0;

// Global measurement variables
int16_t adcReadingVoltage = 0;  // raw ADC reading for voltage (differential)
int16_t adcReadingOhms    = 0;  // raw ADC reading for resistance
int16_t adcReadingCurrent = 0;  // raw ADC reading for current
float batteryVoltage = 0.0;     // measured battery voltage (V)
int16_t countV;

float newVoltageReading = 0.0;  // latest measured voltage (V)
float averageVoltage = 0.0;     // moving average (DC) of voltage
float VAC = 0.0;                // AC RMS component of voltage
float previousVoltage = 0.0;    // previous loop's voltage (for auto-range decision)
float medianVoltage = 0.0;      // exponentially filtered voltage for display
float medianVoltageStep = 0.0;

float rawResistance = 0.0;      // calculated resistance before calibration (Ω)
float calibratedResistance = 0.0; // resistance after applying calibration factors (Ω)
float currentResistance = 0.0;  // final resistance value used for display (before zero offset)
float prevResistance = 0.0;
float zeroOffsetRes = 0.0;      // user-set zero offset for resistance (to zero out test leads)
float displayResistance = 0.0;  // resistance value after subtracting zero offset (displayed)
float ohmsVoltage = 0.0;        // The voltage off the resistance circuit
float timeHigh = 0.0;

float currentShuntVoltage = 0.0;// measured shunt voltage for current (V)
float Ireading = 0.0;           // measured current (A)
float Izero = 0.0;              // baseline offset for current (high-range)
bool Irange = false;            // current range: false = low-range, true = high-range
bool currentOnOff = false;       // whether current measurement is active (false if no sensor detected)
float IHigh = -6.0;             // max current seen (A)
float ILow  = 6.0;              // min current seen (A)
float IMedian = 0.0;            // filtered current for display smoothing

// Rolling buffers for averaging voltage and computing VAC
const int NUM_VOLTAGE_SAMPLES = 100;
float voltageSamples[NUM_VOLTAGE_SAMPLES] = {0.0};
float voltageSquaredSamples[NUM_VOLTAGE_SAMPLES] = {0.0};
int voltageSampleIndex = 0;
float voltageSum = 0.0;
float squaredVoltageSum = 0.0;

// Min/Max tracking for voltage and resistance
float highV = -100.0, lowV = 100.0;
float highR = 0.0, lowR = 3000000.0;
String timeAtMaxV = "", timeAtMinV = "";
String timeAtMaxI = "", timeAtMinI = "";
float voltageAtMaxI = 0.0, voltageAtMinI = 0.0;  // voltage at the time of max/min current
float currentAtMaxV = 0.0, currentAtMinV = 0.0;  // current at the time of max/min voltage

// Formatted display values (with units) and their suffixes
float roundedV = 0.0, roundedVlow = 0.0, roundedVhigh = 0.0;
int vDigits = 0, vDigitslow = 0, vDigitshigh = 0;
String vSuffix = "", vSuffixlow = "", vSuffixhigh = "";
float roundedR = 0.0, roundedRlow = 0.0, roundedRhigh = 0.0;
int rDigits = 0, rDigitslow = 0, rDigitshigh = 0;
String rSuffix = "", rSuffixlow = "", rSuffixhigh = "";

// IR remote button codes
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

const uint8_t PROGMEM OHM_8x8[] = {      // Ω
  0b00111100,
  0b01000010,
  0b10000001,
  0b10000001,
  0b10000001,
  0b10011001,
  0b01000010,
  0b00111100
};

const uint8_t MICRO_5x7[] PROGMEM = {    // µ, 5 wide × 7 high
  0b00000,
  0b10010,
  0b10010,
  0b10010,
  0b10010,
  0b10110,
  0b10000
};

//create default corrections
float CF_A = 1.0;
float CF_B = 1.0;
float CF_C = 1.0;
float CF_D = 1.0;
float CF_E = 1.0;
float CF_F = 1.0;
float CF_G = 1.0;
float CF_H = 1.0;
float CF_I = 1.0;
float CF_J = 1.0;
float CF_K = 1.0;
float CF_L = 1.0;
float CF_M = 1.0;
float CF_N = 1.0;
float CF_O = 1.0;

// Flags and modes
bool ohmsHighRange = true;    // user-selected range for resistance (if auto-range off): true = high-range, false = low-range
bool ohmsAutoRange = true;    // auto-ranging enabled for resistance
bool voltageDisplay = false;  // false = show resistance, true = show voltage (auto-set or via command)
bool smoothVmode = true;      // when in manual voltage mode, true = show average (smooth), false = show instantaneous
bool vDisplayManual = false;  // false = auto switch between fast/smooth for V, true = manual control of smoothVmode
bool MinMaxDisplay = false;   // whether to display min/max on the OLED
bool screenRefreshFast = false; // fast display refresh mode (500 ms)
bool ampsMode = false;     // focus on current measurements for output (changes serial output format)
bool debugMode = false;       // debug mode for extra serial printouts
bool initialZeroSet = false;  // whether initial auto-zero for resistance has been done
bool serialMode = false;      // serial mode
bool useKeyboard = false;       // Mode flag determined by togglePin
bool powerSave;
bool timeHighset;
bool altUnits = false; //Display alternate temp/voltage
bool preciseMode = false;
bool deepSleepTrigger = false;
bool screenSleep = false;
bool deltaMode = false;
bool deltaTrigger = false;
float deltaV = 0.0;
int deltaVdigits = 0;

// Flags for transient states
bool flashlightMode = false;  // flashlight mode active (button held at startup)
bool flashlightChecked = false; // whether we have checked the button for flashlight mode at startup
bool buttonPressed = false;   // debounced state of the button (pressed = true)
unsigned long buttonPressTime = 0; // timestamp when button was pressed
bool vFlag = false;           // indicates voltage alert is active (to manage flashing pattern)
bool rFlag = false;           // indicates resistance continuity alert is active
bool VACPresense = false;       // indicates VAC presence

// Logging buffers for current vs time (for 'L' command)
const int LOG_SIZE = 120;
const int SAMPLE_COUNT = 120;
bool takeLog = false;
int samplesTaken = 0;
float tLogStart = 0;
float tLogEnd = 0;
float loggedCurrents[LOG_SIZE] = {0};
float loggedVoltagesAtI[LOG_SIZE] = {0};
float loggedTimeStamps[LOG_SIZE] = {0};

// ========== Function Prototypes ========== 
void handleIRRemote();
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


//DAC Output

//analogWave wave(DAC);   // Create an instance of the analogWave class, using the DAC pin
//int freq = 10;  // in hertz, change accordingly




// ========== Setup Function ========== 
void setup() {



  Serial.begin(115200);
  delay(100);
  Serial.println("Setup Start");
  
  // Configure pins
  pinMode(TYPE_PIN, INPUT_PULLUP);
  pinMode(CONTINUITY_PIN, OUTPUT);
  pinMode(SETRANGE_PIN, OUTPUT);
  //pinMode(BATT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Assuming active-low button
  pinMode(OHMPWMPIN, OUTPUT);
  pinMode(logPin, INPUT_PULLUP);




  // 5) Assign your named variables
  EEPROM_MAXV  = defaultCalib[17];
  EEPROM_SleepV = defaultCalib[16];
  constantI     = defaultCalib[15];
  VOLTAGE_SCALE= defaultCalib[18];


  // Initialize OLED display


    // ... (existing initialization: Wire, ads.begin, etc.)
    lv_init();                      // Initialize LVGL library
    Display.begin();                // Initialize display and LVGL integration:contentReference[oaicite:3]{index=3}
    Touch.begin();                  // Initialize touch screen input:contentReference[oaicite:4]{index=4}
    // (Optional) lv_disp_set_rotation or lv_disp_set_orientation if needed
    
    // Create GUI objects (labels, bars, buttons, etc.)

    lv_obj_t * valueLabel = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(valueLabel, &lv_font_montserrat_28, 0); // large font
    lv_label_set_text(valueLabel, "0.00 V");  
    lv_obj_align(valueLabel, LV_ALIGN_TOP_MID, 0, 20);  // position near top-center

    lv_obj_t * valueBar = lv_bar_create(lv_scr_act());
    lv_obj_set_size(valueBar, 700, 20);                 // wide bar
    lv_obj_align(valueBar, LV_ALIGN_TOP_MID, 0, 60);    // below the label
    lv_bar_set_range(valueBar, 0, 1000);                // initial range (e.g., 0-1000 for scaling)
    lv_bar_set_value(valueBar, 0, LV_ANIM_OFF);

    // Create a generic styled button function
auto makeButton = [&](const char * txt, lv_align_t align, lv_coord_t x_ofs) {
    lv_obj_t * btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn, 180, 50);
    lv_obj_align(btn, align, x_ofs, -10);  // align relative to bottom
    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, txt);
    lv_obj_center(label);
    return btn;
};
lv_obj_t * nullBtn = makeButton("Null/Type", LV_ALIGN_BOTTOM_LEFT, 20);
lv_obj_t * logBtn  = makeButton("Log",       LV_ALIGN_BOTTOM_MID,   0);
lv_obj_t * modeBtn = makeButton("Mode",      LV_ALIGN_BOTTOM_RIGHT,-20);

lv_obj_t * modeDrop = lv_dropdown_create(lv_scr_act());
lv_dropdown_set_options(modeDrop,
    "Resistance\n"       // index 0
    "Voltage (DC)\n"     // index 1
    "Voltage (AC)\n"     // index 2
    "Current\n"          // index 3
    "High Ω Mode\n"      // index 4
    "Type (USB)"         // index 5
);
lv_obj_set_width(modeDrop, 150);
lv_obj_align(modeDrop, LV_ALIGN_TOP_LEFT, 10, 10);
lv_dropdown_set_selected(modeDrop, 0);  // start at "Resistance"

lv_obj_t * autoSwitch = lv_switch_create(lv_scr_act());
lv_obj_align(autoSwitch, LV_ALIGN_TOP_RIGHT, -60, 15);
lv_obj_t * swLabel = lv_label_create(lv_scr_act());
lv_label_set_text(swLabel, "Auto Range");
lv_obj_align(swLabel, LV_ALIGN_TOP_RIGHT, -120, 20);
lv_obj_add_state(autoSwitch, LV_STATE_CHECKED);  // start ON by default

lv_obj_add_event_cb(nullBtn, [](lv_event_t * e) {
    if(lv_event_get_code(e) == LV_EVENT_LONG_PRESSED) {
        ReZero();                          // long press: reset min/max
        MinMaxDisplay = true;
    } 
    else if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
        // short press: either type value or set null offset
        if(currentMode == Type) {
            // If in "Type" mode, send the value via USB keyboard
            if(voltageDisplay) {
                Keyboard.printf("%.*f\r\n", vDigits, newVoltageReading);
            } else {
                // If displaying resistance, send that value
                if(displayResistance < 1)
                    Keyboard.printf("%.*f\r\n", rDigits+2, displayResistance);
                else
                    Keyboard.printf("%.*f\r\n", rDigits, displayResistance);
            }
            Keyboard.key_code(RIGHT_ARROW);  // move cursor right (Excel/Sheets) after typing
        } else {
            // If not in Type mode: toggle relative zero (null)
            if(voltageDisplay) {
                // For voltage mode, capture current reading as reference
                deltaV = (preciseMode ? newVoltageReading : averageVoltage);
                deltaVdigits = vDigits - 1;
            } else {
                // For resistance mode, set or clear zero offset (null):contentReference[oaicite:6]{index=6}
                if(zeroOffsetRes == 0.0f)
                    zeroOffsetRes = currentResistance;
                else 
                    zeroOffsetRes = 0.0f;
            }
        }
    }
}, LV_EVENT_ALL, NULL);

lv_obj_add_event_cb(logBtn, [](lv_event_t * e) {
    if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
        takeLog = true;
        tLogStart = millis() / 1000.0;  // mark log start time:contentReference[oaicite:10]{index=10}
        // (The loop will handle logging and set takeLog false when done)
    }
}, LV_EVENT_CLICKED, NULL);

lv_obj_add_event_cb(modeBtn, [](lv_event_t * e) {
    if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
        currentMode = static_cast<Mode>((currentMode + 1) % NUM_MODES);
        Serial.printf("Mode changed to %d\n", currentMode);
        // Update dropdown selection to match the new mode:
        int ddIndex = 0;
        switch(currentMode) {
            case Default:      ddIndex = 0; break;  // Resistance
            case Voltmeter:    ddIndex = 1; break;  // DC Voltage
            case VACmanual:    ddIndex = 2; break;  // AC Voltage
            case Type:         ddIndex = 5; break;  // Type (USB)
            case HighRMode:    ddIndex = 4; break;  // High Ω
            // (If other modes like Low, AltUnits, Charging are encountered, you could handle or ignore them)
        }
        lv_dropdown_set_selected(modeDrop, ddIndex);
    }
}, LV_EVENT_CLICKED, NULL);


lv_obj_add_event_cb(modeDrop, [](lv_event_t * e) {
    if(lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        uint16_t sel = lv_dropdown_get_selected(modeDrop);
        switch(sel) {
            case 0: currentMode = Default;    break;  // Resistance mode
            case 1: currentMode = Voltmeter;  break;  // DC Voltage mode
            case 2: currentMode = VACmanual;  break;  // AC Voltage mode
            case 3: 
                // Current mode (no direct Mode enum; enable current reading)
                currentMode = Voltmeter;   // set a safe base mode (voltage) 
                ampsMode   = true;         // focus on current measurements:contentReference[oaicite:14]{index=14}
                break;
            case 4: currentMode = HighRMode;  break;  // High resistance mode
            case 5: currentMode = Type;       break;  // USB keyboard output mode
        }
        // Sync any related flags
        if(sel != 3) { 
            ampsMode = (currentMode == Charging) ? true : false; // only keep ampsMode for a dedicated current mode if defined
            if(sel != 0 && sel != 4) zeroOffsetRes = 0.0f;  // clear resistance offset when leaving resistance modes
        }
        Serial.printf("Mode set to %d via dropdown\n", currentMode);
    }
}, LV_EVENT_VALUE_CHANGED, NULL);


lv_obj_add_event_cb(autoSwitch, [](lv_event_t * e) {
    if(lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        bool on = lv_obj_has_state(autoSwitch, LV_STATE_CHECKED);
        ohmsAutoRange = on;
        Serial.println(on ? "Auto-range On" : "Auto-range Off");
        // (Optionally, if turning auto-range off, one could allow manual range selection via UI)
    }
}, LV_EVENT_VALUE_CHANGED, NULL);


  // Splash screen

/*
  //display.setCursor(0, 0);  
  //display.drawBitmap(0, 0, MICRO_5x7, 5, 7, SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("uMeter #");
  //display.print(EEPROM.read(1));//Reads the EEPROM and determines the correct splash   
  display.setCursor(0, 48);
  display.println("XIAO RA4M1");
  
  //display.display();
  delay(200);
*/


  // Initialize ADS1115 ADC
  if (!ads.begin()) {
    Serial.println("ADS1115 init failed!");
    display.fillScreen(BLACK);

    display.setCursor(0, 0);
    display.setTextSize(1);
    Serial.println("ADS1115 not found");
    display.println("ADS1115 not found");
    //display.display();
    while (1); // halt
  }
  Serial.println("ADS1115 good");
  ads.setDataRate(RATE_ADS1115_16SPS); // slow rate b/c default is resistance
  delay(300);

  // Initialize IR Receiver
  //IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  // Print available commands (for user reference)
  Serial.println(F("Commands: Q,q,Z,C,R,S,V,A,F,M,B,D,e,E,I,L,?"));


  // Initial current zero calibration
  ads.setGain(GAIN_TWOTHIRDS);  // ±6.144V range to read baseline
  Serial.print("ADS gain set");
  adcReadingCurrent = ads.readADC_SingleEnded(3);
  delay(100);
  Serial.print("ADS3Read");
  currentShuntVoltage = adcReadingCurrent * GAIN_FACTOR_TWOTHIRDS / 1000.0;
  if (adcReadingCurrent < 300 || isBetween(currentShuntVoltage, 2.3, 2.7)) {
    // If no current (shunt pulled to ground) or baseline ~2.5V, current sensor is present
    if (isBetween(currentShuntVoltage, 2.3, 2.7)) {
      Irange = true;         // high-range current mode
      Izero = currentShuntVoltage; // store baseline (~2.5V)
    } else {
      Irange = false;        // low-range current mode
      Izero = 0.0;
    }
    currentOnOff = true;
  } else {
    // Current sensor not connected or reading abnormal -> disable current measurement
    currentOnOff = false;
    Ireading = 0.0;
  }
  
 
    display.fillScreen(BLACK);
    Serial.println("Setup End");

//  wave.sine(freq);
//  analogWriteResolution(12);

}

// ========== Main Loop ========== 
void loop() {
    //  Serial.println("Loop Start");
  unsigned long currentMillis = millis();

  previousMode=currentMode;

  checkModeButton();//Check for mode button

  if(currentMode==AltUnitsMode){
    altUnits = true;
  }else{
    altUnits = false;
  }

  if(currentMode==Low){
    preciseMode = true;
  }else{
    preciseMode = false;
  }

  if(currentMode == Type){
    deltaMode = false;
    deltaV = 0;
  }else{
    deltaMode = true;
  }


  // Check for flashlight mode trigger (once after startup)
  if (!flashlightChecked) {
    if (digitalRead(TYPE_PIN) == LOW) {
      flashlightMode = true;
    }
    flashlightChecked = true;
  }

/*
  //Check for Log Pin
  if(LowerButton == 0){
    float nowSec = millis() / 1000.0;
    logCurrentData(newVoltageReading, nowSec, Ireading);
    analogWrite(CONTINUITY_PIN,100);
    Serial.print("data logged");
  }
*/

if(digitalRead(logPin)==0){
  takeLog = true;
  }


if(takeLog == true){
      if (samplesTaken<(SAMPLE_COUNT+1)){
        float nowSec = millis() / 1000.0;
        logCurrentData(newVoltageReading, nowSec, Ireading);
        samplesTaken++;
      }else{
        takeLog = false;
        tLogEnd  = millis() / 1000.0;
        samplesTaken = 0;
        analogWrite(CONTINUITY_PIN, 100);
        Serial.println("Samples logged!");
      }
}

  handleButtonInput();


  // Periodic ADC measurements
  if (currentMillis - previousAdcMillis >= ADC_INTERVAL) {
    previousAdcMillis = currentMillis;
    if(!takeLog){ //Bypass Resistance if taking a log
    measureResistance();
    }
    measureVoltage();
    measureCurrent();
    

if(!powerSave){
    if(ohmsVoltage>EEPROM_MAXV-0.002 && !timeHighset && currentMode != HighRMode){
      timeHigh = millis();
      timeHighset = true;
    }
    if (timeHighset && currentResistance < 2000){
      timeHighset = false;
    }    
    if (timeHighset && timeHigh + 5000 < millis()){
      powerSave = true;
    }
}



if(powerSave || currentMode==Charging){
  analogWrite(OHMPWMPIN, 254);
    
  if(ohmsVoltage < EEPROM_SleepV-0.01){
      powerSave = false;
      timeHighset = false;
      analogWrite(OHMPWMPIN, 0);
    }
  }

    
    // Determine which primary measurement to display automatically
    
    if ( countV>500  || currentMode == Voltmeter) {
      voltageDisplay = true;
      if(preciseMode){
      ads.setDataRate(RATE_ADS1115_16SPS); //slow sampling if 0 is set  
      }else{
      ads.setDataRate(RATE_ADS1115_860SPS); //fast sampling for voltage
      }
    
    }
    if (isBetween(currentResistance, -10.0, 100.0)&& currentMode != Voltmeter) {
      voltageDisplay = false;
      ads.setDataRate(RATE_ADS1115_32SPS); // Slow sampling in resistance mode
    }

    // Calculate displayResistance after zero offset
    displayResistance = currentResistance - zeroOffsetRes;

    // Update resistance min/max if zero reference is set
    if (initialZeroSet) {
      if (displayResistance > highR && displayResistance < 10000000.0) {
        highR = displayResistance;
      }
      if (displayResistance < lowR && displayResistance > -1000.0) {
        lowR = displayResistance;
      }
    }

    // If current is flowing, update IMedian for smoothing (simple approach)
    if (Ireading != 0.0) {
      if (Ireading > IMedian) IMedian += 0.02;
      else IMedian -= 0.02;
    }
  }

  // Handle serial commands (if any received)
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    handleSerialCommands(cmd);
  }

  // Continuous serial output mode
  if (serialMode && currentMillis - previousSerialMillis >= SERIAL_INTERVAL) {
    previousSerialMillis = currentMillis;
    // Prepare and send continuous data output (formatted as lines with "BREAK")
    if (ampsMode) {
      // Current-focused output (6 lines of data)
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
        // Min and Max voltage values
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
      // Voltage-focused output (3 lines of data)
      if (fabs(averageVoltage) < 0.001 && VAC < 0.2) {
        // Indicate very small voltage as "<1mV" or open
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
      // Resistance-focused output (3 lines of data)
      if (ohmsAutoRange || ohmsVoltage < (ZENER_MAX_V - 0.005)) {
        // Only report a numeric value if within range
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

  // Update buzzer/LED alerts or flashlight LED brightness
  updateAlerts();

  // Update display at interval
  if ((currentMillis - previousLcdMillis >= LCD_INTERVAL)&&!takeLog) {
    previousLcdMillis = currentMillis;
    LCD_INTERVAL = screenRefreshFast ? 500 : 1000; // adjust refresh rate if toggled
    updateDisplay();
    
    if(ampsMode || currentMode == Charging) {
        // Display current reading
        float Ival = Ireading;
        const char * unit = "A";
        float displayVal;
        if(!Irange) {
            // low-range: show in mA
            displayVal = Ival * 1000.0f;
            unit = "mA";
        } else {
            displayVal = Ival;
            unit = "A";
        }
        lv_label_set_text_fmt(valueLabel, "%.3f %s", displayVal, unit);
        // Set bar value relative to max (e.g., 100mA or 5A full scale)
        int barMax = Irange ? 5000 : 100;  // example: 5A = 5000 mA max if high-range, 100 mA max if low-range
        int barVal = (int)((Ival * 1000.0f) > barMax ? barMax : (Ival * 1000.0f));
        lv_bar_set_range(valueBar, 0, barMax);
        lv_bar_set_value(valueBar, barVal, LV_ANIM_OFF);
    } 
    else if(voltageDisplay) {
        // Display voltage reading (DC or AC)
        float Vval = (voltageDisplay && !VACPresense ? roundedV : VAC);  // if AC present and in AC mode, use VAC
        const char * unit = VACPresense ? "VAC" : "V";
        lv_label_set_text_fmt(valueLabel, "%.3f %s", Vval, unit);
        // Set bar relative to, say, 6.0 V full scale
        int barVal = (int)((fabs(Vval) / 6.0f) * 1000);
        if(barVal > 1000) barVal = 1000;
        lv_bar_set_range(valueBar, 0, 1000);
        lv_bar_set_value(valueBar, barVal, LV_ANIM_OFF);
    } 
    else {
        // Display resistance reading
        float Rval = displayResistance;
        // Choose an appropriate unit (Ω, kΩ, MΩ) for display
        char unit[3] = "Ω";
        float dispVal = Rval;
        if(fabs(Rval) >= 1e6) {
            dispVal = Rval / 1e6; strcpy(unit, "MΩ");
        } else if(fabs(Rval) >= 1e3) {
            dispVal = Rval / 1e3; strcpy(unit, "kΩ");
        }
        lv_label_set_text_fmt(valueLabel, "%.3f %s", dispVal, unit);
        // Set bar range dynamically (e.g., 0 to 1e6 Ω for normal mode, extend for HighR)
        int maxRange = (currentMode == HighRMode) ? 10000000 : 1000000;  // 10M for high-range mode
        int barVal = (Rval > maxRange ? maxRange : (int)Rval);
        lv_bar_set_range(valueBar, 0, maxRange);
        lv_bar_set_value(valueBar, barVal, LV_ANIM_OFF);
    }

    // Feed the LVGL engine to handle rendering and input
    lv_timer_handler();
    delay(5);  // small delay to avoid 100% CPU usage
}
  }

  if(powerSave && newVoltageReading<0.1 && !VACPresense && Ireading==0){
    if(!deepSleepTrigger){
    deepSleepStart= millis();
    deepSleepTrigger = true;
    }else{
      if (millis() - deepSleepStart > 300000){ //300000 millis = 5min
        screenSleep = true;
      }
    }
  }else{
    screenSleep = false;
    deepSleepTrigger = false;
  }

}

// ========== Input Handling Functions ========== 


void handleSerialCommands(char command) {
  // Handle a single-character command from Serial
  switch (command) {
    case 'Q':  // Query current values (single snapshot)
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
    case 'q':  // Toggle continuous serial output mode
      serialMode = !serialMode;
      if (serialMode) Serial.println("Serial mode ON");
      break;
    case 'Z':  // Set or clear zero reference for resistance (toggle)
      if (!initialZeroSet || fabs(zeroOffsetRes) < 1e-6) {
        // If no zero offset set, set it
        zeroOffsetRes = currentResistance;
        initialZeroSet = true;
        Serial.print("Zero set to ");
        Serial.println(zeroOffsetRes);
      } else {
        // If already set, clear it
        zeroOffsetRes = 0.0;
        Serial.println("Zero offset cleared");
      }
      break;
    case 'C':  // Clear zero offset
      zeroOffsetRes = 0.0;
      Serial.println("Zero offset cleared");
      break;
    case 'R':  // Toggle manual resistance range (if not auto)
      ohmsHighRange = !ohmsHighRange;
      Serial.print("Manual Range now: ");
      Serial.println(ohmsHighRange ? "High" : "Low");
      break;
    case 'S':  // Toggle smoothing (voltage averaging) mode
      smoothVmode = !smoothVmode;
      Serial.print("Smooth mode: ");
      Serial.println(smoothVmode ? "On" : "Off");
      break;
    case 'V':  // Toggle display mode (force voltage display on/off)
      voltageDisplay = !voltageDisplay;
      Serial.println("Display mode toggled");
      break;
    case 'A':  // Toggle auto-range for resistance
      ohmsAutoRange = !ohmsAutoRange;
      Serial.print("Auto-range: ");
      Serial.println(ohmsAutoRange ? "On" : "Off");
      break;
    case 'F':  // Toggle fast screen refresh
      screenRefreshFast = !screenRefreshFast;
      Serial.print("Screen refresh: ");
      Serial.println(screenRefreshFast ? "Fast" : "Normal");
      break;
    case 'M':  // Toggle manual voltage display mode selection
      vDisplayManual = !vDisplayManual;
      Serial.print("Manual voltage display: ");
      Serial.println(vDisplayManual ? "On" : "Off");
      break;
    case 'B':  // Battery voltage query
      Serial.print("Battery: ");
      Serial.print(batteryVoltage, 2);
      Serial.println(" V");
      break;
    case 'D':  // Toggle debug mode
      debugMode = !debugMode;
      Serial.print("Debug mode: ");
      Serial.println(debugMode ? "On" : "Off");
      break;
    case 'e':  // Toggle Min/Max display
      MinMaxDisplay = !MinMaxDisplay;
      Serial.print("Min/Max display: ");
      Serial.println(MinMaxDisplay ? "On" : "Off");
      break;
    case 'E':  // Reset min/max values
      ReZero();
      Serial.println("Min/Max reset");
      break;
    case 'I':  // Toggle current mode for serial output
      ampsMode = !ampsMode;
      Serial.print("Current mode: ");
      Serial.println(ampsMode ? "On" : "Off");
      break;
    case 'L':  // Print logged arrays of I, V, T
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

    case 'l': //take a log
      takeLog = true;
      tLogStart  = millis() / 1000.0;
    break;

    case 'k': //print manual v/t logs
      Serial.print("Time Start (s):");
      Serial.println(tLogStart,3);
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
      Serial.println(tLogEnd,3);
    break;

    case 'j': //print above, but with amperes data as well
      Serial.print("Time Start (s):");
      Serial.println(tLogStart,3);
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
      Serial.println(tLogEnd,3);
    break;  

    
    case '?':  // Help: list commands
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
    // Button was just pressed
    buttonPressed = true;
    buttonPressTime = millis();
  }
  if (!isPressed && buttonPressed) {
    // Button was released
    unsigned long pressDuration = millis() - buttonPressTime;
    buttonPressed = false;
    analogWrite(CONTINUITY_PIN, 0);  // turn off any flashlight LED dimming
    if (pressDuration > 400) {
      // Long press: reset min/max and enable display
      ReZero();
      MinMaxDisplay = true;
    } else if (pressDuration > 50) {
      // Short press: type current reading via USB keyboard
      if(currentMode==Type){      
      if (voltageDisplay) {
        Keyboard.printf("%.*f\r\n",vDigits,newVoltageReading);
      } else {
        if(displayResistance<1){
        Keyboard.printf("%.*f\r\n", rDigits+2, displayResistance);
        }else{
        Keyboard.printf("%.*f\r\n", rDigits, displayResistance);  
        }
      }
      // Press Right Arrow after typing (to move cursor, e.g., to next cell)
      Keyboard.key_code(RIGHT_ARROW);
        //Keyboard.releaseAll();
      }else{
        if(voltageDisplay){
        deltaVdigits = vDigits-1;
        if(preciseMode){
          deltaV = newVoltageReading;
          }else{
          deltaV = averageVoltage;  
          }
        }else{
          if(zeroOffsetRes==0){
          zeroOffsetRes = currentResistance;
          }else{
            zeroOffsetRes = 0;
          }

        }

        }
        
      
    }
  }
  // If button is held, and in flashlight mode, adjust LED brightness (done in updateAlerts)
  if (isPressed && flashlightMode && (millis() - buttonPressTime) > 500) {
    // Provide a dim glow feedback while holding (already handled by flashlight logic in updateAlerts)
  }
}

// ========== Measurement Functions ========== 


// Range‐switch thresholds (200 Ω ±5%)
static const float OHMS_RANGE_THRESHOLD   = 400.0f;
static const float OHMS_RANGE_DEADBAND    = 0.05f;
static const float OHMS_LOW_THRESHOLD     = OHMS_RANGE_THRESHOLD * (1.0f - OHMS_RANGE_DEADBAND);
static const float OHMS_HIGH_THRESHOLD    = OHMS_RANGE_THRESHOLD * (1.0f + OHMS_RANGE_DEADBAND);

// ADC count thresholds for gain switching
static const int ADC_COUNT_LOW_THRESHOLD  = 10000;
static const int ADC_COUNT_HIGH_THRESHOLD = 30000;

// Available ADS1115 gain levels and their voltage factors
//static const int   kGainLevels[]  = {GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN};
static const float kGainFactors[] = {GAIN_FACTOR_TWOTHIRDS, GAIN_FACTOR_1, GAIN_FACTOR_2, GAIN_FACTOR_4, GAIN_FACTOR_8, GAIN_FACTOR_16};
static const int   kNumGainLevels = sizeof(kGainLevels) / sizeof(kGainLevels[0]);

void measureResistance() {
  static float   prevResistance = 0.0f;
  static bool    firstRun       = true;
  static bool    currentRangeHigh;
  static uint8_t gainIndex;

  prevResistance = rawResistance;

  // --- Initialize on first run ---
  if (firstRun) {
    currentRangeHigh = (digitalRead(SETRANGE_PIN) == HIGH);
    gainIndex        = currentRangeHigh ? 0 : (kNumGainLevels - 1);
    firstRun         = false;
  }

  // --- Range control (auto vs. manual) ---
  if (ohmsAutoRange && !powerSave) {
    if (!currentRangeHigh && prevResistance > OHMS_HIGH_THRESHOLD) {
      currentRangeHigh = true;
      digitalWrite(SETRANGE_PIN, HIGH);
    } else if ( currentRangeHigh && prevResistance < OHMS_LOW_THRESHOLD) {
      currentRangeHigh = false;
      digitalWrite(SETRANGE_PIN, LOW);
    }
  } 
    else if(powerSave){
    currentRangeHigh = true;
    digitalWrite(SETRANGE_PIN, HIGH);  
  } 
  else {
    // Manual mode: force based on user setting
    if (ohmsHighRange && !currentRangeHigh) {
      currentRangeHigh = true;
      digitalWrite(SETRANGE_PIN, HIGH);
    } else if (!ohmsHighRange && currentRangeHigh) {
      currentRangeHigh = false;
      digitalWrite(SETRANGE_PIN, LOW);
    }
  }

  // --- ADC measurement & dynamic gain adjustment ---
  ads.setGain(kGainLevels[gainIndex]);
  int16_t adcCount = ads.readADC_SingleEnded(2);

  // If too close to rails, step gain down for larger range
  if (adcCount > ADC_COUNT_HIGH_THRESHOLD && gainIndex > 0) {
    gainIndex--;
    ads.setGain(kGainLevels[gainIndex]);
    adcCount = ads.readADC_SingleEnded(2);
  }
  // If well below full-scale, step gain up for better resolution
  else if (adcCount < ADC_COUNT_LOW_THRESHOLD && gainIndex < kNumGainLevels - 1) {
    gainIndex++;
    ads.setGain(kGainLevels[gainIndex]);
    adcCount = ads.readADC_SingleEnded(2);
  }

  // Convert ADC counts to voltage (V)
  ohmsVoltage = adcCount * kGainFactors[gainIndex] / 1000.0f;
  //Serial.println(ohmsVoltage);



if(powerSave){
  ZENER_MAX_V = EEPROM_SleepV;
}else{
  if(ohmsVoltage>ZENER_MAX_V){
    ohmsVoltage = ZENER_MAX_V-0.0001;
  }else{
    ZENER_MAX_V = EEPROM_MAXV;
  }
}

  // --- Compute raw resistance ---
  if (currentRangeHigh) {
    // High-range: resistive divider + Zener reference
    rawResistance = dividerR * (ohmsVoltage / (ZENER_MAX_V - ohmsVoltage));
  } else {
    // Low-range: constant-current measurement
    rawResistance = ohmsVoltage / (constantI - (ohmsVoltage / constantR));
  }

  // --- Initial auto-zero ---
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

  // --- Adjust display update rate & data rate ---
  if (!isBetween(rawResistance, prevResistance * 0.2f, prevResistance * 5.0f)&& ohmsVoltage<4.995) { 
    ads.setDataRate(RATE_ADS1115_475SPS);  // speed up for big change
    if(!powerSave){
      LCD_INTERVAL = 200;
    }
  
  
  } else if (!voltageDisplay && preciseMode && rawResistance < 3e6f
             && isBetween(rawResistance, prevResistance * 0.98f, prevResistance * 1.02f)) {
    ads.setDataRate(RATE_ADS1115_16SPS);    // slow when stable & null set
  } else if (!voltageDisplay && (ohmsVoltage < (ZENER_MAX_V - 0.02f))) {
    ads.setDataRate(RATE_ADS1115_128SPS);
  }

/*

  // --- Calibration correction factors ---
  if      (rawResistance < 0.75f)   calibratedResistance = rawResistance * CF_A;
  else if (rawResistance < 3.0f)    calibratedResistance = rawResistance * CF_B;
  else if (rawResistance < 7.0f)    calibratedResistance = rawResistance * CF_C;
  else if (rawResistance < 20.0f)   calibratedResistance = rawResistance * CF_D;
  else if (rawResistance < 70.0f)   calibratedResistance = rawResistance * CF_E;
  else if (rawResistance < 170.0f)  calibratedResistance = rawResistance * CF_F;
  else if (rawResistance < 700.0f)  calibratedResistance = rawResistance * CF_G;
  else if (rawResistance < 1700.0f) calibratedResistance = rawResistance * CF_H;
  else if (rawResistance < 7000.0f) calibratedResistance = rawResistance * CF_I;
  else if (rawResistance < 17000.0f)calibratedResistance = rawResistance * CF_J;
  else if (rawResistance < 70000.0f)calibratedResistance = rawResistance * CF_K;
  else if (rawResistance < 170000.0f)calibratedResistance = rawResistance * CF_L;
  else if (rawResistance < 700000.0f)calibratedResistance = rawResistance * CF_M;
  else if (rawResistance < 1700000.0f)calibratedResistance = rawResistance * CF_N;
  else  calibratedResistance = rawResistance * CF_O;

*/

      const float limits[15] = {
        0.75f, 3.0f, 7.0f, 20.0f, 70.0f,
        170.0f, 700.0f, 1700.0f, 7000.0f, 17000.0f,
        70000.0f, 170000.0f, 700000.0f, 1700000.0f, 7000000.0f
      };

      //float calib[15];

      size_t i = 0;
      while (rawResistance >= limits[i]) {
        i++;
      }
      calibratedResistance = rawResistance * defaultCalib[i];


    currentResistance = calibratedResistance;
    if (altUnits) {
      // Convert to °F via thermistor equation
      currentResistance = (1.0f / ((1.0f/298.15f) + (log(currentResistance/10000.0f)/3694.0f)) - 273.15f) * 1.8f + 32.0f;
    }

  // --- Open-circuit detection & final assignment ---
  if(currentMode != HighRMode){
  if (ohmsVoltage > (ZENER_MAX_V - 0.007f)) {
    currentResistance = 8e6f;  // treat as open    
    if (!voltageDisplay) {
      ads.setDataRate(RATE_ADS1115_475SPS);
    }
  }
  } 
} //This closes resistance measurement

void measureVoltage() {
  static float   prevVoltage     = 0.0f;
  static bool    firstVoltRun    = true;
  static size_t  gainIndexVolt;

  prevVoltage = newVoltageReading;

  if (firstVoltRun) {
    // start at highest resolution (max gain)
    gainIndexVolt = kNumGainLevels - 1;
    firstVoltRun  = false;
  }

  if (!(VACPresense && altUnits)) {
    // dynamic gain based on raw counts
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
    // convert to voltage
    newVoltageReading = (countV * kGainFactors[gainIndexVolt] / 1000.0f) * VOLTAGE_SCALE;
  } else {
    // fixed mid‐range gain in VAC‑altUnits mode
    ads.setGain(GAIN_EIGHT);
    countV = ads.readADC_Differential_0_1();
    newVoltageReading = (countV * GAIN_FACTOR_8 / 1000.0f) * VOLTAGE_SCALE;
  }

  // … (any subsequent filtering, scaling, display logic) …

  
  if(altUnits){
    newVoltageReading = (newVoltageReading*50);
  }

  // Track min and max voltages
  if (newVoltageReading > highV) {
    highV = newVoltageReading;
    timeAtMaxV = formatTime(millis());
    currentAtMaxV = Ireading;
    LCD_INTERVAL = 200;  // speed up display to reflect change
    if (MinMaxDisplay && voltageDisplay) {
      analogWrite(CONTINUITY_PIN, 200); // flash LED briefly on new max
      // (The LED flash here is very short; main alert logic handles sustained flashing)
    }
  }
  if (newVoltageReading < lowV) {
    lowV = newVoltageReading;
    timeAtMinV = formatTime(millis());
    currentAtMinV = Ireading;
    LCD_INTERVAL = 200;}

  
  // Update exponential moving median (for display smoothing)
  medianVoltageStep = (newVoltageReading - medianVoltage) * 0.2;
  medianVoltage += medianVoltageStep;
  
  if (!VACPresense && fabs(medianVoltageStep) > 0.3&& !preciseMode && newVoltageReading>0.5) {
    LCD_INTERVAL = 200;  // speed up display if big jump
  }else if(preciseMode && voltageDisplay){
    LCD_INTERVAL = 1000;
  }
  
  // Update moving average and variance (for VAC) using circular buffer
  
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
  if(preciseMode){ //If not in precise units, only update VAC with values above 2mV{
    VAC = 0; //VAC should be 0 when we are in precise (have an Ohm offset) mode
  }

  if( (VAC > 4.9 && (!altUnits && averageVoltage < 0.1) || (altUnits && VAC > 10)) || currentMode == VACmanual){
    VACPresense = true; //If there is more than 1VAC for the reading and average voltage is low, assume reading is VAC. 
  }else{
    VACPresense = false;
  }
}

void measureCurrent() {
  static float prevCurrent        = 0.0f;
  static bool  firstCurrentRun    = true;
  static size_t gainIndexCurrent;

  // remember last reading
  prevCurrent = Ireading;

  if (firstCurrentRun) {
    // start at highest resolution
    gainIndexCurrent  = kNumGainLevels - 1;
    firstCurrentRun   = false;
  }

  if (!currentOnOff && !ampsMode) {
    // skip entirely if user/system doesn’t need current
    return;
  }

  int32_t countI;

  if (Irange) {
    // —— High-RANGE mode: fixed mid-gain for best noise performance ——
    ads.setGain(GAIN_ONE);
    countI = ads.readADC_SingleEnded(3);
    // convert to shunt voltage (mV)
    currentShuntVoltage = (countI * GAIN_FACTOR_1 / 1000.0f)/ 0.185f;


  } else {
    // —— Low-RANGE mode: auto-ranging like measureVoltage() ——
    // set and read at current gain index
    ads.setGain(kGainLevels[gainIndexCurrent]);
    countI = ads.readADC_SingleEnded(3);

    // if we’re saturating, step DOWN resolution
    if (abs(countI) > ADC_COUNT_HIGH_THRESHOLD && gainIndexCurrent > 0) {
      --gainIndexCurrent;
      ads.setGain(kGainLevels[gainIndexCurrent]);
      countI = ads.readADC_SingleEnded(3);

    // if we’re far from full-scale, step UP resolution
    } else if (abs(countI) < ADC_COUNT_LOW_THRESHOLD 
               && gainIndexCurrent < kNumGainLevels - 1) {
      ++gainIndexCurrent;
      ads.setGain(kGainLevels[gainIndexCurrent]);
      countI = ads.readADC_SingleEnded(3);
    }

        // convert to amps (your low-range calibration)
    Ireading = currentShuntVoltage / 1.0f;

    // convert to shunt voltage (mV), then subtract zero‐offset Izero (also in mV)
    currentShuntVoltage = (countI * kGainFactors[gainIndexCurrent] / 1000.0f)
                          - Izero;
    // convert to amps (your high-range calibration)
    Ireading = currentShuntVoltage/1 ; //Value of the sense resistor
  }

  // apply noise floor
  if ((Irange && isBetween(Ireading, -0.01f,  0.1f)) ||
      (!Irange && Ireading < 0.0005f)) {
    Ireading = 0.0f;
  } else {
    // update min/max and log extremes
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
}


// ========== Display and Alert Functions ========== 

void updateDisplay() {
  // Prepare values for display
  //display.fillScreen(BLACK);
  display.fillRect(0, 0, 128, 64, BLACK);
  display.setTextColor(WHITE,BLACK);
  
  if(!screenSleep && currentMode!=Charging){

  // Determine which voltage value to use for display (fast vs smoothed)
  float voltageToDisplay;
  if (!preciseMode) {
    // Auto: use fast value if changing quickly, otherwise smooth
    voltageToDisplay = (fabs(medianVoltageStep) > 0.5) ? newVoltageReading : medianVoltage;
  } else {
    // Precise mode just uses the new voltage reading.
    voltageToDisplay = newVoltageReading;
  }
  // Format the values to a readable form with suffixes
  formatVoltageValue(voltageToDisplay, roundedV, vSuffix, vDigits);
  formatVoltageValue(lowV, roundedVlow, vSuffixlow, vDigitslow);
  formatVoltageValue(highV, roundedVhigh, vSuffixhigh, vDigitshigh);
  formatResistanceValue(displayResistance, roundedR, rSuffix, rDigits);
  formatResistanceValue(lowR, roundedRlow, rSuffixlow, rDigitslow);
  formatResistanceValue(highR, roundedRhigh, rSuffixhigh, rDigitshigh);

  // Battery voltage / mode indicator
  display.setTextSize(1);
  display.setCursor(72, 56);
  display.print("VIN:");
    display.print(batteryVoltage, 1);
    if (batteryVoltage < 5.1) {
      display.setCursor(72, 48);
    }
  
  //Mode Display
  display.setCursor(120, 0);
  display.print(currentMode);


  // If notable current present or in ampsMode, overlay current reading on display
  if (((Irange && !isBetween(Ireading, -0.01, 0.01)) || (!Irange && currentOnOff))
       || ampsMode) {
    display.setTextSize(2);
    display.setCursor(0, 16);
    if (Irange) {
      display.print("A:");
      display.print(Ireading, 2);
    } else {
      display.print("mA:");
      if(Ireading>0.100){
      display.print(Ireading * 1000.0, 2);
      }else if(Ireading>0.010){
      display.print(Ireading * 1000.0, 3);
      }else{
      display.print(Ireading * 1000.0, 4);
      }
    }
  }
  // If any current has been recorded (min/max), display them in small text
  if (IHigh != -6.0) {
    display.setTextSize(1);
    display.setCursor(0, 32);
    display.print("Ilow:");
    display.print(ILow, 3);
    display.print(" t:");
    display.print(timeAtMinI);
    display.println();
    display.print("Ihigh:");
    display.print(IHigh, 3);
    display.print(" t:");
    display.print(timeAtMaxI);
  }

  // Primary measurement display
  display.setTextSize(2);
  display.setCursor(0, 0);
  if (voltageDisplay) {
    // Voltage display mode
    if(VACPresense){
      display.print("VAC:");
      if(altUnits){
      display.print(VAC, 1);
      }else{
        display.print(VAC, 3);
      }
    }else{
    display.print("VDC:");

      if(preciseMode){
        display.setCursor(0, 16);
        display.print(roundedV, (vDigits+1));    
        //display.print(newVoltageReading, (vDigits+1));
        display.println(vSuffix);
        /*
        if(fabs(averageVoltage)<1.1 && newVoltageReading<3){
        display.setTextSize(1);
        display.println("VDC_avg:");
        display.print(averageVoltage,vDigits+5);
        }
        */
      } else {
      display.print(roundedV, vDigits);    
      display.println(vSuffix);
      }
      
      if(deltaV != 0){
      display.setTextSize(1);
      display.print("ref:");
      display.println(deltaV, deltaVdigits);
      display.print("delta:");
      display.println(newVoltageReading-deltaV, 4);  
      }

    
    
    }
    
    if (MinMaxDisplay) {
      // Show min and max voltage with timestamps
      display.setTextSize(1);
      display.setCursor(0, 16);
      display.print("Min:");
      display.print(roundedVlow, vDigitslow);
      display.print(vSuffixlow);
      display.print("  t:");
      display.print(timeAtMinV);
      display.println();
      display.print("Max:");
      display.print(roundedVhigh, vDigitshigh);
      display.print(vSuffixhigh);
      display.print("  t:");
      display.print(timeAtMaxV);
      display.println();
      display.print("Range:");
      display.print(highV - lowV, 3);
    }
    if (VAC > 0.1 && !VACPresense) {
      // Show AC component if significant
      display.setCursor(0, 48);
      display.setTextSize(1);
      display.print("VAC");
      display.setCursor(0, 56);
      display.print("RMS:");
      display.setCursor(26, 48);
      display.setTextSize(2);
      display.print(VAC, 1);
    } else {
      // Otherwise, show elapsed time or debug info
      display.setCursor(0, 48);
      display.setTextSize(2);
      display.print(formatTime(millis()));
      if (debugMode) {
        display.setCursor(0, 32);
        display.setTextSize(1);
        display.print("Vraw:");
        
      }
    }
  } else {
    // Resistance display mode
    
    if (ohmsVoltage < (ZENER_MAX_V - 0.007) || currentMode == HighRMode) {
      // If within measurable range, display the resistance value
      /*
      if(powerSave){
        display.setTextSize(1);
        display.print("R:");
        display.println("SLEEP");
        display.print("mVR:");
        if(ohmsVoltage<1){
        display.print(ohmsVoltage * 1000.0, 1);
        }else{
        display.print(ohmsVoltage * 1000.0, 0);
        }
      
      }else{      
      */
      if(altUnits){
        display.print("F:");  
        }else{
        display.print("R:");
        //display.drawBitmap(0, 0, OHM_8x8, 8, 8, SSD1306_WHITE);
        
        //display.print("R:");
      }
      //display.setCursor(16, 0);
      display.print(roundedR, rDigits);
      display.print(rSuffix);
      if (MinMaxDisplay) {
        // Show min and max resistance
        display.setTextSize(1);
        display.setCursor(0, 16);
        display.print("Min:");
        display.print(roundedRlow, rDigitslow);
        display.print(rSuffixlow);
        display.println();
        display.print("Max:");
        display.print(roundedRhigh, rDigitshigh);
        display.print(rSuffixhigh);
        // Also display the small voltage across resistor in mV (for debug/reference)
        display.setCursor(102, 0);
        display.print("mVR:");
        display.setCursor(102, 8);
        if(ohmsVoltage<1){
        display.print(ohmsVoltage * 1000.0, 1);
        }else{
        display.print(ohmsVoltage * 1000.0, 0);
        }
      } else {
        // If not showing min/max, at least show the mV drop in line
        display.setTextSize(2);
        display.println();
        display.print("mVR:");
        if(ohmsVoltage<1){
        display.print(ohmsVoltage * 1000.0, 1);
        }else{
        display.print(ohmsVoltage * 1000.0, 0);
        }
      }
      display.setTextSize(1);
      display.setCursor(0, 32);
      if (zeroOffsetRes != 0) {
        
        display.print("null mOhms:");
        display.println(zeroOffsetRes*1000, 2);
        if(preciseMode){
          display.print("Precise Mode");
          }
      }
      if (debugMode) {
        display.setCursor(60, 32);
        display.print("ADC:");
        display.print(adcReadingOhms);
      }
      //}
    } else {
      // Out of range (open circuit or over-limit)
      if(!altUnits){
        display.println("R:OPEN");
        display.setTextSize(1);
        display.print("mV:");
        display.print(ohmsVoltage * 1000.0, 2);
        }else{
          display.print("F:OPEN");
        }
      if (MinMaxDisplay) {
        display.setTextSize(1);
        display.setCursor(0, 16);
        display.print("Min:");
        display.print(roundedRlow, rDigitslow);
        display.print(rSuffixlow);
        display.println();
        display.print("Max:");
        display.print(roundedRhigh, rDigitshigh);
        display.print(rSuffixhigh);
      }
    }
      display.setCursor(0, 48);
      display.setTextSize(2);
      display.print(formatTime(millis()));
    
  }
  }else{
    formatTime(millis());
    int step = seconds % 10;    
    display.setCursor((8+(4*step)), 32);
    display.setTextSize(1);
    display.print(".");
  }
  //display.display(); // update the OLED with all the drawn content
}

void updateAlerts() {
  // Manage continuity buzzer/LED and flashlight mode LED brightness
  unsigned long now = millis();
  if (!flashlightMode) {
    // Continuity check (low resistance)
    bool continuity = (isBetween(currentResistance, -20.0, 1.0) ||
                      (isBetween(currentResistance, -20.0, 20.0) && ohmsHighRange))
                      //|| (prevResistance > 2000000 && currentResistance < 1000000)
                      ;

    bool logicVoltage = ((!altUnits && (fabs(newVoltageReading) > 3.2 || (VACPresense && VAC > 1.0))) ||(altUnits && (fabs(newVoltageReading) > 30.2 || (VACPresense && VAC > 15.0))));
    if (continuity) {
      // If continuity detected, beep/flash the continuity pin
      if (!rFlag) {
        // Start of beep
        analogWrite(CONTINUITY_PIN, 200);
        rFlag = true;
      } else if ((now % 1000 <= 100 || isBetween(now % 1000, 300, 400)) && rFlag) {
        // Keep beeping in a pattern: on for 100ms, then off, with a double pulse
        analogWrite(CONTINUITY_PIN, 50);
      } else {
        analogWrite(CONTINUITY_PIN, 0);
      }
    } else if (logicVoltage) {
      // If significant voltage in resistance mode, warning flash
      if (!vFlag) {
        analogWrite(CONTINUITY_PIN, 200);
        vFlag = true;
      } else if ((now % 1000 <= 100) && vFlag && (!VACPresense)) {
        analogWrite(CONTINUITY_PIN, 50);
      } else {
        analogWrite(CONTINUITY_PIN, 0);
      }
    } else {
      // No alert condition
      analogWrite(CONTINUITY_PIN, 0);
      vFlag = false;
      rFlag = false;
    }
  } else {
    // Flashlight mode: map resistance to LED brightness
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

// ========== Utility Functions ========== 

void logCurrentData(float voltage, float timeSec, float current) {
  // Log current, voltage, and relative time to circular buffers (for 'L' command)
  for (int i = LOG_SIZE - 1; i > 0; --i) {
    loggedCurrents[i] = loggedCurrents[i-1];
    loggedVoltagesAtI[i] = loggedVoltagesAtI[i-1];
    loggedTimeStamps[i] = loggedTimeStamps[i-1];
  }
  loggedCurrents[0] = current;
  loggedVoltagesAtI[0] = voltage;
  loggedTimeStamps[0] = timeSec - tLogStart; // Store time relative to first entry to avoid large numbers
  
  /*
  float baseTime = ;
  for (int i = LOG_SIZE - 1; i > 0; --i) {
    loggedTimeStamps[i] = loggedTimeStamps[i-1];
  }
  */
  
}

void formatResistanceValue(float value, float &outValue, String &outSuffix, int &outDigits) {
  // Format resistance value into appropriate scale (Ω, kΩ, MΩ) for display
  if(value > 90000000.0){
    outValue = value / 1000000.0;
    outSuffix = "M";
    outDigits = 1;  
  }else if(value > 9000000.0){
    outValue = value / 1000000.0;
    outSuffix = "M";
    outDigits = 2;
  }else if (value > 900000.0) {        // use megaohm
    outValue = value / 1000000.0;
    outSuffix = "M";
    outDigits = 3;
  } else if (value > 900.0) {    // use kiloohm
    outValue = value / 1000.0;
    outSuffix = "k";
    outDigits = (value > 90000.0) ? 3 : 4;
  } else {
    if (value < 0.9) {           // use milli-ohm (if we ever measure sub-ohm)
      outValue = value * 1000.0;
      outSuffix = "m";
      if(value < 0.09){
        outDigits = 1;
      }else{
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
  // Format voltage value into V or mV for display
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
  // Convert milliseconds to "MM:SS" format
  unsigned long totalSeconds = milliseconds / 1000;
  unsigned int minutes = totalSeconds / 60;
  seconds = totalSeconds % 60;
  char buf[6];
  snprintf(buf, sizeof(buf), "%02u:%02u", minutes, seconds);
  return String(buf);
}

void ReZero() {
  // Reset min/max tracking values (and related state)
  highR = 0.0;
  lowR = 7000000.0;
  highV = -100.0;
  lowV = 100.0;
  IHigh = -6.0;
  ILow = 6.0;
  IMedian = 0.0;
  deltaV = 0;
  // Note: zeroOffsetRes is not cleared here, as we want to keep the user-set zero through resets
  // Also keep initialZeroSet as true to not auto-zero again unless explicitly cleared.
}

void checkModeButton() {
  bool buttonState = digitalRead(BUTTON_PIN);
  unsigned long currentTime = millis();

  // Detect new press only when button transitions from HIGH to LOW
  if (buttonState == LOW && !buttonPreviouslyPressed && (currentTime - lastDebounceTime > DEBOUNCE_DELAY)) {
    lastDebounceTime = currentTime;
    buttonPreviouslyPressed = true;

    // Cycle to the next mode
    currentMode = static_cast<Mode>((currentMode + 1) % NUM_MODES);
    Serial.print("Mode changed to: ");
    Serial.println(currentMode); // Will print enum as int unless you map to strings
  }

  // Detect button release
  if (buttonState == HIGH && buttonPreviouslyPressed) {
    buttonPreviouslyPressed = false;
  }
}