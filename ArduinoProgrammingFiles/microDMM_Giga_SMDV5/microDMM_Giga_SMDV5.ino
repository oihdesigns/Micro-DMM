#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino_GigaDisplay.h>

//GIGA HID Related
#include "PluggableUSBHID.h"
#include "USBKeyboard.h"
USBKeyboard Keyboard;

//Giga USB Related
#include <Arduino_USBHostMbed5.h>
#include <DigitalOut.h>
#include <FATFileSystem.h>

GigaDisplayRGB rgb; //create rgb object
GigaDisplayBacklight backlight; // This is for the backlight

#include "RPC.h"

// ————— USB mass-storage objects —————
USBHostMSD        msd;
mbed::FATFileSystem usb("usb");

uint16_t lut[] = {

    0x0800,0x08c8,0x098f,0x0a52,0x0b0f,0x0bc5,0x0c71,0x0d12,0x0da7,0x0e2e,0x0ea6,0x0f0d,0x0f63,0x0fa7,0x0fd8,0x0ff5,
    0x0fff,0x0ff5,0x0fd8,0x0fa7,0x0f63,0x0f0d,0x0ea6,0x0e2e,0x0da7,0x0d12,0x0c71,0x0bc5,0x0b0f,0x0a52,0x098f,0x08c8,
    0x0800,0x0737,0x0670,0x05ad,0x04f0,0x043a,0x038e,0x02ed,0x0258,0x01d1,0x0159,0x00f2,0x009c,0x0058,0x0027,0x000a,
    0x0000,0x000a,0x0027,0x0058,0x009c,0x00f2,0x0159,0x01d1,0x0258,0x02ed,0x038e,0x043a,0x04f0,0x05ad,0x0670,0x0737

};


static size_t lut_size = sizeof(lut) / sizeof(lut[0]);

// ===== Hardware Setup Constants =====
Adafruit_ADS1115 ads;


#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>    // Touch polling functions :contentReference[oaicite:1]{index=1}

GigaDisplay_GFX display;
Arduino_GigaDisplayTouch touch;

#define BLACK 0x0000
#define WHITE 0xFFFF
#define RED 0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define YELLOW  0xFFE0

const uint16_t btnX[4] = { 700, 600, 700, 600 };
const uint16_t btnY[4] = {  0,  0, 100, 100 };
const uint16_t btnColor[4] = { RED, GREEN, BLUE, YELLOW };
const char*    btnLabel[4] = { "MODE", "LOG", "MinMax", "Ref" };

// Press flags
bool redPress    = false;
bool greenPress  = false;
bool bluePress   = false;
bool yellowPress = false;


// Insert your data array and its length somewhere above or in another file:
extern const float samples[];    // e.g. float samples[] = {1.2, 2.3, 1.8, …};
extern const int SAMPLE_COUNT_PLOT = 100;   // set to the number of elements in samples[]

// Plot configuration:
static const int PLOT_SIZE = 400;   // 400 px square
static const int PLOT_X    = 0;     // top-left corner of plot area
static const int PLOT_Y    = 80;

// Color definitions (RGB565):
#define COLOR_BG     0x0000  // black
#define COLOR_DATA   0xFFFF  // white
#define COLOR_MIN    0x07E0  // green
#define COLOR_MEAN   0xFFE0  // yellow
#define COLOR_MAX    0xF800  // red


// Digital Pin definitions
const int OHMPWMPIN = 2;
const int TYPE_PIN      = 3;    // Mode button (also triggers flashlight mode if held at boot)
#define BUTTON_PIN 4         // For Mode
const int VbridgePin = 5; // Control Voltage Bridge MOSFET
const int CONTINUITY_PIN = 6;   // Buzzer or LED for continuity/alerts
const int SETRANGE_PIN = 7;   // Controls high/low resistance range
const int kbButton = 8;   // Toggle: if LOW -> keyboard mode; if HIGH -> serial mode
const int cycleTrack = 52; // take log pin
const int backlightRef =  64; //Ref for backlight toggle
const int backlightControlHigh =  65; //Makes Backlight Full On
const int backlightControlLow =  63; //Turn Backlight Low



//Analog Pins
const int logButton = A1; // take log pin
const int micPin = A7; // Exposed Mic Pin


//#define IR_RECEIVE_PIN   8      // IR receiver input pin
//const int BATT_PIN      = A2;   // Battery voltage analog input
//#define enablePin  BAT_READ_EN  // Pin for enabling battery voltage reading
//#define BATT_PIN BAT_DET_PIN
//const int LowerButton = 10;   // 



//RPC Related
static bool txFlag = false;        // what we want to share 
static uint8_t lastSent = 0xFF;     // force initial send
static bool humFlag = false;        // what we want to share



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
float constantI            = 0.02012f;   // A for low-resistance constant-current source  *as of 25/08/13
const float constantR            = 330.0f;     // Ω internal resistor in constant-current circuit
const float dividerR             = 22000.0f;   // Ω series resistor for high-resistance divider
float ZENER_MAX_V          = 4.9854f;       // this changes in the program and the value here doesn't / shouldn't matter
float EEPROM_MAXV = 4.9854f; //this is constant. as of 25/08/13
float EEPROM_SleepV = 0.6117; //this is constant.

float VOLTAGE_SCALE = -68.57008; // Calibration scale factor for voltage input
float VOLTAGE_SCALE_Negative = -68.55928;
float giga_absfactor = 0.0;//12; // Giga is high by this on battery power. 

//Mode Rotate Related
#define DEBOUNCE_DELAY 300    // debounce time in milliseconds
enum Mode {
  Voltmeter,
  HighRMode,
  Default,
  VACmanual,
  Type,
  Low,
  AltUnitsMode,
  rPlotMode,
  //Impedance,
  //RelayControl,
  Charging,
  NUM_MODES
};

Mode currentMode = Voltmeter;
Mode previousMode = Voltmeter;

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
unsigned int secondsTime = 0;
unsigned long previousTouchTime;

// Global measurement variables
int16_t adcReadingVoltage = 0;  // raw ADC reading for voltage (differential)
int16_t adcReadingOhms    = 0;  // raw ADC reading for resistance
int16_t adcReadingCurrent = 0;  // raw ADC reading for current
float batteryVoltage = 0.0;     // measured battery voltage (V)
int16_t countV;
int16_t adcCount;
static uint8_t gainIndex;
int32_t countI;

float newVoltageReading = 0.0;  // latest measured voltage (V)
float averageVoltage = 0.0;     // moving average (DC) of voltage
float VAC = 0.0;                // AC RMS component of voltage
float previousVoltage = 0.0;    // previous loop's voltage (for auto-range decision)
float medianVoltage = 0.0;      // exponentially filtered voltage for display
float medianVoltageStep = 0.0;
float previousmeanY = 0.0;
float meanY = 0.0;

float rawResistance = 0.0;      // calculated resistance before calibration (Ω)
float calibratedResistance = 0.0; // resistance after applying calibration factors (Ω)
float currentResistance = 1000001.0;  // final resistance value used for display (before zero offset)
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

//Rolling Buffer for Resistance Samples
const int NUM_RESISTANCE_SAMPLES = 100;
float resistanceSamples[NUM_VOLTAGE_SAMPLES] = {0.0};
int resistanceSampleIndex = 0;

//Rolling Buffer for Current Samples
const int NUM_CURRENT_SAMPLES = 100;
float currentSamples[NUM_VOLTAGE_SAMPLES] = {0.0};
int currentSampleIndex = 0;


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

bool continuity = false;
bool logMode = false;
int usbDelay = 0;

// Flags for transient states
bool flashlightMode = false;  // flashlight mode active (button held at startup)
bool flashlightChecked = false; // whether we have checked the button for flashlight mode at startup
bool buttonPressed = false;   // debounced state of the button (pressed = true)
unsigned long buttonPressTime = 0; // timestamp when button was pressed
bool vFlag = false;           // indicates voltage alert is active (to manage flashing pattern)
bool rFlag = false;           // indicates resistance continuity alert is active
bool VACPresense = false;       // indicates VAC presence
bool cycleTracking = false;
bool vClosedflag = false;
bool vClosedflagPrevious = false;

// Logging buffers for current vs time (for 'L' command)
const int LOG_SIZE = 100;
const int SAMPLE_COUNT = 100;
bool takeLog = false;
int samplesTaken = 0;
float tLogStart = 0;
float tLogEnd = 0;
float loggedCurrents[LOG_SIZE] = {0};
float loggedVoltagesAtI[LOG_SIZE] = {0};
float loggedTimeStamps[LOG_SIZE] = {0};

//V Float Detect Related
bool vFloating = false;
float bridgeV = 0.0;
bool Vzero = true;
bool vClosed = false;
bool vUndefined = true;

int blinkLimit = 0; //Limit how much the LED can blink between screen updates



//Auto Log Related

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


// ========== Setup Function ========== 
void setup() {

  
  Wire1.begin();              // use SDA1/SCL1 (D102/D101)
  Wire1.setClock(400000);     // optional: 100k, 400k, or 1MHz supported
  //Wire1.beginTransmission(0x68);
  //Wire1.write(0x00);
  //Wire1.endTransmission();
  

  Serial.begin(115200);
  delay(100);
  //while (!Serial) { /* wait for USB-Serial */ }
  Serial.println("Setup Start");

    // Initialize OLED display

  display.begin();             // init hardware
  display.setRotation(1);      // landscape mode
  //touch.setRotation(1);



  display.setTextSize(4);
  display.setTextColor(WHITE);
  // Splash screen

  //display.setCursor(0, 0);  
  //display.drawBitmap(0, 0, MICRO_5x7, 5, 7, SSD1306_WHITE);
  display.fillScreen(BLUE);   // clear to black
  display.setCursor(0, 0);
  display.println("GIGA METER");
  //display.print(EEPROM.read(1));//Reads the EEPROM and determines the correct splash   
  //display.setCursor(0, 48);
  //display.println("GIGA METER");
  
  
  // initialize touch
  if (!touch.begin()) {
    // touch init failed — halt
    while (1) {}
  }

    //Related to USB stick usage
        // Enable the USB-A port
      pinMode(PA_15, OUTPUT);
      digitalWrite(PA_15, HIGH);
      
      // Some carriers require explicit VBUS enable
      mbed::DigitalOut vbusEnable(PB_8, 1);    
      // Wait for drive to appear
      Serial.print("Waiting for USB drive");
      display.setTextSize(2);
      display.println("Waiting 400ms for USB drive");
      while (!msd.connect() && usbDelay<2){
        delay(200);
        usbDelay++;
        }
        if(msd.connect()){
        logMode = true;
        Serial.print('\nDrive connected.');
        display.println("Drive connected");
        // Mount the FAT32 filesystem
      int err = usb.mount(&msd);
      if (err) {
        Serial.print("Mount failed: ");
        display.println(err);
        while (true);  // halt
      }
      Serial.println("Filesystem mounted.");      
      }else{
        logMode = false;
        display.println("No USB Drive.");
        Serial.print("No Drive");
      }


  // Configure pins
  pinMode(TYPE_PIN, INPUT_PULLUP);
  pinMode(CONTINUITY_PIN, OUTPUT);
  pinMode(SETRANGE_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Assuming active-low button
  pinMode(OHMPWMPIN, OUTPUT);
  pinMode(logButton, INPUT_PULLUP);
  pinMode(cycleTrack, OUTPUT);
  pinMode(VbridgePin, OUTPUT);
  pinMode(micPin, INPUT);
  pinMode(backlightRef, OUTPUT);
  pinMode(backlightControlHigh, INPUT_PULLUP);
  pinMode(backlightControlLow, INPUT_PULLUP);



/*
  // 5) Assign your named variables
  EEPROM_MAXV  = defaultCalib[17];
  EEPROM_SleepV = defaultCalib[16];
  constantI     = defaultCalib[15];
  VOLTAGE_SCALE= defaultCalib[18];
*/



  // Initialize ADS1115 ADC
  if (!ads.begin(0x48, &Wire1)) {
    display.setCursor(0, 0);
    display.setTextSize(1);
    Serial.println("ADS1115 not found");
    display.println("ADS1115 not found");
    //display.display();
    while (1); // halt
  }
  display.println("ADS1115 found");
  Serial.println("ADS1115 good");

  ads.setDataRate(RATE_ADS1115_16SPS); // slow rate b/c default is resistance
  // Initial current zero calibration
  ads.setGain(GAIN_TWOTHIRDS);  // ±6.144V range to read baseline
  delay(100);

  // Print available commands (for user reference)
  Serial.println(F("Commands: Q,q,Z,C,R,S,V,A,F,M,B,D,e,E,I,L,?"));

  
  
  //Ammeter Auto Detect
    delay(100);
    adcReadingCurrent = ads.readADC_SingleEnded(3);
    
    currentShuntVoltage = adcReadingCurrent * GAIN_FACTOR_TWOTHIRDS / 1000.0;
    if (adcReadingCurrent < 300 || isBetween(currentShuntVoltage, 1.5, 3.5)) {
      // If no current (shunt pulled to ground) or baseline ~2.5V, current sensor is present
      if (isBetween(currentShuntVoltage, 1.5, 3.5)) {
        Irange = true;         // high-range current mode
        Izero = currentShuntVoltage; // store baseline (~2.5V)
      } else {
        Irange = false;        // low-range current mode
        Izero = 0.0;
      }
      currentOnOff = true;
      Serial.print("Ammeter Enabled:");
      Serial.println(currentShuntVoltage);
      display.print("Ammeter Enabled:");
      display.println(currentShuntVoltage);
    } else {
      // Current sensor not connected or reading abnormal -> disable current measurement
      currentOnOff = false;
      Serial.print("Ammeter Disabled:");
      Serial.println(currentShuntVoltage);
      display.print("Ammeter Disabled:");
      display.println(currentShuntVoltage);
      Ireading = 0.0;
    }
  


    
    display.println("Booting Core 2");
    RPC.begin();
    uint8_t pack = (txFlag?1:0) | (humFlag?2:0);
    RPC.write(pack);
    lastSent = pack;
    display.println("Core 2 Booted");


  delay(1000);
  display.fillScreen(BLACK);
  
  // draw our four buttons
  display.setTextSize(2);
  for (int i = 0; i < 4; i++) {
    // outer black border
    display.fillRect(btnX[i], btnY[i], 100, 100, BLACK);
    // inner colored square
    display.fillRect(btnX[i] + 10, btnY[i] + 10, 80, 80, btnColor[i]);

    // label centered
    display.setTextColor(
      (btnColor[i] == YELLOW) ? BLACK : WHITE
    );
    // approximate centering
    int16_t w = strlen(btnLabel[i]) * 6 * 2;    // 6px per char × textSize(2)
    int16_t h = 8  * 2;                         // 8px font height × textSize(2)
    int16_t tx = btnX[i] + (100 - w)/2;
    int16_t ty = btnY[i] + (100 - h)/2;
    display.setCursor(tx, ty);
    display.print(btnLabel[i]);
  }
   
  rgb.begin(); //init the library

  /*

  digitalWrite(backlightRef,LOW);
  backlight.begin();
  backlight.set(50);
  */



  Serial.println("Setup End");

}

// ========== Main Loop ========== 
void loop() {
    //  Serial.println("Loop Start");
  unsigned long currentMillis = millis();

  /*
  //Backlight Control
  if(digitalRead(backlightControlHigh)==LOW){
    backlight.set(100);
  }else if(digitalRead(backlightControlLow)==LOW){
    backlight.set(10);
  }else{
    backlight.set(50);
  }
  */
  
  cycleTracking = !cycleTracking;
  if(cycleTracking){
    digitalWrite(cycleTrack, HIGH);
    }else{
    digitalWrite(cycleTrack, LOW); 
    }

    // read touch points
      uint8_t contacts;
      GDTpoint_t points[5];
      contacts = touch.getTouchPoints(points);

      // clear all flags by default
      redPress    = greenPress = bluePress = yellowPress = false;

      if (contacts > 0 && currentMillis > previousTouchTime + 500) {
        previousTouchTime = millis();
        
        uint16_t x = points[0].x;
        uint16_t y = points[0].y;

        uint16_t rawX = points[0].x;
        uint16_t rawY = points[0].y;
        //uint16_t x, y;

        switch (1) {  // the same value you passed to display.setRotation()
          case 0:  // no rotation
            x = rawX;
            y = rawY;
            break;
          case 1:  // 90° CW
            x = rawY;
            y = display.width() - rawX - 350;
            break;
          case 2:  // 180°
            x = display.width()  - rawX;
            y = display.height() - rawY;
            break;
          case 3:  // 270° CW (or 90° CCW)
            x = display.height() - rawY;
            y = rawX;
            break;
        }

      Serial.print("raw:("); Serial.print(rawX); Serial.print(',');
      Serial.print(rawY); Serial.print(") → mapped:(");
      Serial.print(x); Serial.print(',');
      Serial.print(y); Serial.println(')');

      // check each button
      for (int i = 0; i < 4; i++) {
        if ( x >= btnX[i]
          && x <  btnX[i] + 100
          && y >= btnY[i]
          && y <  btnY[i] + 100
        ) {
          switch(i) {
            case 0: redPress    = true; break;
            case 1: greenPress  = true; break;
            case 2: bluePress   = true; break;
            case 3: yellowPress = true; break;
          }
          break;
        }
      }
    }



  previousMode=currentMode;

  if(bluePress|| isBetween(analogRead(micPin),100,275)){
    MinMaxDisplay = true;
    ReZero();
  }

  if(yellowPress || analogRead(micPin)<60 ){
          // Short press: type current reading via USB keyboard
      if(currentMode==Type){      
      if (voltageDisplay) {
        Keyboard.printf("%.*f",vDigits,newVoltageReading);
      } else {
        if(displayResistance<1){
        Keyboard.printf("%.*f", rDigits+2, displayResistance);
        }else{
        Keyboard.printf("%.*f", rDigits, displayResistance);  
        }
      }
      // Press Right Arrow after typing (to move cursor, e.g., to next cell)
      Keyboard.key_code(RIGHT_ARROW);
      //Keyboard.key_code(UP_ARROW);
      
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

if((digitalRead(logButton)==0) || greenPress) {
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
        rgb.on(0, 255, 0); //Green On
        Serial.println("Samples logged!");

      if(logMode){
      //USB File Write
              // open for append
          
          manuallogArraysToCSV();
          manuallogCount++;
          
          /*
          
          FILE* f = fopen("/usb/data.csv", "a");
          if (f) {
            fprintf(f,"DataLog:");
            fprintf(f,"%.2f,%.2f,%lu\n", newVoltageReading, previousVoltage, millis());
            fclose(f);
            Serial.println("Logged: " + String(newVoltageReading) + "," + String(previousVoltage) + "," + String(millis()));
          } else {
            Serial.println("Error: could not open data.csv");
          }
          
          */
          
          }   
      
      
      
      }
      
}


  handleButtonInput();


  // Periodic ADC measurements
  if (currentMillis - previousAdcMillis >= ADC_INTERVAL) {
    previousAdcMillis = currentMillis;
    if(!takeLog && currentMode != Voltmeter && currentMode != VACmanual){ //Bypass Resistance if taking a log or in voltmeter more
    measureResistance();
    }
    measureVoltage();
    if(currentOnOff){
    measureCurrent();
    }
    
  //PowerSave Related
    if(!powerSave){
        if(ohmsVoltage>EEPROM_MAXV-0.002 && !timeHighset && currentMode != HighRMode){
          timeHigh = millis();
          timeHighset = true;
        }
        if (timeHighset && currentResistance < 20000){
          timeHighset = false;
        }    
        if (timeHighset && timeHigh + 5000 < millis()){
          powerSave = true;
        }
    }

    if(powerSave || currentMode==Charging || currentMode == Voltmeter){
      analogWrite(OHMPWMPIN, 254);
        
      if(ohmsVoltage < EEPROM_SleepV-0.01 && currentMode != Voltmeter){
          powerSave = false;
          timeHighset = false;
          analogWrite(OHMPWMPIN, 0);
        }
      }
  
    // Determine which primary measurement to display automatically
    
  if ( fabs(countV)>500  || currentMode == Voltmeter) {
    voltageDisplay = true;
    if(preciseMode){
    ads.setDataRate(RATE_ADS1115_16SPS); //slow sampling if 0 is set  
    }else{
    ads.setDataRate(RATE_ADS1115_860SPS); //fast sampling for voltage
    }
  
  }
  if (isBetween(currentResistance, -10.0, 100.0)&& currentMode != Voltmeter || currentMode == HighRMode || currentMode == rPlotMode) {
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

  if(!voltageDisplay && isBetween(currentResistance,20,1000000)){ //This is the RPC logic to indicate when to change the output tone
      humFlag = 1; //RPC Comms; Low is continuity
      }else{
      humFlag = 0;
  }

  //Update RPC Comms
  uint8_t pack = (txFlag?1:0) | (humFlag?2:0);
  if (pack != lastSent) {
    RPC.write(pack);
    lastSent = pack;
    //Serial.print("last sent RPC:");
    //Serial.println(lastSent);
  }

  if(!AutologTriggered && writePrimed){    
    autologArraysToCSV();

      
    analogWrite(CONTINUITY_PIN, 100);
    rgb.on(0, 255, 0); //Green On
    autologCount++;
    Serial.println("Values Autologged:");
    Serial.println(autologCount);
    
    writePrimed = 0;
  }
  
  
  if( //Autolog Conditions
    (logMode && !VACPresense) && ( //SD mode is on and we're not traking AC, AND
    (IHigh==Ireading && IHigh > 0.05) || (ILow==Ireading && ILow < -0.05) || (newVoltageReading==highV && highV>0.05) || (newVoltageReading==lowV && lowV < -0.05 ) || //if either I/ V high / low is triggered OR
    (abs(lastLoggedI)>(abs(Ireading)+Istep) || abs(lastLoggedI)<(abs(Ireading)-Istep)) // If I reading is more than Istep different than the previous reading
    // || digitalRead(RED_PIN) == LOW 
    )//SD mode close
  )
  {
    //analogWrite(CONTINUITY_PIN, 10);             
    float TimeS = (millis());
    tLogEnd  = millis() / 1000.0;
    logValues(newVoltageReading, (TimeS/1000), Ireading);
    AutologTriggered=1;
    writePrimed=1;
    lastLoggedI = Ireading;



  }else{ //Update display when autolog not triggered

  AutologTriggered=0;
  
  // Update display at interval
    if ((currentMillis - previousLcdMillis >= LCD_INTERVAL)&&!takeLog) {
      previousLcdMillis = currentMillis;
      LCD_INTERVAL = screenRefreshFast ? 500 : 1000; // adjust refresh rate if toggled
      updateDisplay();
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
  } //close display update




}//close loop



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
  bool isPressed = (digitalRead(TYPE_PIN) == LOW) ;
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
        Keyboard.printf("%.*f",vDigits,newVoltageReading);
      } else {
        if(displayResistance<1){
        Keyboard.printf("%.*f", rDigits+2, displayResistance);
        }else{
        Keyboard.printf("%.*f", rDigits, displayResistance);  
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

float OHMS_RANGE_THRESHOLD   = 400.0f;
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


  prevResistance = rawResistance;

  // --- Initialize on first run ---
  if (firstRun) {
    currentRangeHigh = (digitalRead(SETRANGE_PIN) == LOW);
    gainIndex        = currentRangeHigh ? 0 : (kNumGainLevels - 1);
    firstRun         = false;
  }

  if(currentMode != rPlotMode || currentMode != HighRMode){
  float OHMS_RANGE_THRESHOLD   = 400.0f;
  }else{
    float OHMS_RANGE_THRESHOLD   = 40.0f;
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
  adcCount = ads.readADC_SingleEnded(2);

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
        
        
        
        
        /*
        
        const float limits[15] = {
          0.75f, 3.0f, 7.0f, 20.0f, 70.0f,
          170.0f, 700.0f, 1700.0f, 7000.0f, 17000.0f,
          70000.0f, 170000.0f, 700000.0f, 1700000.0f, 700
           0000.0f
        };

        //float calib[15];

        size_t i = 0;
        while (rawResistance >= limits[i]) {
          i++;
        }
        calibratedResistance = rawResistance * defaultCalib[i];

        */


      currentResistance = calibratedResistance;
      if (altUnits) {
        // Convert to °F via thermistor equation
        currentResistance = (1.0f / ((1.0f/298.15f) + (log(currentResistance/10000.0f)/3694.0f)) - 273.15f) * 1.8f + 32.0f;
      }

    resistanceSamples[resistanceSampleIndex] = ohmsVoltage;
    resistanceSampleIndex = (resistanceSampleIndex + 1) % NUM_RESISTANCE_SAMPLES;
    
    
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

  if(VACPresense || currentMode == VACmanual){
    // fixed mid‐range gain in VAC‑altUnits mode
    ads.setGain(GAIN_ONE);
    countV = ads.readADC_Differential_0_1();
    newVoltageReading = ((countV * GAIN_FACTOR_1 / 1000.0f) * VOLTAGE_SCALE) - giga_absfactor;
  }
  else if (!altUnits) {
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
    if(countV>0){
      newVoltageReading = ((countV * kGainFactors[gainIndexVolt] / 1000.0f) * VOLTAGE_SCALE) - giga_absfactor;
      }else{
      newVoltageReading = ((countV * kGainFactors[gainIndexVolt] / 1000.0f) * VOLTAGE_SCALE_Negative) - giga_absfactor;  
    }
  
  } else {
    // fixed mid‐range gain in VAC‑altUnits mode
    ads.setGain(GAIN_EIGHT);
    countV = ads.readADC_Differential_0_1();
    newVoltageReading = ((countV * GAIN_FACTOR_8 / 1000.0f) * VOLTAGE_SCALE) - giga_absfactor;
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
      rgb.on(0, 255, 0); //Green On
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

  if(((fabs(averageVoltage) < 0.030 && currentMode != VACmanual && fabs(newVoltageReading)<0.05) || (currentMode == VACmanual && VAC<5)) && !preciseMode && voltageDisplay){
    Vzero = true;
    ClosedOrFloat();
  }else{
    Vzero = false;
    vFloating = false;
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

  /*

  if (!currentOnOff) {
    // skip entirely if user/system doesn’t need current
    return;
  }

  */ 

  if (IHigh) {
    // —— High-RANGE mode: fixed mid-gain for best noise performance ——
    ads.setGain(GAIN_TWOTHIRDS);
    countI = ads.readADC_SingleEnded(3);
    // convert to shunt voltage (mV)
    currentShuntVoltage = (countI * GAIN_FACTOR_TWOTHIRDS / 1000.0f);// ;
    Ireading = (currentShuntVoltage - Izero)/0.185f ; //conversion to Amps
    
    /*
    Serial.print("I Read:"); //this block used for debugging
    Serial.print(countI);
    Serial.print(" I Shunt:");
    Serial.print(currentShuntVoltage);
    Serial.print(" I reading:");
    Serial.println(Ireading);
    */


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
    currentShuntVoltage = (countI * kGainFactors[gainIndexCurrent] / 1000.0f);
    // convert to amps (your high-range calibration)
    Ireading = currentShuntVoltage/1 ; //Value of the sense resistor
  }

  // apply noise floor
  if ((Irange  && isBetween(Ireading, -0.01f,  0.01f)) ||
      (!Irange  && Ireading < 0.0005f)) {
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

    currentSamples[currentSampleIndex] = Ireading;
    currentSampleIndex = (currentSampleIndex + 1) % NUM_CURRENT_SAMPLES;

}


// ========== Display and Alert Functions ========== 

void updateDisplay() {
  // Prepare values for display
  //display.fillScreen(BLACK);
  
  
  //Serial.print("Mic Pin Int:");
  //Serial.println(analogRead(micPin));

    
    /*
    //This block used for Resistance mode debugging
    Serial.print("RangePin:");
    Serial.print(digitalRead(SETRANGE_PIN));
    Serial.print(" V:");
    Serial.print(ohmsVoltage,4);
    Serial.print(" Ohms Int:");
    Serial.print(adcCount);
    Serial.print(" range:");
    Serial.println(gainIndex);
    */
  



  blinkLimit = 0; //Clear Blink Limit on Screen Update
  display.fillRect(0, 0, 256, 72, BLACK);
  display.setTextColor(WHITE,BLACK);

  int gfxLine = 8;

  int mmHomeX = 192;
  int mmHomeY = 0;
  
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

/*
  // Battery voltage / mode indicator
  display.setTextSize(1);
  display.setCursor(72, 56);
  display.print("VIN:");
    display.print(batteryVoltage, 1);
    if (batteryVoltage < 5.1) {
      display.setCursor(72, 48);
    }
*/  
  //Mode Display
  display.fillRect(660, 460-(gfxLine*2), 144, 32, BLACK);
  display.setTextSize(2);
  display.setCursor(660, 460-(gfxLine*2));
  display.print(modeToString(currentMode));

  //Time Display
  display.setCursor(660, 460);
  display.setTextSize(2);
  display.print(formatTime(millis()));

  //Logging Display  
  if(logMode){
    int logHomeX = 660;
    int logHomeY = 350;
    
    display.setTextSize(2);
    display.setCursor(logHomeX, logHomeY);
    display.print("USB?: ");
    display.print(logMode);
    display.setCursor(logHomeX, logHomeY+(gfxLine*2));
    display.print("Auto#: ");
    display.print(autologCount);    
    display.setCursor(logHomeX, logHomeY+(gfxLine*4));
    display.print("Manual#: ");
    display.print(manuallogCount);
    }




  // If notable current present or in ampsMode, overlay current reading on display
  if (currentOnOff) {
    
    int IHomeX = 424;
    int IHomeY = 0;




    display.fillRect(IHomeX, IHomeY, 128, 72, BLACK);    
    display.setTextSize(2);
    display.setCursor(IHomeX, IHomeY);
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
  
    // If any current has been recorded (min/max), display them in small text
    if (IHigh != -6.0) {
    
      display.setTextSize(2);
      display.setCursor(IHomeX, IHomeY+(gfxLine*2));
      display.print("Ilow:");
      display.print(ILow, 3);
      display.print(" t:");
      display.print(timeAtMinI);
      display.setCursor(IHomeX, IHomeY+(gfxLine*4));
      display.print("Ihigh:");
      display.print(IHigh, 3);
      display.print(" t:");
      display.print(timeAtMaxI);
    }
  }

  // Primary measurement display
  
  int PrimeX = 0;
  int PrimeY = 0;
  


  
  display.setTextSize(3);
  display.setCursor(PrimeX, PrimeY);
  if (voltageDisplay) {
    if(Vzero){
      if(vFloating){
            display.print("V FLT:");
          }
       else if(vUndefined){
            display.print("V UNDF:");
        }else if(vClosed){
        display.print("V CLD:");
      }
      display.print(bridgeV*1000,0);
      display.print("m");
    }else{

    
    // Voltage display mode
    if(currentMode == VACmanual){
      display.print("VAC:");
      if(altUnits){
      display.print(VAC, 1);
      }else{
        display.print(VAC, 3);
      }
    }else{
    display.print("VDC:");

      if(preciseMode){
        display.setCursor(PrimeX, (PrimeY+ gfxLine*4));
        display.print(roundedV, (vDigits+1));    
        //display.print(newVoltageReading, (vDigits+1));
        display.println(vSuffix);
      } else {
      display.print(roundedV, vDigits);    
      display.println(vSuffix);
      }
    }
      
      if(deltaV != 0){
      display.setCursor(PrimeX, (PrimeY+ gfxLine*4));
      display.setTextSize(2);
      display.print("ref:");
      display.println(deltaV, deltaVdigits);
      display.setCursor(PrimeX, (PrimeY+ gfxLine*6));
      display.print("delta:");
      display.println(newVoltageReading-deltaV, 4);  
      }

    
    
    }
    
    

    if (MinMaxDisplay) {
      // Show min and max voltage with timestamps

      display.fillRect(mmHomeX, mmHomeY, 232, 72, BLACK);
      display.setTextSize(2);
      display.setCursor(mmHomeX, mmHomeY);
      display.print("Min:");
      display.print(roundedVlow, vDigitslow);
      display.print(vSuffixlow);
      display.print("  t:");
      display.print(timeAtMinV);
      display.setCursor(mmHomeX, (mmHomeY+gfxLine*2));
      
      display.print("Max:");
      display.print(roundedVhigh, vDigitshigh);
      display.print(vSuffixhigh);
      display.print("  t:");
      display.print(timeAtMaxV);
      
      display.setCursor(mmHomeX, (mmHomeY+gfxLine*4));
      display.print("Range:");
      display.print(highV - lowV, 3);
    }
    
    if(currentMode != Low){ 
      if (currentMode == VACmanual) {
        // Show AC component if significant
        display.setCursor(PrimeX, (PrimeY+ gfxLine*6));
        display.setTextSize(2);
        display.print("VDC: ");
        display.print(roundedV, vDigits);
      } else {
        display.setCursor(PrimeX, (PrimeY+ gfxLine*6));
        display.setTextSize(2);
        display.print("VAC: ");
        display.print(VAC, vDigits);
    }


      // Otherwise, show elapsed time or debug info
      
      if (debugMode) {
        display.setCursor(0, 32);
        display.setTextSize(1);
        display.print("Vraw:");
        
      }
    }
    
    
    if(!vFloating){ //Only update the graph when voltage present 
      display.setTextSize(2);
      drawPlot(voltageSamples, SAMPLE_COUNT_PLOT);
      }
  
  } else {
    // Resistance display mode
    
    

    if (ohmsVoltage < (ZENER_MAX_V - 0.007) || currentMode == HighRMode) {

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
        display.setTextSize(2);
        display.setCursor(mmHomeX, mmHomeY);
        display.print("Min:");
        display.print(roundedRlow, rDigitslow);
        display.print(rSuffixlow);
        display.setCursor(mmHomeX, mmHomeY+(gfxLine*2));
        display.print("Max:");
        display.print(roundedRhigh, rDigitshigh);
        display.print(rSuffixhigh);        
      } 
        //Show voltage drop:
        display.setTextSize(2);
        display.setCursor(PrimeX, (PrimeY+ gfxLine*4));
        display.print("mVR:");
        if(ohmsVoltage<1){
        display.print(ohmsVoltage * 1000.0, 1);
        }else{
        display.print(ohmsVoltage * 1000.0, 0);
        
      }
      display.setTextSize(2);
      display.setCursor(PrimeX, (PrimeY+ gfxLine*6));
      if (zeroOffsetRes != 0) {
        
        display.print("null mOhms:");
        display.println(zeroOffsetRes*1000, 1);
        
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
        display.setCursor(PrimeX, (PrimeY+ gfxLine*4));
        display.setTextSize(2);
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
      
      if(currentMode == rPlotMode || currentMode == HighRMode){
      display.setTextSize(2); //Resistance Voltage Plot
      drawPlot(resistanceSamples, SAMPLE_COUNT_PLOT);
      }
  
  }
  }else{
    display.fillRect(0, 0, 500, 480, BLACK);
    formatTime(millis());
    int step = secondsTime % 10;    
    display.setCursor((8+(4*step)), 32);
    display.setTextSize(1);
    display.print(".");
  }
}

void updateAlerts() {
  // Manage continuity buzzer/LED and flashlight mode LED brightness
  unsigned long now = millis();
  if (!flashlightMode) {
    // Continuity check (low resistance)
    continuity = (isBetween(currentResistance, -20.0, 1.0) ||
                      (isBetween(currentResistance, -20.0, 20.0) && ohmsHighRange) || 
                      (Vzero && vClosed) && blinkLimit<2)
                      //|| (prevResistance > 2000000 && currentResistance < 1000000)
                      ;

    bool logicVoltage = ((!altUnits && (fabs(newVoltageReading) > 3.2 || (VACPresense && VAC > 1.0))) ||
                         (altUnits && (fabs(newVoltageReading) > 30.2 || (VACPresense && VAC > 15.0)))&& 
                         blinkLimit<2);
    if (continuity) {
      // If continuity detected, beep/flash the continuity pin
      txFlag = 0; //RPC Comms; Low is continuity
      if (!rFlag) {
        // Start of beep
        analogWrite(CONTINUITY_PIN, 200);
        rgb.on(255, 255, 255); //turn on blue pixel
          /*
          Serial.print("triggered white, bridge:"); //This section used for noise trigger debugging.
          Serial.print(bridgeV,4);
          Serial.print(" avg:");
          Serial.print(averageVoltage,4);
          Serial.print(" new:");
          Serial.println(newVoltageReading,4);
          */
        blinkLimit++;
        rFlag = true;
      } else if ((now % 1000 <= 100 || isBetween(now % 1000, 300, 400)) && rFlag) {
        // Keep beeping in a pattern: on for 100ms, then off, with a double pulse
        analogWrite(CONTINUITY_PIN, 50);
          //Serial.print("triggered blue:"); //noise trigger debugging.
          //Serial.println(bridgeV);
        rgb.on(0, 0, 255); //turn on blue pixel
      } else {
        analogWrite(CONTINUITY_PIN, 0);
        rgb.off(); //turn off all pixels
      }
    
    } else if (logicVoltage ) {
      txFlag = 1; //RPC Comms; Low is continuity
      // If significant voltage in resistance mode, warning flash
      if (!vFlag) {
        analogWrite(CONTINUITY_PIN, 200);
        rgb.on(255, 255, 255); //
        blinkLimit++;
        vFlag = true;
      } else if ((now % 1000 <= 100) && vFlag && (!VACPresense)) {
        analogWrite(CONTINUITY_PIN, 50);
        rgb.on(255, 0, 0); //      
      } else {
        analogWrite(CONTINUITY_PIN, 0);
        rgb.off(); //turn off all pixels
      }
    } else {
      // No alert condition
      analogWrite(CONTINUITY_PIN, 0);
      rgb.off(); //turn off all pixels
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
  secondsTime = totalSeconds % 60;
  char buf[6];
  snprintf(buf, sizeof(buf), "%02u:%02u", minutes, secondsTime);
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
  if ((buttonState == LOW || redPress || isBetween(analogRead(micPin),300,450)) && !buttonPreviouslyPressed && (currentTime - lastDebounceTime+100 > DEBOUNCE_DELAY)) {
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

const char* modeToString(Mode m) {
  switch (m) {
    case Default:   return "Default";
    case Voltmeter:  return "Voltmeter";
    case VACmanual: return "VAC";
    case Type:       return "Type";
    case Low:       return "Low";
    case AltUnitsMode:       return "AltUnits";
    case HighRMode:       return "High R";
    case rPlotMode:       return "rPlotMode";
    case Charging:       return "Charging";
    default:        return "Unknown";
  }
}

void drawPlot(const float data[], int n) {
  // 1) Compute min, max, mean
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
  
if(!MinMaxDisplay){
  if(maxY - minY <0.05 && voltageDisplay && currentMode == Low){
    rangeY = 0.1;

  }else if(maxY - minY <0.2 && voltageDisplay && currentMode != Low){
    rangeY = 0.2;

  }else{
    rangeY = maxY - minY;
  }
}else{
minY = lowV;
maxY = highV;
rangeY = maxY - minY;
}
  
  //float rangeY = maxY - minY;

  // 2) Add 10% total margin (5% top, 5% bottom)
  float margin = rangeY * 0.05f;
  float plotMin = minY - margin * 0.5f;
  float plotMax = maxY + margin * 0.5f;
  float yScale  = float(PLOT_SIZE) / (plotMax - plotMin);

  // 3) Clear plot area
  display.fillRect(PLOT_X, PLOT_Y, PLOT_SIZE+100, PLOT_SIZE, COLOR_BG);

  // 4) Draw data polyline
  for (int i = 0; i < n - 1; i++) {
    int x1 = PLOT_X + (i    * PLOT_SIZE) / (n - 1);
    int y1 = PLOT_Y + PLOT_SIZE - int((data[i]     - plotMin) * yScale);
    int x2 = PLOT_X + ((i+1)* PLOT_SIZE) / (n - 1);
    int y2 = PLOT_Y + PLOT_SIZE - int((data[i+1]   - plotMin) * yScale);
    display.drawLine(x1, y1, x2, y2, COLOR_DATA);
  }

  // 5) Draw dashed lines and labels for min, mean, max
  drawStatLine(minY, plotMin, yScale, COLOR_MIN);
  drawStatLine(meanY, plotMin, yScale, COLOR_MEAN);
  drawStatLine(maxY, plotMin, yScale, COLOR_MAX);
  
}

// Helper: draw one dashed horizontal line and its value
void drawStatLine(float value, float plotMin, float yScale, uint16_t color) {
  int y = PLOT_Y + PLOT_SIZE - int((value - plotMin) * yScale);
  const int dashLen = 6;
  for (int x = PLOT_X; x < PLOT_X + PLOT_SIZE; x += dashLen) {
    display.drawFastHLine(x, y, dashLen / 2, color);
  }
  // Label the line
  char buf[16];
  display.setCursor(PLOT_X + PLOT_SIZE + 5, y - 4);
  display.setTextColor(color);
  display.print(value, 3);   // prints `value` with 2 decimal places
  //display.print(buf);
}

void drawTwoPlots(const float data[], const float data2[],  int n) {
  // 1) Compute min, max, mean
  float minY = data[0], maxY = data[0], sumY = data[0];
  for (int i = 1; i < n; i++) {
    float v = data[i];
    sumY += v;
    if (v < minY) minY = v;
    if (v > maxY) maxY = v;
  }
  float meanY = sumY / n;
  float rangeY = 1;
  

  if(maxY - minY <0.05 && voltageDisplay && currentMode == Low){
    rangeY = 0.1;

  }else if(maxY - minY <0.2 && voltageDisplay && currentMode != Low){
    rangeY = 0.4;

  }else{
    rangeY = maxY - minY;
  }
  
  //float rangeY = maxY - minY;

  // 2) Add 10% total margin (5% top, 5% bottom)
  float margin = rangeY * 0.10f;
  float plotMin = minY - margin * 0.5f;
  float plotMax = maxY + margin * 0.5f;
  float yScale  = float(PLOT_SIZE) / (plotMax - plotMin);

  // 3) Clear plot area
  display.fillRect(PLOT_X, PLOT_Y, PLOT_SIZE+100, PLOT_SIZE, COLOR_BG);

  // 4) Draw data polyline
  for (int i = 0; i < n - 1; i++) {
    int x1 = PLOT_X + (i    * PLOT_SIZE) / (n - 1);
    int y1 = PLOT_Y + PLOT_SIZE - int((data[i]     - plotMin) * yScale);
    int x2 = PLOT_X + ((i+1)* PLOT_SIZE) / (n - 1);
    int y2 = PLOT_Y + PLOT_SIZE - int((data[i+1]   - plotMin) * yScale);
    display.drawLine(x1, y1, x2, y2, COLOR_DATA);
  }

  // 5) Draw dashed lines and labels for min, mean, max
  drawStatLine(minY, plotMin, yScale, COLOR_MIN);
  drawStatLine(meanY, plotMin, yScale, COLOR_MEAN);
  drawStatLine(maxY, plotMin, yScale, COLOR_MAX);
}

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
  fprintf(f,"DataLog End Time: ");
  fprintf(f, "%.2f\n", tLogEnd);
  printArrayCSV(f,
      loggedVoltagesAtI,
      sizeof(loggedVoltagesAtI) / sizeof(loggedVoltagesAtI[0]),
      "%.3f");        // two decimal places
  if(currentOnOff){
    printArrayCSV(f,
        loggedCurrents,
        sizeof(loggedCurrents) / sizeof(loggedCurrents[0]),
        "%.3f");        // two decimal places  
  }
  printArrayCSV(f,
      loggedTimeStamps,
      sizeof(loggedTimeStamps) / sizeof(loggedTimeStamps[0]),
      "%.3f");        // two decimal places  
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
  fprintf(f,"DataLogged End Time: ");
  fprintf(f, "%.2f\n", tLogEnd);
  printArrayCSV(f,
      VAutoArray,
      sizeof(VAutoArray) / sizeof(VAutoArray[0]),
      "%.3f");        // two decimal places
  if(currentOnOff){
    printArrayCSV(f,
        IAutoArray,
        sizeof(IAutoArray) / sizeof(IAutoArray[0]),
        "%.3f");        // two decimal places  
    }
  printArrayCSV(f,
      TAutoArray,
      sizeof(TAutoArray) / sizeof(TAutoArray[0]),
      "%.3f");        // two decimal places  
    fclose(f);
  
}

void ClosedOrFloat()
{
  vClosedflagPrevious = vClosedflag;

  digitalWrite(VbridgePin, HIGH);
  delay(2);

  //ads.setDataRate(RATE_ADS1115_64SPS);
  ads.setGain(GAIN_SIXTEEN);
  bridgeV = (ads.readADC_Differential_0_1() * GAIN_FACTOR_16 / 1000.0)*-1.0;

  //currentShuntVoltage = adcReadingCurrent; //I have no recollection of why this is here. Looks like an accidental 

  //Serial.print("voltageRead:");
  //Serial.println(bridgeV,4);
  vClosedflag = false;
  vFloating = false;
  vUndefined = false;

  if(fabs(bridgeV)<0.03){
    vClosedflag = true;
  }else if(fabs(bridgeV)> 0.05){
    vFloating = true;
  }else{
      vUndefined = true;
    }
  if(vClosedflag && vClosedflagPrevious){
    vClosed = true;
  }else{
    vClosed = false;
  }
  
  digitalWrite(VbridgePin, 0);
  //delay(2);
}

          