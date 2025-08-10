#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <IRremote.h>
#include <Keyboard.h>
#include <analogWave.h> // Include the library for analog waveform generation
#include <EEPROM.h>



// ===== Hardware Setup Constants =====
Adafruit_ADS1115 ads;
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin definitions
const int CONTINUITY_PIN = 6;   // Buzzer or LED for continuity/alerts
const int VbridgePin = 8; // Control Voltage Birdge MOSFET
const int SETRANGE_PIN = 7;   // Controls high/low resistance range
//#define IR_RECEIVE_PIN   8      // IR receiver input pin
const int OHMPWMPIN = 9;
//const int BATT_PIN      = A2;   // Battery voltage analog input
#define enablePin  BAT_READ_EN  // Pin for enabling battery voltage reading
#define BATT_PIN BAT_DET_PIN
const int TYPE_PIN      = 3;    // Mode button (also triggers flashlight mode if held at boot)
const int KBPin = 8;   // Toggle: if LOW -> keyboard mode; if HIGH -> serial mode
const int logPin = 1; // take log pin
//const int LowerButton = 10;   // 
#define MODE_BUTTON 10         // Set your button input pin


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
  Default, //0
  Voltmeter, //1
  VACmanual, //2
  Type, //3
  Low, //4
  AltUnitsMode, //5
  HighRMode, //6
  //Impedance,
  //RelayControl,
  Charging, //6
  NUM_MODES
};

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
int16_t adcCount;
static uint8_t gainIndex;

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

bool continuity = false;

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

//V Float Detect Related
bool vFloating = false;
float bridgeV = 0.0;
bool Vzero = true;

int blinkLimit = 0; //Limit how much the LED can blink between screen updates

// ========== Function Prototypes ========== 
//void handleIRRemote();
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
  Keyboard.begin();
  
  // Configure pins
  pinMode(TYPE_PIN, INPUT_PULLUP);
  pinMode(CONTINUITY_PIN, OUTPUT);
  pinMode(SETRANGE_PIN, OUTPUT);
  pinMode(BATT_PIN, INPUT);
  pinMode(MODE_BUTTON, INPUT_PULLUP); // Assuming active-low button
  pinMode(OHMPWMPIN, OUTPUT);
  pinMode(logPin, INPUT_PULLUP);
  pinMode(VbridgePin, OUTPUT);
  pinMode(enablePin, OUTPUT);  // Set the enable pin as an output
  digitalWrite(enablePin, HIGH); // Set the pin high to enable battery voltage reading


  eepromSetup(); //This reads the EEPROM and sets the analog corrective factors

  

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED init failed"));
    analogWrite(CONTINUITY_PIN, 10); // brief buzz to signal error
    while (1); // halt
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  // Splash screen

  //display.setCursor(0, 0);  
  //display.drawBitmap(0, 0, MICRO_5x7, 5, 7, SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("uMeter #");
  display.print(EEPROM.read(1));//Reads the EEPROM and determines the correct splash   
  display.setCursor(0, 48);
  display.println("XIAO RA4M1");
  
  display.display();
  delay(200);

  // Initialize ADS1115 ADC
  if (!ads.begin()) {
    Serial.println("ADS1115 init failed!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    Serial.println("ADS1115 not found");
    display.println("ADS1115 not found");
    display.display();
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
  
 
  display.clearDisplay();

  Serial.println("Setup End");


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

  if(currentMode == Type || currentMode==HighRMode){
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

  // Handle user inputs
  //handleIRRemote();
  handleButtonInput();

  // Periodic battery voltage reading
  if (currentMillis - previousBattMillis >= BATT_INTERVAL) {
    previousBattMillis = currentMillis;
    int raw = analogRead(BATT_PIN);
    // Example conversion: adjust 5V ADC reading to actual battery voltage
    batteryVoltage = raw * (3.3 / 1023.0)*2; // assuming a divider that scales VIN to <5V
    // (This factor 4.545 is derived from the original code comment "0.0048*4.545")
  }

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
  }

  if(powerSave && fabs(newVoltageReading)<0.1 && !VACPresense && Ireading==0){
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
/*
void handleIRRemote() {
  // Check IR receiver for a decoded command
  if (IrReceiver.decode()) {
    if (millis() - lastIrReceiveMillis >= IR_DEBOUNCE_INTERVAL) {
      lastIrReceiveMillis = millis();
      uint32_t irCode = IrReceiver.decodedIRData.command;
      switch (irCode) {
        case SET_ZERO_CODE:      // Set current resistance as zero reference
          zeroOffsetRes = currentResistance;
          initialZeroSet = true;
          break;
        case CLEAR_ZERO_CODE:    // Clear resistance zero reference
          zeroOffsetRes = 0.0;
          break;
        case SERIAL_SEND_CODE:   // Send one-time readings over Serial
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
        case R_RANGE_CODE:       // Toggle manual resistance range (if auto-range off)
          ohmsHighRange = !ohmsHighRange;
          Serial.print("Resistance Range toggled: ");
          Serial.println(ohmsHighRange ? "High" : "Low");
          break;
        case V_SMOOTH_CODE:      // Toggle voltage smoothing mode (manual mode only)
          smoothVmode = !smoothVmode;
          Serial.println("Voltage smoothing toggled");
          break;
        case TYPE_RESISTANCE_CODE: // Type out resistance via USB keyboard
          Keyboard.print(displayResistance, rDigits);
          break;
        case TYPE_VOLTAGE_CODE:  // Type out voltage via USB keyboard
          Keyboard.print(averageVoltage, vDigits);
          break;
        case KEY_DELETE_CODE:    // Simulate Delete key
          Keyboard.press(0xD4); Keyboard.releaseAll();
          break;
        case KEY_UP_CODE:        // Simulate Up Arrow key
          Keyboard.press(0xDA); Keyboard.releaseAll();
          break;
        case KEY_DOWN_CODE:      // Simulate Down Arrow
          Keyboard.press(0xD9); Keyboard.releaseAll();
          break;
        case KEY_LEFT_CODE:      // Simulate Left Arrow
          Keyboard.press(0xD8); Keyboard.releaseAll();
          break;
        case KEY_RIGHT_CODE:     // Simulate Right Arrow
          Keyboard.press(0xD7); Keyboard.releaseAll();
          break;
        case TYPE_MOVE_CODE:     // Type resistance, then press Right Arrow (to move to next cell, etc.)
          Keyboard.print(displayResistance, rDigits);
          Keyboard.press(0xD7); Keyboard.releaseAll();
          break;
        case CLEAR_MINMAX_CODE:  // Toggle displaying min/max on screen
          MinMaxDisplay = !MinMaxDisplay;
          break;
        case RESET_MINMAX_CODE:  // Reset min/max values and timers
          ReZero();
          break;
        case AUTO_RANGE_CODE:    // Toggle auto-ranging for resistance
          ohmsAutoRange = !ohmsAutoRange;
          Serial.print("Auto-Range: ");
          Serial.println(ohmsAutoRange ? "On" : "Off");
          break;
        case CURRENT_MODE_CODE:  // Toggle current-focused mode
          ampsMode = !ampsMode;
          Serial.print("Current Mode: ");
          Serial.println(ampsMode ? "On" : "Off");
          break;
        default:
          // Unrecognized IR code (could print for debugging)
          break;
      }
    }
    IrReceiver.resume(); // ready to receive next IR signal
  }
}
*/

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
      if(currentMode==Type || currentMode==HighRMode){      
      if (voltageDisplay) {
        Keyboard.print(newVoltageReading, vDigits);
      } else {
        if(displayResistance<1){
        Keyboard.print(displayResistance, rDigits+2);
        }else{
        Keyboard.print(displayResistance, rDigits);  
        }
      }
      // Press Right Arrow after typing (to move cursor, e.g., to next cell)
      Keyboard.press(0xD7);
        Keyboard.releaseAll();
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
static const int ADC_COUNT_HIGH_THRESHOLD = 26000;

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

    // --- Range control (auto vs. manual) ---
    if (ohmsAutoRange && !powerSave) {
      if (!currentRangeHigh && prevResistance > OHMS_HIGH_THRESHOLD) {
        currentRangeHigh = true;
        digitalWrite(SETRANGE_PIN, HIGH);
      } else if (currentRangeHigh && prevResistance < OHMS_LOW_THRESHOLD) {
        currentRangeHigh = false;
        digitalWrite(SETRANGE_PIN, LOW);
      }
    } 
      else if(powerSave){
      currentRangeHigh = true;
      digitalWrite(SETRANGE_PIN, LOW);  
    } else {
      // Manual mode: force based on user setting
      if (ohmsHighRange && !currentRangeHigh) {
        currentRangeHigh = true;
        digitalWrite(SETRANGE_PIN, LOW);
      } else if (!ohmsHighRange && currentRangeHigh) {
        currentRangeHigh = false;
        digitalWrite(SETRANGE_PIN, HIGH);
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

  if(EEPROM.read(1) == 5){
    if(((fabs(averageVoltage) < 0.030 && currentMode != VACmanual && newVoltageReading<0.05) || (currentMode == VACmanual && VAC<5)) && !preciseMode && voltageDisplay){
      Vzero = true;
      ClosedOrFloat();
    }else{
      Vzero = false;
      vFloating = false;
    } 
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
  display.clearDisplay();
  blinkLimit = 0;
  if(!screenSleep && currentMode!=Charging){

    //Serial.print("V Bridge:");
    //Serial.println(bridgeV);

    Serial.print("RangePin:");
    Serial.print(digitalRead(SETRANGE_PIN));
    Serial.print(" V:");
    Serial.print(ohmsVoltage,4);
    Serial.print(" Ohms Int:");
    Serial.print(adcCount);
    Serial.print(" range:");
    Serial.println(gainIndex);

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
     if(Vzero){
      if(vFloating && bridgeV<-0.13){
            display.print("Vflt:");
            display.print(bridgeV*1000,0);
          }
       else if(vFloating && bridgeV>-0.13){
            display.print("V ?:");
            display.print(bridgeV*1000,0);
          }else{
        display.print("CLSD:");
        display.print(bridgeV*1000,0);
      }
      display.print("m");
    }else{
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
  display.display(); // update the OLED with all the drawn content
}

void updateAlerts() {
  // Manage continuity buzzer/LED and flashlight mode LED brightness
  unsigned long now = millis();
  if (!flashlightMode) {
    // Continuity check (low resistance)
      continuity = (isBetween(currentResistance, -20.0, 1.0) ||
                      (isBetween(currentResistance, -20.0, 20.0) && ohmsHighRange) || 
                      (Vzero && !vFloating) && blinkLimit<2)
                      //|| (prevResistance > 2000000 && currentResistance < 1000000)
                      ;

    bool logicVoltage = ((!altUnits && (fabs(newVoltageReading) > 3.2 || (VACPresense && VAC > 1.0))) ||
                         (altUnits && (fabs(newVoltageReading) > 30.2 || (VACPresense && VAC > 15.0)))&& 
                         blinkLimit<2);
    if (continuity) {
      // If continuity detected, beep/flash the continuity pin
      if (!rFlag) {
        // Start of beep
        analogWrite(CONTINUITY_PIN, 200);
        blinkLimit++;
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
        blinkLimit++;
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
  bool buttonState = digitalRead(MODE_BUTTON);
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

void ClosedOrFloat()
{
  digitalWrite(VbridgePin, HIGH);

  ads.setGain(GAIN_EIGHT);
  //ads.setDataRate(RATE_ADS1115_64SPS);
    //delay(1);
  bridgeV = (ads.readADC_Differential_0_1() * GAIN_FACTOR_8 / 1000.0) * -1.0;

  currentShuntVoltage = adcReadingCurrent ;

  //Serial.print("voltageRead:");
  //Serial.println(bridgeV);
  
  if(bridgeV < -0.080){
      vFloating = true;
    }else{
      vFloating = false;
    }
  digitalWrite(VbridgePin, LOW);
  //delay(2);


}

          