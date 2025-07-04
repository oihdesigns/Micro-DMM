#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include "Adafruit_ILI9341.h"
//#include <FlickerFreePrint.h>
#include <SPI.h>
#include <IRremote.h>
#include <Keyboard.h>
#include <math.h>
#include <SD.h>
#include "RTC.h"

//#include <analogWave.h> // Include the library for analog waveform generation

// Big Screen Related
#define TFT_CS        10
#define TFT_DC        8
#define TFT_RST   9
const int chipSelect = 4;  // Or whatever your display’s SD CS pin is




// create the display object
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);


// ADS Gain Factors
float gain16 = 0.0078125;
float gain8 = 0.015625;
float gain4 = 0.03125;
float gain2 = 0.0625;
float gain1 =  0.125;
float gain23 = 0.1875;
float gain = 0;


// Initialize ADS1115
Adafruit_ADS1115 ads;

//analogWave wave(DAC);   // Create an instance of the analogWave class, using the DAC pin

/*
// Initialize OLED display (SH1106 with I2C)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1        // Reset pin (if used, set to -1 if not)
#define SCREEN_ADDRESS 0x3C  // Often 0x3C, could also be 0x3D 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
*/

// Timing Variables
int seconds = 0;
int minutes = 0;
unsigned long previousAdcMillis = 0;
unsigned long previousLcdMillis = 0;
unsigned long ADC_INTERVAL = 1;    // Interval to read ADC
unsigned long LCD_INTERVAL = 1000;   // Interval to update LCD
unsigned long lastIrReceiveMillis = 0;
const unsigned long IR_DEBOUNCE_INTERVAL = 300; // Debounce time in milliseconds
unsigned long BATT_INTERVAL = 1000;
unsigned long previousBattMillis = 0;


// Button Long / Short Press CodeAUDIO_PIN
unsigned long lastPressTime = 0; // Stores the time of the last button press
unsigned long holdStartTime = 0; // Tracks when a hold starts
unsigned long debounceDelay = 50; // Debounce delay in milliseconds
unsigned long holdThreshold = 1000; // Time threshold for a hold in milliseconds
int pressCount = 0; // Tracks the number of consecutive presses
bool buttonState = LOW;
bool lastButtonState = LOW;
bool holdTriggered = false;



// Pin Definitions
const int CONTINUITY_PIN = 6;
const int SETRANGE_PIN = 7;
#define IR_RECEIVE_PIN 5
const int BATT_PIN = A2;
const int TYPE_PIN = 3;
const int RED_PIN = 2;
//const int AUDIO_PIN = A0;


bool serialMode = 0;
unsigned long serialUpdate = 200;
unsigned long previousSerialMillis = 0;


//PWM Related
int pwmValue = 20;     // Initial PWM value
int fadeAmount = 5;   // Amount to increment/decrement PWM by
unsigned long previousMillis = 0; // Tracks the time since last update
const long interval = 10;  // Interval for updating PWM (milliseconds)

/*
// Analog Values For PCB1m1
float vFactor = 68.38;
float constantI = 0.015257;
float constantR = 239.7; 
float dividerR = 22088;
float ZenerMax = 4.165; 

// Analog Values For PCB1m2 (no V+ unit)

float constantI = 0.02024;
float constantR = 239.88;
float dividerR = 22007;
float ZenerMax = 2.185;
float vFactor = 68.38;
*/

float constantI = 0.02016;
float constantR = 240.0;
float dividerR = 22000.0;
float ZenerMax = 4.353;
float vFactor = 63.539;

// Measurement Variables
float minR = 0.0 ;
float adjustedRx = 0.0;
float resultRx = 0;
float correctedRx = 0.0;
float Ohms_v_output = 0;
float batt_adjust = 0.0;
float averageVoltage = 0;
float VAC = 0;
float vSquared = 0;
//float rSquared = 0;
int32_t Ohms_ADS_Int = 0;
int32_t voltage_int;
float battery = 0.0;
int16_t batteryPercent = 0;
int16_t writeCount = 0;

float highR = 0.0; //Resistance Formatting and Min Max Variables
float lowR = 3000000.0;
float roundedR;
float roundedRlow = 0.0;
float roundedRhigh = 0.0;
String rSuffix;
String rSuffixlow = "";
String rSuffixhigh = "";
int rDigits = 3;
int rDigitslow = 0;
int rDigitshigh = 0;
float printedR;



float highV = -100.0; //Voltage Formatting and Min Max Variables
float lowV = 100.0;
float roundedV = 0.0;
float roundedVlow = 0.0;
float roundedVhigh = 0.0;
String vSuffix;
String vSuffixlow = "";
String vSuffixhigh = "";
int vDigits = 0;
int vDigitslow = 0;
int vDigitshigh = 0;
float printedV;
float VMedian = 0.0;
float VMedianStep = 0.05;
String VtHigh;
String VtLow;
float IatVHigh = 0;
float IatVLow = 0;
float ILowRolling = 0;
float lastLoggedI = 0;
bool Irange = 0; //0 is low range
float mAcorrection;
//float logTime;
float Istep = 0.1;


int32_t ch3int = 0;
float ch3voltage = 0.0;
float Ireading = 0.0;
float Izero = 0.0;
float IHigh = -6.0;
float ILow = 6.0;
float previousI = 0;
int16_t HighCountI = 0;
float IMedian = 0.0;
float IMedianStep = 0.02;
String ItHigh;
float ItHighm;
String ItLow;
float VatIHigh = 0;
float VatILow = 0;
//float VatRlow = 0;
//float VtHighm = 0;
//float RatRlow =0;

//Logging Functions

#define NUM_VALUES 100

// Declare arrays to store previous values
float VAutoArray[NUM_VALUES] = {0};
float TAutoArray[NUM_VALUES] = {0};
float IAutoArray[NUM_VALUES] = {0};
//float logTime[NUM_VALUES] = {0};
float IManArray[NUM_VALUES] = {0};
float VManArray[NUM_VALUES] = {0};
float TManArray[NUM_VALUES] = {0};


// Function to shift and store float values
void shiftAndStore(float* array, float newValue) {
    for (int i = NUM_VALUES - 1; i > 0; i--) {
        array[i] = array[i - 1];
    }
    array[0] = newValue;
}

void logValues(float VatIHigh, float ItHighm, float IHigh) {
    shiftAndStore(VAutoArray, VatIHigh);
    shiftAndStore(TAutoArray, ItHighm);
    shiftAndStore(IAutoArray, IHigh);
}

void logValuesManual(float VatIHigh, float ItHighm, float IHigh) {
    shiftAndStore(VManArray, VatIHigh);
    shiftAndStore(TManArray, ItHighm);
    shiftAndStore(IManArray, IHigh);
}


void printArray(Print &stream, float arr[], int size, int decimals) {
  for (int i = 0; i < size; i++) {
    stream.print(arr[i], decimals);
    if (i < size - 1) {
      stream.print(", ");
    }
  }
  stream.println();
}


// Calculation Functions
 float ResConVf () {
  return (dividerR * (Ohms_v_output / (ZenerMax - Ohms_v_output)));
  };

float ResConIf () {
  return (Ohms_v_output / (constantI - (Ohms_v_output / constantR)));
  };

float ResADCtoV () {
  return (Ohms_ADS_Int * gain / 1000)-batt_adjust;
}

bool isBetween(float value, float lower, float upper) {
    return (value >= lower) && (value <= upper);
}

void formatResistance(float printedR, float &roundedR, String &rSuffix, int &rDigits) {
  if (printedR > 500000) {
    roundedR = printedR / 1000000;
    rSuffix = "M";
    rDigits = 3;
  } else if (printedR > 900) {
    roundedR = printedR / 1000;
    rSuffix = "k";
    rDigits = (printedR > 90000) ? 2 : 3;
  } else {
    if (printedR < 0.9) {
      roundedR = printedR * 1000;
      rSuffix = "m";
      rDigits = 1;
    } else {
      roundedR = printedR;
      rSuffix = "";
      if (printedR > 100) {
        rDigits = 1;
      } else if (printedR < 9.5) {
        rDigits = 4;
      } else {
        rDigits = 3;
      }
    }
  }
}

void formatVoltage(float printedV, float &roundedV, String &vSuffix, int &vDigits) {
  if (abs(printedV) < 0.9){
    roundedV = printedV*1000;
    vDigits = 1;
    vSuffix = "m";
  } else {
  if (abs(printedV) > 10) {   
    vDigits = 2;              
  } else {
    vDigits = 3;
    }
  roundedV = printedV;
  vSuffix = "";
  }
}

String getVMinMax(float roundedVlow, float vDigitslow, String vSuffixlow, float roundedVhigh, float vDigitshigh, String vSuffixhigh) {
    String VMinMax = "Min:";
    VMinMax += String(roundedVlow, vDigitslow);
    VMinMax += vSuffixlow;
    VMinMax += " Max:";
    VMinMax += String(highV, vDigitshigh);
    VMinMax += vSuffixhigh;
    VMinMax += " Range:";
    VMinMax += String(highV - lowV, 3);
    return VMinMax;
}

String getRMinMax(float roundedRlow, float rDigitslow, String rSuffixlow, float roundedRhigh, float rDigitshigh, String rSuffixhigh) {
    String RMinMax = "Min:";
    RMinMax += String(roundedRlow,rDigitslow);
    RMinMax += rSuffixlow;
    RMinMax += " Max:";
    RMinMax += String(roundedRhigh,rDigitshigh);
    RMinMax += rSuffixhigh;
    return RMinMax;
}

String formatTime(unsigned long milliseconds) {
    // Calculate seconds from the provided milliseconds
    unsigned long totalSeconds = milliseconds / 1000;

    // Calculate minutes and remaining seconds
    unsigned int minutes = totalSeconds / 60;
    unsigned int seconds = totalSeconds % 60;

    // Format the string as "MM:SS"
    char timeString[6];
    snprintf(timeString, sizeof(timeString), "%02u:%02u", minutes, seconds);

    // Return the formatted string
    return String(timeString);
}

String formatTimeMillis(unsigned long milliseconds) {
    // Calculate total seconds, minutes, and remaining milliseconds
    unsigned long totalSeconds = milliseconds / 1000;
    unsigned int minutes = totalSeconds / 60;
    unsigned int seconds = totalSeconds % 60;
    unsigned int millisRemaining = milliseconds % 1000;

    // Format the string as "MM:SS:mmm"
    char timeString[12];
    snprintf(timeString, sizeof(timeString), "%02u:%02u:%03u", minutes, seconds, millisRemaining);

    // Return the formatted string
    return String(timeString);
}


void ReZero(){
      highR = 0;
      lowR = 3000000;
      highV = -100;
      lowV = 100;
      VMedian = 0;
      IHigh = -6;
      ILow = 6;
      HighCountI = 0;
      IMedian = 0;
}


/*
void displayVminmax(){
  tft.print("Min:");
    tft.print(roundedVlow,vDigitslow);
    tft.print(vSuffixlow);
    tft.print("  t:");
    tft.print(VtLow);

  // tft.print(" Md:");
  // tft.print(VMedian,2);
    tft.println("");//LineBreakTest
  // tft.setCursor(0, 24);
    tft.print("Max:");
    tft.print(roundedVhigh,vDigitshigh);
    tft.print(vSuffixhigh);
    tft.print("  t:");
    tft.print(VtHigh);
  // tft.print(" N:");
  // tft.print(HighCountV);
    tft.println("");//LineBreakTest
  //tft.setCursor(0, 32);
    tft.print("Range:");
    tft.print(highV-lowV,3); 
}

*/

void displayRminMax(){
  tft.setTextSize(1); //Auto'd
 // tft.print("Min:");
  tft.print(roundedRlow,rDigitslow);
  tft.print(rSuffixlow);  
  tft.println("  "); //Line Break       
 // tft.print("Max:");
  tft.setCursor(48,88);
  tft.print(roundedRhigh,rDigitshigh);
  tft.print(rSuffixhigh);
  tft.print("  ");
}

// Measurement Value Averaging Variables
const int NUM_READINGS_V = 25;
int32_t voltReadingsInt[NUM_READINGS_V] = {0};
float voltReadingsFloat[NUM_READINGS_V] = {0.0};
float vacReadingsFloat[NUM_READINGS_V] = {0.0};
int readIndexV = 0;
int32_t totalADC = 0;
float totalVoltage = 0.0;
float totalSquaredVoltage = 0.0;
float resistanceRMS = 0; 
float previousV = 0.0;


// Average Ohm Reading Variables
const int NUM_READINGS_R = 50;
int32_t adcOhmReadings[NUM_READINGS_R] = {0};
float resistanceReadingsFloat[NUM_READINGS_R] = {0.0};
float rReadingsFloat[NUM_READINGS_R] = {0.0};
int readOhmIndex = 0;
int32_t totalOhmADC = 0;
float totalResistance = 0.0;
float totalSquaredResistance = 0.0;
float rStability = 0.0;


// IR Button Codes
#define SET_ZERO_CODE 0x16
#define CLEAR_ZERO_CODE 0x0C
#define SERIAL_SEND_CODE 0x40
#define R_RANGE_CODE 0x47
#define V_SMOOTH_CODE 0x5E
#define TYPE_VOLTAGE_CODE 0x07
#define TYPE_RESISTANCE_CODE 0x09
#define KEY_DELETE 0x45
#define KEY_UP_ARROW 0x46
#define KEY_RIGHT_ARROW 0x43
#define KEY_LEFT_ARROW 0x44
#define KEY_DOWN_ARROW 0x15
#define TYPE_MOVE 0x19
#define AUTO_RANGE 0x1C
#define CLEARMINMAX 0x4A
#define RESETMINMAX 0x52
#define CURRENT_MODE 0x18 //2
//#define 3 0x5E
//#define 4 0x08
//#define 6 0x5A
//#define 7 0x42



// Boolean Values
bool ohmsHighRange = true;
bool ohmsAutoRange = true;
bool ohmsAutoRangePin = true;
bool vflag = 0;
bool rflag = 0;
bool MinMaxDisplay = 0;
bool debug = 0;
bool currentMode = 0;
bool currentOnOff = 1;
bool trackingMode = 1;
bool showTimes = 0;
bool prevDisplay = 1;
bool manualWrite = 0;
bool firstMWrite = 1;
bool AutologTriggered = 0;
bool writePrimed = 0;
bool manualWritePrimed = 0;

bool initalZero = false;
bool SDMode = true;

// Display Mode
bool voltageDisplay = false;
bool screenRefreshfast = false;

// Flashlight Mode
bool flashlightMode = false;
bool flashlightFlag = false;
float flashlightPWM = 0.0;


//voltage refresh
bool smoothVmode = true;
float newVoltageReading;

bool vDisplayManual = false;

//Resistance Refresh
bool smoothRmode = true;
float averageResistance = 0.0;




//definitions from below
  char rSpeed;
  char rRange;
  char vMode;
  char receivedChar;
  unsigned long currentPWMMillis;

/*

float cfnbA = 1;  //Corrections Tables
float cfnbB = 1; 
float cfnbC = 1; 
float cfnbD = 1; 
float cfnbE = 1; 
float cfnbF = 1; 
float cfnbG = 1; 
float cfnbH = 1; 
float cfnbI = 1; 
float cfnbJ = 1; 
float cfnbK = 1; 
float cfnbL = 1; 
float cfnbM = 1; 
float cfnbN = 1; 
float cfnbO = 1; 
*/
float cfnbA = 1;
float cfnbB = 0.9986;
float cfnbC = 1.001;
float cfnbD = 0.9861;
float cfnbE = 0.9636;
float cfnbF = 1;
float cfnbG = 0.9961;
float cfnbH = 1.001;
float cfnbI = 1.0017;
float cfnbJ = 1.0106;
float cfnbK = 1.0067;
float cfnbL = 1.0106;
float cfnbM = 1.0515;
float cfnbN = 1.0537;
float cfnbO = 1.3707;







void setup() {    
    Serial.begin(115200);
    Keyboard.begin();

    RTC.begin();
      RTCTime startTime(10, Month::FEBRUARY, 2025, 8, 00, 00, DayOfWeek::MONDAY, SaveLight::SAVING_TIME_INACTIVE);
      RTC.setTime(startTime);


    SPI.beginTransaction(SPISettings(40000000, LSBFIRST, SPI_MODE0));

    // Set Pins
    pinMode(TYPE_PIN, INPUT_PULLUP); 
    pinMode(CONTINUITY_PIN, OUTPUT);
    pinMode(SETRANGE_PIN, OUTPUT);
    pinMode(BATT_PIN, INPUT);
    pinMode(RED_PIN, INPUT_PULLUP);
    //pinMode(AUDIO_PIN, OUTPUT);

        // Initialize the ADS1115
    if (!ads.begin()) {
        Serial.println("Failed to initialize ADS1115!");
        tft.setCursor(0, 0);
//        tft.clearDisplay();
        tft.println("ADSProblem");
        while (1);
    }
    ads.setDataRate(RATE_ADS1115_860SPS);
    
    delay(50);
    // Initialize IR Receiver
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

/* Small Screen
    // Initialize the SH1106 OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SH1106 allocation failed"));
        analogWrite(CONTINUITY_PIN, 10);
        while (1);
    }
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

 */ 

         SD.begin(chipSelect); // Attempt to start SD card
         delay(50);         
         if (!SD.begin(chipSelect)) {
          Serial.println("SD card initialization failed!");
          SDMode = 0;
          }else {
            Serial.println("SD card initialized.");
            SDMode = 1;
           // MinMaxDisplay = 1;          
          } //SD card initialization and Print



   tft.begin();

  pinMode(TFT_CS,   OUTPUT);
  pinMode(TFT_DC,   OUTPUT);
  tft.setRotation(4);
 

    // Display initial message
      tft.fillRect(0, 0, 240, 35, ILI9341_RED);
    tft.setCursor(0, 0);
    tft.setTextSize(1); //Auto'd
    tft.println("ClearMeter");
    tft.println("PCB2 #4");    
//    tft.display();
    delay(500);



      tft.fillRect(0, 0, 240, 320, ILI9341_BLACK); //Fill Screen as black and place other content on top
      tft.setCursor(150,300);
      tft.setTextSize(1); //Auto'd
      tft.print("VIN:");

      tft.setCursor(0,280);
      tft.setTextSize(1); //Auto'd
      if(SDMode == 0){ 
        tft.print("NO SD");            
        }else{
          tft.print("SD OK");
          tft.setCursor(96, 280);
          tft.setTextSize(1);
          tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
          tft.print("W OK:");
        }

      //Initial Resistance Display:

        tft.fillRect(0, 0, 240, 170, ILI9341_BLUE);
        tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
        tft.setCursor(0, 0); //First Line
        tft.setTextSize(3);
        tft.print("R:");

        tft.setCursor (0, 56); 
        tft.setTextSize(1); //Auto'd
        tft.print("mVR:");

        tft.setCursor (0, 72); 
        tft.println("Min:");
        tft.println("Max:");
        
        
        tft.setCursor (0, 108); 
        tft.print("Rx0: ");



    //Print Serial Commands
    
    Serial.println("Commands: Q, q, Z, C, R, S, V, A, F, M, B, or ? for descriptions");


    // Zero the Current Reading
    ads.setGain(GAIN_TWOTHIRDS);
    gain = gain23;
    ch3int = ads.readADC_SingleEnded(3);
    ch3voltage = (ch3int * gain / 1000);

    if(ch3int<300 || isBetween(ch3voltage, 2.3, 2.7)){

    if(isBetween(ch3voltage, 2.3, 2.7)){
      Irange = 1; //Set High Range
      Izero = ch3voltage;
    }else{
      Irange = 0; //Set High Range
      Izero = 0;
    }

        tft.fillRect(0, 170, 240, 100, ILI9341_PINK);
        //tft.fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
        tft.setTextColor(ILI9341_WHITE, ILI9341_PINK);
        
      tft.setCursor(0,180);
        tft.setTextSize(3);
        if(Irange){
        tft.print("A:");
        }else{
          tft.print("mA:");
        }
        
        
        tft.setTextSize(1); //Auto'd
          
          tft.setCursor(0,204);
            tft.print("IMin:");
          tft.setCursor(160,204);
          tft.print("T:");

          tft.setCursor(0,220);  
            tft.print("IMax:");
           tft.setCursor(160,220);
             tft.print("T:");

          tft.setCursor(0,236);
            tft.print("Imedian:");
                
    } else {
      currentOnOff = 0; //If the reading isn't about 2.5 then turn off the current system
      Ireading=0;
    }
      //  tft.clearDisplay();
}

void loop() {
    unsigned long currentMillis = millis();

  RTCTime currentTime;
  // Get current time from RTC
  RTC.getTime(currentTime);

                if(showTimes){
        Serial.print("Loop Start ");
        Serial.println(millis());
                }

//    if(digitalRead(RED_PIN == LOW)){
//      tft.fillScreen(ILI9341_RED);
//      tft.fillScreen(ILI9341_BLUE);}
  
  if(digitalRead(TYPE_PIN) == LOW && !flashlightFlag) {
    flashlightMode = true;                
} else {
  flashlightFlag = true;
}


    // Handle IR Input
  if (IrReceiver.decode()) {
    if (millis() - lastIrReceiveMillis >= IR_DEBOUNCE_INTERVAL) {
        lastIrReceiveMillis = millis(); // Update last receive time

        uint32_t irCode = IrReceiver.decodedIRData.command;

        switch (irCode) {
            case SET_ZERO_CODE:
                minR = adjustedRx;
                break;

            case CLEAR_ZERO_CODE:
                minR = 0;
                break;

            case SERIAL_SEND_CODE:
                Serial.print("Res: ");
                Serial.print(printedR);
                Serial.print(" Vr: ");
                Serial.print(Ohms_v_output);
                Serial.print(" Vdc: ");
                Serial.println(printedV);
                break;

            case R_RANGE_CODE:
                ohmsHighRange = !ohmsHighRange; // Toggle R_Range state
                break;

            case V_SMOOTH_CODE:
                smoothVmode = !smoothVmode;
                Serial.print("smooth toggled");
               break;

            case TYPE_RESISTANCE_CODE:
                Keyboard.print(adjustedRx, 3);
                break;

            case TYPE_VOLTAGE_CODE:
                Keyboard.print(averageVoltage, 3);
                break;                        
            case KEY_DELETE:
                Keyboard.press(0xD4);
                Keyboard.releaseAll();
                break;
            case KEY_UP_ARROW:
                Keyboard.press(0xDA);
                Keyboard.releaseAll();
                break;
            case KEY_LEFT_ARROW:
                Keyboard.press(0xD8);
                Keyboard.releaseAll();
                break;
            case KEY_RIGHT_ARROW:
                Keyboard.press(0xD7);
                Keyboard.releaseAll();
                break;
            case KEY_DOWN_ARROW:
                Keyboard.press(0xD9);
                Keyboard.releaseAll();
                break;
            case TYPE_MOVE:                
                Keyboard.print(adjustedRx, 3);
                Keyboard.press(0xD7);
                Keyboard.releaseAll();
                break;

            case CLEARMINMAX:    
                MinMaxDisplay = !MinMaxDisplay;
                break;

            case RESETMINMAX:
                ReZero();
                break;

            case AUTO_RANGE:
            ohmsAutoRange=!ohmsAutoRange; //toggle AutoRange Mode
            break;

            case CURRENT_MODE:
            currentMode=!currentMode;
            break;


        }
    }
    IrReceiver.resume();
  }

       
// Battery Voltage
   if (currentMillis - previousBattMillis >= BATT_INTERVAL) {
        previousBattMillis = currentMillis;
        
        battery = (analogRead(A2)*0.0218); // 0.0048*4.545
        batteryPercent= (2.5/(battery-4.5)*100);
   }

/*
   if(battery > 3){
    batt_adjust = -0.003; 
    }
    else{
      batt_adjust = 0.000;
    }
*/


// Read ADC values and Calculate Values

    if (currentMillis - previousAdcMillis >= ADC_INTERVAL) {
        previousAdcMillis = currentMillis;

                if(showTimes){
                       Serial.print("Measure V Start ");
        Serial.println(millis());
                }


        previousV = newVoltageReading;
        
        if (abs(previousV) > 15.5){ //If the previous voltage was above this threshold, start in this range 
        
        ads.setGain(GAIN_TWO);
        gain = gain2;
        voltage_int = ads.readADC_Differential_0_1();
        newVoltageReading = (voltage_int * gain / 1000) * vFactor;

          if(abs(newVoltageReading) <=14) { // If voltage is below this threshold, change the range and reread
            ads.setGain(GAIN_SIXTEEN);
            gain = gain16;
            voltage_int = ads.readADC_Differential_0_1();
            newVoltageReading = (voltage_int * gain / 1000) * vFactor;
          }
          
        } else {
            ads.setGain(GAIN_SIXTEEN);
            gain = gain16;
            voltage_int = ads.readADC_Differential_0_1();
            newVoltageReading = (voltage_int * gain / 1000) * vFactor;
            
            if(abs(newVoltageReading) >=16) { // If voltage is above this threshold, change the range
              ads.setGain(GAIN_TWO);
              gain = gain2;
              voltage_int = ads.readADC_Differential_0_1();
              newVoltageReading = (voltage_int * gain / 1000) * vFactor;
          }
        }
          

//          if(!currentMode){
          if (newVoltageReading > highV){            
            highV = newVoltageReading;
            VtHigh = formatTime(millis());
            LCD_INTERVAL=200;
            IatVHigh = Ireading;

            if (MinMaxDisplay && voltageDisplay){
            analogWrite(CONTINUITY_PIN, 200);                        
            }
          }
          if (newVoltageReading < lowV){
            VtLow = formatTime(millis());            
            lowV = newVoltageReading;
            IatVLow = Ireading;
            LCD_INTERVAL=200;
          }
          

/*
          if(trackingMode){
            float TimeS;
            TimeS = (millis());
            TimeS = TimeS/1000;          
          ItHighm = TimeS;              
              logValues(newVoltageReading, TimeS, Ireading);
          }
  */        
         
      
        VMedianStep = (newVoltageReading-VMedian)*0.2; //
        VMedian += VMedianStep;
        if(abs(VMedianStep)>0.3){
          LCD_INTERVAL=200;
        }


        // Update rolling average for voltage

        totalVoltage -= voltReadingsFloat[readIndexV];        
        voltReadingsFloat[readIndexV] = newVoltageReading;        
        totalVoltage += newVoltageReading;
        readIndexV = (readIndexV + 1) % NUM_READINGS_V;        
        averageVoltage = totalVoltage / NUM_READINGS_V;
        
        
        vSquared = (newVoltageReading-averageVoltage)*(newVoltageReading-averageVoltage);
        totalSquaredVoltage -= vacReadingsFloat[readIndexV];
        vacReadingsFloat[readIndexV] = vSquared;
        totalSquaredVoltage += vSquared;
        VAC = sqrt(totalSquaredVoltage/NUM_READINGS_V);

        // Measure Current

        if(currentOnOff || currentMode){ //Only Measure Current if on
              
                      if(showTimes){
                      Serial.print("Measure I Start ");
              Serial.println(millis());
                      }

              previousI = Ireading;

              if(Irange){ //If we're in high range use this math
                ads.setGain(GAIN_TWOTHIRDS);
                gain = gain23;
                ch3int = ads.readADC_SingleEnded(3);
                ch3voltage = (ch3int * gain / 1000)-Izero;
                Ireading = ch3voltage / (0.185);
              }else{
                if(previousI>0.2){
                  ads.setGain(GAIN_ONE);
                  gain = gain1;
                  ch3int = ads.readADC_SingleEnded(3);
                }                
                else if(previousI<0.1){
                  ads.setGain(GAIN_EIGHT);
                  gain = gain8;
                  ch3int = ads.readADC_SingleEnded(3);
                  }else{
                  ads.setGain(GAIN_TWO);
                  gain = gain2;
                  ch3int = ads.readADC_SingleEnded(3);
                  }
                  if(ch3int>100){
                    mAcorrection= 0.0012;
                  }else{
                    mAcorrection= 0;
                  }

                ch3voltage = (ch3int * gain / 1000); //(No IZero term)
                Ireading = (ch3voltage/5)+mAcorrection; //I will need a more exact factor
              }



              if((isBetween(Ireading, -0.01, 0.01)&& Irange) || (Ireading < 0.0001 && !Irange)) {
                Ireading=0;
              } else {
/*              
                       
              if(Ireading>IHigh || Ireading<ILow){
                if(trackingMode){              
                    float TimeS;
                    TimeS = (millis());
                    TimeS = TimeS/1000;          
                    ItHighm = TimeS;              
                    logValues(newVoltageReading, ItHighm, Ireading);
                    logTriggered = 1;                                       
                  }
*/

              if(Ireading>IHigh){              
                IHigh=Ireading;
                VatIHigh = newVoltageReading;
                ItHigh = formatTime(millis()); 
              }           
              
                                          
              if(Ireading<ILow){
                ILow=Ireading;
                VatILow = newVoltageReading;
                ItLow = formatTime(millis());
              }
              }     

            if(Ireading != 0){ //Only update median when current detected
              if(Ireading >IMedian){
                IMedian += IMedianStep;
              } else {
                IMedian -= IMedianStep;
              }
            }
        }//Close Out Current ON/OFF Section


        if((IHigh!=Ireading || highV != newVoltageReading) && (!AutologTriggered && !manualWrite)){ //Only Measure Resistance if neither voltage nor resistance have hit new highs, and we're not in voltage display holding down the red button
        // Measure Resistance
        float previousR = resultRx;
        
        if(showTimes){
        Serial.print("Measure R Start ");
        Serial.println(millis());
        }

        if (ohmsAutoRange) {
          //If pin is already high, read the ADC and calculate R, if below threshold, turn the pin on, read the ADC and recalculate R.
          if(previousR>300000){
            ads.setGain(GAIN_TWOTHIRDS);
            gain = gain23;
            Ohms_ADS_Int = ads.readADC_SingleEnded(2);
            Ohms_v_output = ResADCtoV();
            resultRx = ResConVf();
          }else{
          
          if(digitalRead(SETRANGE_PIN) == HIGH) {
            ads.setGain(GAIN_ONE);
            gain = gain1;
            Ohms_ADS_Int = ads.readADC_SingleEnded(2);
            Ohms_v_output = ResADCtoV();
            resultRx = ResConVf();            
              
              if (isBetween(resultRx, -5, 42)) {
                digitalWrite(SETRANGE_PIN, LOW);
                if(resultRx <= 12){
                  ads.setGain(GAIN_SIXTEEN);
                  gain = gain16;
                  Ohms_ADS_Int = ads.readADC_SingleEnded(2);
                  Ohms_v_output = ResADCtoV();
                  } else {                
                    ads.setGain(GAIN_TWO);
                    gain = gain2;
                    Ohms_ADS_Int = ads.readADC_SingleEnded(2);
                    Ohms_v_output = ResADCtoV();  
                    }
                  resultRx = ResConIf();
                }
               if (resultRx > 300000){
                ads.setGain(GAIN_TWOTHIRDS);
                gain = gain23;
                Ohms_ADS_Int = ads.readADC_SingleEnded(2);
                Ohms_v_output = ResADCtoV();
                resultRx = ResConVf();
              }
            }
            else
            {
            //If the pin starts low, take the resistance, if the reading is above a threshold, turn the pin off and remeasure
            ads.setGain(GAIN_TWO);
            gain = gain2;
            Ohms_ADS_Int = ads.readADC_SingleEnded(2);
            Ohms_v_output = ResADCtoV();
            resultRx = ResConIf();
              if(resultRx <= 12){
                  ads.setGain(GAIN_SIXTEEN);
                  gain = gain16;
                  Ohms_ADS_Int = ads.readADC_SingleEnded(2);
                  Ohms_v_output = ResADCtoV();
                  }
              resultRx = ResConIf();
               if (resultRx > 47){
                digitalWrite(SETRANGE_PIN, HIGH);
                //ads.setGain(GAIN_ONE);
                //gain = gain1;
                Ohms_ADS_Int = ads.readADC_SingleEnded(2);
                Ohms_v_output = ResADCtoV();
                resultRx = ResConVf();
            } 
          }
        }
        }        
          else // This is if we're not in Auto Range Mode
          {
            if(!ohmsHighRange){ // Low Range Measurement
              ads.setGain(GAIN_TWO);
              gain = gain2;
              Ohms_ADS_Int = ads.readADC_SingleEnded(2);
              Ohms_v_output = ResADCtoV();            
              resultRx = ResConIf();
            } else { //Normal Range Measurement
                ads.setGain(GAIN_ONE);
                gain = gain1;
                Ohms_ADS_Int = ads.readADC_SingleEnded(2);
              if (Ohms_ADS_Int >= 100) {
                Ohms_v_output = ResADCtoV();  
                resultRx = ResConIf();
              } else {              
                ads.setGain(GAIN_TWO);
                gain = gain2;
                Ohms_ADS_Int = ads.readADC_SingleEnded(2);
                Ohms_v_output = ResADCtoV();  
                resultRx = ResConIf();              
              }
            }
          }

          if(!isBetween(resultRx, previousR*0.5, previousR*1.5)){LCD_INTERVAL = 200;}



          //Correction Factors
          if(resultRx<0.75){correctedRx = resultRx*cfnbA;} //0.5
          else if (resultRx<3){correctedRx = resultRx*cfnbB;} //1
          else if (resultRx<7){correctedRx = resultRx*cfnbC;} // 4.7
          else if (resultRx<20){correctedRx = resultRx*cfnbD;} // 10
          else if (resultRx<70){correctedRx = resultRx*cfnbE;} //33
          else if (resultRx<170){correctedRx = resultRx*cfnbF;} //100
          else if (resultRx<700){correctedRx = resultRx*cfnbG;} //330                   
          else if (resultRx<1700){correctedRx = resultRx*cfnbH;}
          else if (resultRx<7000){correctedRx = resultRx*cfnbI;}
          else if (resultRx<17000){correctedRx = resultRx*cfnbJ;}
          else if (resultRx<70000){correctedRx = resultRx*cfnbK;}
          else if (resultRx<170000){correctedRx = resultRx*cfnbL;}
          else if (resultRx<700000){correctedRx = resultRx*cfnbM;}
          else if (resultRx<1700000){correctedRx = resultRx*cfnbM;}
          else {correctedRx = resultRx*cfnbO;}

          /*
        //Resistance Smoothing        
          resistanceReadingsFloat[readOhmIndex] = adjustedRx;
          totalResistance += adjustedRx;
          readOhmIndex++;
          if (readOhmIndex == NUM_READINGS_R) {
          // Calculate the average resistance
            averageResistance = totalResistance / NUM_READINGS_R;
                readOhmIndex = 0;
                totalResistance = 0.0;
            }

        */

        // If too close to Zener max, set R to limit
        if (Ohms_v_output > (ZenerMax-0.01)) {
        adjustedRx = 8000000;
        } else {
        adjustedRx = correctedRx ;
        }
        }

        // Auto Zero Code
        if(((adjustedRx < 10) && (adjustedRx > 0.01)) && (!initalZero)){
          minR = adjustedRx;
          initalZero = true;
          Serial.print("Zero Offset Autoset:");
          Serial.println(minR);        
        }
        if(((adjustedRx > 10) && (adjustedRx < 1000000)) && (!initalZero)){
          initalZero = true;
          Serial.println("No Auto Zero Offset");     
        }

        /*        
        // Related to smoothing out the resistance reading
        totalResistance -= resistanceReadingsFloat[readOhmIndex];
        resistanceReadingsFloat[readOhmIndex ] = adjustedRx;
        totalResistance += adjustedRx;
         
         if ((totalResistance / NUM_READINGS_R)<0)
          {averageResistance=0;}
          else
          {
            averageResistance=(totalResistance / NUM_READINGS_R);
          }
        rStability = (adjustedRx/((adjustedRx+averageResistance)/2));        
        readOhmIndex = (readOhmIndex + 1) % NUM_READINGS_R;
        */
        }

  // Continuity Check
    currentPWMMillis = millis(); // Get the current time
    if(!flashlightMode){
                     if(showTimes){
             Serial.print("continuity check start ");
        Serial.println(millis());
                     }
    
    if ((isBetween(resultRx, -20, 1))  
        || 
        ((isBetween(resultRx, -20, 20)) && (ohmsHighRange))) 
        {
        
        //  wave.square(800);       // Generate a sine wave with the initial frequency

          
        /*
        if (currentPWMMillis - previousMillis >= interval) {
            previousMillis = currentPWMMillis; // Save the current time for next update
            // Update PWM value
            pwmValue += fadeAmount;
            // Reverse the direction of fading at the ends of the range
            if (pwmValue <= 10 || pwmValue >= 40) {
              fadeAmount = -fadeAmount; // Change the direction
            } */
            // Write the new PWM value to the pin
        
        
        if (!rflag)
        {analogWrite(CONTINUITY_PIN, 200); //Flash LED and then set rFlag True
        rflag = true;}
      
      else if (((currentPWMMillis % 1000 <= 100) || isBetween(currentPWMMillis % 1000, 300, 400))  && (rflag) ){ //LED on 10% 1s duty cycle
        {analogWrite(CONTINUITY_PIN, 15);}     
        } else {
          analogWrite(CONTINUITY_PIN, 0);}

        
        
        //analogWrite(AUDIO_PIN, 128);

        /*
        if(currentPWMMillis % 2 == 0) {
          analogWrite(AUDIO_PIN, 255);
        } else {
          analogWrite(AUDIO_PIN, 0);
        }
        */
        }      
      
      else if (abs(newVoltageReading)> 3.2 || VAC > 1  ) { // Voltage Check  
      if (!vflag)
        {analogWrite(CONTINUITY_PIN, 200); //Flash LED and then set vFlag True
        vflag = true;}
      
      else if ((currentPWMMillis % 1000 <= 100) && vflag && VAC < 3 ){ //LED on 10% 1s duty cycle
        {analogWrite(CONTINUITY_PIN, 15);}     
        } else {
          analogWrite(CONTINUITY_PIN, 0);}
       
       } else { //If none of the conditions are true, LED is off and vFlag Resets
        analogWrite(CONTINUITY_PIN, 0);
        vflag = false;
        rflag = false;
      //  wave.stop();
      }
    } 
    
    
  if (flashlightMode) {
      if (adjustedRx>=0 && adjustedRx<=10000) {flashlightPWM = adjustedRx;}
      if (adjustedRx<0 || adjustedRx>3000000) {flashlightPWM=0;}
      if (adjustedRx>10000 && adjustedRx<3000000 ) {flashlightPWM=10000;}
      analogWrite(CONTINUITY_PIN, 255 - round(255*(flashlightPWM/10000)));
      }


// Resistance Range Setting

    if(ohmsAutoRange){
      rRange = 'A';
    } else { 
      if (ohmsHighRange) {
      rRange = 'l';
        digitalWrite(SETRANGE_PIN, LOW);
      } else {
        rRange = 'h';
        digitalWrite(SETRANGE_PIN, HIGH);
      }
    }

// Voltage Refresh Rate  

    if(!vDisplayManual) {
      //if(((abs(previousV) > abs(newVoltageReading)*0.9) && (abs(previousV) < abs(newVoltageReading)*1.1)) || (VAC < 0.2) )
      if(abs(VMedianStep)>0.5 || VAC>0.5)
      {
        printedV = newVoltageReading;
      vMode = 'f';
      } else {
        printedV = VMedian;
      vMode = 's';
      }
    }
    
    if (vDisplayManual) {      
      if (smoothVmode) {
      printedV = averageVoltage;
      vMode = 's';
      } else {
      printedV = VMedian;
      vMode = 'f';
      }
    }
 
 /*
    // Resistance Refresh Rate  
    if ((smoothRmode) && (((rStability > 0.95) && (rStability < 1.05)) || (isnan (rStability)))) {
     printedR = averageResistance - minR;
     rSpeed = 's';
    } else {
     printedR = correctedRx - minR;
     rSpeed = 'f';
    }
    */
    printedR = correctedRx - minR;


    //High and Low R Tracking

    if (printedR > highR && printedR < 10000000 ){
      highR = printedR;
    }
    if (isBetween(printedR, -1000, lowR) && initalZero){
      lowR = printedR;
    }

 
 // Resistance Suffix
formatResistance(printedR, roundedR, rSuffix, rDigits);

formatResistance(lowR, roundedRlow, rSuffixlow, rDigitslow);

formatResistance(highR, roundedRhigh, rSuffixhigh, rDigitshigh); 

 // Voltage Format and Suffix
formatVoltage(printedV, roundedV, vSuffix, vDigits);
formatVoltage(lowV, roundedVlow, vSuffixlow, vDigitslow);
formatVoltage(highV, roundedVhigh, vSuffixhigh, vDigitshigh);

                 if(showTimes){
         Serial.print("Format V/R End ");
        Serial.println(millis());
                 }



    // Serial Commands
if (Serial.available() > 0) {
    receivedChar = Serial.read();
    if (receivedChar == 'Q') {
        Serial.print("Res: ");
        if(isBetween(printedR, -100, 8000000)){
          Serial.print(printedR);
          }else{
            Serial.print("OPEN");
          }             
        if(debug){
          Serial.print(" Vr: ");
          Serial.print(Ohms_v_output, 4);
          }
        Serial.print(" Vdc: ");
        Serial.print(printedV, vDigits);
        Serial.print(vSuffix);
        Serial.print(" mA: ");
        Serial.print(Ireading*1000,0);
        if(debug){
        Serial.print(" Aint: ");
        Serial.print(ch3int);
        }
        Serial.println("");

    } 
    else if (receivedChar == 'Z') {
        if(minR==0){
        minR = printedR;  // Set Zero Point
        Serial.print("Zero Set:");
        Serial.println(minR);
        }else{
          minR=0;
          Serial.println("Zero reset:");
        }        
    } 
    else if (receivedChar == 'C') {
        Serial.println("Zero Reset.");
        minR = 0; // Clear Zero Point
    } 
    else if (receivedChar == 'R') {
        ohmsHighRange = !ohmsHighRange;
        Serial.print("Low Range Toggled. Now:");
        Serial.println(ohmsHighRange);
    }
    else if (receivedChar == 'S') {
      smoothVmode =! smoothVmode;
      Serial.print("Smooth Mode Toggled. Now:");
      Serial.println(smoothVmode);
    }
        else if (receivedChar == 'V') {
      voltageDisplay =! voltageDisplay;
      Serial.println("Display Mode Toggled");
    }
        else if (receivedChar == 'A') {
      ohmsAutoRange =! ohmsAutoRange;
      Serial.print("Auto Range Toggled. Now:");
      Serial.println(ohmsAutoRange);
    }
    /*
        else if (receivedChar == 'X') {
      smoothRmode =! smoothRmode;
      Serial.print("Resistance Speed Toggled. Now:");
      Serial.println(smoothRmode);
    }
    */
        else if (receivedChar == 'F') {
      screenRefreshfast = !screenRefreshfast;
      Serial.print("Display Refresh Toggled. Now:");
      Serial.println(screenRefreshfast);
    }
    else if (receivedChar == 'M') {
      vDisplayManual = !vDisplayManual;
      Serial.print("Voltage Manual Display Speed Toggled. Now:");
      Serial.println(vDisplayManual);
    }
    else if (receivedChar == 'B') {
      Serial.print("Battery Level:");
      Serial.println(battery,2);
    }
    else if (receivedChar == 'q') {
      serialMode = !serialMode;
    }

    else if (receivedChar == 'T') {
    showTimes = !showTimes;
    } 
      
    
    else if (receivedChar == 'D') {
    debug = !debug;
    Serial.print("Debug Mode Toggled, Now: ");
    Serial.println (debug);
    }

    else if (receivedChar == 'e') {
    MinMaxDisplay = !MinMaxDisplay;
    Serial.print("Min Max Display Toggled, now: ");
    Serial.println (MinMaxDisplay);
    }


    else if (receivedChar == 'E') {
    ReZero();
    Serial.println("Min / Max Reset");
    }

    else if (receivedChar == 'I'){
      currentMode = !currentMode;
      Serial.print("Current Mode Toggled, Now:");
      Serial.println (currentMode);
    }

    else if (receivedChar == 'L'){
              Serial.print("LOG: I, V@I, T");
              
              Serial.print("BREAK"); //Begin Line 2
              
              Serial.print("I:");

              printArray(Serial, IAutoArray, NUM_VALUES,3);

              Serial.print("BREAK"); //Begin Line 3

              Serial.print("V:");

              printArray(Serial, VAutoArray, NUM_VALUES,3);        
              
              Serial.print("BREAK"); //Begin Line 4
              Serial.print("BREAK "); //Begin Line 5

              Serial.print("T:");      

              printArray(Serial, TAutoArray, NUM_VALUES,3);

              Serial.println(" "); //EndLog               
    }

    else if (receivedChar == 't') {
    // Print out date (DD/MM//YYYY)

    Serial.print(currentTime.getDayOfMonth());
    Serial.print("/");
    Serial.print(Month2int(currentTime.getMonth()));
    Serial.print("/");
    Serial.print(currentTime.getYear());
    Serial.print(" - ");

    // Print time (HH/MM/SS)
    Serial.print(currentTime.getHour());
    Serial.print(":");
    Serial.print(currentTime.getMinutes());
    Serial.print(":");
    Serial.println(currentTime.getSeconds());
    }




    
        else if (receivedChar == '?') {
      Serial.println("Valid Command List: reQuest, set Zero, Clear zero, manual Range, voltage Smooth mode, Voltage display, ohms Auto range, screen reFresh toggle, voltage Manual speed mode");
    }    
}


/*
    // Code for Reading unknown IR Input. 
  if (IrReceiver.decode()) {
    IrReceiver.printIRResultShort(&Serial); // Print the received IR code to Serial
    uint32_t irCode = IrReceiver.decodedIRData.decodedRawData;
    IrReceiver.resume();
  }
  */

/*
if (screenRefreshfast) {
  LCD_INTERVAL = 500;
  }else{
    LCD_INTERVAL = 1000;
  }
  */
 
//Auto display select
prevDisplay = voltageDisplay;

if (abs(averageVoltage) > 0.2 || VAC > 0.2) {
  voltageDisplay= true;
  
  if (prevDisplay != voltageDisplay){
      tft.fillRect(0, 0, 240, 170, ILI9341_BLACK);
      
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
            tft.setCursor(0, 0); //First Line
            tft.setTextSize(3);
            tft.print("VDC:");

            //tft.setCursor (0, 56); //First Line
            //tft.setTextSize(1); //Auto'd
            //tft.print("mVR:");

          tft.setTextSize(1); //Auto'd
            tft.setCursor (0, 72); //First Line
            tft.println("Min:");
            tft.println("Max:");
            tft.println("Range:");

          tft.setCursor (160, 72); //First Line
            tft.println("T:");
          tft.setCursor (160, 88); //First Line 
            tft.println("T:");
           
            tft.setCursor (0, 128); //First Line
            tft.print("VACrms: ");
      }
}

  if(isBetween(adjustedRx, -10, 100)){
  voltageDisplay= false;

  if (prevDisplay != voltageDisplay){
  tft.fillRect(0, 0, 240, 169, ILI9341_BLUE); //This is where all the static Resistance text goes:

        tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
        tft.setCursor(0, 0); //First Line
        tft.setTextSize(3);
        tft.print("R:");

        tft.setCursor (0, 56); 
        tft.setTextSize(1); //Auto'd
        tft.print("mVR:");

        tft.setCursor (0, 72); 
        tft.println("Min:");
        tft.println("Max:");        
        
        tft.setCursor (0, 108); 
        tft.print("Rx0: ");

  }
}

//Button Input Section Start

if(digitalRead(TYPE_PIN) == LOW && !buttonState) { //Detect Button Was Pressed
  buttonState = 1;
  holdStartTime = millis();
  }

if(digitalRead(TYPE_PIN) == LOW && buttonState) { //Illuminate LED if button held down
  if ((millis() - holdStartTime) > 500){
    analogWrite(CONTINUITY_PIN, 25);
  }
}

if(digitalRead(TYPE_PIN) == HIGH && buttonState) { //Determine what to do with button press
  if ((millis() - holdStartTime) > 500){ //Rezero if button was held down
      ReZero();
    
    MinMaxDisplay = 1;
    analogWrite(CONTINUITY_PIN, 0);
    buttonState = 0;
  }
 
  if ((millis() - holdStartTime- debounceDelay) < 450 && !SDMode){ //Type value if button not held down
      if(voltageDisplay){
            
            Keyboard.print(printedV, vDigits);       
          } else {
            Keyboard.print(printedR, rDigits);
          }
          Keyboard.press(0xD7);
          Keyboard.releaseAll();
          buttonState = 0;
          }
  }
        


      /*
      
      if(digitalRead(RED_PIN) == LOW){
        manualWrite=true;
        analogWrite(CONTINUITY_PIN, 10);

          File mLog = SD.open("mLog.csv", FILE_WRITE);
          if (mLog) {
            //dataFile.print("VDC: ");
            
              if(Irange){
                mLog.print(Ireading, 3); //Current
                }else{
                mLog.print(Ireading, 5);
                }
              
              mLog.print(",");

              if(abs(printedV)>0.005){            
              mLog.print(printedV, 3); //Voltage
              }else{
              mLog.print("");  
              }
              mLog.print(",");

              if(isBetween(printedR, -100, 9000000)){
              mLog.print(printedR, 3);
              }else{
                mLog.print("");
              }

            mLog.print(",");

            float timeSD = millis();
            mLog.print(timeSD/1000,3);

            if(firstMWrite){
            mLog.print(",");
            mLog.print("I/V/R/T");
            firstMWrite = 0;
            }

            mLog.println("");
                      
            mLog.close(); // Always close the file!
            writeCount++;


//            tft.setCursor(144, 280);
//            tft.print(writeCount);
          } else {
            Serial.println("Error Writing.");
            tft.setCursor(96, 280);
            tft.setTextSize(1); //Auto'd
            tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
            tft.print("FAIL!");
            }

      }else{
        if(manualWrite != manualWrite){
          analogWrite(CONTINUITY_PIN,0);
          }
        manualWrite=false;
      }
      */


if(serialMode) { //Serial Mode

if (currentMillis - previousSerialMillis >= serialUpdate) { //Serial Output Part
        previousSerialMillis = currentMillis;
    
      if(currentMode){ // If we're in Current Mode
        if(Irange){
          Serial.print("A: ");
          Serial.print(Ireading,3);
          }else{
            Serial.print("mA: ");
            Serial.print(Ireading*1000,2);
          }

        if(debug){
          Serial.print(" Int:");
          Serial.print(ch3int);
          Serial.print(" V:");
          Serial.print(ch3voltage,3);
        }



        Serial.print("BREAK"); //Begin Line 2
        Serial.print("I Low: ");
          Serial.print(ILow,3);
          Serial.print(" (Vdc:");
          Serial.print(VatILow);
          Serial.print(") T:");
          Serial.print(ItLow);

        Serial.print("BREAK"); //Begin Line 3
          Serial.print("I High: ");          
          Serial.print(IHigh,3);
          Serial.print(" (Vdc:");
          Serial.print(VatIHigh);
          Serial.print(") T:");
          Serial.print(ItHigh);
        
        Serial.print("BREAK"); //Begin Line 4
          Serial.print(" V:"); 
          Serial.print(roundedV, vDigits); 
          Serial.print(vSuffix);
          Serial.print("VDC");
        
        Serial.print("BREAK "); //Begin Line 5
          Serial.print(getVMinMax(roundedVlow,vDigitslow, vSuffixlow, roundedVhigh, vDigitshigh, vSuffixhigh));

        Serial.print("BREAK ");//Begin Line 6
          Serial.print("I@max :");
          Serial.print(IatVHigh,3);
          Serial.print("A T:");
          Serial.print(VtHigh);

          Serial.print(" I@min :");
          Serial.print(IatVLow,3);
          Serial.print("A T:");
          Serial.print(VtLow);




        Serial.println(""); //End
      }
      
      else if(voltageDisplay){ // If we're in Voltage Display 
      
        if (abs(averageVoltage) < 0.001 && VAC < 0.2){
          Serial.print("absV <1mV"); // Report Open // Line 1
                  
          Serial.print("BREAK"); //Begin Line 2
          if (MinMaxDisplay){            
                Serial.print(getVMinMax(roundedVlow,vDigitslow, vSuffixlow, roundedVhigh, vDigitshigh, vSuffixhigh));
              }else{
              Serial.print("Min Max Off");
              }

          Serial.print("BREAK"); //Begin Line 3
          Serial.println("");// End
                  
          } else {
            Serial.print(roundedV, vDigits); //Start Line 1
            Serial.print(vSuffix);
            Serial.print("VDC");

            Serial.print("BREAK"); //Begin Line 2
              if (MinMaxDisplay){            
                Serial.print(getVMinMax(roundedVlow,vDigitslow, vSuffixlow, roundedVhigh, vDigitshigh, vSuffixhigh));              
              }else{
                Serial.print("Min Max Off");
              }
              
            Serial.print("BREAK"); //Begin Line 3
              Serial.print("VAC_RMS:"); 
              Serial.print(VAC,1); 
            
            Serial.println(""); //End
          }
        }

    else { // If we're in neither Current Nor Voltage Mode (Resistance)
     if(Ohms_v_output < (ZenerMax-0.012)){ //Line 1 depends on resistance
      Serial.print(roundedR, rDigits);
      Serial.print(rSuffix);
      Serial.print(" Ohms");
      }else{
        Serial.print("R Open");
        }
      
      Serial.print("BREAK"); //Begin Line 2
        if(MinMaxDisplay){
          Serial.print(getRMinMax(roundedRlow,rDigitslow, rSuffixlow, roundedRhigh, rDigitshigh, rSuffixhigh));
        }
        else{
          Serial.print("Min Max Off");
        }
      Serial.print("BREAK");//Begin Line 3
      Serial.print("vR:");
      Serial.print(Ohms_v_output,4);

      Serial.println(""); //End Output

     } // Close Resistance Section
    } // Close Serial Update Timer
} // Close Serial Update


if(digitalRead(RED_PIN) == LOW) // If manual write button pressed
{
    analogWrite(CONTINUITY_PIN, 10);
    float TimeS;
    TimeS = (millis());
    TimeS = TimeS/1000;          
    ItHighm = TimeS;

    logValuesManual(newVoltageReading, ItHighm, Ireading);
    manualWrite=1;
    manualWritePrimed=1;
    //lastLoggedI = Ireading;
}else{
  manualWrite = 0;
}


if(!manualWrite && manualWritePrimed){
  
  File mLog = SD.open("mLog.csv", FILE_WRITE);
    if (mLog) {
    // Write data

    mLog.println("LOG");              
    //dataFile.println(""); //Begin Line 2
    mLog.print("I:,");
    printArray(mLog, IManArray, NUM_VALUES,3);
    //dataFile.println(""); //Begin Line 3
    mLog.print("V:,");
    printArray(mLog, VManArray, NUM_VALUES,3);        
    //dataFile.println(""); //Begin Line 4
    mLog.print("T:,");      
    printArray(mLog, TManArray, NUM_VALUES,3);
    mLog.println(""); //extraLine
    mLog.println(""); //EndLog    
    
    writeCount++;        
    
        // Always close the file to ensure data is actually saved
    mLog.close();
    AutologTriggered = 0;
    manualWritePrimed = 0;
    //analogWrite(CONTINUITY_PIN, 0);

} else {
  Serial.println("Error opening file.");
  tft.setCursor(96, 280);
  tft.setTextSize(2); 
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print("FAIL!");
  }
}



if(!AutologTriggered && writePrimed){
  
  File dataFile = SD.open("dataFile.csv", FILE_WRITE);
    if (dataFile) {
    // Write data

    dataFile.println("LOG");              
    //dataFile.println(""); //Begin Line 2
    dataFile.print("I:,");
    printArray(dataFile, IAutoArray, NUM_VALUES,3);
    //dataFile.println(""); //Begin Line 3
    dataFile.print("V:,");
    printArray(dataFile, VAutoArray, NUM_VALUES,3);        
    //dataFile.println(""); //Begin Line 4
    dataFile.print("T:,");      
    printArray(dataFile, TAutoArray, NUM_VALUES,3);
    dataFile.println(""); //EndLog    
    writeCount++;        
        // Always close the file to ensure data is actually saved
    dataFile.close();
    AutologTriggered = 0;
    writePrimed = 0;
    //analogWrite(CONTINUITY_PIN, 0);



} else {
  Serial.println("Error opening file.");
  tft.setCursor(96, 280);
  tft.setTextSize(2); 
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print("FAIL!");
  }
}

if(Irange){
  Istep = 0.1;
}else{
  Istep =0.005;
}


if(
  (SDMode && currentOnOff) && ( //SD mode and Current mode are on, AND
  (IHigh==Ireading && IHigh > 0.05) || (ILow==Ireading && ILow < -0.05) || (newVoltageReading==highV && highV>0.05) || (newVoltageReading==lowV && lowV < -0.05 ) || //if either I/ V high / low is triggered OR
  (abs(lastLoggedI)>(abs(Ireading)+Istep) || abs(lastLoggedI)<(abs(Ireading)-Istep)) // If I reading is more than Istep different than the previous reading
  // || digitalRead(RED_PIN) == LOW 
  )//SD mode close
)
{
    //analogWrite(CONTINUITY_PIN, 10);             
    float TimeS;
    TimeS = (millis());
    TimeS = TimeS/1000;          
    ItHighm = TimeS;

    logValues(newVoltageReading, ItHighm, Ireading);
    AutologTriggered=1;
    writePrimed=1;
    lastLoggedI = Ireading;

}else if (!manualWrite){
    
    AutologTriggered=0;

    // Update Display
   if ((currentMillis - previousLcdMillis >= LCD_INTERVAL ) && IHigh!=Ireading && ILow!=Ireading && highV!=newVoltageReading){
        previousLcdMillis = currentMillis;

                if(showTimes){
        Serial.print("Display Start ");
        Serial.println(millis());
                }

        tft.startWrite();

        if (screenRefreshfast) {
          LCD_INTERVAL = 500;
          }else{
            LCD_INTERVAL = 1000;
          }
     
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(190,300);
      tft.setTextSize(1); //Auto'd
//      tft.print("VIN:");
      if(battery>3){
        tft.print(battery,1);        
          
          //if(battery<5.1){     
          //  tft.setCursor(72,48);       
          //  tft.print("BATT:LOW");}
          
        }else{
        tft.print("USB");
        }

        if(SDMode){
          tft.setCursor(144, 280);
          tft.setTextSize(1);
          tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
          tft.print(writeCount);
        }

          tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
          tft.setCursor(0, 295); //Clock Display
          tft.setTextSize(3);
          tft.print(formatTime(millis()));

      if(currentOnOff || currentMode){ //Display the current if either Current mode is true or it's on manually

        if(showTimes){
        Serial.print("Display I Start ");
        Serial.println(millis());
        }

        tft.setTextColor(ILI9341_WHITE, ILI9341_PINK);
        tft.setTextSize(3);        
        if(Irange){
          tft.setCursor(32,180);
          tft.print(Ireading,2);
          }else{
            tft.setCursor(48,180);
            tft.print(Ireading*1000,2);
          }
        tft.print("  ");

      if(MinMaxDisplay){
        tft.setTextSize(1); //Auto'd
         tft.setCursor(72,204);
          tft.print(ILow,3);
          tft.print(" ");
        tft.setCursor(176,204);  
          tft.print(ItLow);  
         tft.setCursor(72,220);
          tft.print(IHigh,3);
          tft.print(" ");
        tft.setCursor(176,220);   
          tft.print(ItHigh);
        tft.setCursor(176,236);
          tft.print(IMedian,2);

        }  
      }       
      
      if(voltageDisplay){
                
        if(showTimes){
        Serial.print("Display V Start ");
        Serial.println(millis());
        }
                
                
                tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
          tft.setCursor(80, 0);
          tft.setTextSize(3);
          if ((abs(averageVoltage) <= 0.001 && VAC < 0.1)){ //If the avg V ABS and VAC are below very low thresholds, report open           
            tft.print("<1mV");
            }else{
            if(roundedV<0){
              tft.setTextColor(ILI9341_BLACK,ILI9341_WHITE);
              roundedV=abs(roundedV);
              tft.print(roundedV, vDigits); //If not low, then Display reading
              
            tft.print(vSuffix);
            tft.print("  ");
            tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

            }else{
            
            
            tft.print(roundedV, vDigits); //If not low, then Display reading
            tft.print(vSuffix);
            tft.print("   ");
            }
            
            }
              tft.setTextSize(1); //Auto'd

          if (MinMaxDisplay){ //Display Min / Max Range if on
              tft.setCursor (48, 72);
                tft.print(roundedVlow,vDigitslow);
                tft.print(vSuffixlow);
                tft.print(" ");
              tft.setCursor (176, 72);
                tft.print(VtLow);
              tft.setCursor (48, 88);
                tft.print(roundedVhigh,vDigitshigh);
                tft.print(vSuffixhigh);
                tft.print(" ");
              tft.setCursor (176, 88);
                tft.print(VtHigh);                
              tft.setCursor (72, 104);
                tft.print(highV-lowV,3);
              

              //displayVminmax();
              }
//          tft.setCursor(0, 48);
            
//            if(VAC>0.5){ 
                tft.setCursor (88, 128);
                tft.print(VAC, 1);
                tft.print(" ");
            

            if(debug){
              tft.setCursor(0, 32);        
              tft.print("V_int:");
              tft.print(voltage_int);
                }
           
                
      } else { //Resistance Part

        if(showTimes){
            Serial.print("Display R Start ");
            Serial.println(millis());
                }

        tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
        tft.setCursor(32, 0); 
        tft.setTextSize(3);
//        tft.print("R:");
          if(Ohms_v_output < (ZenerMax-0.012)) { //Show Resistance if below Threshold
              if(roundedR<0){
                tft.setTextColor(ILI9341_BLUE, ILI9341_WHITE);
                roundedR = (abs(roundedR));
                tft.print(roundedR, rDigits);
                tft.print(rSuffix);
                tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
              }else{
              tft.print(roundedR, rDigits);
              tft.print(rSuffix);
              }
               tft.print("  ");
               }else{
                tft.print("OPEN   ");
               }

            tft.setCursor (48, 56); 
            tft.setTextSize(1); //Auto'd
//          tft.print("mVR:");
            if((Ohms_v_output*1000)>50){
            tft.print(Ohms_v_output*1000,0);
            }
            else{
              tft.print(Ohms_v_output*1000,2);
            }
            tft.print("   ");

            tft.setCursor (48, 72); //First Line
              if (MinMaxDisplay){
                displayRminMax();            
                }          

            tft.setCursor (60, 108); 
//            if(minR != 0.0){
//             tft.print("Rx0: ");
              tft.print(minR, 3);
              tft.print("  ");
//                  }

            if(debug){
              tft.setCursor(60, 32);
              tft.print("ADC:"); 
              tft.print(Ohms_ADS_Int);
              }

          

          } // Close Resistance Display
                                    
      
  //  tft.Display();

        tft.endWrite();
    

        if(showTimes){
Serial.print("DISPLAY Finished ");
Serial.println(millis());
        }

} //Close Display Update

}//Close Write Condition Check

        if(showTimes){
Serial.print("Loop Finished ");
Serial.println(millis());
        }


} //Close Loop

/*

Custom Commands:

General: 
Q: Single Data Request  
q: Continuous Data Push
F: Force Fast Screen Refresh
B: Print Battery Voltage
D: Debug Mode
E: Min / Max Value Reset
e: Display Min / Max Values
T: Show Times

Resistance Function:
Z: Sets Zero Point. Clears if already Set.
A: Toggle Auto Range  
	R: Toggle Range

Voltage Function:
M: Voltage Manual Speed Mode Enable
	S: Toggles Average / Instant Reading
V: Force Voltage Display

Current Function:
I: Toggle Current Mode

*/

