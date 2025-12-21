#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>


RTC_PCF8523 rtc;
Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

// track timing
unsigned long lastStatus = 0;
unsigned long lastLog = 0;
unsigned long lastLogMillies = 0;


//Screen Setup
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Pin List
#define SD_CS_PIN 23
const int PIXEL_PIN = 17;       // Onboard NeoPixel
const int sdEnable = 4;
const int ch2Enable = 24;
const int serialEnable = 25;
const int trigThres = A0;
const int autorangePin = 5;


// BUFFER SETTINGS
const size_t MAX_SAMPLES = 2000;   // adjustable
struct Sample {
  float d1;
  float d2;
  float td;
  float t;   // optional: timestamp in microseconds
};
Sample buffer[MAX_SAMPLES];
volatile size_t sampleCount = 0;

// Trigger state tracking
bool triggerPrev = false;

int triggerCount = 0;


//float   multiplier = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results) */
float   multiplier = 1.0F;    /* GAIN TWO ADS1015 @ +/- 2.048V gain (12-bit results) */
//float   multiplier = 0.5F;    /*GAIN FOUR ADS1015 @ +/- 2.048V gain (12-bit results) */

int16_t results01;
int16_t results23;
float voltage01 = 0.0;
float voltage23 = 0.0;
//float vScale = 69.669; //for 15k bridge
float vScale = 14.7482; //for 75k bridge

float trigRaw = 0.0;


bool ch2on = 0;
bool logON = 1;

bool logError = 0;

bool writeTrigger = 0;
bool trigFlag = 0;
float prevVoltage = 0.0;
float vStep = 0.25;
float noiseThreshold = 0.5;
float lastLoggedV01 = 0.25;
float vMax01 = 0.0;
float vMin01 = 0.0;

float prevTriggerT = 0.0;
float deltaT = 0.0;
float triggerT = 0.0;
float timemS = 0.0;

float lastLoggedV23 = 0.25;
float vMax23 = 0.0;
float vMin23 = 0.0;

//ADS autorange related

bool firstVoltRun = 1;
bool autorange = 1;

// ADS1015 gain factors (mV per bit) for each gain setting
const float GAIN_FACTOR_TWOTHIRDS = 3.0;  // 2/3x (±6.144V range)
const float GAIN_FACTOR_1   = 2.0;   // ±4.096V
const float GAIN_FACTOR_2   = 1.0;  // ±2.048V
const float GAIN_FACTOR_4   = 0.5; // ±1.024V
const float GAIN_FACTOR_8   = 0.25;// ±0.512V
const float GAIN_FACTOR_16  = 0.125;// ±0.256V


// instead of “static const int kGainLevels[] = { … };”
static const adsGain_t kGainLevels[] = {
  GAIN_TWOTHIRDS,
  GAIN_ONE,
  GAIN_TWO,
  GAIN_FOUR,
  GAIN_EIGHT,
  GAIN_SIXTEEN
};

  static size_t  gainIndexVolt;
  // ADC count thresholds for gain switching
  static const int ADC_COUNT_LOW_THRESHOLD  = 250;
  static const int ADC_COUNT_HIGH_THRESHOLD = 1800;
  static const float kGainFactors[] = {GAIN_FACTOR_TWOTHIRDS, GAIN_FACTOR_1, GAIN_FACTOR_2, GAIN_FACTOR_4, GAIN_FACTOR_8, GAIN_FACTOR_16};
  static const int   kNumGainLevels = sizeof(kGainLevels) / sizeof(kGainLevels[0]);

  static const int ADC_GUARD_BAND = 100;

const int PIXEL_COUNT = 1;

Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

SdFat SD;
File32 intervalFile;
File32 triggerFile;
File32 logfile;

char filename[40];

char intervalFilename[32];
char triggerFilename[32];

SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

int16_t readRaw01() {
  return ads.readADC_Differential_0_1();   // blocking read -> fresh conversion
}

void printField(Print* pr, char sep, uint8_t v) {
  if (sep) {
    pr->write(sep);
  }
  if (v < 10) {
    pr->write('0');
  }
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

// --- Helper: blink NeoPixel a color for a short time ---
void blinkPixel(uint8_t r, uint8_t g, uint8_t b, int duration = 5) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
  delay(duration);
  pixel.clear();
  pixel.show();
}

void makeUniqueFilename(const char* base, const char* suffix, char* outBuf, size_t outSize) {
  // Try without number first
  snprintf(outBuf, outSize, "/%s%s", base, suffix);
  if (!SD.exists(outBuf)) {
    return;
  }

  // Otherwise increment
  for (int i = 1; i < 10000; i++) {
    snprintf(outBuf, outSize, "/%s%d%s", base, i, suffix);
    if (!SD.exists(outBuf)) {
      return;
    }
  }

  // Safety fallback
  snprintf(outBuf, outSize, "/%s9999%s", base, suffix);
}

void captureSample(float data1, float data2) { // FUNCTION: capture a sample into RAM
  if (sampleCount >= MAX_SAMPLES) return;   // buffer full - ignore or handle differently

  prevTriggerT=triggerT;
  triggerT = micros();
  timemS = millis();
  deltaT =  triggerT-prevTriggerT;


  buffer[sampleCount].d1 = data1;
  buffer[sampleCount].d2 = data2;
  buffer[sampleCount].t  = (timemS+(triggerT/1000));       // optional timestamp
  buffer[sampleCount].td  = deltaT;

    // Success!
  //blinkPixel(0, 0, 255);       //  Blue for success

  sampleCount++;
}

void flushBufferToSD() { //FUNCTION: flush RAM buffer to SD
  logfile = SD.open(filename, FILE_WRITE);
  if (!logfile) {
    Serial.println("ERROR: Could not open capture file");
    return;
  }

  DateTime now = rtc.now();

  //Serial.print("Writing ");
  //Serial.print(sampleCount);
  //Serial.println(" samples to SD...");

  for (size_t i = 0; i < sampleCount; i++) {
    
    logfile.print(buffer[i].td,1);
    logfile.print(",");
    logfile.print(buffer[i].d1, 3);
    logfile.print(",");
    logfile.print(buffer[i].d2, 1);
    logfile.print(",");
    logfile.println(now.timestamp());
  }

  logfile.flush();
  logfile.close();

  //Serial.println("Done writing.");
    // Success!
  blinkPixel(0, 255, 0);       //  GREEN for success

  // Reset buffer
  sampleCount = 0;
}

// --- Regular CSV logging / Serial Print function (call every loop) ---
void logCSV(float data1, float data2) {
  unsigned long currentmillis = millis();
  if (currentmillis - lastStatus < 2000) return;
  lastStatus = currentmillis;

  DateTime now = rtc.now();


  updateDisplay();

  if(digitalRead(serialEnable)){
  Serial.print("T:"); Serial.print(now.timestamp());
  Serial.print("/ V 0-1: "); Serial.print(voltage01,3); Serial.print("V/ ");
  Serial.print("V 2-3: "); Serial.print(voltage23,3); Serial.print("V/ ");
  Serial.print("Total Logs: "); Serial.println(triggerCount);
  //Serial.println(results01);
  //Serial.println(gainIndexVolt);
  } 


  // Success!
  blinkPixel(0, 128, 128);       //  GREEN for success
}


void setup(void){
  Serial.begin(115200);
    delay(1000);
 Serial.println("Hello!");

  Wire.begin();
  Wire.setClock(1000000);   // 1 MHz I2C

  analogReadResolution(12);



  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED init failed"));
    while (1); // halt
  }
  
  display.clearDisplay();
  display.setTextSize(2);
  display.setRotation(2); //rotates text on OLED 1=90 degrees, 2=180 degrees
  display.setTextColor(SSD1306_WHITE);
  // Splash screen
  display.setCursor(0, 0);
  display.print("Datalogger");
  display.display();
  
  delay(200);

  // Start RTC
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
    while (1);
  }

  // Print current time
  DateTime now = rtc.now();
  Serial.print("RTC Time: ");
  Serial.println(now.timestamp());


  pinMode(sdEnable, INPUT_PULLUP);
  pinMode(ch2Enable, INPUT_PULLUP);
  pinMode(serialEnable, INPUT_PULLUP);
  pinMode(trigThres, INPUT);
  pinMode(autorangePin, INPUT_PULLUP);

  
  //ads.setGain(GAIN_TWO);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setDataRate(RATE_ADS1015_3300SPS); //Fast as possible


  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);  
  }
  
  pixel.begin();
  pixel.clear();
  pixel.show();

  if(digitalRead(sdEnable)){
    logON = 1;
    
    if (!SD.begin(config)) {
      Serial.println("SD init failed!");
      while (1) {
        blinkPixel(128, 0, 0, 200);   // Red repeating = SD unavailable
      }
    }


  /* //Not needed with RTC
  // ---- Create trigger capture file ----
  makeUniqueFilename("capture", ".csv", triggerFilename, sizeof(triggerFilename));
  Serial.print("Trigger log file: ");
  Serial.println(triggerFilename);

  triggerFile = SD.open(triggerFilename, FILE_WRITE);
  if (!triggerFile) {
    Serial.println("Failed to create trigger capture file!");
    while (1);
  }
  //triggerFile.println("time_us,data1,data2,data3");   // example header
  //triggerFile.flush();
  */

  // Create unique CSV filename with timestamp
  sprintf(filename, "log_%04d%02d%02d_%02d%02d%02d.csv",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());

  logfile = SD.open(filename, FILE_WRITE);
  if (!logfile) {
    Serial.println("Could not create log file!");
    while (1);
  }

  // First row: human-readable timestamp
  logfile.print("Log Start Time,");
  logfile.println(now.timestamp());
  logfile.println("Delta uS,Voltage Ch1 (V),Voltage Ch2 (V), RTC Stamp");
  logfile.flush();

  Serial.print("Logging to: ");
  Serial.println(filename);


  }else{
    logON = 0;
  }
  
  if(digitalRead(ch2Enable)){
    ch2on = true;    
  }else{
    ch2on = false;
    voltage23 = 0.0;
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
  }

  if(digitalRead(autorangePin)){
    autorange = 1;
    Serial.print("autorange on");
  }else{
    autorange = 0;
    Serial.print("autorange off");
  }
  
}

void loop(void){
  prevVoltage = voltage01;

  trigRaw = analogRead(trigThres);

  vStep = 0.125*((4*trigRaw/4095)*(4*trigRaw/4095)*(4*trigRaw/4095));



  
  //results01 = ads.readADC_Differential_0_1(); // integer from ADS
  
  if(ch2on){
    results01 = ads.readADC_Differential_0_1(); // integer from ADS
    results23 = ads.readADC_Differential_2_3(); // integer from ADS
    voltage23 = ((results23*multiplier)/1000)*vScale; //Convert to voltage
  }else if(autorange){

    if (firstVoltRun) {
      gainIndexVolt = kNumGainLevels - 1; // start at max gain
      firstVoltRun  = false;
      ads.setGain(kGainLevels[gainIndexVolt]);
      (void)readRaw01();                  // throw away first sample after gain set
    }
    
  
  results01 = ads.getLastConversionResults();
  int absCounts = abs(results01);

  bool nearLow  = absCounts < (ADC_COUNT_LOW_THRESHOLD + ADC_GUARD_BAND);
  bool nearHigh = absCounts > (ADC_COUNT_HIGH_THRESHOLD - ADC_GUARD_BAND);
  
  // set current gain
  ads.setGain(kGainLevels[gainIndexVolt]);

  // get a fresh conversion at THIS gain
  results01 = readRaw01();

  // autorange decision
  if (abs(results01) > ADC_COUNT_HIGH_THRESHOLD && gainIndexVolt > 0) {
    --gainIndexVolt;
    ads.setGain(kGainLevels[gainIndexVolt]);
    (void)readRaw01();          // discard one sample after changing gain
    //Serial.print("range+");
    return;                     // don't compute volts from the pre-change sample
  }

  if (abs(results01) < ADC_COUNT_LOW_THRESHOLD && gainIndexVolt < (kNumGainLevels - 1)) {
    ++gainIndexVolt;
    ads.setGain(kGainLevels[gainIndexVolt]);
    (void)readRaw01();          // discard one sample after changing gain
    //Serial.print("range-");
    return;                     // same idea
  }

  // now results01 definitely corresponds to current gainIndexVolt
  voltage01 = ((results01 * kGainFactors[gainIndexVolt]) / 1000.0f) * vScale;
  }else{
        if (firstVoltRun) {
      firstVoltRun  = false;
      ads.setGain(GAIN_TWO);
      (void)readRaw01();                  // throw away first sample after gain set
    }
    
    multiplier = 1.0F; 
    results01  = ads.getLastConversionResults();
    voltage01 = ((results01*multiplier)/1000)*vScale; //Convert to voltage

  }
  

  if(voltage01>vMax01){
    vMax01 = voltage01;
    trigFlag = 1;
  }
  if(voltage01<vMin01){
    vMin01 = voltage01;
    trigFlag = 1;
  }
  
  if(abs(voltage01) > noiseThreshold){
    if(voltage01 > lastLoggedV01+vStep){
      trigFlag = 1;
    }
    if(voltage01 < lastLoggedV01-vStep){
      trigFlag = 1;
    }
  }

  if(digitalRead(ch2Enable)){
    if(voltage23>vMax23){
      vMax23 = voltage23;
      trigFlag = 1;
    }
    if(voltage23<vMin23){
      vMin23 = voltage23;
      trigFlag = 1;
    }
  if(abs(voltage23) > noiseThreshold){ 
      if(voltage23 > lastLoggedV23+vStep){
        trigFlag = 1;
      }
      if(voltage23 < lastLoggedV23-vStep){
        trigFlag = 1;
      }
    }
  }

  
  if(trigFlag) { //trigger when voltage is more or less than vStep from the last logged V
    writeTrigger = true;
    lastLoggedV01 = voltage01;
    lastLoggedV23 = voltage23;
    trigFlag = 0;
  }else{
    writeTrigger = false;
    //blinkPixel(32, 0, 32);       //  Purple for running
  }

  // If trigger is active, capture to RAM
  if (writeTrigger && logON) {
    captureSample(voltage01, voltage23);
    triggerCount++;
  }else{
    logCSV(voltage01, voltage23);
    if(millis() - lastLog > 1000){
      lastLog = millis();
      captureSample(voltage01, voltage23);
      if(logON){
      flushBufferToSD();
      }
    }
    
    writeTrigger = false;
  }

  // Detect trigger falling edge → flush
  if (triggerPrev == true && writeTrigger == false && logON) {
    flushBufferToSD();
  }

  triggerPrev = writeTrigger;
}

void updateDisplay(void) {
  // Prepare values for display
  display.clearDisplay();
  display.setTextSize(2);
  
  // Display Voltages
  display.setCursor(0,0);
  display.print("V01:");
  display.print(voltage01,2);
  display.setCursor(0,16);
  
  display.print("V23:");
  if(digitalRead(ch2Enable)){
    
    display.print(voltage23,2);
  }else{
    display.print("OFF");
  }

  display.setTextSize(1);
  display.setCursor(0,32);
  display.print ("#logs:");
  display.print (triggerCount);

  display.setCursor(64,32);
  display.print ("trig");
  display.print (vStep);

  timemS = millis();
  display.setCursor(0,40);
  display.print ("Runtime (s): ");
  display.print ((timemS/1000),0);
  
  if (logError) {
    display.setTextSize(2);
    display.setCursor(0,48);
    display.print ("SD ERROR");
  }else if(!logON){
    display.setTextSize(2);
    display.setCursor(0,48);
    display.print ("SD OFF");  
  }else{
    display.setTextSize(2);
    display.setCursor(0,48);
    display.print ("SD GOOD");
  }

  display.display(); // update the OLED with all the drawn content
}

