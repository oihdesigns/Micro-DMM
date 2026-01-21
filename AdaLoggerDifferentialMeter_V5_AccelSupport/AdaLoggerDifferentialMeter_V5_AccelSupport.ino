/*
Logging speed notes: 
Voltage only (single channel): ~2kHz (500uS)
Voltage + Current: ~1kHz (1000uS)
Voltage + Accel: 1400uS
Voltage + Current + Accel: 500Hz (2000uS)
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


RTC_PCF8523 rtc;
Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

//when using a second one for current logging
Adafruit_ADS1015 ads2;     /* Use this for the 12-bit version */

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






/*
//breadboard version
const int sdEnable = 4;
const int ch2Enable = 24;
const int serialEnable = 25;
const int trigThres = A0; //10K Pot vcc to gnd
const int autorangePin = 5;
const int markButton = 6;
const int battPin = A2; //2x 4.7K 
const int slowLogPin = 9;
const int screenEnable = 13;
*/

//solder version

const int ch2Enable = 25;
const int voltReadPin = 24;
const int accelPin = 13; //DIP3
const int currentEnable = 12; //DIP4
const int autorangePin = 11; //DIP5
const int slowLogPin = 10; //DIP6
const int screenEnable = 9; //DIP7
const int sdEnable = 6; //DIP8

const int markButton = 5;

const int trigThresPin = A0; //10K Pot vcc to gnd
const int battPin = A2; //2x 4.7K 
//Note: A3 is the temp pin: 10k vcc to pin, ntc pin to ground

Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);
  float accelX = 0.0;
  float accelY = 0.0;
  float accelZ = 0.0;

  float accelThres = 4;

  float accelXMax = 0.0;
  float accelYMax = 0.0;
  float accelZMax = 0.0;

  float accelLastLoggedX = 0.0;
  float accelLastLoggedY = 0.0;
  float accelLastLoggedZ = 0.0;



  unsigned long accelXTime = 0.0;
  unsigned long accelYTime = 0.0;
  unsigned long accelZTime = 0.0;

  unsigned long accelWindowTime = 0.0;

  int accelCount;
  int accelZMaxcount;


// BUFFER SETTINGS
  const size_t MAX_SAMPLES = 2000;   // adjustable
  struct Sample {
    float d1;
    float d2;
    float d3;
    float d4;
    float d5;
    float d6;
    float td;
    float t;   // optional: timestamp in microseconds
  };
  Sample buffer[MAX_SAMPLES];
  volatile size_t sampleCount = 0;

// Trigger state tracking
bool triggerPrev = false;

int logCount = 0;


//float   multiplier = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results) */
float   multiplier = 1.0F;    /* GAIN TWO ADS1015 @ +/- 2.048V gain (12-bit results) */
//float   multiplier = 0.5F;    /*GAIN FOUR ADS1015 @ +/- 2.048V gain (12-bit results) */

int16_t ads0_results23;
int16_t ads1_results23;
int16_t results23;
float voltage01 = 0.0;
float voltage01actual = 0.0;
float voltage23 = 0.0;
//float vScale = 69.669; //for 15k bridge
float vScale = 14.319; //for 75k bridge

float current = 0.0;

float voltageTrigRaw = 0.0;
float accelThresRaw = 0.0;

int tempF= 0;

float battV = 0.0;
float battRaw = 0;


bool ch2on = 0;
bool logON = 1;
bool manualLog = 0;

bool logError = 0;

bool screenInhibit = 0;
bool screenInhibitPrevious = 0;

float logInterval = 0.0;

bool writeTrigger = 0;
bool trigFlag = 0;
float prevVoltage = 0.0;
float vStep = 0.25;
float noiseThreshold = 0.5;
float lastLoggedV01 = 0.25;
float vMax01 = 0.0;
float vMin01 = 0.0;

float lastLoggedI = 0.25;
float iMax = 0.0;
float iMin = 0.0;

float prevTriggerT = 0.0;
float deltaT = 0.0;
float triggerT = 0.0;
float timemS = 0.0;

float lastLoggedV23 = 0.25;
float vMax23 = 0.0;
float vMin23 = 0.0;

//ADS autorange related

bool firstVoltRunAuto = 1;
bool firstVoltRunMan = 1;
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


float readNTCTemperatureC(
    uint8_t analogPin,
    uint8_t adcBits,
    float   vRef,
    float   rFixed,
    float   rNominal,
    float   tNominalC,
    float   beta
  ) {
    // Read ADC
    uint32_t adcMax = (1UL << adcBits) - 1;
    uint32_t adcRaw = analogRead(analogPin);

    // Convert ADC code to voltage
    float vNTC = (adcRaw * vRef) / adcMax;

    // Protect against divide-by-zero and rail conditions
    if (vNTC <= 0.0f || vNTC >= vRef) {
        return NAN;
    }

    // Compute thermistor resistance
    float rNTC = rFixed * (vNTC / (vRef - vNTC));

    // Beta equation
    float t0K = tNominalC + 273.15f;
    float invT = (1.0f / t0K) + (1.0f / beta) * log(rNTC / rNominal);
    float tempK = 1.0f / invT;

    return tempK - 273.15f;
}

void readAccel(void){
  sensors_event_t event;

    //accelWindowTime = micros();

    accel.getEvent(&event);

    accelX = event.acceleration.x;
    accelY = event.acceleration.y;  
    accelZ = event.acceleration.z;
    accelCount++; 

    accelThresRaw = analogRead(trigThresPin);
    accelThres = 0.25*((4*accelThresRaw/4095)*(4*accelThresRaw/4095)*(4*accelThresRaw/4095));;

    if(abs(accelX)-accelThres > abs(accelLastLoggedX)||abs(accelX)+accelThres < abs(accelLastLoggedX)){
      accelXMax = accelX;
      //accelXTime = micros()-accelWindowTime;
      trigFlag = 1;
      //Serial.print(accelX);
    }
    if(abs(accelY)-accelThres > abs(accelLastLoggedY)||abs(accelY)+accelThres < abs(accelLastLoggedY)){
      accelYMax = accelY;
      //accelYTime = micros()-accelWindowTime;
      trigFlag = 1;
    }
    if(abs(accelZ)-accelThres > abs(accelLastLoggedZ)||abs(accelZ)+accelThres < abs(accelLastLoggedZ)){
      accelZMax = accelZ;
      //accelZTime = micros()-accelWindowTime;
      accelZMaxcount++;
      trigFlag = 1;
    }
}

int16_t readRaw23() {
    return ads.readADC_Differential_2_3();   // blocking read -> fresh conversion
  }

int16_t ads2ReadRaw23() {
    return ads2.readADC_Differential_2_3();   // blocking read -> fresh conversion
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

void captureSample(float data1, float data2, float data3, float data4, float data5, float data6) { // FUNCTION: capture a sample into RAM
  if (sampleCount >= MAX_SAMPLES) return;   // buffer full - ignore or handle differently

  prevTriggerT=triggerT;
  triggerT = micros();
  timemS = millis();
  deltaT =  triggerT-prevTriggerT;


  buffer[sampleCount].d1 = data1;
  buffer[sampleCount].d2 = data2;
  buffer[sampleCount].d3 = data3;
  buffer[sampleCount].d4 = data4;
  buffer[sampleCount].d5 = data5;
  buffer[sampleCount].d6 = data6;
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
    logfile.print(buffer[i].d3, 4);
    logfile.print(",");
    logfile.print(buffer[i].d4, 2);
    logfile.print(",");
    logfile.print(buffer[i].d5, 2);
    logfile.print(",");
    logfile.print(buffer[i].d6, 2);
    logfile.print(",");
    logfile.print(tempF);
    logfile.print(",");
    //logfile.println(now.timestamp());
    if(manualLog){
      logfile.print(now.timestamp());
      logfile.println(", MARK");
      
    }else{
      logfile.println(now.timestamp());
    }
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
void outputs(float data1, float data2) {
  unsigned long currentmillis = millis();
  if (currentmillis - lastStatus < 2000) return;
  lastStatus = currentmillis;

  DateTime now = rtc.now();


  if(!screenInhibit){
    updateDisplay();
  

  Serial.print("Time:"); Serial.print(now.timestamp());
  Serial.print("/ V 0-1: "); Serial.print(voltage01,3); Serial.print("V/ ");
  Serial.print("V0-1 actual:");
  Serial.print(voltage01actual);
  //Serial.print("/ Gain: ");
  //Serial.print(gainIndexVolt);
  //Serial.print("/ Vint:");
  //Serial.print(ads0_results23);
  //Serial.print("/ V 2-3: "); Serial.print(voltage23,3); Serial.print("V/ ");
  Serial.print("/ I:");
  Serial.print(current,4);
  
  Serial.print("/ temp"); Serial.print(tempF);
  Serial.print(" Batt:"); Serial.print(battV,2);
  Serial.print(" Total Logs: "); Serial.println(logCount);
  Serial.print ("Accel: ");
  Serial.print("X: "); Serial.print(accelXMax); Serial.print("  T:"); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accelYMax); Serial.print("  T:"); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accelZMax); Serial.print("  T:"); Serial.print(" count:"); Serial.print(accelCount); Serial.print(" maxcount:");Serial.print(accelZMaxcount); Serial.print("  ");Serial.println("m/s^2 ");

  accelXMax = 0;
  accelYMax = 0;
  accelZMax = 0;
  accelWindowTime = 0.0;
  accelCount = 0;
  accelZMaxcount =0;

  //Serial.println(results01);
  //Serial.println(gainIndexVolt);

  // Success!
  blinkPixel(0, 128, 128);       //  GREEN for success
  }


  
}

void measureVoltage(void){

    if(!digitalRead(accelPin)){
    //need to make this only work if V is the only measurement avaliable
    voltageTrigRaw = analogRead(trigThresPin);
    vStep = 0.125*((4*voltageTrigRaw/4095)*(4*voltageTrigRaw/4095)*(4*voltageTrigRaw/4095));
  }

    if(ch2on){
    ads0_results23 = ads.readADC_Differential_2_3();
    results23 = ads.readADC_Differential_0_1();

    voltage01actual = (ads0_results23 * multiplier) / 1000.0f;
    voltage01       = voltage01actual * vScale;

    voltage23       = ((results23 * multiplier) / 1000.0f) * vScale;
  
  }else if(autorange){

    if (firstVoltRunAuto) {
      gainIndexVolt = kNumGainLevels - 1; // start at max gain
      firstVoltRunAuto  = false;
      firstVoltRunMan  = true;
      ads.setGain(kGainLevels[gainIndexVolt]);
      (void)readRaw23();                  // throw away first sample after gain set
    }
    
  
    ads0_results23 = ads.getLastConversionResults();
    int absCounts = abs(ads0_results23);

    bool nearLow  = absCounts < (ADC_COUNT_LOW_THRESHOLD + ADC_GUARD_BAND);
    bool nearHigh = absCounts > (ADC_COUNT_HIGH_THRESHOLD - ADC_GUARD_BAND);
    
    // set current gain
    ads.setGain(kGainLevels[gainIndexVolt]);

    // get a fresh conversion at THIS gain
    ads0_results23 = readRaw23();

    // autorange decision
    if (abs(ads0_results23) > ADC_COUNT_HIGH_THRESHOLD && gainIndexVolt > 0) {
      --gainIndexVolt;
      ads.setGain(kGainLevels[gainIndexVolt]);
      (void)readRaw23();          // discard one sample after changing gain
      //Serial.print("range+");
      return;                     // don't compute volts from the pre-change sample
    }

    if (abs(ads0_results23) < ADC_COUNT_LOW_THRESHOLD && gainIndexVolt < (kNumGainLevels - 1)) {
      ++gainIndexVolt;
      ads.setGain(kGainLevels[gainIndexVolt]);
      (void)readRaw23();          // discard one sample after changing gain
      //Serial.print("range-");
      return;                     // same idea
    }

    // now results01 definitely corresponds to current gainIndexVolt
    voltage01actual = (ads0_results23 * kGainFactors[gainIndexVolt]) / 1000.0f;
    voltage01 = voltage01actual * vScale;
  }else{
        if (firstVoltRunMan) {
      firstVoltRunMan  = false;
      firstVoltRunAuto  = true;
      ads.setGain(GAIN_ONE);
      ads2.setGain(GAIN_SIXTEEN);
      (void)readRaw23();
      (void)ads2ReadRaw23();                  // throw away first sample after gain set
      ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/true);
      ads2.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/true);
    }
    
    multiplier = 2.0F; 
    ads0_results23  = ads.getLastConversionResults();

    voltage01actual = (ads0_results23*multiplier)/1000;
    voltage01 = voltage01actual*vScale; //Convert to voltage


 }
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
  display.setRotation(0); //rotates text on OLED 1=90 degrees, 2=180 degrees
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
  pinMode(accelPin, INPUT_PULLUP);
  pinMode(currentEnable, INPUT_PULLUP);
  pinMode(trigThresPin, INPUT);
  pinMode(autorangePin, INPUT_PULLUP);
  pinMode(voltReadPin, INPUT_PULLUP);
  pinMode(markButton, INPUT_PULLUP);
  pinMode(battPin, INPUT);
  pinMode(slowLogPin, INPUT_PULLUP);
  pinMode(screenEnable, INPUT_PULLUP);

  
  //ads.setGain(GAIN_TWO);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setDataRate(RATE_ADS1015_3300SPS); //Fast as possible


  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS 1.");
    while (1);  
  }

    if (!ads2.begin(0x49)) {
    Serial.println("Failed to initialize ADS 2 .");
    while (1);  
  }

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("Ooops, no ADXL343 detected ... Check your wiring!");
    while(1);
  }

    /* Set the range to whatever is appropriate for your project */
  // accel.setRange(ADXL343_RANGE_16_G);
  // accel.setRange(ADXL343_RANGE_8_G);
  accel.setRange(ADXL343_RANGE_4_G);
  // accel.setRange(ADXL343_RANGE_2_G);
  accel.setDataRate(ADXL343_DATARATE_1600_HZ);
  
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
  logfile.println("Delta uS,V Ch1,V Ch2, I, X, Y, Z, Temp, RTC Stamp");
  logfile.flush();

  Serial.print("Logging to: ");
  Serial.println(filename);


  }else{
    logON = 0;
  }
  
  if(!digitalRead(ch2Enable)){
    ch2on = true;    
  }else{
    ch2on = false;
    voltage23 = 0.0;
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/true);
    ads2.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/true);

  }

 Serial.print("Setup Complete");
  
}

void loop(void){
  
  if(digitalRead(autorangePin)){
      autorange = 1;
      //Serial.print("autorange on");
    }else{
      autorange = 0;
      //Serial.print("autorange off");
  }

  if(digitalRead(slowLogPin)){
    logInterval = 60000;
    }else{
    logInterval = 1000;
  }

  if(digitalRead(screenEnable)){
    screenInhibit = 0;
    screenInhibitPrevious = 0;
    }else{ //This section isn't working. Not sure why.
      screenInhibit = 1;
      if(screenInhibitPrevious = 0){
        display.setTextSize(2);
        display.setRotation(0); //rotates text on OLED 1=90 degrees, 2=180 degrees
        display.setTextColor(SSD1306_WHITE);
    // Splash screen
        display.setCursor(0, 0);
        display.clearDisplay();
        display.print("Inhibit");
        display.display();
        
    }
    screenInhibitPrevious = 1;
  }
  
  prevVoltage = voltage01;

  battRaw = analogRead(battPin);
  battV = (battRaw/4095)*3.3*2;




  if(digitalRead(markButton)){
    manualLog = 0; //no log, because button is pulled up
  }else{
    manualLog = 1; //yes log
    logCount++;
  }

    float tempC = readNTCTemperatureC(
      A3,        // analog pin
      12,        // ADC bits
      3.3,       // Vref
      10000.0,   // fixed resistor (10k)
      10000.0,   // NTC nominal resistance
      25.0,      // nominal temp
      3950.0     // beta value
  );

    tempF = (tempC*9/5)+32;



  
  //results01 = ads.readADC_Differential_2_3(); // integer from ADS

  //Serial.print("accel start");
  //This is where we read acceleration.
  
  if(digitalRead(accelPin)){
  
  readAccel();
  
  }else{
    accelX = 0;
    accelX = 0;
    accelX = 0;
  }

  //Serial.print("accel stop");


  
  //This is where the voltage reading starts

 if(digitalRead(voltReadPin)){
    measureVoltage();
  }else{
    voltage01 = 0.0;
  }

  if(digitalRead(currentEnable)){
     
    ads1_results23 = ads2.getLastConversionResults();
    current = ((ads1_results23*-0.125)/1000)*10;
    }else{
      current=0;
  }
  

  if(voltage01>vMax01){
    vMax01 = voltage01;
    trigFlag = 1;
  }
  if(voltage01<vMin01){
    vMin01 = voltage01;
    trigFlag = 1;
  }

    if(current>iMax){
    iMax = current;
    trigFlag = 1;
  }
    if(current<iMin){
    iMin = current;
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

  if(abs(current) > 0.025){
    if(current > lastLoggedI+0.1){
      trigFlag = 1;
    }
    if(current < lastLoggedI-0.1){
      trigFlag = 1;
    }
  }  




  if(!digitalRead(ch2Enable)){
      if(voltage23>vMax23){
        vMax23 = voltage23;
        trigFlag = 1;
      }
      if(voltage23<vMin23){
        vMin23 = voltage23;
        trigFlag = 1;
      }
    if(abs(voltage23) > noiseThreshold){ 
        if(voltage23 > lastLoggedV23+0.1){
          trigFlag = 1;
        }
        if(voltage23 < lastLoggedV23-0.1){
          trigFlag = 1;
        }
      }
  }

  
  if(trigFlag) { //trigger when voltage is more or less than vStep from the last logged V
    writeTrigger = true;
    lastLoggedV01 = voltage01;
    lastLoggedV23 = voltage23;
    lastLoggedI = current;
    accelLastLoggedX = accelX;
    accelLastLoggedY = accelY;
    accelLastLoggedZ = accelZ;
    trigFlag = 0;
  }else{
    writeTrigger = false;
    //blinkPixel(32, 0, 32);       //  Purple for running
  }

  // If trigger is active, capture to RAM
  if (writeTrigger && logON) {
    captureSample(voltage01, voltage23, current, accelX, accelY, accelZ);
    logCount++;
  }else{
    outputs(voltage01, voltage23);
    if((millis() - lastLog > logInterval) || manualLog){
      if(!manualLog){
      lastLog = millis();
      }
      captureSample(voltage01, voltage23, current, accelX, accelY, accelZ);
      if(logON){
      flushBufferToSD();
      logCount++;
      manualLog = false;
      }
    }
    
    writeTrigger = false;
  }

  // Detect trigger falling edge → flush
  if (((triggerPrev == true && writeTrigger == false) || manualLog == true) && logON) {
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
  display.print("V:");
  display.print(voltage01,2);
  
  display.setCursor(0,16);
  
  display.print("mA:");
  if(digitalRead(currentEnable)){
  display.print((current*1000),0);
  }else{
    display.print("- #4");
  }

  display.setTextSize(1);
  
  if(!digitalRead(ch2Enable)){
  display.setCursor(0,16);
  display.print("V2:");
  display.print(voltage23,2);
  }

  if(digitalRead(accelPin)){
    display.setTextSize(1);
    display.setCursor(108,0);
    display.print("g1");
    display.setCursor(100,8);
    display.print(accelThres);
  }
  

  display.setTextSize(1);
  display.setCursor(0,32);
  display.print ("#logs:");
  display.print (logCount);

  display.setCursor(64,32);
  display.print ("trig");
  display.print (vStep);

  timemS = millis();
  display.setCursor(0,40);
  display.print ("Runtime (s): ");
  display.print ((timemS/1000),0);
  
  if (logError) {
    //display.setTextSize(2);
    display.setCursor(0,48);
    display.print ("SD ERROR");
  }else if(!logON){
    //display.setTextSize(2);
    display.setCursor(0,48);
    display.print ("SD OFF #8");  
  }else{
    //display.setTextSize(2);
    display.setCursor(0,48);
    display.print ("SD GOOD");
  }

  display.setCursor(64,48);
  display.print ("Temp");
  display.print (tempF);
  display.print ("F");


  
  display.setCursor(0,56);
  display.print ("Range: ");
  if(autorange){
    display.print ("Auto");
  }else{
    display.print ("F #5");
  }

  display.setCursor(72,56);
  display.print("Batt: ");
  display.print(battV,1);

  display.setCursor(0,56);  

  display.display(); // update the OLED with all the drawn content
}

