#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

// track timing
unsigned long lastLog = 0;

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
float   multiplier = 1.0F;    /* ADS1015 @ +/- 2.048V gain (12-bit results) */

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




const int PIXEL_COUNT = 1;

Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

SdFat SD;
File32 intervalFile;
File32 triggerFile;

char intervalFilename[32];
char triggerFilename[32];

SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

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
  File32 logfile = SD.open(triggerFilename, FILE_WRITE);
  if (!logfile) {
    Serial.println("ERROR: Could not open capture file");
    return;
  }

  //Serial.print("Writing ");
  //Serial.print(sampleCount);
  //Serial.println(" samples to SD...");

  for (size_t i = 0; i < sampleCount; i++) {
    logfile.print(buffer[i].t,1);
    logfile.print(",");
    logfile.print(buffer[i].d1, 3);
    logfile.print(",");
    logfile.print(buffer[i].d2, 1);
    logfile.print(",");
    logfile.println(buffer[i].td, 1);
  }

  logfile.flush();
  logfile.close();

  //Serial.println("Done writing.");
    // Success!
  blinkPixel(0, 255, 0);       //  GREEN for success

  // Reset buffer
  sampleCount = 0;
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


  pinMode(sdEnable, INPUT_PULLUP);
  pinMode(ch2Enable, INPUT_PULLUP);
  pinMode(serialEnable, INPUT_PULLUP);
  pinMode(trigThres, INPUT);

  
  ads.setGain(GAIN_TWO);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
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

    // ---- Create interval logging file ----
  makeUniqueFilename("log", ".csv", intervalFilename, sizeof(intervalFilename));
  Serial.print("Interval log file: ");
  Serial.println(intervalFilename);

  intervalFile = SD.open(intervalFilename, FILE_WRITE);
  if (!intervalFile) {
    Serial.println("Failed to create interval log!");
    while (1);
  }
  //intervalFile.println("time_ms,data1,data2,data3");  // example header
  //intervalFile.flush();

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
  }else{
  results01  = ads.getLastConversionResults();
  }

  voltage01 = ((results01*multiplier)/1000)*vScale; //Convert to voltage


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
    writeTrigger = false;
  }

  // Detect trigger falling edge → flush
  if (triggerPrev == true && writeTrigger == false) {
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

// --- Regular CSV logging / Serial Print function (call every loop) ---
void logCSV(float data1, float data2) {
  unsigned long now = millis();
  if (now - lastLog < 2000) return;
  lastLog = now;

  

  if(logON){

  intervalFile = SD.open(intervalFilename, FILE_WRITE);

  if (!intervalFilename) {
    Serial.println("ERROR: Unable to open data.csv");
    logError = true;
    blinkPixel(255, 0, 0);     // RED for failure
    return;
  }
  logError = false;

  intervalFile.print(millis());
  intervalFile.print(",");
  intervalFile.print((data1), 1);
  intervalFile.print(",");
  intervalFile.println((data2), 1);
  // Force flush to SD
  intervalFile.flush();
  intervalFile.close();
  }


  updateDisplay();


  if(digitalRead(serialEnable)){
  Serial.print("T:"); Serial.print(millis());
  Serial.print("/ V 0-1: "); Serial.print(voltage01,3); Serial.print("V/ ");
  Serial.print("V 2-3: "); Serial.print(voltage23,3); Serial.print("V/ ");
  Serial.print("Total Logs: "); Serial.println(triggerCount);
  } 


  // Success!
  blinkPixel(0, 128, 128);       //  GREEN for success
}


