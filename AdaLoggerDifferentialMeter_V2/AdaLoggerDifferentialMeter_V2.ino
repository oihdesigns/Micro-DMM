#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_NeoPixel.h>


Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

// track timing
unsigned long lastLog = 0;

// ========================
// BUFFER SETTINGS
// ========================
const size_t MAX_SAMPLES = 20000;   // adjustable
struct Sample {
  float d1;
  float d2;
  uint32_t t;   // optional: timestamp in microseconds
};
Sample buffer[MAX_SAMPLES];
volatile size_t sampleCount = 0;

// Trigger state tracking
bool triggerPrev = false;


float voltage01 = 0.0;
float voltage23 = 0.0;
float vScale = 69.669;



bool writeTrigger = 0;
bool trigFlag = 0;
float prevVoltage = 0.0;
float vStep = 0.25;
float noiseThreshold = 0.5;
float lastLoggedV01 = 0.25;
float vMax01 = 0.0;
float vMin01 = 0.0;

float lastLoggedV23 = 0.25;
float vMax23 = 0.0;
float vMin23 = 0.0;


#define SD_CS_PIN 23
const int PIXEL_PIN = 17;       // Onboard NeoPixel
const int sdSurpress = 4;


const int PIXEL_COUNT = 1;

Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

SdFat SD;
FsFile logfile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);


  float data1 = 1.234;
  float data2 = 5.678;

  int16_t results01;
  int16_t results23;

  //float   multiplier = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results) */
  float   multiplier = 0.5F;    /* ADS1015 @ +/- 2.048V gain (12-bit results) */


// --- Helper: blink NeoPixel a color for a short time ---
void blinkPixel(uint8_t r, uint8_t g, uint8_t b, int duration = 10) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
  delay(duration);
  pixel.clear();
  pixel.show();
}

// ========================
// FUNCTION: capture a sample into RAM
// ========================
void captureSample(float data1, float data2) {
  if (sampleCount >= MAX_SAMPLES) return;   // buffer full - ignore or handle differently

  buffer[sampleCount].d1 = data1;
  buffer[sampleCount].d2 = data2;
  buffer[sampleCount].t  = micros();       // optional timestamp

    // Success!
  //blinkPixel(0, 0, 255);       //  Blue for success

  sampleCount++;
}

// ========================
// FUNCTION: flush RAM buffer to SD
// ========================
void flushBufferToSD() {
  FsFile logfile = SD.open("/capture.csv", FILE_WRITE);
  if (!logfile) {
    Serial.println("ERROR: Could not open capture.csv");
    return;
  }

  Serial.print("Writing ");
  Serial.print(sampleCount);
  Serial.println(" samples to SD...");

  for (size_t i = 0; i < sampleCount; i++) {
    logfile.print(buffer[i].t);
    logfile.print(",");
    logfile.print(buffer[i].d1, 3);
    logfile.print(",");
    logfile.println(buffer[i].d2, 1);
  }

  logfile.flush();
  logfile.close();

  Serial.println("Done writing.");
    // Success!
  blinkPixel(0, 255, 0);       //  GREEN for success

  // Reset buffer
  sampleCount = 0;
}

void setup(void)
{
  Serial.begin(115200);
    delay(500);
 Serial.println("Hello!");

  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  pinMode(sdSurpress, INPUT_PULLUP);

  
  ads.setGain(GAIN_FOUR);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setDataRate(RATE_ADS1015_3300SPS); //Fast as possible


  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);  
  }
  
  pixel.begin();
  pixel.clear();
  pixel.show();

  if (!SD.begin(config)) {
    Serial.println("SD init failed!");
    while (1) {
      blinkPixel(128, 0, 0, 200);   // Red repeating = SD unavailable
    }
  }
  
  }

void loop(void)
{
  prevVoltage = voltage01;

  
  results01 = ads.readADC_Differential_0_1(); // integer from ADS
  results23 = ads.readADC_Differential_2_3(); // integer from ADS

  voltage01 = ((results01*multiplier)/1000)*vScale; //Convert to voltage
  voltage23 = ((results23*multiplier)/1000)*vScale; //Convert to voltage

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

  
  if(trigFlag) { //trigger when voltage is more or less than vStep from the last logged V
    writeTrigger = true;
    lastLoggedV01 = voltage01;
    lastLoggedV23 = voltage23;
    trigFlag = 0;
  }else{
    writeTrigger = false;
    blinkPixel(64, 0, 64);       //  Purple for running
  }

  // If trigger is active, capture to RAM
  if (writeTrigger && digitalRead(sdSurpress)) {
    captureSample(voltage01, voltage23);
  }else{
    logCSV(voltage01, voltage23);
    writeTrigger = false;
  }

  // Detect trigger falling edge → flush
  if (triggerPrev == true && writeTrigger == false) {
    flushBufferToSD();
  }

  triggerPrev = writeTrigger;
  
  //delay(500);
}



// --- Regular CSV logging / Serial Print function (call every loop) ---
void logCSV(float data1, float data2) {
  unsigned long now = millis();
  if (now - lastLog < 1000) return;
  lastLog = now;

  logfile = SD.open("/data.csv", FILE_WRITE);

  if (!logfile) {
    Serial.println("ERROR: Unable to open data.csv");
    blinkPixel(255, 0, 0);     // RED for failure
    return;
  }

  // Attempt to write
  
  logfile.print(micros());
  logfile.print(",");
  logfile.print((data1), 1);
  logfile.print(",");
  logfile.println((data2), 1);


  // Force flush to SD
  logfile.flush();
  logfile.close();

  Serial.print("Differential: "); Serial.print(results01); Serial.print("("); Serial.print(voltage01,3); Serial.println("V)");


  // Success!
  blinkPixel(0, 128, 128);       //  GREEN for success
}


