#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include "SdFat.h"
#include <Adafruit_NeoPixel.h>


Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

// track timing
unsigned long lastLog = 0;

bool writeTrigger = 0;
float prevResults = 0.0;

#define SD_CS_PIN 23
const int PIXEL_PIN = 17;       // Onboard NeoPixel
const int PIXEL_COUNT = 1;

Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

SdFat SD;
FsFile logfile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);


  float data1 = 1.234;
  float data2 = 5.678;

  int16_t results;

  float   multiplier = 3.0F;    /* ADS1015 @ +/- 6.144V gain (12-bit results) */


// --- Helper: blink NeoPixel a color for a short time ---
void blinkPixel(uint8_t r, uint8_t g, uint8_t b, int duration = 60) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
  delay(duration);
  pixel.clear();
  pixel.show();
}

void setup(void)
{
  Serial.begin(115200);
    delay(500);
 Serial.println("Hello!");

  Serial.println("Getting differential reading from AIN0 (P) and AIN1 (N)");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)

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
      blinkPixel(255, 0, 0, 200);   // Red repeating = SD unavailable
    }
  }
  
  }

void loop(void)
{
  prevResults = results;
  
  results = ads.readADC_Differential_0_1();

  if(abs((results*multiplier))+0.25 > prevResults || abs((results*multiplier))-0.5 < prevResults){
    writeTrigger = true;
  }

  logCSV(results, multiplier);
  
  delay(500);
}

// --- CSV logging function (call every loop) ---
void logCSV(float data1, float data2) {
  unsigned long now = millis();
  if (!writeTrigger && now - lastLog < 1000) return;
  lastLog = now;
  writeTrigger = 0;

  logfile = SD.open("/data.csv", FILE_WRITE);

  if (!logfile) {
    Serial.println("ERROR: Unable to open data.csv");
    blinkPixel(255, 0, 0);     // RED for failure
    return;
  }

  // Attempt to write
  
  logfile.print("Differential (mV)");
  logfile.print(",");
  logfile.print((data1 * data2), 1);
  logfile.print(",");
  logfile.println(millis());

  // Force flush to SD
  logfile.flush();
  logfile.close();

  Serial.print("Differential: "); Serial.print(results); Serial.print("("); Serial.print(results * multiplier); Serial.println("mV)");


  // Success!
  blinkPixel(0, 255, 0);       //  GREEN for success
}


