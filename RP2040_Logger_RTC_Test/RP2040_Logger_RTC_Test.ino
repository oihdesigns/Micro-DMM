#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include "SdFat.h"

// --------------------
// Objects
// --------------------
RTC_PCF8523 rtc;
Adafruit_ADS1015 ads;
SdFat sd;
File32 logfile;

// --------------------
// Pins
// --------------------
#define SD_CS_PIN 10
const int TRIGGER_PIN = 6;   // Write-to-SD trigger (falling edge)
bool lastTriggerState = HIGH;

SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16));


// --------------------
// Timing
// --------------------
unsigned long lastPrint = 0;

// --------------------
// Filename Storage
// --------------------
char filename[40];

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

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  // Start RTC
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
    while (1);
  }

  // Print current time
  DateTime now = rtc.now();
  Serial.print("RTC Time: ");
  Serial.println(now.timestamp());

  // Start ADS1015
  ads.begin();
  ads.setGain(GAIN_ONE);  // ±4.096V → differential 0–1V is safe

  // Start SD card
  if (!sd.begin(config)) {
    Serial.println("SD init failed!");
    while (1);
  }

  // Create unique CSV filename with timestamp
  sprintf(filename, "log_%04d%02d%02d_%02d%02d%02d.csv",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());

  logfile = sd.open(filename, FILE_WRITE);
  if (!logfile) {
    Serial.println("Could not create log file!");
    while (1);
  }

  // First row: human-readable timestamp
  logfile.print("Log Start Time,");
  logfile.println(now.timestamp());
  logfile.println("Timestamp,Voltage(V)");
  logfile.flush();

  Serial.print("Logging to: ");
  Serial.println(filename);
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  unsigned long ms = millis();

  // -----------------------------
  // Read ADS1015 differential
  // -----------------------------
  int16_t raw = ads.readADC_Differential_0_1();
  float voltage = raw * 0.003;   // ADS1015 = 3mV/LSB in GAIN_ONE

  // -----------------------------
  // Print once per second
  // -----------------------------
  if (ms - lastPrint >= 1000) {
    //DateTime now = rtc.now();
    printNow(&Serial);
    //Serial.print(now.timestamp());
    Serial.print("  |  V = ");
    Serial.println(voltage, 4);
    lastPrint = ms;
  }

  // -----------------------------
  // Look for falling edge on D6
  // -----------------------------
  bool currentTrigger = digitalRead(TRIGGER_PIN);

  if (lastTriggerState == HIGH && currentTrigger == LOW) {
    // Falling edge detected → log to SD
    DateTime now = rtc.now();

    logfile.print(now.timestamp());
    logfile.print(",");
    logfile.println(voltage, 4);
    logfile.flush();

    Serial.println("Wrote to SD.");
  }

  lastTriggerState = currentTrigger;
}
