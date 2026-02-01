#include <RTClib.h>
#include <Adafruit_ADS1X15.h>
#include "Adafruit_TinyUSB.h"
#include "SdFat.h"

// =============================
// Hardware Objects
// =============================
RTC_DS3231 rtc;
Adafruit_ADS1015 ads;
Adafruit_USBD_HID usb_hid;   // TinyUSB HID device
File32 logfile;              // Correct SD file type
SdFat SD;

// =============================
// Pins
// =============================
const int PIN_KEYBOARD = 5;   // active low
const int PIN_SDWRITE  = 6;   // active low

// Timing
unsigned long lastPrint = 0;

// Filename buffer
char filename[32];

// ==============================================
// Read differential voltage on ADS1015 A0-A1
// ==============================================
float readDiffVoltage() {
  int16_t raw = ads.readADC_Differential_0_1();
  float volts = raw * 0.003f;   // 3mV/LSB default gain
  return volts;
}

// ==============================================
// Write CSV header
// ==============================================
void writeHeader(const DateTime &now) {
  logfile.println("Timestamp,Voltage(V)");
  logfile.print("Start:");
  logfile.println(now.timestamp());
  logfile.flush();
}

// ==============================================
// Setup
// ==============================================
void setup() {
  pinMode(PIN_KEYBOARD, INPUT_PULLUP);
  pinMode(PIN_SDWRITE, INPUT_PULLUP);

  Serial.begin(115200);
  delay(1500);

  // ------------------
  // Initialize RTC
  // ------------------
  DateTime now;
  if (!rtc.begin()) {
    Serial.println("RTC not found!");
  } else {
    now = rtc.now();
  }

  // ------------------
  // ADS1015
  // ------------------
  ads.begin();

  // ------------------
  // SD Card
  // ------------------
  if (!SD.begin()) {
    Serial.println("SD initialization failed!");
  }

  snprintf(filename, sizeof(filename),
           "/%04d%02d%02d_%02d%02d%02d.csv",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  logfile = SD.open(filename, FILE_WRITE);

  if (!logfile) {
    Serial.println("Could not open log file!");
  } else {
    writeHeader(now);
    Serial.print("Logging to: ");
    Serial.println(filename);
  }

  // ------------------
  // Keyboard
  // ------------------
  usb_hid.begin();   // Enable HID device
  Keyboard.begin();  // Keyboard interface
}

// ==============================================
// Loop
// ==============================================
void loop() {

  unsigned long t = millis();

  // ------------------------------
  // Print once per second
  // ------------------------------
  if (t - lastPrint >= 1000) {
    lastPrint = t;
    float volts = readDiffVoltage();
    DateTime now = rtc.now();

    Serial.print(now.timestamp());
    Serial.print("  V = ");
    Serial.println(volts, 6);
  }

  // ------------------------------
  // If D5 low → type value to PC
  // ------------------------------
  if (digitalRead(PIN_KEYBOARD) == LOW) {
    float volts = readDiffVoltage();
    char buf[32];
    snprintf(buf, sizeof(buf), "%.6f\n", volts);
    Keyboard.print(buf);
    delay(300);  // debounce
  }

  // ------------------------------
  // If D6 low → write to SD card
  // ------------------------------
  if (digitalRead(PIN_SDWRITE) == LOW) {
    float volts = readDiffVoltage();
    DateTime now = rtc.now();

    if (logfile) {
      logfile.print(now.timestamp());
      logfile.print(",");
      logfile.println(volts, 6);
      logfile.flush();
      Serial.println("→ Wrote to SD");
    } else {
      Serial.println("SD error: logfile not open.");
    }

    delay(300);  // debounce
  }
}
