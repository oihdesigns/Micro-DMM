#include <Wire.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <Adafruit_ADS1X15.h>

// ==================
// RFM69 CONFIG
// ==================
  #define RFM69_CS   16
  #define RFM69_INT  21
  #define RFM69_RST  17
  #define RF69_FREQ   915.0

RH_RF69 rf69(RFM69_CS, RFM69_INT);

// ==================
// ADS1015
// ==================
Adafruit_ADS1015 ads;

// ==================
// SETUP
// ==================
void setup() {
  Serial.begin(115200);
  //while (!Serial) delay(10);

  // Reset RFM69
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 init failed");
    while (1);
  }

  rf69.setFrequency(RF69_FREQ);
  rf69.setTxPower(13, true);  // High power module

  Serial.println("RFM69 OK");

  if (!ads.begin()) {
    Serial.println("ADS1015 not found");
    while (1);
  }

  // ±2.048V range → good margin for 0–1V diff
  ads.setGain(GAIN_TWO);

  Serial.println("ADS1015 OK");
}

// ==================
// LOOP
// ==================
void loop() {
  int16_t raw = ads.readADC_Differential_0_1();

  // Send raw int16 as 2 bytes
  uint8_t payload[2];
  payload[0] = raw & 0xFF;
  payload[1] = (raw >> 8) & 0xFF;

  rf69.send(payload, sizeof(payload));
  rf69.waitPacketSent();

  Serial.print("Sent raw: ");
  Serial.println(raw);

  delay(1000);
}
