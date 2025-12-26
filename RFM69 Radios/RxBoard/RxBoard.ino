#include <SPI.h>
#include <RH_RF69.h>

// ==================
// RFM69 CONFIG
// ==================
  #define RFM69_CS   16
  #define RFM69_INT  21
  #define RFM69_RST  17
  #define RF69_FREQ   915.0

RH_RF69 rf69(RFM69_CS, RFM69_INT);

// ==================
// SETUP
// ==================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

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
  rf69.setTxPower(13, true);

  Serial.println("RFM69 receiver ready");
}

// ==================
// LOOP
// ==================
void loop() {
  if (rf69.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf69.recv(buf, &len)) {
      if (len == 2) {
        int16_t raw =
          (int16_t)(buf[0] | (buf[1] << 8));

        Serial.print("Time:");
        Serial.print(millis());
        Serial.print(" Received raw: ");
        Serial.println(raw);
      } else {
        Serial.print("Unexpected length: ");
        Serial.println(len);
      }
    }
  }
}
