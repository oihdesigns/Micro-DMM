#include <RH_ASK.h>
#include <SPI.h>

RH_ASK driver(2000, 11, 12); // Create an instance of the driver

void setup() {
  Serial.begin(115200);

  if (!driver.init()) {
    Serial.println("RH_ASK init failed");
    while (1);
  }

  Serial.println("RX ready.");
}

void loop() {
  uint8_t buf[32];
  uint8_t buflen = sizeof(buf);

  if (driver.recv(buf, &buflen)) {
    // Convert to null-terminated string
    buf[buflen] = '\0';

    Serial.print("T:");
    Serial.print(millis());
    Serial.print("/ Received: ");
    Serial.println((char*)buf);
  }
}
