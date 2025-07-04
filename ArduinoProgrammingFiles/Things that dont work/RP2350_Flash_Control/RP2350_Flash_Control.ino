#include <EEPROM.h>

const int EEPROM_SIZE = 512;  // You can increase up to 4096 on Nano ESP32

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to initialise EEPROM");
    while (true) { delay(1000); }
  }
  Serial.println("EEPROM ready. Use 'R addr' to read, 'W addr val' to write.");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() < 2) return;

    char cmd = toupper(line.charAt(0));
    int idx = 1;
    // skip whitespace
    while (idx < line.length() && isspace(line[idx])) idx++;

    // parse first integer (address)
    int addr = line.substring(idx).toInt();

    if (cmd == 'R') {
      if (addr < 0 || addr >= EEPROM_SIZE) {
        Serial.println("Error: address out of range");
      } else {
        uint8_t val = EEPROM.read(addr);
        Serial.printf("R[%d] = %u\n", addr, val);
      }
    }
    else if (cmd == 'W') {
      // find second integer
      // move index past the first number
      while (idx < line.length() && (isdigit(line[idx]) || line[idx]=='-' )) idx++;
      // skip whitespace
      while (idx < line.length() && isspace(line[idx])) idx++;
      int val = line.substring(idx).toInt();
      if (addr < 0 || addr >= EEPROM_SIZE || val < 0 || val > 255) {
        Serial.println("Error: address or value out of range");
      } else {
        EEPROM.write(addr, (uint8_t)val);
        if (EEPROM.commit()) {
          Serial.printf("Wrote %u to %d\n", val, addr);
        } else {
          Serial.println("Error: commit failed");
        }
      }
    }
    else {
      Serial.println("Unknown command. Use 'R addr' or 'W addr val'");
    }
  }
}
