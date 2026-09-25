#include <Wire.h>

// Rev C: JP2 must be LOW (pins 1-2). Match J4 VIO to the host logic voltage.
constexpr uint8_t kAddress = 0x41;
constexpr float kGain[8] = {
  1.0f, 2.5f, 5.02f, 10.09f, 25.04809619f, 49.78048780f, 101.0f, 1001.0f
};

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool beginGainControl() {
  // Clear the power-on 0xFF latch BEFORE enabling the outputs, selecting unity.
  if (!writeRegister(0x01, 0x00)) return false;
  if (!writeRegister(0x03, 0xF0)) return false;  // Outputs; unused P3 stays low.
  return writeRegister(0x50, 0x40);             // Disable internal pull-ups.
}

bool setGainCode(uint8_t code) {
  if (code > 7) return false;
  // Update all three selection bits together, not in separate writes.
  if (!writeRegister(0x01, code)) return false;
  delay(20);  // Initial settling allowance; validate with the actual ADC/load.
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  delay(50);  // Midpoint filter has a nominal 5 ms time constant.
  if (!beginGainControl() || !setGainCode(0)) {
    Serial.println("Board initialization failed; check I2C and power.");
    while (true) delay(1000);
  }
  Serial.println("Gain = 1. Send a digit 0-7 to select a gain.");
}

void loop() {
  if (!Serial.available()) return;
  const int ch = Serial.read();
  if (ch < '0' || ch > '7') return;
  const uint8_t code = static_cast<uint8_t>(ch - '0');
  if (setGainCode(code)) {
    Serial.print("Nominal gain = ");
    Serial.println(kGain[code], 6);
  } else {
    Serial.println("Gain write failed; discard measurement and reinitialize.");
  }
}
