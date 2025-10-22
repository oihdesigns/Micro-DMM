#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

const int DAC_PIN = A0;  // Nano R4 DAC output (true 12-bit)
const float INPUT_MIN = -5.0;
const float INPUT_MAX =  5.0;
const float DAC_MIN = 0.0;
const float DAC_MAX = 5.0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("ADS1115 Differential to DAC Mirror");

  if (!ads.begin(0x48,&Wire1)) {
    Serial.println("Failed to find ADS1115. Check wiring!");
    while (1);
  }

  // ±6.144V range — allows full ±5V input
  ads.setGain(GAIN_TWOTHIRDS);

  analogWriteResolution(12);  // 12-bit DAC (0–4095)
  Serial.println("Setup complete.");
}

void loop() {
  // Read differential input
  int16_t raw = ads.readADC_Differential_0_1();
  float volts = raw * 0.0001875;  // 0.1875 mV/bit for GAIN_TWOTHIRDS

  // Remap ±5V input to 0–5V DAC output
  float mapped = mapVoltage(volts, INPUT_MIN, INPUT_MAX, DAC_MIN, DAC_MAX);

  // Clip to safe range
  if (mapped < 0.0) mapped = 0.0;
  if (mapped > 5.0) mapped = 5.0;

  // Convert to 12-bit DAC value
  int dacValue = (int)(mapped / 5.0 * 4095);
  analogWrite(DAC_PIN, dacValue);

  // Print to serial
  Serial.print("Diff: ");
  Serial.print(volts*10, 4);
  Serial.print(" V  ->  DAC: ");
  Serial.print(mapped, 4);
  Serial.print(" V  (");
  Serial.print(dacValue);
  Serial.println("/4095)");

  delay(1000);
}

float mapVoltage(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
