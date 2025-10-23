#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;
const int dacOutPin = A0;  // DAC output pin on Nano R4

// Timing control
unsigned long lastSerialTime = 0;
unsigned long lastAnalogTime = 0;
const unsigned long serialInterval = 1000; // 1 Hz
const unsigned long analogInterval = 5;   // 200 Hz

// Auto-ranging thresholds
adsGain_t currentGain = GAIN_ONE; // Start with ±4.096V
float gainMax = 4.096;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Keep trying until ADS1115 is found
  while (!ads.begin(0x48,&Wire1)) {
    Serial.println("ADS1115 not found. Retrying...");
    delay(1000);
  }
  Serial.println("ADS1115 connected.");

  ads.setGain(currentGain);
  ads.setDataRate(RATE_ADS1115_128SPS);
}

void loop() {
  unsigned long currentMillis = millis();

  // Read differential voltage
  int16_t raw = ads.readADC_Differential_0_1();
  float voltage = ads.computeVolts(raw);

  // Auto-ranging logic
  if (fabs(voltage) > 0.95 * gainMax) {
    adsGain_t newGain = stepDownGain(currentGain);
    if (newGain != currentGain) {
      currentGain = newGain;
      ads.setGain(currentGain);
      gainMax = getGainMax(currentGain);
      Serial.print("Lowering gain to range ±");
      Serial.println(gainMax);
    }
  } else if (fabs(voltage) < 0.2 * gainMax) {
    adsGain_t newGain = stepUpGain(currentGain);
    if (newGain != currentGain) {
      currentGain = newGain;
      ads.setGain(currentGain);
      gainMax = getGainMax(currentGain);
      Serial.print("Increasing gain to range ±");
      Serial.println(gainMax);
    }
  }

  // 100Hz DAC output update
  if (currentMillis - lastAnalogTime >= analogInterval) {
    lastAnalogTime = currentMillis;

    // Scale voltage to DAC range (0–5V → 0–4095 for 12-bit DAC)
    float scaledVoltage = (voltage + gainMax) * (5.0 / (2.0 * gainMax));
    scaledVoltage = constrain(scaledVoltage, 0.0, 5.0);

    int dacValue = (int)(scaledVoltage * 4095.0 / 5.0);
    analogWriteResolution(12);
    analogWrite(dacOutPin, dacValue);
  }

  // 1Hz serial output update
  if (currentMillis - lastSerialTime >= serialInterval) {
    lastSerialTime = currentMillis;
    Serial.print("VDC: ");
    Serial.println(voltage*7.311, 6); //7.3147
    //Serial.print(" V, Gain range: ±");
    //Serial.println(gainMax, 3);
  }
}

// Gain stepping helpers
adsGain_t stepUpGain(adsGain_t g) {
  switch (g) {
    case GAIN_TWOTHIRDS: return GAIN_ONE;
    case GAIN_ONE: return GAIN_TWO;
    case GAIN_TWO: return GAIN_FOUR;
    case GAIN_FOUR: return GAIN_EIGHT;
    case GAIN_EIGHT: return GAIN_SIXTEEN;
    default: return g;
  }
}

adsGain_t stepDownGain(adsGain_t g) {
  switch (g) {
    case GAIN_SIXTEEN: return GAIN_EIGHT;
    case GAIN_EIGHT: return GAIN_FOUR;
    case GAIN_FOUR: return GAIN_TWO;
    case GAIN_TWO: return GAIN_ONE;
    case GAIN_ONE: return GAIN_TWOTHIRDS;
    default: return g;
  }
}

float getGainMax(adsGain_t gain) {
  switch (gain) {
    case GAIN_TWOTHIRDS: return 6.144;
    case GAIN_ONE: return 4.096;
    case GAIN_TWO: return 2.048;
    case GAIN_FOUR: return 1.024;
    case GAIN_EIGHT: return 0.512;
    case GAIN_SIXTEEN: return 0.256;
    default: return 4.096;
  }
}
