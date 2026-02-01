#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;
const int dacOutPin = A0;  // DAC output pin on Nano R4
const int relayPin = 2;

// Timing control
unsigned long lastSerialTime = 0;
unsigned long lastAnalogTime = 0;
const unsigned long serialInterval = 1000; // 1 Hz
const unsigned long analogInterval = 5;   // 200 Hz

// Auto-ranging thresholds
adsGain_t currentGainVolt = GAIN_ONE;  // ±4.096V
adsGain_t currentGainAmps = GAIN_TWO;  // example default for current
float gainMaxVolt, gainMaxAmps;

const float shunt = 0.1; // example value in ohms

float prevVoltage = 0.0;
float prevOutVoltage = 0.0;

float voltage = 0.0;
float amps = 0.0;

bool relayState = 0;




void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin();

  // Keep trying until ADS1115 is found
  while (!ads.begin(0x48,&Wire1)) {
    Serial.println("ADS1115 not found. Retrying... (restart?)");
    delay(1000);
  }
  
  Serial.println("ADS1115 connected.");

  ads.setGain(currentGainVolt);
  gainMaxVolt = getGainMax(currentGainVolt);
  ads.setDataRate(RATE_ADS1115_128SPS);

  pinMode(relayPin, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  float shunt = 0.1;

  float prevVoltage = voltage;
  float prevAmps = amps;

  // --- Read differential voltage and current ---
  int16_t rawV = ads.readADC_Differential_0_1();
  int16_t rawI = ads.readADC_Differential_2_3();

  voltage = ads.computeVolts(rawV);
  amps = ads.computeVolts(rawI) / shunt;

  // --- Autorange both channels ---
  autoRangeChannel(fabs(voltage), currentGainVolt, gainMaxVolt, "Voltage");
  autoRangeChannel(fabs(amps * shunt), currentGainAmps, gainMaxAmps, "Current"); 
  // using amps*shunt so logic matches ADC input voltage range

  


  if(fabs(voltage*7.311)>2.5){
    if(relayState == 0){
      Serial.println("relay On");
      relayState = 1;
      digitalWrite(relayPin, HIGH);
    }
  }else{
    if(relayState == 1){
      digitalWrite(relayPin, LOW);
      relayState = 0;
      Serial.println("relay Off");
    }
    
    //Serial.println("relay off");
  }


/*
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
*/
  
  
  // 1Hz serial output update
  if ((currentMillis - lastSerialTime >= serialInterval) || (fabs(voltage*7.311)+0.25 < fabs(prevOutVoltage)|| fabs(voltage*7.311)-0.25 > fabs(prevOutVoltage))) {
    lastSerialTime = currentMillis;
    prevOutVoltage = voltage*7.311;
    Serial.print("VDC: ");
    Serial.print(voltage*7.311, 6); //7.3147
    Serial.print(" I: ");
    Serial.print(amps,4);
    Serial.print(" T(ms):");
    Serial.println(millis());
    //Serial.print(" V, Gain range: ±");
    //Serial.println(gainMax, 3);
  }
}

// --- Helper functions ---
adsGain_t stepUpGain(adsGain_t g) {
  switch (g) {
    case GAIN_TWOTHIRDS: return GAIN_ONE;
    case GAIN_ONE:       return GAIN_TWO;
    case GAIN_TWO:       return GAIN_FOUR;
    case GAIN_FOUR:      return GAIN_EIGHT;
    case GAIN_EIGHT:     return GAIN_SIXTEEN;
    default:             return g;
  }
}

adsGain_t stepDownGain(adsGain_t g) {
  switch (g) {
    case GAIN_SIXTEEN:   return GAIN_EIGHT;
    case GAIN_EIGHT:     return GAIN_FOUR;
    case GAIN_FOUR:      return GAIN_TWO;
    case GAIN_TWO:       return GAIN_ONE;
    case GAIN_ONE:       return GAIN_TWOTHIRDS;
    default:             return g;
  }
}

float getGainMax(adsGain_t g) {
  switch (g) {
    case GAIN_TWOTHIRDS: return 6.144;
    case GAIN_ONE:       return 4.096;
    case GAIN_TWO:       return 2.048;
    case GAIN_FOUR:      return 1.024;
    case GAIN_EIGHT:     return 0.512;
    case GAIN_SIXTEEN:   return 0.256;
    default:             return 4.096;
  }
}

// --- Autorange logic for either channel ---
void autoRangeChannel(float absValue, adsGain_t &currentGain, float &gainMax, const char *label) {
  if (absValue > 0.95 * gainMax) {
    adsGain_t newGain = stepDownGain(currentGain);
    if (newGain != currentGain) {
      currentGain = newGain;
      ads.setGain(currentGain);
      gainMax = getGainMax(currentGain);
      Serial.print(label); Serial.print(": Lowering gain to ±");
      Serial.println(gainMax);
    }
  } else if (absValue < 0.2 * gainMax) {
    adsGain_t newGain = stepUpGain(currentGain);
    if (newGain != currentGain) {
      currentGain = newGain;
      ads.setGain(currentGain);
      gainMax = getGainMax(currentGain);
      Serial.print(label); Serial.print(": Increasing gain to ±");
      Serial.println(gainMax);
    }
  }
}