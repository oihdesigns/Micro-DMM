//#include <Wire.h>
#include <Adafruit_ADS1X15.h>

//#define WIRE Wire

Adafruit_ADS1115 ads;
const int relayPin = 2;

// Timing control
unsigned long lastSerialTime = 0;
unsigned long lastAnalogTime = 0;
const unsigned long serialInterval = 1000; // 1 Hz
const unsigned long analogInterval = 5;   // 200 Hz

// Auto-ranging thresholds
adsGain_t currentGain = GAIN_ONE; // Start with ±4.096V
float gainMax = 4.096;

float prevVoltage = 0.0;
float prevOutVoltage = 0.0;

bool relayState = 0;

void setup(void) {
  delay(1000);
  Serial.begin(9600);
  //WIRE.begin();

  // Keep trying until ADS1115 is found
  Serial.println("point A");
  
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  Serial.println("ADS1115 connected.");

  ads.setGain(currentGain);
  ads.setDataRate(RATE_ADS1115_128SPS);

  pinMode(relayPin, OUTPUT);
}

void loop(void) {
  unsigned long currentMillis = millis();

  // Read differential voltage
  int16_t raw = ads.readADC_Differential_0_1();
  float voltage = ads.computeVolts(raw);

  prevVoltage = voltage;
  
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

   
  // 1Hz serial output update
  if ((currentMillis - lastSerialTime >= serialInterval) || (fabs(voltage*7.311)+0.25 < fabs(prevOutVoltage)|| fabs(voltage*7.311)-0.25 > fabs(prevOutVoltage))) {
    lastSerialTime = currentMillis;
    prevOutVoltage = voltage*7.311;
    Serial.print("VDC: ");
    Serial.print(voltage*7.311, 6); //7.3147
    Serial.print(" T(ms):");
    Serial.println(millis());
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
