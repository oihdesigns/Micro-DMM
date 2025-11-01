#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1015 ads;
const int dacOutPin = A0;  // DAC output pin on Nano R4
const int relayPin = 2;

// Timing control
unsigned long lastSerialTime = 0;
unsigned long lastIntervalTime = 0;
unsigned long lastAnalogTime = 0;
const unsigned long serialInterval = 1; // 1 Hz
const unsigned long statusInterval = 2000; // 1 Hz
const unsigned long analogInterval = 5;   // 200 Hz

// Auto-ranging thresholds
adsGain_t currentGainVolt = GAIN_ONE;  // ±4.096V
adsGain_t currentGainAmps = GAIN_TWO;  // example default for current
float gainMaxVolt, gainMaxAmps;

const float shunt = 0.10; // example value in ohms

float prevVoltage = 0.0;
float prevOutVoltage = 0.0;

float voltage = 0.0;
float amps = 0.0;

bool relayState = 0;

bool vClimb = 0;

bool alarm = 0;
bool alarmFlag = 0;




void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin();

  // Keep trying until ADS1115 is found
  while (!ads.begin(0x48,&Wire1)) {
    Serial.println("ADS1115 not found. Retrying... (restart?)");
    delay(1000);
  }
  
  Serial.println("ADS1015 connected.");

  ads.setGain(currentGainVolt);
  gainMaxVolt = getGainMax(currentGainVolt);
  ads.setDataRate(RATE_ADS1015_3300SPS);

  pinMode(relayPin, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  float prevVoltage = voltage;
  float prevAmps = amps;

  // --- Read differential voltage and current ---
  int16_t rawV = ads.readADC_Differential_0_1();
    voltage = ads.computeVolts(rawV);

  autoRangeChannel(fabs(voltage), currentGainVolt, gainMaxVolt, "Voltage");

  /*
  
  int16_t rawI = ads.readADC_Differential_2_3();
  amps = ads.computeVolts(rawI) / shunt;
  autoRangeChannel(fabs(amps * shunt), currentGainAmps, gainMaxAmps, "Current"); 
    // using amps*shunt so logic matches ADC input voltage range
    */

  if(fabs(voltage*7.311) > 5 ){
    alarm = 1;
  } else{
    alarm = 0;
  }


  if(fabs(voltage*7.311)>2.5 && !alarm){
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

  float time = millis();
  if(abs(prevVoltage)<abs(voltage)){
    vClimb = 1;
  }else{
    vClimb = 0;
  }

  float vThreshold;
  if(vClimb){
    vThreshold = 0.25;
  }else{
    vThreshold = 1;
  }
    
  
  // Voltage Updates
  if ((currentMillis - lastSerialTime >= serialInterval) && (fabs(voltage*7.311)+vThreshold < fabs(prevOutVoltage)|| fabs(voltage*7.311)-vThreshold > fabs(prevOutVoltage))) {
    lastSerialTime = currentMillis;
    prevOutVoltage = voltage*7.311;
    Serial.print("VDC: ");
    Serial.print(voltage*7.311, 4); //7.3147
    //Serial.print(" mA: ");
    //Serial.print(amps*1000,1);
    Serial.print(" T(s):");
    Serial.println(time/1000,3);
    //Serial.print(" V, Gain range: ±");
    //Serial.println(gainMax, 3);
  }

//Status Update
  if (currentMillis - lastIntervalTime >= statusInterval){
    lastIntervalTime = currentMillis;
    Serial.print("Status:");
    if(alarm){
      Serial.print(" ALARM! ");
      }else{
        Serial.print(" Good ");
      }

      Serial.print("/ Relay:");
    
    if(relayState){
      Serial.print(" Closed");
    }else{
      Serial.print(" Open");
    }
    Serial.println(" / ");
    Serial.print("VDC:");
    Serial.print(voltage*7.311, 4);
    Serial.print(" T(s):");
    Serial.println(time/1000,3);

  }

  if(alarm && !alarmFlag){
    Serial.print("ALARM");
    Serial.print(" T(s):");
    Serial.println(time/1000,3);
    alarmFlag = 1;
  }

  if(!alarm && alarmFlag){
    alarmFlag = 0;
  }


} //close loop

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
  if (absValue > 0.85 * gainMax) {
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