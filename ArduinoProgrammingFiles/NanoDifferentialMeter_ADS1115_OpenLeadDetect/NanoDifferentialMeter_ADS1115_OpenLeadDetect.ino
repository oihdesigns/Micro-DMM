#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_ADS1115 ads;
const int dacOutPin = A0;  // DAC output pin on Nano R4
const int relayPin = 2;
const int relayVoltagePin = A7;
const int ammeterPin = A6;
const int VbridgePin = 3; // Control Voltage Bridge MOSFET

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Timing control
unsigned long lastSerialTime = 0;
unsigned long lastIntervalTime = 0;
unsigned long lastAnalogTime = 0;
unsigned long lastRelayTime = 0;
unsigned long lastDisplayTime = 0;
const unsigned long serialInterval = 1; // 1 Hz
const unsigned long statusInterval = 2000; // 1 Hz
const unsigned long analogInterval = 5;   // 200 Hz
const unsigned long relayInterval = 100;   // 10 Hz
const unsigned long displayInterval = 1000; // 1 Hz


float timefloating = 0.0;

unsigned long currentMillis = 0;

/*
// Auto-ranging thresholds
adsGain_t currentGainVolt = GAIN_ONE;  // ±4.096V
adsGain_t currentGainAmps = GAIN_TWO;  // example default for current
float gainMaxVolt, gainMaxAmps;
*/

const float shunt = 0.10; // example value in ohms

float prevVoltage = 0.0;
float prevOutVoltage = 0.0;
float convertedVoltage = 0.0;
float newVoltageReading = 0.0;

float relayVoltage = 0.0;

float medianVoltage = 0.0;      // exponentially filtered voltage for display
float medianVoltageStep = 0.0;
int16_t countV;
static uint8_t gainIndex;


float prevOutAmps = 0.0;

float voltage = 0.0;
float amps = 0.0;
float ammeterVoltage = 0.0;
float ammeterOffset = 2.5;

float VOLTAGE_SCALE = 0.0;

bool relayState = 0;

bool vClimb = 0;

bool alarm = 0;
bool alarmFlag = 0;
bool forceStop = 0;
bool updates = 1;
bool debug = 0;
bool firstVoltRun = 1;

float aRef = 5.0; //1.5 when using internal reference

//V Float Detect Related
bool vFloating = false;
float bridgeV = 0.0;
bool Vzero = true;
bool VzeroFlag = true;
bool vClosed = false;
bool vUndefined = true;
bool vClosedflag = false;
bool vClosedflagPrevious = false;
bool vClosedPrevious = false;
bool vFloatingPrevious = false;
bool vUndefinedPrevious = false;

bool vClosedtrig = 0;
bool vFloattrig = 0;
bool vUndefinedtrig = 0;

// ADS1115 gain factors (mV per bit) for each gain setting
const float GAIN_FACTOR_TWOTHIRDS = 0.1875;  // 2/3x (±6.144V range)
const float GAIN_FACTOR_1   = 0.125;   // ±4.096V
const float GAIN_FACTOR_2   = 0.0625;  // ±2.048V
const float GAIN_FACTOR_4   = 0.03125; // ±1.024V
const float GAIN_FACTOR_8   = 0.015625;// ±0.512V
const float GAIN_FACTOR_16  = 0.0078125;// ±0.256V


// instead of “static const int kGainLevels[] = { … };”
static const adsGain_t kGainLevels[] = {
  GAIN_TWOTHIRDS,
  GAIN_ONE,
  GAIN_TWO,
  GAIN_FOUR,
  GAIN_EIGHT,
  GAIN_SIXTEEN
};

void handleSerialCommands(char command);

void updateDisplay();

void measureVoltage();

void statusUpdate();


void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin();


  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
  Serial.println(F("OLED init failed"));
  while (1); // halt
  }
  
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  // Splash screen
  display.setCursor(0, 0);
  display.print("OPEN LEAD DETECT");
  display.display();
    delay(200);


  analogReadResolution(14);
  //analogReference(AR_INTERNAL);

  // Keep trying until ADS1115 is found
  while (!ads.begin(0x48,&Wire1)) {
    Serial.println("ADS1115 not found. Retrying... (restart?)");
    delay(1000);
  }
  
  Serial.println("ADS1015 connected.");

  ads.setGain(GAIN_SIXTEEN);
  //gainMaxVolt = getGainMax(currentGainVolt);
  ads.setDataRate(RATE_ADS1115_860SPS);

  pinMode(relayPin, OUTPUT);
  pinMode(relayVoltagePin, INPUT);
  pinMode(ammeterPin, INPUT);
  pinMode(VbridgePin, OUTPUT);

  digitalWrite(VbridgePin, HIGH);
}

void loop() {
  currentMillis = millis();

    // Handle serial commands (if any received)
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    handleSerialCommands(cmd);
  }

  float prevAmps = amps;

  measureVoltage();

  if(fabs(medianVoltage) > 12.5 || fabs(medianVoltage) < 7){
    alarm = 1;
  } else{
    alarm = 0;
  }


  if(currentMillis - lastRelayTime >= relayInterval){
    lastRelayTime = currentMillis;
  if(!alarm && !forceStop){
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
  }

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

 timefloating = millis();
  
  if(abs(prevVoltage)<abs(newVoltageReading)){
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

  float aThreshold = 0.0;
  aThreshold= 0.5;
  
  relayVoltage = analogRead(relayVoltagePin);
  relayVoltage = (relayVoltage/16383)*aRef*10.8;


  ammeterVoltage = analogRead(ammeterPin);
  amps = ((((ammeterVoltage/16383)*aRef)-ammeterOffset)/0.185 + prevAmps)/2; // Ammeter conversion factor, plus slight average smoothing


  // Voltage Updates
  if ((currentMillis - lastSerialTime >= serialInterval) && (fabs(medianVoltage)+vThreshold < fabs(prevOutVoltage)|| fabs(medianVoltage)-vThreshold > fabs(prevOutVoltage)
  || (fabs(amps)+aThreshold < fabs(prevOutAmps)|| fabs(amps)-aThreshold > fabs(prevOutAmps)))
  //|| (Vzero && !VzeroFlag)
 ) {
    lastSerialTime = currentMillis;
    prevOutVoltage = medianVoltage;
    prevOutAmps = amps;
    VzeroFlag = Vzero;
Serial.print("VDC:");
    Serial.print(medianVoltage,4);
    //Serial.print(roundTo3SigAndHalf(medianVoltage));
    Serial.print(" A:");
    Serial.print(amps, 2);
    Serial.print(" T(s):");
    Serial.println(timefloating/1000,3);
  }

//Status Update
  if ((currentMillis - lastIntervalTime >= statusInterval && updates) || (Vzero && !VzeroFlag)){
    
    statusUpdate();

  }

  if(alarm && !alarmFlag){
    Serial.print("ALARM");
    Serial.print(" T(s):");
    Serial.println(timefloating/1000,3);
    alarmFlag = 1;
  }

  if(!alarm && alarmFlag){
    alarmFlag = 0;
  }

  if(currentMillis - lastDisplayTime >= displayInterval){
      lastDisplayTime = currentMillis;
      updateDisplay();
    }


} //close loop

void measureVoltage(){
    float prevVoltage = newVoltageReading;

  static size_t  gainIndexVolt;
  // ADC count thresholds for gain switching
  static const int ADC_COUNT_LOW_THRESHOLD  = 10000;
  static const int ADC_COUNT_HIGH_THRESHOLD = 30000;
  static const float kGainFactors[] = {GAIN_FACTOR_TWOTHIRDS, GAIN_FACTOR_1, GAIN_FACTOR_2, GAIN_FACTOR_4, GAIN_FACTOR_8, GAIN_FACTOR_16};
  static const int   kNumGainLevels = sizeof(kGainLevels) / sizeof(kGainLevels[0]);

  if (firstVoltRun) {
    // start at highest resolution (max gain)
    gainIndexVolt = kNumGainLevels - 1;
    firstVoltRun  = false;
  }

  VOLTAGE_SCALE = 69.669; //7.576 for earlier divider

    ads.setGain(kGainLevels[gainIndexVolt]);
    countV = ads.readADC_Differential_0_1();
    if (abs(countV) > ADC_COUNT_HIGH_THRESHOLD && gainIndexVolt > 0) {
      --gainIndexVolt;
      ads.setGain(kGainLevels[gainIndexVolt]);
      countV = ads.readADC_Differential_0_1();
    } else if (abs(countV) < ADC_COUNT_LOW_THRESHOLD && gainIndexVolt < kNumGainLevels - 1) {
      ++gainIndexVolt;
      ads.setGain(kGainLevels[gainIndexVolt]);
      countV = ads.readADC_Differential_0_1();
    }
    // convert to voltage
      newVoltageReading = ((countV * kGainFactors[gainIndexVolt] / 1000.0f) * VOLTAGE_SCALE);

      if(newVoltageReading > (prevVoltage*1.5) || newVoltageReading < (prevVoltage*0.66)){
        medianVoltage = newVoltageReading;
      }else{
        medianVoltageStep = (newVoltageReading - medianVoltage) * 0.2;
        medianVoltage += medianVoltageStep;
      }

  

      if(fabs(medianVoltage) < 0.03){
    Vzero = true;
    //VzeroFlag = true;
    ClosedOrFloat();
  }else{
    Vzero = false;
    vFloating = false;
    VzeroFlag = false;
  } 

}


void handleSerialCommands(char command) {
  // Handle a single-character command from Serial
  switch (command) {
    case 'S':  // "S" for STOP
      Serial.println("STOPPING");
        forceStop = 1;
      Serial.println("Force Stop On");
      break;
    case 'G':  // "G" for Go
      Serial.println("Going");
      forceStop = 0;
      break;    
    case 'U':  // "U" for constant Updates
      Serial.println("Constant Updates Toggled");
      updates = !updates;
      break;     
    case 'A':  // "A" for reset ammeter
      Serial.println("Ammeter Zeroed");
      ammeterOffset = (ammeterVoltage/16383)*aRef;
      Serial.print("New 0: ");
      Serial.println(ammeterOffset);
      break;
    case 'D':  // "D" for debug
      Serial.println("Debug Toggled");
      debug = !debug;
      break;      
    
    case '?':  // Help: list commands
      Serial.println("Commands: S (Force STOP), G (Go), U (Updates)");
      
      break;
    default:
      break;
  }
}

float roundTo3SigAndHalf(float x) {
  if (x == 0) return 0.0;

  // Determine order of magnitude
  float sign = (x < 0) ? -1.0 : 1.0;
  x = fabs(x);
  int exp10 = floor(log10(x));

  // Scale so that first digit is in the ones place
  float scaled = x / pow(10, exp10 - 2);  // shift so we keep 3 significant figures total

  // Extract the 4th figure
  int fourth = ((int)(scaled * 10)) % 10;

  // Round the 4th to 0 or 5
  int roundTarget = (fourth < 3) ? 0 : (fourth < 8 ? 5 : 10);

  // Apply rounding
  float rounded;
  if (roundTarget == 10) {
    rounded = (floor(scaled / 10.0) + 1.0) * 10.0;
  } else {
    rounded = floor(scaled) + (roundTarget / 10.0);
  }

  // Scale back
  float result = rounded * pow(10, exp10 - 2);
  return result * sign;
}

void updateDisplay() {
  // Prepare values for display
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print("V: ");
  display.print(medianVoltage,4);
  display.setCursor(0,16);
  display.setTextSize(1);
  display.print("State: ");
  display.setTextSize(2);
  display.setCursor(40,16);
  if(alarm){
    display.print("ALARM!");
    }else if(forceStop){
    display.print("Stopped");
    }else{
      display.print("Good");
    }
  
  display.setCursor(0,32);
  display.print("Leads: ");
  display.setCursor(0,48);
  if(Vzero){
    if(vClosed){
        display.print("Closed");
      }
      else{
        display.print("Floating");
      }}
      else{
      display.print(" ");
      display.print("(V !=0)");
      }
  

  display.display(); // update the OLED with all the drawn content
}

void statusUpdate(){

  lastIntervalTime = currentMillis;
    VzeroFlag = Vzero;
    Serial.print("Status:");
    if(alarm){
      Serial.print(" ALARM! ");
      }else if(forceStop){
      Serial.print(" Force Stop ");
      }else{
        Serial.print(" Good ");
      }

      Serial.print("/ Relay:");
    
    if(relayState){
      Serial.print(" Closed");
    }else{
      Serial.print(" Open");
    }
    Serial.print("/ Relay V:");
    Serial.print(relayVoltage);
    if(Vzero){
      Serial.print(" ");
      Serial.print("voltageRead:");
      Serial.print(bridgeV,4);
      Serial.print(" / ");
      if(vClosed){
        Serial.print("V Closed");
      }else{
        Serial.print("V Floating");
      }
    }else{
      Serial.print(" / ");
      Serial.print("V !0");
    }
    Serial.println(" / ");
    Serial.print("VDC:");
    Serial.print(medianVoltage,4);
    Serial.print(" Count V:");
    Serial.print(countV);
    //Serial.print(roundTo3SigAndHalf(medianVoltage));
    Serial.print(" A:");
    Serial.print(amps, 2);
    Serial.print(" T(s):");
    Serial.println(timefloating/1000,3);
    
    //Serial.println(voltage);

}

void ClosedOrFloat() {
  const float vClosedThres = 0.1;
  //const float vFloatThres  = 0.25;

  // Save previous states
  bool prevClosed    = vClosed;
  bool prevFloating  = vFloating;
  bool prevUndefined = vUndefined;

  float bridgeV1 = 0.0;
  float bridgeV2 = 0.0;

  // Reset current detection flags
  vClosed = vFloating = vUndefined = false;

  // Prepare to take a reading
  digitalWrite(VbridgePin, LOW);
  delay(5);

  ads.setDataRate(RATE_ADS1115_128SPS);
  ads.setGain(GAIN_SIXTEEN);
  bridgeV1 = ads.readADC_Differential_0_1() * (0.0078125f / 1000.0f) * VOLTAGE_SCALE;
  delay(1);
  bridgeV2 = ads.readADC_Differential_0_1() * (0.0078125f / 1000.0f) * VOLTAGE_SCALE;

  
  bridgeV = (abs(bridgeV1)+abs(bridgeV2))/2;


  if (debug) {
    Serial.print("voltageRead: ");
    Serial.println(bridgeV, 4);
  }

  // --- Classification and stability detection ---
  if (fabs(bridgeV) < vClosedThres) {
    // Candidate for "Closed"
    if (vClosedtrig) vClosed = true;  // repeats → stable
    vClosedtrig = true;
    vFloattrig = vUndefinedtrig = false;

    if (debug) Serial.println("V Closed");

  } else if (fabs(bridgeV) > vClosedThres) {
    // Candidate for "Floating"
    if (vFloattrig) vFloating = true;
    vFloattrig = true;
    vClosedtrig = vUndefinedtrig = false;

    if (debug) Serial.println("V Floating");

  } else {
    // Intermediate region → keep last known stable state
    if (prevClosed) {
      vClosed = true;
    } else{
      vFloating = true;
    }
  }

  // Release bridge
  digitalWrite(VbridgePin, HIGH);
}
