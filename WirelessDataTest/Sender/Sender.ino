#include <RH_ASK.h>
#include <SPI.h>  // Required even if unused
#include <math.h>


RH_ASK driver(2000, 11, 10,9); // Create an instance of the driver



unsigned long lastSend = 0;

// Address lines
const int ADDR0 = 2;   // LSB
const int ADDR1 = 3;
const int ADDR2 = 4;
const int ADDR3 = 5;   // MSB

const int txEnable = 6;

int tempF= 0;


uint8_t readAddress() {
  // Read 4 bits, each 0 or 1
  uint8_t b0 = digitalRead(ADDR0) == HIGH ? 1 : 0;
  uint8_t b1 = digitalRead(ADDR1) == HIGH ? 1 : 0;
  uint8_t b2 = digitalRead(ADDR2) == HIGH ? 1 : 0;
  uint8_t b3 = digitalRead(ADDR3) == HIGH ? 1 : 0;

  return (b3 << 3) | (b2 << 2) | (b1 << 1) | b0;
}


void setup() {
  Serial.begin(115200);

  analogReadResolution(14);

  pinMode(txEnable, OUTPUT);


  // Address pins use INPUT_PULLUP so “floating = 1”
  pinMode(ADDR0, INPUT_PULLUP);
  pinMode(ADDR1, INPUT_PULLUP);
  pinMode(ADDR2, INPUT_PULLUP);
  pinMode(ADDR3, INPUT_PULLUP);

  if (!driver.init()) {
    Serial.println("RH_ASK init failed");
    while (1);
  }

  Serial.println("TX ready with hardware address pins 2–5.");

      digitalWrite(txEnable, 0);
}

void loop() {
  unsigned long now = millis();

  
  
  if (now - lastSend >= 1000) {
    lastSend = now;

    float tempC = readNTCTemperatureC(
        A1,        // analog pin
        14,        // ADC bits
        5.0,       // Vref
        10000.0,   // fixed resistor (10k)
        10000.0,   // NTC nominal resistance
        25.0,      // nominal temp
        3950.0     // beta value
    );

    tempF = (tempC*9/5)+32;

    // Read hardware address
    uint8_t addr = readAddress();  // 0x0 to 0xF

    // Read ADCs
    int v0 = analogRead(A0);
    //int v1 = analogRead(A1);
    int v2 = analogRead(A2);
    int v3 = analogRead(A3);

    // Build formatted message
    char msg[64];
    snprintf(msg, sizeof(msg),
             "ADDR:%X 0:%d 1:%dF 2:%d",
             addr, v0, tempF, v2);

    digitalWrite(txEnable, 1);
    
    driver.send((uint8_t*)msg, strlen(msg));
    driver.waitPacketSent();

   digitalWrite(txEnable, 0);

  Serial.println(tempF);
    
    Serial.print("Sent: ");
    Serial.println(msg);
  }
}

float readNTCTemperatureC(
    uint8_t analogPin,
    uint8_t adcBits,
    float   vRef,
    float   rFixed,
    float   rNominal,
    float   tNominalC,
    float   beta
  ) {
    // Read ADC
    uint32_t adcMax = (1UL << adcBits) - 1;
    uint32_t adcRaw = analogRead(analogPin);

    // Convert ADC code to voltage
    float vNTC = (adcRaw * vRef) / adcMax;

    // Protect against divide-by-zero and rail conditions
    if (vNTC <= 0.0f || vNTC >= vRef) {
        return NAN;
    }

    // Compute thermistor resistance
    float rNTC = rFixed * (vNTC / (vRef - vNTC));

    // Beta equation
    float t0K = tNominalC + 273.15f;
    float invT = (1.0f / t0K) + (1.0f / beta) * log(rNTC / rNominal);
    float tempK = 1.0f / invT;

    return tempK - 273.15f;
}