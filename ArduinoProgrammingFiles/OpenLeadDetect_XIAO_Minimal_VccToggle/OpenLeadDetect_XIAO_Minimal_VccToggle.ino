/*
 * OpenLeadDetect_XIAO_Minimal.ino
 *
 * Stripped-down open / closed lead detector.
 * No ADS1X15, no display -- pseudo-differential measurement (A0 - A2),
 * status shown only on the onboard NeoPixel.
 *
 * "Pseudo-differential": the RA4M1 analogRead() API has no native
 * differential mode, so A0 and A2 are each sampled vs. GND and
 * subtracted in software.  Resting differential is ~0 V, so detection
 * works on the magnitude of the deviation from zero.
 *
 * Target ADC:  Renesas RA4M1 (14-bit, analogReadResolution(14) -> 0..16383)
 * Board pinout: Seeed Studio XIAO (for NeoPixel mapping)
 *
 * Measurement logic (repeated continuously):
 *   1. MOSFET held HIGH (resting / bridge connected), read A0 - A2.
 *   2. Voltage-present decision (de-noised):
 *        - any single read beyond VOLT_FAST_MULT * REF_BAND_V -> present;
 *        - otherwise average VOLT_AVG_SAMPLES reads and compare to the band.
 *      If voltage is present the open/closed test is bypassed.
 *   3. Test (only when no voltage):  MOSFET LOW, settle, read A0 - A2.
 *        - |deviation| below OPEN_THRESH_V -> OPEN  (floating) -> LED off
 *        - |deviation| at/above           -> CLOSED           -> green
 *      Repeat the test until the same result appears TEST_AGREE_COUNT
 *      times in a row, then return the MOSFET HIGH and repeat.
 *
 * NeoPixel states:
 *   off            = floating (open lead, no indication)
 *   green flash    = closed (rate-limited to ~2 Hz)
 *   blinking red   = voltage present (test bypassed)
 */

#include <Adafruit_NeoPixel.h>

// ── ADC reference ────────────────────────────────────────────────
// The 2.46 V resting band is ~Vcc/2 for a 5 V supply.  Change to 3.3
// here if this board runs the ADC against a 3.3 V reference.
const float ADC_REF_VOLTAGE = 5.0f;
const int   ADC_RESOLUTION   = 14;          // RA4M1 14-bit
const float ADC_FULL_SCALE   = 16383.0f;    // 2^14 - 1

// ── Detection thresholds ─────────────────────────────────────────
// Differential (A0 - A2) rests near 0 V, so the band is centred on 0
// and detection looks at the magnitude of the deviation.  Retune
// OPEN_THRESH_V against real closed/open readings on the bench.
const float REF_CENTER_V   = -0.55f;   // resting differential (~0 V)
const float REF_BAND_V      = 0.06f;  // |A0-A2| within this -> run the test
const float OPEN_THRESH_V   = 0.9f;  // |deviation| below = open, at/above = closed
const unsigned long SETTLE_Post_MS = 1;     // MOSFET-off settle before test read
const unsigned long SETTLE_Pre_uS = 200;

// ── Noise / averaging ────────────────────────────────────────────
// Voltage-present decision:
//   - if any single read deviates more than VOLT_FAST_MULT * REF_BAND_V
//     from centre, declare voltage present immediately (no averaging);
//   - otherwise average VOLT_AVG_SAMPLES reads and decide on the average.
// MOSFET test: repeat until the same open/closed result is returned
//   TEST_AGREE_COUNT times in a row (capped by TEST_MAX_ATTEMPTS so a
//   noisy boundary can never hang the loop).
int          VOLT_AVG_SAMPLES = 10;     // reads averaged for voltage-present decision
int          TEST_AGREE_COUNT = 2;     // consecutive matching MOSFET tests required
const float  VOLT_FAST_MULT   = 1.5f;  // single-read "voltage present" shortcut
const int    TEST_MAX_ATTEMPTS = 30;   // safety cap on MOSFET test repeats

// ── Pin assignments ──────────────────────────────────────────────
const int SENSE_POS  = A0;   // pseudo-differential positive input
const int SENSE_NEG  = A2;   // pseudo-differential negative input
const int MOSFET_PIN = D1;   // bridge MOSFET gate (HIGH = on/resting)

const int CircuitVPin = D7;   // bridge MOSFET gate (HIGH = on/resting)

// MOSFET drive polarity (resting = HIGH per spec)
#define MOSFET_ON   HIGH
#define MOSFET_OFF  LOW

// ── NeoPixel (Seeed XIAO mapping) ────────────────────────────────
//Uncomment one of the two following lines to define PCB vs. external NeoPixel
//#define LED_PIN  RGB_BUILTIN  // Define the pin for the built-in RGB LED
#define LED_PIN  10 //
#define NUM_PIXELS 1         // Number of WS2812 LEDs

Adafruit_NeoPixel pixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ── Status colours ───────────────────────────────────────────────
const uint8_t COL_CLOSED_R = 0,  COL_CLOSED_G = 128, COL_CLOSED_B = 0;   // green
const uint8_t COL_VOLT_R = 80,   COL_VOLT_G = 0,    COL_VOLT_B = 0;     // red

// ── Blink behaviour ──────────────────────────────────────────────
// Floating  -> LED off (no indication).
// Voltage   -> symmetric on/off blink.
// Closed    -> brief flash, rate-limited (max ~2 Hz) like the big version.
const unsigned long VOLT_BLINK_PERIOD_MS = 150;  // red toggles every 150 ms (~3 Hz)
const unsigned long CLOSED_FLASH_MS      = 200;   // green on-time per flash
const unsigned long CLOSED_BLINK_MIN_MS  = 500;  // min gap between flashes (2 Hz cap)

// ── State ────────────────────────────────────────────────────────
enum LeadState { STATE_FLOAT, STATE_CLOSED, STATE_VOLTAGE };
LeadState leadState = STATE_VOLTAGE;

// Non-blocking blink state
unsigned long lastVoltToggle  = 0;
bool          voltLedOn       = false;
unsigned long lastClosedFlash = 0;
bool          closedFlashing  = false;

unsigned long lastSerialTime = 0;
const unsigned long serialInterval = 250;   // ms between debug prints
const unsigned long generalDelay = 10;

// Most recent values, kept for serial debug
float lastRestV = 0.0f;
float lastTestV = 0.0f;

// ── Helpers ──────────────────────────────────────────────────────
// Pseudo-differential read: sample both pins vs. GND and subtract.
float readVoltage() {
  int rawPos = analogRead(SENSE_POS);
  int rawNeg = analogRead(SENSE_NEG);
  return ((rawPos - rawNeg) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
}

// Decide whether a real voltage is present (MOSFET resting / on).
//   - Any single read deviating more than VOLT_FAST_MULT * REF_BAND_V from
//     centre -> voltage present immediately (no averaging).
//   - Otherwise average VOLT_AVG_SAMPLES reads and test against REF_BAND_V.
// The voltage used for the decision is stored in lastRestV for debug.
bool voltagePresent() {
  float sum = 0.0f;
  for (int i = 0; i < VOLT_AVG_SAMPLES; i++) {
    float v = readVoltage();
    if (fabs(v - REF_CENTER_V) > VOLT_FAST_MULT * REF_BAND_V) {
      lastRestV = v;
      return true;
    }
    sum += v;
  }
  lastRestV = sum / VOLT_AVG_SAMPLES;
  return (fabs(lastRestV - REF_CENTER_V) > REF_BAND_V);
}

// One open/closed test: MOSFET off, settle, read, MOSFET back on.
// Stores the reading in lastTestV for debug.
LeadState runMosfetTest() {
  digitalWrite(MOSFET_PIN, MOSFET_OFF);
  delayMicroseconds(SETTLE_Pre_uS);
  lastTestV = readVoltage();
  delay(SETTLE_Post_MS*2);
  digitalWrite(MOSFET_PIN, MOSFET_ON);   // return to resting state
  return (fabs(lastTestV) > OPEN_THRESH_V) ? STATE_FLOAT : STATE_CLOSED;
}

// Repeat the MOSFET test until the same result appears TEST_AGREE_COUNT
// times in a row (or TEST_MAX_ATTEMPTS is reached, returning the last result).
LeadState runMosfetTestStable() {
  LeadState result = runMosfetTest();
  LeadState prev   = result;
  int agree = 1;
  int attempts = 1;
  while (agree < TEST_AGREE_COUNT && attempts < TEST_MAX_ATTEMPTS) {
    result = runMosfetTest();
    agree  = (result == prev) ? (agree + 1) : 1;
    prev   = result;
    attempts++;
  }
  return result;
}

void setPixel(uint8_t r, uint8_t g, uint8_t b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

// Non-blocking NeoPixel driver.
//   FLOAT   -> off (no indication)
//   VOLTAGE -> blinking red (symmetric on/off)
//   CLOSED  -> brief green flashes, rate-limited to ~2 Hz
void updateLed() {
  static LeadState prevState = STATE_VOLTAGE;
  unsigned long now = millis();

  // On a state change, end any in-progress flash and reset the (symmetric)
  // voltage blink.  Deliberately DO NOT reset lastClosedFlash: the green
  // 2 Hz rate limit must persist across transitions, otherwise a state that
  // briefly bounces out of CLOSED and back would re-fire green every time.
  if (leadState != prevState) {
    prevState       = leadState;
    voltLedOn       = false;
    closedFlashing  = false;
    lastVoltToggle  = now;
    setPixel(0, 0, 0);
  }

  switch (leadState) {
    case STATE_FLOAT:
      setPixel(0, 0, 0);   // floating -> no indication
      break;

    case STATE_VOLTAGE:
      if (now - lastVoltToggle >= VOLT_BLINK_PERIOD_MS) {
        lastVoltToggle = now;
        voltLedOn = !voltLedOn;
        if (voltLedOn) setPixel(COL_VOLT_R, COL_VOLT_G, COL_VOLT_B);
        else           setPixel(0, 0, 0);
      }
      break;

    case STATE_CLOSED:
      if (!closedFlashing && (now - lastClosedFlash >= CLOSED_BLINK_MIN_MS)) {
        closedFlashing  = true;
        lastClosedFlash = now;
        setPixel(COL_CLOSED_R, COL_CLOSED_G, COL_CLOSED_B);   // flash on
      } else if (closedFlashing && (now - lastClosedFlash >= CLOSED_FLASH_MS)) {
        closedFlashing = false;
        setPixel(0, 0, 0);                                    // flash off
      }
      break;
  }
}

// ==================================================================
//  SETUP
// ==================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(ADC_RESOLUTION);

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, MOSFET_ON);   // resting state: MOSFET high

  pinMode(CircuitVPin, OUTPUT);

  pinMode(SENSE_POS, INPUT);
  pinMode(SENSE_NEG, INPUT);

  pinMode(PIN_RGB_EN, OUTPUT); // Set up the power pin
  digitalWrite(PIN_RGB_EN, HIGH); //Turn on power to the LED

  pixel.begin();
  pixel.clear();
  pixel.show();

  Serial.println("OpenLeadDetect_XIAO_Minimal ready.");
}

// ==================================================================
//  MAIN LOOP
// ==================================================================
void loop() {
  // 1. MOSFET resting high, decide whether a real voltage is present
  //    (fast single-read shortcut, else averaged).
  
  digitalWrite(CircuitVPin, HIGH);
  delayMicroseconds(1000); //Settling
  
  digitalWrite(MOSFET_PIN, MOSFET_ON);
  

  if (voltagePresent()) {
    leadState = STATE_VOLTAGE;    
  } else {
    // 2 + 3. No voltage -- run the open/closed test until it agrees with itself.
    leadState = runMosfetTestStable();
  }


  if(leadState == STATE_VOLTAGE){
    digitalWrite(CircuitVPin, HIGH);
  }else{
    digitalWrite(CircuitVPin, LOW);
  }

  // ── Drive the NeoPixel ──
  updateLed();

  // ── Periodic serial debug ──
  if (millis() - lastSerialTime >= serialInterval) {
    lastSerialTime = millis();
    Serial.print("Rest:");
    Serial.print(lastRestV, 3);
    Serial.print("V  ");
    if (leadState == STATE_VOLTAGE) {
      Serial.println("-> VOLTAGE (bypass)");
    } else {
      Serial.print("Test:");
      Serial.print(lastTestV, 3);
      Serial.print("V  -> ");
      Serial.println(leadState == STATE_FLOAT ? "FLOATING" : "CLOSED");
    }
  }

delay(generalDelay); //Limit Overall Frequency
}
