/*
 * OpenLeadDetect_XIAO_Minimal.ino
 *
 * Stripped-down open / closed lead detector.
 * No ADS1X15, no display -- pseudo-differential measurement (A0 - A2),
 * status shown only on the onboard NeoPixel.
 *
 * "Pseudo-differential": analogRead() has no native differential mode, so
 * A0 and A2 are each sampled vs. GND and subtracted in software.  Resting
 * differential is ~0 V, so detection works on the magnitude of the deviation
 * from zero.
 *
 * Multi-board: builds for three Seeed XIAO chips from a single source -- pick
 * one BOARD_XIAO_* define at the top (and match it in Tools > Board):
 *   - XIAO RA4M1  (Renesas, 14-bit ADC, onboard RGB on RGB_BUILTIN/PIN_RGB_EN)
 *   - XIAO SAMD21 (12-bit ADC, onboard NeoPixel on PIN_NEOPIXEL)
 *   - XIAO RP2040 (12-bit ADC, onboard WS2812 data=12 / power=11)
 * All board-specific values live in the BOARD SELECT block; the rest is shared.
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
 * NeoPixel states (all rate-limited flashes; on-time + max rate per state):
 *   dim-blue flash = floating (open lead)
 *   green flash    = closed
 *   red flash      = voltage present (test bypassed)
 *
 * ── Serial diagnostic protocol (115200 baud, line based) ──────────
 * Commands in  (each terminated with newline):
 *   !DIAG[,0|1]      enter/exit diagnostic mode (bare = toggle)
 *   !STREAM[,0|1]    continuous raw streaming on/off (diag only)
 *   !RATE,<ms>       stream interval in ms
 *   !VMODE,<0|1|2>   voltage mode: 0=auto  1=lock ON  2=disable
 *   !MOSFET,<-1|0|1> MOSFET: -1=auto(run detection) 0=hold off 1=hold on
 *   !CAP[,<ms>]      capture ADC across a MOSFET toggle, then dump
 *   !STATUS  / !?    print current status
 * Data out:
 *   $STATUS,diag=..,vmode=..,mosfet=..,stream=..,rate=..,capms=..,res=..,vref=..
 *   $DIAG,<ms>,<rawPos>,<rawNeg>,<posV>,<negV>,<diffV>      (streaming)
 *   $CAPSTART,<n>,<toggleUs>,<durMs>,<fullScale>,<vref>     (capture header)
 *   $CAP,<t_us>,<rawPos>,<rawNeg>                           (capture rows)
 *   $CAPEND
 */

#include <Adafruit_NeoPixel.h>
#include <ctype.h>

// ══════════════════════════════════════════════════════════════════
//  BOARD SELECT  —  uncomment EXACTLY ONE line for the chip you build
// ══════════════════════════════════════════════════════════════════
//#define BOARD_XIAO_RA4M1
#define BOARD_XIAO_SAMD21
//#define BOARD_XIAO_RP2040

// ── Per-board configuration ───────────────────────────────────────
// Everything that differs between the three XIAO chips lives in this block;
// the rest of the sketch is board-agnostic and reads these CFG_* / LED_PIN /
// RGB_POWER_PIN values.  Each block also cross-checks the architecture macro
// so picking the wrong "Tools > Board" gives a clear error instead of a
// mysterious runtime fault.
#if defined(BOARD_XIAO_RA4M1)
  #if !defined(ARDUINO_ARCH_RENESAS)
    #error "BOARD_XIAO_RA4M1 selected, but Tools>Board is not a Renesas/RA4M1 board."
  #endif
  #define CFG_ADC_RESOLUTION  14
  #define CFG_ADC_FULL_SCALE  16383.0f
  #define CFG_MOSFET_PIN      D1
  #define LED_PIN             RGB_BUILTIN     // onboard RGB
  #define RGB_POWER_PIN       PIN_RGB_EN      // onboard RGB power enable
  #define CFG_OPEN_THRESH_V   0.250f          // self-contained proto
  #define CFG_REF_BAND_V      0.05f
  //#define CFG_OPEN_THRESH_V 0.1f            // <- breadboard proto alt (then comment the line above)
  #define CFG_SLEEP_WFI       1               // Cortex-M4 WFI idle

#elif defined(BOARD_XIAO_SAMD21)
  #if !defined(ARDUINO_ARCH_SAMD)
    #error "BOARD_XIAO_SAMD21 selected, but Tools>Board is not a SAMD board."
  #endif
  #define CFG_ADC_RESOLUTION  12
  #define CFG_ADC_FULL_SCALE  4095.0f
  #define CFG_MOSFET_PIN      1               // D1 isn't defined on this core
  #define LED_PIN             PIN_NEOPIXEL    // onboard NeoPixel
  // (no separate RGB power-enable pin on this board)
  #define CFG_OPEN_THRESH_V   0.1f
  #define CFG_REF_BAND_V      0.01f
  #define CFG_SLEEP_WFI       1               // Cortex-M0+ WFI idle

#elif defined(BOARD_XIAO_RP2040)
  #if !defined(ARDUINO_ARCH_RP2040)
    #error "BOARD_XIAO_RP2040 selected, but Tools>Board is not a Pico/RP2040 board."
  #endif
  #define CFG_ADC_RESOLUTION  12
  #define CFG_ADC_FULL_SCALE  4095.0f
  #define CFG_MOSFET_PIN      D1
  #define LED_PIN             12              // onboard WS2812 data
  #define RGB_POWER_PIN       11              // onboard WS2812 power enable
  #define CFG_OPEN_THRESH_V   0.11f
  #define CFG_REF_BAND_V      0.01f
  #define CFG_SLEEP_RP2040    1               // timer-alarm + WFI idle

#else
  #error "No board selected: uncomment one BOARD_XIAO_* line above."
#endif

// Optional: external NeoPixel on the custom PCB (data on D10, no power pin).
// Uncomment all three to override the onboard-LED mapping above.
//#undef  LED_PIN
//#undef  RGB_POWER_PIN
//#define LED_PIN  10

void sleepMs(unsigned long ms);   // defined per-board further down

// ── ADC reference ────────────────────────────────────────────────
// VREFH/AREF is tied to the 3.3 V rail on all three boards.
const float ADC_REF_VOLTAGE = 3.3f;
//const int   ADC_RESOLUTION  = CFG_ADC_RESOLUTION; //This line still has to be commented out for the RP2040 and SAMD21
const float ADC_FULL_SCALE  = CFG_ADC_FULL_SCALE;

// ── Detection thresholds ─────────────────────────────────────────
// Differential (A0 - A2) rests near 0 V, so the band is centred on 0
// and detection looks at the magnitude of the deviation.  Retune
// OPEN_THRESH_V against real closed/open readings on the bench.


const float REF_CENTER_V   = 0.0f;   // resting differential (~0 V)

const unsigned long SETTLE_Post_MS = 1;     // MOSFET-off settle before test read
const unsigned long SETTLE_Pre_uS = 300;

const float OPEN_THRESH_V = CFG_OPEN_THRESH_V;  // |dev| below = open, at/above = closed
const float REF_BAND_V    = CFG_REF_BAND_V;     // |A0-A2| within this -> run the test


// ── Noise / averaging ────────────────────────────────────────────
// Voltage-present decision:
//   - if any single read deviates more than VOLT_FAST_MULT * REF_BAND_V
//     from centre, declare voltage present immediately (no averaging);
//   - otherwise average VOLT_AVG_SAMPLES reads and decide on the average.
// MOSFET test: repeat until the same open/closed result is returned
//   TEST_AGREE_COUNT times in a row (capped by TEST_MAX_ATTEMPTS so a
//   noisy boundary can never hang the loop).
int          VOLT_AVG_SAMPLES = 10;     // reads averaged for voltage-present decision
int          TEST_AGREE_COUNT = 1;     // consecutive matching MOSFET tests required (1 = single test)
const float  VOLT_FAST_MULT   = 1.5f;  // single-read "voltage present" shortcut
const int    TEST_MAX_ATTEMPTS = 30;   // safety cap on MOSFET test repeats

// Display/alert debounce: a newly detected state must repeat for this many
// detection passes in a row before the LED switches to it.  This is what keeps
// the alert steady, independent of TEST_AGREE_COUNT -- so TEST_AGREE_COUNT can
// be lowered to 1 (single fast test) without the LED bouncing at a noisy
// open/closed boundary.  Set to 1 for no display debounce.
const int    STATE_STABLE_COUNT = 2;   // detection passes a new state must repeat

// ── Pin assignments ──────────────────────────────────────────────
const int SENSE_POS  = A0;   // pseudo-differential positive input
const int SENSE_NEG  = A2;   // pseudo-differential negative input
const int MOSFET_PIN = CFG_MOSFET_PIN;   // bridge MOSFET gate (HIGH = on/resting)

// MOSFET drive polarity (resting = HIGH per spec)
#define MOSFET_ON   HIGH
#define MOSFET_OFF  LOW

// ── NeoPixel ──────────────────────────────────────────────────────
// LED_PIN (and any RGB_POWER_PIN) come from the BOARD SELECT block above.
#define NUM_PIXELS 1         // Number of WS2812 LEDs

Adafruit_NeoPixel pixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ── Status colours ───────────────────────────────────────────────
const uint8_t COL_FLOAT_R = 0,   COL_FLOAT_G = 0,   COL_FLOAT_B = 28;   // dim blue
const uint8_t COL_CLOSED_R = 0,  COL_CLOSED_G = 128, COL_CLOSED_B = 0;   // green
const uint8_t COL_VOLT_R = 80,   COL_VOLT_G = 0,    COL_VOLT_B = 0;     // red

// ── Blink behaviour ──────────────────────────────────────────────
// Every state is driven the same way: a brief flash of its colour for
// <STATE>_FLASH_MS, rate-limited so a new flash can't begin until at least
// <STATE>_MIN_MS after the previous one started.  Max blink rate is
// 1000 / <STATE>_MIN_MS  Hz; keep FLASH_MS < MIN_MS so the LED returns to
// off between flashes.
const unsigned long FLOAT_FLASH_MS  = 150;   // dim-blue on-time per flash
const unsigned long FLOAT_MIN_MS    = 1000;  // min gap (1 Hz cap)
const unsigned long CLOSED_FLASH_MS = 200;   // green on-time per flash
const unsigned long CLOSED_MIN_MS   = 500;   // min gap (2 Hz cap)
const unsigned long VOLT_FLASH_MS   = 200;   // red on-time per flash
const unsigned long VOLT_MIN_MS     = 500;   // min gap (2 Hz cap)

// ── State ────────────────────────────────────────────────────────
enum LeadState { STATE_FLOAT, STATE_CLOSED, STATE_VOLTAGE };
LeadState leadState = STATE_VOLTAGE;

// Non-blocking blink state (one flashing flag + last-flash timestamp per state)
bool          floatFlashing   = false;
unsigned long lastFloatFlash  = 0;
bool          closedFlashing  = false;
unsigned long lastClosedFlash = 0;
bool          voltFlashing    = false;
unsigned long lastVoltFlash   = 0;

unsigned long lastSerialTime = 0;
const unsigned long serialInterval = 250;   // ms between debug prints
const unsigned long generalDelay = 50;

// Most recent values, kept for serial debug
float lastRestV = 0.0f;
float lastTestV = 0.0f;

// ── Diagnostic mode ───────────────────────────────────────────────
bool diagMode = false;                 // suppresses human debug, enables $ protocol
bool streamOn = false;                 // continuous raw streaming
unsigned long streamIntervalMs = 20;   // streaming period (~50 Hz default)
unsigned long lastStreamMs = 0;

enum VoltOverride { VOLT_AUTO, VOLT_FORCE_ON, VOLT_DISABLED };
VoltOverride voltOverride = VOLT_AUTO; // applies whenever detection runs

int mosfetHold = -1;                    // -1 auto (run detection), 0 hold off, 1 hold on

// Transient capture buffer (raw counts; volts computed by the host)
const int CAP_MAX_SAMPLES   = 600;      // ~4.8 KB of RAM
const unsigned long CAP_PRE_US = 500;   // baseline sampled before the toggle
unsigned long capDurationMs = 5;        // total capture window
uint32_t capT[CAP_MAX_SAMPLES];
uint16_t capPos[CAP_MAX_SAMPLES];
uint16_t capNeg[CAP_MAX_SAMPLES];
int capCount = 0;

// Serial command line buffer
char cmdBuf[48];
int  cmdLen = 0;

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
  sleepMs(SETTLE_Post_MS*2);            // CPU idles during settle
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

// Generic rate-limited flash for one state.  Turns the pixel on for onMs,
// then off, and won't start another flash until minGapMs after the last one
// began (max rate = 1000 / minGapMs Hz).  The flashing flag and lastFlash
// timestamp are owned by the caller so each state keeps its own cadence.
void flashState(unsigned long now,
                uint8_t r, uint8_t g, uint8_t b,
                unsigned long onMs, unsigned long minGapMs,
                bool &flashing, unsigned long &lastFlash) {
  if (!flashing && (now - lastFlash >= minGapMs)) {
    flashing  = true;
    lastFlash = now;
    setPixel(r, g, b);                 // flash on
  } else if (flashing && (now - lastFlash >= onMs)) {
    flashing = false;
    setPixel(0, 0, 0);                 // flash off
  }
}

// Non-blocking NeoPixel driver.  All three states use the same flash model:
//   FLOAT   -> dim-blue flashes, rate-limited
//   CLOSED  -> green flashes, rate-limited
//   VOLTAGE -> red flashes, rate-limited
void updateLed() {
  static LeadState prevState = STATE_VOLTAGE;
  unsigned long now = millis();

  // On a state change, end any in-progress flash so the LED returns to off.
  // Deliberately DO NOT reset the lastXFlash timestamps: each state's rate
  // limit must persist across transitions, otherwise a state that briefly
  // bounces out and back would re-fire immediately.
  if (leadState != prevState) {
    prevState      = leadState;
    floatFlashing  = false;
    closedFlashing = false;
    voltFlashing   = false;
    setPixel(0, 0, 0);
  }

  switch (leadState) {
    case STATE_FLOAT:
      flashState(now, COL_FLOAT_R, COL_FLOAT_G, COL_FLOAT_B,
                 FLOAT_FLASH_MS, FLOAT_MIN_MS, floatFlashing, lastFloatFlash);
      break;

    case STATE_CLOSED:
      flashState(now, COL_CLOSED_R, COL_CLOSED_G, COL_CLOSED_B,
                 CLOSED_FLASH_MS, CLOSED_MIN_MS, closedFlashing, lastClosedFlash);
      break;

    case STATE_VOLTAGE:
      flashState(now, COL_VOLT_R, COL_VOLT_G, COL_VOLT_B,
                 VOLT_FLASH_MS, VOLT_MIN_MS, voltFlashing, lastVoltFlash);
      break;
  }
}

// ==================================================================
//  DETECTION (one pass) -- sets leadState, honouring voltOverride
// ==================================================================
void runDetection() {
  digitalWrite(MOSFET_PIN, MOSFET_ON);   // resting state

  bool present;
  if (voltOverride == VOLT_FORCE_ON) {
    lastRestV = readVoltage();           // keep a fresh reading for debug
    present = true;
  } else if (voltOverride == VOLT_DISABLED) {
    present = false;
  } else {
    present = voltagePresent();
  }

  LeadState rawState = present ? STATE_VOLTAGE : runMosfetTestStable();

  // Display debounce: commit the raw detection result to leadState (and thus
  // the LED/alert) only after it has repeated STATE_STABLE_COUNT passes in a
  // row.  Without this, a single noisy test (TEST_AGREE_COUNT = 1) could flip
  // leadState every pass, and updateLed() cancels each in-progress flash on a
  // state change -- so the alert LED would never finish a blink.
  static LeadState candidate   = STATE_VOLTAGE;
  static int       stableCount = 0;
  if (rawState == leadState) {
    candidate   = rawState;     // already displayed; nothing pending
    stableCount = 0;
  } else {
    if (rawState != candidate) { candidate = rawState; stableCount = 0; }
    if (++stableCount >= STATE_STABLE_COUNT) {
      leadState   = rawState;
      stableCount = 0;
    }
  }
}

// ==================================================================
//  DIAGNOSTIC: streaming, capture, command handling
// ==================================================================
void printStatus() {
  Serial.print("$STATUS,diag=");  Serial.print(diagMode ? 1 : 0);
  Serial.print(",vmode=");        Serial.print((int)voltOverride);
  Serial.print(",mosfet=");       Serial.print(mosfetHold);
  Serial.print(",stream=");       Serial.print(streamOn ? 1 : 0);
  Serial.print(",rate=");         Serial.print(streamIntervalMs);
  Serial.print(",capms=");        Serial.print(capDurationMs);
  Serial.print(",res=");          Serial.print(ADC_RESOLUTION);
  Serial.print(",vref=");         Serial.println(ADC_REF_VOLTAGE, 3);
}

// One streamed sample: both pins independently + computed differential.
void streamSample() {
  int rawPos = analogRead(SENSE_POS);
  int rawNeg = analogRead(SENSE_NEG);
  float pv = (rawPos / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
  float nv = (rawNeg / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
  Serial.print("$DIAG,");
  Serial.print(millis()); Serial.print(",");
  Serial.print(rawPos);   Serial.print(",");
  Serial.print(rawNeg);   Serial.print(",");
  Serial.print(pv, 4);    Serial.print(",");
  Serial.print(nv, 4);    Serial.print(",");
  Serial.println(pv - nv, 4);
}

// Capture both ADC pins as fast as possible (no settling delays) across a
// MOSFET toggle: hold ON, sample a short baseline, toggle OFF at CAP_PRE_US,
// keep sampling until durationMs elapses or the buffer fills, restore ON,
// then dump the raw buffer to the host.
void runCapture(unsigned long durationMs) {
  digitalWrite(MOSFET_PIN, MOSFET_ON);
  delay(2);                              // settle to resting before baseline

  capCount = 0;
  unsigned long durUs    = durationMs * 1000UL;
  unsigned long toggleUs = 0;
  bool toggled = false;
  unsigned long t0 = micros();

  while (capCount < CAP_MAX_SAMPLES) {
    unsigned long t = micros() - t0;
    if (!toggled && t >= CAP_PRE_US) {
      digitalWrite(MOSFET_PIN, MOSFET_OFF);
      toggleUs = micros() - t0;
      toggled = true;
    }
    if (t >= durUs) break;
    capT[capCount]   = t;
    capPos[capCount] = analogRead(SENSE_POS);
    capNeg[capCount] = analogRead(SENSE_NEG);
    capCount++;
  }

  digitalWrite(MOSFET_PIN, MOSFET_ON);   // restore resting state

  Serial.print("$CAPSTART,");
  Serial.print(capCount);        Serial.print(",");
  Serial.print(toggleUs);        Serial.print(",");
  Serial.print(durationMs);      Serial.print(",");
  Serial.print(ADC_FULL_SCALE, 0); Serial.print(",");
  Serial.println(ADC_REF_VOLTAGE, 3);
  for (int i = 0; i < capCount; i++) {
    Serial.print("$CAP,");
    Serial.print(capT[i]);   Serial.print(",");
    Serial.print(capPos[i]); Serial.print(",");
    Serial.println(capNeg[i]);
  }
  Serial.println("$CAPEND");
}

// Parse one received command line (must start with '!').
void handleLine(char *line) {
  if (line[0] != '!') return;
  char *cmd = line + 1;
  char *arg = strchr(cmd, ',');
  if (arg) { *arg = '\0'; arg++; }
  for (char *p = cmd; *p; ++p) *p = toupper(*p);

  if (strcmp(cmd, "DIAG") == 0) {
    diagMode = arg ? (atoi(arg) != 0) : !diagMode;
    if (!diagMode) { streamOn = false; mosfetHold = -1; }
    printStatus();
  } else if (strcmp(cmd, "STREAM") == 0) {
    streamOn = arg ? (atoi(arg) != 0) : !streamOn;
    printStatus();
  } else if (strcmp(cmd, "RATE") == 0) {
    if (arg) { long r = atol(arg); streamIntervalMs = (r < 1) ? 1 : r; }
    printStatus();
  } else if (strcmp(cmd, "VMODE") == 0) {
    int v = arg ? atoi(arg) : 0;
    voltOverride = (VoltOverride)constrain(v, 0, 2);
    printStatus();
  } else if (strcmp(cmd, "MOSFET") == 0) {
    int m = arg ? atoi(arg) : -1;
    mosfetHold = (m < 0) ? -1 : (m ? 1 : 0);
    printStatus();
  } else if (strcmp(cmd, "CAP") == 0) {
    unsigned long d = arg ? atol(arg) : capDurationMs;
    if (d < 1) d = 1;
    capDurationMs = d;
    runCapture(d);
  } else if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "?") == 0) {
    printStatus();
  } else {
    Serial.print("$ERR,unknown,"); Serial.println(cmd);
  }
}

// ── Low-power idle (sleepMs) ──────────────────────────────────────
// Replaces delay()'s busy-wait with a real CPU idle so the core stops
// switching while we wait.  All variants keep millis()/Serial alive and wake
// on schedule; only the mechanism differs by chip.
#if defined(CFG_SLEEP_WFI)
// Cortex-M (RA4M1, SAMD21): the 1 kHz periodic tick that drives millis()
// (SysTick on SAMD21, an AGT timer on RA4M1) interrupts the core every ms, so a
// plain WFI loop idles between ticks.  This is "Sleep mode" -- CPU clock gated,
// peripheral clocks still running.  For a deeper cut (RA4M1 Software Standby)
// the high-speed clock must be stopped and woken from a standby-capable timer;
// that's a larger, board-specific change left out here.
void sleepMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    __WFI();   // CMSIS intrinsic
  }
}

#elif defined(CFG_SLEEP_RP2040)
// RP2040 (arduino-pico): millis() is driven by the hardware timer, not SysTick,
// so a bare WFI could stall (no periodic IRQ to wake it).  Instead arm a one-
// shot timer alarm for the wait and WFI until that alarm IRQ fires.  The core
// is clock-gated for the interval while clk_sys keeps running -- i.e. light
// sleep, not DORMANT.  It wakes early on any other IRQ (e.g. USB), which is
// harmless because the millis() bound re-checks the deadline.
#include "pico/time.h"
#include "hardware/sync.h"
static volatile bool _wake;
static int64_t _wakeAlarm(alarm_id_t, void *) { _wake = true; return 0; }
void sleepMs(unsigned long ms) {
  _wake = false;
  alarm_id_t id = add_alarm_in_ms(ms, _wakeAlarm, NULL, true);
  if (id <= 0) { delay(ms); return; }            // couldn't arm: fall back, never stall
  unsigned long start = millis();
  while (!_wake && (millis() - start < ms)) {
    __wfi();
  }
  cancel_alarm(id);
}
// NOTE: for a much deeper cut the RP2040 can stop clk_sys (SLEEP) or the
// oscillators (DORMANT) via pico/sleep.h, woken by the AON timer or a GPIO.
// That needs clock reconfiguration + a defined wake source and must be bench-
// verified, so it is intentionally kept out of this portable path.
#endif

// Accumulate serial bytes into cmdBuf; dispatch on newline.
void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) { cmdBuf[cmdLen] = '\0'; handleLine(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
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

  pinMode(SENSE_POS, INPUT);
  pinMode(SENSE_NEG, INPUT);

  // Enable the onboard NeoPixel's power rail if this board has one.
#ifdef RGB_POWER_PIN
  pinMode(RGB_POWER_PIN, OUTPUT);
  digitalWrite(RGB_POWER_PIN, HIGH);
#endif

  pixel.begin();
  pixel.clear();
  pixel.show();

  Serial.println("OpenLeadDetect_XIAO_Minimal ready.");
}

// ==================================================================
//  MAIN LOOP
// ==================================================================
void loop() {
  pollSerial();

  // ── Diagnostic mode ──────────────────────────────────────────────
  if (diagMode) {
    if (mosfetHold >= 0) {
      // Manual MOSFET hold: detection paused, pin parked for observation.
      digitalWrite(MOSFET_PIN, mosfetHold ? MOSFET_ON : MOSFET_OFF);
    } else {
      runDetection();          // detection still runs (LED stays meaningful)
    }

    if (streamOn && (millis() - lastStreamMs >= streamIntervalMs)) {
      lastStreamMs = millis();
      streamSample();
    }

    updateLed();
    delay(1);                  // light idle; capture/streaming set their own pace
    return;
  }

  // ── Normal mode ──────────────────────────────────────────────────
  runDetection();
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

  sleepMs(generalDelay); // pace the loop with a real CPU idle (per-board sleep)
}
