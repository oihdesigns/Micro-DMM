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
 *   slow dim-red blink (1 Hz, 25%) = charging / USB powered (RA4M1 only);
 *     normal alerts suppressed until unplugged or re-enabled via !ALERTS,1
 *   slow green blink = charging and battery full (>= CFG_BATT_FULL_PCT)
 *   power-on cue (RA4M1): 1-4 green blinks = battery charge level
 *     (0-25/25-50/50-75/75-100 %)
 *
 * Speaker (SPEAKER_PIN, on/off, HIGH = sound) -- mirrors the LED, tunable in the
 * "Speaker / audio alert" block.  Continuity and voltage each have independent
 * on-time, off-time, pulse count, and a *_REPEAT flag (1 = repeat while the
 * state holds at *_REPEAT_MS, 0 = beep once).  Defaults:
 *   closed  = single beep, repeated while continuity persists
 *   voltage = double beep, once
 *   both rate-capped so intermittent detections can't beep faster than 2 Hz;
 *   silenced while charging (same lockout as the LED alerts)
 *   boot mute: if the leads read CLOSED on the first detection at power-up, the
 *     speaker stays silent until the next boot cycle (short leads while booting
 *     to mute); floating or voltage present at boot leaves audio enabled
 *
 * ── Serial diagnostic protocol (115200 baud, line based) ──────────
 * Commands in  (each terminated with newline):
 *   !DIAG[,0|1]      enter/exit diagnostic mode (bare = toggle)
 *   !STREAM[,0|1]    continuous raw streaming on/off (diag only)
 *   !RATE,<ms>       stream interval in ms
 *   !VMODE,<0|1|2>   voltage mode: 0=auto  1=lock ON  2=disable
 *   !MOSFET,<-1|0|1> MOSFET: -1=auto(run detection) 0=hold off 1=hold on
 *   !NEGFIX[,0|1|<v>] pin A2 to a fixed value (bare=toggle, <v>=set volts & on)
 *   !THRESH[,<v>]    set open/closed differential threshold in volts (bare=report)
 *   !ALERTS[,0|1]    re-enable normal LED alerts while charging (1=on,0=blink)
 *   !CAL[,0|1]       A5 threshold-calibration sweep (bare=toggle; diag only)
 *   !OLOG[,0|1]      arm/disarm offline battery log (unplug=start, replug=stop)
 *   !ODUMP           stream the stored offline log
 *   !CAP[,<ms>]      capture ADC across a MOSFET toggle, then dump
 *   !STATUS  / !?    print current status
 * Data out:
 *   $STATUS,...,stream=..,cal=..,rate=..,openthr=..,adaptthr=..,negfix=..,negv=..,charge=..,alertovr=..,olog=..,ocount=..,battpct=..,battv=..
 *     (adaptthr=1 -> openthr is derived live from A5, see CFG_A5_THRESH)
 *     (olog: 0=idle 1=armed 2=logging 3=ready; ocount = points stored)
 *   $DIAG,<ms>,<rawPos>,<rawNeg>,<posV>,<negV>,<diffV>      (streaming)
 *   $CAL,<ms>,<a5V>,<testV>                                 (calibration sweep)
 *   $OFFSTART,<n>,<fullScale>,<vref> / $OFF,<i>,<a5raw>,<diffraw> / $OFFEND
 *   $A5,<ms>,<volts>                                        (RA4M1 A5 monitor)
 *   $CAPSTART,<n>,<toggleUs>,<durMs>,<fullScale>,<vref>     (capture header)
 *   $CAP,<t_us>,<rawPos>,<rawNeg>                           (capture rows)
 *   $CAPEND
 */

#include <Adafruit_NeoPixel.h>
#include <ctype.h>

// ══════════════════════════════════════════════════════════════════
//  BOARD SELECT  —  uncomment EXACTLY ONE line for the chip you build
// ══════════════════════════════════════════════════════════════════
#define BOARD_XIAO_RA4M1
//#define BOARD_XIAO_SAMD21
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
  #define LED_PIN             10     // onboard RGB  "RGB_BUILTIN" or "10"
  #define RGB_POWER_PIN       PIN_RGB_EN      // onboard RGB power enable
  #define CFG_OPEN_THRESH_V   0.250f          // self-contained proto
  #define CFG_REF_BAND_V      0.03f
  //#define CFG_OPEN_THRESH_V 0.1f            // <- breadboard proto alt (then comment the line above)
  #define CFG_SLEEP_WFI       1               // Cortex-M4 WFI idle
  // Charge / USB-power detect: A3 is tied to VBUS through a 1:1 divider, so
  // A3 ~= VBUS/2.  Off battery VBUS ~= 0; plugged into USB VBUS ~= 5 V (A3
  // ~= 2.5 V).  Above CFG_CHARGE_THRESH_V we treat the unit as "charging" and
  // suppress the normal alerts.  Only the RA4M1 board has this charge circuit;
  // the SAMD21/RP2040 protos always see high VBUS, so they leave it undefined.
  #define CFG_CHARGE_DETECT   1
  #define CFG_CHARGE_PIN      A3
  #define CFG_CHARGE_THRESH_V 2.0f            // A3 volts (VBUS/2) above this = charging
  // Battery monitor: the XIAO RA4M1 has a built-in battery-sense divider on
  // BAT_DET_PIN (P105) that reads ~Vbatt/2.  The divider is gated by
  // BAT_READ_EN (P400): it MUST be driven HIGH to enable a reading, otherwise
  // BAT_DET_PIN floats.  We only track voltage/percentage (no presence test --
  // there's no reliable way to know a cell is fitted).  Single Li cell mapped
  // linearly EMPTY..FULL for the percentage.  RA4M1 only.
  #define CFG_BATT_MONITOR    1
  #define CFG_BATT_PIN        BAT_DET_PIN     // P105, onboard battery sense (Vbatt/2)
  #define CFG_BATT_EN_PIN     BAT_READ_EN     // P400, HIGH = enable battery sense
  #define CFG_BATT_DIV        2.0f            // BAT_DET_PIN = Vbatt/2 -> multiply back up
  #define CFG_BATT_EMPTY_V    3.30f           // 0%
  #define CFG_BATT_FULL_V     4.20f           // 100%
  #define CFG_BATT_FULL_PCT   90              // >= this % while charging = green blink
  // Spare-pin (A5) features (RA4M1 only).  CFG_A5_PIN is shared by both.
  #define CFG_A5_PIN          A5
  #define CFG_A5_MONITOR      1     // continually print $A5,<ms>,<volts> for probing
  // Adaptive open/closed threshold: derive the differential threshold live from
  // the A5 voltage via A5_THRESH_TABLE (piecewise-linear).  Comment this line
  // out for a fixed threshold.  When enabled it OVERRIDES CFG_OPEN_THRESH_V and
  // any !THRESH command (they are recomputed from A5 every detection pass).
  #define CFG_A5_THRESH       1
  // Offline calibration log (RA4M1): arm over USB (!OLOG,1), unplug to run on
  // battery -- the board then logs A5 + post-toggle diff to RAM (cleaner than
  // sampling over the USB link).  Replug and the GUI pulls the log (!ODUMP).
  // Needs A3 (VBUS sense) + A5; leave undefined on the other boards.
  #define CFG_OFFLINE_LOG     1

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
const int   ADC_RESOLUTION  = CFG_ADC_RESOLUTION; //This line still has to be commented out for the RP2040 and SAMD21
const float ADC_FULL_SCALE  = CFG_ADC_FULL_SCALE;

// ── Detection thresholds ─────────────────────────────────────────
// Differential (A0 - A2) rests near 0 V, so the band is centred on 0
// and detection looks at the magnitude of the deviation.  Retune
// OPEN_THRESH_V against real closed/open readings on the bench.

const float REF_CENTER_V   = 0.00f;   // resting differential (~0 V)

const unsigned long SETTLE_Post_MS = 3;     // MOSFET-off settle before test read
const unsigned long SETTLE_Pre_uS = 300;

float OPEN_THRESH_V = CFG_OPEN_THRESH_V;  // |dev| below = open, at/above = closed
                                          // (runtime-tunable via !THRESH; overwritten
                                          //  every pass from A5 when CFG_A5_THRESH is on)
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
const float  VOLT_FAST_MULT   = 5.0f;  // single-read "voltage present" shortcut
const int    TEST_MAX_ATTEMPTS = 30;   // safety cap on MOSFET test repeats

// Display/alert debounce: a newly detected state must repeat for this many
// detection passes in a row before the LED switches to it.  This is what keeps
// the alert steady, independent of TEST_AGREE_COUNT -- so TEST_AGREE_COUNT can
// be lowered to 1 (single fast test) without the LED bouncing at a noisy
// open/closed boundary.  Set to 1 for no display debounce.
const int    STATE_STABLE_COUNT = 2;   // detection passes a new state must repeat

// ── Pin assignments ──────────────────────────────────────────────
const int SENSE_POS  = A2;   // pseudo-differential positive input
const int SENSE_NEG  = A0;   // pseudo-differential negative input
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

// ── Speaker / audio alert ────────────────────────────────────────
// Simple on/off speaker on SPEAKER_PIN (HIGH = sound, LOW = silent), driven
// non-blocking exactly like the LED.  Each alert plays a short "beep sequence"
// of one or more pulses, and continuity/voltage each have their own on-time,
// off-time (gap between pulses), pulse count, and repeat behaviour:
//   *_ON_MS / *_OFF_MS     duration of each pulse and the gap between pulses
//   *_BEEP_PULSES          how many pulses make up one beep (1 = single, 2 = double)
//   *_REPEAT               1 = keep re-beeping while the state holds, 0 = beep once
//   *_REPEAT_MS            period between repeats when *_REPEAT is 1
// A new sequence can't start until BEEP_MIN_MS after the previous one began, so
// a bouncing / intermittent detection can never beep faster than
// 1000/BEEP_MIN_MS Hz (2 Hz here).  Tune all behaviour from the block below.
#define SPEAKER_PIN 9
#define SPEAKER_ON  HIGH
#define SPEAKER_OFF LOW

const unsigned long BEEP_MIN_MS = 500;   // min gap between sequence starts (2 Hz cap, both alerts)

// Continuity (CLOSED) beep -------------------------------------------------
// The initial contact beep (leads just closed) and the ongoing "still there"
// repeat beeps have independent on/off durations, so they can sound different.
const int           CONT_BEEP_PULSES = 1;     // pulses per beep (1 = single beep)
const unsigned long CONT_ON_MS       = 20;    // initial contact: on-time of each pulse
const unsigned long CONT_OFF_MS      = 10;    // initial contact: off-time between pulses (only matters if pulses > 1)
const int           CONT_REPEAT      = 1;     // 1 = repeat while closed, 0 = beep once
const unsigned long CONT_REPEAT_MS   = 1000;  // repeat period when CONT_REPEAT
const unsigned long CONT_ONGOING_ON_MS  = 50; // ongoing "still there": on-time of each pulse
const unsigned long CONT_ONGOING_OFF_MS = 10;  // ongoing "still there": off-time between pulses

// Voltage (VOLTAGE) beep ---------------------------------------------------
const int           VOLT_BEEP_PULSES = 2;     // pulses per beep (2 = double beep)
const unsigned long VOLT_ON_MS       = 20;    // on-time of each pulse
const unsigned long VOLT_OFF_MS      = 10;    // off-time between pulses
const int           VOLT_REPEAT      = 0;     // 1 = repeat while voltage, 0 = beep once
const unsigned long VOLT_REPEAT_MS   = 1000;  // repeat period when VOLT_REPEAT

// Non-blocking beep-sequence state (owned by the speaker driver further down).
bool          beepOn           = false;  // is the speaker output currently HIGH
int           beepPulsesLeft   = 0;      // pulses still to START in this sequence
unsigned long beepPhaseStart   = 0;      // ms timestamp of the current on/off phase
unsigned long lastBeepSeqStart = 0;      // ms timestamp the last sequence began
unsigned long beepOnMs         = 20;     // on-time  for the sequence in progress
unsigned long beepOffMs        = 10;     // off-time for the sequence in progress

// Boot-time speaker mute (no dedicated input needed): if the leads read CLOSED
// on the first detection at power-up, the speaker stays silent for the whole
// session -- short the leads while booting to mute, then boot-cycle to restore.
// Floating leads (or voltage present) at boot leave the speaker enabled.
bool speakerMuted = false;

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

// ── Charge / USB-power lockout ────────────────────────────────────
// When charging (see CFG_CHARGE_DETECT), the normal float/closed/voltage
// alerts are suppressed and replaced by a slow dim-red "charging" blink:
// 1 Hz, 25% duty (250 ms on / 750 ms off), brightness 64.  The GUI can force
// the normal alerts back on for the current session via !ALERTS,1; that
// override auto-clears when the unit is unplugged.  These are declared on all
// boards so the command/status path is uniform, but chargeActive only ever
// goes true where CFG_CHARGE_DETECT reads the VBUS-sense pin.
bool          chargeActive    = false;  // VBUS sense above threshold (USB in)
bool          alertOverride   = false;  // user re-enabled normal alerts while charging
bool          chargeBlinkOn   = false;
unsigned long lastChargeBlink = 0;
const unsigned long CHARGE_BLINK_PERIOD_MS = 2000;  // 2 Hz
const unsigned long CHARGE_BLINK_ON_MS     = 250;   // 25% duty
const uint8_t       CHARGE_BLINK_BRIGHT    = 25;    // red brightness while charging

// ── Battery monitor ───────────────────────────────────────────────
// Latest battery reading, refreshed each detection pass by updateChargeState()
// (RA4M1 only -- see CFG_BATT_MONITOR).  There is no reliable way to know if a
// cell is physically present, so we only track voltage / percentage: battPct
// drives the green-vs-red charging blink and the power-on level cue.
float battV   = 0.0f;   // measured battery voltage (BAT_DET_PIN * divider)
int   battPct = 0;      // 0..100 %, linear EMPTY..FULL

unsigned long lastSerialTime = 0;
const unsigned long serialInterval = 250;   // ms between debug prints
const unsigned long generalDelay = 50;

#ifdef CFG_A5_MONITOR
unsigned long lastA5Time = 0;
const unsigned long a5Interval = 500;       // ms between A5 reads/prints (2 Hz)
#endif

// Most recent values, kept for serial debug
float lastRestV = 0.0f;
float lastTestV = 0.0f;

// ── Diagnostic mode ───────────────────────────────────────────────
bool diagMode = false;                 // suppresses human debug, enables $ protocol
bool streamOn = false;                 // continuous raw streaming
unsigned long streamIntervalMs = 20;   // streaming period (~50 Hz default)
unsigned long lastStreamMs = 0;

// Calibration sweep (RA4M1/A5): when on, detection is paused and each cycle
// samples the A5 control voltage plus the post-MOSFET-toggle differential,
// emitting $CAL rows.  Sweep the pot to map A5 -> threshold for a known
// resistor.  Uses the streamIntervalMs cadence.
bool calMode = false;
unsigned long lastCalMs = 0;

enum VoltOverride { VOLT_AUTO, VOLT_FORCE_ON, VOLT_DISABLED };
VoltOverride voltOverride = VOLT_AUTO; // applies whenever detection runs

int mosfetHold = -1;                    // -1 auto (run detection), 0 hold off, 1 hold on

// Fixed-negative override (diagnostic): replace the live A2 read with a
// constant so the differential isolates A0 / board noise from A2-reference
// noise.  A2 should rest near 1.250 V; pinning it to that value means any
// remaining ripple in the diff is coming from A0 or the supply, not from the
// reference drifting.  Applies to detection, streaming, AND capture.
bool  negFixed  = false;     // false = read A2 normally, true = use negFixedV
float negFixedV = 1.250f;    // fixed pseudo-reference voltage when negFixed

// Transient capture buffer (raw counts; volts computed by the host)
const int CAP_MAX_SAMPLES   = 600;      // ~4.8 KB of RAM
const unsigned long CAP_PRE_US = 500;   // baseline sampled before the toggle
unsigned long capDurationMs = 5;        // total capture window
uint32_t capT[CAP_MAX_SAMPLES];
uint16_t capPos[CAP_MAX_SAMPLES];
uint16_t capNeg[CAP_MAX_SAMPLES];
int capCount = 0;

#ifdef CFG_OFFLINE_LOG
// Offline log: filled on battery after USB is unplugged, dumped on reconnect.
// Store raw counts (a5 + signed post-toggle diff); the host converts to volts
// with the full-scale/vref sent in the $OFFSTART header.  No timestamps (the
// A5-vs-diff relationship is what matters, not time).
//   OFF_IDLE   -> disarmed
//   OFF_ARMED  -> armed on USB, waiting for the unplug edge
//   OFF_LOGGING-> on battery, recording (serial stays SILENT here)
//   OFF_READY  -> replugged, data held in RAM until dumped/cleared
enum OfflineState { OFF_IDLE, OFF_ARMED, OFF_LOGGING, OFF_READY };
OfflineState offlineState = OFF_IDLE;
const int OFFLINE_MAX = 1500;               // ~6 KB (uint16 + int16 per point)
const unsigned long OFFLINE_INTERVAL_MS = 25;   // ~40 Hz while logging
uint16_t offA5[OFFLINE_MAX];
int16_t  offDiff[OFFLINE_MAX];
int offCount = 0;
unsigned long lastOfflineMs = 0;
#endif

// Serial command line buffer
char cmdBuf[48];
int  cmdLen = 0;

// ── Helpers ──────────────────────────────────────────────────────
// Negative-channel read.  Normally a live A2 sample, but when negFixed is on
// it returns the raw count corresponding to negFixedV instead, so the diff
// isolates A0 / board noise from A2-reference noise.  Centralised here so
// detection, streaming, and capture all honour the override identically.
int readNegRaw() {
  if (negFixed) {
    return (int)((negFixedV / ADC_REF_VOLTAGE) * ADC_FULL_SCALE + 0.5f);
  }
  return analogRead(SENSE_NEG);
}

// Pseudo-differential read: sample both pins vs. GND and subtract.
// The RA4M1 shares one ADC + sample/hold behind an input mux, so the first
// conversion after a channel switch carries residual charge from the previous
// channel (worst when that channel sat at a high-impedance mid-scale voltage,
// e.g. the A5 pot monitor).  Discard one throwaway conversion per channel to
// let the S/H cap settle before the real read.  The fast capture path keeps
// using single reads (readNegRaw) so its cadence is unaffected.
float readVoltage() {
  analogRead(SENSE_POS);                 // throwaway: settle S/H after prior channel
  int rawPos = analogRead(SENSE_POS);
  if (!negFixed) analogRead(SENSE_NEG);  // throwaway: settle after SENSE_POS
  int rawNeg = readNegRaw();
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
  sleepMs(SETTLE_Post_MS);            // CPU idles during settle
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

// ── Speaker driver ───────────────────────────────────────────────
// Mirrors the LED model: startBeep() kicks off a rate-limited beep sequence,
// updateBeep() advances its on/off phases without blocking, and updateSpeaker()
// maps leadState -> beeps (honouring the same charge lockout as the LED).

// Begin a sequence of `pulses` beeps with the given on/off timing, unless one
// is still playing or the 2 Hz rate cap (BEEP_MIN_MS since the last start)
// hasn't elapsed yet.  The timing is latched so the sequence keeps its own
// cadence even if another state's constants differ.
void startBeep(unsigned long now, int pulses, unsigned long onMs, unsigned long offMs) {
  if (beepOn || beepPulsesLeft > 0)         return;   // a sequence is still playing
  if (now - lastBeepSeqStart < BEEP_MIN_MS) return;   // 2 Hz cap on new sequences
  lastBeepSeqStart = now;
  beepPulsesLeft   = pulses;
  beepOnMs         = onMs;
  beepOffMs        = offMs;
  beepPhaseStart   = now;
  beepOn           = true;
  digitalWrite(SPEAKER_PIN, SPEAKER_ON);              // first pulse on
  beepPulsesLeft--;
}

// Advance the current sequence: end each pulse after beepOnMs, then start the
// next one (if any remain) after beepOffMs.  No-op when idle.
void updateBeep(unsigned long now) {
  if (!beepOn && beepPulsesLeft == 0) return;         // idle
  if (beepOn) {
    if (now - beepPhaseStart >= beepOnMs) {
      digitalWrite(SPEAKER_PIN, SPEAKER_OFF);
      beepOn         = false;
      beepPhaseStart = now;
    }
  } else if (beepPulsesLeft > 0 && now - beepPhaseStart >= beepOffMs) {
    digitalWrite(SPEAKER_PIN, SPEAKER_ON);
    beepOn         = true;
    beepPhaseStart = now;
    beepPulsesLeft--;
  }
}

// Immediately silence the speaker and abandon any in-progress sequence.
void silenceSpeaker() {
  if (beepOn) digitalWrite(SPEAKER_PIN, SPEAKER_OFF);
  beepOn         = false;
  beepPulsesLeft = 0;
}

// Map the detection state to audio, mirroring updateLed().  Each state beeps
// once on entry; if its *_REPEAT flag is set it keeps re-beeping every
// *_REPEAT_MS while the state holds.
//   CLOSED  -> continuity beep (CONT_* constants)
//   VOLTAGE -> voltage beep   (VOLT_* constants)
//   FLOAT   -> silent (any in-progress beep is left to finish naturally)
// Suppressed while charging, matching the LED alert lockout.
void updateSpeaker() {
  static LeadState prevState = STATE_VOLTAGE;
  unsigned long now = millis();

  if (speakerMuted) {                     // muted for this session (see boot check)
    silenceSpeaker();
    prevState = leadState;
    return;
  }

#ifdef CFG_CHARGE_DETECT
  if (chargeActive && !alertOverride) {   // charging: stay quiet like the LED
    silenceSpeaker();
    prevState = leadState;                // avoid a stale beep on unplug
    return;
  }
#endif

  bool entered = (leadState != prevState);
  prevState = leadState;

  if (leadState == STATE_CLOSED) {
    if (entered)                          // just made contact: initial beep
      startBeep(now, CONT_BEEP_PULSES, CONT_ON_MS, CONT_OFF_MS);
    else if (CONT_REPEAT && now - lastBeepSeqStart >= CONT_REPEAT_MS)
      startBeep(now, CONT_BEEP_PULSES, CONT_ONGOING_ON_MS, CONT_ONGOING_OFF_MS);
  } else if (leadState == STATE_VOLTAGE) {
    if (entered || (VOLT_REPEAT && now - lastBeepSeqStart >= VOLT_REPEAT_MS))
      startBeep(now, VOLT_BEEP_PULSES, VOLT_ON_MS, VOLT_OFF_MS);
  }

  updateBeep(now);
}

// One clean open/closed measurement at boot to decide the session mute.  Mirrors
// runDetection()'s sequence (adaptive threshold -> voltage check -> MOSFET test)
// but writes only speakerMuted, never leadState.  Leads shorted (CLOSED) at boot
// -> speaker muted until the next boot; floating or voltage present -> audio on.
void bootSpeakerMuteCheck() {
#ifdef CFG_A5_THRESH
  updateAdaptiveThreshold();             // threshold this pass comes from A5
#endif
  digitalWrite(MOSFET_PIN, MOSFET_ON);   // resting state
  if (voltagePresent()) return;          // voltage at boot -> leave audio enabled
  if (runMosfetTestStable() == STATE_CLOSED) speakerMuted = true;
}

#ifdef CFG_CHARGE_DETECT
const int   CHARGE_PIN     = CFG_CHARGE_PIN;
const float CHARGE_THRESH_V = CFG_CHARGE_THRESH_V;

// Read the VBUS-sense pin (A3 = VBUS/2) in volts.
float readChargeV() {
  return (analogRead(CHARGE_PIN) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
}

#ifdef CFG_BATT_MONITOR
const int   BATT_PIN       = CFG_BATT_PIN;
const int   BATT_EN_PIN    = CFG_BATT_EN_PIN;
const float BATT_DIV       = CFG_BATT_DIV;
const float BATT_EMPTY_V   = CFG_BATT_EMPTY_V;
const float BATT_FULL_V    = CFG_BATT_FULL_V;

// Battery voltage via the onboard sense divider (BAT_DET_PIN ~= Vbatt/2, scaled
// back up).  BAT_READ_EN must already be HIGH (set in setup) for this to read a
// real value rather than a floating pin.
float readBattV() {
  return (analogRead(BATT_PIN) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE * BATT_DIV;
}

// Linear state-of-charge between EMPTY (0%) and FULL (100%), clamped.
int battPercentOf(float v) {
  float pct = (v - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100.0f;
  return (int)constrain(pct, 0.0f, 100.0f);
}
#endif

// Update chargeActive from the VBUS sense (and refresh the battery reading).
// VBUS high = USB plugged in -> charging; VBUS low = running on battery.  When
// the unit is unplugged the per-session alert override is cleared so the next
// charge starts in the suppressed-alert (charging-blink) state.
void updateChargeState() {
  chargeActive = (readChargeV() > CHARGE_THRESH_V);
  if (!chargeActive) alertOverride = false;
#ifdef CFG_BATT_MONITOR
  battV   = readBattV();
  battPct = battPercentOf(battV);
#endif
}
#endif

#if defined(CFG_A5_MONITOR) || defined(CFG_A5_THRESH)
const int A5_MONITOR_PIN = CFG_A5_PIN;

// Read the spare monitor pin (A5) in volts.  RA4M1 only.  A5 is high-impedance,
// so discard one conversion after the channel switch to let the S/H settle.
float readA5V() {
  analogRead(A5_MONITOR_PIN);            // throwaway: settle S/H after prior channel
  return (analogRead(A5_MONITOR_PIN) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
}
#endif

#ifdef CFG_A5_THRESH
// Adaptive open/closed threshold from the A5 voltage: piecewise-linear
// interpolation of the bench-calibrated table below.  Values below the first /
// above the last A5 point are clamped to the end thresholds.
//
// Fitted from a 5-resistance A5 sweep (10M/1M/330K/10K/1R).  Goal: trigger
// (CLOSED, |diff| <= thr) at <=330K, reject (OPEN) at >=1M.  Post-toggle |diff|
// rises with resistance, so the threshold rides between the 330K upper envelope
// (p95) and the 1M lower envelope (p05): midpoint where they separate, and just
// under the 1M lower envelope where they overlap (bias to NO false positives).
// NOTE: 330K and 1M are only cleanly separable for A5 >~ 0.85 V; below that the
// front end can't distinguish them and some misclassification is unavoidable
// (a hardware-revision finding, not a table-tuning one).  Recalibrate via the
// GUI "Calibrate A5" sweep if the analog front end changes.
struct A5ThreshPoint { float a5V; float thrV; };
const A5ThreshPoint A5_THRESH_TABLE[] = {
  {0.00f, 0.033f},
  {0.15f, 0.059f},
  {0.35f, 0.067f},
  {0.50f, 0.139f},
  {0.70f, 0.317f},
  {0.85f, 0.400f},
  {1.00f, 0.455f},
  {1.20f, 0.504f},
  {1.35f, 0.535f},
  {1.55f, 0.564f},
  {1.70f, 0.580f},
};
const int A5_THRESH_N = sizeof(A5_THRESH_TABLE) / sizeof(A5_THRESH_TABLE[0]);

float a5ToThreshold(float a5V) {
  if (a5V <= A5_THRESH_TABLE[0].a5V)               return A5_THRESH_TABLE[0].thrV;
  if (a5V >= A5_THRESH_TABLE[A5_THRESH_N - 1].a5V) return A5_THRESH_TABLE[A5_THRESH_N - 1].thrV;
  for (int i = 1; i < A5_THRESH_N; i++) {
    if (a5V < A5_THRESH_TABLE[i].a5V) {
      const A5ThreshPoint &lo = A5_THRESH_TABLE[i - 1];
      const A5ThreshPoint &hi = A5_THRESH_TABLE[i];
      float frac = (a5V - lo.a5V) / (hi.a5V - lo.a5V);
      return lo.thrV + frac * (hi.thrV - lo.thrV);
    }
  }
  return A5_THRESH_TABLE[A5_THRESH_N - 1].thrV;     // unreachable
}

// Refresh OPEN_THRESH_V from A5 (call once per detection pass).  Averages a few
// A5 reads so a single noisy sample can't jerk the threshold around.
void updateAdaptiveThreshold() {
  const int N = 4;
  float sum = 0.0f;
  for (int i = 0; i < N; i++) sum += readA5V();
  OPEN_THRESH_V = a5ToThreshold(sum / N);
}
#endif

// Slow "charging" blink: 1 Hz, 25% duty (CHARGE_BLINK_ON_MS on, the remainder
// of CHARGE_BLINK_PERIOD_MS off), brightness CHARGE_BLINK_BRIGHT.  Red while
// charging; green once the battery is full (>= CFG_BATT_FULL_PCT).
void chargeBlink() {
  unsigned long now   = millis();
  unsigned long phase = now - lastChargeBlink;
  if (!chargeBlinkOn && phase >= CHARGE_BLINK_PERIOD_MS) {
    chargeBlinkOn   = true;
    lastChargeBlink = now;
    uint8_t r = CHARGE_BLINK_BRIGHT, g = 0;
#ifdef CFG_BATT_MONITOR
    if (battPct >= CFG_BATT_FULL_PCT) { r = 0; g = CHARGE_BLINK_BRIGHT; }
#endif
    setPixel(r, g, 0);
  } else if (chargeBlinkOn && phase >= CHARGE_BLINK_ON_MS) {
    chargeBlinkOn = false;
    setPixel(0, 0, 0);
  }
}

// LED owner: while charging (and not overridden by !ALERTS,1) show the
// charging blink and suppress the detection alerts; otherwise run updateLed().
// On a transition between the two modes, blank the pixel and reset both sets
// of blink flags so neither mode inherits a half-finished flash.
void updateAlerts() {
#ifdef CFG_OFFLINE_LOG
  // Offline log states own the LED (on USB): slow blue = armed & waiting for
  // unplug; slow green = data captured & ready to pull.  (Logging itself runs
  // on battery and drives the LED from the loop's logging branch.)
  if (offlineState == OFF_ARMED || offlineState == OFF_READY) {
    static unsigned long last = 0;
    static bool on = false;
    unsigned long now = millis();
    if (!on && now - last >= 1400) {
      on = true; last = now;
      if (offlineState == OFF_ARMED) setPixel(0, 0, 25);   // blue: armed
      else                           setPixel(0, 20, 0);   // green: ready
    } else if (on && now - last >= 80) {
      on = false; setPixel(0, 0, 0);
    }
    return;
  }
#endif
  static bool prevCharging = false;
  bool charging = (chargeActive && !alertOverride);
  if (charging != prevCharging) {
    prevCharging   = charging;
    chargeBlinkOn  = false;
    floatFlashing  = false;
    closedFlashing = false;
    voltFlashing   = false;
    setPixel(0, 0, 0);
  }
  if (charging) chargeBlink();
  else          updateLed();
}

#ifdef CFG_BATT_MONITOR
// Power-on charge-level cue (RA4M1), run once from setup(): 1..4 slow green
// blinks showing battery level (0-25%=1, 25-50%=2, 50-75%=3, 75-100%=4).
// Always shown at boot regardless of power source -- there is no battery-
// presence test, so this simply reflects the measured battery voltage.
void startupBatteryIndicate() {
  // BAT_READ_EN was only just driven HIGH in setup(), so the sense node hasn't
  // settled and the ADC's first conversions on a freshly-selected channel read
  // low.  Discard reads over a short settling window, THEN average -- otherwise
  // the blink count reflects a still-rising voltage (e.g. 1 blink for a full
  // cell that the running loop later reads correctly).
  for (int i = 0; i < 20; i++) { readBattV(); delay(5); }   // ~100 ms settle + discard
  float v = 0.0f;
  for (int i = 0; i < 8; i++) { v += readBattV(); delay(2); }
  v /= 8.0f;

  int blinks = battPercentOf(v) / 25 + 1;     // 1..4 (100% -> 5, clamped below)
  if (blinks > 4) blinks = 4;
  sleepMs(400);                               // brief gap before the count
  for (int i = 0; i < blinks; i++) {
    setPixel(0, CHARGE_BLINK_BRIGHT, 0);
    sleepMs(200);
    setPixel(0, 0, 0);
    sleepMs(250);
  }
}
#endif

// ==================================================================
//  DETECTION (one pass) -- sets leadState, honouring voltOverride
// ==================================================================
void runDetection() {
#ifdef CFG_A5_THRESH
  updateAdaptiveThreshold();             // refresh OPEN_THRESH_V from A5 this pass
#endif
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
  Serial.print(",cal=");          Serial.print(calMode ? 1 : 0);
  Serial.print(",rate=");         Serial.print(streamIntervalMs);
  Serial.print(",capms=");        Serial.print(capDurationMs);
  Serial.print(",res=");          Serial.print(ADC_RESOLUTION);
  Serial.print(",vref=");         Serial.print(ADC_REF_VOLTAGE, 3);
  Serial.print(",openthr=");      Serial.print(OPEN_THRESH_V, 3);
#ifdef CFG_A5_THRESH
  Serial.print(",adaptthr=1");    // openthr is driven live from A5 (read-only)
#else
  Serial.print(",adaptthr=0");
#endif
  Serial.print(",negfix=");       Serial.print(negFixed ? 1 : 0);
  Serial.print(",negv=");         Serial.print(negFixedV, 3);
  Serial.print(",charge=");       Serial.print(chargeActive ? 1 : 0);
  Serial.print(",alertovr=");     Serial.print(alertOverride ? 1 : 0);
#ifdef CFG_OFFLINE_LOG
  Serial.print(",olog=");         Serial.print((int)offlineState);
  Serial.print(",ocount=");       Serial.print(offCount);
#endif
#ifdef CFG_BATT_MONITOR
  Serial.print(",battpct=");     Serial.print(battPct);
  Serial.print(",battv=");       Serial.println(battV, 3);
#else
  Serial.println();
#endif
}

// One streamed sample: both pins independently + computed differential.
// Same per-channel throwaway settle as readVoltage() (see note there) so the
// stream isn't skewed by mux crosstalk from the A5 monitor.
void streamSample() {
  analogRead(SENSE_POS);                 // throwaway: settle S/H after prior channel
  int rawPos = analogRead(SENSE_POS);
  if (!negFixed) analogRead(SENSE_NEG);  // throwaway: settle after SENSE_POS
  int rawNeg = readNegRaw();
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

#if defined(CFG_A5_MONITOR) || defined(CFG_A5_THRESH)
// One calibration sample: read the A5 control voltage, then run the SAME
// post-MOSFET-toggle differential read the detector uses to decide open/closed,
// and emit both.  Sweeping the pot maps A5 -> post-toggle voltage for a known
// threshold resistor, which is how the A5_THRESH_TABLE is built.
// Emits: $CAL,<ms>,<a5V>,<testV>
void calSample() {
  float a5 = 0.0f;
  for (int i = 0; i < 4; i++) a5 += readA5V();   // light average vs. wiper noise
  a5 *= 0.25f;

  // Identical toggle+settle+read to runMosfetTest(), so the captured voltage
  // matches what detection compares against OPEN_THRESH_V.
  digitalWrite(MOSFET_PIN, MOSFET_OFF);
  delayMicroseconds(SETTLE_Pre_uS);
  float testV = readVoltage();
  sleepMs(SETTLE_Post_MS);
  digitalWrite(MOSFET_PIN, MOSFET_ON);           // back to resting

  Serial.print("$CAL,");
  Serial.print(millis()); Serial.print(",");
  Serial.print(a5, 4);    Serial.print(",");
  Serial.println(testV, 4);
}
#endif

#ifdef CFG_OFFLINE_LOG
// One offline log point (battery powered, NO serial): averaged A5 raw + the
// post-toggle differential in raw counts, stored to RAM.  Mirrors calSample()'s
// timing so the offline data lines up with USB-collected calibration data.
void offlineSample() {
  if (offCount >= OFFLINE_MAX) return;           // buffer full

  long a5 = 0;
  for (int i = 0; i < 4; i++) { analogRead(A5_MONITOR_PIN); a5 += analogRead(A5_MONITOR_PIN); }
  a5 /= 4;

  digitalWrite(MOSFET_PIN, MOSFET_OFF);
  delayMicroseconds(SETTLE_Pre_uS);
  analogRead(SENSE_POS); int rp = analogRead(SENSE_POS);   // throwaway + real
  analogRead(SENSE_NEG); int rn = analogRead(SENSE_NEG);
  sleepMs(SETTLE_Post_MS);
  digitalWrite(MOSFET_PIN, MOSFET_ON);

  offA5[offCount]   = (uint16_t)a5;
  offDiff[offCount] = (int16_t)(rp - rn);
  offCount++;
}

// State machine: refresh the VBUS/charge reading, then arm -> log -> ready on
// the USB unplug / replug edges.  Called at the very top of loop() so LOGGING
// can take over before any serial output happens.
void serviceOfflineLog() {
  updateChargeState();                           // refresh chargeActive (A3)
  switch (offlineState) {
    case OFF_ARMED:
      if (!chargeActive) {                        // USB removed -> start logging
        offCount = 0;
        lastOfflineMs = 0;
        offlineState = OFF_LOGGING;
      }
      break;
    case OFF_LOGGING:
      if (chargeActive) offlineState = OFF_READY; // USB back -> stop, hold data
      break;
    default:
      break;
  }
}

// LED while logging on battery: dim-green heartbeat (~1 s) so you can see it's
// alive as you sweep; fast red once the buffer is full.  Kept brief and
// infrequent to minimise NeoPixel supply noise during the reads.
void offlineLoggingLed() {
  static unsigned long last = 0;
  static bool on = false;
  unsigned long now = millis();
  if (offCount >= OFFLINE_MAX) {                  // full: fast red blink
    if (now - last >= 150) { on = !on; last = now; setPixel(on ? 30 : 0, 0, 0); }
    return;
  }
  if (!on && now - last >= 1000) { on = true;  last = now; setPixel(0, 20, 0); }
  else if (on && now - last >= 20) { on = false;             setPixel(0, 0, 0); }
}

// Dump the RAM log to the host on reconnect.  Host converts raw->volts with the
// full-scale/vref in the header.  Emits $OFFSTART / $OFF rows / $OFFEND.
void dumpOfflineLog() {
  Serial.print("$OFFSTART,");
  Serial.print(offCount);          Serial.print(",");
  Serial.print(ADC_FULL_SCALE, 0); Serial.print(",");
  Serial.println(ADC_REF_VOLTAGE, 3);
  for (int i = 0; i < offCount; i++) {
    Serial.print("$OFF,");
    Serial.print(i);          Serial.print(",");
    Serial.print(offA5[i]);   Serial.print(",");
    Serial.println(offDiff[i]);
  }
  Serial.println("$OFFEND");
}
#endif

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
    capNeg[capCount] = readNegRaw();
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
    if (!diagMode) { streamOn = false; mosfetHold = -1; calMode = false; }
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
  } else if (strcmp(cmd, "NEGFIX") == 0) {
    // !NEGFIX        toggle fixed-neg on/off (keeps current negv)
    // !NEGFIX,0|1    off / on (uses current negv)
    // !NEGFIX,<v>    set the fixed voltage AND enable (e.g. !NEGFIX,1.25)
    if (!arg) {
      negFixed = !negFixed;
    } else if (strchr(arg, '.') != NULL) {
      negFixedV = atof(arg);
      negFixed  = true;
    } else {
      negFixed = (atoi(arg) != 0);
    }
    printStatus();
  } else if (strcmp(cmd, "THRESH") == 0) {
    // !THRESH,<v>  set the open/closed differential threshold in volts
    // (bare = just report current value).  |dev| below = open, at/above = closed.
    if (arg) {
      float t = atof(arg);
      if (t > 0.0f) OPEN_THRESH_V = t;
    }
    printStatus();
  } else if (strcmp(cmd, "ALERTS") == 0) {
    // !ALERTS,1 re-enables the normal LED alerts while charging (override the
    // charge lockout); !ALERTS,0 restores the charging blink; bare = toggle.
    // The override auto-clears on unplug (see updateChargeState).
    alertOverride = arg ? (atoi(arg) != 0) : !alertOverride;
    printStatus();
  } else if (strcmp(cmd, "CAL") == 0) {
    // !CAL[,0|1]  start/stop the A5 calibration sweep (bare = toggle).
    // Emits $CAL,<ms>,<a5V>,<testV> rows; detection is paused while active.
#if defined(CFG_A5_MONITOR) || defined(CFG_A5_THRESH)
    calMode = arg ? (atoi(arg) != 0) : !calMode;
    if (!calMode) digitalWrite(MOSFET_PIN, MOSFET_ON);   // leave resting
#else
    calMode = false;
    Serial.println("$ERR,cal,no A5 pin on this board");
#endif
    printStatus();
  } else if (strcmp(cmd, "OLOG") == 0) {
    // !OLOG,1 arm offline logging; !OLOG,0 disarm + clear; bare = toggle.
    // Once armed, unplug USB to start logging on battery; replug to stop.
#ifdef CFG_OFFLINE_LOG
    int v = arg ? atoi(arg) : -1;
    if (v == 0)      { offlineState = OFF_IDLE;  offCount = 0; }
    else if (v == 1) { offlineState = OFF_ARMED; }
    else             { offlineState = (offlineState == OFF_IDLE) ? OFF_ARMED : OFF_IDLE;
                       if (offlineState == OFF_IDLE) offCount = 0; }
#else
    Serial.println("$ERR,olog,unsupported on this board");
#endif
    printStatus();
  } else if (strcmp(cmd, "ODUMP") == 0) {
    // !ODUMP  stream the RAM log ($OFFSTART / $OFF rows / $OFFEND).
#ifdef CFG_OFFLINE_LOG
    dumpOfflineLog();
#else
    Serial.println("$ERR,odump,unsupported on this board");
#endif
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

  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, SPEAKER_OFF);   // speaker off at boot

#ifdef CFG_CHARGE_DETECT
  pinMode(CHARGE_PIN, INPUT);            // A3 = VBUS/2 (USB-power sense)
#endif
#if defined(CFG_A5_MONITOR) || defined(CFG_A5_THRESH)
  pinMode(A5_MONITOR_PIN, INPUT);        // A5: spare-pin monitor / adaptive threshold
#endif
#ifdef CFG_BATT_MONITOR
  pinMode(BATT_PIN, INPUT);              // BAT_DET_PIN (P105) = Vbatt/2 sense
  pinMode(BATT_EN_PIN, OUTPUT);          // BAT_READ_EN (P400)
  digitalWrite(BATT_EN_PIN, HIGH);       // enable the battery-sense divider
  delay(2);                              // let the divider settle before first read
#endif

  // Enable the onboard NeoPixel's power rail if this board has one.
#ifdef RGB_POWER_PIN
  pinMode(RGB_POWER_PIN, OUTPUT);
  digitalWrite(RGB_POWER_PIN, HIGH);
#endif

  pixel.begin();
  pixel.clear();
  pixel.show();

#ifdef CFG_BATT_MONITOR
  startupBatteryIndicate();              // power-on battery charge-level cue
#endif

  bootSpeakerMuteCheck();                // leads CLOSED at boot -> mute for this session
  Serial.print("Speaker: ");
  Serial.println(speakerMuted ? "MUTED (leads closed at boot)" : "enabled");

  Serial.println("OpenLeadDetect_XIAO_Minimal ready.");
}

// ==================================================================
//  MAIN LOOP
// ==================================================================
void loop() {
  pollSerial();

#ifdef CFG_OFFLINE_LOG
  // Offline log takes priority and runs the arm/log/ready state machine.  While
  // LOGGING (on battery) we sample to RAM and emit NO serial -- USB-CDC writes
  // can block when unplugged, which would stall the log.
  serviceOfflineLog();
  if (offlineState == OFF_LOGGING) {
    if (offCount < OFFLINE_MAX && millis() - lastOfflineMs >= OFFLINE_INTERVAL_MS) {
      lastOfflineMs = millis();
      offlineSample();
    }
    offlineLoggingLed();
    silenceSpeaker();          // no audio while logging on battery
    delay(1);
    return;
  }
#endif

#ifdef CFG_A5_MONITOR
  // Spare-pin monitor (RA4M1): continually report the A5 voltage, in every
  // mode, on its own cadence.  Emitted as a structured $A5 line so a host can
  // parse it while staying readable in a plain serial monitor.
  if (millis() - lastA5Time >= a5Interval) {
    lastA5Time = millis();
    Serial.print("$A5,");
    Serial.print(millis());
    Serial.print(",");
    Serial.println(readA5V(), 4);
  }
#endif

  // ── Diagnostic mode ──────────────────────────────────────────────
  if (diagMode) {
    // Calibration sweep takes priority: pause detection and just emit $CAL
    // rows (A5 voltage + post-toggle differential) at the stream cadence.
    if (calMode) {
#if defined(CFG_A5_MONITOR) || defined(CFG_A5_THRESH)
      if (millis() - lastCalMs >= streamIntervalMs) {
        lastCalMs = millis();
        calSample();
      }
#endif
      updateLed();
      silenceSpeaker();        // detection paused during calibration -> quiet
      delay(1);
      return;
    }

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

#ifdef CFG_CHARGE_DETECT
    updateChargeState();
#endif
    updateAlerts();
    updateSpeaker();
    delay(1);                  // light idle; capture/streaming set their own pace
    return;
  }

  // ── Normal mode ──────────────────────────────────────────────────
  runDetection();
#ifdef CFG_CHARGE_DETECT
  updateChargeState();
#endif
  updateAlerts();
  updateSpeaker();

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
