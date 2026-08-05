/*
 * BlinkyHawk_RA4M1.ino
 *
 * Open / closed lead detector -- Blinky Hawk production firmware.
 * Target: Seeed XIAO RA4M1 ONLY (Renesas RA4M1, 14-bit ADC).
 * Descended from OpenLeadDetect_XIAO_Minimal with the multi-MCU support and
 * the A5 potentiometer / adaptive-threshold features removed.
 *
 * "Pseudo-differential": analogRead() has no native differential mode, so
 * SENSE_POS and SENSE_NEG are each sampled vs. GND and subtracted in
 * software.  Resting differential is ~0 V, so detection works on the
 * magnitude of the deviation from zero.
 *
 * Measurement logic (repeated continuously):
 *   1. MOSFET held HIGH (resting / bridge connected), read the differential.
 *   2. Voltage-present decision (de-noised):
 *        - any single read beyond VOLTFAST * REFBAND -> present;
 *        - otherwise average VOLTAVG reads and compare to the band.
 *      If voltage is present the open/closed test is bypassed.
 *   3. Test (only when no voltage):  MOSFET LOW, derive an open/closed metric,
 *      MOSFET back HIGH.  metric > active threshold -> OPEN (blue), else CLOSED
 *      (green).  How the metric is derived is selectable (cfg.detectMethod):
 *        0 SINGLE  : one differential read after settlePreUs; metric = |diff| (V)
 *                    -- the original method (default; unchanged behaviour).
 *        1 TIMERET : sample the recovery; metric = time (ms) for the differential
 *                    to return within detReturnBand of the resting centre.  A
 *                    dead short recovers fastest, an open lead slowest, so a
 *                    LONGER time = MORE OPEN.
 *        2 AREA    : metric = tail-windowed integral (V*ms) of |diff-centre|
 *                    from detAreaStartUs to detWindowUs.  LARGER area = MORE OPEN.
 *      All three keep "larger metric = more open", so the DIP threshold table
 *      and the compare are shared -- but the threshold's UNITS change with the
 *      method (V / ms / V*ms), so THRESH00..11 must be re-tuned after a switch.
 *
 * ── Threshold select: 2x DIP switches on D8 / D10 ─────────────────
 * Both pins have hardware pull-ups; a switch ON connects its pin to ground.
 * The pins are read directly (HIGH = 1 = switch OFF/open, LOW = 0 = ON):
 *     config "XY":  X = D8 reading, Y = D10 reading
 *   11 (both switches OFF)  -> cfg.thresh11   (factory default 0.62 V)
 *   10 (D8 high, D10 low)   -> cfg.thresh10
 *   01 (D8 low,  D10 high)  -> cfg.thresh01
 *   00 (both switches ON)   -> cfg.thresh00
 * The switches are re-read every detection pass, so they can be changed live.
 * The four threshold values themselves are EEPROM configuration (THRESH00..
 * THRESH11), so what each switch position *means* can be re-programmed over
 * serial without reflashing.
 *
 * ── EEPROM configuration ──────────────────────────────────────────
 * Nearly every tunable lives in a Config struct persisted to the RA4M1's
 * data-flash-backed EEPROM.  On boot the stored config is validated
 * (magic + version + CRC); if invalid, factory defaults are loaded and saved.
 * A host (serial terminal or Python GUI) can permanently retune a shipped
 * unit -- e.g. disable the beeper, move REFCENTER, re-map the DIP table --
 * with !SET + !SAVE, no recompile needed.  See the CONFIG FIELD TABLE below
 * for every key, and BlinkyHawk firmware manual (HTML) for full docs.
 * EEPROM here is data flash, which a sketch upload does NOT erase, so a unit
 * keeps its tuning across a reflash.  When the struct layout changes,
 * CFG_VERSION is bumped and a migration (see configMigrateV2) carries the old
 * values forward rather than reverting the unit to factory defaults.
 *
 * ── Serial protocol (115200 baud, line based) ─────────────────────
 * Commands in (each terminated with newline):
 *   !SET,<key>,<value>  set a config value in RAM (takes effect immediately)
 *   !GET,<key>          report one config value
 *   !CFG                dump every config key ($CFG rows + $CFGEND)
 *   !SAVE               persist the RAM config to EEPROM
 *   !LOAD               discard RAM changes, reload from EEPROM
 *   !DEFAULTS           factory defaults into RAM (then !SAVE to keep)
 *   !SN[,<value>]       read the unit serial number, or (with value) write it
 *                       to its own EEPROM block (survives !DEFAULTS; no commas)
 *   !DIAG[,0|1]         enter/exit diagnostic mode (bare = toggle)
 *   !STREAM[,0|1]       continuous raw streaming on/off (diag only)
 *   !RATE,<ms>          stream interval in ms
 *   !VMODE,<0|1|2>      voltage mode: 0=auto  1=lock ON  2=disable
 *   !MOSFET,<-1|0|1>    MOSFET: -1=auto(run detection) 0=hold off 1=hold on
 *   !ALERTS[,0|1]       re-enable normal alerts while charging (1=on,0=blink)
 *   !CAP[,<ms>]         capture ADC across a MOSFET toggle, then dump
 *   !SLEEP[,0]          arm the low-power timeout to fire as soon as it is
 *                       allowed (arm over USB, then unplug); ,0 = disarm
 *   !SLEEPTEST          run one sleeping-mode probe now and report its decision
 *                       ($SLEEPTEST,<state>,metric=..,thr=..,retms=..,areavms=..)
 *   !SLEEPLOG[,0]       dump the probes made while asleep (,0 = clear).  The
 *                       only way to see what the board measured off USB, where
 *                       the ground reference -- and the metric -- differ.
 *   !FLOOR,<0-3>        park the board for a current measurement (see below)
 *   !STATUS  / !?       print current status
 * Data out:
 *   $STATUS,...                          current mode/state summary
 *   $CFG,<key>,<value>                   one config value (from !GET/!SET/!CFG)
 *   $CFGEND                              end of a !CFG dump
 *   $SN,<value>                          unit serial number (empty if unassigned)
 *   $OK,<what> / $ERR,<what>[,detail]    command acknowledge / failure
 *   $DIP,<idx>,<threshV>                 DIP position changed (live)
 *   $DIAG,<ms>,<rawPos>,<rawNeg>,<posV>,<negV>,<diffV>    (streaming)
 *   $CAPSTART,<n>,<toggleUs>,<durMs>,<fullScale>,<vref>   (capture header)
 *   $CAP,<t_us>,<rawPos>,<rawNeg>                         (capture rows)
 *   $CAPEND
 *
 * ── Alerts ────────────────────────────────────────────────────────
 * NeoPixel: dim-blue flash = floating, green flash = closed, red flash =
 * voltage present; slow dim-red 25% blink = charging (green when battery
 * >= BATTFULLPCT); 1-4 green boot blinks = battery level.
 * Speaker: mirrors the LED (continuity beep / voltage double-beep), passive
 * or active buzzer selectable at runtime (PASSIVE key).  Shorting the leads
 * during boot mutes audio for the session (BOOTMUTE key to disable).
 *
 * ── Low-power timeout mode ────────────────────────────────────────
 * After SLEEPSEC seconds in which the leads have only ever read OPEN, the
 * board parks every load it can switch (LED rail, speaker, battery-sense
 * divider, optionally the bridge) and enters Software Standby -- CPU and all
 * peripheral clocks stopped, RAM retained -- woken by the RTC periodic
 * interrupt every 2 s.  Each wake runs one cheap probe (SLEEPAVG reads instead
 * of VOLTAVG, no agreement count, no display debounce) and goes straight back
 * to sleep unless the leads are closed or voltage is present, in which case it
 * returns to full-rate operation.  The probe compares against SLEEPTHR00..11
 * rather than THRESH00..11 when those are non-zero, because the sleeping metric
 * runs a few percent high (the analog path has not fully settled after a
 * standby wake) -- so a THRESH tuned to a target resistance can sit below the
 * sleeping closed-lead reading and silently stop continuity from waking the
 * unit.  0 means "use THRESH for that position".  The sleeping voltage test is
 * deliberately
 * LESS sensitive than the awake one: it uses only the instant-bypass band
 * (VOLTFAST x REFBAND), never the tight averaged REFBAND test, so lead noise
 * cannot wake the unit every tick.  A 6 ms dim-blue flash every
 * SLEEPHB ticks shows it is asleep rather than dead.  Sleeping is skipped
 * entirely while charging or in diagnostic mode; SLEEPSEC = 0 disables it.
 * Worst-case wake latency is 2 s x SLEEPTICKS.
 *
 * NOTE: millis() does not advance during Standby (its timer is clocked off),
 * so the sleeping loop counts ticks rather than timing, and the idle timer is
 * re-based on wake.
 *
 * !FLOOR,<1-3> parks the board in one fixed state so a series ammeter can read
 * a stable current.  The point is the deltas, which say what is worth switching
 * in hardware on a future board revision:
 *   1 vs 3  = what Software Standby buys over a plain WFI idle (the MCU's share)
 *   1 vs 2  = the bridge leg's share (bridge resting vs disconnected)
 *   1       = the firmware-reachable floor; whatever is left is hardware --
 *             the voltage reference's bias current, the LDO and charger
 *             quiescent, and any permanently-connected divider strings.
 * Send the command over USB with the battery attached through the meter, then
 * unplug USB -- the board keeps running on battery in the parked state.
 */

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <ctype.h>
#include <string.h>
#include <stddef.h>   // offsetof (configCrc)

#if !defined(ARDUINO_ARCH_RENESAS)
  #error "BlinkyHawk_RA4M1 targets the Seeed XIAO RA4M1 -- select a Renesas RA4M1 board in Tools > Board."
#endif

// ══════════════════════════════════════════════════════════════════
//  PIN MAP / HARDWARE CONSTANTS  (fixed by the Blinky Hawk PCB)
// ══════════════════════════════════════════════════════════════════
const int   SENSE_POS     = A2;          // pseudo-differential positive input
const int   SENSE_NEG     = A1;          // pseudo-differential negative input
const int   MOSFET_PIN    = D7;          // bridge MOSFET gate (HIGH = on/resting)
const int   SPEAKER_PIN   = D9;          // buzzer element
const int   DIP_PIN_A     = D8;          // threshold-select DIP, first digit  ("X" in XY)
const int   DIP_PIN_B     = D10;         // threshold-select DIP, second digit ("Y" in XY)
#define     LED_PIN         6            // onboard RGB data
#define     RGB_POWER_PIN   PIN_RGB_EN   // onboard RGB power enable
const int   CHARGE_PIN    = A3;          // VBUS/2 divider (USB-power sense)
const int   BATT_PIN      = BAT_DET_PIN; // P105, onboard battery sense (Vbatt/2)
const int   BATT_EN_PIN   = BAT_READ_EN; // P400, HIGH = enable battery sense
const float BATT_DIV      = 2.0f;        // BAT_DET_PIN = Vbatt/2 -> multiply back up

const float ADC_REF_VOLTAGE = 3.3f;      // VREFH tied to the 3.3 V rail
const int   ADC_RESOLUTION  = 14;
const float ADC_FULL_SCALE  = 16383.0f;

#define MOSFET_ON   HIGH
#define MOSFET_OFF  LOW
#define SPEAKER_ON  HIGH
#define SPEAKER_OFF LOW

// ══════════════════════════════════════════════════════════════════
//  EEPROM CONFIGURATION
// ══════════════════════════════════════════════════════════════════
// Every runtime-tunable setting lives in this struct.  It is held in RAM
// (edited by !SET, applied immediately) and persisted to EEPROM by !SAVE.
// Layout changes REQUIRE bumping CFG_VERSION so stale stored data is
// rejected and replaced with defaults instead of being misread.
#define CFG_MAGIC   0x42484B31UL   // "BHK1"
#define CFG_VERSION 5              // bumped: added sleepTickMs (base wake period)
                                   // (v4 added sleepThresh[] wake thresholds;
                                   //  v3 added low-power timeout fields;
                                   //  v2 added detectMethod + recovery params)
#define CFG_EEPROM_ADDR 0

struct Config {
  uint32_t magic;
  uint16_t version;

  // -- Detection ---------------------------------------------------
  float    refCenterV;     // resting differential centre (~0 V)
  float    refBandV;       // |diff - centre| within this -> no voltage, run test
  float    thresh[4];      // open/closed threshold per DIP position [00,01,10,11]
  float    voltFastMult;   // single-read "voltage present" shortcut multiplier
  uint8_t  voltAvgSamples; // reads averaged for the voltage-present decision
  uint8_t  testAgree;      // consecutive matching MOSFET tests required
  uint8_t  stableCount;    // detection passes a new state must repeat (display debounce)
  uint16_t settlePreUs;    // us settle after MOSFET off, before the test read
  uint8_t  settlePostMs;   // ms settle after the test read, before MOSFET on
  uint8_t  negFix;         // 1 = replace the live SENSE_NEG read with negFixV
  float    negFixV;        // fixed pseudo-reference voltage when negFix

  // -- Detection method (how the open/closed metric is derived) -----
  // 0 = SINGLE  : one differential read at settlePreUs; metric = |diff|   (V)
  // 1 = TIMERET : time for |diff-refCentre| to fall back within detReturnBand (ms)
  // 2 = AREA    : tail-windowed integral of |diff-refCentre| over the recovery (V*ms)
  // In every method a LARGER metric = more OPEN, so the DIP threshold table
  // and the "metric > threshold -> FLOAT" test are unchanged -- but the units
  // of the threshold change with the method, so re-tune THRESH00..11 on switch.
  uint8_t  detectMethod;   // 0=single(legacy) 1=time-to-return 2=tail-area
  float    detReturnBand;  // method 1: |diff-refCentre| within this = "returned"
  uint16_t detWindowUs;    // methods 1&2: max sample window / timeout (us from toggle)
  uint16_t detAreaStartUs; // method 2: tail-area integration start (us from toggle)

  // -- Alerts ------------------------------------------------------
  uint8_t  ledEnable;      // 1 = normal detection LED alerts
  uint8_t  beepEnable;     // 1 = speaker alerts (master enable)
  uint8_t  bootMute;       // 1 = leads CLOSED at boot mutes audio for the session
  uint8_t  passiveBuzzer;  // 1 = passive buzzer via tone(), 0 = active (DC on/off)
  uint16_t contFreqHz;     // continuity pitch (passive buzzer only)
  uint16_t voltFreqHz;     // voltage pitch    (passive buzzer only)
  uint8_t  contPulses;     // pulses per continuity beep
  uint8_t  voltPulses;     // pulses per voltage beep
  uint8_t  contRepeat;     // 1 = re-beep while CLOSED holds
  uint8_t  voltRepeat;     // 1 = re-beep while VOLTAGE holds
  uint16_t contRepeatMs;   // repeat period while CLOSED
  uint16_t voltRepeatMs;   // repeat period while VOLTAGE
  uint16_t beepMinMs;      // min gap between beep sequences (rate cap)

  // -- Power / battery ---------------------------------------------
  float    chargeThreshV;  // A3 volts (VBUS/2) above this = charging
  float    battEmptyV;     // battery voltage mapped to 0%
  float    battFullV;      // battery voltage mapped to 100%
  uint8_t  battFullPct;    // >= this % while charging = green charge blink

  // -- Low-power timeout -------------------------------------------
  // After idleTimeoutS seconds in which the leads have only read OPEN, the
  // board parks every switchable load and drops into Software Standby, waking
  // on a 2 s RTC tick to probe.  See the LOW-POWER TIMEOUT MODE section.
  uint16_t idleTimeoutS;   // seconds of open-lead inactivity before sleeping (0 = never)
  // Base wake period.  The RTC periodic interrupt is the wake source and it only
  // supports a fixed set of rates (2000/1000/500/250/125 ms here), so a value
  // set over serial is snapped to the nearest supported one and written back --
  // !CFG therefore always reports the rate actually programmed, not the request.
  uint16_t sleepTickMs;
  uint8_t  sleepPollTicks; // probe every N wake ticks (N * sleepTickMs between probes)
  uint8_t  sleepVoltAvg;   // reads taken for the voltage check while asleep (any
                           // one over the bypass band wakes -> fewer = quieter)
  uint8_t  sleepHbTicks;   // heartbeat flash every N ticks (0 = no heartbeat)
  uint8_t  sleepParkOff;   // 1 = park the bridge MOSFET OFF while asleep

  // Wake threshold per DIP position, used ONLY by the sleeping probe.  The
  // sleeping metric runs a few percent higher than the awake one (the analog
  // path has not fully settled after a standby wake), so a thresh[] value tuned
  // to a particular resistance can sit below the sleeping closed-lead reading
  // and silently stop continuity from waking the unit.  These decouple the two:
  // thresh[] answers "is this resistance low enough to call continuity", while
  // sleepThresh[] answers the coarser "is anything connected at all -- wake up".
  // 0 = fall back to thresh[] for that position (the default, so behaviour is
  // unchanged until a value is deliberately set).
  float    sleepThresh[4];

  // -- Misc --------------------------------------------------------
  uint16_t loopDelayMs;    // main-loop pacing (WFI idle between passes)

  uint16_t crc;            // CRC16 over everything above (must stay LAST)
};

Config cfg;                // live (RAM) configuration
bool   cfgDirty = false;   // RAM differs from EEPROM (informational, in $STATUS)

// Factory defaults.  memset first so struct padding is deterministic and the
// CRC of a defaults-derived image is reproducible.
void configDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic   = CFG_MAGIC;
  cfg.version = CFG_VERSION;

  cfg.refCenterV     = -0.02f;
  cfg.refBandV       = 0.025f;
  cfg.thresh[0]      = 0.15f;    // 00: both switches ON  (most sensitive)
  cfg.thresh[1]      = 0.54f;    // 01: D8 ON,  D10 OFF
  cfg.thresh[2]      = 0.45f;    // 10: D8 OFF, D10 ON
  cfg.thresh[3]      = 0.62f;    // 11: both switches OFF (factory position)
  cfg.voltFastMult   = 5.0f;
  cfg.voltAvgSamples = 10;
  cfg.testAgree      = 1;
  cfg.stableCount    = 2;
  cfg.settlePreUs    = 300;
  cfg.settlePostMs   = 3;
  cfg.negFix         = 1;        // matches the proven XIAO_Minimal behaviour
  cfg.negFixV        = 1.250f;

  cfg.detectMethod   = 0;        // ship on the proven single-sample method
  cfg.detReturnBand  = 0.05f;    // best IsoGnd/Open separation in bench captures
  cfg.detWindowUs    = 1500;     // recovery completes ~0.7-0.95 ms; window past it
  cfg.detAreaStartUs = 400;      // skip the common initial dip; integrate the tail

  cfg.ledEnable      = 1;
  cfg.beepEnable     = 1;
  cfg.bootMute       = 1;
  cfg.passiveBuzzer  = 1;
  cfg.contFreqHz     = 2000;
  cfg.voltFreqHz     = 2500;
  cfg.contPulses     = 1;
  cfg.voltPulses     = 2;
  cfg.contRepeat     = 1;
  cfg.voltRepeat     = 0;
  cfg.contRepeatMs   = 1000;
  cfg.voltRepeatMs   = 1000;
  cfg.beepMinMs      = 500;

  cfg.chargeThreshV  = 2.0f;
  cfg.battEmptyV     = 3.30f;
  cfg.battFullV      = 4.20f;
  cfg.battFullPct    = 90;

  cfg.idleTimeoutS   = 15;       // seconds of open leads before sleeping
  cfg.sleepTickMs    = 2000;     // slowest RTC period = fewest wakes
  cfg.sleepPollTicks = 1;        // probe on every wake
  cfg.sleepVoltAvg   = 3;        // reads per probe; any one over the bypass band wakes
  cfg.sleepHbTicks   = 30;       // "still alive" flash once a minute
  cfg.sleepParkOff   = 0;        // bridge parked resting (as in normal operation)
  for (int i = 0; i < 4; i++) cfg.sleepThresh[i] = 0.0f;   // 0 = use thresh[i]

  cfg.loopDelayMs    = 50;
}

// CRC16-CCITT over an arbitrary byte range (shared by Config and SerialId).
uint16_t crc16_ccitt(const uint8_t *p, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)p[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

// CRC16-CCITT over the struct bytes, excluding the trailing crc field.
// offsetof, NOT sizeof-2: the struct is 4-byte aligned so there are padding
// bytes AFTER crc, and sizeof-2 would run the CRC over the crc field itself --
// making every reload/boot validation fail and re-seed defaults.
uint16_t configCrc(const Config &c) {
  return crc16_ccitt((const uint8_t *)&c, offsetof(Config, crc));
}

// Persist RAM config to EEPROM (data flash).  Only called on !SAVE / first
// boot -- data-flash writes are slow and endurance-limited, so the firmware
// never saves on its own during normal operation.
void configSave() {
  cfg.magic   = CFG_MAGIC;
  cfg.version = CFG_VERSION;
  cfg.crc     = configCrc(cfg);
  EEPROM.put(CFG_EEPROM_ADDR, cfg);
  cfgDirty = false;
}

// Load config from EEPROM into RAM.  Returns true if the stored image was
// valid; on failure the caller decides whether to fall back to defaults.
bool configLoad() {
  Config stored;
  EEPROM.get(CFG_EEPROM_ADDR, stored);
  if (stored.magic != CFG_MAGIC)          return false;
  if (stored.version != CFG_VERSION)      return false;
  if (stored.crc != configCrc(stored))    return false;
  cfg = stored;
  cfgDirty = false;
  return true;
}

// ── Upgrade path from older stored layouts ────────────────────────
// EEPROM here is the RA4M1's data flash, which a sketch upload does NOT erase --
// so a unit reflashed with new firmware still has its old config sitting there.
// configLoad() rejects it (the version differs, and the bytes genuinely can't
// be reinterpreted as the new struct), which for a field-tuned unit means
// silently reverting it to factory thresholds.  Instead, keep each superseded
// layout frozen here and copy the values across.
//
// v4 EXACTLY as it shipped (v3 plus sleepThresh[], before sleepTickMs existed).
// Frozen -- never edit; see the note on ConfigV2.
struct ConfigV4 {
  uint32_t magic;
  uint16_t version;
  float    refCenterV;
  float    refBandV;
  float    thresh[4];
  float    voltFastMult;
  uint8_t  voltAvgSamples;
  uint8_t  testAgree;
  uint8_t  stableCount;
  uint16_t settlePreUs;
  uint8_t  settlePostMs;
  uint8_t  negFix;
  float    negFixV;
  uint8_t  detectMethod;
  float    detReturnBand;
  uint16_t detWindowUs;
  uint16_t detAreaStartUs;
  uint8_t  ledEnable;
  uint8_t  beepEnable;
  uint8_t  bootMute;
  uint8_t  passiveBuzzer;
  uint16_t contFreqHz;
  uint16_t voltFreqHz;
  uint8_t  contPulses;
  uint8_t  voltPulses;
  uint8_t  contRepeat;
  uint8_t  voltRepeat;
  uint16_t contRepeatMs;
  uint16_t voltRepeatMs;
  uint16_t beepMinMs;
  float    chargeThreshV;
  float    battEmptyV;
  float    battFullV;
  uint8_t  battFullPct;
  uint16_t idleTimeoutS;
  uint8_t  sleepPollTicks;
  uint8_t  sleepVoltAvg;
  uint8_t  sleepHbTicks;
  uint8_t  sleepParkOff;
  float    sleepThresh[4];
  uint16_t loopDelayMs;
  uint16_t crc;
};

// v3 EXACTLY as it shipped (v2 plus the low-power timeout block, before
// sleepThresh[] existed).  Frozen -- never edit; see the note on ConfigV2.
struct ConfigV3 {
  uint32_t magic;
  uint16_t version;
  float    refCenterV;
  float    refBandV;
  float    thresh[4];
  float    voltFastMult;
  uint8_t  voltAvgSamples;
  uint8_t  testAgree;
  uint8_t  stableCount;
  uint16_t settlePreUs;
  uint8_t  settlePostMs;
  uint8_t  negFix;
  float    negFixV;
  uint8_t  detectMethod;
  float    detReturnBand;
  uint16_t detWindowUs;
  uint16_t detAreaStartUs;
  uint8_t  ledEnable;
  uint8_t  beepEnable;
  uint8_t  bootMute;
  uint8_t  passiveBuzzer;
  uint16_t contFreqHz;
  uint16_t voltFreqHz;
  uint8_t  contPulses;
  uint8_t  voltPulses;
  uint8_t  contRepeat;
  uint8_t  voltRepeat;
  uint16_t contRepeatMs;
  uint16_t voltRepeatMs;
  uint16_t beepMinMs;
  float    chargeThreshV;
  float    battEmptyV;
  float    battFullV;
  uint8_t  battFullPct;
  uint16_t idleTimeoutS;
  uint8_t  sleepPollTicks;
  uint8_t  sleepVoltAvg;
  uint8_t  sleepHbTicks;
  uint8_t  sleepParkOff;
  uint16_t loopDelayMs;
  uint16_t crc;
};

// This struct is v2 EXACTLY as it shipped.  It must never be edited again --
// it is not "the config minus the new fields", it is a historical record of
// what is actually in the flash of units already in the field.
struct ConfigV2 {
  uint32_t magic;
  uint16_t version;
  float    refCenterV;
  float    refBandV;
  float    thresh[4];
  float    voltFastMult;
  uint8_t  voltAvgSamples;
  uint8_t  testAgree;
  uint8_t  stableCount;
  uint16_t settlePreUs;
  uint8_t  settlePostMs;
  uint8_t  negFix;
  float    negFixV;
  uint8_t  detectMethod;
  float    detReturnBand;
  uint16_t detWindowUs;
  uint16_t detAreaStartUs;
  uint8_t  ledEnable;
  uint8_t  beepEnable;
  uint8_t  bootMute;
  uint8_t  passiveBuzzer;
  uint16_t contFreqHz;
  uint16_t voltFreqHz;
  uint8_t  contPulses;
  uint8_t  voltPulses;
  uint8_t  contRepeat;
  uint8_t  voltRepeat;
  uint16_t contRepeatMs;
  uint16_t voltRepeatMs;
  uint16_t beepMinMs;
  float    chargeThreshV;
  float    battEmptyV;
  float    battFullV;
  uint8_t  battFullPct;
  uint16_t loopDelayMs;
  uint16_t crc;
};

// Upgrade a stored v4 image.  sleepTickMs keeps its default (2000), which is
// the rate v4 hard-coded, so a migrated unit wakes at exactly the same cadence.
bool configMigrateV4() {
  ConfigV4 old;
  EEPROM.get(CFG_EEPROM_ADDR, old);
  if (old.magic != CFG_MAGIC) return false;
  if (old.version != 4)       return false;
  if (old.crc != crc16_ccitt((const uint8_t *)&old, offsetof(ConfigV4, crc))) return false;

  cfg.refCenterV     = old.refCenterV;
  cfg.refBandV       = old.refBandV;
  for (int i = 0; i < 4; i++) cfg.thresh[i] = old.thresh[i];
  cfg.voltFastMult   = old.voltFastMult;
  cfg.voltAvgSamples = old.voltAvgSamples;
  cfg.testAgree      = old.testAgree;
  cfg.stableCount    = old.stableCount;
  cfg.settlePreUs    = old.settlePreUs;
  cfg.settlePostMs   = old.settlePostMs;
  cfg.negFix         = old.negFix;
  cfg.negFixV        = old.negFixV;
  cfg.detectMethod   = old.detectMethod;
  cfg.detReturnBand  = old.detReturnBand;
  cfg.detWindowUs    = old.detWindowUs;
  cfg.detAreaStartUs = old.detAreaStartUs;
  cfg.ledEnable      = old.ledEnable;
  cfg.beepEnable     = old.beepEnable;
  cfg.bootMute       = old.bootMute;
  cfg.passiveBuzzer  = old.passiveBuzzer;
  cfg.contFreqHz     = old.contFreqHz;
  cfg.voltFreqHz     = old.voltFreqHz;
  cfg.contPulses     = old.contPulses;
  cfg.voltPulses     = old.voltPulses;
  cfg.contRepeat     = old.contRepeat;
  cfg.voltRepeat     = old.voltRepeat;
  cfg.contRepeatMs   = old.contRepeatMs;
  cfg.voltRepeatMs   = old.voltRepeatMs;
  cfg.beepMinMs      = old.beepMinMs;
  cfg.chargeThreshV  = old.chargeThreshV;
  cfg.battEmptyV     = old.battEmptyV;
  cfg.battFullV      = old.battFullV;
  cfg.battFullPct    = old.battFullPct;
  cfg.idleTimeoutS   = old.idleTimeoutS;
  cfg.sleepPollTicks = old.sleepPollTicks;
  cfg.sleepVoltAvg   = old.sleepVoltAvg;
  cfg.sleepHbTicks   = old.sleepHbTicks;
  cfg.sleepParkOff   = old.sleepParkOff;
  for (int i = 0; i < 4; i++) cfg.sleepThresh[i] = old.sleepThresh[i];
  cfg.loopDelayMs    = old.loopDelayMs;
  return true;
}

// Upgrade a stored v3 image.  cfg must already hold defaults on entry, so the
// fields v3 never had (sleepThresh[]) keep their defaults -- which are 0, i.e.
// "fall back to thresh[]", so a migrated unit behaves exactly as it did before.
bool configMigrateV3() {
  ConfigV3 old;
  EEPROM.get(CFG_EEPROM_ADDR, old);
  if (old.magic != CFG_MAGIC) return false;
  if (old.version != 3)       return false;
  if (old.crc != crc16_ccitt((const uint8_t *)&old, offsetof(ConfigV3, crc))) return false;

  cfg.refCenterV     = old.refCenterV;
  cfg.refBandV       = old.refBandV;
  for (int i = 0; i < 4; i++) cfg.thresh[i] = old.thresh[i];
  cfg.voltFastMult   = old.voltFastMult;
  cfg.voltAvgSamples = old.voltAvgSamples;
  cfg.testAgree      = old.testAgree;
  cfg.stableCount    = old.stableCount;
  cfg.settlePreUs    = old.settlePreUs;
  cfg.settlePostMs   = old.settlePostMs;
  cfg.negFix         = old.negFix;
  cfg.negFixV        = old.negFixV;
  cfg.detectMethod   = old.detectMethod;
  cfg.detReturnBand  = old.detReturnBand;
  cfg.detWindowUs    = old.detWindowUs;
  cfg.detAreaStartUs = old.detAreaStartUs;
  cfg.ledEnable      = old.ledEnable;
  cfg.beepEnable     = old.beepEnable;
  cfg.bootMute       = old.bootMute;
  cfg.passiveBuzzer  = old.passiveBuzzer;
  cfg.contFreqHz     = old.contFreqHz;
  cfg.voltFreqHz     = old.voltFreqHz;
  cfg.contPulses     = old.contPulses;
  cfg.voltPulses     = old.voltPulses;
  cfg.contRepeat     = old.contRepeat;
  cfg.voltRepeat     = old.voltRepeat;
  cfg.contRepeatMs   = old.contRepeatMs;
  cfg.voltRepeatMs   = old.voltRepeatMs;
  cfg.beepMinMs      = old.beepMinMs;
  cfg.chargeThreshV  = old.chargeThreshV;
  cfg.battEmptyV     = old.battEmptyV;
  cfg.battFullV      = old.battFullV;
  cfg.battFullPct    = old.battFullPct;
  cfg.idleTimeoutS   = old.idleTimeoutS;
  cfg.sleepPollTicks = old.sleepPollTicks;
  cfg.sleepVoltAvg   = old.sleepVoltAvg;
  cfg.sleepHbTicks   = old.sleepHbTicks;
  cfg.sleepParkOff   = old.sleepParkOff;
  cfg.loopDelayMs    = old.loopDelayMs;
  return true;
}

// Try to upgrade a stored v2 image into the live config.  cfg must already
// hold defaults on entry, so the fields v2 never had keep their default values.
// Returns true if a valid v2 image was found and migrated.
bool configMigrateV2() {
  ConfigV2 old;
  EEPROM.get(CFG_EEPROM_ADDR, old);
  if (old.magic != CFG_MAGIC) return false;
  if (old.version != 2)       return false;
  if (old.crc != crc16_ccitt((const uint8_t *)&old, offsetof(ConfigV2, crc))) return false;

  cfg.refCenterV     = old.refCenterV;
  cfg.refBandV       = old.refBandV;
  for (int i = 0; i < 4; i++) cfg.thresh[i] = old.thresh[i];
  cfg.voltFastMult   = old.voltFastMult;
  cfg.voltAvgSamples = old.voltAvgSamples;
  cfg.testAgree      = old.testAgree;
  cfg.stableCount    = old.stableCount;
  cfg.settlePreUs    = old.settlePreUs;
  cfg.settlePostMs   = old.settlePostMs;
  cfg.negFix         = old.negFix;
  cfg.negFixV        = old.negFixV;
  cfg.detectMethod   = old.detectMethod;
  cfg.detReturnBand  = old.detReturnBand;
  cfg.detWindowUs    = old.detWindowUs;
  cfg.detAreaStartUs = old.detAreaStartUs;
  cfg.ledEnable      = old.ledEnable;
  cfg.beepEnable     = old.beepEnable;
  cfg.bootMute       = old.bootMute;
  cfg.passiveBuzzer  = old.passiveBuzzer;
  cfg.contFreqHz     = old.contFreqHz;
  cfg.voltFreqHz     = old.voltFreqHz;
  cfg.contPulses     = old.contPulses;
  cfg.voltPulses     = old.voltPulses;
  cfg.contRepeat     = old.contRepeat;
  cfg.voltRepeat     = old.voltRepeat;
  cfg.contRepeatMs   = old.contRepeatMs;
  cfg.voltRepeatMs   = old.voltRepeatMs;
  cfg.beepMinMs      = old.beepMinMs;
  cfg.chargeThreshV  = old.chargeThreshV;
  cfg.battEmptyV     = old.battEmptyV;
  cfg.battFullV      = old.battFullV;
  cfg.battFullPct    = old.battFullPct;
  cfg.loopDelayMs    = old.loopDelayMs;
  return true;
}

// ══════════════════════════════════════════════════════════════════
//  UNIT SERIAL NUMBER (device identity)
// ══════════════════════════════════════════════════════════════════
// Stored SEPARATELY from Config, in its own EEPROM block, so it survives
// !DEFAULTS and any future CFG_VERSION bump -- the SN is the unit's permanent
// identity, not a tunable.  Written once (usually at first bring-up) via !SN,
// read back on every boot and reported in $STATUS so the host can key its
// per-unit config log on it.
#define SN_MAGIC     0x42485F53UL   // "BH_S"
#define SN_MAX_LEN   16             // buffer size incl. null terminator
#define SN_EEPROM_ADDR 512          // well clear of the Config block at addr 0
                                    // (Config must stay < 512 bytes; it is ~150)

struct SerialId {
  uint32_t magic;
  char     sn[SN_MAX_LEN];
  uint16_t crc;
};

char unitSN[SN_MAX_LEN] = "";       // live copy ("" = unassigned)

// Explicit prototype (see the ConfigField note above): keeps the Arduino
// preprocessor from hoisting an auto-generated prototype above SerialId.
uint16_t snCrc(const SerialId &s);

uint16_t snCrc(const SerialId &s) {
  return crc16_ccitt((const uint8_t *)&s, offsetof(SerialId, crc));
}

// Load the SN from EEPROM into unitSN, or leave it empty if unset/corrupt.
void snLoad() {
  SerialId s;
  EEPROM.get(SN_EEPROM_ADDR, s);
  if (s.magic == SN_MAGIC && s.crc == snCrc(s)) {
    s.sn[SN_MAX_LEN - 1] = '\0';    // paranoia: guarantee termination
    strncpy(unitSN, s.sn, SN_MAX_LEN);
    unitSN[SN_MAX_LEN - 1] = '\0';
  } else {
    unitSN[0] = '\0';
  }
}

// Persist a new SN to EEPROM and update the live copy.
void snSave(const char *sn) {
  SerialId s;
  memset(&s, 0, sizeof(s));         // deterministic padding for a stable CRC
  s.magic = SN_MAGIC;
  strncpy(s.sn, sn, SN_MAX_LEN - 1);
  s.sn[SN_MAX_LEN - 1] = '\0';
  s.crc = snCrc(s);
  EEPROM.put(SN_EEPROM_ADDR, s);
  strncpy(unitSN, s.sn, SN_MAX_LEN);
  unitSN[SN_MAX_LEN - 1] = '\0';
}

// ── Config field table ────────────────────────────────────────────
// Maps serial key names onto struct fields so !SET/!GET/!CFG are generic:
// adding a tunable = add the struct field, a default, and one table row.
// min/max bounds stop a bad !SET from bricking a unit (values are clamped
// and the clamped value is echoed back).
enum FieldType { FT_FLOAT, FT_U8, FT_U16, FT_BOOL };

struct ConfigField {
  const char *name;
  FieldType   type;
  void       *ptr;
  float       minV;
  float       maxV;
};

// Explicit prototypes: the Arduino preprocessor would otherwise hoist its
// auto-generated ones above this struct definition and fail to compile.
const ConfigField *findField(const char *name);
float fieldGet(const ConfigField *f);
void  fieldSet(const ConfigField *f, float v);
void  printField(const ConfigField *f);

const ConfigField CFG_FIELDS[] = {
  // Detection
  { "REFCENTER",   FT_FLOAT, &cfg.refCenterV,     -1.0f,   1.0f   },
  { "REFBAND",     FT_FLOAT, &cfg.refBandV,        0.001f, 1.0f   },
  { "THRESH00",    FT_FLOAT, &cfg.thresh[0],       0.001f, 3.3f   },
  { "THRESH01",    FT_FLOAT, &cfg.thresh[1],       0.001f, 3.3f   },
  { "THRESH10",    FT_FLOAT, &cfg.thresh[2],       0.001f, 3.3f   },
  { "THRESH11",    FT_FLOAT, &cfg.thresh[3],       0.001f, 3.3f   },
  { "VOLTFAST",    FT_FLOAT, &cfg.voltFastMult,    1.0f,   50.0f  },
  { "VOLTAVG",     FT_U8,    &cfg.voltAvgSamples,  1,      50     },
  { "TESTAGREE",   FT_U8,    &cfg.testAgree,       1,      10     },
  { "STABLECOUNT", FT_U8,    &cfg.stableCount,     1,      10     },
  { "SETTLEPREUS", FT_U16,   &cfg.settlePreUs,     0,      5000   },
  { "SETTLEPOSTMS",FT_U8,    &cfg.settlePostMs,    0,      50     },
  { "NEGFIX",      FT_BOOL,  &cfg.negFix,          0,      1      },
  { "NEGV",        FT_FLOAT, &cfg.negFixV,         0.0f,   3.3f   },
  { "DETMETHOD",   FT_U8,    &cfg.detectMethod,    0,      2      },
  { "DETBAND",     FT_FLOAT, &cfg.detReturnBand,   0.005f, 1.0f   },
  { "DETWINUS",    FT_U16,   &cfg.detWindowUs,     200,    5000   },
  { "DETAREAUS",   FT_U16,   &cfg.detAreaStartUs,  0,      5000   },
  // Alerts
  { "LED",         FT_BOOL,  &cfg.ledEnable,       0,      1      },
  { "BEEP",        FT_BOOL,  &cfg.beepEnable,      0,      1      },
  { "BOOTMUTE",    FT_BOOL,  &cfg.bootMute,        0,      1      },
  { "PASSIVE",     FT_BOOL,  &cfg.passiveBuzzer,   0,      1      },
  { "CONTFREQ",    FT_U16,   &cfg.contFreqHz,      100,    10000  },
  { "VOLTFREQ",    FT_U16,   &cfg.voltFreqHz,      100,    10000  },
  { "CONTPULSES",  FT_U8,    &cfg.contPulses,      1,      5      },
  { "VOLTPULSES",  FT_U8,    &cfg.voltPulses,      1,      5      },
  { "CONTREP",     FT_BOOL,  &cfg.contRepeat,      0,      1      },
  { "VOLTREP",     FT_BOOL,  &cfg.voltRepeat,      0,      1      },
  { "CONTREPMS",   FT_U16,   &cfg.contRepeatMs,    100,    60000  },
  { "VOLTREPMS",   FT_U16,   &cfg.voltRepeatMs,    100,    60000  },
  { "BEEPMIN",     FT_U16,   &cfg.beepMinMs,       50,     10000  },
  // Power / battery
  { "CHGTHRESH",   FT_FLOAT, &cfg.chargeThreshV,   0.5f,   3.3f   },
  { "BATTEMPTY",   FT_FLOAT, &cfg.battEmptyV,      2.5f,   4.0f   },
  { "BATTFULL",    FT_FLOAT, &cfg.battFullV,       3.0f,   4.5f   },
  { "BATTFULLPCT", FT_U8,    &cfg.battFullPct,     50,     100    },
  // Low-power timeout
  { "SLEEPSEC",    FT_U16,   &cfg.idleTimeoutS,    0,      65535  },
  { "SLEEPTICKMS", FT_U16,   &cfg.sleepTickMs,     125,    2000   },
  { "SLEEPTICKS",  FT_U8,    &cfg.sleepPollTicks,  1,      100    },
  { "SLEEPAVG",    FT_U8,    &cfg.sleepVoltAvg,    1,      50     },
  { "SLEEPHB",     FT_U8,    &cfg.sleepHbTicks,    0,      255    },
  { "SLEEPPARK",   FT_BOOL,  &cfg.sleepParkOff,    0,      1      },
  // Wake thresholds: 0 = use the matching THRESH value for that DIP position.
  { "SLEEPTHR00",  FT_FLOAT, &cfg.sleepThresh[0],  0.0f,   3.3f   },
  { "SLEEPTHR01",  FT_FLOAT, &cfg.sleepThresh[1],  0.0f,   3.3f   },
  { "SLEEPTHR10",  FT_FLOAT, &cfg.sleepThresh[2],  0.0f,   3.3f   },
  { "SLEEPTHR11",  FT_FLOAT, &cfg.sleepThresh[3],  0.0f,   3.3f   },
  // Misc
  { "LOOPMS",      FT_U16,   &cfg.loopDelayMs,     1,      1000   },
};
const int CFG_FIELD_COUNT = sizeof(CFG_FIELDS) / sizeof(CFG_FIELDS[0]);

const ConfigField *findField(const char *name) {
  for (int i = 0; i < CFG_FIELD_COUNT; i++)
    if (strcmp(name, CFG_FIELDS[i].name) == 0) return &CFG_FIELDS[i];
  return NULL;
}

float fieldGet(const ConfigField *f) {
  switch (f->type) {
    case FT_FLOAT: return *(float *)f->ptr;
    case FT_U16:   return *(uint16_t *)f->ptr;
    default:       return *(uint8_t *)f->ptr;   // FT_U8 / FT_BOOL
  }
}

void fieldSet(const ConfigField *f, float v) {
  if (v < f->minV) v = f->minV;
  if (v > f->maxV) v = f->maxV;
  switch (f->type) {
    case FT_FLOAT: *(float *)f->ptr    = v;                      break;
    case FT_U16:   *(uint16_t *)f->ptr = (uint16_t)(v + 0.5f);   break;
    case FT_BOOL:  *(uint8_t *)f->ptr  = (v != 0.0f) ? 1 : 0;    break;
    default:       *(uint8_t *)f->ptr  = (uint8_t)(v + 0.5f);    break;
  }
}

void printField(const ConfigField *f) {
  Serial.print("$CFG,");
  Serial.print(f->name);
  Serial.print(",");
  if (f->type == FT_FLOAT) Serial.println(*(float *)f->ptr, 4);
  else                     Serial.println((long)fieldGet(f));
}

// ══════════════════════════════════════════════════════════════════
//  RUNTIME STATE (not persisted)
// ══════════════════════════════════════════════════════════════════
const int TEST_MAX_ATTEMPTS = 30;   // safety cap on MOSFET test repeats

// Recovery-transient methods (1 & 2): only start looking for the "return to
// zero" after the differential has actually dipped past this magnitude, so the
// first read (taken microseconds after MOSFET-off, before the node discharges)
// can't be mistaken for an instant return.  The trough is always ~1.1 V.
const float DET_TROUGH_MIN_V = 0.30f;

// LED colours / flash cadence (compile-time; see manual)
const uint8_t COL_FLOAT_R = 0,  COL_FLOAT_G = 0,   COL_FLOAT_B = 28;   // dim blue
const uint8_t COL_CLOSED_R = 0, COL_CLOSED_G = 200, COL_CLOSED_B = 0;  // green
const uint8_t COL_VOLT_R = 200, COL_VOLT_G = 0,    COL_VOLT_B = 0;     // red
const unsigned long FLOAT_FLASH_MS  = 150, FLOAT_MIN_MS  = 1000;  // 1 Hz cap
const unsigned long CLOSED_FLASH_MS = 200, CLOSED_MIN_MS = 500;   // 2 Hz cap
const unsigned long VOLT_FLASH_MS   = 200, VOLT_MIN_MS   = 500;   // 2 Hz cap

// Continuity beep pulse micro-timing (compile-time)
const unsigned long CONT_ON_MS         = 20;  // initial contact pulse on-time
const unsigned long CONT_OFF_MS        = 10;  // gap between pulses
const unsigned long CONT_ONGOING_ON_MS = 50;  // ongoing "still there" pulse on-time
const unsigned long CONT_ONGOING_OFF_MS = 10;
const unsigned long VOLT_ON_MS         = 20;
const unsigned long VOLT_OFF_MS        = 10;

#define NUM_PIXELS 1
Adafruit_NeoPixel pixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

enum LeadState { STATE_FLOAT, STATE_CLOSED, STATE_VOLTAGE };
LeadState leadState = STATE_VOLTAGE;

// Explicit prototypes (see note at the ConfigField struct).
LeadState runMosfetTest();
LeadState runMosfetTestStable();
LeadState lowPowerProbe();
void      slogRecord(LeadState s, bool awake);

// Active open/closed threshold, refreshed from the DIP switches + cfg.thresh
// table every detection pass.
float   activeThreshV = 0.5f;
uint8_t dipIdx        = 3;      // last-read DIP position (0..3)

// Non-blocking blink state
bool floatFlashing = false, closedFlashing = false, voltFlashing = false;
unsigned long lastFloatFlash = 0, lastClosedFlash = 0, lastVoltFlash = 0;

// Charge / USB-power lockout
bool chargeActive  = false;   // VBUS sense above threshold (USB in)
bool alertOverride = false;   // user re-enabled normal alerts while charging
bool chargeBlinkOn = false;
unsigned long lastChargeBlink = 0;
const unsigned long CHARGE_BLINK_PERIOD_MS = 2000;
const unsigned long CHARGE_BLINK_ON_MS     = 250;
const uint8_t       CHARGE_BLINK_BRIGHT    = 25;

// Power-on battery cue.  Everything here is dead time between flipping the
// switch and being able to measure, so it is kept as short as still reads as a
// countable blink.  The pre-gap doubles as the BAT_READ_EN divider settle
// (see setup(), which enables it first thing), so that settle costs nothing.
const unsigned long BOOT_CUE_PREGAP_MS = 120;
const unsigned long BOOT_CUE_ON_MS     = 90;
const unsigned long BOOT_CUE_OFF_MS    = 90;

// VBUS test used before the config is loaded, so it cannot use cfg.chargeThreshV.
const float BOOT_USB_THRESH_V = 2.0f;

// Battery monitor
float battV   = 0.0f;
int   battPct = 0;

// Beep-sequence state
bool          beepOn           = false;
int           beepPulsesLeft   = 0;
unsigned long beepPhaseStart   = 0;
unsigned long lastBeepSeqStart = 0;
unsigned long beepOnMs = 20, beepOffMs = 10;
unsigned int  beepFreq = 0;
bool speakerMuted = false;    // session mute (leads closed at boot)

// Debug values
float lastRestV = 0.0f, lastTestV = 0.0f;
float lastMetric = 0.0f;      // scalar actually compared to the threshold
float lastReturnMs = 0.0f;    // method 1 result (ms), or window on timeout
float lastAreaVms  = 0.0f;    // method 2 result (V*ms)
unsigned long lastSerialTime = 0;
const unsigned long serialInterval = 250;

// Diagnostic mode
bool diagMode = false;
bool streamOn = false;
unsigned long streamIntervalMs = 20;
unsigned long lastStreamMs = 0;
enum VoltOverride { VOLT_AUTO, VOLT_FORCE_ON, VOLT_DISABLED };
VoltOverride voltOverride = VOLT_AUTO;
int mosfetHold = -1;          // -1 auto, 0 hold off, 1 hold on

// Transient capture buffer (raw counts; volts computed by the host)
const int CAP_MAX_SAMPLES = 600;
const unsigned long CAP_PRE_US = 500;
unsigned long capDurationMs = 5;
uint32_t capT[CAP_MAX_SAMPLES];
uint16_t capPos[CAP_MAX_SAMPLES];
uint16_t capNeg[CAP_MAX_SAMPLES];
int capCount = 0;

char cmdBuf[64];
int  cmdLen = 0;

// ══════════════════════════════════════════════════════════════════
//  LOW-POWER IDLE
// ══════════════════════════════════════════════════════════════════
// The 1 kHz AGT tick that drives millis() interrupts the core every ms, so a
// plain WFI loop idles between ticks (CPU clock gated, peripherals running).
void sleepMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    __WFI();
  }
}

// ══════════════════════════════════════════════════════════════════
//  LOW-POWER TIMEOUT MODE
// ══════════════════════════════════════════════════════════════════
// After cfg.idleTimeoutS seconds in which the leads have only ever read OPEN,
// the board stops running the detection loop, parks every load it can switch
// off (LED rail, speaker, battery-sense divider, optionally the bridge) and
// enters Software Standby -- CPU and all peripheral clocks stopped, RAM
// retained.  It wakes on the RTC periodic interrupt, runs one cheap probe, and
// either sleeps again (still open) or returns to full-rate operation (closed
// leads or voltage present).
//
// The energy cost of the mode is (wake current x awake time) / tick period, so
// the knobs that matter are how often it probes (cfg.sleepPollTicks) and how
// much each probe does (cfg.sleepVoltAvg) -- both are EEPROM config so a unit
// can be characterised on the bench without a reflash.
#include "RTC.h"
#include "r_lpm.h"

// The RTC periodic interrupt is the wake source.  Its period is not free-form --
// the library exposes a fixed ladder, of which these are the useful end -- so
// cfg.sleepTickMs is snapped to one of them.  2 s is the library's maximum (and
// the cheapest); below 125 ms the probe cost stops being worth the latency.
// Worst-case detection latency = cfg.sleepTickMs * cfg.sleepPollTicks.
struct SleepTickOption { uint16_t ms; Period period; };
const SleepTickOption SLEEP_TICK_OPTIONS[] = {
  { 2000, Period::ONCE_EVERY_2_SEC   },
  { 1000, Period::ONCE_EVERY_1_SEC   },
  {  500, Period::N2_TIMES_EVERY_SEC },
  {  250, Period::N4_TIMES_EVERY_SEC },
  {  125, Period::N8_TIMES_EVERY_SEC },
};
const int SLEEP_TICK_OPTION_COUNT =
    sizeof(SLEEP_TICK_OPTIONS) / sizeof(SLEEP_TICK_OPTIONS[0]);
const unsigned long SLEEP_HB_MS   = 6;    // heartbeat flash on-time
const uint8_t SLEEP_HB_BRIGHT     = 12;   // heartbeat flash brightness (dim blue)
const unsigned long SLEEP_BOOT_GRACE_MS = 30000;   // never sleep this soon after boot

bool          lowPowerActive = false;  // currently parked + sleeping
bool          sleepArmed     = false;  // !SLEEP: sleep as soon as it is allowed
unsigned long idleSinceMs    = 0;      // millis() of the last non-open activity
unsigned int  sleepTickCount = 0;      // ticks since the last probe
unsigned int  sleepHbCount   = 0;      // ticks since the last heartbeat

// Current-measurement parking (!FLOOR).  Holds the board in one fixed state so
// a series ammeter reads a stable number:
//   1 = parked, bridge resting (ON), Software Standby
//   2 = parked, bridge OFF, Software Standby  -> delta vs 1 = the bridge leg
//   3 = parked, bridge resting (ON), WFI only -> delta vs 1 = what Standby buys
int floorMode = 0;

// ── Offline probe log ─────────────────────────────────────────────
// The sleeping probe runs with USB unplugged and so cannot print, which makes
// "it won't wake when I'm off USB" impossible to diagnose from the bench: the
// board behaves differently precisely when you cannot see it.  Every probe
// therefore records its decision to RAM, and !SLEEPLOG dumps the history once
// the unit is plugged back in.  Ring buffer -- keeps the most recent probes,
// which are the ones next to the behaviour you just observed.
const int SLOG_MAX = 64;
struct SleepLogEntry {
  float   metric;      // what was compared against the threshold
  float   retms;       // method 1/2 recovery time (ms)
  float   rest;        // resting differential at the probe (V)
  float   thr;         // threshold in force (DIP switches are live while asleep)
  uint8_t state;       // LeadState the probe decided
  uint8_t awake;       // 1 = sampled by the awake loop, 0 = by the sleeping probe
};
SleepLogEntry slog[SLOG_MAX];
int           slogValid = 0;   // entries currently held (<= SLOG_MAX)
int           slogHead  = 0;   // next write index
unsigned long slogTotal = 0;   // samples recorded since the last clear
unsigned long lastSlogAwakeMs = 0;   // pacing for awake-on-battery samples
float lastProbeThreshV = 0.0f;       // threshold the last sleeping probe used

// ⚠ SBYCR IS GLOBAL STATE.  R_LPM_Open() / R_LPM_LowPowerReconfigure() write the
// SBYCR register immediately from cfg.low_power_mode:
//     LPM_MODE_SLEEP   -> SBYCR = 0x4000 (SSBY=0)
//     LPM_MODE_STANDBY -> SBYCR = 0xC000 (SSBY=1)
// SSBY decides what EVERY __WFI() in the program does -- R_LPM_LowPowerModeEnter
// itself only executes "dsb; wfi", it does not set SSBY.  So opening the driver
// in STANDBY mode silently converts sleepMs()'s idle WFI into a full Software
// Standby: all peripheral clocks stop (USB dies mid-enumeration, "USB device not
// recognized") and millis() freezes, so sleepMs()'s millis() deadline never
// arrives and the board locks up until reset.
//
// Therefore: stay in SLEEP mode at all times, and switch to STANDBY only for the
// duration of a deliberate sleep in lowPowerTick(), switching straight back.
static lpm_instance_ctrl_t lpmCtrl;
static lpm_cfg_t           lpmSleepCfg;     // SSBY=0: plain WFI idle (the safe default)
static lpm_cfg_t           lpmStandbyCfg;   // SSBY=1: WFI enters Software Standby
static volatile bool       lpmRtcTick = false;
static bool                lpmReady   = false;

static void lpmRtcCallback() { lpmRtcTick = true; }

// Snap cfg.sleepTickMs to a rate the RTC can actually produce and program it.
// Writing the snapped value back means !CFG and the sleep log always describe
// the rate in force rather than what was asked for.  Re-programming is safe to
// repeat: IRQManager only allocates a vector slot the first time (it guards on
// periodic_irq == FSP_INVALID_VECTOR) and merely re-enables thereafter.
bool applySleepTickPeriod() {
  const SleepTickOption *best = &SLEEP_TICK_OPTIONS[0];
  long bestDiff = 0x7FFFFFFF;
  for (int i = 0; i < SLEEP_TICK_OPTION_COUNT; i++) {
    long diff = (long)cfg.sleepTickMs - (long)SLEEP_TICK_OPTIONS[i].ms;
    if (diff < 0) diff = -diff;
    if (diff < bestDiff) { bestDiff = diff; best = &SLEEP_TICK_OPTIONS[i]; }
  }
  cfg.sleepTickMs = best->ms;

  static uint16_t programmedMs = 0;
  if (programmedMs == cfg.sleepTickMs) return true;   // already in force
  if (!RTC.setPeriodicCallback(lpmRtcCallback, best->period)) return false;
  programmedMs = cfg.sleepTickMs;
  return true;
}

// Explicit prototypes: same reason as the ConfigField helpers above -- the
// Arduino preprocessor hoists its auto-generated ones too far up the file.
void enterLowPower();
void exitLowPower();
void serviceLowPower();
void serviceFloorMode();
void pollSerial();

// Bring up the RTC periodic interrupt and open the LPM driver.  Returns false
// if either fails, in which case lowPowerTick() degrades to a WFI idle -- the
// timeout mode still parks every load, it just leaves the core clocked.
bool lowPowerInit() {
  if (!RTC.begin()) return false;

  // The periodic interrupt only runs once the RTC counter is started, so seed a
  // nominal time if it isn't already running.  The value is irrelevant: nothing
  // reads the calendar, we only need the divider ticking.
  RTCTime seed(1, Month::JANUARY, 2025, 0, 0, 0,
               DayOfWeek::WEDNESDAY, SaveLight::SAVING_TIME_INACTIVE);
  RTC.setTimeIfNotRunning(seed);

  if (!applySleepTickPeriod()) return false;

  lpmStandbyCfg.low_power_mode       = LPM_MODE_STANDBY;
  lpmStandbyCfg.standby_wake_sources = LPM_STANDBY_WAKE_SOURCE_RTCPRD;
  lpmSleepCfg.low_power_mode         = LPM_MODE_SLEEP;
  lpmSleepCfg.standby_wake_sources   = 0;

  // Open in SLEEP mode: this leaves SSBY clear, so sleepMs()'s WFI keeps
  // behaving as an ordinary CPU idle.  Opening in STANDBY here would arm every
  // WFI in the program -- see the warning above.
  if (R_LPM_Open(&lpmCtrl, &lpmSleepCfg) != FSP_SUCCESS) return false;

  lpmReady = true;
  return true;
}

// Enter Software Standby until the RTC periodic interrupt fires.  Other enabled
// interrupts can also return from the WFI inside the driver, so this loops until
// the tick flag is actually set.
//
// NOTE: every peripheral clock stops in Standby, including the AGT behind
// millis() -- so millis() does NOT advance while asleep.  Nothing in this mode
// depends on it (ticks are counted, not timed) and exitLowPower() re-bases
// idleSinceMs on wake, but keep it in mind before adding millis() logic here.
void lowPowerTick() {
  if (!lpmReady) { sleepMs(cfg.sleepTickMs); return; }

  // Arm Software Standby (SSBY=1) only for this sleep, and disarm it again
  // immediately afterwards so no other WFI in the program can trip into it.
  R_LPM_LowPowerReconfigure(&lpmCtrl, &lpmStandbyCfg);
  lpmRtcTick = false;
  while (!lpmRtcTick) R_LPM_LowPowerModeEnter(&lpmCtrl);
  R_LPM_LowPowerReconfigure(&lpmCtrl, &lpmSleepCfg);
}

// ── Load parking ──────────────────────────────────────────────────
// The onboard RGB draws its own quiescent current whenever its rail is up, even
// showing black, so blanking the pixel is not enough -- RGB_POWER_PIN has to go.
static void ledPower(bool on) {
  digitalWrite(RGB_POWER_PIN, on ? HIGH : LOW);
  if (on) {
    sleepMs(1);                 // let the rail come up before clocking data out
    pixel.clear();
    pixel.show();
  }
}

// The onboard Vbatt/2 sense divider is gated by BAT_READ_EN, which setup()
// drives HIGH and normal operation never lowers -- so it draws continuously.
// There is no reason to keep it enabled while asleep.
static void battSense(bool on) {
  digitalWrite(BATT_EN_PIN, on ? HIGH : LOW);
  // NOTE: the divider needs time to settle after re-enabling (see
  // startupBatteryIndicate), so the first battV reading after a wake reads low.
}

// Park every switchable load and mark the mode active.
void enterLowPower() {
  lowPowerActive = true;
  sleepTickCount = 0;
  sleepHbCount   = 0;
  applySleepTickPeriod();        // pick up any change made via !SET / !LOAD

  silenceSpeaker();
  speakerOff();                 // park the pin idle, not merely stop the sequence
  setPixel(0, 0, 0);
  ledPower(false);
  battSense(false);
  digitalWrite(MOSFET_PIN, cfg.sleepParkOff ? MOSFET_OFF : MOSFET_ON);
}

// Restore everything and hand control back to the normal loop.
void exitLowPower() {
  lowPowerActive = false;
  digitalWrite(MOSFET_PIN, MOSFET_ON);   // resting state
  battSense(true);
  ledPower(true);

  // millis() froze while we were in Standby, so the idle timer has to be
  // re-based here rather than carried across the sleep.
  idleSinceMs = millis();
}

// One probe pass: is anything still there?  Runs the real runMosfetTest() so
// every detection method (SINGLE / TIMERET / AREA) and the live DIP threshold
// behave exactly as they do awake -- only the averaging is cut, and the test's
// trailing settle is suppressed because the pin is about to be parked anyway.
// Returns STATE_FLOAT if the leads are still open.
LeadState lowPowerProbe() {
  // DIP switches stay live while asleep, but refresh the threshold WITHOUT
  // updateThresholdFromDip(): that announces changes on serial, and a CDC write
  // with USB unplugged (which it always is here) can block.
  dipIdx        = readDipIndex();
  activeThreshV = cfg.thresh[dipIdx];

  // Wake threshold, if one is configured for this DIP position.  Kept separate
  // from thresh[] so the continuity threshold can be tuned to a target
  // resistance without that value having to double as "is anything connected".
  // Restored before returning -- runMosfetTest() reads activeThreshV, and
  // !SLEEPTEST runs this probe while awake, where the awake value must survive.
  float savedThresh = activeThreshV;
  if (cfg.sleepThresh[dipIdx] > 0.0f) activeThreshV = cfg.sleepThresh[dipIdx];

  // Measure under the SAME rail conditions the awake detector sees.  The LED
  // rail and the battery-sense divider are both loads on 3.3 V, and on a
  // floating (battery) supply their current shifts the analog baseline -- the
  // resting differential moves by ~26 mV between rails-up and rails-down, which
  // is more than refBandV.  Probing with them parked off makes the probe's
  // metric incomparable to the awake one, so no single threshold can serve both.
  // The LED is powered but written black, so nothing is visible; a few ms of
  // rail per probe costs well under 5 uA averaged.
  digitalWrite(RGB_POWER_PIN, HIGH);
  digitalWrite(BATT_EN_PIN, HIGH);
  sleepMs(1);                                 // let the rail come up
  pixel.clear();
  pixel.show();                               // hold it dark, not whatever it powered up as

  // Settle at the resting state before testing.  This is NOT optional padding:
  // detection methods 1 and 2 measure the recovery transient from the toggle,
  // and sampleRecovery() only accepts a "return" after the deviation has first
  // dipped past DET_TROUGH_MIN_V.  From an unsettled baseline that trough never
  // registers, returnMs falls back to the detWindowUs timeout, the metric lands
  // above the threshold and a CLOSED lead is misread as FLOAT -- i.e. continuity
  // silently fails to wake the board.  The awake path gets this settle for free
  // from voltagePresent()'s ten reads; the probe has to do it explicitly, and
  // needs it more, having just come out of standby with the ADC clock restarted.
  digitalWrite(MOSFET_PIN, MOSFET_ON);        // bridge to resting for the test
  sleepMs(cfg.settlePostMs);                  // rest at baseline (CPU idles)
  readVoltage();                              // throwaway: flush the ADC after the wake

  uint8_t savedPost = cfg.settlePostMs;
  cfg.settlePostMs  = 0;                      // no point settling before parking

  // voltagePresentSleep() (not voltagePresent()) -- the wide bypass band only,
  // so lead noise can't wake the board every tick.  It reads sleepVoltAvg
  // samples itself, so voltAvgSamples is left alone here.
  bool present = (voltOverride == VOLT_DISABLED) ? false : voltagePresentSleep();
  LeadState result = present ? STATE_VOLTAGE : runMosfetTest();

  cfg.settlePostMs  = savedPost;
  lastProbeThreshV  = activeThreshV;    // what the decision above was made against
  activeThreshV     = savedThresh;      // hand the awake value back

  digitalWrite(MOSFET_PIN, cfg.sleepParkOff ? MOSFET_OFF : MOSFET_ON);

  // Park the rails again, but only if we are actually asleep -- !SLEEPTEST runs
  // this same probe while awake and must not switch the LED off underneath the
  // running alert logic.
  if (lowPowerActive) {
    digitalWrite(RGB_POWER_PIN, LOW);
    digitalWrite(BATT_EN_PIN, LOW);
  }
  return result;
}

// Record one measurement.  Called from the sleeping loop, so it must not print.
// `awake` distinguishes samples taken by the normal loop (rails up) from those
// taken by the sleeping probe -- comparing the two on battery is the whole
// point, since that is where the baseline shift shows up.
void slogRecord(LeadState s, bool awake) {
  slog[slogHead].metric = lastMetric;
  slog[slogHead].retms  = lastReturnMs;
  slog[slogHead].rest   = lastRestV;
  slog[slogHead].thr    = awake ? activeThreshV : lastProbeThreshV;
  slog[slogHead].state  = (uint8_t)s;
  slog[slogHead].awake  = awake ? 1 : 0;
  slogHead = (slogHead + 1) % SLOG_MAX;
  if (slogValid < SLOG_MAX) slogValid++;
  slogTotal++;
}

// Dump the log oldest-first.  Called only from the command handler, i.e. on USB.
void dumpSleepLog() {
  Serial.print("$SLOGSTART,");
  Serial.print(slogValid);   Serial.print(",");
  Serial.print(slogTotal);   Serial.print(",method=");
  Serial.println(cfg.detectMethod);
  int start = (slogHead - slogValid + SLOG_MAX) % SLOG_MAX;
  for (int i = 0; i < slogValid; i++) {
    const SleepLogEntry &e = slog[(start + i) % SLOG_MAX];
    Serial.print("$SLOG,");
    Serial.print(i);         Serial.print(",");
    Serial.print(e.awake ? "AWAKE" : "SLEEP");
    Serial.print(",");
    Serial.print(e.state == STATE_FLOAT ? "FLOAT" :
                 (e.state == STATE_CLOSED ? "CLOSED" : "VOLTAGE"));
    Serial.print(",");       Serial.print(e.metric, 4);
    Serial.print(",");       Serial.print(e.thr, 4);
    Serial.print(",");       Serial.print(e.retms, 3);
    Serial.print(",");       Serial.println(e.rest, 4);
  }
  Serial.println("$SLOGEND");
}

// Brief "asleep, not dead" flash.
static void sleepHeartbeat() {
  if (cfg.sleepHbTicks == 0 || !cfg.ledEnable) return;
  if (++sleepHbCount < cfg.sleepHbTicks) return;
  sleepHbCount = 0;
  digitalWrite(RGB_POWER_PIN, HIGH);
  sleepMs(1);
  setPixel(0, 0, SLEEP_HB_BRIGHT);
  sleepMs(SLEEP_HB_MS);
  setPixel(0, 0, 0);
  ledPower(false);
}

// Is the board in a state where sleeping is allowed?  Diagnostics need the loop
// running, and there is no point sleeping while USB is supplying the power.
//
// The boot grace period is a safety net, not a feature: sleeping breaks the USB
// link, so if the charge detect ever misreads (bad A3 divider, wrong CHGTHRESH)
// a freshly-flashed unit could sleep before a host can reach it.  Holding it
// awake for the first SLEEP_BOOT_GRACE_MS guarantees a window to connect and
// send !SET,SLEEPSEC,0.  (Recovery does not depend on this -- the DFU
// bootloader runs before the sketch, so a double-tap of RESET always works.)
static bool lowPowerAllowed() {
  if (millis() < SLEEP_BOOT_GRACE_MS) return false;
  return (cfg.idleTimeoutS > 0) && !diagMode && !chargeActive;
}

// One pass of the sleeping loop: sleep a tick, then decide whether to wake up
// properly.  Emits no serial -- USB is unplugged by definition here, and CDC
// writes can block when it is.
void serviceLowPower() {
  lowPowerTick();

  // A host may have replugged and sent something; that always ends the mode.
  if (Serial.available()) { exitLowPower(); pollSerial(); return; }

  // Deliberately not updateChargeState(): that also reads the battery, whose
  // divider is gated off right now and would return a meaningless value.
  if (readChargeV() > cfg.chargeThreshV) {
    chargeActive = true;
    exitLowPower();
    return;
  }

  if (++sleepTickCount >= cfg.sleepPollTicks) {
    sleepTickCount = 0;
    LeadState s = lowPowerProbe();
    slogRecord(s, false);            // for !SLEEPLOG -- no serial while asleep
    if (s != STATE_FLOAT) {          // something is connected -- wake up properly
      leadState = s;
      exitLowPower();
      return;
    }
  }

  sleepHeartbeat();
}

// ── Current-measurement parking (!FLOOR) ──────────────────────────
// Holds the board in a fixed, fully-parked state so a series ammeter reads a
// stable number.  No detection, no LED, no probes: the only activity is a check
// for an exit command on each tick (a few hundred microseconds every 2 s, well
// under 1 uA averaged).  Exit with !FLOOR,0 or a reset -- note that exiting over
// USB means the meter is no longer reading the battery-only path.
void serviceFloorMode() {
  static int applied = 0;
  if (applied != floorMode) {
    applied = floorMode;
    silenceSpeaker();
    speakerOff();
    setPixel(0, 0, 0);
    ledPower(false);
    battSense(false);
    digitalWrite(MOSFET_PIN, (floorMode == 2) ? MOSFET_OFF : MOSFET_ON);
  }

  if (floorMode == 3) sleepMs(cfg.sleepTickMs);   // WFI only, for the Standby delta
  else                lowPowerTick();

  if (Serial.available()) {
    pollSerial();
    if (floorMode == 0) {            // !FLOOR,0 -- restore and resume
      applied = 0;
      digitalWrite(MOSFET_PIN, MOSFET_ON);
      battSense(true);
      ledPower(true);
      idleSinceMs = millis();
    }
  }
}

// ══════════════════════════════════════════════════════════════════
//  DIP SWITCHES -> ACTIVE THRESHOLD
// ══════════════════════════════════════════════════════════════════
// Raw pin readings, first digit D8, second digit D10 (HIGH = 1 = switch open).
// Index into cfg.thresh[]: 0b(D8)(D10), i.e. "01" = D8 low + D10 high = 1.
uint8_t readDipIndex() {
  uint8_t a = digitalRead(DIP_PIN_A) ? 1 : 0;   // D8
  uint8_t b = digitalRead(DIP_PIN_B) ? 1 : 0;   // D10
  return (a << 1) | b;
}

// Refresh activeThreshV from the switches; announce live changes on serial.
void updateThresholdFromDip() {
  uint8_t idx = readDipIndex();
  if (idx != dipIdx) {
    dipIdx = idx;
    Serial.print("$DIP,");
    Serial.print(idx);
    Serial.print(",");
    Serial.println(cfg.thresh[idx], 4);
  }
  activeThreshV = cfg.thresh[dipIdx];
}

// ══════════════════════════════════════════════════════════════════
//  MEASUREMENT
// ══════════════════════════════════════════════════════════════════
// Negative-channel read.  Normally a live SENSE_NEG sample, but when
// cfg.negFix is on it returns the count for cfg.negFixV instead, so the diff
// rides on a clean fixed pseudo-reference (the shipped default).
int readNegRaw() {
  if (cfg.negFix) {
    return (int)((cfg.negFixV / ADC_REF_VOLTAGE) * ADC_FULL_SCALE + 0.5f);
  }
  return analogRead(SENSE_NEG);
}

// Pseudo-differential read: sample both pins vs. GND and subtract.
// The RA4M1 shares one ADC + sample/hold behind an input mux, so the first
// conversion after a channel switch carries residual charge from the previous
// channel.  Discard one throwaway conversion per channel before the real read.
float readVoltage() {
  analogRead(SENSE_POS);                 // throwaway: settle S/H after prior channel
  int rawPos = analogRead(SENSE_POS);
  if (!cfg.negFix) analogRead(SENSE_NEG);
  int rawNeg = readNegRaw();
  return ((rawPos - rawNeg) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
}

// Decide whether a real voltage is present (MOSFET resting / on):
// any single read beyond voltFastMult * refBand -> present immediately;
// otherwise average voltAvgSamples reads and test against refBand.
bool voltagePresent() {
  float sum = 0.0f;
  for (int i = 0; i < cfg.voltAvgSamples; i++) {
    float v = readVoltage();
    if (fabs(v - cfg.refCenterV) > cfg.voltFastMult * cfg.refBandV) {
      lastRestV = v;
      return true;
    }
    sum += v;
  }
  lastRestV = sum / cfg.voltAvgSamples;
  return (fabs(lastRestV - cfg.refCenterV) > cfg.refBandV);
}

// Voltage check used ONLY by the sleeping probe.  Same reads as
// voltagePresent(), but the decision uses only the wide "instant bypass" band
// (voltFastMult * refBandV) and never the tight averaged refBandV test -- on a
// floating lead, noise that drifts past refBandV is enough to wake the board
// every couple of seconds, which defeats the whole point of sleeping.
//
// NOTE: with a single band there is nothing left to average.  If no individual
// read exceeds the band then their mean cannot either (the mean deviation is at
// most the largest single deviation), so the averaged comparison would be
// mathematically dead code.  The check is therefore "did ANY of sleepVoltAvg
// reads exceed the band" -- which means MORE reads = MORE chances to trip, the
// opposite of the awake path.  Keep sleepVoltAvg low, and raise VOLTFAST (not
// SLEEPAVG) if genuine noise still wakes it.
bool voltagePresentSleep() {
  float band = cfg.voltFastMult * cfg.refBandV;
  float sum  = 0.0f;
  int   n    = cfg.sleepVoltAvg;
  for (int i = 0; i < n; i++) {
    float v = readVoltage();
    if (fabs(v - cfg.refCenterV) > band) {
      lastRestV = v;
      return true;
    }
    sum += v;
  }
  lastRestV = sum / n;        // for debug only; not a decision input
  return false;
}

// Sample the recovery transient once (methods 1 & 2).  MOSFET is assumed to
// have just been switched OFF by the caller; timing starts here.  In a single
// pass this computes BOTH candidate metrics so either can be thresholded and
// the other reported for tuning:
//   lastReturnMs = time for |diff-refCentre| to fall back within detReturnBand
//                  (or detWindowUs, expressed in ms, if it never does -> OPEN)
//   lastAreaVms  = integral of |diff-refCentre| over [detAreaStartUs, window]
// Returns the metric selected by cfg.detectMethod (ms for 1, V*ms for 2).
float sampleRecovery() {
  unsigned long t0 = micros();
  float area       = 0.0f;
  float prevDev    = 0.0f;
  unsigned long prevT = 0;
  bool  havePrev   = false;
  bool  seenTrough = false;      // the dip has occurred (guards false returns)
  bool  returned   = false;
  float returnMs   = (float)cfg.detWindowUs / 1000.0f;   // timeout default
  unsigned long elapsed;

  while ((elapsed = micros() - t0) < cfg.detWindowUs) {
    float v   = readVoltage();
    float dev = fabs(v - cfg.refCenterV);
    lastTestV = v;

    if (dev > DET_TROUGH_MIN_V) seenTrough = true;

    // Tail-area: trapezoid between consecutive in-window samples.
    if (havePrev && elapsed >= cfg.detAreaStartUs && prevT >= cfg.detAreaStartUs) {
      float dt = (float)(elapsed - prevT) / 1000.0f;     // ms
      area += 0.5f * (prevDev + dev) * dt;
    }

    // Time-to-return: first crossing back within the band, but only after the
    // transient has actually dipped.
    if (!returned && seenTrough && dev <= cfg.detReturnBand) {
      returnMs = (float)elapsed / 1000.0f;
      returned = true;
      if (cfg.detectMethod == 1) break;   // time method needs nothing further
    }

    prevDev = dev; prevT = elapsed; havePrev = true;
  }

  lastReturnMs = returnMs;
  lastAreaVms  = area;
  return (cfg.detectMethod == 1) ? returnMs : area;
}

// One open/closed test: MOSFET off, derive the metric per cfg.detectMethod,
// MOSFET back on.  For every method a LARGER metric means MORE OPEN, so the
// threshold comparison and DIP table are shared across methods.
LeadState runMosfetTest() {
  digitalWrite(MOSFET_PIN, MOSFET_OFF);

  float metric;
  if (cfg.detectMethod == 0) {
    // Legacy single-sample: settle, one differential read, threshold |diff|.
    delayMicroseconds(cfg.settlePreUs);
    lastTestV = readVoltage();
    metric    = fabs(lastTestV);
  } else {
    // Methods 1 & 2 sample the recovery from the toggle instant (no pre-settle:
    // the early samples carry the transient the metrics are built from).
    metric = sampleRecovery();
  }
  lastMetric = metric;

  sleepMs(cfg.settlePostMs);
  digitalWrite(MOSFET_PIN, MOSFET_ON);
  return (metric > activeThreshV) ? STATE_FLOAT : STATE_CLOSED;
}

// Repeat the test until the same result appears cfg.testAgree times in a row
// (capped at TEST_MAX_ATTEMPTS so a noisy boundary can never hang the loop).
LeadState runMosfetTestStable() {
  LeadState result = runMosfetTest();
  LeadState prev   = result;
  int agree = 1, attempts = 1;
  while (agree < cfg.testAgree && attempts < TEST_MAX_ATTEMPTS) {
    result = runMosfetTest();
    agree  = (result == prev) ? (agree + 1) : 1;
    prev   = result;
    attempts++;
  }
  return result;
}

// ══════════════════════════════════════════════════════════════════
//  LED
// ══════════════════════════════════════════════════════════════════
void setPixel(uint8_t r, uint8_t g, uint8_t b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

// Rate-limited flash: on for onMs, then off, and no new flash until minGapMs
// after the last one began.  Flags/timestamps owned per-state by the caller.
void flashState(unsigned long now,
                uint8_t r, uint8_t g, uint8_t b,
                unsigned long onMs, unsigned long minGapMs,
                bool &flashing, unsigned long &lastFlash) {
  if (!flashing && (now - lastFlash >= minGapMs)) {
    flashing  = true;
    lastFlash = now;
    setPixel(r, g, b);
  } else if (flashing && (now - lastFlash >= onMs)) {
    flashing = false;
    setPixel(0, 0, 0);
  }
}

void updateLed() {
  static LeadState prevState = STATE_VOLTAGE;
  unsigned long now = millis();

  // On a state change, end any in-progress flash but keep the lastXFlash
  // timestamps: each state's rate limit persists across transitions so a
  // bouncing state can't re-fire immediately.
  if (leadState != prevState) {
    prevState      = leadState;
    floatFlashing  = false;
    closedFlashing = false;
    voltFlashing   = false;
    setPixel(0, 0, 0);
  }

  if (!cfg.ledEnable) { setPixel(0, 0, 0); return; }

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

// ══════════════════════════════════════════════════════════════════
//  SPEAKER
// ══════════════════════════════════════════════════════════════════
// Passive buzzer (cfg.passiveBuzzer): square wave at `freq` via tone(), so
// each alert can have its own pitch.  Active buzzer: DC level, fixed tone.
void speakerOn(unsigned int freq) {
  if (cfg.passiveBuzzer) tone(SPEAKER_PIN, freq);
  else                   digitalWrite(SPEAKER_PIN, SPEAKER_ON);
}

void speakerOff() {
  noTone(SPEAKER_PIN);                        // harmless when not toning
  digitalWrite(SPEAKER_PIN, SPEAKER_OFF);
}

// Begin a rate-limited sequence of `pulses` beeps.  `force` preempts any
// in-progress sequence and ignores the rate cap (priority alerts).
void startBeep(unsigned long now, int pulses, unsigned long onMs, unsigned long offMs,
               bool force, unsigned int freq) {
  if (!force) {
    if (beepOn || beepPulsesLeft > 0)             return;
    if (now - lastBeepSeqStart < cfg.beepMinMs)   return;
  }
  lastBeepSeqStart = now;
  beepPulsesLeft   = pulses;
  beepOnMs         = onMs;
  beepOffMs        = offMs;
  beepFreq         = freq;
  beepPhaseStart   = now;
  beepOn           = true;
  speakerOn(beepFreq);
  beepPulsesLeft--;
}

void updateBeep(unsigned long now) {
  if (!beepOn && beepPulsesLeft == 0) return;
  if (beepOn) {
    if (now - beepPhaseStart >= beepOnMs) {
      speakerOff();
      beepOn         = false;
      beepPhaseStart = now;
    }
  } else if (beepPulsesLeft > 0 && now - beepPhaseStart >= beepOffMs) {
    speakerOn(beepFreq);
    beepOn         = true;
    beepPhaseStart = now;
    beepPulsesLeft--;
  }
}

void silenceSpeaker() {
  if (beepOn) speakerOff();
  beepOn         = false;
  beepPulsesLeft = 0;
}

// Map the detection state to audio, mirroring updateLed().
//   CLOSED  -> continuity beep;  VOLTAGE -> voltage beep;  FLOAT -> silent.
// Suppressed while charging (same lockout as the LED) and when muted/disabled.
void updateSpeaker() {
  static LeadState prevState = STATE_VOLTAGE;
  unsigned long now = millis();

  if (!cfg.beepEnable || speakerMuted) {
    silenceSpeaker();
    prevState = leadState;
    return;
  }
  if (chargeActive && !alertOverride) {
    silenceSpeaker();
    prevState = leadState;      // avoid a stale beep on unplug
    return;
  }

  bool entered = (leadState != prevState);
  prevState = leadState;

  if (leadState == STATE_CLOSED) {
    if (entered)
      startBeep(now, cfg.contPulses, CONT_ON_MS, CONT_OFF_MS, false, cfg.contFreqHz);
    else if (cfg.contRepeat && now - lastBeepSeqStart >= cfg.contRepeatMs)
      startBeep(now, cfg.contPulses, CONT_ONGOING_ON_MS, CONT_ONGOING_OFF_MS, false, cfg.contFreqHz);
  } else if (leadState == STATE_VOLTAGE) {
    if (entered)                // priority alert: always sounds on entry
      startBeep(now, cfg.voltPulses, VOLT_ON_MS, VOLT_OFF_MS, true, cfg.voltFreqHz);
    else if (cfg.voltRepeat && now - lastBeepSeqStart >= cfg.voltRepeatMs)
      startBeep(now, cfg.voltPulses, VOLT_ON_MS, VOLT_OFF_MS, false, cfg.voltFreqHz);
  }

  updateBeep(now);
}

// One clean measurement at boot to decide the session mute: leads shorted
// (CLOSED) at boot -> speaker muted until the next boot cycle.
void bootSpeakerMuteCheck() {
  if (!cfg.bootMute) return;
  updateThresholdFromDip();
  digitalWrite(MOSFET_PIN, MOSFET_ON);
  if (voltagePresent()) return;          // voltage at boot -> leave audio enabled
  if (runMosfetTestStable() == STATE_CLOSED) speakerMuted = true;
}

// ══════════════════════════════════════════════════════════════════
//  CHARGE / BATTERY
// ══════════════════════════════════════════════════════════════════
float readChargeV() {
  return (analogRead(CHARGE_PIN) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE;
}

float readBattV() {
  return (analogRead(BATT_PIN) / ADC_FULL_SCALE) * ADC_REF_VOLTAGE * BATT_DIV;
}

int battPercentOf(float v) {
  float pct = (v - cfg.battEmptyV) / (cfg.battFullV - cfg.battEmptyV) * 100.0f;
  return (int)constrain(pct, 0.0f, 100.0f);
}

// VBUS high = USB plugged in -> charging.  Unplugging clears the per-session
// alert override so the next charge starts back in the charging-blink state.
void updateChargeState() {
  chargeActive = (readChargeV() > cfg.chargeThreshV);
  if (!chargeActive) alertOverride = false;
  battV   = readBattV();
  battPct = battPercentOf(battV);
}

// Slow "charging" blink: dim red at 25% duty; green once the battery reads
// full (>= cfg.battFullPct).
void chargeBlink() {
  unsigned long now   = millis();
  unsigned long phase = now - lastChargeBlink;
  if (!chargeBlinkOn && phase >= CHARGE_BLINK_PERIOD_MS) {
    chargeBlinkOn   = true;
    lastChargeBlink = now;
    uint8_t r = CHARGE_BLINK_BRIGHT, g = 0;
    if (battPct >= cfg.battFullPct) { r = 0; g = CHARGE_BLINK_BRIGHT; }
    setPixel(r, g, 0);
  } else if (chargeBlinkOn && phase >= CHARGE_BLINK_ON_MS) {
    chargeBlinkOn = false;
    setPixel(0, 0, 0);
  }
}

// LED owner: charging (and not overridden) -> charging blink; otherwise the
// normal detection alerts.  Blank + reset flags on mode transitions.
void updateAlerts() {
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

// Power-on charge-level cue: 1..4 slow green blinks (0-25% = 1 ... 75-100% = 4).
void startupBatteryIndicate() {
  // BAT_READ_EN is driven HIGH at the very top of setup(), so by now the
  // divider has had the whole of init to settle; this pre-gap finishes the job
  // and is the pause the cue wanted anyway, so the settle is effectively free.
  // (It replaces a dedicated 100 ms discard loop that used to sit here.)
  sleepMs(BOOT_CUE_PREGAP_MS);
  for (int i = 0; i < 4; i++) readBattV();      // discard: flush the ADC S/H
  float v = 0.0f;
  for (int i = 0; i < 8; i++) v += readBattV();
  v /= 8.0f;

  int blinks = battPercentOf(v) / 25 + 1;
  if (blinks > 4) blinks = 4;
  for (int i = 0; i < blinks; i++) {
    setPixel(0, CHARGE_BLINK_BRIGHT, 0);
    sleepMs(BOOT_CUE_ON_MS);
    setPixel(0, 0, 0);
    // No trailing gap after the last blink -- nothing follows it to separate
    // from, and it would just delay the first measurement.
    if (i < blinks - 1) sleepMs(BOOT_CUE_OFF_MS);
  }
}

// ══════════════════════════════════════════════════════════════════
//  DETECTION (one pass) -- sets leadState, honouring voltOverride
// ══════════════════════════════════════════════════════════════════
void runDetection() {
  updateThresholdFromDip();              // DIP switches are live
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

  // Display debounce: commit to leadState only after the raw result repeats
  // cfg.stableCount passes in a row, so a single noisy test can't flip the
  // alert and cancel an in-progress LED flash.
  static LeadState candidate   = STATE_VOLTAGE;
  static int       stableCount = 0;
  if (rawState == leadState) {
    candidate   = rawState;
    stableCount = 0;
  } else {
    if (rawState != candidate) { candidate = rawState; stableCount = 0; }
    if (++stableCount >= cfg.stableCount) {
      leadState   = rawState;
      stableCount = 0;
    }
  }
}

// ══════════════════════════════════════════════════════════════════
//  DIAGNOSTICS: status, streaming, capture
// ══════════════════════════════════════════════════════════════════
void printStatus() {
  Serial.print("$STATUS,diag=");  Serial.print(diagMode ? 1 : 0);
  Serial.print(",vmode=");        Serial.print((int)voltOverride);
  Serial.print(",mosfet=");       Serial.print(mosfetHold);
  Serial.print(",stream=");       Serial.print(streamOn ? 1 : 0);
  Serial.print(",rate=");         Serial.print(streamIntervalMs);
  Serial.print(",capms=");        Serial.print(capDurationMs);
  Serial.print(",res=");          Serial.print(ADC_RESOLUTION);
  Serial.print(",vref=");         Serial.print(ADC_REF_VOLTAGE, 3);
  Serial.print(",dip=");          Serial.print(dipIdx);
  Serial.print(",openthr=");      Serial.print(activeThreshV, 3);
  Serial.print(",detmethod=");    Serial.print(cfg.detectMethod);
  Serial.print(",metric=");       Serial.print(lastMetric, 4);
  Serial.print(",retms=");        Serial.print(lastReturnMs, 3);
  Serial.print(",areavms=");      Serial.print(lastAreaVms, 4);
  Serial.print(",negfix=");       Serial.print(cfg.negFix ? 1 : 0);
  Serial.print(",negv=");         Serial.print(cfg.negFixV, 3);
  Serial.print(",charge=");       Serial.print(chargeActive ? 1 : 0);
  Serial.print(",alertovr=");     Serial.print(alertOverride ? 1 : 0);
  Serial.print(",muted=");        Serial.print(speakerMuted ? 1 : 0);
  Serial.print(",dirty=");        Serial.print(cfgDirty ? 1 : 0);
  Serial.print(",lp=");           Serial.print(lowPowerActive ? 1 : 0);
  Serial.print(",armed=");        Serial.print(sleepArmed ? 1 : 0);
  Serial.print(",idle=");         Serial.print((millis() - idleSinceMs) / 1000);
  Serial.print(",floor=");        Serial.print(floorMode);
  Serial.print(",battpct=");      Serial.print(battPct);
  Serial.print(",battv=");        Serial.print(battV, 3);
  Serial.print(",sn=");           Serial.println(unitSN);  // last: may be empty
}

// One streamed sample: both pins independently + computed differential.
void streamSample() {
  analogRead(SENSE_POS);                 // throwaway: settle S/H after prior channel
  int rawPos = analogRead(SENSE_POS);
  if (!cfg.negFix) analogRead(SENSE_NEG);
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

// Capture both ADC pins as fast as possible (no settling delays) across a
// MOSFET toggle: baseline, toggle OFF at CAP_PRE_US, sample until durationMs
// or the buffer fills, restore ON, dump raw counts to the host.
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
  Serial.print(capCount);          Serial.print(",");
  Serial.print(toggleUs);          Serial.print(",");
  Serial.print(durationMs);        Serial.print(",");
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

// ══════════════════════════════════════════════════════════════════
//  SERIAL COMMAND HANDLING
// ══════════════════════════════════════════════════════════════════
void handleLine(char *line) {
  if (line[0] != '!') return;
  char *cmd = line + 1;
  char *arg = strchr(cmd, ',');
  if (arg) { *arg = '\0'; arg++; }
  for (char *p = cmd; *p; ++p) *p = toupper(*p);

  // ── Configuration ────────────────────────────────────────────
  if (strcmp(cmd, "SET") == 0) {
    // !SET,<key>,<value>  -- set a config field in RAM (clamped to its
    // min/max), applied immediately.  Persist with !SAVE.
    char *val = arg ? strchr(arg, ',') : NULL;
    if (!arg || !val) { Serial.println("$ERR,set,usage !SET,<key>,<value>"); return; }
    *val = '\0'; val++;
    for (char *p = arg; *p; ++p) *p = toupper(*p);
    const ConfigField *f = findField(arg);
    if (!f) { Serial.print("$ERR,set,unknown key "); Serial.println(arg); return; }
    fieldSet(f, atof(val));
    // The wake period can only take a value the RTC can produce, so snap it
    // before echoing -- otherwise the reply reports a rate that was never set.
    if (strcmp(f->name, "SLEEPTICKMS") == 0) applySleepTickPeriod();
    cfgDirty = true;
    printField(f);                       // echo the (possibly clamped) value
  } else if (strcmp(cmd, "GET") == 0) {
    if (!arg) { Serial.println("$ERR,get,usage !GET,<key>"); return; }
    for (char *p = arg; *p; ++p) *p = toupper(*p);
    const ConfigField *f = findField(arg);
    if (!f) { Serial.print("$ERR,get,unknown key "); Serial.println(arg); return; }
    printField(f);
  } else if (strcmp(cmd, "CFG") == 0) {
    for (int i = 0; i < CFG_FIELD_COUNT; i++) printField(&CFG_FIELDS[i]);
    Serial.println("$CFGEND");
  } else if (strcmp(cmd, "SAVE") == 0) {
    configSave();
    // Verify: read the flash image back and compare byte-for-byte, so a
    // failed/incomplete data-flash write reports $ERR instead of a false $OK.
    Config check;
    EEPROM.get(CFG_EEPROM_ADDR, check);
    if (memcmp(&check, &cfg, sizeof(Config)) == 0) {
      Serial.println("$OK,save");
    } else {
      Serial.println("$ERR,save,verify failed (flash readback mismatch)");
    }
  } else if (strcmp(cmd, "LOAD") == 0) {
    if (configLoad()) Serial.println("$OK,load");
    else              Serial.println("$ERR,load,stored config invalid");
  } else if (strcmp(cmd, "DEFAULTS") == 0) {
    configDefaults();
    cfgDirty = true;                     // RAM now differs from EEPROM
    Serial.println("$OK,defaults");
  } else if (strcmp(cmd, "SN") == 0) {
    // !SN            -> report the stored serial number ($SN,<value>)
    // !SN,<value>    -> write it to EEPROM (persists immediately; it is
    //                   device identity, not part of the tunable config).
    if (arg) {
      while (*arg == ' ') arg++;         // tolerate a leading space
      if (*arg == '\0') {
        Serial.println("$ERR,sn,empty");
      } else if (strchr(arg, ',')) {
        Serial.println("$ERR,sn,comma not allowed");   // keeps host CSV clean
      } else if (strlen(arg) > SN_MAX_LEN - 1) {
        Serial.print("$ERR,sn,too long (max ");
        Serial.print(SN_MAX_LEN - 1);
        Serial.println(")");
      } else {
        snSave(arg);
        Serial.print("$SN,");  Serial.println(unitSN);
        Serial.println("$OK,sn");
      }
    } else {
      Serial.print("$SN,");  Serial.println(unitSN);
    }

  // ── Diagnostics / overrides ──────────────────────────────────
  } else if (strcmp(cmd, "DIAG") == 0) {
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
  } else if (strcmp(cmd, "ALERTS") == 0) {
    // Re-enable normal alerts while charging (override auto-clears on unplug).
    alertOverride = arg ? (atoi(arg) != 0) : !alertOverride;
    printStatus();
  } else if (strcmp(cmd, "SLEEP") == 0) {
    // !SLEEP  arm the low-power timeout to fire as soon as it is allowed --
    // i.e. expire the countdown now, so the board sleeps the moment it is idle
    // and off USB.  Arm it over USB, then unplug (same pattern as !OLOG).
    // The timeout length and probe cadence are config (SLEEPSEC / SLEEPTICKS
    // / SLEEPAVG / SLEEPHB / SLEEPPARK), set with !SET and kept with !SAVE.
    if (arg && atoi(arg) == 0) {
      sleepArmed = false;                // !SLEEP,0 -- cancel a pending arm
      idleSinceMs = millis();
      Serial.println("$OK,sleep,disarmed");
    } else if (cfg.idleTimeoutS == 0) {
      Serial.println("$ERR,sleep,disabled (set SLEEPSEC > 0)");
    } else {
      sleepArmed = true;
      Serial.println("$OK,sleep,armed");
    }
    printStatus();
  } else if (strcmp(cmd, "SLEEPLOG") == 0) {
    // !SLEEPLOG    dump every probe the board made while it was asleep
    // !SLEEPLOG,0  clear the log
    // Rows are $SLOG,<i>,<state>,<metric>,<thr>,<retms>,<rest> -- oldest first.
    // This is how you see what a CLOSED lead actually measures off USB, where
    // the ground reference (and therefore the metric) is not what it is on the
    // bench.  Tune the DIP threshold against these numbers, not the USB ones.
    if (arg && atoi(arg) == 0) {
      slogValid = 0; slogHead = 0; slogTotal = 0;
      Serial.println("$OK,sleeplog,cleared");
    } else {
      dumpSleepLog();
    }
  } else if (strcmp(cmd, "SLEEPTEST") == 0) {
    // Run one sleeping-mode probe right now, awake and over USB, and report
    // exactly what it decided.  The probe normally runs unplugged and silent,
    // so this is the only way to see its numbers.  Short the leads and compare
    // with the periodic debug line: if this reports CLOSED but a sleeping unit
    // still won't wake, the difference is the standby wake itself rather than
    // the probe logic.
    LeadState s = lowPowerProbe();
    digitalWrite(MOSFET_PIN, MOSFET_ON);      // undo the probe's park
    Serial.print("$SLEEPTEST,");
    Serial.print(s == STATE_FLOAT ? "FLOAT" : (s == STATE_CLOSED ? "CLOSED" : "VOLTAGE"));
    Serial.print(",metric=");  Serial.print(lastMetric, 4);
    Serial.print(",thr=");     Serial.print(lastProbeThreshV, 4);   // wake threshold used
    Serial.print(",awakethr="); Serial.print(activeThreshV, 4);
    Serial.print(",rest=");    Serial.print(lastRestV, 4);
    Serial.print(",retms=");   Serial.print(lastReturnMs, 3);
    Serial.print(",areavms="); Serial.print(lastAreaVms, 4);
    Serial.print(",method=");  Serial.println(cfg.detectMethod);
  } else if (strcmp(cmd, "FLOOR") == 0) {
    // !FLOOR,<0-3>  park the board in a fixed state for a current measurement:
    //   0 = exit    1 = parked, bridge resting, standby
    //   2 = parked, bridge off, standby       3 = parked, bridge resting, WFI only
    // Take the reading on battery with the meter in series; the deltas between
    // levels are what say which loads are worth switching in hardware.
    floorMode = arg ? constrain(atoi(arg), 0, 3) : 0;
    printStatus();
    Serial.flush();              // last words before the board goes quiet
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

// ══════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════
void setup() {
  // Battery sense first.  The BAT_READ_EN divider needs time to settle before
  // the power-on charge cue can read it; starting it here lets that settle
  // overlap the rest of init rather than being paid for as its own delay.
  pinMode(BATT_PIN, INPUT);              // BAT_DET_PIN (P105) = Vbatt/2 sense
  pinMode(BATT_EN_PIN, OUTPUT);          // BAT_READ_EN (P400)
  digitalWrite(BATT_EN_PIN, HIGH);

  analogReadResolution(ADC_RESOLUTION);
  pinMode(CHARGE_PIN, INPUT);            // A3 = VBUS/2 (USB-power sense)

  Serial.begin(115200);
  // The USB settle only buys anything when a host is actually attached.  On
  // battery -- the case that decides time-to-first-measurement -- it is pure
  // dead time, so gate it on VBUS.  cfg is not loaded yet, hence the literal
  // threshold instead of cfg.chargeThreshV.
  readChargeV();                         // throwaway: first conversion after reset
  if (readChargeV() > BOOT_USB_THRESH_V) delay(300);

  // Config first: everything below reads cfg.
  configDefaults();
  bool loaded    = configLoad();
  bool migrated  = false;
  if (!loaded) {
    // Not a current-version image.  Before falling back to factory defaults,
    // see whether it is a valid older layout worth upgrading -- a unit tuned in
    // the field must not lose its thresholds just because the struct grew.
    // Newest layout first, so a v3 image is never mis-read as something older.
    migrated = configMigrateV4() || configMigrateV3() || configMigrateV2();
    configSave();                        // persist the migration (or seed defaults)
  }
  snLoad();                              // unit serial number (separate block)

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, MOSFET_ON);   // resting state: MOSFET high

  pinMode(SENSE_POS, INPUT);
  pinMode(SENSE_NEG, INPUT);

  pinMode(DIP_PIN_A, INPUT);             // hardware pull-ups on the PCB
  pinMode(DIP_PIN_B, INPUT);

  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, SPEAKER_OFF);

  // (CHARGE_PIN / BATT_PIN / BATT_EN_PIN are set up at the top of setup(), so
  //  the battery divider is already settling while the rest of this runs.)

  pinMode(RGB_POWER_PIN, OUTPUT);        // onboard NeoPixel power rail
  digitalWrite(RGB_POWER_PIN, HIGH);

  pixel.begin();
  pixel.clear();
  pixel.show();

  startupBatteryIndicate();              // power-on battery charge-level cue

  bootSpeakerMuteCheck();                // leads CLOSED at boot -> session mute
  Serial.print("SN: ");
  Serial.println(unitSN[0] ? unitSN : "(unassigned -- write with !SN,<value>)");
  Serial.print("Config: ");
  if (loaded)        Serial.println("loaded from EEPROM");
  else if (migrated) Serial.println("migrated from v2 EEPROM (tuning preserved)");
  else               Serial.println("defaults (EEPROM seeded)");
  Serial.print("Speaker: ");
  Serial.println(speakerMuted ? "MUTED (leads closed at boot)" : "enabled");
  Serial.print("DIP: ");
  Serial.print(readDipIndex());
  Serial.print(" -> threshold ");
  Serial.print(cfg.thresh[readDipIndex()], 3);
  Serial.println(" V");

  // Bring up the sleep timer / wake source.  A failure is not fatal: the
  // timeout mode falls back to a WFI idle, which still parks every load.
  bool lpOk = lowPowerInit();
  idleSinceMs = millis();
  Serial.print("Sleep: ");
  if (cfg.idleTimeoutS == 0) {
    Serial.println("disabled (SLEEPSEC=0)");
  } else {
    Serial.print(cfg.idleTimeoutS);
    Serial.print(" s timeout, probe every ");
    Serial.print((unsigned long)cfg.sleepTickMs * cfg.sleepPollTicks);
    Serial.println(lpOk ? " ms (standby)" : " ms (WFI fallback -- RTC/LPM init failed)");
  }

  Serial.println("BlinkyHawk_RA4M1 ready.");
}

// ══════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════════════════════════════
void loop() {
  pollSerial();

  // Measurement parking and the sleeping loop each own the board completely --
  // they run before everything else and return without touching detection.
  if (floorMode)      { serviceFloorMode(); return; }
  if (lowPowerActive) { serviceLowPower();  return; }

  // ── Diagnostic mode ─────────────────────────────────────────
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

    updateChargeState();
    updateAlerts();
    updateSpeaker();
    delay(1);                  // light idle; streaming sets its own pace
    return;
  }

  // ── Normal mode ─────────────────────────────────────────────
  runDetection();
  updateChargeState();
  updateAlerts();
  updateSpeaker();

  // Periodic human-readable debug
  if (millis() - lastSerialTime >= serialInterval) {
    lastSerialTime = millis();
    Serial.print("Rest:");
    Serial.print(lastRestV, 3);
    Serial.print("V  ");
    if (leadState == STATE_VOLTAGE) {
      Serial.println("-> VOLTAGE (bypass)");
    } else {
      // Metric + units depend on the active detection method; print the metric
      // that was actually thresholded plus the raw return/area for tuning.
      Serial.print("m");   Serial.print(cfg.detectMethod);
      Serial.print(" metric:");
      if      (cfg.detectMethod == 1) { Serial.print(lastMetric, 3); Serial.print("ms"); }
      else if (cfg.detectMethod == 2) { Serial.print(lastMetric, 4); Serial.print("Vms"); }
      else                            { Serial.print(lastMetric, 3); Serial.print("V"); }
      Serial.print(" (ret:"); Serial.print(lastReturnMs, 3);
      Serial.print("ms area:"); Serial.print(lastAreaVms, 4);
      Serial.print("Vms thr:"); Serial.print(activeThreshV, 3);
      Serial.print(")  -> ");
      Serial.println(leadState == STATE_FLOAT ? "FLOATING" : "CLOSED");
    }
  }

  // Log awake samples while on battery, at the same cadence the sleeping probe
  // uses.  This is what makes the baseline shift visible: unplug, let it run
  // awake for a while, let it sleep, replug and compare the AWAKE and SLEEP
  // rows in !SLEEPLOG.  Skipped on USB -- the host can already see those.
  if (!chargeActive && millis() - lastSlogAwakeMs >= cfg.sleepTickMs) {
    lastSlogAwakeMs = millis();
    slogRecord(leadState, true);
  }

  // Inactivity timer.  Anything other than an open lead counts as activity, as
  // does any state in which sleeping is disallowed -- so the countdown starts
  // fresh once the board is idle AND allowed to sleep, rather than expiring
  // while it was busy or on USB.  An armed !SLEEP is exempt from the reset:
  // it has to survive being issued over USB until the unit is unplugged.
  if (leadState != STATE_FLOAT || !lowPowerAllowed()) {
    if (!sleepArmed) idleSinceMs = millis();
  } else if (sleepArmed ||
             millis() - idleSinceMs >= (unsigned long)cfg.idleTimeoutS * 1000UL) {
    sleepArmed = false;
    enterLowPower();
    return;
  }

  sleepMs(cfg.loopDelayMs);    // pace the loop with a real CPU idle (WFI)
}
