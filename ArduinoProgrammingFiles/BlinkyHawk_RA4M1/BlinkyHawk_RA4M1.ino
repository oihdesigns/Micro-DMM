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
#define CFG_VERSION 2              // bumped: added detectMethod + recovery params
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
  // BAT_READ_EN was only just driven HIGH, so discard reads over a settling
  // window before averaging -- otherwise the count reflects a rising voltage.
  for (int i = 0; i < 20; i++) { readBattV(); delay(5); }
  float v = 0.0f;
  for (int i = 0; i < 8; i++) { v += readBattV(); delay(2); }
  v /= 8.0f;

  int blinks = battPercentOf(v) / 25 + 1;
  if (blinks > 4) blinks = 4;
  sleepMs(400);
  for (int i = 0; i < blinks; i++) {
    setPixel(0, CHARGE_BLINK_BRIGHT, 0);
    sleepMs(200);
    setPixel(0, 0, 0);
    sleepMs(250);
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
  Serial.begin(115200);
  delay(300);

  // Config first: everything below reads cfg.
  configDefaults();
  bool loaded = configLoad();
  if (!loaded) configSave();             // first boot / stale layout: seed EEPROM
  snLoad();                              // unit serial number (separate block)

  analogReadResolution(ADC_RESOLUTION);

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, MOSFET_ON);   // resting state: MOSFET high

  pinMode(SENSE_POS, INPUT);
  pinMode(SENSE_NEG, INPUT);

  pinMode(DIP_PIN_A, INPUT);             // hardware pull-ups on the PCB
  pinMode(DIP_PIN_B, INPUT);

  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, SPEAKER_OFF);

  pinMode(CHARGE_PIN, INPUT);            // A3 = VBUS/2 (USB-power sense)
  pinMode(BATT_PIN, INPUT);              // BAT_DET_PIN (P105) = Vbatt/2 sense
  pinMode(BATT_EN_PIN, OUTPUT);          // BAT_READ_EN (P400)
  digitalWrite(BATT_EN_PIN, HIGH);       // enable the battery-sense divider
  delay(2);

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
  Serial.println(loaded ? "loaded from EEPROM" : "defaults (EEPROM seeded)");
  Serial.print("Speaker: ");
  Serial.println(speakerMuted ? "MUTED (leads closed at boot)" : "enabled");
  Serial.print("DIP: ");
  Serial.print(readDipIndex());
  Serial.print(" -> threshold ");
  Serial.print(cfg.thresh[readDipIndex()], 3);
  Serial.println(" V");
  Serial.println("BlinkyHawk_RA4M1 ready.");
}

// ══════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════════════════════════════
void loop() {
  pollSerial();

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

  sleepMs(cfg.loopDelayMs);    // pace the loop with a real CPU idle (WFI)
}
