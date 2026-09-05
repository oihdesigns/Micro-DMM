/*
 * DMMTypes.h -- shared type definitions for the microDMM firmware.
 *
 * WHY A HEADER AT ALL, in a sketch that is otherwise .ino tabs:
 * the Arduino preprocessor auto-generates function prototypes and inserts
 * them at the top of the concatenated sketch, ABOVE anything declared in the
 * .ino body.  Any function whose signature mentions Config, ConfigField or
 * Mode would therefore be prototyped before those types exist ("does not name
 * a type").  Types reached through a #include land before the generated
 * prototypes, so putting them here removes the problem for the whole sketch
 * instead of needing a hand-written prototype after every struct.
 *
 * Only declarations live here.  All code is in the .ino tabs.
 */
#ifndef DMM_TYPES_H
#define DMM_TYPES_H

#include <Arduino.h>

// ==================================================================
//  MEASUREMENT MODE
// ==================================================================
// Cycled by the mode button (MODE_BUTTON).  Deliberately a plain enum with
// int underlying type: Serial.print()/display.print() of a uint8_t-based enum
// resolves to the character overload and would print the mode as a glyph.
enum Mode {
  Default,       // 0  auto: pick voltage or resistance from what is connected
  Voltmeter,     // 1  force voltage
  VACmanual,     // 2  force AC voltage
  Type,          // 3  short press types the reading over USB HID
  Low,           // 4  "precise" -- slow ADC, extra digits, no VAC
  AltUnitsMode,  // 5  thermistor degF / x50 voltage
  HighRMode,     // 6  resistance only, no auto-open clamp
  Charging,      // 7  screen off, ohms PWM parked
  NUM_MODES
};

// ==================================================================
//  EEPROM LAYOUT
// ==================================================================
// The RA4M1 data flash is 8 KB, so these are nowhere near the ceiling.
//
// Byte 1 is the ORIGINAL unit-id byte, written by hand on every meter built
// so far and read by the old eepromSetup() to pick a calibration set.  The
// Config block deliberately starts at 64 so that byte survives untouched --
// it is the only record of which calibration a unit shipped with, and
// configSeedFromLegacy() re-reads it to seed a blank config.
#define LEGACY_ID_ADDR    1
#define CFG_EEPROM_ADDR   64
#define SN_EEPROM_ADDR    768     // clear of Config; survives !DEFAULTS

#define CFG_MAGIC         0x55444D31UL   // "UDM1"
#define CFG_VERSION       1

#define R_CAL_BUCKETS     15      // the CF_A..CF_O piecewise ladder
#define VOLT_SAMPLE_MAX   100     // buffer size; cfg.vSamples is the live count
#define SN_MAX_LEN        16

// ==================================================================
//  CONFIG
// ==================================================================
// Layout changes REQUIRE bumping CFG_VERSION -- configLoad() rejects a stored
// image whose version differs, and a mismatched struct read back over the old
// bytes is silently garbage.
//
// The CRC is the LAST field.  Checksum it with offsetof(Config, crc), never
// sizeof(Config)-2: the struct is 4-byte aligned, so tail padding follows crc
// and the sizeof form would run the CRC over the crc field itself, failing
// validation on every boot.
struct Config {
  uint32_t magic;
  uint16_t version;

  // ---- Board / unit -------------------------------------------
  uint8_t  hwRev;          // legacy unit id (splash + !SEEDCAL); not a preference
  uint8_t  bridge;         // has the float-detect bridge MOSFET circuit

  // ---- Resistance calibration ---------------------------------
  float    rCal[R_CAL_BUCKETS];  // piecewise correction, was CF_A..CF_O
  float    constantI;      // A, low-range constant-current source
  float    constantR;      // Ohm, internal resistor in that source
  float    dividerR;       // Ohm, series resistor for the high-range divider
  float    zenerMaxV;      // V, reference ceiling in high-range mode
  float    sleepV;         // V, ohms rail once the power-save PWM is applied

  // ---- Voltage calibration ------------------------------------
  float    voltScale;      // front-end divider ratio
  float    altMult;        // x50 applied in AltUnitsMode
  float    thermR0;        // thermistor nominal resistance at 25 C
  float    thermB;         // thermistor beta

  // ---- Current calibration ------------------------------------
  float    iShunt;         // V per A of the hall sensor / shunt
  float    iZero;          // baseline; overwritten at boot when iAutoZero
  uint8_t  iAutoZero;      // re-detect the baseline at every boot
  float    iNoiseHi;       // deadband applied in high range
  float    iNoiseLo;       // deadband applied in low range
  // Boot detection of the current sensor: a reading below iDetCount counts as
  // a grounded (low-range) shunt, one inside iDetLo..iDetHi as a hall sensor
  // sitting at its mid-rail bias.  Anything else means no sensor is fitted.
  uint16_t iDetCount;
  float    iDetLo;
  float    iDetHi;

  // ---- Ranging ------------------------------------------------
  float    rangeThreshR;   // Ohm, high/low range crossover
  float    rangeDeadband;  // fraction, hysteresis around it
  uint16_t adcCountLow;    // step gain UP below this count
  uint16_t adcCountHigh;   // step gain DOWN above this count
  float    openMargin;     // V below zenerMaxV that still counts as open
  float    openR;          // Ohm reported for an open circuit
  uint16_t vDispCount;     // |differential counts| that auto-selects voltage
  float    rDispMin;       // Ohm window that auto-selects resistance
  float    rDispMax;
  float    zeroAutoMax;    // Ohm, largest reading accepted as a lead auto-zero

  // ---- ADC data-rate scheduling -------------------------------
  // The ADS1115 is run fast while a reading is moving and slow once it
  // settles.  These decide "moving" and "settled".
  float    rateJump;       // ratio outside which a reading counts as a jump
  float    rateBumpV;      // ohms rail above which a jump is ignored
  float    rateSlowV;      // V below zenerMaxV that permits the medium rate
  float    ratePrecR;      // Ohm ceiling for the slow precise-mode rate
  float    rateStable;     // fraction within which a reading counts as stable
  float    mmRMax;         // Ohm bounds on resistance min/max tracking
  float    mmRMin;

  // ---- Power save ---------------------------------------------
  uint16_t psHoldMs;       // ms at the rail before power-save engages
  float    psMargin;       // V below zenerMaxV that counts as "at the rail"
  float    psHyst;         // V below sleepV that releases power-save
  float    psCancelR;      // Ohm reading that cancels a pending power-save
  uint8_t  psPwm;          // ohms-pin PWM duty while in power-save
  uint16_t sleepSec;       // s of quiet before the screen blanks
  float    sleepVMax;      // |V| under which the meter counts as quiet
  uint8_t  psDebug;        // narrate power-save transitions on serial

  // ---- Timing -------------------------------------------------
  uint16_t adcMs;
  uint16_t battMs;
  uint16_t lcdMs;
  uint16_t lcdFastMs;
  uint16_t lcdBumpMs;      // temporary refresh period after a big change
  uint16_t streamMs;       // $LIVE cadence

  // ---- Alerts -------------------------------------------------
  uint8_t  alertsOn;
  float    contMin;        // continuity window, low edge
  float    contMax;        // continuity window, high edge (low range)
  float    contMaxHi;      // continuity window, high edge (high range)
  float    vAlert;         // V that raises the voltage warning
  float    vAlertAlt;      // ...in AltUnitsMode
  float    vacAlert;
  float    vacAlertAlt;
  uint8_t  beepBright;     // first-pulse PWM on the buzzer/LED pin
  uint8_t  beepHold;       // sustain PWM
  uint8_t  blinkLimit;     // alert pulses allowed between screen updates
  // Repeat pattern, as offsets into a rolling window of alertPerMs.  The
  // continuity alert sounds twice per window (0..alertOnMs and
  // alertP2OnMs..alertP2OffMs); the voltage alert uses the first pulse only.
  uint16_t alertPerMs;
  uint16_t alertOnMs;
  uint16_t alertP2OnMs;
  uint16_t alertP2OffMs;

  // ---- AC detection -------------------------------------------
  float    vacThresh;      // VAC rms that declares AC present
  float    vacThreshAlt;   // ...in AltUnitsMode
  float    vacAvgMax;      // |DC average| below which AC may be declared

  // ---- Float / closed bridge ----------------------------------
  float    bridgeThr;      // V, more negative than this = floating
  float    bridgeFltThr;   // V, split between "floating" and "unsure"
  float    bridgeAvgMax;   // gate: |DC average| must be under this
  float    bridgeVMax;     // gate: instantaneous reading must be under this
  float    bridgeVacMax;   // gate in VACmanual: VAC must be under this

  // ---- Filtering / misc ---------------------------------------
  uint8_t  vSamples;       // live length of the rolling voltage buffer
  float    smoothAlpha;    // EMA coefficient for the display value
  float    battScale;      // multiplier on the battery divider
  uint8_t  keyboardEn;     // USB HID typing in Type / HighRMode
  uint16_t btnLongMs;      // TYPE_PIN hold that resets min/max
  uint16_t btnShortMs;     // debounce floor for a short press

  uint16_t crc;            // MUST stay last -- see the offsetof note above
};

// ==================================================================
//  GENERIC KEY TABLE
// ==================================================================
enum FieldType { FT_FLOAT, FT_U16, FT_U8, FT_BOOL };

struct ConfigField {
  const char *name;
  FieldType   type;
  void       *ptr;
  float       minV;
  float       maxV;
};

// ==================================================================
//  LEGACY CALIBRATION
// ==================================================================
// The six per-unit calibration sets that used to be an if/else ladder over
// EEPROM byte 1 in CalibrationConstants.ino.  Kept read-only so a meter
// flashed with this firmware comes up already calibrated; after that the
// values are live config keys and this table is only re-read on !SEEDCAL.
struct LegacyCal {
  uint8_t id;
  float   cf[R_CAL_BUCKETS];
  float   voltScale;
  float   constantI;
  float   zenerMaxV;
  float   sleepV;
  uint8_t bridge;
};

// ==================================================================
//  UNIT SERIAL NUMBER
// ==================================================================
// Its own EEPROM block so it survives !DEFAULTS and any CFG_VERSION bump --
// the serial number is the unit's permanent identity, not part of its tuning.
struct SerialId {
  uint32_t magic;
  char     sn[SN_MAX_LEN];
  uint16_t crc;
};
#define SN_MAGIC 0x554D5F53UL    // "UM_S"

// ==================================================================
//  CROSS-TAB DECLARATIONS
// ==================================================================
// Defined in Config.ino, used from the main tab (which the preprocessor
// places first, so it cannot see them otherwise).
extern Config            cfg;
extern bool              cfgDirty;
extern char              unitSN[SN_MAX_LEN];
extern const ConfigField CFG_FIELDS[];
extern const int         CFG_FIELD_COUNT;
extern const float       R_CAL_EDGES[R_CAL_BUCKETS - 1];

#endif // DMM_TYPES_H
