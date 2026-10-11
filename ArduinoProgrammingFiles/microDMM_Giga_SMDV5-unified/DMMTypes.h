/*
 * DMMTypes.h -- shared type definitions for the GigaMeter firmware.
 *
 * WHY A HEADER AT ALL, in a sketch that is otherwise .ino tabs:
 * the Arduino preprocessor auto-generates function prototypes and inserts
 * them at the top of the concatenated sketch, ABOVE anything declared in the
 * .ino body.  Any function whose signature mentions Config, ConfigField or
 * Mode would therefore be prototyped before those types exist ("does not name
 * a type").  Types reached through a #include land before the generated
 * prototypes, so putting them here removes the problem for the whole sketch.
 *
 * Only declarations live here.  All code is in the .ino tabs.
 */
#ifndef DMM_TYPES_H
#define DMM_TYPES_H

#include <Arduino.h>

// ==================================================================
//  MEASUREMENT MODE
// ==================================================================
// Cycled by the mode button, the red touch button and the mic-jack ladder, in
// this order.  The ORDER IS THE GIGA'S, not the XIAO's -- the GUI learns the
// names from the modes= field of $STATUS rather than assuming either list.
// Plain int-based enum: printing a uint8_t-based enum resolves to the char
// overload and prints a glyph.
enum Mode {
  Voltmeter,     // 0  force voltage; ohms source parked
  HighRMode,     // 1  resistance only, no auto-open margin, plotted
  Default,       // 2  auto: pick voltage or resistance from what is connected
  VACmanual,     // 3  force AC voltage
  Type,          // 4  Ref action types the reading over USB HID
  Low,           // 5  "precise" -- slow ADC, extra digits, no VAC
  AltUnitsMode,  // 6  thermistor degF / x50 voltage
  rPlotMode,     // 7  resistance with the ohms-rail plot
  Charging,      // 8  screen off, ohms source parked
  NUM_MODES
};

// ==================================================================
//  PERSISTENT STORAGE
// ==================================================================
// The Giga has no EEPROM.  Config and serial number live in raw QSPI flash,
// in a window the standard QSPIFormat layout leaves unused: its partitions end
// at 14 MB and the memory-mapped WiFi firmware starts at 15.5 MB.  The two
// blocks are 64 KB apart so that even a 64 KB erase of one cannot touch the
// other.  flashSetup() reads the partition table and refuses to write if any
// partition overlaps this window.
#define FLASH_CFG_ADDR    (14UL * 1024UL * 1024UL)
#define FLASH_SN_ADDR     (FLASH_CFG_ADDR + 64UL * 1024UL)
#define FLASH_REGION_END  (FLASH_CFG_ADDR + 128UL * 1024UL)
#define FLASH_BUF_MAX     1024

#define CFG_MAGIC         0x47444D31UL   // "GDM1" -- not the XIAO's "UDM1"
#define CFG_VERSION       1

#define R_CAL_BUCKETS     15      // the CF_A..CF_O piecewise ladder
#define VOLT_SAMPLE_MAX   100     // buffer size; cfg.vSamples is the live count
#define SN_MAX_LEN        16

// ==================================================================
//  CONFIG
// ==================================================================
// Layout changes REQUIRE bumping CFG_VERSION -- configLoad() rejects a stored
// image whose version differs.  New fields go at the END, immediately before
// crc, so a future migration can memcpy the older layout as a prefix.
//
// The CRC is the LAST field.  Checksum it with offsetof(Config, crc), never
// sizeof(Config)-2: tail padding follows crc in a 4-byte-aligned struct.
struct Config {
  uint32_t magic;
  uint16_t version;

  // ---- Unit ---------------------------------------------------
  uint8_t  hwRev;          // which GigaMeter this is; 0 = never assigned
  uint8_t  bridge;         // has the float-detect bridge MOSFET circuit

  // ---- Resistance calibration ---------------------------------
  float    rCal[R_CAL_BUCKETS];  // piecewise correction, was CF_A..CF_O
  float    constantI;      // A, low-range constant-current source
  float    constantR;      // Ohm, internal resistor in that source
  float    dividerR;       // Ohm, series resistor for the high-range divider
  float    zenerMaxV;      // V, ohms rail with the leads open (was EEPROM_MAXV)
  float    sleepV;         // V, ohms rail while parked (was EEPROM_SleepV)

  // ---- Voltage calibration ------------------------------------
  float    voltScale;      // front-end scale, positive input
  float    voltScaleNeg;   // front-end scale, negative input
  float    voltOffset;     // V subtracted from every reading (was giga_absfactor)
  float    altMult;        // x50 applied in AltUnitsMode
  float    thermR0;        // thermistor nominal resistance at 25 C
  float    thermB;         // thermistor beta

  // ---- Current ------------------------------------------------
  float    iShunt;         // V per A of the hall sensor (high range)
  float    iZero;          // baseline; overwritten at boot when iAutoZero
  uint8_t  iAutoZero;
  float    iNoiseHi;       // deadband applied in high range
  float    iNoiseLo;       // deadband applied in low range
  uint16_t iDetCount;      // |counts| below this = grounded shunt (low range)
  float    iDetLo;         // mid-rail window = hall sensor (high range)
  float    iDetHi;
  uint8_t  iDetSamples;    // reads averaged when detecting
  uint16_t iDetSettleMs;   // settling delay before that burst
  float    iDetPPV;        // max peak-to-peak for a driven input; 0 disables
  float    iShuntR;        // Ohm, low-range sense resistor

  // ---- Ranging ------------------------------------------------
  float    rangeThreshR;
  float    rangeDeadband;
  uint16_t adcCountLow;
  uint16_t adcCountHigh;
  float    openMargin;     // V below zenerMaxV that still counts as open
  float    openR;          // Ohm reported for an open circuit
  uint16_t vDispCount;     // |differential counts| that auto-selects voltage
  float    rDispMin;       // Ohm window that auto-selects resistance
  float    rDispMax;
  float    zeroAutoMax;    // Ohm, largest boot reading accepted as a lead null

  // ---- ADC data-rate scheduling -------------------------------
  float    rateJump;
  float    rateBumpV;
  float    rateSlowV;
  float    ratePrecR;
  float    rateStable;
  float    mmRMax;
  float    mmRMin;

  // ---- Power save ---------------------------------------------
  uint16_t psHoldMs;
  float    psMargin;
  float    psHyst;
  float    psCancelR;
  uint8_t  psPwm;
  uint16_t sleepSec;
  float    sleepVMax;
  uint8_t  psDebug;

  // ---- Timing -------------------------------------------------
  uint16_t adcMs;
  uint16_t lcdMs;
  uint16_t lcdFastMs;
  uint16_t lcdBumpMs;
  uint16_t streamMs;

  // ---- Alerts -------------------------------------------------
  uint8_t  alertsOn;
  float    contMin;
  float    contMax;
  float    contMaxHi;
  float    vAlert;
  float    vAlertAlt;
  float    vacAlert;
  float    vacAlertAlt;
  uint8_t  beepBright;
  uint8_t  beepHold;
  uint8_t  blinkLimit;
  uint16_t alertPerMs;
  uint16_t alertOnMs;
  uint16_t alertP2OnMs;
  uint16_t alertP2OffMs;

  // ---- AC detection -------------------------------------------
  float    vacThresh;
  float    vacThreshAlt;
  float    vacAvgMax;

  // ---- Float / closed bridge (Giga circuit) -------------------
  float    bridgeClsV;     // |bridge V| below this = closed
  float    bridgeOpenV;    // |bridge V| above this = floating; between = unsure
  float    bridgeAvgMax;   // gate: |DC average| must be under this
  float    bridgeVMax;     // gate: |instantaneous| must be under this

  // ---- Filtering / input --------------------------------------
  uint8_t  vSamples;
  float    smoothAlpha;
  uint8_t  keyboardEn;     // USB HID typing in Type mode
  uint16_t btnLongMs;      // TYPE_PIN hold that resets min/max
  uint16_t btnShortMs;     // debounce floor for a short press

  // ---- Mic-jack button ladder (A7) ----------------------------
  uint8_t  micEn;          // 0 ignores A7 entirely
  uint16_t micRefMax;      // below = Ref
  uint16_t micMMLo;        // band = min/max reset
  uint16_t micMMHi;
  uint16_t micModeLo;      // band = next mode
  uint16_t micModeHi;

  // ---- M4 audio -----------------------------------------------
  float    humMin;         // Ohm window that sets the RPC hum flag
  float    humMax;

  uint16_t crc;            // MUST stay last
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
//  PER-UNIT CALIBRATION SETS
// ==================================================================
// The compiled-in starting points for each GigaMeter.  Applied with
// !SEEDCAL,<n>; after that the values are live config keys.
struct LegacyCal {
  uint8_t id;
  float   cf[R_CAL_BUCKETS];
  float   constantI;
  float   zenerMaxV;
  float   sleepV;
  float   voltScale;
  float   voltScaleNeg;
};

// ==================================================================
//  UNIT SERIAL NUMBER
// ==================================================================
// Its own flash block so it survives !DEFAULTS and any CFG_VERSION bump.
struct SerialId {
  uint32_t magic;
  char     sn[SN_MAX_LEN];
  uint16_t crc;
};
#define SN_MAGIC 0x474D5F53UL    // "GM_S"

// ==================================================================
//  HOST COMMAND INPUT
// ==================================================================
// One line assembler per transport (USB serial, BLE UART), so bytes from one
// can never be spliced into a command arriving on the other.
#define CMD_BUF_LEN 64
struct LineIn {
  char    buf[CMD_BUF_LEN];
  uint8_t len;
};

// ==================================================================
//  CROSS-TAB DECLARATIONS
// ==================================================================
extern Config            cfg;
extern bool              cfgDirty;
extern bool              cfgStored;
extern char              unitSN[SN_MAX_LEN];
extern const ConfigField CFG_FIELDS[];
extern const int         CFG_FIELD_COUNT;
extern const float       R_CAL_EDGES[R_CAL_BUCKETS - 1];

#endif // DMM_TYPES_H
