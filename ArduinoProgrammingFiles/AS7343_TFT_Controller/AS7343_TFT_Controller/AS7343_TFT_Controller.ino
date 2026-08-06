/*
  AS7343_TFT_Controller.ino
  Adafruit AS7343 14-channel spectral sensor bench controller.
  Target: Adafruit Feather RP2040 + Adafruit 2.4" TFT FeatherWing v2 (#3315).

  Purpose: expose every AS7343 "lever" (gain, ATIME, ASTEP, WTIME, SMUX mode,
  LED drive, auto-zero, thresholds, channel selection) over a serial protocol so
  the companion Python GUI can drive them, and run timed burst captures so you can
  measure the real achievable sample rate for a given configuration and plot the
  resulting per-channel traces.

  The touchscreen carries a deliberately reduced control set (gain / ATIME / ASTEP /
  SMUX / LED / capture) plus a live spectrum bar chart.  The Python GUI is the full
  control surface.

  ── Libraries required ──────────────────────────────────────────────────────────
  Adafruit AS7343          (spectral sensor)
  Adafruit ILI9341         (TFT driver)
  Adafruit TSC2007         (I2C resistive touchscreen — v2 FeatherWing)
  Adafruit GFX Library     (graphics primitives)
  Adafruit NeoPixel        (status LED)
  EEPROM                   (bundled with the rp2040 core — flash-backed emulation)

  ── Pin assignments (Feather RP2040 + 2.4" TFT FeatherWing v2) ─────────────────
  TFT CS    →  9   (FeatherWing hardwired)
  TFT DC    → 10   (FeatherWing hardwired)
  TS IRQ    →  6   (TSC2007 touch interrupt, FeatherWing hardwired)
  SD CS     →  5   (FeatherWing microSD — unused by this sketch)
  NeoPixel  → 16   (Feather RP2040 onboard NeoPixel)
  I2C       → SDA=GPIO2, SCL=GPIO3
  AS7343    → I2C 0x39
  TSC2007   → I2C 0x48 (default — no conflict with the AS7343, so no jumpers needed)

  ── Channel map (auto-SMUX readout order, index 0..17) ─────────────────────────
   0 FZ  450nm      6 F2  425nm     12 F1  405nm
   1 FY  555nm      7 F3  475nm     13 F7  690nm
   2 FXL 600nm      8 F4  515nm     14 F8  745nm
   3 NIR 855nm      9 F6  640nm     15 F5  550nm
   4 VTL0 (clear)  10 VTL1 (clear)  16 VTL2 (clear)
   5 VBR0 (clear)  11 VBR1 (clear)  17 VBR2 (clear)

  SMUX mode 6 reads indices 0-5, mode 12 reads 0-11, mode 18 reads 0-17.  Fewer
  channels = fewer SMUX cycles = a faster measurement, which is the main hardware
  lever on throughput.  `chmask` selects which of those are stored/streamed; it
  does not change sensor timing (the AS7343 auto-SMUX sets are fixed).

  ── Commands (host → device) ────────────────────────────────────────────────────
  !PING                Identify        → $HELLO,AS7343_TFT,<fw>
  !ID                  Silicon IDs     → $ID,<part>,<rev>,<aux>
  !CFG                 Dump config     → $CFG,<key>,<val>,<min>,<max> … $CFGEND
  !GET,<key>           Read one key    → $VAL,<key>,<val>
  !SET,<key>,<val>     Write one key   → $OK,<key>,<val>  |  $ERR,<key>,<reason>
  !READ                One measurement → $R,<t_ms>,<asat>,<dsat>,<v…>
  !STREAM,0|1          Continuous $R lines every `streamms`
  !CAP[,<ms>]          Timed burst capture (see below)
  !LED,0|1             Shorthand for !SET,led
  !FLICKER             One-shot flicker detect → $FLK,<status>,<freq_hz>
  !SAVE                Persist config to flash-backed EEPROM
  !LOAD                Reload config from EEPROM
  !DEFAULTS            Restore compiled-in defaults (not yet saved)

  ── Capture reply format ────────────────────────────────────────────────────────
  $CAPB,<n>,<nchan>,<mask_hex>,<elapsed_us>,<rate_hz>,<mode>,<gain>,<atime>,<astep>,<smux>,<tint_ms>
  $CH,<name0>,<name1>,…
  $S,<t_us>,<v0>,<v1>,…            (one line per sample, t_us relative to capture start)
  $CAPE,<n>

  Lines beginning with '#' are human-readable debug and can be ignored by the GUI.
*/

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_TSC2007.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_AS7343.h>

#define FW_VERSION "1.0"

// ─── Display / touch pins (FeatherWing v2 hardwired) ────────────────────────
#define TFT_CS    9
#define TFT_DC   10
#define TS_IRQ    6
#define PIXEL_PIN 16

#define TS_I2C_ADDR 0x48   // TSC2007 default; AS7343 is 0x39 so no collision

// ─── Touch calibration (landscape rotation 1) ───────────────────────────────
// Same panel family as DataLogger_TFT — see that sketch for the corner-probe
// procedure if your unit needs different numbers.
#define TS_RAW_Y_LEFT    578
#define TS_RAW_Y_RIGHT  3320
#define TS_RAW_X_TOP    3636
#define TS_RAW_X_BOT     492
#define TS_MIN_Z          10
#define TOUCH_DEBOUNCE_MS 180

// ─── Uncomment to print raw + mapped touch coords for calibration ────────────
// #define TOUCH_DEBUG

// ─── Display geometry (landscape 320×240) ────────────────────────────────────
#define DISP_W   320
#define DISP_H   240
#define HDR_H     28
#define TAB_H     28
#define CONTENT_Y HDR_H
#define CONTENT_H (DISP_H - HDR_H - TAB_H)

// ─── Colour palette (RGB565) ─────────────────────────────────────────────────
#define COL_BG       0x1082
#define COL_HDR      0x2124
#define COL_GREEN    0x07E0
#define COL_RED      0xF800
#define COL_YELLOW   0xFFE0
#define COL_ORANGE   0xFC00
#define COL_WHITE    0xFFFF
#define COL_LTGRAY   0xC618
#define COL_DKGRAY   0x4208
#define COL_CYAN     0x07FF
#define COL_BTN_ON   0x0440
#define COL_BTN_OFF  0x2945

// ─── Pages ───────────────────────────────────────────────────────────────────
#define PAGE_SPECTRUM 0
#define PAGE_CONTROL  1

// ═══════════════════════════════════════════════════════════════════════════════
//  TYPES
//  Declared ahead of any function so the Arduino preprocessor's auto-generated
//  prototypes — which it inserts just before the first function definition —
//  can name them.
// ═══════════════════════════════════════════════════════════════════════════════

struct ChanInfo {
  const char *name;
  uint16_t    nm;      // 0 = broadband clear/VIS photodiode, no single centre
  uint16_t    color;   // RGB565 approximation of the band
};

struct Config {
  uint32_t magic;
  uint16_t version;
  uint8_t  gain;       // as7343_gain_t index 0..12  (0.5x .. 2048x)
  uint8_t  atime;      // 0..255
  uint16_t astep;      // 0..65534
  uint8_t  wtime;      // 0..255
  uint8_t  waiten;     // 0/1
  uint8_t  smux;       // 6 / 12 / 18
  uint8_t  led;        // 0/1
  uint16_t ledma;      // 4..258
  uint8_t  az;         // auto-zero frequency 0..255
  uint32_t chmask;     // 18-bit store/stream mask
  uint16_t streamms;   // stream interval
  uint16_t capms;      // capture window
  uint8_t  capmode;    // 0 = restart-per-sample, 1 = free-run
  uint16_t thlow;
  uint16_t thhigh;
  uint8_t  thch;       // threshold source channel 0..17
  uint8_t  pers;       // 0..15
  uint8_t  spint;      // spectral interrupt enable
  uint32_t checksum;
};

// Key table entry — drives !CFG / !GET / !SET and lets the GUI self-describe.
enum CfgType : uint8_t { CT_U8, CT_U16, CT_U32 };

struct CfgItem {
  const char *key;
  CfgType     type;
  void       *ptr;
  uint32_t    minV;
  uint32_t    maxV;
};

struct CtrlRow { const char *label; };

// ═══════════════════════════════════════════════════════════════════════════════
//  AS7343 RAW REGISTER ACCESS
//  The Adafruit library is used for configuration, but the capture inner loop
//  talks to the chip directly so the per-sample overhead is a couple of short
//  I2C transactions instead of a stack of BusIO register objects.
// ═══════════════════════════════════════════════════════════════════════════════

#define AS_ADDR        0x39
#define AS_REG_ENABLE  0x80
#define AS_REG_STATUS2 0x90
#define AS_REG_STATUS  0x93
#define AS_REG_ASTATUS 0x94   // reading this latches the 36 spectral data bytes
#define AS_REG_CFG0    0xBF
#define AS_BIT_SP_EN   0x02   // ENABLE bit 1
#define AS_BIT_AVALID  0x40   // STATUS2 bit 6

static bool asWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AS_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool asRead(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(AS_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((uint8_t)AS_ADDR, (uint8_t)len);
  if (got != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static uint8_t asRead8(uint8_t reg) {
  uint8_t v = 0;
  asRead(reg, &v, 1);
  return v;
}

// Registers 0x80+ are only visible with REG_BANK=0 (CFG0 bit 4).  The library
// leaves the bank at 0, but make it explicit before any raw access run.
static void asSelectBank0() {
  uint8_t cfg0 = asRead8(AS_REG_CFG0);
  if (cfg0 & 0x10) asWrite8(AS_REG_CFG0, cfg0 & ~0x10);
}

static void asSpectralEnable(bool en) {
  uint8_t e = asRead8(AS_REG_ENABLE);
  asWrite8(AS_REG_ENABLE, en ? (e | AS_BIT_SP_EN) : (e & ~AS_BIT_SP_EN));
}

// STATUS is write-1-to-clear: read it, write the same value back.
static void asClearStatus() {
  asWrite8(AS_REG_STATUS, asRead8(AS_REG_STATUS));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CHANNEL TABLE
// ═══════════════════════════════════════════════════════════════════════════════

static const ChanInfo kChan[18] = {
  { "FZ",   450, 0x001F }, { "FY",   555, 0xC7E0 }, { "FXL",  600, 0xFC00 },
  { "NIR",  855, 0x4208 }, { "VTL0",   0, 0xC618 }, { "VBR0",   0, 0xC618 },
  { "F2",   425, 0x401F }, { "F3",   475, 0x03FF }, { "F4",   515, 0x07E0 },
  { "F6",   640, 0xF800 }, { "VTL1",   0, 0xC618 }, { "VBR1",   0, 0xC618 },
  { "F1",   405, 0x801F }, { "F7",   690, 0xC000 }, { "F8",   745, 0x8000 },
  { "F5",   550, 0x87E0 }, { "VTL2",   0, 0xC618 }, { "VBR2",   0, 0xC618 },
};

// The 12 true spectral channels in wavelength order — used for the TFT bar chart.
static const uint8_t kSpectrumOrder[12] = { 12, 6, 0, 7, 8, 15, 1, 2, 9, 13, 14, 3 };

// ═══════════════════════════════════════════════════════════════════════════════
//  CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════════

#define CFG_MAGIC   0x41537343UL   // 'AS7343'-ish
#define CFG_VERSION 1

static Config cfg;

static void loadDefaults() {
  cfg.magic    = CFG_MAGIC;
  cfg.version  = CFG_VERSION;
  cfg.gain     = AS7343_GAIN_256X;
  cfg.atime    = 29;
  cfg.astep    = 599;      // ≈50 ms, the library's own default
  cfg.wtime    = 0;
  cfg.waiten   = 0;
  cfg.smux     = 18;
  cfg.led      = 0;
  cfg.ledma    = 12;
  cfg.az       = 255;
  cfg.chmask   = 0x3FFFFUL;
  cfg.streamms = 250;
  cfg.capms    = 2000;
  cfg.capmode  = 0;
  cfg.thlow    = 0;
  cfg.thhigh   = 0xFFFF;
  cfg.thch     = 0;
  cfg.pers     = 0;
  cfg.spint    = 0;
  cfg.checksum = 0;
}

// ─── Key table — drives !CFG / !GET / !SET and lets the GUI self-describe ─────
static const CfgItem kCfgItems[] = {
  { "gain",     CT_U8,  &cfg.gain,     0, 12      },
  { "atime",    CT_U8,  &cfg.atime,    0, 255     },
  { "astep",    CT_U16, &cfg.astep,    0, 65534   },
  { "wtime",    CT_U8,  &cfg.wtime,    0, 255     },
  { "waiten",   CT_U8,  &cfg.waiten,   0, 1       },
  { "smux",     CT_U8,  &cfg.smux,     6, 18      },
  { "led",      CT_U8,  &cfg.led,      0, 1       },
  { "ledma",    CT_U16, &cfg.ledma,    4, 258     },
  { "az",       CT_U8,  &cfg.az,       0, 255     },
  { "chmask",   CT_U32, &cfg.chmask,   0, 0x3FFFF },
  { "streamms", CT_U16, &cfg.streamms, 5, 60000   },
  { "capms",    CT_U16, &cfg.capms,    10, 60000  },
  { "capmode",  CT_U8,  &cfg.capmode,  0, 1       },
  { "thlow",    CT_U16, &cfg.thlow,    0, 65535   },
  { "thhigh",   CT_U16, &cfg.thhigh,   0, 65535   },
  { "thch",     CT_U8,  &cfg.thch,     0, 17      },
  { "pers",     CT_U8,  &cfg.pers,     0, 15      },
  { "spint",    CT_U8,  &cfg.spint,    0, 1       },
};
static const uint8_t kNumCfgItems = sizeof(kCfgItems) / sizeof(kCfgItems[0]);

static uint32_t cfgItemGet(const CfgItem &it) {
  switch (it.type) {
    case CT_U8:  return *(uint8_t  *)it.ptr;
    case CT_U16: return *(uint16_t *)it.ptr;
    default:     return *(uint32_t *)it.ptr;
  }
}

static void cfgItemSet(const CfgItem &it, uint32_t v) {
  switch (it.type) {
    case CT_U8:  *(uint8_t  *)it.ptr = (uint8_t)v;  break;
    case CT_U16: *(uint16_t *)it.ptr = (uint16_t)v; break;
    default:     *(uint32_t *)it.ptr = v;           break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GLOBAL OBJECTS AND STATE
// ═══════════════════════════════════════════════════════════════════════════════

Adafruit_AS7343   as7343;
Adafruit_ILI9341  tft(TFT_CS, TFT_DC);
Adafruit_TSC2007  ts;
Adafruit_NeoPixel pixel(1, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

static bool sensorOK = false;
static bool tsOK     = false;

// Live reading state
static uint16_t liveVals[18] = { 0 };
static bool     liveASat = false, liveDSat = false;
static bool     liveValid = false;
static uint8_t  liveState = 0;        // 0 = idle, 1 = integrating
static uint32_t liveStartMs = 0;
static bool     readPending = false;  // a !READ is waiting on the next sample

// Enabled-channel list, rebuilt whenever smux or chmask changes
static uint8_t enIdx[18];
static uint8_t enCount = 0;

// Capture buffers.  ~60 KB of the RP2040's 264 KB SRAM.
#define CAP_MAX_WORDS   24000
#define CAP_MAX_SAMPLES  3000
static uint16_t capData[CAP_MAX_WORDS];
static uint32_t capTime[CAP_MAX_SAMPLES];
static uint16_t capCount = 0;

// Last capture summary (also shown on the TFT)
static float    lastCapRate    = 0.0f;
static uint16_t lastCapSamples = 0;
static uint32_t lastCapElapsed = 0;

static bool          streaming    = false;
static unsigned long lastStreamMs = 0;
static unsigned long lastLiveMs   = 0;
static unsigned long lastHostMs   = 0;   // for the "USB" activity badge

// UI state
static uint8_t currentPage    = PAGE_SPECTRUM;
static bool    needFullRedraw = true;
static bool    liveHold       = false;   // pause background reads
static uint8_t ctrlSel        = 0;       // highlighted row on the CONTROL page

static char    serialBuf[96];
static uint8_t serialLen = 0;

// ═══════════════════════════════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

static const float kGainX[13] = {
  0.5f, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048
};

static uint8_t chansForSmux(uint8_t smux) {
  if (smux <= 6)  return 6;
  if (smux <= 12) return 12;
  return 18;
}

static as7343_smux_mode_t smuxEnum(uint8_t smux) {
  if (smux <= 6)  return AS7343_SMUX_6CH;
  if (smux <= 12) return AS7343_SMUX_12CH;
  return AS7343_SMUX_18CH;
}

// Per-cycle integration time in ms.  Note the AS7343 runs one such integration
// per auto-SMUX cycle, so a full 18-channel measurement takes longer than this —
// the capture rate reported by !CAP is the number to trust.
static float integrationMs() {
  return (float)(cfg.atime + 1) * (float)(cfg.astep + 1) * 0.00278f;
}

static void rebuildEnabled() {
  uint8_t n = chansForSmux(cfg.smux);
  enCount = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (cfg.chmask & (1UL << i)) enIdx[enCount++] = i;
  }
  // Never leave the capture path with nothing to record.
  if (enCount == 0) {
    enIdx[0] = 0;
    enCount  = 1;
  }
}

static void applyConfig() {
  if (!sensorOK) return;
  as7343.setGain((as7343_gain_t)cfg.gain);
  as7343.setATIME(cfg.atime);
  as7343.setASTEP(cfg.astep);
  as7343.setSMUXMode(smuxEnum(cfg.smux));
  as7343.setWaitTime(cfg.wtime);
  as7343.enableWait(cfg.waiten != 0);
  as7343.setLEDCurrent(cfg.ledma);
  as7343.enableLED(cfg.led != 0);
  as7343.setAutoZeroFrequency(cfg.az);
  as7343.setLowThreshold(cfg.thlow);
  as7343.setHighThreshold(cfg.thhigh);
  as7343.setThresholdChannel(cfg.thch);
  as7343.setPersistence(cfg.pers);
  as7343.enableSpectralInterrupt(cfg.spint != 0);
  rebuildEnabled();
}

static uint32_t cfgChecksum() {
  const uint8_t *p = (const uint8_t *)&cfg;
  size_t n = sizeof(Config) - sizeof(uint32_t);   // everything but the checksum
  uint32_t sum = 2166136261UL;                     // FNV-1a
  for (size_t i = 0; i < n; i++) { sum ^= p[i]; sum *= 16777619UL; }
  return sum;
}

static void saveConfig() {
  cfg.magic    = CFG_MAGIC;
  cfg.version  = CFG_VERSION;
  cfg.checksum = cfgChecksum();
  EEPROM.put(0, cfg);
  EEPROM.commit();
}

static bool loadConfig() {
  Config tmp;
  EEPROM.get(0, tmp);
  if (tmp.magic != CFG_MAGIC || tmp.version != CFG_VERSION) return false;
  Config keep = cfg;
  cfg = tmp;
  if (cfgChecksum() != tmp.checksum) { cfg = keep; return false; }
  return true;
}

static void blinkPixel(uint8_t r, uint8_t g, uint8_t b, int dur = 4) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
  delay(dur);
  pixel.clear();
  pixel.show();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MEASUREMENT
// ═══════════════════════════════════════════════════════════════════════════════

// Live measurement runs as a non-blocking state machine so the touch handler and
// serial parser stay responsive no matter how long an integration takes — with
// ATIME/ASTEP at their limits a single conversion is tens of seconds.
//
//   state 0  idle — ready to kick off the next measurement
//   state 1  integrating — poll AVALID, read when it asserts
//
// Each measurement is stop / clear / start, so every sample is a fresh
// integration.  ASTATUS and the data registers are read in one burst, which
// latches the whole set concurrently.

static void liveKickoff() {
  asSelectBank0();
  asSpectralEnable(false);
  asClearStatus();
  asSpectralEnable(true);
  liveStartMs = millis();
  liveState   = 1;
}

// Returns true on the loop pass where a fresh sample lands in liveVals.
static bool liveService() {
  if (!sensorOK || liveState != 1) return false;

  if (!(asRead8(AS_REG_STATUS2) & AS_BIT_AVALID)) {
    uint32_t limit = (uint32_t)(integrationMs() * 6.0f) + 2000;
    if (millis() - liveStartMs > limit) {   // sensor wedged or unplugged
      asSpectralEnable(false);
      liveState = 0;
    }
    return false;
  }

  uint8_t nch = chansForSmux(cfg.smux);
  uint8_t buf[1 + 36];
  if (!asRead(AS_REG_ASTATUS, buf, 1 + 2 * nch)) {
    asSpectralEnable(false);
    liveState = 0;
    return false;
  }
  asSpectralEnable(false);
  liveState = 0;

  uint8_t st2 = asRead8(AS_REG_STATUS2);
  liveASat = (st2 & 0x08) != 0;
  liveDSat = (st2 & 0x10) != 0;

  for (uint8_t i = 0; i < nch; i++) {
    liveVals[i] = (uint16_t)buf[1 + 2 * i] | ((uint16_t)buf[2 + 2 * i] << 8);
  }
  liveValid = true;
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CAPTURE (speed test)
// ═══════════════════════════════════════════════════════════════════════════════

// Fill capData/capTime for `windowMs`, as fast as the configured mode allows.
//
//   mode 0 "restart"  — stop / clear / start / poll / read per sample.  Fully
//                       deterministic: every sample is a fresh integration with
//                       no carry-over, at the cost of restart dead-time.
//   mode 1 "freerun"  — SP_EN stays asserted so the sensor integrates back to
//                       back.  A floor of 90 % of the per-cycle integration time
//                       keeps the loop from re-reading the same conversion, then
//                       AVALID gates the actual read.
//
// Returns the number of samples stored.
static uint16_t runCapture(uint16_t windowMs) {
  rebuildEnabled();
  liveState = 0;   // capture takes the sensor over; abandon any in-flight live read

  uint8_t nch = chansForSmux(cfg.smux);
  uint16_t maxByWords = CAP_MAX_WORDS / enCount;
  uint16_t maxSamples = maxByWords < CAP_MAX_SAMPLES ? maxByWords : CAP_MAX_SAMPLES;

  uint8_t buf[1 + 36];
  capCount = 0;

  asSelectBank0();
  asSpectralEnable(false);
  asClearStatus();

  uint32_t floorUs = 0;
  if (cfg.capmode == 1) {
    floorUs = (uint32_t)(integrationMs() * 900.0f);   // 0.9 × integration, in µs
    asSpectralEnable(true);
  }

  uint32_t t0        = micros();
  uint32_t windowUs  = (uint32_t)windowMs * 1000UL;
  uint32_t lastStamp = 0;
  bool     haveLast  = false;

  while (capCount < maxSamples) {
    uint32_t now = micros();
    if (now - t0 >= windowUs) break;

    if (cfg.capmode == 0) {
      asSpectralEnable(false);
      asClearStatus();
      asSpectralEnable(true);
    } else if (haveLast) {
      while ((uint32_t)(micros() - lastStamp) < floorUs) {
        if ((uint32_t)(micros() - t0) >= windowUs) break;
      }
    }

    // Wait for AVALID, bounded by whatever is left of the capture window.
    bool ready = false;
    while ((uint32_t)(micros() - t0) < windowUs) {
      if (asRead8(AS_REG_STATUS2) & AS_BIT_AVALID) { ready = true; break; }
    }
    if (!ready) break;

    uint32_t stamp = micros();
    if (!asRead(AS_REG_ASTATUS, buf, 1 + 2 * nch)) break;

    uint16_t *dst = &capData[(uint32_t)capCount * enCount];
    for (uint8_t k = 0; k < enCount; k++) {
      uint8_t i = enIdx[k];
      dst[k] = (uint16_t)buf[1 + 2 * i] | ((uint16_t)buf[2 + 2 * i] << 8);
    }
    capTime[capCount] = stamp - t0;
    capCount++;
    lastStamp = stamp;
    haveLast  = true;
  }

  uint32_t elapsed = micros() - t0;
  asSpectralEnable(false);

  lastCapSamples = capCount;
  lastCapElapsed = elapsed;
  lastCapRate    = elapsed ? (float)capCount * 1000000.0f / (float)elapsed : 0.0f;

  // Mirror the final sample into the live display so the bar chart stays current.
  if (capCount) {
    uint16_t *last = &capData[(uint32_t)(capCount - 1) * enCount];
    for (uint8_t k = 0; k < enCount; k++) liveVals[enIdx[k]] = last[k];
    liveValid = true;
  }
  return capCount;
}

static void sendCapture() {
  Serial.print(F("$CAPB,"));
  Serial.print(capCount);                 Serial.print(',');
  Serial.print(enCount);                  Serial.print(',');
  Serial.print(cfg.chmask, HEX);          Serial.print(',');
  Serial.print(lastCapElapsed);           Serial.print(',');
  Serial.print(lastCapRate, 2);           Serial.print(',');
  Serial.print(cfg.capmode);              Serial.print(',');
  Serial.print(cfg.gain);                 Serial.print(',');
  Serial.print(cfg.atime);                Serial.print(',');
  Serial.print(cfg.astep);                Serial.print(',');
  Serial.print(cfg.smux);                 Serial.print(',');
  Serial.println(integrationMs(), 4);

  Serial.print(F("$CH"));
  for (uint8_t k = 0; k < enCount; k++) {
    Serial.print(',');
    Serial.print(kChan[enIdx[k]].name);
  }
  Serial.println();

  for (uint16_t s = 0; s < capCount; s++) {
    Serial.print(F("$S,"));
    Serial.print(capTime[s]);
    uint16_t *row = &capData[(uint32_t)s * enCount];
    for (uint8_t k = 0; k < enCount; k++) { Serial.print(','); Serial.print(row[k]); }
    Serial.println();
  }

  Serial.print(F("$CAPE,"));
  Serial.println(capCount);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SERIAL PROTOCOL
// ═══════════════════════════════════════════════════════════════════════════════

static void sendReading() {
  Serial.print(F("$R,"));
  Serial.print(millis());              Serial.print(',');
  Serial.print(liveASat ? 1 : 0);      Serial.print(',');
  Serial.print(liveDSat ? 1 : 0);
  for (uint8_t k = 0; k < enCount; k++) {
    Serial.print(',');
    Serial.print(liveVals[enIdx[k]]);
  }
  Serial.println();
}

static void sendChannelHeader() {
  Serial.print(F("$CH"));
  for (uint8_t k = 0; k < enCount; k++) {
    Serial.print(',');
    Serial.print(kChan[enIdx[k]].name);
  }
  Serial.println();
}

static void sendConfig() {
  for (uint8_t i = 0; i < kNumCfgItems; i++) {
    Serial.print(F("$CFG,"));
    Serial.print(kCfgItems[i].key);   Serial.print(',');
    Serial.print(cfgItemGet(kCfgItems[i])); Serial.print(',');
    Serial.print(kCfgItems[i].minV);  Serial.print(',');
    Serial.println(kCfgItems[i].maxV);
  }
  Serial.print(F("$CFG,tint_ms,"));   Serial.print(integrationMs(), 4);
  Serial.println(F(",0,0"));
  Serial.print(F("$CFG,nchan,"));     Serial.print(enCount);
  Serial.println(F(",0,0"));
  sendChannelHeader();
  Serial.println(F("$CFGEND"));
}

static const CfgItem *findCfgItem(const char *key) {
  for (uint8_t i = 0; i < kNumCfgItems; i++) {
    if (strcasecmp(key, kCfgItems[i].key) == 0) return &kCfgItems[i];
  }
  return nullptr;
}

static void doFlicker() {
  as7343.enableFlickerDetection(true);
  delay(600);
  uint8_t status = as7343.getFlickerStatus();
  as7343_flicker_t f = as7343.getFlickerFrequency();
  as7343.enableFlickerDetection(false);
  Serial.print(F("$FLK,"));
  Serial.print(status);
  Serial.print(',');
  Serial.println((int)f);
  applyConfig();   // flicker mode disturbs the spectral engine — restore settings
}

static void handleCommand(char *line) {
  lastHostMs = millis();

  if (line[0] != '!') return;
  char *cmd = line + 1;
  char *arg = strchr(cmd, ',');
  if (arg) { *arg = '\0'; arg++; }

  if (strcasecmp(cmd, "PING") == 0) {
    Serial.print(F("$HELLO,AS7343_TFT,"));
    Serial.println(F(FW_VERSION));

  } else if (strcasecmp(cmd, "ID") == 0) {
    Serial.print(F("$ID,"));
    Serial.print(as7343.getPartID());     Serial.print(',');
    Serial.print(as7343.getRevisionID()); Serial.print(',');
    Serial.println(as7343.getAuxID());

  } else if (strcasecmp(cmd, "CFG") == 0) {
    sendConfig();

  } else if (strcasecmp(cmd, "GET") == 0 && arg) {
    const CfgItem *it = findCfgItem(arg);
    if (!it) { Serial.print(F("$ERR,")); Serial.print(arg); Serial.println(F(",nokey")); }
    else {
      Serial.print(F("$VAL,")); Serial.print(it->key); Serial.print(',');
      Serial.println(cfgItemGet(*it));
    }

  } else if (strcasecmp(cmd, "SET") == 0 && arg) {
    char *val = strchr(arg, ',');
    if (!val) { Serial.println(F("$ERR,set,noval")); return; }
    *val = '\0'; val++;
    const CfgItem *it = findCfgItem(arg);
    if (!it) { Serial.print(F("$ERR,")); Serial.print(arg); Serial.println(F(",nokey")); return; }
    uint32_t v = strtoul(val, nullptr, 0);
    if (v < it->minV || v > it->maxV) {
      Serial.print(F("$ERR,")); Serial.print(it->key); Serial.println(F(",range"));
      return;
    }
    if (strcasecmp(it->key, "smux") == 0) v = chansForSmux((uint8_t)v);   // snap to 6/12/18
    cfgItemSet(*it, v);
    applyConfig();
    Serial.print(F("$OK,")); Serial.print(it->key); Serial.print(',');
    Serial.println(cfgItemGet(*it));
    needFullRedraw = true;

  } else if (strcasecmp(cmd, "READ") == 0) {
    // Answered from the live state machine as soon as the next sample lands, so
    // a long integration never blocks the command parser.
    readPending = true;

  } else if (strcasecmp(cmd, "STREAM") == 0) {
    streaming = arg && atoi(arg) != 0;
    Serial.print(F("$OK,stream,")); Serial.println(streaming ? 1 : 0);

  } else if (strcasecmp(cmd, "CAP") == 0) {
    uint16_t win = cfg.capms;
    if (arg) {
      long v = strtol(arg, nullptr, 10);
      if (v >= 10 && v <= 60000) { win = (uint16_t)v; cfg.capms = win; }
    }
    bool wasStreaming = streaming;
    streaming = false;
    runCapture(win);
    sendCapture();
    streaming = wasStreaming;
    needFullRedraw = true;

  } else if (strcasecmp(cmd, "LED") == 0) {
    cfg.led = (arg && atoi(arg) != 0) ? 1 : 0;
    applyConfig();
    Serial.print(F("$OK,led,")); Serial.println(cfg.led);
    needFullRedraw = true;

  } else if (strcasecmp(cmd, "FLICKER") == 0) {
    doFlicker();

  } else if (strcasecmp(cmd, "SAVE") == 0) {
    saveConfig();
    Serial.println(F("$OK,save,1"));

  } else if (strcasecmp(cmd, "LOAD") == 0) {
    if (loadConfig()) { applyConfig(); Serial.println(F("$OK,load,1")); needFullRedraw = true; }
    else Serial.println(F("$ERR,load,nodata"));

  } else if (strcasecmp(cmd, "DEFAULTS") == 0) {
    loadDefaults();
    applyConfig();
    Serial.println(F("$OK,defaults,1"));
    needFullRedraw = true;

  } else {
    Serial.print(F("$ERR,")); Serial.print(cmd); Serial.println(F(",unknown"));
  }
}

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLen) {
        serialBuf[serialLen] = '\0';
        handleCommand(serialBuf);
        serialLen = 0;
      }
    } else if (serialLen < sizeof(serialBuf) - 1) {
      serialBuf[serialLen++] = c;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  DISPLAY
// ═══════════════════════════════════════════════════════════════════════════════

static void drawBtn(int16_t x, int16_t y, int16_t w, int16_t h,
                    const char *label, bool state, bool highlight = false) {
  uint16_t fill   = highlight ? 0x3200 : (state ? COL_BTN_ON : COL_BTN_OFF);
  uint16_t border = highlight ? COL_YELLOW : (state ? COL_GREEN : COL_DKGRAY);
  uint16_t text   = highlight ? COL_YELLOW : (state ? COL_GREEN : COL_LTGRAY);
  tft.fillRoundRect(x, y, w, h, 4, fill);
  tft.drawRoundRect(x, y, w, h, 4, border);
  tft.setTextColor(text);
  tft.setTextSize(1);
  tft.setCursor(x + (w - (int16_t)strlen(label) * 6) / 2, y + (h - 8) / 2);
  tft.print(label);
}

static void drawHeaderFrame() {
  tft.fillRect(0, 0, DISP_W, HDR_H, COL_HDR);
  tft.setTextColor(COL_WHITE, COL_HDR);
  tft.setTextSize(1);
  tft.setCursor(96, 4);  tft.print(F("AS7343 CONTROLLER"));
}

static void updateHeaderDynamics() {
  tft.setTextSize(1);
  tft.setTextColor(sensorOK ? COL_GREEN : COL_RED, COL_HDR);
  tft.setCursor(4, 4);
  tft.print(sensorOK ? F("SENS OK") : F("NO SENS"));

  tft.setCursor(4, 16);
  tft.setTextColor(COL_LTGRAY, COL_HDR);
  tft.print(F("G")); tft.print(kGainX[cfg.gain], (cfg.gain == 0) ? 1 : 0);
  tft.print(F("x  ")); tft.print(integrationMs(), 1); tft.print(F("ms  "));
  tft.print(cfg.smux); tft.print(F("ch    "));

  tft.setCursor(258, 4);
  if      (liveDSat) { tft.setTextColor(COL_RED,    COL_HDR); tft.print(F("DSAT  ")); }
  else if (liveASat) { tft.setTextColor(COL_ORANGE, COL_HDR); tft.print(F("ASAT  ")); }
  else               { tft.setTextColor(COL_DKGRAY, COL_HDR); tft.print(F("  OK  ")); }

  tft.setCursor(258, 16);
  bool host = (millis() - lastHostMs) < 3000;
  tft.setTextColor(host ? COL_CYAN : COL_DKGRAY, COL_HDR);
  tft.print(host ? F("USB   ") : F("      "));
}

static void drawTabBar() {
  tft.fillRect(0, DISP_H - TAB_H, DISP_W, TAB_H, COL_HDR);
  drawBtn(2,   DISP_H - TAB_H + 2, 154, TAB_H - 4, "SPECTRUM", currentPage == PAGE_SPECTRUM);
  drawBtn(162, DISP_H - TAB_H + 2, 156, TAB_H - 4, "CONTROL",  currentPage == PAGE_CONTROL);
}

// ─── SPECTRUM page ───────────────────────────────────────────────────────────
#define SP_BAR_TOP  (CONTENT_Y + 6)
#define SP_BAR_H    120
#define SP_BAR_W    22
#define SP_BAR_GAP  2
#define SP_BAR_X0   8

static void drawSpectrumFrame() {
  tft.fillRect(0, CONTENT_Y, DISP_W, CONTENT_H, COL_BG);
  // Channel labels under the bars
  tft.setTextSize(1);
  for (uint8_t k = 0; k < 12; k++) {
    int16_t x = SP_BAR_X0 + k * (SP_BAR_W + SP_BAR_GAP);
    const ChanInfo &ci = kChan[kSpectrumOrder[k]];
    tft.setTextColor(ci.color, COL_BG);
    tft.setCursor(x + (SP_BAR_W - (int16_t)strlen(ci.name) * 6) / 2, SP_BAR_TOP + SP_BAR_H + 3);
    tft.print(ci.name);
  }
  drawBtn(4,   CONTENT_Y + 150, 70, 22, "HOLD",  liveHold);
  drawBtn(80,  CONTENT_Y + 150, 70, 22, "LED",   cfg.led);
  drawBtn(156, CONTENT_Y + 150, 76, 22, "CAPTURE", false);
}

static void updateSpectrumDynamics() {
  uint16_t maxV = 1;
  for (uint8_t k = 0; k < 12; k++) {
    uint8_t i = kSpectrumOrder[k];
    if (i >= chansForSmux(cfg.smux)) continue;
    if (liveVals[i] > maxV) maxV = liveVals[i];
  }

  for (uint8_t k = 0; k < 12; k++) {
    uint8_t i = kSpectrumOrder[k];
    int16_t x = SP_BAR_X0 + k * (SP_BAR_W + SP_BAR_GAP);
    bool active = (i < chansForSmux(cfg.smux)) && (cfg.chmask & (1UL << i));
    uint16_t v = active ? liveVals[i] : 0;
    int16_t h  = (int16_t)((uint32_t)v * SP_BAR_H / maxV);
    if (h > SP_BAR_H) h = SP_BAR_H;
    tft.fillRect(x, SP_BAR_TOP, SP_BAR_W, SP_BAR_H - h, COL_BG);
    if (h > 0) tft.fillRect(x, SP_BAR_TOP + SP_BAR_H - h, SP_BAR_W, h, kChan[i].color);
    if (!active) tft.drawRect(x, SP_BAR_TOP, SP_BAR_W, SP_BAR_H, COL_DKGRAY);
  }

  tft.setTextSize(1);
  tft.setTextColor(COL_LTGRAY, COL_BG);
  tft.setCursor(240, CONTENT_Y + 152);
  tft.print(F("max ")); tft.print(maxV); tft.print(F("     "));
  tft.setCursor(240, CONTENT_Y + 164);
  if (lastCapSamples) { tft.print(lastCapRate, 1); tft.print(F("Hz    ")); }
  else                { tft.print(F("--       ")); }
}

// ─── CONTROL page ────────────────────────────────────────────────────────────
// Six ± rows: gain, ATIME, ASTEP, SMUX, LED mA, capture window.
#define CT_ROW_H    22
#define CT_ROW_Y0   (CONTENT_Y + 4)
#define CT_MINUS_X  118
#define CT_PLUS_X   232
#define CT_BTN_W     26
#define CT_BTN_H     20

static const CtrlRow kCtrlRows[6] = {
  { "GAIN"    }, { "ATIME"   }, { "ASTEP"   },
  { "SMUX ch" }, { "LED mA"  }, { "CAP ms"  },
};

static void ctrlRowValue(uint8_t row, char *out, size_t n) {
  switch (row) {
    case 0: snprintf(out, n, "%gx", (double)kGainX[cfg.gain]); break;
    case 1: snprintf(out, n, "%u",  cfg.atime);   break;
    case 2: snprintf(out, n, "%u",  cfg.astep);   break;
    case 3: snprintf(out, n, "%u",  cfg.smux);    break;
    case 4: snprintf(out, n, "%u",  cfg.ledma);   break;
    case 5: snprintf(out, n, "%u",  cfg.capms);   break;
  }
}

static void ctrlRowStep(uint8_t row, int dir) {
  switch (row) {
    case 0: {
      int g = (int)cfg.gain + dir;
      cfg.gain = (uint8_t)constrain(g, 0, 12);
      break;
    }
    case 1: {
      int a = (int)cfg.atime + dir;
      cfg.atime = (uint8_t)constrain(a, 0, 255);
      break;
    }
    case 2: {
      long s = (long)cfg.astep + (long)dir * 50;
      cfg.astep = (uint16_t)constrain(s, 0L, 65534L);
      break;
    }
    case 3: {
      if (dir > 0) cfg.smux = (cfg.smux == 6) ? 12 : (cfg.smux == 12 ? 18 : 18);
      else         cfg.smux = (cfg.smux == 18) ? 12 : (cfg.smux == 12 ? 6 : 6);
      break;
    }
    case 4: {
      long m = (long)cfg.ledma + (long)dir * 2;
      cfg.ledma = (uint16_t)constrain(m, 4L, 258L);
      break;
    }
    case 5: {
      long c = (long)cfg.capms + (long)dir * 250;
      cfg.capms = (uint16_t)constrain(c, 10L, 60000L);
      break;
    }
  }
  applyConfig();
}

static void drawControlFrame() {
  tft.fillRect(0, CONTENT_Y, DISP_W, CONTENT_H, COL_BG);
  drawBtn(4,   CONTENT_Y + 142, 74, 22, "CAPTURE", false);
  drawBtn(84,  CONTENT_Y + 142, 74, 22, "SAVE",    false);
  drawBtn(164, CONTENT_Y + 142, 74, 22, "LOAD",    false);
  drawBtn(244, CONTENT_Y + 142, 72, 22, "DEFAULT", false);
}

static void updateControlDynamics() {
  tft.setTextSize(1);
  for (uint8_t r = 0; r < 6; r++) {
    int16_t y  = CT_ROW_Y0 + r * CT_ROW_H;
    bool    sel = (ctrlSel == r);
    uint16_t bc = sel ? COL_YELLOW : COL_DKGRAY;

    tft.setTextColor(sel ? COL_YELLOW : COL_LTGRAY, COL_BG);
    tft.setCursor(6, y + 6);
    tft.print(kCtrlRows[r].label);

    tft.fillRoundRect(CT_MINUS_X, y, CT_BTN_W, CT_BTN_H, 3, COL_BTN_OFF);
    tft.drawRoundRect(CT_MINUS_X, y, CT_BTN_W, CT_BTN_H, 3, bc);
    tft.setTextColor(bc);
    tft.setCursor(CT_MINUS_X + 10, y + 6);
    tft.print('-');

    char buf[16];
    ctrlRowValue(r, buf, sizeof(buf));
    tft.setTextColor(sel ? COL_WHITE : COL_LTGRAY, COL_BG);
    tft.setCursor(CT_MINUS_X + 36, y + 6);
    tft.print(buf); tft.print(F("        "));

    tft.fillRoundRect(CT_PLUS_X, y, CT_BTN_W, CT_BTN_H, 3, COL_BTN_OFF);
    tft.drawRoundRect(CT_PLUS_X, y, CT_BTN_W, CT_BTN_H, 3, bc);
    tft.setTextColor(bc);
    tft.setCursor(CT_PLUS_X + 10, y + 6);
    tft.print('+');
  }

  tft.setTextColor(COL_CYAN, COL_BG);
  tft.setCursor(6, CT_ROW_Y0 + 6 * CT_ROW_H + 2);
  tft.print(F("tint ")); tft.print(integrationMs(), 2); tft.print(F("ms  "));
  if (lastCapSamples) {
    tft.print(F("last ")); tft.print(lastCapRate, 1);
    tft.print(F("Hz n=")); tft.print(lastCapSamples); tft.print(F("     "));
  } else {
    tft.print(F("no capture yet        "));
  }
}

static void redraw(bool full) {
  if (full) {
    drawHeaderFrame();
    drawTabBar();
    if (currentPage == PAGE_SPECTRUM) drawSpectrumFrame();
    else                              drawControlFrame();
  }
  updateHeaderDynamics();
  if (currentPage == PAGE_SPECTRUM) updateSpectrumDynamics();
  else                              updateControlDynamics();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TOUCH
// ═══════════════════════════════════════════════════════════════════════════════

static void mapTouch(uint16_t rawX, uint16_t rawY, int16_t *sx, int16_t *sy) {
  *sx = (int16_t)map((long)rawY, TS_RAW_Y_LEFT, TS_RAW_Y_RIGHT, 0, DISP_W);
  *sy = (int16_t)map((long)rawX, TS_RAW_X_TOP,  TS_RAW_X_BOT,  0, DISP_H);
  *sx = constrain(*sx, 0, DISP_W - 1);
  *sy = constrain(*sy, 0, DISP_H - 1);
}

static inline bool inRect(int16_t px, int16_t py, int16_t x, int16_t y, int16_t w, int16_t h) {
  return (px >= x && px < x + w && py >= y && py < y + h);
}

static bool          touchActive  = false;
static unsigned long touchStartMs = 0;
static unsigned long lastTouchChk = 0;

static void captureFromTouch() {
  tft.fillRect(60, 100, 200, 40, COL_HDR);
  tft.drawRect(60, 100, 200, 40, COL_YELLOW);
  tft.setTextColor(COL_YELLOW, COL_HDR);
  tft.setTextSize(2);
  tft.setCursor(90, 112);
  tft.print(F("CAPTURING"));
  tft.setTextSize(1);

  runCapture(cfg.capms);
  sendCapture();            // a connected GUI picks this up as if it had asked
  blinkPixel(0, 0, 40);
  needFullRedraw = true;
}

static void handleTouch(int16_t sx, int16_t sy) {
  if (sy >= DISP_H - TAB_H) {
    uint8_t p = (sx < 160) ? PAGE_SPECTRUM : PAGE_CONTROL;
    if (p != currentPage) { currentPage = p; needFullRedraw = true; }
    return;
  }

  if (currentPage == PAGE_SPECTRUM) {
    if (inRect(sx, sy, 4,   CONTENT_Y + 150, 70, 22)) { liveHold = !liveHold; needFullRedraw = true; return; }
    if (inRect(sx, sy, 80,  CONTENT_Y + 150, 70, 22)) { cfg.led = !cfg.led; applyConfig(); needFullRedraw = true; return; }
    if (inRect(sx, sy, 156, CONTENT_Y + 150, 76, 22)) { captureFromTouch(); return; }
    return;
  }

  // CONTROL page
  if (inRect(sx, sy, 4,   CONTENT_Y + 142, 74, 22)) { captureFromTouch(); return; }
  if (inRect(sx, sy, 84,  CONTENT_Y + 142, 74, 22)) { saveConfig(); blinkPixel(0, 40, 0); needFullRedraw = true; return; }
  if (inRect(sx, sy, 164, CONTENT_Y + 142, 74, 22)) {
    if (loadConfig()) { applyConfig(); blinkPixel(0, 40, 0); } else blinkPixel(40, 0, 0);
    needFullRedraw = true; return;
  }
  if (inRect(sx, sy, 244, CONTENT_Y + 142, 72, 22)) { loadDefaults(); applyConfig(); needFullRedraw = true; return; }

  for (uint8_t r = 0; r < 6; r++) {
    int16_t y = CT_ROW_Y0 + r * CT_ROW_H;
    if (sy < y || sy >= y + CT_BTN_H) continue;
    ctrlSel = r;
    if (inRect(sx, sy, CT_MINUS_X, y, CT_BTN_W, CT_BTN_H)) ctrlRowStep(r, -1);
    if (inRect(sx, sy, CT_PLUS_X,  y, CT_BTN_W, CT_BTN_H)) ctrlRowStep(r, +1);
    needFullRedraw = true;
    return;
  }
}

static void pollTouch() {
  if (!tsOK) return;
  if (digitalRead(TS_IRQ)) { touchActive = false; return; }

  unsigned long now = millis();
  if (now - lastTouchChk < 20) return;
  lastTouchChk = now;
  if (touchActive) return;

  TS_Point p = ts.getPoint();
  if ((p.x == 0 && p.y == 0) || p.z < TS_MIN_Z) return;
  if (now - touchStartMs < TOUCH_DEBOUNCE_MS) return;

  touchActive  = true;
  touchStartMs = now;
  int16_t sx, sy;
  mapTouch((uint16_t)p.x, (uint16_t)p.y, &sx, &sy);

#ifdef TOUCH_DEBUG
  Serial.print(F("# touch raw ")); Serial.print(p.x); Serial.print(',');
  Serial.print(p.y); Serial.print(F(" z=")); Serial.print(p.z);
  Serial.print(F(" -> ")); Serial.print(sx); Serial.print(','); Serial.println(sy);
#endif

  handleTouch(sx, sy);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP / LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  pixel.begin();
  pixel.setBrightness(40);
  pixel.clear();
  pixel.show();

  Wire.begin();
  Wire.setClock(400000);
  SPI.begin();

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(COL_BG);
  tft.setTextWrap(false);

  pinMode(TS_IRQ, INPUT);   // TSC2007 IRQ: HIGH = idle, LOW = touched
  tsOK = ts.begin(TS_I2C_ADDR, &Wire);

  EEPROM.begin(512);
  loadDefaults();
  bool restored = loadConfig();

  sensorOK = as7343.begin(AS7343_I2CADDR_DEFAULT, &Wire);
  if (sensorOK) applyConfig();
  else          rebuildEnabled();

  Serial.println(F("# AS7343_TFT_Controller " FW_VERSION));
  Serial.print(F("# sensor "));  Serial.println(sensorOK ? F("OK") : F("NOT FOUND"));
  Serial.print(F("# touch "));   Serial.println(tsOK ? F("OK") : F("NOT FOUND"));
  Serial.print(F("# config "));  Serial.println(restored ? F("from EEPROM") : F("defaults"));

  needFullRedraw = true;
  blinkPixel(0, 40, 0, 40);
}

void loop() {
  pollSerial();
  pollTouch();

  unsigned long now = millis();

  // Live measurement state machine — feeds the bar chart, !READ and !STREAM.
  if (sensorOK) {
    if (liveState == 0) {
      bool want = readPending
               || (streaming && (now - lastStreamMs >= cfg.streamms))
               || (!liveHold && (now - lastLiveMs >= 200));
      if (want) liveKickoff();
    }
    if (liveService()) {
      lastLiveMs = now;
      if (readPending) {
        sendReading();
        readPending = false;
      } else if (streaming) {
        sendReading();
        lastStreamMs = now;
      }
    }
  }

  static unsigned long lastDraw = 0;
  if (needFullRedraw) {
    needFullRedraw = false;
    redraw(true);
    lastDraw = now;
  } else if (now - lastDraw >= 150) {
    redraw(false);
    lastDraw = now;
  }
}
