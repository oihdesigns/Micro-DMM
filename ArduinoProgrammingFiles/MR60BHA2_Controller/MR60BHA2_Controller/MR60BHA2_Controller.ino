/*
  MR60BHA2_Controller.ino
  Seeed XIAO 60GHz mmWave Human Breathing and Heartbeat Sensor — MR60BHA2.
  Target: the Seeed XIAO ESP32-C6 that ships plugged into the MR60BHA2 carrier.

  Purpose: expose every lever that actually exists on this part over a serial
  protocol so the companion Python GUI can drive them, and record timed captures
  of the raw phase waveform so you can see what the radar is really producing
  instead of only the two numbers it condescends to report.

  ── What the "levers" are on this sensor ────────────────────────────────────────
  Worth being blunt, because it shapes the whole sketch: unlike a spectral sensor
  with an exposed register map, the MR60BHA2 is a sealed module that streams
  decoded reports over UART.  Seeed's library offers no configuration commands for
  it at all.  So the levers here fall into three groups:

    1. ACQUISITION — how this MCU pumps the module's UART, which is a real and
       measurable lever (see `pumpms` and the library gotcha noted below).
    2. INTERPRETATION — validity gating, median + EMA filtering, presence hold.
       The module's own breath/heart numbers are heavily smoothed already; these
       let you decide how much more (or less) processing sits on top, and the
       capture path lets you re-derive rate from phase yourself.
    3. PROTOCOL — raw frame monitoring, arbitrary frame transmission and a probe
       sweep, so undocumented module commands can be hunted for from the GUI.

  ── Frames are parsed here, not by the library ─────────────────────────────────
  This sketch subclasses SEEED_MR60BHA2 and overrides handleType(), then parses
  every payload itself rather than calling the base implementation.  Two reasons,
  both of which matter for bench work:

    * The library's getters are one-shot: getBreathRate() clears the valid flag
      and returns false thereafter, so "no new frame" and "no reading" look the
      same to the caller.
    * getDistance() returns false when the module's range flag is 0, which hides
      the single most interesting bit in that frame — "I see a chest, but not one
      I trust".  Same for isHumanDetected(), which returns false both for "no
      human" and for "no frame since you last asked".

  Parsing the payloads directly hands the host the flag AND the value, and lets
  every report type be counted for rate statistics even when it is filtered out.

  ── Library gotcha: fetch() blocks ─────────────────────────────────────────────
  SeeedmmWave::fetch(timeout) is `do { drain UART } while (millis() < expire)`.
  It burns the whole timeout every call whether or not data arrives, so the
  ubiquitous `mmWave.update(100)` from Seeed's examples costs 100 ms of dead loop
  per pass.  fetch(0) still executes the body once, which drains everything
  currently buffered and returns immediately — that is what `pumpms 0` does, and
  it is the default.  The lever is left exposed because sweeping it is the
  clearest way to show what the blocking form costs you in report latency.

  ── Libraries required ─────────────────────────────────────────────────────────
  Seeed Arduino mmWave     https://github.com/Love4yzp/Seeed-mmWave-library
  Adafruit NeoPixel        (carrier board's RGB LED on D1)
  Preferences              (bundled with the esp32 core — NVS-backed config)

  ── Wiring / board ─────────────────────────────────────────────────────────────
  Radar UART → XIAO UART0 (D6 TX / D7 RX), 115200 8N1, hardwired by the carrier
  RGB LED    → D1 (single WS2812 on the carrier)
  Host       → USB CDC on the C6's native USB

  Board: "XIAO_ESP32C6" in the esp32 core.  Tools ▸ USB CDC On Boot MUST be
  Enabled, otherwise `Serial` is UART0 and fights the radar for the same pins.

  ── Commands (host → device) ───────────────────────────────────────────────────
  !PING                Identify           → $HELLO,MR60BHA2,<fw>
  !ID                  Module fw version  → $ID,<proj>,<major>,<sub>,<mod>
  !CFG                 Dump config        → $CFG,<key>,<val>,<min>,<max> … $CFGEND
  !GET,<key>           Read one key       → $VAL,<key>,<val>
  !SET,<key>,<val>     Write one key      → $OK,<key>,<val> | $ERR,<key>,<reason>
  !READ                One snapshot       → $R,…
  !TGT                 Current target list → $T,… then $TEND,<n>
  !STREAM,0|1          Continuous $R lines
  !CAP[,<ms>]          Timed capture of the phase waveform (see below)
  !ABORT               End a running capture early and dump what it has
  !STATS[,0]           Per-report-type frame counts and rates (,0 clears them)
  !RAW,0|1|2           Raw frame echo: off / unknown types only / all
  !TX,<type>[,<hex>]   Send one frame to the module (hex type, hex payload)
  !PROBE,<t0>,<t1>[,<gap_ms>]   Sweep a type range looking for replies
  !RESET               Pulse the module reset pin, if `rstpin` is configured
  !ZERO                Reset filters, presence state and frame statistics
  !SAVE / !LOAD / !DEFAULTS     NVS-backed config

  ── Reply lines ────────────────────────────────────────────────────────────────
  $R,<t_ms>,<present>,<rflag>,<dist>,<br>,<hr>,<total>,<breath>,<heart>,
     <br_raw>,<hr_raw>,<dist_raw>,<human>,<ntgt>,<age_ms>
        present  0/1 derived from `holdms`      rflag   module's range-valid flag
        dist     gated + filtered, metres       human   module's presence byte, -1 = never seen
        br/hr    gated + filtered               *_raw   straight off the wire, ungated
        nan      = never received               age_ms  since the last accepted frame

  $T,<i>,<x>,<y>,<dop>,<cluster>,<speed_cm_s>   one per tracked target
  $F,<t_ms>,<type>,<len>,<hex>                  raw frame echo (see `rawmode`)
  $EVT,<t_ms>,<what>,<val>                      presence / target-count transitions
  $STAT,<name>,<type>,<count>,<hz>,<age_ms>     … terminated by $STATEND
  $PRB,<type>,<n_unexpected>,<types…>           … terminated by $PRBEND

  ── Capture reply format ───────────────────────────────────────────────────────
  $CAPB,<n>,<elapsed_us>,<rate_hz>,<mode>,<dt_ms>,<dropped>
  $CH,t_us,total,breath,heart,dist,br,hr
  $S,<t_us>,<total>,<breath>,<heart>,<dist>,<br>,<hr>
  $CAPE,<n>

  Lines beginning with '#' are human-readable debug and can be ignored.
*/

#include <Arduino.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include "Seeed_Arduino_mmWave.h"

#define FW_VERSION "1.0"

#define PIXEL_PIN D1        // carrier board's WS2812
#define RADAR_UART 0        // XIAO ESP32-C6 UART0 = D6 TX / D7 RX

// ═══════════════════════════════════════════════════════════════════════════════
//  TYPES
//  Declared ahead of any function so the Arduino preprocessor's auto-generated
//  prototypes — which it inserts just before the first function definition —
//  can name them.
// ═══════════════════════════════════════════════════════════════════════════════

struct Config {
  uint32_t magic;
  uint16_t version;

  // Acquisition
  uint8_t  pumpms;     // fetch() timeout per loop pass; 0 = non-blocking drain
  uint8_t  strmode;    // 0 = timer-paced $R, 1 = one $R per phase frame
  uint16_t streamms;   // stream period for strmode 0

  // Report handling
  uint16_t repmask;    // which report types are processed (see kTypes[])
  uint8_t  rawmode;    // 0 off, 1 unknown types only, 2 every frame

  // Validity gating
  uint16_t dminmm;     // distance window, millimetres (module reports metres)
  uint16_t dmaxmm;
  uint8_t  brmin;      // plausible breath rate window, brpm
  uint8_t  brmax;
  uint8_t  hrmin;      // plausible heart rate window, bpm
  uint8_t  hrmax;
  uint16_t holdms;     // presence hold after the last accepted frame

  // Filtering
  uint8_t  medn;       // median window, 1..9, snapped odd (1 = off)
  uint16_t emabr;      // EMA alpha × 1000 (1000 = passthrough)
  uint16_t emahr;
  uint16_t emad;

  // Indicator
  uint8_t  ledmode;    // 0 off, 1 presence, 2 breath phase, 3 heart pulse, 4 frames
  uint8_t  ledbri;     // 0..255

  // Capture
  uint16_t capms;      // capture window
  uint8_t  capmode;    // 0 = one sample per phase frame, 1 = uniform grid
  uint16_t capdtms;    // grid period for capmode 1

  // Protocol experiments
  uint8_t  txck;       // append the data checksum to zero-length frames
  uint8_t  unsafe;     // allow !TX / !PROBE outside the known report range
  uint8_t  rstpin;     // module reset GPIO, 255 = not wired

  uint32_t checksum;
};

enum CfgType : uint8_t { CT_U8, CT_U16, CT_U32 };

struct CfgItem {
  const char *key;
  CfgType     type;
  void       *ptr;
  uint32_t    minV;
  uint32_t    maxV;
};

// One row per report type the module is known to emit.  `bit` indexes repmask.
struct TypeInfo {
  uint16_t    type;
  const char *name;
  uint8_t     bit;
};

// Median-then-EMA filter state, one per filtered quantity.
struct Filt {
  float   hist[9];
  uint8_t n;
  uint8_t idx;
  float   ema;
  bool    has;
};

struct CapSample {
  uint32_t t_us;
  float    total, breath, heart;
  float    dist, br, hr;
};

struct TargetN2 {              // our own copy — the library's TargetN is parsed
  float   x, y;                // by a routine we deliberately bypass
  int32_t dop, cluster;
};

// The frame tap.  Every validated frame lands in onFrame(); the base class'
// parsing is intentionally never invoked (see the file header).
static void onFrame(uint16_t type, const uint8_t *data, size_t len);
static void sendReading();

class MmWaveTap : public SEEED_MR60BHA2 {
 public:
  bool handleType(uint16_t type, const uint8_t *data, size_t len) override {
    onFrame(type, data, len);
    return true;
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
//  REPORT TYPE TABLE
// ═══════════════════════════════════════════════════════════════════════════════

#define T_PHASE  0x0A13
#define T_BREATH 0x0A14
#define T_HEART  0x0A15
#define T_DIST   0x0A16
#define T_HUMAN  0x0F09
#define T_PCDET  0x0A08
#define T_PCTGT  0x0A04
#define T_FWVER  0xFFFF

static const TypeInfo kTypes[] = {
  { T_PHASE,  "phase",  0 },   // total / breath / heart phase, 3 floats
  { T_BREATH, "breath", 1 },   // breath rate, 1 float
  { T_HEART,  "heart",  2 },   // heart rate, 1 float
  { T_DIST,   "dist",   3 },   // range flag u32 + range float
  { T_HUMAN,  "human",  4 },   // presence byte
  { T_PCDET,  "pcdet",  5 },   // point-cloud detections
  { T_PCTGT,  "pctgt",  6 },   // tracked targets
  { T_FWVER,  "fwver",  7 },   // module firmware version, u32
};
static const uint8_t kNumTypes = sizeof(kTypes) / sizeof(kTypes[0]);

#define REPMASK_ALL 0x00FF

// ═══════════════════════════════════════════════════════════════════════════════
//  CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════════

#define CFG_MAGIC   0x36304248UL   // '60BH'
#define CFG_VERSION 1
#define NVS_NS      "mr60bha2"

static Config cfg;

static void loadDefaults() {
  cfg.magic    = CFG_MAGIC;
  cfg.version  = CFG_VERSION;
  cfg.pumpms   = 0;        // non-blocking drain — see the header note on fetch()
  cfg.strmode  = 0;
  cfg.streamms = 200;
  cfg.repmask  = REPMASK_ALL;
  cfg.rawmode  = 0;
  cfg.dminmm   = 0;
  cfg.dmaxmm   = 3000;
  cfg.brmin    = 5;
  cfg.brmax    = 40;
  cfg.hrmin    = 40;
  cfg.hrmax    = 150;
  cfg.holdms   = 3000;
  cfg.medn     = 1;
  cfg.emabr    = 1000;     // passthrough: the module already smooths hard
  cfg.emahr    = 1000;
  cfg.emad     = 1000;
  cfg.ledmode  = 1;
  cfg.ledbri   = 40;
  cfg.capms    = 20000;
  cfg.capmode  = 0;
  cfg.capdtms  = 50;
  cfg.txck     = 1;
  cfg.unsafe   = 0;
  cfg.rstpin   = 255;
  cfg.checksum = 0;
}

// ─── Key table — drives !CFG / !GET / !SET and lets the GUI self-describe ─────
static const CfgItem kCfgItems[] = {
  { "pumpms",   CT_U8,  &cfg.pumpms,   0, 200    },
  { "strmode",  CT_U8,  &cfg.strmode,  0, 1      },
  { "streamms", CT_U16, &cfg.streamms, 20, 60000 },
  { "repmask",  CT_U16, &cfg.repmask,  0, 0xFF   },
  { "rawmode",  CT_U8,  &cfg.rawmode,  0, 2      },
  { "dminmm",   CT_U16, &cfg.dminmm,   0, 10000  },
  { "dmaxmm",   CT_U16, &cfg.dmaxmm,   0, 10000  },
  { "brmin",    CT_U8,  &cfg.brmin,    0, 100    },
  { "brmax",    CT_U8,  &cfg.brmax,    0, 100    },
  { "hrmin",    CT_U8,  &cfg.hrmin,    0, 250    },
  { "hrmax",    CT_U8,  &cfg.hrmax,    0, 250    },
  { "holdms",   CT_U16, &cfg.holdms,   0, 60000  },
  { "medn",     CT_U8,  &cfg.medn,     1, 9      },
  { "emabr",    CT_U16, &cfg.emabr,    1, 1000   },
  { "emahr",    CT_U16, &cfg.emahr,    1, 1000   },
  { "emad",     CT_U16, &cfg.emad,     1, 1000   },
  { "ledmode",  CT_U8,  &cfg.ledmode,  0, 4      },
  { "ledbri",   CT_U8,  &cfg.ledbri,   0, 255    },
  { "capms",    CT_U16, &cfg.capms,    100, 60000 },
  { "capmode",  CT_U8,  &cfg.capmode,  0, 1      },
  { "capdtms",  CT_U16, &cfg.capdtms,  5, 5000   },
  { "txck",     CT_U8,  &cfg.txck,     0, 1      },
  { "unsafe",   CT_U8,  &cfg.unsafe,   0, 1      },
  { "rstpin",   CT_U8,  &cfg.rstpin,   0, 255    },
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

HardwareSerial    mmWaveSerial(RADAR_UART);
MmWaveTap         mmWave;
Adafruit_NeoPixel pixel(1, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

Preferences prefs;

// Latest module state.  NAN means "never received".
static float    stTotal = NAN, stBreath = NAN, stHeart = NAN;
static float    stBr = NAN, stHr = NAN, stDist = NAN;
static float    stBrRaw = NAN, stHrRaw = NAN, stDistRaw = NAN;
static uint32_t stRangeFlag = 0;
static int8_t   stHuman = -1;
static uint8_t  stNTgt = 0;
static TargetN2 stTgt[8];

static Filt filtBr, filtHr, filtD;

static uint32_t lastAcceptMs = 0;     // last frame that passed gating
static bool     present      = false;
static uint8_t  lastNTgtRep  = 255;

static uint32_t fwRaw = 0;
static bool     fwSeen = false;

// Per-type frame statistics
static uint32_t typeCount[kNumTypes];
static uint32_t typeFirstMs[kNumTypes];
static uint32_t typeLastMs[kNumTypes];

#define MAX_UNK 8
static uint16_t unkType[MAX_UNK];
static uint32_t unkCount[MAX_UNK];
static uint32_t unkLastMs[MAX_UNK];
static uint8_t  unkN = 0;
static uint32_t badFrames = 0;   // frames whose payload length made no sense

// Capture buffer.  2400 × 28 B ≈ 67 KB of the C6's SRAM.
#define CAP_MAX_SAMPLES 2400
static CapSample capBuf[CAP_MAX_SAMPLES];
static uint16_t  capCount   = 0;
static bool      capActive  = false;
static uint32_t  capStartUs = 0;
static uint32_t  capEndMs   = 0;
static uint32_t  capNextMs  = 0;
static uint16_t  capDropped = 0;   // samples the buffer had no room for

// Probe state — set while !PROBE is walking a type range.
static bool     probeActive = false;
static uint16_t probeSeen[16];
static uint8_t  probeSeenN  = 0;
static uint16_t probeUnexpected = 0;

static bool          streaming    = false;
static unsigned long lastStreamMs = 0;

static char    serialBuf[160];
static uint8_t serialLen = 0;

// ═══════════════════════════════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

static uint32_t rdU32(const uint8_t *p) {
  uint32_t v;
  memcpy(&v, p, 4);            // the module is little-endian, as is the C6
  return v;
}

static float rdF32(const uint8_t *p) {
  float v;
  memcpy(&v, p, 4);            // memcpy, not a cast — the payload is unaligned
  return v;
}

static int8_t typeIndex(uint16_t t) {
  for (uint8_t i = 0; i < kNumTypes; i++) {
    if (kTypes[i].type == t) return (int8_t)i;
  }
  return -1;
}

static void filtReset(Filt &f) {
  f.n = 0;
  f.idx = 0;
  f.ema = 0;
  f.has = false;
}

// Median over the last `medn` samples, then a first-order EMA.  Both stages are
// no-ops at their defaults (medn 1, alpha 1000) so the raw module value passes
// straight through unless you ask for something else.
static float filtPush(Filt &f, float v, uint8_t medn, uint16_t alpha1000) {
  if (medn > 9) medn = 9;
  if (medn < 1) medn = 1;
  if (!(medn & 1)) medn--;          // even windows have no middle element

  f.hist[f.idx] = v;
  f.idx = (uint8_t)((f.idx + 1) % 9);
  if (f.n < 9) f.n++;

  float med = v;
  uint8_t take = medn < f.n ? medn : f.n;
  if (take > 1) {
    float tmp[9];
    for (uint8_t i = 0; i < take; i++) {
      uint8_t k = (uint8_t)((f.idx + 9 - 1 - i) % 9);   // newest backwards
      tmp[i] = f.hist[k];
    }
    for (uint8_t i = 1; i < take; i++) {                // insertion sort
      float x = tmp[i];
      int8_t j = (int8_t)i - 1;
      while (j >= 0 && tmp[j] > x) { tmp[j + 1] = tmp[j]; j--; }
      tmp[j + 1] = x;
    }
    med = tmp[take / 2];
  }

  float a = (float)alpha1000 / 1000.0f;
  if (!f.has) { f.ema = med; f.has = true; }
  else        f.ema = f.ema + a * (med - f.ema);
  return f.ema;
}

static uint32_t cfgChecksum() {
  const uint8_t *p = (const uint8_t *)&cfg;
  size_t n = offsetof(Config, checksum);   // never sizeof-minus: tail padding
  uint32_t sum = 2166136261UL;             // FNV-1a
  for (size_t i = 0; i < n; i++) { sum ^= p[i]; sum *= 16777619UL; }
  return sum;
}

static bool saveConfig() {
  cfg.magic    = CFG_MAGIC;
  cfg.version  = CFG_VERSION;
  cfg.checksum = cfgChecksum();
  prefs.begin(NVS_NS, false);
  size_t n = prefs.putBytes("cfg", &cfg, sizeof(Config));
  prefs.end();
  return n == sizeof(Config);
}

static bool loadConfig() {
  Config tmp;
  prefs.begin(NVS_NS, true);
  size_t n = prefs.getBytes("cfg", &tmp, sizeof(Config));
  prefs.end();
  if (n != sizeof(Config)) return false;
  if (tmp.magic != CFG_MAGIC || tmp.version != CFG_VERSION) return false;
  Config keep = cfg;
  cfg = tmp;
  if (cfgChecksum() != tmp.checksum) { cfg = keep; return false; }
  return true;
}

static void resetState() {
  filtReset(filtBr);
  filtReset(filtHr);
  filtReset(filtD);
  stTotal = stBreath = stHeart = NAN;
  stBr = stHr = stDist = NAN;
  stBrRaw = stHrRaw = stDistRaw = NAN;
  stRangeFlag = 0;
  stHuman = -1;
  stNTgt = 0;
  lastNTgtRep = 255;
  present = false;
  lastAcceptMs = 0;
  for (uint8_t i = 0; i < kNumTypes; i++) {
    typeCount[i] = 0;
    typeFirstMs[i] = 0;
    typeLastMs[i] = 0;
  }
  unkN = 0;
  badFrames = 0;
}

static void sendEvent(const char *what, long val) {
  Serial.printf("$EVT,%lu,%s,%ld\n", (unsigned long)millis(), what, val);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  FRAME HANDLING
//  Called from MmWaveTap::handleType for every frame that passed both the header
//  and payload checksums.  Payload layouts are little-endian, packed.
// ═══════════════════════════════════════════════════════════════════════════════

static void echoRaw(uint16_t type, const uint8_t *data, size_t len, bool known) {
  if (cfg.rawmode == 0) return;
  if (cfg.rawmode == 1 && known) return;
  Serial.printf("$F,%lu,%04X,%u,", (unsigned long)millis(), type, (unsigned)len);
  size_t show = len > 96 ? 96 : len;      // don't let a fat frame flood the link
  for (size_t i = 0; i < show; i++) Serial.printf("%02X", data[i]);
  if (show < len) Serial.print(F(".."));
  Serial.println();
}

// Point-cloud payloads are a u32 count followed by 16 bytes per target.  The
// library trusts that count blindly and walks off the end of short frames; we
// bound it against the payload length instead.
static uint8_t parseTargets(const uint8_t *data, size_t len) {
  if (len < 4) return 0;
  uint32_t n = rdU32(data);
  if (n > 64 || 4 + n * 16 > len) { badFrames++; return 0; }
  const uint8_t *p = data + 4;
  uint8_t keep = n > 8 ? 8 : (uint8_t)n;
  for (uint8_t i = 0; i < keep; i++) {
    stTgt[i].x       = rdF32(p);      p += 4;
    stTgt[i].y       = rdF32(p);      p += 4;
    stTgt[i].dop     = (int32_t)rdU32(p); p += 4;
    stTgt[i].cluster = (int32_t)rdU32(p); p += 4;
  }
  return (uint8_t)n;
}

static void noteUnknown(uint16_t type) {
  for (uint8_t i = 0; i < unkN; i++) {
    if (unkType[i] == type) {
      unkCount[i]++;
      unkLastMs[i] = millis();
      return;
    }
  }
  if (unkN < MAX_UNK) {
    unkType[unkN] = type;
    unkCount[unkN] = 1;
    unkLastMs[unkN] = millis();
    unkN++;
  }
}

static void capAppend(uint32_t t_us);

static void onFrame(uint16_t type, const uint8_t *data, size_t len) {
  uint32_t now = millis();
  int8_t ti = typeIndex(type);

  if (probeActive) {
    if (probeSeenN < 16) probeSeen[probeSeenN++] = type;
    if (ti < 0) probeUnexpected++;      // a spontaneous report is not a reply
  }

  echoRaw(type, data, len, ti >= 0);

  if (ti < 0) {
    noteUnknown(type);
    return;
  }

  // Statistics count every well-formed frame, including ones repmask filters out
  // — the whole point is to see what the module is actually emitting.
  if (typeCount[ti] == 0) typeFirstMs[ti] = now;
  typeCount[ti]++;
  typeLastMs[ti] = now;

  if (!(cfg.repmask & (1u << kTypes[ti].bit))) return;

  bool accepted = false;

  switch (type) {
    case T_PHASE: {
      if (len < 12) { badFrames++; break; }
      stTotal  = rdF32(data);
      stBreath = rdF32(data + 4);
      stHeart  = rdF32(data + 8);
      accepted = true;
      if (capActive && cfg.capmode == 0) capAppend(micros() - capStartUs);
      if (streaming && cfg.strmode == 1) sendReading();
      break;
    }

    case T_BREATH: {
      if (len < 4) { badFrames++; break; }
      float v = rdF32(data);
      stBrRaw = v;
      if (v >= (float)cfg.brmin && v <= (float)cfg.brmax) {
        stBr = filtPush(filtBr, v, cfg.medn, cfg.emabr);
        accepted = true;
      }
      break;
    }

    case T_HEART: {
      if (len < 4) { badFrames++; break; }
      float v = rdF32(data);
      stHrRaw = v;
      if (v >= (float)cfg.hrmin && v <= (float)cfg.hrmax) {
        stHr = filtPush(filtHr, v, cfg.medn, cfg.emahr);
        accepted = true;
      }
      break;
    }

    case T_DIST: {
      if (len < 8) { badFrames++; break; }
      stRangeFlag = rdU32(data);          // the bit getDistance() hides
      float v = rdF32(data + 4);
      stDistRaw = v;
      float mm = v * 1000.0f;
      if (stRangeFlag && mm >= (float)cfg.dminmm && mm <= (float)cfg.dmaxmm) {
        stDist = filtPush(filtD, v, cfg.medn, cfg.emad);
        accepted = true;
      }
      break;
    }

    case T_HUMAN: {
      if (len < 1) { badFrames++; break; }
      int8_t h = data[0] ? 1 : 0;         // "no human" and "no frame" are
      stHuman = h;                        // different things — keep them so
      if (h) accepted = true;
      break;
    }

    case T_PCDET:
    case T_PCTGT: {
      uint8_t n = parseTargets(data, len);
      stNTgt = n;
      if (n != lastNTgtRep) {
        sendEvent("targets", n);
        lastNTgtRep = n;
      }
      if (n) accepted = true;
      break;
    }

    case T_FWVER: {
      if (len < 4) { badFrames++; break; }
      fwRaw = rdU32(data);
      fwSeen = true;
      break;
    }
  }

  if (accepted) lastAcceptMs = now;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  RADAR PUMP
// ═══════════════════════════════════════════════════════════════════════════════

// fetch(0) executes its drain body exactly once and returns — non-blocking.
// processQueuedFrames() with a non-zero timeout drains the whole frame queue in
// one call (its loop condition only tests that the timeout is non-zero, not the
// clock), so one pump call empties both the UART and the queue.
static void pumpRadar() {
  mmWave.fetch(cfg.pumpms);
  mmWave.processQueuedFrames(0xFFFF, 1);
}

static void servicePresence() {
  bool p = (cfg.holdms == 0)
             ? (stRangeFlag != 0)
             : (lastAcceptMs != 0 && (millis() - lastAcceptMs) < cfg.holdms);
  if (p != present) {
    present = p;
    sendEvent("present", p ? 1 : 0);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  FRAME TRANSMISSION
//  Built here rather than via SeeedmmWave::send() because that path omits the
//  data checksum entirely when the payload is empty, which does not match the
//  frame layout its own receiver expects.  `txck` lets you send it either way.
// ═══════════════════════════════════════════════════════════════════════════════

static uint8_t frameChecksum(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) c ^= d[i];
  return (uint8_t)~c;
}

static bool txFrame(uint16_t type, const uint8_t *data, size_t len) {
  static uint16_t txId = 0x8000;
  uint8_t hdr[8];
  hdr[0] = SOF_BYTE;
  hdr[1] = (uint8_t)(txId >> 8);
  hdr[2] = (uint8_t)(txId & 0xFF);
  hdr[3] = (uint8_t)(len >> 8);
  hdr[4] = (uint8_t)(len & 0xFF);
  hdr[5] = (uint8_t)(type >> 8);
  hdr[6] = (uint8_t)(type & 0xFF);
  hdr[7] = frameChecksum(hdr, 7);
  txId++;

  mmWave.write(hdr, sizeof(hdr));
  if (len) {
    mmWave.write(data, len);
    uint8_t ck = frameChecksum(data, len);
    mmWave.write(&ck, 1);
  } else if (cfg.txck) {
    uint8_t ck = frameChecksum(nullptr, 0);   // 0xFF
    mmWave.write(&ck, 1);
  }
  return true;
}

// Precautionary, not authoritative: nothing published maps this module's command
// space, so anything outside the range its reports live in needs `unsafe` set.
static bool txTypeAllowed(uint16_t type) {
  if (cfg.unsafe) return true;
  return (type >= 0x0A00 && type <= 0x0AFF) || (type >= 0x0F00 && type <= 0x0FFF);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CAPTURE
//  Non-blocking: samples accumulate from the frame handler (capmode 0) or from a
//  timer in loop() (capmode 1) while the command parser stays live, then the whole
//  buffer is dumped when the window closes.
//
//    mode 0 "per frame" — one sample per phase report, i.e. the module's native
//                         cadence.  Non-uniform if a frame is ever dropped, but
//                         it is the honest record of what arrived and when.
//    mode 1 "grid"      — a snapshot of the latest values every `capdtms`, so the
//                         series is uniformly sampled and can go straight into an
//                         FFT without resampling.  Repeats values if the grid is
//                         faster than the module reports.
// ═══════════════════════════════════════════════════════════════════════════════

static void capAppend(uint32_t t_us) {
  if (capCount >= CAP_MAX_SAMPLES) { capDropped++; return; }
  CapSample &s = capBuf[capCount++];
  s.t_us   = t_us;
  s.total  = stTotal;
  s.breath = stBreath;
  s.heart  = stHeart;
  s.dist   = stDist;
  s.br     = stBr;
  s.hr     = stHr;
}

static void capStart(uint16_t windowMs) {
  capCount   = 0;
  capDropped = 0;
  capStartUs = micros();
  capEndMs   = millis() + windowMs;
  capNextMs  = millis();
  capActive  = true;
  Serial.printf("# capture started, %u ms, mode %u\n", windowMs, cfg.capmode);
}

static void capFinish() {
  capActive = false;
  uint32_t elapsed = micros() - capStartUs;
  float rate = elapsed ? (float)capCount * 1000000.0f / (float)elapsed : 0.0f;

  Serial.printf("$CAPB,%u,%lu,%.3f,%u,%u,%u\n",
                capCount, (unsigned long)elapsed, rate,
                cfg.capmode, cfg.capdtms, capDropped);
  Serial.println(F("$CH,t_us,total,breath,heart,dist,br,hr"));
  for (uint16_t i = 0; i < capCount; i++) {
    const CapSample &s = capBuf[i];
    Serial.printf("$S,%lu,%.5f,%.5f,%.5f,%.4f,%.3f,%.3f\n",
                  (unsigned long)s.t_us, s.total, s.breath, s.heart,
                  s.dist, s.br, s.hr);
  }
  Serial.printf("$CAPE,%u\n", capCount);
}

static void serviceCapture() {
  if (!capActive) return;
  uint32_t now = millis();
  if (cfg.capmode == 1) {
    while ((int32_t)(now - capNextMs) >= 0) {
      capAppend(micros() - capStartUs);
      capNextMs += cfg.capdtms;
      if (capCount >= CAP_MAX_SAMPLES) break;
    }
  }
  if ((int32_t)(now - capEndMs) >= 0 || capCount >= CAP_MAX_SAMPLES) capFinish();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SERIAL PROTOCOL
// ═══════════════════════════════════════════════════════════════════════════════

static void sendReading() {
  uint32_t now = millis();
  uint32_t age = lastAcceptMs ? (now - lastAcceptMs) : 0;
  Serial.printf("$R,%lu,%u,%lu,%.4f,%.3f,%.3f,%.5f,%.5f,%.5f,%.3f,%.3f,%.4f,%d,%u,%lu\n",
                (unsigned long)now,
                present ? 1 : 0,
                (unsigned long)stRangeFlag,
                stDist, stBr, stHr,
                stTotal, stBreath, stHeart,
                stBrRaw, stHrRaw, stDistRaw,
                (int)stHuman, stNTgt,
                (unsigned long)age);
}

static void sendTargets() {
  for (uint8_t i = 0; i < stNTgt && i < 8; i++) {
    Serial.printf("$T,%u,%.4f,%.4f,%ld,%ld,%.2f\n",
                  i, stTgt[i].x, stTgt[i].y,
                  (long)stTgt[i].dop, (long)stTgt[i].cluster,
                  (float)stTgt[i].dop * RANGE_STEP);   // cm/s, per Seeed's example
  }
  Serial.printf("$TEND,%u\n", stNTgt);
}

static void sendConfig() {
  for (uint8_t i = 0; i < kNumCfgItems; i++) {
    Serial.printf("$CFG,%s,%lu,%lu,%lu\n",
                  kCfgItems[i].key,
                  (unsigned long)cfgItemGet(kCfgItems[i]),
                  (unsigned long)kCfgItems[i].minV,
                  (unsigned long)kCfgItems[i].maxV);
  }
  Serial.printf("$CFG,fwmod,%lu,0,0\n", (unsigned long)fwRaw);
  Serial.printf("$CFG,ntypes,%u,0,0\n", kNumTypes);
  Serial.printf("$CFG,capmax,%u,0,0\n", CAP_MAX_SAMPLES);
  Serial.println(F("$CFGEND"));
}

static void sendStats() {
  uint32_t now = millis();
  for (uint8_t i = 0; i < kNumTypes; i++) {
    uint32_t span = (typeCount[i] > 1) ? (typeLastMs[i] - typeFirstMs[i]) : 0;
    float hz = span ? (float)(typeCount[i] - 1) * 1000.0f / (float)span : 0.0f;
    Serial.printf("$STAT,%s,%04X,%lu,%.3f,%lu\n",
                  kTypes[i].name, kTypes[i].type,
                  (unsigned long)typeCount[i], hz,
                  (unsigned long)(typeCount[i] ? now - typeLastMs[i] : 0));
  }
  for (uint8_t i = 0; i < unkN; i++) {
    Serial.printf("$STAT,unk%u,%04X,%lu,0.000,%lu\n",
                  i, unkType[i], (unsigned long)unkCount[i],
                  (unsigned long)(now - unkLastMs[i]));
  }
  Serial.printf("$STAT,bad,0000,%lu,0.000,0\n", (unsigned long)badFrames);
  Serial.println(F("$STATEND"));
}

// Walk a range of frame types, sending each one and listening for anything the
// module says back.  Spontaneous reports keep arriving throughout, so only types
// outside the known report set are counted as a reply.
static void doProbe(uint16_t t0, uint16_t t1, uint16_t gapms) {
  if (t1 < t0) { uint16_t s = t0; t0 = t1; t1 = s; }
  if ((uint32_t)t1 - t0 > 255) t1 = t0 + 255;
  if (gapms < 10)   gapms = 10;
  if (gapms > 2000) gapms = 2000;

  Serial.printf("# probe %04X..%04X gap %u ms\n", t0, t1, gapms);
  for (uint32_t t = t0; t <= t1; t++) {
    if (!txTypeAllowed((uint16_t)t)) continue;

    probeSeenN = 0;
    probeUnexpected = 0;
    probeActive = true;
    txFrame((uint16_t)t, nullptr, 0);

    uint32_t until = millis() + gapms;
    while ((int32_t)(millis() - until) < 0) pumpRadar();
    probeActive = false;

    if (probeUnexpected) {
      Serial.printf("$PRB,%04X,%u", (uint16_t)t, probeUnexpected);
      for (uint8_t i = 0; i < probeSeenN; i++) Serial.printf(",%04X", probeSeen[i]);
      Serial.println();
    }
  }
  Serial.println(F("$PRBEND"));
}

static const CfgItem *findCfgItem(const char *key) {
  for (uint8_t i = 0; i < kNumCfgItems; i++) {
    if (strcasecmp(key, kCfgItems[i].key) == 0) return &kCfgItems[i];
  }
  return nullptr;
}

static uint8_t hexToBytes(const char *s, uint8_t *out, uint8_t maxLen) {
  uint8_t n = 0;
  while (s[0] && s[1] && n < maxLen) {
    char pair[3] = { s[0], s[1], 0 };
    char *end;
    long v = strtol(pair, &end, 16);
    if (end != pair + 2) break;
    out[n++] = (uint8_t)v;
    s += 2;
  }
  return n;
}

static void doReset() {
  if (cfg.rstpin == 255) { Serial.println(F("$ERR,reset,norstpin")); return; }
  pinMode(cfg.rstpin, OUTPUT);
  digitalWrite(cfg.rstpin, LOW);
  delay(50);
  digitalWrite(cfg.rstpin, HIGH);
  resetState();
  Serial.println(F("$OK,reset,1"));
}

static void handleCommand(char *line) {
  if (line[0] != '!') return;
  char *cmd = line + 1;
  char *arg = strchr(cmd, ',');
  if (arg) { *arg = '\0'; arg++; }

  if (strcasecmp(cmd, "PING") == 0) {
    Serial.print(F("$HELLO,MR60BHA2,"));
    Serial.println(F(FW_VERSION));

  } else if (strcasecmp(cmd, "ID") == 0) {
    // The module announces its version unprompted at power-up; this reports the
    // last one seen rather than pretending we can ask for it on demand.
    if (fwSeen) {
      Serial.printf("$ID,%lu,%lu,%lu,%lu\n",
                    (unsigned long)(fwRaw & 0xFF),
                    (unsigned long)((fwRaw >> 8) & 0xFF),
                    (unsigned long)((fwRaw >> 16) & 0xFF),
                    (unsigned long)((fwRaw >> 24) & 0xFF));
    } else {
      Serial.println(F("$ERR,id,notseen"));
    }

  } else if (strcasecmp(cmd, "CFG") == 0) {
    sendConfig();

  } else if (strcasecmp(cmd, "GET") == 0 && arg) {
    const CfgItem *it = findCfgItem(arg);
    if (!it) { Serial.printf("$ERR,%s,nokey\n", arg); }
    else     { Serial.printf("$VAL,%s,%lu\n", it->key, (unsigned long)cfgItemGet(*it)); }

  } else if (strcasecmp(cmd, "SET") == 0 && arg) {
    char *val = strchr(arg, ',');
    if (!val) { Serial.println(F("$ERR,set,noval")); return; }
    *val = '\0'; val++;
    const CfgItem *it = findCfgItem(arg);
    if (!it) { Serial.printf("$ERR,%s,nokey\n", arg); return; }
    uint32_t v = strtoul(val, nullptr, 0);
    if (v < it->minV || v > it->maxV) {
      Serial.printf("$ERR,%s,range\n", it->key);
      return;
    }
    if (strcasecmp(it->key, "medn") == 0 && !(v & 1)) v--;   // odd windows only
    cfgItemSet(*it, v);
    if (strcasecmp(it->key, "ledbri") == 0) pixel.setBrightness(cfg.ledbri);
    Serial.printf("$OK,%s,%lu\n", it->key, (unsigned long)cfgItemGet(*it));

  } else if (strcasecmp(cmd, "READ") == 0) {
    sendReading();

  } else if (strcasecmp(cmd, "TGT") == 0) {
    sendTargets();

  } else if (strcasecmp(cmd, "STREAM") == 0) {
    streaming = arg && atoi(arg) != 0;
    Serial.printf("$OK,stream,%u\n", streaming ? 1 : 0);

  } else if (strcasecmp(cmd, "CAP") == 0) {
    uint16_t win = cfg.capms;
    if (arg) {
      long v = strtol(arg, nullptr, 10);
      if (v >= 100 && v <= 60000) { win = (uint16_t)v; cfg.capms = win; }
    }
    capStart(win);

  } else if (strcasecmp(cmd, "ABORT") == 0) {
    if (capActive) capFinish();
    else Serial.println(F("$ERR,abort,nocapture"));

  } else if (strcasecmp(cmd, "STATS") == 0) {
    if (arg && arg[0] == '0') {
      for (uint8_t i = 0; i < kNumTypes; i++) {
        typeCount[i] = 0; typeFirstMs[i] = 0; typeLastMs[i] = 0;
      }
      unkN = 0;
      badFrames = 0;
      Serial.println(F("$OK,stats,0"));
    } else {
      sendStats();
    }

  } else if (strcasecmp(cmd, "RAW") == 0) {
    long v = arg ? strtol(arg, nullptr, 10) : 0;
    if (v < 0) v = 0;
    if (v > 2) v = 2;
    cfg.rawmode = (uint8_t)v;
    Serial.printf("$OK,rawmode,%u\n", cfg.rawmode);

  } else if (strcasecmp(cmd, "TX") == 0 && arg) {
    char *payload = strchr(arg, ',');
    if (payload) { *payload = '\0'; payload++; }
    uint16_t type = (uint16_t)strtoul(arg, nullptr, 16);
    if (!txTypeAllowed(type)) {
      Serial.println(F("$ERR,tx,blocked"));
      return;
    }
    uint8_t buf[64];
    uint8_t n = payload ? hexToBytes(payload, buf, sizeof(buf)) : 0;
    txFrame(type, n ? buf : nullptr, n);
    Serial.printf("$OK,tx,%04X\n", type);

  } else if (strcasecmp(cmd, "PROBE") == 0 && arg) {
    if (!cfg.unsafe) { Serial.println(F("$ERR,probe,unsafe")); return; }
    char *a2 = strchr(arg, ','); if (a2) { *a2 = '\0'; a2++; }
    char *a3 = a2 ? strchr(a2, ',') : nullptr; if (a3) { *a3 = '\0'; a3++; }
    uint16_t t0 = (uint16_t)strtoul(arg, nullptr, 16);
    uint16_t t1 = a2 ? (uint16_t)strtoul(a2, nullptr, 16) : t0;
    uint16_t gap = a3 ? (uint16_t)strtoul(a3, nullptr, 10) : 200;
    doProbe(t0, t1, gap);

  } else if (strcasecmp(cmd, "RESET") == 0) {
    doReset();

  } else if (strcasecmp(cmd, "ZERO") == 0) {
    resetState();
    Serial.println(F("$OK,zero,1"));

  } else if (strcasecmp(cmd, "SAVE") == 0) {
    Serial.println(saveConfig() ? F("$OK,save,1") : F("$ERR,save,nvs"));

  } else if (strcasecmp(cmd, "LOAD") == 0) {
    if (loadConfig()) {
      pixel.setBrightness(cfg.ledbri);
      Serial.println(F("$OK,load,1"));
    } else {
      Serial.println(F("$ERR,load,nodata"));
    }

  } else if (strcasecmp(cmd, "DEFAULTS") == 0) {
    loadDefaults();
    pixel.setBrightness(cfg.ledbri);
    Serial.println(F("$OK,defaults,1"));

  } else {
    Serial.printf("$ERR,%s,unknown\n", cmd);
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
//  INDICATOR
// ═══════════════════════════════════════════════════════════════════════════════

static void serviceLed() {
  static uint32_t lastMs = 0;
  static uint32_t lastHeartCount = 0;
  static uint32_t pulseUntil = 0;
  static float    phaseScale = 1.0f;

  uint32_t now = millis();
  if (now - lastMs < 25) return;
  lastMs = now;

  uint8_t r = 0, g = 0, b = 0;

  switch (cfg.ledmode) {
    case 0:
      break;

    case 1:                                     // presence
      if (present) g = 255;
      else         r = 40;
      break;

    case 2: {                                   // breath phase brightness
      // The phase floats have no documented scale, so track a decaying peak and
      // normalise against it rather than inventing a full-scale constant.
      if (!isnan(stBreath)) {
        float a = fabsf(stBreath);
        if (a > phaseScale) phaseScale = a;
        else phaseScale *= 0.999f;
        if (phaseScale < 1e-4f) phaseScale = 1e-4f;
        float u = a / phaseScale;
        if (u > 1.0f) u = 1.0f;
        g = (uint8_t)(40.0f + 215.0f * u);
        b = (uint8_t)(120.0f * u);
      }
      break;
    }

    case 3: {                                   // one flash per heart report
      int8_t hi = typeIndex(T_HEART);
      if (hi >= 0 && typeCount[hi] != lastHeartCount) {
        lastHeartCount = typeCount[hi];
        pulseUntil = now + 80;
      }
      if ((int32_t)(now - pulseUntil) < 0) { r = 255; b = 40; }
      break;
    }

    case 4: {                                   // any frame at all
      uint32_t total = 0;
      for (uint8_t i = 0; i < kNumTypes; i++) total += typeCount[i];
      static uint32_t lastTotal = 0;
      if (total != lastTotal) { lastTotal = total; pulseUntil = now + 40; }
      if ((int32_t)(now - pulseUntil) < 0) { r = g = b = 200; }
      break;
    }
  }

  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP / LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  loadDefaults();
  bool restored = loadConfig();

  pixel.begin();
  pixel.setBrightness(cfg.ledbri);
  pixel.clear();
  pixel.show();

  resetState();
  mmWave.begin(&mmWaveSerial, 115200, 1,
               cfg.rstpin == 255 ? -1 : (int)cfg.rstpin);

  Serial.println(F("# MR60BHA2_Controller " FW_VERSION));
  Serial.print(F("# config "));
  Serial.println(restored ? F("from NVS") : F("defaults"));
  Serial.println(F("# radar UART0 @115200 — send !CFG for the lever list"));
}

void loop() {
  pollSerial();
  pumpRadar();
  servicePresence();
  serviceCapture();
  serviceLed();

  if (streaming && cfg.strmode == 0) {
    uint32_t now = millis();
    if (now - lastStreamMs >= cfg.streamms) {
      lastStreamMs = now;
      sendReading();
    }
  }
}
