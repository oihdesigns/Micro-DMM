/*
 * Config.ino -- EEPROM-backed configuration.
 *
 * Replaces the old CalibrationConstants.ino, which was an if/else ladder over
 * EEPROM byte 1 assigning fifteen correction factors and four constants to
 * globals.  Those same numbers are now a versioned, CRC-checked struct in data
 * flash, addressable by name over serial (!SET / !GET / !CFG / !SAVE).
 *
 * The six hand-tuned per-unit sets survive as LEGACY_CAL below: a meter with
 * no valid config block is seeded from its original byte-1 id, so flashing
 * this firmware onto an existing meter does not lose its calibration.
 *
 * EEPROM is written in exactly one place: configSave(), reached only from
 * !SAVE, !SN and the first-boot seed.  Everything else edits RAM.
 */

Config cfg;
bool   cfgDirty = false;
char   unitSN[SN_MAX_LEN] = "";

// Adding keys grows Config, and silently growing it past SN_EEPROM_ADDR would
// have configSave() overwrite the serial number instead of failing.  Catch it
// at compile time, where the fix is to move the SN block, not to lose one.
static_assert(CFG_EEPROM_ADDR + sizeof(Config) <= SN_EEPROM_ADDR,
              "Config block overruns the serial-number block");

// ==================================================================
//  RESISTANCE CALIBRATION LADDER
// ==================================================================
// Upper edge of each rCal bucket.  These are the same breakpoints the old
// if/else chain used, in the same order, so bucket N applies exactly where
// CF_<Nth letter> used to.  14 edges -> 15 buckets.
const float R_CAL_EDGES[R_CAL_BUCKETS - 1] = {
  0.75f, 3.0f, 7.0f, 20.0f, 70.0f, 170.0f, 700.0f, 1700.0f,
  7000.0f, 17000.0f, 70000.0f, 170000.0f, 700000.0f, 1700000.0f
};

// Which correction factor applies to a raw resistance.  Kept as its own
// function because !CAL needs to pick the same bucket the measurement path
// would, from the same value.
uint8_t rCalIndex(float raw) {
  for (uint8_t i = 0; i < R_CAL_BUCKETS - 1; i++)
    if (raw < R_CAL_EDGES[i]) return i;
  return R_CAL_BUCKETS - 1;
}

// ==================================================================
//  LEGACY PER-UNIT CALIBRATION
// ==================================================================
// Transcribed verbatim from CalibrationConstants.ino.  Where a branch there
// did not assign constantI / zenerMaxV / sleepV, the global kept its file-scope
// initializer -- those initializers (0.02016, 5.0, 0.615) are written out
// explicitly here rather than left implicit, so the table says what a unit
// actually ran with instead of depending on declaration order elsewhere.
//
// `bridge` records which boards have the float-detect circuit: the old code
// gated ClosedOrFloat() on `EEPROM.read(1) == 5 || EEPROM.read(1) == 6`, a
// hardware fact hidden inside a hot-loop condition.  It is a config key now.
static const LegacyCal LEGACY_CAL[] = {
  { 1, { 1.0f,    0.983f,  0.9926f, 0.9735f, 1.0732f, 1.0281f, 1.0018f, 0.9995f,
         0.9987f, 1.0054f, 0.9975f, 0.9904f, 0.9883f, 0.8741f, 0.7294f },
       63.539f,      0.02016f,  5.0f,   0.615f, 0 },
  { 2, { 1.0528f, 1.0135f, 1.0063f, 0.9853f, 0.9695f, 1.0183f, 0.9998f, 1.0002f,
         0.9984f, 1.0056f, 0.9999f, 0.996f,  1.0158f, 0.927f,  0.6924f },
       46.392f,      0.02016f,  5.0f,   0.615f, 0 },
  { 3, { 0.9867f, 1.0007f, 0.9987f, 0.9971f, 0.9952f, 0.9961f, 0.997f,  1.0044f,
         1.0031f, 1.0023f, 1.0044f, 1.0092f, 1.0218f, 1.0725f, 1.2269f },
       92.9128f,     0.020087f, 4.994f, 0.615f, 0 },   // 46.4564 * 2, 20250619
  { 4, { 0.9828f, 0.9958f, 0.9999f, 0.9979f, 0.9954f, 0.9965f, 0.998f,  1.0034f,
         1.0021f, 1.0012f, 1.0031f, 1.0073f, 1.0176f, 1.0611f, 1.113f  },
       46.46764969f, 0.020073f, 4.998f, 0.612f, 0 },   // 20250611
  { 5, { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
         1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
      -68.4264f,     0.020024f, 4.995f, 0.400f, 1 },
  { 6, { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
         1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
      -68.4264f,     0.020062f, 5.006f, 0.627f, 1 },
};
static const int LEGACY_CAL_COUNT = sizeof(LEGACY_CAL) / sizeof(LEGACY_CAL[0]);

// ==================================================================
//  DEFAULTS
// ==================================================================
// Every value here is the constant the pre-rewrite firmware used, so a unit
// running on defaults behaves exactly as it did before.  Anything that reads
// like an arbitrary number in the measurement code should be a key here.
void configDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic   = CFG_MAGIC;
  cfg.version = CFG_VERSION;

  cfg.hwRev  = 6;                  // newest board; overwritten by the legacy seed
  cfg.bridge = 1;

  for (int i = 0; i < R_CAL_BUCKETS; i++) cfg.rCal[i] = 1.0f;
  cfg.constantI = 0.02016f;
  cfg.constantR = 330.0f;
  cfg.dividerR  = 22000.0f;
  cfg.zenerMaxV = 5.0f;
  cfg.sleepV    = 0.615f;

  cfg.voltScale = 46.467649f;      // the old no-match default branch
  cfg.altMult   = 50.0f;
  cfg.thermR0   = 10000.0f;
  cfg.thermB    = 3694.0f;

  cfg.iShunt    = 0.185f;
  cfg.iZero     = 0.0f;
  cfg.iAutoZero = 1;
  cfg.iNoiseHi  = 0.01f;
  cfg.iNoiseLo  = 0.0005f;
  cfg.iDetCount = 300;
  cfg.iDetLo    = 2.3f;
  cfg.iDetHi    = 2.7f;

  cfg.rangeThreshR  = 400.0f;
  cfg.rangeDeadband = 0.05f;
  cfg.adcCountLow   = 10000;
  cfg.adcCountHigh  = 26000;
  cfg.openMargin    = 0.007f;
  cfg.openR         = 8000000.0f;
  cfg.vDispCount    = 500;
  cfg.rDispMin      = -10.0f;
  cfg.rDispMax      = 100.0f;
  cfg.zeroAutoMax   = 10.0f;

  // RATEBUMPV is an absolute 4.995 V, not zenerMaxV minus a margin.  That is
  // how the pre-rewrite code had it, and units 3-6 have zenerMaxV between
  // 4.994 and 5.006, so deriving it would quietly change their behaviour.
  cfg.rateJump   = 5.0f;
  cfg.rateBumpV  = 4.995f;
  cfg.rateSlowV  = 0.02f;
  cfg.ratePrecR  = 3000000.0f;
  cfg.rateStable = 0.02f;
  cfg.mmRMax     = 10000000.0f;
  cfg.mmRMin     = -1000.0f;

  cfg.psHoldMs  = 5000;
  cfg.psMargin  = 0.002f;
  cfg.psHyst    = 0.05f;
  cfg.psCancelR = 2000.0f;
  cfg.psPwm     = 254;
  cfg.sleepSec  = 300;             // was a literal 300000 ms
  cfg.sleepVMax = 0.1f;
  cfg.psDebug   = 1;               // was `bool psDebug = true` in the loop

  cfg.adcMs     = 1;
  cfg.battMs    = 1000;
  cfg.lcdMs     = 1000;
  cfg.lcdFastMs = 500;
  cfg.lcdBumpMs = 200;
  cfg.streamMs  = 200;

  cfg.alertsOn    = 1;
  cfg.contMin     = -20.0f;
  cfg.contMax     = 1.0f;
  cfg.contMaxHi   = 20.0f;
  cfg.vAlert      = 3.2f;
  cfg.vAlertAlt   = 30.2f;
  cfg.vacAlert    = 1.0f;
  cfg.vacAlertAlt = 15.0f;
  cfg.beepBright  = 200;
  cfg.beepHold    = 50;
  cfg.blinkLimit  = 2;
  cfg.alertPerMs   = 1000;
  cfg.alertOnMs    = 100;
  cfg.alertP2OnMs  = 300;
  cfg.alertP2OffMs = 400;

  cfg.vacThresh    = 4.9f;
  cfg.vacThreshAlt = 10.0f;
  cfg.vacAvgMax    = 0.1f;

  cfg.bridgeThr    = -0.080f;
  cfg.bridgeFltThr = -0.25f;
  cfg.bridgeAvgMax = 0.030f;
  cfg.bridgeVMax   = 0.05f;
  cfg.bridgeVacMax = 5.0f;

  cfg.vSamples    = VOLT_SAMPLE_MAX;
  cfg.smoothAlpha = 0.2f;
  cfg.battScale   = 2.0f;
  cfg.keyboardEn  = 1;
  cfg.btnLongMs   = 400;
  cfg.btnShortMs  = 50;
}

// ==================================================================
//  KEY TABLE
// ==================================================================
// Names are what the GUI and the serial protocol use.  Min/max are enforced
// on every write (fieldSet clamps), so a bad value from a host cannot put the
// measurement path into a state it cannot recover from.
const ConfigField CFG_FIELDS[] = {
  // Board / unit.  HWREV describes the unit, not a preference -- it must
  // never be copied between meters, the way BlinkyHawk treats its own HWREV.
  { "HWREV",       FT_U8,    &cfg.hwRev,          0,       255      },
  { "BRIDGE",      FT_BOOL,  &cfg.bridge,         0,       1        },
  // Resistance calibration
  { "RCAL00",      FT_FLOAT, &cfg.rCal[0],        0.1f,    10.0f    },
  { "RCAL01",      FT_FLOAT, &cfg.rCal[1],        0.1f,    10.0f    },
  { "RCAL02",      FT_FLOAT, &cfg.rCal[2],        0.1f,    10.0f    },
  { "RCAL03",      FT_FLOAT, &cfg.rCal[3],        0.1f,    10.0f    },
  { "RCAL04",      FT_FLOAT, &cfg.rCal[4],        0.1f,    10.0f    },
  { "RCAL05",      FT_FLOAT, &cfg.rCal[5],        0.1f,    10.0f    },
  { "RCAL06",      FT_FLOAT, &cfg.rCal[6],        0.1f,    10.0f    },
  { "RCAL07",      FT_FLOAT, &cfg.rCal[7],        0.1f,    10.0f    },
  { "RCAL08",      FT_FLOAT, &cfg.rCal[8],        0.1f,    10.0f    },
  { "RCAL09",      FT_FLOAT, &cfg.rCal[9],        0.1f,    10.0f    },
  { "RCAL10",      FT_FLOAT, &cfg.rCal[10],       0.1f,    10.0f    },
  { "RCAL11",      FT_FLOAT, &cfg.rCal[11],       0.1f,    10.0f    },
  { "RCAL12",      FT_FLOAT, &cfg.rCal[12],       0.1f,    10.0f    },
  { "RCAL13",      FT_FLOAT, &cfg.rCal[13],       0.1f,    10.0f    },
  { "RCAL14",      FT_FLOAT, &cfg.rCal[14],       0.1f,    10.0f    },
  { "CONSTI",      FT_FLOAT, &cfg.constantI,      0.001f,  1.0f     },
  { "CONSTR",      FT_FLOAT, &cfg.constantR,      1.0f,    100000.0f},
  { "DIVR",        FT_FLOAT, &cfg.dividerR,       1.0f,    1000000.0f},
  { "ZENERMAX",    FT_FLOAT, &cfg.zenerMaxV,      0.1f,    6.0f     },
  { "SLEEPV",      FT_FLOAT, &cfg.sleepV,         0.01f,   6.0f     },
  // Voltage calibration.  VSCALE is signed: units 5 and 6 have an inverting
  // front end and legitimately run a negative scale.
  { "VSCALE",      FT_FLOAT, &cfg.voltScale,     -500.0f,  500.0f   },
  { "ALTMULT",     FT_FLOAT, &cfg.altMult,        1.0f,    1000.0f  },
  { "THERMR0",     FT_FLOAT, &cfg.thermR0,        100.0f,  1000000.0f},
  { "THERMB",      FT_FLOAT, &cfg.thermB,         100.0f,  10000.0f },
  // Current calibration
  { "ISHUNT",      FT_FLOAT, &cfg.iShunt,         0.0001f, 100.0f   },
  { "IZERO",       FT_FLOAT, &cfg.iZero,         -6.0f,    6.0f     },
  { "IAUTOZERO",   FT_BOOL,  &cfg.iAutoZero,      0,       1        },
  { "INOISEHI",    FT_FLOAT, &cfg.iNoiseHi,       0.0f,    1.0f     },
  { "INOISELO",    FT_FLOAT, &cfg.iNoiseLo,       0.0f,    1.0f     },
  { "IDETCNT",     FT_U16,   &cfg.iDetCount,      0,       32767    },
  { "IDETLO",      FT_FLOAT, &cfg.iDetLo,         0.0f,    6.0f     },
  { "IDETHI",      FT_FLOAT, &cfg.iDetHi,         0.0f,    6.0f     },
  // Ranging
  { "RANGETHR",    FT_FLOAT, &cfg.rangeThreshR,   1.0f,    100000.0f},
  { "RANGEDB",     FT_FLOAT, &cfg.rangeDeadband,  0.0f,    0.5f     },
  { "ADCLOW",      FT_U16,   &cfg.adcCountLow,    100,     32000    },
  { "ADCHIGH",     FT_U16,   &cfg.adcCountHigh,   100,     32767    },
  { "OPENMARGIN",  FT_FLOAT, &cfg.openMargin,     0.0f,    1.0f     },
  { "OPENR",       FT_FLOAT, &cfg.openR,          1000.0f, 100000000.0f},
  { "VDISPCNT",    FT_U16,   &cfg.vDispCount,     0,       32767    },
  { "RDISPMIN",    FT_FLOAT, &cfg.rDispMin,      -10000.0f,0.0f     },
  { "RDISPMAX",    FT_FLOAT, &cfg.rDispMax,       0.0f,    1000000.0f},
  { "ZEROAUTOMAX", FT_FLOAT, &cfg.zeroAutoMax,    0.0f,    1000.0f  },
  // ADC data-rate scheduling.  RATEJUMP has a floor of 1.001 because the
  // measurement path uses both it and its reciprocal as the bounds of the
  // "unchanged" window; at exactly 1.0 that window is empty and every reading
  // would be treated as a jump.
  { "RATEJUMP",    FT_FLOAT, &cfg.rateJump,       1.001f,  1000.0f  },
  { "RATEBUMPV",   FT_FLOAT, &cfg.rateBumpV,      0.0f,    6.0f     },
  { "RATESLOWV",   FT_FLOAT, &cfg.rateSlowV,      0.0f,    1.0f     },
  { "RATEPRECR",   FT_FLOAT, &cfg.ratePrecR,      1.0f,    100000000.0f},
  { "RATESTABLE",  FT_FLOAT, &cfg.rateStable,     0.0f,    1.0f     },
  { "MMRMAX",      FT_FLOAT, &cfg.mmRMax,         0.0f,    100000000.0f},
  { "MMRMIN",      FT_FLOAT, &cfg.mmRMin,        -100000.0f, 0.0f   },
  // Power save
  { "PSHOLDMS",    FT_U16,   &cfg.psHoldMs,       0,       60000    },
  { "PSMARGIN",    FT_FLOAT, &cfg.psMargin,       0.0f,    1.0f     },
  { "PSHYST",      FT_FLOAT, &cfg.psHyst,         0.0f,    1.0f     },
  { "PSCANCELR",   FT_FLOAT, &cfg.psCancelR,      0.0f,    10000000.0f},
  { "PSPWM",       FT_U8,    &cfg.psPwm,          0,       255      },
  { "SLEEPSEC",    FT_U16,   &cfg.sleepSec,       0,       65535    },
  { "SLEEPVMAX",   FT_FLOAT, &cfg.sleepVMax,      0.0f,    1000.0f  },
  { "PSDEBUG",     FT_BOOL,  &cfg.psDebug,        0,       1        },
  // Timing.  Lower bounds of 1 ms rather than 0 -- a zero period makes the
  // matching `millis() - last >= period` test true on every pass, which is a
  // busy loop, not "as fast as possible".
  { "ADCMS",       FT_U16,   &cfg.adcMs,          1,       10000    },
  { "BATTMS",      FT_U16,   &cfg.battMs,         1,       60000    },
  { "LCDMS",       FT_U16,   &cfg.lcdMs,          1,       60000    },
  { "LCDFASTMS",   FT_U16,   &cfg.lcdFastMs,      1,       60000    },
  { "LCDBUMPMS",   FT_U16,   &cfg.lcdBumpMs,      1,       60000    },
  { "STREAMMS",    FT_U16,   &cfg.streamMs,       1,       60000    },
  // Alerts
  { "ALERTS",      FT_BOOL,  &cfg.alertsOn,       0,       1        },
  { "CONTMIN",     FT_FLOAT, &cfg.contMin,       -10000.0f,0.0f     },
  { "CONTMAX",     FT_FLOAT, &cfg.contMax,        0.0f,    100000.0f},
  { "CONTMAXHI",   FT_FLOAT, &cfg.contMaxHi,      0.0f,    100000.0f},
  { "VALERT",      FT_FLOAT, &cfg.vAlert,         0.0f,    1000.0f  },
  { "VALERTALT",   FT_FLOAT, &cfg.vAlertAlt,      0.0f,    10000.0f },
  { "VACALERT",    FT_FLOAT, &cfg.vacAlert,       0.0f,    1000.0f  },
  { "VACALERTALT", FT_FLOAT, &cfg.vacAlertAlt,    0.0f,    10000.0f },
  { "BEEPBR",      FT_U8,    &cfg.beepBright,     0,       255      },
  { "BEEPHOLD",    FT_U8,    &cfg.beepHold,       0,       255      },
  { "BLINKLIM",    FT_U8,    &cfg.blinkLimit,     0,       255      },
  { "ALERTPERMS",  FT_U16,   &cfg.alertPerMs,     10,      60000    },
  { "ALERTONMS",   FT_U16,   &cfg.alertOnMs,      0,       60000    },
  { "ALERTP2ON",   FT_U16,   &cfg.alertP2OnMs,    0,       60000    },
  { "ALERTP2OFF",  FT_U16,   &cfg.alertP2OffMs,   0,       60000    },
  // AC detection
  { "VACTHRESH",   FT_FLOAT, &cfg.vacThresh,      0.0f,    1000.0f  },
  { "VACTHRALT",   FT_FLOAT, &cfg.vacThreshAlt,   0.0f,    1000.0f  },
  { "VACAVGMAX",   FT_FLOAT, &cfg.vacAvgMax,      0.0f,    1000.0f  },
  // Float / closed bridge
  { "BRIDGETHR",   FT_FLOAT, &cfg.bridgeThr,     -6.0f,    6.0f     },
  { "BRIDGEFLT",   FT_FLOAT, &cfg.bridgeFltThr,  -6.0f,    6.0f     },
  { "BRIDGEAVG",   FT_FLOAT, &cfg.bridgeAvgMax,   0.0f,    10.0f    },
  { "BRIDGEVMAX",  FT_FLOAT, &cfg.bridgeVMax,     0.0f,    10.0f    },
  { "BRIDGEVAC",   FT_FLOAT, &cfg.bridgeVacMax,   0.0f,    1000.0f  },
  // Filtering / misc
  { "VSAMPLES",    FT_U8,    &cfg.vSamples,       1,       VOLT_SAMPLE_MAX },
  { "SMOOTHA",     FT_FLOAT, &cfg.smoothAlpha,    0.001f,  1.0f     },
  { "BATTSCALE",   FT_FLOAT, &cfg.battScale,      0.1f,    100.0f   },
  { "KEYBOARD",    FT_BOOL,  &cfg.keyboardEn,     0,       1        },
  { "BTNLONGMS",   FT_U16,   &cfg.btnLongMs,      50,      10000    },
  { "BTNSHORTMS",  FT_U16,   &cfg.btnShortMs,     0,       5000     },
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
    default:       return *(uint8_t *)f->ptr;    // FT_U8 / FT_BOOL
  }
}

void fieldSet(const ConfigField *f, float v) {
  if (v < f->minV) v = f->minV;
  if (v > f->maxV) v = f->maxV;
  switch (f->type) {
    case FT_FLOAT: *(float *)f->ptr    = v;                     break;
    case FT_U16:   *(uint16_t *)f->ptr = (uint16_t)(v + 0.5f);  break;
    case FT_BOOL:  *(uint8_t *)f->ptr  = (v != 0.0f) ? 1 : 0;   break;
    default:       *(uint8_t *)f->ptr  = (uint8_t)(v + 0.5f);   break;
  }
}

// Floats get 6 decimals: the resistance correction factors and constantI are
// meaningful to the fifth or sixth place, and a value that does not survive a
// !CFG -> edit -> !SET round trip is worse than useless for calibration.
void printField(const ConfigField *f) {
  Serial.print("$CFG,");
  Serial.print(f->name);
  Serial.print(",");
  if (f->type == FT_FLOAT) Serial.println(fieldGet(f), 6);
  else                     Serial.println((long)fieldGet(f));
}

// ==================================================================
//  PERSISTENCE
// ==================================================================
uint16_t crc16_ccitt(const uint8_t *p, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= (uint16_t)(*p++) << 8;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// offsetof, NOT sizeof-2: tail padding follows crc in a 4-byte-aligned struct,
// so the sizeof form would checksum the crc field itself and fail every time.
static uint16_t configCrc() {
  return crc16_ccitt((const uint8_t *)&cfg, offsetof(Config, crc));
}

void configSave() {
  cfg.magic   = CFG_MAGIC;
  cfg.version = CFG_VERSION;
  cfg.crc     = configCrc();
  EEPROM.put(CFG_EEPROM_ADDR, cfg);
  cfgDirty = false;
}

bool configLoad() {
  Config stored;
  EEPROM.get(CFG_EEPROM_ADDR, stored);
  if (stored.magic   != CFG_MAGIC)   return false;
  if (stored.version != CFG_VERSION) return false;
  Config keep = cfg;
  cfg = stored;
  if (configCrc() != stored.crc) { cfg = keep; return false; }
  cfgDirty = false;
  return true;
}

// ==================================================================
//  LEGACY SEED
// ==================================================================
// Copy one of the six original calibration sets into the live config.
// Returns false for an id with no table entry (the old code's else branch --
// unity factors and the default voltage scale -- which configDefaults()
// already produced, so the caller simply keeps those).
bool configSeedFromLegacy(uint8_t id) {
  for (int i = 0; i < LEGACY_CAL_COUNT; i++) {
    if (LEGACY_CAL[i].id != id) continue;
    const LegacyCal &L = LEGACY_CAL[i];
    for (int k = 0; k < R_CAL_BUCKETS; k++) cfg.rCal[k] = L.cf[k];
    cfg.voltScale = L.voltScale;
    cfg.constantI = L.constantI;
    cfg.zenerMaxV = L.zenerMaxV;
    cfg.sleepV    = L.sleepV;
    cfg.bridge    = L.bridge;
    cfg.hwRev     = id;
    return true;
  }
  return false;
}

// ==================================================================
//  SERIAL NUMBER
// ==================================================================
// Explicit prototype: the generated one would be hoisted above SerialId.
uint16_t snCrc(const SerialId &s);

uint16_t snCrc(const SerialId &s) {
  return crc16_ccitt((const uint8_t *)&s, offsetof(SerialId, crc));
}

void snLoad() {
  SerialId s;
  EEPROM.get(SN_EEPROM_ADDR, s);
  if (s.magic == SN_MAGIC && snCrc(s) == s.crc) {
    s.sn[SN_MAX_LEN - 1] = '\0';
    strncpy(unitSN, s.sn, SN_MAX_LEN);
    unitSN[SN_MAX_LEN - 1] = '\0';
  } else {
    unitSN[0] = '\0';
  }
}

void snSave(const char *sn) {
  SerialId s;
  memset(&s, 0, sizeof(s));
  s.magic = SN_MAGIC;
  strncpy(s.sn, sn, SN_MAX_LEN - 1);
  s.crc = snCrc(s);
  EEPROM.put(SN_EEPROM_ADDR, s);
  strncpy(unitSN, s.sn, SN_MAX_LEN);
  unitSN[SN_MAX_LEN - 1] = '\0';
}

// ==================================================================
//  BOOT
// ==================================================================
// Order matters.  Defaults first so every field has a sane value even if the
// stored image is only partly usable; then the stored config; and only if
// there is none, the legacy byte-1 seed.  A unit that has been configured
// through this firmware never re-reads byte 1 again -- its saved config wins.
void configSetup() {
  configDefaults();

  if (configLoad()) {
    Serial.print(F("Config loaded, v"));
    Serial.println(cfg.version);
  } else {
    uint8_t legacyId = EEPROM.read(LEGACY_ID_ADDR);
    bool seeded = configSeedFromLegacy(legacyId);
    configSave();
    Serial.print(F("Config seeded from "));
    if (seeded) {
      Serial.print(F("legacy unit #"));
      Serial.println(legacyId);
    } else {
      Serial.print(F("defaults (no legacy set for byte1="));
      Serial.print(legacyId);
      Serial.println(')');
    }
  }

  snLoad();
}
