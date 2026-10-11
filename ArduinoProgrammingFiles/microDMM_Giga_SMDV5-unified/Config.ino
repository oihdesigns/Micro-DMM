/*
 * Config.ino -- flash-backed configuration.
 *
 * Replaces the compiled-in constants (CF_A..CF_O, constantI, EEPROM_MAXV,
 * EEPROM_SleepV, VOLTAGE_SCALE...) with a versioned, CRC-checked struct in
 * QSPI flash, addressable by name over serial (!SET / !GET / !CFG / !SAVE).
 * One build runs every GigaMeter: each unit's numbers live on that unit.
 *
 * Flash is written in exactly two places: configSave() (!SAVE) and snSave()
 * (!SN).  Everything else edits RAM.
 */

Config cfg;
bool   cfgDirty  = false;
bool   cfgStored = false;      // a valid image was loaded from flash this boot
char   unitSN[SN_MAX_LEN] = "";

static_assert(sizeof(Config) <= FLASH_BUF_MAX, "Config outgrew the flash buffer");
static_assert(sizeof(SerialId) <= FLASH_BUF_MAX, "SerialId outgrew the flash buffer");

// ==================================================================
//  RESISTANCE CALIBRATION LADDER
// ==================================================================
// Upper edge of each rCal bucket -- the breakpoints of the old CF_A..CF_O
// if/else chain, in the same order.  14 edges -> 15 buckets.
const float R_CAL_EDGES[R_CAL_BUCKETS - 1] = {
  0.75f, 3.0f, 7.0f, 20.0f, 70.0f, 170.0f, 700.0f, 1700.0f,
  7000.0f, 17000.0f, 70000.0f, 170000.0f, 700000.0f, 1700000.0f
};

// Which correction factor applies to a raw resistance.  !CAL uses the same
// function so a calibration point lands in the bucket it will be applied from.
uint8_t rCalIndex(float raw) {
  for (uint8_t i = 0; i < R_CAL_BUCKETS - 1; i++)
    if (raw < R_CAL_EDGES[i]) return i;
  return R_CAL_BUCKETS - 1;
}

// ==================================================================
//  PER-UNIT STARTING POINTS
// ==================================================================
static const LegacyCal UNIT_CAL[] = {
  // Unit 1 -- calibrated 25/08/13 (the values microDMM_Giga_SMDV5 shipped with)
  { 1, { 0.9607f, 0.9848f, 0.997f,  0.9958f, 0.9934f, 0.9965f, 0.9982f, 1.0028f,
         1.0012f, 1.0005f, 1.0017f, 1.0035f, 1.0075f, 1.0237f, 1.1226f },
       0.02005f, 4.972f, 0.6117f, 68.5289f, 68.6692f },
  // Unit 2 -- NOT YET CALIBRATED.  Unity ladder; the references and scales are
  // unit 1's as a starting point.  Measure ZENERMAX and SLEEPV first.
  { 2, { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
         1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f },
       0.02005f, 4.972f, 0.6117f, 68.5289f, 68.6692f },
};
static const int UNIT_CAL_COUNT = sizeof(UNIT_CAL) / sizeof(UNIT_CAL[0]);

// ==================================================================
//  DEFAULTS
// ==================================================================
// Every value is the constant the pre-config Giga firmware used, so a unit on
// defaults behaves as it did before.  The calibration fields are unit 1's
// (configSeedFromLegacy(1)), but hwRev stays 0 -- "never assigned" -- so a
// unit running on them says so instead of passing as unit 1.
void configDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic   = CFG_MAGIC;
  cfg.version = CFG_VERSION;

  cfg.bridge = 1;

  cfg.constantR = 330.0f;
  cfg.dividerR  = 22000.0f;
  cfg.voltOffset = 0.0f;
  cfg.altMult   = 50.0f;
  cfg.thermR0   = 10000.0f;
  cfg.thermB    = 3694.0f;
  configSeedFromLegacy(1);         // rCal, constantI, zenerMaxV, sleepV, scales
  cfg.hwRev = 0;

  cfg.iShunt      = 0.185f;
  cfg.iZero       = 0.0f;
  cfg.iAutoZero   = 1;
  cfg.iNoiseHi    = 0.01f;
  cfg.iNoiseLo    = 0.0005f;
  cfg.iDetCount   = 300;
  cfg.iDetLo      = 1.5f;          // the Giga's original 1.5..3.5 V window
  cfg.iDetHi      = 3.5f;
  cfg.iDetSamples = 8;
  cfg.iDetSettleMs = 50;
  cfg.iDetPPV     = 0.05f;
  cfg.iShuntR     = 1.0f;

  cfg.rangeThreshR  = 400.0f;
  cfg.rangeDeadband = 0.05f;
  cfg.adcCountLow   = 10000;
  cfg.adcCountHigh  = 30000;
  cfg.openMargin    = 0.007f;
  cfg.openR         = 8000000.0f;
  cfg.vDispCount    = 500;
  cfg.rDispMin      = -10.0f;
  cfg.rDispMax      = 100.0f;
  cfg.zeroAutoMax   = 10.0f;

  cfg.rateJump   = 5.0f;
  cfg.rateBumpV  = 4.995f;
  cfg.rateSlowV  = 0.02f;
  cfg.ratePrecR  = 3000000.0f;
  cfg.rateStable = 0.02f;
  cfg.mmRMax     = 10000000.0f;
  cfg.mmRMin     = -1000.0f;

  cfg.psHoldMs  = 5000;
  cfg.psMargin  = 0.002f;
  cfg.psHyst    = 0.01f;
  cfg.psCancelR = 20000.0f;
  cfg.psPwm     = 254;
  cfg.sleepSec  = 300;
  cfg.sleepVMax = 0.1f;
  cfg.psDebug   = 0;

  cfg.adcMs     = 1;
  cfg.lcdMs     = 1000;
  cfg.lcdFastMs = 500;
  cfg.lcdBumpMs = 200;
  cfg.streamMs  = 200;

  cfg.alertsOn     = 1;
  cfg.contMin      = -20.0f;
  cfg.contMax      = 1.0f;
  cfg.contMaxHi    = 20.0f;
  cfg.vAlert       = 3.2f;
  cfg.vAlertAlt    = 30.2f;
  cfg.vacAlert     = 1.0f;
  cfg.vacAlertAlt  = 15.0f;
  cfg.beepBright   = 200;
  cfg.beepHold     = 50;
  cfg.blinkLimit   = 2;
  cfg.alertPerMs   = 1000;
  cfg.alertOnMs    = 100;
  cfg.alertP2OnMs  = 300;
  cfg.alertP2OffMs = 400;

  cfg.vacThresh    = 4.9f;
  cfg.vacThreshAlt = 10.0f;
  cfg.vacAvgMax    = 0.1f;

  cfg.bridgeClsV   = 0.03f;
  cfg.bridgeOpenV  = 0.05f;
  cfg.bridgeAvgMax = 0.03f;
  cfg.bridgeVMax   = 0.2f;

  cfg.vSamples    = VOLT_SAMPLE_MAX;
  cfg.smoothAlpha = 0.2f;
  cfg.keyboardEn  = 1;
  cfg.btnLongMs   = 400;
  cfg.btnShortMs  = 50;

  cfg.micEn     = 1;
  cfg.micRefMax = 60;
  cfg.micMMLo   = 100;
  cfg.micMMHi   = 275;
  cfg.micModeLo = 300;
  cfg.micModeHi = 450;

  cfg.humMin = 20.0f;
  cfg.humMax = 1000000.0f;
}

// ==================================================================
//  KEY TABLE
// ==================================================================
// Names match the XIAO firmware wherever the meaning matches, so the GUI's
// descriptions apply.  Min/max are enforced on every write.
const ConfigField CFG_FIELDS[] = {
  // Unit.  HWREV is which GigaMeter this is -- never copy it between meters.
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
  // Voltage calibration
  { "VSCALE",      FT_FLOAT, &cfg.voltScale,     -500.0f,  500.0f   },
  { "VSCALENEG",   FT_FLOAT, &cfg.voltScaleNeg,  -500.0f,  500.0f   },
  { "VOFFSET",     FT_FLOAT, &cfg.voltOffset,    -10.0f,   10.0f    },
  { "ALTMULT",     FT_FLOAT, &cfg.altMult,        1.0f,    1000.0f  },
  { "THERMR0",     FT_FLOAT, &cfg.thermR0,        100.0f,  1000000.0f},
  { "THERMB",      FT_FLOAT, &cfg.thermB,         100.0f,  10000.0f },
  // Current
  { "ISHUNT",      FT_FLOAT, &cfg.iShunt,         0.0001f, 100.0f   },
  { "IZERO",       FT_FLOAT, &cfg.iZero,         -6.0f,    6.0f     },
  { "IAUTOZERO",   FT_BOOL,  &cfg.iAutoZero,      0,       1        },
  { "INOISEHI",    FT_FLOAT, &cfg.iNoiseHi,       0.0f,    1.0f     },
  { "INOISELO",    FT_FLOAT, &cfg.iNoiseLo,       0.0f,    1.0f     },
  { "IDETCNT",     FT_U16,   &cfg.iDetCount,      0,       32767    },
  { "IDETLO",      FT_FLOAT, &cfg.iDetLo,         0.0f,    6.0f     },
  { "IDETHI",      FT_FLOAT, &cfg.iDetHi,         0.0f,    6.0f     },
  { "IDETSAMP",    FT_U8,    &cfg.iDetSamples,    1,       64       },
  { "IDETSETTLE",  FT_U16,   &cfg.iDetSettleMs,   0,       5000     },
  { "IDETPP",      FT_FLOAT, &cfg.iDetPPV,        0.0f,    6.0f     },
  { "ISHUNTR",     FT_FLOAT, &cfg.iShuntR,        0.0001f, 10000.0f },
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
  // ADC data-rate scheduling.  RATEJUMP floors at 1.001: at exactly 1.0 the
  // "unchanged" window is empty and every reading counts as a jump.
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
  // Timing.  Floors of 1 ms: a zero period is a busy loop.
  { "ADCMS",       FT_U16,   &cfg.adcMs,          1,       10000    },
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
  { "BRIDGECLS",   FT_FLOAT, &cfg.bridgeClsV,     0.0f,    1.0f     },
  { "BRIDGEOPEN",  FT_FLOAT, &cfg.bridgeOpenV,    0.0f,    1.0f     },
  { "BRIDGEAVG",   FT_FLOAT, &cfg.bridgeAvgMax,   0.0f,    10.0f    },
  { "BRIDGEVMAX",  FT_FLOAT, &cfg.bridgeVMax,     0.0f,    10.0f    },
  // Filtering / input
  { "VSAMPLES",    FT_U8,    &cfg.vSamples,       1,       VOLT_SAMPLE_MAX },
  { "SMOOTHA",     FT_FLOAT, &cfg.smoothAlpha,    0.001f,  1.0f     },
  { "KEYBOARD",    FT_BOOL,  &cfg.keyboardEn,     0,       1        },
  { "BTNLONGMS",   FT_U16,   &cfg.btnLongMs,      50,      10000    },
  { "BTNSHORTMS",  FT_U16,   &cfg.btnShortMs,     0,       5000     },
  // Mic-jack ladder on A7 (10-bit counts)
  { "MICEN",       FT_BOOL,  &cfg.micEn,          0,       1        },
  { "MICREFMAX",   FT_U16,   &cfg.micRefMax,      0,       1023     },
  { "MICMMLO",     FT_U16,   &cfg.micMMLo,        0,       1023     },
  { "MICMMHI",     FT_U16,   &cfg.micMMHi,        0,       1023     },
  { "MICMODELO",   FT_U16,   &cfg.micModeLo,      0,       1023     },
  { "MICMODEHI",   FT_U16,   &cfg.micModeHi,      0,       1023     },
  // M4 audio
  { "HUMMIN",      FT_FLOAT, &cfg.humMin,         0.0f,    100000000.0f},
  { "HUMMAX",      FT_FLOAT, &cfg.humMax,         0.0f,    100000000.0f},
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

// 6 decimals: the correction factors and constantI matter to the fifth or
// sixth place, and must survive a !CFG -> edit -> !SET round trip.
void printField(const ConfigField *f) {
  Host.print("$CFG,");
  Host.print(f->name);
  Host.print(",");
  if (f->type == FT_FLOAT) Host.println(fieldGet(f), 6);
  else                     Host.println((long)fieldGet(f));
}

// ==================================================================
//  QSPI FLASH
// ==================================================================
static mbed::BlockDevice *flashDev = NULL;
static bool flashOk = false;
static uint8_t flashBuf[FLASH_BUF_MAX];

static uint32_t le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// The window is only ours if no MBR partition reaches into it.  A board
// formatted with some other layout gets a refusal, not a corrupted partition.
static bool flashRegionFree() {
  uint8_t mbr[512];
  if (flashDev->read(mbr, 0, sizeof(mbr)) != 0) return false;
  if (mbr[510] != 0x55 || mbr[511] != 0xAA) return true;   // no partition table
  for (int i = 0; i < 4; i++) {
    const uint8_t *e = mbr + 446 + 16 * i;
    if (e[4] == 0) continue;                                // unused entry
    uint64_t start = (uint64_t)le32(e + 8) * 512ULL;
    uint64_t end   = start + (uint64_t)le32(e + 12) * 512ULL;
    if (start < FLASH_REGION_END && end > FLASH_CFG_ADDR) {
      Host.print(F("$ERR,flash,partition "));
      Host.print(i + 1);
      Host.println(F(" overlaps the config window -- saving disabled"));
      return false;
    }
  }
  return true;
}

void flashSetup() {
  flashDev = mbed::BlockDevice::get_default_instance();
  if (!flashDev || flashDev->init() != 0) {
    Host.println(F("$ERR,flash,QSPI init failed -- saving disabled"));
    flashOk = false;
    return;
  }
  flashOk = flashRegionFree();
}

bool flashAvailable() { return flashOk; }

static size_t roundUp(size_t n, size_t unit) {
  if (unit <= 1) return n;
  return ((n + unit - 1) / unit) * unit;
}

static bool flashRead(uint32_t addr, void *data, size_t len) {
  if (!flashOk) return false;
  size_t padded = roundUp(len, flashDev->get_read_size());
  if (padded > sizeof(flashBuf)) return false;
  if (flashDev->read(flashBuf, addr, padded) != 0) return false;
  memcpy(data, flashBuf, len);
  return true;
}

// One erase unit per block, erased then programmed whole.
static bool flashWrite(uint32_t addr, const void *data, size_t len) {
  if (!flashOk) return false;
  size_t padded = roundUp(len, flashDev->get_program_size());
  if (padded > sizeof(flashBuf)) return false;
  memset(flashBuf, 0xFF, padded);
  memcpy(flashBuf, data, len);
  if (flashDev->erase(addr, flashDev->get_erase_size(addr)) != 0) return false;
  return flashDev->program(flashBuf, addr, padded) == 0;
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

static uint16_t configCrcOf(const Config &c) {
  return crc16_ccitt((const uint8_t *)&c, offsetof(Config, crc));
}

// Writes, reads back and compares.  False on any failure, including a board
// where saving is disabled.
bool configSave() {
  cfg.magic   = CFG_MAGIC;
  cfg.version = CFG_VERSION;
  cfg.crc     = configCrcOf(cfg);
  if (!flashWrite(FLASH_CFG_ADDR, &cfg, sizeof(cfg))) return false;
  Config check;
  if (!flashRead(FLASH_CFG_ADDR, &check, sizeof(check))) return false;
  if (memcmp(&check, &cfg, sizeof(Config)) != 0) return false;
  cfgDirty  = false;
  cfgStored = true;
  return true;
}

bool configLoad() {
  Config stored;
  if (!flashRead(FLASH_CFG_ADDR, &stored, sizeof(stored))) return false;
  if (stored.magic   != CFG_MAGIC)   return false;
  if (stored.version != CFG_VERSION) return false;
  if (configCrcOf(stored) != stored.crc) return false;
  cfg = stored;
  cfgDirty  = false;
  cfgStored = true;
  return true;
}

// ==================================================================
//  PER-UNIT SEED
// ==================================================================
bool configSeedFromLegacy(uint8_t id) {
  for (int i = 0; i < UNIT_CAL_COUNT; i++) {
    if (UNIT_CAL[i].id != id) continue;
    const LegacyCal &L = UNIT_CAL[i];
    for (int k = 0; k < R_CAL_BUCKETS; k++) cfg.rCal[k] = L.cf[k];
    cfg.constantI    = L.constantI;
    cfg.zenerMaxV    = L.zenerMaxV;
    cfg.sleepV       = L.sleepV;
    cfg.voltScale    = L.voltScale;
    cfg.voltScaleNeg = L.voltScaleNeg;
    cfg.hwRev        = id;
    return true;
  }
  return false;
}

// ==================================================================
//  SERIAL NUMBER
// ==================================================================
uint16_t snCrc(const SerialId &s);

uint16_t snCrc(const SerialId &s) {
  return crc16_ccitt((const uint8_t *)&s, offsetof(SerialId, crc));
}

void snLoad() {
  SerialId s;
  unitSN[0] = '\0';
  if (!flashRead(FLASH_SN_ADDR, &s, sizeof(s))) return;
  if (s.magic == SN_MAGIC && snCrc(s) == s.crc) {
    s.sn[SN_MAX_LEN - 1] = '\0';
    strncpy(unitSN, s.sn, SN_MAX_LEN);
    unitSN[SN_MAX_LEN - 1] = '\0';
  }
}

bool snSave(const char *sn) {
  SerialId s;
  memset(&s, 0, sizeof(s));
  s.magic = SN_MAGIC;
  strncpy(s.sn, sn, SN_MAX_LEN - 1);
  s.crc = snCrc(s);
  if (!flashWrite(FLASH_SN_ADDR, &s, sizeof(s))) return false;
  strncpy(unitSN, s.sn, SN_MAX_LEN);
  unitSN[SN_MAX_LEN - 1] = '\0';
  return true;
}

// ==================================================================
//  BOOT
// ==================================================================
// Defaults first so every field is sane; then the stored image if there is
// one.  With none, the unit runs on defaults (unit 1's calibration, HWREV 0)
// and stays flagged unsaved until someone assigns it: !SEEDCAL,<n> then !SAVE.
void configSetup() {
  configDefaults();
  flashSetup();

  if (configLoad()) {
    Host.print(F("$INFO,boot,config,loaded,unit "));
    Host.println(cfg.hwRev);
  } else {
    cfgDirty = true;
    Host.println(F("$INFO,boot,config,none stored -- running defaults; "
                     "assign this unit with !SEEDCAL,<n> then !SAVE"));
  }

  snLoad();
}
