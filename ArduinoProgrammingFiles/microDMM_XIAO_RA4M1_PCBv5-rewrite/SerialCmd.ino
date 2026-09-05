/*
 * SerialCmd.ino -- the host protocol.
 *
 * A clean break from the single-character commands the pre-rewrite firmware
 * used.  Those had no arguments, no acknowledgement, and shared the wire with
 * free-text debug prints, so a host could only ever guess what had happened.
 *
 *   IN   !CMD            or  !CMD,<arg>  or  !SET,<key>,<value>
 *   OUT  $TYPE,<fields...>                one record per line, comma separated
 *
 * Every line the firmware emits starts with '$'.  Anything that does not is
 * boot noise or a library print, and a host should ignore it rather than try
 * to parse it.
 *
 * Records:
 *   $LIVE,ms,R,Rz,V,Vavg,Vac,I,Vr,Vbat,mode,flags
 *   $MINMAX,Vlo,Vhi,Rlo,Rhi,Ilo,Ihi,tVlo,tVhi
 *   $STATUS,key=value,...
 *   $CFG,<key>,<value>  ... terminated by $CFGEND
 *   $SN,<value>
 *   $IDET,<state>,<meanCounts>,<meanV>,<ppV>,<atGnd>,<atMid>,<steady>,<izero>
 *   $CAL,<index>,<raw>,<actual>,<factor>
 *   $LOGSTART,<n> / $LOG,<i>,<t>,<V>,<I> / $LOGEND
 *   $OK,<what> / $ERR,<what>,<why> / $INFO,<what>,...
 *
 * $LIVE flag bits (bit 0 = LSB):
 *   0 voltageDisplay   4 Vzero        8  MinMaxDisplay  12 ampsMode
 *   1 powerSave        5 continuity   9  screenSleep    13 resistance open
 *   2 VACPresense      6 ohmsHighRange 10 preciseMode   14 currentOnOff
 *   3 vFloating        7 ohmsAutoRange 11 altUnits      15 config dirty
 *   16 resistance measured this pass   17 ADS in continuous conversion
 *   18 ohms source parked (the 20 mA LM317 is off)
 *   19 low-power state held by hand (!LOWPWR)
 *   20 wake condition met right now (parked rail is sagging)
 */

#define CMD_BUF_LEN 64
static char          cmdBuf[CMD_BUF_LEN];
static uint8_t       cmdLen        = 0;
static unsigned long lastStreamMs  = 0;
static unsigned long lastMinMaxMs  = 0;

// ==================================================================
//  EMITTERS
// ==================================================================
static uint32_t liveFlags() {
  uint32_t f = 0;
  if (voltageDisplay)  f |= 1u << 0;
  if (powerSave)       f |= 1u << 1;
  if (VACPresense)     f |= 1u << 2;
  if (vFloating)       f |= 1u << 3;
  if (Vzero)           f |= 1u << 4;
  if (continuity)      f |= 1u << 5;
  if (ohmsHighRange)   f |= 1u << 6;
  if (ohmsAutoRange)   f |= 1u << 7;
  if (MinMaxDisplay)   f |= 1u << 8;
  if (screenSleep)     f |= 1u << 9;
  if (preciseMode)     f |= 1u << 10;
  if (altUnits)        f |= 1u << 11;
  if (ampsMode)        f |= 1u << 12;
  if (ohmsVoltage > (zenerActiveV - cfg.openMargin)) f |= 1u << 13;
  if (currentOnOff)    f |= 1u << 14;
  if (cfgDirty)        f |= 1u << 15;
  // Bits 16+ needed the word widened past 16.  A host reading this as a plain
  // integer keeps working; only code that masked it to 16 bits would care.
  if (resistanceMeasured) f |= 1uL << 16;
  if (adsContinuous())    f |= 1uL << 17;
  if (ohmsParked)         f |= 1uL << 18;
  if (forceLowPower)      f |= 1uL << 19;
  // Whether the sag that ends power save is satisfied AT THIS INSTANT,
  // reported whether or not it is being acted on.  With !LOWPWR holding the
  // state, this is what makes SLEEPV and PSHYST tunable: connect something
  // and watch whether the wake would have fired, and at what rail voltage,
  // without the meter waking up and destroying the measurement.
  if (ohmsParked && ohmsVoltage < cfg.sleepV - cfg.psHyst)
    f |= 1uL << 20;
  return f;
}

// One fixed-shape line whatever the meter is doing, so the host never has to
// branch on mode to know what it is reading.  Resistance carries 3 decimals
// because sub-ohm lead nulling is the case that needs them.
void emitLive() {
  Serial.print(F("$LIVE,"));
  Serial.print(millis());              Serial.print(',');
  Serial.print(currentResistance, 3);  Serial.print(',');
  Serial.print(displayResistance, 3);  Serial.print(',');
  Serial.print(newVoltageReading, 6);  Serial.print(',');
  Serial.print(averageVoltage, 6);     Serial.print(',');
  Serial.print(VAC, 4);                Serial.print(',');
  Serial.print(Ireading, 4);           Serial.print(',');
  Serial.print(ohmsVoltage, 5);        Serial.print(',');
  Serial.print(batteryVoltage, 3);     Serial.print(',');
  Serial.print(currentMode);           Serial.print(',');
  Serial.println(liveFlags());
}

void emitMinMax() {
  Serial.print(F("$MINMAX,"));
  Serial.print(lowV, 6);   Serial.print(',');
  Serial.print(highV, 6);  Serial.print(',');
  Serial.print(lowR, 3);   Serial.print(',');
  Serial.print(highR, 3);  Serial.print(',');
  Serial.print(ILow, 4);   Serial.print(',');
  Serial.print(IHigh, 4);  Serial.print(',');
  Serial.print(timeAtMinV); Serial.print(',');
  Serial.println(timeAtMaxV);
}

void emitStatus() {
  Serial.print(F("$STATUS,mode="));   Serial.print(currentMode);
  Serial.print(F(",range="));         Serial.print(ohmsHighRange ? "high" : "low");
  Serial.print(F(",auto="));          Serial.print(ohmsAutoRange ? 1 : 0);
  Serial.print(F(",ps="));            Serial.print(powerSave ? 1 : 0);
  Serial.print(F(",zero="));          Serial.print(zeroOffsetRes, 4);
  Serial.print(F(",bridge="));        Serial.print(cfg.bridge ? 1 : 0);
  Serial.print(F(",stream="));        Serial.print(serialMode ? 1 : 0);
  Serial.print(F(",debug="));         Serial.print(debugMode ? 1 : 0);
  Serial.print(F(",amps="));          Serial.print(currentOnOff ? 1 : 0);
  Serial.print(F(",irange="));        Serial.print(Irange ? "high" : "low");
  Serial.print(F(",rmeas="));         Serial.print(resistanceMeasured ? 1 : 0);
  Serial.print(F(",cont="));          Serial.print(adsContinuous() ? 1 : 0);
  Serial.print(F(",ohmspark="));      Serial.print(ohmsParked ? 1 : 0);
  Serial.print(F(",lowpwr="));        Serial.print(forceLowPower ? 1 : 0);
  Serial.print(F(",dirty="));         Serial.print(cfgDirty ? 1 : 0);
  Serial.print(F(",hwrev="));         Serial.print(cfg.hwRev);
  Serial.print(F(",sn="));            Serial.println(unitSN);
}

static void emitLog() {
  Serial.print(F("$LOGSTART,"));
  Serial.print(LOG_SIZE);       Serial.print(',');
  Serial.println(tLogStart, 3);
  for (int i = 0; i < LOG_SIZE; i++) {
    Serial.print(F("$LOG,"));
    Serial.print(i);                     Serial.print(',');
    Serial.print(loggedTimeStamps[i], 3);Serial.print(',');
    Serial.print(loggedVoltagesAtI[i], 4);Serial.print(',');
    Serial.println(loggedCurrents[i], 4);
  }
  Serial.print(F("$LOGEND,"));
  Serial.println(tLogEnd, 3);
}

static void emitHelp() {
  Serial.println(F("$INFO,help,config,!CFG !GET,K !SET,K,V !SAVE !LOAD !DEFAULTS !SEEDCAL,N !SN[,V]"));
  Serial.println(F("$INFO,help,live,!READ !STREAM[,0|1] !RATE,MS !STATUS !MINMAX[,0|1] !RESET"));
  Serial.println(F("$INFO,help,meter,!ZERO !ZEROCLR !MODE,N !RANGE,0|1|A !VDISP[,0|1] !LOWPWR[,0|1]"));
  Serial.println(F("$INFO,help,cal,!CAL,OHMS[,IDX] !CALV,VOLTS !CALI !IDET !DEBUG !AMPS !FAST !LOG !DUMP"));
}

// ==================================================================
//  CALIBRATION HELPERS
// ==================================================================
// Correction factors multiply the raw reading, so the factor that makes the
// present raw reading land on a known reference is actual/raw.  The bucket is
// chosen from the same raw value the measurement path would use, so a cal
// point always lands in the bucket it will later be applied from.
static void calResistance(float actual, int forcedIdx) {
  // Same reason as !ZERO: calibrating against a held reading would write a
  // correction factor derived from a measurement that never happened.
  if (!resistanceMeasured) {
    Serial.println(F("$ERR,cal,resistance not being measured"));
    return;
  }
  if (rawResistance <= 0.0f) {
    Serial.println(F("$ERR,cal,no raw reading"));
    return;
  }
  if (actual <= 0.0f) {
    Serial.println(F("$ERR,cal,actual must be positive"));
    return;
  }
  int idx = (forcedIdx >= 0) ? forcedIdx : rCalIndex(rawResistance);
  if (idx >= R_CAL_BUCKETS) {
    Serial.println(F("$ERR,cal,index out of range"));
    return;
  }
  float factor = actual / rawResistance;
  char key[8];
  snprintf(key, sizeof(key), "RCAL%02d", idx);
  const ConfigField *f = findField(key);
  if (!f) { Serial.println(F("$ERR,cal,no such bucket")); return; }
  fieldSet(f, factor);
  cfgDirty = true;

  Serial.print(F("$CAL,"));
  Serial.print(idx);                Serial.print(',');
  Serial.print(rawResistance, 4);   Serial.print(',');
  Serial.print(actual, 4);          Serial.print(',');
  Serial.println(fieldGet(f), 6);   // echo the clamped value, not the request
}

static void calVoltage(float actual) {
  if (fabs(newVoltageReading) < 1e-6f) {
    Serial.println(F("$ERR,calv,reading too small to scale"));
    return;
  }
  // voltScale is a linear gain, so scaling it by actual/present moves the
  // present reading onto the reference without touching anything else.
  float scale = cfg.voltScale * (actual / newVoltageReading);
  const ConfigField *f = findField("VSCALE");
  fieldSet(f, scale);
  cfgDirty = true;
  Serial.print(F("$CAL,VSCALE,"));
  Serial.print(newVoltageReading, 6); Serial.print(',');
  Serial.print(actual, 6);            Serial.print(',');
  Serial.println(cfg.voltScale, 6);
}

// ==================================================================
//  COMMAND DISPATCH
// ==================================================================
// Explicit prototype: the generated one would be hoisted above ConfigField.
static void cmdSet(char *arg);

static void cmdSet(char *arg) {
  char *val = arg ? strchr(arg, ',') : NULL;
  if (!arg || !val) { Serial.println(F("$ERR,set,usage !SET,<key>,<value>")); return; }
  *val = '\0'; val++;
  for (char *p = arg; *p; ++p) *p = toupper(*p);

  const ConfigField *f = findField(arg);
  if (!f) { Serial.print(F("$ERR,set,unknown key ")); Serial.println(arg); return; }

  fieldSet(f, atof(val));

  // Resizing the rolling voltage window invalidates the running sums, which
  // still hold samples from outside the new window.
  if (strcmp(f->name, "VSAMPLES") == 0) resetVoltageFilter();
  // zenerActiveV tracks whichever reference is in force; a changed ceiling
  // must reach it now rather than at the next power-save transition.
  if (strcmp(f->name, "ZENERMAX") == 0 && !powerSave) zenerActiveV = cfg.zenerMaxV;

  cfgDirty = true;
  printField(f);                       // echo the possibly clamped value
}

void handleLine(char *line) {
  if (line[0] != '!') return;          // not addressed to us
  char *cmd = line + 1;
  char *arg = strchr(cmd, ',');
  if (arg) { *arg = '\0'; arg++; }
  for (char *p = cmd; *p; ++p) *p = toupper(*p);

  // ---- Configuration -----------------------------------------
  if (strcmp(cmd, "SET") == 0) {
    cmdSet(arg);

  } else if (strcmp(cmd, "GET") == 0) {
    if (!arg) { Serial.println(F("$ERR,get,usage !GET,<key>")); return; }
    for (char *p = arg; *p; ++p) *p = toupper(*p);
    const ConfigField *f = findField(arg);
    if (!f) { Serial.print(F("$ERR,get,unknown key ")); Serial.println(arg); return; }
    printField(f);

  } else if (strcmp(cmd, "CFG") == 0) {
    for (int i = 0; i < CFG_FIELD_COUNT; i++) printField(&CFG_FIELDS[i]);
    Serial.println(F("$CFGEND"));

  } else if (strcmp(cmd, "SAVE") == 0) {
    configSave();
    // Read the flash image back and compare byte for byte, so an incomplete
    // data-flash write reports an error instead of a false $OK.
    Config check;
    EEPROM.get(CFG_EEPROM_ADDR, check);
    if (memcmp(&check, &cfg, sizeof(Config)) == 0) Serial.println(F("$OK,save"));
    else Serial.println(F("$ERR,save,verify failed (flash readback mismatch)"));

  } else if (strcmp(cmd, "LOAD") == 0) {
    if (configLoad()) {
      resetVoltageFilter();
      if (!powerSave) zenerActiveV = cfg.zenerMaxV;
      Serial.println(F("$OK,load"));
    } else {
      Serial.println(F("$ERR,load,stored config invalid"));
    }

  } else if (strcmp(cmd, "DEFAULTS") == 0) {
    // HWREV and BRIDGE describe the board this XIAO is plugged into, not a
    // preference, so they survive a factory reset the way the serial number
    // does.  Reverting BRIDGE would switch the float-detect circuit on for a
    // meter that has no such circuit to drive.
    uint8_t keepRev    = cfg.hwRev;
    uint8_t keepBridge = cfg.bridge;
    configDefaults();
    cfg.hwRev  = keepRev;
    cfg.bridge = keepBridge;
    resetVoltageFilter();
    if (!powerSave) zenerActiveV = cfg.zenerMaxV;
    cfgDirty = true;                   // RAM now differs from EEPROM
    Serial.println(F("$OK,defaults"));

  } else if (strcmp(cmd, "SEEDCAL") == 0) {
    // Re-seed from one of the six original hand-tuned sets.  Not saved --
    // look at it with !CFG first, then !SAVE if it is the right unit.
    if (!arg) { Serial.println(F("$ERR,seedcal,usage !SEEDCAL,<unit id>")); return; }
    uint8_t id = (uint8_t)atoi(arg);
    if (configSeedFromLegacy(id)) {
      if (!powerSave) zenerActiveV = cfg.zenerMaxV;
      cfgDirty = true;
      Serial.print(F("$OK,seedcal,")); Serial.println(id);
    } else {
      Serial.print(F("$ERR,seedcal,no legacy set for ")); Serial.println(id);
    }

  } else if (strcmp(cmd, "SN") == 0) {
    if (arg) {
      while (*arg == ' ') arg++;
      if (*arg == '\0')                        Serial.println(F("$ERR,sn,empty"));
      else if (strchr(arg, ','))               Serial.println(F("$ERR,sn,comma not allowed"));
      else if (strlen(arg) > SN_MAX_LEN - 1)   Serial.println(F("$ERR,sn,too long"));
      else {
        snSave(arg);                           // identity: persists immediately
        Serial.print(F("$SN,")); Serial.println(unitSN);
        Serial.println(F("$OK,sn"));
      }
    } else {
      Serial.print(F("$SN,")); Serial.println(unitSN);
    }

  // ---- Live data ---------------------------------------------
  } else if (strcmp(cmd, "READ") == 0) {
    emitLive();

  } else if (strcmp(cmd, "STREAM") == 0) {
    serialMode = arg ? (atoi(arg) != 0) : !serialMode;
    emitStatus();

  } else if (strcmp(cmd, "RATE") == 0) {
    if (arg) {
      const ConfigField *f = findField("STREAMMS");
      fieldSet(f, atof(arg));
      cfgDirty = true;
    }
    Serial.print(F("$OK,rate,")); Serial.println(cfg.streamMs);

  } else if (strcmp(cmd, "STATUS") == 0) {
    emitStatus();

  } else if (strcmp(cmd, "MINMAX") == 0) {
    MinMaxDisplay = arg ? (atoi(arg) != 0) : !MinMaxDisplay;
    emitMinMax();

  } else if (strcmp(cmd, "RESET") == 0) {
    ReZero();
    Serial.println(F("$OK,reset"));

  // ---- Meter control -----------------------------------------
  } else if (strcmp(cmd, "ZERO") == 0) {
    // Refuse rather than null the leads against a reading the meter is not
    // taking -- in a voltmeter mode, or with the source parked, the value
    // would be whatever was last measured and the null silently wrong.
    if (!resistanceMeasured) {
      Serial.println(F("$ERR,zero,resistance not being measured"));
      return;
    }
    zeroOffsetRes  = currentResistance;
    initialZeroSet = true;
    Serial.print(F("$OK,zero,")); Serial.println(zeroOffsetRes, 4);

  } else if (strcmp(cmd, "ZEROCLR") == 0) {
    zeroOffsetRes = 0.0f;
    Serial.println(F("$OK,zeroclr"));

  } else if (strcmp(cmd, "MODE") == 0) {
    if (!arg) { Serial.print(F("$MODE,")); Serial.println(currentMode); return; }
    int m = atoi(arg);
    if (m < 0 || m >= NUM_MODES) { Serial.println(F("$ERR,mode,out of range")); return; }
    currentMode = static_cast<Mode>(m);
    Serial.print(F("$MODE,")); Serial.println(currentMode);

  } else if (strcmp(cmd, "RANGE") == 0) {
    if (!arg) { emitStatus(); return; }
    if (*arg == 'A' || *arg == 'a') {
      ohmsAutoRange = true;
    } else {
      ohmsAutoRange = false;
      ohmsHighRange = (atoi(arg) != 0);
    }
    emitStatus();

  } else if (strcmp(cmd, "VDISP") == 0) {
    voltageDisplay = arg ? (atoi(arg) != 0) : !voltageDisplay;
    emitStatus();

  // NOTE: the old S (smooth) and M (manual voltage display) commands are not
  // carried over.  Both toggled a flag that nothing ever read -- the display
  // path decides fast-vs-smoothed from preciseMode.  A command that provably
  // does nothing is worse in a new protocol than a missing one.

  } else if (strcmp(cmd, "FAST") == 0) {
    screenRefreshFast = arg ? (atoi(arg) != 0) : !screenRefreshFast;
    Serial.print(F("$OK,fast,")); Serial.println(screenRefreshFast ? 1 : 0);

  } else if (strcmp(cmd, "AMPS") == 0) {
    ampsMode = arg ? (atoi(arg) != 0) : !ampsMode;
    Serial.print(F("$OK,amps,")); Serial.println(ampsMode ? 1 : 0);

  } else if (strcmp(cmd, "LOWPWR") == 0) {
    // Hold the meter in power save: the 20 mA source stays parked, the channel
    // is still measured, and the rail sag that would normally end it is
    // ignored.  Without this the state cannot be observed -- connecting
    // anything to measure the draw or watch the detection wakes it up, which
    // is precisely the behaviour being tuned.
    forceLowPower = arg ? (atoi(arg) != 0) : !forceLowPower;
    if (!forceLowPower) {
      // Release immediately rather than leaving power save latched until the
      // next sag; the arm timer then restarts from a fresh reading.
      powerSave   = false;
      timeHighset = false;
    }
    emitStatus();

  } else if (strcmp(cmd, "IDET") == 0) {
    // Re-run current-sensor detection and report the raw numbers.  Detection
    // is a boot decision by design, so this is a bench tool: plug a sensor in
    // and out, watch the mean and the spread, then set IDETCNT / IDETLO /
    // IDETHI / IDETPP from what this actually reports on this board.
    detectCurrentSensor();

  } else if (strcmp(cmd, "DEBUG") == 0) {
    debugMode = arg ? (atoi(arg) != 0) : !debugMode;
    Serial.print(F("$OK,debug,")); Serial.println(debugMode ? 1 : 0);

  // ---- Logging -----------------------------------------------
  } else if (strcmp(cmd, "LOG") == 0) {
    takeLog     = true;
    tLogStart   = millis() / 1000.0f;
    samplesTaken = 0;
    Serial.println(F("$OK,log,armed"));

  } else if (strcmp(cmd, "DUMP") == 0) {
    emitLog();

  // ---- Calibration -------------------------------------------
  } else if (strcmp(cmd, "CAL") == 0) {
    if (!arg) { Serial.println(F("$ERR,cal,usage !CAL,<actual ohms>[,<index>]")); return; }
    char *idxs = strchr(arg, ',');
    int forced = -1;
    if (idxs) { *idxs = '\0'; forced = atoi(idxs + 1); }
    calResistance(atof(arg), forced);

  } else if (strcmp(cmd, "CALV") == 0) {
    if (!arg) { Serial.println(F("$ERR,calv,usage !CALV,<actual volts>")); return; }
    calVoltage(atof(arg));

  } else if (strcmp(cmd, "CALI") == 0) {
    // Take the present shunt voltage as the zero-current baseline.
    const ConfigField *f = findField("IZERO");
    fieldSet(f, currentShuntVoltage);
    cfgDirty = true;
    Serial.print(F("$CAL,IZERO,")); Serial.println(cfg.iZero, 6);

  } else if (strcmp(cmd, "HELP") == 0) {
    emitHelp();

  } else {
    Serial.print(F("$ERR,cmd,unknown ")); Serial.println(cmd);
  }
}

// ==================================================================
//  PUMP
// ==================================================================
// Line assembly.  An over-long line is truncated rather than wrapped, so a
// burst of noise cannot be spliced onto the front of a real command.
void serialPoll() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleLine(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < CMD_BUF_LEN - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

void serialStreamTick(unsigned long nowMs) {
  if (!serialMode) return;

  if (nowMs - lastStreamMs >= cfg.streamMs) {
    lastStreamMs = nowMs;
    emitLive();
  }
  // Extremes change slowly and carry timestamps; once a second is plenty and
  // keeps the live stream from being half bookkeeping.
  if (nowMs - lastMinMaxMs >= 1000UL) {
    lastMinMaxMs = nowMs;
    emitMinMax();
  }
}
