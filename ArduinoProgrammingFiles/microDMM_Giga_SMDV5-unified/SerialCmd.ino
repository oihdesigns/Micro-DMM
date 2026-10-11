/*
 * SerialCmd.ino -- the host protocol.  Same !CMD / $LINE protocol as the XIAO
 * rewrite, so microdmm_gui.py drives either meter.  The Giga's old
 * single-character commands (Q, q, Z, k, j, ...) are gone.
 *
 * The protocol runs identically over USB serial and the BLE UART (Ble.ino):
 * every reply goes through Host, which writes to both.
 *
 *   IN   !CMD            or  !CMD,<arg>  or  !SET,<key>,<value>
 *   OUT  $TYPE,<fields...>                one record per line
 *
 * Every line the firmware emits starts with '$'; anything else is library
 * noise and a host should ignore it.
 *
 * Records:
 *   $LIVE,ms,R,Rz,V,Vavg,Vac,I,Vr,Vbat,mode,flags      (Vbat always 0 on the Giga)
 *   $MINMAX,Vlo,Vhi,Rlo,Rhi,Ilo,Ihi,tVlo,tVhi
 *   $CONT,<0|1>,R           continuity edge, sent the moment it changes
 *   $STATUS,key=value,...   includes board=giga and modes=<name|name|...>
 *   $CFG,<key>,<value>  ... terminated by $CFGEND
 *   $SN,<value>
 *   $IDET,<state>,<meanCounts>,<meanV>,<ppV>,<atGnd>,<atMid>,<steady>,<izero>
 *   $CAL,<index>,<raw>,<actual>,<factor>   (also $CAL,VSCALE|VSCALENEG,... and $CAL,IZERO,v)
 *   $DIAG,ohmsVraw,zenerActive,ohmsV,adcCount,gainIndex,ps,parked,mic,bridgeV
 *   $LOGSTART,<n>,<t0> / $LOG,<i>,<t>,<V>,<I> / $LOGEND,<t1>
 *   $OK,<what> / $ERR,<what>,<why> / $INFO,<what>,...
 *
 * $LIVE flag bits (bit 0 = LSB):
 *   0 voltageDisplay   4 Vzero        8  MinMaxDisplay  12 ampsMode
 *   1 powerSave        5 continuity   9  screenSleep    13 resistance open
 *   2 VACPresense      6 ohmsHighRange 10 preciseMode   14 currentOnOff
 *   3 vFloating        7 ohmsAutoRange 11 altUnits      15 config dirty
 *   16 resistance measured this pass   17 ADS in continuous conversion
 *   18 ohms source parked              19 low-power held by hand (!LOWPWR)
 *   20 wake condition met right now
 */

static unsigned long lastStreamMs = 0;
static unsigned long lastMinMaxMs = 0;

// ==================================================================
//  EMITTERS
// ==================================================================
static uint32_t liveFlags() {
  uint32_t f = 0;
  if (voltageDisplay)     f |= 1uL << 0;
  if (powerSave)          f |= 1uL << 1;
  if (VACPresense)        f |= 1uL << 2;
  if (vFloating)          f |= 1uL << 3;
  if (Vzero)              f |= 1uL << 4;
  if (continuity)         f |= 1uL << 5;
  if (ohmsHighRange)      f |= 1uL << 6;
  if (ohmsAutoRange)      f |= 1uL << 7;
  if (MinMaxDisplay)      f |= 1uL << 8;
  if (screenSleep)        f |= 1uL << 9;
  if (preciseMode)        f |= 1uL << 10;
  if (altUnits)           f |= 1uL << 11;
  if (ampsMode)           f |= 1uL << 12;
  if (ohmsOpen)           f |= 1uL << 13;
  if (currentOnOff)       f |= 1uL << 14;
  if (cfgDirty)           f |= 1uL << 15;
  if (resistanceMeasured) f |= 1uL << 16;
  if (adsContinuous())    f |= 1uL << 17;
  if (ohmsParked)         f |= 1uL << 18;
  if (forceLowPower)      f |= 1uL << 19;
  if (ohmsParked && ohmsVoltage < cfg.sleepV - cfg.psHyst) f |= 1uL << 20;
  return f;
}

void emitLive() {
  Host.print(F("$LIVE,"));
  Host.print(millis());              Host.print(',');
  Host.print(currentResistance, 3);  Host.print(',');
  Host.print(displayResistance, 3);  Host.print(',');
  Host.print(newVoltageReading, 6);  Host.print(',');
  Host.print(averageVoltage, 6);     Host.print(',');
  Host.print(VAC, 4);                Host.print(',');
  Host.print(Ireading, 4);           Host.print(',');
  Host.print(ohmsVoltage, 5);        Host.print(',');
  Host.print(batteryVoltage, 3);     Host.print(',');
  Host.print(currentMode);           Host.print(',');
  Host.println(liveFlags());
}

void emitMinMax() {
  Host.print(F("$MINMAX,"));
  Host.print(lowV, 6);    Host.print(',');
  Host.print(highV, 6);   Host.print(',');
  Host.print(lowR, 3);    Host.print(',');
  Host.print(highR, 3);   Host.print(',');
  Host.print(ILow, 4);    Host.print(',');
  Host.print(IHigh, 4);   Host.print(',');
  Host.print(timeAtMinV); Host.print(',');
  Host.println(timeAtMaxV);
}

void emitStatus() {
  Host.print(F("$STATUS,board=giga,mode=")); Host.print(currentMode);
  Host.print(F(",modes="));
  for (int m = 0; m < NUM_MODES; m++) {
    if (m) Host.print('|');
    Host.print(modeToString(static_cast<Mode>(m)));
  }
  Host.print(F(",range="));    Host.print(ohmsHighRange ? "high" : "low");
  Host.print(F(",auto="));     Host.print(ohmsAutoRange ? 1 : 0);
  Host.print(F(",ps="));       Host.print(powerSave ? 1 : 0);
  Host.print(F(",zero="));     Host.print(zeroOffsetRes, 4);
  Host.print(F(",bridge="));   Host.print(cfg.bridge ? 1 : 0);
  Host.print(F(",stream="));   Host.print(serialMode ? 1 : 0);
  Host.print(F(",debug="));    Host.print(debugMode ? 1 : 0);
  Host.print(F(",amps="));     Host.print(currentOnOff ? 1 : 0);
  Host.print(F(",irange="));   Host.print(Irange ? "high" : "low");
  Host.print(F(",rmeas="));    Host.print(resistanceMeasured ? 1 : 0);
  Host.print(F(",cont="));     Host.print(adsContinuous() ? 1 : 0);
  Host.print(F(",ohmspark=")); Host.print(ohmsParked ? 1 : 0);
  Host.print(F(",lowpwr="));   Host.print(forceLowPower ? 1 : 0);
  Host.print(F(",dirty="));    Host.print(cfgDirty ? 1 : 0);
  Host.print(F(",stored="));   Host.print(cfgStored ? 1 : 0);
  Host.print(F(",flash="));    Host.print(flashAvailable() ? 1 : 0);
  Host.print(F(",usb="));      Host.print(logMode ? 1 : 0);
  Host.print(F(",ble="));      Host.print(bleState());
  Host.print(F(",hwrev="));    Host.print(cfg.hwRev);
  Host.print(F(",sn="));       Host.println(unitSN);
}

// Raw numbers for bring-up: the unclamped ohms rail is what ZENERMAX (leads
// open, High R) and SLEEPV (leads open, after power save parks) are read from.
static void emitDiag() {
  Host.print(F("$DIAG,"));
  Host.print(ohmsVoltageRaw, 5); Host.print(',');
  Host.print(zenerActiveV, 5);   Host.print(',');
  Host.print(ohmsVoltage, 5);    Host.print(',');
  Host.print(adcCount);          Host.print(',');
  Host.print(gainIndex);         Host.print(',');
  Host.print(powerSave ? 1 : 0); Host.print(',');
  Host.print(ohmsParked ? 1 : 0);Host.print(',');
  Host.print(micLevel);          Host.print(',');
  Host.println(bridgeV, 5);
}

static void emitLog() {
  Host.print(F("$LOGSTART,"));
  Host.print(LOG_SIZE);  Host.print(',');
  Host.println(tLogStart, 3);
  for (int i = 0; i < LOG_SIZE; i++) {
    Host.print(F("$LOG,"));
    Host.print(i);                        Host.print(',');
    Host.print(loggedTimeStamps[i], 3);   Host.print(',');
    Host.print(loggedVoltagesAtI[i], 4);  Host.print(',');
    Host.println(loggedCurrents[i], 4);
  }
  Host.print(F("$LOGEND,"));
  Host.println(tLogEnd, 3);
}

void emitHelp() {
  Host.println(F("$INFO,help,config,!CFG !GET,K !SET,K,V !SAVE !LOAD !DEFAULTS !SEEDCAL,N !SN[,V]"));
  Host.println(F("$INFO,help,live,!READ !STREAM[,0|1] !RATE,MS !STATUS !MINMAX[,0|1] !RESET !DIAG"));
  Host.println(F("$INFO,help,meter,!ZERO !ZEROCLR !MODE,N !RANGE,0|1|A !VDISP[,0|1] !LOWPWR[,0|1]"));
  Host.println(F("$INFO,help,cal,!CAL,OHMS[,IDX] !CALV,VOLTS !CALI !IDET !BRIDGE !DEBUG !AMPS !FAST !LOG !DUMP"));
}

// ==================================================================
//  CALIBRATION HELPERS
// ==================================================================
// The factor that lands the present raw reading on a known reference is
// actual/raw, in the bucket the raw reading selects.
static void calResistance(float actual, int forcedIdx) {
  if (!resistanceMeasured) { Host.println(F("$ERR,cal,resistance not being measured")); return; }
  if (ohmsOpen)            { Host.println(F("$ERR,cal,leads open or over range")); return; }
  if (rawResistance <= 0.0f) { Host.println(F("$ERR,cal,no raw reading")); return; }
  if (actual <= 0.0f)        { Host.println(F("$ERR,cal,actual must be positive")); return; }
  int idx = (forcedIdx >= 0) ? forcedIdx : rCalIndex(rawResistance);
  if (idx >= R_CAL_BUCKETS)  { Host.println(F("$ERR,cal,index out of range")); return; }

  char key[16];
  snprintf(key, sizeof(key), "RCAL%02d", idx);
  const ConfigField *f = findField(key);
  fieldSet(f, actual / rawResistance);
  cfgDirty = true;

  Host.print(F("$CAL,"));
  Host.print(idx);               Host.print(',');
  Host.print(rawResistance, 4);  Host.print(',');
  Host.print(actual, 4);         Host.print(',');
  Host.println(fieldGet(f), 6);  // the clamped value, not the request
}

// The Giga scales positive and negative inputs separately, so the polarity of
// the present reading picks which scale is calibrated.
static void calVoltage(float actual) {
  if (altUnits || VACPresense || currentMode == VACmanual) {
    Host.println(F("$ERR,calv,use a plain DC mode (Voltmeter or Default)"));
    return;
  }
  // newVoltageReading = counts * gf * scale - offset, so work on the part the
  // scale multiplies.
  float scaled = newVoltageReading + cfg.voltOffset;
  float target = actual + cfg.voltOffset;
  if (fabs(scaled) < 1e-6f) { Host.println(F("$ERR,calv,reading too small to scale")); return; }
  if ((scaled > 0) != (target > 0)) { Host.println(F("$ERR,calv,polarity disagrees")); return; }

  bool neg = (countV <= 0);
  const ConfigField *f = findField(neg ? "VSCALENEG" : "VSCALE");
  fieldSet(f, fieldGet(f) * (target / scaled));
  cfgDirty = true;
  Host.print(neg ? F("$CAL,VSCALENEG,") : F("$CAL,VSCALE,"));
  Host.print(newVoltageReading, 6); Host.print(',');
  Host.print(actual, 6);            Host.print(',');
  Host.println(fieldGet(f), 6);
}

// ==================================================================
//  COMMAND DISPATCH
// ==================================================================
static void cmdSet(char *arg) {
  char *val = arg ? strchr(arg, ',') : NULL;
  if (!arg || !val) { Host.println(F("$ERR,set,usage !SET,<key>,<value>")); return; }
  *val = '\0'; val++;
  for (char *p = arg; *p; ++p) *p = toupper(*p);

  const ConfigField *f = findField(arg);
  if (!f) { Host.print(F("$ERR,set,unknown key ")); Host.println(arg); return; }

  fieldSet(f, atof(val));
  if (strcmp(f->name, "VSAMPLES") == 0) resetVoltageFilter();

  cfgDirty = true;
  printField(f);
}

void handleLine(char *line) {
  if (line[0] != '!') return;
  char *cmd = line + 1;
  char *arg = strchr(cmd, ',');
  if (arg) { *arg = '\0'; arg++; }
  for (char *p = cmd; *p; ++p) *p = toupper(*p);

  // ---- Configuration -----------------------------------------
  if (strcmp(cmd, "SET") == 0) {
    cmdSet(arg);

  } else if (strcmp(cmd, "GET") == 0) {
    if (!arg) { Host.println(F("$ERR,get,usage !GET,<key>")); return; }
    for (char *p = arg; *p; ++p) *p = toupper(*p);
    const ConfigField *f = findField(arg);
    if (!f) { Host.print(F("$ERR,get,unknown key ")); Host.println(arg); return; }
    printField(f);

  } else if (strcmp(cmd, "CFG") == 0) {
    for (int i = 0; i < CFG_FIELD_COUNT; i++) printField(&CFG_FIELDS[i]);
    Host.println(F("$CFGEND"));

  } else if (strcmp(cmd, "SAVE") == 0) {
    if (!flashAvailable())  Host.println(F("$ERR,save,config flash unavailable"));
    else if (configSave())  Host.println(F("$OK,save"));
    else                    Host.println(F("$ERR,save,verify failed (flash readback mismatch)"));

  } else if (strcmp(cmd, "LOAD") == 0) {
    if (configLoad()) {
      resetVoltageFilter();
      Host.println(F("$OK,load"));
    } else {
      Host.println(F("$ERR,load,stored config invalid or absent"));
    }

  } else if (strcmp(cmd, "DEFAULTS") == 0) {
    // HWREV and BRIDGE describe this board, not a preference, so they survive.
    uint8_t keepRev    = cfg.hwRev;
    uint8_t keepBridge = cfg.bridge;
    configDefaults();
    cfg.hwRev  = keepRev;
    cfg.bridge = keepBridge;
    resetVoltageFilter();
    cfgDirty = true;
    Host.println(F("$OK,defaults"));

  } else if (strcmp(cmd, "SEEDCAL") == 0) {
    // Load a unit's compiled-in starting point.  RAM only -- check with !CFG,
    // then !SAVE.  This is also how a fresh unit gets its HWREV.
    if (!arg) { Host.println(F("$ERR,seedcal,usage !SEEDCAL,<unit>")); return; }
    uint8_t id = (uint8_t)atoi(arg);
    if (configSeedFromLegacy(id)) {
      cfgDirty = true;
      Host.print(F("$OK,seedcal,")); Host.println(id);
    } else {
      Host.print(F("$ERR,seedcal,no set for unit ")); Host.println(id);
    }

  } else if (strcmp(cmd, "SN") == 0) {
    if (arg) {
      while (*arg == ' ') arg++;
      if (*arg == '\0')                       Host.println(F("$ERR,sn,empty"));
      else if (strchr(arg, ','))              Host.println(F("$ERR,sn,comma not allowed"));
      else if (strlen(arg) > SN_MAX_LEN - 1)  Host.println(F("$ERR,sn,too long"));
      else if (!snSave(arg))                  Host.println(F("$ERR,sn,flash write failed"));
      else {
        Host.print(F("$SN,")); Host.println(unitSN);
        Host.println(F("$OK,sn"));
      }
    } else {
      Host.print(F("$SN,")); Host.println(unitSN);
    }

  // ---- Live data ---------------------------------------------
  } else if (strcmp(cmd, "READ") == 0) {
    emitLive();

  } else if (strcmp(cmd, "STREAM") == 0) {
    serialMode = arg ? (atoi(arg) != 0) : !serialMode;
    emitStatus();

  } else if (strcmp(cmd, "RATE") == 0) {
    if (arg) {
      fieldSet(findField("STREAMMS"), atof(arg));
      cfgDirty = true;
    }
    Host.print(F("$OK,rate,")); Host.println(cfg.streamMs);

  } else if (strcmp(cmd, "STATUS") == 0) {
    emitStatus();

  } else if (strcmp(cmd, "DIAG") == 0) {
    emitDiag();

  } else if (strcmp(cmd, "MINMAX") == 0) {
    MinMaxDisplay = arg ? (atoi(arg) != 0) : !MinMaxDisplay;
    emitMinMax();

  } else if (strcmp(cmd, "RESET") == 0) {
    ReZero();
    Host.println(F("$OK,reset"));

  // ---- Meter control -----------------------------------------
  } else if (strcmp(cmd, "ZERO") == 0) {
    if (!resistanceMeasured) { Host.println(F("$ERR,zero,resistance not being measured")); return; }
    if (ohmsOpen)            { Host.println(F("$ERR,zero,leads open or over range")); return; }
    zeroOffsetRes  = currentResistance;
    initialZeroSet = true;
    Host.print(F("$OK,zero,")); Host.println(zeroOffsetRes, 4);

  } else if (strcmp(cmd, "ZEROCLR") == 0) {
    zeroOffsetRes = 0.0f;
    Host.println(F("$OK,zeroclr"));

  } else if (strcmp(cmd, "MODE") == 0) {
    if (!arg) { Host.print(F("$MODE,")); Host.println(currentMode); return; }
    int m = atoi(arg);
    if (m < 0 || m >= NUM_MODES) { Host.println(F("$ERR,mode,out of range")); return; }
    currentMode = static_cast<Mode>(m);
    Host.print(F("$MODE,")); Host.println(currentMode);

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

  } else if (strcmp(cmd, "FAST") == 0) {
    screenRefreshFast = arg ? (atoi(arg) != 0) : !screenRefreshFast;
    Host.print(F("$OK,fast,")); Host.println(screenRefreshFast ? 1 : 0);

  } else if (strcmp(cmd, "AMPS") == 0) {
    ampsMode = arg ? (atoi(arg) != 0) : !ampsMode;
    Host.print(F("$OK,amps,")); Host.println(ampsMode ? 1 : 0);

  } else if (strcmp(cmd, "LOWPWR") == 0) {
    // Hold power save for bench work: source parked, channel still measured,
    // and the sag that would end it ignored.
    forceLowPower = arg ? (atoi(arg) != 0) : !forceLowPower;
    if (!forceLowPower) {
      powerSave   = false;
      timeHighset = false;
    }
    emitStatus();

  } else if (strcmp(cmd, "BRIDGE") == 0) {
    bridgeDiag();

  } else if (strcmp(cmd, "IDET") == 0) {
    detectCurrentSensor();

  } else if (strcmp(cmd, "DEBUG") == 0) {
    debugMode = arg ? (atoi(arg) != 0) : !debugMode;
    Host.print(F("$OK,debug,")); Host.println(debugMode ? 1 : 0);

  // ---- Logging -----------------------------------------------
  } else if (strcmp(cmd, "LOG") == 0) {
    takeLog = true;                       // serviceManualLog() stamps the start
    Host.println(F("$OK,log,armed"));

  } else if (strcmp(cmd, "DUMP") == 0) {
    emitLog();

  // ---- Calibration -------------------------------------------
  } else if (strcmp(cmd, "CAL") == 0) {
    if (!arg) { Host.println(F("$ERR,cal,usage !CAL,<actual ohms>[,<index>]")); return; }
    char *idxs = strchr(arg, ',');
    int forced = -1;
    if (idxs) { *idxs = '\0'; forced = atoi(idxs + 1); }
    calResistance(atof(arg), forced);

  } else if (strcmp(cmd, "CALV") == 0) {
    if (!arg) { Host.println(F("$ERR,calv,usage !CALV,<actual volts>")); return; }
    calVoltage(atof(arg));

  } else if (strcmp(cmd, "CALI") == 0) {
    fieldSet(findField("IZERO"), currentShuntVoltage);
    cfgDirty = true;
    Host.print(F("$CAL,IZERO,")); Host.println(cfg.iZero, 6);

  } else if (strcmp(cmd, "HELP") == 0) {
    emitHelp();

  } else {
    Host.print(F("$ERR,cmd,unknown ")); Host.println(cmd);
  }
}

// ==================================================================
//  PUMP
// ==================================================================
// Line assembly, shared by USB serial and the BLE UART (Ble.ino).  An
// over-long line is truncated rather than wrapped, so noise cannot be spliced
// onto the front of a real command.
void lineFeed(LineIn &in, char c) {
  if (c == '\n' || c == '\r') {
    if (in.len > 0) {
      in.buf[in.len] = '\0';
      in.len = 0;
      handleLine(in.buf);
    }
  } else if (in.len < CMD_BUF_LEN - 1) {
    in.buf[in.len++] = c;
  }
}

void serialPoll() {
  static LineIn serialLine = {};
  while (Serial.available() > 0) lineFeed(serialLine, (char)Serial.read());
}

void serialStreamTick(unsigned long nowMs) {
  if (!serialMode) return;
  if (nowMs - lastStreamMs >= cfg.streamMs) {
    lastStreamMs = nowMs;
    emitLive();
  }
  if (nowMs - lastMinMaxMs >= 1000UL) {
    lastMinMaxMs = nowMs;
    emitMinMax();
  }
}
