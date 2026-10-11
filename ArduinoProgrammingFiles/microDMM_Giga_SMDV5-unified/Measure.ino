/*
 * Measure.ino -- ADS1115 acquisition for resistance, voltage and current, plus
 * the bridge test that tells an open lead from a closed one.
 *
 * Same derivations as the pre-config Giga firmware; the literals are config
 * keys now, and the CF_A..CF_O ladder is a table lookup over the same edges.
 */

static const adsGain_t kGainLevels[] = {
  GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN
};
static const float kGainFactors[] = {
  GAIN_FACTOR_TWOTHIRDS, GAIN_FACTOR_1, GAIN_FACTOR_2,
  GAIN_FACTOR_4, GAIN_FACTOR_8, GAIN_FACTOR_16
};
static const int kNumGainLevels = sizeof(kGainLevels) / sizeof(kGainLevels[0]);

// The divider maths divides by (ref - reading), so they must not meet.
static const float ZENER_CLAMP_EPS = 0.0001f;

// Bridge test timing.  Fixed constants rather than keys: measured once
// (!BRIDGE) and nothing has needed tuning per unit.  Promote them to config
// keys -- appended, with a CFG_VERSION bump and migration -- if one does.
#define BRIDGE_PERIOD_MS 50UL                // min time between tests (throughput)
#define BRIDGE_RATE      RATE_ADS1115_860SPS  // conversion rate for the test

// The three things this meter measures, by ADS1115 mux setting.
#define ADS_MUX_OHMS    MUX_BY_CHANNEL[2]
#define ADS_MUX_CURRENT MUX_BY_CHANNEL[3]
#define ADS_MUX_VOLTS   ADS1X15_REG_CONFIG_MUX_DIFF_0_1

// ==================================================================
//  ADS1115 ACCESS
// ==================================================================
// Two ways to get a sample out of the part:
//
//   single-shot  write the config register to start a conversion, poll until
//                it finishes, read the result.  Every sample pays a full
//                conversion period.
//   continuous   write the config register once; the ADC free-runs and each
//                sample is a single register read.
//
// Continuous only wins while the config register holds still -- changing the
// mux, gain or data rate restarts the conversion at the same cost as a single
// shot.  So it is used only when a pass reads ONE channel and that channel is
// not the ohms channel (whose path reschedules the data rate on its own):
// Voltmeter and VAC modes, and logging, with no ammeter fitted.  An ammeter
// alternates the mux every pass, and every resistance mode reads two channels,
// so those stay single-shot exactly as before.
//
// TRAP: conversionComplete() reads the OS bit, which stays 0 while the part
// free-runs -- the library's wait loop (and readADC_SingleEnded/Differential,
// which use it) would spin forever.  Nothing on the continuous path may call
// them, so every read in this sketch goes through adsSample(), and the first
// result after a restart is waited out on a timer instead.
//
// Reading faster than the data rate returns the same result twice.  Harmless
// for display; it can slightly understate VAC, since the rolling buffer may
// hold repeats.  Raise the data rate rather than the loop rate if that matters.
static bool      adsContWanted  = false;
static bool      adsContRunning = false;
static uint16_t  adsContMux     = 0xFFFF;
static adsGain_t adsContGain    = GAIN_TWOTHIRDS;
static uint16_t  adsContRate    = RATE_ADS1115_128SPS;

// Conversion period per data rate, microseconds, rounded up.
static uint32_t adsConversionUs(uint16_t rate) {
  switch (rate) {
    case RATE_ADS1115_8SPS:   return 125000;
    case RATE_ADS1115_16SPS:  return  62500;
    case RATE_ADS1115_32SPS:  return  31250;
    case RATE_ADS1115_64SPS:  return  15625;
    case RATE_ADS1115_128SPS: return   7813;
    case RATE_ADS1115_250SPS: return   4000;
    case RATE_ADS1115_475SPS: return   2106;
    case RATE_ADS1115_860SPS: return   1163;
    default:                  return   7813;
  }
}

static void adsWaitOneConversion(uint16_t rate) {
  uint32_t us = adsConversionUs(rate) + 200;      // margin for start-up
  if (us >= 16000) {
    delay(us / 1000UL);
    delayMicroseconds((unsigned int)(us % 1000UL));
  } else {
    delayMicroseconds((unsigned int)us);
  }
}

// Take the part out of continuous mode and wait until it is idle.  Takes a
// write and a wait: the write sets MODE_SINGLE (its start bit is ignored while
// a conversion is in flight), the in-flight conversion finishes, and the part
// powers down.  The wait covers the slower of the free-running rate and the
// library's current one, in case the write did start a conversion, and the
// OS poll confirms it -- the OS bit is valid again once in single-shot mode.
void adsStop() {
  if (!adsContRunning) return;
  ads.startADCReading(adsContMux, /*continuous=*/false);
  uint32_t a = adsConversionUs(adsContRate), b = adsConversionUs(ads.getDataRate());
  adsWaitOneConversion(a > b ? adsContRate : ads.getDataRate());
  while (!ads.conversionComplete()) { }
  adsContRunning = false;
}

// Called once per measurement pass with the number of distinct channels that
// pass will read, and whether the ohms channel is one of them.
void adsPlanPass(uint8_t channelCount, bool ohmsInPass) {
  adsContWanted = (channelCount == 1) && !ohmsInPass;
}

bool adsContinuous() { return adsContRunning; }

// WHILE FREE-RUNNING A CONVERSION IS ALWAYS IN FLIGHT, and the datasheet's OS
// bit "can only be written when in power-down state and has no effect when a
// conversion is ongoing".  A config write therefore does not start anything
// on its own; the in-flight conversion, taken with the OLD mux/gain, finishes
// first.  Every transition below waits that conversion out (at the old rate)
// before trusting a result.  Without it the bridge test's single shot
// returned the voltage path's last conversion scaled at GAIN_SIXTEEN -- the
// pegged "V FLT: 256m" in Voltmeter mode.
int16_t adsSample(uint16_t mux) {
  if (adsContWanted) {
    // Restart whenever anything the config register holds has moved.  Reading
    // gain and rate back from the library means the setGain()/setDataRate()
    // calls scattered through the code need not tell this layer.
    if (!adsContRunning || mux != adsContMux ||
        ads.getGain() != adsContGain || ads.getDataRate() != adsContRate) {
      bool     wasRunning = adsContRunning;
      uint16_t oldRate    = adsContRate;
      ads.startADCReading(mux, /*continuous=*/true);
      adsContMux     = mux;
      adsContGain    = ads.getGain();
      adsContRate    = ads.getDataRate();
      adsContRunning = true;
      if (wasRunning) adsWaitOneConversion(oldRate);   // old-config conversion
      adsWaitOneConversion(adsContRate);               // first new-config result
    }
    return ads.getLastConversionResults();
  }

  // Single shot, from an idle part.
  adsStop();
  ads.startADCReading(mux, /*continuous=*/false);
  while (!ads.conversionComplete()) { }
  return ads.getLastConversionResults();
}

// A conversion guaranteed to have STARTED after this call, for readings that
// must follow an event -- the bridge MOSFET switching, or sensor detection --
// rather than whatever the free-running ADC last finished.
int16_t adsSampleFresh(uint16_t mux) {
  bool saved = adsContWanted;
  adsContWanted = false;
  int16_t v = adsSample(mux);
  adsContWanted = saved;
  return v;
}

// cfg.vSamples is the live window length; changing it must discard the old
// contents or the running sums hold samples from outside the window.
void resetVoltageFilter() {
  for (int i = 0; i < VOLT_SAMPLE_MAX; i++) {
    voltageSamples[i]        = 0.0f;
    voltageSquaredSamples[i] = 0.0f;
  }
  voltageSampleIndex = 0;
  voltageSum         = 0.0f;
  squaredVoltageSum  = 0.0f;
  averageVoltage     = 0.0f;
  VAC                = 0.0f;
}

// ==================================================================
//  RESISTANCE
// ==================================================================
void measureResistance() {
  static bool firstRun = true;
  static bool currentRangeHigh;
  float       prevR = rawResistance;

  if (firstRun) {
    currentRangeHigh = (digitalRead(SETRANGE_PIN) == LOW);
    gainIndex        = currentRangeHigh ? 0 : (kNumGainLevels - 1);
    firstRun         = false;
  }

  // --- Range control (auto / power-save / manual) ---
  const float rangeLow  = cfg.rangeThreshR * (1.0f - cfg.rangeDeadband);
  const float rangeHigh = cfg.rangeThreshR * (1.0f + cfg.rangeDeadband);

  if (ohmsAutoRange && !powerSave) {
    if (!currentRangeHigh && prevR > rangeHigh) {
      currentRangeHigh = true;
      digitalWrite(SETRANGE_PIN, HIGH);
    } else if (currentRangeHigh && prevR < rangeLow) {
      currentRangeHigh = false;
      digitalWrite(SETRANGE_PIN, LOW);
    }
  } else if (powerSave) {
    currentRangeHigh = true;
    digitalWrite(SETRANGE_PIN, HIGH);
  } else {
    if (ohmsHighRange && !currentRangeHigh) {
      currentRangeHigh = true;
      digitalWrite(SETRANGE_PIN, HIGH);
    } else if (!ohmsHighRange && currentRangeHigh) {
      currentRangeHigh = false;
      digitalWrite(SETRANGE_PIN, LOW);
    }
  }

  // --- Read with one step of gain correction ---
  ads.setGain(kGainLevels[gainIndex]);
  adcCount = adsSample(ADS_MUX_OHMS);
  if (adcCount > (int16_t)cfg.adcCountHigh && gainIndex > 0) {
    gainIndex--;
    ads.setGain(kGainLevels[gainIndex]);
    adcCount = adsSample(ADS_MUX_OHMS);
  } else if (adcCount < (int16_t)cfg.adcCountLow && gainIndex < kNumGainLevels - 1) {
    gainIndex++;
    ads.setGain(kGainLevels[gainIndex]);
    adcCount = adsSample(ADS_MUX_OHMS);
  }

  ohmsVoltage    = adcCount * kGainFactors[gainIndex] / 1000.0f;
  ohmsVoltageRaw = ohmsVoltage;

  // --- Which reference is in force ---
  // Derived every pass.  The old form only restored it when the reading was
  // BELOW the reference, so once power save dropped it to sleepV it could
  // stick there and the meter read open until the leads were shorted.
  zenerActiveV = powerSave ? cfg.sleepV : cfg.zenerMaxV;

  // A clamped reading is an open circuit, not a value: in High R the clamp
  // alone works out to ~1.2 G, the "1226.3M" shown with the leads open.
  bool clamped = (ohmsVoltage >= zenerActiveV);
  if (clamped) ohmsVoltage = zenerActiveV - ZENER_CLAMP_EPS;

  // --- Raw resistance ---
  if (currentRangeHigh) {
    rawResistance = cfg.dividerR * (ohmsVoltage / (zenerActiveV - ohmsVoltage));
  } else {
    rawResistance = ohmsVoltage / (cfg.constantI - (ohmsVoltage / cfg.constantR));
  }

  // --- One-shot lead auto-zero ---
  if (!initialZeroSet) {
    if (currentResistance > 0.001f && currentResistance < cfg.zeroAutoMax) {
      zeroOffsetRes  = currentResistance;
      initialZeroSet = true;
      Host.print(F("$INFO,autozero,"));
      Host.println(zeroOffsetRes, 4);
    } else if (currentResistance > cfg.zeroAutoMax && currentResistance < 1e6f) {
      initialZeroSet = true;
      Host.println(F("$INFO,autozero,none"));
    }
  }

  // --- Schedule the ADC data rate against how fast the reading is moving ---
  const float jumpLo = prevR / cfg.rateJump;
  const float jumpHi = prevR * cfg.rateJump;
  if (!isBetween(rawResistance, jumpLo, jumpHi) && ohmsVoltage < cfg.rateBumpV) {
    ads.setDataRate(RATE_ADS1115_475SPS);
    if (!powerSave) lcdInterval = cfg.lcdBumpMs;
  } else if (!voltageDisplay && preciseMode && rawResistance < cfg.ratePrecR &&
             isBetween(rawResistance, prevR * (1.0f - cfg.rateStable),
                                      prevR * (1.0f + cfg.rateStable))) {
    ads.setDataRate(RATE_ADS1115_16SPS);
  } else if (!voltageDisplay && (ohmsVoltage < (zenerActiveV - cfg.rateSlowV))) {
    ads.setDataRate(RATE_ADS1115_128SPS);
  }

  // --- Piecewise correction ---
  calibratedResistance = rawResistance * cfg.rCal[rCalIndex(rawResistance)];
  currentResistance    = calibratedResistance;

  if (altUnits) {
    // Beta equation, then Celsius to Fahrenheit.
    currentResistance = (1.0f / ((1.0f / 298.15f) +
                        (log(currentResistance / cfg.thermR0) / cfg.thermB)) - 273.15f)
                        * 1.8f + 32.0f;
  }

  resistanceSamples[resistanceSampleIndex] = ohmsVoltage;
  resistanceSampleIndex = (resistanceSampleIndex + 1) % NUM_RESISTANCE_SAMPLES;

  // --- Open circuit ---
  // High R skips the usual margin so the reading keeps climbing past where the
  // auto ranges call it open -- but a reading AT the reference is open there too.
  ohmsOpen = (currentMode == HighRMode) ? clamped
                                        : (ohmsVoltage > (zenerActiveV - cfg.openMargin));
  if (ohmsOpen) {
    currentResistance = cfg.openR;
    if (!voltageDisplay && currentMode != HighRMode) ads.setDataRate(RATE_ADS1115_475SPS);
  }
}

// ==================================================================
//  VOLTAGE
// ==================================================================
void measureVoltage() {
  static bool   firstVoltRun = true;
  static size_t gainIndexVolt;

  if (firstVoltRun) {
    gainIndexVolt = kNumGainLevels - 1;
    firstVoltRun  = false;
  }

  if (VACPresense || currentMode == VACmanual) {
    // Fixed mid gain so the rms figure is not chasing its own range changes.
    ads.setGain(GAIN_ONE);
    countV = adsSample(ADS_MUX_VOLTS);
    newVoltageReading = (countV * GAIN_FACTOR_1 / 1000.0f) * cfg.voltScale - cfg.voltOffset;
  } else if (!altUnits) {
    ads.setGain(kGainLevels[gainIndexVolt]);
    countV = adsSample(ADS_MUX_VOLTS);
    if (abs(countV) > (int16_t)cfg.adcCountHigh && gainIndexVolt > 0) {
      --gainIndexVolt;
      ads.setGain(kGainLevels[gainIndexVolt]);
      countV = adsSample(ADS_MUX_VOLTS);
    } else if (abs(countV) < (int16_t)cfg.adcCountLow &&
               gainIndexVolt < (size_t)(kNumGainLevels - 1)) {
      ++gainIndexVolt;
      ads.setGain(kGainLevels[gainIndexVolt]);
      countV = adsSample(ADS_MUX_VOLTS);
    }
    float scale = (countV > 0) ? cfg.voltScale : cfg.voltScaleNeg;
    newVoltageReading = (countV * kGainFactors[gainIndexVolt] / 1000.0f) * scale - cfg.voltOffset;
  } else {
    ads.setGain(GAIN_EIGHT);
    countV = adsSample(ADS_MUX_VOLTS);
    newVoltageReading = (countV * GAIN_FACTOR_8 / 1000.0f) * cfg.voltScale - cfg.voltOffset;
  }

  if (altUnits) newVoltageReading = newVoltageReading * cfg.altMult;

  // --- Min / max ---
  if (newVoltageReading > highV) {
    highV         = newVoltageReading;
    formatTime(millis(), timeAtMaxV);
    currentAtMaxV = Ireading;
    lcdInterval   = cfg.lcdBumpMs;
    if (MinMaxDisplay && voltageDisplay) {
      analogWrite(CONTINUITY_PIN, cfg.beepBright);
      rgb.on(0, 255, 0);
    }
  }
  if (newVoltageReading < lowV) {
    lowV          = newVoltageReading;
    formatTime(millis(), timeAtMinV);
    currentAtMinV = Ireading;
    lcdInterval   = cfg.lcdBumpMs;
  }

  // --- Display smoothing ---
  medianVoltageStep = (newVoltageReading - medianVoltage) * cfg.smoothAlpha;
  medianVoltage    += medianVoltageStep;

  if (!VACPresense && fabs(medianVoltageStep) > 0.3f && !preciseMode && newVoltageReading > 0.5f) {
    lcdInterval = cfg.lcdBumpMs;
  } else if (preciseMode && voltageDisplay) {
    lcdInterval = cfg.lcdMs;
  }

  // --- Rolling average and rms ---
  const uint8_t n = cfg.vSamples;
  voltageSum        -= voltageSamples[voltageSampleIndex];
  squaredVoltageSum -= voltageSquaredSamples[voltageSampleIndex];
  voltageSamples[voltageSampleIndex] = newVoltageReading;
  float diff = newVoltageReading - averageVoltage;
  voltageSquaredSamples[voltageSampleIndex] = diff * diff;
  voltageSum        += newVoltageReading;
  squaredVoltageSum += voltageSquaredSamples[voltageSampleIndex];
  voltageSampleIndex = (voltageSampleIndex + 1) % n;
  averageVoltage     = voltageSum / n;
  VAC                = sqrt(squaredVoltageSum / n);

  if (preciseMode) VAC = 0.0f;   // nulled DC: an rms figure is meaningless

  // --- AC present? ---
  bool acPlain = (!altUnits && VAC > cfg.vacThresh && averageVoltage < cfg.vacAvgMax);
  bool acAlt   = (altUnits && VAC > cfg.vacThreshAlt);
  VACPresense  = (acPlain || acAlt || currentMode == VACmanual);

  // --- Bridge test (open lead vs. closed) ---
  // Not in VACmanual: the test reads the transient the MOSFET produces, which
  // only means anything against a quiet DC input.
  bool bridgeQuiet = (fabs(averageVoltage) < cfg.bridgeAvgMax &&
                      fabs(newVoltageReading) < cfg.bridgeVMax);
  if (cfg.bridge && currentMode != VACmanual && !preciseMode &&
      voltageDisplay && bridgeQuiet) {
    // Rate-limited for throughput, not physics: each test costs ~5 ms and
    // knocks the ADC out of free-running, and the leads do not change state
    // faster than a few times a second.
    static unsigned long lastBridgeMs = 0;
    Vzero = true;
    if (millis() - lastBridgeMs >= BRIDGE_PERIOD_MS) {
      lastBridgeMs = millis();
      ClosedOrFloat();
    }
  } else {
    Vzero     = false;
    vFloating = false;
  }
}

// ==================================================================
//  CURRENT SENSOR DETECTION
// ==================================================================
// Once per boot (and on !IDET):
//   mean near ground   -> a shunt (low range)
//   mean in IDETLO..HI -> a hall sensor at its mid-rail bias (high range)
//   otherwise, or unsteady -> nothing fitted, readings suppressed
bool detectCurrentSensor() {
  ads.setGain(GAIN_TWOTHIRDS);
  delay(cfg.iDetSettleMs);

  int32_t sum = 0;
  int16_t lo  = 32767;
  int16_t hi  = -32768;
  for (uint8_t i = 0; i < cfg.iDetSamples; i++) {
    int16_t c = adsSampleFresh(ADS_MUX_CURRENT);
    sum += c;
    if (c < lo) lo = c;
    if (c > hi) hi = c;
  }

  int16_t mean = (int16_t)(sum / (int32_t)cfg.iDetSamples);
  adcReadingCurrent   = mean;
  currentShuntVoltage = mean * GAIN_FACTOR_TWOTHIRDS / 1000.0f;
  float ppV = (hi - lo) * GAIN_FACTOR_TWOTHIRDS / 1000.0f;

  bool atGround  = (abs((int32_t)mean) < (int32_t)cfg.iDetCount);
  bool atMidRail = isBetween(currentShuntVoltage, cfg.iDetLo, cfg.iDetHi);
  bool steady    = (cfg.iDetPPV <= 0.0f) || (ppV <= cfg.iDetPPV);

  if ((atGround || atMidRail) && steady) {
    Irange       = atMidRail;
    currentOnOff = true;
    if (cfg.iAutoZero) cfg.iZero = atMidRail ? currentShuntVoltage : 0.0f;
  } else {
    currentOnOff = false;
    Irange       = false;
    Ireading     = 0.0f;
  }

  Host.print(F("$IDET,"));
  Host.print(currentOnOff ? (Irange ? "high" : "low") : "off");
  Host.print(',');  Host.print(mean);
  Host.print(',');  Host.print(currentShuntVoltage, 4);
  Host.print(',');  Host.print(ppV, 4);
  Host.print(',');  Host.print(atGround ? 1 : 0);
  Host.print(',');  Host.print(atMidRail ? 1 : 0);
  Host.print(',');  Host.print(steady ? 1 : 0);
  Host.print(',');  Host.println(cfg.iZero, 4);

  return currentOnOff;
}

// ==================================================================
//  CURRENT
// ==================================================================
void measureCurrent() {
  static bool   firstCurrentRun = true;
  static size_t gainIndexCurrent;

  // Nothing found at boot: the channel is a floating pin, so report nothing.
  if (!currentOnOff) {
    Ireading            = 0.0f;
    currentShuntVoltage = 0.0f;
    countI              = 0;
    return;
  }

  if (firstCurrentRun) {
    gainIndexCurrent = kNumGainLevels - 1;
    firstCurrentRun  = false;
  }

  // WAS `if (IHigh)` -- always true, so a detected shunt was read with the
  // hall-sensor formula.  Irange is what detection sets.
  if (Irange) {
    ads.setGain(GAIN_TWOTHIRDS);
    countI = adsSample(ADS_MUX_CURRENT);
    currentShuntVoltage = (countI * GAIN_FACTOR_TWOTHIRDS / 1000.0f);
    Ireading = (currentShuntVoltage - cfg.iZero) / cfg.iShunt;
  } else {
    ads.setGain(kGainLevels[gainIndexCurrent]);
    countI = adsSample(ADS_MUX_CURRENT);
    if (abs(countI) > (int32_t)cfg.adcCountHigh && gainIndexCurrent > 0) {
      --gainIndexCurrent;
      ads.setGain(kGainLevels[gainIndexCurrent]);
      countI = adsSample(ADS_MUX_CURRENT);
    } else if (abs(countI) < (int32_t)cfg.adcCountLow &&
               gainIndexCurrent < (size_t)(kNumGainLevels - 1)) {
      ++gainIndexCurrent;
      ads.setGain(kGainLevels[gainIndexCurrent]);
      countI = adsSample(ADS_MUX_CURRENT);
    }
    currentShuntVoltage = (countI * kGainFactors[gainIndexCurrent] / 1000.0f);
    Ireading = currentShuntVoltage / cfg.iShuntR;
  }

  // --- Noise floor, then min/max ---
  if ((Irange  && isBetween(Ireading, -cfg.iNoiseHi, cfg.iNoiseHi)) ||
      (!Irange && Ireading < cfg.iNoiseLo)) {
    Ireading = 0.0f;
  } else if (Ireading > IHigh || Ireading < ILow) {
    logCurrentData(newVoltageReading, millis() / 1000.0f, Ireading);
    if (Ireading > IHigh) {
      IHigh         = Ireading;
      voltageAtMaxI = newVoltageReading;
      formatTime(millis(), timeAtMaxI);
    }
    if (Ireading < ILow) {
      ILow          = Ireading;
      voltageAtMinI = newVoltageReading;
      formatTime(millis(), timeAtMinI);
    }
  }
}

// ==================================================================
//  BRIDGE TEST (Giga circuit)
// ==================================================================
// One bridge reading: close the MOSFET across the inputs and read the
// differential at high gain.  On unit 2 (!BRIDGE) open leads gave ~65-77 mV at
// 860 SPS and ~9-11 mV at 128; shorted, ~6-9 mV and ~1 mV.  The signal sits
// at the START of the conversion, so the data rate is pinned for the test.
//
// THE ADC MUST BE IDLE FIRST.  The lead check runs straight after a free-
// running voltage read, and the ADC was left converting through the MOSFET
// switch-on; that path read ~0 mV whatever the leads did, while !BRIDGE --
// which always started from an idle ADC -- read correctly.  Both now go
// through this one function, so what !BRIDGE shows is what the meter uses.
float bridgeSample(uint16_t rate) {
  uint16_t  savedRate = ads.getDataRate();
  adsGain_t savedGain = ads.getGain();

  adsStop();
  ads.setGain(GAIN_SIXTEEN);
  ads.setDataRate(rate);
  digitalWrite(VbridgePin, HIGH);
  delay(2);
  float v = adsSampleFresh(ADS_MUX_VOLTS) * GAIN_FACTOR_16 / 1000.0f;
  digitalWrite(VbridgePin, LOW);

  ads.setDataRate(savedRate);
  ads.setGain(savedGain);
  return v;
}

// Near zero = the leads are closed onto something; well away from zero =
// floating; in between = unsure.  "Closed" needs two tests in a row.
void ClosedOrFloat() {
  static bool closedPrev = false;

  bridgeV = bridgeSample(BRIDGE_RATE);

  bool closedNow = (fabs(bridgeV) < cfg.bridgeClsV);
  vFloating  = (fabs(bridgeV) > cfg.bridgeOpenV);
  vUndefined = !closedNow && !vFloating;
  vClosed    = closedNow && closedPrev;
  closedPrev = closedNow;
}

// ==================================================================
//  BRIDGE DIAGNOSTIC (!BRIDGE)
// ==================================================================
// One bridge reading under every combination of ohms source (parked / on),
// SETRANGE_PIN (lo / hi) and data rate (860 / 128 SPS), then everything put
// back.  Run it with the leads open, then shorted.  On unit 2 the source and
// range made no difference (the ohms and voltage inputs are physically
// separate), nor did the pause between tests; the data rate did.
void bridgeDiag() {
  const int  savedRange = digitalRead(SETRANGE_PIN);
  const bool wasParked  = ohmsParked;
  static const uint16_t rates[2]    = { RATE_ADS1115_860SPS, RATE_ADS1115_128SPS };
  static const char    *rateName[2] = { "860", "128" };

  Host.print(F("$INFO,bridge,mode="));   Host.print(currentMode);
  Host.print(F(",parked="));             Host.print(wasParked ? 1 : 0);
  Host.print(F(",range="));              Host.println(savedRange == HIGH ? "hi" : "lo");

  for (int src = 0; src < 2; src++) {               // 0 parked, 1 on
    analogWrite(OHMPWMPIN, src ? 0 : cfg.psPwm);
    delay(50);                                      // let the source settle
    for (int rng = 0; rng < 2; rng++) {             // 0 lo, 1 hi
      digitalWrite(SETRANGE_PIN, rng ? HIGH : LOW);
      delay(20);
      for (int r = 0; r < 2; r++) {
        delay(BRIDGE_PERIOD_MS);
        float v = bridgeSample(rates[r]);
        Host.print(F("$BRIDGE,src="));  Host.print(src ? "on" : "parked");
        Host.print(F(",range="));       Host.print(rng ? "hi" : "lo");
        Host.print(F(",rate="));        Host.print(rateName[r]);
        Host.print(',');                Host.println(v * 1000.0f, 2);   // mV
      }
    }
  }

  // Put the pins back exactly as the loop left them; it only rewrites them on
  // a change, so anything else would stick.
  digitalWrite(SETRANGE_PIN, savedRange);
  analogWrite(OHMPWMPIN, wasParked ? cfg.psPwm : 0);
  if (!wasParked) ohmsUnparkMs = millis();          // treat as a fresh unpark
  Host.println(F("$OK,bridge"));
}
