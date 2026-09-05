/*
 * Measure.ino -- ADS1115 acquisition for resistance, voltage and current,
 * plus the bridge test that tells an open lead from a closed one.
 *
 * The logic here is unchanged from the pre-rewrite firmware; what were bare
 * literals are now config keys, and the correction-factor if/else ladder is a
 * table lookup over the same breakpoints.  Nothing about how a value is
 * derived was altered.
 */

// ADS1115 gain settings, coarsest first, paired with their mV-per-bit factors.
static const adsGain_t kGainLevels[] = {
  GAIN_TWOTHIRDS, GAIN_ONE, GAIN_TWO, GAIN_FOUR, GAIN_EIGHT, GAIN_SIXTEEN
};
static const float kGainFactors[] = {
  GAIN_FACTOR_TWOTHIRDS, GAIN_FACTOR_1, GAIN_FACTOR_2,
  GAIN_FACTOR_4, GAIN_FACTOR_8, GAIN_FACTOR_16
};
static const int kNumGainLevels = sizeof(kGainLevels) / sizeof(kGainLevels[0]);

// Clamp applied when the ohms reading runs past the active reference: the
// divider maths divides by (ref - reading), so letting them meet would divide
// by zero.  Small enough to be invisible, large enough to stay finite.
static const float ZENER_CLAMP_EPS = 0.0001f;

// The three things this meter measures, by ADS1115 mux setting.
#define ADS_MUX_OHMS    MUX_BY_CHANNEL[2]
#define ADS_MUX_CURRENT MUX_BY_CHANNEL[3]
#define ADS_MUX_VOLTS   ADS1X15_REG_CONFIG_MUX_DIFF_0_1

// ==================================================================
//  ADS1115 ACCESS
// ==================================================================
// Two ways to get a sample out of the part:
//
//   single-shot  write the config register to start a conversion, poll the OS
//                bit until it finishes, read the result.  Every sample pays a
//                full conversion period of polling.
//   continuous   write the config register once; the ADC free-runs and each
//                sample is a single register read.
//
// Continuous is only faster when the config register holds still.  Changing
// the mux, the gain or the data rate restarts the conversion, and a restart
// costs the same full conversion period that single-shot pays -- plus an extra
// register write.  So it needs BOTH:
//
//   one channel     alternating channels restarts on every switch;
//   a settled rate  the resistance path reschedules the data rate on its own
//                   each pass, and the display-select block in loop() writes it
//                   again, so any pass that includes the ohms channel would
//                   restart every time and come out slightly BEHIND single-shot.
//
// That leaves voltmeter mode with no ammeter fitted -- the case this was added
// for -- and logging with no ammeter.  HighRMode is a single channel but is the
// ohms channel, so it deliberately stays on single-shot.
//
// TRAP: conversionComplete() reads the config register's OS bit, which is
// permanently 0 while the part is free-running -- the library's own wait loop
// (and readADC_SingleEnded, which uses it) would spin forever.  Nothing on the
// continuous path may call it, so the first sample after a restart is waited
// out on a timer instead.
//
// Sampling faster than the conversion rate returns the same result twice.
// That is inherent to reading a free-running ADC and is harmless for display,
// but it means the rolling voltage buffer can hold repeated samples, which
// slightly understates the rms figure.  Raise the data rate rather than the
// loop rate if that matters.
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

// delayMicroseconds() is only reliable for small values, and the slow data
// rates are well past that.
static void adsWaitOneConversion(uint16_t rate) {
  uint32_t us = adsConversionUs(rate) + 200;      // margin for start-up
  if (us >= 16000) {
    delay(us / 1000UL);
    delayMicroseconds((unsigned int)(us % 1000UL));
  } else {
    delayMicroseconds((unsigned int)us);
  }
}

// Called once per measurement pass with the number of distinct channels that
// pass will read, and whether the ohms channel is one of them.
void adsPlanPass(uint8_t channelCount, bool ohmsInPass) {
  adsContWanted = (channelCount == 1) && !ohmsInPass;
}

bool adsContinuous() { return adsContRunning; }

int16_t adsSample(uint16_t mux) {
  if (adsContWanted) {
    // Restart whenever anything the config register holds has moved.  Reading
    // the gain and rate back from the library means the many setGain() and
    // setDataRate() calls scattered through the measurement code do not each
    // have to remember to tell this layer.
    if (!adsContRunning || mux != adsContMux ||
        ads.getGain() != adsContGain || ads.getDataRate() != adsContRate) {
      ads.startADCReading(mux, /*continuous=*/true);
      adsContMux     = mux;
      adsContGain    = ads.getGain();
      adsContRate    = ads.getDataRate();
      adsContRunning = true;
      adsWaitOneConversion(adsContRate);   // first result after a restart
    }
    return ads.getLastConversionResults();
  }

  // Single shot.  startADCReading writes MODE_SINGLE, which also takes the
  // part out of continuous mode, so no explicit teardown is needed.
  adsContRunning = false;
  ads.startADCReading(mux, /*continuous=*/false);
  while (!ads.conversionComplete()) { }
  return ads.getLastConversionResults();
}

// A conversion that is guaranteed to have STARTED after this call.  Used where
// the reading has to follow a specific event -- the bridge MOSFET toggling,
// or sensor detection -- rather than being whatever the free-running ADC last
// happened to finish.
int16_t adsSampleFresh(uint16_t mux) {
  bool saved = adsContWanted;
  adsContWanted = false;
  int16_t v = adsSample(mux);
  adsContWanted = saved;
  return v;
}

// ==================================================================
//  VOLTAGE FILTER
// ==================================================================
// cfg.vSamples is the live length of the rolling buffer, so changing it has
// to discard the old contents -- otherwise the running sums still hold
// samples that are no longer inside the window and the average is wrong until
// the buffer happens to wrap.  Called at boot and from !SET,VSAMPLES.
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
  static bool  firstRun = true;
  static bool  currentRangeHigh;
  float        prevR = rawResistance;

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
    // Power-save only makes sense on the high range -- the constant-current
    // source is the thing being switched off.
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

  ohmsVoltage = adcCount * kGainFactors[gainIndex] / 1000.0f;

  // --- Which reference is in force ---
  // In power save the constant-current source is parked and the rail sits at
  // sleepV, so the divider maths has to be told about it.
  //
  // Derived from the state every pass rather than edited in place.  The old
  // form only restored zenerMaxV in the branch that ran when the reading was
  // BELOW the reference, so once it had dropped to sleepV any reading above
  // sleepV took the clamp branch instead and left it there -- the reference
  // stayed low, every reading clamped, and the meter read open for good.
  zenerActiveV = powerSave ? cfg.sleepV : cfg.zenerMaxV;
  if (ohmsVoltage > zenerActiveV) ohmsVoltage = zenerActiveV - ZENER_CLAMP_EPS;

  // --- Raw resistance ---
  if (currentRangeHigh) {
    rawResistance = cfg.dividerR * (ohmsVoltage / (zenerActiveV - ohmsVoltage));
  } else {
    rawResistance = ohmsVoltage / (cfg.constantI - (ohmsVoltage / cfg.constantR));
  }

  // --- One-shot lead auto-zero at startup ---
  // No analogWrite here any more: loop() owns OHMPWMPIN outright.  Two writers
  // for one pin is how the park got stuck, and the pin is already unparked at
  // boot, so this write was only ever a redundant second claim on it.
  if (!initialZeroSet) {
    if (currentResistance > 0.001f && currentResistance < cfg.zeroAutoMax) {
      zeroOffsetRes  = currentResistance;
      initialZeroSet = true;
      Serial.print(F("$INFO,autozero,"));
      Serial.println(zeroOffsetRes, 4);
    } else if (currentResistance > cfg.zeroAutoMax && currentResistance < 1e6f) {
      initialZeroSet = true;
      Serial.println(F("$INFO,autozero,none"));
    }
  }

  // --- Schedule the ADC data rate against how fast the reading is moving ---
  const float jumpLo = prevR / cfg.rateJump;
  const float jumpHi = prevR * cfg.rateJump;

  if (!isBetween(rawResistance, jumpLo, jumpHi) && ohmsVoltage < cfg.rateBumpV) {
    ads.setDataRate(RATE_ADS1115_475SPS);          // moving: sample fast
    if (!powerSave) lcdInterval = cfg.lcdBumpMs;
  } else if (!voltageDisplay && preciseMode && rawResistance < cfg.ratePrecR &&
             isBetween(rawResistance, prevR * (1.0f - cfg.rateStable),
                                      prevR * (1.0f + cfg.rateStable))) {
    ads.setDataRate(RATE_ADS1115_16SPS);           // settled and precise
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

  // --- Open circuit ---
  // HighRMode deliberately skips this so the reading keeps climbing past the
  // point the auto ranges call open.
  if (currentMode != HighRMode) {
    if (ohmsVoltage > (zenerActiveV - cfg.openMargin)) {
      currentResistance = cfg.openR;
      if (!voltageDisplay) ads.setDataRate(RATE_ADS1115_475SPS);
    }
  }
}

// ==================================================================
//  VOLTAGE
// ==================================================================
void measureVoltage() {
  static bool   firstVoltRun = true;
  static size_t gainIndexVolt;

  if (firstVoltRun) {
    gainIndexVolt = kNumGainLevels - 1;   // start at the finest resolution
    firstVoltRun  = false;
  }

  if (!(VACPresense && altUnits)) {
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
    newVoltageReading = (countV * kGainFactors[gainIndexVolt] / 1000.0f) * cfg.voltScale;
  } else {
    // AC in AltUnits: hold a fixed mid gain so the rms figure is not chasing
    // its own range changes.
    ads.setGain(GAIN_EIGHT);
    countV = adsSample(ADS_MUX_VOLTS);
    newVoltageReading = (countV * GAIN_FACTOR_8 / 1000.0f) * cfg.voltScale;
  }

  if (altUnits) newVoltageReading = newVoltageReading * cfg.altMult;

  // --- Min / max ---
  if (newVoltageReading > highV) {
    highV         = newVoltageReading;
    formatTime(millis(), timeAtMaxV);
    currentAtMaxV = Ireading;
    lcdInterval   = cfg.lcdBumpMs;
    if (MinMaxDisplay && voltageDisplay) analogWrite(CONTINUITY_PIN, cfg.beepBright);
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

  // Precise mode implies a nulled DC measurement, where an rms figure derived
  // from the same buffer is meaningless.
  if (preciseMode) VAC = 0.0f;

  // --- AC present? ---
  // Parenthesised to the binding the original had: the AltUnits test stands
  // on its own, the plain test additionally requires a small DC average.
  bool acPlain = (!altUnits && VAC > cfg.vacThresh && averageVoltage < cfg.vacAvgMax);
  bool acAlt   = (altUnits && VAC > cfg.vacThreshAlt);
  VACPresense  = (acPlain || acAlt || currentMode == VACmanual);

  // --- Bridge test (open lead vs. closed) ---
  // Only on boards that have the circuit, only in the modes it is wanted in,
  // and only when the reading is small enough that the answer is interesting.
  //
  // NOT in VACmanual.  The test drives the bridge MOSFET across the inputs and
  // reads the transient it produces, which only means anything against a
  // quiet DC input; on an AC input it fires against whatever point of the
  // waveform it lands on.  It used to run here, gated on VAC being below
  // BRIDGEVAC -- that gate is gone and so is the key.
  bool bridgeMode = (currentMode == Voltmeter || currentMode == AltUnitsMode);
  bool bridgeQuiet = (fabs(averageVoltage) < cfg.bridgeAvgMax &&
                      newVoltageReading < cfg.bridgeVMax);

  if (cfg.bridge && bridgeMode && voltageDisplay && bridgeQuiet) {
    Vzero = true;
    ClosedOrFloat();
  } else {
    Vzero     = false;
    vFloating = false;
  }
}

// ==================================================================
//  CURRENT SENSOR DETECTION
// ==================================================================
// Decides, once per boot, what is on the current channel:
//
//   mean at ground    -> a shunt          (low range)
//   mean at mid-rail  -> an ACS712 or similar hall sensor (high range)
//   neither, or an unsteady reading -> nothing fitted, readings suppressed
//
// Both fitted cases DRIVE the pin.  An empty header leaves it floating and
// high impedance, which shows up two ways: the mean lands nowhere in
// particular, and the samples wander.  The spread test is the backstop for a
// floating pin whose mean happens to drift through one of the windows.
//
// Every threshold is a key and every measured number is reported, because
// what a floating input actually reads is a property of the board, not
// something that can be settled from the source: run !IDET on the bench with
// and without a sensor and set IDETCNT / IDETLO / IDETHI / IDETPP from what
// comes back.
//
// Returns true if a sensor was found; sets Irange, currentOnOff, and (when
// IAUTOZERO) cfg.iZero in RAM.
bool detectCurrentSensor() {
  ads.setGain(GAIN_TWOTHIRDS);
  delay(cfg.iDetSettleMs);         // settle BEFORE sampling, not after

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

  // Symmetric, so a floating pin sitting below ground is not read as a shunt.
  bool atGround = (abs((int32_t)mean) < (int32_t)cfg.iDetCount);
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

  Serial.print(F("$IDET,"));
  Serial.print(currentOnOff ? (Irange ? "high" : "low") : "off");
  Serial.print(',');  Serial.print(mean);
  Serial.print(',');  Serial.print(currentShuntVoltage, 4);
  Serial.print(',');  Serial.print(ppV, 4);
  Serial.print(',');  Serial.print(atGround ? 1 : 0);
  Serial.print(',');  Serial.print(atMidRail ? 1 : 0);
  Serial.print(',');  Serial.print(steady ? 1 : 0);
  Serial.print(',');  Serial.println(cfg.iZero, 4);

  return currentOnOff;
}

// ==================================================================
//  CURRENT
// ==================================================================
void measureCurrent() {
  static bool   firstCurrentRun = true;
  static size_t gainIndexCurrent;

  // No sensor was found at boot, so there is nothing on this channel but a
  // floating pin.  Suppressing here rather than at the call site is what makes
  // the boot decision actually mean something: before this, detection set a
  // flag that only the display consulted, so the reading was still computed
  // from a floating input and still went out over $LIVE.
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

  // WAS `if (IHigh)`, which is a float initialised to -6.0 that only ever
  // holds a measured maximum -- always non-zero, so the shunt branch below
  // could never run and a detected shunt was read with the hall-sensor
  // formula.  Changed to Irange, the flag detection actually sets, because
  // otherwise the detection result has no effect on the measurement.
  if (Irange) {
    // High range: fixed coarse gain, hall-sensor style conversion.
    ads.setGain(GAIN_TWOTHIRDS);
    countI = adsSample(ADS_MUX_CURRENT);
    currentShuntVoltage = (countI * GAIN_FACTOR_TWOTHIRDS / 1000.0f);
    Ireading = (currentShuntVoltage - cfg.iZero) / cfg.iShunt;
  } else {
    // Low range: auto-gain, direct shunt conversion.
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
    Ireading = currentShuntVoltage / cfg.iShuntR;   // Ohm's law across the shunt
  }

  // --- Noise floor, then min/max ---
  if ((Irange  && isBetween(Ireading, -cfg.iNoiseHi, cfg.iNoiseHi)) ||
      (!Irange && Ireading < cfg.iNoiseLo)) {
    Ireading = 0.0f;
  } else if (Ireading > IHigh || Ireading < ILow) {
    float nowSec = millis() / 1000.0f;
    logCurrentData(newVoltageReading, nowSec, Ireading);
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
//  BRIDGE TEST
// ==================================================================
// Close the bridge MOSFET across the inputs and look at what the differential
// does.  A lead that is genuinely open sags well negative; one that is closed
// onto a circuit barely moves.
void ClosedOrFloat() {
  digitalWrite(VbridgePin, HIGH);
  ads.setGain(GAIN_EIGHT);
  delay(2);
  // Fresh, not free-running: this reading only means something if the whole
  // conversion happened after the MOSFET went high.
  bridgeV = (adsSampleFresh(ADS_MUX_VOLTS) * GAIN_FACTOR_8 / 1000.0f) * -1.0f;
  vFloating = (bridgeV < cfg.bridgeThr);
  digitalWrite(VbridgePin, LOW);
}
