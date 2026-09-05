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
  adcCount = ads.readADC_SingleEnded(2);

  if (adcCount > (int16_t)cfg.adcCountHigh && gainIndex > 0) {
    gainIndex--;
    ads.setGain(kGainLevels[gainIndex]);
    adcCount = ads.readADC_SingleEnded(2);
  } else if (adcCount < (int16_t)cfg.adcCountLow && gainIndex < kNumGainLevels - 1) {
    gainIndex++;
    ads.setGain(kGainLevels[gainIndex]);
    adcCount = ads.readADC_SingleEnded(2);
  }

  ohmsVoltage = adcCount * kGainFactors[gainIndex] / 1000.0f;

  // --- Which reference is in force ---
  // In power-save the constant-current source is off and the rail sits at
  // sleepV, so the divider maths has to be told about it.
  if (powerSave) {
    zenerActiveV = cfg.sleepV;
  } else if (ohmsVoltage > zenerActiveV) {
    ohmsVoltage = zenerActiveV - ZENER_CLAMP_EPS;
  } else {
    zenerActiveV = cfg.zenerMaxV;
  }

  // --- Raw resistance ---
  if (currentRangeHigh) {
    rawResistance = cfg.dividerR * (ohmsVoltage / (zenerActiveV - ohmsVoltage));
  } else {
    rawResistance = ohmsVoltage / (cfg.constantI - (ohmsVoltage / cfg.constantR));
  }

  // --- One-shot lead auto-zero at startup ---
  if (!initialZeroSet) {
    analogWrite(OHMPWMPIN, 0);
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
    countV = ads.readADC_Differential_0_1();
    if (abs(countV) > (int16_t)cfg.adcCountHigh && gainIndexVolt > 0) {
      --gainIndexVolt;
      ads.setGain(kGainLevels[gainIndexVolt]);
      countV = ads.readADC_Differential_0_1();
    } else if (abs(countV) < (int16_t)cfg.adcCountLow &&
               gainIndexVolt < (size_t)(kNumGainLevels - 1)) {
      ++gainIndexVolt;
      ads.setGain(kGainLevels[gainIndexVolt]);
      countV = ads.readADC_Differential_0_1();
    }
    newVoltageReading = (countV * kGainFactors[gainIndexVolt] / 1000.0f) * cfg.voltScale;
  } else {
    // AC in AltUnits: hold a fixed mid gain so the rms figure is not chasing
    // its own range changes.
    ads.setGain(GAIN_EIGHT);
    countV = ads.readADC_Differential_0_1();
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
  bool bridgeMode = (currentMode == Voltmeter || currentMode == VACmanual ||
                     currentMode == AltUnitsMode);
  bool bridgeQuiet = (fabs(averageVoltage) < cfg.bridgeAvgMax &&
                      currentMode != VACmanual &&
                      newVoltageReading < cfg.bridgeVMax) ||
                     (currentMode == VACmanual && VAC < cfg.bridgeVacMax);

  if (cfg.bridge && bridgeMode && voltageDisplay && bridgeQuiet) {
    Vzero = true;
    ClosedOrFloat();
  } else {
    Vzero     = false;
    vFloating = false;
  }
}

// ==================================================================
//  CURRENT
// ==================================================================
// NOTE: `if (IHigh)` is the pre-rewrite condition, kept verbatim.  IHigh is a
// float that starts at -6.0 and only ever holds a measured maximum, so this is
// effectively always true and the low-range branch below does not run.  That
// is a known open question, deliberately left alone by this rewrite so the
// restructure stays behaviour-preserving -- see the notes in the plan before
// changing it to `if (Irange)`.
void measureCurrent() {
  static bool   firstCurrentRun = true;
  static size_t gainIndexCurrent;

  if (firstCurrentRun) {
    gainIndexCurrent = kNumGainLevels - 1;
    firstCurrentRun  = false;
  }

  if (IHigh) {
    // High range: fixed coarse gain, hall-sensor style conversion.
    ads.setGain(GAIN_TWOTHIRDS);
    countI = ads.readADC_SingleEnded(3);
    currentShuntVoltage = (countI * GAIN_FACTOR_TWOTHIRDS / 1000.0f);
    Ireading = (currentShuntVoltage - cfg.iZero) / cfg.iShunt;
  } else {
    // Low range: auto-gain, direct shunt conversion.
    ads.setGain(kGainLevels[gainIndexCurrent]);
    countI = ads.readADC_SingleEnded(3);

    if (abs(countI) > (int32_t)cfg.adcCountHigh && gainIndexCurrent > 0) {
      --gainIndexCurrent;
      ads.setGain(kGainLevels[gainIndexCurrent]);
      countI = ads.readADC_SingleEnded(3);
    } else if (abs(countI) < (int32_t)cfg.adcCountLow &&
               gainIndexCurrent < (size_t)(kNumGainLevels - 1)) {
      ++gainIndexCurrent;
      ads.setGain(kGainLevels[gainIndexCurrent]);
      countI = ads.readADC_SingleEnded(3);
    }

    currentShuntVoltage = (countI * kGainFactors[gainIndexCurrent] / 1000.0f);
    Ireading = currentShuntVoltage;      // 1 ohm sense resistor
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
  bridgeV = (ads.readADC_Differential_0_1() * GAIN_FACTOR_8 / 1000.0f) * -1.0f;
  vFloating = (bridgeV < cfg.bridgeThr);
  digitalWrite(VbridgePin, LOW);
}
