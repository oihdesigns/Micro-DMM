/*
 * Display.ino -- SSD1306 rendering and the value formatters.
 *
 * Layout is unchanged from the pre-rewrite firmware.  The formatters now hand
 * back a const char* suffix instead of an Arduino String: they are called six
 * times per screen refresh, and six String constructions per refresh is heap
 * churn for four possible values ("", "m", "k", "M").
 */

// ==================================================================
//  FORMATTERS
// ==================================================================
void formatResistanceValue(float value, float &outValue, const char *&outSuffix, int &outDigits) {
  if (value > 90000000.0f) {
    outValue = value / 1000000.0f;  outSuffix = "M";  outDigits = 1;
  } else if (value > 9000000.0f) {
    outValue = value / 1000000.0f;  outSuffix = "M";  outDigits = 2;
  } else if (value > 900000.0f) {
    outValue = value / 1000000.0f;  outSuffix = "M";  outDigits = 3;
  } else if (value > 900.0f) {
    outValue = value / 1000.0f;     outSuffix = "k";
    outDigits = (value > 90000.0f) ? 3 : 4;
  } else if (value < 0.9f) {
    outValue = value * 1000.0f;     outSuffix = "m";
    outDigits = (value < 0.09f) ? 1 : 2;
  } else {
    outValue = value;               outSuffix = "";
    if      (value >= 100.0f) outDigits = 2;
    else if (value < 9.5f)    outDigits = 4;
    else                      outDigits = 3;
  }
}

void formatVoltageValue(float value, float &outValue, const char *&outSuffix, int &outDigits) {
  if (fabs(value) < 0.9f) {
    outValue = value * 1000.0f;  outSuffix = "m";  outDigits = 0;
  } else {
    outValue = value;            outSuffix = "";
    outDigits = (fabs(value) > 11.0f) ? 2 : 3;
  }
}

// ==================================================================
//  SCREEN
// ==================================================================
void updateDisplay() {
  char tbuf[TIME_BUF_LEN];

  display.clearDisplay();
  blinkLimit = 0;

  if (screenSleep || currentMode == Charging) {
    // Screen saver: a single dot walking across the panel, so the OLED is not
    // holding a static image but the unit still visibly has power.
    // formatTime is called for its side effect on `seconds` -- see Util.ino.
    formatTime(millis(), tbuf);
    int step = seconds % 10;
    display.setCursor((8 + (4 * step)), 32);
    display.setTextSize(1);
    display.print(".");
    display.display();
    return;
  }

  // Fast value while the reading is moving, smoothed once it settles.
  float voltageToDisplay;
  if (!preciseMode) {
    voltageToDisplay = (fabs(medianVoltageStep) > 0.5f) ? newVoltageReading : medianVoltage;
  } else {
    voltageToDisplay = newVoltageReading;
  }

  formatVoltageValue(voltageToDisplay, roundedV,     vSuffix,     vDigits);
  formatVoltageValue(lowV,             roundedVlow,  vSuffixlow,  vDigitslow);
  formatVoltageValue(highV,            roundedVhigh, vSuffixhigh, vDigitshigh);
  formatResistanceValue(displayResistance, roundedR,     rSuffix,     rDigits);
  formatResistanceValue(lowR,              roundedRlow,  rSuffixlow,  rDigitslow);
  formatResistanceValue(highR,             roundedRhigh, rSuffixhigh, rDigitshigh);

  // --- Supply voltage and mode number ---
  display.setTextSize(1);
  display.setCursor(72, 56);
  display.print("VIN:");
  display.print(batteryVoltage, 1);
  display.setCursor(120, 0);
  display.print(currentMode);

  // --- Current overlay ---
  if (((Irange && !isBetween(Ireading, -cfg.iNoiseHi, cfg.iNoiseHi)) ||
       (!Irange && currentOnOff)) || ampsMode) {
    display.setTextSize(2);
    display.setCursor(0, 16);
    if (Irange) {
      display.print("A:");
      display.print(Ireading, 2);
    } else {
      display.print("mA:");
      if      (Ireading > 0.100f) display.print(Ireading * 1000.0f, 2);
      else if (Ireading > 0.010f) display.print(Ireading * 1000.0f, 3);
      else                        display.print(Ireading * 1000.0f, 4);
    }
  }

  if (IHigh != I_HIGH_RESET && currentOnOff) {
    display.setTextSize(1);
    display.setCursor(0, 32);
    display.print("Ilow:");   display.print(ILow, 3);
    display.print(" t:");     display.print(timeAtMinI);
    display.println();
    display.print("Ihigh:");  display.print(IHigh, 3);
    display.print(" t:");     display.print(timeAtMaxI);
  }

  // --- Primary reading ---
  display.setTextSize(2);
  display.setCursor(0, 0);

  if (voltageDisplay) {
    if (Vzero) {
      // Bridge result: floating (open lead), unsure, or closed onto something.
      if (vFloating && bridgeV < cfg.bridgeFltThr) {
        display.print("Vflt:");
        display.print(bridgeV * 1000.0f, 0);
      } else if (vFloating && bridgeV > cfg.bridgeFltThr) {
        display.print("V ?:");
        display.print(bridgeV * 1000.0f, 0);
      } else {
        display.print("CLSD:");
        display.print(bridgeV * 1000.0f, 0);
      }
      display.print("m");
    } else if (VACPresense) {
      display.print("VAC:");
      display.print(VAC, altUnits ? 1 : 3);
    } else {
      display.print("VDC:");
      if (preciseMode) {
        display.setCursor(0, 16);
        display.print(roundedV, vDigits + 1);
        display.println(vSuffix);
      } else {
        display.print(roundedV, vDigits);
        display.println(vSuffix);
      }
      if (deltaV != 0) {
        display.setTextSize(1);
        display.print("ref:");
        display.println(deltaV, deltaVdigits);
        display.print("delta:");
        display.println(newVoltageReading - deltaV, 4);
      }
    }

    if (MinMaxDisplay) {
      display.setTextSize(1);
      display.setCursor(0, 16);
      display.print("Min:");  display.print(roundedVlow, vDigitslow);
      display.print(vSuffixlow);
      display.print("  t:");  display.print(timeAtMinV);
      display.println();
      display.print("Max:");  display.print(roundedVhigh, vDigitshigh);
      display.print(vSuffixhigh);
      display.print("  t:");  display.print(timeAtMaxV);
      display.println();
      display.print("Range:");
      display.print(highV - lowV, 3);
    }

    if (VAC > 0.1f && !VACPresense) {
      // Enough ripple to be worth showing, but not enough to call it AC.
      display.setCursor(0, 48);
      display.setTextSize(1);
      display.print("VAC");
      display.setCursor(0, 56);
      display.print("RMS:");
      display.setCursor(26, 48);
      display.setTextSize(2);
      display.print(VAC, 1);
    } else {
      display.setCursor(0, 48);
      display.setTextSize(2);
      formatTime(millis(), tbuf);
      display.print(tbuf);
      if (debugMode) {
        // Label only -- this has never printed a value.  Left as-is rather
        // than inventing one during a restructure.
        display.setCursor(0, 32);
        display.setTextSize(1);
        display.print("Vraw:");
      }
    }

  } else {
    // --- Resistance ---
    if (ohmsVoltage < (zenerActiveV - cfg.openMargin) || currentMode == HighRMode) {
      display.print(altUnits ? "F:" : "R:");
      display.print(roundedR, rDigits);
      display.print(rSuffix);

      if (MinMaxDisplay) {
        display.setTextSize(1);
        display.setCursor(0, 16);
        display.print("Min:");  display.print(roundedRlow, rDigitslow);
        display.print(rSuffixlow);
        display.println();
        display.print("Max:");  display.print(roundedRhigh, rDigitshigh);
        display.print(rSuffixhigh);
        display.setCursor(102, 0);
        display.print("mVR:");
        display.setCursor(102, 8);
        display.print(ohmsVoltage * 1000.0f, ohmsVoltage < 1.0f ? 1 : 0);
      } else {
        display.setTextSize(2);
        display.println();
        display.print("mVR:");
        display.print(ohmsVoltage * 1000.0f, ohmsVoltage < 1.0f ? 1 : 0);
      }

      display.setTextSize(1);
      display.setCursor(0, 32);
      if (zeroOffsetRes != 0) {
        display.print("null mOhms:");
        display.println(zeroOffsetRes * 1000.0f, 2);
        if (preciseMode) display.print("Precise Mode");
      }
      if (debugMode) {
        display.setCursor(60, 32);
        display.print("ADC:");
        display.print(adcCount);
      }
    } else {
      if (!altUnits) {
        display.println("R:OPEN");
        display.setTextSize(1);
        display.print("mV:");
        display.print(ohmsVoltage * 1000.0f, 2);
      } else {
        display.print("F:OPEN");
      }
      if (MinMaxDisplay) {
        display.setTextSize(1);
        display.setCursor(0, 16);
        display.print("Min:");  display.print(roundedRlow, rDigitslow);
        display.print(rSuffixlow);
        display.println();
        display.print("Max:");  display.print(roundedRhigh, rDigitshigh);
        display.print(rSuffixhigh);
      }
    }

    display.setCursor(0, 48);
    display.setTextSize(2);
    formatTime(millis(), tbuf);
    display.print(tbuf);
  }

  display.display();
}
