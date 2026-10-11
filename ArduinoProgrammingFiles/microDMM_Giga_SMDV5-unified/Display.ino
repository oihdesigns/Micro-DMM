/*
 * Display.ino -- GIGA Display rendering, the four touch buttons, the plots,
 * and the value formatters.
 *
 * Layout is the pre-config Giga firmware's.  The formatters hand back a
 * const char* suffix instead of an Arduino String (six per refresh).
 */

// ---- Touch buttons ----
const uint16_t btnX[4]     = { 700, 600, 700, 600 };
const uint16_t btnY[4]     = {   0,   0, 100, 100 };
const uint16_t btnColor[4] = { RED, GREEN, BLUE, YELLOW };
const char    *btnLabel[4] = { "MODE", "LOG", "MinMax", "Ref" };

// ---- Plot ----
static const int SAMPLE_COUNT_PLOT = 100;
static const int PLOT_SIZE = 400;
static const int PLOT_X    = 0;
static const int PLOT_Y    = 80;
#define COLOR_BG     0x0000
#define COLOR_DATA   0xFFFF
#define COLOR_MIN    0x07E0
#define COLOR_MEAN   0xFFE0
#define COLOR_MAX    0xF800

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

const char *modeToString(Mode m) {
  switch (m) {
    case Voltmeter:    return "Voltmeter";
    case HighRMode:    return "High R";
    case Default:      return "Default";
    case VACmanual:    return "VAC";
    case Type:         return "Type";
    case Low:          return "Low";
    case AltUnitsMode: return "AltUnits";
    case rPlotMode:    return "rPlotMode";
    case Charging:     return "Charging";
    default:           return "Unknown";
  }
}

// ==================================================================
//  BUTTONS
// ==================================================================
void drawButtons() {
  display.setTextSize(2);
  for (int i = 0; i < 4; i++) {
    display.fillRect(btnX[i], btnY[i], 100, 100, BLACK);
    display.fillRect(btnX[i] + 10, btnY[i] + 10, 80, 80, btnColor[i]);
    display.setTextColor((btnColor[i] == YELLOW) ? BLACK : WHITE);
    int16_t w  = strlen(btnLabel[i]) * 6 * 2;   // 6 px per char x size 2
    int16_t h  = 8 * 2;
    display.setCursor(btnX[i] + (100 - w) / 2, btnY[i] + (100 - h) / 2);
    display.print(btnLabel[i]);
  }
  display.setTextColor(WHITE);
}

// Sets the four press flags for this pass.  A held finger repeats every 500 ms.
void pollTouch(unsigned long now) {
  static unsigned long previousTouchTime = 0;
  redPress = greenPress = bluePress = yellowPress = false;

  GDTpoint_t points[5];
  uint8_t contacts = touch.getTouchPoints(points);
  if (contacts == 0 || now < previousTouchTime + 500) return;
  previousTouchTime = now;

  // Map the raw touch point into the rotation-1 screen frame.
  uint16_t x = points[0].y;
  uint16_t y = display.width() - points[0].x - 350;

  if (debugMode) {
    Host.print(F("$TOUCH,"));
    Host.print(points[0].x); Host.print(','); Host.print(points[0].y);
    Host.print(','); Host.print(x); Host.print(','); Host.println(y);
  }

  for (int i = 0; i < 4; i++) {
    if (x >= btnX[i] && x < btnX[i] + 100 && y >= btnY[i] && y < btnY[i] + 100) {
      switch (i) {
        case 0: redPress    = true; break;
        case 1: greenPress  = true; break;
        case 2: bluePress   = true; break;
        case 3: yellowPress = true; break;
      }
      break;
    }
  }
}

// ==================================================================
//  SCREEN
// ==================================================================
void updateDisplay() {
  char tbuf[TIME_BUF_LEN];

  blinkLimit = 0;
  display.fillRect(0, 0, 256, 72, BLACK);
  display.setTextColor(WHITE, BLACK);

  const int gfxLine = 8;
  const int mmHomeX = 192;
  const int mmHomeY = 0;

  if (screenSleep || currentMode == Charging) {
    // A single dot walking across, so the panel is not holding a static image.
    display.fillRect(0, 0, 500, 480, BLACK);
    formatTime(millis(), tbuf);
    display.setCursor(8 + 4 * (secondsTime % 10), 32);
    display.setTextSize(1);
    display.print(".");
    return;
  }

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

  // --- Mode, unit and clock (bottom right) ---
  display.fillRect(660, 460 - gfxLine * 4, 140, 52, BLACK);
  display.setTextSize(2);
  display.setCursor(660, 460 - gfxLine * 4);
  if (cfg.hwRev) { display.print("Unit "); display.print(cfg.hwRev); }
  else           { display.print("Unit ?"); }
  if (cfgDirty) display.print(" *");
  display.setCursor(660, 460 - gfxLine * 2);
  display.print(modeToString(currentMode));
  display.setCursor(660, 460);
  formatTime(millis(), tbuf);
  display.print(tbuf);

  // --- USB logging ---
  if (logMode) {
    const int logHomeX = 660, logHomeY = 350;
    display.setCursor(logHomeX, logHomeY);
    display.print("USB?: ");    display.print(logMode);
    display.setCursor(logHomeX, logHomeY + gfxLine * 2);
    display.print("Auto#: ");   display.print(autologCount);
    display.setCursor(logHomeX, logHomeY + gfxLine * 4);
    display.print("Manual#: "); display.print(manuallogCount);
  }

  // --- Current ---
  if (currentOnOff) {
    const int IHomeX = 424, IHomeY = 0;
    display.fillRect(IHomeX, IHomeY, 128, 72, BLACK);
    display.setTextSize(2);
    display.setCursor(IHomeX, IHomeY);
    if (Irange) {
      display.print("A:");
      display.print(Ireading, 2);
    } else {
      display.print("mA:");
      if      (Ireading > 0.100f) display.print(Ireading * 1000.0f, 2);
      else if (Ireading > 0.010f) display.print(Ireading * 1000.0f, 3);
      else                        display.print(Ireading * 1000.0f, 4);
    }
    if (IHigh != I_HIGH_RESET) {
      display.setCursor(IHomeX, IHomeY + gfxLine * 2);
      display.print("Ilow:");  display.print(ILow, 3);
      display.print(" t:");    display.print(timeAtMinI);
      display.setCursor(IHomeX, IHomeY + gfxLine * 4);
      display.print("Ihigh:"); display.print(IHigh, 3);
      display.print(" t:");    display.print(timeAtMaxI);
    }
  }

  // --- Primary reading ---
  const int PrimeX = 0, PrimeY = 0;
  display.setTextSize(3);
  display.setCursor(PrimeX, PrimeY);

  if (voltageDisplay) {
    if (Vzero) {
      if      (vFloating)  display.print("V FLT:");
      else if (vUndefined) display.print("V UNDF:");
      else                 display.print("V CLD:");
      display.print(bridgeV * 1000.0f, 0);
      display.print("m");
    } else {
      if (currentMode == VACmanual) {
        display.print("VAC:");
        display.print(VAC, altUnits ? 1 : 3);
      } else {
        display.print("VDC:");
        if (preciseMode) {
          display.setCursor(PrimeX, PrimeY + gfxLine * 4);
          display.print(roundedV, vDigits + 1);
        } else {
          display.print(roundedV, vDigits);
        }
        display.println(vSuffix);
      }

      if (deltaV != 0) {
        display.setTextSize(2);
        display.setCursor(PrimeX, PrimeY + gfxLine * 4);
        display.print("ref:");
        display.println(deltaV, deltaVdigits);
        display.setCursor(PrimeX, PrimeY + gfxLine * 6);
        display.print("delta:");
        display.println(newVoltageReading - deltaV, 4);
      }
    }

    if (MinMaxDisplay) {
      display.fillRect(mmHomeX, mmHomeY, 232, 72, BLACK);
      display.setTextSize(2);
      display.setCursor(mmHomeX, mmHomeY);
      display.print("Min:");  display.print(roundedVlow, vDigitslow);
      display.print(vSuffixlow);
      display.print("  t:");  display.print(timeAtMinV);
      display.setCursor(mmHomeX, mmHomeY + gfxLine * 2);
      display.print("Max:");  display.print(roundedVhigh, vDigitshigh);
      display.print(vSuffixhigh);
      display.print("  t:");  display.print(timeAtMaxV);
      display.setCursor(mmHomeX, mmHomeY + gfxLine * 4);
      display.print("Range:");
      display.print(highV - lowV, 3);
    }

    if (currentMode != Low) {
      display.setCursor(PrimeX, PrimeY + gfxLine * 6);
      display.setTextSize(2);
      if (currentMode == VACmanual) {
        display.print("VDC: ");
        display.print(roundedV, vDigits);
      } else {
        display.print("VAC: ");
        display.print(VAC, vDigits);
      }
    }

    if (!vFloating) {                     // only plot when there is a signal
      display.setTextSize(2);
      drawPlot(voltageSamples, SAMPLE_COUNT_PLOT);
    }

  } else {
    // --- Resistance ---
    if (!ohmsOpen) {
      display.print(altUnits ? "F:" : "R:");
      display.print(roundedR, rDigits);
      display.print(rSuffix);

      if (MinMaxDisplay) {
        display.setTextSize(2);
        display.setCursor(mmHomeX, mmHomeY);
        display.print("Min:");  display.print(roundedRlow, rDigitslow);
        display.print(rSuffixlow);
        display.setCursor(mmHomeX, mmHomeY + gfxLine * 2);
        display.print("Max:");  display.print(roundedRhigh, rDigitshigh);
        display.print(rSuffixhigh);
      }
      display.setTextSize(2);
      display.setCursor(PrimeX, PrimeY + gfxLine * 4);
      display.print("mVR:");
      display.print(ohmsVoltage * 1000.0f, ohmsVoltage < 1.0f ? 1 : 0);

      display.setCursor(PrimeX, PrimeY + gfxLine * 6);
      if (zeroOffsetRes != 0) {
        display.print("null mOhms:");
        display.println(zeroOffsetRes * 1000.0f, 1);
      }
      if (debugMode) {
        display.setCursor(60, 32);
        display.print("ADC:");
        display.print(adcCount);
      }
    } else {
      if (!altUnits) {
        display.println("R:OPEN");
        display.setCursor(PrimeX, PrimeY + gfxLine * 4);
        display.setTextSize(2);
        display.print("mV:");
        display.print(ohmsVoltage * 1000.0f, 2);
      } else {
        display.print("F:OPEN");
      }
      if (MinMaxDisplay) {
        display.setTextSize(2);
        display.setCursor(mmHomeX, mmHomeY);
        display.print("Min:");  display.print(roundedRlow, rDigitslow);
        display.print(rSuffixlow);
        display.setCursor(mmHomeX, mmHomeY + gfxLine * 2);
        display.print("Max:");  display.print(roundedRhigh, rDigitshigh);
        display.print(rSuffixhigh);
      }
    }

    if (currentMode == rPlotMode || currentMode == HighRMode) {
      display.setTextSize(2);
      drawPlot(resistanceSamples, SAMPLE_COUNT_PLOT);
    }
  }
}

// ==================================================================
//  PLOT
// ==================================================================
static void drawStatLine(float value, float plotMin, float yScale, uint16_t color) {
  int y = PLOT_Y + PLOT_SIZE - int((value - plotMin) * yScale);
  const int dashLen = 6;
  for (int x = PLOT_X; x < PLOT_X + PLOT_SIZE; x += dashLen) {
    display.drawFastHLine(x, y, dashLen / 2, color);
  }
  display.setCursor(PLOT_X + PLOT_SIZE + 5, y - 4);
  display.setTextColor(color);
  display.print(value, 3);
  display.setTextColor(WHITE, BLACK);
}

void drawPlot(const float data[], int n) {
  float minY = data[0], maxY = data[0], sumY = data[0];
  for (int i = 1; i < n; i++) {
    float v = data[i];
    sumY += v;
    if (v < minY) minY = v;
    if (v > maxY) maxY = v;
  }
  float meanY = sumY / n;
  float rangeY;

  if (!MinMaxDisplay) {
    if (maxY - minY < 0.05f && voltageDisplay && currentMode == Low)      rangeY = 0.1f;
    else if (maxY - minY < 0.2f && voltageDisplay && currentMode != Low)  rangeY = 0.2f;
    else                                                                  rangeY = maxY - minY;
  } else {
    minY   = lowV;
    maxY   = highV;
    rangeY = maxY - minY;
  }

  float margin  = rangeY * 0.05f;
  float plotMin = minY - margin * 0.5f;
  float plotMax = maxY + margin * 0.5f;
  if (plotMax - plotMin <= 0.0f) plotMax = plotMin + 1e-6f;   // flat trace: avoid /0
  float yScale  = float(PLOT_SIZE) / (plotMax - plotMin);

  display.fillRect(PLOT_X, PLOT_Y, PLOT_SIZE + 100, PLOT_SIZE, COLOR_BG);

  for (int i = 0; i < n - 1; i++) {
    int x1 = PLOT_X + (i       * PLOT_SIZE) / (n - 1);
    int y1 = PLOT_Y + PLOT_SIZE - int((data[i]     - plotMin) * yScale);
    int x2 = PLOT_X + ((i + 1) * PLOT_SIZE) / (n - 1);
    int y2 = PLOT_Y + PLOT_SIZE - int((data[i + 1] - plotMin) * yScale);
    display.drawLine(x1, y1, x2, y2, COLOR_DATA);
  }

  drawStatLine(minY,  plotMin, yScale, COLOR_MIN);
  drawStatLine(meanY, plotMin, yScale, COLOR_MEAN);
  drawStatLine(maxY,  plotMin, yScale, COLOR_MAX);
}
