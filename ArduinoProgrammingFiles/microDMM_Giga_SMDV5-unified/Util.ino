/*
 * Util.ino -- buttons and the mic ladder, min/max bookkeeping, the on-board
 * log, the USB-stick CSV logs, and small shared helpers.
 */

bool isBetween(float value, float low, float high) {
  return (value >= low && value <= high);
}

// Writes "MM:SS" into out (TIME_BUF_LEN bytes).
// SIDE EFFECT, RELIED ON: also updates the global `secondsTime`, which the screen
// saver uses instead of reading the clock itself.
void formatTime(unsigned long milliseconds, char *out) {
  unsigned long totalSeconds = milliseconds / 1000;
  unsigned int  minutes      = totalSeconds / 60;
  secondsTime = totalSeconds % 60;
  snprintf(out, TIME_BUF_LEN, "%02u:%02u", minutes, secondsTime);
}

// Reset the tracked extremes.  The lead null deliberately survives.
void ReZero() {
  highR  = 0.0f;
  lowR   = 7000000.0f;
  highV  = -100.0f;
  lowV   = 100.0f;
  IHigh  = I_HIGH_RESET;
  ILow   = I_LOW_RESET;
  deltaV = 0.0f;
}

// Newest-first shift register of current, voltage at that moment, and time
// since the log was armed.
void logCurrentData(float voltage, float timeSec, float current) {
  for (int i = LOG_SIZE - 1; i > 0; --i) {
    loggedCurrents[i]    = loggedCurrents[i - 1];
    loggedVoltagesAtI[i] = loggedVoltagesAtI[i - 1];
    loggedTimeStamps[i]  = loggedTimeStamps[i - 1];
  }
  loggedCurrents[0]    = current;
  loggedVoltagesAtI[0] = voltage;
  loggedTimeStamps[0]  = timeSec - tLogStart;
}

// ==================================================================
//  THE REF ACTION
// ==================================================================
// Shared by the yellow touch button, the mic ladder and a short press of
// TYPE_PIN.  Type mode types the reading; otherwise it sets the voltage delta
// reference, or toggles the resistance null.
void refAction() {
  if (currentMode == Type && cfg.keyboardEn) {
    if (voltageDisplay) {
      Keyboard.printf("%.*f", vDigits, newVoltageReading);
    } else if (displayResistance < 1.0f) {
      Keyboard.printf("%.*f", rDigits + 2, displayResistance);
    } else {
      Keyboard.printf("%.*f", rDigits, displayResistance);
    }
    Keyboard.key_code(RIGHT_ARROW);        // move to the next cell
  } else if (voltageDisplay) {
    deltaVdigits = vDigits - 1;
    deltaV = preciseMode ? newVoltageReading : averageVoltage;
  } else if (zeroOffsetRes != 0.0f) {
    zeroOffsetRes = 0.0f;                  // clearing never needs a live reading
  } else if (resistanceMeasured && !ohmsOpen) {
    // Setting one does.  Without the guard an open-lead reading (8M, or the
    // ~1.2G clamp value in High R) became the null: "0.0m", "null mOhms: ovf".
    zeroOffsetRes = currentResistance;
  }
}

// ==================================================================
//  BUTTONS
// ==================================================================
// TYPE_PIN: long press resets min/max, short press is the Ref action.
void handleButtonInput() {
  bool isPressed = (digitalRead(TYPE_PIN) == LOW);

  if (isPressed && !buttonPressed) {
    buttonPressed   = true;
    buttonPressTime = millis();
    return;
  }
  if (isPressed || !buttonPressed) return;

  unsigned long pressDuration = millis() - buttonPressTime;
  buttonPressed = false;
  analogWrite(CONTINUITY_PIN, 0);

  if (pressDuration > cfg.btnLongMs) {
    ReZero();
    MinMaxDisplay = true;
  } else if (pressDuration > cfg.btnShortMs) {
    refAction();
  }
}

// Mic-jack ladder on A7.  Sampled once per loop, and each action fires on the
// EDGE into its band, not while the level sits there -- level-triggered, a pin
// parked in a band re-fired every loop (toggling the null each pass in the Ref
// band).  Idle should read above MICMODEHI: pull A7 HIGH, not low, when
// nothing is plugged in -- low is a permanently held Ref button.
void pollMicLadder() {
  micLevel = analogRead(micPin);
  bool micOn = cfg.micEn;

  static bool minMaxWas = false;
  bool minMaxNow = bluePress ||
                   (micOn && isBetween(micLevel, cfg.micMMLo, cfg.micMMHi));
  if (minMaxNow && !minMaxWas) {
    MinMaxDisplay = true;
    ReZero();
  }
  minMaxWas = minMaxNow;

  static bool refWas = false;
  bool refNow = yellowPress || (micOn && micLevel < cfg.micRefMax);
  if (refNow && !refWas) refAction();
  refWas = refNow;
}

// Mode button, red touch button, or the mic ladder's mode band.  The release
// test uses the same union, so a level parked in the band fires once.
void checkModeButton() {
  unsigned long currentTime = millis();
  bool pressedNow = (digitalRead(BUTTON_PIN) == LOW) || redPress ||
                    (cfg.micEn && isBetween(micLevel, cfg.micModeLo, cfg.micModeHi));

  if (pressedNow && !buttonPreviouslyPressed &&
      (currentTime - lastDebounceTime + 100 > DEBOUNCE_DELAY)) {
    lastDebounceTime        = currentTime;
    buttonPreviouslyPressed = true;
    currentMode = static_cast<Mode>((currentMode + 1) % NUM_MODES);
    Host.print(F("$MODE,"));
    Host.println(currentMode);
  }
  if (!pressedNow && buttonPreviouslyPressed) {
    buttonPreviouslyPressed = false;
  }
}

// ==================================================================
//  MANUAL LOG (logButton / green touch / !LOG)
// ==================================================================
// Fills the on-board buffer at loop rate, then writes it to the USB stick.
void serviceManualLog() {
  static bool wasLogging = false;
  if (takeLog && !wasLogging) {
    tLogStart    = millis() / 1000.0f;
    samplesTaken = 0;
  }
  wasLogging = takeLog;
  if (!takeLog) return;

  if (samplesTaken < SAMPLE_COUNT + 1) {
    logCurrentData(newVoltageReading, millis() / 1000.0f, Ireading);
    samplesTaken++;
    return;
  }
  takeLog      = false;
  wasLogging   = false;
  tLogEnd      = millis() / 1000.0f;
  samplesTaken = 0;
  analogWrite(CONTINUITY_PIN, 100);        // audible "log finished"
  rgb.on(0, 255, 0);
  Host.println(F("$INFO,log,complete"));
  if (logMode) {
    manuallogArraysToCSV();
    manuallogCount++;
  }
}

// ==================================================================
//  USB-STICK AUTOLOG
// ==================================================================
#define NUM_AUTO_VALUES 32
static const float AUTOLOG_ISTEP = 0.1f;   // A of change that triggers a sample
float VAutoArray[NUM_AUTO_VALUES] = {0};
float TAutoArray[NUM_AUTO_VALUES] = {0};
float IAutoArray[NUM_AUTO_VALUES] = {0};

static void shiftAndStore(float *array, float newValue) {
  for (int i = NUM_AUTO_VALUES - 1; i > 0; i--) array[i] = array[i - 1];
  array[0] = newValue;
}

// Captures a sample on a new extreme or a current step, and writes the CSV on
// the first pass after the triggers stop.  Returns true on a capture pass, in
// which the caller skips the screen refresh.
bool serviceAutoLog() {
  static bool  triggered   = false;
  static bool  writePrimed = false;
  static float lastLoggedI = 0.0f;

  if (!triggered && writePrimed) {
    autologArraysToCSV();
    analogWrite(CONTINUITY_PIN, 100);
    rgb.on(0, 255, 0);
    autologCount++;
    Host.print(F("$INFO,autolog,"));
    Host.println(autologCount);
    writePrimed = false;
  }

  bool newExtreme = (IHigh == Ireading && IHigh > 0.05f) ||
                    (ILow == Ireading && ILow < -0.05f) ||
                    (newVoltageReading == highV && highV > 0.05f) ||
                    (newVoltageReading == lowV && lowV < -0.05f);
  bool iStep = fabs(lastLoggedI) > fabs(Ireading) + AUTOLOG_ISTEP ||
               fabs(lastLoggedI) < fabs(Ireading) - AUTOLOG_ISTEP;

  if (logMode && !VACPresense && (newExtreme || iStep)) {
    tLogEnd = millis() / 1000.0f;
    shiftAndStore(VAutoArray, newVoltageReading);
    shiftAndStore(TAutoArray, millis() / 1000.0f);
    shiftAndStore(IAutoArray, Ireading);
    triggered   = true;
    writePrimed = true;
    lastLoggedI = Ireading;
    return true;
  }
  triggered = false;
  return false;
}

// ==================================================================
//  CSV
// ==================================================================
static void printArrayCSV(FILE *f, const float *arr, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    fprintf(f, "%.3f", arr[i]);
    if (i + 1 < len) fputc(',', f);
  }
  fputc('\n', f);
}

void manuallogArraysToCSV() {
  FILE *f = fopen("/usb/data.csv", "a");
  if (!f) { Host.println(F("$ERR,usb,could not open data.csv")); return; }
  fprintf(f, "DataLog End Time: %.2f\n", tLogEnd);
  printArrayCSV(f, loggedVoltagesAtI, LOG_SIZE);
  if (currentOnOff) printArrayCSV(f, loggedCurrents, LOG_SIZE);
  printArrayCSV(f, loggedTimeStamps, LOG_SIZE);
  fclose(f);
}

void autologArraysToCSV() {
  FILE *f = fopen("/usb/data_autologged.csv", "a");
  if (!f) { Host.println(F("$ERR,usb,could not open data_autologged.csv")); return; }
  fprintf(f, "DataLogged End Time: %.2f\n", tLogEnd);
  printArrayCSV(f, VAutoArray, NUM_AUTO_VALUES);
  if (currentOnOff) printArrayCSV(f, IAutoArray, NUM_AUTO_VALUES);
  printArrayCSV(f, TAutoArray, NUM_AUTO_VALUES);
  fclose(f);
}
