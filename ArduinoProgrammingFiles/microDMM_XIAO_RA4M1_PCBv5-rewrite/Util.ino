/*
 * Util.ino -- buttons, min/max bookkeeping, the current log buffer, and small
 * shared helpers.
 */

bool isBetween(float value, float low, float high) {
  return (value >= low && value <= high);
}

// Writes "MM:SS" into out, which must hold TIME_BUF_LEN bytes.
//
// SIDE EFFECT, RELIED ON: this also updates the global `seconds`, and the
// screen-saver animation in Display.ino uses that global rather than reading
// the clock itself.  Calling formatTime purely for the side effect is
// deliberate there.  Do not make `seconds` local.
void formatTime(unsigned long milliseconds, char *out) {
  unsigned long totalSeconds = milliseconds / 1000;
  unsigned int  minutes      = totalSeconds / 60;
  seconds = totalSeconds % 60;
  snprintf(out, TIME_BUF_LEN, "%02u:%02u", minutes, seconds);
}

// Reset the tracked extremes.  zeroOffsetRes and initialZeroSet deliberately
// survive: the lead null is a calibration the user set, not a measurement.
void ReZero() {
  highR   = 0.0f;
  lowR    = 7000000.0f;
  highV   = -100.0f;
  lowV    = 100.0f;
  IHigh   = I_HIGH_RESET;
  ILow    = I_LOW_RESET;
  deltaV  = 0.0f;
}

// Newest-first shift register of current, the voltage at that moment, and the
// time since the log was armed.  Small enough that shifting beats the
// bookkeeping of a ring buffer, and it keeps the dump in time order.
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
//  BUTTONS
// ==================================================================
// TYPE_PIN: long press resets min/max, short press either types the reading
// over USB HID (Type / HighRMode) or sets a reference -- a delta baseline in
// voltage, a lead null in resistance.
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
    return;
  }
  if (pressDuration <= cfg.btnShortMs) return;      // contact bounce

  if ((currentMode == Type || currentMode == HighRMode) && cfg.keyboardEn) {
    if (voltageDisplay) {
      Keyboard.print(newVoltageReading, vDigits);
    } else if (displayResistance < 1.0f) {
      Keyboard.print(displayResistance, rDigits + 2);
    } else {
      Keyboard.print(displayResistance, rDigits);
    }
    Keyboard.press(0xD7);           // right arrow -- move to the next cell
    Keyboard.releaseAll();
  } else if (voltageDisplay) {
    deltaVdigits = vDigits - 1;
    deltaV = preciseMode ? newVoltageReading : averageVoltage;
  } else {
    zeroOffsetRes = (zeroOffsetRes == 0.0f) ? currentResistance : 0.0f;
  }
}

// MODE_BUTTON: cycles currentMode.
void checkModeButton() {
  bool buttonState = digitalRead(MODE_BUTTON);
  unsigned long currentTime = millis();

  if (buttonState == LOW && !buttonPreviouslyPressed &&
      (currentTime - lastDebounceTime > DEBOUNCE_DELAY)) {
    lastDebounceTime       = currentTime;
    buttonPreviouslyPressed = true;
    currentMode = static_cast<Mode>((currentMode + 1) % NUM_MODES);
    Serial.print(F("$MODE,"));
    Serial.println(currentMode);
  }

  if (buttonState == HIGH && buttonPreviouslyPressed) {
    buttonPreviouslyPressed = false;
  }
}
