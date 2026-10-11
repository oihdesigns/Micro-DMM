/*
 * Alerts.ino -- the buzzer/LED on CONTINUITY_PIN, the RGB LED, and the flags
 * the M4 core turns into sound.
 *
 * Continuity wins over the voltage warning.  Each fires one bright pulse on
 * the rising edge, then a dimmer repeat pattern while the condition holds.
 * The M4 also watches CONTINUITY_PIN, so every pulse here is a tone there.
 */

void updateAlerts() {
  unsigned long now = millis();

  if (flashlightMode) {
    // TYPE_PIN held at boot: resistance mapped to LED brightness.
    float pwmValue;
    if (currentResistance >= 0.0f && currentResistance <= 10000.0f) pwmValue = currentResistance;
    else if (currentResistance < 0.0f || currentResistance > 3000000.0f) pwmValue = 0.0f;
    else pwmValue = 10000.0f;
    analogWrite(CONTINUITY_PIN, 255 - (int)round(255 * (pwmValue / 10000.0f)));
    return;
  }

  // Continuity: a low resistance reading, or the bridge finding the leads
  // closed.  The resistance clauses are gated on resistanceMeasured -- without
  // it, leaving a resistance mode with the leads shorted kept the alert on for
  // good, because currentResistance still held the last low reading.
  // && binds tighter than ||, so blinkLimit gates ONLY the bridge clause, as
  // in the original.
  // Worked out before the ALERTS check so a silenced meter still reports it:
  // the host can beep instead.
  continuity = (resistanceMeasured &&
                (isBetween(currentResistance, cfg.contMin, cfg.contMax) ||
                 (isBetween(currentResistance, cfg.contMin, cfg.contMaxHi) &&
                  ohmsHighRange))) ||
               ((Vzero && vClosed) && blinkLimit < cfg.blinkLimit);

  // Pushed the moment it changes, independent of !STREAM and its cadence, so
  // a host beeper is not waiting on the next $LIVE.
  static bool contSent = false;
  if (continuity != contSent) {
    contSent = continuity;
    Host.print(F("$CONT,"));
    Host.print(continuity ? 1 : 0);  Host.print(',');
    Host.println(currentResistance, 3);
  }

  if (!cfg.alertsOn) {
    analogWrite(CONTINUITY_PIN, 0);
    rgb.off();
    vFlag = false;
    rFlag = false;
    return;
  }

  // Same binding as found: blinkLimit gates ONLY the AltUnits clause.
  bool logicVoltage =
      (!altUnits && (fabs(newVoltageReading) > cfg.vAlert ||
                     (VACPresense && VAC > cfg.vacAlert))) ||
      ((altUnits && (fabs(newVoltageReading) > cfg.vAlertAlt ||
                     (VACPresense && VAC > cfg.vacAlertAlt))) &&
       blinkLimit < cfg.blinkLimit);

  const unsigned long phase = now % cfg.alertPerMs;

  if (continuity) {
    txFlag = false;                                   // low tone
    if (!rFlag) {
      analogWrite(CONTINUITY_PIN, cfg.beepBright);
      rgb.on(255, 255, 255);
      if (debugMode) {
        Host.print(F("$TRIG,cont,"));
        Host.print(bridgeV, 4);         Host.print(',');
        Host.print(averageVoltage, 4);  Host.print(',');
        Host.println(newVoltageReading, 4);
      }
      blinkLimit++;
      rFlag = true;
    } else if (phase <= cfg.alertOnMs ||
               isBetween(phase, cfg.alertP2OnMs, cfg.alertP2OffMs)) {
      analogWrite(CONTINUITY_PIN, cfg.beepHold);
      rgb.on(0, 0, 255);
    } else {
      analogWrite(CONTINUITY_PIN, 0);
      rgb.off();
    }

  } else if (logicVoltage) {
    txFlag = true;                                    // high tone
    if (!vFlag) {
      analogWrite(CONTINUITY_PIN, cfg.beepBright);
      rgb.on(255, 255, 255);
      blinkLimit++;
      vFlag = true;
    } else if (phase <= cfg.alertOnMs && !VACPresense) {
      analogWrite(CONTINUITY_PIN, cfg.beepHold);
      rgb.on(255, 0, 0);
    } else {
      analogWrite(CONTINUITY_PIN, 0);
      rgb.off();
    }

  } else {
    analogWrite(CONTINUITY_PIN, 0);
    rgb.off();
    vFlag = false;
    rFlag = false;
  }
}

// The M4 hums while a resistance is across the leads, and picks its alert
// pitch from txFlag.  Sent only on a change.
void updateM4() {
  humFlag = !voltageDisplay && resistanceMeasured &&
            isBetween(currentResistance, cfg.humMin, cfg.humMax);

  uint8_t pack = (txFlag ? 1 : 0) | (humFlag ? 2 : 0);
  if (pack != lastSent) {
    RPC.write(pack);
    lastSent = pack;
  }
}
