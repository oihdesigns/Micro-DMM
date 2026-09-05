/*
 * Alerts.ino -- the buzzer / LED on CONTINUITY_PIN.
 *
 * Two alert conditions share one pin, and continuity wins when both are true.
 * Each fires one bright pulse on the rising edge, then settles into a dimmer
 * repeat pattern for as long as the condition holds: continuity pulses twice
 * per window, the voltage warning once.
 *
 * The flashlight mode that used to wrap this whole function (button held at
 * boot, resistance mapped to LED brightness) has been removed.
 */

void updateAlerts() {
  unsigned long now = millis();

  if (!cfg.alertsOn) {
    analogWrite(CONTINUITY_PIN, 0);
    vFlag = false;
    rFlag = false;
    return;
  }

  // Continuity, from two independent sources: a low resistance reading, or the
  // bridge finding the leads closed.
  //
  // The resistance clauses are gated on resistanceMeasured.  Without it they
  // read a value the meter has stopped refreshing -- switch out of a
  // resistance mode while the leads are shorted and the alert stays on for
  // good, because currentResistance still holds the last low reading it took.
  // The bridge clause needs no such gate; it runs in the voltmeter modes and
  // is exactly what should still raise continuity there.
  //
  // Parenthesised to the binding the pre-rewrite code had: && binds tighter
  // than ||, so blinkLimit gates ONLY the bridge clause.  Left as found.
  continuity = (resistanceMeasured &&
                (isBetween(currentResistance, cfg.contMin, cfg.contMax) ||
                 (isBetween(currentResistance, cfg.contMin, cfg.contMaxHi) &&
                  ohmsHighRange))) ||
               ((Vzero && !vFloating) && blinkLimit < cfg.blinkLimit);

  // Voltage warning.  Same note: blinkLimit gates ONLY the AltUnits clause.
  bool logicVoltage =
      (!altUnits && (fabs(newVoltageReading) > cfg.vAlert ||
                     (VACPresense && VAC > cfg.vacAlert))) ||
      ((altUnits && (fabs(newVoltageReading) > cfg.vAlertAlt ||
                     (VACPresense && VAC > cfg.vacAlertAlt))) &&
       blinkLimit < cfg.blinkLimit);

  const unsigned long phase = now % cfg.alertPerMs;

  if (continuity) {
    if (!rFlag) {
      analogWrite(CONTINUITY_PIN, cfg.beepBright);
      if (debugMode) {
        // Was an unconditional print; gated now so it cannot interleave with
        // the structured $ stream the GUI parses.
        Serial.print(F("$TRIG,cont,"));
        Serial.print(bridgeV, 4);         Serial.print(',');
        Serial.print(averageVoltage, 4);  Serial.print(',');
        Serial.println(newVoltageReading, 4);
      }
      blinkLimit++;
      rFlag = true;
    } else if (phase <= cfg.alertOnMs ||
               isBetween(phase, cfg.alertP2OnMs, cfg.alertP2OffMs)) {
      analogWrite(CONTINUITY_PIN, cfg.beepHold);
    } else {
      analogWrite(CONTINUITY_PIN, 0);
    }

  } else if (logicVoltage) {
    if (!vFlag) {
      analogWrite(CONTINUITY_PIN, cfg.beepBright);
      blinkLimit++;
      vFlag = true;
    } else if (phase <= cfg.alertOnMs && !VACPresense) {
      analogWrite(CONTINUITY_PIN, cfg.beepHold);
    } else {
      analogWrite(CONTINUITY_PIN, 0);
    }

  } else {
    analogWrite(CONTINUITY_PIN, 0);
    vFlag = false;
    rFlag = false;
  }
}
