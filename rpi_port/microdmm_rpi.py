"""Raspberry Pi hardware abstraction for the Micro-DMM Giga firmware.

This module mirrors the pin behaviour documented in ``HARDWARE_BEHAVIOR.md`` and
provides Linux-friendly replacements for the Arduino helpers that the original
sketch relies on (``millis()``, ``analogWrite()``, and ``analogRead()``).  It
uses commonly available Raspberry Pi libraries so the measurement control logic
can be reused on a single-board computer without rewriting every hardware
interaction.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import RPi.GPIO as GPIO
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn


_GAIN_LSB = {
    ADS.Gain.TWOTHIRDS: 0.1875e-3,
    ADS.Gain.ONE: 0.125e-3,
    ADS.Gain.TWO: 0.0625e-3,
    ADS.Gain.FOUR: 0.03125e-3,
    ADS.Gain.EIGHT: 0.015625e-3,
    ADS.Gain.SIXTEEN: 0.0078125e-3,
}


def monotonic_millis() -> int:
    """Return a monotonic millisecond counter (Arduino ``millis`` equivalent)."""

    return math.floor(time.monotonic() * 1000.0)


@dataclass(frozen=True)
class Pinout:
    """Bundle of Raspberry Pi GPIO assignments for the Micro-DMM hardware."""

    ohm_pwm: int
    set_range: int
    continuity_pwm: int
    voltage_bridge: int
    cycle_track: Optional[int] = None
    mode_button: Optional[int] = None
    log_button: Optional[int] = None
    kb_mode_select: Optional[int] = None


class PWMOutput:
    """Helper that emulates Arduino ``analogWrite`` with an 8-bit duty input."""

    def __init__(self, pin: int, frequency_hz: float) -> None:
        self._pin = pin
        self._pwm = GPIO.PWM(pin, frequency_hz)
        self._pwm.start(0.0)

    def write_level8(self, value: int) -> None:
        value = max(0, min(255, value))
        duty_cycle = (value / 255.0) * 100.0
        self._pwm.ChangeDutyCycle(duty_cycle)

    def stop(self) -> None:
        self._pwm.stop()


class DebouncedInput:
    """Mirror of the Arduino debounce helper used by ``checkModeButton``."""

    def __init__(self, delay_ms: int) -> None:
        self._delay_ms = delay_ms
        self._last_transition_ms = monotonic_millis()
        self._last_state = True
        self._stable_state = True

    def update(self, raw_state: bool, now_ms: Optional[int] = None) -> bool:
        now_ms = monotonic_millis() if now_ms is None else now_ms
        if raw_state != self._last_state:
            self._last_state = raw_state
            self._last_transition_ms = now_ms
        elif (now_ms - self._last_transition_ms) >= self._delay_ms:
            self._stable_state = raw_state
        return self._stable_state


class IntervalTimer:
    """Utility that mirrors the Arduino ``previousXYZMillis`` pattern."""

    def __init__(self, period_ms: int) -> None:
        self._period_ms = period_ms
        self._last_fire_ms = monotonic_millis()

    def ready(self, now_ms: Optional[int] = None) -> bool:
        now_ms = monotonic_millis() if now_ms is None else now_ms
        if (now_ms - self._last_fire_ms) >= self._period_ms:
            self._last_fire_ms = now_ms
            return True
        return False


class MicroDMMHardware:
    """Hardware abstraction that drives the Micro-DMM front-end from Linux."""

    def __init__(
        self,
        pins: Pinout,
        *,
        pwm_frequency_hz: float = 976.6,
        ads_address: int = 0x48,
        ohms_channel: int = 2,
        mic_channel: int = 0,
        mic_full_scale: int = 0x7FFFFF,
    ) -> None:
        self._pins = pins
        self._ohms_channel = ohms_channel
        self._mic_channel = mic_channel
        self._mic_full_scale = mic_full_scale

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pins.set_range, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(pins.voltage_bridge, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(pins.ohm_pwm, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(pins.continuity_pwm, GPIO.OUT, initial=GPIO.LOW)
        if pins.cycle_track is not None:
            GPIO.setup(pins.cycle_track, GPIO.OUT, initial=GPIO.LOW)
        if pins.mode_button is not None:
            GPIO.setup(pins.mode_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if pins.log_button is not None:
            GPIO.setup(pins.log_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if pins.kb_mode_select is not None:
            GPIO.setup(pins.kb_mode_select, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self._ohm_pwm = PWMOutput(pins.ohm_pwm, pwm_frequency_hz)
        self._continuity_pwm = PWMOutput(pins.continuity_pwm, pwm_frequency_hz)

        i2c = busio.I2C(board.SCL, board.SDA)
        self._ads = ADS.ADS1115(i2c, address=ads_address)
        self._ads.mode = ADS.Mode.CONTINUOUS
        self._ads.gain = ADS.Gain.TWOTHIRDS
        self._lsb = _GAIN_LSB[self._ads.gain]
        self._ads_inputs = (
            AnalogIn(self._ads, ADS.P0),
            AnalogIn(self._ads, ADS.P1),
            AnalogIn(self._ads, ADS.P2),
            AnalogIn(self._ads, ADS.P3),
        )

        from ads1256 import ADS1256

        self._mic_adc = ADS1256()
        self._mic_adc.calibrate()

    # ----- Arduino helper replacements -----
    def analog_write_ohms_pwm(self, value: int) -> None:
        """Drive the constant-current MOSFET with an 8-bit PWM value."""

        self._ohm_pwm.write_level8(value)

    def analog_write_continuity(self, value: int) -> None:
        self._continuity_pwm.write_level8(value)

    def read_mode_button(self) -> Optional[bool]:
        if self._pins.mode_button is None:
            return None
        return GPIO.input(self._pins.mode_button) == GPIO.HIGH

    def read_log_button(self) -> Optional[bool]:
        if self._pins.log_button is None:
            return None
        return GPIO.input(self._pins.log_button) == GPIO.HIGH

    # ----- Range control -----
    def set_high_range(self) -> None:
        GPIO.output(self._pins.set_range, GPIO.HIGH)

    def set_low_range(self) -> None:
        GPIO.output(self._pins.set_range, GPIO.LOW)

    def set_voltage_bridge(self, enabled: bool) -> None:
        GPIO.output(self._pins.voltage_bridge, GPIO.HIGH if enabled else GPIO.LOW)

    def toggle_cycle_track(self, state: bool) -> None:
        if self._pins.cycle_track is not None:
            GPIO.output(self._pins.cycle_track, GPIO.HIGH if state else GPIO.LOW)

    # ----- ADS1115 measurement helpers -----
    def set_ads_gain(self, gain: ADS.Gain) -> None:
        self._ads.gain = gain
        self._lsb = _GAIN_LSB[gain]

    def read_ohms_counts(self) -> int:
        analog = self._ads_inputs[self._ohms_channel]
        voltage = analog.voltage
        return int(round(voltage / self._lsb))

    def read_bridge_voltage(self) -> float:
        diff = AnalogIn(self._ads, ADS.P0, ADS.P1)
        return diff.voltage

    # ----- Microphone channel -----
    def read_mic_counts(self) -> int:
        raw = self._mic_adc.read_adc(self._mic_channel)
        scaled = (raw + self._mic_full_scale) / (2 * self._mic_full_scale)
        return int(round(scaled * 4095))

    # ----- Cleanup -----
    def close(self) -> None:
        self._ohm_pwm.stop()
        self._continuity_pwm.stop()
        GPIO.cleanup()
