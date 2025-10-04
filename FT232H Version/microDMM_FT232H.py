"""Desktop port of the Micro-DMM firmware for an FT232H + ADS1115 setup.

This module mirrors the core behaviour of the Arduino Giga firmware so the
multimeter can be driven from a Windows PC.  The program exposes a Tkinter GUI
that continuously polls the ADS1115 (connected through an FT232H breakout via
Blinka) and manipulates the digital outputs that replaced the Arduino pins.

The implementation focuses on the logic that directly interacts with external
hardware: voltage, resistance and current measurements plus the control pins
that drive the bridge MOSFET, continuity output, constant current PWM and the
cycle tracking output.  Graphical widgets replace the physical buttons and the
touch display that were present on the original firmware.

Run the file directly (``python microDMM_FT232H.py``) to launch the
application.  No command line arguments are required.
"""

from __future__ import annotations

import csv
import math
import sys
import time
import traceback
from datetime import datetime
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Optional, Tuple

try:  # Import Blinka hardware libraries lazily so the UI can still launch
    import board
    import busio
    import digitalio
    from adafruit_ads1x15.ads1115 import ADS1115
    from adafruit_ads1x15.analog_in import AnalogIn
except Exception:  # pragma: no cover - executed only when hardware libs absent
    board = None  # type: ignore
    busio = None  # type: ignore
    digitalio = None  # type: ignore
    ADS1115 = None  # type: ignore
    AnalogIn = None  # type: ignore

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
except Exception:  # pragma: no cover - optional dependency
    FigureCanvasTkAgg = None  # type: ignore
    Figure = None  # type: ignore


# ---------------------------------------------------------------------------
# Enumerations and simple containers
# ---------------------------------------------------------------------------


class Mode(Enum):
    """Subset of modes from the original firmware."""

    VOLTMETER = auto()
    DEFAULT = auto()
    VAC_MANUAL = auto()
    HIGH_R = auto()
    R_PLOT = auto()
    CHARGING = auto()


class MeasurementError(Exception):
    """Raised when the ADS1115 cannot be read."""


@dataclass
class MeasurementResult:
    voltage_dc: float = 0.0
    voltage_rms: float = 0.0
    voltage_avg: float = 0.0
    voltage_reference: float = 0.0
    resistance: float = 0.0
    current: float = 0.0
    bridge_voltage: float = 0.0
    vac_present: bool = False
    v_floating: bool = False


@dataclass
class OutputPin:
    """Wrapper around a DigitalInOut object with graceful degradation."""

    name: str
    pin: Optional["digitalio.DigitalInOut"] = None
    state: bool = False

    def set(self, value: bool) -> None:
        self.state = bool(value)
        if self.pin is not None:
            self.pin.value = self.state


# ---------------------------------------------------------------------------
# Instrument back-end
# ---------------------------------------------------------------------------


class MicroDMM:
    """Implements the measurement logic ported from the Arduino firmware."""

    # ADS1115 gain levels and matching mV-per-bit scale factors
    GAIN_LEVELS = [2 / 3, 1, 2, 4, 8, 16]
    GAIN_FACTORS = {
        2 / 3: 0.1875,
        1: 0.125,
        2: 0.0625,
        4: 0.03125,
        8: 0.015625,
        16: 0.0078125,
    }
    GAIN_RANGES = {
        2 / 3: 6.144,
        1: 4.096,
        2: 2.048,
        4: 1.024,
        8: 0.512,
        16: 0.256,
    }

    ADS_DATA_RATES = [8, 16, 32, 64, 128, 250, 475, 860]

    # Thresholds replicate the firmware behaviour
    ADC_COUNT_LOW_THRESHOLD = 10000
    ADC_COUNT_HIGH_THRESHOLD = 30000
    OHMS_RANGE_DEADBAND = 0.05

    # Calibration data copied from the original firmware
    RES_CAL_FACTORS = [
        0.9607,
        0.9848,
        0.997,
        0.9958,
        0.9934,
        0.9965,
        0.9982,
        1.0028,
        1.0012,
        1.0005,
        1.0017,
        1.0035,
        1.0075,
        1.0237,
        1.1226,
    ]
    RES_CAL_LIMITS = [
        0.75,
        3.0,
        7.0,
        20.0,
        70.0,
        170.0,
        700.0,
        1700.0,
        7000.0,
        17000.0,
        70000.0,
        170000.0,
        700000.0,
        1700000.0,
        float("inf"),
    ]

    # Resistance circuit constants
    CONSTANT_I = 0.02016  # A
    CONSTANT_R = 330.0  # Ohms
    DIVIDER_R = 22000.0  # Ohms

    EEPROM_SLEEP_V = 0.6119
    EEPROM_MAX_V = 4.979

    VOLTAGE_SCALE_POS = -68.36437
    VOLTAGE_SCALE_NEG = -68.3536

    # Windows GUI polling rate (ms)
    UPDATE_INTERVAL_MS = 100

    VOLTAGE_HISTORY_SIZE = 128

    # Mapping Arduino pins to FT232H "C" pins.  The entries are strings that
    # match attributes under ``board``.
    OUTPUT_MAPPING = {
        "OHMPWMPIN": "C2",  # Arduino D2
        "VbridgePin": "C5",  # Arduino D5
        "CONTINUITY_PIN": "C6",  # Arduino D6
        "SETRANGE_PIN": "C3",  # Arduino D7 (adjust if wired elsewhere)
        "cycleTrack": "C7",  # Arduino D52
    }

    MANUAL_OUTPUTS = ("OHMPWMPIN", "VbridgePin", "SETRANGE_PIN")
    MANUAL_PIN_LABELS = {
        "OHMPWMPIN": "R circuit (C2)",
        "VbridgePin": "V Bridge (C5)",
        "SETRANGE_PIN": "R Range (6)",
    }

    STATS_FIELDS = (
        "voltage_dc",
        "voltage_rms",
        "resistance",
        "current",
        "bridge_voltage",
    )

    def __init__(self) -> None:
        self.mode: Mode = Mode.VOLTMETER
        self.ohms_auto_range: bool = True
        self.ohms_high_range: bool = True
        self.precise_mode: bool = False
        self.alt_units: bool = False
        self.current_enabled: bool = False
        self.power_save: bool = False

        self.manual_sample_rate: Optional[int] = None
        self.manual_gain: Optional[float] = None

        # Measurement state
        self._voltage_gain_index = len(self.GAIN_LEVELS) - 1
        self._res_gain_index = 0
        self._current_gain_index = len(self.GAIN_LEVELS) - 1
        self._res_range_high = True
        self._last_res_value = 0.0
        self._zero_offset_res = 0.0
        self._initial_zero_set = False
        self._bridge_voltage = 0.0
        self._v_floating = False
        self._vac_present = False
        self._voltage_reference = 0.0

        self._voltage_buffer: list[float] = [0.0] * self.VOLTAGE_HISTORY_SIZE
        self._voltage_sq_buffer: list[float] = [0.0] * self.VOLTAGE_HISTORY_SIZE
        self._voltage_index = 0
        self._voltage_avg = 0.0
        self._voltage_sum = 0.0
        self._voltage_sq_sum = 0.0

        self._last_measurement: MeasurementResult = MeasurementResult()
        self._hardware_error: Optional[str] = None

        self._continuity_pattern_start = time.monotonic()
        self._continuity_active = False
        self._logic_voltage_active = False

        self._manual_outputs_enabled = False
        self._manual_output_states: Dict[str, bool] = {
            name: False for name in self.MANUAL_OUTPUTS
        }
        self._auto_output_cache: Dict[str, bool] = {
            name: False for name in self.OUTPUT_MAPPING
        }

        self._stats: Dict[str, Dict[str, Optional[float]]] = {
            field: {"min": None, "max": None} for field in self.STATS_FIELDS
        }

        self._i2c = None
        self._ads: Optional[ADS1115] = None
        self._chan_voltage: Optional[AnalogIn] = None
        self._chan_res: Optional[AnalogIn] = None
        self._chan_current: Optional[AnalogIn] = None
        self._outputs: Dict[str, OutputPin] = {}

        self._initialise_hardware()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def last_measurement(self) -> MeasurementResult:
        return self._last_measurement

    @property
    def hardware_ok(self) -> bool:
        return self._ads is not None and self._hardware_error is None

    @property
    def hardware_error(self) -> Optional[str]:
        return self._hardware_error

    @property
    def manual_outputs_enabled(self) -> bool:
        return self._manual_outputs_enabled

    def get_manual_output_state(self, name: str) -> bool:
        return self._manual_output_states.get(name, False)

    def toggle_cycle_track(self) -> None:
        pin = self._outputs.get("cycleTrack")
        if pin:
            pin.set(not pin.state)

    def zero_resistance(self) -> None:
        self._zero_offset_res = self._last_measurement.resistance
        self._initial_zero_set = True

    def set_voltage_reference(self) -> None:
        self._voltage_reference = self._last_measurement.voltage_dc

    def clear_voltage_reference(self) -> None:
        self._voltage_reference = 0.0

    def clear_vac_history(self) -> None:
        self._voltage_buffer = [0.0] * self.VOLTAGE_HISTORY_SIZE
        self._voltage_sq_buffer = [0.0] * self.VOLTAGE_HISTORY_SIZE
        self._voltage_sum = 0.0
        self._voltage_sq_sum = 0.0
        self._voltage_index = 0
        self._voltage_avg = 0.0
        self._vac_present = False

    def set_manual_sample_rate(self, rate: Optional[int]) -> None:
        if rate is not None and rate not in self.ADS_DATA_RATES:
            raise ValueError("Unsupported ADS1115 sample rate")
        self.manual_sample_rate = rate

    def set_manual_gain(self, gain: Optional[float]) -> None:
        if gain is not None and gain not in self.GAIN_LEVELS:
            raise ValueError("Unsupported ADS1115 gain")
        self.manual_gain = gain

    def set_manual_outputs_enabled(self, enabled: bool) -> None:
        self._manual_outputs_enabled = enabled
        if enabled:
            for name, state in self._manual_output_states.items():
                self._write_output(name, state)
        else:
            for name, state in self._auto_output_cache.items():
                self._write_output(name, state)

    def set_manual_output_state(self, name: str, state: bool) -> None:
        if name not in self.MANUAL_OUTPUTS:
            raise ValueError(f"Unknown manual output: {name}")
        self._manual_output_states[name] = state
        if self._manual_outputs_enabled:
            self._write_output(name, state)
        else:
            auto_state = self._auto_output_cache.get(name)
            if auto_state is not None:
                self._write_output(name, auto_state)

    def get_statistics(self) -> Dict[str, Tuple[Optional[float], Optional[float]]]:
        return {
            field: (values["min"], values["max"])
            for field, values in self._stats.items()
        }

    def reset_statistics(self) -> None:
        for field in self._stats.values():
            field["min"] = None
            field["max"] = None

    def measure(self) -> MeasurementResult:
        """Poll all sensors and update the cached measurement."""

        if not self.hardware_ok:
            raise MeasurementError(self._hardware_error or "ADS1115 unavailable")

        try:
            voltage = self._measure_voltage()
            resistance = self._measure_resistance()
            current = self._measure_current()
            rms = math.sqrt(max(self._voltage_sq_sum / len(self._voltage_sq_buffer), 0.0))
            result = MeasurementResult(
                voltage_dc=voltage,
                voltage_rms=rms,
                voltage_avg=self._voltage_avg,
                voltage_reference=self._voltage_reference,
                resistance=resistance,
                current=current,
                bridge_voltage=self._bridge_voltage,
                vac_present=self._vac_present,
                v_floating=self._v_floating,
            )
            self._update_constant_current_output()
            self._update_continuity_output(result)
            self._hardware_error = None
            self._update_statistics(result)
            self._last_measurement = result
            return result
        except Exception as exc:  # pragma: no cover - hardware dependent
            self._hardware_error = f"Measurement failed: {exc}"
            raise MeasurementError(self._hardware_error) from exc

    # ------------------------------------------------------------------
    # Hardware setup helpers
    # ------------------------------------------------------------------

    def _initialise_hardware(self) -> None:
        if ADS1115 is None or AnalogIn is None or busio is None or board is None:
            self._hardware_error = (
                "Required Blinka libraries not available. Install adafruit-blinka "
                "and adafruit-circuitpython-ads1x15."
            )
            return

        try:
            self._i2c = busio.I2C(board.SCL, board.SDA)
            self._ads = ADS1115(self._i2c)
            self._ads.data_rate = 128
            self._ads.gain = self.GAIN_LEVELS[self._voltage_gain_index]

            self._chan_voltage = AnalogIn(self._ads, 0, 1)
            self._chan_res = AnalogIn(self._ads, 2)
            self._chan_current = AnalogIn(self._ads, 3)

            for logical, pin_name in self.OUTPUT_MAPPING.items():
                try:
                    dio = digitalio.DigitalInOut(getattr(board, pin_name))
                    dio.direction = digitalio.Direction.OUTPUT
                    dio.value = False
                    self._outputs[logical] = OutputPin(logical, dio, False)
                except AttributeError:
                    # Pin not present on this board – keep a virtual pin
                    self._outputs[logical] = OutputPin(logical, None, False)

        except Exception as exc:  # pragma: no cover - hardware dependent
            self._hardware_error = f"Hardware initialisation failed: {exc}"
            self._ads = None
            self._i2c = None
            self._outputs = {name: OutputPin(name) for name in self.OUTPUT_MAPPING}

    # ------------------------------------------------------------------
    # Measurement routines
    # ------------------------------------------------------------------

    def _measure_voltage(self) -> float:
        assert self._ads is not None and self._chan_voltage is not None

        manual_rate = self.manual_sample_rate
        if manual_rate is None:
            if self.precise_mode:
                manual_rate = 16
            else:
                manual_rate = 860 if self.mode in {Mode.VOLTMETER, Mode.DEFAULT} else 128
        self._ads.data_rate = manual_rate

        manual_gain = self.manual_gain

        if self.mode == Mode.VAC_MANUAL and manual_gain is None:
            gain = 1
            scale = self.VOLTAGE_SCALE_POS
            self._ads.gain = gain
            raw = self._chan_voltage.value
            voltage = raw * self.GAIN_FACTORS[gain] / 1000.0 * scale
        else:
            gain = manual_gain if manual_gain is not None else self.GAIN_LEVELS[self._voltage_gain_index]
            self._ads.gain = gain
            raw = self._chan_voltage.value

            if manual_gain is None:
                if abs(raw) > self.ADC_COUNT_HIGH_THRESHOLD and self._voltage_gain_index > 0:
                    self._voltage_gain_index -= 1
                    gain = self.GAIN_LEVELS[self._voltage_gain_index]
                    self._ads.gain = gain
                    raw = self._chan_voltage.value
                elif (
                    abs(raw) < self.ADC_COUNT_LOW_THRESHOLD
                    and self._voltage_gain_index < len(self.GAIN_LEVELS) - 1
                ):
                    self._voltage_gain_index += 1
                    gain = self.GAIN_LEVELS[self._voltage_gain_index]
                    self._ads.gain = gain
                    raw = self._chan_voltage.value

            scale = self.VOLTAGE_SCALE_POS if raw >= 0 else self.VOLTAGE_SCALE_NEG
            voltage = raw * self.GAIN_FACTORS[gain] / 1000.0 * scale

        self._vac_present = abs(voltage) < 0.03 and abs(raw) > self.ADC_COUNT_HIGH_THRESHOLD
        self._update_voltage_history(voltage)

        if (self.mode == Mode.VOLTMETER or self.mode == Mode.DEFAULT) and abs(voltage) < 0.05:
            self._check_floating_voltage()
        else:
            self._v_floating = False

        return voltage

    def _measure_resistance(self) -> float:
        assert self._ads is not None and self._chan_res is not None

        manual_rate = self.manual_sample_rate
        if manual_rate is None:
            manual_rate = 16 if self.precise_mode else 128
        self._ads.data_rate = manual_rate

        range_threshold = 40.0 if self.mode in {Mode.R_PLOT, Mode.HIGH_R} else 400.0
        low_threshold = range_threshold * (1.0 - self.OHMS_RANGE_DEADBAND)
        high_threshold = range_threshold * (1.0 + self.OHMS_RANGE_DEADBAND)

        if self._res_gain_index >= len(self.GAIN_LEVELS):
            self._res_gain_index = len(self.GAIN_LEVELS) - 1

        manual_gain = self.manual_gain
        if manual_gain is None:
            gain = self.GAIN_LEVELS[self._res_gain_index]
            self._ads.gain = gain
        else:
            gain = manual_gain
            self._ads.gain = gain

        if self.ohms_auto_range and not self.power_save:
            if not self._res_range_high and self._last_res_value > high_threshold:
                self._res_range_high = True
                self._set_output("SETRANGE_PIN", True)
            elif self._res_range_high and self._last_res_value < low_threshold:
                self._res_range_high = False
                self._set_output("SETRANGE_PIN", False)
        elif self.power_save:
            self._res_range_high = True
            self._set_output("SETRANGE_PIN", True)
        else:
            desired = self.ohms_high_range
            if desired != self._res_range_high:
                self._res_range_high = desired
                self._set_output("SETRANGE_PIN", desired)

        raw = self._chan_res.value
        if manual_gain is None:
            if raw > self.ADC_COUNT_HIGH_THRESHOLD and self._res_gain_index > 0:
                self._res_gain_index -= 1
                gain = self.GAIN_LEVELS[self._res_gain_index]
                self._ads.gain = gain
                raw = self._chan_res.value
            elif (
                raw < self.ADC_COUNT_LOW_THRESHOLD
                and self._res_gain_index < len(self.GAIN_LEVELS) - 1
            ):
                self._res_gain_index += 1
                gain = self.GAIN_LEVELS[self._res_gain_index]
                self._ads.gain = gain
                raw = self._chan_res.value

        ohms_voltage = raw * self.GAIN_FACTORS[gain] / 1000.0

        zener_max = self.EEPROM_SLEEP_V if self.power_save else self.EEPROM_MAX_V
        if ohms_voltage > zener_max:
            ohms_voltage = zener_max - 0.0001

        if self._res_range_high:
            resistance = self.DIVIDER_R * (ohms_voltage / (zener_max - ohms_voltage))
        else:
            resistance = ohms_voltage / (self.CONSTANT_I - (ohms_voltage / self.CONSTANT_R))

        if not self._initial_zero_set and 0.001 < resistance < 10.0:
            self._set_output("OHMPWMPIN", False)
            self._zero_offset_res = resistance
            self._initial_zero_set = True

        resistance = max(resistance - self._zero_offset_res, 0.0)
        resistance = self._apply_resistance_calibration(resistance)

        if self.alt_units and resistance > 0:
            # Convert thermistor resistance to Fahrenheit using Beta equation
            resistance = (
                1.0
                / ((1.0 / 298.15) + (math.log(resistance / 10000.0) / 3694.0))
                - 273.15
            ) * 1.8 + 32.0

        if ohms_voltage > zener_max - 0.007 and self.mode != Mode.HIGH_R:
            resistance = 8e6

        self._last_res_value = resistance
        return resistance

    def _measure_current(self) -> float:
        assert self._ads is not None and self._chan_current is not None

        if not self.current_enabled:
            return 0.0

        manual_rate = self.manual_sample_rate
        if manual_rate is None:
            manual_rate = 128 if self.precise_mode else 860
        self._ads.data_rate = manual_rate

        manual_gain = self.manual_gain
        if manual_gain is None:
            gain = self.GAIN_LEVELS[self._current_gain_index]
            self._ads.gain = gain
        else:
            gain = manual_gain
            self._ads.gain = gain
        raw = self._chan_current.value

        if manual_gain is None:
            if abs(raw) > self.ADC_COUNT_HIGH_THRESHOLD and self._current_gain_index > 0:
                self._current_gain_index -= 1
                gain = self.GAIN_LEVELS[self._current_gain_index]
                self._ads.gain = gain
                raw = self._chan_current.value
            elif (
                abs(raw) < self.ADC_COUNT_LOW_THRESHOLD
                and self._current_gain_index < len(self.GAIN_LEVELS) - 1
            ):
                self._current_gain_index += 1
                gain = self.GAIN_LEVELS[self._current_gain_index]
                self._ads.gain = gain
                raw = self._chan_current.value

        shunt_voltage = raw * self.GAIN_FACTORS[gain] / 1000.0
        return shunt_voltage  # 1 ohm shunt

    # ------------------------------------------------------------------
    # Utility helpers
    # ------------------------------------------------------------------

    def _update_voltage_history(self, voltage: float) -> None:
        idx = self._voltage_index

        old = self._voltage_buffer[idx]
        old_sq = self._voltage_sq_buffer[idx]
        self._voltage_sum -= old
        self._voltage_sq_sum -= old_sq

        self._voltage_buffer[idx] = voltage
        diff = voltage - self._voltage_avg
        squared = diff * diff
        self._voltage_sq_buffer[idx] = squared

        self._voltage_sum += voltage
        self._voltage_sq_sum += squared

        self._voltage_index = (idx + 1) % len(self._voltage_buffer)
        self._voltage_avg = self._voltage_sum / len(self._voltage_buffer)

    def _check_floating_voltage(self) -> None:
        assert self._ads is not None and self._chan_voltage is not None
        current_gain = self._ads.gain
        self._ads.gain = 8
        self._set_output("VbridgePin", True)
        time.sleep(0.002)
        raw = self._chan_voltage.value
        self._bridge_voltage = raw * self.GAIN_FACTORS[8] / 1000.0
        self._v_floating = self._bridge_voltage < -0.28125
        self._set_output("VbridgePin", False)
        self._ads.gain = current_gain

    def _apply_resistance_calibration(self, resistance: float) -> float:
        for limit, factor in zip(self.RES_CAL_LIMITS, self.RES_CAL_FACTORS):
            if resistance < limit:
                return resistance * factor
        return resistance

    def _update_statistics(self, result: MeasurementResult) -> None:
        for field in self.STATS_FIELDS:
            value = getattr(result, field, None)
            if not isinstance(value, (int, float)):
                continue
            if math.isnan(value) or math.isinf(value):
                continue
            stats = self._stats[field]
            current_min = stats["min"]
            current_max = stats["max"]
            stats["min"] = value if current_min is None else min(current_min, value)
            stats["max"] = value if current_max is None else max(current_max, value)

    def _write_output(self, name: str, value: bool) -> None:
        pin = self._outputs.get(name)
        if pin is None:
            return
        pin.set(value)

    def _set_output(self, name: str, value: bool) -> None:
        if name == "OHMPWMPIN":
            value = value or self.power_save or self.mode == Mode.CHARGING

        self._auto_output_cache[name] = value

        if self._manual_outputs_enabled and name in self._manual_output_states:
            self._write_output(name, self._manual_output_states[name])
        else:
            self._write_output(name, value)

    def _update_constant_current_output(self) -> None:
        state = self.power_save or self.mode == Mode.CHARGING
        self._set_output("OHMPWMPIN", state)

    def _update_continuity_output(self, measurement: MeasurementResult) -> None:
        pin = self._outputs.get("CONTINUITY_PIN")
        if pin is None:
            return

        resistance = measurement.resistance
        continuity = resistance <= 1.0 or (resistance <= 20.0 and self.ohms_high_range)

        if self.alt_units:
            logic_voltage = abs(measurement.voltage_dc) > 30.2 or (
                measurement.vac_present and measurement.voltage_rms > 15.0
            )
        else:
            logic_voltage = abs(measurement.voltage_dc) > 3.2 or (
                measurement.vac_present and measurement.voltage_rms > 1.0
            )

        now = time.monotonic()

        if continuity:
            if not self._continuity_active:
                self._continuity_active = True
                self._logic_voltage_active = False
                self._continuity_pattern_start = now
            cycle = (now - self._continuity_pattern_start) % 1.0
            state = cycle < 0.1 or 0.3 <= cycle < 0.4
            self._auto_output_cache["CONTINUITY_PIN"] = state
            self._write_output("CONTINUITY_PIN", state)
        elif logic_voltage:
            if not self._logic_voltage_active:
                self._logic_voltage_active = True
                self._continuity_active = False
                self._continuity_pattern_start = now
            cycle = (now - self._continuity_pattern_start) % 1.0
            state = cycle < 0.1
            self._auto_output_cache["CONTINUITY_PIN"] = state
            self._write_output("CONTINUITY_PIN", state)
        else:
            if self._continuity_active or self._logic_voltage_active:
                self._continuity_pattern_start = now
            self._continuity_active = False
            self._logic_voltage_active = False
            self._auto_output_cache["CONTINUITY_PIN"] = False
            self._write_output("CONTINUITY_PIN", False)

    def refresh_outputs(self) -> None:
        self._update_continuity_output(self._last_measurement)


# ---------------------------------------------------------------------------
# Tkinter user interface
# ---------------------------------------------------------------------------


class MicroDMMApp:
    LOG_CHANNELS = {
        "Voltage (DC)": ("voltage_dc", "V"),
        "Voltage (RMS)": ("voltage_rms", "V"),
        "Resistance": ("resistance", "Ω"),
        "Current": ("current", "A"),
        "Bridge Voltage": ("bridge_voltage", "V"),
    }

    STAT_LABELS = {
        "voltage_dc": ("Voltage (DC)", "V"),
        "voltage_rms": ("Voltage (RMS)", "V"),
        "resistance": ("Resistance", "Ω"),
        "current": ("Current", "A"),
        "bridge_voltage": ("Bridge Voltage", "V"),
    }

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.dmm = MicroDMM()
        self.root.title("Micro-DMM FT232H")

        self.logging_active = tk.BooleanVar(value=False)
        self._log_records: list[dict[str, object]] = []
        self._log_start_time: Optional[float] = None
        self._plot_data: Dict[str, Dict[str, list[float]]] = {
            label: {"x": [], "y": []} for label in self.LOG_CHANNELS
        }
        self.channel_vars: Dict[str, tk.BooleanVar] = {}
        self._plot_lines: Dict[str, object] = {}
        self.plot_canvas: Optional[FigureCanvasTkAgg] = None
        self._plot_available = Figure is not None and FigureCanvasTkAgg is not None
        self.figure: Optional[Figure] = None
        self.ax = None  # type: ignore[assignment]
        self.log_count_var = tk.StringVar(value="Samples logged: 0")
        self.bridge_widgets: tuple[tk.Widget, ...] = ()
        self.stat_widgets: Dict[str, tuple[tk.Widget, ...]] = {}
        self.bridge_channel_check: Optional[ttk.Checkbutton] = None
        self.bridge_channel_var: Optional[tk.BooleanVar] = None

        self._build_gui()
        self._schedule_update()
        self._schedule_output_refresh()

    # GUI -----------------------------------------------------------------

    def _build_gui(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill="both", expand=True)

        self.notebook = ttk.Notebook(main)
        self.notebook.pack(fill="both", expand=True)

        self.meter_tab = ttk.Frame(self.notebook)
        self.logging_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.meter_tab, text="Meter")
        self.notebook.add(self.logging_tab, text="Logging")

        self._build_meter_tab(self.meter_tab)
        self._build_logging_tab(self.logging_tab)

    def _build_meter_tab(self, parent: ttk.Frame) -> None:
        # Measurement display
        meas_frame = ttk.LabelFrame(parent, text="Measurements")
        meas_frame.pack(fill="x", padx=5, pady=5)

        self.voltage_var = tk.StringVar(value="—")
        self.vac_var = tk.StringVar(value="—")
        self.res_var = tk.StringVar(value="—")
        self.current_var = tk.StringVar(value="—")
        self.status_var = tk.StringVar(value="—")
        self.bridge_var = tk.StringVar(value="—")

        self._add_row(meas_frame, 0, "Voltage (DC):", self.voltage_var)
        self._add_row(meas_frame, 1, "Voltage (RMS):", self.vac_var)
        self._add_row(meas_frame, 2, "Resistance:", self.res_var)
        self._add_row(meas_frame, 3, "Current:", self.current_var)
        self.bridge_widgets = self._add_row(meas_frame, 4, "Bridge Voltage:", self.bridge_var)

        extremes = ttk.LabelFrame(parent, text="Min / Max")
        extremes.pack(fill="x", padx=5, pady=5)

        ttk.Label(extremes, text="Measurement").grid(row=0, column=0, padx=4, pady=2, sticky="w")
        ttk.Label(extremes, text="Min").grid(row=0, column=1, padx=4, pady=2)
        ttk.Label(extremes, text="Max").grid(row=0, column=2, padx=4, pady=2)

        self.stat_min_vars: Dict[str, tk.StringVar] = {}
        self.stat_max_vars: Dict[str, tk.StringVar] = {}

        for row, field in enumerate(MicroDMM.STATS_FIELDS, start=1):
            label_text, _ = self.STAT_LABELS[field]
            label_widget = ttk.Label(extremes, text=label_text + ":")
            label_widget.grid(row=row, column=0, padx=4, pady=2, sticky="w")
            min_var = tk.StringVar(value="—")
            max_var = tk.StringVar(value="—")
            min_widget = ttk.Label(extremes, textvariable=min_var)
            max_widget = ttk.Label(extremes, textvariable=max_var)
            min_widget.grid(row=row, column=1, padx=4, pady=2)
            max_widget.grid(row=row, column=2, padx=4, pady=2)
            self.stat_min_vars[field] = min_var
            self.stat_max_vars[field] = max_var
            self.stat_widgets[field] = (label_widget, min_widget, max_widget)

        self.status_label = ttk.Label(parent, textvariable=self.status_var)
        self.status_label.pack(fill="x", padx=5, pady=5)

        ctrl = ttk.LabelFrame(parent, text="Controls")
        ctrl.pack(fill="x", padx=5, pady=5)

        ttk.Label(ctrl, text="Mode:").grid(row=0, column=0, sticky="w")
        self.mode_var = tk.StringVar(value=self.dmm.mode.name)
        mode_combo = ttk.Combobox(
            ctrl,
            textvariable=self.mode_var,
            values=[mode.name for mode in Mode],
            state="readonly",
            width=15,
        )
        mode_combo.grid(row=0, column=1, padx=5, pady=2, sticky="w")
        mode_combo.bind("<<ComboboxSelected>>", self._on_mode_changed)

        self.auto_var = tk.BooleanVar(value=self.dmm.ohms_auto_range)
        self.high_range_var = tk.BooleanVar(value=self.dmm.ohms_high_range)
        self.precise_var = tk.BooleanVar(value=self.dmm.precise_mode)
        self.alt_units_var = tk.BooleanVar(value=self.dmm.alt_units)
        self.current_var_enabled = tk.BooleanVar(value=self.dmm.current_enabled)
        self.power_save_var = tk.BooleanVar(value=self.dmm.power_save)

        self._add_check(ctrl, 1, "Ohms Auto Range", self.auto_var, self._on_auto_range)
        self._add_check(ctrl, 1, "High Range", self.high_range_var, self._on_high_range, column=2)
        self._add_check(ctrl, 2, "Precise Mode", self.precise_var, self._on_precise)
        self._add_check(ctrl, 2, "Alt Units", self.alt_units_var, self._on_alt_units, column=2)
        self._add_check(ctrl, 3, "Current Enabled", self.current_var_enabled, self._on_current)
        self._add_check(ctrl, 3, "Power Save", self.power_save_var, self._on_power_save, column=2)

        self.advanced_visible = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            ctrl,
            text="Show Advanced Controls",
            variable=self.advanced_visible,
            command=self._refresh_advanced_visibility,
        ).grid(row=4, column=0, sticky="w", padx=4, pady=6, columnspan=3)

        self.advanced_frame = ttk.LabelFrame(parent, text="Advanced Controls")

        self._sample_rate_options = [
            ("Automatic", None),
            *[(f"{rate} SPS", rate) for rate in MicroDMM.ADS_DATA_RATES],
        ]
        self._sample_rate_lookup = {label: value for label, value in self._sample_rate_options}
        sample_label = next(
            (label for label, value in self._sample_rate_options if value == self.dmm.manual_sample_rate),
            "Automatic",
        )
        self.sample_rate_var = tk.StringVar(value=sample_label)

        self._gain_options = [
            ("Automatic", None),
        ]
        for gain in MicroDMM.GAIN_LEVELS:
            if math.isclose(gain, 2 / 3, rel_tol=1e-6):
                gain_label = "2/3"
            else:
                gain_label = f"{gain:g}"
            range_label = MicroDMM.GAIN_RANGES[gain]
            label = f"±{range_label:.3f} V (gain {gain_label})"
            self._gain_options.append((label, gain))
        self._gain_lookup = {label: value for label, value in self._gain_options}
        gain_label = next(
            (label for label, value in self._gain_options if value == self.dmm.manual_gain),
            "Automatic",
        )
        self.gain_var = tk.StringVar(value=gain_label)

        ttk.Label(self.advanced_frame, text="ADS1115 Sample Rate:").grid(
            row=0, column=0, sticky="w", padx=4, pady=4
        )
        sample_combo = ttk.Combobox(
            self.advanced_frame,
            textvariable=self.sample_rate_var,
            values=[label for label, _ in self._sample_rate_options],
            state="readonly",
            width=20,
        )
        sample_combo.grid(row=0, column=1, sticky="w", padx=4, pady=4)
        sample_combo.bind("<<ComboboxSelected>>", self._on_sample_rate_changed)

        ttk.Label(self.advanced_frame, text="ADS1115 Gain:").grid(
            row=1, column=0, sticky="w", padx=4, pady=4
        )
        gain_combo = ttk.Combobox(
            self.advanced_frame,
            textvariable=self.gain_var,
            values=[label for label, _ in self._gain_options],
            state="readonly",
            width=26,
        )
        gain_combo.grid(row=1, column=1, sticky="w", padx=4, pady=4)
        gain_combo.bind("<<ComboboxSelected>>", self._on_gain_changed)

        self.manual_outputs_var = tk.BooleanVar(value=self.dmm.manual_outputs_enabled)
        ttk.Checkbutton(
            self.advanced_frame,
            text="Enable Manual Pin Control",
            variable=self.manual_outputs_var,
            command=self._on_manual_outputs_toggle,
        ).grid(row=2, column=0, sticky="w", padx=4, pady=(8, 4), columnspan=2)

        self.manual_pin_frame = ttk.Frame(self.advanced_frame)
        self.manual_pin_frame.grid(row=3, column=0, columnspan=2, sticky="w", padx=4, pady=(0, 4))

        self.manual_pin_controls: Dict[str, tuple[tk.BooleanVar, ttk.Checkbutton]] = {}
        for idx, (name, label) in enumerate(MicroDMM.MANUAL_PIN_LABELS.items()):
            var = tk.BooleanVar(value=self.dmm.get_manual_output_state(name))
            chk = ttk.Checkbutton(
                self.manual_pin_frame,
                text=label,
                variable=var,
                command=lambda n=name, v=var: self._on_manual_output_toggled(n, v),
            )
            chk.grid(row=0, column=idx, sticky="w", padx=4, pady=2)
            self.manual_pin_controls[name] = (var, chk)

        self._update_manual_pin_controls()

        self.show_bridge_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            self.advanced_frame,
            text="Show Bridge Voltage Readout",
            variable=self.show_bridge_var,
            command=self._on_show_bridge_changed,
        ).grid(row=4, column=0, sticky="w", padx=4, pady=(8, 4), columnspan=2)

        self.btn_frame = ttk.Frame(parent)
        self.btn_frame.pack(fill="x", padx=5, pady=5)

        ttk.Button(self.btn_frame, text="Zero Ohms", command=self.dmm.zero_resistance).pack(
            side="left", padx=2
        )
        ttk.Button(self.btn_frame, text="Set V Ref", command=self.dmm.set_voltage_reference).pack(
            side="left", padx=2
        )
        ttk.Button(self.btn_frame, text="Clear V Ref", command=self.dmm.clear_voltage_reference).pack(
            side="left", padx=2
        )
        ttk.Button(self.btn_frame, text="Clear VAC", command=self._on_clear_vac).pack(
            side="left", padx=2
        )
        ttk.Button(self.btn_frame, text="Toggle Cycle Track", command=self.dmm.toggle_cycle_track).pack(
            side="left", padx=2
        )
        ttk.Button(self.btn_frame, text="Reset Min/Max", command=self._on_reset_stats).pack(
            side="left", padx=2
        )

        self._refresh_advanced_visibility()
        self._update_bridge_visibility()
        self._update_stat_labels()

    def _build_logging_tab(self, parent: ttk.Frame) -> None:
        container = ttk.Frame(parent)
        container.pack(fill="both", expand=True)

        if self._plot_available:
            self.figure = Figure(figsize=(7, 4), dpi=100)
            self.ax = self.figure.add_subplot(111)
            self.ax.set_title("Measurement Log")
            self.ax.set_xlabel("Time (s)")
            self.ax.set_ylabel("Value")

            for label, (field, unit) in self.LOG_CHANNELS.items():
                (line,) = self.ax.plot([], [], label=f"{label} [{unit}]")
                self._plot_lines[label] = line

            self.ax.legend(loc="upper right")
            self.plot_canvas = FigureCanvasTkAgg(self.figure, master=container)
            self.plot_canvas.draw_idle()
            self.plot_canvas.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)
        else:
            ttk.Label(
                container,
                text="Matplotlib is required for plotting. Install matplotlib to enable charts.",
                wraplength=400,
                justify="center",
            ).pack(fill="both", expand=True, padx=10, pady=10)

        controls = ttk.Frame(container)
        controls.pack(fill="x", padx=5, pady=5)

        ttk.Checkbutton(
            controls,
            text="Record measurements",
            variable=self.logging_active,
            command=self._on_logging_toggle,
        ).pack(side="left", padx=2)

        ttk.Button(controls, text="Clear Log", command=self._on_clear_log).pack(side="left", padx=2)
        ttk.Button(controls, text="Export CSV", command=self._on_export_csv).pack(side="left", padx=2)

        ttk.Label(container, textvariable=self.log_count_var).pack(fill="x", padx=5, pady=(0, 5))

        if self._plot_available:
            channel_box = ttk.LabelFrame(container, text="Channels")
            channel_box.pack(fill="x", padx=5, pady=5)
            for idx, label in enumerate(self.LOG_CHANNELS.keys()):
                default_enabled = label != "Bridge Voltage"
                var = tk.BooleanVar(value=default_enabled)
                self.channel_vars[label] = var
                chk = ttk.Checkbutton(
                    channel_box,
                    text=label,
                    variable=var,
                    command=self._refresh_plot,
                )
                chk.grid(row=idx // 3, column=idx % 3, sticky="w", padx=4, pady=2)
                if label == "Bridge Voltage":
                    self.bridge_channel_check = chk
                    self.bridge_channel_var = var

        self._update_bridge_visibility()

    def _add_row(
        self, frame: ttk.Frame, row: int, label: str, var: tk.StringVar
    ) -> tuple[ttk.Label, ttk.Label]:
        label_widget = ttk.Label(frame, text=label)
        value_widget = ttk.Label(frame, textvariable=var)
        label_widget.grid(row=row, column=0, sticky="w", padx=4, pady=2)
        value_widget.grid(row=row, column=1, sticky="w", padx=4, pady=2)
        return (label_widget, value_widget)

    def _add_check(
        self,
        frame: ttk.Frame,
        row: int,
        text: str,
        var: tk.BooleanVar,
        command,
        column: int = 0,
    ) -> None:
        ttk.Checkbutton(frame, text=text, variable=var, command=command).grid(
            row=row, column=column, sticky="w", padx=4, pady=2
        )

    def _refresh_advanced_visibility(self) -> None:
        if getattr(self, "advanced_frame", None) is None:
            return
        if self.advanced_visible.get():
            if not self.advanced_frame.winfo_ismapped():
                pack_kwargs = {"fill": "x", "padx": 5, "pady": 5}
                if hasattr(self, "btn_frame"):
                    pack_kwargs["before"] = self.btn_frame
                self.advanced_frame.pack(**pack_kwargs)
        else:
            if self.advanced_frame.winfo_ismapped():
                self.advanced_frame.pack_forget()

    # Handlers -----------------------------------------------------------

    def _on_mode_changed(self, _event=None) -> None:
        try:
            self.dmm.mode = Mode[self.mode_var.get()]
        except KeyError:
            pass

    def _on_auto_range(self) -> None:
        self.dmm.ohms_auto_range = self.auto_var.get()

    def _on_high_range(self) -> None:
        self.dmm.ohms_high_range = self.high_range_var.get()

    def _on_precise(self) -> None:
        self.dmm.precise_mode = self.precise_var.get()

    def _on_alt_units(self) -> None:
        self.dmm.alt_units = self.alt_units_var.get()
        self._update_stat_labels()

    def _on_current(self) -> None:
        self.dmm.current_enabled = self.current_var_enabled.get()

    def _on_power_save(self) -> None:
        self.dmm.power_save = self.power_save_var.get()
        self.dmm._update_constant_current_output()

    def _on_sample_rate_changed(self, _event=None) -> None:
        label = self.sample_rate_var.get()
        rate = self._sample_rate_lookup.get(label)
        try:
            self.dmm.set_manual_sample_rate(rate)
        except ValueError:
            self.sample_rate_var.set("Automatic")
            self.dmm.set_manual_sample_rate(None)

    def _on_gain_changed(self, _event=None) -> None:
        label = self.gain_var.get()
        gain = self._gain_lookup.get(label)
        try:
            self.dmm.set_manual_gain(gain)
        except ValueError:
            self.gain_var.set("Automatic")
            self.dmm.set_manual_gain(None)

    def _on_manual_outputs_toggle(self) -> None:
        enabled = self.manual_outputs_var.get()
        self.dmm.set_manual_outputs_enabled(enabled)
        self._update_manual_pin_controls()

    def _on_manual_output_toggled(self, name: str, var: tk.BooleanVar) -> None:
        try:
            self.dmm.set_manual_output_state(name, var.get())
        except ValueError:
            pass

    def _on_show_bridge_changed(self) -> None:
        self._update_bridge_visibility()

    def _update_manual_pin_controls(self) -> None:
        enabled = self.manual_outputs_var.get()
        for name, (var, chk) in self.manual_pin_controls.items():
            current = self.dmm.get_manual_output_state(name)
            if var.get() != current:
                var.set(current)
            if enabled:
                chk.state(["!disabled"])
            else:
                chk.state(["disabled"])

    def _on_reset_stats(self) -> None:
        self.dmm.reset_statistics()
        self._update_stat_labels()

    def _on_clear_vac(self) -> None:
        self.dmm.clear_vac_history()
        self.vac_var.set("—")

    def _update_bridge_visibility(self) -> None:
        show = self.show_bridge_var.get() if hasattr(self, "show_bridge_var") else False

        for widget in self.bridge_widgets:
            if show:
                widget.grid()
            else:
                widget.grid_remove()

        stat_widgets = self.stat_widgets.get("bridge_voltage")
        if stat_widgets:
            for widget in stat_widgets:
                if show:
                    widget.grid()
                else:
                    widget.grid_remove()

        if self.bridge_channel_var is not None:
            if show:
                self.bridge_channel_var.set(True)
                if self.bridge_channel_check is not None:
                    self.bridge_channel_check.grid()
                    self.bridge_channel_check.state(["!disabled"])
            else:
                self.bridge_channel_var.set(False)
                if self.bridge_channel_check is not None:
                    self.bridge_channel_check.grid_remove()

        self._refresh_plot()

    def _update_stat_labels(self) -> None:
        if not hasattr(self, "stat_min_vars"):
            return
        stats = self.dmm.get_statistics()
        for field, (min_val, max_val) in stats.items():
            min_var = self.stat_min_vars.get(field)
            max_var = self.stat_max_vars.get(field)
            if min_var is not None:
                min_var.set(self._format_stat_value(field, min_val))
            if max_var is not None:
                max_var.set(self._format_stat_value(field, max_val))

    def _format_stat_value(self, field: str, value: Optional[float]) -> str:
        if value is None:
            return "—"
        _label, unit = self.STAT_LABELS[field]
        if field in {"voltage_dc", "voltage_rms", "bridge_voltage"}:
            return f"{value:.6f} {unit}"
        if field == "current":
            return f"{value:.6f} {unit}"
        if field == "resistance":
            if self.dmm.alt_units:
                return f"{value:.2f} °F"
            if value >= 1_000_000:
                return f"{value / 1_000_000:.3f} MΩ"
            if value >= 1_000:
                return f"{value / 1_000:.3f} kΩ"
            return f"{value:.3f} {unit}"
        return f"{value:.6f} {unit}"

    def _on_logging_toggle(self) -> None:
        if self.logging_active.get():
            now = time.time()
            if self._log_records:
                last_elapsed = float(self._log_records[-1]["elapsed"])
                self._log_start_time = now - last_elapsed
            else:
                self._log_start_time = now
        else:
            self._log_start_time = None
        self._update_log_count()

    def _on_clear_log(self) -> None:
        self._log_records.clear()
        self._log_start_time = time.time() if self.logging_active.get() else None
        for series in self._plot_data.values():
            series["x"].clear()
            series["y"].clear()
        self._update_log_count()
        self._refresh_plot()

    def _on_export_csv(self) -> None:
        if not self._log_records:
            messagebox.showinfo("Export", "No samples have been recorded yet.")
            return
        file_path = filedialog.asksaveasfilename(
            title="Export measurements",
            defaultextension=".csv",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
        )
        if not file_path:
            return

        fieldnames = ["timestamp", "elapsed"] + [field for field, _ in self.LOG_CHANNELS.values()]
        try:
            with open(file_path, "w", newline="", encoding="utf-8") as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                for record in self._log_records:
                    row = {
                        "timestamp": record["timestamp"].isoformat(),
                        "elapsed": f"{float(record['elapsed']):.6f}",
                    }
                    for field, _ in self.LOG_CHANNELS.values():
                        value = record.get(field)
                        if isinstance(value, float):
                            row[field] = f"{value:.12g}"
                        else:
                            row[field] = value
                    writer.writerow(row)
        except Exception as exc:
            messagebox.showerror("Export failed", f"Could not write CSV: {exc}")
            return

        messagebox.showinfo(
            "Export complete",
            f"Saved {len(self._log_records)} samples to {file_path}",
        )

    def _update_log_count(self) -> None:
        self.log_count_var.set(f"Samples logged: {len(self._log_records)}")

    def _update_logging(self, result: MeasurementResult) -> None:
        if not self.logging_active.get():
            return

        now = time.time()
        if self._log_start_time is None:
            if self._log_records:
                last_elapsed = float(self._log_records[-1]["elapsed"])
                self._log_start_time = now - last_elapsed
            else:
                self._log_start_time = now

        elapsed = now - (self._log_start_time or now)
        record: Dict[str, object] = {
            "timestamp": datetime.now(),
            "elapsed": elapsed,
        }

        for label, (field, _unit) in self.LOG_CHANNELS.items():
            value = getattr(result, field)
            record[field] = value
            series = self._plot_data[label]
            series["x"].append(elapsed)
            series["y"].append(value)

        self._log_records.append(record)
        self._update_log_count()
        self._refresh_plot()

    def _refresh_plot(self) -> None:
        if not self._plot_available or self.plot_canvas is None or self.ax is None:
            return

        visible_y: list[float] = []
        visible_x: list[float] = []
        handles = []
        labels = []

        for label, line in self._plot_lines.items():
            series = self._plot_data[label]
            enabled = self.channel_vars.get(label)
            if enabled is not None and enabled.get() and series["x"]:
                line.set_data(series["x"], series["y"])
                visible_y.extend(series["y"])
                visible_x.extend(series["x"])
                handles.append(line)
                labels.append(line.get_label())
            else:
                line.set_data([], [])

        if visible_x:
            x_min = min(visible_x)
            x_max = max(visible_x)
            if math.isclose(x_min, x_max, rel_tol=1e-9, abs_tol=1e-9):
                x_max = x_min + 1.0
            self.ax.set_xlim(x_min, x_max)
        else:
            self.ax.set_xlim(0.0, 1.0)

        if visible_y:
            y_min = min(visible_y)
            y_max = max(visible_y)
            if math.isclose(y_min, y_max, rel_tol=1e-9, abs_tol=1e-9):
                margin = abs(y_min) * 0.1 if y_min else 0.1
                y_min -= margin
                y_max += margin
            else:
                span = y_max - y_min
                margin = span * 0.1
                y_min -= margin
                y_max += margin
            self.ax.set_ylim(y_min, y_max)
        else:
            self.ax.set_ylim(-1.0, 1.0)

        legend = self.ax.get_legend()
        if handles:
            self.ax.legend(handles=handles, labels=labels, loc="upper right")
        elif legend is not None:
            legend.remove()

        self.plot_canvas.draw_idle()

    # Scheduler ----------------------------------------------------------

    def _schedule_update(self) -> None:
        self.root.after(MicroDMM.UPDATE_INTERVAL_MS, self._update)

    def _schedule_output_refresh(self) -> None:
        self.root.after(40, self._refresh_outputs)

    def _update(self) -> None:
        try:
            result = self.dmm.measure()
            hardware_ok = self.dmm.hardware_ok
            self.status_var.set("Hardware OK" if hardware_ok else "Hardware not ready")
            self.status_label.configure(foreground="green" if hardware_ok else "red")
            self.voltage_var.set(f"{result.voltage_dc: .6f} V")
            self.vac_var.set(f"{result.voltage_rms: .6f} V")
            if self.dmm.alt_units:
                self.res_var.set(f"{result.resistance: .2f} °F")
            else:
                if result.resistance >= 1_000_000:
                    self.res_var.set(f"{result.resistance / 1_000_000: .3f} MΩ")
                elif result.resistance >= 1_000:
                    self.res_var.set(f"{result.resistance / 1_000: .3f} kΩ")
                else:
                    self.res_var.set(f"{result.resistance: .3f} Ω")
            self.current_var.set(f"{result.current: .6f} A")
            self.bridge_var.set(
                f"{result.bridge_voltage: .6f} V ({'FLOAT' if result.v_floating else 'CLOSED'})"
            )
            self._update_stat_labels()
            self._update_logging(result)
        except MeasurementError as exc:
            self.status_var.set(str(exc))
            self.status_label.configure(foreground="red")
        except Exception as exc:  # pragma: no cover - GUI safety net
            traceback.print_exc()
            self.status_var.set(f"Unexpected error: {exc}")
            self.status_label.configure(foreground="red")

        self._schedule_update()

    def _refresh_outputs(self) -> None:
        try:
            self.dmm.refresh_outputs()
        finally:
            self._schedule_output_refresh()


def main() -> None:
    root = tk.Tk()
    app = MicroDMMApp(root)
    if not app.dmm.hardware_ok:
        sys.stderr.write((app.dmm.hardware_error or "Hardware unavailable") + "\n")
    root.mainloop()


if __name__ == "__main__":
    main()

