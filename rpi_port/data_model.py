"""Data model and formatting helpers for the Raspberry Pi Micro-DMM UI."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Iterable, Tuple

LOG_SIZE = 100
NUM_AUTO_VALUES = 32


def format_voltage_value(value: float) -> Tuple[float, str, int]:
    """Match the Arduino formatting used by ``formatVoltageValue``.

    Returns a tuple containing the scaled value, suffix ("" or "m"),
    and the number of digits to show after the decimal point.
    """
    if abs(value) < 0.9:
        out_value = value * 1000.0
        suffix = "m"
        digits = 0
    else:
        out_value = value
        suffix = ""
        digits = 2 if abs(value) > 11.0 else 3
    return out_value, suffix, digits


def format_resistance_value(value: float) -> Tuple[float, str, int]:
    """Port of ``formatResistanceValue`` from the firmware."""
    if value > 90_000_000.0:
        return value / 1_000_000.0, "M", 1
    if value > 9_000_000.0:
        return value / 1_000_000.0, "M", 2
    if value > 900_000.0:
        return value / 1_000_000.0, "M", 3
    if value > 900.0:
        digits = 3 if value > 90_000.0 else 4
        return value / 1_000.0, "k", digits
    if value < 0.9:
        digits = 1 if value < 0.09 else 2
        return value * 1000.0, "m", digits
    if value >= 100.0:
        digits = 2
    elif value < 9.5:
        digits = 4
    else:
        digits = 3
    return value, "", digits


def format_time(milliseconds: float) -> str:
    total_seconds = int(milliseconds // 1000)
    minutes, seconds = divmod(total_seconds, 60)
    return f"{minutes:02d}:{seconds:02d}"


@dataclass
class ManualLog:
    voltages: Deque[float] = field(default_factory=lambda: deque([0.0] * LOG_SIZE, maxlen=LOG_SIZE))
    currents: Deque[float] = field(default_factory=lambda: deque([0.0] * LOG_SIZE, maxlen=LOG_SIZE))
    timestamps: Deque[float] = field(default_factory=lambda: deque([0.0] * LOG_SIZE, maxlen=LOG_SIZE))
    sample_count: int = 0

    def add_entry(self, voltage: float, time_sec: float, current: float, reference_start: float) -> None:
        relative_time = time_sec - reference_start
        self.voltages.appendleft(voltage)
        self.currents.appendleft(current)
        self.timestamps.appendleft(relative_time)
        self.sample_count = min(self.sample_count + 1, LOG_SIZE)

    def as_rows(self, include_current: bool, start_time: float = 0.0) -> Iterable[Iterable[float]]:
        yield list(self.voltages)
        if include_current:
            yield list(self.currents)
        yield [t - start_time for t in self.timestamps]


@dataclass
class AutoLog:
    voltages: Deque[float] = field(default_factory=lambda: deque([0.0] * NUM_AUTO_VALUES, maxlen=NUM_AUTO_VALUES))
    currents: Deque[float] = field(default_factory=lambda: deque([0.0] * NUM_AUTO_VALUES, maxlen=NUM_AUTO_VALUES))
    timestamps: Deque[float] = field(default_factory=lambda: deque([0.0] * NUM_AUTO_VALUES, maxlen=NUM_AUTO_VALUES))
    sample_count: int = 0

    def push(self, voltage: float, timestamp: float, current: float) -> None:
        self.voltages.appendleft(voltage)
        self.timestamps.appendleft(timestamp)
        self.currents.appendleft(current)
        self.sample_count = min(self.sample_count + 1, NUM_AUTO_VALUES)

    def as_rows(self, include_current: bool) -> Iterable[Iterable[float]]:
        yield list(self.voltages)
        if include_current:
            yield list(self.currents)
        yield list(self.timestamps)


@dataclass
class MeasurementState:
    """Container for measurements mirrored from the firmware."""

    new_voltage: float = 0.0
    median_voltage: float = 0.0
    median_voltage_step: float = 0.0
    low_voltage: float = float("inf")
    high_voltage: float = float("-inf")
    time_at_min_voltage: str = "00:00"
    time_at_max_voltage: str = "00:00"
    display_resistance: float = 0.0
    ohms_voltage: float = 0.0
    low_resistance: float = float("inf")
    high_resistance: float = float("-inf")
    time_at_min_resistance: str = "00:00"
    time_at_max_resistance: str = "00:00"
    current_reading: float = 0.0
    current_enabled: bool = False
    current_range_high: bool = False
    current_low: float = float("inf")
    current_high: float = float("-inf")
    time_at_min_current: str = "00:00"
    time_at_max_current: str = "00:00"
    precise_mode: bool = False
    delta_mode: bool = False
    delta_voltage: float = 0.0
    zero_offset_res: float = 0.0
    min_max_display: bool = False

    current_mode: str = "Debug"
    voltage_display: bool = True
    log_mode_enabled: bool = False
    manual_log_count: int = 0
    auto_log_count: int = 0

    log_start_time: float = 0.0
    log_end_time: float = 0.0
    auto_log_start_time: float = 0.0

    last_update_ms: float = 0.0

    manual_log: ManualLog = field(default_factory=ManualLog)
    auto_log: AutoLog = field(default_factory=AutoLog)

    def select_voltage_for_display(self) -> float:
        if not self.precise_mode:
            if abs(self.median_voltage_step) > 0.5:
                return self.new_voltage
            return self.median_voltage
        return self.new_voltage

    def record_current_sample(self, voltage: float, time_sec: float, current: float) -> None:
        if self.manual_log.sample_count == 0:
            self.log_start_time = time_sec
        self.manual_log.add_entry(voltage, time_sec, current, self.log_start_time)

    def queue_auto_sample(self, voltage_at_peak: float, timestamp: float, current: float) -> None:
        if self.auto_log.sample_count == 0:
            self.auto_log_start_time = timestamp
        self.auto_log.push(voltage_at_peak, timestamp, current)

    def update_timestamps(self, current_millis: float) -> None:
        self.last_update_ms = current_millis

    def update_extrema(self, timestamp_ms: float) -> None:
        time_string = format_time(timestamp_ms)
        if self.new_voltage < self.low_voltage:
            self.low_voltage = self.new_voltage
            self.time_at_min_voltage = time_string
        if self.new_voltage > self.high_voltage:
            self.high_voltage = self.new_voltage
            self.time_at_max_voltage = time_string

        if self.display_resistance < self.low_resistance:
            self.low_resistance = self.display_resistance
            self.time_at_min_resistance = time_string
        if self.display_resistance > self.high_resistance:
            self.high_resistance = self.display_resistance
            self.time_at_max_resistance = time_string

        if self.current_reading < self.current_low:
            self.current_low = self.current_reading
            self.time_at_min_current = time_string
        if self.current_reading > self.current_high:
            self.current_high = self.current_reading
            self.time_at_max_current = time_string

    def display_values(self) -> dict:
        voltage_to_display = self.select_voltage_for_display()
        voltage_fmt = format_voltage_value(voltage_to_display)
        voltage_low_fmt = format_voltage_value(self.low_voltage)
        voltage_high_fmt = format_voltage_value(self.high_voltage)
        resistance_fmt = format_resistance_value(self.display_resistance)
        resistance_low_fmt = format_resistance_value(self.low_resistance)
        resistance_high_fmt = format_resistance_value(self.high_resistance)
        return {
            "voltage": voltage_fmt,
            "voltage_low": voltage_low_fmt,
            "voltage_high": voltage_high_fmt,
            "resistance": resistance_fmt,
            "resistance_low": resistance_low_fmt,
            "resistance_high": resistance_high_fmt,
        }

    def formatted_time(self) -> str:
        return format_time(self.last_update_ms)
