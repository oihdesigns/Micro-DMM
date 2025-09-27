"""ADS1256 backend that mirrors the Arduino auto-ranging logic.

The original micro-DMM firmware relies on the ADS1115.  When porting the
project to a Raspberry Pi we use the 24‑bit ADS1256 instead.  This module keeps
the higher level behaviour (auto-ranging, calibration factors, and min/max
tracking expectations) compatible so that the UI layer can stay largely
unchanged.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

try:  # pragma: no cover - optional dependency on Raspberry Pi
    import spidev  # type: ignore
except ImportError:  # pragma: no cover
    spidev = None  # type: ignore


# === Low level constants =====================================================
CMD_WAKEUP = 0x00
CMD_RDATA = 0x01
CMD_RDATAC = 0x03
CMD_SDATAC = 0x0F
CMD_RREG = 0x10
CMD_WREG = 0x50
CMD_SELFCAL = 0xF0
CMD_SYNC = 0xFC
CMD_STANDBY = 0xFD
CMD_RESET = 0xFE

REG_STATUS = 0x00
REG_MUX = 0x01
REG_ADCON = 0x02
REG_DRATE = 0x03

AIN_COM = 0x08
FULL_SCALE = (1 << 23) - 1


@dataclass(frozen=True)
class GainSetting:
    """Represents a single ADS1256 PGA gain option."""

    name: str
    code: int
    value: int
    lsb: float  # volts per code


@dataclass(frozen=True)
class MeasurementResult:
    """Container returned by the high level helpers."""

    value: float
    raw_code: int
    gain: GainSetting
    lsb: float


@dataclass(frozen=True)
class VoltageResult(MeasurementResult):
    """Differential voltage reading result."""


@dataclass(frozen=True)
class CurrentResult(MeasurementResult):
    """Single ended current measurement (channel 3)."""

    range_high: bool


@dataclass(frozen=True)
class ResistanceResult(MeasurementResult):
    """Single ended resistance measurement (channel 2)."""

    raw_resistance: float
    calibrated_resistance: float
    range_high: bool
    ohms_voltage: float


def _default_resistance_calibration() -> List[Tuple[float, float]]:
    """Piece-wise calibration table taken from the Arduino firmware."""

    return [
        (0.75, 0.9607),
        (3.0, 0.9848),
        (7.0, 0.9970),
        (20.0, 0.9958),
        (70.0, 0.9934),
        (170.0, 0.9965),
        (700.0, 0.9982),
        (1700.0, 1.0028),
        (7000.0, 1.0012),
        (17000.0, 1.0005),
        (70000.0, 1.0017),
        (170000.0, 1.0035),
        (700000.0, 1.0075),
        (1700000.0, 1.0237),
        (math.inf, 1.1226),
    ]


def _ratio_to_counts(ratio: float) -> int:
    """Scale ADS1115 ratios (based on 16-bit) to ADS1256 counts."""

    return int(ratio * FULL_SCALE)


class ADS1256Backend:
    """High level ADS1256 driver with auto-ranging helpers.

    Parameters mirror the Arduino implementation as closely as possible so that
    the rest of the UI logic (min/max tracking, formatting, etc.) can remain
    unchanged.  The class is stateful and keeps per-channel gain indices to
    perform dynamic gain switching in the same way as the original firmware.
    """

    DATA_RATES: Dict[int, int] = {
        30000: 0xF0,
        15000: 0xE0,
        7500: 0xD0,
        3750: 0xC0,
        2000: 0xB0,
        1000: 0xA1,
        500: 0x92,
        100: 0x82,
        60: 0x72,
        50: 0x63,
        30: 0x53,
        25: 0x43,
        15: 0x33,
        10: 0x23,
        5: 0x13,
        2: 0x03,
    }

    # Equivalent thresholds to the ADS1115 logic (30000/32767 and 10000/32767)
    ADC_COUNT_HIGH_THRESHOLD = _ratio_to_counts(30000 / 32767.0)
    ADC_COUNT_LOW_THRESHOLD = _ratio_to_counts(10000 / 32767.0)

    OHMS_RANGE_DEADBAND = 0.05
    DEFAULT_OHMS_RANGE = 400.0

    GAIN_SETTINGS: List[GainSetting] = []
    for name, code, value in [
        ("PGA1", 0x00, 1),
        ("PGA2", 0x01, 2),
        ("PGA4", 0x02, 4),
        ("PGA8", 0x03, 8),
        ("PGA16", 0x04, 16),
        ("PGA32", 0x05, 32),
        ("PGA64", 0x06, 64),
    ]:
        # Compute the nominal LSB in volts (Vref/(gain * 2^23))
        GAIN_SETTINGS.append(
            GainSetting(name=name, code=code, value=value, lsb=0.0)
        )
    del name, code, value

    def __init__(
        self,
        spi_bus: int = 0,
        spi_device: int = 0,
        *,
        vref: float = 2.5,
        spi: Optional["spidev.SpiDev"] = None,
        drdy_reader: Optional[Callable[[], bool]] = None,
        range_switch: Optional[Callable[[bool], None]] = None,
        resistance_calibration: Optional[List[Tuple[float, float]]] = None,
        voltage_channels: Tuple[int, int] = (0, 1),
        resistance_channel: int = 2,
        current_channel: int = 3,
    ) -> None:
        if spidev is None and spi is None:
            raise RuntimeError(
                "spidev module not available – supply a custom SPI object when "
                "running off-target"
            )

        self.vref = vref
        self.range_switch = range_switch
        self.drdy_reader = drdy_reader

        self.spi = spi or spidev.SpiDev()  # type: ignore[assignment]
        if spi is None:
            self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = 1_000_000
        self.spi.mode = 0b01

        # Pre-compute LSBs per gain using the configured Vref
        self.gains: List[GainSetting] = []
        for gain in self.GAIN_SETTINGS:
            lsb = self.vref / (gain.value * float(1 << 23))
            self.gains.append(
                GainSetting(name=gain.name, code=gain.code, value=gain.value, lsb=lsb)
            )
        self._max_gain_index = len(self.gains) - 1

        # Calibration constants copied from the Arduino firmware
        self.voltage_scale_pos = 68.36437
        self.voltage_scale_neg = 68.3536
        self.giga_abs_factor = 0.0

        self.constant_current = 0.02016
        self.constant_resistor = 330.0
        self.divider_resistor = 22000.0
        self.zener_max_v = 4.979
        self.sleep_zener_v = 0.6117

        self.current_zero_offset = 0.0
        self.current_hall_v_per_a = 0.185  # for the hall sensor (high range)
        self.current_sense_resistance = 1.0

        self.resistance_calibration = (
            resistance_calibration or _default_resistance_calibration()
        )

        # State used for auto-ranging and previous measurements
        self._voltage_first_run = True
        self._voltage_gain_index = self._max_gain_index
        self._voltage_last = 0.0

        self._current_first_run = True
        self._current_gain_index = self._max_gain_index
        self._current_last = 0.0

        self._resistance_first_run = True
        self._resistance_gain_index = self._max_gain_index
        self._resistance_last = 0.0
        self._resistance_range_high = False

        self.zero_offset_res = 0.0

        self.ohms_range_threshold = self.DEFAULT_OHMS_RANGE
        self._update_ohms_thresholds()

        self.data_rate = 1000
        self._conversion_delay = 1.0 / self.data_rate
        self._adcon_base = 0x20  # clock out disabled, sensor detect off
        self._current_gain_index_hw: Optional[int] = None
        self._current_mux_hw: Optional[int] = None

        self._voltage_channels: Tuple[int, int] = self._validate_differential_channels(
            voltage_channels
        )
        self._resistance_channel: int = self._validate_channel(resistance_channel)
        self._current_channel: int = self._validate_channel(current_channel)

        self._initialise_hardware()

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------
    def close(self) -> None:
        if hasattr(self.spi, "close"):
            self.spi.close()

    # Voltage -----------------------------------------------------------------
    def read_voltage(
        self,
        *,
        auto_gain: bool = True,
        vac_present: bool = False,
        vac_manual: bool = False,
        alt_units: bool = False,
    ) -> VoltageResult:
        """Measure the differential voltage between channels 0 and 1.

        Parameters mirror the Arduino implementation.  ``auto_gain`` disables
        dynamic gain switching (useful when the UI wants a fixed setting).
        """

        if self._voltage_first_run:
            self._voltage_gain_index = self._max_gain_index
            self._voltage_first_run = False

        requested_gain_index = self._voltage_gain_index

        if vac_present or vac_manual:
            requested_gain_index = self._gain_index_for_value(1)
            auto_gain = False
        elif alt_units:
            requested_gain_index = self._gain_index_for_value(8)
            auto_gain = False

        raw_code = self._read_differential(
            self._voltage_channels[0], self._voltage_channels[1], requested_gain_index
        )

        if auto_gain:
            if (
                abs(raw_code) > self.ADC_COUNT_HIGH_THRESHOLD
                and requested_gain_index > 0
            ):
                requested_gain_index -= 1
                raw_code = self._read_differential(
                    self._voltage_channels[0],
                    self._voltage_channels[1],
                    requested_gain_index,
                )
            elif (
                abs(raw_code) < self.ADC_COUNT_LOW_THRESHOLD
                and requested_gain_index < self._max_gain_index
            ):
                requested_gain_index += 1
                raw_code = self._read_differential(
                    self._voltage_channels[0],
                    self._voltage_channels[1],
                    requested_gain_index,
                )
            self._voltage_gain_index = requested_gain_index

        gain = self.gains[requested_gain_index]
        volts = raw_code * gain.lsb
        if volts >= 0:
            volts *= self.voltage_scale_pos
        else:
            volts *= self.voltage_scale_neg
        volts -= self.giga_abs_factor

        if alt_units:
            volts *= 50.0

        self._voltage_last = volts

        return VoltageResult(
            value=volts,
            raw_code=raw_code,
            gain=gain,
            lsb=gain.lsb,
        )

    # Resistance ---------------------------------------------------------------
    def read_resistance(
        self,
        *,
        auto_range: bool = True,
        force_high_range: Optional[bool] = None,
        power_save: bool = False,
        alt_units: bool = False,
    ) -> ResistanceResult:
        """Measure resistance using channel 2 and replicate auto-ranging."""

        if self._resistance_first_run:
            if force_high_range is not None:
                self._resistance_range_high = force_high_range
            self._resistance_gain_index = (
                0 if self._resistance_range_high else self._max_gain_index
            )
            self._resistance_first_run = False

        if power_save:
            self._resistance_range_high = True
        elif force_high_range is not None and not auto_range:
            self._resistance_range_high = force_high_range
        elif auto_range:
            if (
                not self._resistance_range_high
                and self._resistance_last > self._ohms_high_threshold
            ):
                self._resistance_range_high = True
            elif (
                self._resistance_range_high
                and self._resistance_last < self._ohms_low_threshold
            ):
                self._resistance_range_high = False

        if self.range_switch is not None:
            self.range_switch(self._resistance_range_high)

        if power_save:
            zener_v = self.sleep_zener_v
        else:
            zener_v = self.zener_max_v

        gain_index = self._resistance_gain_index
        raw_code = self._read_single_ended(self._resistance_channel, gain_index)

        if abs(raw_code) > self.ADC_COUNT_HIGH_THRESHOLD and gain_index > 0:
            gain_index -= 1
            raw_code = self._read_single_ended(self._resistance_channel, gain_index)
        elif (
            abs(raw_code) < self.ADC_COUNT_LOW_THRESHOLD
            and gain_index < self._max_gain_index
        ):
            gain_index += 1
            raw_code = self._read_single_ended(self._resistance_channel, gain_index)
        self._resistance_gain_index = gain_index

        gain = self.gains[gain_index]
        ohms_voltage = raw_code * gain.lsb

        if not power_save:
            if ohms_voltage > zener_v:
                ohms_voltage = zener_v - 1e-4
            else:
                zener_v = self.zener_max_v

        if self._resistance_range_high:
            raw_resistance = self.divider_resistor * (
                ohms_voltage / max(zener_v - ohms_voltage, 1e-9)
            )
        else:
            raw_resistance = ohms_voltage / (
                self.constant_current - (ohms_voltage / self.constant_resistor)
            )

        calibrated = self._apply_resistance_calibration(raw_resistance)

        if alt_units:
            # Convert to Fahrenheit using the thermistor equation from the
            # Arduino code: ((1/T) - ...)^-1 * 1.8 + 32
            calibrated = (
                (1.0 / ((1.0 / 298.15) + (math.log(calibrated / 10000.0) / 3694.0)))
                - 273.15
            ) * 1.8 + 32.0

        calibrated -= self.zero_offset_res
        self._resistance_last = calibrated

        if not self._resistance_range_high and ohms_voltage > (zener_v - 0.007):
            calibrated = 8e6

        return ResistanceResult(
            value=calibrated,
            raw_code=raw_code,
            gain=gain,
            lsb=gain.lsb,
            raw_resistance=raw_resistance,
            calibrated_resistance=calibrated,
            range_high=self._resistance_range_high,
            ohms_voltage=ohms_voltage,
        )

    # Current ------------------------------------------------------------------
    def read_current(
        self,
        *,
        current_enabled: bool = True,
        high_range: bool = False,
        auto_gain: bool = True,
    ) -> CurrentResult:
        """Measure current from channel 3."""

        if not current_enabled:
            return CurrentResult(
                value=0.0,
                raw_code=0,
                gain=self.gains[self._current_gain_index],
                lsb=self.gains[self._current_gain_index].lsb,
                range_high=high_range,
            )

        if self._current_first_run:
            self._current_gain_index = self._max_gain_index
            self._current_first_run = False

        if high_range:
            gain_index = self._gain_index_for_value(1)
            auto_gain = False
        else:
            gain_index = self._current_gain_index

        raw_code = self._read_single_ended(self._current_channel, gain_index)

        if auto_gain:
            if abs(raw_code) > self.ADC_COUNT_HIGH_THRESHOLD and gain_index > 0:
                gain_index -= 1
                raw_code = self._read_single_ended(self._current_channel, gain_index)
            elif (
                abs(raw_code) < self.ADC_COUNT_LOW_THRESHOLD
                and gain_index < self._max_gain_index
            ):
                gain_index += 1
                raw_code = self._read_single_ended(self._current_channel, gain_index)
            self._current_gain_index = gain_index

        gain = self.gains[gain_index]
        shunt_voltage = raw_code * gain.lsb

        if high_range:
            amps = shunt_voltage / self.current_hall_v_per_a
        else:
            amps = (shunt_voltage - self.current_zero_offset) / self.current_sense_resistance

        if high_range and -0.01 <= amps <= 0.1:
            amps = 0.0
        elif not high_range and abs(amps) < 0.0005:
            amps = 0.0

        self._current_last = amps

        return CurrentResult(
            value=amps,
            raw_code=raw_code,
            gain=gain,
            lsb=gain.lsb,
            range_high=high_range,
        )

    # ------------------------------------------------------------------
    # Configuration helpers
    # ------------------------------------------------------------------
    def set_data_rate(self, samples_per_second: int) -> None:
        if samples_per_second not in self.DATA_RATES:
            raise ValueError(f"Unsupported data rate: {samples_per_second}")
        code = self.DATA_RATES[samples_per_second]
        self._write_register(REG_DRATE, code)
        self.data_rate = samples_per_second
        self._conversion_delay = 1.5 / samples_per_second

    def set_gain(self, gain_index: int) -> None:
        if not 0 <= gain_index <= self._max_gain_index:
            raise ValueError("Invalid gain index")
        self._write_register(REG_ADCON, self._adcon_base | self.gains[gain_index].code)
        self._current_gain_index_hw = gain_index

    def set_ohms_range_threshold(self, value: float) -> None:
        self.ohms_range_threshold = value
        self._update_ohms_thresholds()

    def set_zero_offset_resistance(self, value: float) -> None:
        self.zero_offset_res = value

    def set_current_zero_offset(self, value: float) -> None:
        self.current_zero_offset = value

    def configure_channels(
        self,
        *,
        voltage: Optional[Tuple[int, int]] = None,
        resistance: Optional[int] = None,
        current: Optional[int] = None,
    ) -> None:
        """Update the ADS1256 channel assignments used for each measurement."""

        if voltage is not None:
            self._voltage_channels = self._validate_differential_channels(voltage)
        if resistance is not None:
            self._resistance_channel = self._validate_channel(resistance)
        if current is not None:
            self._current_channel = self._validate_channel(current)

        # Force the next conversion to refresh the MUX and gain settings so the
        # hardware picks up the new assignments immediately.
        self._current_mux_hw = None

    def channel_map(self) -> Dict[str, Tuple[int, ...]]:
        """Return the configured channel assignments."""

        return {
            "voltage": self._voltage_channels,
            "resistance": (self._resistance_channel,),
            "current": (self._current_channel,),
        }

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _initialise_hardware(self) -> None:
        self._send_command(CMD_RESET)
        time.sleep(0.05)
        self._send_command(CMD_SDATAC)
        self._write_register(REG_STATUS, 0x00)
        self.set_gain(self._voltage_gain_index)
        self.set_data_rate(self.data_rate)
        self._write_register(
            REG_MUX,
            (self._voltage_channels[0] << 4) | self._voltage_channels[1],
        )
        self._send_command(CMD_SELFCAL)
        time.sleep(0.1)

    @staticmethod
    def _validate_channel(channel: int) -> int:
        if not 0 <= channel <= 7:
            raise ValueError("ADS1256 channels must be between 0 and 7")
        return channel

    def _validate_differential_channels(
        self, channels: Tuple[int, int]
    ) -> Tuple[int, int]:
        pos, neg = channels
        return self._validate_channel(pos), self._validate_channel(neg)

    def _gain_index_for_value(self, value: int) -> int:
        for idx, gain in enumerate(self.gains):
            if gain.value == value:
                return idx
        return self._max_gain_index

    def _update_ohms_thresholds(self) -> None:
        self._ohms_low_threshold = self.ohms_range_threshold * (1.0 - self.OHMS_RANGE_DEADBAND)
        self._ohms_high_threshold = self.ohms_range_threshold * (1.0 + self.OHMS_RANGE_DEADBAND)

    def _apply_resistance_calibration(self, raw: float) -> float:
        for limit, scale in self.resistance_calibration:
            if raw < limit:
                return raw * scale
        return raw

    def _wait_for_data_ready(self) -> None:
        if self.drdy_reader is None:
            time.sleep(self._conversion_delay)
            return
        timeout = time.time() + 0.05
        while time.time() < timeout:
            if not self.drdy_reader():
                return
            time.sleep(0.0001)
        raise TimeoutError("ADS1256 data ready timeout")

    def _read_differential(self, positive: int, negative: int, gain_index: int) -> int:
        mux = (positive << 4) | negative
        return self._perform_conversion(mux, gain_index)

    def _read_single_ended(self, channel: int, gain_index: int) -> int:
        mux = (channel << 4) | AIN_COM
        return self._perform_conversion(mux, gain_index)

    def _perform_conversion(self, mux: int, gain_index: int) -> int:
        if self._current_mux_hw != mux:
            self._write_register(REG_MUX, mux)
            self._current_mux_hw = mux
        if self._current_gain_index_hw != gain_index:
            self.set_gain(gain_index)
        self._send_command(CMD_SYNC)
        self._send_command(CMD_WAKEUP)
        self._wait_for_data_ready()
        self._send_command(CMD_RDATA)
        raw = self._read_bytes(3)
        return self._to_signed(raw)

    def _send_command(self, command: int) -> None:
        self.spi.xfer2([command])

    def _write_register(self, reg: int, value: int) -> None:
        self.spi.xfer2([CMD_WREG | reg, 0x00, value])

    def _read_bytes(self, count: int) -> List[int]:
        return self.spi.xfer2([0x00] * count)

    @staticmethod
    def _to_signed(data: List[int]) -> int:
        value = (data[0] << 16) | (data[1] << 8) | data[2]
        if value & 0x800000:
            value -= 1 << 24
        return value


__all__ = [
    "ADS1256Backend",
    "VoltageResult",
    "ResistanceResult",
    "CurrentResult",
]
