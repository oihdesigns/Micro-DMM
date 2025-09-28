"""Tkinter-based Raspberry Pi UI for the Micro-DMM."""
from __future__ import annotations

import math
import threading
import time
import os
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Dict, Iterable, Optional

import tkinter as tk
from tkinter import ttk

from .data_model import (
    MeasurementState,
    format_resistance_value,
    format_time,
    format_voltage_value,
)
from .logging import PiLogger

if TYPE_CHECKING:  # pragma: no cover - for type checking only
    from .run_pi import MeasurementService

try:
    from gpiozero import Button  # type: ignore
except Exception:  # pragma: no cover - gpiozero is optional on dev hosts
    Button = None


@dataclass
class ButtonMapping:
    label: str
    callback: Callable[[], None]
    hotkey: str
    gpio_pin: Optional[int] = None


MODES: Iterable[str] = (
    "Debug",
    "Default",
    "Voltmeter",
    "VAC",
    "Type",
    "Low",
    "AltUnits",
    "High R",
    "rPlotMode",
    "Charging",
)


class MicroDmmApp:
    """UI controller for the Raspberry Pi port."""

    def __init__(
        self,
        root: tk.Tk,
        state: Optional[MeasurementState] = None,
        logger: Optional[PiLogger] = None,
        gpio_pins: Optional[Dict[str, int]] = None,
        container: Optional[tk.Widget] = None,
    ) -> None:
        self.root = root
        self.container = container or root
        self.state = state or MeasurementState()
        self.logger = logger or PiLogger()
        self.mode_cycle = list(MODES)
        self.mode_index = self.mode_cycle.index(self.state.current_mode) if self.state.current_mode in self.mode_cycle else 0
        self.start_time = time.monotonic()
        self._runtime_job: Optional[str] = None
        self._debug_visible = False
        self._service: Optional["MeasurementService"] = None
        self._display_interval = 0.5
        self._last_display_update = 0.0
        self._display_job: Optional[str] = None
        self._updating_backend_controls = False
        self._gain_options: list[str] = []
        self._data_rate_options: list[int] = []

        self._build_ui()
        if gpio_pins:
            for key, pin in gpio_pins.items():
                if key in self.button_mappings:
                    self.button_mappings[key].gpio_pin = pin
        self._install_keyboard_shortcuts()
        self._install_gpio_buttons()

        self._update_runtime_label()
        self._schedule_runtime_refresh()

        self.update_display()

    # ------------------------------------------------------------------
    # UI construction
    def _build_ui(self) -> None:
        if self.container is self.root:
            self.root.title("Micro-DMM (Raspberry Pi)")
            self.root.configure(padx=12, pady=12)
            parent = self.root
        else:
            parent = self.container
            if hasattr(parent, "configure"):
                try:
                    parent.configure(padding=12)
                except tk.TclError:
                    parent.configure(padx=16, pady=16)

        header = ttk.Label(parent, text="Measurement", font=("TkDefaultFont", 20, "bold"))
        header.grid(row=0, column=0, columnspan=2, sticky="w")

        self.standard_display_frame = ttk.Frame(parent)
        self.standard_display_frame.grid(row=1, column=0, columnspan=2, sticky="w")
        self.primary_value = ttk.Label(self.standard_display_frame, font=("TkDefaultFont", 26, "bold"))
        self.primary_value.grid(row=0, column=0, sticky="w")
        self.primary_suffix = ttk.Label(self.standard_display_frame, font=("TkDefaultFont", 14))
        self.primary_suffix.grid(row=0, column=1, sticky="w")

        self.debug_display_frame = ttk.Frame(parent)
        self.debug_display_frame.grid(row=1, column=0, columnspan=2, sticky="ew")
        self.debug_display_frame.columnconfigure(0, weight=1)
        self.debug_display_frame.columnconfigure(1, weight=1)

        ttk.Label(
            self.debug_display_frame,
            text="Voltage",
            font=("TkDefaultFont", 14, "bold"),
          
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            self.debug_display_frame,
            text="Resistance",
            font=("TkDefaultFont", 14, "bold"),

        ).grid(row=0, column=1, sticky="w")

        voltage_value_frame = ttk.Frame(self.debug_display_frame)
        voltage_value_frame.grid(row=1, column=0, sticky="w")
        self.debug_voltage_value = ttk.Label(
            voltage_value_frame, font=("TkDefaultFont", 24, "bold")
        )
        self.debug_voltage_value.grid(row=0, column=0, sticky="w")
        self.debug_voltage_suffix = ttk.Label(
            voltage_value_frame, font=("TkDefaultFont", 14)

        )
        self.debug_voltage_suffix.grid(row=0, column=1, sticky="w", padx=(8, 0))

        resistance_value_frame = ttk.Frame(self.debug_display_frame)
        resistance_value_frame.grid(row=1, column=1, sticky="w")
        self.debug_resistance_value = ttk.Label(
            resistance_value_frame, font=("TkDefaultFont", 24, "bold")
        )
        self.debug_resistance_value.grid(row=0, column=0, sticky="w")
        self.debug_resistance_suffix = ttk.Label(
            resistance_value_frame, font=("TkDefaultFont", 14)

        )
        self.debug_resistance_suffix.grid(row=0, column=1, sticky="w", padx=(8, 0))

        self.debug_voltage_extra = ttk.Label(
            self.debug_display_frame, font=("TkDefaultFont", 12)
        )
        self.debug_voltage_extra.grid(row=2, column=0, sticky="w")
        self.debug_resistance_voltage = ttk.Label(
            self.debug_display_frame, font=("TkDefaultFont", 12)
        )
        self.debug_resistance_voltage.grid(row=2, column=1, sticky="w")

        self.debug_gain_label = ttk.Label(
            self.debug_display_frame, font=("TkDefaultFont", 11)
        )
        self.debug_gain_label.grid(row=3, column=0, columnspan=2, sticky="w")

        self.debug_backend_label = ttk.Label(
            self.debug_display_frame, font=("TkDefaultFont", 11)
        )
        self.debug_backend_label.grid(row=4, column=0, columnspan=2, sticky="w")

        self.debug_display_frame.grid_remove()

        self.secondary_info = ttk.Label(parent, font=("TkDefaultFont", 12))
        self.secondary_info.grid(row=2, column=0, columnspan=2, sticky="w")

        ttk.Separator(parent, orient="horizontal").grid(row=3, column=0, columnspan=2, sticky="ew", pady=8)

        self.mode_label = ttk.Label(parent, font=("TkDefaultFont", 12))
        self.mode_label.grid(row=4, column=0, sticky="w")

        self.time_label = ttk.Label(parent, font=("TkDefaultFont", 12))
        self.time_label.grid(row=4, column=1, sticky="e")

        self.current_label = ttk.Label(parent, font=("TkDefaultFont", 12))
        self.current_label.grid(row=5, column=0, columnspan=2, sticky="w")

        self.minmax_label = ttk.Label(parent, font=("TkDefaultFont", 11))
        self.minmax_label.grid(row=6, column=0, columnspan=2, sticky="w")

        self.logging_label = ttk.Label(parent, font=("TkDefaultFont", 11))
        self.logging_label.grid(row=7, column=0, columnspan=2, sticky="w")

        ttk.Separator(parent, orient="horizontal").grid(row=8, column=0, columnspan=2, sticky="ew", pady=8)

        self.button_frame = ttk.Frame(parent)
        self.button_frame.grid(row=9, column=0, columnspan=2, sticky="ew")

        self.button_mappings: Dict[str, ButtonMapping] = {
            "mode": ButtonMapping("Mode", self.cycle_mode, "m"),
            "manual_log": ButtonMapping("Manual Log", self.request_manual_log, "l"),
            "auto_log": ButtonMapping("Auto Log", self.request_auto_log, "a"),
            "minmax": ButtonMapping("Reset Min/Max", self.reset_min_max, "z"),
            "type": ButtonMapping("Type/Δ", self.trigger_type_action, "t"),
        }

        for column, mapping in enumerate(self.button_mappings.values()):
            btn = ttk.Button(self.button_frame, text=f"{mapping.label} ({mapping.hotkey.upper()})", command=mapping.callback)
            btn.grid(row=0, column=column, padx=4, pady=4, sticky="ew")
            self.button_frame.columnconfigure(column, weight=1)


        self.backend_controls_frame = ttk.LabelFrame(parent, text="ADC controls", padding=4)
        self.backend_controls_frame.grid(row=10, column=0, columnspan=2, sticky="nsew", pady=(8, 0))
        if hasattr(parent, "rowconfigure"):
            try:
                parent.rowconfigure(10, weight=1)
            except tk.TclError:
                pass

        controls_container = ttk.Frame(self.backend_controls_frame)
        controls_container.grid(row=0, column=0, sticky="nsew")
        self.backend_controls_frame.columnconfigure(0, weight=1)
        controls_container.columnconfigure(0, weight=1)

        gain_frame = ttk.LabelFrame(controls_container, text="Gain", padding=4)

        gain_frame.grid(row=0, column=0, sticky="ew")
        gain_frame.columnconfigure(1, weight=1)

        self.gain_mode_var = tk.StringVar(value="auto")
        ttk.Radiobutton(
            gain_frame,
            text="Automatic",
            value="auto",
            variable=self.gain_mode_var,
            command=self._on_gain_mode_changed,
        ).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(
            gain_frame,
            text="Manual",
            value="manual",
            variable=self.gain_mode_var,
            command=self._on_gain_mode_changed,
        ).grid(row=0, column=1, sticky="w")
        ttk.Label(gain_frame, text="Gain:").grid(row=1, column=0, sticky="w", pady=(4, 0))
        self.manual_gain_var = tk.StringVar()
        self.manual_gain_combo = ttk.Combobox(
            gain_frame,
            textvariable=self.manual_gain_var,
            state="disabled",
            width=10,
        )
        self.manual_gain_combo.grid(row=1, column=1, sticky="ew", padx=(4, 0), pady=(4, 0))
        self.manual_gain_combo.bind("<<ComboboxSelected>>", self._on_manual_gain_selected)

        sampling_frame = ttk.LabelFrame(controls_container, text="Sampling", padding=4)

        sampling_frame.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        sampling_frame.columnconfigure(1, weight=1)
        ttk.Label(sampling_frame, text="Rate:").grid(row=0, column=0, sticky="w")
        self.sample_rate_var = tk.StringVar()
        self.sample_rate_combo = ttk.Combobox(
            sampling_frame,
            textvariable=self.sample_rate_var,
            state="readonly",
            width=10,
        )
        self.sample_rate_combo.grid(row=0, column=1, sticky="ew", padx=(4, 0))
        self.sample_rate_combo.bind("<<ComboboxSelected>>", self._on_sample_rate_selected)
        self.buffer_var = tk.BooleanVar(value=False)
        self.buffer_toggle = ttk.Checkbutton(
            sampling_frame,
            text="Input buffer enabled",
            variable=self.buffer_var,
            command=self._on_buffer_toggled,
        )
        self.buffer_toggle.grid(row=1, column=0, columnspan=2, sticky="w", pady=(6, 0))

        calibration_frame = ttk.LabelFrame(controls_container, text="Voltage calibration", padding=4)
        calibration_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        calibration_frame.columnconfigure(1, weight=1)

        ttk.Label(calibration_frame, text="Offset:").grid(row=0, column=0, sticky="w")
        self.voltage_offset_label = ttk.Label(calibration_frame, text="0.000 V")
        self.voltage_offset_label.grid(row=0, column=1, sticky="e")
        self.zero_button = ttk.Button(
            calibration_frame,

            text="Set zero",
            command=self._on_voltage_zero,
        )
        self.zero_button.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(4, 0))

        ttk.Label(calibration_frame, text="Scale factor:").grid(row=2, column=0, sticky="w", pady=(8, 0))
        self.voltage_scale_label = ttk.Label(calibration_frame, text="1.0000")
        self.voltage_scale_label.grid(row=2, column=1, sticky="e", pady=(8, 0))
        ttk.Label(calibration_frame, text="Target voltage (V):").grid(row=3, column=0, sticky="w")
        self.scale_input_var = tk.StringVar()
        self.scale_entry = ttk.Entry(calibration_frame, textvariable=self.scale_input_var, width=10)
        self.scale_entry.grid(row=3, column=1, sticky="ew", padx=(4, 0))
        self.apply_scale_button = ttk.Button(
            calibration_frame,

            text="Apply scale",
            command=self._on_apply_scale,
        )
        self.apply_scale_button.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(4, 0))

        controls_container.rowconfigure(2, weight=1)

        # Ensure the debug layout starts in the correct state before the first update.
        self._toggle_debug_display(self.state.current_mode == "Debug")

    def _install_keyboard_shortcuts(self) -> None:
        for mapping in self.button_mappings.values():
            self.root.bind(f"<{mapping.hotkey}>", lambda event, cb=mapping.callback: cb())
            self.root.bind(f"<{mapping.hotkey.upper()}>", lambda event, cb=mapping.callback: cb())

    def _install_gpio_buttons(self) -> None:
        if Button is None:
            return
        for mapping in self.button_mappings.values():
            if mapping.gpio_pin is None:
                continue
            button = Button(mapping.gpio_pin)
            button.when_pressed = mapping.callback

    def attach_service(self, service: "MeasurementService") -> None:
        self._service = service
        self._gain_options = service.gain_options()
        if self._gain_options:
            self.manual_gain_combo.configure(values=self._gain_options)
            self.manual_gain_var.set(self._gain_options[0])
        rate_options = service.data_rate_options()
        self._data_rate_options = rate_options
        if rate_options:
            display_values = [f"{rate} Hz" for rate in rate_options]
            self.sample_rate_combo.configure(values=display_values)
        self.update_backend_controls(service.get_backend_status())

    def update_backend_controls(self, status: Dict[str, object]) -> None:
        self._updating_backend_controls = True
        try:
            auto_gain = bool(status.get("auto_gain", True))
            mode = "auto" if auto_gain else "manual"
            self.gain_mode_var.set(mode)
            self.manual_gain_combo.configure(state="readonly" if mode == "manual" else "disabled")
            manual_gain = status.get("manual_gain")
            if isinstance(manual_gain, str) and manual_gain in self._gain_options:
                self.manual_gain_var.set(manual_gain)
            sample_rate = status.get("sample_rate")
            if isinstance(sample_rate, (int, float)) and sample_rate > 0:
                self.sample_rate_var.set(f"{int(round(sample_rate))} Hz")
            buffer_enabled = bool(status.get("buffer_enabled", False))
            self.buffer_var.set(buffer_enabled)
            voltage_scale = status.get("voltage_scale")
            if isinstance(voltage_scale, (int, float)):
                self.voltage_scale_label.configure(text=f"{voltage_scale:.4f}")
            voltage_offset = status.get("voltage_offset")
            if isinstance(voltage_offset, (int, float)):
                self.voltage_offset_label.configure(text=f"{voltage_offset:.3f} V")
        finally:
            self._updating_backend_controls = False

    def _on_gain_mode_changed(self) -> None:
        if self._updating_backend_controls:
            return
        mode = self.gain_mode_var.get()
        self.manual_gain_combo.configure(state="readonly" if mode == "manual" else "disabled")
        if self._service is None:
            return
        if mode == "auto":
            self._service.configure_gain_mode(auto=True)
        else:
            gain_name = self.manual_gain_var.get()
            if not gain_name and self._gain_options:
                gain_name = self._gain_options[0]
            self._service.configure_gain_mode(auto=False, gain_name=gain_name or None)
        self.update_backend_controls(self._service.get_backend_status())

    def _on_manual_gain_selected(self, _event: object) -> None:
        if self._updating_backend_controls or self._service is None:
            return
        gain_name = self.manual_gain_var.get()
        if not gain_name:
            return
        self._service.set_manual_gain(gain_name)
        if self.gain_mode_var.get() == "manual":
            self._service.configure_gain_mode(auto=False, gain_name=gain_name)
        self.update_backend_controls(self._service.get_backend_status())

    def _on_sample_rate_selected(self, _event: object) -> None:
        if self._updating_backend_controls or self._service is None:
            return
        value = self.sample_rate_var.get().split()[0]
        try:
            rate = float(value)
        except ValueError:
            self.logger.emit_keyboard_line(f"Invalid sample rate '{self.sample_rate_var.get()}'")
            return
        try:
            self._service.set_sample_rate(rate)
        except Exception as exc:  # pragma: no cover - hardware error path
            self.logger.emit_keyboard_line(f"Sample rate update failed: {exc}")
        self.update_backend_controls(self._service.get_backend_status())

    def _on_buffer_toggled(self) -> None:
        if self._updating_backend_controls or self._service is None:
            return
        try:
            self._service.set_buffer_enabled(bool(self.buffer_var.get()))
        except Exception as exc:  # pragma: no cover - hardware error path
            self.logger.emit_keyboard_line(f"Buffer toggle failed: {exc}")
        self.update_backend_controls(self._service.get_backend_status())

    def _on_voltage_zero(self) -> None:
        if self._service is None:
            return
        try:
            self._service.calibrate_voltage_zero()
            self.logger.emit_keyboard_line("Voltage zero calibration applied")
        except Exception as exc:  # pragma: no cover - hardware error path
            self.logger.emit_keyboard_line(f"Zero calibration failed: {exc}")
        self.update_backend_controls(self._service.get_backend_status())

    def _on_apply_scale(self) -> None:
        if self._service is None:
            return
        try:
            target = float(self.scale_input_var.get())
        except ValueError:
            self.logger.emit_keyboard_line("Enter a valid voltage for scale calibration")
            return
        try:
            self._service.calibrate_voltage_scale(target)
            self.logger.emit_keyboard_line(f"Voltage scale calibrated to {target:.4f} V")
        except Exception as exc:  # pragma: no cover - hardware error path
            self.logger.emit_keyboard_line(f"Scale calibration failed: {exc}")
        else:
            self.scale_input_var.set("")
        self.update_backend_controls(self._service.get_backend_status())

    def _cancel_pending_display(self) -> None:
        if self._display_job is not None:
            try:
                self.root.after_cancel(self._display_job)
            except tk.TclError:
                pass
            self._display_job = None

    def _queue_display_refresh(self) -> None:
        now = time.monotonic()
        elapsed = now - self._last_display_update
        if elapsed >= self._display_interval:
            self._last_display_update = now
            self.update_display()
        elif self._display_job is None:
            delay_ms = max(1, int((self._display_interval - elapsed) * 1000))
            self._display_job = self.root.after(delay_ms, self._delayed_display_update)

    def _delayed_display_update(self) -> None:
        self._display_job = None
        self._last_display_update = time.monotonic()
        self.update_display()


    def _toggle_debug_display(self, enabled: bool) -> None:
        if enabled and not self._debug_visible:
            self.standard_display_frame.grid_remove()
            self.debug_display_frame.grid()

            self._debug_visible = True
        elif not enabled and self._debug_visible:
            self.debug_display_frame.grid_remove()
            self.standard_display_frame.grid()

            self._debug_visible = False

    def _update_runtime_label(self) -> None:
        elapsed = max(0, int(time.monotonic() - self.start_time))
        hours, remainder = divmod(elapsed, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.time_label.configure(text=f"Runtime: {hours:02d}:{minutes:02d}:{seconds:02d}")

    def _schedule_runtime_refresh(self) -> None:
        self._update_runtime_label()
        self._runtime_job = self.root.after(1000, self._schedule_runtime_refresh)

    # ------------------------------------------------------------------
    # State update API
    def update_state(self, **kwargs: float | str | bool) -> None:
        for key, value in kwargs.items():
            if hasattr(self.state, key):
                setattr(self.state, key, value)
        self.update_display()

    def ingest_measurement(
        self,
        voltage: float,
        resistance: float,
        current: float,
        timestamp_ms: Optional[float] = None,
        ohms_voltage: Optional[float] = None,
        *,
        voltage_gain: Optional[str] = None,
        resistance_gain: Optional[str] = None,
        current_gain: Optional[str] = None,
        voltage_raw: Optional[int] = None,
        voltage_lsb: Optional[float] = None,
        sample_rate: Optional[float] = None,
        buffer_enabled: Optional[bool] = None,
        voltage_scale: Optional[float] = None,
        voltage_offset: Optional[float] = None,

    ) -> None:
        timestamp_ms = timestamp_ms if timestamp_ms is not None else time.time() * 1000.0
        seconds = timestamp_ms / 1000.0
        self.state.new_voltage = voltage
        self.state.median_voltage = voltage
        self.state.display_resistance = resistance
        if ohms_voltage is not None:
            self.state.ohms_voltage = ohms_voltage
        self.state.current_reading = current
        if voltage_gain is not None:
            self.state.voltage_gain = voltage_gain
        if resistance_gain is not None:
            self.state.resistance_gain = resistance_gain
        if current_gain is not None:
            self.state.current_gain = current_gain
        if voltage_raw is not None:
            self.state.voltage_raw_code = voltage_raw
        if voltage_lsb is not None:
            self.state.voltage_lsb = voltage_lsb
        if sample_rate is not None:
            self.state.sample_rate_hz = sample_rate
        if buffer_enabled is not None:
            self.state.buffer_enabled = buffer_enabled
        if voltage_scale is not None:
            self.state.voltage_scale = voltage_scale
        if voltage_offset is not None:
            self.state.voltage_offset = voltage_offset
        self.state.record_current_sample(voltage, seconds, current)
        self.state.update_timestamps(timestamp_ms)
        self.state.update_extrema(timestamp_ms)
        self._queue_display_refresh()

    # ------------------------------------------------------------------
    # Action handlers
    def cycle_mode(self) -> None:
        self.mode_index = (self.mode_index + 1) % len(self.mode_cycle)
        self.state.current_mode = self.mode_cycle[self.mode_index]
        self.update_display()

    def request_manual_log(self) -> None:
        if self.state.manual_log.sample_count == 0:
            return
        self.state.log_end_time = self.state.manual_log.timestamps[0] + self.state.log_start_time
        self.logger.write_manual_log(self.state)
        self.state.manual_log_count += 1
        self.update_display()

    def request_auto_log(self) -> None:
        if self.state.auto_log.sample_count == 0:
            return
        self.logger.write_auto_log(self.state)
        self.state.auto_log_count += 1
        self.update_display()

    def reset_min_max(self) -> None:
        self.state.low_voltage = self.state.high_voltage = self.state.new_voltage
        self.state.low_resistance = self.state.high_resistance = self.state.display_resistance
        self.state.current_low = self.state.current_high = self.state.current_reading
        self.state.time_at_min_voltage = self.state.time_at_max_voltage = format_time(self.state.last_update_ms)
        self.state.time_at_min_resistance = self.state.time_at_max_resistance = format_time(self.state.last_update_ms)
        self.state.time_at_min_current = self.state.time_at_max_current = format_time(self.state.last_update_ms)
        self.logger.emit_keyboard_line("Min/Max reset")
        self.update_display()

    def trigger_type_action(self) -> None:
        if self.state.current_mode == "Type":
            primary_value, _, digits = format_voltage_value(self.state.new_voltage) if self.state.voltage_display else format_resistance_value(self.state.display_resistance)
            base_value = self.state.new_voltage if self.state.voltage_display else self.state.display_resistance
            text = f"{base_value:.{digits}f}"
            self.logger.emit_keyboard_line(text)
        else:
            if self.state.voltage_display:
                self.state.delta_voltage = self.state.new_voltage
            else:
                self.state.zero_offset_res = 0.0 if math.isclose(self.state.zero_offset_res, 0.0, abs_tol=1e-6) else self.state.display_resistance
        self.update_display()

    # ------------------------------------------------------------------
    def update_display(self) -> None:
        self._cancel_pending_display()
        self._last_display_update = time.monotonic()

        debug_mode = self.state.current_mode == "Debug"
        self._toggle_debug_display(debug_mode)

        if self.state.last_update_ms == 0:
            if debug_mode:
                self.debug_voltage_value.configure(text="--")
                self.debug_voltage_suffix.configure(text="")
                self.debug_resistance_value.configure(text="--")
                self.debug_resistance_suffix.configure(text="")
                self.debug_voltage_extra.configure(text="")
                self.debug_resistance_voltage.configure(text="Sense V: --")
                self.debug_gain_label.configure(text="Gains: --")
                backend_parts = []
                sample_rate_hz = getattr(self.state, "sample_rate_hz", 0.0)
                if sample_rate_hz:
                    backend_parts.append(f"Sample: {int(sample_rate_hz)} Hz")
                backend_parts.append(f"Buffer: {'On' if self.state.buffer_enabled else 'Off'}")
                scale_text = "--" if not math.isfinite(self.state.voltage_scale) else f"{self.state.voltage_scale:.4f}"
                offset_text = (
                    "--"
                    if not math.isfinite(self.state.voltage_offset)
                    else f"{self.state.voltage_offset:.3f} V"
                )
                backend_parts.append(f"Scale: {scale_text}")
                backend_parts.append(f"Offset: {offset_text}")
                self.debug_backend_label.configure(text=" | ".join(backend_parts))
            else:
                self.primary_value.configure(text="--")
                self.primary_suffix.configure(text="")
                self.debug_gain_label.configure(text="")
                self.debug_backend_label.configure(text="")

            self.secondary_info.configure(text="Waiting for data…")
        else:
            voltage_value, voltage_suffix, voltage_digits = format_voltage_value(
                self.state.select_voltage_for_display()
            )
            resistance_value, resistance_suffix, resistance_digits = format_resistance_value(
                self.state.display_resistance
            )


            if debug_mode:
                self.debug_voltage_value.configure(text=f"{voltage_value:.{voltage_digits}f}")
                self.debug_voltage_suffix.configure(text=f"{voltage_suffix}V")
                self.debug_resistance_value.configure(text=f"{resistance_value:.{resistance_digits}f}")
                self.debug_resistance_suffix.configure(text=f"{resistance_suffix}Ω")
                self.debug_voltage_extra.configure(text="")
                if math.isfinite(self.state.ohms_voltage):
                    sense_value, sense_suffix, sense_digits = format_voltage_value(self.state.ohms_voltage)
                    sense_text = f"Sense V: {sense_value:.{sense_digits}f} {sense_suffix}V"
                else:
                    sense_text = "Sense V: --"
                self.debug_resistance_voltage.configure(text=sense_text)
                gain_parts = []
                if self.state.voltage_gain:
                    gain_parts.append(f"V: {self.state.voltage_gain}")
                if self.state.resistance_gain:
                    gain_parts.append(f"R: {self.state.resistance_gain}")
                if self.state.current_gain:
                    gain_parts.append(f"I: {self.state.current_gain}")
                gain_text = "Gains: " + " | ".join(gain_parts) if gain_parts else "Gains: --"
                self.debug_gain_label.configure(text=gain_text)
                backend_parts = []
                sample_rate_hz = getattr(self.state, "sample_rate_hz", 0.0)
                if sample_rate_hz:
                    backend_parts.append(f"Sample: {int(sample_rate_hz)} Hz")
                backend_parts.append(f"Buffer: {'On' if self.state.buffer_enabled else 'Off'}")
                scale_text = "--" if not math.isfinite(self.state.voltage_scale) else f"{self.state.voltage_scale:.4f}"
                offset_text = (
                    "--"
                    if not math.isfinite(self.state.voltage_offset)
                    else f"{self.state.voltage_offset:.3f} V"
                )
                backend_parts.append(f"Scale: {scale_text}")
                backend_parts.append(f"Offset: {offset_text}")
                self.debug_backend_label.configure(text=" | ".join(backend_parts))

            else:
                if self.state.voltage_display:
                    primary_value = voltage_value
                    primary_suffix = voltage_suffix
                    primary_digits = voltage_digits
                    unit = "V"
                else:
                    primary_value = resistance_value
                    primary_suffix = resistance_suffix
                    primary_digits = resistance_digits
                    unit = "Ω"
                primary_text = f"{primary_value:.{primary_digits}f}"
                self.primary_value.configure(text=primary_text)
                self.primary_suffix.configure(text=f" {primary_suffix}{unit}")
                self.debug_gain_label.configure(text="")
                self.debug_backend_label.configure(text="")


            if debug_mode:
                lines = []
                if not math.isinf(self.state.low_voltage):
                    low_val, low_suffix, low_digits = format_voltage_value(self.state.low_voltage)
                    high_val, high_suffix, high_digits = format_voltage_value(self.state.high_voltage)
                    lines.append(
                        f"Voltage min {low_val:.{low_digits}f}{low_suffix}V ({self.state.time_at_min_voltage})"
                    )
                    lines.append(
                        f"Voltage max {high_val:.{high_digits}f}{high_suffix}V ({self.state.time_at_max_voltage})"
                    )
                else:
                    lines.append("Gathering voltage min/max…")

                if not math.isinf(self.state.low_resistance):
                    low_val, low_suffix, low_digits = format_resistance_value(self.state.low_resistance)
                    high_val, high_suffix, high_digits = format_resistance_value(self.state.high_resistance)
                    lines.append(
                        f"Resistance min {low_val:.{low_digits}f}{low_suffix}Ω ({self.state.time_at_min_resistance})"
                    )
                    lines.append(
                        f"Resistance max {high_val:.{high_digits}f}{high_suffix}Ω ({self.state.time_at_max_resistance})"
                    )
                else:
                    lines.append("Gathering resistance min/max…")
                secondary = "\n".join(lines)
            else:
                if self.state.voltage_display and not math.isinf(self.state.low_voltage):
                    low_val, low_suffix, low_digits = format_voltage_value(self.state.low_voltage)
                    high_val, high_suffix, high_digits = format_voltage_value(self.state.high_voltage)
                    secondary = (
                        f"Min {low_val:.{low_digits}f}{low_suffix}  ({self.state.time_at_min_voltage})\n"
                        f"Max {high_val:.{high_digits}f}{high_suffix}  ({self.state.time_at_max_voltage})"
                    )
                elif not self.state.voltage_display and not math.isinf(self.state.low_resistance):
                    low_val, low_suffix, low_digits = format_resistance_value(self.state.low_resistance)
                    high_val, high_suffix, high_digits = format_resistance_value(self.state.high_resistance)
                    secondary = (
                        f"Min {low_val:.{low_digits}f}{low_suffix}Ω  ({self.state.time_at_min_resistance})\n"
                        f"Max {high_val:.{high_digits}f}{high_suffix}Ω  ({self.state.time_at_max_resistance})"
                    )
                else:
                    secondary = "Gathering min/max data…"
            self.secondary_info.configure(text=secondary)

        current_text = "Current disabled"
        if self.state.current_enabled and not math.isinf(self.state.current_low):
            unit = "A" if self.state.current_range_high else "mA"
            value = self.state.current_reading if self.state.current_range_high else self.state.current_reading * 1000.0
            digits = 2 if self.state.current_range_high else 3
            current_low = self.state.current_low if self.state.current_range_high else self.state.current_low * 1000.0
            current_high = self.state.current_high if self.state.current_range_high else self.state.current_high * 1000.0
            current_text = (
                f"Current: {value:.{digits}f} {unit} (min {current_low:.3f}, max {current_high:.3f})"
            )
        self.current_label.configure(text=current_text)

        self.mode_label.configure(text=f"Mode: {self.state.current_mode}")

        logging = f"Logging: {'USB' if self.state.log_mode_enabled else 'Off'} | Manual {self.state.manual_log_count} | Auto {self.state.auto_log_count}"
        self.logging_label.configure(text=logging)

        minmax = ""
        if self.state.delta_voltage:
            minmax += f"ΔV ref: {self.state.delta_voltage:.3f} V\n"
        if self.state.zero_offset_res:
            minmax += f"Null offset: {self.state.zero_offset_res * 1000:.1f} mΩ"
        self.minmax_label.configure(text=minmax)

    # ------------------------------------------------------------------
    def run(self) -> None:
        self.root.mainloop()


def _demo_data(app: MicroDmmApp) -> None:
    start = time.time()
    app.update_state(voltage_display=True, current_enabled=True, current_range_high=False, log_mode_enabled=True)
    while True:
        elapsed = time.time() - start
        voltage = math.sin(elapsed / 3.0) * 2.0 + 5.0
        resistance = 1000.0 + math.sin(elapsed / 5.0) * 50
        current = 0.01 + math.cos(elapsed / 4.0) * 0.002
        ohms_voltage = min(resistance * 0.001, 3.0)
        timestamp_ms = time.time() * 1000.0
        app.state.queue_auto_sample(voltage, elapsed, current)
        app.ingest_measurement(
            voltage,
            resistance,
            current,
            timestamp_ms=timestamp_ms,
            ohms_voltage=ohms_voltage,
        )
        time.sleep(1.0)


def main() -> None:
    root = tk.Tk()
    gpio_config: Dict[str, int] = {}
    env = os.getenv("MICRO_DMM_GPIO", "").strip()
    if env:
        for token in env.split(','):
            if '=' not in token:
                continue
            key, value = token.split('=', 1)
            key = key.strip()
            try:
                gpio_config[key] = int(value)
            except ValueError:
                continue
    app = MicroDmmApp(root, gpio_pins=gpio_config or None)
    demo_thread = threading.Thread(target=_demo_data, args=(app,), daemon=True)
    demo_thread.start()
    app.run()


if __name__ == "__main__":
    main()
