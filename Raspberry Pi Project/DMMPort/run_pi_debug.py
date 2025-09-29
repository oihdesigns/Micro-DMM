"""Debugging runtime harness with advanced configuration controls."""
from __future__ import annotations

import argparse
import signal
import sys
from pathlib import Path
from typing import Dict, Optional, Tuple

# ---------------------------------------------------------------------------
# Stand-alone execution support
# ---------------------------------------------------------------------------
if __package__ in (None, ""):
    # Allow ``python rpi_port/run_pi_debug.py`` to work by emulating package imports.
    package_root = Path(__file__).resolve().parent
    parent_dir = str(package_root.parent)
    package_dir = str(package_root)
    sys.path.insert(0, parent_dir)
    if package_dir in sys.path:
        sys.path.remove(package_dir)
    __package__ = package_root.name

try:  # pragma: no cover - optional hardware dependencies on the Pi
    from gpiozero import LED
except Exception:  # pragma: no cover
    LED = None  # type: ignore

from tkinter import ttk
import tkinter as tk

from .ads1256_backend import ADS1256Backend
from .app import MicroDmmApp
from .data_model import MeasurementState
from .logging import PiLogger
from .run_pi import (
    DataReadyReader,
    MeasurementService,
    ResourceGroup,
    _configure_logging,
    _parse_gpio_mappings,
    build_arg_parser,
)


class GpioOutputControl:
    """Base helper that owns a gpiozero LED and exposes reconfiguration APIs."""

    def __init__(self, pin: Optional[int] = None) -> None:
        self._driver: Optional[LED] = None
        self._pin: Optional[int] = None
        self._state: bool = False
        self._mode: str = "automatic"
        if pin is not None:
            self.set_pin(pin)

    # ------------------------------------------------------------------
    def set_pin(self, pin: Optional[int]) -> None:
        if pin == self._pin:
            return
        self._close_driver()
        self._pin = pin
        if pin is None:
            return
        if LED is None:
            raise RuntimeError(
                "gpiozero not available – install gpiozero or run without configuring GPIO outputs"
            )
        self._driver = LED(pin)
        self._write(self._state)

    def pin(self) -> Optional[int]:
        return self._pin

    @property
    def mode(self) -> str:
        return self._mode

    def set_mode(self, mode: str) -> None:
        if mode not in {"automatic", "on", "off"}:
            raise ValueError(f"Unsupported mode '{mode}'")
        self._mode = mode

    def close(self) -> None:
        self._close_driver()

    # ------------------------------------------------------------------
    def _write(self, value: bool) -> None:
        self._state = value
        if self._driver is None:
            return
        if value:
            self._driver.on()
        else:
            self._driver.off()

    def _close_driver(self) -> None:
        if self._driver is not None:
            self._driver.close()
            self._driver = None


class ConfigurableRangeSwitch(GpioOutputControl):
    """gpiozero LED wrapper that supports mode overrides."""

    def __init__(self, pin: Optional[int] = None) -> None:
        super().__init__(pin)
        self._last_backend_value: Optional[bool] = None

    def __call__(self, high_range: bool) -> None:
        self._last_backend_value = high_range
        self._apply_requested(high_range)

    def set_mode(self, mode: str) -> None:  # type: ignore[override]
        super().set_mode(mode)
        requested = self._last_backend_value if self._last_backend_value is not None else False
        self._apply_requested(requested)

    def set_pin(self, pin: Optional[int]) -> None:  # type: ignore[override]
        super().set_pin(pin)
        requested = self._last_backend_value if self._last_backend_value is not None else False
        self._apply_requested(requested)

    def _apply_requested(self, requested: bool) -> None:
        if self.mode == "automatic":
            target = requested
        elif self.mode == "on":
            target = True
        else:
            target = False
        self._write(target)


class ConfigurableCycleTracker(GpioOutputControl):
    """Cycle tracker that supports manual overrides and dynamic pin updates."""

    def __init__(self, pin: Optional[int] = None) -> None:
        super().__init__(pin)
        self._toggle_state = False

    def pulse(self) -> None:
        if self.mode == "automatic":
            self._toggle_state = not self._toggle_state
            self._write(self._toggle_state)
        elif self.mode == "on":
            self._write(True)
        else:
            self._write(False)

    def set_mode(self, mode: str) -> None:  # type: ignore[override]
        super().set_mode(mode)
        if mode == "automatic":
            self._write(self._toggle_state)
        elif mode == "on":
            self._write(True)
        else:
            self._write(False)

    def set_pin(self, pin: Optional[int]) -> None:  # type: ignore[override]
        super().set_pin(pin)
        if self.mode == "automatic":
            self._write(self._toggle_state)
        elif self.mode == "on":
            self._write(True)
        else:
            self._write(False)


class DebugControlPanel:
    """Tkinter panel that exposes ADS1256/GPIO configuration controls."""

    def __init__(
        self,
        parent: tk.Widget,
        backend: ADS1256Backend,
        range_switch: ConfigurableRangeSwitch,
        cycle_tracker: ConfigurableCycleTracker,
        gpio_buttons: Dict[str, int],
        drdy_pin: Optional[int] = None,
    ) -> None:
        self.backend = backend
        self.range_switch = range_switch
        self.cycle_tracker = cycle_tracker
        self.gpio_buttons = gpio_buttons
        self.drdy_pin = drdy_pin
        self.status_var = tk.StringVar(value="Ready.")

        self._build(parent)

    # ------------------------------------------------------------------
    def _build(self, parent: tk.Widget) -> None:
        parent.columnconfigure(0, weight=1)

        channel_frame = ttk.LabelFrame(parent, text="ADS1256 Channel Mapping")
        channel_frame.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        for col in range(2):
            channel_frame.columnconfigure(col, weight=1)

        channels = self.backend.channel_map()
        values = [str(i) for i in range(8)]

        ttk.Label(channel_frame, text="Voltage +").grid(row=0, column=0, sticky="w", pady=2)
        ttk.Label(channel_frame, text="Voltage -").grid(row=0, column=1, sticky="w", pady=2)

        self.voltage_pos = tk.StringVar(value=str(channels["voltage"][0]))
        self.voltage_neg = tk.StringVar(value=str(channels["voltage"][1]))
        self.resistance_channel = tk.StringVar(value=str(channels["resistance"][0]))
        self.current_channel = tk.StringVar(value=str(channels["current"][0]))

        ttk.Combobox(
            channel_frame,
            textvariable=self.voltage_pos,
            values=values,
            state="readonly",
            width=5,
        ).grid(row=1, column=0, sticky="w")
        ttk.Combobox(
            channel_frame,
            textvariable=self.voltage_neg,
            values=values,
            state="readonly",
            width=5,
        ).grid(row=1, column=1, sticky="w")

        ttk.Label(channel_frame, text="Resistance").grid(row=2, column=0, sticky="w", pady=(8, 2))
        ttk.Label(channel_frame, text="Current").grid(row=2, column=1, sticky="w", pady=(8, 2))

        ttk.Combobox(
            channel_frame,
            textvariable=self.resistance_channel,
            values=values,
            state="readonly",
            width=5,
        ).grid(row=3, column=0, sticky="w")
        ttk.Combobox(
            channel_frame,
            textvariable=self.current_channel,
            values=values,
            state="readonly",
            width=5,
        ).grid(row=3, column=1, sticky="w")

        ttk.Button(channel_frame, text="Apply", command=self._apply_channel_map).grid(
            row=4, column=0, columnspan=2, sticky="e", pady=(8, 0)
        )

        gpio_frame = ttk.LabelFrame(parent, text="GPIO Outputs")
        gpio_frame.grid(row=1, column=0, sticky="nsew", padx=8, pady=8)
        gpio_frame.columnconfigure(0, weight=1)
        gpio_frame.columnconfigure(1, weight=1)

        self._build_range_section(gpio_frame)
        self._build_cycle_section(gpio_frame)

        buttons_frame = ttk.LabelFrame(parent, text="GPIO Buttons")
        buttons_frame.grid(row=2, column=0, sticky="nsew", padx=8, pady=8)
        if self.gpio_buttons:
            for row, (name, pin) in enumerate(sorted(self.gpio_buttons.items())):
                ttk.Label(buttons_frame, text=name).grid(row=row, column=0, sticky="w", padx=(0, 8))
                ttk.Label(buttons_frame, text=str(pin)).grid(row=row, column=1, sticky="w")
        else:
            ttk.Label(buttons_frame, text="No button GPIOs configured.").grid(
                row=0, column=0, sticky="w"
            )

        info_frame = ttk.Frame(parent)
        info_frame.grid(row=3, column=0, sticky="ew", padx=8, pady=(0, 8))
        info_frame.columnconfigure(0, weight=1)

        if self.drdy_pin is not None:
            ttk.Label(
                info_frame,
                text=f"DRDY pin: {self.drdy_pin}",
            ).grid(row=0, column=0, sticky="w")
        else:
            ttk.Label(info_frame, text="DRDY pin: not configured").grid(row=0, column=0, sticky="w")

        ttk.Label(info_frame, textvariable=self.status_var, foreground="blue").grid(
            row=1, column=0, sticky="w", pady=(4, 0)
        )

    # ------------------------------------------------------------------
    def _build_range_section(self, parent: ttk.LabelFrame) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Range switch pin").grid(row=0, column=0, sticky="w")
        self.range_pin_var = tk.StringVar(
            value="" if self.range_switch.pin() is None else str(self.range_switch.pin())
        )
        ttk.Entry(frame, textvariable=self.range_pin_var, width=6).grid(row=0, column=1, sticky="w")
        ttk.Button(frame, text="Apply", command=self._update_range_pin).grid(row=0, column=2, padx=(8, 0))

        ttk.Label(frame, text="Mode").grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.range_mode_var = tk.StringVar(value=self.range_switch.mode)
        mode_frame = ttk.Frame(frame)
        mode_frame.grid(row=1, column=1, columnspan=2, sticky="w", pady=(8, 0))
        for idx, (label, mode) in enumerate(
            (("Automatic", "automatic"), ("On", "on"), ("Off", "off"))
        ):
            ttk.Radiobutton(
                mode_frame,
                text=label,
                value=mode,
                variable=self.range_mode_var,
                command=self._update_range_mode,
            ).grid(row=0, column=idx, padx=(0, 8))

    def _build_cycle_section(self, parent: ttk.LabelFrame) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=1, sticky="nsew", padx=8, pady=8)
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="Cycle tracker pin").grid(row=0, column=0, sticky="w")
        self.cycle_pin_var = tk.StringVar(
            value="" if self.cycle_tracker.pin() is None else str(self.cycle_tracker.pin())
        )
        ttk.Entry(frame, textvariable=self.cycle_pin_var, width=6).grid(row=0, column=1, sticky="w")
        ttk.Button(frame, text="Apply", command=self._update_cycle_pin).grid(row=0, column=2, padx=(8, 0))

        ttk.Label(frame, text="Mode").grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.cycle_mode_var = tk.StringVar(value=self.cycle_tracker.mode)
        mode_frame = ttk.Frame(frame)
        mode_frame.grid(row=1, column=1, columnspan=2, sticky="w", pady=(8, 0))
        for idx, (label, mode) in enumerate(
            (("Automatic", "automatic"), ("On", "on"), ("Off", "off"))
        ):
            ttk.Radiobutton(
                mode_frame,
                text=label,
                value=mode,
                variable=self.cycle_mode_var,
                command=self._update_cycle_mode,
            ).grid(row=0, column=idx, padx=(0, 8))

    # ------------------------------------------------------------------
    def _apply_channel_map(self) -> None:
        try:
            voltage = (int(self.voltage_pos.get()), int(self.voltage_neg.get()))
            resistance = int(self.resistance_channel.get())
            current = int(self.current_channel.get())
        except ValueError as exc:
            self.status_var.set(f"Invalid channel selection: {exc}")
            return

        try:
            self.backend.configure_channels(
                voltage=voltage, resistance=resistance, current=current
            )
        except ValueError as exc:
            self.status_var.set(str(exc))
        else:
            self.status_var.set(
                f"ADS1256 channel mapping updated to V{voltage[0]}-{voltage[1]} / R{resistance} / I{current}."
            )

        channel_map = self.backend.channel_map()

        self.voltage_pos.set(str(channel_map["voltage"][0]))
        self.voltage_neg.set(str(channel_map["voltage"][1]))
        self.resistance_channel.set(str(channel_map["resistance"][0]))
        self.current_channel.set(str(channel_map["current"][0]))

    def _update_range_pin(self) -> None:
        value = self.range_pin_var.get().strip()
        try:
            pin = _parse_optional_int(value)
            self.range_switch.set_pin(pin)
        except (ValueError, RuntimeError) as exc:
            self.status_var.set(str(exc))
        else:
            if pin is None:
                self.status_var.set("Range switch disabled.")
            else:
                self.status_var.set(f"Range switch set to GPIO {pin}.")

    def _update_cycle_pin(self) -> None:
        value = self.cycle_pin_var.get().strip()
        try:
            pin = _parse_optional_int(value)
            self.cycle_tracker.set_pin(pin)
        except (ValueError, RuntimeError) as exc:
            self.status_var.set(str(exc))
        else:
            if pin is None:
                self.status_var.set("Cycle tracker disabled.")
            else:
                self.status_var.set(f"Cycle tracker set to GPIO {pin}.")

    def _update_range_mode(self) -> None:
        mode = self.range_mode_var.get()
        try:
            self.range_switch.set_mode(mode)
        except ValueError as exc:
            self.status_var.set(str(exc))
        else:
            self.status_var.set(f"Range switch mode set to {mode}.")

    def _update_cycle_mode(self) -> None:
        mode = self.cycle_mode_var.get()
        try:
            self.cycle_tracker.set_mode(mode)
        except ValueError as exc:
            self.status_var.set(str(exc))
        else:
            self.status_var.set(f"Cycle tracker mode set to {mode}.")


# ------------------------------------------------------------------------------
# CLI helpers
# ------------------------------------------------------------------------------


def _parse_channel_pair(value: str) -> Tuple[int, int]:
    for separator in (",", "-", ":"):
        if separator in value:
            parts = value.split(separator)
            break
    else:
        parts = value.split()
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("Expected two channel numbers (e.g. 6-7)")
    try:
        pos, neg = (int(part.strip()) for part in parts)
    except ValueError as exc:  # pragma: no cover - user input error path
        raise argparse.ArgumentTypeError("Channel identifiers must be integers") from exc
    for channel in (pos, neg):
        _validate_channel(channel)
    return pos, neg


def _parse_single_channel(value: str) -> int:
    try:
        channel = int(value)
    except ValueError as exc:  # pragma: no cover - user input error path
        raise argparse.ArgumentTypeError("Channel identifiers must be integers") from exc
    _validate_channel(channel)
    return channel


def _parse_optional_int(value: str) -> Optional[int]:
    if not value:
        return None
    try:
        return int(value)
    except ValueError as exc:  # pragma: no cover - user input error path
        raise ValueError(f"Invalid integer '{value}'") from exc


def _validate_channel(channel: int) -> None:
    if not 0 <= channel <= 7:
        raise argparse.ArgumentTypeError("ADS1256 channels must be in the range 0-7")


def build_debug_arg_parser() -> argparse.ArgumentParser:
    parser = build_arg_parser()
    parser.description = "Run the Micro-DMM Raspberry Pi port with debugging controls"
    parser.add_argument(
        "--voltage-channels",
        type=_parse_channel_pair,
        default=(6, 7),
        metavar="POS-NEG",
        help="Differential channel pair for voltage measurements (default: 6-7)",
    )
    parser.add_argument(
        "--resistance-channel",
        type=_parse_single_channel,
        default=4,
        help="Single-ended channel for resistance measurements (default: 4)",
    )
    parser.add_argument(
        "--current-channel",
        type=_parse_single_channel,
        default=5,
        help="Single-ended channel for current measurements (default: 5)",
    )
    return parser


# ------------------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------------------


def main(argv: Optional[Tuple[str, ...]] = None) -> int:
    parser = build_debug_arg_parser()
    args = parser.parse_args(argv)

    _configure_logging(args.log_level)

    cleanup = ResourceGroup()

    range_switch = ConfigurableRangeSwitch(args.range_pin)
    cleanup.add(range_switch.close)

    drdy_reader: Optional[DataReadyReader] = None
    if args.drdy_pin is not None:
        drdy_reader = DataReadyReader(args.drdy_pin)
        cleanup.add(drdy_reader.close)

    cycle_tracker = ConfigurableCycleTracker(args.cycle_pin)
    cleanup.add(cycle_tracker.close)

    backend = ADS1256Backend(
        spi_bus=args.spi_bus,
        spi_device=args.spi_device,
        vref=args.vref,
        drdy_reader=(drdy_reader if drdy_reader is None else drdy_reader.__call__),
        range_switch=range_switch,
        voltage_channels=args.voltage_channels,
        resistance_channel=args.resistance_channel,
        current_channel=args.current_channel,
    )
    cleanup.add(backend.close)

    logger = PiLogger(args.log_dir)

    state = MeasurementState()
    state.voltage_display = not args.show_resistance
    state.current_enabled = not args.no_current

    gpio_mapping = _parse_gpio_mappings(args.gpio_button)

    root = tk.Tk()
    notebook = ttk.Notebook(root)
    notebook.pack(fill="both", expand=True)

    measurement_tab = ttk.Frame(notebook)
    debug_tab = ttk.Frame(notebook)

    notebook.add(measurement_tab, text="Measurements")
    notebook.add(debug_tab, text="Debug Controls")

    app = MicroDmmApp(
        root,
        state=state,
        logger=logger,
        gpio_pins=gpio_mapping or None,
        container=measurement_tab,
    )

    DebugControlPanel(
        debug_tab,
        backend=backend,
        range_switch=range_switch,
        cycle_tracker=cycle_tracker,
        gpio_buttons=gpio_mapping,
        drdy_pin=args.drdy_pin,
    )

    service = MeasurementService(
        backend,
        app,
        sample_rate_hz=args.sample_rate,
        cycle_tracker=cycle_tracker,
    )
    cleanup.add(service.close)

    def _shutdown() -> None:
        service.stop()
        cleanup.close()
        root.destroy()

    def _handle_signal(signum: int, _frame) -> None:
        signal.signal(signum, signal.SIG_IGN)
        _shutdown()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    root.protocol("WM_DELETE_WINDOW", lambda: _shutdown())

    service.start()
    try:
        app.run()
    finally:
        cleanup.close()
    return 0


if __name__ == "__main__":  # pragma: no cover - script entry point
    sys.exit(main())
