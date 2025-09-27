"""Runtime harness that glues the ADS1256 backend to the Tk UI on a Raspberry Pi."""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional

try:  # pragma: no cover - optional hardware dependencies
    from gpiozero import DigitalInputDevice, LED
except Exception:  # pragma: no cover
    DigitalInputDevice = None  # type: ignore
    LED = None  # type: ignore

from .ads1256_backend import ADS1256Backend, CurrentResult, ResistanceResult, VoltageResult
from .app import MicroDmmApp
from .data_model import MeasurementState
from .logging import PiLogger


_LOG = logging.getLogger(__name__)


class ResourceGroup:
    """Collect cleanup callables that should run on shutdown."""

    def __init__(self) -> None:
        self._cleanup: List[Callable[[], None]] = []

    def add(self, closer: Callable[[], None]) -> None:
        self._cleanup.append(closer)

    def close(self) -> None:
        while self._cleanup:
            closer = self._cleanup.pop()
            try:
                closer()
            except Exception:  # pragma: no cover - best effort shutdown
                _LOG.exception("Error while cleaning up resource")


class RangeSwitch:
    """Callable wrapper that drives the hi/lo ohms range MOSFET."""

    def __init__(self, pin: int) -> None:
        if LED is None:
            raise RuntimeError(
                "gpiozero not available – install gpiozero or run without --range-pin"
            )
        self._driver = LED(pin)

    def __call__(self, high_range: bool) -> None:
        if high_range:
            self._driver.on()
        else:
            self._driver.off()

    def close(self) -> None:
        self._driver.close()


class CycleTracker:
    """Toggle a GPIO pin each sample so an external scope can watch timing."""

    def __init__(self, pin: int) -> None:
        if LED is None:
            raise RuntimeError(
                "gpiozero not available – install gpiozero or run without --cycle-pin"
            )
        self._driver = LED(pin)
        self._state = False

    def pulse(self) -> None:
        self._state = not self._state
        if self._state:
            self._driver.on()
        else:
            self._driver.off()

    def close(self) -> None:
        self._driver.close()


class DataReadyReader:
    """Read the ADS1256 DRDY pin (active-low)."""

    def __init__(self, pin: int) -> None:
        if DigitalInputDevice is None:
            raise RuntimeError(
                "gpiozero not available – install gpiozero or omit --drdy-pin"
            )
        self._input = DigitalInputDevice(pin, pull_up=True)

    def __call__(self) -> bool:
        # gpiozero returns 1 for high, 0 for low.  The backend expects
        # ``True`` while the ADC is busy and ``False`` when data is ready.
        return bool(self._input.value)

    def close(self) -> None:
        self._input.close()


class MeasurementService:
    """Background thread that polls the ADS1256 and feeds the UI."""

    def __init__(
        self,
        backend: ADS1256Backend,
        app: MicroDmmApp,
        *,
        sample_rate_hz: float,
        cycle_tracker: Optional[CycleTracker] = None,
    ) -> None:
        if sample_rate_hz <= 0:
            raise ValueError("Sample rate must be positive")
        self._backend = backend
        self._app = app
        self._cycle = cycle_tracker
        self._interval = 1.0 / sample_rate_hz
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run_loop, name="microdmm-measure", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def close(self) -> None:
        self.stop()

    # ------------------------------------------------------------------
    def _run_loop(self) -> None:
        state = self._app.state
        state.current_enabled = True
        next_deadline = time.monotonic()
        while not self._stop.is_set():
            start = time.monotonic()
            try:
                voltage = self._backend.read_voltage()
                resistance = self._backend.read_resistance()
                current = self._backend.read_current(
                    current_enabled=state.current_enabled,
                    high_range=state.current_range_high,
                )
            except Exception:
                _LOG.exception("Measurement step failed; retrying after delay")
                self._stop.wait(0.5)
                continue

            self._publish_measurement(voltage, resistance, current)

            if self._cycle is not None:
                self._cycle.pulse()

            next_deadline += self._interval
            delay = max(0.0, next_deadline - time.monotonic())
            self._stop.wait(delay)

    def _publish_measurement(
        self,
        voltage: VoltageResult,
        resistance: ResistanceResult,
        current: CurrentResult,
    ) -> None:
        state = self._app.state
        state.current_range_high = current.range_high
        timestamp_ms = time.time() * 1000.0
        self._app.ingest_measurement(
            voltage=voltage.value,
            resistance=resistance.value,
            current=current.value,
            timestamp_ms=timestamp_ms,
        )


# ==============================================================================
# CLI wiring
# ==============================================================================

def _parse_gpio_mappings(values: Iterable[str]) -> Dict[str, int]:
    mapping: Dict[str, int] = {}
    for token in values:
        if "=" not in token:
            raise argparse.ArgumentTypeError(
                f"Invalid mapping '{token}'. Expected name=pin."
            )
        key, value = token.split("=", 1)
        key = key.strip()
        try:
            mapping[key] = int(value)
        except ValueError as exc:  # pragma: no cover - user error path
            raise argparse.ArgumentTypeError(
                f"Invalid GPIO pin '{value}' in mapping '{token}'"
            ) from exc
    return mapping


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the Micro-DMM Raspberry Pi port")
    parser.add_argument("--spi-bus", type=int, default=0, help="SPI bus index (default: 0)")
    parser.add_argument("--spi-device", type=int, default=0, help="SPI device index (default: 0)")
    parser.add_argument("--vref", type=float, default=2.5, help="Reference voltage used by the ADS1256 (default: 2.5V)")
    parser.add_argument("--sample-rate", type=float, default=20.0, help="UI update/sample rate in Hz")
    parser.add_argument("--drdy-pin", type=int, help="GPIO connected to the ADS1256 DRDY line (BCM numbering)")
    parser.add_argument("--range-pin", type=int, help="GPIO driving the hi/lo resistance range MOSFET")
    parser.add_argument("--cycle-pin", type=int, help="Optional GPIO to pulse each sample for diagnostics")
    parser.add_argument("--log-dir", type=Path, help="Directory where CSV logs will be written")
    parser.add_argument(
        "--gpio-button",
        action="append",
        default=[],
        metavar="NAME=PIN",
        help="Attach a gpiozero Button to a UI control (e.g. mode=17,manual_log=27)",
    )
    parser.add_argument(
        "--no-current",
        action="store_true",
        help="Disable current measurements and display",
    )
    parser.add_argument(
        "--show-resistance",
        action="store_true",
        help="Show resistance on the primary readout instead of voltage",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Logging verbosity",
    )
    return parser


def _configure_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    _configure_logging(args.log_level)

    cleanup = ResourceGroup()

    range_switch: Optional[RangeSwitch] = None
    if args.range_pin is not None:
        range_switch = RangeSwitch(args.range_pin)
        cleanup.add(range_switch.close)

    drdy_reader: Optional[DataReadyReader] = None
    if args.drdy_pin is not None:
        drdy_reader = DataReadyReader(args.drdy_pin)
        cleanup.add(drdy_reader.close)

    cycle_tracker: Optional[CycleTracker] = None
    if args.cycle_pin is not None:
        cycle_tracker = CycleTracker(args.cycle_pin)
        cleanup.add(cycle_tracker.close)

    backend = ADS1256Backend(
        spi_bus=args.spi_bus,
        spi_device=args.spi_device,
        vref=args.vref,
        drdy_reader=(drdy_reader if drdy_reader is None else drdy_reader.__call__),
        range_switch=(range_switch if range_switch is None else range_switch.__call__),
    )
    cleanup.add(backend.close)

    logger = PiLogger(args.log_dir)

    state = MeasurementState()
    state.voltage_display = not args.show_resistance
    state.current_enabled = not args.no_current

    gpio_mapping = _parse_gpio_mappings(args.gpio_button)

    import tkinter as tk  # deferred import until display is needed

    root = tk.Tk()
    app = MicroDmmApp(root, state=state, logger=logger, gpio_pins=gpio_mapping or None)

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
        _LOG.info("Received signal %s, shutting down", signum)
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


if __name__ == "__main__":  # pragma: no cover - script entry
    sys.exit(main())
