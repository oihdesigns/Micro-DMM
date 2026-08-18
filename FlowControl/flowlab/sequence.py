"""sequence.py — time-based setpoint programs.

A Sequence is the smallest useful piece of "control system" above a single
setpoint box: a list of steps, each targeting one device, optionally ramped,
optionally waiting for the reading to settle before the hold time starts. The
runner is a plain thread that talks to DeviceManager, so it works headless and
composes with anything else submitting jobs on the same bus.

Extending this into recipes with branching, interlocks or PID-over-devices
means subclassing Step or writing a second runner against the same manager —
no changes needed below the manager line.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from . import core
from .manager import DeviceManager


@dataclass
class Step:
    device: str
    setpoint: float
    ramp_s: float = 0.0        # 0 = jump straight to the setpoint
    hold_s: float = 10.0       # time at setpoint after the ramp/settle
    settle_tol: Optional[float] = None   # engineering units; None = don't wait
    settle_timeout: float = 30.0
    label: str = ""

    def describe(self) -> str:
        bits = [f"{self.device} -> {self.setpoint:g}"]
        if self.ramp_s:
            bits.append(f"ramp {self.ramp_s:g}s")
        if self.settle_tol is not None:
            bits.append(f"settle ±{self.settle_tol:g}")
        bits.append(f"hold {self.hold_s:g}s")
        return ", ".join(bits)


@dataclass
class Sequence:
    steps: list[Step] = field(default_factory=list)
    loop: bool = False
    zero_at_end: bool = True

    def total_seconds(self) -> float:
        return sum(s.ramp_s + s.hold_s for s in self.steps)


class SequenceRunner(threading.Thread):
    """Executes a Sequence against a DeviceManager until done or stopped."""

    TICK = 0.2                 # ramp update interval, seconds

    def __init__(self, manager: DeviceManager, sequence: Sequence,
                 on_progress: Optional[Callable[[int, str, float], None]] = None):
        super().__init__(name="sequence", daemon=True)
        self.manager = manager
        self.sequence = sequence
        self.on_progress = on_progress
        self._stopping = threading.Event()
        self.current_step = -1
        self.finished = False
        self.error = ""

    def stop(self) -> None:
        self._stopping.set()

    @property
    def running(self) -> bool:
        return self.is_alive() and not self._stopping.is_set()

    # -- internals ----------------------------------------------------------
    def _report(self, index: int, phase: str, fraction: float) -> None:
        self.current_step = index
        if self.on_progress:
            try:
                self.on_progress(index, phase, fraction)
            except Exception:      # a UI callback must never kill the run
                pass

    def _sleep(self, seconds: float) -> bool:
        """Interruptible sleep; False means 'stop was requested'."""
        return not self._stopping.wait(seconds)

    def _start_value(self, step: Step) -> float:
        snap = self.manager.latest(step.device)
        if snap and snap.ok and core.SETPOINT in snap.values:
            return snap.values[core.SETPOINT]
        return 0.0

    def _run_step(self, index: int, step: Step) -> bool:
        if step.ramp_s > 0:
            start = self._start_value(step)
            span = step.setpoint - start
            t0 = time.monotonic()
            while True:
                elapsed = time.monotonic() - t0
                fraction = min(elapsed / step.ramp_s, 1.0)
                self.manager.set_setpoint(step.device, start + span * fraction)
                self._report(index, "ramp", fraction)
                if fraction >= 1.0:
                    break
                if not self._sleep(self.TICK):
                    return False
        else:
            self.manager.set_setpoint(step.device, step.setpoint)
            self._report(index, "set", 1.0)

        if step.settle_tol is not None:
            deadline = time.monotonic() + step.settle_timeout
            while time.monotonic() < deadline:
                snap = self.manager.latest(step.device)
                if snap and snap.ok:
                    value = snap.values.get(core.MASS,
                                            snap.values.get(core.VOLUMETRIC))
                    if value is not None and abs(value - step.setpoint) <= step.settle_tol:
                        break
                self._report(index, "settle", 0.0)
                if not self._sleep(self.TICK):
                    return False

        t0 = time.monotonic()
        while True:
            elapsed = time.monotonic() - t0
            self._report(index, "hold", min(elapsed / max(step.hold_s, 1e-9), 1.0))
            if elapsed >= step.hold_s:
                return True
            if not self._sleep(min(self.TICK, step.hold_s - elapsed)):
                return False

    def run(self) -> None:
        try:
            while True:
                for index, step in enumerate(self.sequence.steps):
                    if self._stopping.is_set():
                        return
                    if step.device not in self.manager.names():
                        self.error = f"step {index + 1}: no device named {step.device!r}"
                        return
                    if not self._run_step(index, step):
                        return
                if not self.sequence.loop or self._stopping.is_set():
                    break
            self.finished = True
        except Exception as exc:       # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            if self.sequence.zero_at_end:
                for name in {s.device for s in self.sequence.steps}:
                    if name in self.manager.names():
                        self.manager.set_setpoint(name, 0.0)
            self._report(-1, "done", 1.0)
