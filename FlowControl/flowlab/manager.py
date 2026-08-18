"""manager.py — the scheduling and state layer between drivers and any UI.

One worker thread per bus. Everything that touches a device — polling, a
setpoint write, a tare, a terminal line — is a job on that bus's queue, so a
shared RS-485 line is never re-entered and the GUI thread never blocks on
serial I/O. Results come back two ways:

  * `Future`s, for the caller that asked (the GUI attaches a callback);
  * an event queue, for anything watching the whole system (the GUI drains it
    on a Tk `after` tick; a headless controller could read it in a loop).

This module is deliberately UI-free: a supervisory control script can import
DeviceManager alone and get polling, history, logging and safe command
serialization without importing tkinter.
"""

from __future__ import annotations

import csv
import queue
import threading
import time
from collections import deque
from concurrent.futures import Future
from dataclasses import dataclass, field
from typing import Callable, Iterable, Optional

from . import core
from .bus import BusConfig, SerialBus
from .core import Device, Snapshot

# Columns written to CSV, in this order, for every device.
LOG_KEYS = [core.MASS, core.SETPOINT, core.VOLUMETRIC, core.PRESSURE,
            core.TEMPERATURE, core.TOTALIZER, core.VALVE_DRIVE]

EVENT_SNAPSHOT = "snapshot"
EVENT_STATUS = "status"      # human-readable line for the log pane
EVENT_ERROR = "error"


@dataclass
class DeviceEntry:
    device: Device
    bus_key: str
    period: float = 0.5
    enabled: bool = True
    next_poll: float = 0.0
    history: deque = field(default_factory=lambda: deque(maxlen=7200))
    consecutive_errors: int = 0


class BusWorker(threading.Thread):
    """Serializes every access to one bus: jobs first, then due polls."""

    def __init__(self, key: str, manager: "DeviceManager", bus: Optional[SerialBus]):
        super().__init__(name=f"bus-{key}", daemon=True)
        self.key = key
        self.manager = manager
        self.bus = bus
        self.jobs: "queue.Queue[tuple[Callable, Future, str]]" = queue.Queue()
        self._stopping = threading.Event()

    def submit(self, fn: Callable, description: str = "") -> Future:
        fut: Future = Future()
        self.jobs.put((fn, fut, description))
        return fut

    def stop(self) -> None:
        self._stopping.set()

    def run(self) -> None:
        while not self._stopping.is_set():
            did_work = self._run_jobs()
            did_work |= self._poll_due()
            if not did_work:
                time.sleep(0.005)
        if self.bus is not None:
            self.bus.close()

    def _run_jobs(self) -> bool:
        ran = False
        while True:
            try:
                fn, fut, description = self.jobs.get_nowait()
            except queue.Empty:
                return ran
            ran = True
            if not fut.set_running_or_notify_cancel():
                continue
            try:
                result = fn()
                fut.set_result(result)
                if description:
                    self.manager.emit(EVENT_STATUS, f"{description}: ok")
            except BaseException as exc:      # noqa: BLE001 - reported, not swallowed
                fut.set_exception(exc)
                self.manager.emit(
                    EVENT_ERROR, f"{description or 'job'} failed: {exc}")

    def _poll_due(self) -> bool:
        now = time.monotonic()
        polled = False
        for entry in self.manager.entries_for_bus(self.key):
            if not entry.enabled or now < entry.next_poll:
                continue
            entry.next_poll = now + entry.period
            polled = True
            try:
                snap = entry.device.poll()
                entry.consecutive_errors = 0
            except BaseException as exc:      # noqa: BLE001
                entry.consecutive_errors += 1
                snap = Snapshot.failure(entry.device.name, exc)
                if entry.consecutive_errors in (1, 10, 100):
                    self.manager.emit(
                        EVENT_ERROR,
                        f"{entry.device.name}: {snap.error}"
                        f"{' (repeating)' if entry.consecutive_errors > 1 else ''}")
            self.manager.publish(entry, snap)
        return polled


class DeviceManager:
    """Owns buses, devices, their history, and the log file."""

    def __init__(self, history_seconds: float = 3600.0):
        self.buses: dict[str, SerialBus] = {}
        self.workers: dict[str, BusWorker] = {}
        self.entries: dict[str, DeviceEntry] = {}
        self.events: "queue.Queue[tuple[str, object]]" = queue.Queue(maxsize=10000)
        self.history_seconds = history_seconds
        self._lock = threading.RLock()
        self._latest: dict[str, Snapshot] = {}
        self._log_file = None
        self._log_writer = None
        self._log_keys = list(LOG_KEYS)
        self._log_lock = threading.Lock()
        self.log_path: Optional[str] = None
        self.log_rows = 0

    # -- wiring -------------------------------------------------------------
    def add_bus(self, cfg: BusConfig) -> Optional[SerialBus]:
        """Register a bus. `protocol == "sim"` gets a worker but no serial port."""
        bus = None if cfg.protocol == "sim" else SerialBus(cfg)
        with self._lock:
            if cfg.id in self.workers:
                raise ValueError(f"bus {cfg.id!r} already exists")
            if bus is not None:
                self.buses[cfg.id] = bus
            worker = BusWorker(cfg.id, self, bus)
            self.workers[cfg.id] = worker
            worker.start()
        return bus

    def add_device(self, device: Device, bus_key: str, poll_hz: float = 2.0) -> DeviceEntry:
        with self._lock:
            if device.name in self.entries:
                raise ValueError(f"device {device.name!r} already exists")
            if bus_key not in self.workers:
                raise KeyError(f"no such bus {bus_key!r}")
            entry = DeviceEntry(device=device, bus_key=bus_key,
                                period=1.0 / max(poll_hz, 0.05))
            self.entries[device.name] = entry
        self.submit(device.name, device.connect, f"connect {device.name}")
        return entry

    def remove_device(self, name: str) -> None:
        with self._lock:
            entry = self.entries.pop(name, None)
            self._latest.pop(name, None)
        if entry is not None:
            self.submit_to_bus(entry.bus_key, entry.device.close, f"close {name}")

    def shutdown(self) -> None:
        for worker in list(self.workers.values()):
            worker.stop()
        for worker in list(self.workers.values()):
            worker.join(timeout=2.0)
        for bus in self.buses.values():
            bus.close()
        self.stop_log()

    # -- job submission -----------------------------------------------------
    def submit_to_bus(self, bus_key: str, fn: Callable, description: str = "") -> Future:
        return self.workers[bus_key].submit(fn, description)

    def submit(self, device_name: str, fn: Callable, description: str = "") -> Future:
        entry = self.entries[device_name]
        return self.workers[entry.bus_key].submit(fn, description)

    # Convenience wrappers — each returns a Future the caller may ignore.
    def set_setpoint(self, name: str, value: float) -> Future:
        device = self.entries[name].device
        return self.submit(name, lambda: device.set_setpoint(value),
                           f"{name} setpoint {value:g}")

    def hold(self, name: str, mode: str) -> Future:
        device = self.entries[name].device
        return self.submit(name, lambda: device.hold(mode), f"{name} hold {mode}")

    def tare_flow(self, name: str) -> Future:
        device = self.entries[name].device
        return self.submit(name, device.tare_flow, f"{name} tare flow")

    def tare_pressure(self, name: str, absolute: bool = False) -> Future:
        device = self.entries[name].device
        return self.submit(name, lambda: device.tare_pressure(absolute),
                           f"{name} tare pressure")

    def set_gas(self, name: str, gas_number: int) -> Future:
        device = self.entries[name].device
        return self.submit(name, lambda: device.set_gas(gas_number),
                           f"{name} gas {gas_number}")

    def reset_totalizer(self, name: str, which: int = 1) -> Future:
        device = self.entries[name].device
        return self.submit(name, lambda: device.reset_totalizer(which),
                           f"{name} reset totalizer")

    def raw_exchange(self, name: str, text: str) -> Future:
        device = self.entries[name].device
        return self.submit(name, lambda: device.raw_exchange(text), "")

    def all_off(self) -> list[Future]:
        """Panic button: zero every controller, then hold its valves closed."""
        futures = []
        for name, entry in list(self.entries.items()):
            if entry.device.info.is_controller:
                futures.append(self.set_setpoint(name, 0.0))
                futures.append(self.hold(name, "closed"))
        return futures

    # -- state --------------------------------------------------------------
    def entries_for_bus(self, bus_key: str) -> list[DeviceEntry]:
        with self._lock:
            return [e for e in self.entries.values() if e.bus_key == bus_key]

    def names(self) -> list[str]:
        with self._lock:
            return list(self.entries)

    def latest(self, name: str) -> Optional[Snapshot]:
        with self._lock:
            return self._latest.get(name)

    def history(self, name: str, key: str) -> tuple[list[float], list[float]]:
        """(times, values) for one reading, oldest first, seconds since epoch."""
        with self._lock:
            entry = self.entries.get(name)
            samples = list(entry.history) if entry else []
        times, values = [], []
        for t, values_map in samples:
            if key in values_map:
                times.append(t)
                values.append(values_map[key])
        return times, values

    def publish(self, entry: DeviceEntry, snap: Snapshot) -> None:
        with self._lock:
            self._latest[entry.device.name] = snap
            if snap.ok:
                entry.history.append((snap.t, dict(snap.values)))
                cutoff = snap.t - self.history_seconds
                while entry.history and entry.history[0][0] < cutoff:
                    entry.history.popleft()
        if snap.ok:
            self._write_log(snap)
        self.emit(EVENT_SNAPSHOT, snap)

    def emit(self, kind: str, payload) -> None:
        try:
            self.events.put_nowait((kind, payload))
        except queue.Full:
            pass       # a UI that stopped draining must not stall acquisition

    def drain_events(self, limit: int = 500) -> list[tuple[str, object]]:
        out = []
        for _ in range(limit):
            try:
                out.append(self.events.get_nowait())
            except queue.Empty:
                break
        return out

    # -- logging ------------------------------------------------------------
    def start_log(self, path: str, keys: Optional[Iterable[str]] = None) -> None:
        self.stop_log()
        keys = list(keys) if keys else list(LOG_KEYS)
        with self._log_lock:
            self._log_file = open(path, "w", newline="", encoding="utf-8")
            self._log_writer = csv.writer(self._log_file)
            self._log_writer.writerow(
                ["iso_time", "epoch", "device", *keys, "gas", "flags"])
            self._log_keys = keys
            self.log_path = path
            self.log_rows = 0
        self.emit(EVENT_STATUS, f"logging to {path}")

    def stop_log(self) -> None:
        with self._log_lock:
            if self._log_file is not None:
                self._log_file.close()
            self._log_file = None
            self._log_writer = None
            path, self.log_path = self.log_path, None
        if path:
            self.emit(EVENT_STATUS, f"log closed: {path} ({self.log_rows} rows)")

    def _write_log(self, snap: Snapshot) -> None:
        with self._log_lock:
            if self._log_writer is None:
                return
            row = [time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(snap.t)),
                   f"{snap.t:.3f}", snap.device]
            row += [f"{snap.values[k]:.5g}" if k in snap.values else ""
                    for k in self._log_keys]
            row += [snap.gas or "", "|".join(snap.flags)]
            self._log_writer.writerow(row)
            self.log_rows += 1
            if self.log_rows % 20 == 0:
                self._log_file.flush()
