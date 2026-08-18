"""discover.py — find out what is actually on a bus.

Two scans, both cheap and both read-only:

  scan_modbus  walks slave addresses, asking each for the gas-number register
  scan_ascii   walks unit IDs A-Z, polling each for a data frame

Neither writes anything to an instrument. Run them through the bus worker
(manager.submit_to_bus) when the manager is live, so the scan takes its turn
on the line instead of colliding with polling.
"""

from __future__ import annotations

from dataclasses import dataclass

from .alicat_ascii import parse_frame
from .bus import SerialBus
from .core import TransportError
from .modbus import ModbusMaster
from .alicat_modbus import GASES, OPT_GAS, REG_VERSION


@dataclass
class Found:
    address: str
    protocol: str
    detail: str = ""

    def __str__(self) -> str:
        return f"{self.protocol} {self.address}: {self.detail}"


def scan_modbus(bus: SerialBus, first: int = 1, last: int = 16,
                timeout: float = 0.15) -> list[Found]:
    original = bus.cfg.timeout
    bus.cfg.timeout = timeout
    try:
        bus.close()
        bus.open()
        found = []
        for slave in range(first, last + 1):
            master = ModbusMaster(bus, slave)
            try:
                gas = master.read_registers(OPT_GAS, 1)[0]
            except TransportError:
                continue
            detail = f"gas {GASES.get(gas, gas)}"
            try:
                major, minor, custom, _ = master.read_registers(REG_VERSION, 4)
                detail += f", firmware {major}v{minor:02d}.{custom}"
            except TransportError:
                detail += ", firmware pre-10v07"
            found.append(Found(str(slave), "modbus", detail))
        return found
    finally:
        bus.cfg.timeout = original
        bus.close()


def scan_ascii(bus: SerialBus, ids: str = "ABCDEFGHIJKLMNOPQRSTUVWXYZ",
               timeout: float = 0.15) -> list[Found]:
    import time

    original = bus.cfg.timeout
    bus.cfg.timeout = timeout
    try:
        bus.close()
        bus.open()
        found = []
        for unit in ids:
            def reader(port, unit=unit):
                deadline = time.monotonic() + timeout * 3
                return bus.read_line(port, deadline).decode("ascii", "replace")

            try:
                line = bus.transaction(f"{unit}\r".encode("ascii"), reader, retries=0)
            except TransportError:
                continue
            line = line.strip()
            if not line or not line.upper().startswith(unit):
                continue
            try:
                snap = parse_frame(line)
                detail = ", ".join(f"{k}={v:g}" for k, v in snap.values.items())
                if snap.gas:
                    detail += f", gas {snap.gas}"
            except Exception:      # noqa: BLE001 - an odd frame still means "present"
                detail = line
            found.append(Found(unit, "ascii", detail))
        return found
    finally:
        bus.cfg.timeout = original
        bus.close()
