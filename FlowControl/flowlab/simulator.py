"""simulator.py — a fake Alicat, good enough to build against.

Not a protocol emulator: it implements the Device interface directly, so the
manager, sequence runner, logger and GUI all exercise their real paths with no
hardware attached. The flow model is a first-order lag toward setpoint with a
little noise and a settling overshoot, which is enough to make ramps, holds
and totalization look and behave the way they do on the bench.
"""

from __future__ import annotations

import math
import random
import time

from . import core
from .core import Device, DeviceInfo, Snapshot


class SimulatedController(Device):
    def __init__(self, info: DeviceInfo, tau: float = 0.35, noise: float = 0.0015,
                 seed: int | None = None):
        super().__init__(info)
        info.address = info.address or "sim"
        self.tau = tau                    # first-order time constant, seconds
        self.noise = noise                # fraction of full scale, 1 sigma
        self.rng = random.Random(seed)
        self.setpoint = 0.0
        self.flow = 0.0
        self.totalizer = 0.0
        self.gas = "N2"
        self.hold_mode = "cancel"
        self.zero_offset = 0.0
        self._last = time.monotonic()
        self._supply_psia = 40.0

    # -- lifecycle ----------------------------------------------------------
    def connect(self) -> None:
        self.info.firmware = self.info.firmware or "10v19.0 (simulated)"
        self.info.serial = self.info.serial or "SIM000"

    # -- model --------------------------------------------------------------
    def _advance(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self._last, 0.0), 1.0)
        self._last = now
        if dt <= 0:
            return

        if self.hold_mode == "closed":
            target = 0.0
        elif self.hold_mode == "current":
            target = self.flow
        elif self.hold_mode == "exhaust":
            target = 0.0
        else:
            target = self.setpoint

        alpha = 1.0 - math.exp(-dt / self.tau)
        self.flow += (target - self.flow) * alpha
        self.flow = max(0.0, min(self.flow, self.info.full_scale * 1.02))
        self.totalizer += self.flow * dt / 60.0     # SCCM -> sccs of volume

    # -- reading ------------------------------------------------------------
    def poll(self) -> Snapshot:
        self._advance()
        fs = self.info.full_scale
        jitter = self.rng.gauss(0.0, self.noise * fs)
        mass = max(0.0, self.flow + jitter + self.zero_offset)
        # Volumetric runs a few percent above mass at ~1 atm, temperature drifts.
        temperature = 25.0 + 0.4 * math.sin(time.time() / 37.0) + self.rng.gauss(0, 0.02)
        pressure = self._supply_psia - 0.02 * mass + self.rng.gauss(0, 0.01)
        volumetric = mass * (14.696 / max(pressure, 1.0)) * ((temperature + 273.15) / 298.15)

        flags = []
        if self.hold_mode in ("closed", "current"):
            flags.append("HLD")
        if self.hold_mode == "exhaust":
            flags.append("EXH")
        if mass > fs:
            flags.append("MOV")

        return Snapshot(
            device=self.name,
            values={core.PRESSURE: pressure, core.TEMPERATURE: temperature,
                    core.VOLUMETRIC: volumetric, core.MASS: mass,
                    core.SETPOINT: self.setpoint, core.TOTALIZER: self.totalizer},
            gas=self.gas, flags=tuple(flags),
            raw=f"sim {mass:.3f}/{self.setpoint:.3f}",
        )

    # -- control ------------------------------------------------------------
    def set_setpoint(self, value: float) -> None:
        self._advance()
        self.setpoint = max(0.0, min(float(value), self.info.full_scale))
        if self.hold_mode != "cancel":
            self.hold_mode = "cancel"    # a new setpoint releases the hold

    def hold(self, mode: str) -> None:
        self._advance()
        if mode not in ("closed", "current", "exhaust", "cancel"):
            raise ValueError(mode)
        self.hold_mode = mode

    def tare_flow(self) -> None:
        self.zero_offset = 0.0

    def tare_pressure(self, absolute: bool = False) -> None:
        pass

    def set_gas(self, gas_number: int) -> None:
        from .alicat_modbus import GASES
        self.gas = GASES.get(int(gas_number), f"gas{gas_number}")

    def reset_totalizer(self, which: int = 1) -> None:
        self.totalizer = 0.0

    def raw_exchange(self, text: str) -> str:
        parts = text.split()
        if not parts:
            return ""
        verb = parts[0].lower()
        if verb in ("sp", "s") and len(parts) > 1:
            self.set_setpoint(float(parts[1]))
            return f"setpoint {self.setpoint:g}"
        if verb == "drift" and len(parts) > 1:      # inject a zero-offset fault
            self.zero_offset = float(parts[1])
            return f"zero offset {self.zero_offset:g}"
        if verb == "supply" and len(parts) > 1:
            self._supply_psia = float(parts[1])
            return f"supply {self._supply_psia:g} psia"
        if verb == "poll":
            return self.poll().raw
        return ("simulated device — try: sp <value>, poll, "
                "drift <offset>, supply <psia>")
