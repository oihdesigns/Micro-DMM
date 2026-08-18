"""core.py — device-agnostic types every driver in flowlab speaks.

Nothing in here knows about Alicat, Modbus or serial ports. A new instrument
(pressure controller, valve manifold, scale, thermocouple reader) joins the
system by subclassing Device and returning Snapshots; the manager, the logger,
the sequence runner and the GUI then work with it unchanged.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

# ── Canonical reading names ────────────────────────────────────────────────
# Drivers map whatever their instrument reports onto these keys. Anything not
# in this list is still allowed in Snapshot.values — the GUI just shows it in
# the "other" row rather than in a dedicated tile.
PRESSURE = "pressure"
SECONDARY_PRESSURE = "secondary_pressure"
BAROMETER = "barometer"
TEMPERATURE = "temperature"
VOLUMETRIC = "volumetric"
MASS = "mass"
SETPOINT = "setpoint"
TOTALIZER = "totalizer"
TOTALIZER2 = "totalizer2"
VALVE_DRIVE = "valve_drive"
HUMIDITY = "humidity"

READING_LABELS = {
    PRESSURE: "Pressure",
    SECONDARY_PRESSURE: "2nd pressure",
    BAROMETER: "Barometer",
    TEMPERATURE: "Temperature",
    VOLUMETRIC: "Volumetric",
    MASS: "Mass flow",
    SETPOINT: "Setpoint",
    TOTALIZER: "Total",
    TOTALIZER2: "Total 2",
    VALVE_DRIVE: "Valve drive",
    HUMIDITY: "Humidity",
}


class DeviceError(Exception):
    """Any failure to talk to, or command, an instrument."""


class TransportError(DeviceError):
    """Wire-level failure: timeout, CRC, framing, port closed."""


class CommandRejected(DeviceError):
    """The instrument answered, and said no."""


@dataclass
class Snapshot:
    """One complete observation of one device, as of `t`.

    `values` holds engineering-unit floats keyed by the constants above.
    `flags` holds instrument status strings (Alicat's HLD/MOV/OVR..., or the
    decoded Modbus status bits). `raw` keeps whatever the driver saw on the
    wire, which is what you actually want when a rig misbehaves at 2am.
    """

    device: str
    t: float = field(default_factory=time.time)
    values: dict[str, float] = field(default_factory=dict)
    gas: Optional[str] = None
    flags: tuple[str, ...] = ()
    raw: str = ""
    ok: bool = True
    error: str = ""

    def get(self, key: str, default: float = float("nan")) -> float:
        return self.values.get(key, default)

    @classmethod
    def failure(cls, device: str, exc: BaseException) -> "Snapshot":
        return cls(device=device, ok=False, error=f"{type(exc).__name__}: {exc}")


@dataclass
class DeviceInfo:
    """Static description of an instrument — what it is, not what it reads."""

    name: str
    model: str = ""
    serial: str = ""
    firmware: str = ""
    address: str = ""          # Modbus slave id or ASCII unit ID, as text
    units: str = ""            # engineering units of the controlled variable
    full_scale: float = 100.0  # used for slider range and % setpoints
    is_controller: bool = True


class Device:
    """The interface the rest of flowlab programs against.

    Every method may raise DeviceError. Implementations are *not* required to
    be thread-safe: serialization is the bus worker's job (see manager.py), so
    a driver can assume one caller at a time.
    """

    def __init__(self, info: DeviceInfo):
        self.info = info

    # -- identity -----------------------------------------------------------
    @property
    def name(self) -> str:
        return self.info.name

    def connect(self) -> None:
        """Open/verify the link and fill in whatever DeviceInfo fields it can."""

    def close(self) -> None:
        pass

    # -- reading ------------------------------------------------------------
    def poll(self) -> Snapshot:
        raise NotImplementedError

    # -- control (controllers only) -----------------------------------------
    def set_setpoint(self, value: float) -> None:
        raise NotImplementedError

    def hold(self, mode: str) -> None:
        """mode: 'closed' | 'current' | 'exhaust' | 'cancel'."""
        raise NotImplementedError

    # -- maintenance --------------------------------------------------------
    def tare_flow(self) -> None:
        raise NotImplementedError

    def tare_pressure(self, absolute: bool = False) -> None:
        raise NotImplementedError

    def set_gas(self, gas_number: int) -> None:
        raise NotImplementedError

    def reset_totalizer(self, which: int = 1) -> None:
        raise NotImplementedError

    # -- escape hatch -------------------------------------------------------
    def raw_exchange(self, text: str) -> str:
        """Send something hand-typed and return what came back.

        The Terminal tab uses this. Keeping it on the interface means every
        driver stays debuggable without the GUI knowing any protocol.
        """
        raise NotImplementedError
