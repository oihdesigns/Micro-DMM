"""alicat_ascii.py — Alicat instruments over the ASCII serial protocol.

This is the protocol on RS-232 units and on RS-485 units not ordered with
Modbus. Commands are "<unit id><verb><args><CR>" and most replies are a data
frame: whitespace-separated numbers whose meaning is fixed by the device's own
data-frame configuration, e.g. for a mass flow controller

    A +014.05 +025.00 +000.00 +000.00 000.00 00000.0 N2 HLD
      pressure temp    volumetric mass  setpt  total  gas status

Field *order* is stable across models; field *count* is not, so the driver
either takes an explicit reading map from the config or infers one from the
number of numeric columns. Units are not transmitted — they live on the
device's display and in the `??D*` response.

Reference: Alicat Serial Primer (Feb 2023, Rev 2).
"""

from __future__ import annotations

import time
from typing import Optional, Sequence

from . import core
from .bus import SerialBus
from .core import CommandRejected, Device, DeviceInfo, Snapshot, TransportError

CR = b"\r"

# Status/error codes that may trail the data frame (Serial Primer page 8).
STATUS_CODES = {"ADC", "EXH", "HLD", "LCK", "MOV", "OPL", "OVR", "POV",
                "TMF", "TOV", "VOV"}

# Column meaning by number of numeric fields, for the common default frames.
FRAME_BY_COUNT = {
    1: [core.PRESSURE],
    2: [core.PRESSURE, core.TEMPERATURE],
    3: [core.PRESSURE, core.TEMPERATURE, core.VOLUMETRIC],
    4: [core.PRESSURE, core.TEMPERATURE, core.VOLUMETRIC, core.MASS],
    5: [core.PRESSURE, core.TEMPERATURE, core.VOLUMETRIC, core.MASS,
        core.SETPOINT],
    6: [core.PRESSURE, core.TEMPERATURE, core.VOLUMETRIC, core.MASS,
        core.SETPOINT, core.TOTALIZER],
}

HOLD_VERB = {"closed": "HC", "current": "HP", "exhaust": "E", "cancel": "C"}


def _is_number(token: str) -> bool:
    try:
        float(token)
    except ValueError:
        return False
    return True


def parse_frame(line: str, readings: Optional[Sequence[str]] = None) -> Snapshot:
    """Turn one data frame into a Snapshot. Raises ValueError on junk."""
    tokens = line.split()
    if not tokens:
        raise ValueError("empty frame")
    if tokens[0] == "?":
        raise CommandRejected("device answered '?' (command not understood)")

    unit_id = tokens[0]
    numbers, trailing = [], []
    for token in tokens[1:]:
        if not trailing and _is_number(token):
            numbers.append(float(token))
        else:
            trailing.append(token)

    names = list(readings) if readings else FRAME_BY_COUNT.get(len(numbers), [])
    values = {name: value for name, value in zip(names, numbers)}
    # Any column beyond the known map is still worth keeping, under a
    # positional name, rather than being silently dropped.
    for i in range(len(names), len(numbers)):
        values[f"field{i + 1}"] = numbers[i]

    gas, flags = None, []
    for token in trailing:
        if token.upper() in STATUS_CODES:
            flags.append(token.upper())
        elif gas is None:
            gas = token
        else:
            flags.append(token.upper())

    return Snapshot(device=unit_id, values=values, gas=gas,
                    flags=tuple(flags), raw=line)


class AlicatAscii(Device):
    """One Alicat in polling mode, addressed by its unit ID letter."""

    def __init__(self, info: DeviceInfo, bus: SerialBus, unit_id: str = "A",
                 readings: Optional[Sequence[str]] = None):
        super().__init__(info)
        self.unit_id = unit_id.strip().upper()[:1] or "A"
        info.address = self.unit_id
        self.bus = bus
        self.readings = list(readings) if readings else None
        self.last_line = ""

    # -- plumbing -----------------------------------------------------------
    def _exchange(self, command: str, *, expect_reply: bool = True) -> str:
        payload = (command + "\r").encode("ascii")

        def reader(port):
            if not expect_reply:
                return ""
            deadline = time.monotonic() + max(self.bus.cfg.timeout, 0.1) * 4
            return self.bus.read_line(port, deadline).decode("ascii", "replace").strip()

        line = self.bus.transaction(payload, reader)
        self.last_line = line
        if line == "?":
            raise CommandRejected(f"{command!r} rejected by unit {self.unit_id}")
        return line

    def _exchange_multiline(self, command: str, quiet: float = 0.35) -> list[str]:
        """For ??D* / ??M* / ??G*, which answer with a table, not a frame."""
        payload = (command + "\r").encode("ascii")

        def reader(port):
            lines, last = [], time.monotonic()
            while time.monotonic() - last < quiet:
                chunk = port.readline()
                if chunk:
                    text = chunk.decode("ascii", "replace").strip()
                    if text:
                        lines.append(text)
                    last = time.monotonic()
            if not lines:
                raise TransportError(f"no response to {command!r}")
            return lines

        return self.bus.transaction(payload, reader)

    # -- lifecycle ----------------------------------------------------------
    def connect(self) -> None:
        self.bus.open()
        snap = self.poll()
        if not self.readings:
            # Lock in whatever the first frame implied, so a later frame that
            # gains a status column cannot shift the column mapping.
            inferred = FRAME_BY_COUNT.get(
                len([k for k in snap.values if not k.startswith("field")]))
            if inferred:
                self.readings = inferred
        self.info.is_controller = core.SETPOINT in snap.values
        try:
            self.info.firmware = self._exchange(f"{self.unit_id}VE")
        except (TransportError, CommandRejected):
            pass

    # -- reading ------------------------------------------------------------
    def poll(self) -> Snapshot:
        line = self._exchange(self.unit_id)
        snap = parse_frame(line, self.readings)
        snap.device = self.name
        return snap

    # -- control ------------------------------------------------------------
    def set_setpoint(self, value: float) -> None:
        # "AS12.5" — the reply is a data frame echoing the new setpoint.
        self._exchange(f"{self.unit_id}S{value:g}")

    def hold(self, mode: str) -> None:
        if mode not in HOLD_VERB:
            raise ValueError(f"hold mode must be one of {sorted(HOLD_VERB)}")
        self._exchange(f"{self.unit_id}{HOLD_VERB[mode]}")

    def tare_flow(self) -> None:
        self._exchange(f"{self.unit_id}V")

    def tare_pressure(self, absolute: bool = False) -> None:
        self._exchange(f"{self.unit_id}{'PC' if absolute else 'P'}")

    def set_gas(self, gas_number: int) -> None:
        self._exchange(f"{self.unit_id}G{int(gas_number)}")

    def reset_totalizer(self, which: int = 1) -> None:
        try:
            self._exchange(f"{self.unit_id}T {int(which)}")
        except CommandRejected:
            self._exchange(f"{self.unit_id}T")   # pre-8v00 single-totalizer form

    def set_loop_variable(self, variable: int) -> None:
        self._exchange(f"{self.unit_id}LV {int(variable)}")

    # -- discovery / recovery ----------------------------------------------
    def query_data_frame(self) -> list[str]:
        """`??D*` — the device's own description of its data frame columns."""
        return self._exchange_multiline(f"{self.unit_id}??D*")

    def query_manufacturer(self) -> list[str]:
        return self._exchange_multiline(f"{self.unit_id}??M*")

    def query_gases(self) -> list[str]:
        return self._exchange_multiline(f"{self.unit_id}??G*")

    def stop_streaming(self, new_unit_id: Optional[str] = None) -> None:
        """Rescue a bus flooded by a device left in streaming mode (@)."""
        target = (new_unit_id or self.unit_id).upper()[:1]
        self._exchange(f"@@ {target}", expect_reply=False)
        time.sleep(0.2)
        self.unit_id = target
        self.info.address = target

    def change_unit_id(self, new_unit_id: str) -> None:
        target = new_unit_id.strip().upper()[:1]
        if not ("A" <= target <= "Z"):
            raise ValueError("unit ID must be a letter A-Z")
        self._exchange(f"{self.unit_id}@={target}", expect_reply=False)
        time.sleep(0.2)
        self.unit_id = target
        self.info.address = target

    # -- terminal -----------------------------------------------------------
    def raw_exchange(self, text: str) -> str:
        """Send a line verbatim. A bare verb gets this device's unit ID
        prefixed, so you can type `V` instead of `AV`; anything starting with
        a unit-ID letter followed by more text is sent unchanged."""
        text = text.strip()
        if not text:
            return ""
        if text[0].upper() != self.unit_id and not text.startswith("@"):
            text = self.unit_id + text
        if text.endswith("*"):
            return "\n".join(self._exchange_multiline(text))
        return self._exchange(text)
