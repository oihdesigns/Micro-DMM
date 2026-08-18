"""alicat_modbus.py — Alicat instruments over Modbus RTU (RS-485).

Register numbers and command IDs below are from the Alicat Modbus Manual
(Nov 2024, Rev 7). Two families of reading registers exist:

  "optimized"  1200+, present since firmware 6v17.0 — a gas number, a status
               bitfield and 20 generic reading slots whose meaning follows the
               device data-frame order (for an MC: pressure, temperature,
               volumetric, mass, setpoint, totalizer).
  "standard"   1346+, added in 10v07.0 — every quantity at a fixed register
               regardless of model, which is nicer but not present on older
               units.

The driver defaults to the optimized block because every Modbus-equipped
Alicat has it, and can be switched per device via `block="standard"`.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

from . import core
from .bus import SerialBus
from .core import CommandRejected, Device, DeviceInfo, Snapshot, TransportError
from .modbus import READ_HOLDING, READ_INPUT, ModbusMaster, decode_float

# ── register numbers (1-based, exactly as printed in the manual) ───────────
REG_CMD_ID = 1000          # write: command ID     read: last executed ID
REG_CMD_ARG = 1001         # write: argument       read: last result
REG_SETPOINT = 1010        # float32, read/write (write-only before 10v07.0)
REG_TEST_VALUE = 1088      # float32, always 1.234567 (10v19.0+)
REG_VERSION = 1090         # major, minor, custom, internal (10v07.0+)
REG_SERIAL = 1094          # int32 (10v19.0+)

OPT_GAS = 1200             # uint16
OPT_STATUS = 1201          # int32 bitfield
OPT_READING1 = 1203        # float32 x 20, consecutive

STD_GAS = 1347
STD_STATUS = 1348
STD_FIRST = 1350           # setpoint, valve, pressure, 2nd, baro, temp, vol,
                           # mass, tot1, tot2, humidity — all float32
STD_ORDER = [core.SETPOINT, core.VALVE_DRIVE, core.PRESSURE,
             core.SECONDARY_PRESSURE, core.BAROMETER, core.TEMPERATURE,
             core.VOLUMETRIC, core.MASS, core.TOTALIZER, core.TOTALIZER2,
             core.HUMIDITY]

# Default meaning of the optimized reading slots for a mass flow controller.
MC_READINGS = [core.PRESSURE, core.TEMPERATURE, core.VOLUMETRIC,
               core.MASS, core.SETPOINT, core.TOTALIZER]
# A meter has no setpoint slot; a pressure controller reports fewer still.
MFM_READINGS = [core.PRESSURE, core.TEMPERATURE, core.VOLUMETRIC, core.MASS]

# ── device status bitfield (register 1201/1348, manual page 17) ────────────
STATUS_BITS = [
    (1 << 0, "TOV", "temperature overrange"),
    (1 << 1, "TUV", "temperature underrange"),
    (1 << 2, "VOV", "volumetric overrange"),
    (1 << 3, "VUV", "volumetric underrange"),
    (1 << 4, "MOV", "mass flow overrange"),
    (1 << 5, "MUV", "mass flow underrange"),
    (1 << 6, "POV", "pressure overrange"),
    (1 << 7, "OVR", "totalizer at limit"),
    (1 << 8, "HLD", "control loop held"),
    (1 << 9, "ERR", "internal hardware error"),
    (1 << 10, "OPL", "over user pressure limit"),
    (1 << 11, "TMF", "flow overrange while totalizing"),
]

# ── command IDs ────────────────────────────────────────────────────────────
CMD_NOOP = 0
CMD_SET_GAS = 1
CMD_MIX_GAS = 2
CMD_TARE = 4               # arg 0 gauge/diff pressure, 1 absolute, 2 flow
CMD_RESET_TOTALIZER1 = 5
CMD_HOLD_VALVE = 6         # arg 0 cancel, 1 closed, 2 current, 3 exhaust
CMD_GAIN_P = 8
CMD_GAIN_D = 9
CMD_GAIN_I = 10
CMD_LOOP_VARIABLE = 11     # arg 0 mass, 1 volumetric, 2 diff P, 3 abs P, 4 gauge P
CMD_LOOP_ALGORITHM = 13    # arg 1 PDF, 2 PD2I
CMD_READ_GAIN = 14         # arg 0 P, 1 D, 2 I
CMD_RESET_TOTALIZER = 80   # arg 1 or 2  (10v19.0+)

TARE_PRESSURE, TARE_ABSOLUTE, TARE_FLOW = 0, 1, 2
HOLD_ARG = {"cancel": 0, "closed": 1, "current": 2, "exhaust": 3}
LOOP_VARIABLES = {0: "mass flow", 1: "volumetric", 2: "differential P",
                  3: "absolute P", 4: "gauge P"}

CMD_STATUS = {
    0: "SUCCESS", 1: "IN_PROGRESS", 2: "INVALID_ID", 3: "INVALID_ARGUMENT",
    4: "UNSUPPORTED", 5: "INVALID_MIX_IDX", 6: "INVALID_MIX_GAS",
    7: "INVALID_MIX_PCT",
}
# ...and the same conditions as seen in the limited-command result register.
CMD_ERROR = {32769: "INVALID_ID", 32770: "INVALID_ARGUMENT", 32771: "UNSUPPORTED",
             32772: "INVALID_MIX_IDX", 32773: "INVALID_MIX_GAS",
             32774: "INVALID_MIX_PCT"}

GASES = {0: "Air", 1: "Ar", 2: "CH4", 3: "CO", 4: "CO2", 5: "C2H6", 6: "H2",
         7: "He", 8: "N2", 9: "N2O", 10: "Ne", 11: "O2", 12: "C3H8",
         13: "nC4H10", 14: "C2H2", 15: "C2H4", 16: "iC4H10", 17: "Kr",
         18: "Xe", 19: "SF6", 20: "C-25", 21: "C-10", 22: "C-8", 23: "C-2",
         24: "C-75", 25: "A-75", 26: "A-25", 27: "A1025", 28: "Star29",
         29: "P-5"}


def decode_status(bits: int) -> tuple[str, ...]:
    return tuple(code for mask, code, _ in STATUS_BITS if bits & mask)


class AlicatModbus(Device):
    """One Alicat on a Modbus RTU segment."""

    def __init__(self, info: DeviceInfo, bus: SerialBus, slave: int,
                 readings: Optional[Sequence[str]] = None,
                 block: str = "optimized", word_order: str = "big"):
        super().__init__(info)
        info.address = str(slave)
        self.bus = bus
        self.mb = ModbusMaster(bus, slave, word_order)
        self.readings = list(readings) if readings else list(MC_READINGS)
        self.block = block
        self._read_fc = READ_HOLDING     # falls back to 4 if 3 is refused
        self._status_fc = READ_INPUT     # command results: manual says FC4

    # -- plumbing -----------------------------------------------------------
    def _read(self, number: int, count: int) -> list[int]:
        try:
            return self.mb.read_registers(number, count, self._read_fc)
        except TransportError as exc:
            if "exception 1" not in str(exc):
                raise
            self._read_fc = READ_INPUT if self._read_fc == READ_HOLDING else READ_HOLDING
            return self.mb.read_registers(number, count, self._read_fc)

    def command(self, cmd_id: int, argument: int = 0, *, check: bool = True) -> int:
        """Run a limited (16-bit) command and return its result register.

        A no-op is issued first because the instrument only acts when the
        ID/argument pair *changes* — without it, taring twice in a row would
        silently do nothing the second time.
        """
        self.mb.write_registers(REG_CMD_ID, [CMD_NOOP, 0])
        self.mb.write_registers(REG_CMD_ID, [int(cmd_id) & 0xFFFF, int(argument) & 0xFFFF])
        if not check:
            return 0
        for _ in range(10):
            try:
                echo_id, result = self.mb.read_registers(REG_CMD_ID, 2, self._status_fc)
            except TransportError:
                echo_id, result = self.mb.read_registers(REG_CMD_ID, 2, READ_HOLDING)
            if echo_id != (int(cmd_id) & 0xFFFF):
                continue
            if result in CMD_ERROR:
                raise CommandRejected(
                    f"command {cmd_id}({argument}) rejected: {CMD_ERROR[result]}")
            return result
        raise CommandRejected(f"command {cmd_id}({argument}) never echoed back")

    # -- lifecycle ----------------------------------------------------------
    def connect(self) -> None:
        self.bus.open()
        try:
            major, minor, custom, _internal = self._read(REG_VERSION, 4)
            self.info.firmware = f"{major}v{minor:02d}.{custom}"
        except TransportError:
            self.info.firmware = ""     # pre-10v07.0: version registers absent
        try:
            hi, lo = self._read(REG_SERIAL, 2)
            self.info.serial = str((hi << 16) | lo)
        except TransportError:
            pass
        try:
            self.mb.detect_order(REG_TEST_VALUE)
        except TransportError:
            pass                        # older firmware: trust the default
        self.poll()                     # prove the reading block answers

    # -- reading ------------------------------------------------------------
    def poll(self) -> Snapshot:
        if self.block == "standard":
            regs = self._read(STD_GAS, 2 + 2 * len(STD_ORDER))
            gas_num, status = regs[0], (regs[1] << 16) | regs[2]
            floats = regs[3:]
            names = STD_ORDER
        else:
            count = 3 + 2 * len(self.readings)
            regs = self._read(OPT_GAS, count)
            gas_num = regs[0]
            status = (regs[1] << 16) | regs[2]
            floats = regs[3:]
            names = self.readings

        values: dict[str, float] = {}
        for i, key in enumerate(names):
            pair = floats[2 * i:2 * i + 2]
            if len(pair) < 2:
                break
            value = decode_float(pair, self.mb.order)
            if not math.isnan(value):   # Alicat returns quiet NaN for "n/a"
                values[key] = value

        return Snapshot(
            device=self.name, values=values,
            gas=GASES.get(gas_num, f"gas{gas_num}"),
            flags=decode_status(status),
            raw=self.mb.last_exchange[1],
        )

    # -- control ------------------------------------------------------------
    def set_setpoint(self, value: float) -> None:
        self.mb.write_float(REG_SETPOINT, value)

    def read_setpoint(self) -> float:
        return self.mb.read_float(REG_SETPOINT, self._read_fc)

    def hold(self, mode: str) -> None:
        if mode not in HOLD_ARG:
            raise ValueError(f"hold mode must be one of {sorted(HOLD_ARG)}")
        self.command(CMD_HOLD_VALVE, HOLD_ARG[mode])

    def tare_flow(self) -> None:
        self.command(CMD_TARE, TARE_FLOW)

    def tare_pressure(self, absolute: bool = False) -> None:
        self.command(CMD_TARE, TARE_ABSOLUTE if absolute else TARE_PRESSURE)

    def set_gas(self, gas_number: int) -> None:
        self.command(CMD_SET_GAS, int(gas_number))

    def reset_totalizer(self, which: int = 1) -> None:
        if which == 1:
            self.command(CMD_RESET_TOTALIZER1, 0)
        else:
            self.command(CMD_RESET_TOTALIZER, which)

    def set_loop_variable(self, variable: int) -> None:
        self.command(CMD_LOOP_VARIABLE, variable)

    def set_gains(self, p: Optional[int] = None, i: Optional[int] = None,
                  d: Optional[int] = None) -> None:
        for value, cmd in ((p, CMD_GAIN_P), (i, CMD_GAIN_I), (d, CMD_GAIN_D)):
            if value is not None:
                self.command(cmd, int(value))

    def read_gains(self) -> dict[str, int]:
        return {name: self.command(CMD_READ_GAIN, arg)
                for name, arg in (("P", 0), ("D", 1), ("I", 2))}

    # -- terminal -----------------------------------------------------------
    def raw_exchange(self, text: str) -> str:
        """Mini command language for the GUI terminal.

            read <register> [count]     read holding/input registers
            float <register>            read a float32 pair
            write <register> <v> [v..]  write raw 16-bit registers
            setf <register> <value>     write a float32 pair
            cmd <id> [arg]              run a command, report the result
            sp <value>                  shorthand for setf 1010
        """
        parts = text.split()
        if not parts:
            return ""
        op, args = parts[0].lower(), parts[1:]
        if op == "read":
            number = int(args[0])
            count = int(args[1]) if len(args) > 1 else 1
            regs = self._read(number, count)
            return " ".join(f"{number + i}={r} (0x{r:04X})" for i, r in enumerate(regs))
        if op == "float":
            number = int(args[0])
            return f"{number}-{number + 1} = {self.mb.read_float(number, self._read_fc)}"
        if op == "write":
            number = int(args[0])
            self.mb.write_registers(number, [int(a, 0) for a in args[1:]])
            return "written"
        if op in ("setf", "sp"):
            number = REG_SETPOINT if op == "sp" else int(args[0])
            value = float(args[-1])
            self.mb.write_float(number, value)
            return f"{number} <- {value}"
        if op == "cmd":
            cmd_id = int(args[0])
            arg = int(args[1]) if len(args) > 1 else 0
            result = self.command(cmd_id, arg)
            return f"result {result} ({CMD_STATUS.get(result, 'value')})"
        raise ValueError(
            f"unknown terminal command {op!r}; try read/float/write/setf/cmd/sp")
