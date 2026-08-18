#!/usr/bin/env python3
"""selftest.py — run the wire code against fake instruments.

    python selftest.py

No hardware and no serial ports involved: a fake Modbus RTU slave and a fake
ASCII unit are wired into SerialBus in place of pyserial, so the CRCs, frame
layout, register decoding, command flow and data-frame parsing all execute for
real. If this passes, what is left to be wrong on the bench is wiring, baud
rate, addresses and firmware age — not the protocol code.
"""

from __future__ import annotations

import struct
import sys
import time

from flowlab import core
from flowlab.alicat_ascii import AlicatAscii, parse_frame
from flowlab.alicat_modbus import (CMD_HOLD_VALVE, CMD_SET_GAS, CMD_TARE,
                                   AlicatModbus, decode_status)
from flowlab.bus import BusConfig, SerialBus
from flowlab.core import CommandRejected, DeviceInfo
from flowlab.modbus import crc16, decode_float, encode_float

FAILURES: list[str] = []


def check(condition: bool, label: str) -> None:
    print(f"  {'PASS' if condition else 'FAIL'}  {label}")
    if not condition:
        FAILURES.append(label)


def close_to(a: float, b: float, tol: float = 1e-3) -> bool:
    return abs(a - b) <= tol


# ── fake instruments ───────────────────────────────────────────────────────
class FakeModbusSlave:
    """Just enough of an Alicat to answer FC3/FC4/FC16 correctly."""

    def __init__(self, slave: int = 1, full_scale: float = 500.0):
        self.slave = slave
        self.full_scale = full_scale
        self.setpoint = 0.0
        self.flow = 0.0
        self.gas = 8               # N2
        self.status = 0
        self.last_command = (0, 0)
        self.command_result = 0
        self.tares = 0

    # -- register bank ------------------------------------------------------
    def registers(self) -> dict[int, int]:
        """Register *number* -> 16-bit value."""
        bank: dict[int, int] = {}

        def put_float(number: int, value: float) -> None:
            hi, lo = struct.unpack(">HH", struct.pack(">f", value))
            bank[number], bank[number + 1] = hi, lo

        bank[1000], bank[1001] = self.last_command[0], self.command_result
        put_float(1010, self.setpoint)
        put_float(1088, 1.234567)
        bank[1090], bank[1091], bank[1092], bank[1093] = 10, 19, 0, 3
        bank[1094], bank[1095] = 0x0007, 0x4661       # serial 476769
        bank[1200] = self.gas
        bank[1201], bank[1202] = (self.status >> 16) & 0xFFFF, self.status & 0xFFFF
        put_float(1203, 14.7 - 0.001 * self.flow)     # pressure
        put_float(1205, 24.9)                         # temperature
        put_float(1207, self.flow * 1.01)             # volumetric
        put_float(1209, self.flow)                    # mass
        put_float(1211, self.setpoint)                # setpoint
        put_float(1213, 12.5)                         # totalizer
        return bank

    def execute(self, cmd_id: int, argument: int) -> None:
        self.last_command = (cmd_id, argument)
        self.command_result = 0
        if cmd_id == CMD_SET_GAS:
            if argument > 29:
                self.command_result = 32770            # INVALID_ARGUMENT
            else:
                self.gas = argument
        elif cmd_id == CMD_TARE:
            self.tares += 1
        elif cmd_id == CMD_HOLD_VALVE:
            self.status = (self.status | 0x100) if argument else (self.status & ~0x100)
        elif cmd_id != 0:
            self.command_result = 32771                # UNSUPPORTED

    # -- framing ------------------------------------------------------------
    def handle(self, request: bytes) -> bytes:
        if crc16(request[:-2]) != struct.unpack("<H", request[-2:])[0]:
            return b""                                  # bad CRC: stay silent
        if request[0] != self.slave:
            return b""
        function = request[1]
        if function in (3, 4):
            address, count = struct.unpack(">HH", request[2:6])
            bank = self.registers()
            payload = b"".join(struct.pack(">H", bank.get(address + 1 + i, 0))
                               for i in range(count))
            body = bytes([self.slave, function, len(payload)]) + payload
        elif function == 16:
            address, count = struct.unpack(">HH", request[2:6])
            values = [struct.unpack(">H", request[7 + 2 * i:9 + 2 * i])[0]
                      for i in range(count)]
            number = address + 1
            if number == 1000 and count == 2:
                self.execute(values[0], values[1])
            elif number == 1010 and count == 2:
                self.setpoint = decode_float(values)
                self.flow = self.setpoint          # instant response, for testing
            body = bytes([self.slave, function]) + request[2:6]
        else:
            body = bytes([self.slave, function | 0x80, 1])
        return body + struct.pack("<H", crc16(body))


class FakeAsciiUnit:
    def __init__(self, unit_id: str = "A", full_scale: float = 500.0):
        self.unit_id = unit_id
        self.setpoint = 0.0
        self.flow = 0.0
        self.gas = "N2"
        self.hold = ""
        self.tares = 0

    def frame(self) -> str:
        status = f" {self.hold}" if self.hold else ""
        return (f"{self.unit_id} +014.05 +025.00 {self.flow * 1.01:+08.2f} "
                f"{self.flow:+08.2f} {self.setpoint:07.2f} 00012.5 "
                f"{self.gas}{status}")

    def handle(self, line: str) -> str:
        line = line.strip()
        if not line or line[0].upper() != self.unit_id:
            return ""
        verb = line[1:].strip()
        if verb == "":
            return self.frame()
        if verb.upper().startswith("S"):
            self.setpoint = float(verb[1:])
            self.flow = self.setpoint
            self.hold = ""
            return self.frame()
        if verb.upper() == "V":
            self.tares += 1
            return self.frame()
        if verb.upper() == "HC":
            self.hold = "HLD"
            return self.frame()
        if verb.upper() == "C":
            self.hold = ""
            return self.frame()
        if verb.upper().startswith("G"):
            self.gas = {8: "N2", 7: "He", 0: "Air"}.get(int(verb[1:]), "?")
            return self.frame()
        if verb.upper() == "VE":
            return f"{self.unit_id} 10v19.03"
        return "?"


class FakePort:
    """The two pyserial methods SerialBus actually uses, plus a buffer."""

    def __init__(self, instrument, ascii_mode: bool):
        self.instrument = instrument
        self.ascii_mode = ascii_mode
        self.buffer = bytearray()
        self.is_open = True

    def write(self, data: bytes) -> int:
        if self.ascii_mode:
            reply = self.instrument.handle(data.decode("ascii"))
            if reply:
                self.buffer += (reply + "\r").encode("ascii")
        else:
            self.buffer += self.instrument.handle(bytes(data))
        return len(data)

    def read(self, n: int = 1) -> bytes:
        chunk, self.buffer = bytes(self.buffer[:n]), self.buffer[n:]
        return chunk

    def readline(self) -> bytes:
        index = self.buffer.find(b"\r")
        if index < 0:
            chunk, self.buffer = bytes(self.buffer), bytearray()
            return chunk
        chunk, self.buffer = bytes(self.buffer[:index + 1]), self.buffer[index + 1:]
        return chunk

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False


class FakeBus(SerialBus):
    def __init__(self, cfg: BusConfig, instrument, ascii_mode: bool):
        super().__init__(cfg)
        self.instrument = instrument
        self.ascii_mode = ascii_mode

    def open(self) -> None:
        if self._port is None or not self._port.is_open:
            self._port = FakePort(self.instrument, self.ascii_mode)


# ── tests ──────────────────────────────────────────────────────────────────
def test_encoding() -> None:
    print("\nModbus encoding")
    # Canonical Modbus example frame: 01 03 00 00 00 01 -> CRC 84 0A.
    frame = bytes.fromhex("010300000001")
    check(struct.pack("<H", crc16(frame)).hex() == "840a", "CRC-16 matches known frame")
    check(close_to(decode_float(encode_float(1.234567)), 1.234567, 1e-6),
          "float32 round-trips through register pairs")
    check(decode_float([0x3F9E, 0x064B]) - 1.234567 < 1e-6,
          "big-endian register order decodes Alicat's test value")
    check(decode_status(0x100 | 0x10) == ("MOV", "HLD"), "status bitfield decode")


def test_modbus_device() -> None:
    print("\nModbus device")
    slave = FakeModbusSlave(slave=3)
    bus = FakeBus(BusConfig(id="t", port="fake", timeout=0.05, retries=0),
                  slave, ascii_mode=False)
    device = AlicatModbus(DeviceInfo(name="MFC", full_scale=500.0), bus, slave=3)
    device.connect()
    check(device.info.firmware == "10v19.0", f"firmware read ({device.info.firmware})")
    check(device.info.serial == "476769", f"serial read ({device.info.serial})")
    check(device.mb.order == "big", "word order detected from register 1088")

    device.set_setpoint(123.5)
    snap = device.poll()
    check(close_to(snap.get(core.SETPOINT), 123.5), "setpoint written and read back")
    check(close_to(snap.get(core.MASS), 123.5), "mass flow reading decoded")
    check(close_to(snap.get(core.TOTALIZER), 12.5), "totalizer slot decoded")
    check(snap.gas == "N2", f"gas number decoded ({snap.gas})")

    device.tare_flow()
    device.tare_flow()
    check(slave.tares == 2, f"repeated tare executes twice ({slave.tares})")

    device.hold("closed")
    check("HLD" in device.poll().flags, "hold sets the HLD status bit")
    device.hold("cancel")
    check("HLD" not in device.poll().flags, "cancel clears it")

    device.set_gas(7)
    check(device.poll().gas == "He", "gas change")
    try:
        device.set_gas(200)
        check(False, "invalid gas is rejected")
    except CommandRejected as exc:
        check("INVALID_ARGUMENT" in str(exc), f"invalid gas is rejected ({exc})")

    check(device.raw_exchange("read 1200 1").startswith("1200="), "terminal: read")
    device.raw_exchange("sp 42")
    check(close_to(slave.setpoint, 42.0), "terminal: sp")


def test_ascii_device() -> None:
    print("\nASCII device")
    snap = parse_frame("A +014.05 +025.00 +000.12 +000.10 000.00 00012.5 N2 HLD")
    check(close_to(snap.get(core.MASS), 0.10), "frame: mass column")
    check(close_to(snap.get(core.TOTALIZER), 12.5), "frame: totalizer column")
    check(snap.gas == "N2" and snap.flags == ("HLD",), "frame: gas and status split")
    meter = parse_frame("B +010.02 +025.00 +128.0 +87.2 He")
    check(close_to(meter.get(core.MASS), 87.2) and meter.gas == "He",
          "frame: 4-column meter")

    unit = FakeAsciiUnit("A")
    bus = FakeBus(BusConfig(id="t", port="fake", protocol="ascii", timeout=0.05,
                            retries=0), unit, ascii_mode=True)
    device = AlicatAscii(DeviceInfo(name="MFC-A", full_scale=500.0), bus, unit_id="A")
    device.connect()
    check(device.info.firmware.endswith("10v19.03"),
          f"firmware query ({device.info.firmware})")
    check(device.info.is_controller, "controller detected from the frame width")

    device.set_setpoint(87.5)
    check(close_to(device.poll().get(core.SETPOINT), 87.5), "setpoint round-trip")
    device.hold("closed")
    check("HLD" in device.poll().flags, "hold flag appears")
    device.hold("cancel")
    check(device.poll().flags == (), "hold cleared")
    device.tare_flow()
    check(unit.tares == 1, "tare flow sent")
    device.set_gas(7)
    check(device.poll().gas == "He", "gas change")
    check(device.raw_exchange("VE").endswith("10v19.03"), "terminal passthrough")


def test_manager() -> None:
    print("\nManager, sequence and log")
    from flowlab import DeviceManager, Sequence, SequenceRunner, Step
    from flowlab.simulator import SimulatedController

    manager = DeviceManager()
    manager.add_bus(BusConfig(id="sim", protocol="sim"))
    manager.add_device(SimulatedController(
        DeviceInfo(name="SIM", full_scale=100.0), tau=0.05), "sim", poll_hz=20)
    time.sleep(0.4)
    check(manager.latest("SIM") is not None and manager.latest("SIM").ok,
          "simulated device polls")

    manager.set_setpoint("SIM", 50).result(1.0)
    time.sleep(0.5)
    check(close_to(manager.latest("SIM").get(core.MASS), 50.0, 2.0),
          "setpoint reaches the reading")

    runner = SequenceRunner(manager, Sequence(
        steps=[Step("SIM", 20, ramp_s=0.4, hold_s=0.2)], zero_at_end=True))
    runner.start()
    runner.join(6)
    check(runner.finished and not runner.error, f"sequence completes ({runner.error})")
    times, values = manager.history("SIM", core.MASS)
    check(len(times) > 10 and len(times) == len(values), "history accumulates")
    manager.shutdown()
    check(not any(w.is_alive() for w in manager.workers.values()),
          "workers stop on shutdown")


def main() -> int:
    test_encoding()
    test_modbus_device()
    test_ascii_device()
    test_manager()
    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED:")
        for label in FAILURES:
            print(f"  - {label}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
