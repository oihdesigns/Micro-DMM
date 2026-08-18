"""modbus.py — a small Modbus RTU master, built directly on SerialBus.

Deliberately dependency-free: pymodbus has churned its API across major
versions and this is 150 lines of very stable protocol. Supported functions
are the three Alicat implements:

    3  read holding registers   (RTU only)
    4  read input registers
    16 write multiple registers

Addressing note, straight from the Alicat Modbus manual: the tables print a
*register number* that is always one greater than the *register address* put
on the wire. Everything in flowlab is written in register numbers (so the code
reads like the manual) and converted here, once.
"""

from __future__ import annotations

import struct
import time
from typing import Sequence

from .bus import SerialBus
from .core import TransportError

READ_HOLDING = 3
READ_INPUT = 4
WRITE_MULTIPLE = 16

EXCEPTIONS = {
    1: "ILLEGAL FUNCTION", 2: "ILLEGAL DATA ADDRESS", 3: "ILLEGAL DATA VALUE",
    4: "SLAVE DEVICE FAILURE", 5: "ACKNOWLEDGE", 6: "SLAVE DEVICE BUSY",
    8: "MEMORY PARITY ERROR", 10: "GATEWAY PATH UNAVAILABLE",
    11: "GATEWAY TARGET FAILED TO RESPOND",
}


def crc16(data: bytes) -> int:
    """Standard Modbus CRC-16 (poly 0xA001), returned host-order."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _frame(body: bytes) -> bytes:
    return body + struct.pack("<H", crc16(body))


def _check(frame: bytes) -> bytes:
    if len(frame) < 4:
        raise TransportError(f"runt frame: {frame.hex(' ')}")
    if crc16(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
        raise TransportError(f"CRC error: {frame.hex(' ')}")
    return frame[:-2]


# ── word/byte order ────────────────────────────────────────────────────────
# Alicat puts the most significant register first ("big endian") and the most
# significant byte first inside each register. Some RS-485 gateways swap one or
# both, which is why the manual publishes a fixed test value (registers
# 1088-1089 read back as 1.234567). ModbusMaster.detect_order() uses it.
ORDERS = ("big", "word_swap", "byte_swap", "both_swap")


def regs_to_bytes(regs: Sequence[int], order: str = "big") -> bytes:
    raw = b"".join(struct.pack(">H", r & 0xFFFF) for r in regs)
    if order in ("byte_swap", "both_swap"):
        raw = b"".join(raw[i:i + 2][::-1] for i in range(0, len(raw), 2))
    if order in ("word_swap", "both_swap"):
        raw = b"".join(raw[i:i + 2] for i in range(len(raw) - 2, -1, -2))
    return raw


def bytes_to_regs(raw: bytes, order: str = "big") -> list[int]:
    if order in ("word_swap", "both_swap"):
        raw = b"".join(raw[i:i + 2] for i in range(len(raw) - 2, -1, -2))
    if order in ("byte_swap", "both_swap"):
        raw = b"".join(raw[i:i + 2][::-1] for i in range(0, len(raw), 2))
    return [struct.unpack(">H", raw[i:i + 2])[0] for i in range(0, len(raw), 2)]


def decode_float(regs: Sequence[int], order: str = "big") -> float:
    return struct.unpack(">f", regs_to_bytes(regs[:2], order))[0]


def encode_float(value: float, order: str = "big") -> list[int]:
    return bytes_to_regs(struct.pack(">f", float(value)), order)


def decode_int32(regs: Sequence[int], order: str = "big", signed: bool = True) -> int:
    return struct.unpack(">i" if signed else ">I", regs_to_bytes(regs[:2], order))[0]


class ModbusMaster:
    """Modbus RTU transactions against one slave address on a shared bus."""

    def __init__(self, bus: SerialBus, slave: int, order: str = "big"):
        self.bus = bus
        self.slave = int(slave)
        self.order = order
        self.last_exchange = ("", "")   # (request hex, response hex) for the log

    # -- primitives ---------------------------------------------------------
    def _txn(self, body: bytes, expect_data_len: int) -> bytes:
        """expect_data_len = payload bytes after [addr][fc], excluding CRC."""
        request = _frame(bytes([self.slave]) + body)

        def reader(port):
            deadline = time.monotonic() + max(self.bus.cfg.timeout, 0.05) * 4
            head = self.bus.read_exact(port, 2, deadline)
            if head[0] != self.slave:
                raise TransportError(
                    f"reply from slave {head[0]}, expected {self.slave}")
            if head[1] & 0x80:
                rest = self.bus.read_exact(port, 3, deadline)
                _check(head + rest)
                code = rest[0]
                raise TransportError(
                    f"modbus exception {code} ({EXCEPTIONS.get(code, 'unknown')})")
            if head[1] != body[0]:
                raise TransportError(f"function {head[1]} echoed, sent {body[0]}")
            rest = self.bus.read_exact(port, expect_data_len + 2, deadline)
            self.last_exchange = (request.hex(" "), (head + rest).hex(" "))
            return _check(head + rest)[2:]

        return self.bus.transaction(request, reader)

    def read_registers(self, number: int, count: int, fc: int = READ_HOLDING) -> list[int]:
        """Read `count` 16-bit registers starting at register *number*."""
        address = number - 1
        body = struct.pack(">BHH", fc, address, count)
        data = self._txn(body, 1 + 2 * count)
        if data[0] != 2 * count:
            raise TransportError(f"byte count {data[0]}, expected {2 * count}")
        return [struct.unpack(">H", data[1 + 2 * i:3 + 2 * i])[0] for i in range(count)]

    def write_registers(self, number: int, values: Sequence[int]) -> None:
        address = number - 1
        payload = b"".join(struct.pack(">H", v & 0xFFFF) for v in values)
        body = struct.pack(">BHHB", WRITE_MULTIPLE, address, len(values), len(payload)) + payload
        echo = self._txn(body, 4)
        start, qty = struct.unpack(">HH", echo)
        if start != address or qty != len(values):
            raise TransportError(f"write echo mismatch: {start}/{qty}")

    # -- typed helpers ------------------------------------------------------
    def read_float(self, number: int, fc: int = READ_HOLDING) -> float:
        return decode_float(self.read_registers(number, 2, fc), self.order)

    def write_float(self, number: int, value: float) -> None:
        self.write_registers(number, encode_float(value, self.order))

    def detect_order(self, number: int = 1088) -> str:
        """Find the word/byte order using Alicat's fixed 1.234567 test register.

        Returns one of ORDERS, and sets self.order. Raises if none matches,
        which almost always means the register pair was read at the wrong
        offset rather than that the ordering is exotic.
        """
        regs = self.read_registers(number, 2)
        for order in ORDERS:
            if abs(decode_float(regs, order) - 1.234567) < 1e-5:
                self.order = order
                return order
        raise TransportError(
            f"no word order matches the 1.234567 test value (got {regs}); "
            "firmware older than 10v19.0 does not implement it")
