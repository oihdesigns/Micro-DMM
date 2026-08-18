"""bus.py — one physical link, shared by every device hanging off it.

An RS-485 multidrop line carries up to 26 Alicats (or a Modbus segment of
them) on a single COM port, and exactly one transaction may be in flight at a
time. SerialBus owns the port and the lock; drivers never touch pyserial.

The same abstraction covers RS-232 (one device per port) — it is then simply a
bus with a single member.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import serial  # pyserial
    import serial.tools.list_ports
except ImportError:  # keep the library importable for simulated runs
    serial = None

from .core import TransportError


@dataclass
class BusConfig:
    id: str = "bus1"
    port: str = "COM1"
    baud: int = 19200         # Alicat factory default, RS-232 and RS-485 alike
    bytesize: int = 8
    parity: str = "N"         # Alicat ships no-parity even for Modbus RTU
    stopbits: float = 1
    timeout: float = 0.25     # per-read timeout, seconds
    protocol: str = "modbus"  # "modbus" | "ascii" | "sim"
    retries: int = 2          # retries per transaction before giving up
    inter_frame: float = 0.005  # quiet time between transactions (s)


def list_ports() -> list[str]:
    if serial is None:
        return []
    return [p.device for p in serial.tools.list_ports.comports()]


class SerialBus:
    """Exclusive, retrying access to one serial port."""

    def __init__(self, cfg: BusConfig):
        self.cfg = cfg
        self._lock = threading.RLock()
        self._port: Optional["serial.Serial"] = None
        self._last_tx = 0.0
        self.stats = {"tx": 0, "rx": 0, "timeouts": 0, "errors": 0, "retries": 0}

    # -- lifecycle ----------------------------------------------------------
    @property
    def is_open(self) -> bool:
        return self._port is not None and self._port.is_open

    def open(self) -> None:
        if serial is None:
            raise TransportError("pyserial is not installed (pip install pyserial)")
        with self._lock:
            if self.is_open:
                return
            c = self.cfg
            try:
                self._port = serial.Serial(
                    port=c.port, baudrate=c.baud, bytesize=c.bytesize,
                    parity=c.parity, stopbits=c.stopbits, timeout=c.timeout,
                    write_timeout=1.0,
                )
            except Exception as exc:  # pyserial raises several unrelated types
                raise TransportError(f"cannot open {c.port}: {exc}") from exc
            time.sleep(0.05)
            self._port.reset_input_buffer()

    def close(self) -> None:
        with self._lock:
            if self._port is not None:
                try:
                    self._port.close()
                finally:
                    self._port = None

    # -- transactions -------------------------------------------------------
    def transaction(self, payload: bytes, reader, *, retries: Optional[int] = None):
        """Write `payload`, then call reader(port) to collect the reply.

        `reader` returns the parsed reply or raises TransportError to trigger a
        retry. Holding the lock across write+read is the whole point: on a
        shared 485 line an interleaved transaction corrupts both.
        """
        if not self.is_open:
            self.open()
        tries = self.cfg.retries if retries is None else retries
        last: Optional[BaseException] = None
        with self._lock:
            for attempt in range(tries + 1):
                if attempt:
                    self.stats["retries"] += 1
                try:
                    gap = self.cfg.inter_frame - (time.monotonic() - self._last_tx)
                    if gap > 0:
                        time.sleep(gap)
                    self._port.reset_input_buffer()
                    self._port.write(payload)
                    self._port.flush()
                    self.stats["tx"] += 1
                    result = reader(self._port)
                    self.stats["rx"] += 1
                    self._last_tx = time.monotonic()
                    return result
                except TransportError as exc:
                    last = exc
                    self.stats["timeouts" if "timeout" in str(exc).lower() else "errors"] += 1
                    self._last_tx = time.monotonic()
                except Exception as exc:
                    last = TransportError(str(exc))
                    self.stats["errors"] += 1
                    self._last_tx = time.monotonic()
        raise last if last else TransportError("transaction failed")

    def read_exact(self, port, n: int, deadline: float) -> bytes:
        """Read exactly n bytes or raise; `deadline` is a monotonic timestamp."""
        buf = bytearray()
        while len(buf) < n:
            chunk = port.read(n - len(buf))
            if chunk:
                buf += chunk
            elif time.monotonic() > deadline:
                raise TransportError(
                    f"timeout after {len(buf)}/{n} bytes: {bytes(buf).hex(' ') or '(nothing)'}")
        return bytes(buf)

    def read_line(self, port, deadline: float, terminator: bytes = b"\r") -> bytes:
        buf = bytearray()
        while True:
            ch = port.read(1)
            if ch:
                if ch in (b"\r", b"\n"):
                    if buf:
                        return bytes(buf)
                    continue  # swallow leading CR/LF left over from a prior frame
                buf += ch
            elif time.monotonic() > deadline:
                raise TransportError(
                    f"timeout waiting for reply: {bytes(buf).decode('ascii', 'replace')!r}")
