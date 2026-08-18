#!/usr/bin/env python3
"""scan_bus.py — first contact with a port, before the GUI is involved.

    python scan_bus.py                          list serial ports
    python scan_bus.py COM5                     scan COM5 for Modbus and ASCII
    python scan_bus.py COM5 --modbus 1 32       Modbus slaves 1..32 only
    python scan_bus.py COM5 --ascii             ASCII unit IDs A..Z only
    python scan_bus.py COM5 --baud 38400
    python scan_bus.py COM5 --watch 1           poll Modbus slave 1 forever
    python scan_bus.py COM5 --watch A --ascii   poll ASCII unit A forever

Everything here is read-only except --watch, which also only reads. Nothing
writes a setpoint or a register.
"""

from __future__ import annotations

import argparse
import sys
import time

from flowlab import discover
from flowlab.alicat_ascii import AlicatAscii
from flowlab.alicat_modbus import AlicatModbus
from flowlab.bus import BusConfig, SerialBus, list_ports
from flowlab.core import DeviceError, DeviceInfo


def watch(bus: SerialBus, address: str, ascii_mode: bool) -> int:
    info = DeviceInfo(name=f"unit {address}")
    device = (AlicatAscii(info, bus, unit_id=address) if ascii_mode
              else AlicatModbus(info, bus, slave=int(address)))
    try:
        device.connect()
    except DeviceError as exc:
        print(f"connect failed: {exc}")
        return 1
    print(f"connected: {info.model or 'Alicat'} firmware {info.firmware or '?'} "
          f"serial {info.serial or '?'}")
    print("Ctrl-C to stop\n")
    try:
        while True:
            snap = device.poll()
            fields = "  ".join(f"{k}={v:.4g}" for k, v in snap.values.items())
            flags = f"  [{' '.join(snap.flags)}]" if snap.flags else ""
            print(f"{time.strftime('%H:%M:%S')}  {fields}  {snap.gas or ''}{flags}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        return 0
    except DeviceError as exc:
        print(f"\npoll failed: {exc}")
        return 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", nargs="?", help="serial port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=19200)
    parser.add_argument("--modbus", nargs=2, type=int, metavar=("FIRST", "LAST"),
                        help="scan this Modbus slave range (default 1 16)")
    parser.add_argument("--ascii", action="store_true", help="ASCII protocol only")
    parser.add_argument("--watch", metavar="ADDRESS",
                        help="poll one device continuously instead of scanning")
    parser.add_argument("--timeout", type=float, default=0.15)
    args = parser.parse_args()

    if not args.port:
        ports = list_ports()
        print("serial ports:" if ports else "no serial ports found")
        for port in ports:
            print(f"  {port}")
        return 0

    cfg = BusConfig(id="scan", port=args.port, baud=args.baud,
                    timeout=args.timeout, retries=0,
                    protocol="ascii" if args.ascii else "modbus")
    bus = SerialBus(cfg)

    if args.watch:
        return watch(bus, args.watch, args.ascii)

    found = []
    if not args.ascii:
        first, last = args.modbus if args.modbus else (1, 16)
        print(f"scanning {args.port} @ {args.baud} for Modbus slaves {first}-{last}…")
        found += discover.scan_modbus(bus, first, last, args.timeout)
    if args.ascii or not args.modbus:
        print(f"scanning {args.port} @ {args.baud} for ASCII unit IDs A-Z…")
        found += discover.scan_ascii(bus, timeout=args.timeout)

    if not found:
        print("\nnothing answered. Check: baud (Alicat default 19200 8N1), "
              "A/B polarity on RS-485, termination, and that no unit is left "
              "in streaming mode (@).")
        return 1
    print()
    for item in found:
        print(f"  {item}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
