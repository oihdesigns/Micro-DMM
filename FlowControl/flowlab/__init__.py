"""flowlab — a small instrument-control library, built around Alicat flow
controllers but not limited to them.

Layers, bottom to top:

    bus.py            one serial port, one lock, retries
    modbus.py         Modbus RTU master (no third-party dependency)
    alicat_modbus.py  \\ two drivers for the same instrument family,
    alicat_ascii.py   / behind one interface
    simulator.py      a fake controller so everything above runs hardware-free
    core.py           Device / Snapshot / DeviceInfo — the interface itself
    manager.py        one worker thread per bus: polling, jobs, history, CSV
    sequence.py       time-based setpoint programs
    config.py         JSON rig files -> a live DeviceManager
    discover.py       read-only bus scans

Adding an instrument means writing a Device subclass; nothing above core.py
needs to know it exists.
"""

from .core import (Device, DeviceError, DeviceInfo, Snapshot, TransportError,
                   CommandRejected, READING_LABELS)
from .bus import BusConfig, SerialBus, list_ports
from .manager import DeviceManager
from .sequence import Sequence, SequenceRunner, Step
from .config import build_manager, load_rig, save_rig, rig_from_manager, DEFAULT_RIG

__all__ = [
    "Device", "DeviceError", "DeviceInfo", "Snapshot", "TransportError",
    "CommandRejected", "READING_LABELS", "BusConfig", "SerialBus", "list_ports",
    "DeviceManager", "Sequence", "SequenceRunner", "Step", "build_manager",
    "load_rig", "save_rig", "rig_from_manager", "DEFAULT_RIG",
]

__version__ = "0.1.0"
