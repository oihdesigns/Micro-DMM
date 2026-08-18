"""config.py — JSON rig description, and the factory that builds it.

A rig file is the whole system on disk: which ports, which protocol, which
devices at which addresses, how fast to poll, what each device's columns mean.
The GUI edits it; a headless controller loads the same file, so the bench and
the production script cannot drift apart.

    {
      "buses": [
        {"id": "bench", "port": "COM5", "baud": 19200, "protocol": "modbus"},
        {"id": "aux",   "port": "COM7", "baud": 19200, "protocol": "ascii"}
      ],
      "devices": [
        {"name": "MFC-N2", "bus": "bench", "address": "1",
         "model": "MC-500SCCM-D", "units": "SCCM", "full_scale": 500,
         "poll_hz": 2.0},
        {"name": "MFC-Ar",  "bus": "aux", "address": "A",
         "model": "MC-20SLPM", "units": "SLPM", "full_scale": 20}
      ]
    }
"""

from __future__ import annotations

import json
from typing import Any, Optional

from .alicat_ascii import AlicatAscii
from .alicat_modbus import AlicatModbus
from .bus import BusConfig
from .core import Device, DeviceInfo
from .manager import DeviceManager
from .simulator import SimulatedController

DEFAULT_RIG: dict[str, Any] = {
    "buses": [{"id": "sim", "protocol": "sim"}],
    "devices": [
        {"name": "MFC-N2", "bus": "sim", "address": "sim", "model": "MC-500SCCM-D",
         "units": "SCCM", "full_scale": 500.0, "poll_hz": 4.0},
        {"name": "MFC-Ar", "bus": "sim", "address": "sim", "model": "MC-50SCCM",
         "units": "SCCM", "full_scale": 50.0, "poll_hz": 4.0, "tau": 0.6},
    ],
}


def load_rig(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        rig = json.load(handle)
    if "buses" not in rig or "devices" not in rig:
        raise ValueError(f"{path}: rig file needs 'buses' and 'devices' lists")
    return rig


def save_rig(path: str, rig: dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(rig, handle, indent=2)
        handle.write("\n")


def bus_config(entry: dict[str, Any]) -> BusConfig:
    cfg = BusConfig(id=entry.get("id", "bus1"),
                    protocol=entry.get("protocol", "modbus"))
    for field_name in ("port", "baud", "bytesize", "parity", "stopbits",
                       "timeout", "retries", "inter_frame"):
        if field_name in entry:
            setattr(cfg, field_name, entry[field_name])
    return cfg


def build_device(entry: dict[str, Any], protocol: str, manager: DeviceManager) -> Device:
    """Create one driver instance from its rig-file entry."""
    info = DeviceInfo(
        name=entry["name"],
        model=entry.get("model", ""),
        units=entry.get("units", ""),
        full_scale=float(entry.get("full_scale", 100.0)),
        address=str(entry.get("address", "")),
        is_controller=bool(entry.get("is_controller", True)),
    )
    readings = entry.get("readings")

    if protocol == "sim":
        return SimulatedController(info, tau=float(entry.get("tau", 0.35)))
    bus = manager.buses[entry["bus"]]
    if protocol == "modbus":
        return AlicatModbus(info, bus, slave=int(entry.get("address", 1)),
                            readings=readings,
                            block=entry.get("block", "optimized"),
                            word_order=entry.get("word_order", "big"))
    if protocol == "ascii":
        return AlicatAscii(info, bus, unit_id=str(entry.get("address", "A")),
                           readings=readings)
    raise ValueError(f"unknown protocol {protocol!r} for device {info.name!r}")


def build_manager(rig: dict[str, Any], manager: Optional[DeviceManager] = None
                  ) -> DeviceManager:
    """Turn a rig dict into a running DeviceManager (buses started, devices
    queued for connect). Devices whose bus is missing are reported, not fatal —
    a rig with one unplugged port should still bring up everything else."""
    manager = manager or DeviceManager()
    protocols: dict[str, str] = {}
    for entry in rig.get("buses", []):
        cfg = bus_config(entry)
        protocols[cfg.id] = cfg.protocol
        manager.add_bus(cfg)
    for entry in rig.get("devices", []):
        bus_key = entry.get("bus")
        if bus_key not in protocols:
            manager.emit("error", f"device {entry.get('name')!r}: no bus {bus_key!r}")
            continue
        try:
            device = build_device(entry, protocols[bus_key], manager)
            manager.add_device(device, bus_key, float(entry.get("poll_hz", 2.0)))
        except Exception as exc:      # noqa: BLE001
            manager.emit("error", f"device {entry.get('name')!r}: {exc}")
    return manager


def rig_from_manager(manager: DeviceManager) -> dict[str, Any]:
    """Serialize the live system back out, so 'Save rig' round-trips."""
    buses = []
    for key, worker in manager.workers.items():
        if worker.bus is None:
            buses.append({"id": key, "protocol": "sim"})
        else:
            cfg = worker.bus.cfg
            buses.append({"id": key, "protocol": cfg.protocol, "port": cfg.port,
                          "baud": cfg.baud, "timeout": cfg.timeout,
                          "retries": cfg.retries})
    devices = []
    for name, entry in manager.entries.items():
        info = entry.device.info
        item = {"name": name, "bus": entry.bus_key, "address": info.address,
                "model": info.model, "units": info.units,
                "full_scale": info.full_scale,
                "poll_hz": round(1.0 / entry.period, 3),
                "is_controller": info.is_controller}
        readings = getattr(entry.device, "readings", None)
        if readings:
            item["readings"] = list(readings)
        devices.append(item)
    return {"buses": buses, "devices": devices}
