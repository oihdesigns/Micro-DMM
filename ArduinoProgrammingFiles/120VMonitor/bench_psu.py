"""
bench_psu.py  --  the bench power supply (Siglent SPD4323X) for calibration.

Two ways in, same interface:

  VirtualBenchPSU   through a running Virtual Bench (TestBench/VirtualBench,
                    http://127.0.0.1:8765).  The bench holds the supply's one
                    SCPI session while it runs, so this is the only way to
                    reach the supply without disconnecting it there first.
  DirectPSU         straight to the supply over SCPI, reusing the Virtual
                    Bench driver (vb_drivers.SiglentSPD4000) without its
                    poller -- channel-first set syntax, every set read back.

open_psu("auto") tries the Virtual Bench first and falls back to direct.

Only the one channel named by the caller is ever changed (2026-10-10 lesson:
a probe once switched CH3 off).  Tracking must be Independent, or CH2's
setpoints would be shared with CH3.
"""

import json
import os
import sys
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
VB_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "TestBench", "VirtualBench"))

PSU_HOST = "192.168.10.6"
PSU_PORT = 5025
VB_URL = "http://127.0.0.1:8765"


class PSUError(Exception):
    pass


class BenchPSU:
    name = "?"

    def ident(self):
        raise NotImplementedError

    def query(self, cmd):
        raise NotImplementedError

    def set(self, ch, volts=None, amps=None):
        raise NotImplementedError

    def output(self, ch, on):
        raise NotImplementedError

    def close(self):
        pass

    # ── shared on top of query() ────────────────────────────────────────────
    def qf(self, cmd):
        try:
            return float(self.query(cmd))
        except (TypeError, ValueError) as e:
            raise PSUError(f"bad reply to {cmd}") from e

    def meas_v(self, ch):
        return self.qf(f"MEAS:VOLT? CH{ch}")

    def meas_i(self, ch):
        return self.qf(f"MEAS:CURR? CH{ch}")

    def setpoints(self, ch):
        return self.qf(f"VOLT? CH{ch}"), self.qf(f"CURR? CH{ch}")

    def is_on(self, ch):
        return self.query(f"OUTP? CH{ch}").strip().upper() in ("1", "ON")

    def check_channel(self, ch):
        """Refuse to run unless the channel is independent of its neighbour."""
        track = self.query("OUTP:TRACK?").strip()
        if ch in (2, 3) and track not in ("0", "0.0"):
            raise PSUError("the supply is in series/parallel tracking mode -- set it to "
                           "Independent first (CH2 and CH3 would share setpoints)")
        return {"on": self.is_on(ch), "vset": self.setpoints(ch)[0], "iset": self.setpoints(ch)[1]}


class DirectPSU(BenchPSU):
    name = "direct SCPI"

    def __init__(self, host=PSU_HOST, port=PSU_PORT):
        if VB_DIR not in sys.path:
            sys.path.insert(0, VB_DIR)
        try:
            import vb_drivers
        except ImportError as e:
            raise PSUError(f"cannot import the Virtual Bench driver from {VB_DIR}: {e}") from e
        self.inst = vb_drivers.SiglentSPD4000("psu", "Power supply", host, port)
        try:
            self.inst.connect()
        except Exception as e:                             # noqa: BLE001
            raise PSUError(f"supply at {host}:{port}: {vb_drivers._msg(e)}") from e
        self.name = f"direct SCPI {host}:{port}"

    def ident(self):
        return self.inst.ident

    def query(self, cmd):
        with self.inst.lock:
            return self.inst.q(cmd)

    def set(self, ch, volts=None, amps=None):
        self.inst.act("set", {"ch": ch, "volts": volts, "amps": amps})

    def output(self, ch, on):
        self.inst.act("output", {"ch": ch, "on": bool(on)})

    def close(self):
        try:
            self.inst.disconnect()
        except Exception:                                  # noqa: BLE001
            pass


class VirtualBenchPSU(BenchPSU):
    def __init__(self, url=VB_URL):
        self.url = url.rstrip("/")
        snap = self._get("/api/inst/psu")
        if not snap.get("connected"):
            raise PSUError("the Virtual Bench is running but its power supply is not connected")
        self._ident = snap.get("ident", "")
        self.name = f"Virtual Bench {self.url}"

    def _call(self, method, path, body=None, timeout=8.0):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.url + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                out = json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            try:
                out = json.loads(e.read() or b"{}")
            except ValueError:
                out = {"ok": False, "error": str(e)}
        except (urllib.error.URLError, OSError) as e:
            raise PSUError(f"Virtual Bench not reachable at {self.url}: {e}") from e
        if out.get("ok") is False:
            raise PSUError(out.get("error") or "Virtual Bench refused the request")
        return out

    def _get(self, path):
        return self._call("GET", path)

    def ident(self):
        return self._ident

    def query(self, cmd):
        return self._call("POST", "/api/inst/psu/scpi", {"cmd": cmd}).get("reply", "")

    def set(self, ch, volts=None, amps=None):
        self._call("POST", "/api/inst/psu/act",
                   {"action": "set", "args": {"ch": ch, "volts": volts, "amps": amps}})

    def output(self, ch, on):
        self._call("POST", "/api/inst/psu/act",
                   {"action": "output", "args": {"ch": ch, "on": bool(on)}})


def open_psu(mode="auto", host=PSU_HOST, port=PSU_PORT, url=VB_URL):
    """mode: auto | bench | direct | sim (sim needs monitor_sim's device)."""
    if mode in ("auto", "bench"):
        try:
            return VirtualBenchPSU(url)
        except PSUError:
            if mode == "bench":
                raise
    if mode == "sim":
        import monitor_sim
        return monitor_sim.SimPSU()
    return DirectPSU(host, port)
