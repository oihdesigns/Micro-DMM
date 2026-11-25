"""
cbt_bench.py -- PSU (Siglent SPD4323X, CH2+CH3 parallel) + load (Rigol DL3021A)
wired across a 12 V lead-acid battery.

Interlock: the supply output and the load input are never on together.
Every on-switch first turns the other one off and reads it back.
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "BlinkyHawkTestSuite")))
from bh_instruments import ScpiSocket  # noqa: E402

PSU_ADDR = ("192.168.10.6", 5025)
LOAD_ADDR = ("192.168.10.4", 5555)
CH = 2          # master channel in parallel (OUTP:TRACK 2) mode


class Bench:
    def __init__(self):
        self.psu = ScpiSocket()
        self.psu.open(*PSU_ADDR, timeout=5)
        self.psu_id = self.psu.identify("PSU")
        self.load = ScpiSocket()
        self.load.open(*LOAD_ADDR, timeout=5)
        self.load_id = self.load.identify("load")
        if self.psu.query("OUTP:TRACK?").strip() != "2":
            raise RuntimeError("PSU is not in parallel (CH2+CH3) mode")

    # ---- reads ------------------------------------------------------------
    def psu_on(self):
        return self.psu.query(f"OUTP? CH{CH}").strip() in ("1", "ON")

    def load_on(self):
        return self.load.query(":SOUR:INP:STAT?").strip() in ("1", "ON")

    def read(self):
        """One sample of everything.  v_psu is the PSU's own sense of the
        battery; when the supply is off no current flows in its leads, so it
        is a Kelvin-style reading of the battery terminals."""
        p = self.psu
        d = {"t": time.time(),
             "v_psu": float(p.query(f"MEAS:VOLT? CH{CH}")),
             "i_ch2": float(p.query("MEAS:CURR? CH2")),
             "i_ch3": float(p.query("MEAS:CURR? CH3")),
             "v_load": float(self.load.query(":MEAS:VOLT?")),
             "i_load": float(self.load.query(":MEAS:CURR?"))}
        return d

    def fast_v(self):
        return float(self.psu.query(f"MEAS:VOLT? CH{CH}"))

    # ---- supply -----------------------------------------------------------
    def psu_set(self, volts, amps):
        for what, val in (("VOLT", volts), ("CURR", amps)):
            self.psu.write(f"{what} CH{CH},{val:.4f}")
            got = float(self.psu.query(f"{what}? CH{CH}"))
            if abs(got - val) > max(0.002, 0.002 * val):
                raise RuntimeError(f"PSU CH{CH} {what} kept {got} instead of {val}")

    def psu_output(self, on):
        if on:
            self.load_input(False)
            if self.load_on():
                raise RuntimeError("load would not switch off")
        self.psu.write(f"OUTP CH{CH},{'ON' if on else 'OFF'}")
        time.sleep(0.3)
        if self.psu_on() != on:
            raise RuntimeError(f"PSU output did not switch {'on' if on else 'off'}")

    # ---- load -------------------------------------------------------------
    def load_setup_cc(self, amps, rng, von):
        if self.load_on():
            self.load_input(False)
        self.load.write(":SOUR:FUNC CURR")
        self.load.write(f":SOUR:CURR:RANG {rng:g}")      # resets the CC level -> set range first
        self.load.write(f":SOUR:CURR:VON {von:g}")
        self.load.write(f":SOUR:CURR:LEV:IMM {amps:g}")
        got = float(self.load.query(":SOUR:CURR:LEV:IMM?"))
        if abs(got - amps) > 0.01:
            raise RuntimeError(f"load CC level kept {got} instead of {amps}")

    def load_level(self, amps):
        self.load.write(f":SOUR:CURR:LEV:IMM {amps:g}")

    def load_input(self, on):
        if on:
            self.psu.write(f"OUTP CH{CH},OFF")
            time.sleep(0.3)
            if self.psu_on():
                raise RuntimeError("PSU output would not switch off")
        self.load.write(f":SOUR:INP:STAT {'ON' if on else 'OFF'}")
        time.sleep(0.1)

    def all_off(self):
        for f in (lambda: self.load.write(":SOUR:INP:STAT OFF"),
                  lambda: self.psu.write(f"OUTP CH{CH},OFF")):
            try:
                f()
            except Exception:
                pass

    def close(self):
        self.psu.close()
        self.load.close()


def all_off_fresh():
    """Last-ditch: new sockets, switch both off.  Used after a comms failure."""
    for addr, cmd in ((LOAD_ADDR, ":SOUR:INP:STAT OFF"), (PSU_ADDR, f"OUTP CH{CH},OFF")):
        try:
            s = ScpiSocket()
            s.open(*addr, timeout=3)
            s.write(cmd)
            s.close()
        except Exception:
            pass
