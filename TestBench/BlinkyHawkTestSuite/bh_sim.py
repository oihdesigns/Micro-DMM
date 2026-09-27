"""
bh_sim.py  --  a simulated bench, for dry-running the suite with nothing plugged in.

SimBench is the shared electrical state (what the relay jig has selected, what
the generator is putting out).  SimBlinkyHawk stands in for the GUI's
SerialManager: it answers the subset of the bench firmware's protocol the suite
uses (!SET/!GET/!CFG/!STATUS/!DETLOG/!VMODE/...) and emits $DET lines with the
firmware's own decision logic (voltagePresent, the threshold compare, the
STABLECOUNT debounce) applied to a made-up front end.

THE NUMBERS IN THE MODEL ARE INVENTED.  They are shaped like the real thing
(asymmetric gain, hum on an open lead, a metric that rises with resistance)
only so every code path -- including the "cannot be separated" ones -- gets
exercised.  Nothing measured from the simulator means anything about hardware.
"""

import math
import random
import threading
import time

from bh_instruments import JIG_LOAD_OHMS


class SimBench:
    def __init__(self):
        self.relay = "FGEN"
        self.fgen = None          # None = output off, ("dc", V, 0) or ("sine", Vrms, Hz)


SIM_DEFAULTS = {
    "HWREV": 3, "REFCENTER": 0.018, "REFBAND": 0.025, "THRESH00": 0.15, "THRESH01": 0.54,
    "THRESH10": 0.45, "THRESH11": 1.0, "THRESHSEL": 3, "VOLTFAST": 5.0, "VOLTAVG": 10,
    "TESTAGREE": 1, "STABLECOUNT": 2, "SETTLEPREUS": 300, "SETTLEPOSTMS": 3, "NEGFIX": 1,
    "NEGV": 1.25, "DETMETHOD": 1, "DETBAND": 0.05, "DETWINUS": 1500, "DETAREAUS": 400,
    "LOOPMS": 50, "CHGINHIBIT": 0,
}


class SimBlinkyHawk:
    """Drop-in for blinkyhawk_bench_gui.SerialManager."""

    # front-end model (invented, see module docstring)
    C0 = -0.020            # resting differential with nothing connected
    GAIN_POS = 0.300       # V of differential per V at the leads, + side
    GAIN_NEG = 0.270       # - side (deliberately asymmetric)
    NOISE = 0.002

    def __init__(self, line_queue, bench):
        self.line_queue = line_queue
        self.bench = bench
        self.cfg = dict(SIM_DEFAULTS)
        self.saved = dict(self.cfg)
        self.sn = "SIM-001"
        self.detlog = False
        self.vmode = 0
        self.lead = "V"
        self._cand, self._cnt = "V", 0
        self._stop = threading.Event()
        self._thread = None
        self._rx = []
        self._lock = threading.Lock()
        self.ser = None

    # --- SerialManager interface ---
    def connect(self, port=None):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.ser = True

    def disconnect(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)
        self.ser = None

    @property
    def is_open(self):
        return self.ser is not None

    def send(self, text):
        with self._lock:
            self._rx.append(text.strip())

    # --- model ---
    def _out(self, line):
        self.line_queue.put(("line", line))

    def _vin(self, t):
        f = self.bench.fgen if self.bench.relay == "FGEN" else None
        if f is None:
            return 0.0
        if f[0] == "dc":
            return f[1]
        return f[1] * math.sqrt(2) * math.sin(2 * math.pi * f[2] * t)

    def _load(self):
        """Resistance across the leads (None = open)."""
        if self.bench.relay == "FGEN":
            return None if self.bench.fgen is None else 50.0   # a source looks like 50R
        return JIG_LOAD_OHMS.get(self.bench.relay)

    def _rest_read(self, t):
        v = self._vin(t)
        g = self.GAIN_POS if v >= 0 else self.GAIN_NEG
        d = self.C0 + g * v
        r = self._load()
        noise = self.NOISE
        if r is None:
            noise += 0.004                    # open lead picks up hum
            d += 0.006 * math.sin(2 * math.pi * 60 * t)
        elif r > 5e6:
            d += 0.004
        d += random.gauss(0, noise)
        return max(-1.2, min(2.0, d))

    def _metrics(self):
        r = self._load()
        rr = 1e12 if r is None else r
        off = 0.3 * abs(self.cfg["REFCENTER"] - self.C0)
        area = 0.9 * rr / (rr + 400e3) + off + random.gauss(0, 0.012)
        band = self.cfg["DETBAND"]
        ret = min(self.cfg["DETWINUS"] / 1000.0,
                  0.25 + 1.1 * rr / (rr + 300e3) * (0.05 / band) ** 0.3 + random.gauss(0, 0.03))
        m0 = 0.05 + 0.6 * rr / (rr + 2e6) + random.gauss(0, 0.05)
        return max(0.0, area), max(0.0, ret), abs(m0)

    def _thr(self):
        return self.cfg["THRESH" + ["00", "01", "10", "11"][int(self.cfg["THRESHSEL"])]]

    def _pass(self):
        c = self.cfg
        t0 = time.time()
        reads = [self._rest_read(t0 + i * 60e-6) for i in range(int(c["VOLTAVG"]))]
        n = len(reads)
        mean, lo, hi = sum(reads) / n, min(reads), max(reads)
        path = "-"
        if self.vmode == 1:
            present, path, n = True, "L", 0
        elif self.vmode == 2:
            present, path, n = False, "D", 0
        else:
            fast = any(abs(v - c["REFCENTER"]) > c["VOLTFAST"] * c["REFBAND"] for v in reads)
            if fast:
                present, path = True, "F"
            else:
                present = abs(mean - c["REFCENTER"]) > c["REFBAND"]
                path = "A" if present else "-"
        metric = ret = area = None
        if present:
            raw = "V"
        else:
            area, ret, m0 = self._metrics()
            metric = {0: m0, 1: ret, 2: area}[int(c["DETMETHOD"])]
            raw = "F" if metric > self._thr() else "C"
        if raw == self.lead:
            self._cand, self._cnt = raw, 0
        else:
            if raw != self._cand:
                self._cand, self._cnt = raw, 0
            self._cnt += 1
            if self._cnt >= c["STABLECOUNT"]:
                self.lead, self._cnt = raw, 0
        if self.detlog:
            ms = int(time.time() * 1000) & 0x7FFFFFFF
            rest = f"{mean:.5f},{lo:.5f},{hi:.5f}" if n else ",,"
            met = f"{metric:.5f},{ret:.4f},{area:.5f}" if metric is not None else ",,"
            self._out(f"$DET,{ms},{raw},{self.lead},{path},{n},{rest},{met},{self._thr():.5f}")

    def _status(self):
        self._out(f"$STATUS,diag=0,hwrev=3,vmode={self.vmode},mosfet=-1,stream=0,rate=20,"
                  f"capms=5,dip={int(self.cfg['THRESHSEL'])},openthr={self._thr():.3f},"
                  f"detmethod={int(self.cfg['DETMETHOD'])},charge=1,chginhibit=0,"
                  f"dirty={int(self.cfg != self.saved)},lpstage=0,gates=111,gforce=aaa,"
                  f"expt=-1,lead={self.lead},detlog={int(self.detlog)},battpct=100,"
                  f"battv=4.100,sn={self.sn}")

    def _cfg_line(self, k):
        v = self.cfg[k]
        self._out(f"$CFG,{k},{v:.4f}" if isinstance(v, float) else f"$CFG,{k},{v}")

    def _handle(self, line):
        if not line.startswith("!"):
            return
        parts = line[1:].split(",")
        cmd = parts[0].upper()
        arg = parts[1:]
        if cmd == "SET" and len(arg) >= 2:
            k = arg[0].upper()
            if k not in self.cfg:
                self._out(f"$ERR,unknown key,{k}")
                return
            v = float(arg[1])
            self.cfg[k] = v if isinstance(SIM_DEFAULTS[k], float) else int(round(v))
            self._cfg_line(k)
        elif cmd == "GET" and arg:
            if arg[0].upper() in self.cfg:
                self._cfg_line(arg[0].upper())
        elif cmd == "CFG":
            for k in self.cfg:
                self._cfg_line(k)
            self._out("$CFGEND")
        elif cmd == "SAVE":
            self.saved = dict(self.cfg)
            self._out("$OK,save")
        elif cmd == "LOAD":
            self.cfg = dict(self.saved)
            self._out("$OK,load")
        elif cmd == "DEFAULTS":
            self.cfg = dict(SIM_DEFAULTS)
            self._out("$OK,defaults")
        elif cmd == "DETLOG":
            self.detlog = (int(arg[0]) != 0) if arg else not self.detlog
            self._out(f"$OK,detlog,{int(self.detlog)}")
        elif cmd == "VMODE":
            self.vmode = max(0, min(2, int(arg[0]) if arg else 0))
            self._status()
        elif cmd in ("STATUS", "?", "DIAG", "MOSFET", "STREAM", "RATE"):
            self._status()
        elif cmd == "SN":
            self._out(f"$SN,{self.sn}")
        elif cmd == "PINS":
            self._out("$PINEND")
        elif cmd in ("GATE", "DEEP"):
            pass
        else:
            self._out(f"$ERR,unknown,{cmd}")

    def _run(self):
        self._out("Blinky Hawk BENCH (SIMULATED)")
        next_t = time.time()
        while not self._stop.is_set():
            with self._lock:
                rx, self._rx = self._rx, []
            for line in rx:
                self._handle(line)
            now = time.time()
            if now >= next_t:
                self._pass()
                next_t = now + self.cfg["LOOPMS"] / 1000.0 + 0.002
            time.sleep(0.003)
