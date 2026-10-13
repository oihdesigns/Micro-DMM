"""
monitor_link.py  --  serial link to the 120VMonitor firmware.

One reader thread owns the port.  Every line goes to `events` (a queue the GUI
drains) except capture data, which is assembled into a Capture first.  Request
helpers (dc(), cfg(), capture() ...) register a one-shot waiter for the reply
prefix they expect, so they can be called from any worker thread.
"""

import queue
import threading
import time
from dataclasses import dataclass, field

import numpy as np

PWR_FIELDS = ("vrms", "irms", "p", "s", "q", "pf", "hz", "vpk", "ipk", "wh", "uptime",
              "n", "depth", "relay", "flags", "vdc", "idc")

FLAG_NAMES = {1: "NO V ZERO-X", 2: "V CLIP", 4: "I CLIP", 8: "TRIPPED",
              16: "TRUNCATED", 32: "LATE"}

SAFE_WHY = {0: "", 1: "instant limit", 2: "over-current (time delay)", 3: "auto-open timer",
            4: "capture complete", 5: "TRIPPK"}

TRIG_MODES = {0: "Immediate", 1: "|I| above level", 2: "V rising zero-cross",
              3: "Close relay (inrush)", 4: "|V| above level"}


def flag_text(flags):
    return "  ".join(name for bit, name in FLAG_NAMES.items() if flags & bit)


class LinkError(Exception):
    pass


@dataclass
class Capture:
    n: int
    ch: int                 # 0 V, 1 I, 2 both
    dt_us: float
    v_per: float
    v_off: float
    i_per: float
    i_off: float
    trig_idx: int
    flags: int
    relay_idx: int
    open_idx: int = -1      # sample where safe start opened the relay
    open_why: int = 0       # SAFE_WHY key
    avg: int = 1            # ADC scans averaged into each sample
    raw: list = field(default_factory=list)
    v: np.ndarray = None    # volts
    i: np.ndarray = None    # amps
    t_ms: np.ndarray = None

    def finish(self):
        a = np.array(self.raw, dtype=np.int16).astype(np.float64)
        if self.ch == 2:
            vc, ic = a[0::2], a[1::2]
        elif self.ch == 0:
            vc, ic = a, None
        else:
            vc, ic = None, a
        if vc is not None:
            self.v = (vc - self.v_off) * self.v_per
        if ic is not None:
            self.i = (ic - self.i_off) * self.i_per
        n = len(vc if vc is not None else ic)
        t0 = self.trig_idx if self.trig_idx >= 0 else 0
        self.t_ms = (np.arange(n) - t0) * self.dt_us / 1000.0
        self.n = n
        self.raw = []
        return self


class MonitorLink:
    def __init__(self, port, baud=115200):
        if port.upper() == "SIM":
            import monitor_sim
            self.ser = monitor_sim.SimSerial()
        else:
            import serial
            self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=1.0)
        self.port = port
        self.events = queue.Queue()
        self._waiters = {}            # prefix -> list of queues
        self._wlock = threading.Lock()
        self._txlock = threading.Lock()
        self._cap = None
        self._alive = True
        self.cfg = {}                 # key -> dict(value, lo, hi, help)
        self.status = {}
        self._rx = threading.Thread(target=self._reader, daemon=True, name="monitor-rx")
        self._rx.start()

    # ── plumbing ────────────────────────────────────────────────────────────
    def close(self):
        self._alive = False
        try:
            self.send("!STOP")
        except Exception:                                  # noqa: BLE001
            pass
        time.sleep(0.15)          # let the board finish its reply before the port goes
        try:
            self.ser.close()
        except Exception:                                  # noqa: BLE001
            pass

    def send(self, cmd):
        with self._txlock:
            self.ser.write((cmd + "\n").encode("ascii"))
        self.events.put(("tx", cmd))

    def _reader(self):
        buf = b""
        while self._alive:
            try:
                chunk = self.ser.read(4096)
            except Exception as e:                         # noqa: BLE001
                self.events.put(("error", f"serial read failed: {e}"))
                self._alive = False
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._line(line.decode("ascii", "replace").strip())

    def _line(self, line):
        if not line:
            return
        if line.startswith("$CD,"):
            if self._cap is not None:
                h = line[4:]
                self._cap.raw.extend(int(h[k:k + 4], 16) - (0x10000 if h[k] in "89ABCDEF" else 0)
                                     for k in range(0, len(h) - 3, 4))
            return
        if line.startswith("$CAP,"):
            p = line[5:].split(",")
            try:
                self._cap = Capture(int(p[0]), int(p[1]), float(p[2]), float(p[3]), float(p[4]),
                                    float(p[5]), float(p[6]), int(p[7]), int(p[8]), int(p[9]),
                                    int(p[10]) if len(p) > 10 else -1,
                                    int(p[11]) if len(p) > 11 else 0,
                                    int(p[12]) if len(p) > 12 else 1)
            except (ValueError, IndexError):
                self._cap = None
            self.events.put(("line", line))
            return
        if line.startswith("$CEND"):
            cap, self._cap = self._cap, None
            if cap is not None:
                cap.finish()
                self._deliver("$CEND", cap)
                self.events.put(("capture", cap))
            return
        if line.startswith("$CFG,"):
            p = line[5:].split(",", 4)
            if len(p) >= 4:
                try:
                    self.cfg[p[0]] = {"value": float(p[1]), "lo": float(p[2]), "hi": float(p[3]),
                                      "help": p[4] if len(p) > 4 else ""}
                except ValueError:
                    pass
        elif line.startswith("$STATUS,"):
            for kv in line[8:].split(","):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    self.status[k] = v
        elif line.startswith("$PWR,"):
            try:
                vals = [float(x) for x in line[5:].split(",")]
                rec = dict(zip(PWR_FIELDS, vals))
                rec["t"] = time.time()
                self.events.put(("pwr", rec))
            except ValueError:
                pass
            return
        self.events.put(("line", line))
        prefix = line.split(",", 1)[0]
        self._deliver(prefix, line)

    def _deliver(self, prefix, obj):
        with self._wlock:
            qs = self._waiters.get(prefix)
            if qs:
                for q in qs:
                    q.put(obj)

    def request(self, cmd, prefixes, timeout=3.0):
        """Send cmd and return (prefix, payload) of the first reply whose
        prefix is in `prefixes`."""
        if isinstance(prefixes, str):
            prefixes = (prefixes,)
        q = queue.Queue()
        with self._wlock:
            for p in prefixes:
                self._waiters.setdefault(p, []).append(q)
        try:
            self.send(cmd)
            obj = q.get(timeout=timeout)
        except queue.Empty:
            raise LinkError(f"no reply to {cmd} within {timeout:g} s") from None
        finally:
            with self._wlock:
                for p in prefixes:
                    self._waiters[p].remove(q)
        if isinstance(obj, str) and obj.startswith("$ERR"):
            raise LinkError(obj[5:])
        return obj

    # ── commands ────────────────────────────────────────────────────────────
    def identify(self):
        return self.request("!ID", ("$READY",), 2.0)

    def read_cfg(self):
        self.request("!CFG", ("$CFGEND", "$ERR"), 3.0)
        return self.cfg

    def get_status(self):
        self.request("!STATUS", ("$STATUS",), 2.0)
        return self.status

    def set(self, key, value):
        line = self.request(f"!SET,{key},{value:.7g}" if isinstance(value, float)
                            else f"!SET,{key},{value}", ("$CFG", "$ERR"), 2.0)
        return self.cfg.get(key.upper(), {}).get("value")

    def save(self):
        self.request("!SAVE", ("$OK", "$ERR"), 3.0)

    def relay(self, on):
        self.request(f"!RELAY,{1 if on else 0}", ("$RLY",), 2.0)

    def stream(self, on):
        self.request("!START" if on else "!STOP", ("$OK",), 2.0)

    def zero(self):
        self.request("!ZERO", ("$OK", "$ERR"), 3.0)
        return self.cfg["VOFF"]["value"], self.cfg["IOFF"]["value"]

    def dc(self, ms=500):
        """Averaged DC of both channels -> dict(v_adc, i_adc, v, i, v_sd, i_sd, n, flags)."""
        line = self.request(f"!DC,{int(ms)}", ("$DC",), ms / 1000.0 + 3.0)
        p = [float(x) for x in line[4:].split(",")]
        d = dict(zip(("v_adc", "i_adc", "v", "i", "v_sd", "i_sd", "n", "flags"), p))
        d["n"], d["flags"] = int(d["n"]), int(d["flags"])
        return d

    def capture(self, rate=20000, n=3000, ch=2, trig=0, level=0.5, pre=10.0,
                timeout_ms=5000, relay_us=-1, ipk_a=0.0, ov_a=0.0, ov_cyc=3, auto_ms=0,
                open_after=False, avg=1):
        """Safe start (0 = off): ipk_a instant trip; ov_a for ov_cyc cycles of per-cycle
        Irms; auto_ms opens that long after the trigger; open_after at the end.
        avg: each stored sample is the mean of this many consecutive ADC scans."""
        cmd = (f"!CAP,{int(rate)},{int(n)},{int(ch)},{int(trig)},{level:g},{pre:g},{int(timeout_ms)},"
               f"{int(relay_us)},{ipk_a:g},{ov_a:g},{int(ov_cyc)},{int(auto_ms)},{1 if open_after else 0},{int(avg)}")
        obj = self.request(cmd, ("$CEND", "$CAPTMO", "$ERR"), timeout_ms / 1000.0 + 15.0)
        if isinstance(obj, str):
            if obj.startswith("$CAPTMO"):
                raise LinkError("capture timed out waiting for the trigger")
            raise LinkError(obj)
        return obj


# ── analysis helpers shared by the GUI and calibration ───────────────────────
def fundamental(x, dt_s, f_guess=60.0):
    """Least-squares fit of a sine at the measured frequency.
    Returns (freq_hz, amplitude_peak, phase_rad)."""
    t = np.arange(len(x)) * dt_s
    f = estimate_freq(x, dt_s) or f_guess
    # refine frequency with a small search around the zero-crossing estimate
    best = None
    for ff in np.linspace(f * 0.995, f * 1.005, 21):
        A = np.column_stack([np.cos(2 * np.pi * ff * t), np.sin(2 * np.pi * ff * t), np.ones_like(t)])
        coef, res, *_ = np.linalg.lstsq(A, x, rcond=None)
        r = float(res[0]) if len(res) else float(np.sum((A @ coef - x) ** 2))
        if best is None or r < best[0]:
            best = (r, ff, coef)
    _, ff, (a, b, _) = best
    return ff, float(np.hypot(a, b)), float(np.arctan2(-b, a))


def estimate_freq(x, dt_s):
    x = np.asarray(x, dtype=float) - np.mean(x)
    # hysteresis: a rising crossing counts only once the signal has been below
    # -h and then gets above +h, so noise around zero can't add extra crossings
    h = 0.2 * np.std(x)
    lo_i = np.nonzero(x < -h)[0]
    hi_i = np.nonzero(x > h)[0]
    if not len(lo_i) or not len(hi_i):
        return None
    fr, k = [], lo_i[0]
    while True:
        j = np.searchsorted(hi_i, k)                 # next sample above +h
        if j >= len(hi_i):
            break
        up = hi_i[j]
        seg = x[k:up + 1]                            # last sign change before `up`
        neg = np.nonzero(seg < 0)[0]
        m = k + neg[-1]
        fr.append(m + x[m] / (x[m] - x[m + 1]))
        j = np.searchsorted(lo_i, up)
        if j >= len(lo_i):
            break
        k = lo_i[j]
    if len(fr) < 2:
        return None
    span = (fr[-1] - fr[0]) * dt_s
    return (len(fr) - 1) / span if span > 0 else None


def harmonics(x, dt_s, f0, nmax=40):
    """Harmonic RMS amplitudes 1..nmax from a whole number of cycles of x."""
    x = np.asarray(x, dtype=float)
    per = 1.0 / (f0 * dt_s)
    cycles = int(len(x) / per)
    if cycles < 1:
        return np.array([]), float("nan")
    m = int(round(cycles * per))
    xs = x[:m] - np.mean(x[:m])
    t = np.arange(m) * dt_s
    amps = []
    for h in range(1, nmax + 1):
        if h * f0 >= 0.5 / dt_s:
            break
        c = np.cos(2 * np.pi * h * f0 * t)
        s = np.sin(2 * np.pi * h * f0 * t)
        a = 2 * np.mean(xs * c)
        b = 2 * np.mean(xs * s)
        amps.append(np.hypot(a, b) / np.sqrt(2))
    amps = np.array(amps)
    thd = float(np.sqrt(np.sum(amps[1:] ** 2)) / amps[0]) if len(amps) > 1 and amps[0] > 0 else float("nan")
    return amps, thd


def capture_stats(cap, f_guess=60.0):
    """RMS / power / harmonics from a capture, over a whole number of cycles."""
    dt = cap.dt_us * 1e-6
    out = {"dt_us": cap.dt_us, "rate": 1e6 / cap.dt_us if cap.dt_us else 0, "n": cap.n}
    ref = cap.v if cap.v is not None else cap.i
    f = estimate_freq(ref, dt) or f_guess
    out["hz"] = f
    per = 1.0 / (f * dt)
    cyc = int(len(ref) / per)
    m = int(round(cyc * per)) if cyc >= 1 else len(ref)
    for name, x in (("v", cap.v), ("i", cap.i)):
        if x is None:
            continue
        xs = x[:m]
        out[name + "rms"] = float(np.sqrt(np.mean((xs - xs.mean()) ** 2)))
        out[name + "pk"] = float(np.max(np.abs(x)))
        out[name + "crest"] = out[name + "pk"] / out[name + "rms"] if out[name + "rms"] > 0 else float("nan")
        out[name + "harm"], out[name + "thd"] = harmonics(x, dt, f)
    if cap.v is not None and cap.i is not None:
        vs, is_ = cap.v[:m] - cap.v[:m].mean(), cap.i[:m] - cap.i[:m].mean()
        out["p"] = float(np.mean(vs * is_))
        s = out["vrms"] * out["irms"]
        out["pf"] = out["p"] / s if s > 0 else float("nan")
    return out
