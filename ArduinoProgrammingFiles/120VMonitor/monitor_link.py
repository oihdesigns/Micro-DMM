"""
monitor_link.py  --  serial link to the 120VMonitor firmware.

One reader thread owns the port.  Every line goes to `events` (a queue the GUI
drains) except capture data, which is assembled into a Capture first.  Request
helpers (dc(), cfg(), capture() ...) register a one-shot waiter for the reply
prefix they expect, so they can be called from any worker thread.
"""

import os
import queue
import threading
import time
from dataclasses import dataclass, field

import numpy as np

PWR_FIELDS = ("vrms", "irms", "p", "s", "q", "pf", "hz", "vpk", "ipk", "wh", "uptime",
              "n", "depth", "relay", "flags", "vdc", "idc", "rng")

FLAG_NAMES = {1: "NO V ZERO-X", 2: "V CLIP", 4: "I CLIP", 8: "TRIPPED",
              16: "TRUNCATED", 32: "LATE"}

SAFE_WHY = {0: "", 1: "instant limit", 2: "over-current (time delay)", 3: "auto-open timer",
            4: "capture complete", 5: "TRIPPK", 6: "remote", 7: "TRIPA (rms over-current)"}

RANGE_NAMES = {0: "HIGH", 1: "LOW"}          # 0.01 ohm / 0.1 ohm shunt
RANGE_MODES = {0: "Auto", 1: "HIGH only", 2: "LOW", 3: "Auto-delayed"}

TRIG_MODES = {0: "Immediate", 1: "|I| above level", 2: "V rising zero-cross",
              4: "|V| above level", 6: "Irms drops below level",
              3: "Close relay - rising zero", 5: "Close relay - level trigger"}
CURRENT_TRIGS = (1, 5, 6)                    # need the current channel
CLOSE_TRIGS = (3, 5)                         # the capture closes the load relay itself


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
    rng: int = 0            # current range the data starts in (0 HIGH, 1 LOW)
    rng_idx: int = -1       # sample from which the range was HIGH (overload), -1 never
    i_per_hi: float = None  # HIGH-range scaling for samples from rng_idx on
    i_off_hi: float = None
    settle_us: float = 0.0  # range relay moving after rng_idx
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
            if self.rng_idx >= 0 and self.i_per_hi is not None:
                k = self.rng_idx
                self.i[k:] = (ic[k:] - self.i_off_hi) * self.i_per_hi
        n = len(vc if vc is not None else ic)
        t0 = self.trig_idx if self.trig_idx >= 0 else 0
        self.t_ms = (np.arange(n) - t0) * self.dt_us / 1000.0
        self.n = n
        self.raw = []
        return self


# ── config snapshots ────────────────────────────────────────────────────────
# What each key is for, so a snapshot can be copied one kind at a time. The
# firmware tags its own keys in !DUMP (fw 1.4+); this table only stands in for
# older firmware, which has !CFG alone.
CFG_GROUPS = {
    "CAL": "VGAIN VOFF IGAIN IOFF IGAINLO IOFFLO PHASEUS ADCVREF",
    "FUSE": "TRIPA TRIPPK",
    "RANGE": "RNGMODE RNGUPA RNGDNA RNGHOLD RNGSETMS RNGCYC",
    "METER": "MAINS SPC CYCLES",
    "WIRING": "RLYPIN RLYINV RNGPIN RNGINV IRPIN",
    "UI": "OLED OLEDADDR OLEDROT IRADDR IROK IRCONF IRARMMS PIXEL LIVEVAC",
}
GROUP_OF = {k: g for g, keys in CFG_GROUPS.items() for k in keys.split()}
GROUP_INFO = {"CAL": "analog calibration: gains, offsets, phase, ADC reference",
              "FUSE": "virtual fuses: TRIPA / TRIPPK",
              "RANGE": "current range switching thresholds and mode",
              "METER": "mains Hz, samples per cycle, window length",
              "WIRING": "relay, range relay and IR pins and their polarity",
              "UI": "OLED, IR remote codes, NeoPixel"}


def same_value(a, b):
    return abs(a - b) <= 1e-6 * max(1.0, abs(a), abs(b))


@dataclass
class Snapshot:
    """Every config key of one board, with its group and where it came from.
    meta: fw, board, uid, cfgver, src (RAM / SAVED / RAM-CFG), dirty, taken."""
    meta: dict
    rows: list = field(default_factory=list)      # (group, key, value)

    def values(self):
        return {k: v for _, k, v in self.rows}

    def same_values(self, other):
        a, b = self.values(), other.values()
        return a.keys() == b.keys() and all(same_value(a[k], b[k]) for k in a)

    def diff(self, current):
        """(group, key, current value or None, snapshot value) for every key
        whose value differs from `current` (key -> value)."""
        return [(g, k, current.get(k), v) for g, k, v in self.rows
                if k not in current or not same_value(current[k], v)]

    def save(self, path):
        with open(path, "w", newline="", encoding="utf-8") as f:
            f.write("# 120VMonitor config snapshot\n")
            for k, v in self.meta.items():
                f.write(f"#{k},{v}\n")
            f.write("group,key,value\n")
            for g, k, v in self.rows:
                f.write(f"{g},{k},{v:.9g}\n")

    @classmethod
    def load(cls, path):
        snap = cls(meta={})
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line.startswith("#"):
                    k, sep, v = line[1:].partition(",")
                    if sep:
                        snap.meta[k.strip()] = v.strip()
                    continue
                p = line.split(",")
                if len(p) >= 3 and p[0] != "group":
                    try:
                        snap.rows.append((p[0].upper(), p[1].upper(), float(p[2])))
                    except ValueError:
                        pass
        if not snap.rows:
            raise ValueError(f"{os.path.basename(path)} holds no config rows")
        return snap


def log_snapshot(snap, folder):
    """Write snap to folder/<board>_<uid>_<time>.csv unless the newest file for
    that board already holds the same values. Returns the path, or None."""
    os.makedirs(folder, exist_ok=True)
    stem = f"{snap.meta.get('board', 'board')}_{snap.meta.get('uid', '')[-8:] or 'nouid'}"
    older = sorted(n for n in os.listdir(folder) if n.startswith(stem + "_") and n.endswith(".csv"))
    if older:
        try:
            if Snapshot.load(os.path.join(folder, older[-1])).same_values(snap):
                return None
        except (OSError, ValueError):
            pass
    path = os.path.join(folder, f"{stem}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    snap.save(path)
    return path


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
        self._cap_want = 0            # int16 values the $CAP header promised
        self._cap_bad = 0             # $CD lines that didn't parse
        self._alive = True
        self.cfg = {}                 # key -> dict(value, lo, hi, help)
        self.status = {}
        self._dump = None             # Snapshot being assembled from $DUMP lines
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
                # every line starts with '$': one mid-line means bytes (and a newline) were
                # lost, so the damaged part ends there and the next line is kept
                parts = line.split(b"$")
                for k, part in enumerate(parts):
                    text = ("$" if k else "") + part.decode("ascii", "replace").strip()
                    try:
                        self._line(text)
                    except Exception as e:                 # noqa: BLE001 - never kill the reader
                        self.events.put(("line", f"!! unreadable line ({e}): {text[:80]}"))

    def _line(self, line):
        if not line or line == "$":
            return
        if line.startswith("$CD,"):
            if self._cap is not None:
                h = line[4:]
                try:
                    if len(h) % 4:
                        raise ValueError("odd length")
                    self._cap.raw.extend(int(h[k:k + 4], 16) - (0x10000 if h[k] in "89ABCDEFabcdef" else 0)
                                         for k in range(0, len(h), 4))
                except ValueError:
                    self._cap_bad += 1                     # the count check at $CEND reports it
            return
        if line.startswith("$CAP,"):
            p = line[5:].split(",")
            try:
                self._cap = Capture(int(p[0]), int(p[1]), float(p[2]), float(p[3]), float(p[4]),
                                    float(p[5]), float(p[6]), int(p[7]), int(p[8]), int(p[9]),
                                    int(p[10]) if len(p) > 10 else -1,
                                    int(p[11]) if len(p) > 11 else 0,
                                    int(p[12]) if len(p) > 12 else 1)
                if len(p) > 17:
                    c = self._cap
                    c.rng, c.rng_idx = int(p[13]), int(p[14])
                    c.i_per_hi, c.i_off_hi, c.settle_us = float(p[15]), float(p[16]), float(p[17])
            except (ValueError, IndexError):
                self._cap = None
            try:
                self._cap_want = int(p[0]) * (2 if self._cap is not None and self._cap.ch == 2 else 1)
            except ValueError:
                self._cap_want = -1
            self._cap_bad = 0
            self.events.put(("line", line))
            return
        if line.startswith("$CEND"):
            cap, self._cap = self._cap, None
            got = len(cap.raw) if cap is not None else 0
            if cap is None or self._cap_bad or got != self._cap_want:
                why = "its header was lost" if cap is None else f"{got} of {self._cap_want} values arrived"
                self._deliver("$CEND", f"$ERR,data damaged in transfer ({why}) - capture again")
                self.events.put(("line", f"!! capture damaged in transfer: {why}"))
                return
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
        elif line.startswith("$DUMPHDR,"):
            self._dump = Snapshot(meta=dict(kv.split("=", 1) for kv in line[9:].split(",") if "=" in kv))
        elif line.startswith("$DUMP,"):
            p = line[6:].split(",")
            if self._dump is not None and len(p) >= 3:
                try:
                    self._dump.rows.append((p[0], p[1], float(p[2])))
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

    def dump(self, saved=False):
        """Snapshot of every key and its group: the running values, or with
        saved=True the flash/EEPROM block (LinkError if none is stored).
        Firmware before 1.4 has no !DUMP; then it is !CFG (RAM only), the
        group table above and no uid."""
        try:
            self.request("!DUMP,SAVED" if saved else "!DUMP", ("$DUMPEND", "$ERR"), 3.0)
        except LinkError as e:
            if "unknown command" not in str(e):
                raise
            self.read_cfg()
            st = self.get_status()
            snap = Snapshot(meta={"fw": st.get("fw", "?"), "board": st.get("board", "RA4M1"),
                                  "uid": "", "cfgver": "?", "src": "RAM-CFG",
                                  "dirty": st.get("dirty", "?")})
            snap.rows = [(GROUP_OF.get(k, "OTHER"), k, r["value"]) for k, r in self.cfg.items()]
        else:
            snap, self._dump = self._dump, None
        snap.meta["taken"] = time.strftime("%Y-%m-%d %H:%M:%S")
        return snap

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

    def range_mode(self, mode):
        """RNGMODE (RAM): 0 auto, 1 HIGH only, 2 LOW. Returns the range now in use."""
        line = self.request(f"!RANGE,{int(mode)}", ("$RNG",), 2.0)
        return int(line.split(",")[1])

    def zero(self):
        self.request("!ZERO", ("$OK", "$ERR"), 3.0)
        return self.cfg["VOFF"]["value"], self.cfg["IOFF"]["value"]

    def dc(self, ms=500):
        """Averaged DC of both channels -> dict(v_adc, i_adc, v, i, v_sd, i_sd, n, flags)."""
        line = self.request(f"!DC,{int(ms)}", ("$DC",), ms / 1000.0 + 3.0)
        p = [float(x) for x in line[4:].split(",")]
        d = dict(zip(("v_adc", "i_adc", "v", "i", "v_sd", "i_sd", "n", "flags", "rng", "rng_changed"), p))
        d["n"], d["flags"] = int(d["n"]), int(d["flags"])
        d["rng"], d["rng_changed"] = int(d.get("rng", 0)), int(d.get("rng_changed", 0))
        return d

    def capture(self, rate=20000, n=3000, ch=2, trig=0, level=0.5, pre=10.0,
                timeout_ms=5000, relay_us=-1, ipk_a=0.0, ov_a=0.0, ov_cyc=3, auto_ms=0,
                open_after=False, avg=1, rng=-1, mon=False):
        """Safe start (0 = off): ipk_a instant trip; ov_a for ov_cyc cycles of per-cycle
        Irms; auto_ms opens that long after the trigger; open_after at the end.
        avg: each stored sample is the mean of this many consecutive ADC scans.
        rng: -1 follow RNGMODE, 0 HIGH, 1 LOW. timeout_ms 0 waits until abort().
        mon: keep $PWR readings coming while armed (made from the capture's samples)."""
        cmd = (f"!CAP,{int(rate)},{int(n)},{int(ch)},{int(trig)},{level:g},{pre:g},{int(timeout_ms)},"
               f"{int(relay_us)},{ipk_a:g},{ov_a:g},{int(ov_cyc)},{int(auto_ms)},{1 if open_after else 0},"
               f"{int(avg)},{int(rng)},{1 if mon else 0}")
        obj = self.request(cmd, ("$CEND", "$CAPTMO", "$ERR"),
                           timeout_ms / 1000.0 + 15.0 if timeout_ms else None)
        if isinstance(obj, str):
            if obj.startswith("$CAPTMO"):
                raise LinkError("aborted" if "ABORT" in obj
                                else "timed out waiting for the trigger")
            raise LinkError(obj)
        return obj

    def abort(self):
        """Stop a capture that is waiting for its trigger; capture() then raises."""
        self.send("!ABORT")


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


def rising_crossings(x):
    """Fractional sample positions of the rising zero crossings of x (mean removed)."""
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
    return fr


def estimate_freq(x, dt_s):
    fr = rising_crossings(x)
    if fr is None or len(fr) < 2:
        return None
    span = (fr[-1] - fr[0]) * dt_s
    return (len(fr) - 1) / span if span > 0 else None


def cycle_phase(ref, dt_s, f_guess=60.0):
    """Each sample's place in its mains cycle, 0..1 from the rising zero crossing of
    `ref` (or None to use f_guess), plus the number of cycles. Each cycle is
    stretched between its own crossings, so drift in the mains frequency doesn't
    smear an overlay; a crossing a dropout hid is filled in at the median period,
    and the part before the first / after the last crossing is extrapolated."""
    n = len(ref) if ref is not None else 0
    idx = np.arange(n, dtype=float)
    fr = rising_crossings(ref) if n else None
    per_guess = 1.0 / (f_guess * dt_s)
    if fr is None or len(fr) < 2:
        return (idx / per_guess) % 1.0, n / per_guess
    fr = np.asarray(fr)
    per = float(np.median(np.diff(fr)))
    full = [fr[0]]
    for c in fr[1:]:
        gap = c - full[-1]
        k = int(round(gap / per))
        if k > 1:                                       # missed crossings: fill them in evenly
            full.extend(full[-1] + gap * j / k for j in range(1, k))
        full.append(c)
    full = np.asarray(full)
    cyc = np.interp(idx, full, np.arange(len(full), dtype=float))
    cyc[idx < full[0]] = (idx[idx < full[0]] - full[0]) / per
    cyc[idx > full[-1]] = len(full) - 1 + (idx[idx > full[-1]] - full[-1]) / per
    return cyc % 1.0, n / per


FLG_LATE = 0x20
SLIP_TOL = 0.5             # samples: a timing slip shortens a cycle by at least one


def slip_mask(ref, flags):
    """(keep, dropped). The board sets LATE when it fell behind and restarted its
    sample clock, so somewhere the samples aren't evenly spaced and that cycle won't
    overlay the others. With LATE set, keep only the cycles whose length is within
    SLIP_TOL samples of the median; the partial cycles at the ends go too, as a slip
    can't be seen there. Without LATE, keep is None: nothing is dropped, so a real
    disturbance at a zero crossing is never hidden."""
    if not flags & FLG_LATE:
        return None, 0
    keep = np.zeros(len(ref), dtype=bool)
    fr = rising_crossings(ref)
    if fr is None or len(fr) < 2:
        return keep, 1
    fr = np.asarray(fr)
    d = np.diff(fr)
    med = float(np.median(d))
    dropped = 0
    for a, b, dd in zip(fr[:-1], fr[1:], d):
        if abs(dd - med) <= SLIP_TOL:
            keep[int(np.ceil(a)):int(np.floor(b)) + 1] = True
        else:
            dropped += 1
    return keep, dropped


def cycle_heatmap(x, phase, lim, nx=240, ny=160, keep=None):
    """Persistence map of x folded onto one cycle: counts[ny, nx] over phase 0..1
    and value -lim..+lim. Traces are linearly upsampled first so a cycle with few
    samples still draws a line rather than dots. keep (bool per sample, from
    slip_mask) leaves out every segment that doesn't have both ends kept."""
    x = np.asarray(x, dtype=float)
    spc = len(x) / max(1.0, float(np.sum(np.diff(phase) < -0.5)) + 1.0)   # samples per cycle
    up = int(min(16, max(1, np.ceil(2.0 * nx / max(spc, 1.0)))))
    if up > 1 and len(x) > 1:
        t = np.arange((len(x) - 1) * up + 1) / up
        xs = np.interp(t, np.arange(len(x)), x)
        # unwrap phase so it interpolates across the 1 -> 0 wrap, then fold again
        ph = np.interp(t, np.arange(len(x)), np.unwrap(phase * 2 * np.pi) / (2 * np.pi)) % 1.0
        if keep is not None:
            seg = np.minimum(np.floor(t).astype(int), len(x) - 2)
            ok = keep[seg] & keep[seg + 1]
            xs, ph = xs[ok], ph[ok]
    else:
        xs, ph = (x, phase) if keep is None else (x[keep], phase[keep])
    h, _, _ = np.histogram2d(np.clip(xs, -lim, lim), ph, bins=[ny, nx], range=[[-lim, lim], [0.0, 1.0]])
    return h


LIVE_RATE = 15000          # S/s for live persistence: 250 samples a 60 Hz cycle
LIVE_AVG = 4               # scans per sample
LIVE_MAX_CYCLES = 30       # per capture; fewer if the board's buffer is smaller


def live_capture_args(cap_max, hz=60.0, cycles=10):
    """!CAP arguments for one live-persistence capture: V + I at once, as many
    cycles as the buffer holds (at least the meter window, so readings while armed
    still produce a $PWR line per capture)."""
    spc = LIVE_RATE / (hz or 60.0)
    ncyc = max(int(cycles), min(LIVE_MAX_CYCLES, int(cap_max // 2 / spc)))
    rate = LIVE_RATE if ncyc * spc * 2 <= cap_max else int(cap_max // 2 / ncyc * (hz or 60.0))
    return dict(rate=rate, n=int(ncyc * rate / (hz or 60.0)), ch=2, trig=0, timeout_ms=1000,
                avg=LIVE_AVG, rng=-1, mon=True)


class Persistence:
    """Cycle heat maps that build up over many captures: a live persistence display.
    tau_s 0 keeps everything; otherwise older captures fade with that time constant.
    Each channel's range only grows (by 25 % headroom), re-binning what's there."""

    def __init__(self, nx=240, ny=160, tau_s=0.0):
        self.nx, self.ny, self.tau_s = nx, ny, tau_s
        self.clear()

    def clear(self):
        self.maps = {}                 # "v" / "i" -> [counts (ny, nx), lim]
        self.cycles = 0.0              # cycles in the map (faded too)
        self.captures = 0
        self.late = 0                  # captures the board flagged LATE
        self.dropped = 0               # cycles left out of those (slip_mask)
        self.cap_s = 0.0               # seconds of mains actually sampled
        self.t0 = self.t_last = None
        self.hz = 60.0
        self.ref = "V"

    def coverage(self, now=None):
        """Fraction of the time since the start that was sampled; the rest fell in the
        gaps while each capture was sent."""
        if self.t0 is None:
            return 0.0
        now = time.monotonic() if now is None else now
        return min(1.0, self.cap_s / max(now - self.t0, 1e-9))

    def _regrid(self, h, lim0, lim1):
        r = np.arange(self.ny)
        v = -lim0 + (r + 0.5) * 2 * lim0 / self.ny
        r1 = np.clip(((v + lim1) / (2 * lim1) * self.ny).astype(int), 0, self.ny - 1)
        out = np.zeros_like(h)
        np.add.at(out, r1, h)
        return out

    def add(self, cap, now=None):
        now = time.monotonic() if now is None else now
        dt = cap.dt_us * 1e-6
        ref = cap.v if cap.v is not None else cap.i
        self.ref = "V" if cap.v is not None else "I"
        self.hz = estimate_freq(ref, dt) or self.hz
        phase, ncyc = cycle_phase(ref, dt, self.hz)
        keep, dropped = slip_mask(ref, cap.flags)
        if keep is not None:
            self.late += 1
            self.dropped += dropped
            ncyc = float(np.sum(keep)) * dt * self.hz
        span = len(ref) * dt
        k = 1.0
        if self.tau_s > 0 and self.t_last is not None:
            k = float(np.exp(-(now - self.t_last) / self.tau_s))
        for name, x in (("v", cap.v), ("i", cap.i)):
            if x is None:
                continue
            h, lim = self.maps.get(name, (None, 0.0))
            pk = float(np.nanmax(np.abs(x))) if len(x) else 0.0
            if pk > lim or not lim:
                new = max(pk * 1.25, 1e-6)
                if h is not None:
                    h = self._regrid(h, lim, new)
                lim = new
            hh = cycle_heatmap(x, phase, lim, self.nx, self.ny, keep)
            self.maps[name] = [hh if h is None else h * k + hh, lim]
        self.cycles = self.cycles * k + ncyc
        self.captures += 1
        self.cap_s += span
        if self.t0 is None:
            self.t0 = now - span
        self.t_last = now


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
        out[name + "dc"] = float(xs.mean())
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
