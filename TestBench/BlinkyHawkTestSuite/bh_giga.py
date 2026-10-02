"""
bh_giga.py  --  the isolated Giga R1 meter (Giga_RemoteController_Dual) as a
suite instrument.

WHAT IT IS.  A battery-powered Arduino Giga R1 with a ~200 Mohm instrumentation
front end, talking over WiFi -- so, unlike a scope probe (earthed ground clip)
or the electronic load (earthed input), it can watch a BlinkyHawk running on its
own battery without re-grounding it.  Wired to the SENSE NODE vs ground (A2-A3).

WHAT IT ADDS over the unit's RAM log.  The log stores one set of numbers per
detection pass; it cannot time anything while the board sleeps (millis() is
frozen in Software Standby).  The meter sees every pass as a pulse on the sense
node -- the pulsed analog rail coming up, the MOSFET test, the recovery -- in
real time, asleep or awake.  So it answers TIMING questions: probe cadence in
each sleep stage, when the deep stage starts, and wake latency.

LOADING.  200 Mohm is negligible while the analog rail is up, but between
passes the node is high impedance and the meter's input is plausibly what
discharges it (the ~15 ms decay seen in the first captures = 200 Mohm x ~75 pF).
Timing is unaffected; levels between passes should not be trusted.

PROTOCOL (Giga_RemoteController_Dual, TCP 8080, ONE command per connection --
the Giga's socket pool wedges if sockets are left open):
  -> PINS:A2,A3|BITS:16|TIME:<ms>|SMOOTH:<n>|LOG:<points>|RATE:<Hz>|TRIG:0,..|MID:0
  <- PIN:A2|...stats...   DATA:A2|VALS:<raw>,<raw>,...   ...   DONE|ELAPSED:<ms>
Field ORDER matters (the sketch slices between field names).  Raw counts ->
volts is the GUI's: v = raw * 3.3/(2^bits-1); a pair is ((pos-neg) - off) * scale,
with off/scale from the named preset in giga_presets.json -- read from the GUI's
own file so there is one calibration, not two.
"""

import json
import os
import socket
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PRESETS = os.path.normpath(os.path.join(HERE, "..", "..", "ArduinoProgrammingFiles",
                                        "Giga_RemoteController_Dual", "giga_presets.json"))
DEFAULT_ADDR = "192.168.40.152:8080"
DEFAULT_PRESET = "BlinkyHawkObservation"
MAX_LOG = 10000                      # Giga_RemoteController_Dual MAX_LOG (points per pin)


class GigaError(Exception):
    pass


class Capture:
    """One capture: t (ms from capture start), v (calibrated volts), and the
    host wall-clock window the capture was requested in."""

    def __init__(self, t, v, elapsed_ms, t_sent, t_done, label="A2-A3"):
        self.t, self.v, self.elapsed_ms = t, v, elapsed_ms
        self.t_sent, self.t_done, self.label = t_sent, t_done, label

    @property
    def dt_ms(self):
        return self.elapsed_ms / max(1, len(self.v))


class GigaMeter:
    name = "Isolated meter (Giga)"

    def __init__(self, addr=DEFAULT_ADDR, preset=DEFAULT_PRESET, presets_path=PRESETS):
        host, _, port = addr.partition(":")
        self.host, self.port = host, int(port or 8080)
        self.ident = f"Giga meter {self.host}:{self.port}"
        self.preset_name = preset
        self.load_preset(preset, presets_path)
        self.connected = True

    def load_preset(self, name, path=PRESETS):
        with open(path, encoding="utf-8") as fh:
            p = json.load(fh)[name]
        if not p.get("diff_enable"):
            raise GigaError(f"preset {name!r} has no differential pair enabled")
        self.pos, self.neg = p["diff_pos"], p["diff_neg"]
        self.d_off, self.d_scale = float(p["diff_offset"]), float(p["diff_scale"])
        self.pin_cal = {k: (float(v["offset"]), float(v["scale"]))
                        for k, v in p["pins"].items()}
        self.bits, self.rate = int(p["bits"]), int(p["rate"])
        self.smooth = max(1, int(p["smooth"]))
        self.label = f"{self.pos}-{self.neg}"

    def connect(self, *_a, **_k):
        """Reachability check only -- every command opens its own socket."""
        with socket.create_connection((self.host, self.port), timeout=3):
            pass
        return self.ident + f"   preset {self.preset_name}: {self.label} x{self.d_scale:g}"

    def close(self):
        pass

    def capture(self, ms, smooth=None, rate=None, points=None):
        """Capture `ms` of the pair.  Default: the preset's rate/smoothing and
        every sample kept (up to MAX_LOG), so short pulses are not decimated
        away.  Longer windows need more smoothing, not decimation: decimating
        PICKS samples and can step over a sub-millisecond pulse entirely."""
        smooth = max(1, int(smooth or self.smooth))
        rate = int(rate or self.rate)
        eff = rate / smooth
        n = int(points or min(MAX_LOG, ms * eff / 1000.0))
        if ms * eff / 1000.0 > MAX_LOG * 1.02 and points is None:
            raise GigaError(f"{ms} ms at {eff:.0f} S/s is {ms * eff / 1000:.0f} points; the meter "
                            f"keeps {MAX_LOG} -- raise the smoothing")
        cmd = (f"PINS:{self.pos},{self.neg}|BITS:{self.bits}|TIME:{int(ms)}|SMOOTH:{smooth}"
               f"|LOG:{n}|RATE:{rate}|TRIG:{','.join(['0'] * 8)}|MID:0\n")
        raw = {}
        elapsed = ms
        t_sent = time.time()
        with socket.create_connection((self.host, self.port), timeout=10) as s:
            s.settimeout(ms / 1000.0 + 15)
            s.sendall(cmd.encode())
            buf = b""
            done = False
            while not done:
                chunk = s.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.decode(errors="ignore").strip()
                    if line.startswith("ERR:"):
                        raise GigaError(f"meter: {line}")
                    if line.startswith("DATA:"):
                        head, _, vals = line.partition("|VALS:")
                        # Drop anything the ADC cannot produce: a transfer glitch
                        # once decoded one sample as ~150 kV.
                        top = 2 ** self.bits - 1
                        raw[head.split(":", 1)[1]] = [int(x) for x in vals.split(",")
                                                      if x.strip().isdigit() and int(x) <= top]
                    elif line.startswith("DONE"):
                        if "|ELAPSED:" in line:
                            try:
                                elapsed = int(line.split("|ELAPSED:")[1])
                            except ValueError:
                                pass
                        done = True
                        break
        t_done = time.time()
        if not done or self.pos not in raw or self.neg not in raw:
            raise GigaError("meter closed the connection before DONE / without both pins")
        step = 3.3 / (2 ** self.bits - 1)
        po, ps = self.pin_cal.get(self.pos, (0.0, 1.0))
        no, ns = self.pin_cal.get(self.neg, (0.0, 1.0))
        a = [(x * step - po) * ps for x in raw[self.pos]]
        b = [(x * step - no) * ns for x in raw[self.neg]]
        n = min(len(a), len(b))
        v = [((a[i] - b[i]) - self.d_off) * self.d_scale for i in range(n)]
        t = [i * elapsed / n for i in range(n)]       # the GUI's own time axis
        cap = Capture(t, v, elapsed, t_sent, t_done, self.label)
        # The single-ended legs, in volts at the meter's pins.  A flat pair
        # with a leg pinned at 0 or 3.3 V means the floating board sits outside
        # the meter's common-mode range, not that the board did nothing.
        cap.legs = (a[:n], b[:n])
        return cap


    # ---- live stream: unbroken coverage, real-time arrival ----
    def stream(self, seconds, hz=20, on_period=None, stop_event=None):
        """Stream the pair for `seconds`.  Returns [(host_time, seq, max_V, mean_V,
        t_board_s)], one per period received.  A capture has to be transferred before the next can
        start (a 5 s window takes ~20 s round trip), so it leaves gaps; the
        stream reports every 1/hz s for as long as it runs, each line arriving
        as the period ends -- which is what timing a stage change or a wake
        needs.  `on_period(row)` is called as each period arrives.

        The board reduces (pos - neg) per sample for D1, so MAX is the true
        peak of the pair in that period: a detection pass shows as a high max.
        """
        cmd = (f"STREAM:D1|D1:{self.pos}-{self.neg}|BITS:{self.bits}|RATE:{self.rate}"
               f"|SMOOTH:{self.smooth}|HZ:{int(hz)}\n")
        step = 3.3 / (2 ** self.bits - 1)
        rows = []
        with socket.create_connection((self.host, self.port), timeout=10) as s:
            s.settimeout(5)
            s.sendall(cmd.encode())
            buf = b""
            t_end = time.time() + seconds
            started = False
            try:
                while time.time() < t_end:
                    if stop_event is not None and stop_event.is_set():
                        break
                    try:
                        chunk = s.recv(65536)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise GigaError("meter closed the stream")
                    now = time.time()
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.decode(errors="ignore").strip()
                        if line.startswith("ERR:"):
                            raise GigaError(f"meter: {line}")
                        if line.startswith("STREAMON"):
                            started = True
                            if f"D1:{self.pos}-{self.neg}" not in line:
                                raise GigaError(f"meter streamed different pins: {line}")
                        elif line.startswith("S|") and "|CH:D1|" in line:
                            kv = dict(x.split(":", 1) for x in line.split("|")[1:] if ":" in x)
                            mx = (float(kv["MAX"]) * step - self.d_off) * self.d_scale
                            mean = (float(kv["MEAN"]) * step - self.d_off) * self.d_scale
                            row = (now, int(kv["SEQ"]), mx, mean, float(kv["T"]) / 1000.0)
                            rows.append(row)
                            if on_period:
                                on_period(row)
            finally:
                try:
                    s.sendall(b"STOP\n")
                    s.settimeout(2)
                    s.recv(4096)                 # STREAMOFF (best effort)
                except OSError:
                    pass
        if not started:
            raise GigaError("the meter never confirmed the stream (STREAMON)")
        return rows


PASS_FLOOR_V = 1.0       # sense node, BlinkyHawkObservation calibration -- see pass_periods


def board_to_host(rows):
    """Offset that maps a period's board time to host time.  Reports can be
    skipped and arrive late when WiFi is slow, so arrival times alone are
    wrong; the board's own T stamps are exact.  The earliest-arriving report
    (smallest arrival - T) sets the offset: it had the least network delay."""
    return min(r[0] - r[4] for r in rows) if rows else 0.0


def pass_periods(rows, level=None):
    """Indices of stream periods that contain a detection pass (high max).
    The level defaults to midway between the quiet and the pass maxima."""
    if not rows:
        return [], None
    mx = sorted(r[2] for r in rows)
    if level is None:
        lo, hi = mx[len(mx) // 10], mx[-1 - len(mx) // 50]
        level = lo + 0.5 * (hi - lo)
        # An adaptive level on a trace with NO passes splits pure noise and
        # invents events.  A pass peaks near 2.2 V on the sense node (the MOSFET
        # edge) and the rail plateau sits at ~1.25 V; nothing between passes
        # reaches 1 V.  So a period must clear an absolute floor as well.
        level = max(level, PASS_FLOOR_V)
    return [i for i, r in enumerate(rows) if r[2] > level], level


# ---------------------------------------------------------------------------
# analysis: one pulse per detection pass
# ---------------------------------------------------------------------------
def find_passes(cap, min_gap_ms=8.0, k=6.0):
    """Times (ms) of the detection passes in a capture.

    Each pass ends with a sharp positive spike (the bridge MOSFET switching back
    on) that stands far above everything else on the node: ~2.4 V against a
    ~1.25 V plateau in the first captures.  Detect it as a jump of k x the
    sample-to-sample noise, keep the first sample of each burst, and enforce a
    minimum gap so one pass is one event.
    """
    v = cap.v
    if len(v) < 10:
        return []
    d = [v[i + 1] - v[i] for i in range(len(v) - 1)]
    ad = sorted(abs(x) for x in d)
    noise = ad[len(ad) // 2] or 1e-6                 # median |step|: robust to the pulses
    hi = sorted(v)[int(len(v) * 0.995)]
    level = max(noise * k, 0.25 * (hi - sorted(v)[len(v) // 2]))
    out = []
    last = -1e9
    for i, x in enumerate(d):
        if x > level and cap.t[i + 1] - last >= min_gap_ms:
            out.append(cap.t[i + 1])
            last = cap.t[i + 1]
    return out


def cadence(times):
    """-> (median interval ms, intervals) or (None, [])."""
    iv = [b - a for a, b in zip(times, times[1:])]
    if not iv:
        return None, []
    s = sorted(iv)
    return s[len(s) // 2], iv
