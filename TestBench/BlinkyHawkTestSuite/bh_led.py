"""
bh_led.py  --  turn AS7343 flash reports into "what is the BlinkyHawk alerting".

The relay jig's AS7343 sits over the BlinkyHawk's SK6812 and reports one
$FLASH line per flash (see RelayMountTestJig.ino): the mean FZ / FY / FXL
counts above the dark baseline -- roughly the sensor's blue (450 nm), green
(555 nm) and red-ish (600 nm) channels -- plus duration and brightness.

1. COLOUR per flash: the three channels are normalised (so brightness drops
   out -- the float alert is much dimmer than the others) and compared by
   cosine similarity with a reference vector per LED colour.  The references
   ship as rough guesses and should be CALIBRATED on the rig: the sensor's
   channel responses to an SK6812 depend on the part, the gain and the aim.
2. STATE over a window, from the SET of colours seen (the firmware alerts):
      blue only           FLOAT       dim blue flash
      green only          CLOSED      green flash
      red only            VDC+        red, red
      red + blue          VDC-        red, blue
      red + blue + green  VAC         red, blue, green
      nothing             dark        LED off / disabled / asleep between heartbeats
      1 flash only        few         too short a window to say -- NOT judged
   A window that straddles a change shows a mix that fits none of these and
   decodes as "?" -- which is why the sequencer only looks after its settle.
   The window must hold at least two flashes of the SLOWEST alert (the float
   blink is typically once a second), so dwell >= ~2.5 s.
"""

import json
import math
import os
import time
from dataclasses import dataclass

HERE = os.path.dirname(os.path.abspath(__file__))
CAL_PATH = os.path.join(HERE, "led_calibration.json")

COLOURS = ("R", "G", "B")
# Uncalibrated guesses: (FZ, FY, FXL) proportions for each SK6812 die.
DEFAULT_REFS = {"B": (0.85, 0.12, 0.03), "G": (0.15, 0.75, 0.10), "R": (0.03, 0.12, 0.85)}
MIN_COSINE = 0.90          # below this against every reference -> "?" (unknown colour)

# LED decode -> the suite's state vocabulary.  C/F as in $DET; voltage kinds as
# the firmware names them.
STATE_NAMES = {"F": "FLOAT", "C": "CLOSED", "VDC+": "VDC+", "VDC-": "VDC-",
               "VAC": "VAC", "dark": "dark", "few": "too few flashes", "?": "?"}


@dataclass
class Flash:
    t: float          # host time the flash STARTED (arrival - duration)
    dur_ms: int
    n: int
    fz: float
    fy: float
    fxl: float
    vis: float
    peak: float
    sat: bool
    colour: str = ""
    cos: float = 0.0


def parse_flash(line, arrival=None):
    """$FLASH,<startMs>,<durMs>,<n>,<fz>,<fy>,<fxl>,<vis>,<peak>,<sat>"""
    f = line.split(",")
    if len(f) < 10 or f[0] != "$FLASH":
        return None
    try:
        dur = int(f[2])
        now = time.time() if arrival is None else arrival
        return Flash(now - dur / 1000.0, dur, int(f[3]), float(f[4]), float(f[5]),
                     float(f[6]), float(f[7]), float(f[8]), f[9].strip() == "1")
    except ValueError:
        return None


def _unit(v):
    v = [max(0.0, x) for x in v]
    n = math.sqrt(sum(x * x for x in v))
    return tuple(x / n for x in v) if n > 0 else None


class LedDecoder:
    def __init__(self, path=CAL_PATH):
        self.path = path
        self.refs = {k: _unit(v) for k, v in DEFAULT_REFS.items()}
        self.calibrated = {}
        self.load()

    # ---- calibration ----
    def load(self):
        try:
            with open(self.path, encoding="utf-8") as fh:
                d = json.load(fh)
            for k in COLOURS:
                if k in d.get("refs", {}):
                    self.refs[k] = _unit(d["refs"][k])
                    self.calibrated[k] = d.get("when", {}).get(k, "?")
        except (OSError, ValueError):
            pass

    def save(self):
        with open(self.path, "w", encoding="utf-8") as fh:
            json.dump({"refs": {k: list(v) for k, v in self.refs.items()},
                       "when": self.calibrated}, fh, indent=2)

    def learn(self, colour, flashes):
        """Set one colour's reference from flashes known to be that colour.
        Returns (ok, message)."""
        vecs = [_unit((f.fz, f.fy, f.fxl)) for f in flashes if not f.sat]
        vecs = [v for v in vecs if v]
        if len(vecs) < 3:
            sat = sum(1 for f in flashes if f.sat)
            return False, (f"{colour}: only {len(vecs)} usable flash(es)"
                           + (f", {sat} saturated -- lower the gain" if sat else ""))
        mean = _unit([sum(v[i] for v in vecs) / len(vecs) for i in range(3)])
        spread = min(sum(a * b for a, b in zip(mean, v)) for v in vecs)
        self.refs[colour] = mean
        self.calibrated[colour] = time.strftime("%Y-%m-%d %H:%M")
        return True, (f"{colour}: {len(vecs)} flashes, ref FZ/FY/FXL = "
                      f"{mean[0]:.3f}/{mean[1]:.3f}/{mean[2]:.3f}, worst cosine {spread:.3f}")

    def separation(self):
        """Smallest cosine distance between two references (bigger = safer)."""
        ks = list(self.refs)
        worst = 1.0
        for i in range(len(ks)):
            for j in range(i + 1, len(ks)):
                c = sum(a * b for a, b in zip(self.refs[ks[i]], self.refs[ks[j]]))
                worst = min(worst, 1 - c)
        return worst

    # ---- per flash ----
    def classify(self, f):
        v = _unit((f.fz, f.fy, f.fxl))
        if v is None:
            f.colour, f.cos = "?", 0.0
            return f
        best, bc = "?", -1.0
        for k, r in self.refs.items():
            c = sum(a * b for a, b in zip(v, r))
            if c > bc:
                best, bc = k, c
        f.colour, f.cos = (best if bc >= MIN_COSINE else "?"), bc
        return f

    # ---- over a window ----
    SET_STATE = {frozenset("B"): "F", frozenset("G"): "C", frozenset("R"): "VDC+",
                 frozenset("RB"): "VDC-", frozenset("RBG"): "VAC"}
    # The firmware's VKIND_SEQ.  The colour SET is not enough: a VDC+ alert
    # (red, red...) followed by the float blink (blue) is also "red + blue", and
    # would read as VDC-.  So a voltage kind must also show its ORDER.
    KIND_CYCLE = {"VDC-": "RB", "VAC": "RBG"}
    MIN_ORDER = 0.75           # fraction of consecutive flashes that follow the cycle

    def decode(self, flashes, min_flashes=2):
        """-> dict(state, counts, colours, n, unknown, hz, sat, dropped).

        If the whole window is a mix that fits no alert, leading flashes are
        dropped (up to half) until the rest decode cleanly: the tail of the
        PREVIOUS condition's alert, still showing while the unit's own
        STABLECOUNT debounce catches up, is the usual cause.  The serial
        verdict skips its first few passes for the same reason.  'dropped'
        says how many were discarded, so it is never silent.
        """
        fl = [self.classify(f) for f in flashes]
        out = self._decode_set(fl, min_flashes)
        out["dropped"] = 0
        if out["state"] == "?":
            for k in range(1, len(fl) // 2 + 1):
                tail = self._decode_set(fl[k:], min_flashes)
                if tail["state"] not in ("?", "few"):
                    tail["dropped"] = k
                    tail["colours"] += f" (first {k} dropped)"
                    return tail
        return out

    def _decode_set(self, fl, min_flashes):
        counts = {k: sum(1 for f in fl if f.colour == k) for k in COLOURS}
        unknown = sum(1 for f in fl if f.colour == "?")
        known = sum(counts.values())
        out = {"n": len(fl), "counts": counts, "unknown": unknown,
               "sat": any(f.sat for f in fl),
               "colours": " ".join(f"{k}{counts[k]}" for k in COLOURS if counts[k])
                          + (f" ?{unknown}" if unknown else "")}
        if len(fl) >= 2:
            span = fl[-1].t - fl[0].t
            out["hz"] = (len(fl) - 1) / span if span > 0 else 0.0
        if not fl:
            out["state"] = "dark"
        elif known < min_flashes:
            # one flash (or a couple of unknown colours) is not a pattern
            out["state"] = "few" if unknown == 0 else "?"
        else:
            st = self.SET_STATE.get(frozenset(k for k in COLOURS if counts[k]), "?")
            cyc = self.KIND_CYCLE.get(st)
            if cyc:
                seq = [f.colour for f in fl if f.colour != "?"]
                ok = sum(1 for a, b in zip(seq, seq[1:])
                         if cyc[(cyc.index(a) + 1) % len(cyc)] == b)
                if ok < self.MIN_ORDER * (len(seq) - 1):
                    st = "?"            # right colours, wrong order: a transition
            out["state"] = st
        return out


def led_matches(expected, state):
    """Does an LED decode satisfy a sequencer expectation?  None = not judged."""
    if not expected or state is None or state == "few":
        return None
    if expected == "!V":
        return state in ("F", "C")
    if expected == "V":
        return state in ("VDC+", "VDC-", "VAC")
    return state == expected
