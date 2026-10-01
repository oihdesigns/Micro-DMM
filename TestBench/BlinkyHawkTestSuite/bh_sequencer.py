"""
bh_sequencer.py  --  runs conditions across the bench and records what happened.

A CONDITION is one thing done to the BlinkyHawk's leads: a relay-jig load, a
generator DC level, a generator sine, or the generator path with its output
off.  For each one the runner
    1. turns the generator output off (never switch relays under drive)
    2. moves the relay jig (break-before-make is the jig firmware's job)
    3. programs the generator and turns it on, if the condition needs it
    4. waits the settle time
    5. records $DET passes from the BlinkyHawk for the dwell time, AND/OR the
       LED flashes the relay jig's AS7343 sees over the same window
    6. optionally captures the scope
    7. turns the generator off again
With no serial link (the unit on its own battery) step 5 is the LED alone --
that is the only honest view of a floating unit, since any wire to it would
re-ground it.  Jobs built on this: run_plan() (the "big test"),
run_autotune(), and run_led_calibration().

run_autotune() can also measure ON BATTERY (p["battery"]): the unit logs its
own passes to RAM with USB unplugged and they are read back afterwards -- see
bh_battery.py for why that is the only trustworthy way to tune.

Everything lands in a run folder under Runs/:
    run_info.txt       instruments, settings, timing
    config_before.csv  the BlinkyHawk's full !CFG at the start
    passes.csv         every $DET pass, tagged with its condition
    flashes.csv        every LED flash the sensor saw, with its decoded colour
    battery.csv        (battery-level runs) the electronic load's V / I every 2 s
    levels.csv         (battery-level runs) one row per level: resting V before
                       and after the tests, charge drained, pass/fail counts
    summary.csv        one row per condition (bh_tuner.summarize)
    scope.csv          Capture-Scope.ps1 layout -> ..\\Scope\\Plot-Capture.ps1 renders it
    tuning_report.txt  (auto-tune only) the full reasoning, verdicts, proposal
"""

import csv
import os
import queue
import re
import subprocess
import threading
import time
from dataclasses import dataclass
from datetime import datetime

import bh_tuner as T
from bh_battery import BatteryBench
from bh_instruments import JIG_LOADS, fmt_ohms, write_scope_csv, InstrumentError
from bh_led import LedDecoder

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(HERE, "Runs")
PLOT_PS1 = os.path.normpath(os.path.join(HERE, "..", "Scope", "Plot-Capture.ps1"))


class Aborted(Exception):
    pass


# ---------------------------------------------------------------------------
# BlinkyHawk access from a worker thread
# ---------------------------------------------------------------------------
class TeeQueue(queue.Queue):
    """The GUI's line queue, with extra listeners fed from the reader thread.

    The GUI keeps consuming it as before; a worker that subscribes gets its own
    copy of every line immediately, without waiting on the Tk loop.
    """

    def __init__(self):
        super().__init__()
        self._subs = []
        self._sl = threading.Lock()

    def put(self, item, block=True, timeout=None):
        super().put(item, block, timeout)
        if item and item[0] == "line":
            with self._sl:
                subs = list(self._subs)
            for q in subs:
                q.put(item[1])

    def subscribe(self):
        q = queue.Queue()
        with self._sl:
            self._subs.append(q)
        return q

    def unsubscribe(self, q):
        with self._sl:
            if q in self._subs:
                self._subs.remove(q)


class DeviceClient:
    def __init__(self, get_link, tee, stop_event):
        self.get_link = get_link     # callable -> the GUI's current serial manager
        self.tee = tee
        self.stop = stop_event
        self.wlock = threading.Lock()

    def send(self, cmd):
        link = self.get_link()
        if link is None or not link.is_open:
            raise InstrumentError("BlinkyHawk is not connected")
        with self.wlock:
            link.send(cmd)

    def request(self, cmd, match, timeout=3.0):
        q = self.tee.subscribe()
        try:
            self.send(cmd)
            end = time.time() + timeout
            while time.time() < end:
                try:
                    line = q.get(timeout=0.1)
                except queue.Empty:
                    continue
                if line.startswith("$ERR"):
                    raise InstrumentError(f"BlinkyHawk refused {cmd}: {line}")
                if match(line):
                    return line
            raise InstrumentError(f"BlinkyHawk did not answer {cmd}")
        finally:
            self.tee.unsubscribe(q)

    def get_cfg(self):
        q = self.tee.subscribe()
        cfg = {}
        try:
            self.send("!CFG")
            end = time.time() + 5
            while time.time() < end:
                try:
                    line = q.get(timeout=0.1)
                except queue.Empty:
                    continue
                if line.startswith("$CFG,"):
                    f = line.split(",")
                    if len(f) >= 3:
                        cfg[f[1]] = f[2]
                elif line.startswith("$CFGEND"):
                    return cfg
            raise InstrumentError("no $CFGEND from BlinkyHawk")
        finally:
            self.tee.unsubscribe(q)

    def status(self):
        line = self.request("!STATUS", lambda l: l.startswith("$STATUS"))
        return dict(t.split("=", 1) for t in line.split(",")[1:] if "=" in t)

    def set(self, key, value):
        """!SET and return the echoed (possibly clamped) value."""
        val = f"{value:.5g}" if isinstance(value, float) else str(value)
        line = self.request(f"!SET,{key},{val}",
                            lambda l: l.startswith(f"$CFG,{key},"))
        return line.split(",")[2]

    def prepare(self):
        """Normal (non-diagnostic) mode so the loop runs at its real LOOPMS pace --
        diag mode loops every ~1 ms, which changes how long the bridge rests
        between tests and so changes the metric.  Sleep is not a risk: the
        firmware never sleeps while the USB port is open."""
        self.send("!DIAG,0")
        self.send("!VMODE,0")
        self.send("!MOSFET,-1")
        self.request("!DETLOG,1", lambda l: l.startswith("$OK,detlog"))

    def release(self):
        try:
            self.send("!DETLOG,0")
            self.send("!VMODE,0")
        except Exception:
            pass

    def collect(self, seconds, min_passes=0):
        """Record $DET passes for `seconds` (extended until min_passes arrive)."""
        q = self.tee.subscribe()
        out = []
        try:
            t0 = time.time()
            hard_end = t0 + max(seconds * 4, seconds + 10)
            while True:
                if self.stop.is_set():
                    raise Aborted()
                now = time.time()
                if now - t0 >= seconds and len(out) >= min_passes:
                    break
                if now > hard_end:
                    break
                try:
                    line = q.get(timeout=0.1)
                except queue.Empty:
                    continue
                p = T.parse_det(line) if line.startswith("$DET") else None
                if p:
                    out.append(p)
        finally:
            self.tee.unsubscribe(q)
        if not out:
            raise InstrumentError("no $DET lines -- is this BlinkyHawk_Unified (or Bench) "
                                  "firmware with !DETLOG?")
        return out


# ---------------------------------------------------------------------------
# conditions
# ---------------------------------------------------------------------------
@dataclass
class Cond:
    label: str
    kind: str                 # load | dc | ac | fgen_off
    relay: str                # relay-jig mode
    value: float = 0.0        # dc volts, or ac volts RMS
    freq: float = 60.0
    ohms: object = None       # loads only; None = open
    expected: object = None   # 'C' 'F' 'V' '!V' or None

    def describe(self):
        if self.kind == "load":
            return f"relay load {self.relay} ({fmt_ohms(self.ohms)})"
        if self.kind == "dc":
            return f"generator DC {self.value:+.3f} V"
        if self.kind == "ac":
            return f"generator sine {self.value:g} Vrms {self.freq:g} Hz"
        return "generator path, output OFF"


def expected_for_load(ohms, closed_max, open_min):
    r = float("inf") if ohms is None else ohms
    if r <= closed_max:
        return "C"
    if r >= open_min:
        return "F"
    return None


def load_conditions(names, closed_max, open_min):
    out = []
    for name, ohms in JIG_LOADS:
        if name in names:
            out.append(Cond(f"load {name}", "load", name, ohms=ohms,
                            expected=expected_for_load(ohms, closed_max, open_min)))
    return out


def wake_loads(loads, closed_max, open_min):
    """(stay, wake) for the sleep/wake check: the OPEN-side load nearest the
    boundary (must NOT wake a sleeping unit) and the CLOSED-side load nearest
    it (must).  Either may be None."""
    r = lambda c: float("inf") if c.ohms is None else c.ohms
    opn = sorted((c for c in loads if r(c) >= open_min), key=r)
    cls = sorted((c for c in loads if r(c) <= closed_max), key=r)
    return (opn[0] if opn else None), (cls[-1] if cls else None)


def dc_condition(v, vt=None):
    """Above the trip (+10 %) the unit must alert AND name the polarity."""
    exp = None
    if vt:
        if abs(v) >= vt * 1.1:
            exp = "VDC+" if v > 0 else "VDC-"
        elif abs(v) <= vt * 0.9:
            exp = "!V"
    return Cond(f"DC {v:+.3f}V", "dc", "FGEN", value=v, expected=exp)


def ac_condition(vrms, freq):
    return Cond(f"AC {vrms:g}Vrms {freq:g}Hz", "ac", "FGEN", value=vrms, freq=freq,
                expected="VAC")


def parse_levels(text):
    """'0.8,-0.8'  or  '-1.2:1.2:0.1' (start:stop:step)  or a mix, comma-separated."""
    out = []
    for tok in re.split(r"[,\s]+", text.strip()):
        if not tok:
            continue
        if ":" in tok:
            a, b, s = (float(x) for x in tok.split(":"))
            s = abs(s) if b >= a else -abs(s)
            n = int(round((b - a) / s))
            out.extend(round(a + i * s, 6) for i in range(n + 1))
        else:
            out.append(float(tok))
    seen, res = set(), []
    for v in out:
        if v not in seen:
            seen.add(v)
            res.append(v)
    return res


class ManualRelay:
    """Stand-in when the relay jig is not connected: asks the operator."""
    name = "Relay (manual)"
    connected = True

    def __init__(self, prompt):
        self.prompt = prompt       # callable(text) -> bool (False = abort)
        self.mode = None

    def set_mode(self, mode, timeout=None):
        if mode == self.mode:
            return
        desc = "the function generator" if mode == "FGEN" else \
            f"the {mode} load ({fmt_ohms(dict(JIG_LOADS).get(mode))})"
        if not self.prompt(f"Connect the BlinkyHawk leads to {desc}, then press OK."):
            raise Aborted()
        self.mode = mode


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------
class Runner:
    def __init__(self, device, relay, fgen, scope, emit, stop_event, use_led=True,
                 decoder=None):
        """device None = the BlinkyHawk has no serial link (battery): LED only."""
        self.dev, self.relay, self.fgen, self.scope = device, relay, fgen, scope
        self.emit = emit             # callable(kind, payload) -> GUI queue
        self.stop = stop_event
        self.led = relay if (use_led and getattr(relay, "has_led", False)) else None
        self.decoder = decoder or LedDecoder()
        self.dir = None
        self.flash_rows = []
        self.pass_rows = []
        self.summaries = []
        self.scope_conds = []
        self.scope_note = ""
        self.started = None
        self.load = None             # RigolDL3000 / SimLoad (battery-level runs)
        self.usb_auto = False        # switch the DUT's USB with the jig's K8 relay
        self.quiet_beeps = True      # BEEP=0 (RAM) for the length of a job
        self._beep_orig = None       # BEEP as the job found it, while silenced
        self._usb_orig = None        # USB state before this job first moved it
        self.prompt = None           # callable(text) -> bool, set by the GUI
        self.batt_rows = []          # [t_s, phase, V, A, load state]
        self.level_rows = []
        self._batt_phase = ""

    # ---------- plumbing ----------
    def log(self, text):
        self.emit("log", text)

    def check_stop(self):
        if self.stop.is_set():
            raise Aborted()

    def sleep(self, s):
        end = time.time() + s
        while time.time() < end:
            self.check_stop()
            time.sleep(0.05)

    def open_run(self, kind, note):
        self.started = datetime.now()
        safe = re.sub(r"[^\w\-. ]", "", note or "").strip().replace(" ", "_")[:40]
        name = self.started.strftime("%Y-%m-%d_%H%M%S") + f"_{kind}" + (f"_{safe}" if safe else "")
        self.dir = os.path.join(RUNS_DIR, name)
        os.makedirs(self.dir, exist_ok=True)
        self.scope_note = note or kind
        return self.dir

    def fgen_off(self):
        if self.fgen is not None:
            try:
                self.fgen.output(False)
            except Exception as exc:
                self.log(f"!! could not turn the generator off: {exc}")

    def apply(self, cond, settle_s):
        self.check_stop()
        needs_fgen = cond.kind in ("dc", "ac")
        if needs_fgen and self.fgen is None:
            raise InstrumentError("needs the function generator, which is not connected")
        self.fgen_off()
        self.relay.set_mode(cond.relay)
        if cond.kind == "dc":
            self.fgen.apply_dc(cond.value)
            self.fgen.output(True)
        elif cond.kind == "ac":
            self.fgen.apply_sine(cond.value, cond.freq)
            self.fgen.output(True)
        self.sleep(settle_s)

    def measure(self, cond, phase, dwell_s, settle_s, scope_mode=None, scope_timeout=5.0,
                min_passes=0, stable_count=2):
        """Apply, record, (scope), summarise.  Returns (passes, summary)."""
        self.emit("status", f"{phase}: {cond.label}")
        self.apply(cond, settle_s)
        flashes, passes = self.record(dwell_s, min_passes)
        cap = None
        if scope_mode and self.scope is not None:
            try:
                cap = self.scope.capture(scope_mode, scope_timeout, self.stop)
                if cap is None:
                    self.log(f"   scope: no trigger within {scope_timeout:g} s")
            except Exception as exc:
                self.log(f"   scope: {exc}")
        self.fgen_off()

        for i, p in enumerate(passes):
            self.pass_rows.append([phase, cond.label, i] + [p[k] for k in T.DET_FIELDS])
        led = None
        if self.led is not None:
            led = self.decoder.decode(flashes)
            t0 = flashes[0].t if flashes else 0
            for f in flashes:
                self.flash_rows.append([phase, cond.label, f"{f.t - t0:.3f}", f.dur_ms, f.n,
                                        f"{f.fz:.1f}", f"{f.fy:.1f}", f"{f.fxl:.1f}",
                                        f"{f.vis:.1f}", f"{f.peak:.1f}", int(f.sat),
                                        f.colour, f"{f.cos:.3f}"])
        s = T.summarize(cond.label, cond.expected, passes, stable_count, led)
        s["phase"] = phase
        self.summaries.append(s)
        if cap:
            tdiv, trig, waves = cap
            st = "; ".join(f"{w.channel} mean {w.stats()['mean']:+.4g} "
                           f"acrms {w.stats()['acrms']:.4g} pkpk {w.stats()['pkpk']:.4g}"
                           for w in waves)
            s["scope"] = st
            self.scope_conds.append({"label": f"{phase} {cond.label}", "note": cond.describe(),
                                     "time": datetime.now().strftime("%H:%M:%S"),
                                     "tdiv": tdiv, "trigger": trig, "waves": waves})
            self.log(f"   scope: {st}")
        self.emit("summary", s)
        parts = [f"   {cond.label:>20}:"]
        if passes:
            parts.append(f"n={s['n']:3d}  lead C/F/V {s['lead_C'] * 100:3.0f}/"
                         f"{s['lead_F'] * 100:3.0f}/{s['lead_V'] * 100:3.0f} %"
                         + (f" {s['kind']}" if s["kind"] and s["lead_V"] else ""))
        if led is not None:
            parts.append(f"LED {led['state']} ({led['colours'] or 'no flashes'})"
                         + ("  SATURATED" if led.get("sat") else ""))
        parts.append(s["result"])
        self.log("  ".join(parts))
        self.write_files()
        return passes, s

    def record(self, dwell_s, min_passes=0):
        """Record for the dwell: $DET passes (if there is a serial link) and LED
        flashes (if the jig has the sensor) over the same window."""
        q = self.led.subscribe_flashes() if self.led is not None else None
        t0 = time.time()
        try:
            if self.dev is not None:
                passes = self.dev.collect(dwell_s, min_passes)
            else:
                passes = []
                self.sleep(dwell_s)
        finally:
            if q is not None:
                self.led.unsubscribe_flashes(q)
        flashes = []
        while q is not None:
            try:
                f = q.get_nowait()
            except queue.Empty:
                break
            if f.t >= t0 - 0.05:          # a flash begun before the window is not ours
                flashes.append(f)
        flashes.sort(key=lambda f: f.t)
        if self.dev is None and self.led is None:
            raise InstrumentError("nothing to record: no serial link and no LED sensor")
        return flashes, passes

    # ---------- files ----------
    def write_files(self):
        if not self.dir:
            return
        with open(os.path.join(self.dir, "passes.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["phase", "condition", "i"] + T.DET_FIELDS)
            w.writerows(self.pass_rows)
        if self.batt_rows:
            with open(os.path.join(self.dir, "battery.csv"), "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t_s", "level", "volts", "amps", "load"])
                w.writerows(list(self.batt_rows))
        if self.level_rows:
            with open(os.path.join(self.dir, "levels.csv"), "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(self.level_rows[0]))
                w.writeheader()
                w.writerows(self.level_rows)
        if self.flash_rows:
            with open(os.path.join(self.dir, "flashes.csv"), "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["phase", "condition", "t_s", "dur_ms", "samples", "fz", "fy",
                            "fxl", "vis", "peak", "saturated", "colour", "cosine"])
                w.writerows(self.flash_rows)
        with open(os.path.join(self.dir, "summary.csv"), "w", newline="") as fh:
            cols = ["phase"] + T.SUMMARY_COLS + ["scope"]
            w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            for s in self.summaries:
                w.writerow({k: (f"{v:.6g}" if isinstance(v, float) else v)
                            for k, v in s.items()})

    def write_scope(self, plot=True):
        if not self.scope_conds:
            return None
        path = os.path.join(self.dir, "scope.csv")
        write_scope_csv(path, getattr(self.scope, "ident", ""),
                        self.started.strftime("%Y-%m-%d %H:%M:%S"),
                        f"BlinkyHawk test suite: {self.scope_note}", self.scope_conds)
        if plot and os.path.exists(PLOT_PS1):
            try:
                subprocess.run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
                                "-File", PLOT_PS1, path, "-NoOpen"],
                               capture_output=True, timeout=120)
            except Exception as exc:
                self.log(f"   (plot not built: {exc})")
        return path

    def write_info(self, lines):
        with open(os.path.join(self.dir, "run_info.txt"), "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")

    def write_cfg(self, cfg, name):
        with open(os.path.join(self.dir, name), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["key", "value"])
            w.writerows(cfg.items())

    def instrument_lines(self):
        return [f"relay:  {getattr(self.relay, 'ident', getattr(self.relay, 'name', '-'))}",
                f"fgen:   {getattr(self.fgen, 'ident', 'not connected')}",
                f"scope:  {getattr(self.scope, 'ident', 'not connected')}",
                f"load:   {getattr(self.load, 'ident', 'not connected')}"]

    # ---------- job 1: the big test ----------
    def run_plan(self, conds, dwell_s, settle_s, scope_mode, scope_timeout, note):
        self.open_run("Test", note)
        self.log(f"== test run -> {self.dir}")
        if self.dev is not None:
            cfg = self.dev.get_cfg()
            self.write_cfg(cfg, "config_before.csv")
            stable = int(float(cfg.get("STABLECOUNT", 2)))
        else:
            stable = 2
            self.log("   no serial link: judging by the LED alone (unit on battery)")
        self.write_info([f"BlinkyHawk test suite -- test run {self.started}", f"note: {note}",
                         *self.instrument_lines(),
                         f"dwell {dwell_s} s, settle {settle_s} s, scope {scope_mode or 'off'}",
                         "judged by: " + " + ".join(
                             x for x, on in (("serial $DET", self.dev is not None),
                                             ("LED sensor", self.led is not None)) if on),
                         "conditions:", *[f"  {c.label}: {c.describe()}, expect "
                                          f"{c.expected or '-'}" for c in conds]])
        if self.dev is not None:
            self.dev.prepare()
        try:
            self.quiet_begin()
            for i, c in enumerate(conds, 1):
                self.emit("progress", (i, len(conds)))
                try:
                    self.measure(c, "test", dwell_s, settle_s, scope_mode, scope_timeout,
                                 stable_count=stable)
                except InstrumentError as exc:
                    self.log(f"   {c.label}: SKIPPED -- {exc}")
        finally:
            self.fgen_off()
            self.quiet_end()
            if self.dev is not None:
                self.dev.release()
            self.write_files()
            self.write_scope()
        fails = [s["label"] for s in self.summaries if s["result"] == "FAIL"]
        self.log(f"== done: {len(self.summaries)} condition(s), "
                 f"{len(fails)} fail{'' if len(fails) == 1 else 's'}"
                 + (f": {', '.join(fails)}" if fails else ""))
        return self.dir

    # ---------- silence the buzzer for the run ----------
    def quiet_begin(self):
        """BEEP=0 in RAM for the whole job -- a test run otherwise chirps on every
        condition change.  Called AFTER the job has snapshotted the config, so
        config_before.csv (and anything reverted from it) keeps the unit's real
        BEEP.  RAM only: a reset of the board brings the beeps back by itself."""
        if not self.quiet_beeps or self.dev is None or self._beep_orig is not None:
            return
        try:
            v = self.dev.get_cfg().get("BEEP")
            if v is None:
                self.log("   (this firmware has no BEEP key -- beeps not silenced)")
                return
            self._beep_orig = v
            if v.strip() not in ("0", "0.0"):
                self.dev.set("BEEP", 0)
                self.log("   beeps silenced for the run (BEEP=0, RAM only)")
        except Exception as exc:
            self._beep_orig = None
            self.log(f"   !! could not silence the beeps: {exc}")

    def quiet_end(self):
        """Put BEEP back.  Runs from each job's cleanup, so a stopped or failed
        run restores it too -- as long as the board is still reachable."""
        if self._beep_orig is None:
            return
        orig, self._beep_orig = self._beep_orig, None
        if orig.strip() in ("0", "0.0"):
            return                            # it was already off: nothing to undo
        try:
            self.dev.set("BEEP", orig)
            self.log("   beeps re-enabled (BEEP restored)")
        except Exception as exc:
            self.log(f"   !! could not re-enable the beeps ({exc}) -- BEEP is still 0 in "
                     "RAM: !SET,BEEP,1, '!LOAD', or a power cycle brings them back")

    # ---------- the BlinkyHawk's USB (relay jig K8) ----------
    def usb_auto_ok(self):
        return self.usb_auto and getattr(self.relay, "has_usb", False)

    def usb_set(self, on):
        if self._usb_orig is None:
            self._usb_orig = getattr(self.relay, "usb_state", True)
        self.relay.usb(on)
        self.log(f"   USB relay: BlinkyHawk USB {'connected' if on else 'DISCONNECTED'}")

    def usb_restore(self):
        """Put the USB back the way the job found it.  Returns True if it was
        reconnected (the GUI then reconnects the serial port)."""
        if self._usb_orig is None or not getattr(self.relay, "has_usb", False):
            return False
        try:
            if self.relay.usb_state != self._usb_orig:
                self.relay.usb(self._usb_orig)
                self.log(f"   USB relay: restored to "
                         f"{'connected' if self._usb_orig else 'disconnected'}")
                return bool(self._usb_orig)
        except Exception as exc:
            self.log(f"!! could not restore the USB relay: {exc}")
        return False

    # ---------- battery drain (electronic load) ----------
    def _batt_sampler(self, stop_ev):
        """Log the load's V / I every 2 s for the whole run (battery.csv), and
        integrate the charge the load has taken."""
        t0 = time.time()
        while not stop_ev.wait(2.0):
            try:
                v, a = self.load.volts(), self.load.amps()
            except Exception:
                continue
            self.batt_rows.append([round(time.time() - t0, 1), self._batt_phase,
                                   round(v, 4), round(a, 4), self.load.state])
            self.emit("battery", (v, a, self.load.state))

    def _drained_mah(self, phase):
        rows = [r for r in self.batt_rows if r[1] == phase]
        return sum(r[3] for r in rows) * 2.0 / 3.6      # A x 2 s -> mAh

    def rest_ocv(self, rest_s, min_s=5.0):
        """Load off, then wait for the cell to stop recovering: less than 1 mV
        change over 10 s, or rest_s, whichever first.  Li-ion keeps creeping up
        for minutes after a load comes off, so a reading taken the moment the
        load stops is several tens of mV low."""
        self.load.input(False)
        hist = []
        t0 = time.time()
        while True:
            self.check_stop()
            v = self.load.volts()
            hist.append((time.time(), v))
            el = time.time() - t0
            old = [x for t, x in hist if t <= hist[-1][0] - 10.0]
            if el >= rest_s or (el >= min_s and old and abs(v - old[-1]) < 0.001):
                return v
            self.sleep(1.0)

    def drain_to(self, target, amps, tol, rest_s, floor, max_s):
        """Drain until the RESTED voltage is within `tol` of `target`.

        Loaded voltage sits below the resting voltage by I x R, and the cell
        recovers when the load comes off, so the loop is: estimate R from the
        step when the load goes on, drain until (loaded V + I x R) reaches the
        target, rest, measure, and repeat.  Each round removes less; it stops
        at the first rest inside the band.  Returns the rested voltage.
        """
        L = self.load
        if target - tol <= floor:
            raise InstrumentError(f"target {target:g} V is too close to the {floor:g} V floor")
        v = self.rest_ocv(min(rest_s, 10.0))
        if v <= target + tol:
            self.log(f"   battery rests at {v:.3f} V -- already at or below {target:g} V, "
                     "no drain")
            return v
        L.set_cc(amps, floor)
        t_start = time.time()
        try:
            for rnd in range(1, 13):
                L.input(True)
                self.sleep(1.5)
                v_on, i = L.volts(), L.amps()
                if i < 0.5 * amps:
                    raise InstrumentError(
                        f"the load is not sinking ({i * 1000:.0f} mA at {v_on:.3f} V).  Check "
                        "the wiring, that Von Latch is OFF, and that the battery is above "
                        f"the {floor:g} V floor")
                r = max(0.0, (v - v_on) / i)
                self.log(f"   drain round {rnd}: rested {v:.3f} V -> {v_on:.3f} V at "
                         f"{i * 1000:.0f} mA (~{r * 1000:.0f} mohm); draining to an "
                         f"estimated rest of {target:g} V")
                while True:
                    vl, il = L.volts(), L.amps()
                    self.emit("status", f"{self._batt_phase}: draining, {vl:.3f} V at "
                                        f"{il * 1000:.0f} mA")
                    if vl <= floor + 0.02:
                        self.log(f"   !! reached the {floor:g} V floor under load -- stopping")
                        break
                    if time.time() - t_start > max_s:
                        raise InstrumentError(f"drain took longer than {max_s:g} s")
                    if vl + il * r <= target:
                        break
                    self.sleep(1.0)
                L.input(False)
                self.emit("status", f"{self._batt_phase}: resting")
                v = self.rest_ocv(rest_s)
                self.log(f"   rested at {v:.3f} V")
                if v <= target + tol:
                    return v
            raise InstrumentError("drain did not settle inside the band in 12 rounds")
        finally:
            L.input(False)

    # ---------- job 4: the whole plan at each battery level ----------
    def run_battery_levels(self, p):
        """p: levels [float|None] (None = as found), conds, dwell, settle, amps,
        tol, rest_s, floor, max_drain_s, judge 'led'|'log', samples, host, note.

        USB must be unplugged the whole time the battery is being measured or
        drained -- with it in, VBUS charges the battery and the drain fights the
        charger.  In 'log' mode it has to go back in to arm each capture and read
        it back (a few seconds of charging per level, which is why the resting
        voltage is measured again right before the conditions run)."""
        if self.load is None:
            raise InstrumentError("battery levels need the electronic load")
        judge = p["judge"]
        host = p.get("host")
        self.open_run("BattLevels", p.get("note", ""))
        self.log(f"== battery-level run -> {self.dir}")
        stable = 2
        if judge == "log":
            if self.dev is None:
                raise InstrumentError("the battery-log mode arms each capture over USB -- "
                                      "connect the BlinkyHawk first")
            from bh_battery import BatteryBench
            self.bat = BatteryBench(self, host)
            cfg = self.dev.get_cfg()
            self.write_cfg(cfg, "config_before.csv")
            stable = int(float(cfg.get("STABLECOUNT", 2)))
            self.quiet_begin()
        elif self.quiet_beeps:
            self.log("   (no serial link in LED-only mode, so the beeps cannot be silenced)")
        levels = p["levels"]
        self.write_info([f"BlinkyHawk test suite -- battery-level run {self.started}",
                         f"note: {p.get('note', '')}", *self.instrument_lines(),
                         "levels: " + ", ".join("as found" if l is None else f"{l:g} V"
                                                for l in levels),
                         f"drain {p['amps'] * 1000:.0f} mA, band +/-{p['tol'] * 1000:.0f} mV, "
                         f"rest up to {p['rest_s']:g} s, floor {p['floor']:g} V (also set as "
                         "the load's Von)",
                         "judged by: " + ("the unit's own battery log (USB replugged to read "
                                          "it at each level)" if judge == "log" else
                                          "the LED watcher only (USB out the whole run)"),
                         f"dwell {p['dwell']} s, settle {p['settle']} s",
                         "conditions:", *[f"  {c.label}: {c.describe()}, expect "
                                          f"{c.expected or '-'}" for c in p["conds"]]])

        def unplug(why):
            if host is not None and host.port:
                if self.bat is None:
                    from bh_battery import BatteryBench
                    self.bat = BatteryBench(self, host)
                self.bat.unplugged(why)
            elif self.usb_auto_ok():
                self.usb_set(False)          # no COM port known to confirm it by
                self.sleep(2.0)
            elif self.prompt and not self.prompt(why + "\n\nPress OK once it is unplugged."):
                raise Aborted()

        stop_ev = threading.Event()
        sampler = threading.Thread(target=self._batt_sampler, args=(stop_ev,), daemon=True)
        self.bat = getattr(self, "bat", None)
        try:
            self.load.input(False)
            sampler.start()
            unplug("UNPLUG the BlinkyHawk's USB now -- it must run on its battery for the "
                   "whole level run (USB would charge it).")
            for li, target in enumerate(levels, 1):
                tag = "as found" if target is None else f"{target:g}V"
                self._batt_phase = f"L{li} {tag}"
                self.emit("stage", f"level {li}/{len(levels)}: {tag}")
                self.log(f"-- level {li}/{len(levels)}: {tag}")
                if target is None:
                    v0 = self.rest_ocv(min(p["rest_s"], 10.0))
                else:
                    v0 = self.drain_to(target, p["amps"], p["tol"], p["rest_s"],
                                       p["floor"], p["max_drain_s"])
                drained = self._drained_mah(self._batt_phase)
                n_before = len(self.summaries)
                phase = f"{self._batt_phase} ({v0:.2f}V)"
                if judge == "log":
                    self.bat.plugged("Plug the BlinkyHawk's USB back in to arm this level's "
                                     "capture.")
                    self.bat_measure(p["conds"], phase, p["dwell"], p["settle"],
                                     p["samples"], 0, stable)
                    # the capture ends with USB plugged in: out again before draining
                    if li < len(levels):
                        unplug("UNPLUG the BlinkyHawk's USB again for the next drain.")
                else:
                    for i, c in enumerate(p["conds"], 1):
                        self.emit("progress", (i, len(p["conds"])))
                        try:
                            self.measure(c, phase, p["dwell"], p["settle"], stable_count=stable)
                        except InstrumentError as exc:
                            self.log(f"   {c.label}: SKIPPED -- {exc}")
                self.fgen_off()
                v1 = self.load.volts()
                res = [s["result"] for s in self.summaries[n_before:]]
                row = {"level": tag, "rest_before_V": round(v0, 4),
                       "after_tests_V": round(v1, 4), "drained_mAh": round(drained, 1),
                       "pass": res.count("PASS"), "fail": res.count("FAIL"),
                       "no_data": sum(1 for r in res if r not in ("PASS", "FAIL"))}
                self.level_rows.append(row)
                self.emit("level", row)
                self.log(f"   level {tag}: rested {v0:.3f} V before, {v1:.3f} V after; "
                         f"{row['pass']} pass, {row['fail']} fail, {row['no_data']} no data")
                self.write_files()
        finally:
            try:
                self.load.input(False)
            except Exception as exc:
                self.log(f"!! could not turn the load input off: {exc} -- TURN IT OFF BY HAND")
            stop_ev.set()
            sampler.join(timeout=3)
            self.fgen_off()
            self.quiet_end()
            self.write_files()
        self.log("== battery levels done:")
        for r in self.level_rows:
            self.log(f"   {r['level']:>9}: {r['rest_before_V']:.3f} V  "
                     f"{r['pass']} pass / {r['fail']} fail / {r['no_data']} no data")
        return self.dir

    # ---------- battery captures (see bh_battery.py) ----------
    def bat_measure(self, conds, phase, dwell, settle, samples, vmode, stable):
        """Battery twin of measure() for a whole list: one or more unplug
        cycles, then the same summaries / passes.csv rows.  -> {label: passes}"""
        res = self.bat.capture(conds, dwell, settle, samples, vmode=vmode, phase=phase)
        for c in conds:
            passes = res.get(c.label, [])
            for i, p in enumerate(passes):
                self.pass_rows.append([phase, c.label, i] + [p.get(k) for k in T.DET_FIELDS])
            s = T.summarize(c.label, c.expected, passes, stable, None, skip=0)
            s["phase"] = phase
            self.summaries.append(s)
            self.emit("summary", s)
            self.log(f"   {c.label:>20}:  n={s['n']:3d}  lead C/F/V {s['lead_C'] * 100:3.0f}/"
                     f"{s['lead_F'] * 100:3.0f}/{s['lead_V'] * 100:3.0f} %"
                     + (f" {s['kind']}" if s["kind"] and s["lead_V"] else "")
                     + f"  {s['result']}")
        self.write_files()
        return res

    # ---------- job 2: auto-tune ----------
    def run_autotune(self, p):
        """p: dict of the Auto-Tune tab's settings (see the GUI)."""
        self.open_run("AutoTune", p.get("note", ""))
        self.log(f"== auto-tune -> {self.dir}")
        original = self.dev.get_cfg()
        self.write_cfg(original, "config_before.csv")
        stable = int(float(original.get("STABLECOUNT", 2)))
        fast = float(original.get("VOLTFAST", 5))
        st = self.dev.status()
        slot = int(st.get("dip", original.get("THRESHSEL", 3)))
        thresh_key = "THRESH" + format(slot, "02b")
        N, dwell, settle = p["passes"], p["dwell"], p["settle"]
        battery = bool(p.get("battery"))
        if battery:
            self.bat = BatteryBench(self, p["host"])
            dwell, settle, N = p["bat_dwell"], p["bat_settle"], p["bat_samples"]
        report = [f"BlinkyHawk auto-tune  {self.started:%Y-%m-%d %H:%M:%S}",
                  ("MODE: ON BATTERY, floating -- passes read back from the unit's RAM log "
                   "after each unplug cycle" if battery else
                   "MODE: over USB -- WARNING: USB earths the board, and numbers taken this "
                   "way do not hold on battery (see README)"),
                  *self.instrument_lines(),
                  f"target trip +/-{p['vt']} V; CLOSED <= {fmt_ohms(p['closed_max'])}, "
                  f"OPEN >= {fmt_ohms(p['open_min'] if p['open_min'] != float('inf') else None)}",
                  f"active threshold slot: {thresh_key} (dip={slot}); STABLECOUNT {stable}",
                  f"{N} passes minimum per condition, {dwell} s dwell, {settle} s settle", ""]
        applied = {}
        touched = set()          # every key this run has changed, applied or not
        ok = False

        def setk(k, v):
            touched.add(k)
            return self.dev.set(k, v)
        self.write_info(report)
        if not battery:
            self.dev.prepare()
        loads = load_conditions(p["loads"], p["closed_max"], p["open_min"])
        try:
            self.quiet_begin()                # after `original` was snapshotted
            # ---- 1. voltage ----
            if p["do_voltage"]:
                if self.fgen is None:
                    report += ["VOLTAGE: skipped -- needs the function generator.", ""]
                    self.log("voltage tuning skipped: no function generator")
                else:
                    self.emit("stage", "Voltage: characterising")
                    novolt, dc, ac = {}, {}, {}
                    levels = sorted(set(p["dc_levels"]) | {0.0, p["vt"], -p["vt"]})
                    if battery:
                        dcc = {v: dc_condition(v) for v in levels}
                        acc = [ac_condition(vrms, f) for vrms, f in p["ac"]]
                        res = self.bat_measure(loads + list(dcc.values()) + acc, "volt",
                                               dwell, settle, N, 0, stable)
                        novolt = {c.relay: res.get(c.label, []) for c in loads}
                        dc = {v: res.get(c.label, []) for v, c in dcc.items()}
                        ac = {c.label: res.get(c.label, []) for c in acc}
                    for c in (loads if not battery else []):
                        novolt[c.relay], _ = self.measure(c, "volt", dwell, settle,
                                                          min_passes=N, stable_count=stable)
                    for v in (levels if not battery else []):
                        dc[v], _ = self.measure(dc_condition(v), "volt", dwell, settle,
                                                min_passes=N, stable_count=stable)
                    for vrms, f in (p["ac"] if not battery else []):
                        c = ac_condition(vrms, f)
                        ac[c.label], _ = self.measure(c, "volt", dwell * 2, settle,
                                                      min_passes=2 * N, stable_count=stable)
                    rv = T.tune_voltage(dc, novolt, ac, p["vt"], fast, stable, p["guard"])
                    report += rv["lines"] + [f"   warning: {w}" for w in rv["warnings"]] + [""]
                    self.emit("tune_voltage", rv)
                    for k, v in rv["proposal"].items():
                        applied[k] = setk(k, v)
                    if rv["proposal"]:
                        self.log("   applied (RAM): " + ", ".join(
                            f"{k}={v}" for k, v in rv["proposal"].items()))

            # ---- 2. open / closed ----
            if p["do_threshold"]:
                self.emit("stage", "Open/closed: characterising")
                if not battery:
                    self.dev.send("!VMODE,2")   # voltage check off: every pass runs the test
                bands = p["detbands"]
                if battery and len(bands) > 1:
                    # Each group is its own unplug cycle on battery.  The tail area
                    # is identical in every group (it does not depend on DETBAND),
                    # and time-to-return is the metric that fails on battery, so
                    # one band is enough to decide between them.
                    report.append(f"   battery mode: DETBAND {bands[0]:g} only (each extra "
                                  "candidate would cost another unplug cycle)")
                    bands = bands[:1]
                groups = [({"DETMETHOD": 2, "DETBAND": b}, b) for b in bands]
                if p["include_m0"]:
                    groups.append(({"DETMETHOD": 0}, None))
                cands = []
                for gi, (settings, band) in enumerate(groups):
                    for k, v in settings.items():
                        setk(k, v)
                    rec = []
                    if battery:     # VMODE 2 is set for the capture by bh_battery
                        res = self.bat_measure(loads, f"thr{gi}", dwell, settle, N, 2, stable)
                        rec = [(c.relay, c.ohms, res.get(c.label, [])) for c in loads]
                    for c in (loads if not battery else []):
                        passes, _ = self.measure(c, f"thr{gi}", dwell, settle,
                                                 min_passes=N, stable_count=stable)
                        rec.append((c.relay, c.ohms, passes))
                    if settings["DETMETHOD"] == 0:
                        cands.append({"label": "single |diff| (method 0)", "method": 0,
                                      "settings": {},
                                      "data": [(l, o, [x["metric"] for x in ps])
                                               for l, o, ps in rec]})
                        continue
                    cands.append({"label": f"time-to-return, DETBAND {band:g}", "method": 1,
                                  "settings": {"DETBAND": band},
                                  "data": [(l, o, [x["retms"] for x in ps]) for l, o, ps in rec]})
                    if gi == 0:     # area does not depend on DETBAND: record it once
                        cands.append({"label": "tail area (method 2)", "method": 2,
                                      "settings": {},
                                      "data": [(l, o, [x["area"] for x in ps])
                                               for l, o, ps in rec]})
                self.dev.send("!VMODE,0")
                # Back to the unit's own method before applying the winner, so a
                # winner that does not name DETBAND does not inherit the last
                # group's value.
                for k in ("DETMETHOD", "DETBAND"):
                    self.dev.set(k, original[k])
                rt = T.tune_threshold(cands, p["closed_max"], p["open_min"])
                report += rt["lines"] + [""]
                self.emit("tune_threshold", rt)
                if rt.get("threshold") is not None:
                    for k, v in rt["proposal"].items():
                        applied[k] = setk(k, v)
                    applied[thresh_key] = setk(thresh_key, rt["threshold"])
                    if battery:
                        # Measured with the unit awake; the sleeping probe reads a
                        # little higher (~+0.01 V*ms on V3b), which the wake check
                        # below confirms rather than assumes.
                        skey = "SLEEP" + thresh_key.replace("THRESH", "THR")
                        applied[skey] = setk(skey, rt["threshold"])
                        report.append(f"   {skey} set to the same value -- the sleep/wake "
                                      "check in VERIFY is what proves it wakes.")

            # ---- 3. verify with everything applied ----
            if p["do_verify"]:
                self.emit("stage", "Verify")
                report.append("VERIFY -- the tuned settings, run for real (lead state after "
                              "STABLECOUNT debounce):")
                conds = list(loads)
                if self.fgen is not None:
                    vt = p["vt"]
                    for v in sorted({vt * 1.15, -vt * 1.15, vt * 0.85, -vt * 0.85,
                                     *p["verify_dc"]}):
                        conds.append(dc_condition(round(v, 4), vt))
                    conds += [ac_condition(vrms, f) for vrms, f in p["ac"]]
                    conds.append(Cond("gen output off", "fgen_off", "FGEN", expected="F"))
                vs = []
                n0 = len(self.summaries)
                if battery:
                    self.bat_measure(conds, "verify", dwell, settle, N, 0, stable)
                    vs = self.summaries[n0:]
                for c in (conds if not battery else []):
                    try:
                        _, s = self.measure(c, "verify", dwell, settle, min_passes=N,
                                            stable_count=stable)
                        vs.append(s)
                    except InstrumentError as exc:
                        self.log(f"   {c.label}: SKIPPED -- {exc}")
                for s in vs:
                    report.append(f"   {s['label']:>22}: expect {s['expected'] or '-':>2}  "
                                  f"C/F/V {s['lead_C'] * 100:5.1f}/{s['lead_F'] * 100:5.1f}/"
                                  f"{s['lead_V'] * 100:5.1f} %   {s['result']}")
                fails = [s for s in vs if s["result"] == "FAIL"]
                wake_ok = True
                if battery and p.get("wake_check", True):
                    stay, wake = wake_loads(loads, p["closed_max"], p["open_min"])
                    if stay is None or wake is None:
                        report.append("   sleep/wake check skipped: needs a load on each "
                                      "side of the CLOSED/OPEN criteria")
                    else:
                        self.emit("stage", "Verify: sleep/wake")
                        wl, wake_ok = self.bat.wake_check(stay, wake)
                        report += [""] + wl
                        for l in wl:
                            self.log(l)
                ok = not fails and wake_ok
                report += ["", "VERIFY VERDICT: " + (
                    "every condition read as expected" if ok else
                    "; ".join(x for x in (
                        f"{len(fails)} condition(s) failed: " + ", ".join(s["label"] for s in fails)
                        if fails else "", "" if wake_ok else "sleep/wake check failed") if x))]
            report += ["", "Applied to device RAM (NOT saved): " +
                       (", ".join(f"{k}={v}" for k, v in applied.items()) or "nothing"),
                       "Press 'Save to EEPROM' to keep them, or 'Revert' to put back "
                       "config_before.csv."]
        except Aborted:
            report += ["", "ABORTED -- reverting every changed key to its starting value."]
            self.log("aborted: reverting the device config")
            self.revert(original, touched | set(applied))
            applied = {}
            raise
        finally:
            self.fgen_off()
            self.quiet_end()
            try:
                self.dev.release()
            except Exception:
                pass
            if battery:
                p["host"].notice(None)
            self.write_files()
            with open(os.path.join(self.dir, "tuning_report.txt"), "w", encoding="utf-8") as fh:
                fh.write("\n".join(report) + "\n")
            self.emit("report", "\n".join(report))
        self.emit("tune_done", {"original": original, "applied": applied, "ok": ok,
                                "dir": self.dir})
        return self.dir

    # ---------- job 3: teach the LED decoder the three colours ----------
    def run_led_calibration(self, dc_volts, dwell_s, settle_s):
        """OPEN -> the float alert is blue; SHORT -> closed is green; a DC level
        well over the trip -> VDC+ is red-red.  Each is recorded and becomes
        that colour's reference.  Needs the unit to be alerting normally (LED
        on, not locked out by charging)."""
        if self.led is None:
            raise InstrumentError("the relay jig has no LED sensor (jig firmware 1.1 + AS7343)")
        self.open_run("LedCal", "")
        plan = [("B", Cond("cal OPEN", "load", "OPEN", ohms=None, expected="F")),
                ("G", Cond("cal SHORT", "load", "SHORT", ohms=0.0, expected="C"))]
        if self.fgen is not None:
            plan.append(("R", Cond(f"cal DC {dc_volts:+g}V", "dc", "FGEN", value=dc_volts,
                                   expected="VDC+")))
        else:
            self.log("   no generator: red cannot be calibrated (needs a VDC+ alert)")
        lines = []
        try:
            if self.dev is not None:
                self.dev.prepare()
            self.quiet_begin()
            for colour, c in plan:
                self.emit("status", f"LED calibration: {c.label}")
                self.apply(c, settle_s)
                flashes, passes = self.record(dwell_s)
                self.fgen_off()
                if passes:
                    s = T.summarize(c.label, c.expected, passes)
                    if s["serial_result"] == "FAIL":
                        lines.append(f"{colour}: SKIPPED -- the unit was not alerting "
                                     f"{c.expected} (lead C/F/V {s['lead_C']:.0%}/"
                                     f"{s['lead_F']:.0%}/{s['lead_V']:.0%})")
                        continue
                ok, msg = self.decoder.learn(colour, flashes)
                lines.append(msg)
        finally:
            self.fgen_off()
            self.quiet_end()
            if self.dev is not None:
                self.dev.release()
        self.decoder.save()
        sep = self.decoder.separation()
        lines.append(f"closest two references differ by {sep:.3f} in cosine "
                     + ("(good)" if sep > 0.1 else "(TOO CLOSE -- re-aim the sensor or "
                                                   "try 12-channel mode)"))
        with open(os.path.join(self.dir, "led_calibration.txt"), "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")
        for l in lines:
            self.log("   " + l)
        self.emit("ledcal", lines)
        return self.dir

    def revert(self, original, keys):
        for k in keys:
            if k in original:
                try:
                    self.dev.set(k, original[k])
                except Exception as exc:
                    self.log(f"!! could not revert {k}: {exc}")
