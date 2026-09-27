"""
bh_sequencer.py  --  runs conditions across the bench and records what happened.

A CONDITION is one thing done to the BlinkyHawk's leads: a relay-jig load, a
generator DC level, a generator sine, or the generator path with its output
off.  For each one the runner
    1. turns the generator output off (never switch relays under drive)
    2. moves the relay jig (break-before-make is the jig firmware's job)
    3. programs the generator and turns it on, if the condition needs it
    4. waits the settle time
    5. records $DET passes from the BlinkyHawk for the dwell time
    6. optionally captures the scope
    7. turns the generator off again
Two jobs are built on that: run_plan() (the "big test") and run_autotune().

Everything lands in a run folder under Runs/:
    run_info.txt       instruments, settings, timing
    config_before.csv  the BlinkyHawk's full !CFG at the start
    passes.csv         every $DET pass, tagged with its condition
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
from bh_instruments import JIG_LOADS, fmt_ohms, write_scope_csv, InstrumentError

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
            raise InstrumentError("no $DET lines -- is this the bench firmware with !DETLOG "
                                  "(flash BlinkyHawk_Bench from this commit)?")
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


def dc_condition(v, vt=None):
    exp = None
    if vt:
        exp = "V" if abs(v) >= vt * 1.1 else ("!V" if abs(v) <= vt * 0.9 else None)
    return Cond(f"DC {v:+.3f}V", "dc", "FGEN", value=v, expected=exp)


def ac_condition(vrms, freq):
    return Cond(f"AC {vrms:g}Vrms {freq:g}Hz", "ac", "FGEN", value=vrms, freq=freq,
                expected="V")


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
    def __init__(self, device, relay, fgen, scope, emit, stop_event):
        self.dev, self.relay, self.fgen, self.scope = device, relay, fgen, scope
        self.emit = emit             # callable(kind, payload) -> GUI queue
        self.stop = stop_event
        self.dir = None
        self.pass_rows = []
        self.summaries = []
        self.scope_conds = []
        self.scope_note = ""
        self.started = None

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
        passes = self.dev.collect(dwell_s, min_passes)
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
        s = T.summarize(cond.label, cond.expected, passes, stable_count)
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
        self.log(f"   {cond.label:>20}: n={s['n']:3d}  lead C/F/V "
                 f"{s['lead_C'] * 100:3.0f}/{s['lead_F'] * 100:3.0f}/{s['lead_V'] * 100:3.0f} %"
                 f"  rest {s['rest_mean'] if s['rest_mean'] is not None else float('nan'):+.4f} V"
                 f"  {s['result']}")
        self.write_files()
        return passes, s

    # ---------- files ----------
    def write_files(self):
        if not self.dir:
            return
        with open(os.path.join(self.dir, "passes.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["phase", "condition", "i"] + T.DET_FIELDS)
            w.writerows(self.pass_rows)
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
                f"scope:  {getattr(self.scope, 'ident', 'not connected')}"]

    # ---------- job 1: the big test ----------
    def run_plan(self, conds, dwell_s, settle_s, scope_mode, scope_timeout, note):
        self.open_run("Test", note)
        self.log(f"== test run -> {self.dir}")
        cfg = self.dev.get_cfg()
        self.write_cfg(cfg, "config_before.csv")
        stable = int(float(cfg.get("STABLECOUNT", 2)))
        self.write_info([f"BlinkyHawk test suite -- test run {self.started}", f"note: {note}",
                         *self.instrument_lines(),
                         f"dwell {dwell_s} s, settle {settle_s} s, scope {scope_mode or 'off'}",
                         "conditions:", *[f"  {c.label}: {c.describe()}, expect "
                                          f"{c.expected or '-'}" for c in conds]])
        self.dev.prepare()
        try:
            for i, c in enumerate(conds, 1):
                self.emit("progress", (i, len(conds)))
                try:
                    self.measure(c, "test", dwell_s, settle_s, scope_mode, scope_timeout,
                                 stable_count=stable)
                except InstrumentError as exc:
                    self.log(f"   {c.label}: SKIPPED -- {exc}")
        finally:
            self.fgen_off()
            self.dev.release()
            self.write_files()
            self.write_scope()
        fails = [s["label"] for s in self.summaries if s["result"] == "FAIL"]
        self.log(f"== done: {len(self.summaries)} condition(s), "
                 f"{len(fails)} fail{'' if len(fails) == 1 else 's'}"
                 + (f": {', '.join(fails)}" if fails else ""))
        return self.dir

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
        report = [f"BlinkyHawk auto-tune  {self.started:%Y-%m-%d %H:%M:%S}",
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
        self.dev.prepare()
        loads = load_conditions(p["loads"], p["closed_max"], p["open_min"])
        try:
            # ---- 1. voltage ----
            if p["do_voltage"]:
                if self.fgen is None:
                    report += ["VOLTAGE: skipped -- needs the function generator.", ""]
                    self.log("voltage tuning skipped: no function generator")
                else:
                    self.emit("stage", "Voltage: characterising")
                    novolt, dc, ac = {}, {}, {}
                    for c in loads:
                        novolt[c.relay], _ = self.measure(c, "volt", dwell, settle,
                                                          min_passes=N, stable_count=stable)
                    levels = sorted(set(p["dc_levels"]) | {0.0, p["vt"], -p["vt"]})
                    for v in levels:
                        dc[v], _ = self.measure(dc_condition(v), "volt", dwell, settle,
                                                min_passes=N, stable_count=stable)
                    for vrms, f in p["ac"]:
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
                self.dev.send("!VMODE,2")       # voltage check off: every pass runs the test
                groups = [({"DETMETHOD": 2, "DETBAND": b}, b) for b in p["detbands"]]
                if p["include_m0"]:
                    groups.append(({"DETMETHOD": 0}, None))
                cands = []
                for gi, (settings, band) in enumerate(groups):
                    for k, v in settings.items():
                        setk(k, v)
                    rec = []
                    for c in loads:
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
                for c in conds:
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
                ok = not fails
                report += ["", "VERIFY VERDICT: " + ("every condition read as expected" if ok else
                                                     f"{len(fails)} condition(s) failed: " +
                                                     ", ".join(s["label"] for s in fails))]
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
            self.dev.release()
            self.write_files()
            with open(os.path.join(self.dir, "tuning_report.txt"), "w", encoding="utf-8") as fh:
                fh.write("\n".join(report) + "\n")
            self.emit("report", "\n".join(report))
        self.emit("tune_done", {"original": original, "applied": applied, "ok": ok,
                                "dir": self.dir})
        return self.dir

    def revert(self, original, keys):
        for k in keys:
            if k in original:
                try:
                    self.dev.set(k, original[k])
                except Exception as exc:
                    self.log(f"!! could not revert {k}: {exc}")
