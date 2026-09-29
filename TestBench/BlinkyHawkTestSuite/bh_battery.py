"""
bh_battery.py  --  measure a BlinkyHawk on its own battery, floating.

WHY THIS EXISTS.  Every number the suite used to tune from came in over USB,
and USB earths the board through the host PC.  On V3b that moves the resting
differential by ~26-36 mV and changes the shape of the recovery tail, so
thresholds tuned on USB were wrong on battery -- badly enough that a shorted
lead read OPEN on every pass (Sep 2026: the unit went to sleep with the leads
shorted and never woke).  An earthed signal generator on the leads of an
earthed board is also a ground loop, which is what made USB voltage sweeps
asymmetric and noisy.  A scope ground clip on the board earths it the same way.
The ONLY condition that matters is the one the product lives in: on battery,
floating.  Alerts are locked out on USB anyway (CHGINHIBIT=1).

HOW.  Nothing can be printed while USB is unplugged, so the firmware logs to
RAM instead (BlinkyHawk_Unified: !SLEEPLOG,2 arms a 192-entry log that stops
when full and takes every VOLTAVG read per pass; !SLEEPLOG,D dumps it as
$SLOGD rows with the same fields as $DET).  One CAPTURE is:

    1. over USB: park the relay on the pre-marker state, pick the log spacing,
       stop the board sleeping (SLEEPSEC 0), arm the log
    2. operator unplugs USB           -> detected by the COM port vanishing
    3. a marker, then each condition in turn, driven by the jig + generator
    4. operator plugs USB back in     -> detected, the GUI reconnects
    5. !SLEEPLOG,D, restore the settings, turn the rows back into passes

The board logs one entry per LOG SPACING, not per pass; each entry is one real
detection pass.  Entries carry millis(), so they are placed on the host's
timeline by the marker: a +2 V DC step (voltage captures, generator needed) or
a SHORT -> OPEN step in the metric (VMODE 2 captures, where voltage is off).
A plan too long for one log is split into several captures, i.e. several
unplug cycles -- the operator is told each time.

wake_check() is separate: it lets the board really sleep, reach the deep
stage, and checks that the OPEN-side load does NOT wake it and the CLOSED-side
load does.  That is the sleeping probe, which uses SLEEPTHRxx and was the part
that failed in the field.

Pure-ish: the instruments go through the Runner (so the generator-off /
relay / generator-on ordering and Stop handling are the same as every other
job), and the GUI is reached only through the `host` object:
    host.port                 the BlinkyHawk's COM port
    host.reconnect()          ask the GUI to reconnect (called from this thread)
    host.notice(text|None)    show / clear an operator instruction
"""

import math
import time

import serial.tools.list_ports

import bh_tuner as T
from bh_instruments import InstrumentError

SLOG_CAP = 192              # BlinkyHawk_Unified SLOG_MAX
SLOG_FILL = 176             # plan to this, leave slack for timing slop
PRE_S = 1.5                 # after the unplug is seen, before the marker
MARKER_S = 2.0
MARKER_V = 2.0
LINK_TIMEOUT_S = 180        # longest we wait for an unplug / replug


def port_present(port):
    return any(p.device == port for p in serial.tools.list_ports.comports())


def parse_slogd(line):
    """$SLOGD,<i>,<ms>,<src>,<raw>,<lead>,<n>,<mean>,<min>,<max>,<metric>,<retms>,
             <area>,<thr>,<rawkind>,<kind>,<vpos>,<vneg>
    -> a dict with parse_det()'s keys (vpath unknown) plus 'src' (A/S/D)."""
    f = line.split(",")
    if len(f) < 18 or f[0] != "$SLOGD":
        return None
    g = T._f
    try:
        return {"ms": int(f[2]), "src": f[3], "raw": f[4], "lead": f[5], "vpath": "",
                "n": int(f[6]), "mean": g(f[7]), "min": g(f[8]), "max": g(f[9]),
                "metric": g(f[10]), "retms": g(f[11]), "area": g(f[12]), "thr": g(f[13]),
                "rawkind": f[14].strip() or None, "kind": f[15].strip() or None,
                "vpos": g(f[16]), "vneg": g(f[17])}
    except ValueError:
        return None


class BatteryBench:
    def __init__(self, runner, host):
        self.r = runner
        self.host = host
        self.captures = 0

    # ---------- plumbing ----------
    def log(self, text):
        self.r.log(text)

    def _dev(self):
        if self.r.dev is None:
            raise InstrumentError("battery capture needs the BlinkyHawk connected over USB "
                                  "between captures (to arm the log and read it back)")
        return self.r.dev

    def _wait(self, want_present, what):
        self.host.notice(what)
        end = time.time() + LINK_TIMEOUT_S
        while port_present(self.host.port) != want_present:
            self.r.check_stop()
            if time.time() > end:
                raise InstrumentError(f"timed out waiting for: {what}")
            time.sleep(0.1)
        self.host.notice(None)

    def _relink(self):
        """USB is back: have the GUI reconnect, then wait for a $STATUS."""
        time.sleep(1.5)                       # let Windows finish enumerating
        dev = self._dev()
        end = time.time() + 20
        last = None
        while time.time() < end:
            self.r.check_stop()
            try:
                self.host.reconnect()
                dev.status()
                return
            except Exception as exc:      # not open yet / not answering yet
                last = exc
                time.sleep(1.0)
        raise InstrumentError(f"the BlinkyHawk did not come back on {self.host.port}: {last}")

    def _dump(self):
        dev = self._dev()
        q = dev.tee.subscribe()
        rows = []
        try:
            dev.send("!SLEEPLOG,D")
            end = time.time() + 10
            while time.time() < end:
                try:
                    line = q.get(timeout=0.2)
                except Exception:
                    continue
                if line.startswith("$SLOGD"):
                    p = parse_slogd(line)
                    if p:
                        rows.append(p)
                elif line.startswith("$SLOGEND"):
                    return rows
        finally:
            dev.tee.unsubscribe(q)
        raise InstrumentError("no $SLOGEND -- is this BlinkyHawk_Unified firmware with "
                              "!SLEEPLOG,D (Sep 2026 or later)?")

    def _arm(self, settings):
        """settings: {key: value} applied for the capture.  Returns what to restore."""
        dev = self._dev()
        dev.release()                         # !DETLOG off: never print to a dead USB port
        cfg = dev.get_cfg()
        restore = {}
        for k, v in settings.items():
            if k.startswith("!"):
                continue
            restore[k] = cfg.get(k)
            dev.set(k, v)
        for k, v in settings.items():
            if k.startswith("!"):
                dev.send(f"{k},{v}")
        dev.request("!SLEEPLOG,2", lambda l: l.startswith("$OK,sleeplog"))
        return cfg, restore

    def _disarm(self, restore):
        dev = self._dev()
        for k, v in restore.items():
            if v is not None:
                dev.set(k, v)
        dev.send("!VMODE,0")
        dev.request("!SLEEPLOG,0", lambda l: l.startswith("$OK,sleeplog"))

    # ---------- sizing ----------
    @staticmethod
    def plan(n_conds, dwell, settle, samples, tick_ms):
        """-> (sleep_ticks, spacing_ms, conds_per_capture)."""
        want = max(1.0, (dwell - settle) * 1000.0 / max(1, samples))
        k = max(1, int(want // tick_ms))
        spacing = max(100.0, k * tick_ms)
        fit = int((SLOG_FILL * spacing / 1000.0 - PRE_S - MARKER_S) // dwell)
        if fit < 1:
            raise InstrumentError(f"a {dwell:g} s dwell does not fit the {SLOG_CAP}-entry log "
                                  f"at {spacing:g} ms spacing -- ask for fewer samples")
        return k, spacing, min(fit, n_conds)

    # ---------- one capture ----------
    def capture(self, conds, dwell, settle, samples, vmode=0, phase="battery"):
        """Run `conds` on battery.  Returns {cond.label: [passes]} plus, per
        condition, a summary through the Runner's usual tables.  Splits into
        several unplug cycles if the plan does not fit one log."""
        dev = self._dev()
        cfg = dev.get_cfg()
        tick = float(cfg.get("SLEEPTICKMS", 63))
        if vmode == 0 and self.r.fgen is None:
            raise InstrumentError("the voltage marker needs the function generator")
        k, spacing, per = self.plan(len(conds), dwell, settle, samples, tick)
        chunks = [conds[i:i + per] for i in range(0, len(conds), per)]
        self.log(f"   battery capture: {len(conds)} condition(s) in {len(chunks)} unplug "
                 f"cycle(s); one log entry per {spacing:.0f} ms "
                 f"(~{int((dwell - settle) * 1000 // spacing)} per condition)")
        out = {}
        for ci, chunk in enumerate(chunks, 1):
            out.update(self._capture_one(chunk, dwell, settle, k, spacing, vmode,
                                         f"{phase} {ci}/{len(chunks)}"))
        return out

    def _capture_one(self, conds, dwell, settle, k, spacing, vmode, tag):
        r = self.r
        marker_v = (vmode == 0)
        # Pre-marker state: generator path off for a voltage marker, SHORT for
        # the metric marker.  Set while still on USB.
        r.fgen_off()
        r.relay.set_mode("OPEN" if marker_v else "SHORT")
        # CHGINHIBIT=1 so the log cannot start (and fill) before the unplug:
        # the firmware only logs awake passes while charge does not inhibit.
        cfg, restore = self._arm({"SLEEPSEC": 0, "SLEEPTICKS": k, "CHGINHIBIT": 1,
                                  "!VMODE": vmode})
        self.captures += 1
        try:
            rows, timeline = self._run_armed(conds, dwell, marker_v, tag)
        except BaseException:
            self._stranded()
            raise
        self._disarm(restore)
        return self._assign(rows, timeline, settle, marker_v, spacing)

    def _stranded(self):
        self.host.notice(None)
        self.log("!! capture interrupted: the unit may still hold the capture settings in "
                 "RAM (SLEEPSEC 0 = never sleeps, SLEEPTICKS, CHGINHIBIT 1, VMODE, log armed). "
                 "Plug it in and press 'Reload EEPROM -> RAM' on the Configuration tab, or "
                 "power-cycle it.  Nothing was saved to EEPROM.")

    def _run_armed(self, conds, dwell, marker_v, tag):
        r = self.r
        r.emit("stage", f"{tag}: UNPLUG USB")
        self._wait(False, f"{tag}: UNPLUG the BlinkyHawk's USB now.  "
                          "Leave it unplugged until told -- the run is automatic.")
        timeline = []
        try:
            r.sleep(PRE_S)
            t0 = time.time()
            if marker_v:
                r.fgen.apply_dc(MARKER_V)
                r.relay.set_mode("FGEN")
                r.fgen.output(True)
            else:
                r.relay.set_mode("OPEN")
            timeline.append(("__marker__", 0.0, MARKER_S))
            r.sleep(MARKER_S)
            for c in conds:
                r.emit("status", f"{tag}: {c.label}")
                t = time.time() - t0
                r.apply(c, 0.0)
                timeline.append((c.label, t, dwell))
                r.sleep(dwell)
        finally:
            r.fgen_off()
            try:
                r.relay.set_mode("OPEN")
            except Exception:
                pass
        r.emit("stage", f"{tag}: PLUG USB BACK IN")
        self._wait(True, f"{tag}: done -- plug the BlinkyHawk's USB back in.")
        self._relink()
        return self._dump(), timeline

    def _assign(self, rows, timeline, settle, marker_v, spacing):
        awake = [e for e in rows if e["src"] == "A"]
        if not awake:
            raise InstrumentError("the log is empty -- the board never logged on battery "
                                  "(was it really unplugged?  CHGINHIBIT with a 5 V supply "
                                  "on its input stops it)")
        off = self._align(awake, marker_v, spacing)
        res = {}
        for label, t, dw in timeline[1:]:
            lo, hi = t + settle, t + dw - 0.05
            res[label] = [e for e in awake if lo <= e["ms"] / 1000.0 - off < hi]
            if not res[label]:
                self.log(f"   !! {label}: no log entries in its window")
        if len(rows) >= SLOG_CAP:
            self.log("   (log filled -- any conditions after that point have no entries)")
        return res

    def _align(self, awake, marker_v, spacing):
        """Board-seconds value of host t=0 (the marker start)."""
        first = None
        if marker_v:
            first = next((e for e in awake if e["raw"] == "V"), None)
        else:
            m = [e["metric"] for e in awake]
            base = [x for x in m[:3] if x is not None]
            if base and len(m) > 6:
                b = sorted(base)[len(base) // 2]
                top = max(x for x in m[:40] if x is not None)
                cut = b + 0.5 * (top - b)
                if top - b > 1e-6:
                    first = next((e for e in awake if e["metric"] is not None
                                  and e["metric"] > cut), None)
        if first is None:
            self.log("   !! marker not found in the log -- aligning on the unplug instead "
                     "(windows may be off by a few hundred ms)")
            return awake[0]["ms"] / 1000.0 + PRE_S
        # The marker lands between two logged passes: split the difference.
        return first["ms"] / 1000.0 - spacing / 2000.0

    # ---------- the sleeping probe ----------
    def wake_check(self, stay, wake, hold_s=5.0):
        """stay/wake: Cond for the load that must NOT wake a deep-sleeping unit
        and the one that must.  Returns (lines, ok)."""
        r = self.r
        dev = self._dev()
        cfg = dev.get_cfg()
        sleep_s = float(cfg.get("SLEEPSEC", 1))
        deep_s = float(cfg.get("DEEPSEC", 0))
        deep_hz = float(cfg.get("DEEPHZ", 1))
        thr_note = cfg.get("SLEEPTHR" + format(int(dev.status().get("dip", 3)), "02b"))
        # Sleep for real, but reach the deep stage quickly: stage-1 probes log
        # 16x/s and would fill the log before a long DEEPSEC ran out.  WHEN the
        # deep stage starts does not change how its probe measures.
        settings = {"SLEEPSEC": min(sleep_s, 1) if sleep_s > 0 else 1,
                    "SLEEPTICKS": 1, "CHGINHIBIT": 1, "!VMODE": 0}
        if deep_s > 0:
            settings["DEEPSEC"] = min(deep_s, 2)
        r.fgen_off()
        r.relay.set_mode("OPEN")
        cfg0, restore = self._arm(settings)
        open_s = settings["SLEEPSEC"] + settings.get("DEEPSEC", 0) + 3.0
        try:
            rows = self._wake_run(stay, wake, hold_s, open_s)
        except BaseException:
            self._stranded()
            raise
        self._disarm(restore)
        return self._wake_judge(rows, cfg, settings, stay, wake, hold_s, open_s,
                                deep_s, deep_hz, thr_note)

    def _wake_run(self, stay, wake, hold_s, open_s):
        r = self.r
        r.emit("stage", "wake check: UNPLUG USB")
        self._wait(False, "Sleep/wake check: UNPLUG the BlinkyHawk's USB now.  "
                          f"It should fall asleep, then stay asleep on {stay.label} and "
                          f"wake on {wake.label} (about {open_s + 2 * hold_s:.0f} s).")
        try:
            r.sleep(PRE_S + open_s)
            r.emit("status", f"wake check: {stay.label} (must stay asleep)")
            r.apply(stay, 0.0)
            r.sleep(hold_s)
            r.emit("status", f"wake check: {wake.label} (must wake)")
            r.apply(wake, 0.0)
            r.sleep(hold_s)
        finally:
            r.fgen_off()
            try:
                r.relay.set_mode("OPEN")
            except Exception:
                pass
        r.emit("stage", "wake check: PLUG USB BACK IN")
        self._wait(True, "Sleep/wake check done -- plug the BlinkyHawk's USB back in.")
        self._relink()
        return self._dump()

    def _wake_judge(self, rows, cfg, settings, stay, wake, hold_s, open_s,
                    deep_s, deep_hz, thr_note):
        L = [f"SLEEP/WAKE CHECK (on battery; SLEEPTHR in force: {thr_note})"]
        probes = [e for e in rows if e["src"] in "SD"]
        deep = [e for e in probes if e["src"] == "D"]
        if not probes:
            L.append("   FAIL: the board never slept -- nothing to judge")
            return L, False
        wake_i = next((i for i, e in enumerate(probes) if e["raw"] != "F"), None)
        want_deep = deep_s > 0
        if want_deep and not deep:
            L.append("   !! never reached the deep stage (log full?) -- stage 1 only judged")
        if wake_i is None:
            L.append(f"   FAIL: no probe ever read anything but FLOAT -- {wake.label} did "
                     "not wake it")
            ok = False
        else:
            w = probes[wake_i]
            stage = {"S": "stage 1", "D": "deep stage"}[w["src"]]
            same = [p for p in probes[:wake_i] if p["src"] == w["src"]]
            # millis() is frozen in Standby, so probes cannot be timed -- they are
            # COUNTED.  The stage the wake came from started (SLEEPSEC [+ DEEPSEC])
            # after the unplug and the wake load went on PRE_S + open_s + hold_s
            # after it, so that many seconds of probes should precede the wake.
            # Waking early, on `stay`, leaves ~hold_s worth fewer.
            rate = deep_hz if w["src"] == "D" else 1000.0 / float(cfg.get("SLEEPTICKMS", 63))
            start = settings["SLEEPSEC"] + (settings.get("DEEPSEC", 0) if w["src"] == "D" else 0)
            expect = (PRE_S + open_s + hold_s - start) * rate
            enough = len(same) >= expect - 0.5 * hold_s * rate
            ok = w["raw"] in "CV" and enough
            L.append(f"   woke from the {stage} on a probe reading {w['raw']} "
                     f"(metric {w['metric']}, threshold {w['thr']})")
            stay_probes = same[-max(1, int(hold_s * rate) - 1):]
            ms = [p["metric"] for p in stay_probes if p["metric"] is not None]
            if ms:
                L.append(f"   the last {len(stay_probes)} {stage} probe(s) before it -- "
                         f"{stay.label} on the leads -- all FLOAT, metric "
                         f"{min(ms):.4f}..{max(ms):.4f} (margin {min(ms) - w['thr']:+.4f} "
                         "over the threshold)")
            L.append(f"   {len(same)} {stage} probe(s) before the wake, ~{expect:.0f} expected")
            if not enough:
                L.append(f"   FAIL: too few -- it most likely woke while {stay.label} was on")
        L.append("   verdict: " + ("PASS" if ok else "FAIL"))
        return L, ok
