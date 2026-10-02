"""
bh_timing.py  --  sleep / wake TIMING on battery, watched by the isolated Giga meter.

The unit's RAM log cannot time anything while the board sleeps (millis() is
frozen in Software Standby; the wake check counts probes instead).  The Giga
meter on the sense node sees every detection pass as a pulse in real time, so
this job measures what the log can't:

  1. awake cadence        leads CLOSED -> one pass every LOOPMS + the pass
  2. deep entry           an unbroken STREAM from the moment the leads open:
                          the cadence drops from every tick to DEEPHZ.  NOTE the
                          awake loop and stage 1 can both sit near 61 ms, so the
                          awake -> stage 1 step is not visible; deep entry is.
  3. stage-1 cadence      leads OPEN, after SLEEPSEC: one probe per RTC tick
                          (1/16 s nominal).  Optionally the scope on the same
                          node as a second timebase.
  4. deep cadence         a capture inside the deep stage
  5. wake latency         from deep sleep, close the leads mid-stream: time to
                          the first pass after that (the probe that sees it),
                          repeated

TWO INSTRUMENT MODES, chosen per step.  A CAPTURE is exact (6.3 kS/s on the
meter's own sample clock -- checked to 0.1 % against the board's millis()) but
has to be transferred before the next can start: a 5 s window takes ~20 s round
trip.  The live STREAM reports every 50 ms without gaps; its reports carry the
meter's own period timestamps, placed on the PC clock by the earliest-arriving
report (typically ~40 ms behind, <100 ms worst).  So cadences come from
captures; anything lined up with a relay switch comes from the stream, to
within one period (+-50 ms).

Everything runs with USB disconnected by the relay jig's K8 (on USB, CHGINHIBIT
keeps the board awake) and goes back the way it was found.

FOUND WITH THIS (Oct 2026, SN 20260928_002): the board's RTC runs ~2.4 % fast
-- stage 1 at 61.0 ms for 62.5 nominal, deep at 976.5 ms for 1000 -- consistent
with an RC (LOCO) sub-clock rather than a 32.768 kHz crystal.  DEEPSEC and
SLEEPHB are counted in RTC ticks, so they run 2.4 % short too.
"""

import csv
import os
import statistics
import threading
import time

from bh_giga import board_to_host, cadence, find_passes, pass_periods
from bh_instruments import InstrumentError

FINE = dict(smooth=7)            # 6.3 kS/s, ~1.6 s max -- the preset's own setting
LONG = dict(smooth=22)           # 2.0 kS/s, ~5 s max -- passes stay clear (rail plateau ~12 ms)
STREAM_HZ = 20


class Capt:
    """A scope trace dressed as a bh_giga.Capture for find_passes()."""

    def __init__(self, wave):
        self.v = wave.volts
        self.t = [(wave.t0 + i * wave.dt) * 1000.0 for i in range(len(wave.volts))]


def _ms(x):
    return "n/a" if x is None else f"{x:.1f} ms"


class TimingCheck:
    def __init__(self, runner, meter):
        self.r = runner
        self.m = meter
        self.lines = []
        self.rows = []               # (step, quantity, measured_ms, expected_ms, note)
        self.n_saved = 0

    def log(self, s):
        self.lines.append(s)
        self.r.log(s)

    def _save(self, name, header, rows):
        """Every raw capture / stream goes into the run folder: the numbers in
        the report are only as good as the pulse detection, and this is how it
        gets checked afterwards."""
        if not getattr(self.r, "dir", None):
            return
        d = os.path.join(self.r.dir, "meter")
        os.makedirs(d, exist_ok=True)
        self.n_saved += 1
        with open(os.path.join(d, f"{self.n_saved:02d}_{name}.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(header)
            w.writerows(rows)

    def capture(self, ms, shape, name="capture"):
        self.r.check_stop()
        t0 = time.time()
        c = self.m.capture(ms, **shape)
        p = find_passes(c)
        a, b = getattr(c, "legs", ([0], [0]))
        self.log(f"      ({name}: requested {time.strftime('%H:%M:%S', time.localtime(t0))}, "
                 f"took {time.time() - t0:.1f} s; legs A {min(a):.2f}..{max(a):.2f} V, "
                 f"B {min(b):.2f}..{max(b):.2f} V)")
        self._save(name, ["t_ms", "volts", "pass"],
                   [(f"{t:.3f}", f"{v:.4f}", int(t in p)) for t, v in zip(c.t, c.v)])
        self.log(f"      ({name}: {len(c.v)} pts over {c.elapsed_ms} ms, "
                 f"{min(c.v):.2f}..{max(c.v):.2f} V, {len(p)} passes)")
        return c

    def row(self, step, what, got, want, note=""):
        self.rows.append((step, what, got, want, note))
        err = ""
        if got is not None and want:
            err = f"  ({100 * (got - want) / want:+.1f} % vs {want:g} ms)"
        self.log(f"   {step:>8}: {what:<30} {_ms(got)}{err}" + (f"  -- {note}" if note else ""))

    # ------------------------------------------------------------------
    def run(self, cfg, repeats=3, scope=None):
        r = self.r
        sleep_s = float(cfg.get("SLEEPSEC", 1))
        tick = float(cfg.get("SLEEPTICKMS", 63))
        tick = 62.5 if abs(tick - 63) < 0.6 else tick          # the 1/16 s rung, shown rounded
        ticks = int(float(cfg.get("SLEEPTICKS", 1)))
        deep_s = float(cfg.get("DEEPSEC", 0))
        deep_hz = float(cfg.get("DEEPHZ", 1) or 1)
        loop_ms = float(cfg.get("LOOPMS", 50))
        stable = int(float(cfg.get("STABLECOUNT", 2)))
        stage1 = tick * ticks
        deep_ms = 1000.0 / deep_hz
        if sleep_s <= 0:
            raise InstrumentError("SLEEPSEC is 0 -- the board never sleeps; nothing to time")
        self.log(f"TIMING CHECK (on battery, USB off; meter {self.m.label} x{self.m.d_scale:g})")
        self.log(f"   config: awake every LOOPMS {loop_ms:g} ms + the pass; sleeps after "
                 f"{sleep_s:g} s open; stage 1 every {stage1:g} ms; "
                 + (f"deep after a further {deep_s:g} s at {deep_ms:g} ms" if deep_s > 0
                    else "no deep stage (DEEPSEC 0)"))
        settle_deep = sleep_s + deep_s * 1.05 + 1.0     # open -> safely inside the deep stage

        # 1. awake cadence
        r.relay.set_mode("SHORT")
        r.sleep(2.0)
        awake, _ = cadence(find_passes(self.capture(1500, FINE, "awake")))
        self.row("awake", "pass interval", awake, None, f"LOOPMS {loop_ms:g} + the pass itself")

        # 2. open the leads with a stream already running: deep entry
        if deep_s > 0:
            rows, t_open = self._stream_around(settle_deep + 3.0,
                                               lambda: r.relay.set_mode("OPEN"), 1.0)
            entry = self._deep_entry(rows, t_open, deep_ms)
            self.row("deep", "entry after leads opened", entry, (sleep_s + deep_s) * 1000.0,
                     "stream, +-50 ms; DEEPSEC is counted in RTC ticks")
            r.relay.set_mode("SHORT")          # wake it for the stage-1 measurement
            r.sleep(1.5)

        # 3. stage-1 cadence (+ the scope as a second timebase)
        r.relay.set_mode("OPEN")
        t_open = time.time()
        r.sleep(sleep_s + 0.7)
        sc_res = {}
        th = None
        if scope is not None:
            th = threading.Thread(target=self._scope_period, args=(scope, sc_res), daemon=True)
            th.start()
        s1, _ = cadence(find_passes(self.capture(1500, FINE, "stage1")))
        if th is not None:
            th.join(timeout=60)
        self.row("stage 1", "probe interval", s1, stage1, "meter capture")
        if scope is not None:
            if sc_res.get("period"):
                self.row("stage 1", "probe interval (scope)", sc_res["period"], stage1,
                         "scope crystal as a second reference")
            else:
                self.log(f"   scope: {sc_res.get('error', 'no period found')}")

        if deep_s > 0:
            # 4. deep cadence: the leads have stayed open since t_open
            r.sleep(max(0.0, t_open + settle_deep - time.time()))
            dp, _ = cadence(find_passes(self.capture(5000, LONG, "deep")))
            self.row("deep", "probe interval", dp, deep_ms, "meter capture")
            if dp:
                rtc = 100.0 * (deep_ms - dp) / deep_ms
                self.log(f"   RTC: deep probes come {abs(rtc):.1f} % "
                         f"{'early -- the sleep clock runs fast' if rtc > 0 else 'late -- the sleep clock runs slow'}"
                         + (" (consistent with an RC sub-clock, not a crystal)"
                            if abs(rtc) > 0.5 else ""))

            # 5. wake latency, repeated
            lats = []
            for k in range(repeats):
                if k:
                    r.relay.set_mode("OPEN")
                    r.sleep(settle_deep)
                lat = self._wake_once()
                if lat is not None:
                    lats.append(lat)
                self.log(f"   wake {k + 1}: first pass {_ms(lat)} after the leads closed")
            if lats:
                alert = stable * (awake or loop_ms)
                self.row("wake", f"to first pass (median of {len(lats)})",
                         statistics.median(lats), None,
                         f"range {min(lats):.0f}..{max(lats):.0f} ms; worst case = one deep "
                         f"interval ({dp or deep_ms:.0f} ms); the alert follows ~{alert:.0f} ms "
                         f"later (STABLECOUNT {stable} awake passes)")
        r.relay.set_mode("OPEN")
        return self.lines, self.rows

    # ------------------------------------------------------------------
    def _stream_around(self, seconds, action, after_s):
        """Stream for `seconds`, doing `action()` `after_s` in.
        -> (rows, host time of the action)."""
        res = {}

        def go():
            try:
                res["rows"] = self.m.stream(seconds, hz=STREAM_HZ, stop_event=self.r.stop)
            except Exception as exc:
                res["err"] = exc
        th = threading.Thread(target=go, daemon=True)
        th.start()
        time.sleep(after_s)
        t_act = time.time()
        action()
        th.join(timeout=seconds + 30)
        self.r.check_stop()
        if "err" in res:
            raise InstrumentError(f"meter stream failed: {res['err']}")
        rows = res.get("rows", [])
        off = board_to_host(rows)
        self._save("stream", ["t_from_action_ms", "seq", "max_V", "mean_V"],
                   [(f"{(r[4] + off - t_act) * 1000:.0f}", r[1], f"{r[2]:.3f}", f"{r[3]:.3f}")
                    for r in rows])
        return rows, t_act

    def _pass_times(self, rows):
        """Host times of the stream periods holding a pass (the period's END,
        so up to one period after the pass itself)."""
        idx, _ = pass_periods(rows)
        off = board_to_host(rows)
        return [rows[i][4] + off for i in idx]

    def _deep_entry(self, rows, t_open, deep_ms):
        """ms after `t_open` of the last fast pass before the cadence slows."""
        ts = self._pass_times(rows)
        for a, b in zip(ts, ts[1:]):
            if a > t_open and (b - a) * 1000.0 > 0.5 * deep_ms:
                return (a - t_open) * 1000.0
        self.log("   deep entry: no slow-down seen in the stream")
        return None

    def _wake_once(self):
        """In deep sleep: stream, close the leads 1.5 s in, time the first pass."""
        rows, t_close = self._stream_around(4.0, lambda: self.r.relay.set_mode("SHORT"), 1.5)
        after = [t for t in self._pass_times(rows) if t >= t_close]
        return (after[0] - t_close) * 1000.0 if after else None

    def _scope_period(self, scope, out):
        """1 s window at 1 MS/s, auto trigger; restores the scope afterwards."""
        try:
            keep = {q: scope.query(q) for q in (":TIM:SCAL?", ":TIM:DEL?", ":ACQ:MDEP?")}
            try:
                scope.write(":ACQ:MDEP 1M")
                scope.write(":TIM:SCAL 0.1")
                scope.write(":TIM:DEL 0")
                scope.write(":TRIG:MODE AUTO")
                scope.write(":TRIG:RUN")
                time.sleep(2.5)
                tdiv, trig, waves = scope.capture("grab")
                p = find_passes(Capt(waves[0]))
                out["period"], _ = cadence(p)
                out["n"] = len(p)
            finally:
                scope.write(f":ACQ:MDEP {keep[':ACQ:MDEP?']}")
                scope.write(f":TIM:SCAL {keep[':TIM:SCAL?']}")
                scope.write(f":TIM:DEL {keep[':TIM:DEL?']}")
                scope.restore()
        except Exception as exc:
            out["error"] = f"scope check failed: {exc}"
