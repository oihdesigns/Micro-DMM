"""
battery_sequence.py -- unattended health / partial-capacity run on a 12 V
flooded lead-acid battery.

Phases (run in order, --start PHASE resumes partway):
  charge     CV 14.4 V *at the battery* (regulated from the load's voltage
             reading -- no current flows in the load leads while charging, so it
             is a clean battery reading; the PSU leads drop ~50 mOhm), CC 5 A.
             Done when current < END_I for 10 min, or flat for an hour after
             2 h of absorption, or 6 h absorption / 18 h total.
  rest1      3 h open circuit (surface charge off) -> full-charge OCV.
  ir_full    current-pulse IR test at full charge (ir_test.run).
  discharge  CC C/20 from the load, stops at 50 % of rated Ah or when the
             battery reaches DIS_FLOOR (a weak battery).  Load Von is the
             hardware backstop if this script dies.
  rest2      2 h open circuit -> OCV after the known Ah removed.
  recharge   same as charge.

Battery voltage: charge -> v_load; otherwise -> v_psu (supply output off).
Writes samples.csv (append), events.log, status.json in OUTDIR.

usage: py -3 battery_sequence.py OUTDIR [--start PHASE] [--ah 65]
"""
import argparse
import csv
import json
import os
import time
import traceback

import ir_test
from cbt_bench import Bench, all_off_fresh

PHASES = ("charge", "rest1", "ir_full", "discharge", "rest2", "recharge")

V_ABSORB = 14.40
I_CHARGE = 5.0          # CH2 setpoint; parallel total reads ~5.19 A
VSET_MAX = 14.80        # never set the supply above this
V_ABORT = 14.90         # battery above this -> everything off, stop
REST1_S = 3 * 3600
REST2_S = 2 * 3600
DIS_FLOOR = 11.75       # loaded battery V that ends the discharge early
DIS_VON = 11.50         # load hardware backstop
SAMPLE_S = 5.0

FIELDS = ["t", "iso", "phase", "v_bat", "v_psu", "v_load", "i_ch2", "i_ch3", "i_chg",
          "i_load", "vset", "ah_in", "ah_out", "wh_in", "wh_out"]


class Run:
    def __init__(self, out, ah):
        self.out = out
        self.ah_rated = ah
        self.b = None
        self.ah_in = self.ah_out = self.wh_in = self.wh_out = 0.0
        self.last = None
        self.phase = None
        self.notes = {}
        sp = os.path.join(out, "status.json")
        if os.path.exists(sp):                       # resume: keep counters
            st = json.load(open(sp))
            for k in ("ah_in", "ah_out", "wh_in", "wh_out"):
                setattr(self, k, st.get(k, 0.0))
            self.notes = st.get("notes", {})

    # ---- plumbing -----------------------------------------------------------
    def log(self, msg):
        line = f"{time.strftime('%Y-%m-%d %H:%M:%S')}  {msg}"
        print(line, flush=True)
        with open(os.path.join(self.out, "events.log"), "a") as f:
            f.write(line + "\n")

    def connect(self):
        for k in range(24):                          # ~2 min of retries
            try:
                self.b = Bench()
                return
            except Exception as e:
                self.log(f"connect failed ({e}); retry")
                time.sleep(5)
        raise RuntimeError("instruments unreachable")

    def sample(self, vset=None):
        try:
            r = self.b.read()
        except Exception as e:
            self.log(f"read failed ({e}); reconnecting")
            try:
                self.b.close()
            except Exception:
                pass
            self.connect()
            r = self.b.read()
        r["i_chg"] = r["i_ch2"] + r["i_ch3"] if self.b_psu_on else 0.0
        r["v_bat"] = r["v_load"] if self.b_psu_on else r["v_psu"]
        if self.last:
            dt = (r["t"] - self.last["t"]) / 3600
            self.ah_in += 0.5 * (r["i_chg"] + self.last["i_chg"]) * dt
            self.ah_out += 0.5 * (r["i_load"] + self.last["i_load"]) * dt
            self.wh_in += 0.5 * (r["i_chg"] * r["v_bat"] + self.last["i_chg"] * self.last["v_bat"]) * dt
            self.wh_out += 0.5 * (r["i_load"] * r["v_bat"] + self.last["i_load"] * self.last["v_bat"]) * dt
        r.update(iso=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(r["t"])),
                 phase=self.phase, vset=vset, ah_in=self.ah_in, ah_out=self.ah_out,
                 wh_in=self.wh_in, wh_out=self.wh_out)
        self.last = r
        path = os.path.join(self.out, "samples.csv")
        new = not os.path.exists(path)
        with open(path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
            if new:
                w.writeheader()
            w.writerow(r)
        self.status(r)
        if r["v_bat"] > V_ABORT:
            raise RuntimeError(f"battery at {r['v_bat']:.3f} V > {V_ABORT} V")
        return r

    def status(self, r=None):
        st = {"phase": self.phase, "updated": time.strftime("%Y-%m-%d %H:%M:%S"),
              "ah_in": self.ah_in, "ah_out": self.ah_out, "wh_in": self.wh_in,
              "wh_out": self.wh_out, "notes": self.notes, "last": r}
        tmp = os.path.join(self.out, "status.json.tmp")
        json.dump(st, open(tmp, "w"), indent=2)
        os.replace(tmp, os.path.join(self.out, "status.json"))

    b_psu_on = False

    def wait(self, s):
        time.sleep(max(0.0, s))

    # ---- phases ---------------------------------------------------------------
    def charge(self, name):
        b = self.b
        b.load_input(False)
        vset = V_ABSORB
        b.psu_set(vset, I_CHARGE)
        b.psu_output(True)
        self.b_psu_on = True
        t0 = time.time()
        t_abs = None
        hist = []                                    # (t, i) during absorption
        low_since = None
        i_cc = 0.0
        end_i = max(0.01 * self.ah_rated, 0.3)
        self.log(f"{name}: start, target {V_ABSORB} V at battery, {I_CHARGE} A, end at < {end_i:.2f} A")
        try:
            while True:
                self.wait(SAMPLE_S)
                r = self.sample(vset)
                vb, i = r["v_bat"], r["i_chg"]
                # sanity: supply terminal must sit at or above the battery while charging
                if not (-0.05 < r["v_psu"] - vb < 0.6):
                    self.log(f"v_psu {r['v_psu']:.3f} vs v_load {vb:.3f} implausible; "
                             "regulating on the supply's own reading")
                    vb = r["v_psu"]
                err = V_ABSORB - vb
                if abs(err) > 0.004:
                    step = max(-0.10, min(0.05, 0.5 * err))
                    new = round(max(13.0, min(VSET_MAX, vset + step)), 3)
                    if new != vset:
                        vset = new
                        b.psu_set(vset, I_CHARGE)
                i_cc = max(i_cc, i)
                # absorption = the supply has left current limit (current tapering),
                # not merely the battery touching 14.35 V while still at full current
                if t_abs is None and vb >= V_ABSORB - 0.05 and i < 0.93 * i_cc:
                    t_abs = time.time()
                    self.log(f"{name}: absorption reached after {(t_abs-t0)/3600:.2f} h, "
                             f"{self.ah_in:.2f} Ah in, I {i:.2f} A")
                    self.notes[f"{name}_t_bulk_h"] = (t_abs - t0) / 3600
                if t_abs:
                    hist.append((time.time(), i))
                    hist = [h for h in hist if time.time() - h[0] < 3700]
                    low_since = (low_since or time.time()) if i < end_i else None
                    reason = None
                    if low_since and time.time() - low_since > 600:
                        reason = f"current {i:.3f} A below {end_i:.2f} A for 10 min"
                    elif (time.time() - t_abs > 7200 and hist[0][0] < time.time() - 3500
                          and hist[0][1] - i < 0.05 and i < 0.9 * i_cc):
                        reason = f"current flat at {i:.3f} A for an hour (was {hist[0][1]:.3f})"
                    elif time.time() - t_abs > 6 * 3600:
                        reason = "6 h absorption limit"
                    if reason:
                        break
                if time.time() - t0 > 18 * 3600:
                    reason = "18 h charge limit"
                    break
                if int(time.time() - t0) % 600 < SAMPLE_S:
                    self.log(f"{name}: {(time.time()-t0)/3600:.2f} h  Vbat {vb:.3f}  Vset {vset:.3f}  "
                             f"I {i:.3f} A  {self.ah_in:.2f} Ah in")
        finally:
            b.psu_output(False)
            self.b_psu_on = False
        self.notes[f"{name}_end"] = {"reason": reason, "hours": (time.time() - t0) / 3600,
                                     "final_i": i, "ah_in_total": self.ah_in}
        self.log(f"{name}: done -- {reason}; {(time.time()-t0)/3600:.2f} h, {self.ah_in:.2f} Ah in total")

    def rest(self, name, secs):
        self.b.all_off()
        t0 = time.time()
        self.log(f"{name}: open-circuit rest {secs/3600:.1f} h")
        while time.time() - t0 < secs:
            self.wait(30)
            r = self.sample()
            if int(time.time() - t0) % 1800 < 30:
                self.log(f"{name}: {(time.time()-t0)/3600:.2f} h  OCV {r['v_bat']:.4f}")
        vs = []
        for _ in range(10):
            vs.append(self.sample()["v_bat"])
            self.wait(1)
        ocv = sum(vs) / len(vs)
        self.notes[f"{name}_ocv"] = ocv
        self.log(f"{name}: OCV after rest {ocv:.4f} V")

    def ir_full(self):
        self.log("ir_full: pulse test")
        s = ir_test.run(self.b, self.out, "ir_full")
        self.notes["ir_full_r_slope_mohm"] = s["r_slope_mohm"]
        self.log(f"ir_full: R_slope {s['r_slope_mohm']:.2f} mOhm; "
                 + ", ".join(f"{st['i_set']:g}A R_fast {st['r_fast_mohm']:.1f}" for st in s["steps"]))
        self.rest("ir_recover", 900)

    def discharge(self):
        b = self.b
        i_dis = self.ah_rated / 20.0
        target = 0.5 * self.ah_rated
        b.load_setup_cc(i_dis, 4 if i_dis < 4 else 40, DIS_VON)
        a0 = self.ah_out
        t0 = time.time()
        self.log(f"discharge: {i_dis:.2f} A until {target:.1f} Ah or Vbat <= {DIS_FLOOR} V "
                 f"(Von backstop {DIS_VON} V)")
        b.load_input(True)
        try:
            while True:
                self.wait(SAMPLE_S)
                r = self.sample()
                done = self.ah_out - a0
                if done >= target:
                    reason = f"removed {done:.2f} Ah (50 % of rated)"
                    break
                if r["v_bat"] <= DIS_FLOOR and r["i_load"] > 0.5 * i_dis:
                    reason = f"battery reached {r['v_bat']:.3f} V after {done:.2f} Ah"
                    break
                if r["i_load"] < 0.5 * i_dis and time.time() - t0 > 30:
                    reason = f"load stopped sinking ({r['i_load']:.2f} A, Vbat {r['v_bat']:.3f})"
                    break
                if int(time.time() - t0) % 1800 < SAMPLE_S:
                    self.log(f"discharge: {(time.time()-t0)/3600:.2f} h  Vbat {r['v_bat']:.3f}  "
                             f"I {r['i_load']:.3f}  {done:.2f} Ah out")
        finally:
            b.load_input(False)
        self.notes["discharge"] = {"reason": reason, "ah": self.ah_out - a0, "amps": i_dis,
                                   "hours": (time.time() - t0) / 3600,
                                   "v_end_loaded": r["v_bat"]}
        self.log(f"discharge: done -- {reason}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--start", default="charge", choices=PHASES)
    ap.add_argument("--ah", type=float, default=65.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    run = Run(a.out, a.ah)
    run.notes["ah_rated"] = a.ah
    run.connect()
    try:
        for ph in PHASES[PHASES.index(a.start):]:
            run.phase = ph
            run.status()
            {"charge": lambda: run.charge("charge"),
             "rest1": lambda: run.rest("rest1", REST1_S),
             "ir_full": run.ir_full,
             "discharge": run.discharge,
             "rest2": lambda: run.rest("rest2", REST2_S),
             "recharge": lambda: run.charge("recharge")}[ph]()
        run.phase = "finished"
        run.log("sequence finished")
    except BaseException as e:
        run.log(f"ABORT in {run.phase}: {e!r}\n{traceback.format_exc()}")
        run.phase = f"aborted in {run.phase}"
        raise
    finally:
        try:
            run.b.all_off()
        except Exception:
            pass
        all_off_fresh()
        run.status()


if __name__ == "__main__":
    main()
