"""
ir_test.py -- DC internal resistance of a 12 V battery by current pulses.

For each step: 3 s rest -> load ON at I for PULSE_S -> load OFF, 20 s recovery.
R_fast = drop at the first sample after the load settles (~0.3 s), mostly ohmic.
R_5s   = drop at the end of the pulse, which adds some polarisation.
Voltage is the PSU's sense of the battery (supply off, so no current flows
in its leads).  The load's own voltage reading is logged alongside it.

usage: py -3 ir_test.py OUTDIR [label]
"""
import csv
import json
import os
import sys
import time

from cbt_bench import Bench, all_off_fresh

STEPS = (2.0, 5.0, 10.0, 15.0)
PULSE_S = 5.0
REST_S = 20.0
VON = 10.5


def sample_for(b, seconds, tag, I, rows):
    t_end = time.time() + seconds
    while time.time() < t_end:
        r = b.read()
        r.update(tag=tag, i_set=I)
        rows.append(r)


def mean(xs):
    return sum(xs) / len(xs)


def run(b, out, label, close=False):
    """Pulse test on an open Bench.  Leaves both outputs off.  Returns summary."""
    rows, results = [], []
    summary = None
    try:
        b.psu_output(False)
        b.load_setup_cc(0.0, 40, VON)
        for I in STEPS:
            pre = []
            sample_for(b, 3.0, "rest", I, pre)
            v0 = mean([r["v_psu"] for r in pre[-5:]])
            vl0 = mean([r["v_load"] for r in pre[-5:]])
            b.load_level(I)
            t_on = time.time()
            b.load_input(True)
            pulse = []
            sample_for(b, PULSE_S, "pulse", I, pulse)
            b.load_input(False)
            post = []
            sample_for(b, REST_S, "recover", I, post)
            rows += pre + pulse + post
            settled = [r for r in pulse if r["t"] - t_on > 0.3 and r["i_load"] > 0.9 * I]
            if not settled:
                raise RuntimeError(f"load never reached {I} A (got {pulse[-1]['i_load']:.2f} A)")
            first, last = settled[0], settled[-3:]
            i_end = mean([r["i_load"] for r in last])
            res = {"i_set": I, "v_rest": v0, "i_meas": i_end,
                   "v_fast": first["v_psu"], "t_fast": first["t"] - t_on,
                   "v_end": mean([r["v_psu"] for r in last]),
                   "r_fast_mohm": (v0 - first["v_psu"]) / first["i_load"] * 1e3,
                   "r_5s_mohm": (v0 - mean([r["v_psu"] for r in last])) / i_end * 1e3,
                   "lead_drop_v": (vl0 - mean([r["v_load"] for r in last]))
                                  - (v0 - mean([r["v_psu"] for r in last])),
                   "v_recovered": mean([r["v_psu"] for r in post[-5:]])}
            results.append(res)
            print(f"{I:5.1f} A: rest {v0:.4f} V  end {res['v_end']:.4f} V  "
                  f"R_fast {res['r_fast_mohm']:.2f} mOhm (@{res['t_fast']:.2f}s)  "
                  f"R_5s {res['r_5s_mohm']:.2f} mOhm  load-lead drop {res['lead_drop_v']*1e3:.0f} mV",
                  flush=True)
        # slope of end-of-pulse V against I across all steps (cancels the
        # rest-voltage drift from one step to the next only partly)
        xs = [r["i_meas"] for r in results]
        ys = [r["v_rest"] - r["v_end"] for r in results]
        mx, my = mean(xs), mean(ys)
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
        summary = {"label": label, "steps": results, "r_slope_mohm": slope * 1e3,
                   "rows": len(rows), "psu": b.psu_id, "load": b.load_id}
        print(f"slope R (5 s, all steps): {slope*1e3:.2f} mOhm; {len(rows)} samples "
              f"({len(rows)/ (len(STEPS)*(PULSE_S+REST_S+3)):.1f}/s)")
    except BaseException:
        b.all_off()
        all_off_fresh()
        raise
    finally:
        b.all_off()
        if rows:
            with open(os.path.join(out, f"{label}_samples.csv"), "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
        if close:
            b.close()
    with open(os.path.join(out, f"{label}_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    return summary


def main():
    out = sys.argv[1]
    label = sys.argv[2] if len(sys.argv) > 2 else "ir"
    os.makedirs(out, exist_ok=True)
    run(Bench(), out, label, close=True)


if __name__ == "__main__":
    main()
