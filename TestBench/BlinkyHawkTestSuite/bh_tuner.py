"""
bh_tuner.py  --  analysis for the BlinkyHawk test suite.  Pure functions, no I/O.

Input is $DET passes from the bench firmware's !DETLOG (see parse_det).  Two
questions get answered:

1. VOLTAGE  (REFCENTER / REFBAND / VOLTFAST)
   With !DETLOG on, every pass reports the mean, min and max of ALL its
   VOLTAVG resting reads.  The firmware's voltage decision is
       any read beyond VOLTFAST*REFBAND of REFCENTER   (fast trip)
       or |mean - REFCENTER| > REFBAND                 (averaged)
   and "any read beyond k of c" is exactly "min < c-k or max > c+k", so each
   recorded pass can be re-decided OFFLINE for any candidate centre/band/fast
   -- no need to re-run the bench per candidate.

   A DC sweep from the generator gives the front end's transfer curve
   (input volts -> resting differential).  For a target trip voltage Vt:
       centre = (d(+Vt) + d(-Vt)) / 2      band = |d(+Vt) - d(-Vt)| / 2
   which puts the averaged-decision boundary at exactly +Vt and -Vt even when
   the two polarities have different gain.  Every no-voltage condition (the
   relay loads) is then re-decided with those numbers to look for false
   positives, and the smallest Vt that is clean is searched for and reported.

2. OPEN / CLOSED  (DETMETHOD / DETBAND / THRESHxx)
   The open/closed metric depends on REFCENTER (methods 1 and 2 measure
   |diff - REFCENTER|), so it is characterised AFTER the voltage settings are
   applied.  Per candidate metric, every relay load gets a distribution; the
   closed set (R <= closed_max) must sit wholly below the open set
   (R >= open_min).  If it does, the threshold goes between them weighted by
   their spreads; if not, the best achievable error rate is reported, plus a
   map of which adjacent loads CAN be cleanly told apart.
"""

import math

INF = float("inf")

# Firmware clamps (CFG_FIELDS in BlinkyHawk_Unified.ino) -- a proposal outside
# these would be silently clamped by !SET, so the tuner flags it instead.
LIMITS = {"REFCENTER": (-1.0, 1.0), "REFBAND": (0.001, 1.0), "VOLTFAST": (1.0, 50.0),
          "THRESH": (0.001, 3.3), "DETBAND": (0.005, 1.0)}

METHOD_UNITS = {0: "V", 1: "ms", 2: "V*ms"}


# ---------------------------------------------------------------------------
# parsing / stats
# ---------------------------------------------------------------------------
def _f(s):
    s = s.strip()
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        return None


DET_FIELDS = ["ms", "raw", "lead", "vpath", "n", "mean", "min", "max",
              "metric", "retms", "area", "thr", "rawkind", "kind", "vpos", "vneg"]

VOLT_KINDS = ("VDC+", "VDC-", "VAC")


def parse_det(line):
    """$DET,<ms>,<raw>,<lead>,<vpath>,<n>,<mean>,<min>,<max>,<metric>,<retms>,<area>,<thr>
           [,<rawkind>,<kind>,<vpos>,<vneg>]   (voltage kind: bench firmware v5+)"""
    f = line.split(",")
    if len(f) < 13 or f[0] != "$DET":
        return None
    f += [""] * (17 - len(f))
    try:
        return {"ms": int(f[1]), "raw": f[2], "lead": f[3], "vpath": f[4],
                "n": int(f[5]), "mean": _f(f[6]), "min": _f(f[7]), "max": _f(f[8]),
                "metric": _f(f[9]), "retms": _f(f[10]), "area": _f(f[11]),
                "thr": _f(f[12]), "rawkind": f[13].strip() or None,
                "kind": f[14].strip() or None, "vpos": _f(f[15]), "vneg": _f(f[16])}
    except ValueError:
        return None


def stats(vals):
    v = sorted(x for x in vals if x is not None and not math.isnan(x))
    n = len(v)
    if not n:
        return {"n": 0}
    mean = sum(v) / n
    std = math.sqrt(sum((x - mean) ** 2 for x in v) / (n - 1)) if n > 1 else 0.0

    def pct(p):
        k = (n - 1) * p
        lo, hi = int(math.floor(k)), int(math.ceil(k))
        return v[lo] + (v[hi] - v[lo]) * (k - lo)
    return {"n": n, "mean": mean, "std": std, "min": v[0], "max": v[-1],
            "p01": pct(0.01), "median": pct(0.5), "p99": pct(0.99)}


# ---------------------------------------------------------------------------
# voltage
# ---------------------------------------------------------------------------
def volt_decision(p, c, b, fast):
    """Re-run the firmware's voltagePresent() on one recorded pass.  None = no rest data."""
    if not p.get("n") or p.get("mean") is None:
        return None
    single = max(abs(p["min"] - c), abs(p["max"] - c))
    return single > fast * b or abs(p["mean"] - c) > b


def volt_rate(passes, c, b, fast):
    d = [volt_decision(p, c, b, fast) for p in passes]
    d = [x for x in d if x is not None]
    return (sum(d) / len(d)) if d else None


def debounce(raw_seq, stable_count, start="V"):
    """The firmware's display debounce, replayed over a raw-state sequence."""
    lead, cand, cnt, out = start, start, 0, []
    for r in raw_seq:
        if r == lead:
            cand, cnt = r, 0
        else:
            if r != cand:
                cand, cnt = r, 0
            cnt += 1
            if cnt >= stable_count:
                lead, cnt = r, 0
        out.append(lead)
    return out


class Transfer:
    """Piecewise-linear input-volts -> resting-differential curve from a DC sweep."""

    def __init__(self, points):
        # points: {vin: [passes]} -> median of the per-pass means
        self.pts = []
        for vin, passes in points.items():
            s = stats([p["mean"] for p in passes if p.get("n")])
            if s["n"]:
                self.pts.append((vin, s["median"], s["std"], s["n"]))
        self.pts.sort()

    @property
    def ok(self):
        return len(self.pts) >= 2

    @property
    def vrange(self):
        return (self.pts[0][0], self.pts[-1][0]) if self.pts else (0, 0)

    def __call__(self, v):
        p = self.pts
        if v <= p[0][0]:
            a, b = p[0], p[1]
        elif v >= p[-1][0]:
            a, b = p[-2], p[-1]
        else:
            for i in range(len(p) - 1):
                if p[i][0] <= v <= p[i + 1][0]:
                    a, b = p[i], p[i + 1]
                    break
        if b[0] == a[0]:
            return a[1]
        return a[1] + (b[1] - a[1]) * (v - a[0]) / (b[0] - a[0])

    def extrapolates(self, v):
        return v < self.pts[0][0] or v > self.pts[-1][0]

    def centre_band(self, vt):
        dp, dn = self(vt), self(-vt)
        return (dp + dn) / 2.0, abs(dp - dn) / 2.0, dp, dn


def _worst_novolt(novolt, c, b, fast):
    """Largest |mean-c|/b and single-read/(fast*b) over every no-voltage pass."""
    wm = ws = 0.0
    for passes in novolt.values():
        for p in passes:
            if not p.get("n") or p.get("mean") is None:
                continue
            wm = max(wm, abs(p["mean"] - c) / b)
            ws = max(ws, max(abs(p["min"] - c), abs(p["max"] - c)) / b)
    return wm, ws        # ws is in units of b, compare to fast


def tune_voltage(dc, novolt, ac, vt, fast, stable_count=2, guard=1.25):
    """
    dc      {vin: [passes]}  generator DC levels (include 0 V)
    novolt  {label: [passes]}  conditions that must NOT read as voltage (loads)
    ac      {label: [passes]}  conditions that MUST read as voltage
    vt      target trip voltage (V at the leads, both polarities)
    fast    current VOLTFAST
    guard   required ratio between the band and the worst no-voltage deviation
    Returns a result dict; 'lines' is the human report.
    """
    R = {"ok": False, "lines": [], "proposal": {}, "warnings": []}
    L = R["lines"]
    tf = Transfer(dc)
    R["transfer"] = tf
    if not tf.ok:
        L.append("VOLTAGE: not enough DC sweep points to build a transfer curve "
                 "(need the generator).")
        return R

    L.append("VOLTAGE -- front-end transfer (input V -> resting differential, median of passes)")
    for vin, d, sd, n in tf.pts:
        L.append(f"   {vin:+7.3f} V  ->  {d:+8.4f} V   (sd {sd * 1000:6.2f} mV, n={n})")
    for v in (vt, -vt):
        if tf.extrapolates(v):
            R["warnings"].append(f"{v:+.3f} V is outside the swept range "
                                 f"{tf.vrange[0]:+.2f}..{tf.vrange[1]:+.2f} V -- extrapolated")

    c, b, dp, dn = tf.centre_band(vt)
    zero = tf(0.0)
    if not ((dp - zero) * (dn - zero) < 0):
        L.append(f"   !! +{vt} V and -{vt} V do not move the differential in opposite "
                 f"directions (d(+)={dp:+.4f}, d(0)={zero:+.4f}, d(-)={dn:+.4f}).  The front "
                 "end is saturated or not responding at this level -- no symmetric band "
                 "exists.")
        return R
    R["centre"], R["band"] = c, b
    gain_p = (dp - zero) / vt
    gain_n = (zero - dn) / vt
    L.append(f"   gain: +side {gain_p * 1000:.1f} mV/V, -side {gain_n * 1000:.1f} mV/V"
             f"   (asymmetry {100 * (gain_p - gain_n) / max(abs(gain_p), 1e-9):+.1f} %)")
    L.append("")
    L.append(f"For a trip at +/-{vt:g} V:  REFCENTER = {c:+.4f}   REFBAND = {b:.4f}")
    for key, val in (("REFCENTER", c), ("REFBAND", b)):
        lo, hi = LIMITS[key]
        if not lo <= val <= hi:
            R["warnings"].append(f"{key}={val:.4f} is outside the firmware range "
                                 f"{lo}..{hi} and would be clamped")

    # --- false positives on the no-voltage conditions ---
    wm, ws = _worst_novolt(novolt, c, b, fast)
    need_fast = ws * guard
    new_fast = fast
    if need_fast > fast:
        new_fast = min(LIMITS["VOLTFAST"][1], math.ceil(need_fast * 10) / 10)
    L.append("")
    L.append(f"No-voltage conditions re-decided with those values (VOLTFAST {new_fast:g}):")
    fp_any = False
    for label, passes in novolt.items():
        rate = volt_rate(passes, c, b, new_fast)
        s = stats([p["mean"] for p in passes if p.get("n")])
        if rate is None:
            continue
        fp_any |= rate > 0
        dev = (abs(s["mean"] - c) / b) if s["n"] else float("nan")
        L.append(f"   {label:>10}: rest {s['mean']:+.4f} V (sd {s['std'] * 1000:.2f} mV), "
                 f"at {dev * 100:5.1f} % of band, false-voltage rate {rate * 100:5.1f} %")
    L.append(f"   worst averaged deviation {wm * 100:.1f} % of the band (want <= "
             f"{100 / guard:.0f} %), worst single read {ws:.2f} x band (VOLTFAST is {fast:g})")
    if new_fast != fast:
        L.append(f"   -> VOLTFAST {fast:g} lets single noisy reads trip; propose "
                 f"VOLTFAST = {new_fast:g}")
    clean_novolt = (wm * guard <= 1.0) and not fp_any

    # --- the smallest trip voltage that stays clean ---
    lo_v = max(0.01, min(abs(tf.vrange[0]), abs(tf.vrange[1])) * 0.01)
    hi_v = min(abs(tf.vrange[0]), abs(tf.vrange[1]))
    best = None
    v = lo_v
    while v <= hi_v + 1e-9:
        cc, bb, pp, nn = tf.centre_band(v)
        if bb > 0 and (pp - zero) * (nn - zero) < 0:
            m, _ = _worst_novolt(novolt, cc, bb, new_fast)
            if m * guard <= 1.0:
                best = v
                break
        v += 0.005
    R["min_clean_vt"] = best
    if best is None:
        L.append("   No trip voltage inside the swept range is clean of false positives -- "
                 "some no-voltage condition rests too far from the others.")
    else:
        L.append(f"   Smallest trip voltage that stays clean with a {guard:g}x guard: "
                 f"+/-{best:.3f} V")

    # --- how sharp the trip is: detection rate at every swept level ---
    fast_use = new_fast
    L.append("")
    L.append("Detection rate per swept DC level (per pass, re-decided):")
    lo_full = {1: None, -1: None}
    hi_none = {1: 0.0, -1: 0.0}
    for vin, passes in sorted(dc.items()):
        rate = volt_rate(passes, c, b, fast_use)
        if rate is None:
            continue
        sgn = 1 if vin > 0 else -1
        if vin != 0:
            if rate >= 1.0 and (lo_full[sgn] is None or abs(vin) < abs(lo_full[sgn])):
                lo_full[sgn] = vin
            if rate == 0.0:
                hi_none[sgn] = max(hi_none[sgn], abs(vin))
        L.append(f"   {vin:+7.3f} V : {rate * 100:6.1f} %")
    R["always_from"] = lo_full
    L.append(f"   Always detected from {lo_full[1]} V (+) and {lo_full[-1]} V (-); "
             f"never detected up to {hi_none[1]:g} V (+) / {hi_none[-1]:g} V (-)")

    # --- AC ---
    ac_ok = True
    if ac:
        L.append("")
        L.append("AC conditions (per pass, then through STABLECOUNT debounce):")
        for label, passes in ac.items():
            dec = [volt_decision(p, c, b, fast_use) for p in passes]
            dec = [d for d in dec if d is not None]
            if not dec:
                continue
            raw = ["V" if d else "C" for d in dec]
            lead = debounce(raw, stable_count, start="C")[stable_count + 2:]
            pr = sum(dec) / len(dec)
            lr = lead.count("V") / len(lead) if lead else 0.0
            ac_ok &= lr >= 1.0
            L.append(f"   {label:>10}: {pr * 100:5.1f} % of passes see voltage, lead state "
                     f"VOLTAGE {lr * 100:5.1f} % of the time")
            if pr < 1.0:
                L.append("              (the misses are passes whose ten reads landed near "
                         "a zero crossing -- raise VOLTAVG or lower the trip voltage)")

    R["proposal"] = {"REFCENTER": round(c, 4), "REFBAND": round(b, 4)}
    if new_fast != fast:
        R["proposal"]["VOLTFAST"] = new_fast
    R["ok"] = clean_novolt and ac_ok
    L.append("")
    L.append("VOLTAGE VERDICT: " + (
        f"clean -- trips at +/-{vt:g} V with no false positives" if R["ok"] else
        "NOT clean -- " + ("false positives on a no-voltage condition; "
                           f"smallest clean trip is {best}" if not clean_novolt
                           else "AC not held as VOLTAGE 100 % of the time")))
    return R


# ---------------------------------------------------------------------------
# open / closed
# ---------------------------------------------------------------------------
def _sort_key(item):
    r = item[1]
    return INF if r is None else r


def _fmt_r(r):
    if r is None:
        return "open"
    if r == 0:
        return "short"
    if r >= 1e6:
        return f"{r / 1e6:g}M"
    if r >= 1e3:
        return f"{r / 1e3:g}k"
    return f"{r:g}R"


def separation(values_by_load, closed_max, open_min):
    """
    values_by_load: [(label, ohms_or_None, [metric values])]
    Returns dict: clean, threshold, margin_sigma, margin_abs, error_rate,
                  per-load stats, adjacent-pair map, monotonic flag.
    """
    loads = sorted(values_by_load, key=_sort_key)
    rinf = lambda r: INF if r is None else r
    st = [(lab, r, stats(v), sorted(x for x in v if x is not None)) for lab, r, v in loads]
    st = [x for x in st if x[2]["n"]]
    out = {"loads": [(lab, r, s) for lab, r, s, _ in st]}
    closed = [x for x in st if rinf(x[1]) <= closed_max]
    opn = [x for x in st if rinf(x[1]) >= open_min]
    meds = [x[2]["median"] for x in st]
    out["monotonic"] = all(meds[i] <= meds[i + 1] for i in range(len(meds) - 1))

    # adjacent pairs: which neighbouring loads can be told apart at all
    pairs = []
    for a, b in zip(st, st[1:]):
        sa, sb = a[2], b[2]
        clean = sa["max"] < sb["min"]
        gap = sb["min"] - sa["max"]
        spread = max(sa["std"] + sb["std"], 1e-12)
        pairs.append((a[0], b[0], clean, gap, (sb["mean"] - sa["mean"]) / spread))
    out["pairs"] = pairs

    if not closed or not opn:
        out["clean"] = False
        out["error_rate"] = None
        out["threshold"] = None
        return out
    hi = max(x[2]["max"] for x in closed)
    lo = min(x[2]["min"] for x in opn)
    A = max(closed, key=lambda x: x[2]["mean"])       # closed load nearest the line
    B = min(opn, key=lambda x: x[2]["mean"])          # open load nearest the line
    if hi < lo:
        sa, sb = A[2]["std"], B[2]["std"]
        if sa + sb > 0:
            t = (A[2]["mean"] * sb + B[2]["mean"] * sa) / (sa + sb)
        else:
            t = (hi + lo) / 2
        # never let the spread weighting push the line past a recorded sample
        t = min(max(t, hi + 0.25 * (lo - hi)), lo - 0.25 * (lo - hi))
        ms_a = (t - A[2]["mean"]) / sa if sa > 0 else INF
        ms_b = (B[2]["mean"] - t) / sb if sb > 0 else INF
        out.update(clean=True, threshold=t, margin_abs=min(t - hi, lo - t),
                   margin_sigma=min(ms_a, ms_b), error_rate=0.0, nearest=(A[0], B[0]))
    else:
        # overlap: the threshold that misclassifies the fewest recorded passes
        cv = sorted(x for c in closed for x in c[3])
        ov = sorted(x for o in opn for x in o[3])
        cand = sorted(set(cv + ov))
        best_t, best_err = None, None
        n = len(cv) + len(ov)
        import bisect
        for i in range(len(cand)):
            t = cand[i] if i + 1 == len(cand) else (cand[i] + cand[i + 1]) / 2
            err = (len(cv) - bisect.bisect_right(cv, t)) + bisect.bisect_right(ov, t)
            if best_err is None or err < best_err:
                best_t, best_err = t, err
        out.update(clean=False, threshold=best_t, margin_abs=lo - hi, margin_sigma=None,
                   error_rate=best_err / n, nearest=(A[0], B[0]))
    return out


def tune_threshold(candidates, closed_max, open_min):
    """
    candidates: [{"label", "method", "settings": {key: val}, "data": [(label, ohms, [values])]}]
    Picks the clean candidate with the largest sigma margin, else the lowest error.
    """
    R = {"lines": [], "proposal": {}, "ok": False, "results": []}
    L = R["lines"]
    L.append(f"OPEN/CLOSED -- must read CLOSED at <= {_fmt_r(closed_max)} and OPEN at "
             f">= {_fmt_r(open_min if open_min != INF else None)}")
    best = None
    for cand in candidates:
        sep = separation(cand["data"], closed_max, open_min)
        cand = dict(cand, sep=sep)
        R["results"].append(cand)
        u = METHOD_UNITS[cand["method"]]
        L.append("")
        L.append(f"[{cand['label']}]  (DETMETHOD {cand['method']}, units {u})")
        for lab, r, s in sep["loads"]:
            L.append(f"   {lab:>7} ({_fmt_r(r):>6}): mean {s['mean']:9.4f}  sd {s['std']:8.4f}"
                     f"  range {s['min']:9.4f} .. {s['max']:9.4f}  n={s['n']}")
        if not sep["monotonic"]:
            L.append("   !! medians are not in resistance order -- this metric does not "
                     "track resistance monotonically here")
        cl = [f"{a}|{b}" for a, b, ok, _, _ in sep["pairs"] if ok]
        L.append("   clean cut points between adjacent loads: " + (", ".join(cl) or "none"))
        if sep.get("threshold") is None:
            L.append("   (no loads on one side of the boundary were recorded)")
            continue
        if sep["clean"]:
            L.append(f"   CLEAN: threshold {sep['threshold']:.4f} {u}, "
                     f"{sep['margin_sigma']:.1f} sigma / {sep['margin_abs']:.4f} {u} from the "
                     f"nearest recorded pass ({sep['nearest'][0]} vs {sep['nearest'][1]})")
            lo, hi = LIMITS["THRESH"]
            if not lo <= sep["threshold"] <= hi:
                L.append(f"   !! outside the THRESH range {lo}..{hi} -- unusable as is")
                continue
            if best is None or not best["sep"]["clean"] or \
                    sep["margin_sigma"] > best["sep"]["margin_sigma"]:
                best = cand
        else:
            L.append(f"   OVERLAP: {sep['nearest'][0]} and {sep['nearest'][1]} distributions "
                     f"overlap by {-sep['margin_abs']:.4f} {u}; best threshold "
                     f"{sep['threshold']:.4f} still misreads {sep['error_rate'] * 100:.2f} % "
                     "of passes")
            if best is None or (not best["sep"]["clean"] and
                                sep["error_rate"] < best["sep"]["error_rate"]):
                best = cand
    R["best"] = best
    L.append("")
    if best is None:
        L.append("OPEN/CLOSED VERDICT: nothing usable was recorded.")
        return R
    R["ok"] = best["sep"]["clean"]
    R["proposal"] = dict(best["settings"], DETMETHOD=best["method"])
    R["threshold"] = round(best["sep"]["threshold"], 4)
    L.append("OPEN/CLOSED VERDICT: " + (
        f"clean with [{best['label']}], threshold {R['threshold']} "
        f"{METHOD_UNITS[best['method']]}" if R["ok"] else
        f"NOT cleanly separable with any tried metric; least-bad is [{best['label']}] at "
        f"{best['sep']['error_rate'] * 100:.2f} % per-pass error.  See the cut points above "
        "for what CAN be discriminated."))
    return R


# ---------------------------------------------------------------------------
# per-condition summary (used by the sequencer and the verify pass)
# ---------------------------------------------------------------------------
def _combine(*results):
    r = [x for x in results if x]
    if not r:
        return ""
    return "FAIL" if "FAIL" in r else "PASS"


def summarize(label, expected, passes, stable_count=2, led=None, skip=None):
    """One condition's verdict.

    expected: 'C' | 'F' | 'V' (any voltage) | 'VDC+' | 'VDC-' | 'VAC' |
              '!V' (anything but voltage) | None (don't care)
    passes:   $DET passes (empty when the BlinkyHawk is on battery, no serial)
    led:      bh_led.LedDecoder.decode() result, or None without the LED sensor

    The serial verdict uses the DEBOUNCED lead state -- what the unit alerts --
    and, for a voltage kind, the debounced kind while lead is VOLTAGE (skipped
    if the firmware predates the classifier).  The LED verdict is what the
    unit actually showed.  Both must pass when both are available.

    skip: passes to drop at the start (default STABLECOUNT + 2, for the
    debounce to catch up).  Battery captures pass 0: their entries are one per
    log spacing (hundreds of ms), so the debounce has long settled by the first.
    """
    from bh_led import led_matches
    if skip is None:
        skip = stable_count + 2       # let the debounce catch up with the new condition
    use = passes[skip:] if len(passes) > skip + 3 else passes
    n = len(use)
    frac = lambda key, s: (sum(1 for p in use if p[key] == s) / n) if n else 0.0
    rest = stats([p["mean"] for p in use if p.get("n")])
    met = stats([p["metric"] for p in use])
    kinds = [p.get("kind") for p in use if p["lead"] == "V" and p.get("kind")]
    kind_major = max(set(kinds), key=kinds.count) if kinds else ""
    vpos = stats([p.get("vpos") for p in use])
    vneg = stats([p.get("vneg") for p in use])
    out = {"label": label, "expected": expected or "", "n": n,
           "lead_C": frac("lead", "C"), "lead_F": frac("lead", "F"), "lead_V": frac("lead", "V"),
           "raw_C": frac("raw", "C"), "raw_F": frac("raw", "F"), "raw_V": frac("raw", "V"),
           "kind": kind_major,
           "kind_frac": (kinds.count(kind_major) / len(kinds)) if kinds else None,
           "vpos_max": vpos.get("max"), "vneg_max": vneg.get("max"),
           "rest_mean": rest.get("mean"), "rest_sd": rest.get("std"),
           "rest_min": rest.get("min"), "rest_max": rest.get("max"),
           "metric_mean": met.get("mean"), "metric_sd": met.get("std"),
           "metric_min": met.get("min"), "metric_max": met.get("max")}

    serial_res = ""
    if n and expected:
        if expected == "!V":
            serial_res = "PASS" if out["lead_V"] == 0 else "FAIL"
        elif expected in ("C", "F"):
            serial_res = "PASS" if out["lead_" + expected] == 1.0 else "FAIL"
        else:                                        # V or a voltage kind
            ok = out["lead_V"] == 1.0
            if ok and expected in VOLT_KINDS and kinds:
                ok = all(k == expected for k in kinds)
            serial_res = "PASS" if ok else "FAIL"
    out["serial_result"] = serial_res

    led_res = ""
    if led is not None:
        out["led_state"] = led.get("state", "")
        out["led_colours"] = led.get("colours", "")
        out["led_n"] = led.get("n", 0)
        m = led_matches(expected, out["led_state"], led.get("counts"))
        led_res = "" if m is None else ("PASS" if m else "FAIL")
    out["led_result"] = led_res

    # Nothing judged although something was expected (e.g. battery mode with a
    # window too short for the LED) is NO DATA, never a silent blank.
    out["result"] = _combine(serial_res, led_res) or ("NO DATA" if expected else "")
    return out


SUMMARY_COLS = ["label", "expected", "result", "serial_result", "led_result", "n",
                "lead_C", "lead_F", "lead_V", "raw_C", "raw_F", "raw_V",
                "kind", "kind_frac", "vpos_max", "vneg_max",
                "led_state", "led_colours", "led_n",
                "rest_mean", "rest_sd", "rest_min", "rest_max",
                "metric_mean", "metric_sd", "metric_min", "metric_max"]
