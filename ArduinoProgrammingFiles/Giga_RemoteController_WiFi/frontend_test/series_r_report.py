"""
Companion to transient_report.py: what a series source resistance (here 1 Mohm in series with the
Rigol, standing in for a high-impedance source) does to the readings, against the 50 ohm baseline.

  py series_r_report.py BASE_TRANS BASE_BENCH SERIES_TRANS SERIES_BENCH [R_ohm] [out.pdf]
  py series_r_report.py runs/20261105-0754_transients_usb runs/20261105-0617_usb_bench \
                        runs/<stamp>_transients_usb_1M runs/<stamp>_usb_1M_series 1e6

Model: the baseline 3-pole fit plus ONE extra pole, fitted to (series sweep gain - baseline sweep
gain); that pole is R x C_in. Mains hum: the high-impedance node picks up 60 Hz. For the series run every
capture is analysed twice: as recorded, and after subtracting 60-300 Hz sinusoids fitted jointly with
the transient's own expected response (so the fit can't eat the pulses), then re-aligned.
"""
import json, math, os, sys
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
import transient_report as TR
from transient_report import (C_TRUE, C_MEAS, C_MODEL, C_AVG, C_BAND, fig_png, us, fmt_w, fmt_a,
                              stacked_panel, legend_strip, STD_LEGEND, window, fwhm, model_period, board_template)
from frontend_characterize import C2V, SCALE

MAINS = 60.0
HUM_K = range(1, 6)

def extra_pole(base_bench_dirs, ser_bench):
    L = lambda R: {round(p["f"]): p for p in R["sweep"]["points"]}
    bases = [L(json.load(open(os.path.join(d, "results.json")))) for d in base_bench_dirs]; S = L(ser_bench)
    f, r = [], []
    for k, p in sorted(S.items()):
        if k <= 50e3 and all(k in b for b in bases):
            f.append(k); r.append(p["gain_db"] - np.mean([b[k]["gain_db"] for b in bases]))
    f = np.array(f, float); r = np.array(r)
    q = least_squares(lambda q: q[0] - 10 * np.log10(1 + (f / 10 ** q[1]) ** 2) - r, [0, 3.9]).x
    fp = 10 ** q[1]; res = r - (q[0] - 10 * np.log10(1 + (f / fp) ** 2))
    return fp, q[0], f, r, float(np.sqrt(np.mean(res ** 2)))

def hum_cols(t, P):
    cols = []
    for k in HUM_K:
        fk = MAINS * k
        if P and abs(fk * P - round(fk * P)) < 0.05: continue   # coincides with a harmonic of the transient itself
        cols += [np.cos(2 * np.pi * fk * t), np.sin(2 * np.pi * fk * t)]
    return cols

def with_x(c, x):
    c2 = dict(c); c2["A2"] = list(x / (C2V * SCALE)); c2["A3"] = [0.0] * len(x); return c2

def analyse_dehum(c, poles, r):
    """(as recorded, hum removed, fitted hum amplitude at 60 Hz in V)."""
    A0 = TR.analyse(c, poles, r); x = A0["x"].copy(); n = len(x); t = np.arange(n) * A0["dt"]
    A = A0; hum60 = np.nan
    for _ in range(2):
        cols = hum_cols(t, A.get("P"))
        base =([A["m"] - A["m"].mean()] if "m" in A else []) + cols + [np.ones(n)]
        M = np.column_stack(base)
        co = np.linalg.lstsq(M, x, rcond=None)[0]
        o = 1 if "m" in A else 0
        hum = M[:, o:o + len(cols)] @ co[o:o + len(cols)]
        if len(cols) >= 2 and abs(MAINS * (A.get("P") or 1) - round(MAINS * (A.get("P") or 1))) >= 0.05:
            hum60 = float(np.hypot(co[o], co[o + 1]))
        A = TR.analyse(with_x(c, x - hum), poles, r)
        if "m" not in A: break
    return A0, A, hum60

def fit_poles_text(p): return ", ".join(f"{v/1e3:.1f} kHz" if v < 1e5 else f"{v/1e3:.0f} kHz" for v in p)

# ------------------------------------------------------------------ figures
def fig_resp(Mb, poles1, fp, fr, rr, ser_bench, R):
    fig, axs = plt.subplots(1, 2, figsize=(7.3, 2.7))
    ax = axs[0]; ff = np.logspace(1, 5.7, 400)
    sp = [p for p in ser_bench["sweep"]["points"] if p["f"] <= 520e3]
    ax.semilogx(Mb["f"], Mb["db"], "o", color=C_MEAS, ms=3.2, label="50 ohm source, measured")
    ax.semilogx(ff, Mb["mag"](ff), color=C_MEAS, lw=1.0)
    ax.semilogx([p["f"] for p in sp], [p["rel_db"] for p in sp], "s", color=C_MODEL, ms=3.2, label=f"{R/1e6:g} Mohm in series, measured")
    m1 = Mb["mag"](ff) - 10 * np.log10(1 + (ff / fp) ** 2)
    ax.semilogx(ff, m1 - m1[0] + Mb["mag"](ff)[0], color=C_MODEL, lw=1.0, ls="--", label=f"+ one pole at {fp/1e3:.2f} kHz")
    ax.set_ylim(-60, 3); ax.set_xlabel("frequency (Hz)"); ax.set_ylabel("response (dB)")
    ax.set_title("Frequency response", loc="left"); ax.legend(frameon=False, loc="lower left", fontsize=6.8)
    ax = axs[1]; tr = {"kind": "pulse", "width": 1e-3, "amp": 1.0, "period": 4e-3}
    t, u, y0 = model_period(tr, Mb["poles"]); TR._cache.clear(); _, _, y1 = model_period(tr, poles1); TR._cache.clear()
    k = t < 250e-6
    ax.plot(us(t[k]), u[k], color=C_TRUE, lw=1.1, label="input step")
    ax.plot(us(t[k]), y0[k], color=C_MEAS, lw=1.3, label="50 ohm source")
    ax.plot(us(t[k]), y1[k], color=C_MODEL, lw=1.3, ls="--", label=f"{R/1e6:g} Mohm source")
    ax.set_xlabel("time (us)"); ax.set_ylabel("V (per 1 V step)"); ax.set_title("Step response (from the fits)", loc="left")
    ax.legend(frameon=False, loc="lower right"); fig.tight_layout(); return fig_png(fig)

def fig_hum(cb, c1, R):
    fig, axs = plt.subplots(1, 2, figsize=(7.3, 2.7), gridspec_kw={"width_ratios": [1.5, 1]})
    for c, col, lab in ((cb, C_MEAS, "50 ohm source"), (c1, C_MODEL, f"{R/1e6:g} Mohm source")):
        x = TR.scaled(c.get("data", c)); fs = c["fs"]; t = np.arange(len(x)) / fs; k = t < 0.1
        axs[0].plot(t[k] * 1e3, x[k] * 1e3, color=col, lw=0.6, label=lab)
        w = np.hanning(len(x)); X = np.abs(np.fft.rfft((x - x.mean()) * w)) * 2 / w.sum(); f = np.fft.rfftfreq(len(x), 1 / fs)
        kk = f <= 1000; axs[1].plot(f[kk], X[kk] * 1e3, color=col, lw=0.9, label=lab)
    axs[0].set_xlabel("time (ms)"); axs[0].set_ylabel("recorded, input-referred (mV)")
    axs[0].set_title("Generator at 0 V, 10 kS/s: the first 100 ms", loc="left"); axs[0].legend(frameon=False, loc="upper right", fontsize=6.8)
    axs[1].set_xlabel("frequency (Hz)"); axs[1].set_ylabel("amplitude (mV peak)"); axs[1].set_title("Spectrum, 0-1 kHz", loc="left")
    axs[1].legend(frameon=False, loc="upper right", fontsize=6.8); fig.tight_layout(); return fig_png(fig)

def fig_pairs(pairs, title_amp, R, ylab_hum=True):
    fig = plt.figure(figsize=(7.3, 9.0))
    gs = fig.add_gridspec(len(pairs), 2, hspace=0.62, wspace=0.28, top=0.93, bottom=0.05)
    for i, (Ab, A1) in enumerate(pairs):
        w = Ab["tr"]["width"]; pre, post = max(20e-6, 0.25 * w), w + max(150e-6, 0.3 * w)
        _, a = stacked_panel(gs[i, 0], fig, Ab, f"{fmt_w(w)}, {title_amp}: 50 ohm source", pre, post)
        _, b = stacked_panel(gs[i, 1], fig, A1, f"{fmt_w(w)}, {title_amp}: {R/1e6:g} Mohm" + (", hum removed" if ylab_hum else ""), pre, post)
        lo = min(a.get_ylim()[0], b.get_ylim()[0]); hi = max(a.get_ylim()[1], b.get_ylim()[1]); a.set_ylim(lo, hi); b.set_ylim(lo, hi)
    legend_strip(fig, STD_LEGEND); return fig_png(fig)

def fig_area(Pb, P1, FWb, FW1, Mb, poles1, R):
    fig, axs = plt.subplots(1, 2, figsize=(7.3, 3.0))
    ws = np.logspace(np.log10(0.3e-6), np.log10(2e-3), 50)
    for poles, col, lab, pts, fwp, mk in ((Mb["poles"], C_MEAS, "50 ohm", Pb, FWb, "o"), (poles1, C_MODEL, f"{R/1e6:g} Mohm", P1, FW1, "s")):
        pk, fw = [], []
        for w in ws:
            tr = {"kind": "pulse", "width": w, "amp": 1.0, "period": max(2e-3, 10 * w)}
            t, u, y = model_period(tr, poles); pk.append(y.max()); fw.append(fwhm(t, y))
        TR._cache.clear()
        axs[0].loglog(us(ws), pk, color=col, lw=1.0, ls="--")
        axs[0].loglog([us(p[0]) for p in pts], [p[1] for p in pts], mk, color=col, ms=4, label=f"{lab}: measured (raw, 1 V)")
        axs[1].loglog(us(ws), us(np.array(fw)), color=col, lw=1.0, ls="--")
        axs[1].loglog([us(p[0]) for p in fwp], [us(p[1]) for p in fwp], mk, color=col, ms=4, label=f"{lab}: measured")
    axs[0].set_xlabel("true pulse width (us)"); axs[0].set_ylabel("recorded peak / true height"); axs[0].set_ylim(0.005, 1.5)
    axs[0].set_title("Peak height (dashes: expected)", loc="left"); axs[0].legend(frameon=False, loc="lower right", fontsize=6.8)
    axs[1].loglog(us(ws), us(ws), color=C_TRUE, lw=0.8, ls=":", label="apparent = true")
    axs[1].set_xlabel("true pulse width (us)"); axs[1].set_ylabel("apparent width at half height (us)")
    axs[1].set_title("Apparent duration", loc="left"); axs[1].legend(frameon=False, loc="upper left", fontsize=6.8)
    fig.tight_layout(); return fig_png(fig)

def fig_threshold(widths, curves):
    fig, ax = plt.subplots(figsize=(7.3, 3.0))
    for lab, col, ls, mk, ys in curves:
        ax.loglog(us(np.array(widths)), np.array(ys) * 1e3, marker=mk, ls=ls, color=col, ms=3.5, lw=1.2, label=lab)
    ax.set_xlabel("pulse width (us)"); ax.set_ylabel("smallest height seen at 5 sigma (mV)")
    ax.set_title("Smallest single-shot pulse you can see, 500 kS/s SMOOTH 8", loc="left")
    ax.legend(frameon=False, loc="upper right", fontsize=7); fig.tight_layout(); return fig_png(fig)

# ------------------------------------------------------------------ main
def main():
    bt, bb, st, sb = sys.argv[1:5]
    R = float(sys.argv[5]) if len(sys.argv) > 5 else 1e6
    here = os.path.dirname(os.path.abspath(__file__))
    out = sys.argv[6] if len(sys.argv) > 6 else os.path.join(here, f"Series_{R/1e6:g}Mohm_source_report.pdf")
    Tb = json.load(open(os.path.join(bt, "transients.json"))); T1 = json.load(open(os.path.join(st, "transients.json")))
    Bb = json.load(open(os.path.join(bb, "results.json"))); B1 = json.load(open(os.path.join(sb, "results.json")))
    base_benches = [bb] + [os.path.join(os.path.dirname(bb), d) for d in os.listdir(os.path.dirname(bb))
                           if d.endswith("_usb_bench_repeat") and d[:8] == os.path.basename(bb)[:8]]
    Mb = TR.fit_model(Bb); pb = Mb["poles"]
    fp, fdc, fr, rr, fres = extra_pole(base_benches, B1); p1 = sorted(pb + [fp])
    Cin = 1 / (2 * math.pi * fp * R)
    print(f"base poles {[round(p) for p in pb]}, extra {fp:.0f} Hz (C {Cin*1e12:.1f} pF, resid {fres:.2f} dB)")

    def prep(T, poles, dehum):
        caps = T["captures"]
        get = lambda tag, **kw: [c for c in caps if c["tag"] == tag and all(c["truth"].get(k) == v for k, v in kw.items())]
        TR._cache.clear()
        r, ppm = TR.clock_ratio(get("pulse_r8", width=50e-6, amp=1.0)[0], poles)
        if dehum:
            an = lambda c: analyse_dehum(c, poles, r)
        else:
            an = lambda c: (lambda A: (A, A, np.nan))(TR.analyse(c, poles, r))
        D = {"get": get, "r": r, "ppm": ppm, "caps": caps}
        D["G8"] = {(c["truth"]["width"], c["truth"]["amp"]): an(c) for c in get("pulse_r8")}
        D["G1"] = {(c["truth"]["width"], c["truth"]["amp"]): an(c) for c in get("pulse_r1")}
        D["n1"] = an(get("noise_r1")[0]); D["n8"] = an(get("noise_r8")[0])
        D["BR"] = {c["truth"]["freq"]: an(c) for c in get("burst_r1")}
        D["BR8"] = {c["truth"]["freq"]: an(c) for c in get("burst_r8")}
        return D
    Db = prep(Tb, pb, False); D1 = prep(T1, p1, True)
    widths = sorted({k[0] for k in Db["G8"]}); amps = sorted({k[1] for k in Db["G8"]})

    # validation: fitted gains on clear captures (hum removed)
    gains1 = [v[1]["a"] for v in list(D1["G8"].values()) + list(D1["G1"].values()) if v[1]["snr"] >= 10]
    gainsb = [v[1]["a"] for v in list(Db["G8"].values()) + list(Db["G1"].values()) if v[1]["snr"] >= 10]
    g1 = float(np.median(gains1)); gb = float(np.median(gainsb))
    g1lo, g1hi = np.percentile(gains1, 5), np.percentile(gains1, 95)
    hum60s = [v[2] for v in list(D1["G8"].values()) if not np.isnan(v[2])]
    hum60 = float(np.median(hum60s))
    sb1, sb8 = Db["n1"][1]["sigma"], Db["n8"][1]["sigma"]
    s1r, s8r = D1["n1"][0]["sigma"], D1["n8"][0]["sigma"]       # 1M, as recorded
    s1, s8 = D1["n1"][1]["sigma"], D1["n8"][1]["sigma"]         # 1M, hum removed
    print(f"sigma raw/S8: base {sb1*1e3:.1f}/{sb8*1e3:.1f}  1M as rec {s1r*1e3:.1f}/{s8r*1e3:.1f}  1M dehum {s1*1e3:.1f}/{s8*1e3:.1f}  hum60 {hum60*1e3:.0f} mV")
    print(f"gains: base median {gb:.3f}, 1M median {g1:.3f} ({g1lo:.2f}-{g1hi:.2f}, n={len(gains1)})")
    for k in sorted(D1["G8"]):
        print(f"  S8 w={k[0]*1e6:7.1f} A={k[1]*1e3:5.0f}  base a={Db['G8'][k][1]['a']:.2f} snr={Db['G8'][k][1]['snr']:6.1f} | 1M a={D1['G8'][k][1]['a']:.2f} snr={D1['G8'][k][1]['snr']:6.1f} (as rec {D1['G8'][k][0]['snr']:6.1f})")

    ppv = lambda D, G, g: [D[G][(w, 1.0)][1]["peak_model"] * g for w in widths]
    ppv8b, ppv8_1 = ppv(Db, "G8", gb), ppv(D1, "G8", g1)
    ppv1b, ppv1_1 = ppv(Db, "G1", gb), ppv(D1, "G1", g1)
    thr = lambda pp, s, w: 5 * s / np.interp(np.log(w), np.log(widths), pp)
    def avg_peak(A):
        w = A["tr"]["width"]; W = window(A, 1, 30e-6, w + 200e-6); return float(W["xavg"].max()) if W else np.nan
    def area_pts(D):
        P, F = [], []
        for w in widths:
            A = D["G1"][(w, 1.0)][1]; W = window(A, 1, 30e-6, w + 250e-6)
            if A["snr"] * math.sqrt(W["n"]) >= 8: P.append((w, float(W["xavg"].max())))
            if A["snr"] * math.sqrt(W["n"]) >= 15: F.append((w, fwhm(W["tavg"], W["xavg"])))
        return P, F
    Pb, FWb = area_pts(Db); P1, FW1 = area_pts(D1)
    def rise(D, poles):
        A = D["G1"][(1e-3, 1.0)][1]; W = window(A, 1, 30e-6, 400e-6)
        xa, ta = W["xavg"], W["tavg"]; hi = np.median(xa[ta > 250e-6])
        TR._cache.clear(); t, u, y = model_period({"kind": "pulse", "width": 1e-3, "amp": 1.0, "period": 1e-2}, poles); TR._cache.clear()
        return ta[np.argmax(xa > 0.9 * hi)] - ta[np.argmax(xa > 0.1 * hi)], (np.argmax(y > 0.9) - np.argmax(y > 0.1)) * TR.DTF
    rb, rbm = rise(Db, pb); r1, r1m = rise(D1, p1)
    def fwfloor(poles):
        TR._cache.clear(); t, u, y = model_period({"kind": "pulse", "width": 1e-6, "amp": 1.0, "period": 2e-3}, poles); TR._cache.clear()
        return fwhm(t, y), float(y.max())
    fwb, pk1b = fwfloor(pb); fw1, pk1_1 = fwfloor(p1)
    teq_b, teq_1 = 1e-6 / pk1b, 1e-6 / pk1_1        # peak = A * w / teq for short pulses
    burst = []
    for f in sorted(Db["BR"]):
        Ab, A1 = Db["BR"][f][1], D1["BR"][f][1]
        burst.append((f, Ab["peak_fit"] / Ab["tr"]["vpk"], A1["peak_fit"] / A1["tr"]["vpk"], Ab["snr"], A1["snr"]))
    dcb, dc1 = Bb["dc"], B1["dc"]
    print("rise", rb, rbm, r1, r1m, "fw", fwb, fw1, "teq", teq_b, teq_1); print("bursts", burst)
    print("dc keys", list(dcb.keys()) if isinstance(dcb, dict) else type(dcb))
    json.dump({"extra_pole_Hz": fp, "C_in_pF": Cin * 1e12, "fit_resid_db": fres, "poles_series": p1,
               "sigma_base_raw": sb1, "sigma_base_s8": sb8, "sigma_ser_raw_recorded": s1r, "sigma_ser_s8_recorded": s8r,
               "sigma_ser_raw_dehum": s1, "sigma_ser_s8_dehum": s8, "hum60_V": hum60, "gain_med_base": gb, "gain_med_ser": g1,
               "rise_base": [rb, rbm], "rise_ser": [r1, r1m], "fwhm_floor": [fwb, fw1], "teq": [teq_b, teq_1], "bursts": burst},
              open(os.path.join(st, "series_summary.json"), "w"), indent=1)

    # ------------- story
    Rs = f"{R/1e6:g} Mohm"
    f3b, f3s = Bb["sweep"]["f_3db"], B1["sweep"]["f_3db"]
    dgain = dc1["gain_vs_preset_pct"] - dcb["gain_vs_preset_pct"]
    doff = dc1["offset_mV_in"] - dcb["offset_mV_in"]
    ibias = abs(doff) * 1e-3 / R
    th = {w: (thr(ppv8b, sb8, w), thr(ppv8_1, s8, w), thr(ppv8_1, s8r, w)) for w in (1e-6, 10e-6, 100e-6, 1e-3)}
    fp10 = fp * R / 10e6
    sp = [("h1", f"A {Rs} source: what changes"),
          ("s", f"Bench test {T1['date'][:10]}: the same board, firmware and captures as the transient report "
                f"(Transient_visibility_report.pdf), repeated with a {Rs} resistor in series with the Rigol output, "
                f"standing in for a weak, high-impedance source. The Giga was on USB. Everything below compares the "
                f"{len(T1['captures'])} new captures with the {len(Tb['captures'])} from the 50 ohm run."),
          ("h2", "The short version"),
          ("ul", [f"<b>The source resistance and the input capacitance add a low-pass at {fp/1e3:.2f} kHz</b> "
                  f"(time constant {1e6/(2*math.pi*fp):.1f} us, so the input looks like about {Cin*1e12:.0f} pF: board plus "
                  f"wiring). The -3 dB point drops from {f3b/1e3:.1f} to {f3s/1e3:.1f} kHz and a step now takes "
                  f"{r1*1e6:.0f} us to rise 10-90 % (was {rb*1e6:.0f} us).",
                  f"<b>Short pulses lose more height.</b> Below ~15 us the recorded peak is still the pulse's area, but "
                  f"now height x width / {teq_1*1e6:.0f} us instead of / {teq_b*1e6:.0f} us: about {teq_1/teq_b:.1f}x smaller. "
                  f"Nothing looks shorter than ~{fw1*1e6:.0f} us (was ~{fwb*1e6:.0f} us).",
                  f"<b>Mains hum appears: {hum60*1e3:.0f} mV peak at 60 Hz</b> (it was under 1 mV with the 50 ohm source). "
                  f"The high-impedance node picks it up. As recorded, it raises the noise at SMOOTH 8 from {sb8*1e3:.1f} to "
                  f"{s8r*1e3:.0f} mV rms, and SMOOTH can't average it away.",
                  f"<b>Remove the hum and the noise is nearly back where it was</b>: {s8*1e3:.1f} mV at SMOOTH 8 and {s1*1e3:.1f} mV "
                  f"raw (was {sb8*1e3:.1f} / {sb1*1e3:.1f}). Little broadband noise is added; the small rise at SMOOTH 8 is "
                  f"low-frequency pickup the 60-300 Hz fit doesn't catch. Hum is "
                  f"slow (16.7 ms per cycle), so over a short transient it is only a drifting baseline.",
                  f"<b>DC is unaffected.</b> Gain moved {dgain:+.3f} %, which is inside the run-to-run scatter (up to 0.07 %), so the "
                  f"input resistance is above ~1.5 Gohm. The offset moved {doff:+.0f} mV, which would be about "
                  f"{ibias*1e9:.0f} nA of input current through {Rs}."]),
          ("h2", "Smallest single-shot pulse at 5 sigma, 500 kS/s SMOOTH 8"),
          ("table", [["true duration", "50 ohm source", f"{Rs}, hum removed", f"{Rs}, as recorded"]] +
                    [[fmt_w(w), f"{th[w][0]*1e3:.0f} mV", f"{th[w][1]*1e3:.0f} mV", f"{th[w][2]*1e3:.0f} mV"] for w in (1e-3, 100e-6, 10e-6)] +
                    [["1 us", f"{th[1e-6][0]:.2g} V", f"{th[1e-6][1]:.2g} V", f"{th[1e-6][2]:.2g} V"]],
           [0.22, 0.26, 0.26, 0.26]),
          ("sp", 4),
          ("s", f"Same method as the main report: 5 x the noise sigma at that setting, divided by the recorded fraction of "
                f"the height. The response with the extra pole is validated by the captures: the fitted amplitude on the "
                f"{len(gains1)} clear {Rs} captures has a median of {g1:.3f} ({g1lo:.2f} to {g1hi:.2f}, 5th-95th percentile), "
                f"against {gb:.3f} for the 50 ohm run. 'As recorded' uses the noise sigma including the hum, so it's the "
                f"pessimistic case: the hum is a slow baseline, and any local baseline removal recovers most of the "
                f"'hum removed' figure."),
          ("pb", None), ("h2", "1. Frequency response and step"),
          ("p", f"The {Rs} sweep (squares) is the 50 ohm sweep times one extra pole at {fp/1e3:.2f} kHz; the fit to the "
                f"difference is within {fres:.2f} dB rms up to 50 kHz. Poles of the {Rs} model: {fit_poles_text(p1)}."),
          ("img", fig_resp(Mb, p1, fp, fr, rr, B1, R)),
          ("h2", "2. Mains hum on a high-impedance input"),
          ("p", f"Generator at 0 V, so the board input sits behind {Rs} with nothing driving it. 60 Hz and its harmonics "
                f"appear; at the 50 ohm source they were below 1 mV. How much hum a real source picks up depends on the "
                f"wiring: leads, clips and their distance to mains wiring. It scales with the source impedance, so a "
                f"10 Mohm source would pick up several times more."),
          ("img", fig_hum(*[json.load(open(os.path.join(d, "raw_captures.json")))["noise_10000"] for d in (bb, sb)], R))]
    wsel = [1e-3, 100e-6, 20e-6, 5e-6, 1e-6]
    sp += [("pb", None), ("h2", "3. Pulse length at 1 V, 500 kS/s raw: 50 ohm vs " + Rs),
           ("p", f"Left the 50 ohm run, right {Rs} with the hum removed; same scales in each row. Long pulses still reach "
                 f"full height; their edges are slower. Short pulses come out smaller and wider."),
           ("img", fig_pairs([(Db["G1"][(w, 1.0)][1], D1["G1"][(w, 1.0)][1]) for w in wsel], "1 V raw", R))]
    wsel8 = [1e-3, 100e-6, 50e-6, 20e-6, 10e-6]
    sp += [("pb", None), ("h2", "4. Weak pulses: 100 mV, SMOOTH 8"),
           ("p", f"With the hum removed, the noise is close to the 50 ohm run ({s8*1e3:.1f} vs {sb8*1e3:.1f} mV), so long weak "
                 f"pulses are seen almost as well. The loss is at the short end, where the extra pole takes height away."),
           ("img", fig_pairs([(Db["G8"][(w, 0.1)][1], D1["G8"][(w, 0.1)][1]) for w in wsel8], "100 mV S8", R))]
    sp += [("pb", None), ("h2", "5. Height and apparent duration vs pulse length"),
           ("img", fig_area(Pb, P1, FWb, FW1, Mb, p1, R)),
           ("img", fig_threshold(widths, [("50 ohm source", C_MEAS, "-", "o", [5 * sb8 / p for p in ppv8b]),
                                          (f"{Rs}, hum removed", C_MODEL, "--", "s", [5 * s8 / p for p in ppv8_1]),
                                          (f"{Rs}, as recorded (hum in the noise)", C_TRUE, ":", "^", [5 * s8r / p for p in ppv8_1])])),
           ("h2", "6. Ringing (3-cycle bursts, 2 V peak, raw)"),
           ("table", [["frequency", "recorded / true, 50 ohm", f"recorded / true, {Rs}"]] +
                     [[f"{f/1e3:g} kHz", f"{a:.2f}" + ("" if sa >= 5 else " (noise)"), f"{b:.2f}" + ("" if sb_ >= 5 else " (noise)")]
                      for f, a, b, sa, sb_ in burst], [0.3, 0.35, 0.35])]
    sp += [("pb", None), ("h2", "7. What it means for a real high-impedance source"),
           ("ul", [f"<b>Expect the extra pole to scale with the source resistance.</b> With ~{Cin*1e12:.0f} pF at the input, "
                   f"10 Mohm would put it near {fp10:.0f} Hz (time constant {1e6/(2*math.pi*fp10):.0f} us). Then even 1 ms events "
                   f"would be rounded, and anything under ~100 us would be recorded only as its area. The capacitance is the "
                   f"lever: a shorter, lower-capacitance lead to the source or a buffer at the source helps more than any setting.",
                   "<b>Deal with the hum.</b> Twisted or shielded leads, with the shield on the input common, keep it out at the "
                   "source. Otherwise, subtract a fitted 60/120/180 Hz in software as this report does, or remove a local baseline "
                   "around each event. Either way, don't count on SMOOTH to remove it.",
                   f"<b>The offset shifts with source resistance.</b> {doff:+.0f} mV at {Rs} suggests ~{ibias*1e9:.0f} nA of input "
                   f"current; at 10 Mohm that would be ~{abs(doff)*10:.0f} mV. Null it with the source connected, "
                   f"not with the input shorted.",
                   "<b>Settings don't change.</b> 500 kS/s with SMOOTH 8 is still the best single-shot setting."]),
           ("h2", "Not covered"),
           ("ul", ["Only 1 Mohm was measured; the 10 Mohm numbers above are extrapolated from the capacitance and the offset shift.",
                   "The hum figure is for this bench's wiring. A different lead layout will pick up more or less."]),
           ("h2", "Reproducing this report"),
           ("p", "py frontend_characterize.py --board COM32 --label usb_1M_series<br/>"
                 "py transient_capture.py --board COM32 --label usb_1M<br/>"
                 "py series_r_report.py runs/&lt;base&gt;_transients_usb runs/&lt;base&gt;_usb_bench "
                 "runs/&lt;stamp&gt;_transients_usb_1M runs/&lt;stamp&gt;_usb_1M_series 1e6"),
           ("s", f"Data: {os.path.basename(bt)}, {os.path.basename(bb)}, {os.path.basename(st)}, {os.path.basename(sb)}.")]
    TR.build_pdf(out, sp)
    print("wrote", out)

if __name__ == "__main__":
    main()
