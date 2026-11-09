"""
Builds the transient PDF from a transient_capture.py run plus a frontend_characterize.py run.

  py transient_report.py runs/<stamp>_transients_usb runs/<stamp>_usb_bench [out.pdf]

How the captured data is lined up with the generator ("truth"):
  The board capture is free-running, so the time of each transient in the record
  is unknown. The generator repeats the transient at an exact period P. Every
  sample is folded onto the period's phase, and the phase is found by
  cross-correlating with the expected response (an FFT matched filter over all
  the events in the record). The ratio between the Giga and Rigol clocks is
  measured once, from a strong capture. The single-shot plots are one event
  from the record (the first complete one), not a selected or averaged one.
"""
import io, json, math, os, sys
import numpy as np
from scipy.signal import lfilter
from scipy.optimize import least_squares
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from frontend_characterize import C2V, SCALE

# palette: reference instance (light), validated categorical slots 1-3 + neutrals
C_TRUE, C_MEAS, C_MODEL, C_AVG = "#52514e", "#2a78d6", "#eb6834", "#1baf7a"
C_GRID, C_BAND, C_TEXT2 = "#e4e3df", "#ecebe7", "#52514e"
plt.rcParams.update({"font.size": 8, "axes.titlesize": 8.5, "axes.labelsize": 8, "xtick.labelsize": 7,
                     "ytick.labelsize": 7, "legend.fontsize": 7, "axes.edgecolor": "#b9b8b2",
                     "axes.linewidth": 0.6, "axes.grid": True, "grid.color": C_GRID, "grid.linewidth": 0.5,
                     "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.4,
                     "xtick.color": C_TEXT2, "ytick.color": C_TEXT2, "axes.labelcolor": C_TEXT2,
                     "axes.titlecolor": "#0b0b0b", "figure.dpi": 100})

DTF = 20e-9        # model simulation step
DELTA = 0.2e-6     # phase-bin width for alignment

# ------------------------------------------------------------------ model
def fit_model(bench):
    pts = [p for p in bench["sweep"]["points"] if p["f"] <= 520e3]
    f = np.array([p["f"] for p in pts]); db = np.array([p["rel_db"] for p in pts])
    def mag(q, f):
        return q[0] - sum(10 * np.log10(1 + (f / 10 ** l) ** 2) for l in q[1:])
    q = least_squares(lambda q: mag(q, f) - db, [0, 4.0, 5.3, 5.6]).x
    poles = sorted(10 ** q[1:])
    return {"poles": poles, "g_db": q[0], "f": f, "db": db, "mag": lambda ff: mag(np.r_[0, np.log10(poles)], ff)}

def respond(u, poles):
    y = u.astype(float)
    for fp in poles:
        a = math.exp(-2 * math.pi * fp * DTF); y = lfilter([1 - a], [1, -a], y)
    return y

def truth_wave(t, tr):
    if tr["kind"] == "pulse":
        return np.where((t >= 0) & (t < tr["width"]), tr["amp"], 0.0)
    if tr["kind"] == "burst":
        f = tr["freq"]; return np.where((t >= 0) & (t < tr["ncyc"] / f), tr["vpk"] * np.sin(2 * np.pi * f * t), 0.0)
    return np.zeros_like(t)

_cache = {}
def model_period(tr, poles):
    """Steady-state response over one period on the DTF grid (index 0 = input edge)."""
    key = json.dumps(tr, sort_keys=True)
    if key not in _cache:
        P = tr["period"]; n = int(round(P / DTF)); t = np.arange(n) * DTF
        u = truth_wave(t, tr); y = respond(np.r_[u, u], poles)[n:]
        _cache[key] = (t, u, y)
    return _cache[key]

def board_template(y, rate, N):
    """What one logged sample reads when its block starts at each phase: the mean of the N raw
    samples in the block (SMOOTH N), on the DTF grid."""
    m = int(round(1 / rate / DTF))
    return np.mean([np.roll(y, -k * m) for k in range(N)], axis=0)

# ------------------------------------------------------------------ data
def scaled(c):
    d = (np.array(c["A2"], float) - np.array(c["A3"], float)) * C2V * SCALE
    return d - np.median(d)

def align(x, dt, P, Tfine, r):
    """Edge time (s, in capture time) of the event at phase 0 that best matches the template."""
    nb = int(round(P / DELTA)); step = int(round(DELTA / DTF))
    Td = Tfine[::step][:nb]; Td = Td - Td.mean()
    ti = np.arange(len(x)) * dt * r
    b = np.floor((ti % P) / DELTA).astype(int) % nb
    X = np.bincount(b, weights=x - x.mean(), minlength=nb)
    C = np.fft.ifft(np.fft.fft(X) * np.conj(np.fft.fft(Td))).real
    s = int(np.argmax(C)); return s * DELTA, C[s]

def analyse(c, poles, r):
    tr = c["truth"]; rate = c["rate"]; N = c["smooth"]; dt = N / rate
    x = scaled(c); out = {"c": c, "x": x, "dt": dt, "N": N, "rate": rate, "tr": tr}
    if tr["kind"] == "zero":
        out["sigma"] = x.std(); return out
    t, u, y = model_period(tr, poles); T = board_template(y, rate, N); P = tr["period"]
    tau0, _ = align(x, dt, P, T, r)
    ti = np.arange(len(x)) * dt * r
    ph = (ti - tau0) % P
    m = np.interp(ph, t, T, period=P)
    a = float(np.dot(x - x.mean(), m - m.mean()) / np.dot(m - m.mean(), m - m.mean()))
    resid = x - x.mean() - a * (m - m.mean())
    out.update(tau0=tau0, a=a, sigma=float(resid.std()), m=m, ti=ti, t=t, u=u, y=y, T=T, P=P)
    out["peak_model"] = float(np.max(np.abs(T)))
    out["peak_fit"] = abs(a) * out["peak_model"]
    out["snr"] = out["peak_fit"] / out["sigma"]
    # events: edge times inside the record
    ks = np.arange(-1, int(ti[-1] / P) + 2); edges = tau0 + ks * P
    out["edges"] = edges[(edges >= 0) & (edges <= ti[-1])]
    return out

def window(A, k_choice, pre, post):
    """(times relative to the edge at the block centre, values) for one event, and the
    nearest-sample ensemble average over all complete events."""
    x, ti, N, rate = A["x"], A["ti"], A["N"], A["rate"]
    centre = (N - 1) / 2 / rate
    good = [e for e in A["edges"] if e - pre >= ti[0] and e + post <= ti[-1]]
    if not good:
        return None
    dt_r = ti[1] - ti[0]
    L = int(round((pre + post) / dt_r)) + 1
    segs, rels = [], []
    for e in good:
        j0 = int(round((e - pre) / dt_r))
        if j0 < 0 or j0 + L > len(x): continue
        segs.append(x[j0:j0 + L]); rels.append(ti[j0:j0 + L] - e + centre)
    k = min(k_choice, len(segs) - 1)
    return {"t1": rels[k], "x1": segs[k], "tavg": np.mean(rels, 0), "xavg": np.mean(segs, 0), "n": len(segs)}

def clock_ratio(c, poles):
    """Giga sample clock / nominal, from the fold sharpness of a strong capture."""
    tr = c["truth"]; rate = c["rate"]; N = c["smooth"]; dt = N / rate
    x = scaled(c); t, u, y = model_period(tr, poles); T = board_template(y, rate, N)
    best = None
    for ppm in np.arange(-300, 301, 5):
        _, v = align(x, dt, tr["period"], T, 1 + ppm * 1e-6)
        if best is None or v > best[1]: best = (ppm, v)
    for ppm in np.arange(best[0] - 5, best[0] + 5.01, 0.5):
        _, v = align(x, dt, tr["period"], T, 1 + ppm * 1e-6)
        if v > best[1]: best = (ppm, v)
    return 1 + best[0] * 1e-6, best[0]

# ------------------------------------------------------------------ figures
def fig_png(fig):
    b = io.BytesIO(); fig.savefig(b, format="png", dpi=220, bbox_inches="tight"); plt.close(fig); b.seek(0); return b

def us(v): return v * 1e6

def fmt_w(w):
    return f"{w*1e3:g} ms" if w >= 1e-3 else f"{w*1e6:g} us"

def fmt_a(a):
    return f"{a:g} V" if a >= 1 else f"{a*1e3:g} mV"

def stacked_panel(gs_cell, fig, A, title, pre, post, show_model=True, ylim=None):
    sub = gs_cell.subgridspec(2, 1, height_ratios=[1, 2.3], hspace=0.08)
    a0 = fig.add_subplot(sub[0]); a1 = fig.add_subplot(sub[1], sharex=a0)
    tt = np.linspace(-pre, post, 4000)
    a0.plot(us(tt), truth_wave(tt, A["tr"]) * 1e3, color=C_TRUE, lw=1.2)
    a0.set_title(title, loc="left"); a0.tick_params(labelbottom=False)
    a0.set_ylabel("true\nmV", fontsize=7)
    W = window(A, 1, pre, post)
    s = A["sigma"] * 1e3
    a1.axhspan(-3 * s, 3 * s, color=C_BAND, lw=0, zorder=0)
    if show_model:
        ph = np.linspace(-pre, post, 3000); centre = (A["N"] - 1) / 2 / A["rate"]
        mv = np.interp((ph - centre) % A["P"], A["t"], A["T"], period=A["P"])
        a1.plot(us(ph), mv * 1e3, color=C_MODEL, lw=1.1, ls="--", zorder=2)
    if W is not None:
        a1.plot(us(W["t1"]), W["x1"] * 1e3, color=C_MEAS, lw=1.0, marker="o", ms=2.2, zorder=3)
    a1.set_ylabel("recorded\nmV", fontsize=7); a1.set_xlabel("time from the input edge (us)")
    a1.set_xlim(us(-pre), us(post))
    if ylim: a1.set_ylim(*ylim)
    return a0, a1

def legend_strip(fig, items, y=0.995):
    hs = [plt.Line2D([], [], color=c, lw=lw, ls=ls, marker=mk, ms=3) for _, c, lw, ls, mk in items]
    fig.legend(hs, [l for l, *_ in items], loc="upper center", ncol=len(items), frameon=False,
               bbox_to_anchor=(0.5, y), fontsize=7.5)

STD_LEGEND = [("true input (generator setting)", C_TRUE, 1.2, "-", None),
              ("recorded, one event", C_MEAS, 1.0, "-", "o"),
              ("expected from the measured response", C_MODEL, 1.1, "--", None),
              ("noise +/-3 sigma", C_BAND, 6, "-", None)]

def fig_response(M, sig):
    fig, axs = plt.subplots(1, 2, figsize=(7.3, 2.6))
    ax = axs[0]; ff = np.logspace(1, 6, 400)
    ax.semilogx(M["f"], M["db"], "o", color=C_MEAS, ms=3.5, label="measured (2 Vpp sine)")
    ax.semilogx(ff, M["mag"](ff), color=C_MODEL, lw=1.2, label="3-pole fit")
    ax.set_xlabel("frequency (Hz)"); ax.set_ylabel("response (dB)"); ax.set_ylim(-60, 3)
    ax.set_title("Frequency response", loc="left"); ax.legend(frameon=False, loc="lower left")
    ax = axs[1]; tr = {"kind": "pulse", "width": 1e-3, "amp": 1.0, "period": 4e-3}
    t, u, y = model_period(tr, M["poles"]); k = t < 200e-6
    ax.plot(us(t[k]), u[k], color=C_TRUE, lw=1.2, label="input step")
    ax.plot(us(t[k]), y[k], color=C_MODEL, lw=1.4, label="front-end output")
    ax.set_xlabel("time (us)"); ax.set_ylabel("V (per 1 V step)"); ax.set_title("Step response (from the fit)", loc="left")
    ax.legend(frameon=False, loc="lower right")
    fig.tight_layout(); return fig_png(fig)

def fig_width_series(As, title_amp, pres):
    fig = plt.figure(figsize=(7.3, 8.6))
    gs = fig.add_gridspec(4, 2, hspace=0.55, wspace=0.28, top=0.93, bottom=0.05)
    for i, A in enumerate(As):
        w = A["tr"]["width"]; pre, post = pres(w, A)
        stacked_panel(gs[i // 2, i % 2], fig, A, f"{fmt_w(w)} pulse, {title_amp}", pre, post)
    legend_strip(fig, STD_LEGEND); return fig_png(fig)

def fig_area(P1, P8, P8w, M):
    fig, axs = plt.subplots(1, 2, figsize=(7.3, 3.0))
    ws = np.logspace(np.log10(0.3e-6), np.log10(2e-3), 60)
    for N, rate, col, lab, pts in ((1, 500e3, C_MEAS, "500 kS/s raw (average event)", P1),
                                   (8, 500e3, C_AVG, "500 kS/s SMOOTH 8 (fitted to all events)", P8)):
        mod = []
        for w in ws:
            tr = {"kind": "pulse", "width": w, "amp": 1.0, "period": max(2e-3, 10 * w)}
            t, u, y = model_period(tr, M["poles"]); mod.append(np.max(board_template(y, rate, N)))
        axs[0].loglog(us(ws), mod, color=col, lw=1.1, ls="--")
        axs[0].loglog([us(p[0]) for p in pts], [p[1] for p in pts], "o", color=col, ms=4, label=lab)
    axs[0].loglog(us(ws), np.minimum(1, ws / (1 / (2 * math.pi * M["poles"][0]))), color=C_TRUE, lw=0.8, ls=":",
                  label=f"A x width / {1e6/(2*math.pi*M['poles'][0]):.1f} us")
    axs[0].set_xlabel("true pulse width (us)"); axs[0].set_ylabel("recorded peak / true height")
    axs[0].set_title("Peak height: short pulses read as their area", loc="left")
    axs[0].legend(frameon=False, loc="lower right"); axs[0].set_ylim(0.01, 1.5)
    ax = axs[1]
    ax.loglog(us(ws), us(ws), color=C_TRUE, lw=0.8, ls=":", label="apparent = true")
    fw = []
    for w in ws:
        tr = {"kind": "pulse", "width": w, "amp": 1.0, "period": max(2e-3, 10 * w)}
        t, u, y = model_period(tr, M["poles"]); fw.append(fwhm(t, y))
    ax.loglog(us(ws), us(np.array(fw)), color=C_MODEL, lw=1.1, ls="--", label="expected")
    ax.loglog([us(p[0]) for p in P8w], [us(p[1]) for p in P8w], "o", color=C_MEAS, ms=4, label="measured (raw, 1 V)")
    ax.set_xlabel("true pulse width (us)"); ax.set_ylabel("apparent width at half height (us)")
    ax.set_title(f"Duration: nothing looks shorter than ~{us(min(fw)):.0f} us", loc="left"); ax.legend(frameon=False, loc="upper left")
    fig.tight_layout(); return fig_png(fig)

def fig_edge(A, M):
    fig, ax = plt.subplots(figsize=(7.3, 2.4))
    W = window(A, 1, 30e-6, 150e-6)
    tt = np.linspace(-30e-6, 150e-6, 2000)
    ax.plot(us(tt), truth_wave(tt, A["tr"]) * 1e3, color=C_TRUE, lw=1.2, label="true input (1 V step)")
    centre = (A["N"] - 1) / 2 / A["rate"]
    mv = np.interp((tt - centre) % A["P"], A["t"], A["T"], period=A["P"])
    ax.plot(us(tt), mv * 1e3, color=C_MODEL, lw=1.1, ls="--", label="expected")
    ax.plot(us(W["tavg"]), W["xavg"] * 1e3, "o-", color=C_MEAS, ms=2.5, lw=0.9, label=f"recorded, average of {W['n']} edges (raw)")
    ax.set_xlabel("time from the input edge (us)"); ax.set_ylabel("mV"); ax.set_xlim(-30, 150)
    ax.set_title("Rising edge of a 1 ms, 1 V pulse", loc="left"); ax.legend(frameon=False, loc="lower right")
    fig.tight_layout(); return fig_png(fig)

def fwhm(t, y):
    y = np.asarray(y); pk = y.max()
    if pk <= 0: return np.nan
    above = np.flatnonzero(y >= pk / 2)
    return (above[-1] - above[0]) * (t[1] - t[0])

def fig_ladder(rows):
    fig = plt.figure(figsize=(7.3, 8.8))
    gs = fig.add_gridspec(len(rows), 3, hspace=0.62, wspace=0.3, top=0.93, bottom=0.05)
    pre, post = 60e-6, 260e-6
    for i, (amp, A1, A8) in enumerate(rows):
        for j, (A, lab, avg) in enumerate(((A1, "500 kS/s raw, one event", False), (A8, "SMOOTH 8, one event", False),
                                           (A8, "SMOOTH 8, average of all events", True))):
            ax = fig.add_subplot(gs[i, j])
            if A is None: ax.axis("off"); continue
            W = window(A, 1, pre, post); s = A["sigma"] * 1e3
            if avg and W: s = s / math.sqrt(W["n"])
            ax.axhspan(-3 * s, 3 * s, color=C_BAND, lw=0, zorder=0)
            tt = np.linspace(-pre, post, 2000)
            ax.plot(us(tt), truth_wave(tt, A["tr"]) * 1e3, color=C_TRUE, lw=1.0, zorder=1)
            if W:
                if avg:
                    ax.plot(us(W["tavg"]), W["xavg"] * 1e3, color=C_AVG, lw=1.1, marker="o", ms=2, zorder=3)
                else:
                    ax.plot(us(W["t1"]), W["x1"] * 1e3, color=C_MEAS, lw=0.9, marker="o", ms=1.8 if A["N"] == 1 else 2.4, zorder=3)
            ttl = f"{fmt_a(amp)}: {lab}" + (f" ({W['n']})" if avg and W else "")
            ax.set_title(ttl, loc="left", fontsize=7.5)
            if i == len(rows) - 1: ax.set_xlabel("time (us)")
            if j == 0: ax.set_ylabel("mV")
            span = max(amp * 1e3 * 1.25, 4 * s, 1)
            ax.set_ylim(-span * (0.9 if avg else 1.0), span * 1.35)
    legend_strip(fig, [("true input", C_TRUE, 1.0, "-", None), ("recorded, one event", C_MEAS, 0.9, "-", "o"),
                       ("average of every event in the record", C_AVG, 1.1, "-", "o"),
                       ("noise +/-3 sigma", C_BAND, 6, "-", None)])
    return fig_png(fig)

def fig_map(G8, widths, amps, sig8):
    fig, ax = plt.subplots(figsize=(7.3, 3.9))
    Z = np.full((len(amps), len(widths)), np.nan)
    for (w, a), A in G8.items():
        Z[amps.index(a), widths.index(w)] = A["snr"]
    cat = np.digitize(Z, [3, 5, 10])        # 0 <3, 1 3-5, 2 5-10, 3 >=10
    cols = ["#f1f0ec", "#c5daf3", "#7aa9e6", "#2a78d6"]
    from matplotlib.colors import ListedColormap
    ax.imshow(cat, cmap=ListedColormap(cols), vmin=-0.5, vmax=3.5, aspect="auto", origin="lower")
    for i in range(len(amps)):
        for j in range(len(widths)):
            v = Z[i, j]
            ax.text(j, i, f"{v:.1f}" if v < 10 else f"{v:.0f}", ha="center", va="center", fontsize=6.8,
                    color="#ffffff" if cat[i, j] == 3 else "#0b0b0b")
    ax.set_xticks(range(len(widths)), [fmt_w(w).replace(" ", "\n") for w in widths])
    ax.set_yticks(range(len(amps)), [fmt_a(a) for a in amps]); ax.grid(False)
    ax.set_xlabel("pulse width"); ax.set_ylabel("pulse height")
    ax.set_title(f"Single-shot visibility at 500 kS/s SMOOTH 8: recorded peak / noise sigma (sigma = {sig8*1e3:.1f} mV)",
                 loc="left")
    hs = [plt.Rectangle((0, 0), 1, 1, color=c) for c in cols]
    ax.legend(hs, ["< 3: lost in the noise", "3-5: visible if you know where to look", "5-10: clearly visible",
                   ">= 10: clean"], loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=4, frameon=False, fontsize=7)
    fig.tight_layout(); return fig_png(fig)

def fig_threshold(widths, ppv1, ppv8, sig1, sig8, nev):
    fig, ax = plt.subplots(figsize=(7.3, 2.9))
    w = np.array(widths)
    ax.loglog(us(w), 5 * sig1 / np.array(ppv1) * 1e3, "o-", color=C_MEAS, ms=3.5, label="500 kS/s raw, one event")
    ax.loglog(us(w), 5 * sig8 / np.array(ppv8) * 1e3, "o-", color=C_AVG, ms=3.5, label="500 kS/s SMOOTH 8, one event")
    ax.loglog(us(w), 5 * sig8 / np.array(ppv8) / math.sqrt(nev) * 1e3, "o--", color=C_MODEL, ms=3.5,
              label=f"SMOOTH 8, averaged over {nev} repeats")
    ax.set_xlabel("pulse width (us)"); ax.set_ylabel("smallest height seen at 5 sigma (mV)")
    ax.set_title("Smallest pulse you can see, by duration", loc="left"); ax.legend(frameon=False, loc="upper right")
    fig.tight_layout(); return fig_png(fig)

def fig_bursts(rows):
    fig = plt.figure(figsize=(7.3, 8.6))
    gs = fig.add_gridspec(len(rows), 2, hspace=0.6, wspace=0.28, top=0.93, bottom=0.05)
    for i, (A1, A8) in enumerate(rows):
        f = A1["tr"]["freq"]; dur = 3 / f
        pre = max(20e-6, 0.3 * dur); post = dur + max(120e-6, 0.4 * dur)
        stacked_panel(gs[i, 0], fig, A1, f"{f/1e3:g} kHz, 3 cycles, 2 V peak: raw", pre, post)
        stacked_panel(gs[i, 1], fig, A8, f"{f/1e3:g} kHz: SMOOTH 8", pre, post)
    legend_strip(fig, STD_LEGEND); return fig_png(fig)

def fig_settings(As):
    fig = plt.figure(figsize=(7.3, 6.7))
    gs = fig.add_gridspec(3, 2, hspace=0.6, wspace=0.28, top=0.92, bottom=0.06)
    for i, A in enumerate(As):
        lab = f"{A['rate']/1e3:g} kS/s" + (f" SMOOTH {A['N']}" if A["N"] > 1 else " raw") + \
              f": {A['dt']*1e6:g} us/point, sigma {A['sigma']*1e3:.1f} mV"
        stacked_panel(gs[i // 2, i % 2], fig, A, lab, 60e-6, 260e-6, ylim=(-110, 330))
    legend_strip(fig, STD_LEGEND); return fig_png(fig)

# ------------------------------------------------------------------ PDF
def build_pdf(out_path, story_parts):
    from reportlab.lib.pagesizes import letter
    from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Image, Table, TableStyle, KeepTogether
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.lib import colors
    from reportlab.lib.units import inch
    ss = getSampleStyleSheet()
    H1 = ParagraphStyle("H1", parent=ss["Heading1"], fontName="Helvetica-Bold", fontSize=17, leading=21, spaceAfter=4,
                        textColor=colors.HexColor("#0b0b0b"))
    H2 = ParagraphStyle("H2", parent=ss["Heading2"], fontName="Helvetica-Bold", fontSize=12.5, leading=16, spaceBefore=2,
                        spaceAfter=4, textColor=colors.HexColor("#0b0b0b"))
    B = ParagraphStyle("B", parent=ss["BodyText"], fontName="Helvetica", fontSize=9.2, leading=12.6, spaceAfter=5,
                       textColor=colors.HexColor("#1c1c1b"))
    S = ParagraphStyle("S", parent=B, fontSize=8, leading=10.5, textColor=colors.HexColor("#52514e"))
    BL = ParagraphStyle("BL", parent=B, leftIndent=12, bulletIndent=2)
    doc = SimpleDocTemplate(out_path, pagesize=letter, leftMargin=0.6 * inch, rightMargin=0.6 * inch,
                            topMargin=0.55 * inch, bottomMargin=0.55 * inch,
                            title="Transients through the 12 V front end", author="Giga_RemoteController_WiFi bench test")
    fw = letter[0] - 1.2 * inch
    def page_no(c, d):
        c.saveState(); c.setFont("Helvetica", 7.5); c.setFillColor(colors.HexColor("#8a8984"))
        c.drawRightString(letter[0] - 0.6 * inch, 0.35 * inch, f"Transients through the 12 V front end  |  page {d.page}")
        c.restoreState()
    flow = []
    for kind, val, *extra in story_parts:
        if kind == "h1": flow.append(Paragraph(val, H1))
        elif kind == "h2": flow.append(Paragraph(val, H2))
        elif kind == "p": flow.append(Paragraph(val, B))
        elif kind == "s": flow.append(Paragraph(val, S))
        elif kind == "ul":
            for item in val: flow.append(Paragraph(item, BL, bulletText="•"))
        elif kind == "img":
            from reportlab.lib.utils import ImageReader
            ir = ImageReader(val); iw, ih = ir.getSize(); w = fw * (extra[0] if extra else 1.0)
            flow.append(Image(val, width=w, height=w * ih / iw)); val.seek(0)
        elif kind == "table":
            rows = [[Paragraph(str(c), S if r else ParagraphStyle("th", parent=S, fontName="Helvetica-Bold",
                                                              textColor=colors.HexColor("#0b0b0b")))
                     for c in row] for r, row in enumerate(val)]
            cw = extra[0] if extra else None
            t = Table(rows, colWidths=[fw * x for x in cw] if cw else None, hAlign="LEFT", repeatRows=1)
            t.setStyle(TableStyle([("LINEBELOW", (0, 0), (-1, 0), 0.8, colors.HexColor("#b9b8b2")),
                                   ("LINEBELOW", (0, 1), (-1, -1), 0.3, colors.HexColor("#e4e3df")),
                                   ("VALIGN", (0, 0), (-1, -1), "TOP"), ("TOPPADDING", (0, 0), (-1, -1), 2.5),
                                   ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5)]))
            flow.append(t)
        elif kind == "sp": flow.append(Spacer(1, val))
        elif kind == "pb": flow.append(PageBreak())
    doc.build(flow, onFirstPage=page_no, onLaterPages=page_no)

# ------------------------------------------------------------------ main
def main():
    tdir, bdir = sys.argv[1], sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                             "Transient_visibility_report.pdf")
    T = json.load(open(os.path.join(tdir, "transients.json"))); bench = json.load(open(os.path.join(bdir, "results.json")))
    M = fit_model(bench); poles = M["poles"]
    tau1 = 1 / (2 * math.pi * poles[0])
    caps = T["captures"]
    get = lambda tag, **kw: [c for c in caps if c["tag"] == tag and all(c["truth"].get(k) == v for k, v in kw.items())]
    ref = get("pulse_r8", width=50e-6, amp=1.0)[0]
    r, ppm = clock_ratio(ref, poles)
    print(f"poles {[round(p) for p in poles]} Hz, tau1 {tau1*1e6:.1f} us, clock ratio {ppm:+.1f} ppm")
    An = lambda c: analyse(c, poles, r)
    sig1 = An(get("noise_r1")[0])["sigma"]; sig8 = An(get("noise_r8")[0])["sigma"]
    widths = sorted({c["truth"]["width"] for c in caps if c["tag"] == "pulse_r8"})
    amps = sorted({c["truth"]["amp"] for c in caps if c["tag"] == "pulse_r8"})
    G8 = {(c["truth"]["width"], c["truth"]["amp"]): An(c) for c in caps if c["tag"] == "pulse_r8"}
    G1 = {(c["truth"]["width"], c["truth"]["amp"]): An(c) for c in caps if c["tag"] == "pulse_r1"}
    for k, A in sorted(G8.items()):
        print(f"  S8 w={k[0]*1e6:7.1f}us A={k[1]*1e3:6.0f}mV  a={A['a']:.3f}  snr={A['snr']:6.1f}  sigma={A['sigma']*1e3:.1f}")

    raw_amps = sorted({k[1] for k in G1})
    # how well the prediction matches: fitted gain of every capture with a clear signal
    strong = [A for A in list(G8.values()) + list(G1.values()) if A["snr"] >= 10]
    gains = [A["a"] for A in strong]; g_med = float(np.median(gains))
    g_lo, g_hi = np.percentile(gains, 5), np.percentile(gains, 95)
    # area plot: raw = peak of the average event (1 V); SMOOTH 8 = fitted amplitude x predicted peak
    def avg_peak(A):
        w = A["tr"]["width"]; W = window(A, 1, 30e-6, w + 150e-6)
        return float(W["xavg"].max()) if W else np.nan
    P1 = [(w, avg_peak(G1[(w, 1.0)])) for w in widths if G1[(w, 1.0)]["snr"] * math.sqrt(10) >= 8]
    P8 = [(w, G8[(w, 1.0)]["peak_fit"]) for w in widths if G8[(w, 1.0)]["snr"] >= 3]
    P1w = []
    for w in widths:
        A = G1[(w, 1.0)]; W = window(A, 1, 30e-6, w + 150e-6)
        if A["snr"] * math.sqrt(W["n"]) >= 15:
            P1w.append((w, fwhm(W["tavg"], W["xavg"])))
    fw10 = dict(P1w).get(10e-6, np.nan)
    tr1 = {"kind": "pulse", "width": 1e-6, "amp": 1.0, "period": 2e-3}
    t_, u_, y_ = model_period(tr1, poles); fw_floor = fwhm(t_, y_)
    # peak per volt of input, from the validated response x the median fitted gain
    ppv1 = [G1[(w, 1.0)]["peak_model"] / 1.0 * g_med for w in widths]
    ppv8 = [G8[(w, 1.0)]["peak_model"] / 1.0 * g_med for w in widths]
    nev = 16
    ratio = sig1 / sig8

    # rise time, raw, 1 V 1 ms
    A = G1[(1e-3, 1.0)]; W = window(A, 1, 30e-6, 300e-6)
    xa, ta = W["xavg"], W["tavg"]; hi = np.median(xa[ta > 150e-6])
    t10 = ta[np.argmax(xa > 0.1 * hi)]; t90 = ta[np.argmax(xa > 0.9 * hi)]
    rise_meas = t90 - t10
    t, u, y = model_period({"kind": "pulse", "width": 1e-3, "amp": 1.0, "period": 1e-2}, poles)
    rise_mod = (np.argmax(y > 0.9) - np.argmax(y > 0.1)) * DTF

    # bursts
    BR = [(An(get("burst_r1", freq=f)[0]), An(get("burst_r8", freq=f)[0])) for f in sorted({c["truth"]["freq"] for c in caps if c["tag"] == "burst_r1"})]
    # settings
    SA = [An(c) for c in get("settings")]

    def thr(ppv, sig, w):  # 5-sigma threshold height at width w
        return 5 * sig / np.interp(np.log(w), np.log(widths), ppv)
    th = {w: (thr(ppv1, sig1, w), thr(ppv8, sig8, w)) for w in (1e-6, 10e-6, 100e-6, 1e-3)}

    def pick(G, a, ws):
        return [G[(w, a)] for w in ws]
    wsel = [1e-3, 200e-6, 50e-6, 20e-6, 10e-6, 5e-6, 2e-6, 1e-6]
    def pres(w, A):
        return max(20e-6, 0.25 * w), w + max(120e-6, 0.3 * w)

    # ------------- story
    sp = []
    sp += [("h1", "Transients through the 12 V front end: what you can and can't see"),
           ("s", f"Bench test {T['date'][:10]}: Arduino Giga R1 (firmware Giga_RemoteController_WiFi, dual-ADC build) reading the "
                 "12 V op-amp buffer board, preset OpAmpBuffer_12vboard (input = 7.005 x (A2 - A3)). Transients came from a Rigol "
                 f"DG852 Pro into the board input; the Giga was on USB to the PC. Every \"recorded\" trace in this document is real "
                 f"data from the board; {len(caps)} captures in total."),
           ("h2", "The short version"),
           ("ul", [
               f"<b>The front end is a low-pass filter with a {tau1*1e6:.1f} us time constant</b> (-3 dB at {bench['sweep']['f_3db']/1e3:.1f} kHz). "
               f"Every transient comes out smoothed by it: a step takes {rise_meas*1e6:.0f} us to rise 10-90 %.",
               f"<b>Pulses longer than about 50 us</b> are recorded with their true height and a duration you can read off the "
               f"plot (the edges are rounded by the {rise_meas*1e6:.0f} us rise).",
               f"<b>Pulses shorter than about 15 us lose height in proportion to their length</b>: what you see is their "
               f"<i>area</i>. Recorded peak = height x width / {tau1*1e6:.1f} us. A 1 V, 1 us pulse reads about "
               f"{ppv1[widths.index(1e-6)]*1e3:.0f} mV, and it always looks about {fw_floor*1e6:.0f} us wide, however short it really was.",
               f"<b>Noise sets the floor.</b> At 500 kS/s raw the noise is {sig1*1e3:.0f} mV rms; with SMOOTH 8 it is "
               f"{sig8*1e3:.1f} mV. Single-shot, a 100 us pulse needs about {thr(ppv8, sig8, 100e-6)*1e3:.0f} mV to stand out "
               f"clearly (5 sigma) with SMOOTH 8, a 10 us pulse about {thr(ppv8, sig8, 10e-6)*1e3:.0f} mV and a 1 us pulse "
               f"about {thr(ppv8, sig8, 1e-6):.2g} V.",
               f"<b>For weak transients use 500 kS/s with SMOOTH 8.</b> It sees everything raw 500 kS/s sees, at {ratio:.1f}x lower "
               "noise. Lower sample rates without SMOOTH can step right over a short event.",
               "<b>Repeating transients can be averaged</b>: averaging N events lowers the floor by sqrt(N); 16 events take a "
               "pulse that is invisible in one shot to clearly visible.",
               "<b>Fast ringing shrinks with frequency</b>: a 10 kHz ring keeps about three quarters of its size, 30 kHz "
               "about a third, 100 kHz about a tenth (still visible raw) and 300 kHz only a few % (at the noise floor). "
               "SMOOTH 8 can't follow ringing above ~20 kHz; use raw 500 kS/s for that."]),
           ("h2", "What a transient of a given length looks like"),
           ("table", [["true duration", "what the recording shows", "smallest height seen single-shot (5 sigma)<br/>raw / SMOOTH 8"],
                      ["1 ms and longer", f"correct height, correct length, edges rounded ({rise_meas*1e6:.0f} us rise)",
                       f"{th[1e-3][0]*1e3:.0f} mV / {th[1e-3][1]*1e3:.0f} mV"],
                      ["100 us", "correct height, length readable, edges clearly rounded",
                       f"{th[100e-6][0]*1e3:.0f} mV / {th[100e-6][1]*1e3:.0f} mV"],
                      ["10 us", f"a rounded bump about {fw10*1e6:.0f} us wide (half height) at ~{100*np.interp(np.log(10e-6), np.log(widths), ppv1):.0f} % of the true height",
                       f"{th[10e-6][0]*1e3:.0f} mV / {th[10e-6][1]*1e3:.0f} mV"],
                      ["1 us", f"a small bump about {fw_floor*1e6:.0f} us wide at ~{100*np.interp(np.log(1e-6), np.log(widths), ppv1):.0f} % of the true height; "
                               "only its area is real", f"{th[1e-6][0]:.2g} V / {th[1e-6][1]:.2g} V"]],
            [0.17, 0.53, 0.30]),
           ("sp", 6),
           ("s", "Smallest heights: 5 x the measured noise sigma at that setting, divided by the recorded fraction of the height "
                 "for that duration (from the response, which the captures confirm within a few %). 5 sigma rarely happens by "
                 "chance in a 10000-point record; a 3-sigma bump is visible when you know where to look. All heights are "
                 "input-referred (at the board input).")]
    sp += [("pb", None), ("h2", "How the test works and how to read the plots"),
           ("p", "The generator repeats each transient at a fixed interval (2 ms for most pulses) and the board takes an "
                 "ordinary free-running capture. Because the generator timing is exact, every sample can be placed on the "
                 "transient's own time axis: the capture is folded on the repeat interval and lined up with the expected "
                 "response by a matched filter that uses every event in the record. The single-event traces are the first "
                 "complete event in each record, not a chosen one."),
           ("ul", ["<b>Grey, top strip:</b> the true input, as the generator was set (edges 3 ns).",
                   "<b>Blue dots:</b> what the board recorded for one event, input-referred, baseline removed.",
                   "<b>Orange dashes:</b> what the measured frequency response predicts the board should record, including "
                   "SMOOTH averaging. It is a prediction, not a fit: where blue sits on orange, the front end behaves as measured.",
                   "<b>Grey band:</b> +/-3 sigma of the recording noise at that setting. A bump inside it can't be told from noise."]),
           ("img", fig_response(M, sig8)),
           ("p", f"The response fit uses three real poles, at {poles[0]/1e3:.1f} kHz, {poles[1]/1e3:.0f} kHz and "
                 f"{poles[2]/1e3:.0f} kHz, matched to the sine sweep in the bench run (10 Hz to 500 kHz). The first pole does "
                 f"almost everything; the other two only matter above ~150 kHz. On the pulse captures the recorded amplitude "
                 f"matches the prediction within {100*(g_lo-1):+.0f} to {100*(g_hi-1):+.0f} % (5th to 95th percentile of the "
                 f"{len(gains)} captures with a clear signal, median {100*(g_med-1):+.1f} %), so the response can be used to "
                 f"predict transients that weren't tested."),
           ("table", [["setting", "time per point", "noise (input-referred rms)", "notes"],
                      ["500 kS/s raw", "2 us", f"{sig1*1e3:.1f} mV", "finest time detail"],
                      ["500 kS/s SMOOTH 8", "16 us", f"{sig8*1e3:.1f} mV", "best all-round for weak transients"]] +
                     [[f"{A['rate']/1e3:g} kS/s" + (f" SMOOTH {A['N']}" if A['N'] > 1 else " raw"), f"{A['dt']*1e6:g} us",
                       f"{A['sigma']*1e3:.1f} mV", ""] for A in SA[2:]],
            [0.25, 0.17, 0.25, 0.33])]
    sp += [("pb", None), ("h2", "1. Pulse length, at 1 V, 500 kS/s raw"),
           ("p", "The same 1 V pulse from 1 ms down to 1 us. Long pulses come through with their height and length; below "
                 f"about 15 us the recorded bump stops getting narrower and just gets smaller. Note the different scales: the "
                 f"true strip is always 1 V tall."),
           ("img", fig_width_series(pick(G1, 1.0, wsel), "1 V", pres))]
    sp += [("pb", None), ("h2", "2. Weak pulses: 100 mV, 500 kS/s SMOOTH 8"),
           ("p", f"The same lengths at 100 mV with SMOOTH 8 (noise {sig8*1e3:.1f} mV). Down to 20 us the pulse is plain "
                 f"({G8[(20e-6, 0.1)]['snr']:.0f} sigma); 10 us is a bump at the edge of the noise ({G8[(10e-6, 0.1)]['snr']:.0f} "
                 "sigma); 5 us and shorter are lost in a single shot. SMOOTH 8 puts the points 16 us apart, so short events "
                 "become two or three points: fine for seeing that something happened and how big its area was, not for its shape."),
           ("img", fig_width_series(pick(G8, 0.1, wsel), "100 mV, SMOOTH 8", pres))]
    sp += [("pb", None), ("h2", "3. Short pulses are recorded as their area"),
           ("p", f"Left: recorded peak as a fraction of the true height (measured from the average event of each 1 V record, "
                 f"dots; prediction, dashes). Above ~50 us the full height is reached. Below ~15 us the peak falls in direct "
                 f"proportion to width (dotted line: height x width / {tau1*1e6:.1f} us), so a 1 us, 1 V pulse and a 10 us, "
                 f"100 mV pulse look the same. SMOOTH 8 lowers short peaks further (it averages over 16 us), but its noise is "
                 f"lower by more, so it still sees them better."),
           ("p", f"Right: the apparent duration (width at half height, raw, where the average event is clear). Nothing reads "
                 f"shorter than about {fw_floor*1e6:.0f} us; longer pulses read their true length plus a little. "
                 "So an event that looks 12-15 us wide may really have been anything from 10 us down to a spike."),
           ("img", fig_area(P1, P8, P1w, M)),
           ("img", fig_edge(G1[(1e-3, 1.0)], M)),
           ("p", f"A step rises 10-90 % in {rise_meas*1e6:.0f} us measured ({rise_mod*1e6:.0f} us predicted), so the "
                 "start time of an event can be placed to a few us, but its edges always look rounded.")]
    rows = []
    for a in sorted([a for a in amps if a <= 1.0], reverse=True):
        if a in (1.0, 0.1, 0.05, 0.02, 0.01, 0.005):
            rows.append((a, G1.get((100e-6, a)), G8[(100e-6, a)]))
    sp += [("pb", None), ("h2", "4. How weak can a transient be? (100 us pulse)"),
           ("p", f"A 100 us pulse is long enough to be recorded at full height, so this isolates the noise. Raw 500 kS/s "
                 f"(left, noise {sig1*1e3:.0f} mV rms) loses it below about 100 mV. SMOOTH 8 (middle, {sig8*1e3:.1f} mV) still "
                 "shows 50 mV, while 20 mV is at the edge of the noise. If the event repeats, averaging the events (right) shows "
                 "10 mV clearly and 5 mV just about. "
                 "Raw was only captured at 100 mV and 1 V."),
           ("img", fig_ladder(rows))]
    sp += [("pb", None), ("h2", "5. Visibility map: every length and height tested"),
           ("p", "Each cell is one capture at 500 kS/s SMOOTH 8: the recorded peak divided by the noise sigma, for a single "
                 "event. Read it as: pick your transient's length (column) and size (row)."),
           ("img", fig_map(G8, widths, amps, sig8)),
           ("p", "Below: the smallest height that reaches 5 sigma in one shot, for each length, with and without SMOOTH, and "
                 "with averaging of 16 repeats. The curves flatten above ~50 us (full height reached) and rise as 1/width below "
                 "~15 us (the area rule)."),
           ("img", fig_threshold(widths, ppv1, ppv8, sig1, sig8, nev))]
    sp += [("pb", None), ("h2", "6. Oscillating transients (3-cycle sine bursts, 2 V peak)"),
           ("p", "Ringing behaves the way the frequency response says. A 2 kHz burst is recorded intact; 10 kHz loses about a "
                 "quarter and lags by about 12 us; 30 kHz arrives at about a third of its size (the first cycle a little "
                 "larger); 100 kHz at about a tenth, still clearly visible raw; 300 kHz is a few % and sits at the noise. "
                 "With SMOOTH 8 (right) the points are 16 us apart: 30 kHz is undersampled and distorted, and 100 kHz "
                 "merges into one small bump. If the frequency of a ring matters, use raw 500 kS/s."),
           ("img", fig_bursts(BR))]
    sp += [("pb", None), ("h2", "7. One transient, six settings (20 us, 300 mV)"),
           ("p", "The board settings decide how much of what the front end passes is kept. At low sample rates without SMOOTH, the "
                 "points are further apart than the event: at 10 kS/s a point lands on the bump only by luck, and the peak height "
                 "is usually wrong. SMOOTH at 500 kS/s gives the same point spacing as a lower rate but averages every sample "
                 "in between, so nothing is stepped over."),
           ("img", fig_settings(SA)),
           ("table", [["for", "use", "why"],
                      ["weak events, any length", "500 kS/s, SMOOTH 8", f"every sample used, {ratio:.1f}x lower noise, 16 us/point"],
                      ["the very weakest long events (> 200 us)", "500 kS/s, SMOOTH 16-64", "more averaging, still faster than the event"],
                      ["the shape of fast edges or ringing", "500 kS/s raw", "2 us/point, keeps 30-100 kHz content"],
                      ["long records", "500 kS/s with SMOOTH raised", "not a lower rate: the record keeps 10000 points, so raise SMOOTH "
                                                                         "(e.g. SMOOTH 50 for 1 s) instead of letting it skip points"]],
            [0.30, 0.25, 0.45])]
    sp += [("pb", None), ("h2", "8. What this test does not cover"),
           ("ul", ["<b>A high-impedance source.</b> The generator drives the input from 50 ohms. With a high-impedance source, "
                   "the source resistance and the input capacitance form an extra low-pass, and the input picks up mains hum. "
                   "A 1 Mohm source was measured separately: see Series_1Mohm_source_report.pdf.",
                   "<b>Signals above +/-10 V.</b> The generator stops at +/-10 V; +/-12 V is not verified (the pins would sit at "
                   "0.61-2.33 V, inside the ADC range).",
                   "<b>Triggering.</b> These records are free-running and were lined up afterwards from the generator timing. The "
                   "board's own trigger fires on one pin's change between consecutive points, and each pin on its own carries "
                   "the ~632 kHz supply ripple and more noise than the A2-A3 difference. A weak transient that only shows after "
                   "differencing and SMOOTH may not fire it reliably. Not tested here.",
                   "<b>Battery power.</b> All of this was on USB. USB and battery were compared afterwards (2026-11-05) "
                   "and measured the same noise, ripple and hum, so these results hold on battery too.",
                   "<b>Polarity and DC level.</b> Only positive pulses from 0 V were used. The front end is linear across +/-10 V "
                   "(see the bench run), so negative or offset transients behave the same."]),
           ("h2", "Reproducing this report"),
           ("p", "py transient_capture.py --board COM32 --label usb &nbsp;&nbsp;(about 7 minutes, drives the Rigol, restores it after)"
                 "<br/>py transient_report.py runs/&lt;stamp&gt;_transients_usb runs/&lt;stamp&gt;_usb_bench"),
           ("s", f"Data: {os.path.basename(tdir)} and {os.path.basename(bdir)} in Giga_RemoteController_WiFi/frontend_test/runs. "
                 f"Clock offset between the Giga and the Rigol, measured from the data: {ppm:+.1f} ppm.")]
    build_pdf(out, sp)
    print("wrote", out)

if __name__ == "__main__":
    main()
