"""
Front-end characterisation for Giga_RemoteController_WiFi + the 12 V op-amp buffer board
(preset OpAmpBuffer_12vboard: input = 7.005 x (V_A2 - V_A3)).

It drives the Rigol DG852 Pro CH1 (SCPI on 192.168.10.3:5025) into the board input and
captures A2/A3 with the board's normal PINS: command, so the firmware is unchanged.

  Tests:  noise     Rigol at 0 V DC (low-impedance short), several sample rates; rms,
                    spectrum peaks, mains hum, the ~632 kHz isolated-supply ripple
          dc        DC transfer -10 V .. +10 V (Rigol limit): gain, offset, linearity,
                    pin headroom and the extrapolated pin voltages at +/-12 V
          sweep     small-signal (2 Vpp) frequency response, 10 Hz .. 10 MHz
          large     large-signal (19.9 Vpp) response up to 1 MHz (slew / clipping)
          thd       19.9 Vpp at 100 Hz and 1 kHz: THD and peak flattening

  Each run lands in runs/<date-time>_<label>/ (raw captures + results.json + summary.md).
  The Rigol CH1 settings are read first and restored at the end.

Usage
  py frontend_characterize.py --board COM32 --label usb
  py frontend_characterize.py --board 192.168.1.42 --label battery     (Wi-Fi, port 8080)
  py frontend_characterize.py --board COM32 --label quick --tests noise,dc
  py frontend_characterize.py --compare runs/<runA> runs/<runB>
"""
import argparse, datetime, json, math, os, socket, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCALE = 7.005            # preset diff_scale: input volts per ADC volt (A2 - A3)
VREF = 3.3
FGEN = ("192.168.10.3", 5025)

# ---------------------------------------------------------------- links
class Rigol:
    def __init__(self, host, port):
        self.k = socket.create_connection((host, port), timeout=5); self.f = self.k.makefile("rb")
    def w(self, c): self.k.sendall((c + "\n").encode())
    def q(self, c): self.w(c); return self.f.readline().decode().strip()
    def sync(self): return self.q("*OPC?")
    def sine(self, f, vpp, offs=0.0):
        """Set CH1 to a sine and confirm it from APPL? (the DG852 silently clamps the amplitude
        when the old offset + new amplitude would exceed +/-10 V, e.g. straight after DC +10 V)."""
        self.w(":SOUR1:APPL:DC 1,1,0"); self.sync()
        for _ in range(3):
            self.w(f":SOUR1:APPL:SIN {f:.3f},{vpp},{offs}"); self.sync()
            k, fr, a, o, _ = self.q(":SOUR1:APPL?").strip('"').split(",")
            if k == "SIN" and abs(float(a) - vpp) < 1e-3 * vpp + 1e-4 and abs(float(fr) - f) < 1e-6 * f + 1e-3:
                return
        raise RuntimeError(f"Rigol did not accept SIN {f} Hz {vpp} Vpp (reads {k},{fr},{a},{o})")
    def dc(self, v):
        self.w(f":SOUR1:APPL:DC 1,1,{v:.4f}"); self.sync()
        k, _, _, o, _ = self.q(":SOUR1:APPL?").strip('"').split(",")
        if k != "DC" or abs(float(o) - v) > 1e-3:
            raise RuntimeError(f"Rigol did not accept DC {v} V (reads {k}, {o})")


class Board:
    """One PINS: capture per call, over USB serial (COMx) or Wi-Fi TCP (host[:port])."""
    def __init__(self, addr):
        self.addr = addr
        self.tcp = not addr.upper().startswith("COM") and not addr.startswith("/dev/")
        if not self.tcp:
            import serial
            self.s = serial.Serial(addr, 115200, timeout=0.5); time.sleep(0.3); self.s.reset_input_buffer()

    def _lines(self, cmd, timeout):
        if self.tcp:
            host, _, port = self.addr.partition(":")
            k = socket.create_connection((host, int(port or 8080)), timeout=timeout)
            k.sendall(cmd.encode()); f = k.makefile("rb")
            try:
                for raw in f:
                    yield raw.decode(errors="ignore").strip()
            finally:
                k.close()        # the board handles one command per connection
        else:
            self.s.reset_input_buffer(); self.s.write(cmd.encode()); t = time.time()
            while time.time() - t < timeout:
                yield self.s.readline().decode(errors="ignore").strip()

    def capture(self, rate, ms, pins=("A2", "A3"), bits=12, retries=3, smooth=1):
        n = len(pins)
        cmd = (f"PINS:{','.join(pins)}|BITS:{bits}|TIME:{ms}|SMOOTH:{smooth}|LOG:10000|RATE:{rate}"
               f"|TRIG:{','.join(['0'] * n)}|MID:0\n")
        res = {"cmd": cmd.strip(), "pins": {}, "data": {}, "info": {}}
        for l in self._lines(cmd, 30 + ms / 1000):
            if l.startswith("PIN:"):
                p = dict(kv.split(":", 1) for kv in l.split("|")); res["pins"][p["PIN"]] = p
            elif l.startswith("DATA:"):
                a, b = l.split("|VALS:"); res["data"][a.split(":")[1]] = [int(v) for v in b.split(",") if v]
            elif l.startswith("INFO:RATE"):
                res["info"] = dict(kv.split(":", 1) for kv in l.split("|")[1:])
            elif l.startswith("ERR"):
                raise RuntimeError(l)
            elif l.startswith("DONE"):
                res["elapsed_ms"] = int(l.split("ELAPSED:")[1]); break
        if not res["data"]:
            raise RuntimeError("no data from board")
        # Integrity check: every pin present, the count the board says it logged, every value in range.
        # (Old firmware dropped bytes over Wi-Fi: glued samples, lost samples, a mangled DATA header.)
        bad = [p for p in pins if p not in res["data"] or p not in res["pins"]
               or len(res["data"][p]) != int(res["pins"][p].get("LOGGED", -1))
               or any(v < 0 or v >= (1 << bits) for v in res["data"][p])]
        if bad:
            if retries > 0:
                print(f"    (capture data damaged on {','.join(bad)}, retrying)")
                return self.capture(rate, ms, pins, bits, retries - 1, smooth)
            raise RuntimeError(f"capture data damaged on {bad}")
        if res["info"].get("DROP", "0") != "0" and retries > 0:
            print(f"    (capture dropped {res['info']['DROP']} buffer(s), retrying)")
            return self.capture(rate, ms, pins, bits, retries - 1, smooth)
        res["fs"] = float(res["info"].get("USED", rate))
        self.captures = getattr(self, "captures", 0) + 1; self.last_info = res["info"]
        return res

# ---------------------------------------------------------------- maths
C2V = VREF / 4095.0

def arrays(cap):
    a = np.array(cap["data"]["A2"], float); b = np.array(cap["data"]["A3"], float)
    m = min(len(a), len(b)); return a[:m], b[:m]

def alias(f, fs):
    return abs(f - round(f / fs) * fs)

def nudge(f, fs, n):
    """Move f slightly so its alias is >= 12 bins from DC and from Nyquist (a fixed-phase
    alias would make the amplitude read wrong)."""
    guard = 12 * fs / n; f0 = f
    if f < fs / 2 - guard:
        return f                    # no aliasing
    for k in range(400):
        fa = alias(f, fs)
        if guard < fa < fs / 2 - guard:
            return f
        f = f0 * (1 + 0.0021 * (k + 1))
    return f

def fit(x, fs, f_expect, ppm=300):
    """Amplitude/phase of the tone that f_expect aliases to. Returns (amp, phase, f_alias)."""
    n = len(x); t = np.arange(n) / fs; xm = x - x.mean()
    fa0 = alias(f_expect, fs)
    W = np.abs(np.fft.rfft(xm * np.hanning(n))); fr = np.fft.rfftfreq(n, 1 / fs)
    half = 3 * fs / n + f_expect * ppm * 1e-6
    m = (fr > fa0 - half) & (fr < fa0 + half) & (fr > fs / n)
    if not m.any():
        m = fr > fs / n
    i = np.flatnonzero(m)[np.argmax(W[m])]
    # parabolic refine on the log magnitude
    if 0 < i < len(W) - 1:
        l, c, r = np.log(W[i - 1:i + 2] + 1e-12); d = 0.5 * (l - r) / (l - 2 * c + r)
    else:
        d = 0
    fa = (i + d) * fs / n
    best = None
    for df in np.linspace(-0.3, 0.3, 13) * fs / n:          # small LS search around the peak
        w = 2 * np.pi * (fa + df)
        A = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones(n)])
        coef, res, *_ = np.linalg.lstsq(A, xm, rcond=None)
        r2 = float(((xm - A @ coef) ** 2).sum())
        if best is None or r2 < best[0]:
            best = (r2, coef, fa + df)
    _, (s, c, _), fa = best
    return float(math.hypot(s, c)), float(math.atan2(c, s)), float(fa)

def spectrum_peaks(x, fs, k=6, scale=1.0):
    n = len(x); w = np.hanning(n); X = np.abs(np.fft.rfft((x - x.mean()) * w)) * 2 / w.sum() * scale
    fr = np.fft.rfftfreq(n, 1 / fs); X[:3] = 0
    out = []; Y = X.copy()
    for _ in range(k):
        i = int(np.argmax(Y)); out.append((float(fr[i]), float(X[i])))
        Y[max(0, i - 4):i + 5] = 0
    return out, fr, X

def tone_at(fr, X, f0, tol):
    m = (fr > f0 - tol) & (fr < f0 + tol)
    return float(X[m].max()) if m.any() else 0.0

# ---------------------------------------------------------------- tests
def t_noise(board, fg, raw):
    fg.dc(0.0); time.sleep(0.5)
    out = []
    for fs, ms in ((500000, 25), (250000, 45), (100000, 120), (10000, 1100)):
        cap = board.capture(fs, ms); raw[f"noise_{fs}"] = cap
        a, b = arrays(cap); fs = cap["fs"]; d = a - b
        r = {"fs": fs, "n": len(d),
             "A2_mean": a.mean(), "A3_mean": b.mean(), "diff_mean_counts": d.mean(),
             "offset_mV_in": d.mean() * C2V * SCALE * 1e3,
             "A2_sd": a.std(), "A3_sd": b.std(), "diff_sd_counts": d.std(),
             "diff_sd_mV_in": d.std() * C2V * SCALE * 1e3,
             "diff_pp_mV_in": (d.max() - d.min()) * C2V * SCALE * 1e3,
             "corr": float(np.corrcoef(a, b)[0, 1]),
             "drops": cap["info"].get("DROP"), "resyncs": cap["info"].get("RESYNC")}
        pk, fr, X = spectrum_peaks(d, fs, 6, C2V * SCALE * 1e3)
        r["peaks_mV_in"] = pk
        if fs <= 20000:   # long capture: mains hum
            tol = 3 * fs / len(d)
            r["hum60_mV_in"] = tone_at(fr, X, 60, tol); r["hum120_mV_in"] = tone_at(fr, X, 120, tol)
            r["hum180_mV_in"] = tone_at(fr, X, 180, tol)
        else:             # the isolated DC-DC ripple near 632 kHz, aliased: per pin + diff
            fa = alias(632e3, fs); tol = 25e3
            for nm, v in (("A2", a), ("A3", b), ("diff", d)):
                _, fr2, X2 = spectrum_peaks(v, fs, 1, C2V * SCALE * 1e3)
                r[f"ripple_{nm}_mV_in"] = tone_at(fr2, X2, fa, tol)
            for N in (4, 16, 64):
                m = len(d) // N * N; r[f"smooth{N}_sd_mV_in"] = d[:m].reshape(-1, N).mean(1).std() * C2V * SCALE * 1e3
        out.append(r)
        print(f"  noise {fs/1e3:6.0f} kS/s: diff {r['diff_sd_counts']:5.2f} counts = {r['diff_sd_mV_in']:6.2f} mV rms in, "
              f"corr {r['corr']:+.2f}, offset {r['offset_mV_in']:+.1f} mV, drops {r['drops']}")
    return out

def t_dc(board, fg, raw):
    out = []
    for v in np.arange(-10, 10.01, 1.0):
        v = float(np.clip(v, -9.999, 9.999))
        fg.dc(v); time.sleep(0.3)
        cap = board.capture(10000, 200); raw[f"dc_{v:+.1f}"] = cap
        a, b = arrays(cap); d = a - b
        out.append({"vin": v, "A2_V": a.mean() * C2V, "A3_V": b.mean() * C2V,
                    "A2_min": a.min(), "A2_max": a.max(), "A3_min": b.min(), "A3_max": b.max(),
                    "diff_V": d.mean() * C2V, "vin_meas": d.mean() * C2V * SCALE, "sd_counts": d.std()})
        print(f"  dc {v:+6.2f} V -> {out[-1]['vin_meas']:+8.4f} V   (A2 {out[-1]['A2_V']:.3f} V, A3 {out[-1]['A3_V']:.3f} V)")
    vin = np.array([r["vin"] for r in out]); dv = np.array([r["diff_V"] for r in out])
    g, o = np.polyfit(vin, dv, 1); res = dv - (g * vin + o)
    pa = np.polyfit(vin, [r["A2_V"] for r in out], 1); pb = np.polyfit(vin, [r["A3_V"] for r in out], 1)
    summ = {"points": out,
            "gain_V_per_V": g, "gain_vs_preset_pct": (g * SCALE - 1) * 100,
            "offset_mV_in": o / g * 1e3,
            "max_nonlin_mV_in": float(np.abs(res).max() / g * 1e3),
            "max_nonlin_pct_fs": float(np.abs(res).max() / g / 10 * 100),
            "A2_fit": list(pa), "A3_fit": list(pb),
            "pins_at_pm12V": {"A2": [np.polyval(pa, -12), np.polyval(pa, 12)],
                               "A3": [np.polyval(pb, -12), np.polyval(pb, 12)]},
            "adc_limited_range_V": float(min(abs((0 - pa[1]) / pa[0]), abs((VREF - pa[1]) / pa[0]),
                                            abs((0 - pb[1]) / pb[0]), abs((VREF - pb[1]) / pb[0])))}
    return summ

FREQS_SMALL = [10, 20, 50, 100, 200, 500, 1e3, 2e3, 5e3, 10e3, 20e3, 30e3, 50e3, 70e3, 100e3, 150e3, 200e3,
               300e3, 500e3, 700e3, 1e6, 1.5e6, 2e6, 3e6, 5e6, 10e6]
FREQS_LARGE = [100, 1e3, 10e3, 20e3, 50e3, 100e3, 200e3, 500e3, 1e6]

def plan(f):
    if f < 200:  return 10000, 1100
    if f < 2000: return 100000, 120
    return 500000, 25

def t_sweep(board, fg, raw, vpp, freqs, tag):
    out = []
    for f in freqs:
        fs, ms = plan(f); f = nudge(f, fs, 6000)
        fg.sine(f, vpp); time.sleep(0.25)
        cap = board.capture(fs, ms); raw[f"{tag}_{f:.0f}"] = cap
        a, b = arrays(cap); fs = cap["fs"]; d = a - b
        ad, pd, fa = fit(d, fs, f); aa, pa, _ = fit(a, fs, f); ab, pb, _ = fit(b, fs, f)
        vin_pk = ad * C2V * SCALE
        r = {"f": f, "fs": fs, "alias": fa, "vpp_set": vpp, "vin_pk": vin_pk,
             "gain_db": 20 * math.log10(max(vin_pk, 1e-9) / (vpp / 2)),
             "A2_pk_V": aa * C2V, "A3_pk_V": ab * C2V,
             "leg_match_db": 20 * math.log10(max(aa, 1e-9) / max(ab, 1e-9)),
             "leg_phase_deg": float((math.degrees(pa - pb) + 360) % 360),
             "A2_min": a.min(), "A2_max": a.max(), "A3_min": b.min(), "A3_max": b.max()}
        out.append(r)
        print(f"  {tag} {f/1e3:10.3f} kHz: {r['gain_db']:+7.2f} dB   legs {r['leg_match_db']:+.2f} dB "
              f"{r['leg_phase_deg']:6.1f} deg   A2 {r['A2_min']:.0f}..{r['A2_max']:.0f}")
    ref = min(out, key=lambda r: abs(math.log10(r["f"] / 1e3)))["gain_db"]
    for r in out: r["rel_db"] = r["gain_db"] - ref
    return {"points": out, "ref_db_at_1k": ref, "f_3db": cross(out, -3), "f_1db": cross(out, -1),
            "f_0p1db": cross(out, -0.1)}

def cross(pts, lvl):
    pts = sorted(pts, key=lambda r: r["f"])
    hi = [r for r in pts if r["f"] >= 1e3]
    for p, q in zip(hi, hi[1:]):
        if p["rel_db"] > lvl >= q["rel_db"]:
            x = math.log10(p["f"]) + (lvl - p["rel_db"]) / (q["rel_db"] - p["rel_db"]) * (math.log10(q["f"]) - math.log10(p["f"]))
            return 10 ** x
    return None

def t_thd(board, fg, raw):
    out = []
    # >= 100 cycles per capture, and the fundamental + harmonics 2..7 + DC are fitted jointly:
    # fitting each harmonic on its own lets the fundamental leak into it over a short window.
    for f, fs, ms in ((100, 10000, 1100), (1000, 100000, 120)):
        f = nudge(f, fs, 6000)
        fg.sine(f, 19.9); time.sleep(0.3)
        cap = board.capture(fs, ms); raw[f"thd_{f:.0f}"] = cap
        a, b = arrays(cap); fs = cap["fs"]; d = a - b
        _, _, f1 = fit(d, fs, f)
        n = len(d); t = np.arange(n) / fs
        ks = [k for k in range(1, 8) if k * f1 < fs / 2]
        cols = [np.ones(n)]
        for k in ks:
            cols += [np.sin(2 * np.pi * k * f1 * t), np.cos(2 * np.pi * k * f1 * t)]
        A = np.column_stack(cols); coef = np.linalg.lstsq(A, d, rcond=None)[0]
        amps = [math.hypot(coef[1 + 2 * i], coef[2 + 2 * i]) for i in range(len(ks))]
        a1, hs = amps[0], amps[1:]
        fund = A[:, :3] @ coef[:3]                      # DC + fundamental only
        resid = d - A @ coef
        pk = np.abs(fund - coef[0]) > 0.97 * a1
        flat = float(np.mean(np.sign(fund[pk] - coef[0]) * (fund[pk] - d[pk])))   # >0: peaks squashed
        r = {"f": f, "fs": fs, "vin_pk": a1 * C2V * SCALE, "harm_mV_in": [h * C2V * SCALE * 1e3 for h in hs],
             "thd_pct": 100 * math.sqrt(sum(h * h for h in hs)) / a1,
             "resid_rms_mV_in": resid.std() * C2V * SCALE * 1e3,
             "peak_flattening_mV_in": flat * C2V * SCALE * 1e3,
             "A2_range": [a.min() * C2V, a.max() * C2V], "A3_range": [b.min() * C2V, b.max() * C2V]}
        out.append(r)
        print(f"  thd {f:7.1f} Hz 19.9 Vpp: {r['vin_pk']:.3f} V pk, THD {r['thd_pct']:.3f} %, harmonics mV "
              f"{[round(h, 1) for h in r['harm_mV_in']]}, peak flattening {r['peak_flattening_mV_in']:+.1f} mV")
    return out

# ---------------------------------------------------------------- report
def summary_md(R):
    L = [f"# Front-end run: {R['label']}", "",
         f"- date: {R['date']}  board: {R['board']}  tests: {', '.join(R['tests'])}",
         f"- conditions: {R.get('note') or '(none given)'}",
         f"- scale: input = {SCALE} x (V_A2 - V_A3), 12-bit, VREF {VREF} V",
         f"- board health after {R.get('board_health', {}).get('captures')} captures: "
         f"{ {k: v for k, v in R.get('board_health', {}).get('last_info', {}).items() if k in ('HEAPMAX', 'ADCFIX', 'LASTFIX')} }", ""]
    if "noise" in R:
        L += ["## Noise (Rigol at 0 V DC)", "",
              "| rate | diff rms counts | rms mV in | p-p mV in | corr A2/A3 | offset mV | ripple diff mV | ripple A2 mV | smooth16 mV | 60 Hz mV | drops |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in R["noise"]:
            L.append(f"| {r['fs']/1e3:.0f}k | {r['diff_sd_counts']:.2f} | {r['diff_sd_mV_in']:.1f} | {r['diff_pp_mV_in']:.0f} | "
                     f"{r['corr']:+.2f} | {r['offset_mV_in']:+.1f} | {r.get('ripple_diff_mV_in', float('nan')):.2f} | "
                     f"{r.get('ripple_A2_mV_in', float('nan')):.2f} | {r.get('smooth16_sd_mV_in', float('nan')):.2f} | "
                     f"{r.get('hum60_mV_in', float('nan')):.2f} | {r['drops']} |")
        L.append("")
    if "dc" in R:
        d = R["dc"]
        L += ["## DC transfer (-10 .. +10 V)", "",
              f"- gain {d['gain_V_per_V']*1e3:.4f} mV/V at the ADC, {d['gain_vs_preset_pct']:+.3f} % vs the preset 7.005",
              f"- offset {d['offset_mV_in']:+.1f} mV input-referred, worst non-linearity {d['max_nonlin_mV_in']:.1f} mV ({d['max_nonlin_pct_fs']:.3f} % of 10 V)",
              f"- pins at +/-12 V (extrapolated): A2 {d['pins_at_pm12V']['A2'][0]:.3f} / {d['pins_at_pm12V']['A2'][1]:.3f} V, "
              f"A3 {d['pins_at_pm12V']['A3'][0]:.3f} / {d['pins_at_pm12V']['A3'][1]:.3f} V",
              f"- the ADC itself would clip at about +/-{d['adc_limited_range_V']:.1f} V input", "",
              "| Vin set | Vin measured | error mV | A2 V | A3 V |", "|---|---|---|---|---|"]
        for p in d["points"]:
            L.append(f"| {p['vin']:+.1f} | {p['vin_meas']:+.4f} | {(p['vin_meas']-p['vin'])*1e3:+.1f} | {p['A2_V']:.4f} | {p['A3_V']:.4f} |")
        L.append("")
    for key, name in (("sweep", "Small-signal response (2 Vpp)"), ("large", "Large-signal response (19.9 Vpp)")):
        if key not in R: continue
        s = R[key]; fmt = lambda v: "n/a (above the sweep)" if v is None else (f"{v/1e3:.1f} kHz" if v < 1e6 else f"{v/1e6:.2f} MHz")
        L += [f"## {name}", "", f"- gain at 1 kHz {s['ref_db_at_1k']:+.3f} dB vs the preset scale",
              f"- -0.1 dB {fmt(s['f_0p1db'])}, -1 dB {fmt(s['f_1db'])}, -3 dB {fmt(s['f_3db'])}",
              "- the 19.9 Vpp sweep has few points, so its -0.1/-1 dB figures are coarse; compare -3 dB",
              "- a point within ~5 dB of the floor (about 0.2 mV at each pin, i.e. -55 dB at 2 Vpp, -75 dB at 19.9 Vpp)"
              " is noise, not response: there the leg match / phase columns are meaningless", "",
              "| f | rel dB | Vin pk | A2/A3 leg match dB | leg phase deg |", "|---|---|---|---|---|"]
        for p in s["points"]:
            L.append(f"| {p['f']:.0f} | {p['rel_db']:+.2f} | {p['vin_pk']:.4f} | {p['leg_match_db']:+.2f} | {p['leg_phase_deg']:.1f} |")
        L.append("")
    if "thd" in R:
        L += ["## Large-signal distortion (19.9 Vpp)", "", "| f | Vin pk | THD % | harmonics 2..7 mV | resid rms mV | peak flattening mV |", "|---|---|---|---|---|---|"]
        for r in R["thd"]:
            L.append(f"| {r['f']:.0f} | {r['vin_pk']:.3f} | {r['thd_pct']:.3f} | {', '.join(f'{h:.1f}' for h in r['harm_mV_in'])} | "
                     f"{r['resid_rms_mV_in']:.1f} | {r['peak_flattening_mV_in']:+.1f} |")
    return "\n".join(L) + "\n"

def compare(a_dir, b_dir):
    A = json.load(open(os.path.join(a_dir, "results.json"))); B = json.load(open(os.path.join(b_dir, "results.json")))
    print(f"{'metric':44s} {A['label']:>14s} {B['label']:>14s}")
    def row(name, va, vb, f="{:.3f}"):
        s = lambda v: "-" if v is None else f.format(v)
        print(f"{name:44s} {s(va):>14s} {s(vb):>14s}")
    na = {r["fs"]: r for r in A.get("noise", [])}; nb = {r["fs"]: r for r in B.get("noise", [])}
    for fs in sorted(set(na) | set(nb), reverse=True):
        ra, rb = na.get(fs, {}), nb.get(fs, {})
        row(f"noise {fs/1e3:.0f}k diff rms mV in", ra.get("diff_sd_mV_in"), rb.get("diff_sd_mV_in"), "{:.2f}")
        row(f"noise {fs/1e3:.0f}k corr A2/A3", ra.get("corr"), rb.get("corr"), "{:+.2f}")
        row(f"noise {fs/1e3:.0f}k offset mV", ra.get("offset_mV_in"), rb.get("offset_mV_in"), "{:+.1f}")
        for k in ("ripple_diff_mV_in", "ripple_A2_mV_in", "smooth16_sd_mV_in", "hum60_mV_in", "hum120_mV_in"):
            if k in ra or k in rb: row(f"noise {fs/1e3:.0f}k {k}", ra.get(k), rb.get(k), "{:.2f}")
    if "dc" in A or "dc" in B:
        da, db = A.get("dc", {}), B.get("dc", {})
        row("dc gain vs preset %", da.get("gain_vs_preset_pct"), db.get("gain_vs_preset_pct"))
        row("dc offset mV in", da.get("offset_mV_in"), db.get("offset_mV_in"), "{:+.1f}")
        row("dc worst non-linearity mV", da.get("max_nonlin_mV_in"), db.get("max_nonlin_mV_in"), "{:.1f}")
    for key in ("sweep", "large"):
        sa, sb = A.get(key, {}), B.get(key, {})
        for k in ("f_0p1db", "f_1db", "f_3db"):
            if k in sa or k in sb: row(f"{key} {k} kHz", (sa.get(k) or 0) / 1e3 or None, (sb.get(k) or 0) / 1e3 or None, "{:.1f}")
    for ra, rb in zip(A.get("thd", []), B.get("thd", [])):
        row(f"thd {ra['f']:.0f} Hz %", ra["thd_pct"], rb["thd_pct"])

# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", help="COMx or board IP[:port]")
    ap.add_argument("--label", default="run")
    ap.add_argument("--note", default="", help="free text: power source, wiring, ...")
    ap.add_argument("--tests", default="noise,dc,sweep,large,thd")
    ap.add_argument("--fgen", default=f"{FGEN[0]}:{FGEN[1]}")
    ap.add_argument("--compare", nargs=2, metavar="RUN_DIR")
    a = ap.parse_args()
    if a.compare:
        return compare(*a.compare)
    if not a.board:
        ap.error("--board is required")
    tests = [t.strip() for t in a.tests.split(",") if t.strip()]
    h, _, p = a.fgen.partition(":"); fg = Rigol(h, int(p or 5025))
    saved = {"appl": fg.q(":SOUR1:APPL?").strip('"'), "out": fg.q(":OUTP1?")}
    print("Rigol CH1 was:", saved)
    board = Board(a.board)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M")
    out_dir = os.path.join(HERE, "runs", f"{stamp}_{a.label}"); os.makedirs(out_dir, exist_ok=True)
    R = {"label": a.label, "note": a.note, "date": datetime.datetime.now().isoformat(timespec="seconds"),
         "board": a.board, "tests": tests, "rigol_before": saved}
    raw = {}
    try:
        fg.w(":OUTP1 ON")
        if "noise" in tests: print("noise"); R["noise"] = t_noise(board, fg, raw)
        if "dc" in tests:    print("dc transfer"); R["dc"] = t_dc(board, fg, raw)
        if "sweep" in tests: print("small-signal sweep"); R["sweep"] = t_sweep(board, fg, raw, 2.0, FREQS_SMALL, "ss")
        if "large" in tests: print("large-signal sweep"); R["large"] = t_sweep(board, fg, raw, 19.9, FREQS_LARGE, "ls")
        if "thd" in tests:   print("distortion"); R["thd"] = t_thd(board, fg, raw)
    finally:
        R["board_health"] = {"captures": getattr(board, "captures", 0), "last_info": getattr(board, "last_info", {})}
        k, *v = saved["appl"].split(",")
        if k == "DC":
            fg.w(f":SOUR1:APPL:DC 1,1,{float(v[2])}")
        else:
            fg.w(f":SOUR1:APPL:{k[:3]} {float(v[0])},{float(v[1])},{float(v[2])},{float(v[3])}")
        fg.w(f":OUTP1 {'ON' if saved['out'] == '1' else 'OFF'}"); fg.sync()
        print("Rigol CH1 restored:", fg.q(":SOUR1:APPL?"))
        conv = lambda o: o.item() if isinstance(o, np.generic) else o
        json.dump(raw, open(os.path.join(out_dir, "raw_captures.json"), "w"))
        json.dump(R, open(os.path.join(out_dir, "results.json"), "w"), indent=1, default=conv)
        open(os.path.join(out_dir, "summary.md"), "w").write(summary_md(json.loads(json.dumps(R, default=conv))))
        print("saved", out_dir)

if __name__ == "__main__":
    main()
