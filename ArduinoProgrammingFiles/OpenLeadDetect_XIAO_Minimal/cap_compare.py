"""
Compare differentiation between resistance conditions for two bridge-cap values:
  - 100nF  (folder: "Data LM6040 100nF Cap normal bridge")
  - 2200nF (folder: "Data LM6040 2200nF Cap normal bridge")

The signal of interest is diffV vs t_ms_from_toggle. A toggle at t=0 drives a
sharp dip, then the bridge recovers along an RC-shaped curve whose rate depends
on the DUT resistance. "Differentiation between conditions" = how separable the
resistance curves are. We quantify it and check how the cap changes it.
"""
import os
import re
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = os.path.dirname(os.path.abspath(__file__))
FOLDERS = {
    "100nF": "Data LM6040 100nF Cap normal bridge",
    "2200nF": "Data LM6040 2200nF Cap normal bridge",
}

def res_label(fname):
    """Extract a resistance label like '10KOhm' from a filename."""
    m = re.search(r"([\d.]+[KM]?)Ohm", fname)
    return m.group(1) + "Ohm" if m else None

def res_value(label):
    """Convert label like '4.7MOhm' to ohms (float)."""
    m = re.match(r"([\d.]+)([KM]?)Ohm", label)
    num = float(m.group(1))
    mult = {"": 1, "K": 1e3, "M": 1e6}[m.group(2)]
    return num * mult

def load_folder(folder):
    data = {}
    for f in os.listdir(folder):
        if not f.endswith(".csv"):
            continue
        if "DUTFacing" in f:   # ignore per instructions
            continue
        label = res_label(f)
        if label is None:
            continue
        df = pd.read_csv(os.path.join(folder, f))
        data[label] = df
    return data

datasets = {cap: load_folder(os.path.join(BASE, folder)) for cap, folder in FOLDERS.items()}

# Resistances present in BOTH caps -> fair comparison set
common = sorted(
    set(datasets["100nF"]) & set(datasets["2200nF"]),
    key=res_value,
)
print("Common resistances:", common)

# ---- Resample each curve onto a common time grid (post-toggle) ----
TGRID = np.linspace(0.0, 4.4, 220)   # ms after toggle

def curve_on_grid(df):
    t = df["t_ms_from_toggle"].values
    v = df["diffV"].values
    order = np.argsort(t)
    t, v = t[order], v[order]
    return np.interp(TGRID, t, v)

colors = plt.cm.viridis(np.linspace(0, 0.95, len(common)))

# ===== FIGURE 1: overlaid recovery curves, one subplot per cap =====
fig, axes = plt.subplots(1, 2, figsize=(15, 6), sharex=True, sharey=True)
for ax, cap in zip(axes, FOLDERS):
    for c, label in zip(colors, common):
        ax.plot(TGRID, curve_on_grid(datasets[cap][label]), color=c, lw=1.6, label=label)
    ax.set_title(f"{cap} bridge cap")
    ax.set_xlabel("t (ms) from toggle")
    ax.axhline(0, color="gray", lw=0.6)
    ax.grid(alpha=0.3)
axes[0].set_ylabel("diffV (V)")
axes[0].legend(title="DUT R", fontsize=8)
fig.suptitle("Recovery curves by DUT resistance — 100nF vs 2200nF bridge cap")
fig.tight_layout()
fig.savefig(os.path.join(BASE, "cap_compare_curves.png"), dpi=130)

# Recovery window: skip the common dip (t<0.1) and the settled noise tail.
# This is where resistance actually changes the curve shape.
REC = (TGRID >= 0.10) & (TGRID <= 1.5)

# Noise floor: jitter of diffV after it has fully settled (t>2.5ms).
def noise_floor(cap):
    vals = []
    for label in common:
        df = datasets[cap][label]
        tail = df[df["t_ms_from_toggle"] > 2.5]["diffV"].values
        vals.append(np.std(tail))
    return np.mean(vals)

nf = {cap: noise_floor(cap) for cap in FOLDERS}
print(f"\nNoise floor (std of settled diffV, t>2.5ms): "
      f"100nF={nf['100nF']:.4f} V, 2200nF={nf['2200nF']:.4f} V")

# ===== Differentiation metric (recovery window only) =====
def gap_matrix(cap):
    G = np.vstack([curve_on_grid(datasets[cap][l])[REC] for l in common])
    n = len(common)
    M = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            M[i, j] = np.sqrt(np.mean((G[i] - G[j]) ** 2))
    return G, M

results = {}
for cap in FOLDERS:
    G, M = gap_matrix(cap)
    results[cap] = (G, M)

# Adjacent-pair separation (neighbouring decades), the practical question:
# "can I tell 10K from 100K from 1M from 10M?"
print("\nAdjacent-resistance RMS separation of diffV over recovery 0.1-1.5ms (V):")
print(f"{'pair':>16} | {'100nF':>9} | {'2200nF':>9} | {'2200/100':>8}")
adj_rows = []
for i in range(len(common) - 1):
    pair = f"{common[i]}->{common[i+1]}"
    a = results["100nF"][1][i, i + 1]
    b = results["2200nF"][1][i, i + 1]
    ratio = b / a if a else float("nan")
    adj_rows.append((pair, a, b, ratio))
    print(f"{pair:>16} | {a:9.4f} | {b:9.4f} | {ratio:8.2f}")

mean100 = np.mean([r[1] for r in adj_rows])
mean2200 = np.mean([r[2] for r in adj_rows])
print(f"\nMean adjacent separation: 100nF={mean100:.4f} V, 2200nF={mean2200:.4f} V, "
      f"ratio={mean2200/mean100:.2f}x")
print(f"Separation-to-noise ratio: 100nF={mean100/nf['100nF']:.2f}x, "
      f"2200nF={mean2200/nf['2200nF']:.2f}x  (>~2 means reliably distinguishable)")

# ===== FIGURE 3: zoomed recovery window =====
fig3, axes3 = plt.subplots(1, 2, figsize=(15, 6), sharex=True, sharey=True)
for ax, cap in zip(axes3, FOLDERS):
    for c, label in zip(colors, common):
        ax.plot(TGRID, curve_on_grid(datasets[cap][label]), color=c, lw=1.8, label=label)
    ax.axvspan(0.10, 1.5, color="orange", alpha=0.08)
    ax.set_title(f"{cap} bridge cap")
    ax.set_xlabel("t (ms) from toggle")
    ax.axhline(0, color="gray", lw=0.6)
    ax.set_xlim(0, 2.0)
    ax.set_ylim(-0.30, 0.06)
    ax.grid(alpha=0.3)
axes3[0].set_ylabel("diffV (V)")
axes3[0].legend(title="DUT R", fontsize=8)
fig3.suptitle("Recovery window (zoom) — shaded = differentiation region 0.1-1.5ms")
fig3.tight_layout()
fig3.savefig(os.path.join(BASE, "cap_compare_zoom.png"), dpi=130)

# Also report the initial dip depth and a simple recovery time (time to cross
# halfway back to ~0) per condition, which is what the cap directly stretches.
print("\nPer-condition dip depth and time-to-half-recovery:")
print(f"{'R':>10} | {'cap':>7} | {'min diffV':>9} | {'t@min(ms)':>9} | {'t_half(ms)':>10}")
for label in common:
    for cap in FOLDERS:
        df = datasets[cap][label]
        post = df[df["t_ms_from_toggle"] >= 0]
        v = post["diffV"].values
        t = post["t_ms_from_toggle"].values
        imin = np.argmin(v)
        vmin = v[imin]
        tmin = t[imin]
        half = vmin / 2.0
        t_half = np.nan
        for k in range(imin, len(v)):
            if v[k] >= half:
                t_half = t[k]
                break
        print(f"{label:>10} | {cap:>7} | {vmin:9.4f} | {tmin:9.3f} | {t_half:10.3f}")

# ===== FIGURE 2: differentiation bar chart =====
fig2, ax2 = plt.subplots(figsize=(10, 5))
x = np.arange(len(adj_rows))
w = 0.38
ax2.bar(x - w/2, [r[1] for r in adj_rows], w, label="100nF", color="#1f77b4")
ax2.bar(x + w/2, [r[2] for r in adj_rows], w, label="2200nF", color="#d62728")
ax2.set_xticks(x)
ax2.set_xticklabels([r[0] for r in adj_rows], rotation=20, ha="right", fontsize=8)
ax2.set_ylabel("RMS diffV separation (V)")
ax2.set_title("Differentiation between adjacent resistances (higher = easier to tell apart)")
ax2.grid(axis="y", alpha=0.3)
ax2.legend()
fig2.tight_layout()
fig2.savefig(os.path.join(BASE, "cap_compare_separation.png"), dpi=130)

print("\nSaved: cap_compare_curves.png, cap_compare_zoom.png, cap_compare_separation.png")
