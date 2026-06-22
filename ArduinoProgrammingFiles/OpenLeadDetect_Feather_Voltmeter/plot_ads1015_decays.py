#!/usr/bin/env python3
"""Overlay all ADS1015 capture decays (DataCollection_ADS1015_*.csv) on one plot.

Plots diffV vs t_ms_from_toggle for every root-folder DataCollection_ADS1015_*.csv,
legend ordered by resistance. Run from the project folder:
    python plot_ads1015_decays.py
"""
import csv
import glob
import os
import re

import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))


def ohms(name):
    """Resistance in ohms parsed from a filename like ..._5600KOhm.csv (for sorting)."""
    m = re.search(r"_(\d+)(K|M)?Ohm", name, re.IGNORECASE)
    if not m:
        return float("inf")
    val = float(m.group(1))
    unit = (m.group(2) or "").upper()
    return val * {"K": 1e3, "M": 1e6, "": 1.0}[unit]


def label(name):
    m = re.search(r"_(\d+(?:K|M)?Ohm)", name, re.IGNORECASE)
    return m.group(1) if m else os.path.basename(name)


def load(path):
    t, v = [], []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            try:
                t.append(float(row["t_ms_from_toggle"]))
                v.append(float(row["diffV"]))
            except (KeyError, ValueError):
                continue
    return t, v


def main():
    files = glob.glob(os.path.join(HERE, "DataCollection_ADS1015_*.csv"))
    files.sort(key=ohms)
    if not files:
        print("No DataCollection_ADS1015_*.csv files found in", HERE)
        return

    fig, ax = plt.subplots(figsize=(10, 6))
    for path in files:
        t, v = load(path)
        if t:
            ax.plot(t, v, ".-", ms=3, lw=1.0, label=label(path))

    ax.set_title("ADS1015 capture decays vs. bridge resistance")
    ax.set_xlabel("time from toggle (ms)")
    ax.set_ylabel("differential voltage (V)")
    ax.axvline(0.0, color="k", ls="--", lw=0.8, alpha=0.6)
    ax.grid(True, alpha=0.3)
    ax.legend(title="bridge R", fontsize=9)
    fig.tight_layout()

    out = os.path.join(HERE, "ads1015_decays.png")
    fig.savefig(out, dpi=130)
    print("Saved", out)
    plt.show()


if __name__ == "__main__":
    main()
