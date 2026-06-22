#!/usr/bin/env python3
"""Overlay RA4M1/XIAO capture decays, one panel per test condition.

Four conditions are inferred from the filename:
  - bare:                DataCollection_RA4M1_<R>Ohm.csv
  - DUT-facing:          DataCollection_RA4M1_DUT-Facing_<R>Ohm.csv
  - LM4060:              DataCollection_RA4M1_LM4060_<R>Ohm.csv
  - LM4060 + DUT-facing: DataCollection_RA4M1_LM4060-DUTFacing_<R>Ohm.csv

Each panel overlays diffV vs t_ms_from_toggle for every resistance in that
condition, legend ordered by resistance. Run from the project folder:
    python plot_xiao_decays.py
"""
import csv
import glob
import os
import re

import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))

# (has_lm4060, is_dut_facing) -> panel title
CONDITIONS = [
    (False, False, "Bare"),
    (False, True,  "DUT-facing"),
    (True,  False, "LM4060"),
    (True,  True,  "LM4060 + DUT-facing"),
]


def classify(name):
    flat = name.replace("-", "").replace("_", "").lower()
    return ("lm4060" in flat, "dutfacing" in flat)


def ohms(name):
    m = re.search(r"_(\d+(?:\.\d+)?)(K|M)Ohm", name, re.IGNORECASE)
    if not m:
        return float("inf")
    return float(m.group(1)) * {"K": 1e3, "M": 1e6}[m.group(2).upper()]


def label(name):
    m = re.search(r"(\d+(?:\.\d+)?(?:K|M)Ohm)", name, re.IGNORECASE)
    return m.group(1) if m else os.path.basename(name)


def load(path):
    """Return (t_ms, v) for either CSV format used in this folder:
      - firmware capture:  t_ms_from_toggle, diffV
      - scope/DUT export:  'Time (ms)', 'A2-A3 (V)'
    """
    t, v = [], []
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        cols = reader.fieldnames or []
        if "diffV" in cols:
            tcol, vcol = "t_ms_from_toggle", "diffV"
        elif "Time (ms)" in cols:
            tcol, vcol = "Time (ms)", "A2-A3 (V)"
        else:
            return t, v
        for row in reader:
            try:
                t.append(float(row[tcol]))
                v.append(float(row[vcol]))
            except (KeyError, ValueError):
                continue
    return t, v


def main():
    files = glob.glob(os.path.join(HERE, "DataCollection_RA4M1_*.csv"))
    if not files:
        print("No DataCollection_RA4M1_*.csv files found in", HERE)
        return

    buckets = {(lm, dut): [] for lm, dut, _ in CONDITIONS}
    for path in files:
        buckets[classify(os.path.basename(path))].append(path)

    # Each panel autoscales: the firmware-capture and scope-export files use
    # different sign/amplitude scales, so shared axes would hide one of them.
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    for ax, (lm, dut, title) in zip(axes.flat, CONDITIONS):
        group = sorted(buckets[(lm, dut)], key=lambda p: ohms(os.path.basename(p)))
        for path in group:
            t, v = load(path)
            if t:
                ax.plot(t, v, ".-", ms=2, lw=0.9, label=label(os.path.basename(path)))
        ax.set_title(f"{title}  ({len(group)} traces)")
        ax.axvline(0.0, color="k", ls="--", lw=0.8, alpha=0.6)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("time from toggle (ms)")
        ax.set_ylabel("differential voltage (V)")
        if group:
            ax.legend(title="bridge R", fontsize=8)

    fig.suptitle("RA4M1 / XIAO capture decays by test condition", fontsize=14)
    fig.tight_layout()

    out = os.path.join(HERE, "xiao_decays.png")
    fig.savefig(out, dpi=130)
    print("Saved", out)


if __name__ == "__main__":
    main()
