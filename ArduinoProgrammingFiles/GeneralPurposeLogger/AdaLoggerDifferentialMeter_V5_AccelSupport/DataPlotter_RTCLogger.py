import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from pathlib import Path

# ===============================
# USER SETTINGS
# ===============================
GAP_THRESHOLD_US = 50_000      # Highlight gaps larger than this (microseconds)
GROUP_BY_SECOND = True        # 1 Hz grouping using RTC
SHOW_DRIFT_PLOT = True        # Show RTC vs reconstructed time drift
OVERLAY_ALPHA = 0.85

# ===============================
# DATA LOADING
# ===============================
def load_log(filename):
    """
    Expected CSV format:
        Row 0: Log Start Time,<ISO8601>
        Row 1: Column headers
        Data rows:
            delta_us, ch1, ch2, rtc_timestamp
    """

    df = pd.read_csv(
        filename,
        skiprows=1,
        usecols=[0, 1, 2, 3],
        names=["delta_us", "ch1", "ch2", "rtc"]
    )

    # ---- Force numeric conversion
    df["delta_us"] = pd.to_numeric(df["delta_us"], errors="coerce")
    df["ch1"] = pd.to_numeric(df["ch1"], errors="coerce")
    df["ch2"] = pd.to_numeric(df["ch2"], errors="coerce")

    # ---- Explicit datetime parsing (critical)
    df["rtc"] = pd.to_datetime(
        df["rtc"],
        format="mixed",
        errors="coerce"
    )

    # Drop any malformed rows
    df = df.dropna(subset=["rtc", "delta_us"])

    # ---- Relative time reconstruction
    df["elapsed_us"] = df["delta_us"].cumsum()
    df["elapsed_s"] = df["elapsed_us"] / 1_000_000.0

    # ---- RTC-relative elapsed time
    t0 = df["rtc"].iloc[0]
    df["rtc_elapsed_s"] = (df["rtc"] - t0).dt.total_seconds()

    df["source"] = Path(filename).stem
    return df

# ===============================
# MAIN PLOT
# ===============================
def plot_logs(filenames):
    dfs = [load_log(f) for f in filenames]
    df_all = pd.concat(dfs, ignore_index=True)

    fig, ax = plt.subplots(figsize=(14, 7))

    # ---- Plot signals
    for name, df in df_all.groupby("source"):
        if GROUP_BY_SECOND:
            df_plot = (
                df.set_index("rtc")
                  .resample("1s")
                  .mean(numeric_only=True)
                  .dropna()
            )
        else:
            df_plot = df.set_index("rtc")

        ax.plot(
            df_plot.index,
            df_plot["ch1"],
            label=f"{name} – CH1",
            alpha=OVERLAY_ALPHA
        )
        ax.plot(
            df_plot.index,
            df_plot["ch2"],
            linestyle="--",
            label=f"{name} – CH2",
            alpha=OVERLAY_ALPHA
        )

    # ---- Highlight timing gaps
    gaps = df_all[df_all["delta_us"] > GAP_THRESHOLD_US]
    if not gaps.empty:
        ax.scatter(
            gaps["rtc"],
            gaps["ch1"],
            color="red",
            s=18,
            label=f"Gaps > {GAP_THRESHOLD_US/1000:.0f} ms"
        )

    # ---- Axis formatting
    ax.set_xlabel("RTC Time")
    ax.set_ylabel("Voltage (V)")
    ax.set_title("Voltage vs Time (RTC with Timing Diagnostics)")
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))

    # ===============================
    # SECONDARY X AXIS (Elapsed Time)
    # ===============================
    rtc_base = df_all["rtc"].min()
    rtc_base_num = mdates.date2num(rtc_base)

    def rtc_to_elapsed(x):
        # x is matplotlib date numbers (days)
        return (x - rtc_base_num) * 86400.0  # days → seconds

    def elapsed_to_rtc(x):
        # seconds → matplotlib date numbers
        return x / 86400.0 + rtc_base_num

    secax = ax.secondary_xaxis(
        "top",
        functions=(rtc_to_elapsed, elapsed_to_rtc)
    )
    secax.set_xlabel("Elapsed Time (s)")

    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

    if SHOW_DRIFT_PLOT:
        plot_drift(df_all)

# ===============================
# DRIFT ANALYSIS
# ===============================
def plot_drift(df):
    df = df.copy()
    df["drift_s"] = df["rtc_elapsed_s"] - df["elapsed_s"]

    plt.figure(figsize=(12, 4))
    plt.plot(df["rtc"], df["drift_s"], linewidth=1.2)
    plt.axhline(0, color="black", linewidth=0.8)

    plt.xlabel("RTC Time")
    plt.ylabel("RTC – Δt Accumulation (s)")
    plt.title("Clock Drift: RTC vs Reconstructed Time")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

# ===============================
# ENTRY POINT
# ===============================
if __name__ == "__main__":
    FILES = [
        "log_20251220_055743.csv",
        # add more files here
    ]

    plot_logs(FILES)
