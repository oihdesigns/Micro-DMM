import pandas as pd
import matplotlib.pyplot as plt

def load_and_process(filename):
    """
    CSV format:
        0: microseconds since previous reading (Δt)
        1: CH1 voltage
        2: CH2 voltage
        3+: extra columns ignored
    """

    df = pd.read_csv(filename, header=None, usecols=[0, 1, 2],
                     names=["delta_us", "ch1", "ch2"])

    # Build absolute timestamp using cumulative sum
    df["abs_us"] = df["delta_us"].cumsum()

    # Convert absolute microseconds → timedelta
    df["timestamp"] = pd.to_timedelta(df["abs_us"], unit="us")

    return df

def plot_capture(filename):
    df = load_and_process(filename)

    # Total runtime is the last cumulative timestamp
    total_runtime = df["timestamp"].iloc[-1]

    plt.figure(figsize=(12, 6))

    plt.plot(df["timestamp"], df["ch1"], label="Channel 1", linewidth=1.4)
    plt.plot(df["timestamp"], df["ch2"], label="Channel 2", linewidth=1.4)

    plt.xlabel("Time (HH:MM:SS.ffffff)")
    plt.ylabel("Voltage (V)")
    plt.title("Voltage vs Time (Reconstructed Timeline)")

    # Add runtime annotation in the upper-left corner
    plt.text(
        0.01, 0.97,
        f"Total Runtime: {total_runtime}",
        transform=plt.gca().transAxes,
        fontsize=11,
        verticalalignment="top",
        bbox=dict(facecolor="white", alpha=0.7, edgecolor="none")
    )

    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_capture("capture9.csv")
