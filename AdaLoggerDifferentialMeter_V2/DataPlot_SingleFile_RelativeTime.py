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

    # Load at least 3 columns, ignore extras
    df = pd.read_csv(filename, header=None, usecols=[0, 1, 2],
                     names=["delta_us", "ch1", "ch2"])

    # Convert Δt column (microseconds) into a cumulative absolute timestamp
    df["abs_us"] = df["delta_us"].cumsum()

    # Convert absolute microseconds → timedelta for plotting
    df["timestamp"] = pd.to_timedelta(df["abs_us"], unit="us")

    return df

def plot_capture(filename):
    df = load_and_process(filename)

    plt.figure(figsize=(12, 6))

    plt.plot(df["timestamp"], df["ch1"], label="Channel 1", linewidth=1.4)
    plt.plot(df["timestamp"], df["ch2"], label="Channel 2", linewidth=1.4)

    plt.xlabel("Time (HH:MM:SS.ffffff)")
    plt.ylabel("Voltage (V)")
    plt.title("Voltage vs Time (Reconstructed Timeline)")
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_capture("capture3.csv")
