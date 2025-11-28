import pandas as pd
import matplotlib.pyplot as plt

def load_and_process(filename):
    """Load CSV and convert microseconds → timestamp."""
    df = pd.read_csv(filename, header=None, names=["microseconds", "ch1", "ch2"])
    df["timestamp"] = pd.to_timedelta(df["microseconds"], unit="us")
    return df

def plot_combined_channels(file1, file2):
    # Load both
    df1 = load_and_process(file1)
    df2 = load_and_process(file2)

    # Combine into one dataframe
    df = pd.concat([df1, df2], ignore_index=True)

    # Sort by timestamp (important if files overlap)
    df = df.sort_values(by="timestamp")

    plt.figure(figsize=(12, 6))

    # Plot two channels only
    plt.plot(df["timestamp"], df["ch1"], label="Channel 1", linewidth=1.4)
    plt.plot(df["timestamp"], df["ch2"], label="Channel 2", linewidth=1.4)

    plt.xlabel("Time (uS)")
    plt.ylabel("Voltage (V)")
    plt.title("Combined Voltage vs Time (CH1 & CH2)")
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_combined_channels("capture.csv", "data.csv")
