import pandas as pd
import matplotlib.pyplot as plt

def load_and_process(filename):
    """
    Load CSV with columns:
        0: milliseconds since start
        1: ch1 voltage
        2: ch2 voltage
        3: microseconds since previous reading (ignored here)
    Convert time to timedelta for plotting.
    """
    df = pd.read_csv(
        filename,
        header=None,
        names=["milliseconds", "ch1", "ch2", "delta_us"]
    )

    # Convert milliseconds to HH:MM:SS.ffffff-style timedelta
    df["timestamp"] = pd.to_timedelta(df["milliseconds"], unit="ms")
    return df

def plot_combined_channels(file1, file2):
    # Load both files
    df1 = load_and_process(file1)
    df2 = load_and_process(file2)

    # Combine into one dataframe and sort by time
    df = pd.concat([df1, df2], ignore_index=True)
    df = df.sort_values(by="timestamp")

    # Plot
    plt.figure(figsize=(12, 6))

    plt.plot(df["timestamp"], df["ch1"], label="Channel 1", linewidth=1.4)
    plt.plot(df["timestamp"], df["ch2"], label="Channel 2", linewidth=1.4)

    plt.xlabel("Time")
    plt.ylabel("Voltage (V)")
    plt.title("Combined Voltage vs Time (CH1 & CH2)")
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Adjust names if you changed them
    plot_combined_channels("capture4.csv", "log4.csv")
