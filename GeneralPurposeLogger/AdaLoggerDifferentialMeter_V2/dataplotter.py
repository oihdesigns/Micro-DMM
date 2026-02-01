import pandas as pd
import matplotlib.pyplot as plt

def load_and_process(filename):
    """Load CSV and convert microseconds → timestamp."""
    df = pd.read_csv(filename, header=None, names=["microseconds", "voltage", "ch2"])
    df["timestamp"] = pd.to_timedelta(df["microseconds"], unit="us")
    return df

def plot_two_files(file1, file2):
    df1 = load_and_process(file1)
    df2 = load_and_process(file2)

    plt.figure(figsize=(12, 6))

    plt.plot(df1["timestamp"], df1["voltage"], label=file1, linewidth=1.3)
    plt.plot(df2["timestamp"], df2["voltage"], label=file2, linewidth=1.3)

    plt.xlabel("Time (HH:MM:SS.ffffff)")
    plt.ylabel("Voltage (V)")
    plt.title("Voltage vs Time — Combined Plot")
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_two_files("capture.csv", "data.csv")
