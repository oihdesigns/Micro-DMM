import pandas as pd
import matplotlib.pyplot as plt

def plot_csv(filename):
    # Load CSV (3 columns: time_us, voltage, unused)
    df = pd.read_csv(filename, header=None, names=["microseconds", "voltage", "ch2"])

    # Convert microseconds → HH:MM:SS.ffffff
    df["timestamp"] = pd.to_timedelta(df["microseconds"], unit="us")

    # Plot
    plt.figure(figsize=(10, 5))
    plt.plot(df["timestamp"], df["voltage"])
    plt.xlabel("Time (HH:MM:SS.ffffff)")
    plt.ylabel("Voltage (V)")
    plt.title("Voltage vs Time")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_csv("capture.csv")
