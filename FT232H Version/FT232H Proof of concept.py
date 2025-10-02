import time
import math
import board
import busio
import digitalio
import tkinter as tk
from tkinter import ttk
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn

# -----------------------------
# I2C & ADS1115 Setup
# -----------------------------
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS1115(i2c)

# Defaults
ads.gain = 1          # +/- 4.096 V
ads.data_rate = 128   # SPS

# Channels: CH0-CH1 differential, CH2 single, CH3 single
chan_diff = AnalogIn(ads, 0, 1)   # differential A0-A1
chan2 = AnalogIn(ads, 2)          # single-ended A2
chan3 = AnalogIn(ads, 3)          # single-ended A3

# -----------------------------
# GPIO Setup (C0-C9)
# -----------------------------
gpio_pins = {}
for i in range(10):
    pin_name = f"C{i}"
    try:
        pin = digitalio.DigitalInOut(getattr(board, pin_name))
        pin.direction = digitalio.Direction.OUTPUT
        pin.value = False
        gpio_pins[pin_name] = pin
    except AttributeError:
        print(f"Pin {pin_name} not available on this board")

# -----------------------------
# ADS1115 Gain/Data-Rate helpers
# -----------------------------
# Combobox display -> numeric value the driver expects
GAIN_MAP = {
    "2/3": 2/3,  # ±6.144 V
    "1": 1,      # ±4.096 V
    "2": 2,      # ±2.048 V
    "4": 4,      # ±1.024 V
    "8": 8,      # ±0.512 V
    "16": 16,    # ±0.256 V
}

# Numeric gain -> full-scale voltage (±FS)
FS_FOR_GAIN = {
    2/3: 6.144,
    1:   4.096,
    2:   2.048,
    4:   1.024,
    8:   0.512,
    16:  0.256,
}

RATE_OPTIONS = ["8", "16", "32", "64", "128", "250", "475", "860"]
GAIN_OPTIONS = list(GAIN_MAP.keys())  # ["2/3","1","2","4","8","16"]

def gain_numeric_to_str(g):
    # Handle 2/3 being a float
    if abs(g - (2/3)) < 1e-9:
        return "2/3"
    return str(int(g))

def current_fullscale_and_lsb(g):
    fs = FS_FOR_GAIN[g]
    # ADS1115 is 16-bit signed: LSB = FS / 32768
    lsb_v = fs / 32768.0
    return fs, lsb_v

# -----------------------------
# GUI Functions
# -----------------------------
def update_adc():
    """Update ADC readings in GUI"""
    try:
        diff_val.set(f"{chan_diff.voltage:.6f} V")
    except Exception as e:
        diff_val.set(f"ERR: {e}")

    try:
        ch2_val.set(f"{chan2.voltage:.6f} V")
    except Exception as e:
        ch2_val.set(f"ERR: {e}")

    try:
        ch3_val.set(f"{chan3.voltage:.6f} V")
    except Exception as e:
        ch3_val.set(f"ERR: {e}")

    root.after(500, update_adc)  # schedule again in 500ms

def toggle_gpio(pin_name):
    """Toggle GPIO pin output"""
    pin = gpio_pins[pin_name]
    pin.value = not pin.value
    gpio_states[pin_name].set("HIGH" if pin.value else "LOW")

def set_gain(event=None):
    """Set ADS1115 gain from dropdown (handles '2/3')"""
    sel = gain_combo.get()
    g = GAIN_MAP[sel]
    ads.gain = g
    fs, lsb = current_fullscale_and_lsb(ads.gain)
    fs_label_var.set(f"Range: ±{fs:.3f} V   •   1 LSB ≈ {lsb*1e6:.1f} µV")
    print(f"Set ADS1115 gain = {sel} (±{fs:.3f} V)")

def set_data_rate(event=None):
    """Set ADS1115 sample rate from dropdown"""
    val = int(rate_combo.get())
    ads.data_rate = val
    print(f"Set ADS1115 data rate = {ads.data_rate} SPS")

# -----------------------------
# Build GUI
# -----------------------------
root = tk.Tk()
root.title("FT232H ADS1115 + GPIO Test")

# Frame for ADC readings
adc_frame = ttk.LabelFrame(root, text="ADS1115 Readings")
adc_frame.pack(padx=10, pady=10, fill="x")

ttk.Label(adc_frame, text="CH0-CH1 (Diff):").grid(row=0, column=0, sticky="w")
diff_val = tk.StringVar()
ttk.Label(adc_frame, textvariable=diff_val).grid(row=0, column=1, sticky="w")

ttk.Label(adc_frame, text="CH2:").grid(row=1, column=0, sticky="w")
ch2_val = tk.StringVar()
ttk.Label(adc_frame, textvariable=ch2_val).grid(row=1, column=1, sticky="w")

ttk.Label(adc_frame, text="CH3:").grid(row=2, column=0, sticky="w")
ch3_val = tk.StringVar()
ttk.Label(adc_frame, textvariable=ch3_val).grid(row=2, column=1, sticky="w")

# Frame for ADS1115 controls
ctrl_frame = ttk.LabelFrame(root, text="ADS1115 Controls")
ctrl_frame.pack(padx=10, pady=10, fill="x")

# Gain control
ttk.Label(ctrl_frame, text="Gain:").grid(row=0, column=0, sticky="w")
gain_combo = ttk.Combobox(ctrl_frame, values=GAIN_OPTIONS, state="readonly", width=6)
gain_combo.set(gain_numeric_to_str(ads.gain))
gain_combo.grid(row=0, column=1, padx=5, pady=5)
gain_combo.bind("<<ComboboxSelected>>", set_gain)

# Sample rate control
ttk.Label(ctrl_frame, text="Data Rate (SPS):").grid(row=1, column=0, sticky="w")
rate_combo = ttk.Combobox(ctrl_frame, values=RATE_OPTIONS, state="readonly", width=6)
# Ensure the default is in the allowed list; fallback to 128
rate_combo.set(str(ads.data_rate if str(ads.data_rate) in RATE_OPTIONS else "128"))
rate_combo.grid(row=1, column=1, padx=5, pady=5)
rate_combo.bind("<<ComboboxSelected>>", set_data_rate)

# Range/LSB display
fs_label_var = tk.StringVar()
fs, lsb = current_fullscale_and_lsb(ads.gain)
fs_label_var.set(f"Range: ±{fs:.3f} V   •   1 LSB ≈ {lsb*1e6:.1f} µV")
ttk.Label(ctrl_frame, textvariable=fs_label_var).grid(row=2, column=0, columnspan=2, sticky="w", pady=(4,0))

# Frame for GPIO controls
gpio_frame = ttk.LabelFrame(root, text="GPIO Controls (C0-C9)")
gpio_frame.pack(padx=10, pady=10, fill="x")

gpio_states = {}
for i, pin_name in enumerate(gpio_pins.keys()):
    gpio_states[pin_name] = tk.StringVar(value="LOW")
    btn = ttk.Button(
        gpio_frame,
        text=pin_name,
        command=lambda p=pin_name: toggle_gpio(p)
    )
    btn.grid(row=i//5, column=(i%5)*2, padx=5, pady=5)
    lbl = ttk.Label(gpio_frame, textvariable=gpio_states[pin_name])
    lbl.grid(row=i//5, column=(i%5)*2+1, padx=5, pady=5)

# Start updating ADC readings
update_adc()

root.mainloop()
