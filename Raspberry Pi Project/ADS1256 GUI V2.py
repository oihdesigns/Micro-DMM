import spidev
import lgpio
import time
import tkinter as tk
from tkinter import ttk, filedialog
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from collections import deque
import csv
import datetime
import statistics
import gc

# ---------------- ADS1256 low-level ----------------

DRDY_PIN = 17
CS_PIN   = 22
RST_PIN  = 18

CMD_WAKEUP  = 0x00
CMD_RDATA   = 0x01
CMD_RDATAC  = 0x03
CMD_SDATAC  = 0x0F
CMD_WREG    = 0x50
CMD_SYNC    = 0xFC
CMD_RESET   = 0xFE

DRATE_TABLE = {
    "30k SPS": 0xF0, "15k SPS": 0xE0, "7.5k SPS": 0xD0,
    "3.75k SPS": 0xC0, "2k SPS": 0xB0, "1k SPS": 0xA1,
    "500 SPS": 0x92, "100 SPS": 0x82, "60 SPS": 0x72,
    "50 SPS": 0x63, "30 SPS": 0x53, "25 SPS": 0x43,
    "15 SPS": 0x33, "10 SPS": 0x23, "5 SPS": 0x13,
    "2.5 SPS": 0x03,
}

GAIN_TABLE = {
    "1x": 0, "2x": 1, "4x": 2, "8x": 3,
    "16x": 4, "32x": 5, "64x": 6,
}

# Channel map: label, (pos, neg), type
CHANNEL_MAP = [
    ("CH0 (SE)", (0, 8), "SE"),
    ("CH1 (SE)", (1, 8), "SE"),
    ("CH2 (SE)", (2, 8), "SE"),
    ("CH3 (SE)", (3, 8), "SE"),
    ("CH4-5 (Diff)", (4, 5), "DIFF"),
    ("CH6-7 (Diff)", (6, 7), "DIFF"),
]

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 2000000  # 2 MHz
spi.mode = 1

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, CS_PIN)
lgpio.gpio_claim_output(h, RST_PIN)
lgpio.gpio_claim_input(h, DRDY_PIN)

def cs_low():  lgpio.gpio_write(h, CS_PIN, 0)
def cs_high(): lgpio.gpio_write(h, CS_PIN, 1)

def wait_drdy():
    while lgpio.gpio_read(h, DRDY_PIN) == 1:
        time.sleep(0.0001)

def wait_drdy_fast():
    while lgpio.gpio_read(h, DRDY_PIN) == 1:
        pass  # tight loop for speed

def send_cmd(cmd):
    cs_low(); spi.xfer2([cmd]); cs_high()

def write_reg(reg, data):
    cs_low(); spi.xfer2([CMD_WREG | reg, 0x00, data]); cs_high()

def read_data():
    cs_low()
    spi.xfer2([CMD_RDATA])
    raw = spi.xfer2([0xFF, 0xFF, 0xFF])
    cs_high()
    v = (raw[0] << 16) | (raw[1] << 8) | raw[2]
    if v & 0x800000: v -= 1 << 24
    return v

def read_data_raw_fast():
    raw = spi.xfer2([0xFF, 0xFF, 0xFF])
    v = (raw[0] << 16) | (raw[1] << 8) | raw[2]
    if v & 0x800000: v -= 1 << 24
    return v

def set_channel(p, n):
    mux = (p << 4) | n
    write_reg(0x01, mux)

# ---------------- ADC config ----------------

current_gain = 1

def ads1256_init():
    lgpio.gpio_write(h, RST_PIN, 0); time.sleep(0.01)
    lgpio.gpio_write(h, RST_PIN, 1); time.sleep(0.05)
    send_cmd(CMD_RESET); time.sleep(0.1); wait_drdy()
    set_buffer(False); set_drate("30k SPS"); set_gain("1x")

def set_buffer(enable):
    val = 0x01 if enable else 0x00
    write_reg(0x00, val)

def set_drate(name):
    code = DRATE_TABLE.get(name, 0xF0)
    write_reg(0x03, code)

def set_gain(name):
    global current_gain
    gain_code = GAIN_TABLE.get(name, 0)
    current_gain = (1 << gain_code) if gain_code > 0 else 1
    adcon = gain_code & 0x07
    write_reg(0x02, adcon)

def read_channel(ch_index, return_volts=True):
    set_channel(*CHANNEL_MAP[ch_index][1])
    send_cmd(CMD_SYNC); send_cmd(CMD_WAKEUP)
    wait_drdy()
    raw = read_data()
    if return_volts:
        return raw * 5.0 / (0x7FFFFF * current_gain)
    else:
        return raw

ads1256_init()

# ---------------- GUI ----------------

root = tk.Tk()
root.title("ADS1256 Control Panel with Data Logger")
root.geometry("800x480")
root.resizable(False, False)

notebook = ttk.Notebook(root)
notebook.pack(fill="both", expand=True)

# Control Panel
control_frame = tk.Frame(notebook)
notebook.add(control_frame, text="Control Panel")

channel_vars, labels = [], []
for i, (label_txt, _, _) in enumerate(CHANNEL_MAP):
    var = tk.BooleanVar(value=True)
    cb = tk.Checkbutton(control_frame, text=label_txt, variable=var)
    cb.grid(row=i, column=0, sticky="w", padx=5)
    val = tk.Label(control_frame, text="---", font=("Courier", 12))
    val.grid(row=i, column=1, padx=5)
    channel_vars.append(var); labels.append(val)

buffer_var = tk.BooleanVar(value=False)
tk.Checkbutton(control_frame, text="Enable Buffer", variable=buffer_var,
               command=lambda: set_buffer(buffer_var.get())).grid(row=8, column=0, pady=5, sticky="w")

tk.Label(control_frame, text="Sample Rate:").grid(row=9, column=0, sticky="w", padx=5)
drate_var = tk.StringVar(value="30k SPS")
drate_menu = ttk.Combobox(control_frame, textvariable=drate_var,
                          values=list(DRATE_TABLE.keys()), state="readonly")
drate_menu.grid(row=9, column=1, padx=5)
def change_drate(*args): set_drate(drate_var.get())
drate_var.trace_add("write", change_drate)

tk.Label(control_frame, text="Gain:").grid(row=10, column=0, sticky="w", padx=5)
gain_var = tk.StringVar(value="1x")
gain_menu = ttk.Combobox(control_frame, textvariable=gain_var,
                         values=list(GAIN_TABLE.keys()), state="readonly")
gain_menu.grid(row=10, column=1, padx=5)
def change_gain(*args): set_gain(gain_var.get())
gain_var.trace_add("write", change_gain)

raw_var = tk.BooleanVar(value=False)
tk.Checkbutton(control_frame, text="Show Raw Integer", variable=raw_var).grid(row=11, column=0, pady=5, sticky="w")

# Live Graph Panel
graph_frame = tk.Frame(notebook)
notebook.add(graph_frame, text="Live Graphs")

fig, ax = plt.subplots(figsize=(5, 2.5), dpi=100)
canvas = FigureCanvasTkAgg(fig, master=graph_frame)
canvas.get_tk_widget().pack(fill="both", expand=True)

max_points = 200
data_buffers = [deque(maxlen=max_points) for _ in CHANNEL_MAP]

def update_values():
    return_volts = not raw_var.get()
    for i in range(len(CHANNEL_MAP)):
        if channel_vars[i].get():
            val = read_channel(i, return_volts)
            labels[i].config(text=f"{val:.6f}" if return_volts else str(val))
            data_buffers[i].append(val)
        else:
            labels[i].config(text="OFF")
            data_buffers[i].append(None)

    ax.clear()
    ax.set_xlim(0, max_points)
    if return_volts:
        ax.set_ylim(-0.1, 5.1); ax.set_ylabel("Volts")
    else:
        ax.set_ylim(-9000000, 9000000); ax.set_ylabel("Raw Int")
    for i, (label_txt, _, _) in enumerate(CHANNEL_MAP):
        if channel_vars[i].get():
            y = [v if v is not None else float("nan") for v in data_buffers[i]]
            x = list(range(len(y)))
            ax.plot(x, y, label=label_txt)
    ax.legend(loc="upper right")
    canvas.draw()

    root.after(500, update_values)

# ---------------- Data Logger Tab ----------------

logger_frame = tk.Frame(notebook)
notebook.add(logger_frame, text="Data Logger")

tk.Label(logger_frame, text="Points to Capture:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
points_var = tk.StringVar(value="1000")
points_entry = tk.Entry(logger_frame, textvariable=points_var, width=10)
points_entry.grid(row=0, column=1, padx=5, pady=5)

log_fig, log_ax = plt.subplots(figsize=(5, 2.5), dpi=100)
log_canvas = FigureCanvasTkAgg(log_fig, master=logger_frame)
log_canvas.get_tk_widget().grid(row=1, column=0, columnspan=2, padx=2, pady=2)

stats_frame = tk.Frame(logger_frame)
stats_frame.grid(row=1, column=2, padx=10, sticky="n")

captured_data = None
timestamps = None

def capture_data():
    global captured_data, timestamps
    n_points = int(points_var.get())
    return_volts = not raw_var.get()
    active_channels = [i for i, v in enumerate(channel_vars) if v.get()]
    captured_data = {i: [] for i in active_channels}
    timestamps = []
    t0 = None
    for sample_idx in range(n_points):
        for i in active_channels:
            set_channel(*CHANNEL_MAP[i][1])
            send_cmd(CMD_SYNC); send_cmd(CMD_WAKEUP)
            wait_drdy()
            raw = read_data()
            val = raw * 5.0 / (0x7FFFFF * current_gain) if return_volts else raw
            if t0 is None: t0 = time.time()
            timestamps.append((time.time() - t0) * 1000.0)
            captured_data[i].append(val)

    # Plot
    log_ax.clear()
    for i in active_channels:
        y = captured_data[i]
        x = list(range(len(y))) if len(active_channels) > 1 else timestamps
        log_ax.plot(x, y, label=CHANNEL_MAP[i][0])
    log_ax.legend(loc="upper right")
    log_ax.set_title(f"Captured {n_points} points per channel")
    log_ax.set_xlabel("Samples" if len(active_channels) > 1 else "Time (ms)")
    log_ax.set_ylabel("Volts" if return_volts else "Raw Int")
    log_canvas.draw()

    # Stats
    for w in stats_frame.winfo_children(): w.destroy()
    for i in active_channels:
        vals = captured_data[i]
        txt = (f"{CHANNEL_MAP[i][0]}:\n"
               f"Min: {min(vals):.6f}\n"
               f"Max: {max(vals):.6f}\n"
               f"Avg: {statistics.mean(vals):.6f}\n"
               f"Std: {statistics.pstdev(vals):.6f}\n")
        tk.Label(stats_frame, text=txt, justify="left", anchor="w").pack(anchor="w", pady=2)

def benchmark_capture():
    global captured_data, timestamps
    n_points = int(points_var.get())
    return_volts = not raw_var.get()
    active_channels = [i for i, v in enumerate(channel_vars) if v.get()]
    if not active_channels: return
    ch_index = active_channels[0]

    set_channel(*CHANNEL_MAP[ch_index][1])
    send_cmd(CMD_RDATAC)
    wait_drdy_fast()

    gc_was_enabled = gc.isenabled()
    if gc_was_enabled: gc.disable()

    cs_low()
    t0 = time.time()
    captured_data = {ch_index: [0.0] * n_points}
    timestamps = [0.0] * n_points
    scale = 5.0 / (0x7FFFFF * current_gain) if return_volts else 1.0
    for k in range(n_points):
        wait_drdy_fast()
        raw = read_data_raw_fast()
        captured_data[ch_index][k] = raw * scale
        timestamps[k] = (time.time() - t0) * 1000.0
    cs_high()

    if gc_was_enabled: gc.enable()
    send_cmd(CMD_SDATAC)

    elapsed_ms = (time.time() - t0) * 1000.0
    rate = n_points / (elapsed_ms / 1000.0)

    # Plot
    log_ax.clear()
    y = captured_data[ch_index]; x = timestamps
    log_ax.plot(x, y, label=CHANNEL_MAP[ch_index][0])
    log_ax.legend(loc="upper right")
    log_ax.set_title(f"Benchmark {n_points} points\n{rate:.1f} SPS effective")
    log_ax.set_xlabel("Time (ms)")
    log_ax.set_ylabel("Volts" if return_volts else "Raw Int")
    log_canvas.draw()

    # Stats
    for w in stats_frame.winfo_children(): w.destroy()
    txt = (f"{CHANNEL_MAP[ch_index][0]}:\n"
           f"Samples: {n_points}\n"
           f"Rate: {rate:.1f} SPS\n"
           f"Min: {min(y):.6f}\n"
           f"Max: {max(y):.6f}\n"
           f"Avg: {statistics.mean(y):.6f}\n"
           f"Std: {statistics.pstdev(y):.6f}\n")
    tk.Label(stats_frame, text=txt, justify="left", anchor="w").pack(anchor="w", pady=2)

def export_csv():
    global captured_data, timestamps
    if captured_data is None: return
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = filedialog.asksaveasfilename(
        defaultextension=".csv",
        initialfile=f"ads1256_log_{ts}.csv",
        filetypes=[("CSV files", "*.csv")])
    if not fname: return
    with open(fname, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["Time_ms"] + [CHANNEL_MAP[i][0] for i in captured_data.keys()]
        writer.writerow(header)
        for idx in range(len(next(iter(captured_data.values())))):
            row = [f"{timestamps[idx]:.7g}"]
            for i in captured_data.keys():
                val = captured_data[i][idx]
                if isinstance(val, float): row.append(f"{val:.7g}")
                else: row.append(val)
            writer.writerow(row)

tk.Button(logger_frame, text="Capture", command=capture_data).grid(row=0, column=2, padx=5, pady=5)
tk.Button(logger_frame, text="Benchmark Capture", command=benchmark_capture).grid(row=0, column=3, padx=5, pady=5)
tk.Button(logger_frame, text="Export CSV", command=export_csv).grid(row=3, column=2, padx=5, pady=5)

# ---------------- Main Loop ----------------

update_values()
root.mainloop()

# Cleanup
spi.close()
lgpio.gpiochip_close(h)
