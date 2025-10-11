import csv
import datetime
import gc
import math
import statistics
import time
from collections import deque

import lgpio
import matplotlib.pyplot as plt
import spidev
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tkinter import ttk, filedialog

# ---------------- ADS1256 low-level ----------------

DRDY_PIN = 17
CS_PIN = 22
RST_PIN = 18
RANGE_PIN = 23

CMD_WAKEUP = 0x00
CMD_RDATA = 0x01
CMD_RDATAC = 0x03
CMD_SDATAC = 0x0F
CMD_WREG = 0x50
CMD_SYNC = 0xFC
CMD_RESET = 0xFE

DRATE_TABLE = {
    "30k SPS": 0xF0,
    "15k SPS": 0xE0,
    "7.5k SPS": 0xD0,
    "3.75k SPS": 0xC0,
    "2k SPS": 0xB0,
    "1k SPS": 0xA1,
    "500 SPS": 0x92,
    "100 SPS": 0x82,
    "60 SPS": 0x72,
    "50 SPS": 0x63,
    "30 SPS": 0x53,
    "25 SPS": 0x43,
    "15 SPS": 0x33,
    "10 SPS": 0x23,
    "5 SPS": 0x13,
    "2.5 SPS": 0x03,
}

GAIN_TABLE = {
    "1x": 0,
    "2x": 1,
    "4x": 2,
    "8x": 3,
    "16x": 4,
    "32x": 5,
    "64x": 6,
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

TRIGGER_MODES = ("Benchmark", "Multi")
TRIGGER_SLOPES = ("Rising", "Falling")
TRIGGER_POSITIONS = ("Start", "Mid", "End")

ALL_GPIO_PINS = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16, 19, 20, 21, 24, 25, 26, 27]
SPI_PINS = {8, 9, 10, 11}
RESERVED_PINS = {DRDY_PIN, CS_PIN, RST_PIN, RANGE_PIN} | SPI_PINS
USER_GPIO_PINS = [pin for pin in ALL_GPIO_PINS if pin not in RESERVED_PINS]

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 2_000_000
spi.mode = 1

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, CS_PIN)
lgpio.gpio_claim_output(h, RST_PIN)
lgpio.gpio_claim_input(h, DRDY_PIN)
lgpio.gpio_claim_output(h, RANGE_PIN)
lgpio.gpio_write(h, CS_PIN, 1)


def cs_low() -> None:
    lgpio.gpio_write(h, CS_PIN, 0)


def cs_high() -> None:
    lgpio.gpio_write(h, CS_PIN, 1)


def wait_drdy() -> None:
    while lgpio.gpio_read(h, DRDY_PIN) == 1:
        time.sleep(0.0001)


def wait_drdy_fast() -> None:
    while lgpio.gpio_read(h, DRDY_PIN) == 1:
        pass


def send_cmd(cmd: int) -> None:
    cs_low()
    spi.xfer2([cmd])
    cs_high()


def write_reg(reg: int, data: int) -> None:
    cs_low()
    spi.xfer2([CMD_WREG | reg, 0x00, data])
    cs_high()


def read_data() -> int:
    cs_low()
    spi.xfer2([CMD_RDATA])
    raw = spi.xfer2([0xFF, 0xFF, 0xFF])
    cs_high()
    value = (raw[0] << 16) | (raw[1] << 8) | raw[2]
    if value & 0x800000:
        value -= 1 << 24
    return value


def read_data_raw_fast() -> int:
    cs_low()
    raw = spi.xfer2([0xFF, 0xFF, 0xFF])
    cs_high()
    value = (raw[0] << 16) | (raw[1] << 8) | raw[2]
    if value & 0x800000:
        value -= 1 << 24
    return value


def set_channel(p: int, n: int) -> None:
    mux = (p << 4) | n
    write_reg(0x01, mux)


# ---------------- ADC config ----------------

current_gain = 1


def ads1256_init() -> None:
    lgpio.gpio_write(h, RST_PIN, 0)
    time.sleep(0.01)
    lgpio.gpio_write(h, RST_PIN, 1)
    time.sleep(0.05)
    send_cmd(CMD_RESET)
    time.sleep(0.1)
    wait_drdy()
    send_cmd(CMD_SDATAC)
    set_buffer(False)
    set_drate("30k SPS")
    set_gain("1x")


def set_buffer(enable: bool) -> None:
    val = 0x01 if enable else 0x00
    write_reg(0x00, val)


def set_drate(name: str) -> None:
    code = DRATE_TABLE.get(name, 0xF0)
    write_reg(0x03, code)


def set_gain(name: str) -> None:
    global current_gain
    gain_code = GAIN_TABLE.get(name, 0)
    current_gain = (1 << gain_code) if gain_code > 0 else 1
    write_reg(0x02, gain_code & 0x07)


def read_channel_raw(ch_index: int) -> int:
    set_channel(*CHANNEL_MAP[ch_index][1])
    send_cmd(CMD_SYNC)
    send_cmd(CMD_WAKEUP)
    wait_drdy()
    return read_data()


def raw_to_volts(raw: int) -> float:
    return raw * 5.0 / (0x7FFFFF * current_gain)


ads1256_init()

# ---------------- GUI setup ----------------

root = tk.Tk()
root.title("ADS1256 Control Panel with Triggering")
root.geometry("1500x900")
root.configure(padx=12, pady=12)
root.columnconfigure(1, weight=1)
root.rowconfigure(0, weight=1)
root.rowconfigure(1, weight=1)

HEADER_FONT = ("TkDefaultFont", 18, "bold")
CHANNEL_FONT = ("TkDefaultFont", 14)
VALUE_FONT = ("Courier New", 16, "bold")
UNIT_FONT = ("TkDefaultFont", 12, "bold")
BUTTON_FONT = ("TkDefaultFont", 13)

status_var = tk.StringVar(value="Idle")

left_panel = ttk.Frame(root)
left_panel.grid(row=0, column=0, rowspan=2, sticky="nsw", padx=(0, 12))

right_panel = ttk.Frame(root)
right_panel.grid(row=0, column=1, sticky="nsew")
right_panel.columnconfigure(0, weight=1)
right_panel.rowconfigure(0, weight=1)
right_panel.rowconfigure(1, weight=1)

channel_frame = ttk.LabelFrame(left_panel, text="Channel Controls")
channel_frame.grid(row=0, column=0, sticky="nw", pady=(0, 12))

channel_vars: list[tk.BooleanVar] = []
value_labels: list[tk.Label] = []
unit_labels: list[tk.Label] = []
channel_units: list[str] = [""] * len(CHANNEL_MAP)

for row, (label_txt, _, _) in enumerate(CHANNEL_MAP):
    var = tk.BooleanVar(value=True)
    cb = tk.Checkbutton(
        channel_frame,
        text=label_txt,
        font=CHANNEL_FONT,
        variable=var,
        anchor="w",
        justify="left",
    )
    cb.grid(row=row, column=0, sticky="w", padx=4, pady=2)
    val_label = tk.Label(channel_frame, text="---", font=VALUE_FONT, width=14, anchor="e")
    val_label.grid(row=row, column=1, padx=6, pady=2, sticky="e")
    unit_label = tk.Label(channel_frame, text="", font=UNIT_FONT, width=6, anchor="w")
    unit_label.grid(row=row, column=2, padx=4, pady=2, sticky="w")
    channel_vars.append(var)
    value_labels.append(val_label)
    unit_labels.append(unit_label)

options_frame = ttk.LabelFrame(left_panel, text="Acquisition Options")
options_frame.grid(row=1, column=0, sticky="new")

buffer_var = tk.BooleanVar(value=False)

buffer_check = tk.Checkbutton(
    options_frame,
    text="Enable Buffer",
    variable=buffer_var,
    font=CHANNEL_FONT,
    command=lambda: set_buffer(buffer_var.get()),
)
buffer_check.grid(row=0, column=0, sticky="w", padx=4, pady=4)

raw_var = tk.BooleanVar(value=False)


def on_raw_toggle() -> None:
    update_trigger_units()


raw_check = tk.Checkbutton(
    options_frame,
    text="Show Raw Integer",
    variable=raw_var,
    font=CHANNEL_FONT,
    command=on_raw_toggle,
)
raw_check.grid(row=0, column=1, sticky="w", padx=4, pady=4)

rate_label = ttk.Label(options_frame, text="Sample Rate:", font=CHANNEL_FONT)
rate_label.grid(row=1, column=0, sticky="w", padx=4, pady=4)

drate_var = tk.StringVar(value="30k SPS")
drate_menu = ttk.Combobox(
    options_frame,
    textvariable=drate_var,
    values=list(DRATE_TABLE.keys()),
    state="readonly",
    width=12,
)
drate_menu.grid(row=1, column=1, sticky="w", padx=4, pady=4)

def change_drate(*_: object) -> None:
    set_drate(drate_var.get())


drate_var.trace_add("write", change_drate)

gain_label = ttk.Label(options_frame, text="Gain:", font=CHANNEL_FONT)
gain_label.grid(row=2, column=0, sticky="w", padx=4, pady=4)

gain_var = tk.StringVar(value="1x")
gain_menu = ttk.Combobox(
    options_frame,
    textvariable=gain_var,
    values=list(GAIN_TABLE.keys()),
    state="readonly",
    width=12,
)
gain_menu.grid(row=2, column=1, sticky="w", padx=4, pady=4)

def change_gain(*_: object) -> None:
    set_gain(gain_var.get())


gain_var.trace_add("write", change_gain)

# DMM math
math_frame = ttk.LabelFrame(left_panel, text="DMM Math")
math_frame.grid(row=2, column=0, sticky="new", pady=12)

dmm_math_var = tk.BooleanVar(value=False)
dmm_range_var = tk.BooleanVar(value=True)


def apply_range_gpio(state: bool) -> None:
    try:
        lgpio.gpio_write(h, RANGE_PIN, 1 if state else 0)
    except Exception:
        status_var.set(f"Failed to set GPIO {RANGE_PIN} range state")


def on_range_toggle() -> None:
    apply_range_gpio(dmm_range_var.get())
    update_trigger_units()


def on_dmm_math_toggle() -> None:
    range_check.configure(state="normal" if dmm_math_var.get() else "disabled")
    update_trigger_units()
    update_values()


dmm_math_check = tk.Checkbutton(
    math_frame,
    text="Enable DMM Math",
    font=CHANNEL_FONT,
    variable=dmm_math_var,
    command=on_dmm_math_toggle,
)
dmm_math_check.grid(row=0, column=0, sticky="w", padx=4, pady=4)

range_check = tk.Checkbutton(
    math_frame,
    text=f"High Range (GPIO {RANGE_PIN})",
    font=CHANNEL_FONT,
    variable=dmm_range_var,
    command=on_range_toggle,
    state="disabled",
)
range_check.grid(row=1, column=0, sticky="w", padx=4, pady=4)

apply_range_gpio(dmm_range_var.get())

# GPIO control

gpio_frame = ttk.LabelFrame(left_panel, text="GPIO Controls")
gpio_frame.grid(row=3, column=0, sticky="new")

ttk.Label(
    gpio_frame,
    text=f"Reserved pins: {', '.join(str(p) for p in sorted(RESERVED_PINS))}",
    font=("TkDefaultFont", 11, "italic"),
).grid(row=0, column=0, columnspan=2, sticky="w", padx=4, pady=(4, 8))

gpio_mode_vars: dict[int, tk.StringVar] = {}
gpio_mode_last: dict[int, str] = {}
gpio_unavailable: set[int] = set()


def set_gpio_mode(pin: int, mode: str, update_status: bool = True) -> bool:
    try:
        try:
            lgpio.gpio_free(h, pin)
        except Exception:
            pass
        if mode == "Input":
            lgpio.gpio_claim_input(h, pin)
        else:
            lgpio.gpio_claim_output(h, pin)
            lgpio.gpio_write(h, pin, 1 if mode == "Output High" else 0)
        gpio_mode_last[pin] = mode
        return True
    except Exception as exc:
        if update_status:
            status_var.set(f"Failed to configure GPIO {pin} as {mode}: {exc}")
        return False


def on_gpio_mode_change(pin: int, *_: object) -> None:
    if pin in gpio_unavailable:
        return
    mode = gpio_mode_vars[pin].get()
    if not set_gpio_mode(pin, mode):
        previous = gpio_mode_last.get(pin, "Input")
        if previous != mode:
            gpio_mode_vars[pin].set(previous)


for idx, pin in enumerate(USER_GPIO_PINS, start=1):
    ttk.Label(gpio_frame, text=f"GPIO {pin}", font=CHANNEL_FONT).grid(
        row=idx, column=0, sticky="w", padx=4, pady=2
    )
    var = tk.StringVar(value="Input")
    combo = ttk.Combobox(
        gpio_frame,
        textvariable=var,
        values=["Input", "Output Low", "Output High"],
        state="readonly",
        width=14,
    )
    combo.grid(row=idx, column=1, sticky="ew", padx=4, pady=2)
    gpio_mode_vars[pin] = var
    if set_gpio_mode(pin, "Input", update_status=False):
        combo.bind("<<ComboboxSelected>>", lambda _e, p=pin: on_gpio_mode_change(p))
    else:
        gpio_unavailable.add(pin)
        combo.configure(state="disabled", values=("Unavailable",))
        var.set("Unavailable")
        status_var.set(f"GPIO {pin} unavailable; control disabled")

# Right panel content

live_frame = ttk.LabelFrame(right_panel, text="Live Graphs")
live_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 12))
live_frame.columnconfigure(0, weight=1)
live_frame.columnconfigure(1, weight=1)
live_frame.rowconfigure(1, weight=1)

live_values_var = tk.BooleanVar(value=True)
live_graph_var = tk.BooleanVar(value=True)


def on_live_values_toggle() -> None:
    if live_values_var.get():
        status_var.set("Live readings resumed")
        for idx, lbl in enumerate(value_labels):
            if channel_vars[idx].get():
                lbl.config(text="---")
        update_values()
    else:
        if not is_capturing:
            status_var.set("Channel readings paused")
        for idx, lbl in enumerate(value_labels):
            if channel_vars[idx].get():
                lbl.config(text="PAUSED")


def on_live_graph_toggle() -> None:
    if live_graph_var.get():
        status_var.set("Live graph enabled")
        update_graph()
    else:
        if not is_capturing:
            status_var.set("Live graph paused")


values_toggle = tk.Checkbutton(
    live_frame,
    text="Update Channel Readings",
    font=CHANNEL_FONT,
    variable=live_values_var,
    command=on_live_values_toggle,
)
values_toggle.grid(row=0, column=0, sticky="w", padx=4, pady=(4, 0))

live_toggle = tk.Checkbutton(
    live_frame,
    text="Live Graph Enabled",
    font=CHANNEL_FONT,
    variable=live_graph_var,
    command=on_live_graph_toggle,
)
live_toggle.grid(row=0, column=1, sticky="w", padx=4, pady=(4, 0))

fig, ax = plt.subplots(figsize=(9, 4), dpi=100)
canvas = FigureCanvasTkAgg(fig, master=live_frame)
canvas.get_tk_widget().grid(row=1, column=0, columnspan=2, sticky="nsew")

max_points = 300
data_buffers = [deque(maxlen=max_points) for _ in CHANNEL_MAP]

logger_frame = ttk.LabelFrame(right_panel, text="Data Logger & Triggering")
logger_frame.grid(row=1, column=0, sticky="nsew")
logger_frame.columnconfigure(0, weight=1)
logger_frame.columnconfigure(1, weight=1)
logger_frame.columnconfigure(2, weight=1)

points_var = tk.StringVar(value="1000")
trigger_level_var = tk.StringVar(value="0.0")
trigger_mode_var = tk.StringVar(value=TRIGGER_MODES[0])
trigger_slope_var = tk.StringVar(value=TRIGGER_SLOPES[0])
trigger_position_var = tk.StringVar(value=TRIGGER_POSITIONS[0])

channel_names = [c[0] for c in CHANNEL_MAP]
trigger_channel_var = tk.StringVar(value=channel_names[0])

CHANNEL_LOOKUP = {name: idx for idx, name in enumerate(channel_names)}

row_counter = 0

points_label = ttk.Label(logger_frame, text="Points to Capture:", font=CHANNEL_FONT)
points_label.grid(row=row_counter, column=0, sticky="w", padx=4, pady=4)
points_entry = ttk.Entry(logger_frame, textvariable=points_var, width=12, font=CHANNEL_FONT)
points_entry.grid(row=row_counter, column=1, sticky="w", padx=4, pady=4)
row_counter += 1

trigger_mode_label = ttk.Label(logger_frame, text="Mode:", font=CHANNEL_FONT)
trigger_mode_label.grid(row=row_counter, column=0, sticky="w", padx=4, pady=4)

trigger_mode_combo = ttk.Combobox(
    logger_frame,
    textvariable=trigger_mode_var,
    values=list(TRIGGER_MODES),
    state="readonly",
    width=14,
)
trigger_mode_combo.grid(row=row_counter, column=1, sticky="w", padx=4, pady=4)
row_counter += 1

trigger_channel_label = ttk.Label(logger_frame, text="Trigger Channel:", font=CHANNEL_FONT)
trigger_channel_label.grid(row=row_counter, column=0, sticky="w", padx=4, pady=4)

trigger_channel_combo = ttk.Combobox(
    logger_frame,
    textvariable=trigger_channel_var,
    values=channel_names,
    state="readonly",
    width=18,
)
trigger_channel_combo.grid(row=row_counter, column=1, sticky="w", padx=4, pady=4)
row_counter += 1

trigger_level_label = ttk.Label(logger_frame, text="Trigger Level:", font=CHANNEL_FONT)
trigger_level_label.grid(row=row_counter, column=0, sticky="w", padx=4, pady=4)

trigger_level_entry = ttk.Entry(logger_frame, textvariable=trigger_level_var, width=12, font=CHANNEL_FONT)
trigger_level_entry.grid(row=row_counter, column=1, sticky="w", padx=4, pady=4)

trigger_units_label = ttk.Label(logger_frame, text="Units: V", font=CHANNEL_FONT)
trigger_units_label.grid(row=row_counter, column=2, sticky="w", padx=4, pady=4)
row_counter += 1

trigger_slope_label = ttk.Label(logger_frame, text="Slope:", font=CHANNEL_FONT)
trigger_slope_label.grid(row=row_counter, column=0, sticky="w", padx=4, pady=4)

trigger_slope_combo = ttk.Combobox(
    logger_frame,
    textvariable=trigger_slope_var,
    values=list(TRIGGER_SLOPES),
    state="readonly",
    width=14,
)
trigger_slope_combo.grid(row=row_counter, column=1, sticky="w", padx=4, pady=4)
row_counter += 1

trigger_position_label = ttk.Label(
    logger_frame, text="Trigger Position:", font=CHANNEL_FONT
)
trigger_position_label.grid(row=row_counter, column=0, sticky="w", padx=4, pady=4)

trigger_position_combo = ttk.Combobox(
    logger_frame,
    textvariable=trigger_position_var,
    values=list(TRIGGER_POSITIONS),
    state="readonly",
    width=14,
)
trigger_position_combo.grid(row=row_counter, column=1, sticky="w", padx=4, pady=4)
row_counter += 1

status_label = ttk.Label(logger_frame, textvariable=status_var, font=CHANNEL_FONT)
status_label.grid(row=row_counter, column=0, columnspan=3, sticky="w", padx=4, pady=6)
row_counter += 1

button_frame = ttk.Frame(logger_frame)
button_frame.grid(row=row_counter, column=0, columnspan=3, sticky="w", padx=4, pady=6)

captured_data: dict[int, list[float]] | None = None
timestamps: list[float] | None = None
is_capturing = False
update_job: str | None = None

arm_button = tk.Button(button_frame, text="Arm Trigger", font=BUTTON_FONT)
arm_button.grid(row=0, column=0, padx=4)
capture_button = tk.Button(button_frame, text="Capture Now", font=BUTTON_FONT)
capture_button.grid(row=0, column=1, padx=4)
export_button = tk.Button(button_frame, text="Export CSV", font=BUTTON_FONT)
export_button.grid(row=0, column=2, padx=4)

log_fig, log_ax = plt.subplots(figsize=(9, 3), dpi=100)
log_canvas = FigureCanvasTkAgg(log_fig, master=logger_frame)
log_canvas.get_tk_widget().grid(row=row_counter + 1, column=0, columnspan=3, sticky="nsew", padx=4, pady=6)

stats_frame = ttk.Frame(logger_frame)
stats_frame.grid(row=row_counter + 2, column=0, columnspan=3, sticky="new", padx=4, pady=(0, 8))
stats_frame.columnconfigure(0, weight=1)
# ---------------- Helper functions ----------------

def compute_dmm_value(ch_index: int, volts: float) -> tuple[float | None, str]:
    try:
        if ch_index == 5:
            scaled = volts * -68.36437
        elif ch_index == 3:
            if dmm_range_var.get():
                denom = 5.001 - volts
                if abs(denom) < 1e-9:
                    return None, "OVER"
                scaled = 22000.0 * (volts / denom)
            else:
                denom = 0.02016 - (volts / 330.0)
                if abs(denom) < 1e-9:
                    return None, "OVER"
                scaled = volts / denom
        else:
            return volts, f"{volts:.6f}"
    except ZeroDivisionError:
        return None, "OVER"

    if math.isnan(scaled) or math.isinf(scaled) or abs(scaled) > 10_000_000:
        return None, "OVER"
    return scaled, f"{scaled:.6f}"


def compute_sample_value(ch_index: int, raw_value: int) -> tuple[float | None, str, str, float]:
    volts = raw_to_volts(raw_value)
    units = "V"
    if dmm_math_var.get():
        value, text = compute_dmm_value(ch_index, volts)
        units = "Ω" if ch_index == 3 else "Calc"
        if text == "OVER":
            return None, text, units, volts
        return value, text, units, volts
    if raw_var.get():
        return float(raw_value), str(raw_value), "Raw", volts
    return volts, f"{volts:.6f}", units, volts


def determine_trigger_units(ch_index: int) -> str:
    if dmm_math_var.get():
        return "Ω" if ch_index == 3 else "Calc"
    if raw_var.get():
        return "Raw"
    return "V"


def update_trigger_units(*_: object) -> None:
    idx = CHANNEL_LOOKUP.get(trigger_channel_var.get(), 0)
    trigger_units_label.config(text=f"Units: {determine_trigger_units(idx)}")


def compute_trigger_value(ch_index: int, raw_value: int) -> float | None:
    volts = raw_to_volts(raw_value)
    if dmm_math_var.get():
        value, text = compute_dmm_value(ch_index, volts)
        if text == "OVER":
            return None
        return value
    if raw_var.get():
        return float(raw_value)
    return volts


def compute_capture_values(ch_index: int, raw_value: int) -> tuple[float, float | None]:
    volts = raw_to_volts(raw_value)
    if dmm_math_var.get():
        value, text = compute_dmm_value(ch_index, volts)
        if text == "OVER":
            return float("nan"), None
        return value, value
    if raw_var.get():
        val = float(raw_value)
        return val, val
    return volts, volts


def update_graph() -> None:
    if not live_graph_var.get():
        return
    ax.clear()
    ax.set_title("Live Channel Data", fontsize=16)
    ax.set_xlabel("Samples")
    if dmm_math_var.get():
        ax.set_ylabel("DMM Value")
    elif raw_var.get():
        ax.set_ylabel("Raw")
    else:
        ax.set_ylabel("Volts")

    for i, (label_txt, _, _) in enumerate(CHANNEL_MAP):
        if not channel_vars[i].get():
            continue
        y = []
        for value in data_buffers[i]:
            if value is None or (isinstance(value, float) and math.isnan(value)):
                y.append(float("nan"))
            else:
                y.append(value)
        if not y:
            continue
        x = list(range(len(y)))
        ax.plot(x, y, label=label_txt)

    if ax.has_data():
        ax.legend(loc="upper right")

    canvas.draw_idle()


def update_values() -> None:
    global update_job
    if is_capturing:
        update_job = root.after(500, update_values)
        return

    readings_enabled = live_values_var.get()
    graph_enabled = live_graph_var.get()

    for i in range(len(CHANNEL_MAP)):
        if channel_vars[i].get():
            if readings_enabled:
                raw_value = read_channel_raw(i)
                value, text, units, _ = compute_sample_value(i, raw_value)
                value_labels[i].config(text=text)
                unit_labels[i].config(text=units)
                channel_units[i] = units
                buffer_value = float('nan') if value is None else value
            else:
                if value_labels[i].cget('text') != 'PAUSED':
                    value_labels[i].config(text='PAUSED')
                buffer_value = float('nan')
            data_buffers[i].append(buffer_value)
        else:
            value_labels[i].config(text='OFF')
            unit_labels[i].config(text='')
            data_buffers[i].append(float('nan'))

    if graph_enabled:
        update_graph()

    update_job = root.after(500, update_values)


def wait_for_trigger(ch_index: int, threshold: float, slope: str) -> None:
    status_var.set(f"Waiting for {slope.lower()} trigger on {CHANNEL_MAP[ch_index][0]}")
    prev_value: float | None = None
    set_channel(*CHANNEL_MAP[ch_index][1])
    while True:
        send_cmd(CMD_SYNC)
        send_cmd(CMD_WAKEUP)
        wait_drdy()
        raw_value = read_data()
        current_value = compute_trigger_value(ch_index, raw_value)
        if current_value is None:
            prev_value = current_value
            continue
        if prev_value is None:
            prev_value = current_value
            continue
        if slope == "Rising":
            if prev_value < threshold <= current_value:
                return
        else:
            if prev_value > threshold >= current_value:
                return
        prev_value = current_value


def run_benchmark_capture(
    n_points: int,
    ch_index: int,
    threshold: float,
    slope: str,
    armed: bool,
    position: str,
) -> tuple[dict[int, list[float]], list[float]]:
    set_channel(*CHANNEL_MAP[ch_index][1])

    if not armed or position == "Start":
        if armed:
            wait_for_trigger(ch_index, threshold, slope)
            status_var.set("Trigger detected - benchmarking...")
        else:
            status_var.set("Benchmark capture in progress...")

        send_cmd(CMD_RDATAC)
        wait_drdy_fast()

        gc_was_enabled = gc.isenabled()
        if gc_was_enabled:
            gc.disable()

        captured = {ch_index: [float("nan")] * n_points}
        times = [0.0] * n_points
        start = time.time()
        try:
            for idx in range(n_points):
                wait_drdy_fast()
                raw = read_data_raw_fast()
                sample_value, _ = compute_capture_values(ch_index, raw)
                captured[ch_index][idx] = sample_value
                times[idx] = (time.time() - start) * 1000.0
        finally:
            if gc_was_enabled:
                gc.enable()
            send_cmd(CMD_SDATAC)
        return captured, times

    status_var.set(
        f"Buffering for {position.lower()} trigger on {CHANNEL_MAP[ch_index][0]}"
    )
    send_cmd(CMD_RDATAC)
    wait_drdy_fast()

    gc_was_enabled = gc.isenabled()
    if gc_was_enabled:
        gc.disable()

    try:
        if position == "End":
            buffer: deque[tuple[float, float]] = deque(maxlen=n_points)
            prev_value: float | None = None
            while True:
                wait_drdy_fast()
                raw = read_data_raw_fast()
                sample_value, trigger_value = compute_capture_values(ch_index, raw)
                sample_ts = time.time()
                buffer.append((sample_value, sample_ts))
                if trigger_value is None:
                    prev_value = trigger_value
                    continue
                if prev_value is None:
                    prev_value = trigger_value
                    continue
                if len(buffer) >= n_points:
                    if slope == "Rising":
                        triggered = prev_value < threshold <= trigger_value
                    else:
                        triggered = prev_value > threshold >= trigger_value
                    if triggered:
                        break
                prev_value = trigger_value
            samples = list(buffer)
            status_var.set("Trigger detected - finalizing benchmark capture...")
        else:
            pre_count = n_points // 2
            if pre_count > 0:
                pre_buffer: deque[tuple[float, float]] = deque(maxlen=pre_count)
            else:
                pre_buffer = deque()
            prev_value: float | None = None
            trigger_sample: tuple[float, float] | None = None
            while True:
                wait_drdy_fast()
                raw = read_data_raw_fast()
                sample_value, trigger_value = compute_capture_values(ch_index, raw)
                sample_ts = time.time()
                can_trigger = pre_count == 0 or len(pre_buffer) >= pre_count
                triggered = False
                if (
                    trigger_value is not None
                    and prev_value is not None
                    and can_trigger
                ):
                    if slope == "Rising":
                        triggered = prev_value < threshold <= trigger_value
                    else:
                        triggered = prev_value > threshold >= trigger_value
                if triggered:
                    trigger_sample = (sample_value, sample_ts)
                    break
                if pre_count > 0:
                    pre_buffer.append((sample_value, sample_ts))
                prev_value = trigger_value

            status_var.set(
                "Trigger detected - capturing remaining benchmark samples..."
            )
            post_count = n_points - (len(pre_buffer) if pre_count > 0 else 0)
            samples = list(pre_buffer)
            post_samples: list[tuple[float, float]] = []
            if trigger_sample is not None:
                post_samples.append(trigger_sample)
            while len(post_samples) < post_count:
                wait_drdy_fast()
                raw = read_data_raw_fast()
                sample_value, _ = compute_capture_values(ch_index, raw)
                sample_ts = time.time()
                post_samples.append((sample_value, sample_ts))
            samples.extend(post_samples)
    finally:
        if gc_was_enabled:
            gc.enable()
        send_cmd(CMD_SDATAC)

    captured = {ch_index: [value for value, _ in samples]}
    if samples:
        first_time = samples[0][1]
        times = [(ts - first_time) * 1000.0 for _, ts in samples]
    else:
        times = []
    return captured, times


def run_multi_capture(
    n_points: int,
    active_channels: list[int],
    trigger_index: int,
    threshold: float,
    slope: str,
    armed: bool,
    position: str,
) -> tuple[dict[int, list[float]], list[float]]:
    if not armed or position == "Start":
        if armed:
            wait_for_trigger(trigger_index, threshold, slope)
            status_var.set("Trigger detected - capturing multi-channel data...")
        else:
            status_var.set("Capturing multi-channel data...")

        captured = {idx: [] for idx in active_channels}
        times: list[float] = []
        start_time: float | None = None

        for _sample_idx in range(n_points):
            sample_timestamp = time.time()
            for idx in active_channels:
                raw = read_channel_raw(idx)
                sample_value, _ = compute_capture_values(idx, raw)
                captured[idx].append(sample_value)
            if start_time is None:
                start_time = sample_timestamp
            times.append((sample_timestamp - start_time) * 1000.0)

        return captured, times

    status_var.set(
        f"Buffering for {position.lower()} trigger on {CHANNEL_MAP[trigger_index][0]}"
    )

    def read_frame() -> tuple[dict[int, float], float | None, float]:
        sample_timestamp = time.time()
        frame: dict[int, float] = {}
        trigger_value: float | None = None
        for idx in active_channels:
            raw_val = read_channel_raw(idx)
            sample_value, trig_value = compute_capture_values(idx, raw_val)
            frame[idx] = sample_value
            if idx == trigger_index:
                trigger_value = trig_value
        return frame, trigger_value, sample_timestamp

    if position == "End":
        buffer: deque[tuple[dict[int, float], float]] = deque(maxlen=n_points)
        prev_value: float | None = None
        while True:
            frame, trig_value, ts = read_frame()
            buffer.append((frame, ts))
            if trig_value is None:
                prev_value = trig_value
                continue
            if prev_value is None:
                prev_value = trig_value
                continue
            if len(buffer) >= n_points:
                if slope == "Rising":
                    triggered = prev_value < threshold <= trig_value
                else:
                    triggered = prev_value > threshold >= trig_value
                if triggered:
                    break
            prev_value = trig_value
        frames = list(buffer)
        status_var.set("Trigger detected - finalizing multi-channel capture...")
    else:
        pre_count = n_points // 2
        if pre_count > 0:
            pre_buffer: deque[tuple[dict[int, float], float]] = deque(maxlen=pre_count)
        else:
            pre_buffer = deque()
        prev_value: float | None = None
        trigger_frame: tuple[dict[int, float], float] | None = None
        while True:
            frame, trig_value, ts = read_frame()
            can_trigger = pre_count == 0 or len(pre_buffer) >= pre_count
            triggered = False
            if (
                trig_value is not None
                and prev_value is not None
                and can_trigger
            ):
                if slope == "Rising":
                    triggered = prev_value < threshold <= trig_value
                else:
                    triggered = prev_value > threshold >= trig_value
            if triggered:
                trigger_frame = (frame, ts)
                break
            if pre_count > 0:
                pre_buffer.append((frame, ts))
            prev_value = trig_value

        status_var.set(
            "Trigger detected - capturing remaining multi-channel samples..."
        )
        post_count = n_points - (len(pre_buffer) if pre_count > 0 else 0)
        frames = list(pre_buffer)
        post_frames: list[tuple[dict[int, float], float]] = []
        if trigger_frame is not None:
            post_frames.append(trigger_frame)
        while len(post_frames) < post_count:
            frame, _, ts = read_frame()
            post_frames.append((frame, ts))
        frames.extend(post_frames)

    captured = {idx: [] for idx in active_channels}
    times: list[float] = []
    if frames:
        first_time = frames[0][1]
    else:
        first_time = None
    for frame, ts in frames:
        for idx in active_channels:
            captured[idx].append(frame.get(idx, float("nan")))
        if first_time is None:
            first_time = ts
            times.append(0.0)
        else:
            times.append((ts - first_time) * 1000.0)

    return captured, times


def update_logger_plot() -> None:
    log_ax.clear()
    log_ax.set_title(f"{trigger_mode_var.get()} capture", fontsize=14)
    first_channel = next(iter(captured_data.keys())) if captured_data else None
    if timestamps and captured_data and len(next(iter(captured_data.values()))) == len(timestamps):
        x = timestamps
        log_ax.set_xlabel("Time (ms)")
    else:
        length = len(next(iter(captured_data.values()))) if captured_data else 0
        x = list(range(length))
        log_ax.set_xlabel("Samples")

    if dmm_math_var.get():
        log_ax.set_ylabel("DMM Value")
    elif raw_var.get():
        log_ax.set_ylabel("Raw")
    else:
        log_ax.set_ylabel("Volts")

    if captured_data:
        for idx, values in captured_data.items():
            y = [float("nan") if (v is None or math.isnan(v)) else v for v in values]
            log_ax.plot(x, y, label=CHANNEL_MAP[idx][0])
        if log_ax.has_data():
            log_ax.legend(loc="upper right")

    log_canvas.draw_idle()


def update_stats() -> None:
    for widget in stats_frame.winfo_children():
        widget.destroy()
    if not captured_data:
        return

    for idx, values in captured_data.items():
        valid = [v for v in values if not math.isnan(v)]
        unit = determine_trigger_units(idx)
        if not valid:
            summary = "No valid samples"
        else:
            summary = (
                f"Min: {min(valid):.6f}\n"
                f"Max: {max(valid):.6f}\n"
                f"Avg: {statistics.mean(valid):.6f}\n"
                f"Std: {statistics.pstdev(valid):.6f}"
            )
        text = f"{CHANNEL_MAP[idx][0]} ({unit})\n{summary}"
        ttk.Label(stats_frame, text=text, font=CHANNEL_FONT, anchor="w", justify="left").pack(
            anchor="w", pady=2
        )


def export_csv() -> None:
    if not captured_data:
        status_var.set("No capture data to export")
        return
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = filedialog.asksaveasfilename(
        defaultextension=".csv",
        initialfile=f"ads1256_log_{ts}.csv",
        filetypes=[("CSV files", "*.csv")],
    )
    if not fname:
        return

    num_samples = len(next(iter(captured_data.values())))
    with open(fname, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["Time_ms"] + [CHANNEL_MAP[idx][0] for idx in captured_data.keys()]
        writer.writerow(header)
        for sample_idx in range(num_samples):
            if timestamps and len(timestamps) == num_samples:
                row = [f"{timestamps[sample_idx]:.7g}"]
            else:
                row = [sample_idx]
            for idx in captured_data.keys():
                value = captured_data[idx][sample_idx]
                if value is None or math.isnan(value):
                    row.append("OVER")
                else:
                    row.append(f"{value:.7g}")
            writer.writerow(row)

    status_var.set(f"Exported {num_samples} samples to CSV")

def perform_capture(armed: bool) -> None:
    global captured_data, timestamps, is_capturing
    if is_capturing:
        return

    try:
        n_points = int(points_var.get())
        if n_points <= 0:
            raise ValueError
    except ValueError:
        status_var.set("Invalid point count")
        return

    try:
        threshold = float(trigger_level_var.get())
    except ValueError:
        if armed:
            status_var.set("Invalid trigger level")
            return
        threshold = 0.0

    trigger_label = trigger_channel_var.get()
    trigger_index = CHANNEL_LOOKUP.get(trigger_label, 0)
    active_channels = [i for i, var in enumerate(channel_vars) if var.get()]
    if not active_channels:
        status_var.set("Select at least one channel")
        return

    mode = trigger_mode_var.get()
    slope = trigger_slope_var.get()
    position = trigger_position_var.get()

    if mode == "Benchmark":
        active_channels = [trigger_index]
    elif trigger_index not in active_channels:
        active_channels.insert(0, trigger_index)

    is_capturing = True
    arm_button.config(state="disabled")
    capture_button.config(state="disabled")
    export_button.config(state="disabled")
    status_var.set("Awaiting trigger..." if armed else "Capturing now...")
    root.update_idletasks()

    try:
        if mode == "Benchmark":
            capture, times = run_benchmark_capture(
                n_points, trigger_index, threshold, slope, armed, position
            )
        else:
            capture, times = run_multi_capture(
                n_points,
                active_channels,
                trigger_index,
                threshold,
                slope,
                armed,
                position,
            )
        captured_data = capture
        timestamps = times
        update_logger_plot()
        update_stats()
        sample_count = len(next(iter(captured_data.values()), []))
        status_var.set(f"Capture complete ({sample_count} samples)")
    except Exception as exc:
        captured_data = None
        timestamps = None
        status_var.set(f"Capture failed: {exc}")
    finally:
        is_capturing = False
        arm_button.config(state="normal")
        capture_button.config(state="normal")
        export_button.config(state="normal")


def on_close() -> None:
    global update_job
    if update_job is not None:
        try:
            root.after_cancel(update_job)
        except Exception:
            pass
        update_job = None
    root.destroy()


arm_button.config(command=lambda: perform_capture(True))
capture_button.config(command=lambda: perform_capture(False))
export_button.config(command=export_csv)
trigger_channel_var.trace_add("write", update_trigger_units)

update_trigger_units()
update_job = root.after(200, update_values)
root.protocol("WM_DELETE_WINDOW", on_close)

try:
    root.mainloop()
finally:
    try:
        spi.close()
    except Exception:
        pass
    try:
        lgpio.gpiochip_close(h)
    except Exception:
        pass
