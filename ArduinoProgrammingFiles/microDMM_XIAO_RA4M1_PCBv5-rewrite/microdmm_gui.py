#!/usr/bin/env python3
"""
microdmm_gui.py  --  Host GUI for the microDMM PCBv5 firmware.

Talks to the meter (Seeed XIAO RA4M1) over USB serial at 115200 baud, using
the !CMD / $LINE protocol in SerialCmd.ino.

Four tabs:

  Live
    The reason this GUI exists.  One large primary readout that follows
    whatever the meter has auto-selected, a row of secondary values, state
    lamps, running min/max, and a rolling strip chart of any single trace.

  Config
    Every EEPROM key, read from the device's own !CFG dump.  Edit and push to
    RAM with !SET (applied immediately), persist with !SAVE.  Keys this script
    has never heard of still appear, under "Other" -- adding a firmware key
    does not require editing this file.

  Calibration
    The 15-bucket piecewise resistance ladder, plus voltage scale and current
    zero.  Put a known reference across the leads, type its value, press
    Capture; the firmware works out the correction factor for whichever bucket
    the present raw reading falls in and reports it back.

  Log
    Arm the on-board 120-sample current/voltage log, dump it, plot it, save
    it as CSV.

Dependencies:
    pip install pyserial matplotlib

Run:
    py microdmm_gui.py
"""

import csv
import os
import queue
import threading
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import filedialog, messagebox, simpledialog, ttk

import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

BAUD = 115200
LIVE_MAXLEN = 3000          # samples kept in the rolling live plot
PLOT_REFRESH_MS = 100       # chart redraw cadence
UI_TICK_MS = 40             # queue drain cadence

UNITS_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "microdmm_units.csv")
UNITS_CSV_FIXED = ["SN", "timestamp"]

# HWREV says which physical unit this is -- it selects the legacy calibration
# set and decides whether the float-detect bridge exists.  BRIDGE is the same
# kind of statement.  Neither is a preference, so neither may be cloned from
# one meter onto another.
NEVER_COPY_KEYS = {"HWREV", "BRIDGE"}

MODE_NAMES = ["Default", "Voltmeter", "VAC manual", "Type", "Precise",
              "Alt units", "High R", "Charging"]

# $LIVE flag bits -- must match the header comment in SerialCmd.ino.
FLAG_BITS = [
    ("voltageDisplay", 0), ("powerSave", 1), ("VAC present", 2),
    ("floating", 3), ("bridge armed", 4), ("continuity", 5),
    ("high range", 6), ("auto range", 7), ("min/max", 8),
    ("screen asleep", 9), ("precise", 10), ("alt units", 11),
    ("amps mode", 12), ("R open", 13), ("current sensor", 14),
    ("unsaved config", 15), ("R measured", 16), ("ADC continuous", 17),
]

# Lamps shown on the Live tab, in order, with the colour they light.
LAMPS = [
    ("CONT", 5, "#1faa3f"), ("VAC", 2, "#d68000"), ("FLOAT", 3, "#0b6fb8"),
    ("OPEN", 13, "#7a7a7a"), ("PWRSAVE", 1, "#8a4bbd"), ("ASLEEP", 9, "#555555"),
    ("AUTO", 7, "#1faa3f"), ("HIGH R", 6, "#0b6fb8"), ("I SENSOR", 14, "#1faa3f"),
    ("CONT ADC", 17, "#0b6fb8"), ("DIRTY", 15, "#c02020"),
]

# Detection is a boot decision: if no ammeter was found the channel stays
# suppressed until the meter is power-cycled.  Bit 14 says which.
BIT_I_SENSOR = 14
BIT_AMPS_MODE = 12
# Voltmeter mode and logging skip the ohms channel entirely, so the resistance
# fields in $LIVE hold whatever they last read.  Bit 16 says whether this
# record's resistance is a live measurement or a leftover.
BIT_R_MEASURED = 16
BIT_CONTINUOUS = 17

# Traces sourced from the ohms channel, which bit 16 can invalidate.
R_TRACES = {"Resistance (ohm)", "Resistance nulled", "Ohms rail (V)"}

# Upper edge of each RCAL bucket -- mirrors R_CAL_EDGES in Config.ino.  Used
# only to label the calibration rows and to show which bucket is live.
R_CAL_EDGES = [0.75, 3.0, 7.0, 20.0, 70.0, 170.0, 700.0, 1700.0,
               7000.0, 17000.0, 70000.0, 170000.0, 700000.0, 1700000.0]

# Traces the strip chart can show: label -> ($LIVE field index, unit).
TRACES = {
    "Resistance (ohm)":      (1, "ohm"),
    "Resistance nulled":     (2, "ohm"),
    "Voltage (V)":           (3, "V"),
    "Voltage average (V)":   (4, "V"),
    "VAC rms (V)":           (5, "V"),
    "Current (A)":           (6, "A"),
    "Ohms rail (V)":         (7, "V"),
    "Supply (V)":            (8, "V"),
}

# ---------------------------------------------------------------------------
# Config-key metadata: (group, editor kind, description).
# The table is built from the device's own !CFG dump, so a key absent here
# still appears (under "Other").  This only adds grouping and prose.
# ---------------------------------------------------------------------------
KEY_META = {
    "HWREV":     ("Unit", "num", "Legacy unit id; picks the !SEEDCAL set. Never copy between meters."),
    "BRIDGE":    ("Unit", "bool", "Board has the float-detect bridge MOSFET (rev 5+)."),

    "CONSTI":    ("Resistance calibration", "num", "Constant-current source, amps."),
    "CONSTR":    ("Resistance calibration", "num", "Internal resistor in that source, ohms."),
    "DIVR":      ("Resistance calibration", "num", "High-range series divider, ohms."),
    "ZENERMAX":  ("Resistance calibration", "num", "Reference ceiling, volts."),
    "SLEEPV":    ("Resistance calibration", "num", "Ohms rail once power save parks the source."),

    "VSCALE":    ("Voltage calibration", "num", "Front-end divider ratio. Signed: units 5/6 invert."),
    "ALTMULT":   ("Voltage calibration", "num", "Multiplier applied in Alt units mode."),
    "THERMR0":   ("Voltage calibration", "num", "Thermistor nominal resistance at 25 C."),
    "THERMB":    ("Voltage calibration", "num", "Thermistor beta."),

    "ISHUNT":    ("Current", "num", "Hall sensor scale, volts per amp (high range)."),
    "ISHUNTR":   ("Current", "num", "Sense resistor, ohms (low range / shunt)."),
    "IZERO":     ("Current", "num", "Zero-current baseline (set by !CALI or at boot)."),
    "IAUTOZERO": ("Current", "bool", "Re-detect the baseline at every boot."),
    "INOISEHI":  ("Current", "num", "High-range deadband, amps."),
    "INOISELO":  ("Current", "num", "Low-range deadband, amps."),
    "IDETCNT":   ("Current", "num", "Detect: |counts| below this reads as a grounded shunt."),
    "IDETLO":    ("Current", "num", "Detect: mid-rail window low edge, volts."),
    "IDETHI":    ("Current", "num", "Detect: mid-rail window high edge, volts."),
    "IDETSAMP":  ("Current", "num", "Detect: reads averaged."),
    "IDETSETTLE":("Current", "num", "Detect: settling delay before sampling, ms."),
    "IDETPP":    ("Current", "num", "Detect: max peak-to-peak volts for a driven input. 0 disables."),

    "RANGETHR":  ("Ranging", "num", "High/low resistance crossover, ohms."),
    "RANGEDB":   ("Ranging", "num", "Hysteresis around it, as a fraction."),
    "ADCLOW":    ("Ranging", "num", "Step gain up below this ADC count."),
    "ADCHIGH":   ("Ranging", "num", "Step gain down above this ADC count."),
    "OPENMARGIN":("Ranging", "num", "Volts below the reference that still reads open."),
    "OPENR":     ("Ranging", "num", "Ohms reported for an open circuit."),
    "VDISPCNT":  ("Ranging", "num", "Differential counts that auto-select voltage."),
    "RDISPMIN":  ("Ranging", "num", "Ohms window that auto-selects resistance, low edge."),
    "RDISPMAX":  ("Ranging", "num", "...high edge."),
    "ZEROAUTOMAX":("Ranging", "num", "Largest boot reading accepted as a lead null."),

    "RATEJUMP":  ("ADC scheduling", "num", "Ratio outside which a reading counts as moving."),
    "RATEBUMPV": ("ADC scheduling", "num", "Ohms rail above which a jump is ignored."),
    "RATESLOWV": ("ADC scheduling", "num", "Volts below the reference permitting the medium rate."),
    "RATEPRECR": ("ADC scheduling", "num", "Ohms ceiling for the slow precise rate."),
    "RATESTABLE":("ADC scheduling", "num", "Fraction within which a reading counts as settled."),
    "MMRMAX":    ("ADC scheduling", "num", "Upper bound on resistance min/max tracking."),
    "MMRMIN":    ("ADC scheduling", "num", "Lower bound on it."),

    "PSHOLDMS":  ("Power save", "num", "Milliseconds pegged at the rail before parking."),
    "PSMARGIN":  ("Power save", "num", "Volts below the reference that counts as pegged."),
    "PSHYST":    ("Power save", "num", "Volts below SLEEPV that releases power save."),
    "PSCANCELR": ("Power save", "num", "Reading that cancels a pending power save, ohms."),
    "PSPWM":     ("Power save", "num", "Ohms-pin PWM duty while parked."),
    "SLEEPSEC":  ("Power save", "num", "Seconds of quiet before the screen blanks."),
    "SLEEPVMAX": ("Power save", "num", "Volts under which the meter counts as quiet."),
    "PSDEBUG":   ("Power save", "bool", "Emit $PS transitions on serial."),

    "ADCMS":     ("Timing", "num", "Measurement period, ms."),
    "BATTMS":    ("Timing", "num", "Battery read period, ms."),
    "LCDMS":     ("Timing", "num", "Screen refresh period, ms."),
    "LCDFASTMS": ("Timing", "num", "...in fast mode."),
    "LCDBUMPMS": ("Timing", "num", "Temporary period after a reading jumps."),
    "STREAMMS":  ("Timing", "num", "$LIVE cadence, ms."),

    "ALERTS":    ("Alerts", "bool", "Master enable for buzzer and alert LED."),
    "CONTMIN":   ("Alerts", "num", "Continuity window low edge, ohms."),
    "CONTMAX":   ("Alerts", "num", "Continuity window high edge, low range."),
    "CONTMAXHI": ("Alerts", "num", "Continuity window high edge, high range."),
    "VALERT":    ("Alerts", "num", "Volts that raise the voltage warning."),
    "VALERTALT": ("Alerts", "num", "...in Alt units mode."),
    "VACALERT":  ("Alerts", "num", "VAC rms that raises the warning."),
    "VACALERTALT":("Alerts", "num", "...in Alt units mode."),
    "BEEPBR":    ("Alerts", "num", "First-pulse PWM, 0-255."),
    "BEEPHOLD":  ("Alerts", "num", "Sustain PWM, 0-255."),
    "BLINKLIM":  ("Alerts", "num", "Pulses allowed between screen refreshes."),
    "ALERTPERMS":("Alerts", "num", "Repeat window, ms."),
    "ALERTONMS": ("Alerts", "num", "First pulse length within the window, ms."),
    "ALERTP2ON": ("Alerts", "num", "Second pulse start, ms (continuity only)."),
    "ALERTP2OFF":("Alerts", "num", "Second pulse end, ms."),

    "VACTHRESH": ("AC detection", "num", "VAC rms that declares AC present."),
    "VACTHRALT": ("AC detection", "num", "...in Alt units mode."),
    "VACAVGMAX": ("AC detection", "num", "DC average must be under this to declare AC."),

    "BRIDGETHR": ("Bridge test", "num", "Volts below which the leads read floating."),
    "BRIDGEFLT": ("Bridge test", "num", "Split between floating and unsure."),
    "BRIDGEAVG": ("Bridge test", "num", "Gate: DC average must be under this."),
    "BRIDGEVMAX":("Bridge test", "num", "Gate: instantaneous reading under this. Not run in VAC mode."),

    "VSAMPLES":  ("Filtering", "num", "Rolling voltage window length (max 100)."),
    "SMOOTHA":   ("Filtering", "num", "Display smoothing coefficient, 0-1."),
    "BATTSCALE": ("Filtering", "num", "Multiplier on the battery divider."),
    "KEYBOARD":  ("Filtering", "bool", "USB HID typing in Type / High R modes."),
    "BTNLONGMS": ("Filtering", "num", "Hold that resets min/max, ms."),
    "BTNSHORTMS":("Filtering", "num", "Debounce floor for a short press, ms."),
}
for _i in range(15):
    _lo = "0" if _i == 0 else f"{R_CAL_EDGES[_i-1]:g}"
    _hi = "inf" if _i == 14 else f"{R_CAL_EDGES[_i]:g}"
    KEY_META[f"RCAL{_i:02d}"] = ("Resistance ladder", "num",
                                 f"Correction factor for {_lo} to {_hi} ohms.")

GROUP_ORDER = ["Unit", "Resistance calibration", "Resistance ladder",
               "Voltage calibration", "Current", "Ranging", "ADC scheduling",
               "Power save", "Timing", "Alerts", "AC detection",
               "Bridge test", "Filtering", "Other"]


def eng(value, unit, digits=4):
    """Format a value with an SI prefix, e.g. 1234.5 ohm -> '1.2345 k'."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return "--"
    if v != v:                       # NaN
        return "--"
    a = abs(v)
    if a >= 1e6:
        return f"{v/1e6:.{digits}g} M{unit}"
    if a >= 1e3:
        return f"{v/1e3:.{digits}g} k{unit}"
    if a >= 1.0 or a == 0.0:
        return f"{v:.{digits}g} {unit}"
    if a >= 1e-3:
        return f"{v*1e3:.{digits}g} m{unit}"
    return f"{v*1e6:.{digits}g} u{unit}"


class SerialManager:
    """Background serial reader. Pushes decoded lines onto a queue."""

    def __init__(self, line_queue):
        self.line_queue = line_queue
        self.ser = None
        self._stop = threading.Event()
        self._thread = None

    def connect(self, port):
        self.disconnect()
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    @property
    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def send(self, text):
        if self.is_open:
            if not text.endswith("\n"):
                text += "\n"
            self.ser.write(text.encode("ascii", errors="ignore"))

    def _read_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self.ser.read(256)
            except Exception as exc:
                self.line_queue.put(("__error__", str(exc)))
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", errors="replace").strip()
                if line:
                    self.line_queue.put(("line", line))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("microDMM -- Live, Configuration & Calibration")
        self.geometry("1180x860")

        self.line_queue = queue.Queue()
        self.ser = SerialManager(self.line_queue)

        # --- live state ---
        self.live = None                  # last parsed $LIVE field list
        self.flags = 0
        self.mode = 0
        self.unit_sn = ""
        self.sn_prompted = False
        self.dirty = False
        self.hwrev = ""
        self.t0 = None
        self.trace_t = deque(maxlen=LIVE_MAXLEN)
        self.trace_y = deque(maxlen=LIVE_MAXLEN)
        self.minmax = None

        # --- config table state ---
        self.cfg_rows = {}
        self.cfg_groups_done = set()
        self.cfg_next_row = 0
        self.cfg_placeholder = None
        self.cfg_snapshot = {}            # last full device dump
        self.cfg_collecting = False
        self.pending_csv_log = False

        # --- log capture state ---
        self.log_rows = []
        self.log_collecting = False

        self._build_ui()
        self.after(UI_TICK_MS, self._tick)
        self.after(PLOT_REFRESH_MS, self._redraw_live)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ==================================================================
    #  UI
    # ==================================================================
    def _build_ui(self):
        top = ttk.Frame(self)
        top.pack(fill="x", padx=8, pady=6)

        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=22)
        self.port_cb.pack(side="left", padx=(4, 4))
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=6)

        self.mode_lbl = ttk.Label(top, text="mode --", font=("Segoe UI", 10, "bold"))
        self.mode_lbl.pack(side="left", padx=(18, 0))
        self.sn_lbl = ttk.Label(top, text="SN --", foreground="#555")
        self.sn_lbl.pack(side="left", padx=(18, 0))
        self.dirty_lbl = ttk.Label(top, text="", foreground="#c02020",
                                   font=("Segoe UI", 10, "bold"))
        self.dirty_lbl.pack(side="right")

        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=8, pady=(0, 4))
        self._build_live_tab()
        self._build_cfg_tab()
        self._build_cal_tab()
        self._build_log_tab()

        self.log_text = tk.Text(self, height=7, font=("Consolas", 9), wrap="none")
        self.log_text.pack(fill="x", padx=8, pady=(0, 8))

        self._refresh_ports()

    # ---------------- Live tab ----------------
    def _build_live_tab(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="Live")

        # Primary readout.  Deliberately the largest thing on screen -- this is
        # a meter, and the number is the point.
        head = ttk.Frame(tab)
        head.pack(fill="x", pady=(10, 4))
        self.primary_lbl = tk.Label(head, text="--", font=("Consolas", 52, "bold"),
                                    fg="#0b6fb8")
        self.primary_lbl.pack(side="left", padx=(16, 10))
        self.primary_cap = tk.Label(head, text="", font=("Segoe UI", 12), fg="#555")
        self.primary_cap.pack(side="left", anchor="s", pady=(0, 14))

        # Secondary values.
        sec = ttk.Frame(tab)
        sec.pack(fill="x", padx=16)
        self.sec_lbls = {}
        for i, name in enumerate(["Resistance", "Nulled", "Voltage", "Average",
                                  "VAC rms", "Current", "Ohms rail", "Supply"]):
            col = ttk.Frame(sec)
            col.grid(row=0, column=i, padx=(0, 16), sticky="w")
            ttk.Label(col, text=name, foreground="#777",
                      font=("Segoe UI", 8)).pack(anchor="w")
            lbl = tk.Label(col, text="--", font=("Consolas", 12))
            lbl.pack(anchor="w")
            self.sec_lbls[name] = lbl

        # Lamps.
        lamp_row = ttk.Frame(tab)
        lamp_row.pack(fill="x", padx=16, pady=(10, 4))
        self.lamps = {}
        for name, bit, colour in LAMPS:
            lbl = tk.Label(lamp_row, text=name, font=("Segoe UI", 8, "bold"),
                           width=11, relief="ridge", bg="#eeeeee", fg="#aaaaaa")
            lbl.pack(side="left", padx=2, ipady=3)
            self.lamps[name] = (lbl, bit, colour)

        # Chart.
        ctrl = ttk.Frame(tab)
        ctrl.pack(fill="x", padx=16, pady=(8, 2))
        ttk.Label(ctrl, text="Trace").pack(side="left")
        self.trace_var = tk.StringVar(value="Voltage (V)")
        cb = ttk.Combobox(ctrl, textvariable=self.trace_var, width=22,
                          values=list(TRACES.keys()), state="readonly")
        cb.pack(side="left", padx=(4, 12))
        cb.bind("<<ComboboxSelected>>", lambda e: self._clear_trace())
        ttk.Label(ctrl, text="Span (s)").pack(side="left")
        self.span_var = tk.StringVar(value="60")
        ttk.Combobox(ctrl, textvariable=self.span_var, width=6, state="readonly",
                     values=["10", "30", "60", "120", "300", "all"]).pack(side="left", padx=4)
        ttk.Button(ctrl, text="Clear", command=self._clear_trace).pack(side="left", padx=8)
        ttk.Button(ctrl, text="Save CSV...", command=self._save_trace).pack(side="left")

        self.stream_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctrl, text="Stream", variable=self.stream_var,
                        command=self._toggle_stream).pack(side="right")
        ttk.Button(ctrl, text="Read once", command=lambda: self._send("!READ")).pack(side="right", padx=6)

        fig = Figure(figsize=(9, 3.4), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.grid(alpha=0.3)
        self.line, = self.ax.plot([], [], lw=1.2)
        fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(fig, master=tab)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=12, pady=6)

        # Min/max and meter controls.
        bot = ttk.Frame(tab)
        bot.pack(fill="x", padx=16, pady=(0, 10))
        self.mm_lbl = tk.Label(bot, text="min/max --", font=("Consolas", 9),
                               justify="left", anchor="w")
        self.mm_lbl.pack(side="left")

        btns = ttk.Frame(bot)
        btns.pack(side="right")
        ttk.Button(btns, text="Reset min/max", command=lambda: self._send("!RESET")).pack(side="left", padx=2)
        ttk.Button(btns, text="Null leads", command=lambda: self._send("!ZERO")).pack(side="left", padx=2)
        ttk.Button(btns, text="Clear null", command=lambda: self._send("!ZEROCLR")).pack(side="left", padx=2)
        ttk.Label(btns, text="Mode").pack(side="left", padx=(12, 2))
        self.mode_set = ttk.Combobox(btns, width=12, state="readonly",
                                     values=[f"{i} {n}" for i, n in enumerate(MODE_NAMES)])
        self.mode_set.pack(side="left")
        self.mode_set.bind("<<ComboboxSelected>>", self._on_mode_set)
        ttk.Label(btns, text="Range").pack(side="left", padx=(12, 2))
        rng = ttk.Combobox(btns, width=6, state="readonly", values=["auto", "high", "low"])
        rng.set("auto")
        rng.pack(side="left")
        rng.bind("<<ComboboxSelected>>",
                 lambda e, w=rng: self._send({"auto": "!RANGE,A", "high": "!RANGE,1",
                                              "low": "!RANGE,0"}[w.get()]))

    # ---------------- Config tab ----------------
    def _build_cfg_tab(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="Config")

        bar = ttk.Frame(tab)
        bar.pack(fill="x", pady=6)
        ttk.Button(bar, text="Read all (!CFG)", command=self._cfg_read_all).pack(side="left", padx=4)
        ttk.Button(bar, text="Save to EEPROM", command=self._cfg_save).pack(side="left", padx=4)
        ttk.Button(bar, text="Reload EEPROM", command=self._cfg_load).pack(side="left", padx=4)
        ttk.Button(bar, text="Factory defaults", command=self._cfg_defaults).pack(side="left", padx=4)
        ttk.Button(bar, text="Seed legacy cal...", command=self._cfg_seedcal).pack(side="left", padx=16)
        ttk.Button(bar, text="Write SN...", command=self._write_sn_dialog).pack(side="left", padx=4)

        wrap = ttk.Frame(tab)
        wrap.pack(fill="both", expand=True)
        self.cfg_canvas = tk.Canvas(wrap, highlightthickness=0)
        sb = ttk.Scrollbar(wrap, orient="vertical", command=self.cfg_canvas.yview)
        self.cfg_inner = ttk.Frame(self.cfg_canvas)
        self.cfg_inner.bind("<Configure>", lambda e: self.cfg_canvas.configure(
            scrollregion=self.cfg_canvas.bbox("all")))
        self.cfg_canvas.create_window((0, 0), window=self.cfg_inner, anchor="nw")
        self.cfg_canvas.configure(yscrollcommand=sb.set)
        self.cfg_canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.bind_all("<MouseWheel>", self._cfg_scroll)

        self.cfg_placeholder = ttk.Label(
            self.cfg_inner, text="Connect and press Read all.", foreground="#777")
        self.cfg_placeholder.grid(row=0, column=0, padx=12, pady=12, sticky="w")
        self.cfg_next_row = 1

    # ---------------- Calibration tab ----------------
    def _build_cal_tab(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="Calibration")

        note = ("Put a known reference across the leads, type its value, press Capture. "
                "The meter computes the factor for whichever bucket the raw reading falls in.\n"
                "Values go to RAM only -- press Save to EEPROM on the Config tab to keep them.")
        ttk.Label(tab, text=note, foreground="#555", justify="left").pack(
            anchor="w", padx=12, pady=(8, 4))

        live = ttk.Frame(tab)
        live.pack(fill="x", padx=12, pady=4)
        ttk.Label(live, text="Live resistance:").pack(side="left")
        self.cal_live_lbl = tk.Label(live, text="--", font=("Consolas", 14, "bold"),
                                     fg="#0b6fb8")
        self.cal_live_lbl.pack(side="left", padx=8)
        ttk.Label(live, text="active bucket:").pack(side="left", padx=(16, 0))
        self.cal_bucket_lbl = tk.Label(live, text="--", font=("Consolas", 12))
        self.cal_bucket_lbl.pack(side="left", padx=8)

        grid = ttk.Frame(tab)
        grid.pack(fill="both", expand=True, padx=12, pady=6)
        hdrs = ["Bucket", "Range (ohm)", "Factor", "Known value", ""]
        for c, h in enumerate(hdrs):
            ttk.Label(grid, text=h, font=("Segoe UI", 9, "bold")).grid(
                row=0, column=c, sticky="w", padx=6, pady=(0, 4))

        self.cal_rows = {}
        for i in range(15):
            lo = 0.0 if i == 0 else R_CAL_EDGES[i - 1]
            hi = None if i == 14 else R_CAL_EDGES[i]
            rng = f"{lo:g} - {hi:g}" if hi is not None else f"{lo:g} and up"
            r = i + 1
            name = tk.Label(grid, text=f"RCAL{i:02d}", font=("Consolas", 9))
            name.grid(row=r, column=0, sticky="w", padx=6)
            ttk.Label(grid, text=rng, foreground="#666").grid(row=r, column=1, sticky="w", padx=6)
            fac = tk.Label(grid, text="--", font=("Consolas", 9), fg="#06a")
            fac.grid(row=r, column=2, sticky="w", padx=6)
            var = tk.StringVar()
            ent = ttk.Entry(grid, width=12, textvariable=var)
            ent.grid(row=r, column=3, sticky="w", padx=6)
            ent.bind("<Return>", lambda e, k=i: self._cal_capture(k))
            ttk.Button(grid, text="Capture", width=8,
                       command=lambda k=i: self._cal_capture(k)).grid(row=r, column=4, padx=6, pady=1)
            self.cal_rows[i] = {"var": var, "factor": fac, "name": name}

        other = ttk.LabelFrame(tab, text="Voltage and current")
        other.pack(fill="x", padx=12, pady=10)
        ttk.Label(other, text="Known voltage (V):").grid(row=0, column=0, sticky="w", padx=6, pady=4)
        self.calv_var = tk.StringVar()
        ttk.Entry(other, width=12, textvariable=self.calv_var).grid(row=0, column=1, padx=6)
        ttk.Button(other, text="Capture VSCALE",
                   command=self._cal_voltage).grid(row=0, column=2, padx=6)
        self.calv_lbl = tk.Label(other, text="VSCALE --", font=("Consolas", 9), fg="#06a")
        self.calv_lbl.grid(row=0, column=3, sticky="w", padx=12)

        ttk.Label(other, text="Current zero:").grid(row=1, column=0, sticky="w", padx=6, pady=4)
        ttk.Button(other, text="Capture IZERO (leads open, no current)",
                   command=lambda: self._send("!CALI")).grid(row=1, column=1, columnspan=2,
                                                             sticky="w", padx=6)
        self.cali_lbl = tk.Label(other, text="IZERO --", font=("Consolas", 9), fg="#06a")
        self.cali_lbl.grid(row=1, column=3, sticky="w", padx=12)

        det = ttk.LabelFrame(tab, text="Ammeter detection")
        det.pack(fill="x", padx=12, pady=(0, 10))
        ttk.Label(det, justify="left", foreground="#555", text=(
            "The meter decides once at boot what is on the current channel, and "
            "suppresses readings entirely if it finds nothing.\n"
            "Run this with and without your sensor plugged in, then set "
            "IDETCNT / IDETLO / IDETHI / IDETPP from the numbers it reports.\n"
            "Re-detecting here is a bench aid; what the meter runs with is "
            "whatever it decided at power-on.")
        ).grid(row=0, column=0, columnspan=2, sticky="w", padx=6, pady=(4, 6))
        ttk.Button(det, text="Re-detect now (!IDET)",
                   command=lambda: self._send("!IDET")).grid(row=1, column=0,
                                                             sticky="w", padx=6, pady=(0, 6))
        self.idet_lbl = tk.Label(det, text="not run this session",
                                 font=("Consolas", 9), fg="#777", justify="left")
        self.idet_lbl.grid(row=1, column=1, sticky="w", padx=12, pady=(0, 6))

    # ---------------- Log tab ----------------
    def _build_log_tab(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="Log")

        bar = ttk.Frame(tab)
        bar.pack(fill="x", pady=6, padx=12)
        ttk.Button(bar, text="Arm log (!LOG)", command=lambda: self._send("!LOG")).pack(side="left", padx=4)
        ttk.Button(bar, text="Dump (!DUMP)", command=self._log_dump).pack(side="left", padx=4)
        ttk.Button(bar, text="Save CSV...", command=self._save_log).pack(side="left", padx=4)
        self.log_status = ttk.Label(bar, text="no capture", foreground="#777")
        self.log_status.pack(side="left", padx=16)

        fig = Figure(figsize=(9, 5), dpi=100)
        self.log_ax = fig.add_subplot(111)
        self.log_ax.grid(alpha=0.3)
        self.log_ax2 = self.log_ax.twinx()
        fig.tight_layout()
        self.log_canvas = FigureCanvasTkAgg(fig, master=tab)
        self.log_canvas.get_tk_widget().pack(fill="both", expand=True, padx=12, pady=6)

    # ==================================================================
    #  SERIAL PLUMBING
    # ==================================================================
    def _ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def _refresh_ports(self):
        ports = self._ports()
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.ser.is_open:
            self.ser.disconnect()
            self.connect_btn.config(text="Connect")
            self._log("-- disconnected")
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("microDMM", "Pick a serial port first.")
            return
        try:
            self.ser.connect(port)
        except Exception as exc:
            messagebox.showerror("microDMM", f"Could not open {port}:\n{exc}")
            return
        self.connect_btn.config(text="Disconnect")
        self.sn_prompted = False
        self._log(f"-- connected to {port}")
        self.after(600, lambda: self._send("!STATUS"))
        self.after(800, self._cfg_read_all)

    def _send(self, cmd):
        if not self.ser.is_open:
            self._log("** not connected")
            return
        self.ser.send(cmd)
        self._log(f">> {cmd}")

    def _log(self, text):
        self.log_text.insert("end", text + "\n")
        self.log_text.see("end")
        if float(self.log_text.index("end-1c").split(".")[0]) > 400:
            self.log_text.delete("1.0", "200.0")

    # ==================================================================
    #  LINE HANDLING
    # ==================================================================
    def _tick(self):
        try:
            while True:
                kind, payload = self.line_queue.get_nowait()
                if kind == "__error__":
                    self._log(f"** serial error: {payload}")
                    self.ser.disconnect()
                    self.connect_btn.config(text="Connect")
                else:
                    self._handle_line(payload)
        except queue.Empty:
            pass
        self.after(UI_TICK_MS, self._tick)

    def _handle_line(self, line):
        if not line.startswith("$"):
            self._log(f"   {line}")          # boot noise / library prints
            return
        parts = line.split(",")
        tag = parts[0]

        if tag == "$LIVE":
            self._handle_live(parts)
            return                            # far too frequent to echo
        if tag == "$MINMAX":
            self._handle_minmax(parts)
            return
        if tag == "$CFG":
            self._handle_cfg(parts)
            return
        if tag == "$LOG":
            if self.log_collecting and len(parts) >= 5:
                self.log_rows.append([float(x) for x in parts[2:5]])
            return

        self._log(f"<< {line}")

        if tag == "$STATUS":
            self._handle_status(parts)
        elif tag == "$CFGEND":
            self.cfg_collecting = False
            if self.pending_csv_log:
                self.pending_csv_log = False
                self._write_units_csv()
        elif tag == "$SN":
            self.unit_sn = parts[1] if len(parts) > 1 else ""
            self.sn_lbl.config(text=f"SN {self.unit_sn or '--'}")
        elif tag == "$MODE" and len(parts) > 1:
            self._set_mode(int(parts[1]))
        elif tag == "$IDET":
            self._handle_idet(parts)
        elif tag == "$CAL":
            self._handle_cal(parts)
        elif tag == "$LOGSTART":
            self.log_rows = []
            self.log_collecting = True
            self.log_status.config(text="receiving...", foreground="#d68000")
        elif tag == "$LOGEND":
            self.log_collecting = False
            self._finish_log()
        elif tag == "$OK" and len(parts) > 1 and parts[1] == "save":
            self._on_save_ok()
        elif tag == "$ERR":
            self.bell()

    def _handle_live(self, p):
        if len(p) < 12:
            return
        try:
            vals = [float(x) for x in p[1:11]]
            flags = int(p[11])
        except ValueError:
            return
        self.live = vals
        self.flags = flags
        self._set_mode(int(vals[9]))

        ms = vals[0]
        if self.t0 is None:
            self.t0 = ms
        t = (ms - self.t0) / 1000.0

        r_live = bool(flags & (1 << BIT_R_MEASURED))

        # Don't extend a resistance trace with samples the meter did not take
        # this pass -- it would draw a flat line that looks like a steady
        # reading rather than a gap where nothing was measured.
        trace_name = self.trace_var.get()
        if r_live or trace_name not in R_TRACES:
            idx, unit = TRACES[trace_name]
            self.trace_t.append(t)
            self.trace_y.append(vals[idx])

        # Primary readout follows whatever the meter selected for itself.
        volt_disp = bool(flags & (1 << 0))
        have_i = bool(flags & (1 << BIT_I_SENSOR))
        # Amps mode cannot promote a suppressed channel to the primary readout:
        # with no sensor detected the firmware holds Ireading at zero, and
        # showing a big confident 0 A would read as a measurement.
        amps = bool(flags & (1 << BIT_AMPS_MODE)) and have_i
        r_open = bool(flags & (1 << 13))
        if amps:
            self.primary_lbl.config(text=eng(vals[6], "A"), fg="#1faa3f")
            self.primary_cap.config(text="current")
        elif volt_disp:
            if flags & (1 << 2):
                self.primary_lbl.config(text=eng(vals[5], "V"), fg="#d68000")
                self.primary_cap.config(text="AC rms")
            else:
                self.primary_lbl.config(text=eng(vals[3], "V"), fg="#0b6fb8")
                self.primary_cap.config(text="DC volts")
        elif not r_live:
            self.primary_lbl.config(text="--", fg="#aaaaaa")
            self.primary_cap.config(text="resistance not measured in this mode")
        elif r_open:
            self.primary_lbl.config(text="OPEN", fg="#7a7a7a")
            self.primary_cap.config(text="resistance")
        else:
            self.primary_lbl.config(text=eng(vals[2], "ohm"), fg="#0b6fb8")
            self.primary_cap.config(text="resistance (nulled)")

        for name, val, unit_ in [
                ("Resistance", vals[1], "ohm"), ("Nulled", vals[2], "ohm"),
                ("Voltage", vals[3], "V"), ("Average", vals[4], "V"),
                ("VAC rms", vals[5], "V"), ("Current", vals[6], "A"),
                ("Ohms rail", vals[7], "V"), ("Supply", vals[8], "V")]:
            if name == "Current" and not have_i:
                # Say why there is no number rather than showing 0 A, which
                # looks like a reading of zero current.
                self.sec_lbls[name].config(text="no sensor", fg="#aaaaaa")
            elif not r_live and name in ("Resistance", "Nulled", "Ohms rail"):
                # Held over from the last mode that measured it, so show it as
                # stale rather than as a present-tense reading.
                self.sec_lbls[name].config(text="not measured", fg="#aaaaaa")
            else:
                self.sec_lbls[name].config(text=eng(val, unit_, 4), fg="black")

        for name, (lbl, bit, colour) in self.lamps.items():
            on = bool(flags & (1 << bit))
            lbl.config(bg=colour if on else "#eeeeee",
                       fg="white" if on else "#aaaaaa")

        self.cal_live_lbl.config(text=eng(vals[1], "ohm"))
        b = self._bucket_of(vals[1])
        self.cal_bucket_lbl.config(text=f"RCAL{b:02d}")
        for i, row in self.cal_rows.items():
            row["name"].config(fg="#c02020" if i == b else "black")

        self.dirty = bool(flags & (1 << 15))
        self.dirty_lbl.config(text="UNSAVED CONFIG" if self.dirty else "")

    def _handle_minmax(self, p):
        if len(p) < 9:
            return
        try:
            v = [float(x) for x in p[1:7]]
        except ValueError:
            return
        self.minmax = v
        self.mm_lbl.config(
            text=(f"V  {eng(v[0],'V')} .. {eng(v[1],'V')}   (t {p[7]} / {p[8]})\n"
                  f"R  {eng(v[2],'ohm')} .. {eng(v[3],'ohm')}\n"
                  f"I  {eng(v[4],'A')} .. {eng(v[5],'A')}"))

    def _handle_status(self, p):
        kv = {}
        for item in p[1:]:
            if "=" in item:
                k, _, val = item.partition("=")
                kv[k] = val
        if "sn" in kv:
            self.unit_sn = kv["sn"]
            self.sn_lbl.config(text=f"SN {self.unit_sn or '--'}")
        if "hwrev" in kv:
            self.hwrev = kv["hwrev"]
        if "mode" in kv:
            try:
                self._set_mode(int(kv["mode"]))
            except ValueError:
                pass
        if "dirty" in kv:
            self.dirty = kv["dirty"] == "1"
            self.dirty_lbl.config(text="UNSAVED CONFIG" if self.dirty else "")
        if "stream" in kv:
            self.stream_var.set(kv["stream"] == "1")
        # !STATUS is echoed after every !SET, so the prompt has to be once per
        # connection or editing config would raise a dialog on every keystroke.
        if not self.unit_sn and not self.sn_prompted:
            self.sn_prompted = True
            self.after(300, self._prompt_for_sn)

    def _set_mode(self, m):
        self.mode = m
        name = MODE_NAMES[m] if 0 <= m < len(MODE_NAMES) else "?"
        self.mode_lbl.config(text=f"mode {m} {name}")

    def _on_mode_set(self, _event):
        sel = self.mode_set.get()
        if sel:
            self._send(f"!MODE,{sel.split()[0]}")

    # ==================================================================
    #  CONFIG TABLE
    # ==================================================================
    def _cfg_scroll(self, event):
        if self.nb.index(self.nb.select()) == 1:
            self.cfg_canvas.yview_scroll(int(-event.delta / 120), "units")

    def _cfg_group_of(self, key):
        return KEY_META.get(key, ("Other",))[0]

    def _handle_cfg(self, p):
        if len(p) < 3:
            return
        key, value = p[1], p[2]
        self.cfg_snapshot[key] = value
        self._cfg_update(key, value)
        if key.startswith("RCAL"):
            try:
                self.cal_rows[int(key[4:])]["factor"].config(text=value)
            except (ValueError, KeyError):
                pass
        elif key == "VSCALE":
            self.calv_lbl.config(text=f"VSCALE {value}")
        elif key == "IZERO":
            self.cali_lbl.config(text=f"IZERO {value}")
        elif key == "HWREV":
            self.hwrev = value

    def _cfg_add_group_header(self, group):
        ttk.Label(self.cfg_inner, text=group, font=("Segoe UI", 10, "bold"),
                  foreground="#0b6fb8").grid(row=self.cfg_next_row, column=0,
                                             columnspan=5, sticky="w",
                                             padx=4, pady=(10, 2))
        self.cfg_next_row += 1

    def _cfg_add_row(self, key, value):
        if self.cfg_placeholder is not None:
            self.cfg_placeholder.destroy()
            self.cfg_placeholder = None

        group = self._cfg_group_of(key)
        if group not in self.cfg_groups_done:
            self.cfg_groups_done.add(group)
            self._cfg_add_group_header(group)

        meta = KEY_META.get(key)
        kind = meta[1] if meta else "num"
        desc = meta[2] if meta else ""

        r = self.cfg_next_row
        self.cfg_next_row += 1

        ttk.Label(self.cfg_inner, text=key, font=("Consolas", 10)).grid(
            row=r, column=0, sticky="w", padx=(16, 6), pady=1)
        dev_lbl = ttk.Label(self.cfg_inner, text=value, width=12,
                            font=("Consolas", 10), foreground="#06a")
        dev_lbl.grid(row=r, column=1, sticky="w", padx=6)

        var = tk.StringVar(value=value)
        if kind == "bool":
            editor = ttk.Combobox(self.cfg_inner, width=6, textvariable=var,
                                  values=["0", "1"], state="readonly")
        else:
            editor = ttk.Entry(self.cfg_inner, width=12, textvariable=var)
            editor.bind("<Return>", lambda e, k=key: self._cfg_set(k))
        editor.grid(row=r, column=2, sticky="w", padx=6)

        ttk.Button(self.cfg_inner, text="Set", width=4,
                   command=lambda k=key: self._cfg_set(k)).grid(row=r, column=3, padx=2)
        ttk.Label(self.cfg_inner, text=desc, foreground="#555").grid(
            row=r, column=4, sticky="w", padx=10)

        self.cfg_rows[key] = {"var": var, "dev_lbl": dev_lbl}

    def _cfg_update(self, key, value):
        if key not in self.cfg_rows:
            self._cfg_add_row(key, value)
            return
        row = self.cfg_rows[key]
        row["dev_lbl"].config(text=value)
        row["var"].set(value)

    def _cfg_read_all(self):
        self.cfg_collecting = True
        self._send("!CFG")

    def _cfg_set(self, key):
        row = self.cfg_rows.get(key)
        if row is None:
            return
        val = row["var"].get().strip()
        try:
            float(val)
        except ValueError:
            self._log(f"** {key}: '{val}' is not a number")
            return
        self._send(f"!SET,{key},{val}")
        self._send("!STATUS")

    def _cfg_save(self):
        self._send("!SAVE")
        self._send("!STATUS")

    def _cfg_load(self):
        self._send("!LOAD")
        self._cfg_read_all()

    def _cfg_defaults(self):
        if messagebox.askyesno("microDMM",
                               "Reset every key to compiled defaults?\n\n"
                               "HWREV and BRIDGE are preserved (they describe the "
                               "board, not a preference). This changes RAM only -- "
                               "Save to EEPROM to keep it."):
            self._send("!DEFAULTS")
            self._cfg_read_all()

    def _cfg_seedcal(self):
        n = simpledialog.askinteger(
            "Seed legacy calibration",
            "Original unit id (1-6).\n\n"
            "Loads that unit's factory-tuned correction factors, voltage scale "
            "and constants into RAM. Check them, then Save to EEPROM.",
            parent=self, minvalue=1, maxvalue=6)
        if n is not None:
            self._send(f"!SEEDCAL,{n}")
            self._cfg_read_all()

    def _on_save_ok(self):
        self._log("-- saved to EEPROM")
        self.pending_csv_log = True
        self._cfg_read_all()          # re-read, then log the fresh dump to CSV

    # ==================================================================
    #  CALIBRATION
    # ==================================================================
    @staticmethod
    def _bucket_of(raw):
        for i, edge in enumerate(R_CAL_EDGES):
            if raw < edge:
                return i
        return 14

    def _cal_capture(self, idx):
        val = self.cal_rows[idx]["var"].get().strip()
        try:
            actual = float(val)
        except ValueError:
            messagebox.showwarning("microDMM", "Type the known resistance first.")
            return
        if self.live is not None:
            live_bucket = self._bucket_of(self.live[1])
            if live_bucket != idx and not messagebox.askyesno(
                    "microDMM",
                    f"The live reading sits in RCAL{live_bucket:02d}, not "
                    f"RCAL{idx:02d}.\n\nCapture into RCAL{idx:02d} anyway?"):
                return
        self._send(f"!CAL,{actual},{idx}")
        self._send(f"!GET,RCAL{idx:02d}")

    def _cal_voltage(self):
        try:
            actual = float(self.calv_var.get().strip())
        except ValueError:
            messagebox.showwarning("microDMM", "Type the known voltage first.")
            return
        self._send(f"!CALV,{actual}")
        self._send("!GET,VSCALE")

    def _handle_idet(self, p):
        # $IDET,state,meanCounts,meanV,ppV,atGnd,atMid,steady,izero
        if len(p) < 9:
            return
        state, mean, meanv, ppv, at_gnd, at_mid, steady, izero = p[1:9]
        pretty = {"high": "hall sensor (high range)",
                  "low": "shunt (low range)",
                  "off": "nothing fitted -- readings suppressed"}.get(state, state)
        colour = "#c02020" if state == "off" else "#1faa3f"
        self.idet_lbl.config(
            text=(f"{pretty}\n"
                  f"mean {mean} counts / {meanv} V, peak-peak {ppv} V\n"
                  f"at ground {at_gnd}, at mid-rail {at_mid}, steady {steady}, "
                  f"IZERO {izero}"),
            fg=colour)
        self.cali_lbl.config(text=f"IZERO {izero}")

    def _handle_cal(self, p):
        # $CAL,IZERO carries only the new baseline, so it is shorter than the
        # resistance and voltage forms -- check it before the length guard.
        if len(p) < 3:
            return
        if p[1] == "IZERO":
            self.cali_lbl.config(text=f"IZERO {p[2]}")
            return
        if len(p) < 5:
            return
        if p[1] == "VSCALE":
            self.calv_lbl.config(text=f"VSCALE {p[4]}")
        else:
            try:
                idx = int(p[1])
                self.cal_rows[idx]["factor"].config(text=p[4])
                self._log(f"-- RCAL{idx:02d}: raw {p[2]} -> {p[3]}, factor {p[4]}")
            except (ValueError, KeyError):
                pass

    # ==================================================================
    #  LIVE PLOT
    # ==================================================================
    def _clear_trace(self):
        self.trace_t.clear()
        self.trace_y.clear()
        self.t0 = None

    def _toggle_stream(self):
        self._send(f"!STREAM,{1 if self.stream_var.get() else 0}")

    def _redraw_live(self):
        if self.trace_t:
            span = self.span_var.get()
            ts = list(self.trace_t)
            ys = list(self.trace_y)
            if span != "all":
                cutoff = ts[-1] - float(span)
                keep = [i for i, t in enumerate(ts) if t >= cutoff]
                if keep:
                    ts = ts[keep[0]:]
                    ys = ys[keep[0]:]
            self.line.set_data(ts, ys)
            self.ax.set_xlim(ts[0], max(ts[-1], ts[0] + 1e-3))
            lo, hi = min(ys), max(ys)
            pad = (hi - lo) * 0.1 or (abs(hi) * 0.1 or 1.0)
            self.ax.set_ylim(lo - pad, hi + pad)
            self.ax.set_ylabel(self.trace_var.get())
            self.ax.set_xlabel("seconds")
            self.canvas.draw_idle()
        self.after(PLOT_REFRESH_MS, self._redraw_live)

    def _save_trace(self):
        if not self.trace_t:
            messagebox.showinfo("microDMM", "Nothing recorded yet.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile=f"microdmm_trace_{datetime.now():%Y%m%d_%H%M%S}.csv")
        if not path:
            return
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["seconds", self.trace_var.get()])
            w.writerows(zip(self.trace_t, self.trace_y))
        self._log(f"-- trace saved to {path}")

    # ==================================================================
    #  ON-BOARD LOG
    # ==================================================================
    def _log_dump(self):
        self.log_rows = []
        self._send("!DUMP")

    def _finish_log(self):
        if not self.log_rows:
            self.log_status.config(text="empty capture", foreground="#c02020")
            return
        t = [r[0] for r in self.log_rows]
        v = [r[1] for r in self.log_rows]
        i = [r[2] for r in self.log_rows]
        self.log_ax.clear()
        self.log_ax2.clear()
        self.log_ax.plot(t, v, lw=1.2, color="#0b6fb8", label="V")
        self.log_ax2.plot(t, i, lw=1.2, color="#d68000", label="A")
        self.log_ax.set_xlabel("seconds since arm")
        self.log_ax.set_ylabel("volts", color="#0b6fb8")
        self.log_ax2.set_ylabel("amps", color="#d68000")
        self.log_ax.grid(alpha=0.3)
        self.log_canvas.draw_idle()
        self.log_status.config(text=f"{len(self.log_rows)} samples",
                               foreground="#1faa3f")

    def _save_log(self):
        if not self.log_rows:
            messagebox.showinfo("microDMM", "Dump a capture first.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile=f"microdmm_log_{datetime.now():%Y%m%d_%H%M%S}.csv")
        if not path:
            return
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["seconds", "volts", "amps"])
            w.writerows(self.log_rows)
        self._log(f"-- log saved to {path}")

    # ==================================================================
    #  SERIAL NUMBER + PER-UNIT CSV
    # ==================================================================
    def _prompt_for_sn(self):
        if self.unit_sn or not self.ser.is_open:
            return
        sn = simpledialog.askstring(
            "microDMM",
            "This meter has no serial number.\n\n"
            "Assign one so its saved configuration can be logged\n"
            "(e.g. 20260905_001). Cancel to skip.",
            parent=self)
        if sn:
            self._write_sn(sn.strip())

    def _write_sn_dialog(self):
        sn = simpledialog.askstring("microDMM", "Serial number (max 15 chars, no commas):",
                                    initialvalue=self.unit_sn, parent=self)
        if sn:
            self._write_sn(sn.strip())

    def _write_sn(self, sn):
        if "," in sn or len(sn) > 15:
            messagebox.showwarning("microDMM", "Max 15 characters, no commas.")
            return
        self._send(f"!SN,{sn}")

    def _read_units_csv(self):
        if not os.path.exists(UNITS_CSV):
            return [], []
        with open(UNITS_CSV, newline="") as fh:
            rows = list(csv.DictReader(fh))
            fields = rows and list(rows[0].keys()) or []
        return rows, fields

    def _write_units_csv(self):
        """Append or update this unit's row after a successful !SAVE."""
        if not self.unit_sn:
            self._log("** no serial number: per-unit CSV log skipped")
            return
        if not self.cfg_snapshot:
            return
        rows, fields = self._read_units_csv()
        record = {"SN": self.unit_sn,
                  "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
        record.update(self.cfg_snapshot)

        cols = list(UNITS_CSV_FIXED)
        for f in fields:
            if f not in cols:
                cols.append(f)
        for k in record:
            if k not in cols:
                cols.append(k)

        replaced = False
        for r in rows:
            if r.get("SN") == self.unit_sn:
                r.clear()
                r.update(record)
                replaced = True
                break
        if not replaced:
            rows.append(record)

        with open(UNITS_CSV, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols, restval="")
            w.writeheader()
            for r in rows:
                w.writerow({c: r.get(c, "") for c in cols})
        self._log(f"-- {'updated' if replaced else 'added'} {self.unit_sn} in "
                  f"{os.path.basename(UNITS_CSV)}")

    def _on_close(self):
        try:
            self.ser.disconnect()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
