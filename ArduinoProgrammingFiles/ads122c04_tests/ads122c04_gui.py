"""
ADS122C04 Test GUI
Requires: pip install pyserial matplotlib
"""

import sys
import math
import time
import threading
import queue
from collections import deque
import bisect

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ── lookup tables (must match Arduino sketch order) ───────────────────────────
MUX_LABELS = [
    "AIN0 − AIN1 (diff)", "AIN0 − AIN2 (diff)", "AIN0 − AIN3 (diff)",
    "AIN1 − AIN0 (diff)", "AIN1 − AIN2 (diff)", "AIN1 − AIN3 (diff)",
    "AIN2 − AIN3 (diff)", "AIN3 − AIN2 (diff)",
    "AIN0 (SE)", "AIN1 (SE)", "AIN2 (SE)", "AIN3 (SE)",
    "(REF+−REF−)/4", "(AVDD−AVSS)/4", "Shorted",
]

GAIN_LABELS  = ["1×", "2×", "4×", "8×", "16×", "32×", "64×", "128×"]
RATE_LABELS  = ["20/40 SPS", "45/90 SPS", "90/180 SPS", "175/350 SPS",
                "330/660 SPS", "600/1200 SPS", "1000/2000 SPS"]
RATE_NOMINAL = [20, 45, 90, 175, 330, 600, 1000]

MAX_DISPLAY_PTS = 5_000   # downsample limit for streaming waveform render

TC_UNITS = ["°C", "°F", "mV"]

# EMA filter, applied to the thermocouple traces in the GUI rather than on the
# board: the filter state is rebuilt from the sample buffer whenever a setting
# changes, so a new time constant re-filters the history already on screen
# instead of only affecting samples that arrive afterwards.
TRACE_MODES = ["Instant + EMA", "Instant only", "EMA only"]
EMA_MODES = ["Time constant (s)", "Alpha (per sample)"]

# Meter layout, in characters of the monospaced display font: the value column
# holds the widest reading any mode produces ("+0.007629", "+2501.60") and the
# unit column holds "V", "mV", "°C", "°F".
METER_VALUE_W = 9
METER_UNIT_W  = 2
METER_W = METER_VALUE_W + 1 + METER_UNIT_W
CJ_SOURCES = ["Internal sensor", "Fixed value"]

# ── type-K thermocouple math (NIST ITS-90) ────────────────────────────────────
# Same fits as the sketch. The board already sends a compensated temperature;
# these are here so the GUI can re-derive one — for burst captures, and for any
# sample when the cold junction or units change after the fact.

_TK_NEG = (0.0, 0.394501280250E-01, 0.236223735980E-04, -0.328589067840E-06,
           -0.499048287770E-08, -0.675090591730E-10, -0.574103274280E-12,
           -0.310888728940E-14, -0.104516093650E-16, -0.198892668780E-19,
           -0.163226974860E-22)
_TK_POS = (-0.176004136860E-01, 0.389212049750E-01, 0.185587700320E-04,
           -0.994575928740E-07, 0.318409457190E-09, -0.560728448890E-12,
           0.560750590590E-15, -0.320207200030E-18, 0.971511471520E-22,
           -0.121047212750E-25)
_TK_A0, _TK_A1, _TK_A2 = 0.118597600000E+00, -0.118343200000E-03, 0.126968600000E+03

_TK_INV_NEG = (0.0, 2.5173462E+01, -1.1662878E+00, -1.0833638E+00,
               -8.9773540E-01, -3.7342377E-01, -8.6632643E-02, -1.0450598E-02,
               -5.1920577E-04)
_TK_INV_MID = (0.0, 2.508355E+01, 7.860106E-02, -2.503131E-01, 8.315270E-02,
               -1.228034E-02, 9.804036E-04, -4.413030E-05, 1.057734E-06,
               -1.052755E-08)
_TK_INV_HIGH = (-1.318058E+02, 4.830222E+01, -1.646031E+00, 5.464731E-02,
                -9.650715E-04, 8.802193E-06, -3.110810E-08)

# Range limits are the forward polynomial evaluated at the endpoints, not
# the rounded table values: at 1372 °C it returns 54.8864 mV, so a limit of
# 54.886 would reject the top hundredth of a degree of the scale.
TK_MV_MIN, TK_MV_MAX = -5.89141, 54.88637   # -200 °C … 1372 °C


def _poly(coeffs, x):
    out, p = 0.0, 1.0
    for c in coeffs:
        out += c * p
        p *= x
    return out


def tk_temp_to_mv(t_c: float) -> float:
    """Type-K EMF in mV for a junction at t_c, referenced to 0 °C."""
    if t_c < 0.0:
        return _poly(_TK_NEG, t_c)
    return _poly(_TK_POS, t_c) + _TK_A0 * math.exp(_TK_A1 * (t_c - _TK_A2) ** 2)


def tk_mv_to_temp(mv: float) -> float:
    """Inverse: mV referenced to 0 °C -> °C. NaN outside the type-K range."""
    if mv != mv or mv < TK_MV_MIN or mv > TK_MV_MAX:
        return float("nan")
    if mv < 0.0:
        return _poly(_TK_INV_NEG, mv)
    if mv < 20.644:
        return _poly(_TK_INV_MID, mv)
    return _poly(_TK_INV_HIGH, mv)


def tk_hot_temp(v_tc: float, cj_c: float) -> float:
    """Cold-junction-compensated hot-junction temperature, °C.

    v_tc is what the ADC sees across the thermocouple, in volts; cj_c is the
    cold junction. Adding back the EMF the cold junction is not producing puts
    the total on the 0 °C reference the tables use.
    """
    if cj_c != cj_c:
        return float("nan")
    return tk_mv_to_temp(v_tc * 1000.0 + tk_temp_to_mv(cj_c))


def c_to_f(t_c: float) -> float:
    return t_c * 9.0 / 5.0 + 32.0

BG     = "#1a1a1a"
PANEL  = "#222222"
ACCENT = "#00ff32"
AMBER  = "#ffaa00"
CYAN   = "#00ccff"   # EMA trace — distinct from the green instant trace
DIM    = "#888888"
WHITE  = "#e0e0e0"
RED    = "#ff4444"
SEP    = "#444444"


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ADS122C04 Tester")
        self.configure(bg=BG)
        self.resizable(True, True)

        self._ser: serial.Serial | None = None
        self._rx_queue: queue.Queue = queue.Queue()
        self._rx_thread: threading.Thread | None = None
        self._running = False
        self._streaming = False

        # streaming plot data — maxlen caps memory; display window is time-based
        self._voltages:   deque = deque(maxlen=200_000)
        self._timestamps: deque = deque(maxlen=200_000)
        self._t0 = time.monotonic()

        # thermocouple mode: hot-junction °C and raw EMF in mV, in lockstep
        # with _timestamps, so the unit selector is a pure display transform
        self._tc_mode = False
        self._tc_c:  deque = deque(maxlen=200_000)
        self._tc_mv: deque = deque(maxlen=200_000)
        self._last_cj_c = float("nan")

        # EMA of those two series, maintained incrementally in lockstep with
        # them. Both are kept because °C→°F is affine (so the filter commutes
        # with it) but volts→°C is not: filtering mV and converting would not
        # give the same answer as filtering the temperature.
        self._ema_c:  deque = deque(maxlen=200_000)
        self._ema_mv: deque = deque(maxlen=200_000)
        self._ema_state_c = None
        self._ema_state_mv = None
        self._ema_last_t = None

        self._last_stream_plot = 0.0   # throttle streaming plot to ~30 fps

        # burst state
        self._in_burst     = False
        self._burst_total  = 0      # expected sample count from header
        self._burst_lsb_v  = 0.0   # volts per LSB from header
        self._burst_sps    = 0.0
        self._burst_raws:  list = []
        self._burst_volts: list = []

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._poll_queue)

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=8, pady=4)

        tk.Label(top, text="Port:", bg=BG, fg=WHITE).pack(side="left")
        self._port_var = tk.StringVar()
        self._port_cb = ttk.Combobox(top, textvariable=self._port_var,
                                     width=14, state="readonly")
        self._port_cb.pack(side="left", padx=4)

        tk.Label(top, text="Baud:", bg=BG, fg=WHITE).pack(side="left")
        self._baud_var = tk.StringVar(value="115200")
        ttk.Combobox(top, textvariable=self._baud_var,
                     values=["9600","57600","115200","230400","460800"],
                     width=8, state="readonly").pack(side="left", padx=4)

        tk.Button(top, text="↺", bg=PANEL, fg=WHITE, relief="flat",
                  command=self._refresh_ports).pack(side="left")

        self._connect_btn = tk.Button(top, text="Connect", bg="#225522", fg=ACCENT,
                                      relief="flat", width=10,
                                      command=self._toggle_connect)
        self._connect_btn.pack(side="left", padx=8)

        self._status_lbl = tk.Label(top, text="Disconnected", bg=BG, fg=RED)
        self._status_lbl.pack(side="left")

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=8, pady=4)

        left = tk.Frame(body, bg=BG, width=264)
        left.pack(side="left", fill="y", padx=(0, 8))
        left.pack_propagate(False)

        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True)

        self._build_controls(left)
        self._build_display(right)

    def _lbl(self, parent, text, color=DIM):
        tk.Label(parent, text=text, bg=BG, fg=color, anchor="w").pack(
            fill="x", pady=(6, 0))

    def _sep(self, parent):
        tk.Frame(parent, bg=SEP, height=1).pack(fill="x", pady=6)

    def _build_controls(self, f):
        self._lbl(f, "Input MUX")
        self._mux_var = tk.StringVar(value=MUX_LABELS[8])
        mux_cb = ttk.Combobox(f, textvariable=self._mux_var,
                               values=MUX_LABELS, state="readonly")
        mux_cb.pack(fill="x")
        mux_cb.bind("<<ComboboxSelected>>",
                    lambda _: self._send_indexed("!MUX", MUX_LABELS, self._mux_var))

        self._lbl(f, "Gain")
        self._gain_var = tk.StringVar(value=GAIN_LABELS[0])
        gain_cb = ttk.Combobox(f, textvariable=self._gain_var,
                                values=GAIN_LABELS, state="readonly")
        gain_cb.pack(fill="x")
        gain_cb.bind("<<ComboboxSelected>>",
                     lambda _: self._send_indexed("!GAIN", GAIN_LABELS, self._gain_var))

        self._lbl(f, "Data Rate")
        self._rate_var = tk.StringVar(value=RATE_LABELS[0])
        rate_cb = ttk.Combobox(f, textvariable=self._rate_var,
                                values=RATE_LABELS, state="readonly")
        rate_cb.pack(fill="x")
        rate_cb.bind("<<ComboboxSelected>>",
                     lambda _: self._send_indexed("!RATE", RATE_LABELS, self._rate_var))

        self._lbl(f, "Options")
        tog = tk.Frame(f, bg=BG)
        tog.pack(fill="x")

        self._pga_var = tk.BooleanVar(value=False)
        tk.Checkbutton(tog, text="PGA Enabled", variable=self._pga_var,
                       bg=BG, fg=WHITE, selectcolor=PANEL, activebackground=BG,
                       command=lambda: self._send_bool("!PGA", self._pga_var)
                       ).pack(anchor="w")

        self._turbo_var = tk.BooleanVar(value=False)
        tk.Checkbutton(tog, text="Turbo Mode", variable=self._turbo_var,
                       bg=BG, fg=WHITE, selectcolor=PANEL, activebackground=BG,
                       command=lambda: self._send_bool("!TURBO", self._turbo_var)
                       ).pack(anchor="w")

        self._temp_var = tk.BooleanVar(value=False)
        tk.Checkbutton(tog, text="Temp Sensor", variable=self._temp_var,
                       bg=BG, fg=WHITE, selectcolor=PANEL, activebackground=BG,
                       command=lambda: self._send_bool("!TEMP", self._temp_var)
                       ).pack(anchor="w")

        # ── thermocouple section ──────────────────────────────────────────────
        self._sep(f)
        self._lbl(f, "Thermocouple (type K)", color=ACCENT)

        self._tc_var = tk.BooleanVar(value=False)
        tk.Checkbutton(f, text="TC Mode", variable=self._tc_var,
                       bg=BG, fg=WHITE, selectcolor=PANEL, activebackground=BG,
                       command=self._on_tc_toggle).pack(anchor="w")

        unit_row = tk.Frame(f, bg=BG)
        unit_row.pack(fill="x", pady=2)
        tk.Label(unit_row, text="Units:", bg=BG, fg=WHITE,
                 width=8, anchor="w").pack(side="left")
        self._tc_unit_var = tk.StringVar(value=TC_UNITS[0])
        unit_cb = ttk.Combobox(unit_row, textvariable=self._tc_unit_var,
                               values=TC_UNITS, state="readonly", width=6)
        unit_cb.pack(side="left")
        unit_cb.bind("<<ComboboxSelected>>", lambda _: self._on_units_changed())

        cj_row = tk.Frame(f, bg=BG)
        cj_row.pack(fill="x", pady=2)
        tk.Label(cj_row, text="Cold jct:", bg=BG, fg=WHITE,
                 width=8, anchor="w").pack(side="left")
        self._cj_src_var = tk.StringVar(value=CJ_SOURCES[0])
        cj_cb = ttk.Combobox(cj_row, textvariable=self._cj_src_var,
                             values=CJ_SOURCES, state="readonly", width=14)
        cj_cb.pack(side="left")
        cj_cb.bind("<<ComboboxSelected>>",
                   lambda _: self._send(f"!TCCJSRC,{CJ_SOURCES.index(self._cj_src_var.get())}"))

        vcmd_f2 = self.register(self._validate_float)
        fixed_row = tk.Frame(f, bg=BG)
        fixed_row.pack(fill="x", pady=1)
        tk.Label(fixed_row, text="Fixed:", bg=BG, fg=DIM,
                 width=8, anchor="w").pack(side="left")
        self._cj_val_var = tk.StringVar(value="25.0")
        e = tk.Entry(fixed_row, textvariable=self._cj_val_var, width=7, bg=PANEL,
                     fg=WHITE, insertbackground=WHITE, relief="flat",
                     validate="key", validatecommand=(vcmd_f2, "%P"))
        e.pack(side="left", padx=2)
        e.bind("<Return>", lambda _: self._send_float("!TCCJVAL", self._cj_val_var))
        tk.Label(fixed_row, text="°C", bg=BG, fg=DIM).pack(side="left")

        trim_row = tk.Frame(f, bg=BG)
        trim_row.pack(fill="x", pady=1)
        tk.Label(trim_row, text="Trim:", bg=BG, fg=DIM,
                 width=8, anchor="w").pack(side="left")
        self._cj_trim_var = tk.StringVar(value="0.0")
        e = tk.Entry(trim_row, textvariable=self._cj_trim_var, width=7, bg=PANEL,
                     fg=WHITE, insertbackground=WHITE, relief="flat",
                     validate="key", validatecommand=(vcmd_f2, "%P"))
        e.pack(side="left", padx=2)
        e.bind("<Return>", lambda _: self._send_float("!TCCJTRIM", self._cj_trim_var))
        tk.Label(trim_row, text="°C", bg=BG, fg=DIM).pack(side="left")

        tc_btns = tk.Frame(f, bg=BG)
        tc_btns.pack(fill="x", pady=(3, 0))
        tk.Button(tc_btns, text="Zero TC", bg=PANEL, fg=WHITE, relief="flat",
                  command=lambda: self._send("!TCZERO")).pack(side="left")
        tk.Button(tc_btns, text="Clear zero", bg=PANEL, fg=DIM, relief="flat",
                  command=lambda: self._send("!TCOFF,0")).pack(side="left", padx=4)

        # ── EMA filter ────────────────────────────────────────────────────────
        self._lbl(f, "EMA filter")

        trace_row = tk.Frame(f, bg=BG)
        trace_row.pack(fill="x", pady=2)
        tk.Label(trace_row, text="Traces:", bg=BG, fg=WHITE,
                 width=8, anchor="w").pack(side="left")
        self._trace_var = tk.StringVar(value=TRACE_MODES[0])
        trace_cb = ttk.Combobox(trace_row, textvariable=self._trace_var,
                                values=TRACE_MODES, state="readonly", width=14)
        trace_cb.pack(side="left")
        trace_cb.bind("<<ComboboxSelected>>", lambda _: self._on_units_changed())

        ema_row = tk.Frame(f, bg=BG)
        ema_row.pack(fill="x", pady=2)
        tk.Label(ema_row, text="Filter:", bg=BG, fg=WHITE,
                 width=8, anchor="w").pack(side="left")
        self._ema_mode_var = tk.StringVar(value=EMA_MODES[0])
        ema_cb = ttk.Combobox(ema_row, textvariable=self._ema_mode_var,
                              values=EMA_MODES, state="readonly", width=14)
        ema_cb.pack(side="left")
        ema_cb.bind("<<ComboboxSelected>>", lambda _: self._on_ema_mode_changed())

        val_row = tk.Frame(f, bg=BG)
        val_row.pack(fill="x", pady=1)
        tk.Label(val_row, text="τ / α:", bg=BG, fg=DIM,
                 width=8, anchor="w").pack(side="left")
        self._ema_val_var = tk.StringVar(value="2.0")
        ema_entry = tk.Entry(val_row, textvariable=self._ema_val_var, width=7,
                             bg=PANEL, fg=WHITE, insertbackground=WHITE,
                             relief="flat", validate="key",
                             validatecommand=(vcmd_f2, "%P"))
        ema_entry.pack(side="left", padx=2)
        ema_entry.bind("<Return>",   lambda _: self._ema_recompute())
        ema_entry.bind("<FocusOut>", lambda _: self._ema_recompute())
        self._ema_hint = tk.Label(val_row, text="s", bg=BG, fg=DIM)
        self._ema_hint.pack(side="left")

        self._burnout_var = tk.BooleanVar(value=False)
        tk.Checkbutton(f, text="Burn-out current (open TC detect)",
                       variable=self._burnout_var, bg=BG, fg=WHITE,
                       selectcolor=PANEL, activebackground=BG,
                       command=lambda: self._send_bool("!BURNOUT", self._burnout_var)
                       ).pack(anchor="w")

        tk.Label(f, bg=BG).pack(pady=4)
        self._stream_btn = tk.Button(f, text="▶  Start Stream",
                                     bg="#224422", fg=ACCENT,
                                     relief="flat", height=2,
                                     command=self._toggle_stream)
        self._stream_btn.pack(fill="x")

        # ── plot display controls ─────────────────────────────────────────────
        self._sep(f)
        self._lbl(f, "Plot Display")

        hist_row = tk.Frame(f, bg=BG)
        hist_row.pack(fill="x", pady=2)
        tk.Label(hist_row, text="History:", bg=BG, fg=WHITE,
                 width=8, anchor="w").pack(side="left")
        self._history_var = tk.StringVar(value="5.0")
        vcmd_pos = self.register(lambda s: s == "" or s.replace(".", "", 1).isdigit())
        tk.Entry(hist_row, textvariable=self._history_var,
                 width=6, bg=PANEL, fg=WHITE, insertbackground=WHITE,
                 relief="flat", validate="key", validatecommand=(vcmd_pos, "%P")
                 ).pack(side="left", padx=4)
        tk.Label(hist_row, text="s", bg=BG, fg=DIM).pack(side="left")

        self._autoscale_var = tk.BooleanVar(value=True)
        tk.Checkbutton(f, text="Auto Scale Y", variable=self._autoscale_var,
                       bg=BG, fg=WHITE, selectcolor=PANEL, activebackground=BG,
                       command=self._on_autoscale_toggle).pack(anchor="w")

        ylim_row = tk.Frame(f, bg=BG)
        ylim_row.pack(fill="x")
        tk.Label(ylim_row, text="Min:", bg=BG, fg=DIM).pack(side="left")
        self._ymin_var = tk.StringVar(value="-1.0")
        vcmd_f = self.register(self._validate_float)
        self._ymin_entry = tk.Entry(ylim_row, textvariable=self._ymin_var,
                 width=7, bg=PANEL, fg=WHITE, insertbackground=WHITE,
                 relief="flat", validate="key", validatecommand=(vcmd_f, "%P"),
                 state="disabled")
        self._ymin_entry.pack(side="left", padx=(2, 6))
        tk.Label(ylim_row, text="Max:", bg=BG, fg=DIM).pack(side="left")
        self._ymax_var = tk.StringVar(value="1.0")
        self._ymax_entry = tk.Entry(ylim_row, textvariable=self._ymax_var,
                 width=7, bg=PANEL, fg=WHITE, insertbackground=WHITE,
                 relief="flat", validate="key", validatecommand=(vcmd_f, "%P"),
                 state="disabled")
        self._ymax_entry.pack(side="left", padx=(2, 0))

        # ── burst section ─────────────────────────────────────────────────────
        self._sep(f)
        self._lbl(f, "Burst Capture", color=AMBER)

        dur_row = tk.Frame(f, bg=BG)
        dur_row.pack(fill="x", pady=2)
        tk.Label(dur_row, text="Duration:", bg=BG, fg=WHITE).pack(side="left")
        self._burst_dur_var = tk.StringVar(value="1.0")
        vcmd = self.register(lambda s: s == "" or s.replace(".", "", 1).isdigit())
        tk.Entry(dur_row, textvariable=self._burst_dur_var,
                 width=6, bg=PANEL, fg=WHITE, insertbackground=WHITE,
                 relief="flat", validate="key", validatecommand=(vcmd, "%P")
                 ).pack(side="left", padx=4)
        tk.Label(dur_row, text="s", bg=BG, fg=DIM).pack(side="left")

        self._burst_btn = tk.Button(f, text="⚡  Trigger Burst",
                                    bg="#332200", fg=AMBER,
                                    relief="flat", height=2,
                                    command=self._trigger_burst)
        self._burst_btn.pack(fill="x", pady=(4, 2))

        self._burst_status = tk.Label(f, text="Ready", bg=BG, fg=DIM,
                                       anchor="w", font=("Courier New", 8))
        self._burst_status.pack(fill="x")

        # ── device config readback ────────────────────────────────────────────
        self._sep(f)
        self._lbl(f, "Device Config")
        self._cfg_lbl = tk.Label(f, text="—", bg=PANEL, fg=DIM,
                                  justify="left", anchor="w",
                                  wraplength=230, padx=4, pady=4)
        self._cfg_lbl.pack(fill="x")

    def _build_display(self, f):
        # ── live meter ────────────────────────────────────────────────────────
        meter = tk.Frame(f, bg=PANEL)
        meter.pack(fill="x", pady=(0, 4))

        # Fixed character width: the reading is padded to METER_W columns so
        # 2-digit and 4-digit values occupy the same space and nothing packed
        # after this label slides around as the temperature changes.
        self._volt_lbl = tk.Label(meter, text=self._meter_text("-------", "V"),
                                   bg=PANEL, fg=ACCENT, width=METER_W,
                                   anchor="w", font=("Courier New", 36, "bold"))
        self._volt_lbl.pack(side="left", padx=16, pady=8)

        stats = tk.Frame(meter, bg=PANEL)
        stats.pack(side="left", padx=16)
        self._raw_lbl  = self._stat_row(stats, "Raw")
        self._sps_lbl  = self._stat_row(stats, "Actual SPS")
        self._nom_lbl  = self._stat_row(stats, "Nominal SPS")
        self._mode_lbl = self._stat_row(stats, "Mode")

        tc_stats = tk.Frame(meter, bg=PANEL)
        tc_stats.pack(side="left", padx=16)
        self._cj_lbl     = self._stat_row(tc_stats, "Cold Jct")
        self._tcmv_lbl   = self._stat_row(tc_stats, "TC EMF")
        self._tcalt_lbl  = self._stat_row(tc_stats, "Hot Jct")
        self._ema_lbl    = self._stat_row(tc_stats, "EMA")
        self._tcflag_lbl = self._stat_row(tc_stats, "TC Status")

        # ── waveform plot ─────────────────────────────────────────────────────
        fig = Figure(figsize=(6, 3), facecolor=BG)
        self._ax = fig.add_subplot(111)
        self._ax.set_facecolor(PANEL)
        self._ax.tick_params(colors=DIM)
        for sp in self._ax.spines.values():
            sp.set_edgecolor(SEP)
        self._ax.set_xlabel("Time (s)", color=DIM)
        self._ax.set_ylabel("Voltage (V)", color=DIM)
        self._stream_line, = self._ax.plot([], [], color=ACCENT, linewidth=0.8,
                                            label="stream")
        self._burst_line,  = self._ax.plot([], [], color=AMBER,  linewidth=0.8,
                                            label="burst")
        # one EMA line serves both, since the plot shows stream or burst, never both
        self._ema_line,    = self._ax.plot([], [], color=CYAN,   linewidth=1.4,
                                            label="EMA")
        self._plot_title = self._ax.set_title("", color=DIM, fontsize=8, pad=3)

        canvas = FigureCanvasTkAgg(fig, master=f)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas = canvas

        # ── burst statistics bar ──────────────────────────────────────────────
        self._stats_frame = tk.Frame(f, bg=PANEL)
        self._stats_frame.pack(fill="x", pady=(2, 2))

        self._bstat_count = self._bstat(self._stats_frame, "Count")
        self._bstat_sps   = self._bstat(self._stats_frame, "SPS")
        self._bstat_mean  = self._bstat(self._stats_frame, "Mean")
        self._bstat_std   = self._bstat(self._stats_frame, "Std Dev")
        self._bstat_min   = self._bstat(self._stats_frame, "Min")
        self._bstat_max   = self._bstat(self._stats_frame, "Max")

        # ── console ───────────────────────────────────────────────────────────
        self._console = tk.Text(f, height=5, bg="#111111", fg=DIM,
                                font=("Courier New", 8), state="disabled",
                                relief="flat")
        self._console.pack(fill="x")

    def _stat_row(self, parent, title, width=21):
        row = tk.Frame(parent, bg=PANEL)
        row.pack(anchor="w")
        tk.Label(row, text=f"{title}:", bg=PANEL, fg=DIM,
                 width=12, anchor="w").pack(side="left")
        # width is in characters and clips anything longer, so it has to cover
        # the widest value each row can show ("25.00 °C / 77.00 °F")
        lbl = tk.Label(row, text="—", bg=PANEL, fg=WHITE, anchor="w", width=width)
        lbl.pack(side="left")
        return lbl

    @staticmethod
    def _meter_text(value: str, unit: str = "") -> str:
        """Pad a reading into the meter's fixed column layout."""
        return f"{value:>{METER_VALUE_W}} {unit:<{METER_UNIT_W}}"

    def _set_meter(self, value: str, unit: str = "", color: str = ACCENT):
        self._volt_lbl.config(text=self._meter_text(value, unit), fg=color)

    def _bstat(self, parent, title):
        col = tk.Frame(parent, bg=PANEL)
        col.pack(side="left", expand=True, padx=8, pady=4)
        tk.Label(col, text=title, bg=PANEL, fg=DIM,
                 font=("Courier New", 8)).pack()
        lbl = tk.Label(col, text="—", bg=PANEL, fg=AMBER,
                       font=("Courier New", 9, "bold"))
        lbl.pack()
        return lbl

    # ── serial helpers ────────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_cb["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])

    def _toggle_connect(self):
        if self._ser and self._ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self._port_var.get()
        baud = int(self._baud_var.get())
        if not port:
            messagebox.showerror("Error", "Select a port first.")
            return
        try:
            self._ser = serial.Serial(port, baud, timeout=0.05)
            time.sleep(1.5)
            self._ser.reset_input_buffer()
            self._running = True
            self._rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
            self._rx_thread.start()
            self._connect_btn.config(text="Disconnect", bg="#552222", fg=RED)
            self._status_lbl.config(text=f"Connected  {port}", fg=ACCENT)
            self._send("!CFG")
        except serial.SerialException as e:
            messagebox.showerror("Serial error", str(e))

    def _disconnect(self):
        self._streaming = False
        self._running = False
        if self._ser:
            try:
                self._send("!STOP")
                time.sleep(0.1)
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self._connect_btn.config(text="Connect", bg="#225522", fg=ACCENT)
        self._status_lbl.config(text="Disconnected", fg=RED)
        self._stream_btn.config(text="▶  Start Stream", bg="#224422")
        self._streaming = False

    def _send(self, msg: str):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write((msg + "\n").encode())
            except serial.SerialException:
                pass

    def _send_indexed(self, cmd, labels, var):
        self._send(f"{cmd},{labels.index(var.get())}")

    def _send_bool(self, cmd, var):
        self._send(f"{cmd},{1 if var.get() else 0}")

    def _toggle_stream(self):
        if not self._ser or not self._ser.is_open:
            messagebox.showinfo("Not connected", "Connect to a port first.")
            return
        if self._streaming:
            self._send("!STOP")
            self._streaming = False
            self._stream_btn.config(text="▶  Start Stream", bg="#224422")
        else:
            self._send("!START")
            self._streaming = True
            self._t0 = time.monotonic()
            self._voltages.clear()
            self._timestamps.clear()
            self._tc_c.clear()
            self._tc_mv.clear()
            self._ema_reset()
            self._stream_btn.config(text="■  Stop Stream", bg="#552200")

    def _trigger_burst(self):
        if not self._ser or not self._ser.is_open:
            messagebox.showinfo("Not connected", "Connect to a port first.")
            return
        try:
            dur = float(self._burst_dur_var.get() or "1.0")
        except ValueError:
            dur = 1.0
        dur = max(0.05, min(dur, 60.0))

        self._in_burst    = True
        self._burst_raws  = []
        self._burst_volts = []
        self._burst_total = 0
        self._burst_btn.config(state="disabled")
        self._burst_status.config(text="Waiting for capture…", fg=AMBER)
        self._send(f"!BURST,{dur:.3f}")

    # ── RX thread ─────────────────────────────────────────────────────────────

    def _rx_worker(self):
        buf = b""
        while self._running:
            try:
                chunk = self._ser.read(256)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._rx_queue.put(line.decode(errors="replace").strip())
            except serial.SerialException:
                break

    # ── main-thread queue poll ────────────────────────────────────────────────

    def _poll_queue(self):
        try:
            for _ in range(60):
                line = self._rx_queue.get_nowait()
                try:
                    self._handle_line(line)
                except Exception as e:
                    self._log(f"!! GUI error: {e}")
        except queue.Empty:
            pass
        finally:
            self.after(20, self._poll_queue)

    def _handle_line(self, line: str):
        # ── burst sample lines: don't log, update counter instead ────────────
        if line.startswith("$BD,"):
            if self._in_burst:
                try:
                    raw = int(line[4:])
                    self._burst_raws.append(raw)
                    self._burst_volts.append(raw * self._burst_lsb_v)
                    n = len(self._burst_raws)
                    if n % 50 == 0 or n == self._burst_total:
                        self._burst_status.config(
                            text=f"Receiving: {n}/{self._burst_total}", fg=AMBER)
                except ValueError:
                    pass
            return   # never log $BD to console

        self._log(line)

        if line.startswith("$ADC,"):
            parts = line[5:].split(",")
            if len(parts) < 8:
                return
            raw     = int(parts[0])
            volts   = float(parts[1])
            mux_i   = int(parts[2])
            gain_i  = int(parts[3])
            pga     = int(parts[4])
            rate_i  = int(parts[5])
            turbo   = int(parts[6])
            act_sps = float(parts[7])

            is_se   = 8 <= mux_i <= 11
            nom_sps = RATE_NOMINAL[rate_i] * (2 if turbo else 1)

            self._set_meter(f"{volts:+.6f}", "V")
            self._raw_lbl.config(text=f"0x{raw & 0xFFFFFF:06X}  ({raw})")
            self._sps_lbl.config(text=f"{act_sps:.1f}")
            self._nom_lbl.config(text=str(nom_sps))
            self._mode_lbl.config(text="SE" if is_se else "DIFF")

            # sync controls from live board state — catches any GUI/board desync
            if self._mux_var.get()  != MUX_LABELS[mux_i]:
                self._mux_var.set(MUX_LABELS[mux_i])
            if self._gain_var.get() != GAIN_LABELS[gain_i]:
                self._gain_var.set(GAIN_LABELS[gain_i])
            if self._rate_var.get() != RATE_LABELS[rate_i]:
                self._rate_var.set(RATE_LABELS[rate_i])
            if bool(self._pga_var.get())   != bool(pga):
                self._pga_var.set(bool(pga))
            if bool(self._turbo_var.get()) != bool(turbo):
                self._turbo_var.set(bool(turbo))

            now = time.monotonic() - self._t0
            self._voltages.append(volts)
            self._timestamps.append(now)
            self._update_stream_plot()

        elif line.startswith("$TC,"):
            parts = line[4:].split(",")
            if len(parts) < 9:
                return
            raw     = int(parts[0])
            v_tc    = float(parts[1])
            cj_c    = self._to_float(parts[2])
            hot_c   = self._to_float(parts[3])
            gain_i  = int(parts[4])
            rate_i  = int(parts[5])
            turbo   = int(parts[6])
            act_sps = float(parts[7])
            flags   = int(parts[8])

            self._last_cj_c = cj_c
            unit = self._tc_unit_var.get()
            mv = v_tc * 1000.0

            color = RED if flags else ACCENT
            if unit == "mV":
                self._set_meter(f"{mv:+.4f}", "mV", color)
            elif unit == "°F":
                self._set_meter("-------" if hot_c != hot_c
                                else f"{c_to_f(hot_c):+.2f}", "°F", color)
            else:
                self._set_meter("-------" if hot_c != hot_c
                                else f"{hot_c:+.2f}", "°C", color)

            self._raw_lbl.config(text=f"0x{raw & 0xFFFFFF:06X}  ({raw})")
            self._sps_lbl.config(text=f"{act_sps:.1f}")
            self._nom_lbl.config(text=str(RATE_NOMINAL[rate_i] * (2 if turbo else 1)))
            self._mode_lbl.config(text="TC (type K)")
            self._cj_lbl.config(
                text="—" if cj_c != cj_c else f"{cj_c:.3f} °C / {c_to_f(cj_c):.2f} °F")
            self._tcmv_lbl.config(text=f"{mv:+.4f} mV")
            # the unit selector picks one for the meter; show the other here
            if hot_c != hot_c:
                self._tcalt_lbl.config(text="—")
            elif unit == "°F":
                self._tcalt_lbl.config(text=f"{hot_c:+.2f} °C")
            else:
                self._tcalt_lbl.config(text=f"{c_to_f(hot_c):+.2f} °F")
            self._tcflag_lbl.config(text=self._tc_flag_text(flags),
                                    fg=RED if flags else WHITE)

            if self._gain_var.get() != GAIN_LABELS[gain_i]:
                self._gain_var.set(GAIN_LABELS[gain_i])
            if self._rate_var.get() != RATE_LABELS[rate_i]:
                self._rate_var.set(RATE_LABELS[rate_i])

            now = time.monotonic() - self._t0
            self._timestamps.append(now)
            self._tc_c.append(hot_c)
            self._tc_mv.append(mv)
            self._voltages.append(v_tc)
            self._ema_push(now, hot_c, mv)
            ema_now = self._ema_display_series()[-1] if self._ema_c else float("nan")
            self._ema_lbl.config(
                text="—" if ema_now != ema_now else f"{ema_now:+.4f} {unit}")
            self._update_stream_plot()

        elif line.startswith("$TCCFG,"):
            self._apply_tccfg_readback(line[7:].split(","))

        elif line.startswith("$TEMP,"):
            parts = line[6:].split(",")
            if len(parts) < 4:
                return
            self._set_meter(f"{float(parts[0]):.4f}", "°C")
            self._sps_lbl.config(text=f"{float(parts[3]):.1f}")
            self._mode_lbl.config(text="TEMP")

        elif line.startswith("$BURST_START,"):
            parts = line[13:].split(",")
            if len(parts) >= 3:
                self._burst_total  = int(parts[0])
                self._burst_sps    = float(parts[1])
                self._burst_lsb_v  = float(parts[2])
                self._burst_status.config(
                    text=f"Receiving: 0/{self._burst_total}", fg=AMBER)

        elif line.startswith("$BURST_END,"):
            parts = line[11:].split(",")
            if len(parts) >= 4:
                self._finish_burst(float(parts[0]), float(parts[1]),
                                   float(parts[2]), float(parts[3]))

        elif line.startswith("$CFG,"):
            self._apply_cfg_readback(line[5:].split(","))

        elif line.startswith("$READY"):
            if self._in_burst:
                self._in_burst = False
                self._burst_btn.config(state="normal")
                self._burst_status.config(text="Board restarted", fg=RED)
            if self._streaming:
                self._streaming = False
                self._stream_btn.config(text="▶  Start Stream", bg="#224422")

        elif line.startswith("$ERR,"):
            self._set_meter("ERROR", "", RED)
            if self._in_burst:
                self._in_burst = False
                self._burst_btn.config(state="normal")
                self._burst_status.config(text="Error — see console", fg=RED)

    def _finish_burst(self, mean_v, min_v, max_v, std_v):
        self._in_burst = False
        self._burst_btn.config(state="normal")

        count = len(self._burst_volts)
        self._burst_status.config(
            text=f"Done — {count} samples @ {self._burst_sps:.1f} SPS", fg=ACCENT)

        self._bstat_count.config(text=str(count))
        self._bstat_sps.config(text=f"{self._burst_sps:.1f}")
        if self._tc_mode:
            # the board's stats are in volts; temperature is not a linear
            # function of them, so recompute in the displayed unit
            ys = [y for y in self._burst_series() if y == y]
            unit = self._tc_unit_var.get()
            if ys:
                mean = sum(ys) / len(ys)
                std  = (sum((y - mean) ** 2 for y in ys) / len(ys)) ** 0.5
                self._bstat_mean.config(text=f"{mean:+.4f} {unit}")
                self._bstat_std.config(text=f"{std:.4f} {unit}")
                self._bstat_min.config(text=f"{min(ys):+.4f} {unit}")
                self._bstat_max.config(text=f"{max(ys):+.4f} {unit}")
            else:
                for lbl in (self._bstat_mean, self._bstat_std,
                            self._bstat_min, self._bstat_max):
                    lbl.config(text="—")
        else:
            self._bstat_mean.config(text=f"{mean_v:+.6f} V")
            self._bstat_std.config(text=f"{std_v:.6f} V")
            self._bstat_min.config(text=f"{min_v:+.6f} V")
            self._bstat_max.config(text=f"{max_v:+.6f} V")

        self._update_burst_plot()

    def _apply_cfg_readback(self, parts):
        if len(parts) < 6:
            return
        mux_i  = int(parts[0])
        gain_i = int(parts[1])
        pga    = int(parts[2])
        rate_i = int(parts[3])
        turbo  = int(parts[4])
        temp   = int(parts[5])

        self._mux_var.set(MUX_LABELS[mux_i])
        self._gain_var.set(GAIN_LABELS[gain_i])
        self._rate_var.set(RATE_LABELS[rate_i])
        self._pga_var.set(bool(pga))
        self._turbo_var.set(bool(turbo))
        self._temp_var.set(bool(temp))

        self._cfg_lbl.config(
            text=(f"MUX: {MUX_LABELS[mux_i]}\n"
                  f"Gain: {GAIN_LABELS[gain_i]}  PGA: {'On' if pga else 'Off'}\n"
                  f"Rate: {RATE_LABELS[rate_i]}  Turbo: {'On' if turbo else 'Off'}\n"
                  f"Temp: {'On' if temp else 'Off'}"),
            fg=WHITE)

    # ── thermocouple helpers ──────────────────────────────────────────────────

    @staticmethod
    def _to_float(text: str) -> float:
        try:
            return float(text)
        except ValueError:
            return float("nan")     # the sketch prints nan for "no reading"

    @staticmethod
    def _tc_flag_text(flags: int) -> str:
        if not flags:
            return "OK"
        names = []
        if flags & 0x01:
            names.append("RANGE")
        if flags & 0x02:
            names.append("CJ STALE")
        if flags & 0x04:
            names.append("SAT/OPEN")
        return " ".join(names)

    def _send_float(self, cmd: str, var: tk.StringVar):
        try:
            self._send(f"{cmd},{float(var.get()):.4f}")
        except ValueError:
            pass

    def _on_tc_toggle(self):
        self._set_tc_mode(self._tc_var.get())
        self._send_bool("!TC", self._tc_var)

    def _set_tc_mode(self, on: bool):
        """Switch the plot between the voltage and thermocouple series."""
        if on == self._tc_mode:
            return
        self._tc_mode = on
        self._voltages.clear()
        self._timestamps.clear()
        self._tc_c.clear()
        self._tc_mv.clear()
        self._ema_reset()
        self._t0 = time.monotonic()
        self._on_units_changed()

    def _on_units_changed(self):
        self._update_stream_plot(force=True)
        if self._burst_volts:
            self._update_burst_plot()

    # ── EMA filter ────────────────────────────────────────────────────────────

    def _on_ema_mode_changed(self):
        by_time = self._ema_mode_var.get() == EMA_MODES[0]
        self._ema_hint.config(text="s" if by_time else "")
        self._ema_val_var.set("2.0" if by_time else "0.1")
        self._ema_recompute()

    def _ema_setting(self) -> float:
        try:
            return float(self._ema_val_var.get())
        except ValueError:
            return 2.0 if self._ema_mode_var.get() == EMA_MODES[0] else 0.1

    def _ema_alpha(self, dt: float) -> float:
        """Weight for one sample.

        In time-constant mode the weight is derived from the actual gap between
        samples — a = 1 - exp(-dt/tau) — so the filter keeps the same physical
        response when the data rate changes, and a long gap simply lets it
        re-converge instead of dragging a stale value forward.
        """
        if self._ema_mode_var.get() == EMA_MODES[1]:
            return min(max(self._ema_setting(), 0.0), 1.0)
        tau = self._ema_setting()
        if tau <= 0.0:
            return 1.0
        return 1.0 - math.exp(-max(dt, 0.0) / tau)

    def _ema_reset(self):
        self._ema_c.clear()
        self._ema_mv.clear()
        self._ema_state_c = None
        self._ema_state_mv = None
        self._ema_last_t = None

    def _ema_push(self, t: float, c_val: float, mv_val: float):
        """Advance the filter by one sample and record its output."""
        if c_val != c_val:          # out-of-range reading: hold state, gap the trace
            self._ema_c.append(float("nan"))
            self._ema_mv.append(float("nan"))
            return
        dt = 0.0 if self._ema_last_t is None else max(t - self._ema_last_t, 0.0)
        alpha = self._ema_alpha(dt)
        if self._ema_state_c is None:
            self._ema_state_c, self._ema_state_mv = c_val, mv_val
        else:
            self._ema_state_c  += alpha * (c_val  - self._ema_state_c)
            self._ema_state_mv += alpha * (mv_val - self._ema_state_mv)
        self._ema_last_t = t
        self._ema_c.append(self._ema_state_c)
        self._ema_mv.append(self._ema_state_mv)

    def _ema_recompute(self):
        """Re-run the filter over everything buffered, then redraw.

        This is what makes the time constant tunable: change it and the whole
        visible history is re-filtered, rather than having to wait for new
        samples to work through the old setting.
        """
        times, temps, mvs = (list(self._timestamps), list(self._tc_c),
                             list(self._tc_mv))
        self._ema_reset()
        for t, c_val, mv_val in zip(times, temps, mvs):
            self._ema_push(t, c_val, mv_val)
        self._on_units_changed()

    def _ema_offline(self, ys, dt: float):
        """Filter an already-captured series (burst) with uniform spacing."""
        alpha = self._ema_alpha(dt)
        out, state = [], None
        for y in ys:
            if y != y:
                out.append(float("nan"))
                continue
            state = y if state is None else state + alpha * (y - state)
            out.append(state)
        return out

    def _show_instant(self) -> bool:
        return self._trace_var.get() != TRACE_MODES[2]

    def _show_ema(self) -> bool:
        return self._tc_mode and self._trace_var.get() != TRACE_MODES[1]

    def _ema_display_series(self):
        """The EMA in the unit on screen — see the note in __init__ on why the
        °C and mV filters are kept separately."""
        return self._tc_convert(self._ema_c, self._ema_mv)

    def _y_label(self) -> str:
        if not self._tc_mode:
            return "Voltage (V)"
        unit = self._tc_unit_var.get()
        return {"°C": "Temperature (°C)", "°F": "Temperature (°F)",
                "mV": "Thermocouple EMF (mV)"}[unit]

    def _tc_convert(self, temps_c, mvs):
        """Pick the display series for the selected unit."""
        unit = self._tc_unit_var.get()
        if unit == "mV":
            return list(mvs)
        if unit == "°F":
            return [c_to_f(t) if t == t else float("nan") for t in temps_c]
        return list(temps_c)

    def _apply_tccfg_readback(self, parts):
        if len(parts) < 7:
            return
        tc_on   = int(parts[0])
        cj_src  = int(parts[1])
        cj_val  = float(parts[2])
        cj_trim = float(parts[3])
        burnout = int(parts[6])

        self._tc_var.set(bool(tc_on))
        self._set_tc_mode(bool(tc_on))
        self._cj_src_var.set(CJ_SOURCES[1 if cj_src else 0])
        self._cj_val_var.set(f"{cj_val:g}")
        self._cj_trim_var.set(f"{cj_trim:g}")
        self._burnout_var.set(bool(burnout))

    # ── plot helpers ──────────────────────────────────────────────────────────

    def _validate_float(self, s: str) -> bool:
        if s in ("", "-", "+", ".", "-.", "+."):
            return True
        try:
            float(s)
            return True
        except ValueError:
            return False

    def _on_autoscale_toggle(self):
        state = "disabled" if self._autoscale_var.get() else "normal"
        self._ymin_entry.config(state=state)
        self._ymax_entry.config(state=state)

    def _apply_y_range(self, visible_ys=None):
        if visible_ys:
            # out-of-range TC samples come through as NaN — ignore them here
            visible_ys = [y for y in visible_ys if y == y and abs(y) != float("inf")]
        if not hasattr(self, "_autoscale_var") or self._autoscale_var.get():
            if visible_ys:
                lo, hi = min(visible_ys), max(visible_ys)
                pad = (hi - lo) * 0.05 if hi != lo else 0.01
                self._ax.set_ylim(lo - pad, hi + pad)
            else:
                self._ax.autoscale_view()
        else:
            try:
                ymin = float(self._ymin_var.get())
                ymax = float(self._ymax_var.get())
                if ymin < ymax:
                    self._ax.set_ylim(ymin, ymax)
                else:
                    self._ax.autoscale_view()
            except ValueError:
                self._ax.autoscale_view()

    @staticmethod
    def _set_line(line, xs, ys):
        """Set a trace, blanking x as well when there is no y — matplotlib
        rejects mismatched lengths at draw time, not at set_data time."""
        if len(ys):
            line.set_data(xs, ys)
        else:
            line.set_data([], [])

    def _update_legend(self):
        """Only worth the space when both traces are on screen."""
        show = self._show_ema() and self._show_instant()
        existing = self._ax.get_legend()
        if show and existing is None:
            self._ax.legend(loc="upper left", fontsize=8, facecolor=PANEL,
                            edgecolor=SEP, labelcolor=DIM)
        elif not show and existing is not None:
            existing.remove()

    def _update_stream_plot(self, force: bool = False):
        now = time.monotonic()
        if not force and now - self._last_stream_plot < 1 / 30:
            return
        self._last_stream_plot = now

        self._ax.set_ylabel(self._y_label(), color=DIM)
        if not self._timestamps:
            self._stream_line.set_data([], [])
            self._ema_line.set_data([], [])
            self._canvas.draw_idle()
            return
        ts = list(self._timestamps)
        vs = (self._tc_convert(self._tc_c, self._tc_mv) if self._tc_mode
              else list(self._voltages))
        es = self._ema_display_series() if self._show_ema() else []

        # rolling history window
        try:
            hist_s = max(float(self._history_var.get() or "5"), 0.1)
        except (ValueError, AttributeError):
            hist_s = 5.0
        t_end  = ts[-1]
        cutoff = t_end - hist_s
        idx = bisect.bisect_left(ts, cutoff)
        xs = ts[idx:]
        ys = vs[idx:]

        # the EMA is filtered before this, so decimating it here only thins
        # the drawn line rather than changing the filter's response
        fs = es[idx:] if es else []

        # downsample to keep render fast
        if len(xs) > MAX_DISPLAY_PTS:
            step = max(len(xs) // MAX_DISPLAY_PTS, 1)
            xs = xs[::step]
            ys = ys[::step]
            fs = fs[::step]

        self._set_line(self._stream_line, xs, ys if self._show_instant() else [])
        self._set_line(self._ema_line, xs, fs)
        self._burst_line.set_data([], [])
        self._plot_title.set_text("Streaming")
        self._plot_title.set_color(DIM)
        self._ax.set_xlabel("Time (s)", color=DIM)
        self._ax.set_xlim(cutoff, t_end)   # explicit x-range keeps rolling effect
        self._apply_y_range((ys if self._show_instant() else []) + fs)
        self._update_legend()
        self._canvas.draw_idle()

    def _burst_series(self):
        """Burst samples in the units currently being displayed.

        The board sends a burst as raw counts plus volts-per-LSB, so in TC mode
        the conversion happens here, against the most recent cold junction.
        """
        if not self._tc_mode:
            return list(self._burst_volts)
        temps = [tk_hot_temp(v, self._last_cj_c) for v in self._burst_volts]
        return self._tc_convert(temps, [v * 1000.0 for v in self._burst_volts])

    def _update_burst_plot(self):
        if not self._burst_volts:
            return
        ys = self._burst_series()
        count = len(ys)
        # x-axis: time in seconds
        dt = 1.0 / self._burst_sps if self._burst_sps > 0 else 1.0
        xs = [i * dt for i in range(count)]
        fs = self._ema_offline(ys, dt) if self._show_ema() else []
        self._set_line(self._burst_line, xs, ys if self._show_instant() else [])
        self._set_line(self._ema_line, xs, fs)
        self._stream_line.set_data([], [])
        self._plot_title.set_text(f"Burst  ({count} samples @ {self._burst_sps:.1f} SPS)")
        self._plot_title.set_color(AMBER)
        self._ax.set_xlabel("Time (s)", color=DIM)
        self._ax.set_ylabel(self._y_label(), color=DIM)
        self._ax.relim()
        self._ax.set_xlim(0, xs[-1] if count > 1 else 1.0)
        self._apply_y_range((ys if self._show_instant() else []) + fs)
        self._update_legend()
        self._canvas.draw_idle()

    def _log(self, text: str):
        self._console.config(state="normal")
        self._console.insert("end", text + "\n")
        self._console.see("end")
        lines = int(self._console.index("end-1c").split(".")[0])
        if lines > 200:
            self._console.delete("1.0", f"{lines - 200}.0")
        self._console.config(state="disabled")

    def on_close(self):
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
