"""
ADS122C04 AC Power Monitor GUI

Two ways to look at the same front end:
  Trends   — the AC numbers the board computes, one point per measurement
             window: Vrms, Irms, real power, power factor.
  Waveform — the raw conversions behind them, so the shape of a load can be
             seen. Captured on demand, optionally on repeat.

Plus a load relay on the board's D5, driven from the button in the top bar.

Requires: pip install pyserial matplotlib
"""

import io
import json
import math
import os
import queue
import threading
import time
from collections import deque

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ── lookup tables (must match the sketch order) ───────────────────────────────
GAIN_LABELS = ["1×", "2×", "4×", "8×", "16×", "32×", "64×", "128×"]
RATE_LABELS = ["20/40 SPS", "45/90 SPS", "90/180 SPS", "175/350 SPS",
               "330/660 SPS", "600/1200 SPS", "1000/2000 SPS"]
RATE_NOMINAL = [20, 45, 90, 175, 330, 600, 1000]
WAVE_CH_LABELS = ["Voltage (AIN0−AIN1)", "Current (AIN2−AIN3)",
                  "Both, interleaved"]
I2C_LABELS = ["100 kHz", "400 kHz", "1 MHz"]
I2C_VALUES = [100000, 400000, 1000000]

# $PWR flag bits, in the sketch's order
FLAG_NAMES = [(0x01, "NO ZERO X"), (0x02, "V CLIP"), (0x04, "I CLIP"),
              (0x08, "TRIPPED"),   (0x10, "TRUNCATED"), (0x20, "ADC ERR")]

MAX_TREND_PTS = 4000     # points drawn per trend trace before decimation
PROFILE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "ac_power_profile.json")

BG     = "#1a1a1a"
PANEL  = "#222222"
ACCENT = "#00ff32"   # voltage
AMBER  = "#ffaa00"   # current
CYAN   = "#00ccff"   # power
DIM    = "#888888"
WHITE  = "#e0e0e0"
RED    = "#ff4444"
SEP    = "#444444"


# ── AC math, mirroring the sketch ─────────────────────────────────────────────
# The board already reports these for streaming windows; the same maths runs
# here so a waveform capture can be analysed too, and so a changed phase
# calibration can be re-applied to a capture already on screen.

def _trapz_mean(series, a, b):
    """Mean of a piecewise-linear series over the continuous index range [a, b].

    a and b are fractional because they are interpolated zero crossings. The
    whole-sample trapezoid sum is taken first and the two partial end segments
    are then removed, which is what keeps a window that starts and stops
    mid-sample from putting about a percent of ripple on every reading.
    Reads index floor(b) + 1, so callers must keep b <= len(series) - 2.
    """
    ka, kb = int(math.floor(a)), int(math.floor(b))
    fa, fb = a - ka, b - kb

    total = 0.0
    for k in range(ka, kb + 1):
        total += 0.5 * (series[k] + series[k + 1])

    at_a = series[ka] + fa * (series[ka + 1] - series[ka])
    at_b = series[kb] + fb * (series[kb + 1] - series[kb])
    total -= 0.5 * fa * (series[ka] + at_a)             # strip [ka, a]
    total -= 0.5 * (1.0 - fb) * (at_b + series[kb + 1])  # strip [b, kb+1]
    return total / (b - a)


def _rising_crossings(series, baseline, first, last):
    """Fractional indices where the series crosses baseline going up.

    The scan starts at `first` so the earliest crossing it can report is
    first - 1, keeping the interpolation below clear of the start of the buffer.
    """
    out = []
    prev = series[first - 1] - baseline
    for k in range(first, last + 1):
        cur = series[k] - baseline
        if prev <= 0.0 < cur:
            frac = (-prev / (cur - prev)) if cur != prev else 0.0
            out.append(k - 1 + frac)
        prev = cur
    return out


def _lagrange4(alpha):
    """Cubic Lagrange weights for nodes at -1, 0, +1, +2, evaluated at alpha.

    This is what places the voltage at the instant the current sample was
    taken. Straight linear interpolation would get the phase right but shave
    the amplitude: the chord across a sine cuts inside the arc by cos(pi*f*dt),
    which at ~10 samples per 60 Hz cycle is 4.9 % — the whole error budget of a
    power monitor, spent on an avoidable approximation. A cubic through four
    points cuts that to 0.35 %, and at alpha = 0.5 its symmetry makes the phase
    exact rather than merely close. _interp_gain then removes what is left.
    """
    a = alpha
    return (-a * (a - 1.0) * (a - 2.0) / 6.0,
            (a + 1.0) * (a - 1.0) * (a - 2.0) / 2.0,
            -(a + 1.0) * a * (a - 2.0) / 2.0,
            (a + 1.0) * a * (a - 1.0) / 6.0)


def _interp_gain(alpha, theta):
    """Magnitude response of that interpolator at theta radians per sample.

    Even the cubic still shaves ~0.35 % off a 60 Hz sine at 10 samples per
    cycle, and it lands entirely on real power — a resistive load would read
    PF 0.9965 and invite someone to "fix" it by mistuning PHASECAL. The
    response is a known function of the measured frequency, so it is divided
    back out. Correcting at the fundamental alone is not a simplification but
    the right answer: mains voltage is near-sinusoidal, so P = sum over
    harmonics of Vh*Ih*cos(phi_h) only has a term where V has content, and the
    interpolator only ever touches the voltage.
    """
    w = _lagrange4(alpha)
    re = (w[0] * math.cos(theta) + w[1] + w[2] * math.cos(theta)
          + w[3] * math.cos(2.0 * theta))
    im = (-w[0] * math.sin(theta) + w[2] * math.sin(theta)
          + w[3] * math.sin(2.0 * theta))
    return math.hypot(re, im)


def analyze_ac(v, i, dt, phasecal=0.5):
    """RMS / power figures for a captured window.

    v and i are already in real-world units and share an index grid spaced dt
    apart; i may be None for a single-channel capture. Returns None when the
    capture is too short to bracket a cycle.
    """
    n = len(v)
    if n < 12 or dt <= 0.0:
        return None

    # Index budget: the power integrand at k reads k+2, and _trapz_mean
    # evaluates one sample past its upper bound, so the deepest read is b + 3.
    # It also reads k-1, so the lower bound may not fall below 1.
    last = n - 4
    rough = sum(v[1:last + 1]) / last
    xs = _rising_crossings(v, rough, 2, last)

    if len(xs) >= 2 and (xs[-1] - xs[0]) >= 1.0:
        a, b, cycles, nocross = xs[0], xs[-1], len(xs) - 1, False
    else:
        a, b, cycles, nocross = 1.0, float(last), 0, True
    if b - a < 1.0:
        return None

    span = (b - a) * dt
    vm = _trapz_mean(v, a, b)
    vd = [x - vm for x in v]

    r = {
        "vrms": math.sqrt(max(_trapz_mean([x * x for x in vd], a, b), 0.0)),
        "vpk":  max(abs(x) for x in vd[int(a):int(b) + 1]),
        "hz":   (cycles / span) if (cycles and span > 0) else float("nan"),
        "span": span,
        "cycles": cycles,
        "nocross": nocross,
    }

    if i is not None:
        im = _trapz_mean(i, a, b)
        idd = [x - im for x in i]
        # the voltage each current sample should be paired with: the current
        # was taken phasecal of a step after v[k], so place the voltage there
        w = _lagrange4(phasecal)
        p_series = [0.0] * n
        for k in range(1, n - 2):
            vi = (w[0] * vd[k - 1] + w[1] * vd[k]
                  + w[2] * vd[k + 1] + w[3] * vd[k + 2])
            p_series[k] = vi * idd[k]

        gain = 1.0
        if cycles and span > 0.0:
            gain = _interp_gain(phasecal, 2.0 * math.pi * r["hz"] * dt)
            if gain < 0.5:      # nonsense frequency — do not amplify noise
                gain = 1.0
        p = _trapz_mean(p_series, a, b) / gain
        irms = math.sqrt(max(_trapz_mean([x * x for x in idd], a, b), 0.0))
        s = r["vrms"] * irms
        r.update({
            "irms": irms,
            "ipk":  max(abs(x) for x in idd[int(a):int(b) + 1]),
            "p": p,
            "s": s,
            "q": math.sqrt(max(s * s - p * p, 0.0)),
            "pf": (p / s) if s > 1e-9 else 0.0,
        })
    return r


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ADS122C04 AC Power Monitor")
        self.configure(bg=BG)

        self._ser: serial.Serial | None = None
        self._rx_queue: queue.Queue = queue.Queue()
        self._rx_thread: threading.Thread | None = None
        self._running = False
        self._streaming = False
        self._relay = False

        # trend series, one point per measurement window
        self._t0 = time.monotonic()
        self._tr_t:    deque = deque(maxlen=200_000)
        self._tr_vrms: deque = deque(maxlen=200_000)
        self._tr_irms: deque = deque(maxlen=200_000)
        self._tr_w:    deque = deque(maxlen=200_000)
        self._tr_pf:   deque = deque(maxlen=200_000)
        self._last_trend_plot = 0.0

        # waveform capture
        self._in_wave = False
        self._wave_expect = 0
        self._wave_ch = 2
        self._wave_dt = 0.0
        self._wave_lsb_v = 0.0
        self._wave_lsb_i = 0.0
        self._wave_v: list = []
        self._wave_i: list = []
        self._wave_repeat = False
        self._wave_pending_after: str | None = None

        # last $ACFG readback, the source of truth for scaling the waveform
        self._acfg: dict = {}

        # CSV log
        self._log_fh: io.TextIOBase | None = None
        self._log_rows = 0

        self._build_ui()
        self._refresh_ports()
        self._load_profile()
        self.after(50, self._poll_queue)

    # ══ UI construction ═══════════════════════════════════════════════════════

    def _build_ui(self):
        self._build_topbar()

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=8, pady=4)

        left = tk.Frame(body, bg=BG, width=300)
        left.pack(side="left", fill="y", padx=(0, 8))
        left.pack_propagate(False)

        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True)

        self._build_controls(left)
        self._build_display(right)

    def _build_topbar(self):
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=8, pady=6)

        tk.Label(top, text="Port:", bg=BG, fg=WHITE).pack(side="left")
        self._port_var = tk.StringVar()
        self._port_cb = ttk.Combobox(top, textvariable=self._port_var,
                                     width=12, state="readonly")
        self._port_cb.pack(side="left", padx=4)

        tk.Label(top, text="Baud:", bg=BG, fg=WHITE).pack(side="left")
        self._baud_var = tk.StringVar(value="115200")
        ttk.Combobox(top, textvariable=self._baud_var,
                     values=["115200", "230400", "460800", "921600"],
                     width=8, state="readonly").pack(side="left", padx=4)

        tk.Button(top, text="↺", bg=PANEL, fg=WHITE, relief="flat",
                  command=self._refresh_ports).pack(side="left")

        self._connect_btn = tk.Button(top, text="Connect", bg="#225522",
                                      fg=ACCENT, relief="flat", width=10,
                                      command=self._toggle_connect)
        self._connect_btn.pack(side="left", padx=8)

        self._status_lbl = tk.Label(top, text="Disconnected", bg=BG, fg=RED)
        self._status_lbl.pack(side="left")

        # The load switch earns the most prominent spot in the window: it is the
        # only control here that moves mains current.
        self._relay_btn = tk.Button(top, text="LOAD  OFF", bg="#3a1414", fg=RED,
                                    relief="flat", width=14, height=2,
                                    font=("Courier New", 12, "bold"),
                                    command=self._toggle_relay)
        self._relay_btn.pack(side="right", padx=(8, 0))

        self._flag_lbl = tk.Label(top, text="", bg=BG, fg=RED,
                                  font=("Courier New", 9, "bold"))
        self._flag_lbl.pack(side="right", padx=8)

    # ── left column ───────────────────────────────────────────────────────────

    def _lbl(self, parent, text, color=DIM):
        tk.Label(parent, text=text, bg=BG, fg=color, anchor="w").pack(
            fill="x", pady=(6, 1))

    def _sep(self, parent):
        tk.Frame(parent, bg=SEP, height=1).pack(fill="x", pady=5)

    def _entry_row(self, parent, label, var, unit="", width=8, on_enter=None,
                   label_w=9):
        row = tk.Frame(parent, bg=BG)
        row.pack(fill="x", pady=1)
        tk.Label(row, text=label, bg=BG, fg=WHITE, width=label_w,
                 anchor="w").pack(side="left")
        e = tk.Entry(row, textvariable=var, width=width, bg=PANEL, fg=WHITE,
                     insertbackground=WHITE, relief="flat", validate="key",
                     validatecommand=(self._vcmd_float, "%P"))
        e.pack(side="left", padx=2)
        if on_enter:
            e.bind("<Return>", lambda _: on_enter())
        if unit:
            tk.Label(row, text=unit, bg=BG, fg=DIM).pack(side="left")
        return e

    def _combo_row(self, parent, label, var, values, on_change, width=16,
                   label_w=9):
        row = tk.Frame(parent, bg=BG)
        row.pack(fill="x", pady=1)
        tk.Label(row, text=label, bg=BG, fg=WHITE, width=label_w,
                 anchor="w").pack(side="left")
        cb = ttk.Combobox(row, textvariable=var, values=values,
                          state="readonly", width=width)
        cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>", lambda _: on_change())
        return cb

    def _build_controls(self, f):
        self._vcmd_float = self.register(self._validate_float)

        nb = ttk.Notebook(f)
        nb.pack(fill="both", expand=True)
        measure = tk.Frame(nb, bg=BG)
        scaling = tk.Frame(nb, bg=BG)
        adc     = tk.Frame(nb, bg=BG)
        nb.add(measure, text="Measure")
        nb.add(scaling, text="Scaling")
        nb.add(adc,     text="ADC")

        self._build_measure_tab(measure)
        self._build_scaling_tab(scaling)
        self._build_adc_tab(adc)

    def _build_measure_tab(self, f):
        self._lbl(f, "Live readings", ACCENT)
        self._stream_btn = tk.Button(f, text="▶  Start Stream", bg="#224422",
                                     fg=ACCENT, relief="flat", height=2,
                                     command=self._toggle_stream)
        self._stream_btn.pack(fill="x")

        self._trend_hist_var = tk.StringVar(value="60")
        self._entry_row(f, "History:", self._trend_hist_var, "s")

        tk.Button(f, text="Clear trends", bg=PANEL, fg=DIM, relief="flat",
                  command=self._clear_trends).pack(fill="x", pady=(2, 0))

        self._sep(f)
        self._lbl(f, "Waveform capture", AMBER)

        self._wave_ch_var = tk.StringVar(value=WAVE_CH_LABELS[2])
        self._combo_row(f, "Channel:", self._wave_ch_var, WAVE_CH_LABELS,
                        lambda: self._send(
                            f"!WCH,{WAVE_CH_LABELS.index(self._wave_ch_var.get())}"),
                        width=18)

        self._wave_len_var = tk.StringVar(value="0.2")
        self._entry_row(f, "Length:", self._wave_len_var, "s",
                        on_enter=lambda: self._send_float("!WLEN", self._wave_len_var))

        self._wave_btn = tk.Button(f, text="⚡  Capture Waveform", bg="#332200",
                                   fg=AMBER, relief="flat", height=2,
                                   command=self._trigger_wave)
        self._wave_btn.pack(fill="x", pady=(4, 2))

        rep = tk.Frame(f, bg=BG)
        rep.pack(fill="x")
        self._wave_repeat_var = tk.BooleanVar(value=False)
        tk.Checkbutton(rep, text="Repeat every", variable=self._wave_repeat_var,
                       bg=BG, fg=WHITE, selectcolor=PANEL, activebackground=BG,
                       command=self._on_repeat_toggle).pack(side="left")
        self._wave_period_var = tk.StringVar(value="1.0")
        tk.Entry(rep, textvariable=self._wave_period_var, width=5, bg=PANEL,
                 fg=WHITE, insertbackground=WHITE, relief="flat",
                 validate="key", validatecommand=(self._vcmd_float, "%P")
                 ).pack(side="left", padx=2)
        tk.Label(rep, text="s", bg=BG, fg=DIM).pack(side="left")

        self._wave_status = tk.Label(f, text="Ready", bg=BG, fg=DIM, anchor="w",
                                     font=("Courier New", 8))
        self._wave_status.pack(fill="x", pady=(2, 0))

        self._sep(f)
        self._lbl(f, "Protection", RED)
        self._trip_var = tk.StringVar(value="0")
        self._entry_row(f, "Trip at:", self._trip_var, "A rms",
                        on_enter=lambda: self._send_float("!TRIP", self._trip_var))
        tk.Label(f, text="0 disables. A trip opens the relay\nand latches until "
                         "LOAD is switched on.",
                 bg=BG, fg=DIM, justify="left", anchor="w",
                 font=("Courier New", 7)).pack(fill="x")

        self._sep(f)
        self._lbl(f, "CSV log")
        self._log_btn = tk.Button(f, text="● Start logging", bg=PANEL, fg=WHITE,
                                  relief="flat", command=self._toggle_log)
        self._log_btn.pack(fill="x")
        self._log_lbl = tk.Label(f, text="Not logging", bg=BG, fg=DIM,
                                 anchor="w", font=("Courier New", 8),
                                 wraplength=270, justify="left")
        self._log_lbl.pack(fill="x")

    def _build_scaling_tab(self, f):
        self._lbl(f, "Voltage channel  AIN0−AIN1", ACCENT)
        self._v_adc_var = tk.StringVar(value="1.5")
        self._v_fs_var  = tk.StringVar(value="170")
        self._entry_row(f, "ADC diff:", self._v_adc_var, "V",
                        on_enter=self._send_vscale, label_w=10)
        self._entry_row(f, "equals:", self._v_fs_var, "V",
                        on_enter=self._send_vscale, label_w=10)

        self._lbl(f, "Current channel  AIN2−AIN3", AMBER)
        self._i_adc_var = tk.StringVar(value="1.5")
        self._i_fs_var  = tk.StringVar(value="25")
        self._entry_row(f, "ADC diff:", self._i_adc_var, "V",
                        on_enter=self._send_iscale, label_w=10)
        self._entry_row(f, "equals:", self._i_fs_var, "A",
                        on_enter=self._send_iscale, label_w=10)

        tk.Button(f, text="Apply scaling", bg=PANEL, fg=ACCENT, relief="flat",
                  command=self._send_both_scales).pack(fill="x", pady=(4, 0))
        tk.Label(f, text="These are peak-referred: 1.5 V of ADC swing\n"
                         "= 170 V peak = 120 V rms.",
                 bg=BG, fg=DIM, justify="left", anchor="w",
                 font=("Courier New", 7)).pack(fill="x", pady=(2, 0))

        self._sep(f)
        self._lbl(f, "Phase calibration")
        self._phase_var = tk.StringVar(value="0.5")
        self._entry_row(f, "PHASECAL:", self._phase_var, "",
                        on_enter=lambda: self._send_float("!PHASECAL", self._phase_var),
                        label_w=10)
        tk.Label(f, text="Where between v[k] and v[k+1] the current\n"
                         "sample landed. 0.5 nominal; trim it to null\n"
                         "a CT or filter phase error against a known\n"
                         "resistive load (PF should read 1.000).",
                 bg=BG, fg=DIM, justify="left", anchor="w",
                 font=("Courier New", 7)).pack(fill="x")

        self._sep(f)
        self._lbl(f, "Offsets")
        tk.Button(f, text="Zero both channels", bg=PANEL, fg=WHITE,
                  relief="flat", command=self._zero_channels).pack(fill="x")
        tk.Label(f, text="Run with the front end powered but no\nmains and no load.",
                 bg=BG, fg=DIM, justify="left", anchor="w",
                 font=("Courier New", 7)).pack(fill="x")

        off = tk.Frame(f, bg=BG)
        off.pack(fill="x", pady=(2, 0))
        tk.Button(off, text="Clear V", bg=PANEL, fg=DIM, relief="flat",
                  command=lambda: self._send("!VOFF,0")).pack(side="left")
        tk.Button(off, text="Clear I", bg=PANEL, fg=DIM, relief="flat",
                  command=lambda: self._send("!IOFF,0")).pack(side="left", padx=4)

        self._dcrem_var = tk.BooleanVar(value=True)
        tk.Checkbutton(f, text="Remove per-window DC", variable=self._dcrem_var,
                       bg=BG, fg=WHITE, selectcolor=PANEL, activebackground=BG,
                       command=lambda: self._send_bool("!DCREM", self._dcrem_var)
                       ).pack(anchor="w", pady=(4, 0))

        self._sep(f)
        self._lbl(f, "Measurement window")
        self._cycles_var = tk.StringVar(value="10")
        self._entry_row(f, "Cycles:", self._cycles_var, "",
                        on_enter=lambda: self._send_int("!CYCLES", self._cycles_var),
                        label_w=10)
        self._mains_var = tk.StringVar(value="60")
        self._entry_row(f, "Mains:", self._mains_var, "Hz",
                        on_enter=lambda: self._send_float("!MAINS", self._mains_var),
                        label_w=10)

        self._sep(f)
        prof = tk.Frame(f, bg=BG)
        prof.pack(fill="x")
        tk.Button(prof, text="Save profile", bg=PANEL, fg=WHITE, relief="flat",
                  command=self._save_profile).pack(side="left")
        tk.Button(prof, text="Load + send", bg=PANEL, fg=WHITE, relief="flat",
                  command=self._apply_profile).pack(side="left", padx=4)

    def _build_adc_tab(self, f):
        self._lbl(f, "Conversion")
        self._rate_var = tk.StringVar(value=RATE_LABELS[6])
        self._combo_row(f, "Rate:", self._rate_var, RATE_LABELS,
                        lambda: self._send(f"!RATE,{RATE_LABELS.index(self._rate_var.get())}"))

        self._turbo_var = tk.BooleanVar(value=True)
        tk.Checkbutton(f, text="Turbo mode", variable=self._turbo_var, bg=BG,
                       fg=WHITE, selectcolor=PANEL, activebackground=BG,
                       command=lambda: self._send_bool("!TURBO", self._turbo_var)
                       ).pack(anchor="w")

        self._i2c_var = tk.StringVar(value=I2C_LABELS[1])
        self._combo_row(f, "I2C clock:", self._i2c_var, I2C_LABELS,
                        lambda: self._send(
                            f"!I2C,{I2C_VALUES[I2C_LABELS.index(self._i2c_var.get())]}"))
        tk.Label(f, text="I2C time is most of the per-sample cost;\n"
                         "1 MHz buys roughly a third more pairs\nper second if the "
                         "wiring tolerates it.",
                 bg=BG, fg=DIM, justify="left", anchor="w",
                 font=("Courier New", 7)).pack(fill="x")

        self._sep(f)
        self._lbl(f, "Gain")
        self._vgain_var = tk.StringVar(value=GAIN_LABELS[0])
        self._combo_row(f, "Voltage:", self._vgain_var, GAIN_LABELS,
                        lambda: self._send(
                            f"!VGAIN,{GAIN_LABELS.index(self._vgain_var.get())}"),
                        width=8)
        self._igain_var = tk.StringVar(value=GAIN_LABELS[0])
        self._combo_row(f, "Current:", self._igain_var, GAIN_LABELS,
                        lambda: self._send(
                            f"!IGAIN,{GAIN_LABELS.index(self._igain_var.get())}"),
                        width=8)

        self._pga_var = tk.BooleanVar(value=False)
        tk.Checkbutton(f, text="PGA in circuit", variable=self._pga_var, bg=BG,
                       fg=WHITE, selectcolor=PANEL, activebackground=BG,
                       command=lambda: self._send_bool("!PGA", self._pga_var)
                       ).pack(anchor="w")
        tk.Label(f, text="Bypassed, the inputs swing rail to rail\n"
                         "at gains up to 4×. Above 4× the PGA has\n"
                         "to be in, which narrows the common mode\n"
                         "the front end may sit at.",
                 bg=BG, fg=DIM, justify="left", anchor="w",
                 font=("Courier New", 7)).pack(fill="x")

        self._sep(f)
        self._lbl(f, "Reference")
        self._vref_var = tk.StringVar(value="2.048")
        self._entry_row(f, "VREF:", self._vref_var, "V",
                        on_enter=lambda: self._send_float("!VREF", self._vref_var))

        self._sep(f)
        self._lbl(f, "Relay driver")
        self._rlyinv_var = tk.BooleanVar(value=False)
        tk.Checkbutton(f, text="Active low (D5)", variable=self._rlyinv_var,
                       bg=BG, fg=WHITE, selectcolor=PANEL, activebackground=BG,
                       command=lambda: self._send_bool("!RLYINV", self._rlyinv_var)
                       ).pack(anchor="w")

        self._sep(f)
        self._lbl(f, "Device config")
        self._cfg_lbl = tk.Label(f, text="—", bg=PANEL, fg=DIM, justify="left",
                                 anchor="w", wraplength=265, padx=4, pady=4,
                                 font=("Courier New", 8))
        self._cfg_lbl.pack(fill="x")

    # ── right column ──────────────────────────────────────────────────────────

    def _tile(self, parent, title, unit, color=WHITE, big=False):
        col = tk.Frame(parent, bg=PANEL)
        col.pack(side="left", expand=True, fill="x", padx=6, pady=4)
        tk.Label(col, text=title, bg=PANEL, fg=DIM,
                 font=("Courier New", 8)).pack()
        val = tk.Label(col, text="—", bg=PANEL, fg=color,
                       font=("Courier New", 20 if big else 13, "bold"))
        val.pack()
        tk.Label(col, text=unit, bg=PANEL, fg=DIM,
                 font=("Courier New", 8)).pack()
        return val

    def _build_display(self, f):
        meters = tk.Frame(f, bg=PANEL)
        meters.pack(fill="x", pady=(0, 4))

        row1 = tk.Frame(meters, bg=PANEL)
        row1.pack(fill="x")
        self._m_vrms = self._tile(row1, "VOLTAGE", "V rms", ACCENT, big=True)
        self._m_irms = self._tile(row1, "CURRENT", "A rms", AMBER,  big=True)
        self._m_w    = self._tile(row1, "REAL POWER", "W",   CYAN,   big=True)
        self._m_pf   = self._tile(row1, "POWER FACTOR", "",  WHITE,  big=True)

        row2 = tk.Frame(meters, bg=PANEL)
        row2.pack(fill="x")
        self._m_va  = self._tile(row2, "APPARENT", "VA")
        self._m_var = self._tile(row2, "REACTIVE + DIST", "var")
        self._m_hz  = self._tile(row2, "FREQUENCY", "Hz")
        self._m_wh  = self._tile(row2, "ENERGY", "Wh")
        self._m_pk  = self._tile(row2, "PEAK V / I", "V / A")
        self._m_sps = self._tile(row2, "PAIR RATE", "pairs/s")

        nb = ttk.Notebook(f)
        nb.pack(fill="both", expand=True, pady=(0, 4))
        trends = tk.Frame(nb, bg=BG)
        wave   = tk.Frame(nb, bg=BG)
        nb.add(trends, text="  Trends  ")
        nb.add(wave,   text="  Waveform  ")
        self._nb = nb
        self._tab_trends, self._tab_wave = trends, wave

        self._build_trend_plot(trends)
        self._build_wave_plot(wave)

        self._console = tk.Text(f, height=5, bg="#111111", fg=DIM,
                                font=("Courier New", 8), state="disabled",
                                relief="flat")
        self._console.pack(fill="x")

    def _style_axes(self, ax, xlabel=None, ylabel=None, color=DIM):
        ax.set_facecolor(PANEL)
        ax.tick_params(colors=DIM, labelsize=8)
        for sp in ax.spines.values():
            sp.set_edgecolor(SEP)
        if xlabel:
            ax.set_xlabel(xlabel, color=DIM, fontsize=8)
        if ylabel:
            ax.set_ylabel(ylabel, color=color, fontsize=8)

    def _build_trend_plot(self, f):
        fig = Figure(figsize=(7, 4), facecolor=BG)
        fig.subplots_adjust(left=0.09, right=0.91, top=0.97, bottom=0.1,
                            hspace=0.15)

        self._ax_v = fig.add_subplot(211)
        self._ax_i = self._ax_v.twinx()
        self._style_axes(self._ax_v, ylabel="V rms", color=ACCENT)
        self._style_axes(self._ax_i, ylabel="A rms", color=AMBER)
        self._ax_v.tick_params(labelbottom=False)

        self._ax_w  = fig.add_subplot(212, sharex=self._ax_v)
        self._ax_pf = self._ax_w.twinx()
        self._style_axes(self._ax_w, xlabel="Time (s)", ylabel="Watts", color=CYAN)
        self._style_axes(self._ax_pf, ylabel="Power factor", color=WHITE)
        self._ax_pf.set_ylim(-1.05, 1.05)

        self._ln_vrms, = self._ax_v.plot([], [], color=ACCENT, lw=1.0)
        self._ln_irms, = self._ax_i.plot([], [], color=AMBER,  lw=1.0)
        self._ln_w,    = self._ax_w.plot([], [], color=CYAN,   lw=1.0)
        self._ln_pf,   = self._ax_pf.plot([], [], color=WHITE, lw=0.8, ls="--")

        canvas = FigureCanvasTkAgg(fig, master=f)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._trend_canvas = canvas

    def _build_wave_plot(self, f):
        fig = Figure(figsize=(7, 4), facecolor=BG)
        fig.subplots_adjust(left=0.09, right=0.91, top=0.93, bottom=0.12)

        self._ax_wv = fig.add_subplot(111)
        self._ax_wi = self._ax_wv.twinx()
        self._style_axes(self._ax_wv, xlabel="Time (ms)", ylabel="Volts", color=ACCENT)
        self._style_axes(self._ax_wi, ylabel="Amps", color=AMBER)

        self._ln_wv, = self._ax_wv.plot([], [], color=ACCENT, lw=1.0,
                                        marker=".", ms=2, label="voltage")
        self._ln_wi, = self._ax_wi.plot([], [], color=AMBER, lw=1.0,
                                        marker=".", ms=2, label="current")
        self._wave_title = self._ax_wv.set_title("", color=DIM, fontsize=8, pad=4)

        canvas = FigureCanvasTkAgg(fig, master=f)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._wave_canvas = canvas

        stats = tk.Frame(f, bg=PANEL)
        stats.pack(fill="x")
        self._ws_vrms = self._tile(stats, "Vrms", "V")
        self._ws_irms = self._tile(stats, "Irms", "A")
        self._ws_p    = self._tile(stats, "P", "W")
        self._ws_s    = self._tile(stats, "S", "VA")
        self._ws_pf   = self._tile(stats, "PF", "")
        self._ws_hz   = self._tile(stats, "Freq", "Hz")
        self._ws_cf   = self._tile(stats, "Crest V / I", "")
        self._ws_n    = self._tile(stats, "Samples", "@ SPS")

    # ══ serial ════════════════════════════════════════════════════════════════

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
        if not port:
            messagebox.showerror("Error", "Select a port first.")
            return
        try:
            self._ser = serial.Serial(port, int(self._baud_var.get()), timeout=0.05)
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
        # leave the load open rather than trusting whatever state it was in
        if self._ser and self._ser.is_open:
            try:
                self._send("!RELAY,0")
                self._send("!STOP")
                time.sleep(0.15)
            except Exception:
                pass
        self._streaming = False
        self._running = False
        self._wave_repeat = False
        if self._wave_repeat_var.get():
            self._wave_repeat_var.set(False)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self._connect_btn.config(text="Connect", bg="#225522", fg=ACCENT)
        self._status_lbl.config(text="Disconnected", fg=RED)
        self._stream_btn.config(text="▶  Start Stream", bg="#224422")
        self._set_relay_button(False)

    def _rx_worker(self):
        buf = b""
        while self._running:
            try:
                chunk = self._ser.read(1024)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._rx_queue.put(line.decode(errors="replace").strip())
            except (serial.SerialException, AttributeError, TypeError):
                break

    def _send(self, msg: str):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write((msg + "\n").encode())
            except serial.SerialException:
                pass

    def _send_bool(self, cmd, var):
        self._send(f"{cmd},{1 if var.get() else 0}")

    def _send_float(self, cmd, var):
        try:
            self._send(f"{cmd},{float(var.get()):.6f}")
        except ValueError:
            pass

    def _send_int(self, cmd, var):
        try:
            self._send(f"{cmd},{int(float(var.get()))}")
        except ValueError:
            pass

    def _send_vscale(self):
        try:
            self._send(f"!VSCALE,{float(self._v_adc_var.get()):.6f},"
                       f"{float(self._v_fs_var.get()):.6f}")
        except ValueError:
            pass

    def _send_iscale(self):
        try:
            self._send(f"!ISCALE,{float(self._i_adc_var.get()):.6f},"
                       f"{float(self._i_fs_var.get()):.6f}")
        except ValueError:
            pass

    def _send_both_scales(self):
        self._send_vscale()
        self._send_iscale()

    def _zero_channels(self):
        if self._relay and not messagebox.askyesno(
                "Load is on",
                "Zeroing takes the present readings as the new zero, so it must "
                "run with no mains and no load.\n\nThe load relay is closed. "
                "Zero anyway?"):
            return
        self._send("!ZERO")

    # ══ controls ══════════════════════════════════════════════════════════════

    def _toggle_relay(self):
        if not self._ser or not self._ser.is_open:
            messagebox.showinfo("Not connected", "Connect to a port first.")
            return
        self._send(f"!RELAY,{0 if self._relay else 1}")

    def _set_relay_button(self, on: bool):
        self._relay = on
        if on:
            self._relay_btn.config(text="LOAD  ON", bg="#1e4d1e", fg=ACCENT)
        else:
            self._relay_btn.config(text="LOAD  OFF", bg="#3a1414", fg=RED)

    def _toggle_stream(self):
        if not self._ser or not self._ser.is_open:
            messagebox.showinfo("Not connected", "Connect to a port first.")
            return
        if self._streaming:
            self._send("!STOP")
            self._streaming = False
            self._stream_btn.config(text="▶  Start Stream", bg="#224422")
        else:
            self._clear_trends()
            self._send("!START")
            self._streaming = True
            self._stream_btn.config(text="■  Stop Stream", bg="#552200")

    def _clear_trends(self):
        self._t0 = time.monotonic()
        for d in (self._tr_t, self._tr_vrms, self._tr_irms, self._tr_w, self._tr_pf):
            d.clear()
        self._update_trend_plot(force=True)

    def _trigger_wave(self):
        if not self._ser or not self._ser.is_open:
            messagebox.showinfo("Not connected", "Connect to a port first.")
            return
        self._in_wave = True
        self._wave_v, self._wave_i = [], []
        self._wave_expect = 0
        self._wave_btn.config(state="disabled")
        self._wave_status.config(text="Waiting for capture…", fg=AMBER)
        self._send("!WAVE")

    def _on_repeat_toggle(self):
        self._wave_repeat = self._wave_repeat_var.get()
        if self._wave_repeat and not self._in_wave:
            self._trigger_wave()

    def _schedule_repeat(self):
        if self._wave_pending_after:
            try:
                self.after_cancel(self._wave_pending_after)
            except Exception:
                pass
            self._wave_pending_after = None
        if not self._wave_repeat:
            return
        try:
            period = max(float(self._wave_period_var.get() or "1.0"), 0.2)
        except ValueError:
            period = 1.0
        self._wave_pending_after = self.after(int(period * 1000),
                                              self._repeat_fire)

    def _repeat_fire(self):
        self._wave_pending_after = None
        if self._wave_repeat and self._ser and self._ser.is_open:
            self._trigger_wave()

    # ══ CSV log ═══════════════════════════════════════════════════════════════

    def _toggle_log(self):
        if self._log_fh:
            self._close_log()
            return
        path = filedialog.asksaveasfilename(
            title="Log measurements to",
            defaultextension=".csv",
            initialfile=time.strftime("ac_power_%Y%m%d_%H%M%S.csv"),
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        try:
            self._log_fh = open(path, "w", encoding="utf-8", newline="")
        except OSError as e:
            messagebox.showerror("Log error", str(e))
            return
        self._log_fh.write("iso_time,elapsed_s,vrms_v,irms_a,real_w,apparent_va,"
                           "reactive_var,pf,freq_hz,vpeak_v,ipeak_a,energy_wh,"
                           "pairs,pairs_per_s,relay,flags\n")
        self._log_rows = 0
        self._log_btn.config(text="■ Stop logging", fg=RED)
        self._log_lbl.config(text=os.path.basename(path), fg=ACCENT)
        self._log_path = path

    def _close_log(self):
        if not self._log_fh:
            return
        try:
            self._log_fh.close()
        except OSError:
            pass
        self._log_fh = None
        self._log_btn.config(text="● Start logging", fg=WHITE)
        self._log_lbl.config(text=f"Saved {self._log_rows} rows", fg=DIM)

    def _log_row(self, p, elapsed):
        if not self._log_fh:
            return
        try:
            self._log_fh.write(
                f"{time.strftime('%Y-%m-%dT%H:%M:%S')},{elapsed:.3f},"
                f"{p['vrms']:.4f},{p['irms']:.5f},{p['w']:.4f},{p['va']:.4f},"
                f"{p['var']:.4f},{p['pf']:.4f},{p['hz']:.3f},{p['vpk']:.4f},"
                f"{p['ipk']:.5f},{p['wh']:.6f},{p['pairs']},{p['sps']:.1f},"
                f"{p['relay']},{p['flags']}\n")
            self._log_fh.flush()
            self._log_rows += 1
            self._log_lbl.config(text=f"{os.path.basename(self._log_path)} — "
                                      f"{self._log_rows} rows")
        except OSError as e:
            self._log(f"!! log write failed: {e}")
            self._close_log()

    # ══ profile ═══════════════════════════════════════════════════════════════

    def _profile_vars(self):
        return {
            "port":      self._port_var,
            "baud":      self._baud_var,
            "v_adc_fs":  self._v_adc_var,
            "v_fs":      self._v_fs_var,
            "i_adc_fs":  self._i_adc_var,
            "i_fs":      self._i_fs_var,
            "phasecal":  self._phase_var,
            "cycles":    self._cycles_var,
            "mains_hz":  self._mains_var,
            "trip_a":    self._trip_var,
            "vref":      self._vref_var,
            "wave_len":  self._wave_len_var,
            "wave_ch":   self._wave_ch_var,
            "rate":      self._rate_var,
            "vgain":     self._vgain_var,
            "igain":     self._igain_var,
            "i2c":       self._i2c_var,
            "trend_hist": self._trend_hist_var,
        }

    def _save_profile(self):
        data = {k: v.get() for k, v in self._profile_vars().items()}
        data["turbo"]  = self._turbo_var.get()
        data["pga"]    = self._pga_var.get()
        data["dcrem"]  = self._dcrem_var.get()
        data["rlyinv"] = self._rlyinv_var.get()
        try:
            with open(PROFILE_FILE, "w", encoding="utf-8") as fh:
                json.dump(data, fh, indent=2)
            self._log(f"-- profile saved to {PROFILE_FILE}")
        except OSError as e:
            messagebox.showerror("Profile error", str(e))

    def _load_profile(self):
        """Restore the last saved settings into the widgets, without sending."""
        try:
            with open(PROFILE_FILE, encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError):
            return
        for key, var in self._profile_vars().items():
            if key in data:
                var.set(data[key])
        for key, var in (("turbo", self._turbo_var), ("pga", self._pga_var),
                         ("dcrem", self._dcrem_var), ("rlyinv", self._rlyinv_var)):
            if key in data:
                var.set(bool(data[key]))

    def _apply_profile(self):
        """Reload the file and push every value at the board."""
        self._load_profile()
        if not (self._ser and self._ser.is_open):
            self._log("-- profile loaded into the GUI (not connected)")
            return
        self._send_float("!VREF", self._vref_var)
        self._send(f"!RATE,{RATE_LABELS.index(self._rate_var.get())}")
        self._send_bool("!TURBO", self._turbo_var)
        self._send_bool("!PGA", self._pga_var)
        self._send(f"!VGAIN,{GAIN_LABELS.index(self._vgain_var.get())}")
        self._send(f"!IGAIN,{GAIN_LABELS.index(self._igain_var.get())}")
        self._send(f"!I2C,{I2C_VALUES[I2C_LABELS.index(self._i2c_var.get())]}")
        self._send_both_scales()
        self._send_float("!PHASECAL", self._phase_var)
        self._send_int("!CYCLES", self._cycles_var)
        self._send_float("!MAINS", self._mains_var)
        self._send_float("!TRIP", self._trip_var)
        self._send_bool("!DCREM", self._dcrem_var)
        self._send_bool("!RLYINV", self._rlyinv_var)
        self._send_float("!WLEN", self._wave_len_var)
        self._send(f"!WCH,{WAVE_CH_LABELS.index(self._wave_ch_var.get())}")
        self._send("!CFG")

    # ══ RX handling ═══════════════════════════════════════════════════════════

    def _poll_queue(self):
        try:
            for _ in range(500):
                line = self._rx_queue.get_nowait()
                try:
                    self._handle_line(line)
                except Exception as e:
                    self._log(f"!! GUI error on {line[:40]!r}: {e}")
        except queue.Empty:
            pass
        finally:
            self.after(20, self._poll_queue)

    def _handle_line(self, line: str):
        # waveform samples arrive in the thousands — never logged
        if line.startswith("$WD,"):
            if self._in_wave:
                self._collect_wave_sample(line[4:])
            return

        self._log(line)

        if line.startswith("$PWR,"):
            self._handle_pwr(line[5:].split(","))
        elif line.startswith("$WAVE,"):
            self._handle_wave_header(line[6:].split(","))
        elif line.startswith("$WEND"):
            self._finish_wave()
        elif line.startswith("$CFG,"):
            self._apply_cfg(line[5:].split(","))
        elif line.startswith("$ACFG,"):
            self._apply_acfg(line[6:].split(","))
        elif line.startswith("$RLY,"):
            self._set_relay_button(line[5:].strip() == "1")
        elif line.startswith("$TRIP,"):
            parts = line[6:].split(",")
            self._flag_lbl.config(text=f"TRIPPED @ {parts[0]} A")
            messagebox.showwarning(
                "Over-current trip",
                f"Load opened: {parts[0]} A rms exceeded the "
                f"{parts[1]} A limit.\n\nSwitch LOAD back on to clear it.")
        elif line.startswith("$READY"):
            self._streaming = False
            self._stream_btn.config(text="▶  Start Stream", bg="#224422")
            self._set_relay_button(False)
            if self._in_wave:
                self._abort_wave("Board restarted")
        elif line.startswith("$ERR,"):
            if self._in_wave:
                self._abort_wave("Error — see console")

    def _handle_pwr(self, parts):
        if len(parts) < 15:
            return
        p = {
            "vrms": float(parts[0]), "irms": float(parts[1]),
            "w":    float(parts[2]), "va":   float(parts[3]),
            "var":  float(parts[4]), "pf":   float(parts[5]),
            "hz":   float(parts[6]), "vpk":  float(parts[7]),
            "ipk":  float(parts[8]), "wh":   float(parts[9]),
            "uptime": float(parts[10]), "pairs": int(parts[11]),
            "sps":  float(parts[12]), "relay": int(parts[13]),
            "flags": int(parts[14]),
        }

        self._m_vrms.config(text=f"{p['vrms']:.2f}")
        self._m_irms.config(text=f"{p['irms']:.3f}")
        self._m_w.config(text=f"{p['w']:.2f}")
        self._m_pf.config(text=f"{p['pf']:+.3f}")
        self._m_va.config(text=f"{p['va']:.2f}")
        self._m_var.config(text=f"{p['var']:.2f}")
        self._m_hz.config(text="—" if p["hz"] <= 0 else f"{p['hz']:.3f}")
        self._m_wh.config(text=f"{p['wh']:.4f}")
        self._m_pk.config(text=f"{p['vpk']:.1f}/{p['ipk']:.2f}")
        self._m_sps.config(text=f"{p['sps']:.0f}")

        self._flag_lbl.config(text=self._flag_text(p["flags"]))
        self._set_relay_button(bool(p["relay"]))

        now = time.monotonic() - self._t0
        self._tr_t.append(now)
        self._tr_vrms.append(p["vrms"])
        self._tr_irms.append(p["irms"])
        self._tr_w.append(p["w"])
        self._tr_pf.append(p["pf"])
        self._log_row(p, now)
        self._update_trend_plot()

    @staticmethod
    def _flag_text(flags: int) -> str:
        return "  ".join(name for bit, name in FLAG_NAMES if flags & bit)

    # ── waveform ──────────────────────────────────────────────────────────────

    def _handle_wave_header(self, parts):
        if len(parts) < 6:
            return
        self._wave_expect = int(parts[0])
        self._wave_ch     = int(parts[1])
        self._wave_dt     = float(parts[2]) * 1e-6      # us -> s
        self._wave_lsb_v  = float(parts[3])
        self._wave_lsb_i  = float(parts[4])
        self._wave_v, self._wave_i = [], []
        self._in_wave = True
        self._wave_status.config(text=f"Receiving 0/{self._wave_expect}", fg=AMBER)

    def _collect_wave_sample(self, payload: str):
        parts = payload.split(",")
        try:
            self._wave_v.append(int(parts[0]))
            if len(parts) > 1:
                self._wave_i.append(int(parts[1]))
        except ValueError:
            return
        n = len(self._wave_v)
        if n % 200 == 0:
            self._wave_status.config(text=f"Receiving {n}/{self._wave_expect}",
                                     fg=AMBER)

    def _abort_wave(self, why: str):
        self._in_wave = False
        self._wave_btn.config(state="normal")
        self._wave_status.config(text=why, fg=RED)
        self._schedule_repeat()

    def _finish_wave(self):
        self._in_wave = False
        self._wave_btn.config(state="normal")
        n = len(self._wave_v)
        sps = (1.0 / self._wave_dt) if self._wave_dt > 0 else 0.0
        self._wave_status.config(
            text=f"{n} samples @ {sps:.0f} SPS", fg=ACCENT)
        self._render_wave()
        self._schedule_repeat()

    def _scaled_wave(self):
        """Raw counts -> volts and amps, using the board's own scaling."""
        a = self._acfg
        v_scale = a.get("v_fs", 170.0) / a.get("v_adc_fs", 1.5)
        i_scale = a.get("i_fs", 25.0) / a.get("i_adc_fs", 1.5)
        v_off   = a.get("v_off_uv", 0.0) * 1e-6
        i_off   = a.get("i_off_uv", 0.0) * 1e-6

        if self._wave_ch == 1:      # single-channel current capture
            cur = [(r * self._wave_lsb_i - i_off) * i_scale for r in self._wave_v]
            return None, cur
        volts = [(r * self._wave_lsb_v - v_off) * v_scale for r in self._wave_v]
        if self._wave_ch == 0:
            return volts, None
        amps = [(r * self._wave_lsb_i - i_off) * i_scale for r in self._wave_i]
        return volts, amps

    def _render_wave(self):
        if not self._wave_v or self._wave_dt <= 0:
            return
        volts, amps = self._scaled_wave()
        n = len(self._wave_v)
        xs = [k * self._wave_dt * 1000.0 for k in range(n)]   # ms

        if volts is not None:
            self._ln_wv.set_data(xs, volts)
            self._autoscale(self._ax_wv, volts)
        else:
            self._ln_wv.set_data([], [])
        if amps is not None:
            self._ln_wi.set_data(xs[:len(amps)], amps)
            self._autoscale(self._ax_wi, amps)
        else:
            self._ln_wi.set_data([], [])

        self._ax_wv.set_xlim(0, xs[-1] if n > 1 else 1.0)
        sps = 1.0 / self._wave_dt
        label = WAVE_CH_LABELS[self._wave_ch]
        self._wave_title.set_text(f"{label} — {n} samples @ {sps:.0f} SPS")
        self._wave_title.set_color(AMBER)
        self._wave_canvas.draw_idle()

        self._update_wave_stats(volts, amps, n, sps)

    def _update_wave_stats(self, volts, amps, n, sps):
        phasecal = self._acfg.get("phasecal", 0.5)
        # a single-channel capture has one series to analyse; feed whichever
        # one arrived in as the "voltage" and read the RMS back out of it
        primary = volts if volts is not None else amps
        r = analyze_ac(primary, amps if (volts is not None and amps is not None)
                       else None, self._wave_dt, phasecal)

        for lbl in (self._ws_vrms, self._ws_irms, self._ws_p, self._ws_s,
                    self._ws_pf, self._ws_hz, self._ws_cf):
            lbl.config(text="—")
        self._ws_n.config(text=f"{n} @ {sps:.0f}")
        if not r:
            return

        crest_v = crest_i = float("nan")
        if volts is not None:
            self._ws_vrms.config(text=f"{r['vrms']:.2f}")
            if r["vrms"] > 1e-9:
                crest_v = r["vpk"] / r["vrms"]
        else:
            # the single series was current
            self._ws_irms.config(text=f"{r['vrms']:.3f}")
            if r["vrms"] > 1e-9:
                crest_i = r["vpk"] / r["vrms"]

        if "irms" in r:
            self._ws_irms.config(text=f"{r['irms']:.3f}")
            self._ws_p.config(text=f"{r['p']:.2f}")
            self._ws_s.config(text=f"{r['s']:.2f}")
            self._ws_pf.config(text=f"{r['pf']:+.3f}")
            if r["irms"] > 1e-9:
                crest_i = r["ipk"] / r["irms"]

        self._ws_hz.config(text="—" if r["hz"] != r["hz"] else f"{r['hz']:.3f}")
        parts = [f"{c:.2f}" if c == c else "—" for c in (crest_v, crest_i)]
        self._ws_cf.config(text="/".join(parts))

    # ── plots ─────────────────────────────────────────────────────────────────

    @staticmethod
    def _autoscale(ax, ys):
        vals = [y for y in ys if y == y and abs(y) != float("inf")]
        if not vals:
            return
        lo, hi = min(vals), max(vals)
        pad = (hi - lo) * 0.08 if hi != lo else max(abs(hi) * 0.1, 0.01)
        ax.set_ylim(lo - pad, hi + pad)

    def _update_trend_plot(self, force: bool = False):
        now = time.monotonic()
        if not force and now - self._last_trend_plot < 1 / 10:
            return
        self._last_trend_plot = now

        if not self._tr_t:
            for ln in (self._ln_vrms, self._ln_irms, self._ln_w, self._ln_pf):
                ln.set_data([], [])
            self._trend_canvas.draw_idle()
            return

        try:
            hist = max(float(self._trend_hist_var.get() or "60"), 1.0)
        except ValueError:
            hist = 60.0

        ts = list(self._tr_t)
        t_end = ts[-1]
        cutoff = t_end - hist
        # bisect would need a plain list anyway, and the windows are ~5/s —
        # a scan back from the end is cheaper than converting the deque
        idx = 0
        for k in range(len(ts) - 1, -1, -1):
            if ts[k] < cutoff:
                idx = k + 1
                break

        def cut(dq):
            out = list(dq)[idx:]
            if len(out) > MAX_TREND_PTS:
                step = max(len(out) // MAX_TREND_PTS, 1)
                out = out[::step]
            return out

        xs = cut(self._tr_t)
        vs, i_s = cut(self._tr_vrms), cut(self._tr_irms)
        ws, pfs = cut(self._tr_w), cut(self._tr_pf)

        self._ln_vrms.set_data(xs, vs)
        self._ln_irms.set_data(xs, i_s)
        self._ln_w.set_data(xs, ws)
        self._ln_pf.set_data(xs, pfs)

        self._ax_v.set_xlim(cutoff, t_end if t_end > cutoff else cutoff + 1)
        self._autoscale(self._ax_v, vs)
        self._autoscale(self._ax_i, i_s)
        self._autoscale(self._ax_w, ws)
        self._trend_canvas.draw_idle()

    # ══ config readback ═══════════════════════════════════════════════════════

    def _apply_cfg(self, parts):
        if len(parts) < 7:
            return
        rate, turbo, pga = int(parts[0]), int(parts[1]), int(parts[2])
        vgain, igain     = int(parts[3]), int(parts[4])
        relay, streaming = int(parts[5]), int(parts[6])

        self._rate_var.set(RATE_LABELS[rate])
        self._turbo_var.set(bool(turbo))
        self._pga_var.set(bool(pga))
        self._vgain_var.set(GAIN_LABELS[vgain])
        self._igain_var.set(GAIN_LABELS[igain])
        self._set_relay_button(bool(relay))

        self._streaming = bool(streaming)
        self._stream_btn.config(
            text="■  Stop Stream" if streaming else "▶  Start Stream",
            bg="#552200" if streaming else "#224422")

        nominal = RATE_NOMINAL[rate] * (2 if turbo else 1)
        self._cfg_lbl.config(
            text=(f"Rate     {RATE_LABELS[rate]}  ({nominal} SPS)\n"
                  f"Gain     V {GAIN_LABELS[vgain]}   I {GAIN_LABELS[igain]}\n"
                  f"PGA      {'in circuit' if pga else 'bypassed'}\n"
                  f"Relay    {'CLOSED' if relay else 'open'}"),
            fg=WHITE)

    def _apply_acfg(self, parts):
        if len(parts) < 15:
            return
        keys = ["v_adc_fs", "v_fs", "i_adc_fs", "i_fs", "phasecal", "cycles",
                "mains_hz", "trip_a", "dcrem", "v_off_uv", "i_off_uv",
                "rly_inv", "wave_len_s", "wave_ch", "vref"]
        self._acfg = {k: float(v) for k, v in zip(keys, parts)}
        a = self._acfg

        # Board state is the truth, so mirror it back into the boxes. Anything
        # the user typed has already been sent, and the board echoes $ACFG in
        # reply, so this converges rather than fighting the keyboard.
        self._v_adc_var.set(f"{a['v_adc_fs']:g}")
        self._v_fs_var.set(f"{a['v_fs']:g}")
        self._i_adc_var.set(f"{a['i_adc_fs']:g}")
        self._i_fs_var.set(f"{a['i_fs']:g}")
        self._phase_var.set(f"{a['phasecal']:g}")
        self._cycles_var.set(f"{int(a['cycles'])}")
        self._mains_var.set(f"{a['mains_hz']:g}")
        self._trip_var.set(f"{a['trip_a']:g}")
        self._dcrem_var.set(bool(a["dcrem"]))
        self._rlyinv_var.set(bool(a["rly_inv"]))
        self._wave_len_var.set(f"{a['wave_len_s']:g}")
        self._wave_ch_var.set(WAVE_CH_LABELS[int(a["wave_ch"])])
        self._vref_var.set(f"{a['vref']:g}")

        # a capture already on screen is re-scaled by the new numbers
        if self._wave_v and not self._in_wave:
            self._render_wave()

    # ══ misc ══════════════════════════════════════════════════════════════════

    @staticmethod
    def _validate_float(s: str) -> bool:
        if s in ("", "-", "+", ".", "-.", "+."):
            return True
        try:
            float(s)
            return True
        except ValueError:
            return False

    def _log(self, text: str):
        self._console.config(state="normal")
        self._console.insert("end", text + "\n")
        self._console.see("end")
        lines = int(self._console.index("end-1c").split(".")[0])
        if lines > 300:
            self._console.delete("1.0", f"{lines - 300}.0")
        self._console.config(state="disabled")

    def on_close(self):
        self._close_log()
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
