"""
ADS122C04 Test GUI
Requires: pip install pyserial matplotlib
"""

import sys
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

BG     = "#1a1a1a"
PANEL  = "#222222"
ACCENT = "#00ff32"
AMBER  = "#ffaa00"
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

        self._volt_lbl = tk.Label(meter, text="------- V",
                                   bg=PANEL, fg=ACCENT,
                                   font=("Courier New", 36, "bold"))
        self._volt_lbl.pack(side="left", padx=16, pady=8)

        stats = tk.Frame(meter, bg=PANEL)
        stats.pack(side="left", padx=16)
        self._raw_lbl  = self._stat_row(stats, "Raw")
        self._sps_lbl  = self._stat_row(stats, "Actual SPS")
        self._nom_lbl  = self._stat_row(stats, "Nominal SPS")
        self._mode_lbl = self._stat_row(stats, "Mode")

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

    def _stat_row(self, parent, title):
        row = tk.Frame(parent, bg=PANEL)
        row.pack(anchor="w")
        tk.Label(row, text=f"{title}:", bg=PANEL, fg=DIM,
                 width=12, anchor="w").pack(side="left")
        lbl = tk.Label(row, text="—", bg=PANEL, fg=WHITE, anchor="w", width=16)
        lbl.pack(side="left")
        return lbl

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

            self._volt_lbl.config(text=f"{volts:+.6f} V")
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

        elif line.startswith("$TEMP,"):
            parts = line[6:].split(",")
            if len(parts) < 4:
                return
            self._volt_lbl.config(text=f"{float(parts[0]):.4f} °C")
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
            self._volt_lbl.config(text="ERROR")
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

    def _update_stream_plot(self):
        now = time.monotonic()
        if now - self._last_stream_plot < 1 / 30:
            return
        self._last_stream_plot = now

        if not self._timestamps:
            return
        ts = list(self._timestamps)
        vs = list(self._voltages)

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

        # downsample to keep render fast
        if len(xs) > MAX_DISPLAY_PTS:
            step = max(len(xs) // MAX_DISPLAY_PTS, 1)
            xs = xs[::step]
            ys = ys[::step]

        self._stream_line.set_data(xs, ys)
        self._burst_line.set_data([], [])
        self._plot_title.set_text("Streaming")
        self._plot_title.set_color(DIM)
        self._ax.set_xlabel("Time (s)", color=DIM)
        self._ax.set_xlim(cutoff, t_end)   # explicit x-range keeps rolling effect
        self._apply_y_range(ys)
        self._canvas.draw_idle()

    def _update_burst_plot(self):
        if not self._burst_volts:
            return
        count = len(self._burst_volts)
        # x-axis: time in seconds
        dt = 1.0 / self._burst_sps if self._burst_sps > 0 else 1.0
        xs = [i * dt for i in range(count)]
        self._burst_line.set_data(xs, self._burst_volts)
        self._stream_line.set_data([], [])
        self._plot_title.set_text(f"Burst  ({count} samples @ {self._burst_sps:.1f} SPS)")
        self._plot_title.set_color(AMBER)
        self._ax.set_xlabel("Time (s)", color=DIM)
        self._ax.relim()
        self._apply_y_range(self._burst_volts)
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
