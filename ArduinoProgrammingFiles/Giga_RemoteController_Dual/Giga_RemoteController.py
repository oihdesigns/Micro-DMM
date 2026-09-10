import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import queue
import time
import math
import csv
import json
import os
import numpy as np
from scipy.optimize import curve_fit
from scipy.fft import rfft, rfftfreq
from scipy.cluster.vq import kmeans2
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import socket


class TcpSerial:
    """Minimal pyserial look-alike over TCP, for the Giga_RemoteController_WiFi
    sketch (listens on port 8080, same line protocol as USB serial).

    Implements just what this GUI uses: write(bytes), readline()->bytes,
    close(), .is_open, and the `with ... as ser:` context-manager form."""

    def __init__(self, host, port=8080, timeout=60):
        # Accept "host:port" in one string (e.g. what the sketch prints).
        if isinstance(host, str) and ":" in host:
            host, port_str = host.rsplit(":", 1)
            try:
                port = int(port_str)
            except ValueError:
                port = 8080
        self._sock = socket.create_connection((host, int(port)), timeout=timeout)
        self._sock.settimeout(timeout)
        self._buf = bytearray()
        self.is_open = True

    def write(self, data: bytes) -> int:
        self._sock.sendall(data)
        return len(data)

    def readline(self) -> bytes:
        """One line incl. trailing '\\n', or b'' on timeout/close (matches pyserial)."""
        while b"\n" not in self._buf:
            try:
                chunk = self._sock.recv(4096)
            except socket.timeout:
                return b""
            if not chunk:                       # peer closed (EOF)
                self.is_open = False
                line, self._buf = bytes(self._buf), bytearray()
                return line
            self._buf.extend(chunk)
        idx = self._buf.index(b"\n") + 1
        line = bytes(self._buf[:idx])
        del self._buf[:idx]
        return line

    def close(self):
        self.is_open = False
        try:
            self._sock.close()
        except OSError:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


class GigaTestGUI:
    CHANNEL_COLORS = {
        "A0": "#1f77b4",
        "A1": "#ff7f0e",
        "A2": "#2ca02c",
        "A3": "#d62728",
        "A4": "#9467bd",
        "A5": "#8c564b",
        "A6": "#e377c2",
        "A7": "#7f7f7f",
    }

    # Pre-filled WiFi target for the Giga_RemoteController_Dual sketch. Also kept
    # in the Port/IP dropdown so you can switch between USB and WiFi in one click.
    DEFAULT_WIFI_ADDR = "192.168.40.152:8080"

    # Live-stream reporting rate. Each period the board reports the mean, min,
    # max and standard deviation of every sample it took in that period.
    DEFAULT_STREAM_HZ = 4

    def __init__(self, root):
        self.root = root
        self.root.title("Giga ADC Performance - Plotting & Export")

        self.RATE_BIT_DEFAULTS = {
            "8000":    "16",
            "16000":   "16",
            "44100":   "16",
            "100000":  "14",
            "250000":  "12",
            "500000":  "12",
            "1000000": "10",
        }

        self.port_var   = tk.StringVar(value=self.DEFAULT_WIFI_ADDR)
        self.bit_var    = tk.StringVar(value="12")
        self.time_var   = tk.StringVar(value="1000")
        self.smooth_var = tk.StringVar(value="7")
        self.log_var    = tk.StringVar(value="1000")
        self.rate_var   = tk.StringVar(value="500000")

        self.captured_data    = {}
        self.capture_time_ms  = 1000
        self.capture_warning  = ""
        self.fit_var          = tk.StringVar(value="Square")
        self.diff_enable     = tk.BooleanVar(value=False)
        self.diff_pos_var    = tk.StringVar(value="A4")
        self.diff_neg_var    = tk.StringVar(value="A5")
        self.diff_type_var   = tk.StringVar(value="V")
        self.diff_offset_var = tk.StringVar(value="0.0")
        self.diff_scale_var  = tk.StringVar(value="1.0")
        self.diff_true_var   = tk.StringVar(value="1.0")

        self.diff2_enable     = tk.BooleanVar(value=False)
        self.diff2_pos_var    = tk.StringVar(value="A2")
        self.diff2_neg_var    = tk.StringVar(value="A3")
        self.diff2_type_var   = tk.StringVar(value="V")
        self.diff2_offset_var = tk.StringVar(value="0.0")
        self.diff2_scale_var  = tk.StringVar(value="1.0")
        self.diff2_true_var   = tk.StringVar(value="1.0")
        self._plot_lines     = {}   # {pin: (Line2D, axes)}

        self.trig_enable   = tk.BooleanVar(value=False)
        self.trig_mode_var = tk.StringVar(value="Normal")
        self._active_ser   = None

        self.diff_trig_thresh  = tk.StringVar(value="0")
        self.diff2_trig_thresh = tk.StringVar(value="0")
        self._diff_trig_lbl    = tk.StringVar(value="D1:")
        self._diff2_trig_lbl   = tk.StringVar(value="D2:")

        self.preset_var  = tk.StringVar()
        self.preset_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "giga_presets.json")
        self.presets     = {}

        self.live_win = None

        self.pin_configs = {}
        for i in range(8):
            name  = f"A{i}"
            s_val = "44.77" if name == "A0" else ("10.74" if name == "A1" else "1.0")
            o_val = "1.6204" if name in ("A0", "A1") else "0.0"
            self.pin_configs[name] = {
                "active":      tk.BooleanVar(value=(name == "A0")),
                "type":        tk.StringVar(value="V"),
                "offset":      tk.StringVar(value=o_val),
                "scale":       tk.StringVar(value=s_val),
                "true_val":    tk.StringVar(value="1.0"),
                "trig_thresh": tk.StringVar(value="0.5"),
            }

        self._load_presets_file()
        self.setup_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def setup_ui(self):
        # ── Top bar ───────────────────────────────────────────────────
        top = ttk.Frame(self.root)
        top.pack(fill="x", padx=10, pady=5)

        ports = [p.device for p in serial.tools.list_ports.comports()] + [self.DEFAULT_WIFI_ADDR]
        ttk.Label(top, text="Port/IP:").pack(side="left")
        # Editable: pick a COM port (USB sketch) or the board's WiFi address,
        # e.g. 192.168.1.42:8080 (WiFi sketch, shown on the touchscreen).
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, values=ports, width=18)
        self.port_combo.pack(side="left", padx=5)
        ttk.Button(top, text="↺", width=2, command=self._refresh_ports).pack(side="left", padx=(0, 5))

        ttk.Label(top, text="Rate (Hz):").pack(side="left", padx=2)
        self.rate_combo = ttk.Combobox(
            top, textvariable=self.rate_var,
            values=list(self.RATE_BIT_DEFAULTS.keys()), width=9, state="readonly")
        self.rate_combo.pack(side="left", padx=5)
        self.rate_combo.bind("<<ComboboxSelected>>", self._on_rate_change)

        for label, var, w in [("Bits:", self.bit_var, 4), ("MS:", self.time_var, 6),
                               ("Smooth:", self.smooth_var, 4), ("Points:", self.log_var, 5)]:
            ttk.Label(top, text=label).pack(side="left", padx=2)
            ttk.Entry(top, textvariable=var, width=w).pack(side="left", padx=5)

        # ── Calibration grid ──────────────────────────────────────────
        # Channels as columns (A0-A7), fields as rows — 8×7 transposed layout
        pin_frame = ttk.LabelFrame(self.root, text="Calibration: (Raw_V − Offset) × Scale")
        pin_frame.pack(fill="x", padx=10, pady=5)

        for r, lbl in enumerate(["", "Type", "Offset", "Scale", "", "True Val", ""]):
            ttk.Label(pin_frame, text=lbl, foreground="gray").grid(
                row=r, column=0, padx=(6, 3), sticky="e")

        for i in range(8):
            name = f"A{i}"
            col  = i + 1
            cfg  = self.pin_configs[name]
            ttk.Checkbutton(pin_frame, text=name, variable=cfg["active"]).grid(
                row=0, column=col, padx=4, pady=(3, 1))
            ttk.Combobox(pin_frame, textvariable=cfg["type"], values=["V", "I"],
                         width=3, state="readonly").grid(row=1, column=col, padx=3, pady=1)
            ttk.Entry(pin_frame, textvariable=cfg["offset"], width=7).grid(
                row=2, column=col, padx=3, pady=1)
            ttk.Entry(pin_frame, textvariable=cfg["scale"], width=7).grid(
                row=3, column=col, padx=3, pady=1)
            ttk.Button(pin_frame, text="Zero", width=5,
                       command=lambda n=name: self._set_zero(n)).grid(
                row=4, column=col, padx=3, pady=1)
            ttk.Entry(pin_frame, textvariable=cfg["true_val"], width=5).grid(
                row=5, column=col, padx=3, pady=1)
            ttk.Button(pin_frame, text="Scale", width=5,
                       command=lambda n=name: self._set_scale(n)).grid(
                row=6, column=col, padx=3, pady=(1, 3))

        # ── Pseudo-differential ───────────────────────────────────────
        diff_frame = ttk.LabelFrame(self.root, text="Pseudo-Differential")
        diff_frame.pack(fill="x", padx=10, pady=(0, 5))

        for c, txt in enumerate(["", "(+)", "−", "(−)", "Type", "Offset", "Scale", "True Val", ""]):
            ttk.Label(diff_frame, text=txt, foreground="gray").grid(
                row=0, column=c, padx=4, pady=(3, 0))

        def _diff_row(row, en, pv, nv, tv, ov, sv, trv, scale_cmd):
            ttk.Checkbutton(diff_frame, text=f"Diff {row}", variable=en).grid(
                row=row, column=0, padx=5, pady=2, sticky="w")
            ttk.Combobox(diff_frame, textvariable=pv,
                         values=[f"A{i}" for i in range(8)], width=5,
                         state="readonly").grid(row=row, column=1, padx=3)
            ttk.Label(diff_frame, text="−").grid(row=row, column=2)
            ttk.Combobox(diff_frame, textvariable=nv,
                         values=[f"A{i}" for i in range(8)], width=5,
                         state="readonly").grid(row=row, column=3, padx=3)
            ttk.Combobox(diff_frame, textvariable=tv, values=["V", "I"],
                         width=3, state="readonly").grid(row=row, column=4, padx=3)
            ttk.Entry(diff_frame, textvariable=ov, width=8).grid(row=row, column=5, padx=3)
            ttk.Entry(diff_frame, textvariable=sv, width=8).grid(row=row, column=6, padx=3)
            ttk.Entry(diff_frame, textvariable=trv, width=6).grid(row=row, column=7, padx=3)
            ttk.Button(diff_frame, text="Scale", width=5,
                       command=scale_cmd).grid(row=row, column=8, padx=3)

        _diff_row(1, self.diff_enable,  self.diff_pos_var,  self.diff_neg_var,
                  self.diff_type_var,  self.diff_offset_var,  self.diff_scale_var,
                  self.diff_true_var,  self._set_diff_scale)
        _diff_row(2, self.diff2_enable, self.diff2_pos_var, self.diff2_neg_var,
                  self.diff2_type_var, self.diff2_offset_var, self.diff2_scale_var,
                  self.diff2_true_var, self._set_diff2_scale)

        ttk.Label(diff_frame,
                  text="(pins auto-captured; check both above to also see them individually)",
                  foreground="gray").grid(row=3, column=0, columnspan=9,
                                          sticky="w", padx=5, pady=(1, 3))

        # ── Trigger logging ───────────────────────────────────────────
        trig_frame = ttk.LabelFrame(self.root, text="Trigger Logging  (engineering units, OR logic)")
        trig_frame.pack(fill="x", padx=10, pady=(0, 5))

        left_trig = ttk.Frame(trig_frame)
        left_trig.pack(side="left", padx=8, pady=3)
        mode_row = ttk.Frame(left_trig)
        mode_row.pack(anchor="w")
        ttk.Checkbutton(mode_row, text="Enable", variable=self.trig_enable).pack(side="left")
        ttk.Combobox(mode_row, textvariable=self.trig_mode_var,
                     values=["Normal", "Midpoint"], width=9,
                     state="readonly").pack(side="left", padx=(6, 0))
        self.trig_hint_label = ttk.Label(left_trig, text="", foreground="gray")
        self.trig_hint_label.pack(anchor="w", pady=(2, 0))

        ttk.Separator(trig_frame, orient="vertical").pack(side="left", fill="y", padx=6, pady=4)

        thresh_grid = ttk.Frame(trig_frame)
        thresh_grid.pack(side="left", pady=3)
        for i in range(8):
            name = f"A{i}"
            row, col = divmod(i, 4)
            ttk.Label(thresh_grid, text=f"{name}:").grid(
                row=row, column=col * 2, padx=(8, 1), pady=2, sticky="e")
            ttk.Entry(thresh_grid, textvariable=self.pin_configs[name]["trig_thresh"], width=7).grid(
                row=row, column=col * 2 + 1, padx=(0, 4), pady=2)

        ttk.Separator(thresh_grid, orient="horizontal").grid(
            row=2, column=0, columnspan=8, sticky="ew", pady=(3, 0))
        ttk.Label(thresh_grid, textvariable=self._diff_trig_lbl).grid(
            row=3, column=0, padx=(8, 1), pady=2, sticky="e")
        ttk.Entry(thresh_grid, textvariable=self.diff_trig_thresh, width=7).grid(
            row=3, column=1, padx=(0, 4), pady=2)
        ttk.Label(thresh_grid, textvariable=self._diff2_trig_lbl).grid(
            row=3, column=2, padx=(8, 1), pady=2, sticky="e")
        ttk.Entry(thresh_grid, textvariable=self.diff2_trig_thresh, width=7).grid(
            row=3, column=3, padx=(0, 4), pady=2)

        self.trig_enable.trace_add("write",   lambda *_: self._update_trig_hint())
        self.trig_mode_var.trace_add("write", lambda *_: self._update_trig_hint())
        self.bit_var.trace_add("write",       lambda *_: self._update_trig_hint())
        for _n in [f"A{i}" for i in range(8)]:
            self.pin_configs[_n]["trig_thresh"].trace_add("write", lambda *_: self._update_trig_hint())
        for _v in (self.diff_trig_thresh, self.diff2_trig_thresh,
                   self.diff_pos_var, self.diff_neg_var,
                   self.diff2_pos_var, self.diff2_neg_var,
                   self.diff_enable, self.diff2_enable):
            _v.trace_add("write", lambda *_: self._update_trig_hint())
        self._update_trig_hint()

        # ── Presets ───────────────────────────────────────────────────
        preset_frame = ttk.LabelFrame(self.root, text="Presets")
        preset_frame.pack(fill="x", padx=10, pady=(0, 5))
        self.preset_combo = ttk.Combobox(preset_frame, textvariable=self.preset_var, width=22)
        self.preset_combo.pack(side="left", padx=5, pady=3)
        self._update_preset_combo()
        ttk.Button(preset_frame, text="Save",   command=self._preset_save).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Load",   command=self._preset_load).pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Delete", command=self._preset_delete).pack(side="left", padx=2)
        ttk.Label(preset_frame, text="(type a name to create; select to load/delete)",
                  foreground="gray").pack(side="left", padx=10)

        # ── Action buttons ────────────────────────────────────────────
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill="x", padx=10)
        self.run_btn    = ttk.Button(btn_frame, text="Run Test",   command=self.start_test_thread)
        self.export_btn = ttk.Button(btn_frame, text="Export CSV", command=self.export_csv,  state="disabled")
        self.fit_btn    = ttk.Button(btn_frame, text="Fit",        command=self.run_fit,      state="disabled")
        self.abort_btn  = ttk.Button(btn_frame, text="Abort",      command=self._abort_capture, state="disabled")
        self.live_btn   = ttk.Button(btn_frame, text="Live Stream", command=self.open_live_stream)
        self.run_btn.pack(side="left", pady=5)
        self.export_btn.pack(side="left", padx=10)
        ttk.Button(btn_frame, text="Reset Cal", command=self._reset_cal).pack(side="left", padx=10)
        self.abort_btn.pack(side="left", padx=10)
        self.live_btn.pack(side="left", padx=10)
        ttk.Separator(btn_frame, orient="vertical").pack(side="left", fill="y", padx=10, pady=4)
        ttk.Label(btn_frame, text="Waveform:").pack(side="left")
        ttk.Combobox(btn_frame, textvariable=self.fit_var, values=["DC", "Square", "Sine"],
                     width=8, state="readonly").pack(side="left", padx=5)
        self.fit_btn.pack(side="left")

        # ── Results table ─────────────────────────────────────────────
        cols = ("Pin", "Samples", "Triggers", "Min", "Max", "Mean", "RMS", "StdDev_S")
        self.tree = ttk.Treeview(self.root, columns=cols, show="headings", height=5)
        for col in cols:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=80, anchor="center")
        self.tree.pack(padx=10, pady=5, fill="x")

        # ── Plot ──────────────────────────────────────────────────────
        self.fig, self.ax = plt.subplots(figsize=(5, 3), dpi=100)
        self.ax2 = None
        self.ax.set_title("Captured Signal")
        self.ax.set_xlabel("Time (ms)")
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(padx=10, pady=10, fill="both", expand=True)

        # ── Fit results ───────────────────────────────────────────────
        fit_frame = ttk.LabelFrame(self.root, text="Fit Results")
        fit_frame.pack(fill="x", padx=10, pady=(0, 8))
        self.fit_result_label = ttk.Label(fit_frame, text="No fit yet.", anchor="w")
        self.fit_result_label.pack(fill="x", padx=6, pady=3)

    # ------------------------------------------------------------------
    # Calibration helpers
    # ------------------------------------------------------------------
    def _on_rate_change(self, _event=None):
        paired = self.RATE_BIT_DEFAULTS.get(self.rate_var.get())
        if paired:
            self.bit_var.set(paired)

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()] + [self.DEFAULT_WIFI_ADDR]
        self.port_combo["values"] = ports
        cur = self.port_var.get()
        # Keep a manually-typed WiFi address (IP / host:port); only auto-fill
        # if the box is empty or holds a COM port that has gone away.
        if not self._is_network_port(cur) and cur not in ports:
            self.port_var.set(ports[0] if ports else "")

    @staticmethod
    def _is_network_port(port):
        """True if the Port box holds a WiFi target (IP, host:port, or *.local)
        rather than a serial COM/tty device."""
        p = (port or "").strip()
        if not p or p.upper().startswith("COM") or p.startswith("/"):
            return False
        return ("." in p) or (":" in p)

    def _open_connection(self, timeout=60, port=None):
        """Open either a USB serial port or a TCP socket, chosen from the Port box.

        `port` may be given to skip reading the Tk variable - background threads
        must not touch Tk, so the live-stream worker resolves the port on the
        main thread and passes it in here."""
        port = (port if port is not None else self.port_var.get()).strip()
        if self._is_network_port(port):
            return TcpSerial(port, 8080, timeout=timeout)
        return serial.Serial(port, 115200, timeout=timeout)

    # ------------------------------------------------------------------
    # Trigger logging helpers
    # ------------------------------------------------------------------
    def _trig_raw_counts(self):
        """Return list of 8 raw-count thresholds (0 = disabled) for A0–A7."""
        if not self.trig_enable.get():
            return [0] * 8
        try:
            bits   = int(self.bit_var.get())
            v_step = 3.3 / (2 ** bits - 1)
        except (ValueError, ZeroDivisionError):
            return [0] * 8
        result = []
        for name in [f"A{i}" for i in range(8)]:
            cfg = self.pin_configs[name]
            try:
                eng   = float(cfg["trig_thresh"].get())
                scale = float(cfg["scale"].get())
                result.append(max(1, round(eng / abs(scale) / v_step)) if eng > 0 and scale != 0 else 0)
            except (ValueError, ZeroDivisionError):
                result.append(0)

        # Overlay differential channel thresholds onto both component channels.
        # T_raw = eng_threshold / (diff_scale * v_step); applied to pos AND neg via OR logic.
        for en, pos_name, neg_name, scale_str, thresh_var in [
            (self.diff_enable.get(),  self.diff_pos_var.get(),  self.diff_neg_var.get(),
             self.diff_scale_var.get(),  self.diff_trig_thresh),
            (self.diff2_enable.get(), self.diff2_pos_var.get(), self.diff2_neg_var.get(),
             self.diff2_scale_var.get(), self.diff2_trig_thresh),
        ]:
            if not en:
                continue
            try:
                eng     = float(thresh_var.get())
                d_scale = float(scale_str)
            except (ValueError, ZeroDivisionError):
                continue
            if eng <= 0 or abs(d_scale) < 1e-12:
                continue
            T_raw = max(1, round(eng / abs(d_scale) / v_step))
            for pin_name in [pos_name, neg_name]:
                idx = int(pin_name[1:])
                result[idx] = T_raw if result[idx] == 0 else min(result[idx], T_raw)
        return result

    def _update_diff_labels(self):
        p1, n1 = self.diff_pos_var.get(), self.diff_neg_var.get()
        p2, n2 = self.diff2_pos_var.get(), self.diff2_neg_var.get()
        self._diff_trig_lbl.set(f"D1 ({p1}−{n1}):")
        self._diff2_trig_lbl.set(f"D2 ({p2}−{n2}):")

    def _update_trig_hint(self):
        self._update_diff_labels()
        if not self.trig_enable.get():
            self.trig_hint_label.config(text="disabled — evenly-spaced points")
            return
        raws  = self._trig_raw_counts()
        armed = [f"A{i}:{raws[i]}cts" for i in range(8) if raws[i] > 0]
        mode  = self.trig_mode_var.get()
        try:
            pts = int(self.log_var.get())
            mid_note = f"  |  pre={pts//2} post={pts-pts//2}" if mode == "Midpoint" else ""
        except ValueError:
            mid_note = ""

        # Note any active diff thresholds in the hint
        diff_notes = []
        try:
            bits   = int(self.bit_var.get())
            v_step = 3.3 / (2 ** bits - 1)
            for en, pos, neg, scale_str, thresh_var, lbl in [
                (self.diff_enable.get(),  self.diff_pos_var.get(),  self.diff_neg_var.get(),
                 self.diff_scale_var.get(), self.diff_trig_thresh,  "D1"),
                (self.diff2_enable.get(), self.diff2_pos_var.get(), self.diff2_neg_var.get(),
                 self.diff2_scale_var.get(), self.diff2_trig_thresh, "D2"),
            ]:
                if not en:
                    continue
                val = float(thresh_var.get())
                if val > 0:
                    T_raw = max(1, round(val / abs(float(scale_str)) / v_step))
                    diff_notes.append(f"{lbl}({pos}-{neg}):{val}→{T_raw}cts ea")
        except (ValueError, ZeroDivisionError):
            pass
        diff_hint = ("  [" + ", ".join(diff_notes) + "]") if diff_notes else ""

        self.trig_hint_label.config(
            text=(f"{mode} — {len(armed)} channel(s) armed: {', '.join(armed)}{diff_hint}{mid_note}"
                  if armed else f"{mode} — no thresholds > 0{diff_hint}"))

    # ------------------------------------------------------------------
    # Presets
    # ------------------------------------------------------------------
    def _load_presets_file(self):
        if os.path.exists(self.preset_file):
            try:
                with open(self.preset_file) as f:
                    self.presets = json.load(f)
            except Exception:
                self.presets = {}

    def _save_presets_file(self):
        with open(self.preset_file, "w") as f:
            json.dump(self.presets, f, indent=2)

    def _update_preset_combo(self):
        self.preset_combo["values"] = list(self.presets.keys())

    def _preset_save(self):
        name = self.preset_var.get().strip()
        if not name:
            messagebox.showwarning("Presets", "Enter a preset name first.")
            return
        if name not in self.presets and len(self.presets) >= 100:
            messagebox.showwarning("Presets", "Maximum 100 presets reached — delete one first.")
            return
        self.presets[name] = {
            "rate":         self.rate_var.get(),
            "bits":         self.bit_var.get(),
            "time":         self.time_var.get(),
            "smooth":       self.smooth_var.get(),
            "log":          self.log_var.get(),
            "diff_enable":  self.diff_enable.get(),
            "diff_pos":     self.diff_pos_var.get(),
            "diff_neg":     self.diff_neg_var.get(),
            "diff_type":    self.diff_type_var.get(),
            "diff_offset":  self.diff_offset_var.get(),
            "diff_scale":   self.diff_scale_var.get(),
            "diff_true":    self.diff_true_var.get(),
            "diff2_enable": self.diff2_enable.get(),
            "diff2_pos":    self.diff2_pos_var.get(),
            "diff2_neg":    self.diff2_neg_var.get(),
            "diff2_type":   self.diff2_type_var.get(),
            "diff2_offset": self.diff2_offset_var.get(),
            "diff2_scale":  self.diff2_scale_var.get(),
            "diff2_true":   self.diff2_true_var.get(),
            "trig_enable":       self.trig_enable.get(),
            "trig_mode":         self.trig_mode_var.get(),
            "trig_thresh":       {n: cfg["trig_thresh"].get() for n, cfg in self.pin_configs.items()},
            "diff_trig_thresh":  self.diff_trig_thresh.get(),
            "diff2_trig_thresh": self.diff2_trig_thresh.get(),
            "pins": {
                n: {
                    "active":   cfg["active"].get(),
                    "type":     cfg["type"].get(),
                    "offset":   cfg["offset"].get(),
                    "scale":    cfg["scale"].get(),
                    "true_val": cfg["true_val"].get(),
                }
                for n, cfg in self.pin_configs.items()
            },
        }
        self._save_presets_file()
        self._update_preset_combo()

    def _preset_load(self):
        name = self.preset_var.get().strip()
        if name not in self.presets:
            messagebox.showwarning("Presets", f"No preset named '{name}'.")
            return
        p = self.presets[name]
        self.rate_var.set(p.get("rate",    "500000"))
        self.bit_var.set(p.get("bits",    "12"))
        self.time_var.set(p.get("time",   "1000"))
        self.smooth_var.set(p.get("smooth", "7"))
        self.log_var.set(p.get("log",     "1000"))
        self.diff_enable.set(p.get("diff_enable",  False))
        self.diff_pos_var.set(p.get("diff_pos",    "A4"))
        self.diff_neg_var.set(p.get("diff_neg",    "A5"))
        self.diff_type_var.set(p.get("diff_type",   "V"))
        self.diff_offset_var.set(p.get("diff_offset", "0.0"))
        self.diff_scale_var.set(p.get("diff_scale",  "1.0"))
        self.diff_true_var.set(p.get("diff_true",   "1.0"))
        self.diff2_enable.set(p.get("diff2_enable", False))
        self.diff2_pos_var.set(p.get("diff2_pos",   "A2"))
        self.diff2_neg_var.set(p.get("diff2_neg",   "A3"))
        self.diff2_type_var.set(p.get("diff2_type",  "V"))
        self.diff2_offset_var.set(p.get("diff2_offset", "0.0"))
        self.diff2_scale_var.set(p.get("diff2_scale",  "1.0"))
        self.diff2_true_var.set(p.get("diff2_true",  "1.0"))
        self.trig_enable.set(p.get("trig_enable", False))
        self.trig_mode_var.set(p.get("trig_mode", "Normal"))
        self.diff_trig_thresh.set(p.get("diff_trig_thresh",   "0"))
        self.diff2_trig_thresh.set(p.get("diff2_trig_thresh", "0"))
        saved_thresh = p.get("trig_thresh", {})
        for n, cfg in self.pin_configs.items():
            cfg["trig_thresh"].set(saved_thresh.get(n, "0.5") if isinstance(saved_thresh, dict) else "0.5")
        for n, vals in p.get("pins", {}).items():
            if n in self.pin_configs:
                self.pin_configs[n]["active"].set(vals.get("active",   False))
                self.pin_configs[n]["type"].set(vals.get("type",     "V"))
                self.pin_configs[n]["offset"].set(vals.get("offset",   "0.0"))
                self.pin_configs[n]["scale"].set(vals.get("scale",    "1.0"))
                self.pin_configs[n]["true_val"].set(vals.get("true_val", "1.0"))

    def _preset_delete(self):
        name = self.preset_var.get().strip()
        if name not in self.presets:
            messagebox.showwarning("Presets", f"No preset named '{name}'.")
            return
        if not messagebox.askyesno("Presets", f"Delete preset '{name}'?"):
            return
        del self.presets[name]
        self._save_presets_file()
        self.preset_var.set("")
        self._update_preset_combo()

    def _mean_raw_voltage(self, pin_name):
        """Return mean raw voltage (pre-calibration) from last capture, or None."""
        data = self.captured_data.get(pin_name)
        if not data:
            return None
        cfg    = self.pin_configs[pin_name]
        scale  = float(cfg["scale"].get())
        offset = float(cfg["offset"].get())
        # Reverse: cal = (raw - offset) * scale  →  raw = cal/scale + offset
        return float(np.mean(data)) / scale + offset

    def _set_diff2_scale(self):
        self._do_diff_scale(self.diff2_pos_var, self.diff2_neg_var,
                            self.diff2_scale_var, self.diff2_true_var)

    def _do_diff_scale(self, pos_var, neg_var, scale_var, true_var):
        diff_key = f"{pos_var.get()}-{neg_var.get()}"
        data = self.captured_data.get(diff_key)
        if not data:
            messagebox.showwarning("No Data", "Run a capture first to scale the diff channel.")
            return
        try:
            d_scale  = float(scale_var.get())
            true_val = float(true_var.get())
        except ValueError:
            return
        if abs(d_scale) < 1e-12:
            return
        mean_unscaled = float(np.mean(data)) / d_scale
        if abs(mean_unscaled) < 1e-12:
            messagebox.showerror("Scale Error",
                "Diff signal is at zero — capture a non-zero reference first.")
            return
        scale_var.set(f"{true_val / mean_unscaled:.6f}")

    def _set_diff_scale(self):
        self._do_diff_scale(self.diff_pos_var, self.diff_neg_var,
                            self.diff_scale_var, self.diff_true_var)

    def _reset_cal(self):
        for cfg in self.pin_configs.values():
            cfg["offset"].set("0.0")
            cfg["scale"].set("1.0")

    def _abort_capture(self):
        ser = self._active_ser
        if ser and ser.is_open:
            try:
                ser.write(b"!ABORT\n")
            except Exception:
                pass

    def _set_zero(self, pin_name):
        raw = self._mean_raw_voltage(pin_name)
        if raw is None:
            messagebox.showwarning("No Data", f"Run a capture first to zero {pin_name}.")
            return
        self.pin_configs[pin_name]["offset"].set(f"{raw:.6f}")

    def _set_scale(self, pin_name):
        raw = self._mean_raw_voltage(pin_name)
        if raw is None:
            messagebox.showwarning("No Data", f"Run a capture first to scale {pin_name}.")
            return
        cfg      = self.pin_configs[pin_name]
        offset   = float(cfg["offset"].get())
        true_val = float(cfg["true_val"].get())
        unscaled = raw - offset
        if abs(unscaled) < 1e-12:
            messagebox.showerror("Scale Error",
                f"{pin_name}: signal is at the zero point — capture a non-zero reference first.")
            return
        cfg["scale"].set(f"{true_val / unscaled:.6f}")

    def _get_pin_type(self, pin_name):
        cfg = self.pin_configs.get(pin_name)
        if cfg:
            return cfg["type"].get()
        if pin_name == f"{self.diff_pos_var.get()}-{self.diff_neg_var.get()}":
            return self.diff_type_var.get()
        if pin_name == f"{self.diff2_pos_var.get()}-{self.diff2_neg_var.get()}":
            return self.diff2_type_var.get()
        return "V"

    def _channel_color(self, pin_name):
        if pin_name in self.CHANNEL_COLORS:
            return self.CHANNEL_COLORS[pin_name]
        if pin_name == f"{self.diff_pos_var.get()}-{self.diff_neg_var.get()}":
            return "#FF0000"          # diff 1 — red
        if pin_name == f"{self.diff2_pos_var.get()}-{self.diff2_neg_var.get()}":
            return "#FF00FF"          # diff 2 — magenta
        import hashlib
        h = int(hashlib.md5(pin_name.encode()).hexdigest()[:6], 16)
        return f"#{(h >> 16) & 0xFF:02x}{(h >> 8) & 0xFF:02x}{h & 0xFF:02x}"

    # ------------------------------------------------------------------
    # Acquisition
    # ------------------------------------------------------------------
    def open_live_stream(self):
        """Open (or raise) the live view. It reuses this window's port,
        channel selection, sample rate and calibration."""
        if self.live_win is not None and self.live_win.winfo_exists():
            self.live_win.deiconify()
            self.live_win.lift()
            return
        self.live_win = LiveStreamWindow(self)

    def _stop_live_for_capture(self):
        """A capture needs the board to itself: over WiFi the sketch holds the
        stream socket and will not accept a second connection."""
        if self.live_win is not None and self.live_win.winfo_exists() and self.live_win.running:
            self.live_win.stop()
            for _ in range(60):
                if not self.live_win.running:
                    break
                self.root.update()
                time.sleep(0.05)

    def start_test_thread(self):
        self._stop_live_for_capture()
        threading.Thread(target=self.run_test, daemon=True).start()

    def run_test(self):
        active_pins = [n for n, v in self.pin_configs.items() if v["active"].get()]

        diff_auto = set()
        for en, pv, nv in [
            (self.diff_enable.get(),  self.diff_pos_var.get(),  self.diff_neg_var.get()),
            (self.diff2_enable.get(), self.diff2_pos_var.get(), self.diff2_neg_var.get()),
        ]:
            if en:
                for p in [pv, nv]:
                    if p not in active_pins:
                        active_pins.append(p)
                        diff_auto.add(p)

        if not active_pins or not self.port_var.get():
            messagebox.showwarning("Error", "Check Port/Pins.")
            return

        self.run_btn.config(state="disabled")
        self.export_btn.config(state="disabled")
        self.captured_data   = {}
        self.capture_warning = ""
        self.capture_time_ms = int(self.time_var.get())

        try:
            with self._open_connection(timeout=60) as ser:
                trig_raws = self._trig_raw_counts()
                mid_flag  = 1 if (self.trig_enable.get() and self.trig_mode_var.get() == "Midpoint") else 0
                if mid_flag:
                    self._active_ser = ser
                    self.abort_btn.config(state="normal")
                cmd = (f"PINS:{','.join(active_pins)}"
                       f"|BITS:{self.bit_var.get()}"
                       f"|TIME:{self.time_var.get()}"
                       f"|SMOOTH:{self.smooth_var.get()}"
                       f"|LOG:{self.log_var.get()}"
                       f"|RATE:{self.rate_var.get()}"
                       f"|TRIG:{','.join(str(t) for t in trig_raws)}"
                       f"|MID:{mid_flag}\n")
                ser.write(cmd.encode())
                self.tree.delete(*self.tree.get_children())
                effective_time_ms = self.capture_time_ms

                while True:
                    line = ser.readline().decode(errors="ignore").strip()
                    if not line:
                        # A serial port stays open through quiet gaps, but a TCP
                        # peer that closed sends EOF forever — bail instead of
                        # spinning if the board dropped the link before DONE.
                        if not getattr(ser, "is_open", True):
                            messagebox.showerror(
                                "Connection closed",
                                "The board closed the connection before finishing.\n"
                                "Check the IP/port and try again.")
                            return
                        continue
                    if line.startswith("ERR:ADC_BEGIN_FAILED"):
                        messagebox.showerror("ADC Error",
                            f"Board rejected {self.rate_var.get()} Hz at {self.bit_var.get()}-bit.\n"
                            "Try a lower rate or fewer bits.")
                        return
                    if line.startswith("ERR:"):
                        messagebox.showerror("Board error", line)
                        return
                    if line.startswith("WARN|DROPPED:"):
                        # The board's M7 fell far enough behind the M4 that the
                        # shared frame ring overflowed. The data is still valid,
                        # just short - say so on the plot rather than silently.
                        self.capture_warning = (
                            f"{line.split(':')[1]} frames dropped between cores")
                        continue
                    if "DONE" in line:
                        try:
                            effective_time_ms = int(line.split("|ELAPSED:")[1])
                        except (IndexError, ValueError):
                            pass
                        break

                    if "PIN:" in line:
                        p      = {kv.split(':')[0]: kv.split(':')[1] for kv in line.split('|')}
                        pn     = p['PIN']
                        scale  = float(self.pin_configs[pn]["scale"].get())
                        offset = float(self.pin_configs[pn]["offset"].get())
                        v_step = 3.3 / (math.pow(2, int(p['BITS'])) - 1)
                        cal    = lambda v: (float(v) * v_step - offset) * scale
                        self.tree.insert("", "end", values=(
                            pn, p['COUNT'], p.get('LOGGED', '—'),
                            f"{cal(p['MIN']):.4f}", f"{cal(p['MAX']):.4f}",
                            f"{cal(p['MEAN']):.4f}", f"{cal(p['RMS']):.4f}",
                            f"{float(p['STD']) * v_step * scale:.4f}",
                        ))

                    if "DATA:" in line:
                        parts    = line.split('|VALS:')
                        pin_name = parts[0].split(':')[1]
                        raw_vals = parts[1].split(',')
                        scale    = float(self.pin_configs[pin_name]["scale"].get())
                        offset   = float(self.pin_configs[pin_name]["offset"].get())
                        v_step   = 3.3 / (math.pow(2, int(self.bit_var.get())) - 1)
                        self.captured_data[pin_name] = [
                            (float(v) * v_step - offset) * scale for v in raw_vals if v
                        ]

                def _compute_diff(en, pos, neg, off_var, scale_var):
                    if not en:
                        return
                    if pos not in self.captured_data or neg not in self.captured_data:
                        return
                    n        = min(len(self.captured_data[pos]), len(self.captured_data[neg]))
                    diff_key = f"{pos}-{neg}"
                    try:
                        d_off   = float(off_var.get())
                        d_scale = float(scale_var.get())
                    except ValueError:
                        d_off, d_scale = 0.0, 1.0
                    self.captured_data[diff_key] = [
                        (self.captured_data[pos][i] - self.captured_data[neg][i] - d_off) * d_scale
                        for i in range(n)
                    ]
                    arr = np.array(self.captured_data[diff_key])
                    self.tree.insert("", "end", values=(
                        diff_key, len(arr), len(arr),
                        f"{arr.min():.4f}", f"{arr.max():.4f}",
                        f"{arr.mean():.4f}", f"{np.sqrt(np.mean(arr**2)):.4f}",
                        f"{arr.std(ddof=1):.4f}",
                    ))

                _compute_diff(self.diff_enable.get(),  self.diff_pos_var.get(),  self.diff_neg_var.get(),
                              self.diff_offset_var,  self.diff_scale_var)
                _compute_diff(self.diff2_enable.get(), self.diff2_pos_var.get(), self.diff2_neg_var.get(),
                              self.diff2_offset_var, self.diff2_scale_var)
                for p in diff_auto:
                    self.captured_data.pop(p, None)

                self.capture_time_ms = effective_time_ms
                self.update_plot()
                self.export_btn.config(state="normal")
                self.fit_btn.config(state="normal")
        except Exception as e:
            messagebox.showerror("Error", str(e))
        finally:
            self._active_ser = None
            self.abort_btn.config(state="disabled")
            self.run_btn.config(state="normal")

    # ------------------------------------------------------------------
    # Plotting
    # ------------------------------------------------------------------
    def update_plot(self):
        if self.ax2 is not None:
            self.ax2.remove()
            self.ax2 = None
        self.ax.clear()
        self.ax.set_title("Captured Signal" if not self.capture_warning
                          else f"Captured Signal  [{self.capture_warning}]")
        self.ax.set_xlabel("Time (ms)")
        self.ax.grid(True, color="gray", linewidth=0.4, alpha=0.5)

        has_V = any(self._get_pin_type(p) == "V" for p in self.captured_data)
        has_I = any(self._get_pin_type(p) == "I" for p in self.captured_data)

        self.ax.set_ylabel("Voltage" if has_V else "")
        if has_I:
            self.ax2 = self.ax.twinx()
            self.ax2.set_ylabel("Current")

        self._plot_lines = {}
        for pin, values in self.captured_data.items():
            n   = len(values)
            t   = [i * self.capture_time_ms / n for i in range(n)]
            ax  = self.ax2 if (self._get_pin_type(pin) == "I" and self.ax2) else self.ax
            ln, = ax.plot(t, values, label=f"{pin} ({self._get_pin_type(pin)})",
                          color=self._channel_color(pin))
            self._plot_lines[pin] = (ln, ax)

        self._refresh_legend()
        self.canvas.draw()

    def _refresh_legend(self):
        all_lines  = [ln for ln, _ in self._plot_lines.values()]
        all_labels = [ln.get_label() for ln in all_lines]
        if all_lines:
            self.ax.legend(all_lines, all_labels)

    # ------------------------------------------------------------------
    # Fitting
    # ------------------------------------------------------------------
    def run_fit(self):
        if not self.captured_data:
            return
        self.update_plot()
        waveform = self.fit_var.get()
        results  = []

        for pin, values in self.captured_data.items():
            if not values:
                continue
            t_ms = [i * self.capture_time_ms / len(values) for i in range(len(values))]

            ln, plot_ax = self._plot_lines.get(pin, (None, self.ax))
            color = ln.get_color() if ln else "gray"

            r = (self._fit_dc(values)           if waveform == "DC"     else
                 self._fit_square(values)        if waveform == "Square" else
                 self._fit_sine(values, t_ms))

            if r is None:
                results.append(f"{pin}: fit failed")
                continue

            if waveform == "DC":
                plot_ax.axhline(r["High"], color=color, linestyle="--", linewidth=1.2, alpha=0.8)
                results.append(f"{pin}  Level={r['High']:.4f}")
            elif waveform == "Square":
                plot_ax.axhline(r["High"], color=color, linestyle="--", linewidth=1.2, alpha=0.8)
                plot_ax.axhline(r["Low"],  color=color, linestyle=":",  linewidth=1.2, alpha=0.8)
                results.append(f"{pin}  High={r['High']:.4f}  Low={r['Low']:.4f}  ΔV={r['High']-r['Low']:.4f}")
            else:
                A, f, phi, C = r["_popt"]
                t_fit = np.linspace(t_ms[0], t_ms[-1], 1000)
                y_fit = A * np.sin(2 * np.pi * f * (t_fit / 1000.0) + phi) + C
                plot_ax.plot(t_fit, y_fit, "--", color=color, linewidth=1.5, alpha=0.85,
                             label=f"{pin} fit")
                results.append(
                    f"{pin}  High={r['High']:.4f}  Low={r['Low']:.4f}"
                    f"  ΔV={r['High']-r['Low']:.4f}  f={r['Freq']:.2f} Hz")

        self._refresh_legend()
        self.canvas.draw()
        self.fit_result_label.config(text="     ".join(results) if results else "No results.")

    def _fit_dc(self, values):
        level = float(np.median(values))
        return {"High": level, "Low": level}

    def _fit_square(self, values):
        arr = np.array(values, dtype=float)
        if np.ptp(arr) == 0:
            return {"High": float(arr[0]), "Low": float(arr[0])}
        centroids, _ = kmeans2(arr, 2, minit="points", seed=0)
        low, high = sorted(centroids.flatten())
        return {"High": float(high), "Low": float(low)}

    def _fit_sine(self, values, t_ms):
        t_sec = np.array(t_ms) / 1000.0
        vals  = np.array(values, dtype=float)
        N     = len(vals)
        if N < 4:
            return None
        dt         = (t_sec[-1] - t_sec[0]) / (N - 1) if N > 1 else 1.0
        fft_mag    = np.abs(rfft(vals))
        fft_mag[0] = 0
        freqs      = rfftfreq(N, dt)
        freq_guess = float(freqs[np.argmax(fft_mag)]) if np.any(fft_mag > 0) else 1.0
        if freq_guess <= 0:
            freq_guess = 1.0

        def sine_func(t, A, f, phi, C):
            return A * np.sin(2 * np.pi * f * t + phi) + C

        try:
            popt, _ = curve_fit(sine_func, t_sec, vals,
                                 p0=[float(np.ptp(vals)) / 2, freq_guess, 0.0, float(np.mean(vals))],
                                 maxfev=10000)
            A, f, phi, C = popt
            return {"High": C + abs(A), "Low": C - abs(A), "Freq": abs(f), "_popt": popt}
        except (RuntimeError, ValueError):
            return None

    # ------------------------------------------------------------------
    # Export
    # ------------------------------------------------------------------
    def export_csv(self):
        if not self.captured_data:
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            filetypes=[("CSV Files", "*.csv")])
        if not path:
            return
        try:
            pins    = list(self.captured_data.keys())
            max_len = max(len(self.captured_data[p]) for p in pins)
            headers = ["Time (ms)"] + [f"{p} ({self._get_pin_type(p)})" for p in pins]
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(headers)
                for i in range(max_len):
                    t_ms = f"{i * self.capture_time_ms / max_len:.5g}"
                    row  = [t_ms] + [
                        f"{self.captured_data[p][i]:.5g}" if i < len(self.captured_data[p]) else ""
                        for p in pins
                    ]
                    writer.writerow(row)
            messagebox.showinfo("Success", "Data exported successfully.")
        except Exception as e:
            messagebox.showerror("Export Error", str(e))


class LiveStreamWindow(tk.Toplevel):
    """Live view for the Giga_RemoteController_Dual firmware.

    The board's M4 core reduces every sample it takes into one summary per
    reporting period, so what arrives here is not a subsample — it is the mean,
    min, max and standard deviation of *all* samples in that period, however
    many that was (n is shown, and at 250 kS/s with 4 Hz reporting it is tens of
    thousands). The headline number is the period mean; the band around it on
    the chart is that period's min and max.

    Protocol (one exchange per period):
        S|SEQ:n|T:ms|SPAN:ms|CH:A0|N:..|MIN:..|MAX:..|MEAN:..|STD:..|LAST:..
        E|SEQ:n|BITS:14|DROP:0
    """

    HZ_CHOICES = ["1", "2", "4", "5", "10", "20"]

    def __init__(self, app):
        super().__init__(app.root)
        self.app = app
        self.title("Giga Live Stream")
        self.geometry("1100x760")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.running   = False
        self._stop_evt = threading.Event()
        self._q        = queue.Queue()
        self._thread   = None
        self._bits     = int(app.bit_var.get() or 12)
        self._names    = []
        self._diff_legs = {}          # {"D1": "A4-A5"} as reported by the board
        self._hist      = {}
        self._cards     = {}
        self._want_legs = {}
        self._lines    = {}           # {name: (mean_line, min_line, max_line, axes)}
        self._t0_ms    = None
        self._last_draw = 0.0
        self._dropped  = 0

        self.hz_var      = tk.StringVar(value=str(app.DEFAULT_STREAM_HZ))
        self.window_var  = tk.StringVar(value="60")
        self.band_var    = tk.BooleanVar(value=True)
        self.status_var  = tk.StringVar(value="Idle.")

        self._build_ui()

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------
    def _build_ui(self):
        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=6)

        ttk.Label(top, text="Update rate (Hz):").pack(side="left")
        ttk.Combobox(top, textvariable=self.hz_var, values=self.HZ_CHOICES,
                     width=5, state="readonly").pack(side="left", padx=(4, 12))

        ttk.Label(top, text="Chart window (s):").pack(side="left")
        ttk.Entry(top, textvariable=self.window_var, width=6).pack(side="left", padx=(4, 12))

        ttk.Checkbutton(top, text="Min/max band", variable=self.band_var,
                        command=self._redraw_plot).pack(side="left", padx=(0, 12))

        self.start_btn = ttk.Button(top, text="Start", command=self.start)
        self.stop_btn  = ttk.Button(top, text="Stop",  command=self.stop, state="disabled")
        self.start_btn.pack(side="left", padx=2)
        self.stop_btn.pack(side="left", padx=2)
        ttk.Button(top, text="Clear",      command=self._clear).pack(side="left", padx=(12, 2))
        ttk.Button(top, text="Export CSV", command=self._export).pack(side="left", padx=2)

        ttk.Label(top, textvariable=self.status_var, foreground="gray").pack(side="left", padx=14)

        # Headline cards — one per streamed channel, rebuilt when a stream starts
        self.card_frame = ttk.LabelFrame(self, text="Live readings")
        self.card_frame.pack(fill="x", padx=10, pady=(0, 6))
        ttk.Label(self.card_frame,
                  text="Press Start. Channels, calibration and sample rate come "
                       "from the main window.",
                  foreground="gray").pack(padx=8, pady=8)

        self.fig, self.ax = plt.subplots(figsize=(9, 3.6), dpi=100)
        self.ax2 = None
        self.ax.set_xlabel("Time (s)")
        self.ax.grid(alpha=0.25)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=(0, 10))

    def _unit(self, name):
        if name.startswith("D"):
            v = self.app.diff_type_var if name == "D1" else self.app.diff2_type_var
            return "A" if v.get() == "I" else "V"
        return "A" if self.app.pin_configs[name]["type"].get() == "I" else "V"

    def _cal_pair(self, name):
        """(offset, scale) for a channel or a differential pair."""
        try:
            if name == "D1":
                return float(self.app.diff_offset_var.get()), float(self.app.diff_scale_var.get())
            if name == "D2":
                return float(self.app.diff2_offset_var.get()), float(self.app.diff2_scale_var.get())
            cfg = self.app.pin_configs[name]
            return float(cfg["offset"].get()), float(cfg["scale"].get())
        except (ValueError, KeyError):
            return 0.0, 1.0

    def _cal(self, name, raw):
        off, scale = self._cal_pair(name)
        v_step = 3.3 / (2 ** self._bits - 1)
        return (raw * v_step - off) * scale

    def _cal_spread(self, name, raw):
        """A spread (min-max width, sigma) carries the gain but not the offset."""
        _, scale = self._cal_pair(name)
        v_step = 3.3 / (2 ** self._bits - 1)
        return raw * v_step * abs(scale)

    def _build_cards(self):
        for w in self.card_frame.winfo_children():
            w.destroy()
        self._cards = {}
        n = len(self._names)
        if n == 0:
            return
        cols  = 1 if n <= 2 else (2 if n <= 6 else 3)
        big   = 40 if n <= 2 else (30 if n <= 4 else (24 if n <= 6 else 19))
        for i, name in enumerate(self._names):
            cell = ttk.Frame(self.card_frame, relief="groove", borderwidth=1)
            cell.grid(row=i // cols, column=i % cols, sticky="nsew", padx=4, pady=4)
            head = name + (f"  ({self._diff_legs[name]})" if name in self._diff_legs else "")
            tk.Label(cell, text=head, fg=self.app._channel_color(name),
                     font=("TkDefaultFont", 10, "bold")).pack(anchor="w", padx=8, pady=(4, 0))
            val = tk.Label(cell, text="--", font=("TkFixedFont", big, "bold"), anchor="w")
            val.pack(anchor="w", padx=8)
            sub = tk.Label(cell, text="", font=("TkDefaultFont", 8), fg="gray", anchor="w",
                           justify="left")
            sub.pack(anchor="w", padx=8, pady=(0, 5))
            self._cards[name] = (val, sub)
        for c in range(cols):
            self.card_frame.columnconfigure(c, weight=1)

    # ------------------------------------------------------------------
    # Stream control
    # ------------------------------------------------------------------
    def _selected_channels(self):
        """What the main window has configured, as board stat-slot names."""
        names = [n for n, v in self.app.pin_configs.items() if v["active"].get()]
        if self.app.diff_enable.get():
            names.append("D1")
        if self.app.diff2_enable.get():
            names.append("D2")
        return names

    def _diff_pins(self, name):
        """The (pos, neg) pins this window means by D1 / D2."""
        if name == "D1":
            return self.app.diff_pos_var.get(), self.app.diff_neg_var.get()
        return self.app.diff2_pos_var.get(), self.app.diff2_neg_var.get()

    def start(self):
        if self.running:
            return
        names = self._selected_channels()
        if not names:
            messagebox.showwarning("Live stream", "No channels selected in the main window.")
            return
        if not self.app.port_var.get():
            messagebox.showwarning("Live stream", "Set a Port/IP in the main window.")
            return

        self._bits      = int(self.app.bit_var.get() or 12)
        self._names     = names
        self._diff_legs = {}
        self._hist      = {n: {"t": [], "mean": [], "min": [], "max": [], "std": [], "n": []}
                           for n in names}
        self._t0_ms     = None
        self._dropped   = 0
        self._build_cards()
        self._rebuild_axes()

        # Tk variables belong to the main thread, so freeze every value the
        # worker needs before starting it.
        self._port = self.app.port_var.get().strip()
        # Name the legs behind D1/D2 explicitly. The board has its own Diff
        # config, and its built-in preset table drifts from giga_presets.json -
        # so "Diff 1" on the board is not necessarily Diff 1 here. Leaving the
        # board to pick pairs one number at a time silently pairs its pins with
        # this window's calibration, which reads as a plausible wrong number.
        self._want_legs = {}
        diff_fields = ""
        for n in names:
            if not n.startswith("D"):
                continue
            pos, neg = self._diff_pins(n)
            self._want_legs[n] = f"{pos}-{neg}"
            diff_fields += f"|{n}:{pos}-{neg}"

        self._cmd  = ("STREAM:" + ",".join(names)
                      + diff_fields
                      + "|BITS:"   + str(self._bits)
                      + "|RATE:"   + self.app.rate_var.get()
                      + "|SMOOTH:" + self.app.smooth_var.get()
                      + "|HZ:"     + self.hz_var.get() + chr(10))

        self._stop_evt = threading.Event()
        self._q        = queue.Queue()
        self.running   = True
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.status_var.set("Connecting...")
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        self.after(40, self._drain)

    def stop(self):
        if not self.running:
            return
        self._stop_evt.set()
        self.stop_btn.config(state="disabled")
        self.status_var.set("Stopping...")

    def _on_close(self):
        self.stop()
        # Give the worker a moment to send STOP and close the socket, so the
        # board releases it instead of waiting out its own timeout.
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self.app.live_win = None
        self.destroy()

    # ------------------------------------------------------------------
    # Worker thread
    # ------------------------------------------------------------------
    @staticmethod
    def _kv(line):
        out = {}
        for part in line.split("|"):
            if ":" in part:
                k, _, v = part.partition(":")
                out[k] = v
        return out

    def _worker(self):
        ser = None
        try:
            ser = self.app._open_connection(timeout=5, port=self._port)
        except Exception as e:
            self._q.put(("err", f"Could not open {self._port}: {e}"))
            self._q.put(("closed", None))
            return
        try:
            ser.write(self._cmd.encode())

            period, t_ms = {}, 0
            while not self._stop_evt.is_set():
                raw = ser.readline()
                line = raw.decode(errors="ignore").strip()
                if not line:
                    # A serial port stays open through quiet gaps; a TCP peer
                    # that closed reports EOF forever, so bail rather than spin.
                    if not getattr(ser, "is_open", True):
                        self._q.put(("err", "The board closed the connection."))
                        break
                    continue
                if line.startswith("ERR:"):
                    self._q.put(("err", line))
                    break
                if line.startswith("STREAMOFF"):
                    break
                if line.startswith("STREAMON"):
                    self._q.put(("on", self._kv(line)))
                    continue
                if line.startswith("S|"):
                    d = self._kv(line)
                    period[d.get("CH", "?")] = d
                    t_ms = int(d.get("T", t_ms))
                elif line.startswith("E|"):
                    e = self._kv(line)
                    if period:
                        self._q.put(("period", (t_ms, period, int(e.get("DROP", 0)))))
                    period = {}

            # Ask the board to stand down and drain its acknowledgement, so the
            # socket is released cleanly rather than being left for its timeout.
            try:
                ser.write(b"STOP\n")
                deadline = time.time() + 2.0
                while time.time() < deadline:
                    ack = ser.readline()
                    if not ack or b"STREAMOFF" in ack:
                        break
            except Exception:
                pass
        except Exception as e:
            self._q.put(("err", str(e)))
        finally:
            try:
                ser.close()
            except Exception:
                pass
            self._q.put(("closed", None))

    # ------------------------------------------------------------------
    # Tk thread: consume the queue and repaint
    # ------------------------------------------------------------------
    def _drain(self):
        got_period = False
        try:
            while True:
                kind, payload = self._q.get_nowait()
                if kind == "err":
                    self.status_var.set(payload)
                elif kind == "on":
                    for d in ("D1", "D2"):
                        if d in payload:
                            self._diff_legs[d] = payload[d]
                    if self._diff_legs:
                        self._build_cards()
                    # The board echoes the legs it actually used. If they are not
                    # the ones asked for, the numbers are being calibrated
                    # against the wrong pins - say so rather than plot it.
                    wrong = [f"{d}: asked {w}, board used {self._diff_legs.get(d)}"
                             for d, w in self._want_legs.items()
                             if self._diff_legs.get(d) not in (None, w)]
                    if wrong:
                        self.status_var.set("PIN MISMATCH - " + "; ".join(wrong))
                    else:
                        self.status_var.set(
                            f"Streaming at {payload.get('HZ','?')} Hz, "
                            f"{payload.get('RATE','?')} S/s, {payload.get('BITS','?')}-bit")
                elif kind == "period":
                    self._on_period(*payload)
                    got_period = True
                elif kind == "closed":
                    self.running = False
                    self.start_btn.config(state="normal")
                    self.stop_btn.config(state="disabled")
                    if self.status_var.get().startswith("Stopping"):
                        self.status_var.set("Stopped.")
        except queue.Empty:
            pass

        if got_period:
            self._update_cards()
            # Repaint at most 10x/s: at 20 Hz reporting the numbers still update
            # every period, the chart just coalesces.
            now = time.time()
            if now - self._last_draw >= 0.1:
                self._last_draw = now
                self._redraw_plot()

        if self.running:
            self.after(40, self._drain)

    def _on_period(self, t_ms, period, dropped):
        self._dropped = dropped
        if self._t0_ms is None:
            self._t0_ms = t_ms
        t_s = (t_ms - self._t0_ms) / 1000.0
        for name, d in period.items():
            h = self._hist.get(name)
            if h is None:
                continue
            h["t"].append(t_s)
            h["mean"].append(self._cal(name, float(d["MEAN"])))
            h["min"].append(self._cal(name, float(d["MIN"])))
            h["max"].append(self._cal(name, float(d["MAX"])))
            h["std"].append(self._cal_spread(name, float(d["STD"])))
            h["n"].append(int(d["N"]))

        # Trim to the chart window.
        try:
            span = float(self.window_var.get())
        except ValueError:
            span = 60.0
        if span > 0:
            cutoff = t_s - span
            for h in self._hist.values():
                while h["t"] and h["t"][0] < cutoff:
                    for key in h:
                        h[key].pop(0)

    def _update_cards(self):
        for name, (val, sub) in self._cards.items():
            h = self._hist.get(name)
            if not h or not h["t"]:
                continue
            u = self._unit(name)
            val.config(text=f"{h['mean'][-1]:+.5g} {u}")
            sub.config(text=(f"min {h['min'][-1]:.5g}   max {h['max'][-1]:.5g}   "
                             f"σ {h['std'][-1]:.4g}   n = {h['n'][-1]:,}"))

    def _rebuild_axes(self):
        """One axis per unit — volts left, amps right, same as the capture plot."""
        if self.ax2 is not None:
            self.ax2.remove()
            self.ax2 = None
        self.ax.clear()
        self.ax.set_xlabel("Time (s)")
        self.ax.grid(alpha=0.25)
        self._lines = {}

        units = {n: self._unit(n) for n in self._names}
        has_i = any(u == "A" for u in units.values())
        has_v = any(u == "V" for u in units.values())
        self.ax.set_ylabel("Volts" if has_v else "Amps")
        if has_i and has_v:
            self.ax2 = self.ax.twinx()
            self.ax2.set_ylabel("Amps")

        for name in self._names:
            target = self.ax2 if (units[name] == "A" and self.ax2 is not None) else self.ax
            color  = self.app._channel_color(name)
            mean,  = target.plot([], [], color=color, lw=1.8, label=name)
            lo,    = target.plot([], [], color=color, lw=0.8, ls="--", alpha=0.5)
            hi,    = target.plot([], [], color=color, lw=0.8, ls="--", alpha=0.5)
            self._lines[name] = (mean, lo, hi, target)

        handles = [v[0] for v in self._lines.values()]
        if handles:
            self.ax.legend(handles, [h.get_label() for h in handles],
                           loc="upper left", fontsize=8, ncol=min(5, len(handles)))
        self.canvas.draw_idle()

    def _redraw_plot(self):
        if not self._lines:
            return
        show_band = self.band_var.get()
        for name, (mean, lo, hi, _target) in self._lines.items():
            h = self._hist.get(name)
            if not h:
                continue
            mean.set_data(h["t"], h["mean"])
            if show_band:
                lo.set_data(h["t"], h["min"])
                hi.set_data(h["t"], h["max"])
            else:
                lo.set_data([], [])
                hi.set_data([], [])
        for axis in (self.ax, self.ax2):
            if axis is None:
                continue
            axis.relim()
            axis.autoscale_view()
        title = "Live stream - line = period mean, dashes = period min/max"
        if self._dropped:
            title += f"   [{self._dropped} frames dropped]"
        self.ax.set_title(title, fontsize=9)
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    def _clear(self):
        for h in self._hist.values():
            for key in h:
                h[key].clear()
        self._t0_ms = None
        self._redraw_plot()

    def _export(self):
        if not any(h["t"] for h in self._hist.values()):
            messagebox.showinfo("Export", "Nothing recorded yet.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            filetypes=[("CSV Files", "*.csv")])
        if not path:
            return
        names = [n for n in self._names if self._hist[n]["t"]]
        rows  = max(len(self._hist[n]["t"]) for n in names)
        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                header = ["Time (s)"]
                for n in names:
                    u = self._unit(n)
                    header += [f"{n} mean ({u})", f"{n} min ({u})", f"{n} max ({u})",
                               f"{n} std ({u})", f"{n} samples"]
                w.writerow(header)
                for i in range(rows):
                    h0 = self._hist[names[0]]
                    row = [f"{h0['t'][i]:.4f}" if i < len(h0["t"]) else ""]
                    for n in names:
                        h = self._hist[n]
                        if i < len(h["t"]):
                            row += [f"{h['mean'][i]:.6g}", f"{h['min'][i]:.6g}",
                                    f"{h['max'][i]:.6g}", f"{h['std'][i]:.6g}", h["n"][i]]
                        else:
                            row += ["", "", "", "", ""]
                    w.writerow(row)
            messagebox.showinfo("Export", "Live stream exported.")
        except Exception as e:
            messagebox.showerror("Export Error", str(e))


if __name__ == "__main__":
    root = tk.Tk()
    app  = GigaTestGUI(root)
    root.mainloop()
