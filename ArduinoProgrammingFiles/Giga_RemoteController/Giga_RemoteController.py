import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
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

        self.port_var   = tk.StringVar()
        self.bit_var    = tk.StringVar(value="12")
        self.time_var   = tk.StringVar(value="1000")
        self.smooth_var = tk.StringVar(value="7")
        self.log_var    = tk.StringVar(value="1000")
        self.rate_var   = tk.StringVar(value="500000")

        self.captured_data   = {}
        self.capture_time_ms = 1000
        self.fit_var         = tk.StringVar(value="Square")
        self.diff_enable     = tk.BooleanVar(value=False)
        self.diff_pos_var    = tk.StringVar(value="A4")
        self.diff_neg_var    = tk.StringVar(value="A5")
        self._plot_lines     = {}   # {pin: (Line2D, axes)}

        self.preset_var  = tk.StringVar()
        self.preset_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "giga_presets.json")
        self.presets     = {}

        self.pin_configs = {}
        for i in range(8):
            name  = f"A{i}"
            s_val = "44.77" if name == "A0" else ("10.74" if name == "A1" else "1.0")
            o_val = "1.6204" if name in ("A0", "A1") else "0.0"
            self.pin_configs[name] = {
                "active":   tk.BooleanVar(value=(name == "A0")),
                "type":     tk.StringVar(value="V"),
                "offset":   tk.StringVar(value=o_val),
                "scale":    tk.StringVar(value=s_val),
                "true_val": tk.StringVar(value="1.0"),
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

        ports = [p.device for p in serial.tools.list_ports.comports()]
        ttk.Label(top, text="Port:").pack(side="left")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, values=ports, width=12)
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
        # Per group (7 cols): [Pin/chk] [Type] [Offset] [Scale] [Zero] [True Val] [Scale btn]
        # Left group cols 0-6 | gap col 7 | Right group cols 8-14
        pin_frame = ttk.LabelFrame(self.root, text="Calibration: (Raw_V − Offset) × Scale")
        pin_frame.pack(fill="x", padx=10, pady=5)

        hdr = ["Pin", "Type", "Offset", "Scale", "", "True Val", ""]
        for c, txt in enumerate(hdr):
            ttk.Label(pin_frame, text=txt, foreground="gray").grid(row=0, column=c,     padx=2)
            ttk.Label(pin_frame, text=txt, foreground="gray").grid(row=0, column=c + 8, padx=2)

        for i in range(8):
            name    = f"A{i}"
            col_off = 8 if i > 3 else 0
            row     = (i % 4) + 1
            cfg     = self.pin_configs[name]

            ttk.Checkbutton(pin_frame, text=name, variable=cfg["active"]).grid(
                row=row, column=0 + col_off, padx=5, sticky="w")
            ttk.Combobox(pin_frame, textvariable=cfg["type"], values=["V", "I"],
                         width=3, state="readonly").grid(
                row=row, column=1 + col_off, padx=2)
            ttk.Entry(pin_frame, textvariable=cfg["offset"], width=8).grid(
                row=row, column=2 + col_off, padx=2)
            ttk.Entry(pin_frame, textvariable=cfg["scale"], width=8).grid(
                row=row, column=3 + col_off, padx=2)
            ttk.Button(pin_frame, text="Zero", width=5,
                       command=lambda n=name: self._set_zero(n)).grid(
                row=row, column=4 + col_off, padx=2)
            ttk.Entry(pin_frame, textvariable=cfg["true_val"], width=6).grid(
                row=row, column=5 + col_off, padx=2)
            ttk.Button(pin_frame, text="Scale", width=5,
                       command=lambda n=name: self._set_scale(n)).grid(
                row=row, column=6 + col_off, padx=2)

        # ── Pseudo-differential ───────────────────────────────────────
        diff_frame = ttk.LabelFrame(self.root, text="Pseudo-Differential")
        diff_frame.pack(fill="x", padx=10, pady=(0, 5))
        ttk.Checkbutton(diff_frame, text="Enable", variable=self.diff_enable).pack(side="left", padx=5, pady=3)
        ttk.Label(diff_frame, text="(+)").pack(side="left")
        ttk.Combobox(diff_frame, textvariable=self.diff_pos_var,
                     values=[f"A{i}" for i in range(8)], width=5, state="readonly").pack(side="left", padx=3)
        ttk.Label(diff_frame, text=" − ").pack(side="left")
        ttk.Combobox(diff_frame, textvariable=self.diff_neg_var,
                     values=[f"A{i}" for i in range(8)], width=5, state="readonly").pack(side="left", padx=3)
        ttk.Label(diff_frame,
                  text="(pins auto-captured; check both above to also see them individually)",
                  foreground="gray").pack(side="left", padx=10)

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
        self.run_btn.pack(side="left", pady=5)
        self.export_btn.pack(side="left", padx=10)
        ttk.Button(btn_frame, text="Reset Cal", command=self._reset_cal).pack(side="left", padx=10)
        ttk.Separator(btn_frame, orient="vertical").pack(side="left", fill="y", padx=10, pady=4)
        ttk.Label(btn_frame, text="Waveform:").pack(side="left")
        ttk.Combobox(btn_frame, textvariable=self.fit_var, values=["DC", "Square", "Sine"],
                     width=8, state="readonly").pack(side="left", padx=5)
        self.fit_btn.pack(side="left")

        # ── Results table ─────────────────────────────────────────────
        cols = ("Pin", "Samples", "Min", "Max", "Mean", "RMS", "StdDev_S")
        self.tree = ttk.Treeview(self.root, columns=cols, show="headings", height=5)
        for col in cols:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=90, anchor="center")
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
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if self.port_var.get() not in ports:
            self.port_var.set(ports[0] if ports else "")

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
        if name not in self.presets and len(self.presets) >= 10:
            messagebox.showwarning("Presets", "Maximum 10 presets reached — delete one first.")
            return
        self.presets[name] = {
            "rate":   self.rate_var.get(),
            "bits":   self.bit_var.get(),
            "time":   self.time_var.get(),
            "smooth": self.smooth_var.get(),
            "log":    self.log_var.get(),
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
        self.rate_var.set(p.get("rate",   "500000"))
        self.bit_var.set(p.get("bits",    "12"))
        self.time_var.set(p.get("time",   "1000"))
        self.smooth_var.set(p.get("smooth", "7"))
        self.log_var.set(p.get("log",     "1000"))
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

    def _reset_cal(self):
        for cfg in self.pin_configs.values():
            cfg["offset"].set("0.0")
            cfg["scale"].set("1.0")

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
        """Returns 'V' or 'I'; derived channels (e.g. diff) default to 'V'."""
        cfg = self.pin_configs.get(pin_name)
        return cfg["type"].get() if cfg else "V"

    def _channel_color(self, pin_name):
        if pin_name in self.CHANNEL_COLORS:
            return self.CHANNEL_COLORS[pin_name]
        # stable color for derived channels (e.g. "A4-A5") via name hash
        import hashlib
        h = int(hashlib.md5(pin_name.encode()).hexdigest()[:6], 16)
        return f"#{(h >> 16) & 0xFF:02x}{(h >> 8) & 0xFF:02x}{h & 0xFF:02x}"

    # ------------------------------------------------------------------
    # Acquisition
    # ------------------------------------------------------------------
    def start_test_thread(self):
        threading.Thread(target=self.run_test, daemon=True).start()

    def run_test(self):
        active_pins = [n for n, v in self.pin_configs.items() if v["active"].get()]

        diff_auto = set()
        if self.diff_enable.get():
            for p in [self.diff_pos_var.get(), self.diff_neg_var.get()]:
                if p not in active_pins:
                    active_pins.append(p)
                    diff_auto.add(p)

        if not active_pins or not self.port_var.get():
            messagebox.showwarning("Error", "Check Port/Pins.")
            return

        self.run_btn.config(state="disabled")
        self.export_btn.config(state="disabled")
        self.captured_data   = {}
        self.capture_time_ms = int(self.time_var.get())

        try:
            with serial.Serial(self.port_var.get(), 115200, timeout=15) as ser:
                cmd = (f"PINS:{','.join(active_pins)}"
                       f"|BITS:{self.bit_var.get()}"
                       f"|TIME:{self.time_var.get()}"
                       f"|SMOOTH:{self.smooth_var.get()}"
                       f"|LOG:{self.log_var.get()}"
                       f"|RATE:{self.rate_var.get()}\n")
                ser.write(cmd.encode())
                self.tree.delete(*self.tree.get_children())

                while True:
                    line = ser.readline().decode().strip()
                    if not line:
                        continue
                    if line.startswith("ERR:ADC_BEGIN_FAILED"):
                        messagebox.showerror("ADC Error",
                            f"Board rejected {self.rate_var.get()} Hz at {self.bit_var.get()}-bit.\n"
                            "Try a lower rate or fewer bits.")
                        return
                    if "DONE" in line:
                        break

                    if "PIN:" in line:
                        p      = {kv.split(':')[0]: kv.split(':')[1] for kv in line.split('|')}
                        pn     = p['PIN']
                        scale  = float(self.pin_configs[pn]["scale"].get())
                        offset = float(self.pin_configs[pn]["offset"].get())
                        v_step = 3.3 / (math.pow(2, int(p['BITS'])) - 1)
                        cal    = lambda v: (float(v) * v_step - offset) * scale
                        self.tree.insert("", "end", values=(
                            pn, p['COUNT'],
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

                if self.diff_enable.get():
                    pos = self.diff_pos_var.get()
                    neg = self.diff_neg_var.get()
                    if pos in self.captured_data and neg in self.captured_data:
                        n        = min(len(self.captured_data[pos]), len(self.captured_data[neg]))
                        diff_key = f"{pos}-{neg}"
                        self.captured_data[diff_key] = [
                            self.captured_data[pos][i] - self.captured_data[neg][i] for i in range(n)
                        ]
                        arr = np.array(self.captured_data[diff_key])
                        self.tree.insert("", "end", values=(
                            diff_key, len(arr),
                            f"{arr.min():.4f}", f"{arr.max():.4f}",
                            f"{arr.mean():.4f}", f"{np.sqrt(np.mean(arr**2)):.4f}",
                            f"{arr.std(ddof=1):.4f}",
                        ))
                    for p in diff_auto:
                        self.captured_data.pop(p, None)

                self.update_plot()
                self.export_btn.config(state="normal")
                self.fit_btn.config(state="normal")
        except Exception as e:
            messagebox.showerror("Error", str(e))
        finally:
            self.run_btn.config(state="normal")

    # ------------------------------------------------------------------
    # Plotting
    # ------------------------------------------------------------------
    def update_plot(self):
        if self.ax2 is not None:
            self.ax2.remove()
            self.ax2 = None
        self.ax.clear()
        self.ax.set_title("Captured Signal")
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
            ln, = ax.plot(t, values, label=pin, color=self._channel_color(pin))
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
            max_len = max(len(v) for v in self.captured_data.values())
            pins    = list(self.captured_data.keys())
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(pins)
                for i in range(max_len):
                    writer.writerow([
                        self.captured_data[p][i] if i < len(self.captured_data[p]) else ""
                        for p in pins
                    ])
            messagebox.showinfo("Success", "Data exported successfully.")
        except Exception as e:
            messagebox.showerror("Export Error", str(e))


if __name__ == "__main__":
    root = tk.Tk()
    app  = GigaTestGUI(root)
    root.mainloop()
