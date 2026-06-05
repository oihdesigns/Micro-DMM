import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import math
import csv
import numpy as np
from scipy.optimize import curve_fit
from scipy.fft import rfft, rfftfreq
from scipy.cluster.vq import kmeans2
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class GigaTestGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Giga ADC Performance - Plotting & Export")
        
        # Default bit depth paired with each sample rate (STM32H747 ADC limits)
        self.RATE_BIT_DEFAULTS = {
            "8000":    "16",
            "16000":   "16",
            "44100":   "16",
            "100000":  "14",
            "250000":  "12",
            "500000":  "12",
            "1000000": "10",
        }

        self.port_var = tk.StringVar()
        self.bit_var = tk.StringVar(value="12")
        self.time_var = tk.StringVar(value="1000")
        self.smooth_var = tk.StringVar(value="7")
        self.log_var = tk.StringVar(value="1000")
        self.rate_var = tk.StringVar(value="500000")
        
        self.captured_data = {} # Stores {pin_name: [calibrated_values]}
        self.capture_time_ms = 1000
        self.fit_var = tk.StringVar(value="Square")
        self.diff_enable = tk.BooleanVar(value=False)
        self.diff_pos_var = tk.StringVar(value="A4")
        self.diff_neg_var = tk.StringVar(value="A5")
        
        self.pin_configs = {}
        for i in range(8):
            name = f"A{i}"
            s_val, o_val = ("44.77", "1.6204") if name == "A0" else (("10.74", "1.6204") if name == "A1" else ("1.0", "0.0"))
            self.pin_configs[name] = {
                "active": tk.BooleanVar(value=(name == "A0")),
                "scale": tk.StringVar(value=s_val),
                "offset": tk.StringVar(value=o_val)
            }
        
        self.setup_ui()

    def setup_ui(self):
        # Configuration Bar
        top_frame = ttk.Frame(self.root)
        top_frame.pack(fill="x", padx=10, pady=5)
        
        ports = [p.device for p in serial.tools.list_ports.comports()]
        ttk.Label(top_frame, text="Port:").pack(side="left")
        self.port_combo = ttk.Combobox(top_frame, textvariable=self.port_var, values=ports, width=12)
        self.port_combo.pack(side="left", padx=5)
        
        ttk.Label(top_frame, text="Rate (Hz):").pack(side="left", padx=2)
        self.rate_combo = ttk.Combobox(
            top_frame, textvariable=self.rate_var,
            values=list(self.RATE_BIT_DEFAULTS.keys()), width=9, state="readonly"
        )
        self.rate_combo.pack(side="left", padx=5)
        self.rate_combo.bind("<<ComboboxSelected>>", self._on_rate_change)

        for label, var, w in [("Bits:", self.bit_var, 4), ("MS:", self.time_var, 6),
                               ("Smooth:", self.smooth_var, 4), ("Points:", self.log_var, 5)]:
            ttk.Label(top_frame, text=label).pack(side="left", padx=2)
            ttk.Entry(top_frame, textvariable=var, width=w).pack(side="left", padx=5)

        # Calibration Grid
        pin_frame = ttk.LabelFrame(self.root, text="Calibration: (V - Offset) * Scale")
        pin_frame.pack(fill="x", padx=10, pady=5)
        
        for i in range(8):
            name = f"A{i}"
            col_off = 4 if i > 3 else 0
            row_idx = (i % 4)
            ttk.Checkbutton(pin_frame, text=name, variable=self.pin_configs[name]["active"]).grid(row=row_idx, column=0+col_off, padx=5, sticky="w")
            ttk.Entry(pin_frame, textvariable=self.pin_configs[name]["offset"], width=8).grid(row=row_idx, column=1+col_off, padx=2)
            ttk.Entry(pin_frame, textvariable=self.pin_configs[name]["scale"], width=8).grid(row=row_idx, column=2+col_off, padx=2)

        # Pseudo-Differential
        diff_frame = ttk.LabelFrame(self.root, text="Pseudo-Differential")
        diff_frame.pack(fill="x", padx=10, pady=(0, 5))
        ttk.Checkbutton(diff_frame, text="Enable", variable=self.diff_enable).pack(side="left", padx=5, pady=3)
        ttk.Label(diff_frame, text="(+)").pack(side="left")
        ttk.Combobox(diff_frame, textvariable=self.diff_pos_var,
                     values=[f"A{i}" for i in range(8)], width=5, state="readonly").pack(side="left", padx=3)
        ttk.Label(diff_frame, text=" − ").pack(side="left")
        ttk.Combobox(diff_frame, textvariable=self.diff_neg_var,
                     values=[f"A{i}" for i in range(8)], width=5, state="readonly").pack(side="left", padx=3)
        ttk.Label(diff_frame, text="(pins auto-captured; check both above to also see them individually)",
                  foreground="gray").pack(side="left", padx=10)

        # Buttons
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill="x", padx=10)
        self.run_btn = ttk.Button(btn_frame, text="Run Test", command=self.start_test_thread)
        self.run_btn.pack(side="left", pady=5)
        self.export_btn = ttk.Button(btn_frame, text="Export CSV", command=self.export_csv, state="disabled")
        self.export_btn.pack(side="left", padx=10)
        ttk.Separator(btn_frame, orient="vertical").pack(side="left", fill="y", padx=10, pady=4)
        ttk.Label(btn_frame, text="Waveform:").pack(side="left")
        ttk.Combobox(btn_frame, textvariable=self.fit_var, values=["DC", "Square", "Sine"],
                     width=8, state="readonly").pack(side="left", padx=5)
        self.fit_btn = ttk.Button(btn_frame, text="Fit", command=self.run_fit, state="disabled")
        self.fit_btn.pack(side="left")

        # Results Table
        cols = ("Pin", "Samples", "Min", "Max", "Mean", "RMS", "StdDev_S")
        self.tree = ttk.Treeview(self.root, columns=cols, show='headings', height=5)
        for col in cols:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=90, anchor="center")
        self.tree.pack(padx=10, pady=5, fill="x")

        # Plotting Area
        self.fig, self.ax = plt.subplots(figsize=(5, 3), dpi=100)
        self.ax.set_title("Captured Signal")
        self.ax.set_xlabel("Time (ms)")
        self.ax.set_ylabel("Calibrated Value")
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(padx=10, pady=10, fill="both", expand=True)

        # Fit Results
        fit_frame = ttk.LabelFrame(self.root, text="Fit Results")
        fit_frame.pack(fill="x", padx=10, pady=(0, 8))
        self.fit_result_label = ttk.Label(fit_frame, text="No fit yet.", anchor="w")
        self.fit_result_label.pack(fill="x", padx=6, pady=3)

    def _on_rate_change(self, _event=None):
        paired = self.RATE_BIT_DEFAULTS.get(self.rate_var.get())
        if paired:
            self.bit_var.set(paired)

    def start_test_thread(self):
        threading.Thread(target=self.run_test, daemon=True).start()

    def run_test(self):
        active_pins = [n for n, v in self.pin_configs.items() if v["active"].get()]

        # Always capture diff pins even if unchecked; track which were auto-added
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
        self.captured_data = {}
        self.capture_time_ms = int(self.time_var.get())
        
        try:
            with serial.Serial(self.port_var.get(), 115200, timeout=15) as ser:
                cmd = f"PINS:{','.join(active_pins)}|BITS:{self.bit_var.get()}|TIME:{self.time_var.get()}|SMOOTH:{self.smooth_var.get()}|LOG:{self.log_var.get()}|RATE:{self.rate_var.get()}\n"
                ser.write(cmd.encode())
                self.tree.delete(*self.tree.get_children())
                
                while True:
                    line = ser.readline().decode().strip()
                    if not line: continue
                    if line.startswith("ERR:ADC_BEGIN_FAILED"):
                        messagebox.showerror("ADC Error",
                            f"Board rejected {self.rate_var.get()} Hz at {self.bit_var.get()}-bit.\n"
                            "Try a lower rate or fewer bits.")
                        return
                    if "DONE" in line: break
                    
                    # Handle Summary Stats
                    if "PIN:" in line:
                        p = {item.split(':')[0]: item.split(':')[1] for item in line.split('|')}
                        pn = p['PIN']
                        scale = float(self.pin_configs[pn]["scale"].get())
                        offset = float(self.pin_configs[pn]["offset"].get())
                        v_step = 3.3 / (math.pow(2, int(p['BITS'])) - 1)
                        
                        def cal(v): return (float(v) * v_step - offset) * scale

                        self.tree.insert("", "end", values=(
                            pn, p['COUNT'], f"{cal(p['MIN']):.4f}", f"{cal(p['MAX']):.4f}",
                            f"{cal(p['MEAN']):.4f}", f"{cal(p['RMS']):.4f}", f"{(float(p['STD'])*v_step*scale):.4f}"
                        ))
                    
                    # Handle Bulk Data
                    if "DATA:" in line:
                        parts = line.split('|VALS:')
                        pin_name = parts[0].split(':')[1]
                        raw_vals = parts[1].split(',')
                        
                        scale = float(self.pin_configs[pin_name]["scale"].get())
                        offset = float(self.pin_configs[pin_name]["offset"].get())
                        v_step = 3.3 / (math.pow(2, int(self.bit_var.get())) - 1)
                        
                        self.captured_data[pin_name] = [(float(v) * v_step - offset) * scale for v in raw_vals if v]

                # Compute pseudo-differential channel
                if self.diff_enable.get():
                    pos = self.diff_pos_var.get()
                    neg = self.diff_neg_var.get()
                    if pos in self.captured_data and neg in self.captured_data:
                        n = min(len(self.captured_data[pos]), len(self.captured_data[neg]))
                        diff_key = f"{pos}-{neg}"
                        self.captured_data[diff_key] = [
                            self.captured_data[pos][i] - self.captured_data[neg][i] for i in range(n)
                        ]
                        arr = np.array(self.captured_data[diff_key])
                        self.tree.insert("", "end", values=(
                            diff_key, len(arr), f"{arr.min():.4f}", f"{arr.max():.4f}",
                            f"{arr.mean():.4f}", f"{np.sqrt(np.mean(arr**2)):.4f}", f"{arr.std(ddof=1):.4f}"
                        ))
                    # Remove auto-captured pins so they don't clutter the plot
                    for p in diff_auto:
                        self.captured_data.pop(p, None)

                self.update_plot()
                self.export_btn.config(state="normal")
                self.fit_btn.config(state="normal")
        except Exception as e:
            messagebox.showerror("Error", str(e))
        finally:
            self.run_btn.config(state="normal")

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title("Captured Signal")
        self.ax.set_xlabel("Time (ms)")
        self.ax.set_ylabel("Calibrated Value")
        self.ax.grid(True, color="gray", linewidth=0.4, alpha=0.5)
        for pin, values in self.captured_data.items():
            n = len(values)
            t = [i * self.capture_time_ms / n for i in range(n)]
            self.ax.plot(t, values, label=pin)
        if self.captured_data:
            self.ax.legend()
        self.canvas.draw()

    def run_fit(self):
        if not self.captured_data:
            return
        self.update_plot()  # redraw clean data first
        waveform = self.fit_var.get()
        results = []

        for idx, (pin, values) in enumerate(self.captured_data.items()):
            n = len(values)
            if n == 0:
                continue
            t_ms = [i * self.capture_time_ms / n for i in range(n)]
            color = self.ax.lines[idx].get_color() if idx < len(self.ax.lines) else f"C{idx}"

            if waveform == "DC":
                r = self._fit_dc(values)
            elif waveform == "Square":
                r = self._fit_square(values)
            else:
                r = self._fit_sine(values, t_ms)

            if r is None:
                results.append(f"{pin}: fit failed")
                continue

            if waveform == "DC":
                self.ax.axhline(r["High"], color=color, linestyle="--", linewidth=1.2, alpha=0.8)
                results.append(f"{pin}  Level={r['High']:.4f}")
            elif waveform == "Square":
                self.ax.axhline(r["High"], color=color, linestyle="--", linewidth=1.2, alpha=0.8, label=f"{pin} high")
                self.ax.axhline(r["Low"],  color=color, linestyle=":",  linewidth=1.2, alpha=0.8, label=f"{pin} low")
                results.append(f"{pin}  High={r['High']:.4f}  Low={r['Low']:.4f}  ΔV={r['High']-r['Low']:.4f}")
            else:
                A, f, phi, C = r["_popt"]
                t_fit = np.linspace(t_ms[0], t_ms[-1], 1000)
                y_fit = A * np.sin(2 * np.pi * f * (t_fit / 1000.0) + phi) + C
                self.ax.plot(t_fit, y_fit, "--", color=color, linewidth=1.5, alpha=0.85, label=f"{pin} fit")
                results.append(f"{pin}  High={r['High']:.4f}  Low={r['Low']:.4f}  ΔV={r['High']-r['Low']:.4f}  f={r['Freq']:.2f} Hz")

        self.ax.legend()
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
        vals = np.array(values, dtype=float)
        N = len(vals)
        if N < 4:
            return None
        dt = (t_sec[-1] - t_sec[0]) / (N - 1) if N > 1 else 1.0
        fft_mag = np.abs(rfft(vals))
        fft_mag[0] = 0  # ignore DC component
        freqs = rfftfreq(N, dt)
        freq_guess = float(freqs[np.argmax(fft_mag)]) if np.any(fft_mag > 0) else 1.0
        if freq_guess <= 0:
            freq_guess = 1.0
        A_guess = float(np.ptp(vals)) / 2
        C_guess = float(np.mean(vals))

        def sine_func(t, A, f, phi, C):
            return A * np.sin(2 * np.pi * f * t + phi) + C

        try:
            popt, _ = curve_fit(sine_func, t_sec, vals,
                                p0=[A_guess, freq_guess, 0.0, C_guess],
                                maxfev=10000)
            A, f, phi, C = popt
            return {"High": C + abs(A), "Low": C - abs(A), "Freq": abs(f), "_popt": popt}
        except (RuntimeError, ValueError):
            return None

    def export_csv(self):
        if not self.captured_data: return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV Files", "*.csv")])
        if not path: return
        
        try:
            # Find max length to align columns
            max_len = max(len(v) for v in self.captured_data.values())
            pins = list(self.captured_data.keys())
            
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(pins)
                for i in range(max_len):
                    row = [self.captured_data[p][i] if i < len(self.captured_data[p]) else "" for p in pins]
                    writer.writerow(row)
            messagebox.showinfo("Success", "Data exported successfully.")
        except Exception as e:
            messagebox.showerror("Export Error", str(e))

if __name__ == "__main__":
    root = tk.Tk()
    app = GigaTestGUI(root)
    root.mainloop()
