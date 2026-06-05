import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import math
import csv
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class GigaTestGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Giga ADC Performance - Plotting & Export")
        
        self.port_var = tk.StringVar()
        self.bit_var = tk.StringVar(value="12")
        self.time_var = tk.StringVar(value="1000")
        self.smooth_var = tk.StringVar(value="7")
        self.log_var = tk.StringVar(value="1000")
        
        self.captured_data = {} # Stores {pin_name: [calibrated_values]}
        
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

        # Buttons
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill="x", padx=10)
        self.run_btn = ttk.Button(btn_frame, text="Run Test", command=self.start_test_thread)
        self.run_btn.pack(side="left", pady=5)
        self.export_btn = ttk.Button(btn_frame, text="Export CSV", command=self.export_csv, state="disabled")
        self.export_btn.pack(side="left", padx=10)

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
        self.ax.set_xlabel("Sample #")
        self.ax.set_ylabel("Calibrated Value")
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(padx=10, pady=10, fill="both", expand=True)

    def start_test_thread(self):
        threading.Thread(target=self.run_test, daemon=True).start()

    def run_test(self):
        active_pins = [n for n, v in self.pin_configs.items() if v["active"].get()]
        if not active_pins or not self.port_var.get():
            messagebox.showwarning("Error", "Check Port/Pins.")
            return

        self.run_btn.config(state="disabled")
        self.export_btn.config(state="disabled")
        self.captured_data = {}
        
        try:
            with serial.Serial(self.port_var.get(), 115200, timeout=15) as ser:
                cmd = f"PINS:{','.join(active_pins)}|BITS:{self.bit_var.get()}|TIME:{self.time_var.get()}|SMOOTH:{self.smooth_var.get()}|LOG:{self.log_var.get()}\n"
                ser.write(cmd.encode())
                self.tree.delete(*self.tree.get_children())
                
                while True:
                    line = ser.readline().decode().strip()
                    if not line: continue
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

                self.update_plot()
                self.export_btn.config(state="normal")
        except Exception as e:
            messagebox.showerror("Error", str(e))
        finally:
            self.run_btn.config(state="normal")

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title("Captured Signal")
        for pin, values in self.captured_data.items():
            self.ax.plot(values, label=pin)
        if self.captured_data:
            self.ax.legend()
        self.canvas.draw()

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
