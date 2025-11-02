import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import serial, serial.tools.list_ports, threading, time, csv
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class SerialLoggerApp:
    def __init__(self, master):
        self.master = master
        self.master.title("Arduino Differential Meter Logger")
        self.master.geometry("1000x850")

        # ---- State variables ----
        self.serial_conn = None
        self.running = False
        self.logging = False
        self.data = []
        self.latest_vdc = tk.StringVar(value="---")
        self.rolloff_value = tk.DoubleVar(value=30)
        self.rolloff_unit = tk.StringVar(value="seconds")
        self.live_updates = True
        self.status_text = tk.StringVar(value="Unknown")
        self.relay_text = tk.StringVar(value="Unknown")
        self.relay_v_text = tk.StringVar(value="--- V")
        self.last_line_time = time.time()
        self.timeout_seconds = 2.0
        self.last_status_color = "gray"
        self.relay_v_fault = False  # track current fault state

        # ---- GUI styling ----
        style = ttk.Style()
        style.configure("TLabel", font=("Segoe UI", 12))
        style.configure("TButton", font=("Segoe UI", 11))
        pad = {"padx": 6, "pady": 6}

        # ---- Serial connection row ----
        self.port_var = tk.StringVar()
        self.refresh_ports()
        ttk.Label(master, text="Serial Port:").grid(row=0, column=0, sticky="e", **pad)
        self.port_combo = ttk.Combobox(master, textvariable=self.port_var,
                                       values=self.port_list, width=15)
        self.port_combo.grid(row=0, column=1, sticky="w", **pad)
        ttk.Button(master, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, **pad)
        self.connect_btn = ttk.Button(master, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, **pad)

        # ---- Control buttons ----
        self.start_btn = ttk.Button(master, text="Start Logging",
                                    command=self.start_logging, state="disabled")
        self.start_btn.grid(row=1, column=0, **pad)
        self.stop_btn = tk.Button(master, text="Force Stop", bg="#d32f2f", fg="white",
                                  font=("Segoe UI", 11, "bold"), command=self.force_stop)
        self.stop_btn.grid(row=1, column=1, **pad)
        self.go_btn = tk.Button(master, text="Go", bg="#2e7d32", fg="white",
                                font=("Segoe UI", 11, "bold"), command=self.go_command)
        self.go_btn.grid(row=1, column=2, **pad)
        self.live_btn = ttk.Button(master, text="Live Updates: ON",
                                   command=self.toggle_live_updates)
        self.live_btn.grid(row=1, column=3, **pad)

        # ---- Custom command row ----
        ttk.Label(master, text="Custom Command:").grid(row=2, column=0, sticky="e", **pad)
        self.cmd_entry = ttk.Entry(master, width=20)
        self.cmd_entry.grid(row=2, column=1, **pad)
        ttk.Button(master, text="Send", command=self.send_command).grid(row=2, column=2, **pad)

        # ---- Graph roll-off ----
        ttk.Label(master, text="Graph Rolloff Window:").grid(row=3, column=0, sticky="e", **pad)
        ttk.Entry(master, textvariable=self.rolloff_value, width=10)\
            .grid(row=3, column=1, sticky="w", **pad)
        self.rolloff_units = ttk.Combobox(master, textvariable=self.rolloff_unit,
                                          values=["milliseconds", "seconds",
                                                  "minutes", "hours", "points"],
                                          width=12)
        self.rolloff_units.grid(row=3, column=2, **pad)
        ttk.Label(master, text="(Display window)").grid(row=3, column=3, sticky="w", **pad)

        # ---- Status indicators ----
        status_frame = ttk.LabelFrame(master, text="Status Indicators")
        status_frame.grid(row=4, column=0, columnspan=4, padx=10, pady=5, sticky="ew")

        # System
        self.status_canvas = tk.Canvas(status_frame, width=25, height=25,
                                       highlightthickness=1, highlightbackground="gray")
        self.status_canvas.grid(row=0, column=0, padx=10, pady=5)
        self.status_indicator = self.status_canvas.create_oval(5, 5, 20, 20, fill="gray")
        ttk.Label(status_frame, text="System Status:").grid(row=0, column=1, sticky="e")
        ttk.Label(status_frame, textvariable=self.status_text).grid(row=0, column=2, sticky="w")

        # Relay state
        self.relay_canvas = tk.Canvas(status_frame, width=25, height=25,
                                      highlightthickness=1, highlightbackground="gray")
        self.relay_canvas.grid(row=1, column=0, padx=10, pady=5)
        self.relay_indicator = self.relay_canvas.create_oval(5, 5, 20, 20, fill="gray")
        ttk.Label(status_frame, text="Relay State:").grid(row=1, column=1, sticky="e")
        ttk.Label(status_frame, textvariable=self.relay_text).grid(row=1, column=2, sticky="w")

        # Relay voltage
        self.relayv_canvas = tk.Canvas(status_frame, width=25, height=25,
                                       highlightthickness=1, highlightbackground="gray")
        self.relayv_canvas.grid(row=2, column=0, padx=10, pady=5)
        self.relayv_indicator = self.relayv_canvas.create_oval(5, 5, 20, 20, fill="gray")
        ttk.Label(status_frame, text="Relay Supply:").grid(row=2, column=1, sticky="e")
        ttk.Label(status_frame, textvariable=self.relay_v_text).grid(row=2, column=2, sticky="w")

        # ---- Numeric readout ----
        ttk.Label(master, text="Live Voltage:").grid(row=5, column=0, sticky="e", **pad)
        self.vdc_label = ttk.Label(master, textvariable=self.latest_vdc,
                                   font=("Consolas", 36, "bold"), foreground="#0078D4")
        self.vdc_label.grid(row=5, column=1, columnspan=3, sticky="w", **pad)

        # ---- Matplotlib plot ----
        self.fig, self.ax = plt.subplots(figsize=(8, 4))
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Voltage (VDC)")
        self.line, = self.ax.plot([], [], "r-", linewidth=1.5)
        self.ax.grid(True)
        self.canvas = FigureCanvasTkAgg(self.fig, master)
        self.canvas.get_tk_widget().grid(row=6, column=0, columnspan=4, padx=10, pady=10)

        # ---- Serial monitor ----
        ttk.Label(master, text="Serial Monitor:").grid(row=7, column=0, sticky="nw", **pad)
        self.serial_text = scrolledtext.ScrolledText(master, width=100, height=10,
                                                     font=("Consolas", 10))
        self.serial_text.grid(row=7, column=0, columnspan=4, padx=10, pady=10)

        # Periodic tasks
        self.master.after(200, self.update_plot_periodic)
        self.master.after(500, self.connection_watchdog)

    # ---------- Serial Handling ----------
    def refresh_ports(self):
        self.port_list = [p.device for p in serial.tools.list_ports.comports()]
        if self.port_list:
            self.port_var.set(self.port_list[0])

    def toggle_connection(self):
        if self.serial_conn:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        try:
            port = self.port_var.get()
            self.serial_conn = serial.Serial(port, 9600, timeout=1)
            self.connect_btn.config(text="Disconnect")
            self.start_btn.config(state="normal")
            self.running = True
            threading.Thread(target=self.read_serial, daemon=True).start()
            self.last_line_time = time.time()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect: {e}")
            self.serial_conn = None

    def disconnect_serial(self):
        self.running = False
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None
        self.connect_btn.config(text="Connect")
        self.start_btn.config(state="disabled")

    def read_serial(self):
        while self.running:
            try:
                if not self.serial_conn:
                    break
                line = self.serial_conn.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                self.last_line_time = time.time()
                self.master.after(0, self.append_serial_text, line)
                self.master.after(0, self.parse_status, line)
                if "VDC" in line and "T(s)" in line:
                    try:
                        vdc = float(line.split("VDC:")[1].split()[0])
                        t = float(line.split("T(s):")[1])
                        self.data.append((t, vdc))
                        self.master.after(0, self.update_readout, vdc)
                    except Exception:
                        continue
            except Exception:
                break
        self.master.after(0, self.handle_disconnect)

    # ---------- Parsing & Indicators ----------
    def append_serial_text(self, line):
        self.serial_text.insert(tk.END, line + "\n")
        self.serial_text.see(tk.END)

    def parse_status(self, line):
        # System status
        if "Good" in line:
            self.set_status("Good", "green")
        elif "Force Stop" in line:
            self.set_status("Force Stop", "yellow")
        elif "ALARM" in line:
            self.set_status("ALARM", "red")

        # Relay state
        if "Relay: Open" in line:
            self.relay_text.set("Open")
            self.relay_canvas.itemconfig(self.relay_indicator, fill="red")
        elif "Relay: Closed" in line:
            self.relay_text.set("Closed")
            self.relay_canvas.itemconfig(self.relay_indicator, fill="green")

        # Relay voltage monitor
        if "Relay V:" in line:
            try:
                val = float(line.split("Relay V:")[1].split()[0])
                self.relay_v_text.set(f"{val:.2f} V")
                if 22.0 <= val <= 25.0:
                    self.relayv_canvas.itemconfig(self.relayv_indicator, fill="green")
                    if self.relay_v_fault:
                        self.relay_v_fault = False
                        self.set_status("Good", "green")
                else:
                    self.relayv_canvas.itemconfig(self.relayv_indicator, fill="red")
                    if not self.relay_v_fault:
                        self.relay_v_fault = True
                        self.set_status("Relay V Fault", "red")
                        self.force_stop()  # send "S" automatically
            except Exception:
                pass

    def set_status(self, text, color):
        self.status_text.set(text)
        self.status_canvas.itemconfig(self.status_indicator, fill=color)
        self.last_status_color = color

    def update_readout(self, vdc):
        if self.live_updates:
            self.latest_vdc.set(f"{vdc:.4f} V")

    # ---------- Watchdog ----------
    def connection_watchdog(self):
        now = time.time()
        if self.serial_conn and self.running:
            if now - self.last_line_time > self.timeout_seconds:
                self.handle_disconnect()
        self.master.after(500, self.connection_watchdog)

    def handle_disconnect(self):
        self.status_text.set("Disconnected")
        self.status_canvas.itemconfig(self.status_indicator, fill="gray")
        self.relay_text.set("Unknown")
        self.relay_canvas.itemconfig(self.relay_indicator, fill="gray")
        self.relay_v_text.set("--- V")
        self.relayv_canvas.itemconfig(self.relayv_indicator, fill="gray")
        self.master.bell()

    # ---------- Plotting ----------
    def get_rolloff_seconds(self):
        v = self.rolloff_value.get()
        u = self.rolloff_unit.get()
        if u == "milliseconds": return v / 1000.0
        if u == "seconds": return v
        if u == "minutes": return v * 60
        if u == "hours": return v * 3600
        if u == "points": return int(v)
        return 30

    def update_plot_periodic(self):
        if self.live_updates and self.data:
            rolloff = self.get_rolloff_seconds()
            times, volts = zip(*self.data)
            subset = self.data[-rolloff:] if isinstance(rolloff, int) \
                     else [(t, v) for t, v in self.data if t >= times[-1] - rolloff]
            if subset:
                t_sub, v_sub = zip(*subset)
                self.line.set_data(t_sub, v_sub)
                self.ax.relim()
                self.ax.autoscale_view()
                self.canvas.draw_idle()
        self.master.after(200, self.update_plot_periodic)

    # ---------- Logging ----------
    def start_logging(self):
        self.logging = True
        self.data = []
        self.start_btn.config(state="disabled")
        self.csv_file = filedialog.asksaveasfilename(defaultextension=".csv",
                                                     filetypes=[("CSV Files", "*.csv")])
        if not self.csv_file:
            self.logging = False
            return
        threading.Thread(target=self.write_csv_loop, daemon=True).start()

    def write_csv_loop(self):
        with open(self.csv_file, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["Time(s)", "Voltage(VDC)"])
            last = 0
            while self.logging:
                if len(self.data) > last:
                    for t, v in self.data[last:]:
                        w.writerow([t, v])
                    last = len(self.data)
                    f.flush()
                time.sleep(0.5)

    # ---------- Commands ----------
    def force_stop(self):
        if self.serial_conn:
            try:
                self.serial_conn.write(b"S\n")
            except Exception:
                pass
            self.append_serial_text("> S (Force Stop)")

    def go_command(self):
        if self.serial_conn:
            try:
                self.serial_conn.write(b"G\n")
            except Exception:
                pass
            self.append_serial_text("> G (Go)")

    def toggle_live_updates(self):
        self.live_updates = not self.live_updates
        self.live_btn.config(text=f"Live Updates: {'ON' if self.live_updates else 'OFF'}")
        self.vdc_label.config(foreground="#0078D4" if self.live_updates else "gray")

    def send_command(self):
        cmd = self.cmd_entry.get().strip()
        if self.serial_conn and cmd:
            try:
                self.serial_conn.write((cmd + "\n").encode())
            except Exception:
                pass
            self.append_serial_text(f"> {cmd}")


if __name__ == "__main__":
    root = tk.Tk()
    app = SerialLoggerApp(root)
    root.mainloop()
