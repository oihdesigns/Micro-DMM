#!/usr/bin/env python3
"""Minimal WireHawk GUI. Requires: pip install pyserial"""

import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial
import serial.tools.list_ports

BAUD = 115200
HEARTBEAT_MS = 500
STALE_SECONDS = 2.0


class SerialWorker:
    def __init__(self, output_queue: queue.Queue):
        self.output_queue = output_queue
        self.ser = None
        self.stop_event = threading.Event()
        self.thread = None
        self.last_rx = 0.0

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def open(self, port: str) -> None:
        self.close()
        self.ser = serial.Serial(port, BAUD, timeout=0.2, write_timeout=0.5)
        self.ser.reset_input_buffer()
        self.last_rx = time.monotonic()
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def close(self) -> None:
        self.stop_event.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=0.5)
        self.thread = None
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None

    def write(self, command: str) -> None:
        if not self.connected:
            raise serial.SerialException("Serial port is not connected")
        self.ser.write((command.rstrip() + "\n").encode("ascii"))

    def _reader(self) -> None:
        try:
            while not self.stop_event.is_set() and self.connected:
                raw = self.ser.readline()
                if not raw:
                    continue
                self.last_rx = time.monotonic()
                line = raw.decode("ascii", errors="replace").strip()
                if line:
                    self.output_queue.put(("line", line))
        except (serial.SerialException, OSError) as exc:
            if not self.stop_event.is_set():
                self.output_queue.put(("error", str(exc)))


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("WireHawk Minimal")
        self.geometry("620x430")
        self.queue = queue.Queue()
        self.serial = SerialWorker(self.queue)
        self.relay_on = False

        self._build()
        self.refresh_ports()
        self.after(50, self.pump_queue)
        self.after(HEARTBEAT_MS, self.heartbeat)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.bind("<Escape>", lambda _event: self.send("OFF"))

    def _build(self) -> None:
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")
        ttk.Label(top, text="Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=22, state="readonly")
        self.port_box.pack(side="left", padx=5)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side="left")
        self.connect_button = ttk.Button(top, text="Connect", command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=5)
        self.link_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.link_var).pack(side="left", padx=8)

        self.state_var = tk.StringVar(value="IDLE")
        tk.Label(self, textvariable=self.state_var, font=("Segoe UI", 28, "bold"),
                 bg="#303030", fg="white", pady=12).pack(fill="x", padx=10)

        readings = ttk.Frame(self, padding=14)
        readings.pack(fill="x")
        self.current_var = tk.StringVar(value="--.- mA")
        self.mv_var = tk.StringVar(value="A0: --.- mV")
        self.bits_var = tk.StringVar(value="D4 D3: - -")
        ttk.Label(readings, textvariable=self.current_var,
                  font=("Consolas", 30, "bold")).pack()
        ttk.Label(readings, textvariable=self.mv_var, font=("Consolas", 12)).pack()
        ttk.Label(readings, textvariable=self.bits_var, font=("Consolas", 12)).pack(pady=4)

        controls = ttk.Frame(self, padding=10)
        controls.pack(fill="x")
        self.on_button = tk.Button(controls, text="RELAY ON", command=lambda: self.send("ON"),
                                   bg="#267a35", fg="white", font=("Segoe UI", 12, "bold"))
        self.on_button.pack(side="left", expand=True, fill="x", padx=4)
        tk.Button(controls, text="RELAY OFF (Esc)", command=lambda: self.send("OFF"),
                  bg="#9a2020", fg="white", font=("Segoe UI", 12, "bold")).pack(
                      side="left", expand=True, fill="x", padx=4)

        self.raw_var = tk.StringVar(value="No data yet")
        ttk.Label(self, textvariable=self.raw_var, padding=10, wraplength=580).pack(fill="x")

        self.log = tk.Text(self, height=7, font=("Consolas", 9), state="disabled")
        self.log.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    def refresh_ports(self) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connection(self) -> None:
        if self.serial.connected:
            self.disconnect("Disconnected")
            return
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("WireHawk", "Select a serial port.")
            return
        try:
            self.serial.open(port)
        except (serial.SerialException, OSError) as exc:
            messagebox.showerror("WireHawk", f"Could not open {port}:\n{exc}")
            return
        self.connect_button.config(text="Disconnect")
        self.link_var.set("Connected; waiting for data")
        self.append_log(f"Connected to {port}")
        self.send("STATUS")

    def disconnect(self, reason: str) -> None:
        if self.serial.connected:
            try:
                self.serial.write("OFF")
            except (serial.SerialException, OSError):
                pass
        self.serial.close()
        self.relay_on = False
        self.connect_button.config(text="Connect")
        self.link_var.set(reason)
        self.state_var.set("LINK DOWN")

    def send(self, command: str) -> None:
        try:
            self.serial.write(command)
            if command == "ON":
                self.relay_on = True
            elif command in ("OFF", "STOP"):
                self.relay_on = False
        except (serial.SerialException, OSError) as exc:
            self.disconnect(f"Serial error: {exc}")

    def pump_queue(self) -> None:
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "error":
                    self.append_log(f"Serial error: {payload}")
                    self.disconnect("Connection lost")
                else:
                    self.handle_line(payload)
        except queue.Empty:
            pass
        self.after(50, self.pump_queue)

    def handle_line(self, line: str) -> None:
        self.raw_var.set(line)
        parts = line.split(",")
        if parts[0] != "DATA":
            self.append_log(line)
            return
        if len(parts) != 8:
            self.append_log(f"Rejected malformed DATA line ({len(parts)} fields): {line}")
            return
        try:
            _ms = int(parts[1])
            relay = bool(int(parts[2]))
            current_ma = float(parts[3])
            mv = float(parts[4])
            d4 = int(parts[5])
            d3 = int(parts[6])
            state = parts[7]
        except ValueError:
            self.append_log(f"Rejected nonnumeric DATA line: {line}")
            return

        self.relay_on = relay
        self.current_var.set(f"{current_ma:.1f} mA")
        self.mv_var.set(f"A0: {mv:.1f} mV")
        self.bits_var.set(f"D4 D3: {d4} {d3}")
        self.state_var.set(state)
        self.link_var.set("Connected")

    def heartbeat(self) -> None:
        if self.serial.connected:
            if self.relay_on:
                self.send("PING")
            if time.monotonic() - self.serial.last_rx > STALE_SECONDS:
                self.append_log("No incoming data for 2 seconds")
                self.disconnect("Data stream stopped")
        self.after(HEARTBEAT_MS, self.heartbeat)

    def append_log(self, text: str) -> None:
        self.log.config(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def on_close(self) -> None:
        self.disconnect("Closed")
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
