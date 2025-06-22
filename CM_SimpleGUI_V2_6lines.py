import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import os
from datetime import datetime

class SerialMonitorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Simple DMM GUI")
        self.root.geometry("450x600")

        self.serial_port = None
        self.stop_thread = False

        self.create_widgets()

    def create_widgets(self):
        # Serial Port Selection
        self.port_label = tk.Label(self.root, text="Serial Port:")
        self.port_label.pack()
        
        self.port_combobox = ttk.Combobox(self.root, state="readonly")
        self.port_combobox.pack()
        self.refresh_ports()
        
        self.refresh_button = tk.Button(self.root, text="Refresh Ports", command=self.refresh_ports)
        self.refresh_button.pack()

        # Baud Rate Selection
        self.baud_label = tk.Label(self.root, text="Baud Rate:")
        self.baud_label.pack()
        
        self.baud_combobox = ttk.Combobox(self.root, state="readonly")
        self.baud_combobox.pack()
        self.baud_combobox['values'] = [115200, 9600, 19200, 38400, 57600]
        self.baud_combobox.current(0)

        # Connect Button
        self.connect_button = tk.Button(self.root, text="Connect", command=self.connect_serial)
        self.connect_button.pack()

        # Command Buttons
        self.command_frame = tk.Frame(self.root)
        self.command_frame.pack()

        commands = [
            {"label": "Start/Stop", "command": "q"},
            {"label": "Manual Update", "command": "Q"},
            {"label": "MinMax Toggle", "command": "e"},
            {"label": "MinMax ZERO", "command": "E"},
            
        ]

        for cmd in commands:
            button = tk.Button(self.command_frame, text=cmd["label"], command=lambda c=cmd["command"]: self.send_command(c))
            button.pack(side=tk.LEFT, padx=5)

        # Custom Command Entry
        self.custom_command_label = tk.Label(self.root, text="Custom Commands:")
        self.custom_command_label.pack()

        self.custom_command_entry = tk.Entry(self.root)
        self.custom_command_entry.pack()
        self.custom_command_entry.bind("<Return>", lambda event: self.send_custom_command())

        self.send_custom_button = tk.Button(self.root, text="Send", command=self.send_custom_command)
        self.send_custom_button.pack()

        # Data Display
        self.data_label = tk.Label(self.root, text="Incoming Data:", font=("Arial", 14))
        self.data_label.pack()

        self.data_display_1 = tk.Label(self.root, text="--", font=("Arial", 24), wraplength=1000, justify="left")
        self.data_display_1.pack()
        self.data_display_2 = tk.Label(self.root, text="--", font=("Arial", 14), wraplength=1000, justify="left")
        self.data_display_2.pack()
        self.data_display_3 = tk.Label(self.root, text="--", font=("Arial", 14), wraplength=1000, justify="left")
        self.data_display_3.pack()
        self.data_display_4 = tk.Label(self.root, text="--", font=("Arial", 24), wraplength=1000, justify="left")
        self.data_display_4.pack()
        self.data_display_5 = tk.Label(self.root, text="--", font=("Arial", 14), wraplength=1000, justify="left")
        self.data_display_5.pack()
        self.data_display_6 = tk.Label(self.root, text="--", font=("Arial", 14), wraplength=1000, justify="left")
        self.data_display_6.pack()

        # Save Data Button
        self.save_button = tk.Button(self.root, text="Save Data", command=self.save_data)
        self.save_button.pack()

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.port_combobox['values'] = [port.device for port in ports]
        if ports:
            self.port_combobox.current(0)

    def connect_serial(self):
        if self.serial_port:
            self.disconnect_serial()
        else:
            try:
                port = self.port_combobox.get()
                baud_rate = int(self.baud_combobox.get())
                
                self.serial_port = serial.Serial(port, baud_rate, timeout=1)
                self.connect_button.config(text="Disconnect")

                self.stop_thread = False
                self.thread = threading.Thread(target=self.read_serial)
                self.thread.start()

            except Exception as e:
                messagebox.showerror("Error", f"Failed to connect: {e}")

    def disconnect_serial(self):
        self.stop_thread = True
        if self.thread.is_alive():
            self.thread.join()

        if self.serial_port:
            self.serial_port.close()
            self.serial_port = None
            self.connect_button.config(text="Connect")

    def send_command(self, command):
        if self.serial_port:
            try:
                self.serial_port.write(f"{command}\n".encode())
            except Exception as e:
                messagebox.showerror("Error", f"Failed to send command: {e}")
        else:
            messagebox.showwarning("Warning", "Not connected to any serial port!")

    def send_custom_command(self):
        command = self.custom_command_entry.get()
        if command:
            self.send_command(command)

    def read_serial(self):
        while not self.stop_thread:
            if self.serial_port and self.serial_port.in_waiting:
                try:
                    line = self.serial_port.readline().decode().strip()
                    if line:
                        if len(line) > 1000:
                            line = '\n'.join([line[i:i+1000] for i in range(0, len(line), 1000)])
                        parts = line.split('BREAK')
                        self.data_display_1.config(text=f"{parts[0] if len(parts) > 0 else '--'}")
                        self.data_display_2.config(text=f"{parts[1] if len(parts) > 1 else '--'}")
                        self.data_display_3.config(text=f"{parts[2] if len(parts) > 2 else '--'}")
                        self.data_display_4.config(text=f"{parts[3] if len(parts) > 3 else '--'}")
                        self.data_display_5.config(text=f"{parts[4] if len(parts) > 4 else '--'}")
                        self.data_display_6.config(text=f"{parts[5] if len(parts) > 5 else '--'}")
                except Exception as e:
                    self.data_display_1.config(text=f"Error: {e}")
            time.sleep(0.1)

    def save_data(self):
        try:
            data = [
                self.data_display_1.cget("text"),
                self.data_display_2.cget("text"),
                self.data_display_3.cget("text"),
                self.data_display_4.cget("text"),
                self.data_display_5.cget("text"),
                self.data_display_6.cget("text")
            ]
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"SerialData_{timestamp}.txt"
            with open(filename, "w") as file:
                file.write("\n".join(data))
            messagebox.showinfo("Success", f"Data saved to {filename}")
            os.startfile(filename)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save data: {e}")

    def on_close(self):
        self.disconnect_serial()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = SerialMonitorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
