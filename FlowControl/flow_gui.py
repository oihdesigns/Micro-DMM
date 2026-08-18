#!/usr/bin/env python3
"""flow_gui.py — control panel for a rig of Alicat flow controllers.

Proof of concept, but built as the top layer of a stack rather than as one
program: every serial detail lives in the flowlab package, and this file only
knows about Device, Snapshot and DeviceManager. Point it at a rig JSON file
(or run it with no arguments for two simulated controllers).

    python flow_gui.py                 # simulated rig, no hardware needed
    python flow_gui.py bench_rig.json  # real instruments

Tabs
    Live       one card per device: readings, setpoint, valve, tare
    Devices    buses, bus scan, add/remove devices, save/load the rig
    Sequence   ramp/hold setpoint programs across any set of devices
    Terminal   hand-typed protocol traffic, per device
    Log        CSV logging, bus statistics, event history

Requires: pyserial, matplotlib
    pip install pyserial matplotlib
"""

from __future__ import annotations

import json
import os
import sys
import time
import tkinter as tk
from concurrent.futures import Future
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from flowlab import core, discover
from flowlab.bus import BusConfig, list_ports
from flowlab.config import (DEFAULT_RIG, build_manager, load_rig,
                            rig_from_manager, save_rig)
from flowlab.manager import (EVENT_ERROR, EVENT_SNAPSHOT, EVENT_STATUS,
                             DeviceManager)
from flowlab.sequence import Sequence, SequenceRunner, Step

UI_TICK_MS = 200          # how often the GUI drains events and repaints
PLOT_EVERY = 3            # repaint the chart every Nth tick
PLOT_KEYS = [core.MASS, core.VOLUMETRIC, core.PRESSURE, core.TEMPERATURE,
             core.SETPOINT, core.TOTALIZER]
WINDOWS = {"1 min": 60, "5 min": 300, "15 min": 900, "1 hour": 3600}

OK_COLOR = "#1a7f37"
BAD_COLOR = "#b42318"
IDLE_COLOR = "#8a8a8a"
TRACE_COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e",
                "#17becf", "#8c564b", "#e377c2"]


def fmt(value: float | None, digits: int = 2) -> str:
    if value is None:
        return "--"
    magnitude = abs(value)
    if magnitude >= 1000:
        return f"{value:,.0f}"
    if magnitude >= 100:
        return f"{value:.1f}"
    return f"{value:.{digits}f}"


# ──────────────────────────────────────────────────────────────────────────
# Live tab
# ──────────────────────────────────────────────────────────────────────────
class DeviceCard(ttk.LabelFrame):
    """One controller: what it reads, and every control you reach for."""

    SECONDARY = [(core.PRESSURE, "P"), (core.TEMPERATURE, "T"),
                 (core.VOLUMETRIC, "Vol"), (core.TOTALIZER, "Total")]

    def __init__(self, parent, app: "App", name: str):
        super().__init__(parent, text=name, padding=8)
        self.app = app
        self.name = name
        info = app.manager.entries[name].device.info
        self.full_scale = info.full_scale or 100.0
        self.units = info.units or ""

        # header ------------------------------------------------------------
        head = ttk.Frame(self)
        head.pack(fill="x")
        self.dot = tk.Canvas(head, width=12, height=12, highlightthickness=0)
        self.dot_id = self.dot.create_oval(2, 2, 11, 11, fill=IDLE_COLOR, outline="")
        self.dot.pack(side="left", padx=(0, 6))
        subtitle = " · ".join(x for x in (info.model, f"addr {info.address}") if x)
        ttk.Label(head, text=subtitle, foreground="#555").pack(side="left")
        self.flag_var = tk.StringVar(value="")
        ttk.Label(head, textvariable=self.flag_var, foreground=BAD_COLOR).pack(side="right")

        # primary reading -----------------------------------------------------
        big = ttk.Frame(self)
        big.pack(fill="x", pady=(6, 2))
        self.reading_var = tk.StringVar(value="--")
        tk.Label(big, textvariable=self.reading_var, font=("Consolas", 30, "bold"),
                 anchor="e").pack(side="left")
        ttk.Label(big, text=f" {self.units}", font=("Segoe UI", 11)).pack(
            side="left", anchor="s", pady=(0, 8))
        self.readback_var = tk.StringVar(value="setpoint --")
        ttk.Label(big, textvariable=self.readback_var, foreground="#555").pack(
            side="right", anchor="s", pady=(0, 8))

        # secondary readings --------------------------------------------------
        row = ttk.Frame(self)
        row.pack(fill="x")
        self.secondary_vars = {}
        for key, label in self.SECONDARY:
            cell = ttk.Frame(row)
            cell.pack(side="left", expand=True, fill="x")
            ttk.Label(cell, text=label, foreground="#777",
                      font=("Segoe UI", 8)).pack(anchor="w")
            var = tk.StringVar(value="--")
            ttk.Label(cell, textvariable=var, font=("Consolas", 10)).pack(anchor="w")
            self.secondary_vars[key] = var

        ttk.Separator(self).pack(fill="x", pady=6)

        # setpoint ------------------------------------------------------------
        sp = ttk.Frame(self)
        sp.pack(fill="x")
        ttk.Label(sp, text="Setpoint").pack(side="left")
        self.sp_entry = ttk.Entry(sp, width=9, justify="right")
        self.sp_entry.pack(side="left", padx=4)
        self.sp_entry.bind("<Return>", lambda _e: self.apply_entry())
        ttk.Button(sp, text="Set", width=5, command=self.apply_entry).pack(side="left")
        for pct in (0, 25, 50, 100):
            ttk.Button(sp, text=f"{pct}%", width=4,
                       command=lambda p=pct: self.apply(self.full_scale * p / 100.0)
                       ).pack(side="left", padx=1)

        self.slider_var = tk.DoubleVar(value=0.0)
        slider = ttk.Scale(self, from_=0.0, to=self.full_scale,
                           variable=self.slider_var, orient="horizontal")
        slider.pack(fill="x", pady=(6, 0))
        # Only commit on release: dragging must not flood a 19200-baud bus.
        slider.bind("<ButtonRelease-1>", lambda _e: self.apply(self.slider_var.get()))

        # actions -------------------------------------------------------------
        actions = ttk.Frame(self)
        actions.pack(fill="x", pady=(8, 0))
        ttk.Button(actions, text="Tare flow", width=9,
                   command=lambda: self.app.manager.tare_flow(self.name)).pack(side="left")
        ttk.Button(actions, text="Tare P", width=7,
                   command=lambda: self.app.manager.tare_pressure(self.name)).pack(
                       side="left", padx=2)
        self.hold_var = tk.StringVar(value="cancel")
        hold = ttk.Combobox(actions, textvariable=self.hold_var, width=8, state="readonly",
                            values=["cancel", "closed", "current", "exhaust"])
        hold.pack(side="left", padx=(6, 2))
        hold.bind("<<ComboboxSelected>>",
                  lambda _e: self.app.manager.hold(self.name, self.hold_var.get()))
        ttk.Button(actions, text="Zero total", width=10,
                   command=lambda: self.app.manager.reset_totalizer(self.name)).pack(
                       side="right")

    # -- actions ------------------------------------------------------------
    def apply(self, value: float) -> None:
        value = max(0.0, min(float(value), self.full_scale))
        self.slider_var.set(value)
        self.sp_entry.delete(0, "end")
        self.sp_entry.insert(0, f"{value:g}")
        self.app.manager.set_setpoint(self.name, value)

    def apply_entry(self) -> None:
        try:
            self.apply(float(self.sp_entry.get()))
        except ValueError:
            messagebox.showerror("Setpoint", f"{self.sp_entry.get()!r} is not a number")

    # -- display ------------------------------------------------------------
    def update_from(self, snap) -> None:
        if snap is None:
            return
        if not snap.ok:
            self.dot.itemconfig(self.dot_id, fill=BAD_COLOR)
            self.flag_var.set(snap.error.split(":")[0])
            self.reading_var.set("--")
            return
        self.dot.itemconfig(self.dot_id, fill=OK_COLOR)
        self.flag_var.set(" ".join(snap.flags))

        primary = snap.values.get(core.MASS, snap.values.get(core.VOLUMETRIC))
        self.reading_var.set(fmt(primary))
        setpoint = snap.values.get(core.SETPOINT)
        gas = f" · {snap.gas}" if snap.gas else ""
        self.readback_var.set(
            f"setpoint {fmt(setpoint)}{gas}" if setpoint is not None else f"—{gas}")
        for key, var in self.secondary_vars.items():
            var.set(fmt(snap.values.get(key)) if key in snap.values else "--")


class LiveTab(ttk.Frame):
    def __init__(self, parent, app: "App"):
        super().__init__(parent, padding=6)
        self.app = app
        self.cards: dict[str, DeviceCard] = {}
        self.lines: dict[tuple[str, str], object] = {}
        self._tick = 0

        panes = ttk.PanedWindow(self, orient="horizontal")
        panes.pack(fill="both", expand=True)

        # left: scrollable card column
        left = ttk.Frame(panes)
        self.canvas = tk.Canvas(left, width=430, highlightthickness=0)
        scroll = ttk.Scrollbar(left, orient="vertical", command=self.canvas.yview)
        self.card_host = ttk.Frame(self.canvas)
        self.card_host.bind("<Configure>", lambda _e: self.canvas.configure(
            scrollregion=self.canvas.bbox("all")))
        self.canvas.create_window((0, 0), window=self.card_host, anchor="nw")
        self.canvas.configure(yscrollcommand=scroll.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        panes.add(left, weight=0)

        # right: chart
        right = ttk.Frame(panes)
        controls = ttk.Frame(right)
        controls.pack(fill="x")
        ttk.Label(controls, text="Plot").pack(side="left")
        self.plot_key = tk.StringVar(value=core.MASS)
        key_box = ttk.Combobox(controls, textvariable=self.plot_key, width=12,
                               state="readonly", values=PLOT_KEYS)
        key_box.pack(side="left", padx=4)
        key_box.bind("<<ComboboxSelected>>", lambda _e: self.rebuild_lines())
        ttk.Label(controls, text="window").pack(side="left", padx=(10, 2))
        self.window_var = tk.StringVar(value="5 min")
        ttk.Combobox(controls, textvariable=self.window_var, width=7, state="readonly",
                     values=list(WINDOWS)).pack(side="left")
        self.show_sp = tk.BooleanVar(value=True)
        ttk.Checkbutton(controls, text="show setpoint", variable=self.show_sp,
                        command=self.rebuild_lines).pack(side="left", padx=10)

        self.figure = Figure(figsize=(6.5, 4.4), dpi=100)
        self.axes = self.figure.add_subplot(111)
        self.axes.grid(alpha=0.3)
        self.figure.tight_layout()
        self.canvas_plot = FigureCanvasTkAgg(self.figure, master=right)
        self.canvas_plot.get_tk_widget().pack(fill="both", expand=True)
        panes.add(right, weight=1)

    # -- structure ----------------------------------------------------------
    def rebuild(self) -> None:
        for card in self.cards.values():
            card.destroy()
        self.cards.clear()
        for name in self.app.manager.names():
            card = DeviceCard(self.card_host, self.app, name)
            card.pack(fill="x", pady=4, padx=4)
            self.cards[name] = card
        self.rebuild_lines()

    def rebuild_lines(self) -> None:
        self.axes.clear()
        self.axes.grid(alpha=0.3)
        self.lines.clear()
        key = self.plot_key.get()
        for index, name in enumerate(self.app.manager.names()):
            color = TRACE_COLORS[index % len(TRACE_COLORS)]
            self.lines[(name, key)], = self.axes.plot([], [], color=color, label=name)
            if self.show_sp.get() and key in (core.MASS, core.VOLUMETRIC):
                self.lines[(name, core.SETPOINT)], = self.axes.plot(
                    [], [], color=color, linestyle="--", alpha=0.55, linewidth=1.0)
        self.axes.set_ylabel(core.READING_LABELS.get(key, key))
        self.axes.set_xlabel("seconds ago")
        if self.lines:
            self.axes.legend(loc="upper left", fontsize=8)
        self.canvas_plot.draw_idle()

    # -- refresh ------------------------------------------------------------
    def refresh(self) -> None:
        for name, card in self.cards.items():
            card.update_from(self.app.manager.latest(name))
        self._tick += 1
        if self._tick % PLOT_EVERY:
            return
        span = WINDOWS[self.window_var.get()]
        now = time.time()
        low, high = None, None
        for (name, key), line in self.lines.items():
            times, values = self.app.manager.history(name, key)
            xs = [t - now for t, v in zip(times, values) if t >= now - span]
            ys = [v for t, v in zip(times, values) if t >= now - span]
            line.set_data(xs, ys)
            if ys:
                low = min(ys) if low is None else min(low, min(ys))
                high = max(ys) if high is None else max(high, max(ys))
        self.axes.set_xlim(-span, 0)
        if low is not None:
            pad = max((high - low) * 0.1, abs(high) * 0.02, 0.1)
            self.axes.set_ylim(low - pad, high + pad)
        self.canvas_plot.draw_idle()


# ──────────────────────────────────────────────────────────────────────────
# Devices tab
# ──────────────────────────────────────────────────────────────────────────
class DevicesTab(ttk.Frame):
    def __init__(self, parent, app: "App"):
        super().__init__(parent, padding=8)
        self.app = app

        # buses ---------------------------------------------------------------
        bus_frame = ttk.LabelFrame(self, text="Buses", padding=8)
        bus_frame.pack(fill="x")
        self.bus_tree = ttk.Treeview(bus_frame, columns=("protocol", "port", "baud"),
                                     show="headings", height=3)
        for column, width in (("protocol", 90), ("port", 110), ("baud", 80)):
            self.bus_tree.heading(column, text=column)
            self.bus_tree.column(column, width=width, anchor="w")
        self.bus_tree.pack(side="left", fill="x", expand=True)

        add = ttk.Frame(bus_frame)
        add.pack(side="right", padx=(10, 0))
        self.new_bus_id = tk.StringVar(value="bench")
        self.new_port = tk.StringVar()
        self.new_baud = tk.StringVar(value="19200")
        self.new_proto = tk.StringVar(value="modbus")
        grid = [("id", ttk.Entry(add, textvariable=self.new_bus_id, width=10)),
                ("port", ttk.Combobox(add, textvariable=self.new_port, width=10,
                                      values=list_ports())),
                ("baud", ttk.Combobox(add, textvariable=self.new_baud, width=10,
                                      values=["9600", "19200", "38400", "57600", "115200"])),
                ("protocol", ttk.Combobox(add, textvariable=self.new_proto, width=10,
                                          state="readonly",
                                          values=["modbus", "ascii", "sim"]))]
        for row, (label, widget) in enumerate(grid):
            ttk.Label(add, text=label).grid(row=row, column=0, sticky="e", padx=2)
            widget.grid(row=row, column=1, sticky="w", pady=1)
        ttk.Button(add, text="Add bus", command=self.add_bus).grid(
            row=len(grid), column=0, columnspan=2, sticky="ew", pady=(4, 0))
        ttk.Button(add, text="Scan bus", command=self.scan_bus).grid(
            row=len(grid) + 1, column=0, columnspan=2, sticky="ew")

        # devices -------------------------------------------------------------
        device_frame = ttk.LabelFrame(self, text="Devices", padding=8)
        device_frame.pack(fill="both", expand=True, pady=(8, 0))
        columns = ("bus", "address", "model", "units", "full_scale", "poll_hz",
                   "firmware", "serial")
        self.device_tree = ttk.Treeview(device_frame, columns=columns, show="tree headings")
        self.device_tree.heading("#0", text="name")
        self.device_tree.column("#0", width=130)
        for column in columns:
            self.device_tree.heading(column, text=column)
            self.device_tree.column(column, width=90, anchor="w")
        self.device_tree.pack(side="left", fill="both", expand=True)

        side = ttk.Frame(device_frame)
        side.pack(side="right", fill="y", padx=(8, 0))
        ttk.Button(side, text="Add device…", command=self.add_device_dialog).pack(fill="x")
        ttk.Button(side, text="Remove", command=self.remove_device).pack(fill="x", pady=2)
        ttk.Separator(side).pack(fill="x", pady=6)
        ttk.Button(side, text="Load rig…", command=self.app.load_rig_dialog).pack(fill="x")
        ttk.Button(side, text="Save rig…", command=self.app.save_rig_dialog).pack(fill="x", pady=2)

        self.scan_output = tk.Text(self, height=6, wrap="none", font=("Consolas", 9))
        self.scan_output.pack(fill="x", pady=(8, 0))
        self.scan_output.insert("end", "Bus scan results appear here.\n")

    # -- helpers ------------------------------------------------------------
    def refresh(self) -> None:
        self.bus_tree.delete(*self.bus_tree.get_children())
        for key, worker in self.app.manager.workers.items():
            if worker.bus is None:
                self.bus_tree.insert("", "end", iid=key, values=("sim", "-", "-"))
            else:
                cfg = worker.bus.cfg
                self.bus_tree.insert("", "end", iid=key,
                                     values=(cfg.protocol, cfg.port, cfg.baud))
        self.device_tree.delete(*self.device_tree.get_children())
        for name, entry in self.app.manager.entries.items():
            info = entry.device.info
            self.device_tree.insert(
                "", "end", iid=name, text=name,
                values=(entry.bus_key, info.address, info.model, info.units,
                        f"{info.full_scale:g}", f"{1 / entry.period:.3g}",
                        info.firmware, info.serial))

    def selected_bus(self) -> str | None:
        selection = self.bus_tree.selection()
        return selection[0] if selection else None

    def add_bus(self) -> None:
        cfg = BusConfig(id=self.new_bus_id.get().strip() or "bus",
                        port=self.new_port.get().strip(),
                        baud=int(self.new_baud.get() or 19200),
                        protocol=self.new_proto.get())
        try:
            self.app.manager.add_bus(cfg)
        except ValueError as exc:
            messagebox.showerror("Add bus", str(exc))
            return
        self.app.log_line(f"bus {cfg.id} added ({cfg.protocol} {cfg.port})")
        self.refresh()

    def scan_bus(self) -> None:
        key = self.selected_bus()
        if key is None:
            messagebox.showinfo("Scan", "Select a bus in the table first.")
            return
        worker = self.app.manager.workers[key]
        if worker.bus is None:
            messagebox.showinfo("Scan", "The simulated bus has nothing to scan.")
            return
        protocol = worker.bus.cfg.protocol
        self.scan_output.delete("1.0", "end")
        self.scan_output.insert("end", f"scanning {key} ({protocol})…\n")
        self.scan_output.update_idletasks()

        def job():
            if protocol == "ascii":
                return discover.scan_ascii(worker.bus)
            return discover.scan_modbus(worker.bus)

        # The scan runs on the bus worker so it cannot collide with polling.
        future = self.app.manager.submit_to_bus(key, job, f"scan {key}")
        self.app.watch(future, lambda found: self.show_scan(key, found),
                       lambda exc: self.scan_output.insert("end", f"failed: {exc}\n"))

    def show_scan(self, bus_key: str, found) -> None:
        if not found:
            self.scan_output.insert("end", "nothing answered\n")
            return
        for item in found:
            self.scan_output.insert("end", f"  {item}\n")
        self.scan_output.insert(
            "end", f"{len(found)} device(s) on {bus_key}; use Add device… to keep them\n")

    def add_device_dialog(self) -> None:
        DeviceDialog(self, self.app)

    def remove_device(self) -> None:
        selection = self.device_tree.selection()
        if not selection:
            return
        name = selection[0]
        if messagebox.askyesno("Remove", f"Remove {name} from the rig?"):
            self.app.manager.remove_device(name)
            self.app.rebuild_views()


class DeviceDialog(tk.Toplevel):
    """Add one device to a bus."""

    def __init__(self, parent: DevicesTab, app: "App"):
        super().__init__(parent)
        self.title("Add device")
        self.app = app
        self.resizable(False, False)
        self.transient(parent.winfo_toplevel())

        buses = list(app.manager.workers)
        self.fields = {
            "name": tk.StringVar(value=f"MFC-{len(app.manager.names()) + 1}"),
            "bus": tk.StringVar(value=buses[0] if buses else ""),
            "address": tk.StringVar(value="1"),
            "model": tk.StringVar(value="MC-500SCCM-D"),
            "units": tk.StringVar(value="SCCM"),
            "full_scale": tk.StringVar(value="500"),
            "poll_hz": tk.StringVar(value="2"),
        }
        body = ttk.Frame(self, padding=10)
        body.pack(fill="both", expand=True)
        for row, (label, var) in enumerate(self.fields.items()):
            ttk.Label(body, text=label).grid(row=row, column=0, sticky="e", padx=4, pady=2)
            if label == "bus":
                ttk.Combobox(body, textvariable=var, values=buses, width=18,
                             state="readonly").grid(row=row, column=1, sticky="w")
            else:
                ttk.Entry(body, textvariable=var, width=20).grid(
                    row=row, column=1, sticky="w")
        ttk.Label(body, text="address = Modbus slave number, or ASCII unit ID letter",
                  foreground="#666").grid(row=len(self.fields), column=0, columnspan=2,
                                          pady=(6, 0))
        buttons = ttk.Frame(body)
        buttons.grid(row=len(self.fields) + 1, column=0, columnspan=2, pady=(10, 0))
        ttk.Button(buttons, text="Add", command=self.commit).pack(side="left", padx=4)
        ttk.Button(buttons, text="Cancel", command=self.destroy).pack(side="left")

    def commit(self) -> None:
        entry = {key: var.get().strip() for key, var in self.fields.items()}
        try:
            entry["full_scale"] = float(entry["full_scale"])
            entry["poll_hz"] = float(entry["poll_hz"])
        except ValueError:
            messagebox.showerror("Add device", "full_scale and poll_hz must be numbers")
            return
        if self.app.add_device_from_entry(entry):
            self.destroy()


# ──────────────────────────────────────────────────────────────────────────
# Sequence tab
# ──────────────────────────────────────────────────────────────────────────
class SequenceTab(ttk.Frame):
    COLUMNS = ("device", "setpoint", "ramp_s", "hold_s", "settle_tol")

    def __init__(self, parent, app: "App"):
        super().__init__(parent, padding=8)
        self.app = app
        self.runner: SequenceRunner | None = None

        self.tree = ttk.Treeview(self, columns=self.COLUMNS, show="headings", height=12)
        for column in self.COLUMNS:
            self.tree.heading(column, text=column)
            self.tree.column(column, width=110, anchor="w")
        self.tree.pack(side="left", fill="both", expand=True)

        side = ttk.Frame(self)
        side.pack(side="right", fill="y", padx=(10, 0))

        editor = ttk.LabelFrame(side, text="Step", padding=6)
        editor.pack(fill="x")
        self.inputs = {"device": tk.StringVar(), "setpoint": tk.StringVar(value="100"),
                       "ramp_s": tk.StringVar(value="0"), "hold_s": tk.StringVar(value="30"),
                       "settle_tol": tk.StringVar(value="")}
        for row, key in enumerate(self.COLUMNS):
            ttk.Label(editor, text=key).grid(row=row, column=0, sticky="e", padx=2)
            if key == "device":
                self.device_box = ttk.Combobox(editor, textvariable=self.inputs[key],
                                               width=14, state="readonly")
                self.device_box.grid(row=row, column=1, sticky="w")
            else:
                ttk.Entry(editor, textvariable=self.inputs[key], width=16).grid(
                    row=row, column=1, sticky="w", pady=1)
        ttk.Button(editor, text="Add step", command=self.add_step).grid(
            row=len(self.COLUMNS), column=0, columnspan=2, sticky="ew", pady=(6, 0))

        ttk.Button(side, text="Remove step", command=self.remove_step).pack(fill="x", pady=(8, 2))
        ttk.Button(side, text="Move up", command=lambda: self.move(-1)).pack(fill="x")
        ttk.Button(side, text="Move down", command=lambda: self.move(1)).pack(fill="x", pady=2)
        ttk.Separator(side).pack(fill="x", pady=6)
        self.loop_var = tk.BooleanVar(value=False)
        self.zero_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(side, text="loop", variable=self.loop_var).pack(anchor="w")
        ttk.Checkbutton(side, text="zero setpoints at end",
                        variable=self.zero_var).pack(anchor="w")
        ttk.Separator(side).pack(fill="x", pady=6)
        self.run_button = ttk.Button(side, text="Run", command=self.run)
        self.run_button.pack(fill="x")
        ttk.Button(side, text="Stop", command=self.stop).pack(fill="x", pady=2)
        self.progress = ttk.Progressbar(side, length=160)
        self.progress.pack(fill="x", pady=(8, 2))
        self.status_var = tk.StringVar(value="idle")
        ttk.Label(side, textvariable=self.status_var, foreground="#555").pack(anchor="w")
        ttk.Separator(side).pack(fill="x", pady=6)
        ttk.Button(side, text="Load…", command=self.load).pack(fill="x")
        ttk.Button(side, text="Save…", command=self.save).pack(fill="x", pady=2)

    # -- model --------------------------------------------------------------
    def refresh_devices(self) -> None:
        names = self.app.manager.names()
        self.device_box["values"] = names
        if names and self.inputs["device"].get() not in names:
            self.inputs["device"].set(names[0])

    def steps(self) -> list[Step]:
        steps = []
        for item in self.tree.get_children():
            device, setpoint, ramp, hold, tol = self.tree.item(item, "values")
            steps.append(Step(device=device, setpoint=float(setpoint),
                              ramp_s=float(ramp), hold_s=float(hold),
                              settle_tol=float(tol) if tol not in ("", "-") else None))
        return steps

    def add_step(self) -> None:
        if not self.inputs["device"].get():
            messagebox.showinfo("Step", "No devices in the rig yet.")
            return
        try:
            values = (self.inputs["device"].get(),
                      f"{float(self.inputs['setpoint'].get()):g}",
                      f"{float(self.inputs['ramp_s'].get() or 0):g}",
                      f"{float(self.inputs['hold_s'].get() or 0):g}",
                      self.inputs["settle_tol"].get().strip() or "")
        except ValueError:
            messagebox.showerror("Step", "setpoint, ramp and hold must be numbers")
            return
        self.tree.insert("", "end", values=values)

    def remove_step(self) -> None:
        for item in self.tree.selection():
            self.tree.delete(item)

    def move(self, delta: int) -> None:
        for item in self.tree.selection():
            index = self.tree.index(item)
            self.tree.move(item, "", max(0, index + delta))

    # -- run ----------------------------------------------------------------
    def run(self) -> None:
        if self.runner and self.runner.is_alive():
            return
        steps = self.steps()
        if not steps:
            messagebox.showinfo("Sequence", "Add at least one step.")
            return
        sequence = Sequence(steps=steps, loop=self.loop_var.get(),
                            zero_at_end=self.zero_var.get())
        self.runner = SequenceRunner(self.app.manager, sequence, self.on_progress)
        self.runner.start()
        self.app.log_line(f"sequence started: {len(steps)} steps, "
                          f"{sequence.total_seconds():.0f}s nominal")

    def stop(self) -> None:
        if self.runner:
            self.runner.stop()
            self.app.log_line("sequence stop requested")

    def on_progress(self, index: int, phase: str, fraction: float) -> None:
        # Called from the sequence thread — only store, never touch widgets.
        self._progress = (index, phase, fraction)

    def refresh(self) -> None:
        state = getattr(self, "_progress", None)
        if self.runner and self.runner.is_alive() and state:
            index, phase, fraction = state
            children = self.tree.get_children()
            if 0 <= index < len(children):
                self.tree.selection_set(children[index])
            self.progress["value"] = fraction * 100
            self.status_var.set(f"step {index + 1} · {phase}")
            self.run_button.state(["disabled"])
        else:
            self.run_button.state(["!disabled"])
            if self.runner and not self.runner.is_alive():
                self.progress["value"] = 0
                self.status_var.set(
                    f"error: {self.runner.error}" if self.runner.error
                    else ("finished" if self.runner.finished else "stopped"))
                if self.runner.error:
                    self.app.log_line(f"sequence error: {self.runner.error}", error=True)
                self.runner = None

    # -- files --------------------------------------------------------------
    def save(self) -> None:
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("Sequence", "*.json")])
        if not path:
            return
        payload = {"loop": self.loop_var.get(), "zero_at_end": self.zero_var.get(),
                   "steps": [s.__dict__ for s in self.steps()]}
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        self.app.log_line(f"sequence saved to {path}")

    def load(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("Sequence", "*.json")])
        if not path:
            return
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        self.tree.delete(*self.tree.get_children())
        self.loop_var.set(payload.get("loop", False))
        self.zero_var.set(payload.get("zero_at_end", True))
        for step in payload.get("steps", []):
            tol = step.get("settle_tol")
            self.tree.insert("", "end", values=(
                step["device"], f"{step['setpoint']:g}", f"{step.get('ramp_s', 0):g}",
                f"{step.get('hold_s', 0):g}", "" if tol is None else f"{tol:g}"))
        self.app.log_line(f"sequence loaded from {path}")


# ──────────────────────────────────────────────────────────────────────────
# Terminal tab
# ──────────────────────────────────────────────────────────────────────────
class TerminalTab(ttk.Frame):
    def __init__(self, parent, app: "App"):
        super().__init__(parent, padding=8)
        self.app = app

        bar = ttk.Frame(self)
        bar.pack(fill="x")
        ttk.Label(bar, text="Device").pack(side="left")
        self.device_var = tk.StringVar()
        self.device_box = ttk.Combobox(bar, textvariable=self.device_var, width=16,
                                       state="readonly")
        self.device_box.pack(side="left", padx=4)
        self.device_box.bind("<<ComboboxSelected>>", lambda _e: self.show_help())
        self.entry = ttk.Entry(bar)
        self.entry.pack(side="left", fill="x", expand=True, padx=4)
        self.entry.bind("<Return>", lambda _e: self.send())
        self.entry.bind("<Up>", lambda _e: self.recall(-1))
        self.entry.bind("<Down>", lambda _e: self.recall(1))
        ttk.Button(bar, text="Send", command=self.send).pack(side="left")

        self.output = tk.Text(self, wrap="none", font=("Consolas", 9), height=24)
        self.output.pack(fill="both", expand=True, pady=(8, 0))
        self.output.tag_config("tx", foreground="#0b5cad")
        self.output.tag_config("rx", foreground="#111")
        self.output.tag_config("err", foreground=BAD_COLOR)
        self.history: list[str] = []
        self.history_index = 0

    def refresh_devices(self) -> None:
        names = self.app.manager.names()
        self.device_box["values"] = names
        if names and self.device_var.get() not in names:
            self.device_var.set(names[0])
            self.show_help()

    def show_help(self) -> None:
        name = self.device_var.get()
        if not name or name not in self.app.manager.entries:
            return
        device = self.app.manager.entries[name].device
        doc = (device.raw_exchange.__doc__ or "").strip()
        self.write(f"--- {name}: {type(device).__name__} ---\n", "rx")
        if doc:
            self.write("\n".join(line.strip() for line in doc.splitlines()) + "\n", "rx")

    def write(self, text: str, tag: str = "rx") -> None:
        self.output.insert("end", text, tag)
        self.output.see("end")

    def recall(self, delta: int) -> None:
        if not self.history:
            return
        self.history_index = max(0, min(len(self.history) - 1,
                                        self.history_index + delta))
        self.entry.delete(0, "end")
        self.entry.insert(0, self.history[self.history_index])

    def send(self) -> None:
        name = self.device_var.get()
        text = self.entry.get().strip()
        if not name or not text:
            return
        self.history.append(text)
        self.history_index = len(self.history) - 1
        self.entry.delete(0, "end")
        self.write(f"> {text}\n", "tx")
        future = self.app.manager.raw_exchange(name, text)
        self.app.watch(future,
                       lambda result: self.write(f"{result}\n", "rx"),
                       lambda exc: self.write(f"! {exc}\n", "err"))


# ──────────────────────────────────────────────────────────────────────────
# Log tab
# ──────────────────────────────────────────────────────────────────────────
class LogTab(ttk.Frame):
    def __init__(self, parent, app: "App"):
        super().__init__(parent, padding=8)
        self.app = app

        bar = ttk.LabelFrame(self, text="CSV log", padding=8)
        bar.pack(fill="x")
        self.path_var = tk.StringVar(
            value=os.path.abspath(datetime.now().strftime("flowlog_%Y%m%d_%H%M.csv")))
        ttk.Entry(bar, textvariable=self.path_var).pack(side="left", fill="x",
                                                       expand=True, padx=(0, 6))
        ttk.Button(bar, text="Browse…", command=self.browse).pack(side="left")
        self.toggle = ttk.Button(bar, text="Start logging", command=self.toggle_log)
        self.toggle.pack(side="left", padx=6)
        self.rows_var = tk.StringVar(value="not logging")
        ttk.Label(bar, textvariable=self.rows_var, foreground="#555").pack(side="left")

        stats = ttk.LabelFrame(self, text="Bus statistics", padding=8)
        stats.pack(fill="x", pady=(8, 0))
        self.stats_var = tk.StringVar(value="—")
        ttk.Label(stats, textvariable=self.stats_var, font=("Consolas", 9),
                  justify="left").pack(anchor="w")

        events = ttk.LabelFrame(self, text="Events", padding=8)
        events.pack(fill="both", expand=True, pady=(8, 0))
        self.text = tk.Text(events, wrap="word", font=("Consolas", 9), height=20)
        self.text.pack(fill="both", expand=True)
        self.text.tag_config("err", foreground=BAD_COLOR)

    def browse(self) -> None:
        path = filedialog.asksaveasfilename(defaultextension=".csv",
                                            filetypes=[("CSV", "*.csv")])
        if path:
            self.path_var.set(path)

    def toggle_log(self) -> None:
        manager = self.app.manager
        if manager.log_path:
            manager.stop_log()
            self.toggle.config(text="Start logging")
        else:
            try:
                manager.start_log(self.path_var.get())
            except OSError as exc:
                messagebox.showerror("Log", str(exc))
                return
            self.toggle.config(text="Stop logging")

    def append(self, line: str, error: bool = False) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.text.insert("end", f"{stamp}  {line}\n", "err" if error else "")
        if float(self.text.index("end-1c").split(".")[0]) > 2000:
            self.text.delete("1.0", "500.0")
        self.text.see("end")

    def refresh(self) -> None:
        manager = self.app.manager
        self.rows_var.set(f"{manager.log_rows} rows" if manager.log_path
                          else "not logging")
        lines = []
        for key, worker in manager.workers.items():
            if worker.bus is None:
                lines.append(f"{key:<10} simulated")
            else:
                stats = worker.bus.stats
                cfg = worker.bus.cfg
                lines.append(
                    f"{key:<10} {cfg.port:<7} {cfg.protocol:<7} "
                    f"tx {stats['tx']:<7} rx {stats['rx']:<7} "
                    f"retries {stats['retries']:<5} timeouts {stats['timeouts']:<5} "
                    f"errors {stats['errors']}")
        self.stats_var.set("\n".join(lines) or "—")


# ──────────────────────────────────────────────────────────────────────────
# Application
# ──────────────────────────────────────────────────────────────────────────
class App(tk.Tk):
    def __init__(self, rig_path: str | None = None):
        super().__init__()
        self.title("flowlab — flow controller panel")
        self.geometry("1300x840")
        self.manager = DeviceManager()
        self.rig_path = rig_path
        self._watched: list[tuple[Future, object, object]] = []

        self.build_toolbar()
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=6, pady=(0, 6))
        self.live = LiveTab(self.notebook, self)
        self.devices = DevicesTab(self.notebook, self)
        self.sequence = SequenceTab(self.notebook, self)
        self.terminal = TerminalTab(self.notebook, self)
        self.log = LogTab(self.notebook, self)
        for tab, label in ((self.live, "Live"), (self.devices, "Devices"),
                           (self.sequence, "Sequence"), (self.terminal, "Terminal"),
                           (self.log, "Log")):
            self.notebook.add(tab, text=label)

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.load_rig(rig_path)
        self.after(UI_TICK_MS, self.tick)

    # -- chrome -------------------------------------------------------------
    def build_toolbar(self) -> None:
        bar = ttk.Frame(self, padding=(6, 6, 6, 2))
        bar.pack(fill="x")
        ttk.Button(bar, text="Load rig…", command=self.load_rig_dialog).pack(side="left")
        ttk.Button(bar, text="Save rig…", command=self.save_rig_dialog).pack(
            side="left", padx=4)
        ttk.Button(bar, text="Reload simulated rig",
                   command=lambda: self.load_rig(None)).pack(side="left")
        self.rig_var = tk.StringVar(value="simulated rig")
        ttk.Label(bar, textvariable=self.rig_var, foreground="#555").pack(
            side="left", padx=12)
        stop = tk.Button(bar, text="ALL OFF", command=self.all_off, bg="#b42318",
                         fg="white", font=("Segoe UI", 10, "bold"), relief="raised",
                         padx=14)
        stop.pack(side="right")
        self.status_var = tk.StringVar(value="ready")
        ttk.Label(bar, textvariable=self.status_var).pack(side="right", padx=10)

    # -- rig ----------------------------------------------------------------
    def load_rig(self, path: str | None) -> None:
        self.manager.shutdown()
        self.manager = DeviceManager()
        try:
            rig = load_rig(path) if path else DEFAULT_RIG
        except (OSError, ValueError) as exc:
            messagebox.showerror("Rig", f"{exc}\n\nFalling back to the simulated rig.")
            rig, path = DEFAULT_RIG, None
        build_manager(rig, self.manager)
        self.rig_path = path
        self.rig_var.set(os.path.basename(path) if path else "simulated rig")
        self.rebuild_views()

    def load_rig_dialog(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("Rig", "*.json")])
        if path:
            self.load_rig(path)

    def save_rig_dialog(self) -> None:
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("Rig", "*.json")])
        if not path:
            return
        save_rig(path, rig_from_manager(self.manager))
        self.rig_path = path
        self.rig_var.set(os.path.basename(path))
        self.log_line(f"rig saved to {path}")

    def add_device_from_entry(self, entry: dict) -> bool:
        from flowlab.config import build_device

        bus_key = entry.get("bus")
        if bus_key not in self.manager.workers:
            messagebox.showerror("Add device", f"no bus named {bus_key!r}")
            return False
        worker = self.manager.workers[bus_key]
        protocol = "sim" if worker.bus is None else worker.bus.cfg.protocol
        try:
            device = build_device(entry, protocol, self.manager)
            self.manager.add_device(device, bus_key, float(entry.get("poll_hz", 2.0)))
        except Exception as exc:      # noqa: BLE001
            messagebox.showerror("Add device", str(exc))
            return False
        self.log_line(f"{entry['name']} added on {bus_key} ({protocol})")
        self.rebuild_views()
        return True

    def rebuild_views(self) -> None:
        self.live.rebuild()
        self.devices.refresh()
        self.sequence.refresh_devices()
        self.terminal.refresh_devices()

    # -- helpers ------------------------------------------------------------
    def watch(self, future: Future, on_ok, on_error=None) -> None:
        """Deliver a Future's result on the Tk thread (never in the callback)."""
        self._watched.append((future, on_ok, on_error))

    def log_line(self, text: str, error: bool = False) -> None:
        self.log.append(text, error)
        self.status_var.set(text if len(text) < 80 else text[:77] + "…")

    def all_off(self) -> None:
        if self.sequence.runner:
            self.sequence.stop()
        self.manager.all_off()
        self.log_line("ALL OFF: setpoints zeroed, valves held closed", error=True)

    # -- main loop ----------------------------------------------------------
    def tick(self) -> None:
        for kind, payload in self.manager.drain_events():
            if kind == EVENT_SNAPSHOT:
                continue                       # cards read state directly
            self.log_line(str(payload), error=(kind == EVENT_ERROR))

        still_waiting = []
        for future, on_ok, on_error in self._watched:
            if not future.done():
                still_waiting.append((future, on_ok, on_error))
                continue
            exception = future.exception()
            if exception is not None:
                if on_error:
                    on_error(exception)
            elif on_ok:
                on_ok(future.result())
        self._watched = still_waiting

        self.live.refresh()
        self.sequence.refresh()
        self.log.refresh()
        self.after(UI_TICK_MS, self.tick)

    def on_close(self) -> None:
        if self.sequence.runner:
            self.sequence.stop()
        self.manager.shutdown()
        self.destroy()


def main() -> int:
    rig = sys.argv[1] if len(sys.argv) > 1 else None
    App(rig).mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
