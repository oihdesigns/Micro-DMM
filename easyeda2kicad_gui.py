"""Small GUI front end for easyeda2kicad.

Paste an LCSC part number (or an LCSC URL), hit Import, and it runs:

    py -m easyeda2kicad --lcsc_id=C53055273 --symbol --footprint --3d --output <lib>

then registers the resulting library in KiCad's global symbol/footprint tables so
the part shows up in the choosers without any manual "Add library" step.

Multiple part numbers can be pasted at once, separated by spaces or commas.
Settings are remembered in ~/.easyeda2kicad_gui.json
"""

import json
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import filedialog, ttk

CONFIG_PATH = os.path.join(os.path.expanduser("~"), ".easyeda2kicad_gui.json")
DEFAULT_OUTPUT = os.path.join(os.path.expanduser("~"), "Documents", "KiCad", "easyeda_parts")
KICAD_CONFIG_ROOT = os.path.join(os.environ.get("APPDATA", os.path.expanduser("~")), "kicad")

# Matches C53055273 anywhere -- handles bare IDs, "LCSC: C123", and lcsc.com URLs.
LCSC_RE = re.compile(r"C\d{3,}", re.IGNORECASE)

# Processes that rewrite the library tables on exit, clobbering edits made behind them.
KICAD_PROCESSES = ("kicad.exe", "eeschema.exe", "pcbnew.exe")


def parse_part_numbers(text):
    """Pull every LCSC id out of pasted text, de-duplicated, order preserved."""
    found = []
    for match in LCSC_RE.findall(text):
        part = "C" + match[1:]
        if part not in found:
            found.append(part)
    return found


def detect_kicad_configs():
    """[(version, config_dir)] for each KiCad config that has library tables, newest first."""
    found = []
    try:
        names = os.listdir(KICAD_CONFIG_ROOT)
    except OSError:
        return found
    for name in names:
        path = os.path.join(KICAD_CONFIG_ROOT, name)
        if (os.path.isfile(os.path.join(path, "sym-lib-table"))
                and os.path.isfile(os.path.join(path, "fp-lib-table"))):
            found.append((name, path))

    def version_key(item):
        try:
            return tuple(int(part) for part in item[0].split("."))
        except ValueError:
            return (0,)

    found.sort(key=version_key, reverse=True)
    return found


def kicad_running():
    """Names of running KiCad processes -- they rewrite the lib tables when they close."""
    try:
        proc = subprocess.run(
            ["tasklist", "/fo", "csv", "/nh"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            encoding="utf-8",
            errors="replace",
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
    except OSError:
        return []
    listing = (proc.stdout or "").lower()
    return [name for name in KICAD_PROCESSES if '"%s"' % name in listing]


def update_lib_table(table_path, lib_name, uri, descr):
    """Add or correct one (lib ...) entry. Returns 'added', 'updated' or 'present'."""
    with open(table_path, "r", encoding="utf-8", newline="") as handle:
        text = handle.read()
    eol = "\r\n" if "\r\n" in text else "\n"

    entry = '  (lib (name "%s")(type "KiCad")(uri "%s")(options "")(descr "%s"))' % (
        lib_name, uri, descr)
    # [^\r\n]* rather than .* -- '.' would swallow the CR of a CRLF line ending.
    pattern = re.compile(
        r'[ \t]*\(lib[ \t]+\(name[ \t]+"%s"[ \t]*\)[^\r\n]*' % re.escape(lib_name))
    match = pattern.search(text)

    if match and match.group(0).strip() == entry.strip():
        return "present"

    if match:
        new_text = text[:match.start()] + entry + text[match.end():]
        action = "updated"
    else:
        close = text.rindex(")")  # the table's own closing paren
        new_text = text[:close] + entry + eol + text[close:]
        action = "added"

    backup = table_path + ".easyeda2kicad_gui.bak"
    if not os.path.exists(backup):
        shutil.copy2(table_path, backup)
    with open(table_path, "w", encoding="utf-8", newline="") as handle:
        handle.write(new_text)
    return action


def register_in_kicad(config_dir, out_base, want_symbol, want_footprint):
    """Point KiCad's global tables at the library easyeda2kicad just wrote.

    Yields (tag, message) pairs for the log.
    """
    lib_name = os.path.basename(out_base)
    if not lib_name:
        yield ("err", "Cannot register: output library has no name.\n")
        return

    busy = kicad_running()
    if busy:
        yield ("err",
               "KiCad is running (%s). It rewrites the library tables when it closes, so it\n"
               "would discard these entries -- close KiCad and press 'Register in KiCad' again.\n"
               % ", ".join(busy))
        return

    targets = []
    if want_symbol:
        targets.append(("sym-lib-table", out_base + ".kicad_sym", "symbol"))
    if want_footprint:
        targets.append(("fp-lib-table", out_base + ".pretty", "footprint"))

    for table_name, lib_path, kind in targets:
        if not os.path.exists(lib_path):
            yield ("err", "No %s library at %s -- nothing to register.\n" % (kind, lib_path))
            continue
        table_path = os.path.join(config_dir, table_name)
        uri = lib_path.replace("\\", "/")
        try:
            action = update_lib_table(table_path, lib_name, uri, "easyeda2kicad imports")
        except (OSError, ValueError) as exc:
            yield ("err", "Could not update %s: %s\n" % (table_path, exc))
            continue
        if action == "present":
            yield (None, "%s table already lists '%s'.\n" % (kind.capitalize(), lib_name))
        else:
            yield ("ok", "%s %s library '%s' -> %s\n" % (action.capitalize(), kind, lib_name, uri))

    yield (None, "Restart KiCad (or Preferences > Manage Libraries) to see new entries.\n")


def load_config():
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return {}


def save_config(cfg):
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as handle:
            json.dump(cfg, handle, indent=2)
    except OSError:
        pass


class App:
    def __init__(self, root):
        self.root = root
        self.cfg = load_config()
        self.msgq = queue.Queue()
        self.running = False

        root.title("easyeda2kicad importer")
        root.minsize(720, 480)

        frm = ttk.Frame(root, padding=10)
        frm.pack(fill="both", expand=True)
        frm.columnconfigure(1, weight=1)

        # --- part number ---
        ttk.Label(frm, text="LCSC part #:").grid(row=0, column=0, sticky="w", pady=(0, 6))
        self.part_var = tk.StringVar()
        self.part_entry = ttk.Entry(frm, textvariable=self.part_var, font=("Consolas", 12))
        self.part_entry.grid(row=0, column=1, sticky="ew", padx=6, pady=(0, 6))
        self.part_entry.bind("<Return>", lambda _e: self.start())
        ttk.Button(frm, text="Paste", command=self.paste).grid(row=0, column=2, pady=(0, 6))

        # --- output library ---
        ttk.Label(frm, text="Output library:").grid(row=1, column=0, sticky="w")
        self.out_var = tk.StringVar(value=self.cfg.get("output", DEFAULT_OUTPUT))
        ttk.Entry(frm, textvariable=self.out_var).grid(row=1, column=1, sticky="ew", padx=6)
        ttk.Button(frm, text="Browse...", command=self.browse).grid(row=1, column=2)

        ttk.Label(
            frm,
            text="(base name -- easyeda2kicad makes <name>.kicad_sym, <name>.pretty, <name>.3dshapes)",
            foreground="#666666",
        ).grid(row=2, column=1, sticky="w", padx=6, pady=(2, 8))

        # --- options ---
        opts = ttk.Frame(frm)
        opts.grid(row=3, column=1, sticky="w", padx=6, pady=(0, 8))
        self.symbol_var = tk.BooleanVar(value=self.cfg.get("symbol", True))
        self.footprint_var = tk.BooleanVar(value=self.cfg.get("footprint", True))
        self.model_var = tk.BooleanVar(value=self.cfg.get("model", True))
        self.overwrite_var = tk.BooleanVar(value=self.cfg.get("overwrite", False))
        for text, var in (
            ("Symbol", self.symbol_var),
            ("Footprint", self.footprint_var),
            ("3D model", self.model_var),
            ("Overwrite existing", self.overwrite_var),
        ):
            ttk.Checkbutton(opts, text=text, variable=var).pack(side="left", padx=(0, 12))

        # --- KiCad registration ---
        self.kicad_configs = detect_kicad_configs()
        kicad_row = ttk.Frame(frm)
        kicad_row.grid(row=4, column=1, sticky="w", padx=6, pady=(0, 8))
        self.register_var = tk.BooleanVar(value=self.cfg.get("register", True))
        ttk.Checkbutton(kicad_row, text="Add library to KiCad after import",
                        variable=self.register_var).pack(side="left", padx=(0, 12))
        ttk.Label(kicad_row, text="KiCad version:").pack(side="left")
        self.kicad_var = tk.StringVar()
        versions = [version for version, _path in self.kicad_configs]
        self.kicad_combo = ttk.Combobox(kicad_row, textvariable=self.kicad_var, values=versions,
                                        state="readonly", width=8)
        self.kicad_combo.pack(side="left", padx=6)
        saved_version = self.cfg.get("kicad_version")
        if saved_version in versions:
            self.kicad_var.set(saved_version)
        elif versions:
            self.kicad_var.set(versions[0])
        else:
            self.kicad_combo.configure(state="disabled")
            self.register_var.set(False)

        # --- action row ---
        actions = ttk.Frame(frm)
        actions.grid(row=5, column=1, sticky="ew", padx=6, pady=(0, 8))
        self.run_btn = ttk.Button(actions, text="Import", command=self.start)
        self.run_btn.pack(side="left")
        ttk.Button(actions, text="Register in KiCad", command=self.register_only).pack(side="left", padx=8)
        ttk.Button(actions, text="Open output folder", command=self.open_folder).pack(side="left")
        ttk.Button(actions, text="Clear log", command=self.clear_log).pack(side="left", padx=8)
        self.status = ttk.Label(actions, text="Ready", foreground="#444444")
        self.status.pack(side="left", padx=12)

        # --- log ---
        logframe = ttk.Frame(frm)
        logframe.grid(row=6, column=0, columnspan=3, sticky="nsew")
        frm.rowconfigure(6, weight=1)
        logframe.rowconfigure(0, weight=1)
        logframe.columnconfigure(0, weight=1)
        self.log = tk.Text(logframe, height=14, wrap="word", font=("Consolas", 9),
                           background="#101010", foreground="#d0d0d0", insertbackground="#d0d0d0")
        self.log.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(logframe, command=self.log.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scroll.set, state="disabled")
        self.log.tag_configure("cmd", foreground="#00b7ff")
        self.log.tag_configure("ok", foreground="#00ff32")
        self.log.tag_configure("err", foreground="#ff5555")

        self.part_entry.focus_set()
        self.root.after(100, self.drain)

    # ------------------------------------------------------------------ utils

    def write(self, text, tag=None):
        self.log.configure(state="normal")
        self.log.insert("end", text, tag)
        self.log.see("end")
        self.log.configure(state="disabled")

    def clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def paste(self):
        try:
            self.part_var.set(self.root.clipboard_get())
        except tk.TclError:
            pass

    def browse(self):
        current = self.out_var.get()
        initial = os.path.dirname(current) or os.path.expanduser("~")
        path = filedialog.asksaveasfilename(
            title="Output library base name (no extension)",
            initialdir=initial,
            initialfile=os.path.basename(current),
            confirmoverwrite=False,
        )
        if path:
            self.out_var.set(os.path.splitext(path)[0])

    def open_folder(self):
        folder = os.path.dirname(self.out_var.get()) or "."
        if os.path.isdir(folder):
            os.startfile(folder)
        else:
            self.write("Folder does not exist yet: %s\n" % folder, "err")

    def persist(self):
        save_config({
            "output": self.out_var.get(),
            "symbol": self.symbol_var.get(),
            "footprint": self.footprint_var.get(),
            "model": self.model_var.get(),
            "overwrite": self.overwrite_var.get(),
            "register": self.register_var.get(),
            "kicad_version": self.kicad_var.get(),
        })

    def kicad_config_dir(self):
        for version, path in self.kicad_configs:
            if version == self.kicad_var.get():
                return path
        return None

    def register_only(self):
        """Register the current library without importing anything."""
        if self.running:
            return
        config_dir = self.kicad_config_dir()
        if config_dir is None:
            self.write("No KiCad configuration found under %s.\n" % KICAD_CONFIG_ROOT, "err")
            return
        self.persist()
        for tag, message in register_in_kicad(
            config_dir, self.out_var.get().strip(),
            self.symbol_var.get(), self.footprint_var.get()
        ):
            self.write(message, tag)

    # ------------------------------------------------------------------- run

    def start(self):
        if self.running:
            return
        parts = parse_part_numbers(self.part_var.get())
        if not parts:
            self.write("No LCSC part number found in that text (expecting something like C53055273).\n", "err")
            return
        if not (self.symbol_var.get() or self.footprint_var.get() or self.model_var.get()):
            self.write("Pick at least one of Symbol / Footprint / 3D model.\n", "err")
            return

        out = self.out_var.get().strip()
        parent = os.path.dirname(out)
        if parent and not os.path.isdir(parent):
            try:
                os.makedirs(parent, exist_ok=True)
                self.write("Created %s\n" % parent)
            except OSError as exc:
                self.write("Cannot create %s: %s\n" % (parent, exc), "err")
                return

        config_dir = self.kicad_config_dir() if self.register_var.get() else None
        if self.register_var.get() and config_dir is None:
            self.write("No KiCad configuration found under %s -- skipping registration.\n"
                       % KICAD_CONFIG_ROOT, "err")

        self.persist()
        self.running = True
        self.run_btn.configure(state="disabled")
        self.status.configure(text="Working...", foreground="#b07000")
        threading.Thread(target=self.worker, args=(parts, out, config_dir), daemon=True).start()

    def worker(self, parts, out, config_dir):
        want_symbol = self.symbol_var.get()
        want_footprint = self.footprint_var.get()
        want_model = self.model_var.get()
        want_overwrite = self.overwrite_var.get()
        failures = 0

        for part in parts:
            cmd = [sys.executable, "-m", "easyeda2kicad", "--lcsc_id=%s" % part]
            if want_symbol:
                cmd.append("--symbol")
            if want_footprint:
                cmd.append("--footprint")
            if want_model:
                cmd.append("--3d")
            if want_overwrite:
                cmd.append("--overwrite")
            cmd += ["--output", out]

            self.msgq.put(("cmd", "\n> " + subprocess.list2cmdline(cmd) + "\n"))
            try:
                proc = subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
            except OSError as exc:
                self.msgq.put(("err", "Failed to launch easyeda2kicad: %s\n" % exc))
                failures += 1
                continue

            if proc.stdout:
                text = proc.stdout if proc.stdout.endswith("\n") else proc.stdout + "\n"
                self.msgq.put((None, text))
            if proc.returncode == 0:
                self.msgq.put(("ok", "%s done.\n" % part))
            else:
                failures += 1
                self.msgq.put(("err", "%s failed (exit %d).\n" % (part, proc.returncode)))

        if config_dir is not None and failures < len(parts):
            self.msgq.put((None, "\nRegistering '%s' with KiCad...\n" % os.path.basename(out)))
            for tag, message in register_in_kicad(config_dir, out, want_symbol, want_footprint):
                self.msgq.put((tag, message))

        self.msgq.put(("__done__", (len(parts), failures)))

    def drain(self):
        try:
            while True:
                tag, payload = self.msgq.get_nowait()
                if tag == "__done__":
                    total, failures = payload
                    self.running = False
                    self.run_btn.configure(state="normal")
                    if failures:
                        self.status.configure(text="%d of %d failed" % (failures, total), foreground="#c00000")
                    else:
                        self.status.configure(text="Imported %d part(s)" % total, foreground="#008000")
                        self.part_var.set("")
                    self.part_entry.focus_set()
                else:
                    self.write(payload, tag)
        except queue.Empty:
            pass
        self.root.after(100, self.drain)


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except tk.TclError:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
