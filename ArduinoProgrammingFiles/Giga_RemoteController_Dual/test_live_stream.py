"""Drive the live-stream window against a scripted fake board."""
import sys, time, threading, queue
sys.path.insert(0, r"C:\Users\Nick\Documents\GitHub\Micro-DMM\ArduinoProgrammingFiles\Giga_RemoteController_Dual")
import matplotlib
matplotlib.use("Agg")
import tkinter as tk
import Giga_RemoteController as G


class FakeBoard:
    """Emits the same lines Giga_RemoteController_Dual does.

    Its own Diff config is the REVERSE of the GUI's, which is exactly the drift
    that made D1 report D2's value: the board's built-in preset table and the
    GUI's giga_presets.json disagree about which pair is number one.
    """

    BOARD_OWN_PAIRS = {"D1": "A2-A3", "D2": "A4-A5"}

    def __init__(self, periods=6, hz=4):
        self.is_open = True
        self.written = []
        self.honoured = {}
        self._lines = queue.Queue()
        self._periods = periods
        self._hz = hz

    def write(self, data):
        self.written.append(data)
        txt = data.decode()
        if txt.startswith("STREAM:"):
            # Honour the legs the host named; fall back to the board's own pairs
            # when it names none (the old, broken behaviour).
            named = {}
            for part in txt.strip().split("|"):
                if part.startswith(("D1:", "D2:")):
                    k, _, v = part.partition(":")
                    named[k] = v
            slots = txt.split("|")[0].removeprefix("STREAM:").split(",")
            self.honoured = {d: named.get(d, self.BOARD_OWN_PAIRS[d])
                             for d in ("D1", "D2") if d in slots}
            echo = "".join(f"|{d}:{v}" for d, v in self.honoured.items())
            line = ("STREAMON|HZ:4|BITS:14|RATE:250000|SMOOTH:8"
                    "|CH:" + ",".join(slots) + echo + "\n")
            self._lines.put(line.encode())
            threading.Thread(target=self._pump, daemon=True).start()
        elif txt.startswith("STOP"):
            self._lines.put(b"STREAMOFF\n")
        return len(data)

    def _pump(self):
        span = int(1000 / self._hz)
        for seq in range(1, self._periods + 1):
            time.sleep(0.02)
            t = seq * span
            for ch, base in (("A0", 8200), ("A1", 4100), ("D1", -1500)):
                self._lines.put(
                    f"S|SEQ:{seq}|T:{t}|SPAN:{span}|CH:{ch}|N:31250"
                    f"|MIN:{base-40}|MAX:{base+40}|MEAN:{base + seq}.2500"
                    f"|STD:24.7500|LAST:{base+3}\n".encode())
            self._lines.put(f"E|SEQ:{seq}|BITS:14|DROP:0\n".encode())

    def readline(self):
        try:
            return self._lines.get(timeout=0.5)
        except queue.Empty:
            return b""

    def close(self):
        self.is_open = False


def main():
    root = tk.Tk()
    root.withdraw()
    app = G.GigaTestGUI(root)

    # A0 + A1 active, Diff 1 on (A4-A5 in this window)
    for name, on in [("A0", True), ("A1", True)] + [(f"A{i}", False) for i in range(2, 8)]:
        app.pin_configs[name]["active"].set(on)
    app.diff_enable.set(True)
    app.diff_pos_var.set("A4")
    app.diff_neg_var.set("A5")
    app.bit_var.set("14")

    board = FakeBoard()
    app._open_connection = lambda timeout=60, port=None: board

    app.open_live_stream()
    win = app.live_win
    assert win.hz_var.get() == "4", f"default rate is {win.hz_var.get()}, expected 4"
    win.start()

    deadline = time.time() + 8
    while time.time() < deadline and len(win._hist["A0"]["t"]) < 6:
        root.update()
        time.sleep(0.02)

    print("status            :", win.status_var.get())
    print("channels streamed :", win._names)
    print("command sent      :", board.written[0].decode().strip())
    print("D1 pins asked     :", win._want_legs)
    print("D1 pins board used:", board.honoured)
    for n in win._names:
        h = win._hist[n]
        print(f"  {n:3s} periods={len(h['t'])} "
              f"last mean={h['mean'][-1]:.5g} min={h['min'][-1]:.5g} "
              f"max={h['max'][-1]:.5g} std={h['std'][-1]:.5g} n={h['n'][-1]}")
    for n, (val, sub) in win._cards.items():
        print(f"  card {n:3s}: {val.cget('text'):>16s} | {sub.cget('text')}")

    assert len(win._hist["A0"]["t"]) >= 6, "no periods received"

    # The regression: the GUI must NAME the pins rather than leave the board to
    # pick from its own (here reversed) Diff config.
    cmd = board.written[0].decode().strip()
    assert "|D1:A4-A5" in cmd, f"STREAM command did not name D1's pins: {cmd}"
    assert board.honoured["D1"] == "A4-A5", \
        f"board fell back to its own pair: {board.honoured}"
    assert win._diff_legs.get("D1") == "A4-A5", win._diff_legs
    assert not win.status_var.get().startswith("PIN MISMATCH"), win.status_var.get()
    print("pin naming        : ok (board used the legs the GUI asked for)")

    # Calibration, checked by hand against the raw counts the fake board sent.
    v_step = 3.3 / (2 ** 14 - 1)
    off = float(app.pin_configs["A0"]["offset"].get())
    scl = float(app.pin_configs["A0"]["scale"].get())
    want = (8206.25 * v_step - off) * scl
    got = win._hist["A0"]["mean"][-1]
    assert abs(want - got) < 1e-9, f"calibration mismatch: {got} vs {want}"
    want_sd = 24.75 * v_step * abs(scl)
    assert abs(win._hist["A0"]["std"][-1] - want_sd) < 1e-9, "sigma should carry gain only"
    d_off = float(app.diff_offset_var.get())
    d_scl = float(app.diff_scale_var.get())
    want_d = (-1494.25 * v_step - d_off) * d_scl
    assert abs(win._hist["D1"]["mean"][-1] - want_d) < 1e-9, "diff calibration"
    print("calibration       : ok (channel mean, sigma, and pair)")

    win.stop()
    deadline = time.time() + 5
    while time.time() < deadline and win.running:
        root.update()
        time.sleep(0.02)
    print("stop sent         :", any(w.startswith(b"STOP") for w in board.written))
    print("running after stop:", win.running)

    win._on_close()
    root.destroy()
    print("main checks       : PASSED")


def test_mismatch_is_flagged():
    """A board that ignores the named pins must not be plotted silently."""
    root = tk.Tk()
    root.withdraw()
    app = G.GigaTestGUI(root)
    for name, on in [("A0", True)] + [(f"A{i}", False) for i in range(1, 8)]:
        app.pin_configs[name]["active"].set(on)
    app.diff_enable.set(True)
    app.diff_pos_var.set("A4")
    app.diff_neg_var.set("A5")

    class StubbornBoard(FakeBoard):
        def write(self, data):
            txt = data.decode()
            if txt.startswith("STREAM:"):
                self.written.append(data)
                self._lines.put(
                    b"STREAMON|HZ:4|BITS:14|RATE:250000|SMOOTH:8|CH:A0,D1|D1:A2-A3\n")
                return len(data)
            return super().write(data)

    board = StubbornBoard()
    app._open_connection = lambda timeout=60, port=None: board
    app.open_live_stream()
    win = app.live_win
    win.start()
    deadline = time.time() + 4
    while time.time() < deadline and not win.status_var.get().startswith("PIN MISMATCH"):
        root.update()
        time.sleep(0.02)
    print("mismatch status   :", win.status_var.get())
    assert win.status_var.get().startswith("PIN MISMATCH"), \
        "a board using different pins must be reported, not plotted"
    win.stop()
    deadline = time.time() + 3
    while time.time() < deadline and win.running:
        root.update()
        time.sleep(0.02)
    win._on_close()
    root.destroy()
    print("mismatch detection: PASSED")


main()
test_mismatch_is_flagged()
print("\nALL CHECKS PASSED")
