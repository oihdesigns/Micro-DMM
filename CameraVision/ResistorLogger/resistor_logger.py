"""Logs a resistance and a proof photo each time the DM858 settles on a new contact.

Needs LiveViewer running. Leaves the DMM's function and range as they are. Set the
photo exposure with --aperture/--shutter/--iso; the old settings return on exit.
Create a file named STOP in the session folder to end a run cleanly.
"""
import argparse
import csv
import io
import json
import socket
import time
import urllib.parse
import urllib.request
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image

VIEWER = "http://localhost:8765"
DMM = ("192.168.10.5", 5025)
OVERLOAD = 1e30        # The DM858 reports open leads and over-range as 9.9E37.
SETTLE_READS = 3
HELD_READS = 3
STILL_LIMIT = 1.6      # Mean frame change, in grey levels, that counts as a still scene.
STILL_WAIT = 4.0       # Shoot anyway after this long, and say so in the log.


class Dmm:
    def __init__(self):
        self.sock = socket.create_connection(DMM, timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.file = self.sock.makefile("rb")

    def query(self, cmd):
        self.sock.sendall((cmd + "\n").encode())
        return self.file.readline().decode(errors="replace").strip()

    def read(self):
        return float(self.query("READ?"))


def post(path, **query):
    query["who"] = "resistor-logger"
    url = VIEWER + path + "?" + urllib.parse.urlencode(query)
    with urllib.request.urlopen(urllib.request.Request(url, data=b"", method="POST"), timeout=40) as r:
        return json.load(r)


def status():
    with urllib.request.urlopen(VIEWER + "/status", timeout=5) as r:
        return json.load(r)


def frame():
    with urllib.request.urlopen(VIEWER + "/frame.jpg", timeout=5) as r:
        return np.asarray(Image.open(io.BytesIO(r.read())).convert("L"), dtype=float)


def wait_for_still():
    """Waits for hands to leave. A settled DMM can come well before the scene is clear."""
    end = time.time() + STILL_WAIT
    previous = frame()
    while time.time() < end:
        time.sleep(0.3)
        current = frame()
        if np.abs(current - previous).mean() < STILL_LIMIT:
            return True
        previous = current
    return False


def tolerance(ohms):
    return max(abs(ohms) * 0.001, 0.0005)


def settled(reads):
    return len(reads) >= SETTLE_READS and max(reads) - min(reads) <= tolerance(reads[-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--aperture", default="11")
    ap.add_argument("--shutter", default="1/40")
    ap.add_argument("--iso", default="1600")
    ap.add_argument("--minutes", type=float, default=120)
    args = ap.parse_args()

    root = Path(__file__).resolve().parent.parent / "captures"
    session = root / datetime.now().strftime("resistors_%Y%m%d_%H%M%S")
    session.mkdir(parents=True)
    log = session / "readings.csv"
    print("session", session, flush=True)

    dmm = Dmm()
    print("DMM", dmm.query("CONF?"), flush=True)
    before = status()
    for key in ("aperture", "shutter", "iso"):
        post("/api/set", key=key, value=getattr(args, key))

    count = 0
    reads = []
    last = None
    armed = True
    end = time.time() + args.minutes * 60
    try:
        with log.open("w", newline="", encoding="utf-8") as f:
            out = csv.writer(f)
            out.writerow(["n", "time", "ohms", "spread", "held", "still", "photo"])
            while time.time() < end and not (session / "STOP").exists():
                v = dmm.read()
                if v > OVERLOAD:
                    reads.clear()
                    armed = True
                    continue
                reads = (reads + [v])[-SETTLE_READS:]
                if not settled(reads):
                    continue
                ohms = sum(reads) / len(reads)
                if not armed and abs(ohms - last) <= tolerance(last) * 5:
                    continue

                still = wait_for_still()
                count += 1
                shot = post("/api/capture")["file"]
                after = [dmm.read() for _ in range(HELD_READS)]
                held = all(abs(x - ohms) <= tolerance(ohms) * 3 for x in after)
                spread = max(reads + after) - min(reads + after)
                Image.open(root / shot).crop((1900, 900, 3300, 2100)).save(session / f"{count:02d}.jpg", quality=90)
                out.writerow([count, datetime.now().strftime("%H:%M:%S"), f"{ohms:.6g}", f"{spread:.3g}", held, still, shot])
                f.flush()
                print(f"#{count} {ohms:.6g} ohm held={held} still={still} {shot}", flush=True)
                last = ohms
                armed = False
                reads.clear()
    finally:
        for key in ("aperture", "shutter", "iso"):
            post("/api/set", key=key, value=before[key])
        print("settings restored; readings:", count, flush=True)


if __name__ == "__main__":
    main()
