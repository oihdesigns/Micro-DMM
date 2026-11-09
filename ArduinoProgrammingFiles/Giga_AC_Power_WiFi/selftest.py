#!/usr/bin/env python3
"""
End-to-end self-test for Giga_AC_Power_WiFi. It needs no wiring: it switches
the board to its synthetic source (!SYNTH), which still runs at the ADC's pace,
and checks what comes back over the link against the values it asked for.

  py -3 selftest.py 192.168.40.225        # Wi-Fi (TCP port 8080)
  py -3 selftest.py COM36                 # USB serial
  py -3 selftest.py 192.168.40.225 20 --jumper   # also the range-switch timing test

What it checks:
  1. accuracy: Vrms, Irms, P, PF, Hz, THD at several phase angles and with a
     3rd harmonic, at 60 and 50 Hz
  2. link: every stream on at once (SMASK 31) for a while, so $PWR/$CYC/$HARM/
     $AVG sequence numbers must not skip, and $INFO must report no lost samples
  3. capture: a !WAVE capture must arrive complete while the streams keep
     running, and P recomputed from its samples must match
  4. current range: auto mode must come down to the sensitive range on a
     small current and go back up at once on a big one, reading right on both
  5. (--jumper, needs a wire from D6 to A2) the range switch must land on the
     scan where D6 actually changed, not before and not after
It returns the board to the ADC (!SYNTH,0) at the end.
"""
import math
import re
import socket
import sys
import time

import serial


class Link:
    def __init__(self, target):
        self.buf = b""
        if re.match(r"^\d+\.\d+\.\d+\.\d+(:\d+)?$", target):
            host, _, port = target.partition(":")
            self.sock = socket.create_connection((host, int(port or 8080)), timeout=15)
            self.sock.settimeout(0.2)
            self.ser = None
        else:
            self.ser = serial.Serial(target, 115200, timeout=0.2)
            self.sock = None
        time.sleep(0.3)

    def send(self, cmd):
        data = (cmd + "\n").encode()
        if self.sock:
            self.sock.sendall(data)
        else:
            self.ser.write(data)

    def lines(self, seconds):
        """Yield lines for `seconds`."""
        t_end = time.time() + seconds
        while time.time() < t_end:
            try:
                chunk = self.sock.recv(65536) if self.sock else self.ser.read(65536)
            except socket.timeout:
                chunk = b""
            if chunk:
                self.buf += chunk
            while b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                yield line.decode(errors="replace").strip()

    def wait_for(self, prefix, seconds=5):
        for l in self.lines(seconds):
            if l.startswith(prefix):
                return l
        return None

    def drain(self, seconds=0.5):
        for _ in self.lines(seconds):
            pass


def pwr_fields(line):
    f = line.split(",")[1:]
    keys = ["vrms", "irms", "p", "s", "q", "pf", "hz", "vpk", "ipk", "wh", "t", "n", "fs",
            "relay", "flags", "vthd", "ithd", "dpf", "cycles", "seq", "irange"]
    return {k: float(v) for k, v in zip(keys, f)}


def check(name, got, want, tol_rel=None, tol_abs=None):
    err = got - want
    ok = True
    if tol_abs is not None and abs(err) > tol_abs:
        ok = False
    if tol_rel is not None and want != 0 and abs(err / want) > tol_rel:
        ok = False
    rel = f"{100 * err / want:+.4f} %" if want else f"{err:+.5f}"
    print(f"    {'ok  ' if ok else 'FAIL'} {name:5s} {got:12.5f}  want {want:12.5f}  ({rel})")
    return ok


def accuracy(link):
    print("1. accuracy, synthetic V/I through the full pipeline")
    allok = True
    cases = [  # vrms, irms, deg, h3 %, hz
        (120.0, 5.0, 0.0, 0.0, 60.0),
        (120.0, 5.0, 30.0, 0.0, 60.0),
        (120.0, 5.0, 60.0, 0.0, 60.0),
        (120.0, 5.0, -45.0, 0.0, 60.0),
        (120.0, 2.0, 0.0, 30.0, 60.0),
        (230.0 / 2, 10.0, 25.0, 0.0, 50.0),
        (120.0, 5.0, 30.0, 0.0, 59.7),
    ]
    for vrms, irms, deg, h3, hz in cases:
        link.send(f"!MAINS,{50 if hz < 55 else 60}")
        link.send(f"!SYNTH,1,{vrms},{irms},{deg},{h3},{hz}")
        link.send("!SMASK,1")
        link.drain(1.0)               # let the analyser lock and a window pass
        rows = []
        for l in link.lines(2.5):
            if l.startswith("$PWR"):
                rows.append(pwr_fields(l))
        if len(rows) < 3:
            print(f"  FAIL: only {len(rows)} $PWR lines"); allok = False; continue
        r = rows[-1]
        # with a 3rd harmonic on I, P comes only from the fundamental
        i1 = irms / math.sqrt(1 + (h3 / 100) ** 2)
        p = vrms * i1 * math.cos(math.radians(deg))
        print(f"  V {vrms} V, I {irms} A, {deg} deg, h3 {h3} %, {hz} Hz   flags {int(r['flags'])}")
        allok &= check("vrms", r["vrms"], vrms, tol_rel=2e-4)
        allok &= check("irms", r["irms"], irms, tol_rel=2e-4)
        allok &= check("P", r["p"], p, tol_abs=max(2e-4 * vrms * irms, 0.01))
        allok &= check("PF", r["pf"], p / (vrms * irms), tol_abs=3e-4)
        allok &= check("Hz", r["hz"], hz, tol_rel=1e-4)
        allok &= check("ITHD", r["ithd"], h3, tol_abs=0.05)
        allok &= check("DPF", r["dpf"], math.cos(math.radians(deg)), tol_abs=3e-4)
    link.send("!MAINS,60")
    return allok


def link_test(link, seconds=20):
    print(f"2. link: all streams for {seconds} s, plus a capture in the middle")
    link.send("!SYNTH,1,120,5,30,10,60")
    link.send("!WLEN,1.0")
    link.send("!SMASK,31")
    link.drain(1.0)
    seqs = {"$PWR": [], "$CYC": [], "$HARM": [], "$AVG": []}
    infos = []
    wave = None
    wave_samples = []
    nbytes = 0
    asked = False
    t0 = time.time()
    for l in link.lines(seconds):
        nbytes += len(l) + 1
        if not asked and time.time() - t0 > seconds / 3:
            link.send("!WAVE"); asked = True
        head = l.split(",", 1)[0]
        if head in ("$PWR",):
            seqs[head].append(int(l.split(",")[20]))
        elif head == "$CYC":
            seqs[head].append(int(l.split(",")[1]))
        elif head in ("$HARM", "$AVG"):
            seqs[head].append(int(l.split(",")[1]))
        elif head == "$INFO":
            infos.append(l)
        elif head == "$WAVE":
            wave = l.split(",")[1:]; wave_samples = []
        elif head == "$WD":
            a, b = l.split(",")[1:3]
            wave_samples.append((float(a), float(b)))
        elif head == "$WEND":
            print(f"   capture: header n={wave[0]}, got {len(wave_samples)} samples")
    el = time.time() - t0
    ok = True
    for k, v in seqs.items():
        miss = sum(b - a - 1 for a, b in zip(v, v[1:]) if b - a > 1)
        rate = len(v) / el
        # $PWR must be complete. The bulk streams are shed on purpose when an
        # output backs up (a capture is streaming, or the link is slow).
        allowed = 0 if k == "$PWR" else max(2, len(v) // 20)
        print(f"   {k:5s} {len(v):5d} lines  {rate:6.1f}/s   skipped {miss}"
              + ("" if k == "$PWR" else f" (bulk, {allowed} allowed)"))
        ok &= miss <= allowed and len(v) > 0
    print(f"   {nbytes / el / 1024:.1f} KB/s received")
    if infos:
        f = infos[-1].split(",")
        print(f"   last $INFO: rate {f[1]}  gaps {f[2]}  load {f[4]} %  loop max {f[5]} ms  "
              f"backlog {f[6]}  txdrop tcp {f[7]} ser {f[8]}  tcp busy {f[9]}")
        ok &= int(f[2]) == 0 and int(f[7]) == 0
    else:
        print("   FAIL: no $INFO"); ok = False
    if wave and len(wave_samples) == int(wave[0]):
        n = len(wave_samples)
        kv, bv, ki, bi, alpha = map(float, wave[2:7])
        v = [a * kv + bv for a, _ in wave_samples]
        i = [b * ki + bi for _, b in wave_samples]
        # P with V interpolated (linearly is enough here) to the I instants
        p = sum((v[k] + alpha * (v[k + 1] - v[k])) * i[k] for k in range(n - 1)) / (n - 1)
        want = 120 * (5 / math.sqrt(1.01)) * math.cos(math.radians(30))
        print(f"   capture P from samples {p:.3f} W, want ~{want:.3f} (window not cycle-aligned)")
        ok &= abs(p - want) / want < 0.01
    else:
        print("   FAIL: capture incomplete"); ok = False
    link.send("!SMASK,0")
    return ok


def range_test(link):
    print("4. current range, auto mode (range 0 = 25 A, range 1 = 2.5 A per 1.5 V)")
    link.send("!ISCALE,1.5,25,0"); link.send("!ISCALE,1.5,2.5,1")
    link.send("!IRANGE,0"); link.send("!SMASK,1")
    link.send("!SYNTH,1,120,1,30,0,60")
    link.drain(1.0)
    ok = True
    link.send("!IRANGE,A")
    rows = [pwr_fields(l) for l in link.lines(4.0) if l.startswith("$PWR")]
    ranges = [int(r["irange"]) for r in rows]
    print(f"   1 A rms (1.41 A pk): ranges seen {sorted(set(ranges))}, last {ranges[-1] if ranges else '-'}")
    ok &= bool(ranges) and ranges[-1] == 1
    for r in rows[-3:]:
        ok &= check("irms", r["irms"], 1.0, tol_rel=2e-4)
    t0 = time.time()
    link.send("!SYNTH,1,120,3,30,0,60")
    rows, t_up = [], None
    for l in link.lines(2.5):
        if l.startswith("$IRNG") and l.split(",")[1] == "0" and t_up is None:
            t_up = time.time() - t0
        if l.startswith("$PWR"):
            rows.append(pwr_fields(l))
    ranges = [int(r["irange"]) for r in rows]
    print(f"   3 A rms (4.24 A pk): back on range 0 after {t_up if t_up is None else round(t_up, 2)} s, "
          f"last {ranges[-1] if ranges else '-'}")
    ok &= t_up is not None and t_up < 0.5 and bool(ranges) and ranges[-1] == 0
    for r in rows[-3:]:
        ok &= check("irms", r["irms"], 3.0, tol_rel=2e-4)
    link.send("!IRANGE,0")
    link.drain(0.5)
    return ok


def jumper_test(link):
    """D6 drives A2 directly. On range 0 the pin is LOW, so A2 reads 0 V; on
    range 1 it is HIGH, 3.3 V. With the current channel single-ended, each
    scan is then one of four values, and two of them can only come from a
    scan scaled with the wrong range: switched too early or too late."""
    print("5. range switch timing (jumper D6 -> A2)")
    for c in ("!SYNTH,0", "!SMASK,0", "!SE,0,1", "!ISCALE,1.5,25,0", "!ISCALE,1.5,2.5,1",
              "!IOFF,0,0", "!IOFF,0,1", "!IRSETTLE,0", "!IRANGE,0", "!WLEN,0.02", "!WPRE,0.5",
              "!DECIM,1"):
        link.send(c)
    link.drain(1.5)
    ok = True
    for target in ("1", "0"):
        link.send("!WAVE"); link.send(f"!IRANGE,{target}")
        head, d = None, []
        for l in link.lines(6.0):
            if l.startswith("$WAVE"):
                head = l.split(",")[1:]
            elif l.startswith("$WD") and head:
                d.append(float(l.split(",")[2]))
            elif l.startswith("$WEND") and head:
                break
        if not head or len(d) != int(head[0]):
            print("   FAIL: capture incomplete"); ok = False; continue
        ki = float(head[4])
        amps = [x * ki for x in d]
        # LOW on range 0: -27.5 A; HIGH on range 1: +2.75 A. HIGH scaled as
        # range 0 reads +27.5 A, LOW scaled as range 1 reads -2.75 A.
        lo0 = sum(a < -10 for a in amps); hi1 = sum(0 < a < 10 for a in amps)
        hi0 = sum(a > 10 for a in amps); lo1 = sum(-10 < a <= 0 for a in amps)
        if target == "1":
            right, early, late = (lo0, hi1), lo1, hi0
        else:
            right, early, late = (hi1, lo0), hi0, lo1
        print(f"   to range {target}: {right[0]} + {right[1]} scans scaled right, "
              f"{early} too early, {late} too late (one scan = 12.5 us)")
        ok &= right[0] > 100 and right[1] > 100 and early <= 1 and late <= 1
    for c in ("!DECIM,10", "!SE,0,0", "!IRSETTLE,20", "!IRANGE,0", "!WLEN,0.5", "!WPRE,0.25"):
        link.send(c)
    link.drain(1.5)
    return ok


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "192.168.40.225"
    link = Link(target)
    link.send("!CFG")
    print(link.wait_for("$ADC") or "no $ADC")
    a = accuracy(link)
    args = [x for x in sys.argv[2:] if not x.startswith("--")]
    b = link_test(link, float(args[0]) if args else 20)
    c = range_test(link)
    d = jumper_test(link) if "--jumper" in sys.argv else True
    link.send("!SYNTH,0")
    link.send("!SMASK,0")
    link.drain(0.5)
    ok = a and b and c and d
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
