#!/usr/bin/env python3
"""
End-to-end self-test for Giga_AC_Power_WiFi. It needs no wiring: it switches
the board to its synthetic source (!SYNTH), which still runs at the ADC's pace,
and checks what comes back over the link against the values it asked for.

  py -3 selftest.py 192.168.40.225        # Wi-Fi (TCP port 8080)
  py -3 selftest.py COM36                 # USB serial

What it checks:
  1. accuracy: Vrms, Irms, P, PF, Hz, THD at several phase angles and with a
     3rd harmonic, at 60 and 50 Hz
  2. link: every stream on at once (SMASK 31) for a while, so $PWR/$CYC/$HARM/
     $AVG sequence numbers must not skip, and $INFO must report no lost samples
  3. capture: a !WAVE capture must arrive complete while the streams keep
     running, and P recomputed from its samples must match
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
            "relay", "flags", "vthd", "ithd", "dpf", "cycles", "seq"]
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
            seqs[head].append(int(l.rsplit(",", 1)[1]))
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


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "192.168.40.225"
    link = Link(target)
    link.send("!CFG")
    print(link.wait_for("$ADC") or "no $ADC")
    a = accuracy(link)
    b = link_test(link, float(sys.argv[2]) if len(sys.argv) > 2 else 20)
    link.send("!SYNTH,0")
    link.send("!SMASK,0")
    link.drain(0.5)
    print("PASS" if (a and b) else "FAIL")
    return 0 if (a and b) else 1


if __name__ == "__main__":
    sys.exit(main())
