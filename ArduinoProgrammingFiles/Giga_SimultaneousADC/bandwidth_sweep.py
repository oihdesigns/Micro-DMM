"""Bandwidth sweep: drives the Rigol DG852 Pro CH1 (SCPI 192.168.10.3:5025) from 1 kHz to 50 MHz and
records the Giga's per-channel amplitude (sqrt(2) x block sd, valid above Nyquist because of the scan
jitter). Wiring: CH1 -> A0 and A3, A1/A2 -> the Giga's DACs. Usage: py bandwidth_sweep.py [SIM,DIFF] [smp]
Restores CH1 to 10 kHz / 2 Vpp / 1 V offset afterwards; edit the last :APPL line if yours differs."""
import socket, sys
class Scpi:
    def __init__(s, host="192.168.10.3", port=5025):
        s.k = socket.create_connection((host, port), timeout=3); s.f = s.k.makefile("rb")
    def w(s, c): s.k.sendall((c + "\n").encode())
    def q(s, c): s.w(c); return s.f.readline().decode().strip()
import serial, time, math, json, sys
FREQS = [1e3, 1e4, 1e5, 2e5, 5e5, 1e6, 2e6, 3e6, 5e6, 7e6, 1e7, 1.5e7, 2e7, 2.5e7, 3e7, 4e7, 5e7]
MODES = sys.argv[1].split(",") if len(sys.argv) > 1 else ["SIM", "DIFF"]
SMP = sys.argv[2] if len(sys.argv) > 2 else "3"
g = serial.Serial("COM36", 115200, timeout=0.2); time.sleep(0.3); g.reset_input_buffer()
def cmd(c, until):
    g.write((c + "\n").encode()); buf = ""; t = time.time()
    while time.time() - t < 10:
        buf += g.read(8192).decode(errors="replace")
        if until in buf: time.sleep(0.05); buf += g.read(8192).decode(errors="replace"); return buf
    raise RuntimeError("timeout on " + c + ": " + buf[-300:])
r = Scpi()
r.w(":SOUR1:APPL:SIN 10000,2,1.65,0"); r.w(":OUTP1 ON"); time.sleep(0.5)
cmd("!JIT,5", "$OK"); cmd("!SMP," + SMP, "$OK"); cmd("!DAC,DC,2048", "A13")
out = {}
for m in MODES:
    cmd("!MODE," + m, "$OK")
    rows = []
    for f in FREQS:
        r.w(f":SOUR1:FREQ {f:g}"); time.sleep(0.3)
        txt = cmd("!RUN", "$SKEW")
        ch = {}
        for line in txt.splitlines():
            p = line.split()
            if len(p) >= 6 and p[0] in ("VP", "VN", "IP", "IN", "V", "I"):
                ch[p[0]] = dict(mean=float(p[3]), pp=float(p[4]), sd=float(p[5]))
        rows.append((f, ch)); print(m, f"{f:>10g}", {k: round(v["sd"] * math.sqrt(2), 1) for k, v in ch.items()}, flush=True)
    out[m] = rows
json.dump(out, open(f"sweep_smp{SMP}.json", "w"))
r.w(":SOUR1:APPL:SIN 10000,2,1,0")          # back to the user's original setting
cmd("!JIT,1", "$OK"); cmd("!SMP,3", "$OK")
print("CH1 restored:", r.q(":SOUR1:APPL?"))
