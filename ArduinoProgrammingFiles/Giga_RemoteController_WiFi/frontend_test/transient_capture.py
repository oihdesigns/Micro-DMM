"""
Transient capture set for the transient report (transient_report.py).

The Rigol CH1 makes repeating transients into the 12 V board input; the board
captures them with its normal PINS: command. Everything is saved raw so the
report can be rebuilt or re-analysed without the hardware:

  runs/<stamp>_transients/transients.json

  pulses     0 V -> A rectangular pulses, widths 0.5 us .. 1 ms, A = 5 mV .. 1 V,
             at 500 kS/s raw (2 us/sample) and 500 kS/s SMOOTH 8 (16 us/sample)
  bursts     3-cycle sine bursts, 4 Vpp, 2 kHz .. 300 kHz
  settings   one transient (20 us, 300 mV) at six rate / SMOOTH settings

The Rigol settings are read first and restored at the end.

  py transient_capture.py --board COM32 --label usb
"""
import argparse, datetime, json, os, time
from frontend_characterize import Board, Rigol, FGEN, HERE

WIDTHS = [0.5e-6, 1e-6, 2e-6, 5e-6, 10e-6, 20e-6, 50e-6, 100e-6, 200e-6, 500e-6, 1e-3]
AMPS = [0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0]
RAW_AMPS = [0.1, 0.3, 1.0]
BURST_F = [2e3, 10e3, 30e3, 100e3, 300e3]
BURST_VPP, BURST_N = 4.0, 3

# (rate, SMOOTH, TIME ms): each gives up to 10000 logged points with no decimation
R1 = (500000, 1, 25)
R8 = (500000, 8, 165)
SETTINGS = [(500000, 1, 25), (500000, 8, 165), (500000, 16, 330), (100000, 1, 105), (31250, 1, 330), (10000, 1, 1050)]

def period_for(w):
    return max(2e-3, 10 * w)

def set_pulse(fg, w, amp):
    per = period_for(w)
    fg.w(":SOUR1:BURS:STAT OFF"); fg.dc(0.0)
    for _ in range(3):
        fg.w(":SOUR1:FUNC:PULS:WIDT 1e-6")                 # keep the width legal while the period changes
        fg.w(f":SOUR1:APPL:PULS {1/per:.6f},{amp},{amp/2}")
        fg.w(":SOUR1:VOLT:LOW 0"); fg.w(f":SOUR1:VOLT:HIGH {amp}")
        fg.w(f":SOUR1:FUNC:PULS:WIDT {w:.9g}"); fg.sync()
        got = (fg.q(":SOUR1:APPL?").strip('"').split(",")[0], float(fg.q(":SOUR1:FUNC:PULS:WIDT?")),
               float(fg.q(":SOUR1:VOLT:HIGH?")), float(fg.q(":SOUR1:VOLT:LOW?")), float(fg.q(":SOUR1:FREQ?")))
        if got[0] == "PULS" and abs(got[1] - w) < 1e-3 * w and abs(got[2] - amp) < 1e-3 * amp + 1e-5 \
                and abs(got[3]) < 1e-5 and abs(got[4] - 1 / per) < 1e-6 / per:
            return {"kind": "pulse", "width": w, "amp": amp, "period": per, "edge": 3e-9}
    raise RuntimeError(f"Rigol did not take pulse w={w} A={amp}: {got}")

def set_burst(fg, f):
    # DG852 Pro: the internal burst trigger is ":TRIG1:SOUR IMM" (":SOUR1:BURS:TRIG:SOUR" doesn't
    # exist, and with the source left at BUS/EXT no bursts come out at all).
    want = max(2e-3, 4 * BURST_N / f)
    fg.w(":SOUR1:BURS:STAT OFF"); fg.sine(f, BURST_VPP)
    for c in (":SOUR1:BURS:MODE TRIG", f":SOUR1:BURS:NCYC {BURST_N}", ":TRIG1:SOUR IMM",
              f":SOUR1:BURS:INT:PER {want}", ":SOUR1:BURS:PHAS 0", ":SOUR1:BURS:STAT ON"):
        fg.w(c)
    fg.sync()
    per = float(fg.q(":SOUR1:BURS:INT:PER?"))
    if fg.q(":SOUR1:BURS:STAT?") != "1" or int(float(fg.q(":SOUR1:BURS:NCYC?"))) != BURST_N             or fg.q(":TRIG1:SOUR?") != "IMM" or abs(per - want) > 1e-6:
        raise RuntimeError("Rigol burst setup failed")
    return {"kind": "burst", "freq": f, "vpk": BURST_VPP / 2, "ncyc": BURST_N, "period": per}

def grab(board, setting, truth, tag, out):
    rate, sm, ms = setting
    c = board.capture(rate, ms, smooth=sm)
    out.append({"tag": tag, "truth": truth, "rate": c["fs"], "smooth": sm, "time_ms": ms,
                "info": c["info"], "A2": c["data"]["A2"], "A3": c["data"]["A3"]})
    print(f"  {tag:10s} {truth.get('kind')} {truth.get('width', truth.get('freq'))} "
          f"{truth.get('amp', truth.get('vpk'))}  {rate/1e3:.1f}k S{sm}  n={len(c['data']['A2'])} drop={c['info'].get('DROP')}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", required=True); ap.add_argument("--label", default="usb")
    ap.add_argument("--note", default="")
    ap.add_argument("--only", choices=["bursts", "noise"], help="re-capture just this part ...")
    ap.add_argument("--update", help="... into this existing run folder (replaces those captures)")
    a = ap.parse_args()
    fg = Rigol(*FGEN)
    saved = {"appl": fg.q(":SOUR1:APPL?").strip('"'), "out": fg.q(":OUTP1?"), "burst": fg.q(":SOUR1:BURS:STAT?"),
             "trig": fg.q(":TRIG1:SOUR?")}
    print("Rigol CH1 was:", saved)
    board = Board(a.board)
    if a.update:
        out_dir = a.update; old = json.load(open(os.path.join(out_dir, "transients.json")))
        caps = [c for c in old["captures"] if not c["tag"].startswith(a.only)]
    else:
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M")
        out_dir = os.path.join(HERE, "runs", f"{stamp}_transients_{a.label}"); os.makedirs(out_dir, exist_ok=True)
        caps = []
    try:
        fg.w(":OUTP1 ON")
        fg.dc(0.0); time.sleep(0.5)
        for w in ([] if a.only else WIDTHS):
            for amp in AMPS:
                t = set_pulse(fg, w, amp); time.sleep(0.25)
                grab(board, R8, t, "pulse_r8", caps)
                if amp in RAW_AMPS:
                    grab(board, R1, t, "pulse_r1", caps)
        if a.only in (None, "noise"):
            # the generator must be back at 0 V: the last pulse setting is still running otherwise
            fg.w(":SOUR1:BURS:STAT OFF"); fg.dc(0.0); time.sleep(0.5)
            grab(board, R1, {"kind": "zero"}, "noise_r1", caps); grab(board, R8, {"kind": "zero"}, "noise_r8", caps)
        if not a.only:
            t = set_pulse(fg, 20e-6, 0.3); time.sleep(0.25)
            for st in SETTINGS:
                grab(board, st, t, "settings", caps)
        for f in (BURST_F if a.only in (None, "bursts") else []):
            t = set_burst(fg, f); time.sleep(0.3)
            grab(board, R1, t, "burst_r1", caps); grab(board, R8, t, "burst_r8", caps)
    finally:
        fg.w(":SOUR1:BURS:STAT OFF"); fg.dc(0.0)
        k, *v = saved["appl"].split(",")
        if k == "DC":
            fg.w(f":SOUR1:APPL:DC 1,1,{float(v[2])}")
        else:
            fg.w(f":SOUR1:APPL:{k[:3]} {float(v[0])},{float(v[1])},{float(v[2])},{float(v[3])}")
        fg.w(f":TRIG1:SOUR {saved['trig']}")
        if saved["burst"] == "1": fg.w(":SOUR1:BURS:STAT ON")
        fg.w(f":OUTP1 {'ON' if saved['out'] == '1' else 'OFF'}"); fg.sync()
        print("Rigol CH1 restored:", fg.q(":SOUR1:APPL?"), "burst", fg.q(":SOUR1:BURS:STAT?"), "trig", fg.q(":TRIG1:SOUR?"))
        if a.update:
            a.label, a.note = old["label"], old["note"]
        json.dump({"label": a.label, "note": a.note, "board": a.board,
                   "date": datetime.datetime.now().isoformat(timespec="seconds"), "captures": caps},
                  open(os.path.join(out_dir, "transients.json"), "w"))
        print("saved", out_dir, len(caps), "captures")

if __name__ == "__main__":
    main()
