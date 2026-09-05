#!/usr/bin/env python3
"""
test_gui_protocol.py -- feed microdmm_gui's parser the exact lines the
firmware emits, with no hardware attached.

The GUI and SerialCmd.ino have to agree on field order and field COUNT for
every $ record.  Nothing enforces that at build time, so a firmware edit that
adds a field to $LIVE, or a $CAL form with a different width, would otherwise
show up as a silently blank readout on the bench.  The sample lines below are
transcribed from the emitters in SerialCmd.ino; update them together.

    py test_gui_protocol.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import microdmm_gui as G

app = G.App()
app.withdraw()                      # no visible window during the test
fails = []

def chk(label, got, want):
    if got != want:
        fails.append(f"{label}: got {got!r} want {want!r}")

# --- $LIVE, exactly the field order emitLive() prints -------------------
# ms,R,Rz,V,Vavg,Vac,I,Vr,Vbat,mode,flags   flags: voltageDisplay|autoRange
app._handle_line("$LIVE,12345,4700.125,4699.875,3.300000,3.299000,0.0021,0.0000,2.50000,4.980,1,129")
chk("primary", app.primary_lbl.cget("text"), "3.3 V")
chk("caption", app.primary_cap.cget("text"), "DC volts")
chk("secondary R", app.sec_lbls["Resistance"].cget("text"), "4.7 kohm")
chk("mode label", app.mode_lbl.cget("text"), "mode 1 Voltmeter")
chk("trace len", len(app.trace_t), 1)

# resistance display, R open, config dirty  -> bits 7,13,15 = 41088
app._handle_line("$LIVE,12545,8000000.000,8000000.000,0.000100,0.000050,0.0001,0.0000,4.99900,4.980,0,41088")
chk("open readout", app.primary_lbl.cget("text"), "OPEN")
chk("dirty banner", app.dirty_lbl.cget("text"), "UNSAVED CONFIG")
chk("OPEN lamp lit", app.lamps["OPEN"][0].cget("bg"), "#7a7a7a")

# AC present (bits 0,2,7) = 133
app._handle_line("$LIVE,12745,8000000.000,8000000.000,0.010000,0.000100,120.4000,0.0000,4.99900,4.980,2,133")
chk("ac caption", app.primary_cap.cget("text"), "AC rms")

# --- $MINMAX -----------------------------------------------------------
app._handle_line("$MINMAX,-0.001200,12.400000,0.500,8000000.000,0.0000,1.2500,00:04,01:12")
if "12.4 V" not in app.mm_lbl.cget("text"):
    fails.append(f"minmax text: {app.mm_lbl.cget('text')!r}")

# --- $STATUS -----------------------------------------------------------
app._handle_line("$STATUS,mode=4,range=high,auto=1,ps=0,zero=0.0000,bridge=1,"
                 "stream=1,debug=0,amps=1,irange=high,dirty=0,hwrev=6,sn=20260905_001")
chk("sn label", app.sn_lbl.cget("text"), "SN 20260905_001")
chk("hwrev", app.hwrev, "6")
chk("stream checkbox", app.stream_var.get(), True)
chk("mode from status", app.mode_lbl.cget("text"), "mode 4 Precise")

# --- $CFG rows, including one this GUI has never heard of --------------
for line in ["$CFG,HWREV,6", "$CFG,RCAL03,0.997100", "$CFG,VSCALE,-68.426399",
             "$CFG,IZERO,2.500000", "$CFG,KEYBOARD,1", "$CFG,FUTUREKEY,42"]:
    app._handle_line(line)
app._handle_line("$CFGEND")
chk("cal factor mirrored", app.cal_rows[3]["factor"].cget("text"), "0.997100")
chk("vscale label", app.calv_lbl.cget("text"), "VSCALE -68.426399")
chk("izero label", app.cali_lbl.cget("text"), "IZERO 2.500000")
chk("unknown key present", "FUTUREKEY" in app.cfg_rows, True)
chk("unknown key grouped", app._cfg_group_of("FUTUREKEY"), "Other")
chk("bool editor", str(app.cfg_rows["KEYBOARD"]["var"].get()), "1")

# --- $CAL, all three shapes -------------------------------------------
app._handle_line("$CAL,7,1004.500000,1000.0000,0.995520")
chk("cal rcal07", app.cal_rows[7]["factor"].cget("text"), "0.995520")
app._handle_line("$CAL,VSCALE,3.310000,3.300000,-68.219000")
chk("cal vscale", app.calv_lbl.cget("text"), "VSCALE -68.219000")
app._handle_line("$CAL,IZERO,2.487500")          # only 3 fields
chk("cal izero", app.cali_lbl.cget("text"), "IZERO 2.487500")

# --- log capture -------------------------------------------------------
app._handle_line("$LOGSTART,120,10.500")
for i in range(3):
    app._handle_line(f"$LOG,{i},{i*0.1:.3f},{3.3-i*0.1:.4f},{0.25+i*0.01:.4f}")
app._handle_line("$LOGEND,22.300")
chk("log rows", len(app.log_rows), 3)
chk("log status", app.log_status.cget("text"), "3 samples")

# --- bucket boundaries must agree with rCalIndex() in Config.ino -------
for raw, want in [(0.0, 0), (0.749, 0), (0.75, 1), (2.999, 1), (3.0, 2),
                  (1699999.0, 13), (1700000.0, 14), (9e9, 14)]:
    chk(f"bucket({raw})", G.App._bucket_of(raw), want)

# --- non-$ lines must not crash the parser ----------------------------
app._handle_line("Setup Start")
app._handle_line("$ERR,set,unknown key NOPE")
app._handle_line("$PS,armed")
app._handle_line("$INFO,boot,ready")

app.destroy()
if fails:
    print("FAIL")
    for f in fails:
        print("  -", f)
    sys.exit(1)
print("ALL GUI PARSER CHECKS PASSED")
