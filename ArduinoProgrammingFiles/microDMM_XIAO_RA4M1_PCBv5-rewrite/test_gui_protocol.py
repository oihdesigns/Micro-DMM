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
# ms,R,Rz,V,Vavg,Vac,I,Vr,Vbat,mode,flags
# flags: voltageDisplay | autoRange | Rmeasured  (mode 0, so R is measured)
app._handle_line("$LIVE,12345,4700.125,4699.875,3.300000,3.299000,0.0021,0.0000,2.50000,4.980,0,65665")
chk("primary", app.primary_lbl.cget("text"), "3.3 V")
chk("caption", app.primary_cap.cget("text"), "DC volts")
chk("secondary R", app.sec_lbls["Resistance"].cget("text"), "4.7 kohm")
chk("mode label", app.mode_lbl.cget("text"), "mode 0 Default")
chk("trace len", len(app.trace_t), 1)

# resistance display, R open, config dirty, R measured -> 106624
app._handle_line("$LIVE,12545,8000000.000,8000000.000,0.000100,0.000050,0.0001,0.0000,4.99900,4.980,0,106624")
chk("open readout", app.primary_lbl.cget("text"), "OPEN")
chk("dirty banner", app.dirty_lbl.cget("text"), "UNSAVED CONFIG")
chk("OPEN lamp lit", app.lamps["OPEN"][0].cget("bg"), "#7a7a7a")

# AC present (bits 0,2,7,16) = 65669
app._handle_line("$LIVE,12745,8000000.000,8000000.000,0.010000,0.000100,120.4000,0.0000,4.99900,4.980,2,65669")
chk("ac caption", app.primary_cap.cget("text"), "AC rms")

# --- ammeter suppression ----------------------------------------------
# No sensor (bit 14 clear) while amps mode IS on (bit 12): the current channel
# must read "no sensor", and amps mode must NOT take over the primary readout.
app._handle_line("$LIVE,13000,4700.000,4700.000,0.001000,0.001000,0.0010,0.0000,2.50000,4.980,0,69760")
chk("suppressed current", app.sec_lbls["Current"].cget("text"), "no sensor")
chk("suppressed is grey", app.sec_lbls["Current"].cget("fg"), "#aaaaaa")
chk("amps did not take primary", app.primary_cap.cget("text"), "resistance (nulled)")
chk("sensor lamp dark", app.lamps["I SENSOR"][0].cget("bg"), "#eeeeee")

# Sensor present (bits 12 and 14) -> amps becomes the primary readout again.
app._handle_line("$LIVE,13200,4700.000,4700.000,0.001000,0.001000,0.0010,1.2500,2.50000,4.980,0,86144")
chk("amps primary", app.primary_cap.cget("text"), "current")
chk("amps value", app.primary_lbl.cget("text"), "1.25 A")
chk("current shown", app.sec_lbls["Current"].cget("text"), "1.25 A")
chk("sensor lamp lit", app.lamps["I SENSOR"][0].cget("bg"), "#1faa3f")

# --- $IDET, all three outcomes ----------------------------------------
app._handle_line("$IDET,off,-1834,-0.3439,0.2118,0,0,0,0.0000")
if "nothing fitted" not in app.idet_lbl.cget("text"):
    fails.append(f"idet off: {app.idet_lbl.cget('text')!r}")
chk("idet off is red", app.idet_lbl.cget("fg"), "#c02020")
app._handle_line("$IDET,high,13333,2.4999,0.0037,0,1,1,2.4999")
if "hall sensor" not in app.idet_lbl.cget("text"):
    fails.append(f"idet high: {app.idet_lbl.cget('text')!r}")
chk("idet izero mirrored", app.cali_lbl.cget("text"), "IZERO 2.4999")
app._handle_line("$IDET,low,12,0.0022,0.0041,1,0,1,0.0000")
if "shunt" not in app.idet_lbl.cget("text"):
    fails.append(f"idet low: {app.idet_lbl.cget('text')!r}")

# --- voltmeter mode: resistance not measured ---------------------------
# bits 0 (voltageDisplay), 7 (auto), 14 (sensor), 17 (continuous) = 147585.
# Bit 16 CLEAR: the resistance fields are leftovers, not measurements.
n0 = len(app.trace_t)
app.trace_var.set("Resistance (ohm)")
app._handle_line("$LIVE,14000,4700.000,4700.000,12.000000,12.000000,0.0100,0.0000,2.50000,4.980,1,147585")
chk("stale R hidden", app.sec_lbls["Resistance"].cget("text"), "not measured")
chk("stale nulled hidden", app.sec_lbls["Nulled"].cget("text"), "not measured")
chk("stale rail hidden", app.sec_lbls["Ohms rail"].cget("text"), "not measured")
chk("voltage still live", app.sec_lbls["Voltage"].cget("text"), "12 V")
chk("R trace not extended", len(app.trace_t), n0)
chk("continuous lamp lit", app.lamps["CONT ADC"][0].cget("bg"), "#0b6fb8")

# Same record with bit 16 SET and 17 clear -> R live, single-shot again.
# (measured R and continuous are mutually exclusive: continuous needs the ohms
#  channel out of the pass.)
app._handle_line("$LIVE,14200,4700.000,4700.000,12.000000,12.000000,0.0100,0.0000,2.50000,4.980,1,82049")
chk("live R shown", app.sec_lbls["Resistance"].cget("text"), "4.7 kohm")
chk("R trace extended", len(app.trace_t), n0 + 1)
app.trace_var.set("Voltage (V)")

# Resistance primary with bit 16 clear must not present a leftover as a
# reading: flags 128 (auto range only), voltageDisplay clear, R not measured.
app._handle_line("$LIVE,14400,4700.000,4700.000,0.001000,0.001000,0.0010,0.0000,2.50000,4.980,0,128")
chk("no stale primary", app.primary_lbl.cget("text"), "--")
chk("stale primary caption", app.primary_cap.cget("text"),
    "resistance not measured in this mode")

# --- ohms source parked (bit 18) ---------------------------------------
# Voltmeter mode: R not measured, ADC free-running, 20 mA source off.
# bits 0,7,14,17,18 = 1+128+16384+131072+262144 = 409729
app._handle_line("$LIVE,14600,4700.000,4700.000,12.000000,12.000000,0.0100,0.0000,2.50000,4.980,1,409729")
chk("source-off lamp lit", app.lamps["SRC OFF"][0].cget("bg"), "#8a4bbd")
chk("pwrsave lamp dark", app.lamps["PWRSAVE"][0].cget("bg"), "#eeeeee")

# Measuring resistance again: source back on, timeout not fired.
# bits 7,16 = 128+65536 = 65664
app._handle_line("$LIVE,14800,4700.000,4700.000,0.001000,0.001000,0.0010,0.0000,2.50000,4.980,0,65664")
chk("source-off lamp dark", app.lamps["SRC OFF"][0].cget("bg"), "#eeeeee")

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
