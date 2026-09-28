# BlinkyHawk test suite

The BlinkyHawk bench GUI, plus the relay jig, the Rigol DG852 Pro and the
Siglent scope driven together, so a BlinkyHawk can be put through a list of
inputs automatically. The suite logs both what the board **measured** and what
it **decided**, and can auto-tune the voltage and open/closed detection
settings.

```
py blinkyhawk_testsuite_gui.py
```

Needs `pip install pyserial matplotlib`, and the **bench firmware with
`!DETLOG`** (`ArduinoProgrammingFiles/BlinkyHawk_Bench`, same commit as this).

| File | What it is |
|---|---|
| `blinkyhawk_testsuite_gui.py` | The GUI. It **subclasses** `blinkyhawk_bench_gui.App` (imported from `ArduinoProgrammingFiles/BlinkyHawk_Bench`) rather than copying it, so the Diagnostics / Configuration / Bench tabs are the same code, and so is the per-unit CSV (`blinkyhawk_bench_units.csv`) that **Restore this unit…** reads |
| `bh_instruments.py` | Relay jig (USB), Rigol DG800 Pro (SCPI socket), Siglent SDS800X HD (a Python port of `../Scope/ScopeLib.ps1`), and a writer for Capture-Scope's CSV layout |
| `bh_sequencer.py` | Condition runner, the big-test job and the auto-tune job, and the run-folder writer |
| `bh_tuner.py` | The analysis. Pure functions with no I/O |
| `bh_led.py` | Turns the relay jig's AS7343 flash reports into the alert the unit is showing |
| `bh_sim.py` | A simulated BlinkyHawk and bench, for dry runs |

## The three new tabs

**Test Rig**: connect and poke each instrument by hand. Type `SIM` in any
port or IP box to simulate that instrument; **Simulate everything** does all
four, including the BlinkyHawk. The simulator's numbers are invented. Use it
to check the flow, not to learn anything about the hardware.

**Test Sequence** is the big test. For each selected condition (relay loads,
generator DC levels, a generator sine, and the generator path with its output
off) the runner:

1. turns the generator off (it never switches relays under drive)
2. moves the relay jig
3. programs the generator and turns it on
4. settles
5. records every detection pass from the BlinkyHawk
6. optionally grabs or single-triggers the scope
7. turns the generator off again

The table shows the debounced lead state as CLOSED/OPEN/VOLT % against the
pass/fail criteria.

**Auto-Tune**: see below.

If the relay jig isn't connected, a run pops up a box asking you to move the
leads by hand, then carries on. Conditions that need the generator are
skipped, and marked as skipped, when it isn't connected.

## Voltage kind: VDC+, VDC−, VAC

The bench firmware now has the production classifier (ported from production
commit `4f4fecf`; bench config v5, keys `VCLASS`, `ACWINMS`, `ACBAND`,
`VOLTLONGMS`, `VACPULSES`). The expectations follow it:

- DC more than 10 % above the trip voltage must read **VDC+** or **VDC−**
  according to its sign.
- The sine must read **VAC**.

A voltage condition passes only if the unit says voltage **and** names the
right kind. The table's `kind` column is the debounced kind while the lead
state is VOLTAGE. `$DET` carries the raw and debounced kind and the two peaks
(`vpos`/`vneg`) the decision was made on.

## LED watcher and battery mode

A scope probe on the BlinkyHawk would clip its ground to earth, and that
defeats the point of testing it on its own battery. Instead, an **Adafruit
AS7343** held over the SK6812 watches the alert the unit is actually giving.
It has no electrical contact with the unit and takes no current from it.

**Wiring:** STEMMA QT / I2C to the relay jig's UNO R4: VIN to 5 V (the Adafruit
board regulates and level-shifts), GND, SDA, SCL on the header pins. On a
Qwiic cable to a board that has the connector, change `LED_WIRE` to `Wire1` in
`RelayMountTestJig.ino`. **Flash jig firmware 1.1.** The Test Rig tab then
shows "sensor OK" with its sample rate (about 200 per second at the default
2.8 ms integration).

**Aim and shield it.** Tape or a short tube over the LED keeps room light out.
Tick "raw counts" and check that the LED lifts the counts well clear of the
dark level without reaching full scale. A flash marked SATURATED means lower
the gain. The flash threshold is in counts above the tracked dark level
(default 20).

**Calibrate once per aim**, with the unit tuned and alerting normally:
**Auto-calibrate** runs OPEN (float = blue), SHORT (closed = green) and a DC
level (VDC+ = red), and learns each colour's FZ/FY/FXL signature. It is saved
in `led_calibration.json`. If the serial link is up, a condition the unit
isn't actually alerting is skipped rather than learned. Until you calibrate,
rough default colours are used.

**How the alert is read** (from the firmware's own patterns):

| Flashes seen | Alert |
|---|---|
| blue | FLOAT |
| green | CLOSED |
| red, red… | VDC+ |
| red, blue, red, blue… | VDC− |
| red, blue, green… | VAC |

- **Order is checked, not just colour:** a VDC+ alert followed by a blue float
  blink is also "red + blue", and would otherwise read as VDC−.
- **Mixed windows:** a window that is still showing the previous condition's
  tail has leading flashes dropped (up to half), and the table says so. If it
  still fits no alert, it reads `?`.
- **Too few flashes:** fewer than two flashes is not judged (NO DATA), so give
  the LED **≥ 2.5 s of dwell**, because the float blink is about once a
  second.

**Battery mode** (Test Sequence, "unit on battery"): the run doesn't touch the
serial port at all. The LED is the only judge, and the pass/fail table shows
what it saw. With serial up and the watcher on, both must agree for a PASS,
which also tests that what the firmware decides is what the user sees.

What still touches the unit on battery: the test leads themselves. The relay
network floats, but the generator's output return is earthed, so generator
conditions are "measuring an earth-referenced source". That is realistic for
mains and bench supplies, but it isn't a fully floating system.

## Run folders (`Runs/<date>_<kind>_<note>/`)

| File | Contents |
|---|---|
| `run_info.txt` | instruments, timing, the condition list with expectations |
| `config_before.csv` | the BlinkyHawk's full `!CFG` at the start |
| `passes.csv` | every `$DET` pass: rest mean/min/max, metric, time-to-return, area, raw and debounced state, voltage kind and its peaks |
| `flashes.csv` | every LED flash the AS7343 saw: timing, FZ/FY/FXL counts, decoded colour and how close it was |
| `summary.csv` | one row per condition, pass/fail, scope stats |
| `scope.csv` + `scope.html` | **Capture-Scope.ps1's layout**, so `..\Scope\Plot-Capture.ps1` renders it. The suite runs it for you; `scope.html` is the plot |
| `tuning_report.txt` | auto-tune only: the whole reasoning, with verdicts |

## Auto-tune: what it does and how to read it

The order is fixed because it has to be: the open/closed metric measures
`|diff − REFCENTER|`, so moving REFCENTER moves the metric.

**1. Voltage (REFCENTER, REFBAND, VOLTFAST).** The generator sweeps DC levels
(default −1.2…+1.2 V in 0.1 V steps) while each relay load is also recorded.
This gives the front end's transfer curve, input volts → resting
differential. For a target trip voltage Vt:

```
REFCENTER = (d(+Vt) + d(−Vt)) / 2        REFBAND = |d(+Vt) − d(−Vt)| / 2
```

That puts the averaged decision's boundary at exactly **+Vt and −Vt**, even
when the two polarities have different gain. The suite doesn't have to re-run
the bench for each candidate. With `!DETLOG` on, every pass reports the
mean/min/max of all VOLTAVG resting reads, and that is enough to replay the
firmware's `voltagePresent()` exactly for any centre, band and VOLTFAST. Each
relay load is replayed with the proposed values to look for false positives,
and the report gives:

- the false-voltage rate per load and the worst deviation as a % of the band
  (the guard, default 1.25×, wants ≤ 80 %)
- a raised VOLTFAST, if single noisy reads would otherwise trip the fast path
- **the smallest trip voltage that stays clean**, which answers "what can be
  cleanly discriminated" for voltage
- detection rate at every swept level (how sharp the trip is), and the level
  from which detection is 100 %
- the sine: % of passes that see it, and % of time the debounced lead state
  holds VOLTAGE. A pass whose ten reads land near a zero crossing misses; the
  report says so.

**2. Open/closed (DETMETHOD, DETBAND, THRESHxx).** With voltage detection
disabled (`!VMODE,2`, so every pass runs the MOSFET test), each relay load is
recorded under method 2, which computes both time-to-return and tail area in
one pass. That repeats for each DETBAND candidate, plus method 0 if ticked.
For every candidate metric:

- per-load mean/sd/range
- whether the medians follow resistance order
- **clean cut points**: every adjacent pair of loads whose ranges don't
  overlap, which is the "what values can be discriminated" answer
- if CLOSED ≤ 150k and OPEN ≥ 1M separate: a threshold weighted by the two
  nearest loads' spreads, with the margin in sigma
- if they don't: **it says so**, with the overlap and the best achievable
  per-pass error rate

The clean candidate with the largest sigma margin wins. Its method, DETBAND
and threshold go into the **active** slot (`THRESHSEL` on HWREV 3, the DIP
position on HWREV 2).

**3. Verify.** It applies everything and runs the loads, DC at ±1.15·Vt
(expect VOLTAGE) and ±0.85·Vt (expect not VOLTAGE), your extra DC levels, the
sine, and the generator off, all through the real firmware with the real
debounce. Pass/fail goes in the table.

**Nothing is saved.** Results are left in device RAM. Use **Save to EEPROM**
(which also logs the unit's row by SN) or **Revert to before**. **Stop**
mid-run reverts everything the run had changed.

What it doesn't tune: SLEEPTHRxx (the sleeping probe's wake thresholds), the
other three THRESH slots, or VOLTAVG and STABLECOUNT. Those are yours.

## Generator notes (Rigol DG852 Pro)

- **Connecting forces HighZ.** The Rigol defaults to a 50 Ω load, and at that
  setting it scales its output to hit the programmed level *into 50 Ω*. Into
  the BlinkyHawk's high-impedance input that is **twice** the voltage asked
  for. `setup()` sets `:OUTP1:LOAD INF` and refuses to continue if the query
  doesn't come back as HighZ.
- Sine levels are entered in **Vrms** and sent as Vpp (5 Vrms = 14.14 Vpp; the
  HighZ limit is 20 Vpp). The safety limit on the Test Rig tab (default 8 V
  peak) refuses anything above it.
- Network: static IP **192.168.10.3**, mask 255.255.255.0, gateway
  192.168.10.1, with DHCP **and** Auto IP turned off on the instrument (they
  take priority over a static IP). The PC is 192.168.10.1 and the scope .2.
- SCPI port is **5025**. The instrument reports it (`:SYST:COMM:LAN:CONT?`),
  and nothing listens on 5555 or 5000, even though the programming guide's
  example says 5000.
- Checked on the bench unit (DG852 Pro, fw 00.01.00.00.22, 2026-09-27) with
  the output off: connect/setup, `APPLy:DC DEF,DEF,±0.8` and
  `APPLy:SINusoid 60,14.1421,0,0` all read back correctly with no SCPI
  errors, and the safety limit refuses 6 Vrms. Output ON has not been driven
  into a BlinkyHawk yet.

## Why the runs use normal mode, not diagnostic mode

In diagnostic mode the firmware loops every ~1 ms. In normal mode it loops at
LOOPMS (50 ms). How long the bridge rests between tests changes the metric, so
tuning in diag mode would tune for a timing the unit never runs at. Runs send
`!DIAG,0`. Sleep is not a risk while the port is open (`lowPowerAllowed()`
returns false whenever `Serial` is up).

## Troubleshooting: the BlinkyHawk won't connect

**It is probably asleep.** With `CHGINHIBIT=0`, USB power no longer keeps the
board awake. Only a program holding its port open does. So 8 s after a reset
or an upload, with the leads reading open, it drops into Software Standby. In
Standby the USB peripheral stops, so Windows still lists the COM port but
nothing answers, and writes block. On this rig open leads are the normal
state: the relay jig rests on FGEN and the generator output is off.

- **Wake it:** close the leads (relay jig **SHORT**, or a jumper) or press
  reset, then connect. Once the port is open it won't sleep again, and the jig
  can go back to FGEN.
- The GUI now opens the port in the background, with a 1 s write timeout. If
  there's no `$STATUS` within 3 s it says so, and if the relay jig is
  connected it offers to switch to SHORT and reconnect for you. (The original
  bench GUI opened and wrote on the GUI thread with no timeout, so a sleeping
  board froze the whole window.)
- **"Access is denied"** means another program has the port: usually the
  Arduino IDE's Serial Monitor, an upload in progress, or a second copy of the
  GUI.
- The port can move to a new COM number after an upload. Both port lists
  refresh when you open the dropdown, and the relay jig has a **Refresh**
  button.

## Troubleshooting: runs but shows nothing, or an instrument won't connect

- **Every on-screen update from a background job goes through one polling
  loop:** result rows, status, and the instrument connect replies. An
  exception there used to stop the loop for good. The run carried on and wrote
  its files, but the window stopped changing. Errors are now caught one
  message at a time, shown in the logs as `!! GUI error ...`, and appended to
  `suite_errors.log` next to the script. Send me that file if you see one.
- **The scope and generator serve one SCPI client at a time.** A second client
  connects, but nothing answers it. The suite now says so ("accepted the
  connection but did not answer *IDN?"). Close whatever else holds the
  session: another copy of this GUI, `Capture-Scope.ps1`, or the instrument's
  web page.
