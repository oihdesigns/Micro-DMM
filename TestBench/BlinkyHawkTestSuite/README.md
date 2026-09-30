# BlinkyHawk test suite

The BlinkyHawk bench GUI, plus the relay jig, the Rigol DG852 Pro and the
Siglent scope driven together, so a BlinkyHawk can be put through a list of
inputs automatically. The suite logs both what the board **measured** and what
it **decided**, and can auto-tune the voltage and open/closed detection
settings.

```
py blinkyhawk_testsuite_gui.py
```

Needs `pip install pyserial matplotlib`, and **BlinkyHawk_Unified firmware**
(`ArduinoProgrammingFiles/BlinkyHawk_Unified`, Sep 2026 or later). It needs
`!DETLOG` for USB runs, and `!SLEEPLOG,2` / `!SLEEPLOG,D` for battery tuning.

> **Tune on battery, not over USB.** USB earths the board, and numbers taken
> that way do not hold once it is unplugged. See
> [Tuning on battery](#tuning-on-battery-the-default). Auto-Tune does this by
> default.

| File | What it is |
|---|---|
| `blinkyhawk_testsuite_gui.py` | The GUI. It **subclasses** the unified firmware's GUI (`ArduinoProgrammingFiles/BlinkyHawk_Unified/blinkyhawk_gui.py`) rather than copying it. The Diagnostics / Configuration / Power tabs are the same code, and so is the per-unit CSV (`blinkyhawk_units.csv`) that **Restore this unit…** reads. If that folder is missing, it falls back to the old bench GUI |
| `bh_battery.py` | Battery captures: arms the unit's RAM log over USB, runs the conditions while it is unplugged, reads the log back as passes, and runs the sleep/wake check |
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

The firmware has the production classifier (ported from production
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
shows "sensor OK" with its sample rate (about 80 per second at the default
ATIME 15 / ASTEP 256 = 11.4 ms integration, full scale 4112 counts).

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
- **Too few flashes:** fewer than two flashes is not judged (NO DATA), except
  that a single blue flash counts as FLOAT, since nothing else starts blue.
- **FLOAT accepts "dark":** the float blink is dim and brief, and waiting long
  enough to be sure of catching it would stretch every open-lead condition. So
  an expected FLOAT (and "not voltage") passes on no flashes at all. CLOSED
  and voltage conditions still need to see their colours, so a dead or
  disabled LED still fails the run there.

**Battery mode** (Test Sequence, "unit on battery"): the run doesn't touch the
serial port at all. The LED is the only judge, and the pass/fail table shows
what it saw. With serial up and the watcher on, both must agree for a PASS,
which also tests that what the firmware decides is what the user sees.

What still touches the unit on battery: the test leads themselves. The relay
network floats, but the generator's output return is earthed, so generator
conditions are "measuring an earth-referenced source". That is realistic for
mains and bench supplies, but it isn't a fully floating system.

## Battery levels (Rigol DL3000 electronic load)

The Test Sequence can run the whole plan at several battery voltages, for
example as found (~4.1 V), then 3.9, 3.6 and 3.3 V. Between rounds, the
electronic load drains the battery. It is wired in parallel with the
BlinkyHawk's battery, + to + and − to −. With its input off it is high
impedance, and the unit runs normally.

**Load setup (once):**

1. Utility → Interface → LAN: DHCP **off**, Auto IP **off**, static IP
   **192.168.10.4**, mask 255.255.255.0, gateway 192.168.10.1. A plain DL3021
   or DL3031 needs the LAN option; the A models have LAN built in.
2. **Von Latch OFF.** The suite sends the floor voltage (default 3.0 V) as
   Von, so the load itself stops sinking below it even if the PC or script
   dies mid-drain. That only works with the latch off, and there is no SCPI
   command for it.
3. Test Rig tab → Electronic load → Connect. The port is 5555 on most Rigols.
   The DG852 here turned out to be 5025, so try that if 5555 doesn't answer.

**How a level is reached.** Loaded voltage sits below resting voltage by
I×R, and a cell springs back up for minutes after the load comes off. So the
drain is iterative:

1. Measure R from the step when the load goes on.
2. Drain until (loaded V + I×R) reaches the target.
3. Rest until the voltage stops rising (under 1 mV per 10 s, or the rest
   limit).
4. Measure, and repeat if still above the band.

The load can only take charge out, so levels must go down. The run folder
gets `battery.csv` (V and I every 2 s, the whole run) and `levels.csv`
(rested V before and after each round's tests, mAh drained, pass/fail
counts). The table shows a summary row per level.

**USB stays out.** With USB in, VBUS charges the battery and the drain
fights the charger. There are two ways to judge each level:

- **LED only:** fully automatic; USB is unplugged once at the start. Your
  notes say the AS7343 misses the dim green and blue flashes on V3b, so
  FLOAT and CLOSED are weakly judged this way.
- **Unit's battery log:** the same unplug/replug captures as battery
  Auto-Tune (`bh_battery.py`). At each level you plug USB in to arm the
  capture, unplug for the run, and plug in again to read it. Replugging
  charges the battery for a few seconds, so each level's `after tests`
  voltage shows the effect.

**Check the load's isolation first.** With the load powered, measure
resistance from its − input terminal to the mains earth pin. It should be
open. If it isn't, the load earths the battery exactly the way USB does,
and battery-level results won't represent a floating unit (see the Sep 28
USB-vs-battery findings).

## USB switched by the relay jig (K8)

The jig's spare relay, K8, drives four external relays, one per USB wire
(VBUS, D+, D−, GND). K8 energised means the BlinkyHawk's USB is connected.
With jig firmware **1.2**, every "unplug the USB / plug it back in" step
happens automatically: the battery-log captures and wake checks in
Auto-Tune, and battery-level runs.

- **Every switch is confirmed**, not assumed. The suite waits for the COM port
  to vanish or reappear. If the relay clicks but the port doesn't change
  within 20 s, the run stops with a message pointing at the K8 wiring. A relay
  that isn't actually in the cable can't pass for an unplug.
- **Put back afterwards:** at the end of any job, including a stopped or
  failed one, the USB goes back to how the job found it, and the serial port
  is reopened if it came back.
- **Test Rig → relay jig:** Connect USB / Disconnect USB buttons, the live
  state, and "switch it automatically in battery runs" (on by default; untick
  it to go back to being told to unplug by hand).
- **Jig firmware:** `!USB[,0|1]` → `$USB,<0|1>`, and `$STATE` gains a trailing
  USB field. K8 is kept out of the load selector: mode changes and `!ALL`
  never move it, and `!K,8,x` is treated as `!USB`. It boots **connected**
  (`USB_BOOT_CONNECTED`), so the rig behaves like an ordinary cable until a
  job unplugs it. If your USB relays connect with K8 released instead, set
  `RELAY_NO_SWAPPED[K8]` rather than changing the logic.
- **A jig reset drops the USB briefly:** the pins float while the UNO R4
  resets. Connect the relay jig before the BlinkyHawk.

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

## Tuning on battery (the default)

**Why.** In September 2026 a V3b tuned over USB looked perfect on the bench.
On battery, a shorted lead read OPEN on every pass: the unit went to sleep with
its leads shorted and never woke. The measurements behind that:

| | USB | Battery, floating |
|---|---|---|
| resting differential | 0.000 V | −0.026 V (−0.036 with a scope ground clip on) |
| SHORT, tail area | 0.076 V·ms | 0.165 V·ms |
| OPEN, tail area | 0.137 V·ms | 0.353 V·ms |
| time-to-return, any load | 0.74–1.14 ms | timeout, every load |

USB earths the board through the PC, and that changes both the resting level
and the shape of the recovery tail. There is a second problem too. The Rigol's
output ground is earth, so on an earthed board the generator is a ground loop,
not a floating source. USB voltage sweeps came out lopsided (a +0.8 V swing
moved the reading a third as much as −0.8 V) and noisy (40 mV pass-to-pass).
A scope ground clip on the board earths it just as USB does.

The product lives on battery, floating, and alerts are locked out on USB
anyway (`CHGINHIBIT=1`). So that is the only condition worth tuning in.

**How.** Tick **measure ON BATTERY** on the Auto-Tune tab (it is on by default).
You need:

- the relay jig on the leads and the generator connected
- **every scope probe off the unit**
- USB connected at the start, and the battery fitted

Nothing can be printed while USB is unplugged, so the unit logs to RAM and the
log is read back afterwards. Each **capture** goes like this:

1. over USB, the suite arms the log. That sets `SLEEPSEC 0`, the log
   `SLEEPTICKS` spacing, `CHGINHIBIT 1`, `VMODE`, and `!SLEEPLOG,2`.
2. a red banner says **UNPLUG USB**. The suite waits for the COM port to
   vanish.
3. a marker, then every condition, run automatically by the jig and generator
4. the banner says **PLUG USB BACK IN**. The GUI reconnects and reads
   `!SLEEPLOG,D`, and the settings are put back.

Each log entry is one real detection pass, with the same fields as `$DET`.
The log has 192 entries, one every few hundred ms. **Log entries/condition**
sets the spacing. A plan that doesn't fit one log is split into several
captures, and the log says how many. Entries are placed on the timeline by a
marker:
- **Voltage captures** use a +2 V step from the generator.
- **Open/closed captures** use a SHORT → OPEN step in the metric. Voltage
  detection is off for these, so no voltage marker is possible.

A full battery auto-tune takes about five unplug cycles:

| Capture | Cycles |
|---|---|
| Voltage | 2 (the default DC sweep does not fit one log) |
| Open/closed | 1 (plus 1 more if method 0 is ticked) |
| Verify | 1 |
| Sleep/wake check | 1 |

On battery, only the **first DETBAND** candidate is used. Every extra one would
cost another cycle, and time-to-return is the metric that fails on battery anyway.

**The sleep/wake check** is new, and it tests the part that failed in the
field. The unit really sleeps and reaches the deep stage; `DEEPSEC` is cut to
2 s just for this, which does not change how the deep probe measures. Then:

- the **OPEN-side** load nearest the boundary goes on: it must **not** wake
  the unit
- the **CLOSED-side** load nearest it goes on: it **must** wake it

`millis()` is frozen in Standby, so the probes are counted, not timed. A wake
on the wrong load shows up as too few probes before the wake. In battery mode
the tuned threshold is written to `SLEEPTHRxx` as well as `THRESHxx`. The
sleeping probe reads about 0.01 V·ms higher than the awake loop on V3b, and
this check is what proves the margin holds.

**If a capture is interrupted** (Stop, a timeout, a crash), the unit can be
left holding the capture settings in RAM. `SLEEPSEC 0` means it never sleeps.
Plug it in and press **Reload EEPROM → RAM**, or power-cycle it. None of it is
ever saved to EEPROM.

**Two gotchas:**

- **CHGINHIBIT and a 5 V supply.** A unit powered through its 5 V input (a
  bench supply or a boost converter) never looks unplugged, so its log never
  starts. Battery mode needs the real battery.
- **The LED watcher can't see dim flashes.** The AS7343 does not see the dim
  green and blue flashes at the default brightness. The sleep/wake check
  therefore judges by the unit's own log, not by flashes.

Battery tuning of the first V3b gave:

| Key | Value |
|---|---|
| `REFCENTER` | −0.026 |
| `REFBAND` | 0.049 (trip at ±0.6 V) |
| `VOLTFAST` | 1.5 |
| `ACBAND` | 0.05 |
| `DETMETHOD` | 2 (tail area) |
| `THRESH11` / `SLEEPTHR11` | 0.28 |

Areas: 1M 0.237, 10.6M 0.329. 10.6M must read OPEN, because the DMM's own
10 MΩ is always across the leads. The auto-tuner reproduces `REFCENTER` /
`REFBAND` from that run's log (−0.0256 / 0.0486). It never *lowers*
`VOLTFAST`: 1.5 was chosen by hand to catch AC above about 1 Vrms.

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
