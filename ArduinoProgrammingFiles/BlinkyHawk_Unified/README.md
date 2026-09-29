# BlinkyHawk_Unified — firmware for every Blinky Hawk board

One sketch for OpenLead_Headless V2, OpenLead_Headless V3 and **BlinkyHawk
V3b**, merged from the two firmwares it replaces:

| Came from | What it brought |
|---|---|
| `BlinkyHawk_RA4M1` (production) | Detection, alerts, voltage kind, sleep, EEPROM config and its v2–v8 migration chain, the host protocol |
| `BlinkyHawk_Bench` | Power gates, two-stage sleep (`DEEPSEC`/`DEEPHZ`/`DEEPPARK`), `CHGINHIBIT`, `!GATE` / `!EXPT` / `!DETLOG`, gate-aware `!CAP` |
| new | V3b support (HWREV 4) and PAM8904 volume control |

## Board revisions

`HWREV` decides what the pins are. It is read before any pin is configured.

| Pin | V2 (2) | V3 (3) | **V3b (4)** |
|---|---|---|---|
| A1 | SENSE_NEG | NC | **VBUS/2 charge sense** |
| A2 | sense node | sense node | sense node |
| A3 | VBUS/2 charge sense | VBUS/2 charge sense | **Analog_Rail enable** (TPS22914, active high) |
| D6 | LED1 data | LED1 data | LED1 data |
| D7 | bridge MOSFET | bridge MOSFET | bridge MOSFET |
| D8 | DIP switch | buzzer BZ1+ | **PAM8904 EN1** |
| D9 | buzzer | buzzer BZ1− | **PAM8904 EN2** |
| D10 | DIP switch | NC (parked low) | **LEDRail enable** (TPS22914, active high) |
| D15 | — | — | **PAM8904 DIN** (XIAO back pad P101 → J7) |

A blank EEPROM defaults to **4**. Stored configs keep their revision: v6–v8
carry it across, and anything older is pinned to 2. `!DEFAULTS` never
changes it.

> **A V3b that has ever run the v8 production firmware stored HWREV 3**, which
> was v8's blank-EEPROM default. It will keep that value. Fix it once with
> `!SET,HWREV,4` then `!SAVE`. `!PINS` or the boot banner shows which map is in
> force.

The pin map is **not** config any more. The bench fork's `PIN*`, `*POL` and
`AUX*` keys are gone. `!PINS` still reports the map (read-only) in the bench's
`$PIN` row format, so the bench GUI and `TestBench/BlinkyHawkTestSuite` still
parse it.

## Volume (V3b)

The PAM8904 takes the tone on DIN and the gain on EN1/EN2. From the
datasheet's mode table:

| EN1 | EN2 | Mode |
|---|---|---|
| 0 | 0 | shutdown |
| 0 | 1 | 1x |
| 1 | 0 | 2x |
| 1 | 1 | 3x |

| Key | Default | Means |
|---|---|---|
| `CONTVOL` | 2 | Continuity beep: 0 = silent, 1–3 = gain step |
| `VOLTVOL` | 3 | Voltage alert: 0 = silent, 1–3 = gain step |
| `SPKDUTY` | 50 | Fine trim below a step, 1–50 % duty on D15. The fundamental scales with sin(π·duty): 25 ≈ −3 dB, 12 ≈ −8 dB, 5 ≈ −16 dB |

- 1x puts roughly the same swing across the element as V3's anti-phase drive
  did. 3x is about +9.5 dB over that.
- D15 is a real GPT output (GTIOC5A), so the tone is hardware PWM. That is why
  the duty trim costs nothing.
- The amp is held in **shutdown** (EN1 = EN2 = 0) whenever no pulse is
  sounding, so volume has no idle cost. Its own auto-standby would take 42 ms
  after each beep, with the charge pump running meanwhile.
- A volume of 0 silences that alert on **every** revision. Levels 1–3 only
  change anything on V3b. `PASSIVE` and `SPKDIFF` do not apply to V3b.

**Setting it by ear:** `!TONE[,<vol>[,<ms>[,<hz>]]]` plays one pulse now
(default `VOLTVOL`, 200 ms, `VOLTFREQ`). It ignores `BEEP`, the boot mute and
the charge lockout, because all three are normally in force when the board
is on USB at the bench. The GUI's Diagnostics tab has a **Speaker test** box
for this.

## Power gates (V3b)

| Key | Default | Means |
|---|---|---|
| `ANAMODE` | 2 | Analog rail. 0 = always on, 1 = off while asleep, 2 = pulsed: up only for each measurement |
| `ANAUS` | 4000 | Settle after raising the analog rail (µs) |
| `LEDGMODE` | 2 | LED1 rail. Same modes; in mode 2 it is up only while a colour shows |
| `LEDGMS` | 2 | Settle after raising the LED rail (ms) |

The defaults are the scheme the bench unit was characterised on. **`ANAUS` was
tuned on a breadboard load switch, not the TPS22914.** Take a `!CAP` on a V3b:
the capture brings the rail up inside the trace (shaded red, then a green
settle band), and the trace must be flat by the end of the green band.

Modes 1 and 2 move the resting differential, so tune `THRESH*` / `SLEEPTHR*`
under the mode you ship. The factory thresholds were tuned on V3 with an
always-on rail.

Gate rules carried over from the bench fork:

- **Every ADC consumer raises the analog gate**: detection, the sleeping
  probe, `!CAP`, `!STREAM`, `!VTEST`, and the boot mute check. The boot mute
  check was the one the bench fork missed.
- **LED1 is never clocked while its rail is down.** Otherwise the data pin
  would back-feed the SK6812 through its input protection. A colour raises the
  rail, and the next dark write drops it. That is how the heartbeat and the
  boot cue work in every mode.

## Config (v9) and migration

Production's block: magic `BHK1` at address 0. v9 is the v8 layout with these
fields **appended** before `crc`: the gate modes and settles, `DEEPSEC` /
`DEEPHZ` / `DEEPPARK`, `CHGINHIBIT`, and `CONTVOL` / `VOLTVOL` / `SPKDUTY`.

- **v8 → v9** is a prefix copy of the frozen `ConfigV8`. `static_assert`s pin
  the layout, and a field-by-field check against production's struct was run
  when this was written.
- v2–v7 use production's migrations unchanged.
- **Any migrated unit starts with `DEEPSEC=0`**, so a field unit's sleep
  behaves exactly as before. Turn the deep stage on deliberately.
- The bench block (`BHKX` at 1024) is never read or written. A unit can still
  go back to the bench firmware and find its bench settings.
- Keep appending future fields before `crc`, and bump the version.

## Other behaviour worth knowing

- `CHARGE` is on A1 on V3b. The boot USB check now runs **after** the config
  load, so it reads the right pin.
- An open host serial port holds the board awake (from the bench fork). Arm
  `!SLEEP`, then close the port or unplug.
- `$STATUS` gains `amp=`. `gates=` / `gforce=` are two characters (ANA, LED),
  not the bench's three. The bench GUI simply stops showing its gate label.

## Tuning: do it on battery

USB earths the board through the PC. That moves the resting differential by
about 26–36 mV and changes the recovery tail. Thresholds tuned over USB read
every shorted lead as OPEN once the unit was unplugged: it slept and never
woke. The Rigol generator is also earth-referenced, so over USB it forms a
ground loop, and a scope ground clip on the board earths it too.

**Tune on battery, with nothing earthed connected.** For that, the sleep log
doubles as a battery-side `$DET`:

| Command | Does |
|---|---|
| `!SLEEPLOG,2` | Clear and arm: every awake pass takes all `VOLTAVG` reads (full rest mean/min/max), and the log **stops when full** instead of wrapping |
| `!SLEEPLOG,D` | Detailed dump: `$SLOGD,<i>,<ms>,<src A/S/D>,<raw>,<lead>,<n>,<mean>,<min>,<max>,<metric>,<retms>,<area>,<thr>,<rawkind>,<kind>,<vpos>,<vneg>` |
| `!SLEEPLOG,0` | Clear, back to normal wrapping |

The log holds 192 entries, which is the most that fits beside the core's
fixed 8 KB heap. Awake entries are recorded only while charge does not
inhibit, i.e. on battery. `TestBench/BlinkyHawkTestSuite` drives all of this:
its Auto-Tune tab measures on battery by default.

Values from the first V3b tuned this way (SN 20260928_001):

| Key | Value |
|---|---|
| `REFCENTER` | −0.026 |
| `REFBAND` | 0.049 (trip at ±0.6 V) |
| `VOLTFAST` | 1.5 |
| `ACBAND` | 0.05 |
| `DETMETHOD` | 2 |
| `THRESH11` / `SLEEPTHR11` | 0.28 |

## Host GUI

```
py blinkyhawk_gui.py
```

It is the bench GUI minus the pin remapper, plus:

- volume keys on the config table
- a **Speaker test** box on the Diagnostics tab
- a read-only pin map on the **Power** tab

The unit log `blinkyhawk_units.csv` was seeded from the production GUI's log,
so an upgraded V3 can still be restored from its old row.

## Building

```
arduino-cli compile -b Seeeduino:renesas_uno:XIAO_RA4M1 BlinkyHawk_Unified
```

On this machine `arduino-cli` is at
`C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`
and needs `--config-file %USERPROFILE%\.arduinoIDE\arduino-cli.yaml`.

Current build: 91 KB flash (34 %), 12.7 KB RAM.

## Not yet verified on hardware

Everything here compiles and the GUI was smoke-tested against synthetic
device lines. Nothing has run on a V3b yet. To check first:

1. `!PINS` shows HWREV 4. The TPS22914 rails switch (`!GATE,ANA,1` / `0`).
2. `!TONE,1` / `!TONE,2` / `!TONE,3` get louder, and `!TONE,0` is silent. If
   1 and 2 sound swapped, EN1/EN2 are swapped on the board.
3. `!TONE,3` with `SPKDUTY` at 50 / 25 / 10 steps down.
4. A `!CAP` with `ANAMODE=2`, to set `ANAUS`.
5. Re-characterise `THRESH11` / `SLEEPTHR11` on V3b with the test suite.
6. Sleep current with the amp parked: EN low, DIN low.
