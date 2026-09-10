# BlinkyHawk_Bench — experimental breadboard fork

An experimental fork of `BlinkyHawk_RA4M1` for bench work where the XIAO RA4M1
and the Blinky Hawk PCB are two separate parts on a breadboard, jumpered
together, so individual sections of the circuit can be cut into and measured.

Detection, alerts, sleep, the config system, and the host protocol are the
production firmware unchanged, so numbers taken here transfer back.

## What it adds

**A runtime pin map.** Every pin the firmware drives is an EEPROM key instead
of a compile-time constant, so a jumper can move and the firmware be told about
it over serial rather than by recompiling.

**A second sleep stage.** Production has one sleeping state, and it is now the
normal operating mode. This adds a deeper one under it — see below.

**Three power gates.** Spare GPIOs drive external load switches so a section of
the circuit can be powered down and the saving measured:

| Gate | Meant for | Why it can't be done on the real board |
|------|-----------|----------------------------------------|
| `ANA` | The analog front end — TL431 reference bias and the divider strings | Permanently across the rail on V3; the prime candidate for a hardware gate |
| `LED` | LED1's supply | On V3, LED1 hangs off BatteryRail with nothing in series, so its quiescent draw is a hardware floor firmware cannot reach |
| `AUX` | Spare | — |

## Its EEPROM is a separate block

Magic `"BHKX"` at address **1024**, not `"BHK1"` at 0. EEPROM on the RA4M1 is
data flash and a sketch upload does not erase it, so without this the two
firmwares would fight over the same bytes. As built:

- flashing this onto a real unit does **not** touch that unit's production config
- flashing production back restores it untouched
- the unit serial-number block (512) is shared and read normally
- older bench layouts **are migrated** (see below); only a genuinely
  unreadable image falls back to defaults

## Config versions and migration

This fork originally shipped with no migrations, on the reasoning that a bench
config is only a few minutes of `!SET`. That was wrong — three `CFG_VERSION`
bumps in three days wiped a unit that was mid-experiment. A bench unit
characterised against a current trace is as much a record as a field unit;
losing it costs a re-characterisation, not a re-typing.

`configMigrateBench()` now carries older layouts forward. It does not use the
production fork's frozen-struct-per-version approach, because every bench
version so far is the previous one with fields **appended immediately before
`crc`** — which makes every older image a byte-identical *prefix* of the current
struct. One function covers all past versions and every future append: CRC-check
the stored image over its own prefix, copy that prefix over a
defaults-initialised config, and anything the old version lacked keeps its new
default. The boot banner says `MIGRATED` when this happens.

**The append-only invariant is load-bearing.** If a field is ever reordered,
resized, or removed rather than appended, do *not* add a row to
`BENCH_LAYOUTS` — change `CFG_MAGIC` instead, which rejects every old image
cleanly. Three `static_assert`s catch a reordering of the fields already listed.

Failure is safe in the only direction that matters: a wrong table row means the
CRC does not match, the migration declines, and the board falls back to defaults
— exactly the behaviour it had before this existed.

| Version | Added |
|---|---|
| 1 | Pin map + power gates |
| 2 | Deep-sleep stage (`DEEPSEC`, `DEEPHZ`) |
| 3 | `DEEPPARK` |
| 4 | `CHGINHIBIT` |

### Recovering a config that was already lost

Every successful `!SAVE` appends this unit's whole config to
`blinkyhawk_bench_units.csv`, keyed by serial number. **Restore this unit…** in
the Configuration tab re-sends that row.

It is a separate button from **Copy from unit…** because the two obey different
safety rules, and conflating them is what made the first recovery impossible:

- *Copy from unit* clones tuning **between** boards, so it must never carry the
  pin map or gate polarities — those describe wiring, not tuning.
- *Restore this unit* replays a row that came off **this same board**, so the
  wiring keys are exactly what you want back, and they are sent.

`HWREV` stays protected in both. If the XIAO has moved to a different PCB since
that row was written, a stale `HWREV` is the one value that can drive D8 into a
short — set it deliberately.

A unit with no serial number has no row and cannot be restored, so assign one
with **Write SN…** before the first save.

## Two-stage sleep

Stage 1 is the existing sleeping mode: after `SLEEPSEC` of open leads, park
everything and wake every `SLEEPTICKMS` to probe (~2 mA at the shipping 63 ms /
1 s config). Stage 2 sits under it:

| Key | Default | Means |
|-----|---------|-------|
| `DEEPSEC` | 5 | Seconds of stage-1 sleep, every probe reading open, before dropping to stage 2. `0` = never — exactly the production behaviour |
| `DEEPHZ` | 1 | Stage-2 probe rate in Hz |
| `DEEPPARK` | 2 | Bridge MOSFET in stage 2: `0` resting (ON), `1` parked OFF, `2` follow `SLEEPPARK` |

`DEEPHZ` re-programs the **RTC wake period**, not just the probe divisor. That
distinction is the point: at these rates the standby wake itself is most of the
cost, so skipping probes on a fast tick would save almost nothing. At the
default the board goes from waking ~16×/s to once a second.

The rate is snapped to what the RTC ladder can produce, **never faster than
asked** (a 3 Hz request lands on 2 Hz, not 4), and the achieved value is written
back to `DEEPHZ` so `!CFG` always reports what is in force. Read the derived
schedule with `!DEEP`:

```
$DEEP,stage=1,deepsec=5,hz=1.000,tickms=1000,ticks=1,probems=1000,
      lightms=63,lightticks=1,lightprobems=63,forced=0
```

Anything that ends stage 1 — a connected lead, USB, a serial byte — ends stage 2
the same way, and the next sleep restarts at stage 1. The countdown is in
**ticks**, not `millis()`, which is frozen in Software Standby.

**The cost is latency.** At 1 Hz a closed lead can wait a second to be noticed,
against ~63 ms in stage 1. `DEEPSEC` is where you place that trade: fast while
the unit might still be in use, slow once it clearly is not.

`SLEEPHB` counts ticks, so the heartbeat flash slows with the stage — 32 ticks
is ~2 s in stage 1 and ~32 s at a 1 s deep rung. That is intended; the flash is
not free.

`!DEEP,1` forces stage 2 immediately (only while already asleep) so a meter
reading can be taken without waiting out `DEEPSEC`; `!DEEP,0` goes back up.

### The bridge MOSFET in deep sleep

`SLEEPPARK` (default 0 = resting/ON) governs both stages unless `DEEPPARK` says
otherwise. The bridge resting leaves the 100 kΩ sense leg connected to the 2.5 V
reference, so it draws continuously — this is the "bridge leg's share" that
`!FLOOR` level 1 vs 2 was built to measure, and the obvious next thing to switch
off once the wake rate is already down. `DEEPPARK=1` does that for stage 2 only.

`DEEPPARK=2` (follow `SLEEPPARK`) is the default for two reasons: adding the key
changed nothing, and a unit set to `SLEEPPARK=1` cannot end up with a *deep*
stage that is *less* parked than its light stage because only one of the two
keys got changed.

**Before trusting `DEEPPARK=1`:** the probe re-establishes the resting state and
waits `SETTLEPOSTMS` before testing, but in stage 2 the node will have been
floating for a whole deep period (a second at the default) rather than one 63 ms
tick. If closed leads stop waking the board after enabling this, raise
`SETTLEPOSTMS` before suspecting anything else — and check what it did to the
metric with `!SLEEPTEST` / `!SLEEPLOG`, not by eye.

`!DEEP` prints the resolved state for both stages (`lightbridge=`, `deepbridge=`,
`bridgenow=`), because with the default `DEEPPARK=2` the key on its own does not
tell you what the pin is doing.

## Running off something that is not a battery

`CHGINHIBIT` (default 1). A3 reads VBUS/2, and on a real unit VBUS present means
"on the charger", which the firmware treats as a reason to stop: no sleeping, no
normal alerts (the charging blink takes the LED, the speaker is silenced), and
an immediate wake if it appears while asleep. Those are the right defaults for a
product whose charger is a USB cable that can earth-ground the meter.

They are wrong the moment the 5 V input is a bench supply, a boost converter, or
primary cells behind a boost — VBUS is then just *how the board is powered*, it
never goes away, and the unit would never sleep, never alert, and never be
measurable.

```
!SET,CHGINHIBIT,0
!SAVE
```

Charge is still **detected and reported** (`$STATUS charge=`, and the GUI's
readout, which says `5V in (ignored)`) — it just inhibits nothing. That split is
deliberate: keeping the telemetry is how you tell a working boost supply from a
disconnected one while the board ignores it.

Two details worth knowing:

- While asleep with `CHGINHIBIT=0` the wake-on-charge check is **skipped
  entirely**, not merely ignored — otherwise it would fire on the first tick and
  wake the board forever. That also saves an ADC conversion per wake. As a
  consequence `charge=` goes stale while asleep and refreshes on the next wake.
- The 300 ms boot USB settle still runs, because it happens before the config is
  loaded. Harmless, but it is 300 ms of every boot on a boost supply.

## Wiring a gate

Each gate is one GPIO driving a load switch in the supply of the thing being
gated. A P-FET high-side switch is the obvious part, and it is usually **active
low** — which is why `ANAPOL`/`LEDGPOL`/`AUXPOL` default to 0. Getting the
polarity backwards leaves the rail permanently on, and that reads as "gating
saves nothing" rather than as a wiring error, so check it against `!GATE` before
trusting a measurement.

Wire the switch, then tell the firmware which pin drives it:

```
!PINS                  # read the pin-name table first: keys take NUMBERS
!SET,PINANA,<num>      # e.g. the number !PINS printed for D3
!SET,ANAPOL,0          # 0 = LOW enables the rail (P-FET high-side)
!SET,ANAUS,2000        # settle before reading, in microseconds
!SET,ANAMODE,1         # 0 always on / 1 off while asleep / 2 pulsed
!SAVE
```

## Gate modes and the diagnostics

**A pulsed gate powers the front end only during a detection pass**, so anything
else that reads the ADC has to raise it too. `runDetection()` and the sleeping
probe always did; `!CAP` and `!STREAM` did not, which made both read a
powered-down front end and plot flat lines. Both now raise the gates around
their reads.

`!CAP` goes further: with a pulsed gate in play, the capture brings the rail up
**inside** the sampled window rather than before it. The trace runs
rail-down baseline → rail rises → settle → MOSFET toggle, so one capture shows
both how long the front end really takes to come good (which is what `ANAUS`
should be set from) and the detection transient from a properly settled
baseline. The rail-up phase is *added* to the requested duration, so the
post-toggle window stays the length you asked for.

`$CAPSTART` gained three appended fields — `gateUs`, `gateMask`, `gateSettleUs`
— and the GUI shades the rail-down region red, marks the rail edge, and shades
the settle green. **If the trace is still moving when the settle region ends,
`ANAUS` is too short.** Saved capture CSVs gain a `rail_up` column.

## Gate modes

| Mode | Behaviour | What it tells you |
|------|-----------|-------------------|
| 0 `ALWAYS` | Held on | The control case — no gating |
| 1 `SLEEP` | On while awake, dropped while asleep or in `!FLOOR` | What the load costs during the sleeping duty cycle, which is the normal operating state |
| 2 `PULSED` | Off at rest, raised only for the settle + measure window | The floor — but the one most likely to move the measurement |

Modes 1 and 2 **perturb the measurement**. Rail loading moves the resting
differential by more than `REFBAND`; this is exactly why the production
firmware's sleeping probe powers its rails back up before probing, and a pulsed
reference is the same problem with a slower time constant. Expect to re-tune
`THRESH`/`SLEEPTHR` per gating scheme and compare the current saving against
the detection cost, rather than reusing one threshold across all of them. If a
mode-2 metric disagrees with a mode-0 one, raise `ANAUS` before suspecting the
detection method.

## Sharing a pin between two functions

`!PINS` ends with a `$PINSHARED` line for any pin claimed by more than one
function. That is legal and sometimes deliberate — A1 is a hard no-connect on
V3, so using it as a gate output while `PINSENSEN` nominally still names it is
the right move (`NEGFIX=1` means `SENSE_NEG` is never read). It is also an easy
way to break something by accident, hence the report.

`applyPinMap()` configures in a fixed order and **the last writer wins**: sense
and charge inputs, then the MOSFET, then the gates, then the buzzer/DIP pins.

## Commands

Everything the production firmware has, plus:

| Command | Does |
|---------|------|
| `!PINS` | Dump the live pin map and this core's pin name-to-number table |
| `!GATE` | Report all three gates |
| `!GATE,<ANA\|LED\|AUX>,<0\|1\|-1>` | Hold a gate off / on, or `-1` to follow its `MODE` again |
| `!DEEP` | Report the two-stage sleep schedule |
| `!DEEP,<0\|1>` | While asleep: force stage 2 (`1`) or go back to stage 1 (`0`) |
| `!EXPT[,<sec>]` | Run the ten-step power profile, `<sec>` per step (10 default). `!EXPT,0` aborts |

A `!GATE` hold is RAM only and **survives into sleep and `!FLOOR`** — that is
how the sleeping current for a given scheme gets measured: set the holds, then
`!SLEEP` or `!FLOOR`, then unplug.

## Measuring

Both the existing `!FLOOR` levels and the new `!EXPT` profile are read the same
way as before: on battery, with the meter in series, and **the deltas are the
answer**. The absolute number here includes the breadboard, the jumpers, and
whatever the meter is doing.

`!EXPT` runs ten equal steps — the ANA and LED gates swept while running, the
same sweep asleep in stage 1, then the best and worst of those repeated in the
deep stage — so one PPK2 capture contains every case with the boundaries at
known times. Read together, the last two answer whether gating and the deeper
stage are additive or whether the slower probe rate has already taken most of
what gating the analog rail would have. It prints the whole schedule before
anything moves, because the run itself happens with USB unplugged:

```
!EXPT,30            # read the printed plan, then unplug and let it run
```

Any serial byte aborts, so replugging ends the run. Parked steps are timed by
counting RTC ticks rather than by `millis()` (which is frozen in Software
Standby), so real boundaries land within one tick of the printed offsets — one
*deep* tick for the deep steps, which is why those count against the deep period
rather than the stage-1 one.

## Host GUI

```
py blinkyhawk_bench_gui.py
```

The production GUI plus a **Bench** tab: the pin map with a reassign dropdown,
the live gate state with Auto/On/Off holds, the two-stage sleep schedule with
force-stage buttons, and the experiment runner with a "Save plan CSV" that
writes the step boundaries as start/end seconds for slicing a capture. The top
bar shows the live sleep stage and gate levels.

Two things it deliberately keeps separate from the production tooling:

- the per-unit log is `blinkyhawk_bench_units.csv` — bench numbers are not unit
  tuning records
- the pin map and gate polarities are in `NEVER_COPY_KEYS`, so "Copy from
  unit…" will not clone them. They describe *this* breadboard, in the same way
  `HWREV` describes which PCB is underneath

## Config version

See "Config versions and migration" above. This fork has **no migrations** by
design, so a board holding a v1 bench block is reseeded with defaults on the
first boot of this firmware — re-enter any bench tuning. (Production config at
address 0 is untouched either way.)

## Building

```
arduino-cli compile -b Seeeduino:renesas_uno:XIAO_RA4M1 BlinkyHawk_Bench
```

On this machine `arduino-cli` is at
`C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`
and needs `--config-file %USERPROFILE%\.arduinoIDE\arduino-cli.yaml`.

Note: the XIAO RA4M1 variant defines `A0`–`A3` only. `A4`/`A5` do not exist and
adding them to `PIN_NAMES` will not compile.
