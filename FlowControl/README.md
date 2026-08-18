# FlowControl — Alicat flow controller GUI + `flowlab` library

A proof-of-concept control panel for Alicat mass flow controllers, written so
that most of it survives being reused: the GUI is a thin consumer of a library
(`flowlab`) that knows about buses, devices, polling, logging and sequences —
not about tkinter.

Built and verified against the register map in the **Alicat Modbus Manual**
(Nov 2024, Rev 7) and the command set in the **Alicat Serial Primer**
(Feb 2023, Rev 2). Both protocols are implemented:

| Ordered as | Protocol | Driver |
|---|---|---|
| `…-MODBUS-485` (the MC-500SCCM on the bench) | Modbus RTU | `flowlab/alicat_modbus.py` |
| RS-232 / RS-485 without the Modbus option | ASCII polling frames | `flowlab/alicat_ascii.py` |

Mixed rigs are fine — one Modbus segment on COM5 and an RS-232 unit on COM7
appear side by side in the same window.

## Quick start

```
pip install pyserial matplotlib

python selftest.py          # protocol checks against fake instruments, no hardware
python flow_gui.py          # GUI with two simulated controllers
python scan_bus.py          # list serial ports
python scan_bus.py COM5     # find out what is actually on the bus
python flow_gui.py bench_rig.json
```

Nothing needs hardware except the last two lines. The simulated rig is a real
`Device` implementation with first-order flow dynamics, so ramps, holds,
totalization and logging all behave while you build the UI around them.

### First contact with real hardware

1. `python scan_bus.py COM5` — scans Modbus slaves 1–16 **and** ASCII unit IDs
   A–Z, read-only. Add `--baud 38400` if the unit was ordered otherwise;
   Alicat's factory default is **19200 8N1, no parity**, for both protocols.
2. `python scan_bus.py COM5 --watch 1` — poll one unit at 2 Hz and watch the
   numbers, before any GUI is involved.
3. Copy `bench_rig.example.json`, fix ports/addresses/full scales, then
   `python flow_gui.py my_rig.json`.

If the scan finds nothing: check A/B polarity, termination, that the baud rate
matches, and that no unit was left in **streaming mode** (`@`) — a streaming
device floods a shared line. `AlicatAscii.stop_streaming()` recovers it.

## The GUI

| Tab | What it does |
|---|---|
| **Live** | One card per device: big flow reading, setpoint entry + slider + 0/25/50/100 % buttons, pressure/temperature/volumetric/totalizer, tare, valve hold, totalizer reset, status flags. Rolling chart of any reading with setpoint overlay. |
| **Devices** | Add buses (port, baud, protocol), scan a bus, add/remove devices, save and load the rig JSON. |
| **Sequence** | Ramp/hold setpoint programs across any mix of devices, with optional settle-before-hold, loop, and zero-at-end. |
| **Terminal** | Hand-typed traffic per device. Modbus: `read 1200 8`, `float 1010`, `cmd 4 2`, `sp 250`. ASCII: `V`, `S12.5`, `??D*` — the unit ID is prefixed for you. |
| **Log** | CSV logging (one row per poll per device), per-bus tx/rx/retry/timeout counters, event history. |

**ALL OFF** (top right) zeroes every setpoint and holds every valve closed. The
sequence runner stops with it.

## Architecture

```
flow_gui.py            tkinter — knows only Device / Snapshot / DeviceManager
example_headless.py    the same rig driven with no GUI (blend-ratio loop)
scan_bus.py            read-only bus discovery CLI
selftest.py            fake Modbus slave + fake ASCII unit, exercises the wire code

flowlab/
  core.py              Device, Snapshot, DeviceInfo, reading names, errors
  bus.py               SerialBus: one port, one lock, retries, stats
  modbus.py            Modbus RTU master (FC 3/4/16, CRC, float/word order)
  alicat_modbus.py     register map, command IDs, status bits
  alicat_ascii.py      data-frame parsing, ASCII command set
  simulator.py         a fake controller with plausible dynamics
  manager.py           one worker thread per bus: polling, jobs, history, CSV
  sequence.py          Step / Sequence / SequenceRunner
  config.py            rig JSON <-> live DeviceManager
  discover.py          scan_modbus / scan_ascii
```

Three decisions matter most for reuse:

**One worker thread per bus, everything as a job.** A shared RS-485 line can
carry one transaction at a time. Polls and commands both queue on the bus's
worker, so a setpoint write from the GUI can never interleave with a poll of a
different device on the same line. Commands return a `Future`; the GUI marshals
results back onto the Tk thread rather than touching widgets from a worker.

**`Device` is the only extension point.** A pressure controller, a scale, a
thermocouple reader or one of the Arduino boards elsewhere in this repo joins
the system by subclassing `Device` and returning `Snapshot`s. The manager,
sequence runner, logger and GUI need no changes. `SimulatedController` is the
worked example — it implements the interface and nothing else.

**No pymodbus.** The RTU master is ~150 lines against `SerialBus` and does not
churn between library major versions. It exposes register *numbers* (as the
manual prints them) and converts to wire addresses in exactly one place.

## Rig files

```json
{
  "buses": [
    {"id": "bench", "protocol": "modbus", "port": "COM5", "baud": 19200},
    {"id": "aux",   "protocol": "ascii",  "port": "COM7", "baud": 19200},
    {"id": "sim",   "protocol": "sim"}
  ],
  "devices": [
    {"name": "MFC-N2", "bus": "bench", "address": "1",
     "model": "MC-500SCCM-D", "units": "SCCM", "full_scale": 500, "poll_hz": 4},
    {"name": "MFM-vent", "bus": "aux", "address": "B", "units": "SLPM",
     "full_scale": 5, "is_controller": false,
     "readings": ["pressure", "temperature", "volumetric", "mass"]}
  ]
}
```

`address` is the Modbus slave number or the ASCII unit-ID letter. `readings`
overrides the column mapping when a device's data frame is not the default
(pressure, temperature, volumetric, mass, setpoint, totalizer) — several SCCM
ranges of the same model share one mapping, but a meter, a totalizer-less unit
or a custom frame will not.

## Protocol notes worth keeping

*Modbus RTU*

- Register **number** = wire **address** + 1. The manual prints both; the code
  uses numbers.
- 32-bit values span two registers, most significant register first, MSB first
  inside each register, IEEE-754 floats.
- Setpoint: float32 at register **1010–1011**, read/write (write-only before
  firmware 10v07.0).
- "Optimized" reading block from **1200**: gas number, 32-bit status bitfield,
  then 20 float32 reading slots in the device's own data-frame order. Present
  since 6v17.0 — this is the block the driver uses by default.
- "Standard" block from **1346** (10v07.0+) puts each quantity at a fixed
  register regardless of model: set `"block": "standard"` per device to use it.
- Commands: write `[id, argument]` to registers **1000–1001**, then read them
  back; a result of 32769–32774 is an error code. The instrument only acts when
  the ID/argument pair *changes*, so the driver sends a no-op first — otherwise
  taring twice in a row silently does nothing the second time.
- Command IDs used here: 1 set gas, 4 tare (0 gauge/diff P, 1 absolute, 2 flow),
  5 reset totalizer 1, 6 hold valve (0 cancel, 1 closed, 2 current, 3 exhaust),
  8/9/10 P/I/D gain, 11 loop variable, 14 read gain, 80 reset totalizer *n*.
- Registers 1088–1089 always read back 1.234567; `ModbusMaster.detect_order()`
  uses that to catch a gateway that swaps words or bytes (10v19.0+ only).
- Function 3 and 4 both work on RTU; the driver starts with 3 and falls back to
  4 if the instrument refuses it.

*ASCII*

- Poll = the unit ID alone. Commands: `S<value>` setpoint, `V` tare flow,
  `P`/`PC` tare gauge/absolute pressure, `G<n>` gas, `HC`/`HP`/`E`/`C` valve
  hold closed/current/exhaust/cancel, `T <n>` reset totalizer, `VE` firmware,
  `??D*` data-frame description, `??M*` manufacturing info.
- Engineering units are **not** transmitted — they come from the device
  display or `??D*`, which is why `units` is a rig-file field.
- Status codes trail the frame: ADC, EXH, HLD, LCK, MOV, OPL, OVR, POV, TMF,
  TOV, VOV.

## Verified vs. still to confirm on hardware

`selftest.py` drives the real driver code against a fake Modbus slave and a
fake ASCII unit: CRCs, framing, exception responses, register decoding, the
command handshake, the repeated-command no-op, and data-frame parsing all pass.
What that cannot prove:

- **Firmware age.** A unit older than 6v17.0 has no optimized reading block at
  all; the legacy float registers at 2041–2052 exist on every RTU unit and
  would be a small addition to the driver if a unit needs them.
- **Reading slot order** on non-MC models — check the Live tab against the
  device's own display once, and set `readings` in the rig file if it differs.
- **`T <n>` totalizer reset spacing** on older ASCII firmware; the driver
  falls back to bare `T` if the spaced form is rejected.
- **Multi-drop timing.** `inter_frame` (default 5 ms) and `timeout` (250 ms)
  are conservative starting values; a long 485 run with many units may want
  more. The Log tab's retry/timeout counters are the thing to watch.

## Where this grows

The pieces intended to be reused as-is: `core.Device`, `bus.SerialBus`,
`modbus.ModbusMaster`, `manager.DeviceManager`, `sequence.*`, `config.*`.
Natural next steps, in rough order of value:

1. More device types behind `Device` — pressure controllers, valves, a scale,
   the Arduino instruments in this repo (their `$`/`!` line protocol is a
   `Device` subclass, nothing more).
2. Interlocks: a supervisor that watches `Snapshot.flags` and trips `all_off()`
   on OPL/POV, running in the same event loop as the sequence runner.
3. Recipes with branching and per-step logging, replacing the flat step list.
4. Closed-loop control *across* devices (blend ratio, pressure-by-flow) — see
   `example_headless.py` for the shape of it.
