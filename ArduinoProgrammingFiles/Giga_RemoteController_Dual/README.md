# Giga_RemoteController_Dual

Dual-core rework of `Giga_RemoteController_WiFi`, plus a live streaming mode.

The STM32H747 on the Giga R1 has two cores. This version puts the ADC on the
Cortex-M4 and everything else on the Cortex-M7, and adds a continuous stream in
which the board reports, once per period, the **mean, min, max and standard
deviation of every sample it took in that period**.

```
Giga_RemoteController_Dual/          M7 sketch + Python GUI     <- flash SECOND
  Giga_RemoteController_Dual.ino     WiFi, serial, touchscreen, USB logging
  giga_dual_shared.h                 shared memory layout + capture engine
  Giga_RemoteController.py           GUI, now with a Live Stream window
Giga_RemoteController_M4/            M4 sketch                  <- flash FIRST
  Giga_RemoteController_M4.ino       owns the ADC, ~50 lines
  giga_dual_shared.h                 identical copy of the header
```

## Flashing

Set **`Tools ▸ Flash split` to `1MB M7 + 1MB M4` for BOTH sketches.**

That is **not** the IDE default. A custom board menu defaults to its first
entry, which for the Giga is `2MB M7 + M4 in SDRAM` — and that entry links the
M4 sketch to run from SDRAM at `0x60000000` *and* leaves `upload.address_m4`
empty in `boards.txt`, so the M4 image has no flash destination and `bootM4()`
points the M4 at uninitialised SDRAM. The board then looks exactly as if the M4
sketch had never been flashed. Both sketches now **refuse to compile** with that
setting rather than failing silently.

1. Open `Giga_RemoteController_M4`, set `Tools ▸ Target core ▸ M4 Co-processor`
   and `Tools ▸ Flash split ▸ 1MB M7 + 1MB M4`, upload.
2. Open `Giga_RemoteController_Dual`, set `Tools ▸ Target core ▸ Main Core`,
   **same flash split**, upload.

`1.5MB M7 + 0.5MB M4` also works and leaves the M7 more room (it only needs
~350 KB of either). `1MB M7 + 1MB M4` is the recommendation because it puts the
M4 image at `0x08100000`, which is the STM32H747's own default CM4 boot address
— so it starts whether the M4 is released by `bootM4()` or auto-boots from the
`BCM4` option bit.

The M4 stays halted until the M7 calls `bootM4()` in `setup()` (unless `BCM4` is
set, in which case it boots at reset instead — the handshake copes with either
order). If the M4 never answers, the M7 says so on the serial console, repeats
the diagnosis for the first few seconds so a late USB enumeration still catches
it, keeps re-probing so a slow start recovers on its own, and shows the state in
the settings-screen header.

`giga_dual_shared.h` is duplicated in both folders because Arduino cannot share a
header between sibling sketches. **Keep the copies identical** — copy one over
the other after editing. `GIGA_SHM_PROTO_VER` is baked into the magic word, so
if they drift the M7 reports a protocol mismatch rather than reading a
mismatched struct.

## If the M7 reports the M4 is not running

Open the Serial Monitor at 115200. The board prints a block like:

```
M4 ADC co-processor DID NOT START
  SRAM4 round-trip: ok
  CM4_BINARY_START = 0x8100000
  option bytes: BCM4=0  BOOT4_CUR=0x1FF0810
  shared block @0x38000000  magic=0x0 (expected 0x4741D002)
                            bootId=0x9E3779B9 ack=0x0  heartbeat=0
  -> the M4 is not executing. ...
```

Read it as:

| Symptom | Meaning |
|---|---|
| `SRAM4 round-trip: FAILED` | The M7 cannot use `0x38000000` at all — the shared-memory assumption is wrong on this build, not a flashing problem. |
| `CM4_BINARY_START = 0x60000000` | The SDRAM split slipped through. Rebuild both with `1MB M7 + 1MB M4`. |
| `heartbeat = 0`, `magic = 0` | The M4 is not executing: its image was never flashed, or went to a different address than `CM4_BINARY_START`. Re-upload the M4 sketch and check the two splits match. |
| `heartbeat` climbing, `magic` wrong | The M4 runs but the two copies of `giga_dual_shared.h` disagree. Copy one over the other and reflash both. |
| `heartbeat` climbing, `magic` right, `ack != bootId` | The M4 is one build behind — reflash it. |

`#define ADC_ON_M4 0` in the M7 sketch sidesteps the M4 entirely and runs the
same engine on the M7, which is the quickest way to confirm the rest of the
firmware is healthy.

## Why split it

The single-core sketch smoothed and reduced every sample inside a blocking
`while()` loop. During a capture, WiFi and the display were dead, and any time
the socket stack ran long the ADC's DMA queue could overrun.

Now the M4 does all per-sample work and the M7 only `memcpy`s finished frames out
of a ring in SRAM4. Consequences:

- Acquisition is unaffected by what the M7 is doing — it can push a
  10 000-point `DATA` dump over TCP mid-capture without perturbing timing.
- A live stream can run indefinitely while commands keep being served.
- The M7's RAM use dropped from 75 % to 44 % (`logData` is `uint16_t` now, and
  the per-sample accumulators moved to the M4).

Set `#define ADC_ON_M4 0` at the top of the M7 sketch for a single-core build
that runs the *same* engine on the M7. Still non-blocking, but acquisition then
competes with WiFi for the core. Useful for bisecting a problem.

### How the two cores talk

A fixed-address block at `0x38000000` (SRAM4, D3 domain — 64 KB, untouched by
either core unless RPC/OpenAMP or PDM is linked in, and this pair uses neither).
No RPC, no OpenAMP: just a struct and a frame ring.

The M7 has a write-back D-cache and the M4 has none, so the block is split into
three 32-byte-aligned regions by *writer* — host→engine, engine→host, and the
ring — and the M7 cleans only what it writes and invalidates only what the M4
writes. Neither core ever clears the other's region, so the two can start in
either order; instead the M7 stamps a fresh `hostBootId` each boot and the M4
echoes it, and a match is what "the engine is live" means. Each "ready" flag
(`cmdSeq`, `statSeq`, `state`, `ringHead`) is written
after a `__DMB()` so it can never become visible before the payload it guards.
Period statistics are read under a seqlock retry, so a report can never blend two
periods.

## Live stream

### From the GUI

`Live Stream` in the main button row opens a window that reuses the main
window's port, channel selection, sample rate, bit depth, smoothing and
calibration. Each channel gets a card with a large headline number (the period
mean) and its min / max / σ / n underneath, plus a strip chart where the solid
line is the period mean and the dashed lines are that period's min and max.
`Export CSV` writes the whole period history.

Default reporting rate is 4 Hz, selectable 1–20 Hz. At 250 kS/s with smoothing 8
and 4 Hz reporting, each headline is the mean of ~7 800 samples and n is shown so
you can confirm it.

### From the touchscreen

The `LIVE` button on the settings screen streams the configured channels and
differential pairs to the display. `Live Rate (Hz)` is the new bottom parameter
row.

### Protocol

Same line-based text protocol as the capture commands, on the same channel
(USB serial or TCP port 8080).

```
->  STREAM:A0,A1,D1|D1:A4-A5|BITS:14|RATE:250000|SMOOTH:8|HZ:4
<-  STREAMON|HZ:4|BITS:14|RATE:250000|SMOOTH:8|CH:A0,A1,D1|D1:A4-A5
<-  S|SEQ:1|T:250|SPAN:251|CH:A0|N:7812|MIN:8123|MAX:8455|MEAN:8290.3125|STD:41.2300|LAST:8301
<-  S|SEQ:1|T:250|SPAN:251|CH:A1|...
<-  E|SEQ:1|BITS:14|DROP:0
    ... one S-block per channel plus an E line, every period ...
->  STOP
<-  STREAMOFF
```

`D1` / `D2` are differential pairs. A pair's min/max/σ cannot be recovered from
its legs' statistics, so the engine reduces `(pos - neg)` per sample alongside
the real channels.

**The host must name the legs** with `|D1:A4-A5` / `|D2:A2-A3`. The board has its
own Diff config (touchscreen + built-in preset table) and it drifts from the
GUI's `giga_presets.json`, so "Diff 1" on one side is not necessarily Diff 1 on
the other — leaving the board to choose pairs one number at a time pairs its
pins with the host's calibration and reads as a plausible wrong measurement.
`STREAMON` echoes the legs actually used, the GUI checks the echo against what
it asked for and shows `PIN MISMATCH` rather than plotting, and the on-screen
rows label each pair with its legs. Omitting the fields still falls back to the
board's own config, which is what a stream started from the `LIVE` button
does. Values are raw ADC counts — the GUI owns calibration, as it
does for captures. `MIN`/`MAX`/`LAST` can be negative for a pair.

Over WiFi the stream **holds its TCP socket open** (every other command still
gets one-command-per-connection, because the Giga's socket pool is tiny and
wedges if a socket is left dangling). While it is held that socket serves `STOP`
and nothing else; anything else gets `ERR:STREAM_ACTIVE`. USB serial can always
force a stop. The GUI's `Run Test` stops a running stream first.

## What else changed

- **`WARN|DROPPED:n`** may precede `DONE` if the M7 fell far enough behind that
  the shared ring overflowed. The data is still valid, just short; the GUI shows
  it in the plot title. It should not happen in normal use.
- **The USB datalogger** now rides on the same stream mode: the engine forwards
  frames whose raw counts moved past a converted threshold, and the M7 applies
  the exact engineering-unit test to the frames it receives. Row timestamps are
  taken when a frame is drained rather than when it was sampled — the same
  one-DMA-buffer lag the single-core version had.
- **`DONE|ELAPSED`** still comes from the sample maths, not the wall clock (see
  the note in `engFinishCapture`).

## Status

Both sketches and the GUI build clean. `test_live_stream.py` drives the live
path end to end against a scripted fake board — protocol parsing, calibration of
channels and pairs, start/stop, that the STREAM command names its pins, and that
a board using different pins is reported rather than plotted. Run it with
`python test_live_stream.py`; no hardware needed. The dual-core firmware runs on hardware. There is no host compiler on this
machine, so the engine itself is exercised only on the board.
