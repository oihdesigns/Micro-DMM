# Giga ADS1256 (Wi-Fi)

An ADS1256 24-bit ADC on an **Arduino Giga R1 WiFi**, controlled over Wi-Fi
(TCP 8080) or USB. The PC front end, `gads_gui.py`, is the Wi-Fi version of
`Raspberry Pi Project/ADS1256 GUI V3_triggering.py`. It has the same panels,
channel map, DMM math, trigger modes and CSV format. The Wi-Fi side (join
thread, non-blocking output, up to five networks) comes from
`Giga_AC_Power_WiFi`.

| File | |
|---|---|
| `GigaADS1256_WiFi.ino` + `gads_types.h`, `net_sink.h` | firmware (`arduino:mbed_giga:giga`, no extra libraries) |
| `arduino_secrets.example.h` | copy to `arduino_secrets.h` (git-ignored), fill in the Wi-Fi networks |
| `gads_gui.py` | the GUI: `py -3 gads_gui.py` (matplotlib, numpy; pyserial for USB) |
| `gads_sim.py` | a stand-in board for trying the GUI: `py -3 gads_sim.py`, then connect to `127.0.0.1` |

## Quick start

1. Wire the ADS1256 (below), fit the Wi-Fi antenna, flash the sketch.
2. `py -3 gads_gui.py`, type the board's IP (or a COM port) in **Board**, Connect.
   Over USB the IP is shown in the status line. The GUI pushes all of its settings
   to the board on connect, and remembers them in `gads_gui_settings.json`.

## Wiring

| ADS1256 | Giga | |
|---|---|---|
| SCLK | D13 | SPI1, 1.875 MHz (limit is fCLKIN/4 = 1.92 MHz) |
| DIN | D11 (COPI) | |
| DOUT | D12 (CIPO) | |
| CS | D10 | |
| DRDY | D2 | edge interrupt. Not D4/D8: the Wi-Fi chip owns EXTI line 8 |
| RESET | D7 | optional; set to `-1` in the sketch (the default now) when it isn't brought out |
| AINCOM | GND | the reference for CH0–CH3 (SE) |
| SYNC/PDWN | 3.3 V | |
| DMM high-range output | D6 | the Pi's GPIO 23 (`!RANGE`) |

The free pins (D3, D4, D5, D8, D9, D22–D33) show up in the GUI's GPIO panel,
with each input's level read back once a second.

## What moved from the Pi to the board

The Pi GUI bit-banged SPI from Python, so its timing depended on the
interpreter. Here a high-priority RTOS thread owns the ADS1256. DRDY is an
interrupt that timestamps each conversion (1 µs) and wakes the thread, and the
Wi-Fi/command loop never touches SPI.

- **Benchmark** = one channel in RDATAC at the full data rate. Every DRDY edge
  is counted, so a conversion that got overwritten before it was read is
  reported (`gaps`), not silently lost.
- **Multi** = the datasheet's pipelined mux cycling (write the next MUX, SYNC,
  WAKEUP, then RDATA the channel that just finished). Each value is a settled
  conversion, with the trigger channel first in each frame (as on the Pi).
- **Triggers run on the board**, on every conversion, with the pre-trigger
  history in an SDRAM ring (up to 524 288 frames / 1 M values). Start = the
  trigger frame is first, Mid = half before it, End = it is last. Same rules
  as the Pi GUI. The capture comes back with real per-frame timestamps, and
  the trigger is marked on the plot.
- **Trigger levels in DMM units work**: the GUI turns the level into a raw
  code by inverting the DMM formula. On CH6-7 (×−68.36) that also turns a
  rising trigger into a falling one.
- **Live readings are averages.** Each 500 ms reading is the mean of every
  scan in that period (the Pi read one conversion), and the grey σ next to it
  is the noise over the same period, in the display units.

New in the GUI: Abort, a trigger timeout, Self-Cal, a Console window, and the
capture plot redraws when you switch V / Raw / DMM math (the raw codes are
kept).

## Differences you may notice from the Pi

- **"Enable Buffer" now really enables the buffer.** The Pi GUI wrote `0x01` to
  STATUS, which is the read-only DRDY bit. BUFEN is bit 1 (`0x02`), so on the
  Pi the buffer stayed off. With it on, the inputs must stay within
  AGND…AVDD−2 V (0–3 V on 5 V AVDD).
- Volts = code × 2·VREF / (gain × 0x7FFFFF) with VREF = 2.5 V. The Pi wrote
  this with "VREF = 5.0" and no factor of 2, which gives the same number, so
  the DMM math constants are unchanged.

## For the Isolated_ADS1256 board

Turn the buffer on (its COM sits at 1.5 V), and edit `CHANNEL_MAP` at the top
of `gads_gui.py`, e.g. `("HV", (0, 1), "DIFF")`, `("Shunt", (2, 3), "DIFF")`,
and AIN4–7 against AINCOM (8). The board takes whatever map the GUI sends
(`!CHMAP`, up to 8 channels). The DMM math is tied to channel positions 3 and
5, as on the Pi.

## Status

Run on hardware 2026-11-11: a Giga R1 WiFi and a generic ADS1256 dev board
(7.68 MHz crystal, RESET not brought out, so `PIN_RST = -1`), on jumper wires.
The test inputs were the Giga's 3.3 V, GND and its two DACs. Firmware 300 KB
flash.

| | Result |
|---|---|
| Chip detection | STATUS ID 3; DRATE and ADCON read back as written |
| Benchmark, 30k SPS | 30 000 samples, 33.33 µs apart; 4–5 per 30 000 dropped as reads a DRDY edge overlapped (counted as gaps) |
| Multi, 6 channels at 30k | 4058 conversions/s, no wrong values in 37 500 |
| Noise, 30k SPS, gain 1 | 12 µV rms with AIN4–AIN5 shorted |
| Accuracy | 3.3 V rail reads 3.266 V; DAC sine 0.966 V amplitude / 1.579 V offset (requested 1.0 / 1.65, DAC tolerance) |
| Triggers | Start / Mid / End, rising and falling, Benchmark and Multi: all land on the crossing |
| Transfer | 30 000 Benchmark samples: 1.6 s over USB, 9–11 s over Wi-Fi (RSSI −60) |

Inputs tied to the Giga's 3.3 V show ~1 ms bursts of ±7–15 mV while the
radio transmits (more over Wi-Fi than USB). That is the rail itself; the
shorted pair shows nothing. Use a quiet source for accuracy work.

## Hardware notes (found on the bench)

- **Slew rate.** At mbed's default (very high) GPIO speed, the COPI (DIN) edge
  coupled into the SCLK jumper. The ADS1256 saw an extra clock and about 36 %
  of Multi values came back shifted one bit (exactly ×2, or wrapped negative
  past full scale). The sketch sets D11 and D13 to the slowest speed at boot,
  and that brought it to 0 in 37 500. `!SLEW,<sck>,<copi>` changes it (0..3;
  `$STATUS slew=`).
- **DIN while reading.** After RDATA the sketch clocks `0xFF` (DIN is already
  high); in RDATAC it clocks `0x00` (DIN already low). DIN doesn't move when a
  read starts, which keeps the margin if the wiring gets longer.
- **AINCOM must go to GND** (or your reference) for the single-ended channels.
  A floating input reads ~⅓ of 3.3 V and picks up the previous channel's
  charge in Multi.
- Keep SCLK, DIN and DOUT short and away from each other. A ground wire
  alongside helps.
- `!DAC` / `!DACSINE` drive A12 / A13 as a test source: wire them to two
  inputs to check readings and triggers with nothing else on the bench.

## Protocol

The header of `GigaADS1256_WiFi.ino` lists every command and line. Commands
start with `!`, replies with `$`, one per line. One TCP client at a time;
a new connection replaces the old one.
