# Front-end characterisation (12 V op-amp buffer board)

`frontend_characterize.py` measures the analog chain the Giga sees through
preset `OpAmpBuffer_12vboard`: input = 7.005 x (V_A2 - V_A3). It drives the
Rigol DG852 Pro CH1 (192.168.10.3:5025) into the board input and captures with
the board's normal `PINS:` command, so the firmware needs nothing special.
It reads the Rigol's CH1 settings first and puts them back at the end.

```
py frontend_characterize.py --board COM32 --label usb_bench --note "on USB to the PC"
py frontend_characterize.py --board 192.168.1.42 --label battery --note "battery, Wi-Fi"   # port 8080
py frontend_characterize.py --compare runs/20261105-0617_usb_bench runs/<battery run>
```

A full run is 62 captures and takes about 3 minutes. `--tests noise,dc,sweep,large,thd`
picks a subset. Each run is saved to `runs/<date-time>_<label>/`:
- `summary.md`: the readable tables.
- `results.json`: every number, which is what `--compare` reads.
- `raw_captures.json`: every A2/A3 sample, for re-analysis later.

## What each test does

| test | Rigol | captures | gives |
|---|---|---|---|
| noise | 0 V DC (a low-impedance short) | 500k, 250k, 100k, 10k | rms / p-p noise, A2-A3 correlation, offset, the ~632 kHz supply ripple per pin and in the diff, SMOOTH 4/16/64, 60/120/180 Hz hum |
| dc | -10 .. +10 V in 1 V steps | 10k x 200 ms each | gain vs 7.005, offset, linearity, each pin's voltage, extrapolation to +/-12 V |
| sweep | 2 Vpp sine, 10 Hz .. 10 MHz | 10k / 100k / 500k | small-signal response, -0.1 / -1 / -3 dB points, A2 vs A3 leg match |
| large | 19.9 Vpp sine, 100 Hz .. 1 MHz | as sweep | large-signal response (slew / clipping) |
| thd | 19.9 Vpp at 100 Hz and 1 kHz | 1100 / 120 ms | harmonics 2-7, THD, peak flattening |

**How the sweep measures:** each point is a least-squares sine fit at the frequency the tone appears
at after sampling. That works above Nyquist too: a 3 MHz tone sampled at 500k is fitted
where it lands. The test frequency is nudged by a fraction of a percent when its image
would land on DC or Nyquist.

**Power:** USB to the PC is the default. USB and battery were compared on 2026-11-05 and measured the same
(see "Comparing against battery power" below), so a battery run is only needed if the setup changes.

## Baseline: 2026-11-05, Giga on USB to the PC (two runs back to back)

The two runs, `20261105-0617_usb_bench` and `_0620_usb_bench_repeat`, agree within about 0.5 mV
noise, 0.1 kHz on the -3 dB point and 0.005 % gain. Treat differences that size as run-to-run
scatter when comparing against a battery run.

- **Bandwidth: one pole at about 11 kHz.** The -3 dB point is 10.9 kHz; -1 dB is 5.4 kHz and
  -0.1 dB is 2 kHz. The response falls 20 dB/decade up to about 200 kHz (-27.5 dB), then
  faster: -34 dB at 300 kHz and -48 dB at 500 kHz. At 1 MHz it is below -72 dB, which is
  the floor at 19.9 Vpp.
  - The 19.9 Vpp curve matches the 2 Vpp one (-3 dB at 10.6 kHz), so there is no slew limit.
  - The A2/A3 legs stay at 180.0 deg +/-1 to 300 kHz.
- **Gain and offset:** gain is +0.014 % vs the preset 7.005. The offset is +10 mV input-referred at 0 V.
- **Range:** linear across the full +/-10 V the Rigol can make, with no compression at the ends.
  - The pins run 0.76 to 2.18 V at +/-10 V. Extrapolated to +/-12 V they would be 0.61 to 2.33 V.
  - The Giga ADC itself would not clip until about +/-20.6 V.
  - **+/-12 V itself is not verified.** That needs a source beyond 10 V (see below).
- **Linearity: worst error about 55 mV over +/-10 V, with THD 0.25 % at both 100 Hz and 1 kHz.**
  - The error is not the board. It is the Giga ADC: repeatable steps of about 7 counts at particular codes.
  - The same code read the same error on A2 (ADC1) and A3 (ADC2). A held level repeated within +/-0.7 counts.
  - The harmonics are mostly 5th and 7th and identical at both frequencies, which is a static staircase
    error, not a frequency effect.
  - The library only runs the ADC's offset calibration (`ADC_CALIB_OFFSET`), not the linearity
    calibration. That is the likely fix.
- **Noise: 32 mV rms input-referred (5.7 counts in the diff), the same at every sample rate.**
  - SMOOTH 16 at 500k brings it to about 8.3 mV.
  - Mains hum is under 1 mV on USB.
- **Ripple:**
  - The ~632 kHz ripple is about 45 mV input-referred on each pin, but only about 6 mV in the A2-A3 diff,
    because the two pins are sampled at the same instant.
  - At 632 kHz the front end attenuates by more than 55 dB. A 45 mV ripple at the pins therefore isn't
    coming in through the input. It is injected after the 11 kHz filter, in the output or
    isolation-amp stage or by coupling from the isolated DC-DC.
  - The broadband noise behaves the same way. It falls as 1/sqrt(N) with SMOOTH, so it is white out to
    250 kHz and also arises after the filter.

**What this means for settings:** above about 20 kHz there is no signal left, only noise. Capturing at
500k with SMOOTH 8 to 16 reduces the noise by 3-4x and costs little of the 11 kHz bandwidth: SMOOTH 16
at 500k is a 31 kS/s stream.

## Verifying +/-12 V

The Rigol stops at +/-10 V into high-Z. Two ways to test beyond it:
- **SPD4323X:** set one floating channel to 12 V, then 15 V, across the input, and reverse the leads
  for the negative side.
- **Both Rigol channels:** if the input is truly differential, drive one side with CH1 and the
  other side with CH2 in inverted phase.

## Comparing against battery power

Wi-Fi credentials: the sketch reads `SECRET_SSID` / `SECRET_PASS` from `../arduino_secrets.h`, which is
git-ignored. Copy `arduino_secrets.example.h` to make it; the sketch stops with an `#error` if it is missing.

Run it again with the board on battery, and record the power source in `--note`. Over Wi-Fi, pass the
board's IP as `--board`; the IP is shown on the settings screen header. Then compare the two runs with
`--compare`.

The numbers most likely to move:
- the 60/120 Hz hum (ground loop through the PC's USB),
- the 632 kHz ripple per pin,
- the noise rms and the offset.

### Result: 2026-11-05, battery + Wi-Fi (`20261105-1959_battery_wifi`)

**Tested USB vs battery: no difference. USB power is the default for this bench from here on.** Every noise
number is inside the scatter between the two USB runs:

| | USB (2 runs) | battery |
|---|---|---|
| noise rms, 10k-500k | 31.7-32.8 mV | 30.7-32.0 mV |
| A2/A3 correlation | 0.68-0.72 | 0.68-0.70 |
| 632 kHz ripple per pin | 36-51 mV | 41-47 mV |
| ripple in the diff | 4.7-7.6 mV | 5.7-7.6 mV |
| SMOOTH 16 | 7.4-8.4 mV | 7.4-8.4 mV |
| 60/120/180 Hz | 0.8-1.2 mV | 0.7-1.5 mV |

- **Hum:** the hum figures are at the measurement floor either way. A 1.1 s capture of 32 mV white noise
  reads about 1 mV in any single bin, and random bins read up to 3-4 mV. So hum is below ~1 mV on both supplies.
- **Ripple:** it didn't change, so it comes from the board's own isolated DC-DC, not from USB.
- **The only consistent shift is the offset,** about 4 mV lower on battery at every rate (+4.6 to +6.8 mV vs +8.4 to
  +11.1 mV). That is under one ADC count.
- **DC transfer:** gain moved +0.07 % (0.086 % vs 0.014-0.018 %), and the non-linearity was the same at 53.6 mV.
- **Caveat:** the Rigol stays connected, and its ground is earthed. This run removed the PC/USB path, but not every
  earth connection to the input.

The first attempt, `20261105-1902_battery_wifi`, is damaged by Wi-Fi byte drops; see the NOTE.md in that folder.
`tcp_stream.h` fixes the cause.

## Transient visibility report

`Transient_visibility_report.pdf` (11 pages) shows what weak transients of different lengths look like through
this front end: pulses from 0.5 us to 1 ms at 5 mV to 1 V, 3-cycle sine bursts from 2 to 300 kHz, and one pulse at
six rate/SMOOTH settings. Each plot shows the true generator waveform beside the recorded data.

```
py transient_capture.py --board COM32 --label usb          # ~7 min, 128 captures -> runs/<stamp>_transients_usb/
py transient_report.py runs/<stamp>_transients_usb runs/<stamp>_usb_bench [out.pdf]
py transient_capture.py --board COM32 --only bursts --update runs/<stamp>_transients_usb   # redo just the bursts
```

DG852 Pro gotcha: the internal burst trigger is `:TRIG1:SOUR IMM`. With the source left at BUS (how it was found),
no bursts come out, and `:SOUR1:BURS:TRIG:SOUR` doesn't exist.

## High-impedance source: 1 Mohm in series (2026-11-06)

`Series_1Mohm_source_report.pdf` (6 pages) compares the 50 ohm runs with a 1 Mohm resistor in series with the Rigol,
standing in for a high-impedance source. Data: `runs/20261106-0735_usb_1M_series` (bench) and
`runs/20261106-0739_transients_usb_1M` (128 transient captures).

```
py frontend_characterize.py --board COM32 --label usb_1M_series --note "1 Mohm in series"
py transient_capture.py --board COM32 --label usb_1M
py series_r_report.py runs/20261105-0754_transients_usb runs/20261105-0617_usb_bench \
                      runs/20261106-0739_transients_usb_1M runs/20261106-0735_usb_1M_series 1e6
```

- **One extra pole at 8.67 kHz** (tau 18.4 us, so about 18 pF at the input). It fits the sweep difference to 0.14 dB rms.
  - The -3 dB point drops from 10.9 to 6.0 kHz, and the 10-90 % rise goes from 34 to 58 us.
  - Short pulses read as height x width / 43 us, against / 16 us with 50 ohm.
  - The apparent-width floor goes from ~12 to ~39 us.
  - The 4-pole model matches the captures (median fitted gain 1.006).
- **60 Hz hum: 78 mV peak**, against under 1 mV with 50 ohm. SMOOTH can't average it away.
  - As recorded, the noise is 57 mV rms at SMOOTH 8 (was 11.7 mV). With 60-300 Hz fitted out it is 13.0 mV, and raw it
    is 32.0 mV (was 32.6 mV).
- **DC:** gain is unchanged within scatter, so the input resistance is above about 1.5 Gohm. The offset moved -14 mV,
  which would be about 14 nA of input current.
- **5-sigma single-shot minimum at SMOOTH 8, 50 ohm / 1 Mohm with the hum removed / 1 Mohm as recorded:**

  | pulse | 50 ohm | 1 Mohm, hum removed | 1 Mohm, as recorded |
  |---|---|---|---|
  | >= 100 us | 57 mV | 66 mV | 289 mV |
  | 10 us | 152 mV | 297 mV | 1.3 V |
  | 1 us | 1.4 V | 2.9 V | 13 V |
- **Not measured: 10 Mohm.** It is extrapolated in the report: a pole near 870 Hz, more hum, and an offset of about 140 mV.

Script fix found here: `transient_capture.py` took its "noise" captures with the last pulse setting (1 ms, 1 V) still
running. It now returns the generator to 0 V first. `--only noise --update RUN` re-takes just those captures. The
2026-11-05 50 ohm run was not affected: its noise captures came from an earlier pass at 0 V.
