# Giga_SimultaneousADC

Can the Arduino Giga R1 (STM32H747) sample a pin pair at one instant, like
`120VMonitor` does on the Feather STM32F405? **Yes, on paper.** This sketch
tests it on the hardware.

## The short answer

| | Feather F405 (120VMonitor fw 1.4) | Giga R1 (STM32H747) |
|---|---|---|
| Dual-ADC pair | ADC1 + ADC2 (`ADC123_COMMON`, MULTI = 5) | ADC1 + ADC2 (`ADC12_COMMON`, DUAL = 00101) |
| Mode | injected simultaneous, software start | the same |
| Results | JDR1/JDR2 on each ADC | JDR1..JDR4 on each ADC |
| Resolution | 12 bit | 16 bit |
| One conversion | ~1.9 us (28 + 12 cyc @ 21 MHz) | ~0.83 us (16.5 + 8.5 cyc @ 30 MHz) |
| V -> I lag | one conversion | one conversion, **or 0 with true differential inputs** |

Which Giga pins can go on which ADC (from the H747 datasheet):

| Giga | MCU | ADC channel | Notes |
|---|---|---|---|
| A0 | PC4 | ADC12_INP4 | differential ch 4 (+) |
| A1 | PC5 | ADC12_INP8 | = INN4, differential ch 4 (-) |
| A2 | PB0 | ADC12_INP9 | = INN5, differential ch 5 (-) |
| A3 | PB1 | ADC12_INP5 | differential ch 5 (+) |
| A4 | PC3 | ADC12_INP13 | |
| A5 | PC2 | ADC12_INP12 | |
| A6 | PC0 | ADC123_INP10 | |
| A7 | PA0 | ADC1_INP16 | **ADC1 only** |
| A8 / A9 | PC2_C / PC3_C | ADC3 | **not in the dual pair** |

So A0 to A6 can sit on either ADC, and any two of them make a simultaneous
pair. A0/A1 and A3/A2 are also true differential pairs. With channel 4 on
ADC1 and channel 5 on ADC2, both pairs convert at **the same instant**, which
the F405 can't do. That is the `DIFF` mode below. The catch is that the H7's
differential mode wants the input common mode near VREF/2. An isolation amp's
output common mode (often ~1.44 V) may need checking against the datasheet
limit.

## What the sketch does

It uses the same pins as 120VMonitor, `VP=A0 VN=A1 IP=A3 IN=A2`, and the ADCs
are driven directly from their registers. It never calls `analogRead()`, so
the core doesn't get involved. It runs three modes:

- **SIM**: ADC1 = VP, IP and ADC2 = VN, IN, in dual injected simultaneous mode. This is the F405 build's arrangement.
- **SEQ**: ADC1 converts VP, VN, IP, IN one after another. It is there for contrast, and it also proves the sign of the measurement.
- **DIFF**: ADC1 = A0-A1 (differential ch 4) and ADC2 = A3-A2 (differential ch 5), in dual simultaneous mode.

**The test signal** comes from the Giga's own DAC, so no extra equipment is needed:
- A12 (DAC0) is a hardware triangle wave, 0.83 to 2.47 V at about 244 Hz. TIM6 paces it, so it uses no CPU.
- A13 (DAC1) is a steady mid-rail DC level.

**How it measures timing:**
1. Each scan is timestamped with the cycle counter.
2. A block of scans is sine-fitted at the fundamental frequency.
3. Two pins carrying the *same* signal show a phase difference, and that difference is how far apart in time the two pins were sampled.

**Expected results:**
- SIM: about 0 ns within each pair.
- SEQ: one conversion per rank, about 0.83 µs.

Random jitter between scans stops the DAC's 1 µs staircase from locking to the
scan rate and biasing the result.

## Wiring

- **SIM / SEQ** (the default boot test): jumper **A12 to A0, A1, A2 and A3**.
- **DIFF**: jumper **A12 to A0 and A3**, and **A13 to A1 and A2**.

You can also use an external sine, from a function generator or the isolation
amps on mains. Send `!DAC,OFF`. For 60 Hz, also send `!SPACE,10` so that one
block spans about 5 cycles. Keep every input between 0 and 3.3 V.

## Running it

1. Flash it as a normal M7 sketch (`arduino:mbed_giga:giga`).
2. Open the serial monitor at any baud rate. On boot it prints the status and runs `!CMP`, which is SIM, then SEQ, then a verdict. A passing run should end something like this (the numbers are illustrative, not measured):

```
pair skew VP->VN:  SIM 0.4 ns   SEQ 833.6 ns
VERDICT: SIMULTANEOUS. ADC1/ADC2 pair halves land within 0.6 ns of each other; ...
```

Things to check on the first run:
- **`scan ... us` against `expected conversion`.** The ADC clock is set to HCLK/4, and the sketch assumes rev-V silicon's extra /2, which gives 30 MHz. If the measured time per rank is about half the expected time, the clock is 60 MHz and over spec, so reduce it.
- **`conversion timeouts`** in SIM or DIFF mean ADC2 never started alongside ADC1. In that case dual mode isn't working.
- **`pair difference rms`** is the practical result. With a moving common-mode signal, SIM should show only noise on VP-VN, while SEQ shows slew × 0.83 µs.

Commands: `!CMP !RUN !MODE,SIM|SEQ|DIFF !AUTO,ms !DAC,hz|DC,code|OFF !N,scans
!SMP,0..7 !SPACE,us !JIT,us !FREQ,hz !IRQ,0|1 !RAW,n !STATUS !HELP`.
`$SKEW,<mode>,<f>,<lag1>,<lag2>,<lag3>,<scan_us>,<timeouts>` is the
machine-readable result line. Each lag is in ns, measured against the first channel.

## Status

Compiles (mbed_giga 4.4.1: 139 KB flash, 148 KB RAM). **Verified on hardware 2026-11-04**
using a 10 kHz sine from the function generator on A0 and A1 (A2/A3 floating), over three runs:

| | SIM (A0 on ADC1, A1 on ADC2) | SEQ (both on ADC1) |
|---|---|---|
| A0 -> A1 sample skew | -1.4 to -2.6 ns | 830 to 835 ns |
| A0 - A1 difference rms | ~35 counts (noise) | ~790 counts (slew x 830 ns) |

With all four pins on the same 10 kHz sine (three runs):

| | SIM | SEQ |
|---|---|---|
| VP -> VN (A0/A1) | -2.0 to -2.8 ns | 830 to 832 ns |
| IP -> IN (A3/A2) | -1.9 to -2.6 ns | 830 to 832 ns |
| V -> I (A0 -> A3) | 828 to 835 ns (one conversion) | 1667 to 1668 ns (two) |
| pair difference rms | ~34 counts | ~780 counts |

DIFF mode (10 kHz sine on A0 and A3, A1 and A2 at DC 1.65 V from the DACs, using `!DAC,DC,2048`
so that DAC0 isn't a triangle), three runs: **V -> I lag 0.0 to 0.4 ns**. Both pairs convert at one
instant. V and I agree to 0.04 % in amplitude and ~14 counts in offset. A differential code is
32768 x (Vp - Vn) / VREF, which is half the single-ended counts per volt, because the range is +/-VREF.

- The rev-V ADC clock assumption is confirmed: 830 ns = 25 cycles at 30 MHz. The extra ~0.2 us per
  rank in the scan time is software overhead (register polling), not conversion time.
- A floating pin shows a large fake "signal" (the previous channel's charge left on the sampling
  capacitor). The verdict now only uses pairs whose two halves have matching fitted amplitudes.

## Input bandwidth (measured 2026-11-04)

`bandwidth_sweep.py` steps the Rigol DG852 Pro (2 Vpp, 1.65 V offset, high-Z) from 1 kHz to 50 MHz
into A0 and A3, with A1/A2 on the DACs. At each step it reads the amplitude from each block's RMS,
which doesn't depend on sample timing, so it stays valid far above Nyquist. Each value is relative to 10 kHz:

| f | SIM (single-ended) | DIFF |
|---|---|---|
| 100 kHz | +0.05 dB | -0.03 dB |
| 500 kHz | -0.16 dB | -0.15 dB |
| 1 MHz | -0.26 dB | -0.34 dB |
| 3 MHz | -1.26 dB | -1.4 dB |
| 5 MHz | -2.65 dB | -3.4 dB |
| 10 MHz | -6.3 dB | -7.5 dB |
| 20 MHz | -11.6 dB | -13.1 dB |
| 50 MHz | -20.5 dB | -22.2 dB |

The response is one clean RC pole:
- Single-ended: **-3 dB at about 5.5 MHz** (tau about 29 ns).
- Differential: **about 4.6 MHz**.
- Channels and ADCs match within about 0.1 dB.

The test doesn't tell the board's input path apart from the ADC's own switch and capacitor. Either way, the
input doesn't filter anything below the sample rate: everything up to tens of MHz aliases straight in,
so the anti-alias filtering has to come from the front end (the isolation amps). DC pins fed by the DACs
pick up about -35 dB crosstalk around 20 MHz, and less than -66 dB below 1 MHz.

## Other route: AdvancedADCDual

The `Arduino_AdvancedAnalog` library (1.5.0, installed) has `AdvancedADCDual`,
which is a different route to simultaneous sampling:
- It uses ADC1 + ADC2 in **regular** simultaneous mode, timer-paced, with DMA into buffers.
- It is gap-free at a fixed sample rate, but you only get buffers, not one scan on demand.
- See the library's `examples/Advanced/ADC_Dual_Mode`.

The register-level injected route here keeps 120VMonitor's `adcScan()` shape:
one start and polled results, paced by the cycle counter. That shape is what
the metering and capture loops are built around.
