# Isolated ADS1256 — rev A

This is an isolated 24-bit ADC board, 95 × 62 mm with 4 layers. The ADS1256, its reference, clock and LDOs all sit on the isolated side. SPI crosses a TI ISO7762 (5 kVrms reinforced, 4 forward / 2 reverse channels). Power crosses a Murata MEJ1S0509SC (5 V in, 9 V out, 5.2 kVDC rated, 3 pF isolation capacitance).

Status: generated, routed and checked: ERC 0, DRC 0 at all severities, 0 unconnected, schematic parity 0. It has not been ordered or tested on hardware.

## Channels

| ADS1256 inputs | Function | Scaling | Suggested setup |
|---|---|---|---|
| AIN0 / AIN1 | ±250 VDC differential | Each leg is 3 × 825k + 10k, returned to COM. k = 10k / 2.485 MΩ = 1/248.5, so 250 V gives 1.006 V. **Vin = Vdiff × 248.5** | BUFEN = 1, PGA 4: ±1.25 V full scale = ±310 V |
| AIN2 / AIN3 | 0.1 Ω shunt (RS1, TLMQ2HDR10FTE, 1.5 W) | Kelvin taps through 100 Ω, with 10 nF C0G across the pair. **I = Vdiff / 0.1** | PGA 8: ±625 mV full scale = ±6.25 A. The shunt is rated about 3.8 A continuous. |
| AIN4–7, AINCOM | Broken out on J4 | Each through 1k | — |
| D0–D3 | ADS1256 GPIOs, on J5 (3.3 V logic, isolated) | — | — |

## COM: the isolated reference (read this before connecting anything)

- **COM sits 1.5 V above GNDI.** GNDI is the ADC's own ground. U5 (OPA333) buffers COM, with 47 Ω and 10 µF on its output. This puts every input in the middle of the ADS1256 buffered input range (GNDI to AVDD−2 V, i.e. 0–3 V), so both the shunt and the divider can swing negative relative to COM.
- **COM is the node the measured circuit connects to.** JP1 is bridged by default and ties the shunt's I− end to COM. The shunt therefore defines where the isolated side sits.
- **Do not connect GNDI to the measured circuit as well.** That would short the COM buffer.
- **Common-mode limit:** each HV terminal must stay within about ±370 V of COM. The shunt must stay within about ±1.2 V of COM, which it does when JP1 is bridged.

## Connectors

- **J1 (host):** 5V, GND, VIO, ~SYNC, ~CS, DIN (MOSI), SCLK, ~DRDY, DOUT (MISO), GND.
  - VIO is your logic rail, 2.25–5.5 V.
  - 5 V input current is roughly 80 mA.
  - The isolator outputs default high, so ~CS and ~SYNC idle high when the host is off.
- **J4 (isolated analog):** A4, A5, A6, A7, AINCOM (each through 1k), COM, GNDI, 5VA (AVDD; keep the load at or below 5 mA).
- **J5 (isolated GPIO):** D0, D1, D2, D3, 3V3 (DVDD), GND.
- **J2:** HV+ / HV− screw terminal.
- **J3:** I+ / I− screw terminal.

## Firmware notes

- **~RESET** is held high by a 10k pull-up. Use the RESET command (0xFE) after power-up.
- **D0/CLKOUT** outputs 7.68 MHz after reset. Clear CLK1:0 in ADCON unless you want the clock on J5.
- **Input buffer:** use BUFEN = 1. The divider has about 10 kΩ source impedance, which is too high for the unbuffered input (about 150 kΩ / PGA).

## Isolation

- **Barrier width:** a 5.6 mm copper-free band, set by the MEJ1 module's 5.9 mm pin gap. The ISO7762 has 8 mm internally.
- **HV divider area:** no copper on In1, In2 or B.Cu underneath it. The HV netclass enforces 1.5 mm clearance on F.Cu.
- **Rating:** none of this is a certified insulation rating. Treat the board as basic insulation for a few hundred volts working, and use nylon hardware in the isolated-side mounting holes.

## Files and regenerating

- `tools/design.py` is the single source of truth.
- `tools/make_lib.py` generates `Isolated_ADS1256.kicad_sym` (ADS1256, MEJ1S0509SC, TPS7A4901).
- `tools/gen_sch.py` writes the schematic. `tools/gen_pcb.py` writes the placed, unrouted board. **Running gen_pcb wipes the routing.**
- **Routing flow** (needs a `kicad-cli api-server` running; see `tools/kipc.py`), run in this order:
  1. `stitch_planes.py`
  2. `retry.py --via` for the two LDO thermal pads
  3. `route_pass.py`
  4. `manual_ain.py`: hand-drawn AIN4–7 lanes and Kelvin taps, then +5VA and COM re-routed
  5. `post_silk.py`
  6. a manual U5 GND-to-R6 tie
- Snapshots of each stage are in `build/`.
- Output files: `Isolated_ADS1256_schematic.pdf` and `Isolated_ADS1256_BOM.csv`.
