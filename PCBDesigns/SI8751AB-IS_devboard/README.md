# Si8751 AC/DC development board — revision C

KiCad 10 bench prototype targeting **120 V RMS AC / 170 V DC, less than 1 A**, initially with resistive loads. Two SI8751AB-IS drivers operate back-to-back STP12N60M2 MOSFETs. **AMC3330DWER** measures switched voltage with integrated isolated power; **ACS725LLCTR-05AB-T** measures bidirectional load current. The entire board uses one regulated 3.3 V input. All resistors and capacitors are **0805**. The board is two layers, 110 × 90 mm, for 1.6 mm FR-4 and 35 µm copper.

Open `SI8751AB-IS_devboard.kicad_pro`. Reload schematic and board if an earlier revision is still open. Local symbols and footprints, a schematic PDF, BOM, and validation reports/previews are included. Prototype Gerbers and drills are in `fabrication/revC/`. `revision_B_before_3V3_sensor.zip` preserves revision B; the earlier revision A backup is also retained. The inventory workbook was read without modification; existing project preferences were preserved.

## Changes from revision B

- Replaced ACS712-05B with **ACS725LLCTR-05AB-T**, powered from +3V3.
- Removed the 5 V net. J1-7 is **NC**, preserving all other revision B header pin numbers.
- Removed R15/R16, the 2:1 current-output divider. J1-8 now receives the sensor output directly.
- C17 remains 100 nF; the ACS725 internal filter resistance gives approximately 884 Hz bandwidth. C18 is a direct 1 nF output load.
- Current conversion at nominal supply is now **I = (Vout − 1.65 V) / 0.264 V/A**.
- All passives remain 0805. The ACS725 was not found in the supplied purchase history and is a new procurement item.

## Earlier changes: revision A to B

- Added ACS712-05B between Q2 drain and J3 line. Positive current flows toward the load.
- Replaced AMC0330R with AMC3330. Removed SCT01F03S05, TPS70933 and the converter slot. U3/U4 and C9/C10 references are retired.
- Expanded J1 from 5 to 10 pins. **Pin 5 changed from ground to OUTN.** Voltage-interface firmware scaling also changed.
- Added regulated 5 V input, current filtering, 2:1 current-output attenuation, and AMC diagnostic output.
- Changed voltage divider to stocked 1 MΩ / 15 kΩ parts and EN pull-downs to stocked 75 kΩ parts.
- Corrected plated-pad solder-mask serialization in the generator; both mask openings are verified in the saved board.

## Connections

| Pin | Function |
|---|---|
| J1-1 | Regulated +3.3 V input; provide at least 100 mA capacity |
| J1-2 | Logic ground |
| J1-3 | EN, 3.3 V active high; both MOSFETs commanded together |
| J1-4 | VOUT_P, AMC positive differential output |
| J1-5 | VOUT_N, AMC negative differential output; **not ground** |
| J1-6 | Logic ground |
| J1-7 | NC, no connection; formerly the 5 V input in revision B |
| J1-8 | IOUT_ADC, direct ACS725 output referenced to logic ground |
| J1-9 | DIAG_N, AMC status, 10 kΩ pull-up to 3.3 V; LOW means invalid measurement |
| J1-10 | Logic ground |
| J2-1 / J2-2 | Source line / source return |
| J3-1 / J3-2 | Switched line / load return |

The regulated 3.3 V supply returns to GND_LOGIC, which must remain separate from GND_LOAD_RETURN and SOURCE_FLOAT. The ACS725 requires 3.0–3.6 V; do not power the board from 5 V. Power the host ADC with the board to avoid back-powering an unpowered ADC through the current output. J1-7 is unused; supply power at J1-1.

J2/J3 are Wuerth **691311400102**, 7.62 mm headers. Two **691351400002** cable plugs are additional assembly items outside the fitted PCB BOM. [Wuerth mating series](https://www.we-online.com/en/components/products/TBL_7_62_3514_VERTICAL_69135140000X).

## Switching circuit

```text
J2 LINE -- D Q1 S -- SOURCE_FLOAT -- S Q2 D -- ACS725 IP+ to IP- -- J3 LINE
             G1                         G2
             |                          |
          U1 GATE                    U2 GATE
          U1 SOURCE ---- SOURCE_FLOAT --- U2 SOURCE
J2 RETURN ---------------- GND_LOAD_RETURN -------------------- J3 RETURN
```

Opposing body diodes block either polarity when both gates are off. Driver gate outputs are separate. R1/R3 are 100 Ω input resistors, R2/R4 are 75 kΩ default-off pull-downs, and R5/R6 set TT to 10 kΩ. C3/C4 are 10 pF, 1 kV C0G drain-to-MCAP1 capacitors; MCAP2 is unused. A conventional gate/source bleed resistor would overload the weak generated gate supply and is omitted. [Skyworks datasheet](https://www.skyworksinc.com/-/media/SkyWorks/SL/documents/public/data-sheets/Si8751-2.pdf).

Q1/Q2 are 600 V STP12N60M2, TO-220, pins 1=G, 2=D, 3=S. Tabs are live drains at different potentials; do not attach both directly to one conductive heatsink. At 1 A and 0.45 Ω per MOSFET, combined conduction loss is about 0.9 W at the specified 10 V gate drive. Actual VGS, transition time and dissipation need measurement because driver output is load dependent. Begin at no more than one full on/off cycle per second. [ST part information](https://www.st.com/en/power-transistors/stp12n60m2.html).

## Voltage measurement

U5 receives 3.3 V on its logic side and generates isolated power internally. Its high-side reference connects to load return. OUTP/OUTN have differential gain 2 and need a differential ADC or external difference amplifier. This is not the old ratiometric output. Keep ADC wiring short: direct capacitive load limits are 500 pF per output to ground or 250 pF differential. The ADC must accept approximately 1.44 V output common mode. [TI AMC3330 datasheet](https://www.ti.com/lit/ds/symlink/amc3330.pdf).

R10–R12 are 1 MΩ each; R13 is 15 kΩ, all 0.1%. C14 is 1 nF. Nominal conversion:

```text
divider = (3,000,000 + 15,000) / 15,000 = 201
Vrail = 100.5 × (VOUT_P − VOUT_N)
```

| Calculated quantity | Nominal value |
|---|---:|
| Sense input at 120 V RMS sine | 0.8443 V peak |
| Sense input at 132 V RMS sine (+10% check) | 0.9288 V peak |
| Sense input at 170 V DC | 0.8458 V |
| Divider range for ±1 V linear AMC input | ±201 V instantaneous |
| Each top resistor at 132 V RMS | 61.92 V peak, 1.917 mW average |
| Divider current at 120 V RMS | 39.80 µA RMS |
| Divider / C14 pole | 10.66 kHz |

The 132 V calculation is a margin check, not a higher operating rating. RG2012 top resistors have a 150 V limiting element rating and 0.125 W regular power rating. Three parts share voltage; do not replace them with one 0805. [Susumu RG specification](https://www.susumu.co.jp/common/pdf/e_all.pdf).

C5/C6/C19 bypass DCDC_OUT/HLDO_IN; C7/C8 bypass HLDO_OUT; C11/C12 bypass VDD; C13 bypasses DCDC_IN/LDO_OUT. Use low-ESR ceramics with adequate effective capacitance under DC bias. No external LDO load or ferrite beads are fitted. EMI performance remains unqualified.

DIAG_N LOW invalidates measurements. Startup, loss of power or a zero reading does not establish that the rail is safe to touch. Calibrate zero/gain. For RMS, square calibrated instantaneous samples, average over complete cycles, then take the square root. Subtract the mean only for AC-only RMS.

## Current measurement

U6 is ACS725LLCTR-05AB-T, a 3.3 V, ±5 A bidirectional Hall sensor. Its analog output connects directly to J1-8 with no attenuation. At nominal 3.3 V supply:

```text
IOUT_ADC = 1.65 V + 0.264 V/A × I
I = (IOUT_ADC − 1.65 V) / 0.264 V/A
```

At ±1 A DC, output is 1.386–1.914 V. At 1 A RMS sine it moves ±0.3734 V about zero-current level. Nominal ±5 A span is 0.33–2.97 V, while the board load target remains below 1 A. ADC signal sensitivity is approximately 2.85 times revision B's divided output.

C17=100 nF and the nominal internal 1.8 kΩ resistance give a pole near 884 Hz, approximately 0.23% attenuation and 3.9° phase lag at 60 Hz. C18=1 nF is a direct output load. Total output capacitance, including the cable and ADC, must remain at or below 10 nF; DC load resistance must be at least 4.7 kΩ. Keep the ADC connection short and qualify its sampling-settling behavior. [Allegro ACS725 datasheet](https://www.allegromicro.com/-/media/files/datasheets/acs725-datasheet.ashx).

Calibrate zero with no current and gain against a known current. The formula assumes nominal supply: both zero and sensitivity vary with supply and temperature, so calibration at the actual operating supply is preferred. Do not assume simple ideal ratiometric cancellation removes every gain error. Offset and noise remain significant below 1 A. Characterize or compensate the filter phase response before power-factor or real-power measurements. This circuit monitors current; it does not implement hardware overcurrent shutdown.

The sensor pin functions match the earlier SOIC-8 part: 1/2 IP+, 3/4 IP−, 5 ground, 6 FILTER, 7 output, 8 supply. It cannot be fitted to an unmodified revision B assembly powered at 5 V. Use revision C supply wiring, component population and firmware scaling.

## Inventory and assembly

`BOM_inventory.csv` records MPNs, workbook row numbers, historical quantities and unmatched parts. **28 of 37 fitted references match exact MPNs.** U5 has the stocked automotive alternative **AMC3330QDWERQ1**: its DWE footprint, pin functions, integrated power and gain suit this design. Use its separate error/temperature specifications when qualifying that assembly. [TI automotive variant](https://www.ti.com/lit/ds/symlink/amc3330-q1.pdf).

Source: `C:\Users\Nick\Dropbox (Personal)\AvaliableParts\Combined_parts_DigiKey_and_Mouser.xlsx`, sheet `Combined parts`. Quantities describe purchase history, not verified remaining stock; repeated references consume shared stock.

| Items not found as exact inventory matches | Needed per board |
|---|---:|
| STP12N60M2 | 2 |
| ACS725LLCTR-05AB-T | 1 |
| C0805C100JDGACTU, 10 pF / 1 kV C0G | 2 |
| 691311400102 terminal header | 2 |
| 1×10, 2.54 mm vertical control header | 1 |
| AMC3330DWER, if stocked Q1 alternative is not used | 1 |

Cable plugs, an external fuse/holder and insulating mounting hardware are additional. Stocked 20–50 V MOSFETs are unsuitable for this voltage target.

## Layout and validation

Load-current traces are 0.8–1.0 mm wide; signal branches are 0.3 mm. High-voltage nodes use 0.8 mm copper clearance, the floating gate domain 0.3 mm, and logic-to-field copper 2.5 mm. There is no converter exception or slot. ACS IP+ and IP− are internally joined through a milliohm conductor, so mutual clearance is 0.3 mm; this exception never applies across its isolation barrier. TO-220 pads retain 1.1 mm drills with 1.65 mm copper width, giving 0.89 mm adjacent clearance. Four 3.2 mm mounting holes are included.

See `validation/revC_erc.rpt` and `validation/revC_drc.rpt` for ERC, DRC and schematic/PCB parity. Schematic, copper plot and board render are visually reviewed. These checks establish CAD consistency, not hardware performance or mains certification.

The entire board has **not** been certified for mains service. Use an appropriately isolated, current-limited bench source and an external fuse no greater than 1 A, rated for the actual AC/DC source and prospective fault current. Inductive/capacitive loads, inrush, surges, fault clearing, thermal limits and EMC remain unqualified. Terminals and MOSFET tabs are hazardous when energized.

Begin with no switched source: inspect assembly, check separation of all three ground domains, then apply regulated 3.3 V with a current limit. Check DIAG_N, voltage-output difference near zero, and current output near 1.65 V. Test switching and sensing first using current-limited low-voltage DC with a resistive load. Measure both VGS values and temperatures before increasing voltage. Calibrate voltage/current and evaluate AC response using isolated differential instrumentation. Never connect an earth-referenced scope ground to a live field node.

## Rebuilding

`build_design.py` writes schematic/metadata; `build_pcb.py` places/routes using KiCad Python and NumPy; `make_rules.py` creates rules; `match_bom.py` adds inventory matches using openpyxl. Run ERC/DRC after regeneration. Generators replace CAD contents: preserve manual edits before rerunning. Existing project preferences are preserved.
