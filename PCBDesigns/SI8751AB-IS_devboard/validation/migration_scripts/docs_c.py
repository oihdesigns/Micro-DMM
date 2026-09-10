from pathlib import Path
p=Path('README.md');s=p.read_text(encoding='utf-8-sig')
s=s.replace('development board — revision B','development board — revision C')
s=s.replace('**ACS712ELCTR-05B-T** measures bidirectional load current.','**ACS725LLCTR-05AB-T** measures bidirectional load current. The entire board uses one regulated 3.3 V input.')
s=s.replace('if revision A is still open','if an earlier revision is still open').replace('fabrication/revB/','fabrication/revC/')
s=s.replace('`revision_A_before_current_sensor.zip` preserves the previous design.','`revision_B_before_3V3_sensor.zip` preserves revision B; the earlier revision A backup is also retained.')
s=s.replace('## Changes from revision A','''## Changes from revision B

- Replaced ACS712-05B with **ACS725LLCTR-05AB-T**, powered from +3V3.
- Removed the 5 V net. J1-7 is **NC**, preserving all other revision B header pin numbers.
- Removed R15/R16, the 2:1 current-output divider. J1-8 now receives the sensor output directly.
- C17 remains 100 nF; the ACS725 internal filter resistance gives approximately 884 Hz bandwidth. C18 is a direct 1 nF output load.
- Current conversion at nominal supply is now **I = (Vout − 1.65 V) / 0.264 V/A**.
- All passives remain 0805. The ACS725 was not found in the supplied purchase history and is a new procurement item.

## Earlier changes: revision A to B''')
s=s.replace('| J1-7 | Regulated +5 V for ACS712, 4.5–5.5 V; provide at least 25 mA capacity |','| J1-7 | NC, no connection; formerly the 5 V input in revision B |')
s=s.replace('| J1-8 | IOUT_ADC, divided current output referenced to logic ground |','| J1-8 | IOUT_ADC, direct ACS725 output referenced to logic ground |')
s=s.replace('Both low-voltage supplies share GND_LOGIC. Neither may connect to GND_LOAD_RETURN or SOURCE_FLOAT. The board does not generate 5 V from 3.3 V. Power the host ADC before relying on measurements; the passive current divider does not prevent back-powering an unpowered ADC.','The regulated 3.3 V supply returns to GND_LOGIC, which must remain separate from GND_LOAD_RETURN and SOURCE_FLOAT. The ACS725 requires 3.0–3.6 V; do not power the board from 5 V. Power the host ADC with the board to avoid back-powering an unpowered ADC through the current output. J1-7 is unused; supply power at J1-1.')
s=s.replace('ACS712 IP+ to IP-','ACS725 IP+ to IP-')
start=s.index('## Current measurement');end=s.index('## Inventory and assembly',start)
s=s[:start]+'''## Current measurement

U6 is ACS725LLCTR-05AB-T, a 3.3 V, ±5 A bidirectional Hall sensor. Its analog output connects directly to J1-8 with no attenuation. At nominal 3.3 V supply:

```text
IOUT_ADC = 1.65 V + 0.264 V/A × I
I = (IOUT_ADC − 1.65 V) / 0.264 V/A
```

At ±1 A DC, output is 1.386–1.914 V. At 1 A RMS sine it moves ±0.3734 V about zero-current level. Nominal ±5 A span is 0.33–2.97 V, while the board load target remains below 1 A. ADC signal sensitivity is approximately 2.85 times revision B's divided output.

C17=100 nF and the nominal internal 1.8 kΩ resistance give a pole near 884 Hz, approximately 0.23% attenuation and 3.9° phase lag at 60 Hz. C18=1 nF is a direct output load. Total output capacitance, including the cable and ADC, must remain at or below 10 nF; DC load resistance must be at least 4.7 kΩ. Keep the ADC connection short and qualify its sampling-settling behavior. [Allegro ACS725 datasheet](https://www.allegromicro.com/-/media/files/datasheets/acs725-datasheet.ashx).

Calibrate zero with no current and gain against a known current. The formula assumes nominal supply: both zero and sensitivity vary with supply and temperature, so calibration at the actual operating supply is preferred. Do not assume simple ideal ratiometric cancellation removes every gain error. Offset and noise remain significant below 1 A. Characterize or compensate the filter phase response before power-factor or real-power measurements. This circuit monitors current; it does not implement hardware overcurrent shutdown.

The sensor pin functions match the earlier SOIC-8 part: 1/2 IP+, 3/4 IP−, 5 ground, 6 FILTER, 7 output, 8 supply. It cannot be fitted to an unmodified revision B assembly powered at 5 V. Use revision C supply wiring, component population and firmware scaling.

'''+s[end:]
s=s.replace('**31 of 39 fitted references match exact MPNs.**','**28 of 37 fitted references match exact MPNs.**')
s=s.replace('| STP12N60M2 | 2 |','| STP12N60M2 | 2 |\n| ACS725LLCTR-05AB-T | 1 |')
s=s.replace('validation/revB_erc.rpt','validation/revC_erc.rpt').replace('validation/revB_drc.rpt','validation/revC_drc.rpt')
s=s.replace('then apply 3.3 V and 5 V with current limits','then apply regulated 3.3 V with a current limit').replace('current output near 1.25 V','current output near 1.65 V')
p.write_text(s,encoding='utf-8')
