"""LightBoard -- single source of truth for the circuit.

4-channel bright white LED board built from parts on hand (AvaliableParts xlsx):
  4 x Samsung LH351D (SPHWHTL3DA0GF4TPQ6, 3535, 3 A max)  driven by
  4 x Microchip MIC4801YM (linear 600 mA current sink, 3.0-5.5 V, PWM on EN), one per LED.

Supply: +5 V on J1 (5.5 V absolute max -- the MIC4801 is a 6 V part).
  Each channel: +5V -> LED anode;  LED cathode (Kn) -> Rdrop (2.7 R) -> DRVn (MIC4801 D1 pins) -> GND.
  Rdrop burns the excess headroom so the SOIC-8 doesn't: at 330 mA, Vf ~2.75 V, the resistor drops 0.9 V
  and leaves ~1.35 V (0.45 W) on the MIC4801.  Fit 0 R instead if running from a 1-cell Li-ion (3.0-4.2 V).
  ILED = 4920 / (RSET[k] - 0.14):  15k -> 331 mA (default), 10k -> 499 mA (heatsink!), 22k -> 225 mA.

Control: SW1 selects EN for all four drivers -- ON (EN pulled to +5V via 1k) or EXT (EN = J2 PWM input via 1k,
  200 Hz - 10 kHz for linear dimming, 3.3 V or 5 V logic; nothing connected = off via R13 150k pull-down).
Reverse-polarity: Q1 (AOSS32334C) low-side N-FET in the return lead.

Each part: ref -> dict(lib, value, fp, pins{pin: net}, pn, desc, dnp, block, stock)
"""

R08 = 'Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder'
R25 = 'Resistor_SMD:R_2512_6332Metric_Pad1.40x3.35mm_HandSolder'
C08 = 'Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder'
LED08 = 'LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder'
SO8 = 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm'
SOT23 = 'Package_TO_SOT_SMD:SOT-23'
LH351 = 'LightBoard:LED_Samsung_LH351D_3.5x3.5mm'
TB = 'TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-2-2.54_1x02_P2.54mm_Horizontal'
SWF = 'Button_Switch_THT:SW_Slide_SPDT_Angled_CK_OS102011MA1Q'
MH = 'MountingHole:MountingHole_3.2mm_M3'

RLIB = 'Device:R_Small_US'
CLIB = 'Device:C_Small'

PARTS = {}


def add(ref, lib, value, fp, pins, pn='', desc='', dnp=False, block='', stock=True):
    assert ref not in PARTS, ref
    PARTS[ref] = dict(lib=lib, value=value, fp=fp, pins=dict(pins), pn=pn, desc=desc, dnp=dnp, block=block,
                      stock=stock)


def R(ref, value, a, b, fp=R08, pn='', desc='', block='', stock=True):
    add(ref, RLIB, value, fp, {'1': a, '2': b}, pn, desc, block=block, stock=stock)


def C(ref, value, a, b, fp=C08, pn='', desc='', block=''):
    add(ref, CLIB, value, fp, {'1': a, '2': b}, pn, desc, block=block)


# ================================================================ POWER IN + CONTROL
B = 'in'
add('J1', 'Connector_Generic:Conn_01x02', '+5V IN', TB, {'1': '+5V', '2': 'GND_IN'}, '1725656',
    'Power in: pin 1 = +5 V, pin 2 = 0 V.  4.5-5.5 V, ~1.4 A at 330 mA/ch (2 A at 500 mA/ch)', block=B)
add('Q1', 'Transistor_FET:AO3400A', 'AOSS32334C', SOT23, {'1': 'QG', '2': 'GND', '3': 'GND_IN'}, 'AOSS32334C',
    'Reverse-polarity protection: low-side N-FET in the return (26 mOhm at Vgs 4.5 V)', block=B)
R('R9', '10k', '+5V', 'QG', pn='RN73H2ATTD1002B25', desc='Q1 gate', block=B)
C('C5', '22uF 25V', '+5V', 'GND', pn='CL21A226MAYNNNE', desc='Bulk input', block=B)
C('C6', '22uF 25V', '+5V', 'GND', pn='CL21A226MAYNNNE', desc='Bulk input', block=B)
R('R10', '1k', '+5V', 'PLED', pn='RR1220P-102-D', desc='Power LED ~3 mA', block=B)
add('D5', 'Device:LED_Small', 'RED', LED08, {'1': 'GND', '2': 'PLED'}, 'LS R976-NR-1-0-20-R18', 'Power indicator', block=B)
add('SW1', 'Switch:SW_SPDT', 'ON / EXT', SWF, {'1': 'EN_ON', '2': 'EN', '3': 'PWM_SW'}, 'OS102011MA1QN1',
    'EN source: ON = always on (full current), EXT = J2 PWM input', block=B)
R('R11', '1k', '+5V', 'EN_ON', pn='RR1220P-102-D', desc='EN pull-up for the ON position', block=B)
add('J2', 'Connector_Generic:Conn_01x02', 'PWM IN', TB, {'1': 'GND', '2': 'PWM_IN'}, '1725656',
    'PWM / enable input: pin 1 = GND, pin 2 = PWM (3.3 or 5 V logic, 200 Hz-10 kHz)', block=B)
R('R12', '1k', 'PWM_IN', 'PWM_SW', pn='RR1220P-102-D', desc='PWM input series resistor', block=B)
R('R13', '150k', 'EN', 'GND', pn='ERA-6AED154V', desc='EN pull-down: off when EXT is selected and nothing is driving J2',
  block=B)
for i in range(1, 5):
    add('H%d' % i, 'Mechanical:MountingHole', 'MountingHole', MH, {}, '', 'M3 - bolt to a heatsink through a thermal pad',
        block=B)

# ================================================================ 4 LED CHANNELS
B = 'led'
for n in range(1, 5):
    add('U%d' % n, 'LightBoard:MIC4801', 'MIC4801YM', SO8,
        {'1': '+5V', '2': 'EN', '3': 'RSET%d' % n, '4': 'GND', '5': 'DRV%d' % n, '6': 'DRV%d' % n,
         '7': 'DRV%d' % n, '8': 'DRV%d' % n}, 'MIC4801YM-TR', 'LED %d current sink' % n, block=B)
    add('D%d' % n, 'LightBoard:LH351D', 'LH351D 4000K', LH351, {'1': 'K%d' % n, '2': '+5V', '3': 'GND'},
        'SPHWHTL3DA0GF4TPQ6', 'Samsung LH351D 4000 K; thermal pad tied to GND copper for heat spreading', block=B)
    R('R%d' % n, '2.7R 2W', 'DRV%d' % n, 'K%d' % n, fp=R25, pn='RC2512FK-7W2R7L',
      desc='Headroom dropper (0.3 W at 330 mA, 0.7 W at 500 mA). NOT IN STOCK - order (LCSC C5843102). 0R for Li-ion supply',
      block=B, stock=False)
    R('R%d' % (n + 4), '15k 0.1%', 'RSET%d' % n, 'GND', pn='RG2012P-153-B-T5',
      desc='RSET: ILED = 4920/(R[k]-0.14) = 331 mA. 10k -> 499 mA (heatsink), 22k -> 225 mA', block=B)
    C('C%d' % n, '2.2uF 16V', '+5V', 'GND', pn='CL21B225KOFNNNE', desc='MIC4801 VIN (>= 2.2 uF per datasheet)', block=B)


def nets():
    out = {}
    for ref, p in PARTS.items():
        for pin, n in p['pins'].items():
            out.setdefault(n, []).append((ref, pin))
    return out


if __name__ == '__main__':
    ns = nets()
    for n in sorted(ns):
        print('%-10s %s' % (n, ' '.join('%s.%s' % x for x in ns[n])))
    single = [n for n, v in ns.items() if len(v) < 2]
    print('parts', len(PARTS), 'nets', len(ns), 'single-pin nets:', single)
