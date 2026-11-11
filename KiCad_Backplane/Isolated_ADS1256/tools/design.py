"""Isolated_ADS1256 -- single source of truth for the circuit.

Fully isolated ADS1256 breakout. The whole ADC (with its reference, clock and LDOs) lives on the isolated
side; SPI crosses an ISO7762 (4 fwd / 2 rev) and power crosses a Murata MEJ1S0509SC (5 V -> 9 V, 5.2 kVDC).

Isolated-side grounding: GNDI = ADS1256 AGND/DGND.  COM = buffered 1.5 V above GNDI, and COM is the
reference node the outside world connects to (the shunt's I- end is tied to COM by JP1).  That puts every
input in the middle of the ADS1256's buffered input range (GNDI .. AVDD-2 V = 0 .. 3 V), so the shunt and the
divider can both swing negative relative to COM.

  AIN0/AIN1  +/-250 VDC   symmetric divider: 3 x 825k + 10k per leg, legs return to COM
                          k = 10k / 2.485M = 1/248.5  -> 250 V = 1.006 V differential (PGA 4: +/-1.25 V FS = +/-310 V)
  AIN2/AIN3  shunt        0.1 Ohm 1.5 W (TLMQ2HDR10FTE), Kelvin taps through 100R, 10 nF C0G
                          PGA 8: +/-625 mV FS = +/-6.25 A (shunt rating ~3.8 A continuous)
  AIN4..7, AINCOM         broken out on J4 through 1k
  D0..D3                  ADS1256 GPIOs broken out on J5

Each part: ref -> dict(lib, value, fp, pins{pin: net}, pn, desc, dnp, block)
"""

R06 = 'Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder'
R08 = 'Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder'
R12 = 'Resistor_SMD:R_1206_3216Metric_Pad1.30x1.75mm_HandSolder'
R20 = 'Resistor_SMD:R_2010_5025Metric_Pad1.40x2.65mm_HandSolder'
C06 = 'Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder'
C08 = 'Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder'
FB08 = 'Inductor_SMD:L_0805_2012Metric_Pad1.05x1.20mm_HandSolder'
SO8 = 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm'
SO16W = 'Package_SO:SOIC-16W_7.5x10.3mm_P1.27mm'
SSOP28 = 'Package_SO:SSOP-28_5.3x10.2mm_P0.65mm'
HVSSOP = 'Package_SO:HVSSOP-8-1EP_3x3mm_P0.65mm_EP1.57x1.89mm'
SOT235 = 'Package_TO_SOT_SMD:SOT-23-5'
XTAL = 'Crystal:Crystal_SMD_HC49-SD'
TB = 'TerminalBlock_RND:TerminalBlock_RND_205-00232_1x02_P5.08mm_Horizontal'
MEJ = 'Converter_DCDC:Converter_DCDC_muRata_MEJ1SxxxxSC_THT'
JP2 = 'Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm'
MH = 'MountingHole:MountingHole_3.2mm_M3'


def HDR(n):
    return 'Connector_PinHeader_2.54mm:PinHeader_1x%02d_P2.54mm_Vertical' % n


RLIB = 'Device:R_Small_US'
CLIB = 'Device:C_Small'
FLIB = 'Device:FerriteBead_Small'

PARTS = {}


def add(ref, lib, value, fp, pins, pn='', desc='', dnp=False, block=''):
    assert ref not in PARTS, ref
    PARTS[ref] = dict(lib=lib, value=value, fp=fp, pins=dict(pins), pn=pn, desc=desc, dnp=dnp, block=block)


def R(ref, value, a, b, fp=R06, pn='', desc='', dnp=False, block=''):
    add(ref, RLIB, value, fp, {'1': a, '2': b}, pn, desc, dnp, block)


def C(ref, value, a, b, fp=C06, pn='', desc='', dnp=False, block=''):
    add(ref, CLIB, value, fp, {'1': a, '2': b}, pn, desc, dnp, block)


def LDO(ref, vin, vout, fb, nr, gnd, rtop, rref, cref, nref, coref, cinref, vtxt, pn_rtop, block):
    add(ref, 'Isolated_ADS1256:TPS7A4901DGN', 'TPS7A4901DGNR', HVSSOP,
        {'8': vin, '5': vin, '1': vout, '2': fb, '6': nr, '4': gnd, '9': gnd},
        'TPS7A4901DGNR', vtxt, block=block)
    R(rref[0], rtop, vout, fb, pn=pn_rtop, block=block)
    R(rref[1], '10k', fb, gnd, pn='ERJ-3EKF1002V', block=block)
    C(cref, '10nF C0G', vout, fb, pn='GRM1885C1H103JA01D', desc='CFF', block=block)
    C(nref, '10nF', nr, gnd, pn='GRM188R71H103KA01D', desc='CNR/SS', block=block)
    C(coref, '10uF', vout, gnd, fp=C08, pn='GRM21BR61E106KA73L', block=block)
    C(cinref, '1uF 25V', vin, gnd, pn='GRM188R61E105KA12D', block=block)


# ================================================================ HOST SIDE (GND, +5V, VIO)
B = 'host'
add('J1', 'Connector_Generic:Conn_01x10', 'HOST', HDR(10),
    {'1': '+5V', '2': 'GND', '3': 'VIO', '4': 'SYNC_H', '5': 'CS_H', '6': 'DIN_H', '7': 'SCLK_H',
     '8': 'DRDY_H', '9': 'DOUT_H', '10': 'GND'},
    'PH1-10-UA', 'Host: 5V, GND, VIO (2.25-5.5 V logic), ~SYNC, ~CS, DIN(MOSI), SCLK, ~DRDY, DOUT(MISO), GND', block=B)
add('FB1', FLIB, '330R@100MHz', FB08, {'1': '+5V', '2': '+5V_F'}, 'BLM21PG331SN1D', 'DC/DC input filter', block=B)
C('C1', '10uF', '+5V_F', 'GND', fp=C08, pn='GRM21BR61E106KA73L', block=B)
C('C2', '100nF', '+5V_F', 'GND', pn='GRM188R71H104KA93D', block=B)
add('PS1', 'Isolated_ADS1256:MEJ1S0509SC', 'MEJ1S0509SC', MEJ,
    {'1': '+5V_F', '2': 'GND', '7': '+9VI', '5': 'GNDI'},
    'MEJ1S0509SC', '5.2 kVDC isolated 5 V -> 9 V (unregulated, keep >= 11 mA load; max 10 uF on output)', block=B)
add('U1', 'Isolator:ISO7762DW', 'ISO7762DWR', SO16W,
    {'1': 'VIO', '2': 'SYNC_H', '3': 'CS_H', '4': 'DIN_H', '5': 'SCLK_H', '6': 'DRDY_H', '7': 'DOUT_H', '8': 'GND',
     '9': 'GNDI', '10': 'DOUT', '11': 'DRDY', '12': 'SCLK_I', '13': 'DIN', '14': 'CS', '15': 'SYNC', '16': '+3V3I'},
    'ISO7762DWR', '5 kVrms reinforced 4F/2R digital isolator, default-high outputs (CS/SYNC idle high)', block=B)
C('C3', '100nF', 'VIO', 'GND', pn='GRM188R71H104KA93D', block=B)
C('C4', '1uF', 'VIO', 'GND', pn='GRM188R61E105KA12D', block=B)
C('C5', '100nF', '+3V3I', 'GNDI', pn='GRM188R71H104KA93D', block=B)
C('C6', '1uF', '+3V3I', 'GNDI', pn='GRM188R61E105KA12D', block=B)
for i in range(1, 5):
    add('H%d' % i, 'Mechanical:MountingHole', 'MountingHole', MH, {}, '', 'M3, non-plated (use nylon hardware on the isolated side)', block=B)

# ================================================================ ISOLATED SUPPLIES
B = 'psu'
C('C7', '4.7uF 25V', '+9VI', 'GNDI', fp=C08, pn='GRM21BR61E475KA12L', desc='MEJ1 output (total <= 10 uF)', block=B)
C('C8', '100nF', '+9VI', 'GNDI', pn='GRM188R71H104KA93D', block=B)
LDO('U2', '+9VI', '+5VA', 'FB_A', 'NR_A', 'GNDI', '32.4k', ('R1', 'R2'), 'C9', 'C10', 'C11', 'C12',
    'AVDD 5.02 V = 1.185*(1+32.4k/10k)', 'ERJ-3EKF3242V', B)
LDO('U3', '+9VI', '+3V3I', 'FB_D', 'NR_D', 'GNDI', '17.8k', ('R3', 'R4'), 'C13', 'C14', 'C15', 'C16',
    'DVDD 3.29 V = 1.185*(1+17.8k/10k)', 'ERJ-3EKF1782V', B)
add('U4', 'Reference_Voltage:REF5025AD', 'REF5025AIDR', SO8, {'2': '+5VA', '4': 'GNDI', '6': 'VREF', '5': 'NR_REF'},
    'REF5025AIDR', '2.5 V, 3 ppm/C, low-noise reference for VREFP', block=B)
C('C17', '1uF', '+5VA', 'GNDI', pn='GRM188R61E105KA12D', block=B)
C('C18', '1uF', 'NR_REF', 'GNDI', pn='GRM188R61E105KA12D', desc='REF5025 TRIM/NR noise filter', block=B)
C('C19', '10uF', 'VREF', 'GNDI', fp=C08, pn='GRM21BR61E106KA73L', desc='VREFP-VREFN 10 uF + 100 nF (datasheet)', block=B)
C('C20', '100nF', 'VREF', 'GNDI', pn='GRM188R71H104KA93D', block=B)
# COM = 1.5 V buffered mid-range
R('R5', '10k', 'VREF', 'VCM_SET', pn='ERJ-3EKF1002V', block=B)
R('R6', '15k', 'VCM_SET', 'GNDI', pn='ERJ-3EKF1502V', desc='VCM = 2.5 V * 15k/25k = 1.5 V', block=B)
C('C21', '1uF', 'VCM_SET', 'GNDI', pn='GRM188R61E105KA12D', block=B)
add('U5', 'Amplifier_Operational:OPA333xxDBV', 'OPA333AIDBVR', SOT235,
    {'3': 'VCM_SET', '4': 'COM_DRV', '1': 'COM_DRV', '5': '+5VA', '2': 'GNDI'},
    'OPA333AIDBVR', 'COM buffer (1.5 V above GNDI)', block=B)
C('C22', '100nF', '+5VA', 'GNDI', pn='GRM188R71H104KA93D', block=B)
R('R7', '47', 'COM_DRV', 'COM', pn='ERJ-3EKF47R0V', desc='Isolates the buffer from C23', block=B)
C('C23', '10uF', 'COM', 'GNDI', fp=C08, pn='GRM21BR61E106KA73L', desc='COM reservoir: soaks up common-mode transients', block=B)

# ================================================================ ADS1256
B = 'adc'
add('U6', 'Isolated_ADS1256:ADS1256', 'ADS1256IDBR', SSOP28,
    {'1': '+5VA', '2': 'GNDI', '3': 'GNDI', '4': 'VREF', '5': 'AINCOM', '6': 'AIN0', '7': 'AIN1', '8': 'AIN2',
     '9': 'AIN3', '10': 'AIN4', '11': 'AIN5', '12': 'AIN6', '13': 'AIN7', '14': 'SYNC', '15': 'RESET',
     '16': '+3V3I', '17': 'GNDI', '18': 'XTAL2', '19': 'XTAL1', '20': 'CS', '21': 'DRDY', '22': 'DOUT',
     '23': 'DIN', '24': 'SCLK', '25': 'D0', '26': 'D1', '27': 'D2', '28': 'D3'},
    'ADS1256IDBR', '', block=B)
C('C24', '10uF', '+5VA', 'GNDI', fp=C08, pn='GRM21BR61E106KA73L', desc='AVDD', block=B)
C('C25', '100nF', '+5VA', 'GNDI', pn='GRM188R71H104KA93D', desc='AVDD', block=B)
C('C26', '1uF', '+3V3I', 'GNDI', pn='GRM188R61E105KA12D', desc='DVDD', block=B)
C('C27', '100nF', '+3V3I', 'GNDI', pn='GRM188R71H104KA93D', desc='DVDD', block=B)
R('R8', '10k', 'RESET', '+3V3I', pn='ERJ-3EKF1002V', desc='RESET held high; reset with the RESET command', block=B)
R('R9', '33', 'SCLK_I', 'SCLK', pn='ERJ-3EKF33R0V', desc='SCLK edge damping', block=B)
add('Y1', 'Device:Crystal', '7.68MHz', XTAL, {'1': 'XTAL1', '2': 'XTAL2'}, 'ECS-76.8-18-5PXEN-TR',
    '7.68 MHz, 18 pF load, HC-49/US SMD', block=B)
C('C28', '18pF C0G', 'XTAL1', 'GNDI', pn='GRM1885C1H180JA01D', block=B)
C('C29', '18pF C0G', 'XTAL2', 'GNDI', pn='GRM1885C1H180JA01D', block=B)

# ================================================================ +/-250 V divider (AIN0/AIN1)
B = 'hv'
add('J2', 'Connector:Screw_Terminal_01x02', 'HV +/-250V', TB, {'1': 'HV_P', '2': 'HV_N'},
    'TBJ05-02-1-G-G', 'Differential voltage input, +/-250 VDC (each terminal within ~+/-370 V of COM)', block=B)
for leg, chain, rb in (('P', ('R10', 'R11', 'R12'), 'R13'), ('N', ('R14', 'R15', 'R16'), 'R17')):
    nodes = ['HV_' + leg, 'HV_%sA' % leg, 'HV_%sB' % leg, 'AIN0' if leg == 'P' else 'AIN1']
    for i, r in enumerate(chain):
        R(r, '825k 0.1%', nodes[i], nodes[i + 1], fp=R12, pn='ERA-8AEB8253V', desc='1206 thin film, 150 V max each', block=B)
    R(rb, '10k 0.1%', nodes[3], 'COM', fp=R08, pn='ERA-6AEB103V', desc='Divider bottom: k = 10k/2.485M', block=B)
C('C30', '10nF C0G', 'AIN0', 'AIN1', fp=C08, pn='GRM2195C1H103JA01D', desc='Differential filter, ~800 Hz', block=B)

# ================================================================ Shunt (AIN2/AIN3)
B = 'shunt'
add('J3', 'Connector:Screw_Terminal_01x02', 'SHUNT I+ I-', TB, {'1': 'ISH_P', '2': 'ISH_N'},
    'TBJ05-02-1-G-G', 'Current through the on-board 0.1 Ohm shunt (I- is COM when JP1 is bridged)', block=B)
R('RS1', '0.1 1.5W', 'ISH_P', 'ISH_N', fp=R20, pn='TLMQ2HDR10FTE', desc='0.1 Ohm 1%, 1.5 W 2010: ~3.8 A continuous', block=B)
R('R18', '100', 'ISH_P', 'AIN2', pn='ERA-3AEB101V', desc='Kelvin tap', block=B)
R('R19', '100', 'ISH_N', 'AIN3', pn='ERA-3AEB101V', desc='Kelvin tap', block=B)
C('C31', '10nF C0G', 'AIN2', 'AIN3', fp=C08, pn='GRM2195C1H103JA01D', desc='Differential filter, ~80 kHz', block=B)
add('JP1', 'Jumper:SolderJumper_2_Bridged', 'I- to COM', JP2, {'1': 'ISH_N', '2': 'COM'}, '',
    'Ties the shunt low side to COM (default bridged). Cut only if COM is tied elsewhere.', block=B)

# ================================================================ Breakouts
B = 'brk'
add('J4', 'Connector_Generic:Conn_01x08', 'ANALOG', HDR(8),
    {'1': 'X_AIN4', '2': 'X_AIN5', '3': 'X_AIN6', '4': 'X_AIN7', '5': 'X_AINCOM', '6': 'COM', '7': 'GNDI', '8': '+5VA'},
    'PH1-08-UA', 'Isolated analog breakout: AIN4-7, AINCOM (each via 1k), COM, GNDI, AVDD (<= 5 mA)', block=B)
for r, n in [('R20', 'AIN4'), ('R21', 'AIN5'), ('R22', 'AIN6'), ('R23', 'AIN7'), ('R24', 'AINCOM')]:
    R(r, '1k', 'X_' + n, n, pn='ERJ-3EKF1001V', desc='Input protection / current limit', block=B)
add('J5', 'Connector_Generic:Conn_01x06', 'GPIO', HDR(6),
    {'1': 'D0', '2': 'D1', '3': 'D2', '4': 'D3', '5': '+3V3I', '6': 'GNDI'},
    'PH1-06-UA', 'ADS1256 GPIO D0-D3 (isolated, DVDD logic). D0 = CLKOUT at power-up', block=B)

HV_NETS = ['HV_P', 'HV_N', 'HV_PA', 'HV_NA', 'HV_PB', 'HV_NB']


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
