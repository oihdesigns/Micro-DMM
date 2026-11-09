"""IsoOpAmpShield -- single source of truth for the circuit.

Arduino Giga R1 shield with three isolated channels, each read differentially on two analog pins:
  * Current   : 10 mOhm shunt -> AMC3301 (+/-250 mV)          -> A4(+)/A5(-)
  * +/-200 V  : 2x499k per leg + 10k sense -> AMC3330 (+/-1 V) -> A0(+)/A1(-)   (Giga HW diff pair)
  * High-Z    : 3-op-amp INA (unity, RG pad) -> /10 diff amp -> AMC3330 -> A3(+)/A2(-) (Giga HW diff pair)
The High-Z front end runs on +/-11 V from a TDK CCG3-12-12DF (+/-12 V, 1.5 kVDC) through
10 uH/22 uF LC filters and TPS7A4901/TPS7A3001 low-noise LDOs.
All three AMC DIAG outputs are wired-OR onto A6 (ISO_FAULT, low = fault).

Each part: ref -> dict(lib, value, fp, pins{pin: net}, fields, dnp, block)
Pins that are absent from `pins` get a no-connect flag.
"""

R08 = 'Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder'
R12 = 'Resistor_SMD:R_1206_3216Metric_Pad1.30x1.75mm_HandSolder'
R20 = 'Resistor_SMD:R_2010_5025Metric_Pad1.40x2.65mm_HandSolder'
C06 = 'Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder'
C08 = 'Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder'
C12 = 'Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder'
FBF = 'Inductor_SMD:L_0402_1005Metric_Pad0.77x0.64mm_HandSolder'
L12 = 'Inductor_SMD:L_1210_3225Metric_Pad1.42x2.65mm_HandSolder'
SO16W = 'Package_SO:SOIC-16W_7.5x10.3mm_P1.27mm'
SO8 = 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm'
HVSSOP = 'Package_SO:HVSSOP-8-1EP_3x3mm_P0.65mm_EP1.57x1.89mm'
TB = 'TerminalBlock_RND:TerminalBlock_RND_205-00232_1x02_P5.08mm_Horizontal'
HDR8 = 'Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical'
CCG = 'IsoOpAmpShield:TDK_CCG3_Dual_THT'
JP3 = 'Jumper:SolderJumper-3_P1.3mm_Bridged12_RoundedPad1.0x1.5mm'
MH = 'IsoOpAmpShield:MountingHole_3.2mm_M3_Giga'

RLIB = 'Device:R_Small_US'
CLIB = 'Device:C_Small'
LLIB = 'Device:L_Small'
FLIB = 'Device:FerriteBead_Small'

PARTS = {}


def add(ref, lib, value, fp, pins, pn='', desc='', dnp=False, block=''):
    assert ref not in PARTS, ref
    PARTS[ref] = dict(lib=lib, value=value, fp=fp, pins=dict(pins), pn=pn, desc=desc, dnp=dnp, block=block)


def R(ref, value, a, b, fp=R08, pn='', desc='', dnp=False, block=''):
    add(ref, RLIB, value, fp, {'1': a, '2': b}, pn, desc, dnp, block)


def C(ref, value, a, b, fp=C06, pn='', desc='', dnp=False, block=''):
    add(ref, CLIB, value, fp, {'1': a, '2': b}, pn, desc, dnp, block)


def AMC(ref, value, pn, pre, hgnd, inp, inn, outp, outn, block):
    """AMC3330/AMC3301 core + TI-recommended decoupling (datasheet Fig. 7-5 / Table 7-3)."""
    add(ref, 'IsoOpAmpShield:AMC3330', value, SO16W, {
        '1': pre + 'DCDC_OUT', '2': hgnd, '3': pre + 'DCDC_OUT', '5': pre + 'HLDO',
        '6': inp, '7': inn, '8': hgnd, '9': 'GND', '10': outn, '11': outp,
        '12': 'VDD_AMC', '13': pre + 'DCDC_IN', '14': 'ISO_FAULT', '15': 'GND', '16': pre + 'DCDC_IN'},
        pn, '', block=block)


# ---------------------------------------------------------------- Giga interface / low side
B = 'giga'
add('J1', 'Connector_Generic:Conn_01x08', 'Giga POWER', HDR8,
    {'4': '+3V3', '5': '+5V', '6': 'GND', '7': 'GND'},
    'TSW-108-07-G-S', 'Giga power header: NC, IOREF, RESET, 3V3, 5V, GND, GND, VIN', block=B)
add('J2', 'Connector_Generic:Conn_01x08', 'Giga A0-A7', HDR8,
    {'1': 'ADC_A0', '2': 'ADC_A1', '3': 'ADC_A2', '4': 'ADC_A3', '5': 'ADC_A4', '6': 'ADC_A5',
     '7': 'ISO_FAULT'},
    'TSW-108-07-G-S', 'Giga analog header A0..A7', block=B)
add('JP1', 'Jumper:SolderJumper_3_Bridged12', 'AMC VDD 3V3|5V', JP3,
    {'1': '+3V3', '2': 'VDD_AMC', '3': '+5V'}, '',
    'AMC low-side supply select. Default 1-2 bridged = 3.3 V (outputs can never exceed the Giga ADC rail)', block=B)
C('C1', '22uF', 'VDD_AMC', 'GND', fp=C08, pn='CL21A226MAYNNNE', block=B)
R('R1', '10k', 'ISO_FAULT', '+3V3', pn='ERJ-6ENF1002V', desc='DIAG wired-OR pull-up (open drain, low = fault)', block=B)
for i in range(1, 4):
    add('H%d' % i, 'Mechanical:MountingHole', 'MountingHole', MH, {}, '', 'Giga/Mega hole (low-voltage side only)', block=B)

# ---------------------------------------------------------------- ADC output RC (low side, at J2)
B = 'adc'
for r, c, src, dst in [('R5', 'C13', 'I_OUTP', 'ADC_A4'), ('R6', 'C14', 'I_OUTN', 'ADC_A5'),
                       ('R12', 'C24', 'V_OUTP', 'ADC_A0'), ('R13', 'C25', 'V_OUTN', 'ADC_A1'),
                       ('R27', 'C37', 'Z_OUTP', 'ADC_A3'), ('R28', 'C38', 'Z_OUTN', 'ADC_A2')]:
    R(r, '100', src, dst, pn='ERJ-6ENF1000V', block=B)
    C(c, '330pF C0G', dst, 'GND', pn='C0603C331J5GACTU', block=B)

# ---------------------------------------------------------------- Current channel (unchanged from DualAMCBoard)
B = 'cur'
add('J3', 'Connector:Screw_Terminal_01x02', 'CURRENT', TB, {'1': 'I_SHP', '2': 'I_SHN'},
    'TBJ05-02-1-G-G', 'Current in/out through the on-board shunt', block=B)
R('R2', '10m', 'I_SHN', 'I_SHP', fp=R20, pn='TLRP2H20WR010FTE', desc='Shunt, 10 mOhm 2 W (+/-25 A linear range of AMC3301)', block=B)
add('FB1', FLIB, 'FerriteBead', FBF, {'1': 'I_SHP', '2': 'I_FP'}, '74269244182', block=B)
add('FB2', FLIB, 'FerriteBead', FBF, {'1': 'I_SHN', '2': 'I_FN'}, '74269244182', block=B)
add('FB3', FLIB, 'FerriteBead', FBF, {'1': 'I_SHN', '2': 'HGND_I'}, '74269244182', block=B)
R('R3', '10', 'I_FP', 'IAMC_P', pn='CRT0805ABY-10R0ELF', block=B)
R('R4', '10', 'I_FN', 'IAMC_N', pn='CRT0805ABY-10R0ELF', block=B)
C('C4', '10nF', 'IAMC_P', 'IAMC_N', fp=C08, pn='C0805X103JCRAC3316', desc='AMC3301 input filter (TI: 10 Ohm + 10 nF)', block=B)
AMC('U1', 'AMC3301DWER', 'AMC3301DWER', 'I_', 'HGND_I', 'IAMC_P', 'IAMC_N', 'I_OUTP', 'I_OUTN', B)
C('C5', '1nF', 'I_DCDC_OUT', 'HGND_I', pn='C0603C102K5RACTU', block=B)
C('C6', '100nF', 'I_DCDC_OUT', 'HGND_I', pn='C0603C104K5RACAUTO', block=B)
C('C7', '1uF', 'I_DCDC_OUT', 'HGND_I', pn='CGA3E1X7R1E105K080AC', block=B)
C('C8', '100nF C0G', 'I_HLDO', 'HGND_I', fp=C12, pn='CGA5L2NP01H104J160AA', block=B)
C('C9', '1nF', 'I_HLDO', 'HGND_I', fp=C12, pn='12065C102KAT2A', block=B)
C('C10', '100nF', 'I_DCDC_IN', 'GND', pn='C0603C104K5RACAUTO', block=B)
C('C11', '1nF', 'VDD_AMC', 'GND', fp=C12, pn='12065C102KAT2A', block=B)
C('C12', '1uF', 'VDD_AMC', 'GND', fp=C12, pn='12063C105KAT2A', block=B)

# ---------------------------------------------------------------- +/-200 V channel
B = 'v200'
add('J4', 'Connector:Screw_Terminal_01x02', '+/-200V', TB, {'1': 'VIN_P', '2': 'VIN_N'},
    'TBJ05-02-1-G-G', '+/-200 V full scale (AMC clips at +/-250 V)', block=B)
R('R7', '499k', 'VIN_P', 'V_MP', pn='ERA-6AED4993V', block=B)
R('R8', '499k', 'V_MP', 'V_DP', pn='ERA-6AED4993V', block=B)
R('R9', '499k', 'VIN_N', 'V_MN', pn='ERA-6AED4993V', block=B)
R('R10', '499k', 'V_MN', 'V_DN', pn='ERA-6AED4993V', block=B)
R('R11', '10k 0.1%', 'V_DP', 'V_DN', pn='RG2012P-103-B-T5', desc='RSNS: 200 V * 10k/2.006M = 0.997 V', block=B)
add('FB4', FLIB, 'FerriteBead', FBF, {'1': 'V_DP', '2': 'VAMC_P'}, '74269244182', block=B)
add('FB5', FLIB, 'FerriteBead', FBF, {'1': 'V_DN', '2': 'VAMC_N'}, '74269244182', block=B)
add('FB6', FLIB, 'FerriteBead', FBF, {'1': 'V_DN', '2': 'HGND_V'}, '74269244182', block=B)
C('C15', '1nF C0G', 'VAMC_P', 'VAMC_N', fp=C08, pn='C0805C102J5GACTU',
  desc='FIXED: was 10 nF. TI AMC3330 Fig 7-2/7-5 = 1 nF; with ~10k source -> ~16 kHz', block=B)
AMC('U2', 'AMC3330QDWERQ1', 'AMC3330QDWERQ1', 'V_', 'HGND_V', 'VAMC_P', 'VAMC_N', 'V_OUTP', 'V_OUTN', B)
C('C16', '1nF', 'V_DCDC_OUT', 'HGND_V', pn='C0603C102K5RACTU', block=B)
C('C17', '100nF', 'V_DCDC_OUT', 'HGND_V', pn='C0603C104K5RACAUTO', block=B)
C('C18', '1uF', 'V_DCDC_OUT', 'HGND_V', pn='CGA3E1X7R1E105K080AC', block=B)
C('C19', '100nF C0G', 'V_HLDO', 'HGND_V', fp=C12, pn='CGA5L2NP01H104J160AA', block=B)
C('C20', '1nF', 'V_HLDO', 'HGND_V', fp=C12, pn='12065C102KAT2A', block=B)
C('C21', '100nF', 'V_DCDC_IN', 'GND', pn='C0603C104K5RACAUTO', block=B)
C('C22', '1nF', 'VDD_AMC', 'GND', fp=C12, pn='12065C102KAT2A', block=B)
C('C23', '1uF', 'VDD_AMC', 'GND', fp=C12, pn='12063C105KAT2A', block=B)

# ---------------------------------------------------------------- High-Z channel
B = 'hiz'
add('J5', 'Connector:Screw_Terminal_01x02', 'HI-Z', TB, {'1': 'ZIN_P', '2': 'ZIN_N'},
    'TBJ05-02-1-G-G', 'High-impedance differential input, +/-10 V FS with RG open', block=B)
R('R14', '100M', 'ZIN_P', 'HGND_Z', fp=R12, pn='HVC1206Z1008KET', desc='Bias return (sets input impedance)', block=B)
R('R15', '100M', 'ZIN_N', 'HGND_Z', fp=R12, pn='HVC1206Z1008KET', desc='Bias return (sets input impedance)', block=B)
R('R16', '100k', 'ZIN_P', 'Z_P', fp=R12, pn='ERJ-8ENF1003V', desc='Input protection (op-amp clamp current limit)', block=B)
R('R17', '100k', 'ZIN_N', 'Z_N', fp=R12, pn='ERJ-8ENF1003V', desc='Input protection (op-amp clamp current limit)', block=B)
C('C26', '10pF C0G', 'Z_P', 'HGND_Z', pn='C0603C100J5GACTU', dnp=True, desc='Optional RF filter', block=B)
C('C27', '10pF C0G', 'Z_N', 'HGND_Z', pn='C0603C100J5GACTU', dnp=True, desc='Optional RF filter', block=B)
add('U3', 'Amplifier_Operational:OPA2197xD', 'OPA2191IDR', SO8,
    {'1': 'Z_PBUF', '2': 'Z_PFB', '3': 'Z_P', '5': 'Z_N', '6': 'Z_NFB', '7': 'Z_NBUF', '8': 'P11', '4': 'N11'},
    'OPA2191IDR', 'INA input stage', block=B)
R('R18', '15k 0.1%', 'Z_PBUF', 'Z_PFB', pn='RG2012P-153-B-T5', block=B)
R('R19', '15k 0.1%', 'Z_NBUF', 'Z_NFB', pn='RG2012P-153-B-T5', block=B)
R('R20', 'RG (DNP)', 'Z_PFB', 'Z_NFB', dnp=True,
  desc='Gain pad: G = 0.1*(1+30k/RG). Open = +/-10 V FS; 30k = +/-5 V; 3.33k = +/-1 V', block=B)
add('U4', 'Amplifier_Operational:OPA2197xD', 'OPA2191IDR', SO8,
    {'1': 'Z_DIFF', '2': 'Z_DN', '3': 'Z_DP', '5': 'HGND_Z', '6': 'U4B_OUT', '7': 'U4B_OUT', '8': 'P11', '4': 'N11'},
    'OPA2191IDR', 'Difference amp, gain 0.1 (B half parked as grounded follower)', block=B)
R('R21', '100k 0.1%', 'Z_NBUF', 'Z_DN', pn='RG2012P-104-B-T5', block=B)
R('R23', '10k 0.1%', 'Z_DN', 'Z_DIFF', pn='RG2012P-103-B-T5', block=B)
R('R22', '100k 0.1%', 'Z_PBUF', 'Z_DP', pn='RG2012P-104-B-T5', block=B)
R('R24', '10k 0.1%', 'Z_DP', 'HGND_Z', pn='RG2012P-103-B-T5', block=B)
R('R25', '4.99k 0.1%', 'Z_DIFF', 'ZAMC_P', pn='RG2012P-4991-B-T5', block=B)
R('R26', '4.99k 0.1%', 'HGND_Z', 'ZAMC_N', pn='RG2012P-4991-B-T5', block=B)
C('C28', '1nF C0G', 'ZAMC_P', 'ZAMC_N', fp=C08, pn='C0805C102J5GACTU', desc='Same ~16 kHz pole as the 200 V channel', block=B)
AMC('U5', 'AMC3330QDWERQ1', 'AMC3330QDWERQ1', 'Z_', 'HGND_Z', 'ZAMC_P', 'ZAMC_N', 'Z_OUTP', 'Z_OUTN', B)
C('C29', '1nF', 'Z_DCDC_OUT', 'HGND_Z', pn='C0603C102K5RACTU', block=B)
C('C30', '100nF', 'Z_DCDC_OUT', 'HGND_Z', pn='C0603C104K5RACAUTO', block=B)
C('C31', '1uF', 'Z_DCDC_OUT', 'HGND_Z', pn='CGA3E1X7R1E105K080AC', block=B)
C('C32', '100nF C0G', 'Z_HLDO', 'HGND_Z', fp=C12, pn='CGA5L2NP01H104J160AA', block=B)
C('C33', '1nF', 'Z_HLDO', 'HGND_Z', fp=C12, pn='12065C102KAT2A', block=B)
C('C34', '100nF', 'Z_DCDC_IN', 'GND', pn='C0603C104K5RACAUTO', block=B)
C('C35', '1nF', 'VDD_AMC', 'GND', fp=C12, pn='12065C102KAT2A', block=B)
C('C36', '1uF', 'VDD_AMC', 'GND', fp=C12, pn='12063C105KAT2A', block=B)
C('C39', '100nF', 'P11', 'HGND_Z', pn='C0603C104K5RACAUTO', block=B)
C('C40', '100nF', 'N11', 'HGND_Z', pn='C0603C104K5RACAUTO', block=B)
C('C41', '100nF', 'P11', 'HGND_Z', pn='C0603C104K5RACAUTO', block=B)
C('C42', '100nF', 'N11', 'HGND_Z', pn='C0603C104K5RACAUTO', block=B)

# ---------------------------------------------------------------- Isolated +/-11 V supply for the High-Z front end
B = 'psu'
add('L1', LLIB, '10uH', L12, {'1': '+5V', '2': 'P5_DC'}, 'LQH32PN100MN0L', 'CCG3 input filter', block=B)
C('C2', '22uF', 'P5_DC', 'GND', fp=C08, pn='CL21A226MAYNNNE', block=B)
C('C3', '100nF', 'P5_DC', 'GND', pn='C0603C104K5RACAUTO', block=B)
add('PS1', 'IsoOpAmpShield:CCG3-12-12DF', 'CCG3-12-12DF', CCG,
    {'1': 'P5_DC', '2': 'GND', '3': 'GND', '4': 'P12_RAW', '5': 'HGND_Z', '6': 'N12_RAW'},
    'CCG3-12-12DF', 'RC tied to -Vin = always on (negative logic)', block=B)
C('C43', '4.7uF', 'P12_RAW', 'HGND_Z', fp=C08, pn='CL21A475KAQNNNG', block=B)
C('C44', '4.7uF', 'HGND_Z', 'N12_RAW', fp=C08, pn='CL21A475KAQNNNG', block=B)
add('L2', LLIB, '10uH', L12, {'1': 'P12_RAW', '2': 'P12_F'}, 'LQH32PN100MN0L', '400 kHz ripple filter', block=B)
add('L3', LLIB, '10uH', L12, {'1': 'N12_RAW', '2': 'N12_F'}, 'LQH32PN100MN0L', '400 kHz ripple filter', block=B)
C('C45', '22uF', 'P12_F', 'HGND_Z', fp=C08, pn='CL21A226MAYNNNE', block=B)
C('C46', '22uF', 'HGND_Z', 'N12_F', fp=C08, pn='CL21A226MAYNNNE', block=B)
add('U7', 'IsoOpAmpShield:TPS7A4901DGN', 'TPS7A4901DGNR', HVSSOP,
    {'8': 'P12_F', '5': 'P12_F', '1': 'P11', '2': 'FB_P', '6': 'NR_P', '4': 'HGND_Z', '9': 'HGND_Z'},
    'TPS7A4901DGNR', '+11 V (1.185*(1+82.5/10) = 10.96 V)', block=B)
R('R29', '82.5k', 'P11', 'FB_P', pn='ERJ-6ENF8252V', block=B)
R('R30', '10k', 'FB_P', 'HGND_Z', pn='ERJ-6ENF1002V', block=B)
C('C47', '10nF C0G', 'P11', 'FB_P', pn='C0603C103J5GACTU', desc='CFF', block=B)
C('C48', '10nF', 'NR_P', 'HGND_Z', pn='C0603C103K5RACTU', desc='CNR/SS', block=B)
C('C49', '22uF', 'P11', 'HGND_Z', fp=C08, pn='CL21A226MAYNNNE', block=B)
add('U8', 'IsoOpAmpShield:TPS7A3001DGN', 'TPS7A3001DGNR', HVSSOP,
    {'8': 'N12_F', '5': 'N12_F', '1': 'N11', '2': 'FB_N', '6': 'NR_N', '4': 'HGND_Z', '9': 'HGND_Z'},
    'TPS7A3001DGNR', '-11 V (-1.176*(1+82.5/10) = -10.88 V)', block=B)
R('R31', '82.5k', 'N11', 'FB_N', pn='ERJ-6ENF8252V', block=B)
R('R32', '10k', 'FB_N', 'HGND_Z', pn='ERJ-6ENF1002V', block=B)
C('C50', '10nF C0G', 'N11', 'FB_N', pn='C0603C103J5GACTU', desc='CFF', block=B)
C('C51', '10nF', 'NR_N', 'HGND_Z', pn='C0603C103K5RACTU', desc='CNR/SS', block=B)
C('C52', '22uF', 'HGND_Z', 'N11', fp=C08, pn='CL21A226MAYNNNE', block=B)

# Island membership (for zones / clearance checks)
ISLANDS = {
    'HGND_I': ['I_SHP', 'I_SHN', 'I_FP', 'I_FN', 'IAMC_P', 'IAMC_N', 'I_DCDC_OUT', 'I_HLDO', 'HGND_I'],
    'HGND_V': ['VIN_P', 'VIN_N', 'V_MP', 'V_MN', 'V_DP', 'V_DN', 'VAMC_P', 'VAMC_N', 'V_DCDC_OUT', 'V_HLDO', 'HGND_V'],
    'HGND_Z': ['ZIN_P', 'ZIN_N', 'Z_P', 'Z_N', 'Z_PBUF', 'Z_PFB', 'Z_NBUF', 'Z_NFB', 'Z_DP', 'Z_DN', 'Z_DIFF',
               'U4B_OUT', 'ZAMC_P', 'ZAMC_N', 'Z_DCDC_OUT', 'Z_HLDO', 'HGND_Z', 'P12_RAW', 'N12_RAW', 'P12_F',
               'N12_F', 'P11', 'N11', 'FB_P', 'FB_N', 'NR_P', 'NR_N'],
}


def nets():
    out = {}
    for ref, p in PARTS.items():
        for pin, n in p['pins'].items():
            out.setdefault(n, []).append((ref, pin))
    return out


if __name__ == '__main__':
    ns = nets()
    for n in sorted(ns):
        print('%-12s %s' % (n, ' '.join('%s.%s' % x for x in ns[n])))
    single = [n for n, v in ns.items() if len(v) < 2]
    print('parts', len(PARTS), 'nets', len(ns), 'single-pin nets:', single)
