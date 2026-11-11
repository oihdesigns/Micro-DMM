"""Build Isolated_ADS1256.kicad_sym: ADS1256 (SSOP-28), Murata MEJ1S0509SC, TPS7A4901DGN (copied from IsoOpAmpShield)."""
import os
import sexp
from sexp import Q

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'Isolated_ADS1256.kicad_sym')
ISO_LIB = os.path.join(HERE, '..', '..', 'IsoOpAmpShield', 'IsoOpAmpShield.kicad_sym')
F = ['effects', ['font', ['size', 1.27, 1.27]]]
FH = ['effects', ['font', ['size', 1.27, 1.27]], ['hide', 'yes']]


def prop(k, v, x=0, y=0, hide=False):
    return ['property', Q(k), Q(v), ['at', x, y, 0], FH if hide else F]


def pin(etype, num, name, x, y, ang, length=2.54):
    return ['pin', etype, 'line', ['at', x, y, ang], ['length', length],
            ['name', Q(name), F], ['number', Q(num), F]]


def symbol(name, ref, value, fp, ds, desc, box, pins, kw=''):
    x0, y0, x1, y1 = box
    s = ['symbol', Q(name), ['exclude_from_sim', 'no'], ['in_bom', 'yes'], ['on_board', 'yes'],
         prop('Reference', ref, x0, y0 + 1.27), prop('Value', value, x0, y1 - 1.27),
         prop('Footprint', fp, 0, y1 - 3.81, True), prop('Datasheet', ds, 0, 0, True),
         prop('Description', desc, 0, 0, True)]
    if kw:
        s.append(prop('ki_keywords', kw, 0, 0, True))
    s.append(['symbol', Q(name + '_0_1'), ['rectangle', ['start', x0, y0], ['end', x1, y1],
                                         ['stroke', ['width', 0.254], ['type', 'default']], ['fill', ['type', 'background']]]])
    s.append(['symbol', Q(name + '_1_1')] + pins)
    return s


# ---------------------------------------------------------------- ADS1256 (DB / SSOP-28)
L, R = -17.78, 17.78
left = [  # (num, name, type)  analog side, top -> bottom
    ('1', 'AVDD', 'power_in'), ('4', 'VREFP', 'input'), ('3', 'VREFN', 'input'), None,
    ('6', 'AIN0', 'input'), ('7', 'AIN1', 'input'), ('8', 'AIN2', 'input'), ('9', 'AIN3', 'input'),
    ('10', 'AIN4', 'input'), ('11', 'AIN5', 'input'), ('12', 'AIN6', 'input'), ('13', 'AIN7', 'input'),
    ('5', 'AINCOM', 'input'), None, ('2', 'AGND', 'power_in')]
right = [  # digital side
    ('16', 'DVDD', 'power_in'), None, ('20', '~{CS}', 'input'), ('24', 'SCLK', 'input'), ('23', 'DIN', 'input'),
    ('22', 'DOUT', 'tri_state'), ('21', '~{DRDY}', 'output'), ('14', '~{SYNC}/~{PDWN}', 'input'),
    ('15', '~{RESET}', 'input'), ('19', 'XTAL1/CLKIN', 'input'), ('18', 'XTAL2', 'output'),
    ('25', 'D0/CLKOUT', 'bidirectional'), ('26', 'D1', 'bidirectional'), ('27', 'D2', 'bidirectional'),
    ('28', 'D3', 'bidirectional'), None, ('17', 'DGND', 'power_in')]
pins = []
top = 20.32
for i, p in enumerate(left):
    if p:
        pins.append(pin(p[2], p[0], p[1], L - 2.54, top - i * 2.54, 0))
for i, p in enumerate(right):
    if p:
        pins.append(pin(p[2], p[0], p[1], R + 2.54, top - i * 2.54, 180))
ads = symbol('ADS1256', 'U', 'ADS1256IDBR', 'Package_SO:SSOP-28_5.3x10.2mm_P0.65mm',
             'https://www.ti.com/lit/ds/symlink/ads1256.pdf',
             'Very low noise 24-bit delta-sigma ADC, 30 kSPS, 8-ch mux, PGA 1-64, SPI, SSOP-28',
             (L, top + 2.54, R, top - len(right) * 2.54), pins, 'ADC 24bit SPI')

# ---------------------------------------------------------------- MEJ1S0509SC
mej = symbol('MEJ1S0509SC', 'PS', 'MEJ1S0509SC', 'Converter_DCDC:Converter_DCDC_muRata_MEJ1SxxxxSC_THT',
             'https://www.murata.com/products/productdata/8807029768222/kdc-mej1.pdf',
             'Murata MEJ1 1 W isolated DC/DC, 5 V in, 9 V / 111 mA out (unregulated), 5.2 kVDC, 3 pF, SIP-7 (pins 1,2,5,7)',
             (-7.62, 5.08, 7.62, -5.08),
             [pin('power_in', '1', '+VIN', -10.16, 2.54, 0), pin('power_in', '2', '-VIN', -10.16, -2.54, 0),
              pin('power_out', '7', '+VOUT', 10.16, 2.54, 180), pin('power_out', '5', '-VOUT', 10.16, -2.54, 180)],
             'isolated dc-dc 5.2kV')

# ---------------------------------------------------------------- TPS7A4901DGN from IsoOpAmpShield
iso = sexp.parse(open(ISO_LIB, encoding='utf-8').read())[0]
tps = [s for s in sexp.find(iso, 'symbol') if s[1] == 'TPS7A4901DGN'][0]

root = ['kicad_symbol_lib', ['version', 20241209], ['generator', Q('kicad_symbol_editor')],
        ['generator_version', Q('9.0')], ads, mej, tps]
open(OUT, 'w', encoding='utf-8', newline='').write(sexp.dump(root) + '\n')
print('wrote', os.path.abspath(OUT))
