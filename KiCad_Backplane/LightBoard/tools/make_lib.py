"""Build LightBoard.kicad_sym (MIC4801, Samsung LH351D) and LightBoard.pretty/LED_Samsung_LH351D_3.5x3.5mm.

MIC4801YM (SOIC-8): 1 VIN, 2 EN, 3 RSET, 4 GND, 5-8 D1 (LED cathode / current sink).
LH351D 3535: anode strip, cathode strip, electrically isolated centre thermal pad.
  Land pattern from the Samsung LH351D datasheet Rev 9.1 p.16 "Recommended Soldering Pattern":
  3.3 x 0.5 anode / cathode strips at +/-1.4 mm (each with a 0.5 x 0.4 mm centre tab outward),
  3.3 x 1.3 mm thermal pad in the middle, 0.5 mm gaps.
"""
import os
import uuid
import sexp
from sexp import Q

HERE = os.path.dirname(os.path.abspath(__file__))
SYM_OUT = os.path.join(HERE, '..', 'LightBoard.kicad_sym')
FP_DIR = os.path.join(HERE, '..', 'LightBoard.pretty')
F = ['effects', ['font', ['size', 1.27, 1.27]]]
FH = ['effects', ['font', ['size', 1.27, 1.27]], ['hide', 'yes']]
NS = uuid.UUID('6b1f0c2e-55a1-4d7b-9a43-0c8e2f7d1a55')


def prop(k, v, x=0, y=0, hide=False):
    return ['property', Q(k), Q(v), ['at', x, y, 0], FH if hide else F]


def pin(etype, num, name, x, y, ang, length=2.54, hide=False):
    p = ['pin', etype, 'line', ['at', x, y, ang], ['length', length]]
    if hide:
        p.append(['hide', 'yes'])
    return p + [['name', Q(name), F], ['number', Q(num), F]]


def symbol(name, ref, value, fp, ds, desc, box, pins, kw='', graphics=None, pin_names_hide=False):
    x0, y0, x1, y1 = box
    s = ['symbol', Q(name)]
    if pin_names_hide:
        s.append(['pin_names', ['offset', 0], ['hide', 'yes']])
    s += [['exclude_from_sim', 'no'], ['in_bom', 'yes'], ['on_board', 'yes'],
          prop('Reference', ref, x0, y0 + 1.27), prop('Value', value, x0, y1 - 1.27),
          prop('Footprint', fp, 0, y1 - 3.81, True), prop('Datasheet', ds, 0, 0, True),
          prop('Description', desc, 0, 0, True)]
    if kw:
        s.append(prop('ki_keywords', kw, 0, 0, True))
    g = graphics if graphics is not None else [
        ['rectangle', ['start', x0, y0], ['end', x1, y1], ['stroke', ['width', 0.254], ['type', 'default']],
         ['fill', ['type', 'background']]]]
    s.append(['symbol', Q(name + '_0_1')] + g)
    s.append(['symbol', Q(name + '_1_1')] + pins)
    return s


# ---------------------------------------------------------------- MIC4801
mic = symbol('MIC4801', 'U', 'MIC4801YM', 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm',
             'https://ww1.microchip.com/downloads/en/DeviceDoc/mic4801.pdf',
             'Microchip MIC4801 600 mA single-channel linear WLED current sink, 3.0-5.5 V, PWM on EN, SOIC-8',
             (-7.62, 6.35, 7.62, -6.35),
             [pin('power_in', '1', 'VIN', -10.16, 3.81, 0),
              pin('input', '2', 'EN', -10.16, 1.27, 0),
              pin('passive', '3', 'RSET', -10.16, -1.27, 0),
              pin('power_in', '4', 'GND', -10.16, -3.81, 0),
              pin('passive', '5', 'D1', 10.16, 3.81, 180),
              pin('passive', '6', 'D1', 10.16, 3.81, 180, hide=True),
              pin('passive', '7', 'D1', 10.16, 3.81, 180, hide=True),
              pin('passive', '8', 'D1', 10.16, 3.81, 180, hide=True)],
             'LED driver linear current sink')

# ---------------------------------------------------------------- LH351D (LED with thermal pad)
led_g = [
    ['polyline', ['pts', ['xy', -1.27, -1.27], ['xy', -1.27, 1.27]], ['stroke', ['width', 0.254], ['type', 'default']], ['fill', ['type', 'none']]],
    ['polyline', ['pts', ['xy', 1.27, -1.27], ['xy', 1.27, 1.27], ['xy', -1.27, 0], ['xy', 1.27, -1.27]],
     ['stroke', ['width', 0.254], ['type', 'default']], ['fill', ['type', 'none']]],
    ['polyline', ['pts', ['xy', -1.27, 0], ['xy', 1.27, 0]], ['stroke', ['width', 0], ['type', 'default']], ['fill', ['type', 'none']]],
    ['polyline', ['pts', ['xy', -0.254, 1.778], ['xy', 0.762, 2.794]], ['stroke', ['width', 0], ['type', 'default']], ['fill', ['type', 'none']]],
    ['polyline', ['pts', ['xy', 0.508, 1.778], ['xy', 1.524, 2.794]], ['stroke', ['width', 0], ['type', 'default']], ['fill', ['type', 'none']]],
    ['rectangle', ['start', -2.54, -2.032], ['end', 2.54, -2.54], ['stroke', ['width', 0], ['type', 'default']], ['fill', ['type', 'outline']]],
]
led = symbol('LH351D', 'D', 'LH351D', 'LightBoard:LED_Samsung_LH351D_3.5x3.5mm',
             'https://www1.futurelightingsolutions.com/FutureLightingFiles/Datasheets/LEDDatasheets/Data_Sheet_LH351D_Rev.9.1.pdf',
             'Samsung LH351D high-power white LED, 3535 ceramic, 3 A max, isolated thermal pad (TP)',
             (-2.54, 3.81, 2.54, -3.81),
             [pin('passive', '1', 'K', -5.08, 0, 0, length=3.81),
              pin('passive', '2', 'A', 5.08, 0, 180, length=3.81),
              pin('passive', '3', 'TP', 0, -5.08, 90, length=2.54)],
             'LED white high power 3535', graphics=led_g, pin_names_hide=True)

root = ['kicad_symbol_lib', ['version', 20241209], ['generator', Q('kicad_symbol_editor')],
        ['generator_version', Q('9.0')], mic, led]
open(SYM_OUT, 'w', encoding='utf-8', newline='').write(sexp.dump(root) + '\n')
print('wrote', os.path.abspath(SYM_OUT))


# ---------------------------------------------------------------- LH351D footprint
def u(*k):
    return Q(str(uuid.uuid5(NS, '/'.join(map(str, k)))))


def pad(num, x, y, w, h, paste=True):
    layers = [Q('F.Cu'), Q('F.Mask')] + ([Q('F.Paste')] if paste else [])
    return ['pad', Q(num), 'smd', 'rect', ['at', x, y], ['size', w, h], ['layers'] + layers, ['uuid', u('pad', num, x, y)]]


def line(x0, y0, x1, y1, layer, w):
    return ['fp_line', ['start', x0, y0], ['end', x1, y1], ['stroke', ['width', w], ['type', 'solid']],
            ['layer', Q(layer)], ['uuid', u('ln', layer, x0, y0, x1, y1)]]


name = 'LED_Samsung_LH351D_3.5x3.5mm'
fp = ['footprint', Q(name), ['version', 20241229], ['generator', Q('pcbnew')], ['generator_version', Q('9.0')],
      ['layer', Q('F.Cu')],
      ['descr', Q('Samsung LH351D 3535 high-power LED, land pattern per datasheet Rev 9.1 p.16; '
                  'pad 1 = cathode, 2 = anode, 3 = isolated thermal pad')],
      ['tags', Q('LED 3535 Samsung LH351D')],
      ['property', Q('Reference'), Q('REF**'), ['at', 0, -3.0, 0], ['layer', Q('F.SilkS')], ['uuid', u('ref')],
       ['effects', ['font', ['size', 0.8, 0.8], ['thickness', 0.12]]]],
      ['property', Q('Value'), Q('LH351D'), ['at', 0, 3.0, 0], ['layer', Q('F.Fab')], ['uuid', u('val')],
       ['effects', ['font', ['size', 0.8, 0.8], ['thickness', 0.12]]]],
      ['attr', 'smd']]
# fab body + lens
for a in ((-1.75, -1.75, 1.75, -1.75), (1.75, -1.75, 1.75, 1.75), (1.75, 1.75, -1.75, 1.75), (-1.75, 1.75, -1.75, -1.75)):
    fp.append(line(*a, 'F.Fab', 0.1))
fp.append(['fp_circle', ['center', 0, 0], ['end', 1.65, 0], ['stroke', ['width', 0.1], ['type', 'solid']],
           ['fill', 'no'], ['layer', Q('F.Fab')], ['uuid', u('lens')]])
# silk: corner brackets + cathode-side bar (cathode = +y in footprint coordinates)
s = 1.9
for a in ((-s, -s, -s, -1.0), (s, -s, s, -1.0), (-s, s, -s, 1.0), (s, s, s, 1.0)):
    fp.append(line(*a, 'F.SilkS', 0.15))
fp.append(line(-s, 2.35, s, 2.35, 'F.SilkS', 0.15))
# courtyard
c = 2.6
for a in ((-c, -c, c, -c), (c, -c, c, c), (c, c, -c, c), (-c, c, -c, -c)):
    fp.append(line(*a, 'F.CrtYd', 0.05))
# pads: 2 = anode (-y), 1 = cathode (+y), 3 = thermal
fp.append(pad('2', 0, -1.4, 3.3, 0.5))
fp.append(pad('2', 0, -1.85, 0.5, 0.4, paste=False))
fp.append(pad('1', 0, 1.4, 3.3, 0.5))
fp.append(pad('1', 0, 1.85, 0.5, 0.4, paste=False))
fp.append(pad('3', 0, 0, 3.3, 1.3))
fp.append(['embedded_fonts', 'no'])
os.makedirs(FP_DIR, exist_ok=True)
open(os.path.join(FP_DIR, name + '.kicad_mod'), 'w', encoding='utf-8', newline='').write(sexp.dump(fp) + '\n')
print('wrote', name)
