"""Generate IsoOpAmpShield.kicad_sch from design.py.

Every connected pin gets a local net label exactly on its end point (no wires), unused pins get
a no-connect flag, and nets with power inputs but no power output get a PWR_FLAG. Parts are grouped
into titled blocks. Re-running regenerates the sheet from the data model (stable UUIDs).
"""
import os
import sys
import uuid

import sexp
import symlib
from sexp import Q
import design

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'IsoOpAmpShield.kicad_sch')
PROJECT = 'IsoOpAmpShield'
NS = uuid.UUID('6b1d0c52-3a1e-4f39-9d0b-1f3c2a7e5d11')
SHEET_UUID = str(uuid.uuid5(NS, 'sheet'))
G = 2.54


def uid(*k):
    return Q(str(uuid.uuid5(NS, '/'.join(map(str, k)))))


def snap(v):
    return round(v / 1.27) * 1.27


FONT = ['effects', ['font', ['size', 1.27, 1.27]]]

BLOCKS = [
    # key, title, column
    ('giga', 'GIGA R1 INTERFACE  (A0/A1 = +/-200 V, A3/A2 = HI-Z, A4/A5 = CURRENT, A6 = ISO_FAULT)', 0),
    ('v200', '+/-200 V CHANNEL  (2 x 499k per leg + 10k -> AMC3330, 1 nF C0G input filter)', 0),
    ('adc', 'ADC OUTPUT FILTERS (at the Giga header, low side)', 1),
    ('cur', 'CURRENT CHANNEL  (10 mOhm shunt -> AMC3301, +/-25 A linear)', 1),
    ('psu', 'ISOLATED +/-11 V FOR THE HI-Z FRONT END  (CCG3 -> 10 uH/22 uF LC -> TPS7A49/TPS7A30)', 1),
    ('hiz', 'HI-Z CHANNEL  (100M bias, unity INA w/ RG pad -> /10 diff amp -> AMC3330)', 2),
]


class Sheet:
    def __init__(self):
        self.items = []
        self.lib_syms = {}
        self.flags_needed = set()

    def label(self, net, x, y, ang):
        just = {0: ['justify', 'left', 'bottom'], 90: ['justify', 'left', 'bottom'],
                180: ['justify', 'right', 'bottom'], 270: ['justify', 'right', 'bottom']}[ang]
        self.items.append(['label', Q(net), ['at', x, y, ang], ['fields_autoplaced', 'yes'],
                           ['effects', ['font', ['size', 1.27, 1.27]], just], ['uuid', uid('lbl', net, x, y)]])

    def noconnect(self, x, y):
        self.items.append(['no_connect', ['at', x, y], ['uuid', uid('nc', x, y)]])

    def text(self, s, x, y, size=1.7, bold=True):
        f = ['font', ['size', size, size]] + ([['bold', 'yes']] if bold else [])
        self.items.append(['text', Q(s), ['exclude_from_sim', 'no'], ['at', x, y, 0],
                           ['effects', f, ['justify', 'left', 'bottom']], ['uuid', uid('txt', s, x, y)]])

    def rect(self, x0, y0, x1, y1):
        self.items.append(['rectangle', ['start', x0, y0], ['end', x1, y1],
                           ['stroke', ['width', 0.254], ['type', 'dash']], ['fill', ['type', 'none']],
                           ['uuid', uid('rect', x0, y0)]])

    def symbol(self, ref, lib_id, unit, x, y, value, fp, pn, desc, dnp, pins_nets, ref_vis=True):
        sym = self.lib_syms.setdefault(lib_id, symlib.get(lib_id))
        is_power = ref.startswith('#')
        rx, ry = x + 2.54, y - 1.27
        vx, vy = x + 2.54, y + 1.27
        pl = symlib.pins(sym)
        upins = [p for p in pl if p[0] in (unit, 0)]
        if lib_id.startswith('Device:'):
            pass
        else:
            ys = [p[5] for p in upins] or [0]
            xs = [p[4] for p in upins] or [0]
            top = y - max(ys) - 3.81
            if lib_id.startswith('Connector'):
                rx, ry = x - 2.54, y - max(ys) - 2.54
                vx, vy = x - 2.54, y - min(ys) + 2.54
            else:
                rx, ry = x + min(xs) + 2.54, top
                vx, vy = x + min(xs) + 2.54, y - min(ys) + 3.81
        lib_fp = symlib.prop(sym, 'Footprint')
        props = [
            ['property', Q('Reference'), Q(ref), ['at', snap(rx), snap(ry), 0],
             ['effects', ['font', ['size', 1.27, 1.27]], ['justify', 'left']] + ([] if ref_vis else [['hide', 'yes']])],
            ['property', Q('Value'), Q(value), ['at', snap(vx), snap(vy), 0],
             ['effects', ['font', ['size', 1.27, 1.27]], ['justify', 'left']] + ([['hide', 'yes']] if is_power else [])],
            ['property', Q('Footprint'), Q(fp or lib_fp), ['at', x, y, 0], ['effects', ['font', ['size', 1.27, 1.27]], ['hide', 'yes']]],
            ['property', Q('Datasheet'), Q(symlib.prop(sym, 'Datasheet') or '~'), ['at', x, y, 0],
             ['effects', ['font', ['size', 1.27, 1.27]], ['hide', 'yes']]],
            ['property', Q('Description'), Q(desc or symlib.prop(sym, 'Description')), ['at', x, y, 0],
             ['effects', ['font', ['size', 1.27, 1.27]], ['hide', 'yes']]],
        ]
        if pn:
            props.append(['property', Q('PN'), Q(pn), ['at', x, y, 0], ['effects', ['font', ['size', 1.27, 1.27]], ['hide', 'yes']]])
        node = ['symbol', ['lib_id', Q(lib_id)], ['at', x, y, 0], ['unit', unit],
                ['exclude_from_sim', 'no'],
                ['in_bom', 'no' if is_power else (sexp.find1(sym, 'in_bom') or [0, 'yes'])[1]],
                ['on_board', 'no' if is_power else (sexp.find1(sym, 'on_board') or [0, 'yes'])[1]], ['dnp', 'yes' if dnp else 'no'],
                ['uuid', uid('sym', ref, unit)]] + props
        for p in pl:
            node.append(['pin', Q(p[1]), ['uuid', uid('pin', ref, p[1])]])
        node.append(['instances', ['project', Q(PROJECT), ['path', Q('/' + SHEET_UUID), ['reference', Q(ref)], ['unit', unit]]]])
        self.items.append(node)
        # connections
        for (u, num, name, etype, px, py, ang, hidden) in upins:
            ex, ey = x + px, y - py
            net = pins_nets.get(num)
            if net is None:
                if etype != 'no_connect' and not hidden and not is_power:
                    self.noconnect(ex, ey)
                continue
            self.label(net, ex, ey, {0: 180, 180: 0, 90: 270, 270: 90}[ang])

    def bbox(self, lib_id, unit):
        sym = self.lib_syms.setdefault(lib_id, symlib.get(lib_id))
        ps = [p for p in symlib.pins(sym) if p[0] in (unit, 0)]
        xs = [p[4] for p in ps] or [0]
        ys = [p[5] for p in ps] or [0]
        return min(xs), max(xs), min(ys), max(ys)


def layout_block(sh, key, parts, refs, width):
    """Return (placements, height). placements: (ref, unit, dx, dy) relative to block origin."""
    import re

    def refkey(r):
        m = re.match(r'([A-Z#]+)(\d+)', r)
        return (m.group(1), int(m.group(2)))
    refs = sorted(refs, key=refkey)
    big = [r for r in refs if not parts[r]['lib'].startswith('Device:')]
    small = [r for r in refs if parts[r]['lib'].startswith('Device:')]
    pl = []
    x, y, row_h = 14, 10, 0
    for r in big:
        p = parts[r]
        sym = sh.lib_syms.setdefault(p['lib'], symlib.get(p['lib']))
        for u in symlib.units(sym):
            x0, x1, y0, y1 = sh.bbox(p['lib'], u)
            w = (x1 - x0) + 30
            if x + w > width and x > 14:
                x, y, row_h = 14, y + row_h + 12, 0
            pl.append((r, u, x - x0 + 2, y + y1 + 6))
            x += w
            row_h = max(row_h, (y1 - y0) + 14)
    if big:
        y += row_h + 16
    x = 8
    step = 15.24
    for r in small:
        if x + step > width:
            x, y = 8, y + 33
        pl.append((r, 1, x, y + 14))
        x += step
    if small:
        y += 33
    return pl, y + 4


def build():
    sh = Sheet()
    parts = design.PARTS
    by_block = {}
    for ref, p in parts.items():
        by_block.setdefault(p['block'], []).append(ref)
    colw = 182
    cols = [[12, 32], [12 + colw + 8, 32], [12 + 2 * (colw + 8), 32]]
    for key, title, ci in BLOCKS:
        pl, h = layout_block(sh, key, parts, by_block.get(key, []), colw - 6)
        col = cols[ci]
        ox, oy = col
        sh.text(title, ox + 3, oy + 3)
        for (r, u, dx, dy) in pl:
            p = parts[r]
            cx = round((ox + dx) / G) * G
            cy = round((oy + dy) / G) * G
            sh.symbol(r, p['lib'], u, cx, cy, p['value'], p['fp'], p['pn'], p['desc'], p['dnp'], p['pins'])
        sh.rect(ox, oy - 2, ox + colw, oy + h)
        col[1] = oy + h + 10

    # PWR_FLAGs: nets with a power input but no power output
    drive = {}
    for ref, p in parts.items():
        sym = sh.lib_syms[p['lib']]
        types = {pp[1]: pp[3] for pp in symlib.pins(sym)}
        for pin, net in p['pins'].items():
            drive.setdefault(net, set()).add(types.get(pin, 'passive'))
    need = sorted(n for n, t in drive.items() if 'power_in' in t and 'power_out' not in t)
    col = cols[0]
    fx, fy = col[0], col[1]
    sh.text('PWR_FLAGs (nets fed from the Giga header / through LC filters)', fx + 3, fy + 3, size=1.27)
    for i, n in enumerate(need):
        cx = round((fx + 15 + i * 25) / G) * G
        cy = round((fy + 22) / G) * G
        sh.symbol('#FLG%02d' % (i + 1), 'power:PWR_FLAG', 1, cx, cy, 'PWR_FLAG', '', '', '', False, {'1': n}, ref_vis=False)
    col[1] = fy + 40

    notes = [
        'IsoOpAmpShield rev A -- derived from DualAMCBoard (SE output stage removed; outputs go straight to the Giga as differential pairs).',
        'C15 FIXED: 10 nF -> 1 nF C0G per TI AMC3330 datasheet Fig. 7-2 / 7-5 (input filter now ~16 kHz instead of ~1.6 kHz).',
        'HI-Z gain: G = 0.1 x (1 + 30k/RG).  RG open: +/-10 V FS.  RG 30k: +/-5 V.  RG 3.32k: +/-1 V.  Input Z = 100 Mohm per leg to HGND_Z.',
        'JP1 selects AMC VDD: 1-2 (default) = 3V3 so AMC outputs can never exceed the Giga ADC rail; 2-3 = 5V.',
        'PS1 CCG3-12-12DF isolation is 1500 VDC -- the HI-Z channel is limited to that; the other two keep the AMC rating.',
        'Giga pairs: A0+/A1- and A3+/A2- are STM32H747 hardware differential pairs (ADC1/ADC2 simultaneous); A4/A5 simultaneous pair.',
    ]
    col = cols[0]
    for i, t in enumerate(notes):
        sh.text(t, col[0] + 3, col[1] + 6 + i * 5, size=1.27, bold=False)

    root = ['kicad_sch', ['version', 20250114], ['generator', Q('eeschema')], ['generator_version', Q('9.0')],
            ['uuid', Q(SHEET_UUID)], ['paper', Q('A2')],
            ['title_block', ['title', Q('IsoOpAmpShield - isolated V/I/Hi-Z front end for Arduino Giga R1')],
             ['date', Q('2026-11-07')], ['rev', Q('A')], ['company', Q('OIH Designs')],
             ['comment', 1, Q('Generated by tools/gen_sch.py from tools/design.py')]],
            ['lib_symbols'] + list(sh.lib_syms.values())]
    root += sh.items
    root.append(['sheet_instances', ['path', Q('/'), ['page', Q('1')]]])
    root.append(['embedded_fonts', 'no'])
    with open(OUT, 'w', encoding='utf-8', newline='') as f:
        f.write(sexp.dump(root) + chr(10))
    print('wrote', os.path.abspath(OUT), 'flags:', need, 'col bottoms:', [round(c[1]) for c in cols])


if __name__ == '__main__':
    build()
