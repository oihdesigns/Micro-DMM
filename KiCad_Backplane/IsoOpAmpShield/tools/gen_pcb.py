"""Generate IsoOpAmpShield.kicad_pcb (placement, nets, outline, zones, isolation keepouts) from design.py.

Board = Arduino Mega/Giga outline, 101.6 x 53.34 mm, KiCad origin of the board's top-left corner at (100,100).
Board coords: X from the left edge (USB side), Y up from the bottom edge.  KiCad: x = 100 + X, y = 153.34 - Y.
Low-voltage side = bottom strip with the Giga POWER and A0-A7 headers.  Isolated islands along the top:
    V200 x 100.3-127, HI-Z x 131-179, CURRENT x 183-201.3.
Barrier band y 127.5-134.5 under every AMC (pad rows at y 126.35 / 135.65) and the CCG3.
No through-hole parts left of x=118 (Giga USB-A/USB-C sit under that corner).

Routing is NOT done here (see route_ipc.py); this script only (re)builds placement + zones.
"""
import copy
import os
import uuid

import sexp
from sexp import Q
import design

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'IsoOpAmpShield.kicad_pcb')
FPROOT = 'C:/Program Files/KiCad/10.0/share/kicad/footprints'
NS = uuid.UUID('0c7f6a8e-55b1-4c62-8f0e-3d3a0b8e9a21')

X0, Y0, W, H = 100.0, 100.0, 101.6, 53.34
BAND = (127.5, 134.5)
ISL = {'HGND_V': (100.3, 127.0), 'HGND_Z': (131.0, 179.0), 'HGND_I': (183.0, 201.3)}


def uid(*k):
    return Q(str(uuid.uuid5(NS, '/'.join(map(str, k)))))


def B(Xb, Yb):
    """board coords (from bottom-left, Arduino convention) -> KiCad"""
    return X0 + Xb, Y0 + H - Yb


# ------------------------------------------------------------------ placement  ref: (x, y, rot)
P = {}
# --- Giga headers & holes (Mega R3 geometry)
P['J1'] = (*B(27.94, 2.54), 90)     # POWER: NC IOREF RESET 3V3 5V GND GND VIN, pins go +x
P['J2'] = (*B(50.80, 2.54), 90)     # A0..A7
P['H1'] = (*B(13.97, 2.54), 0)
P['H2'] = (*B(66.04, 7.62), 0)
P['H3'] = (*B(96.52, 2.54), 0)
# --- AMCs straddling the barrier (rot 270: pins 1-8 = high side on top row)
P['U2'] = (113.5, 131.0, 270)
P['U5'] = (172.0, 131.0, 270)
P['U1'] = (192.0, 131.0, 270)
# --- CCG3: inputs on the low side row (y 135.65), outputs at y 121.75
P['PS1'] = (134.5, 135.65, 90)

# --- low side
P['JP1'] = (130.5, 146.0, 0)
P['C1'] = (126.4, 146.0, 90)
P['R1'] = (172.0, 146.2, 0)
P['L1'] = (134.5, 140.6, 90)
P['C2'] = (137.4, 140.0, 90)
P['C3'] = (139.6, 140.0, 90)
for i, (r, c) in enumerate([('R12', 'C24'), ('R13', 'C25'), ('R28', 'C38'), ('R27', 'C37'), ('R5', 'C13'), ('R6', 'C14')]):
    x = 148.6 + 2.54 * i
    P[r] = (x, 141.0, 270)
    P[c] = (x, 145.0, 90)
P['C22'] = (113.8, 139.9, 90)
P['C23'] = (116.2, 139.9, 90)
P['C21'] = (118.6, 139.4, 90)
P['C35'] = (172.3, 139.9, 90)
P['C36'] = (174.7, 139.9, 90)
P['C34'] = (177.1, 139.4, 90)
P['C11'] = (192.3, 139.9, 90)
P['C12'] = (194.7, 139.9, 90)
P['C10'] = (197.1, 139.4, 90)

# --- +/-200 V island (x 100.3-127); THT only right of x=118
P['J4'] = (125.5, 105.2, 180)          # pin1 VIN_P x125.5, pin2 VIN_N x120.42; body reaches y 109.7
P['R7'] = (125.5, 112.6, 270)
P['R8'] = (125.5, 117.0, 270)
P['R9'] = (120.42, 112.6, 270)
P['R10'] = (120.42, 117.0, 270)
P['R11'] = (122.96, 120.8, 180)
P['FB4'] = (112.6, 120.9, 270)
P['FB5'] = (110.6, 120.9, 270)
P['FB6'] = (108.6, 120.9, 270)
P['C15'] = (110.95, 123.9, 180)
P['C16'] = (114.9, 122.4, 90)
P['C17'] = (116.6, 122.4, 90)
P['C18'] = (118.3, 122.4, 90)
P['C19'] = (113.6, 117.6, 0)
P['C20'] = (113.6, 114.6, 0)

# --- current island (x 183-201.3)
P['J3'] = (196.5, 105.2, 180)          # pin1 I_SHP x196.5, pin2 I_SHN x191.42
P['R2'] = (194.0, 112.2, 0)            # pad1 I_SHN (x191.6), pad2 I_SHP (x196.4)
P['FB1'] = (196.4, 116.0, 270)
P['FB2'] = (191.6, 116.0, 270)
P['FB3'] = (189.6, 116.0, 270)
P['R3'] = (193.4, 119.4, 180)
P['R4'] = (188.6, 119.4, 180)
P['C4'] = (189.46, 123.9, 180)
P['C5'] = (193.4, 122.6, 90)
P['C6'] = (195.1, 122.6, 90)
P['C7'] = (196.8, 122.6, 90)
P['C8'] = (199.6, 120.4, 90)
P['C9'] = (199.6, 115.0, 90)

# --- HI-Z island signal block (x 149-179)
P['J5'] = (160.0, 105.2, 180)          # pin1 ZIN_P x160, pin2 ZIN_N x154.92
P['R16'] = (160.0, 112.8, 270)
P['R14'] = (162.6, 112.8, 270)
P['R17'] = (154.92, 112.8, 270)
P['R15'] = (152.3, 112.8, 270)
P['U3'] = (157.5, 118.6, 180)
P['R18'] = (162.6, 119.2, 90)
P['R19'] = (152.3, 118.6, 90)
P['C26'] = (164.6, 117.0, 90)
P['C27'] = (150.3, 116.0, 90)
P['R20'] = (157.5, 123.3, 0)
P['C39'] = (152.3, 123.3, 0)
P['C40'] = (164.6, 120.9, 90)
P['U4'] = (170.6, 113.6, 0)
P['R22'] = (165.3, 112.6, 90)
P['R21'] = (165.3, 108.2, 90)
P['R23'] = (168.4, 109.6, 0)
P['R24'] = (168.0, 117.6, 0)
P['C41'] = (173.6, 108.9, 0)
P['C42'] = (172.0, 118.3, 0)
P['R25'] = (170.9, 121.0, 270)
P['R26'] = (168.2, 121.0, 270)
P['C28'] = (169.46, 124.0, 180)
P['C29'] = (173.4, 122.6, 90)
P['C30'] = (175.1, 122.6, 90)
P['C31'] = (176.8, 122.6, 90)
P['C32'] = (176.8, 118.6, 0)
P['C33'] = (176.8, 115.6, 0)
# --- HI-Z island isolated supply block (x 131-149, above PS1)
P['U7'] = (135.0, 103.0, 0)
P['U8'] = (144.0, 103.0, 0)
P['C49'] = (132.0, 107.6, 90)
P['C47'] = (134.4, 107.6, 90)
P['R29'] = (136.6, 107.6, 90)
P['C48'] = (138.8, 107.6, 90)
P['C52'] = (141.2, 107.6, 90)
P['C50'] = (143.6, 107.6, 90)
P['R31'] = (145.8, 107.6, 90)
P['C51'] = (148.0, 107.6, 90)
P['C45'] = (132.0, 112.0, 90)
P['R30'] = (134.6, 112.0, 90)
P['C46'] = (141.2, 112.0, 90)
P['R32'] = (143.8, 112.0, 90)
P['L2'] = (133.0, 116.9, 90)
P['C43'] = (136.4, 116.9, 90)
P['L3'] = (141.2, 116.9, 90)
P['C44'] = (144.6, 116.9, 90)


SILK_REFS = ('J', 'JP', 'PS', 'U', 'R20')
FAB_REFS = ('U7', 'U8')


def _unconnected():
    import symlib
    res = {}
    for ref, part in design.PARTS.items():
        if not part['pins'] and 'MountingHole' in part['lib']:
            continue
        sym = symlib.get(part['lib'])
        for (u, num, name, etype, x, y, a, hidden) in symlib.pins(sym):
            if num not in part['pins']:
                nm = name if name and name != '~' else ''
                res.setdefault(ref, {})[num] = 'unconnected-(%s-%sPad%s)' % (ref, nm + '-' if nm else '', num)
    return res


UNCONN = _unconnected()


# ------------------------------------------------------------------ footprint loading
def load_fp(fpid):
    lib, name = fpid.split(':')
    if lib == 'IsoOpAmpShield':
        path = os.path.join(HERE, '..', 'IsoOpAmpShield.pretty', name + '.kicad_mod')
    else:
        path = os.path.join(FPROOT, lib + '.pretty', name + '.kicad_mod')
    return sexp.parse(open(path, encoding='utf-8').read())[0]


def place(ref, part, netcode):
    x, y, rot = P[ref]
    fp = copy.deepcopy(load_fp(part['fp']))
    out = ['footprint', Q(part['fp'])]
    body = []
    for e in fp[2:]:
        if not isinstance(e, list):
            continue
        k = e[0]
        if k in ('version', 'generator', 'generator_version', 'layer'):
            continue
        if k == 'property':
            if e[1] == 'Reference':
                e[2] = Q(ref)
                if not any(ref.startswith(k) for k in SILK_REFS) or ref in FAB_REFS:
                    lay = sexp.find1(e, 'layer')
                    if lay:
                        lay[1] = Q('F.Fab')
                eff = sexp.find1(e, 'effects')
                if eff:
                    fnt = sexp.find1(eff, 'font')
                    if fnt:
                        for sub in fnt:
                            if isinstance(sub, list) and sub[0] == 'size':
                                sub[1], sub[2] = 0.8, 0.8
                            if isinstance(sub, list) and sub[0] == 'thickness':
                                sub[1] = 0.12
            elif e[1] == 'Datasheet':
                continue  # re-added below from the schematic symbol
            elif e[1] == 'Value':
                e[2] = Q(part['value'])
            elif e[1] == 'Description':
                continue  # re-added below from the schematic symbol
            at = sexp.find1(e, 'at')
            if at is not None:
                while len(at) < 4:
                    at.append(0)
                at[3] = float(at[3]) + 0 if True else at[3]
            e.insert(3 if len(e) > 3 else len(e), ['uuid', uid(ref, 'prop', e[1])]) if sexp.find1(e, 'uuid') is None else None
        if k == 'pad':
            at = sexp.find1(e, 'at')
            ang = float(at[3]) if len(at) > 3 else 0.0
            ang = (ang + rot) % 360
            if len(at) > 3:
                at[3] = ang
            elif ang:
                at.append(ang)
            num = str(e[1])
            net = part['pins'].get(num)
            if net is None and num in UNCONN.get(ref, {}):
                net = UNCONN[ref][num]
            # strip existing net
            for sub in list(e):
                if isinstance(sub, list) and sub and sub[0] == 'net':
                    e.remove(sub)
            if net:
                e.append(['net', netcode[net], Q(net if net.startswith('unconnected-') else '/' + net)])
            e.append(['uuid', uid(ref, 'pad', num, sexp.dump(at))])
        body.append(e)
    import gen_sch
    out += [['layer', Q('F.Cu')], ['uuid', uid(ref, 'fp')], ['at', x, y] + ([rot] if rot else [])]
    tail = [['path', Q('/' + str(gen_sch.uid('sym', ref, 1)))], ['sheetname', Q('/')],
            ['sheetfile', Q('IsoOpAmpShield.kicad_sch')]]
    import symlib
    desc = part['desc'] or symlib.prop(symlib.get(part['lib']), 'Description')
    ds = symlib.prop(symlib.get(part['lib']), 'Datasheet')
    props = [['property', Q('Datasheet'), Q(ds if ds != '~' else ''), ['at', 0, 0, 0], ['layer', Q('F.Fab')], ['hide', 'yes'],
              ['uuid', uid(ref, 'ds')], ['effects', ['font', ['size', 1, 1], ['thickness', 0.15]]]]]
    props += [['property', Q('Description'), Q(desc), ['at', 0, 0, 0], ['layer', Q('F.Fab')], ['hide', 'yes'],
              ['uuid', uid(ref, 'desc')], ['effects', ['font', ['size', 1, 1], ['thickness', 0.15]]]]]
    props += [['property', Q('PN'), Q(part['pn']), ['at', 0, 0, 0], ['layer', Q('F.Fab')], ['hide', 'yes'],
              ['uuid', uid(ref, 'pn')], ['effects', ['font', ['size', 1, 1], ['thickness', 0.15]]]]] if part['pn'] else []
    attr_idx = None
    res = out + body[:0]
    # keep property order: library properties first, then PN
    lib_props = [b for b in body if b[0] == 'property']
    rest = [b for b in body if b[0] != 'property']
    res += lib_props + props + tail + rest
    if part['dnp']:
        for b in res:
            if isinstance(b, list) and b and b[0] == 'attr':
                if 'dnp' not in b:
                    b.append('dnp')
                break
        else:
            res.append(['attr', 'smd', 'dnp'])
    return res


def rect_pts(x0, y0, x1, y1):
    return ['pts', ['xy', x0, y0], ['xy', x1, y0], ['xy', x1, y1], ['xy', x0, y1]]


def zone(net, code, layers, pts, prio=0, name=None):
    z = ['zone', ['net', code], ['net_name', Q('/' + net)], ['layers'] + [Q(l) for l in layers],
         ['uuid', uid('zone', net, *layers, sexp.dump(pts))]]
    if name:
        z.append(['name', Q(name)])
    z += [['hatch', 'edge', 0.5], ['priority', prio], ['connect_pads', ['clearance', 0.3]], ['min_thickness', 0.25],
          ['filled_areas_thickness', 'no'], ['fill', 'yes', ['thermal_gap', 0.35], ['thermal_bridge_width', 0.4]],
          ['polygon', pts]]
    return z


def keepout(name, pts):
    return ['zone', ['net', 0], ['net_name', Q('')], ['layers', Q('F.Cu'), Q('In1.Cu'), Q('In2.Cu'), Q('B.Cu')],
            ['uuid', uid('keepout', name)], ['name', Q(name)], ['hatch', 'edge', 0.5], ['connect_pads', ['clearance', 0]],
            ['min_thickness', 0.25], ['filled_areas_thickness', 'no'],
            ['keepout', ['tracks', 'not_allowed'], ['vias', 'not_allowed'], ['pads', 'allowed'],
             ['copperpour', 'not_allowed'], ['footprints', 'allowed']],
            ['fill', ['thermal_gap', 0.5], ['thermal_bridge_width', 0.5]], ['polygon', pts]]


def gtext(s, x, y, size=1.0, layer='F.SilkS', ang=0, bold=False):
    f = ['font', ['size', size, size], ['thickness', 0.15 if not bold else 0.22]]
    return ['gr_text', Q(s), ['at', x, y, ang], ['layer', Q(layer)], ['uuid', uid('txt', s, x, y)], ['effects', f]]


def gline(x0, y0, x1, y1, layer='F.SilkS', w=0.15):
    return ['gr_line', ['start', x0, y0], ['end', x1, y1], ['stroke', ['width', w], ['type', 'default']],
            ['layer', Q(layer)], ['uuid', uid('ln', layer, x0, y0, x1, y1)]]


HEADER = r'''(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")
 (general (thickness 1.6) (legacy_teardrops no))
 (paper "A4")
 (title_block (title "IsoOpAmpShield") (date "2026-11-07") (rev "A") (company "OIH Designs") (comment 1 "Arduino Giga R1 shield: isolated current / +-200 V / Hi-Z channels"))
 (layers
  (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) (2 "B.Cu" signal)
  (9 "F.Adhes" user "F.Adhesive") (11 "B.Adhes" user "B.Adhesive") (13 "F.Paste" user) (15 "B.Paste" user)
  (5 "F.SilkS" user "F.Silkscreen") (7 "B.SilkS" user "B.Silkscreen") (1 "F.Mask" user) (3 "B.Mask" user)
  (17 "Dwgs.User" user "User.Drawings") (19 "Cmts.User" user "User.Comments") (21 "Eco1.User" user "User.Eco1")
  (23 "Eco2.User" user "User.Eco2") (25 "Edge.Cuts" user) (27 "Margin" user) (31 "F.CrtYd" user "F.Courtyard")
  (29 "B.CrtYd" user "B.Courtyard") (35 "F.Fab" user) (33 "B.Fab" user))
 (setup
  (stackup
   (layer "F.SilkS" (type "Top Silk Screen")) (layer "F.Paste" (type "Top Solder Paste"))
   (layer "F.Mask" (type "Top Solder Mask") (thickness 0.01)) (layer "F.Cu" (type "copper") (thickness 0.035))
   (layer "dielectric 1" (type "prepreg") (thickness 0.1) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))
   (layer "In1.Cu" (type "copper") (thickness 0.035))
   (layer "dielectric 2" (type "core") (thickness 1.24) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))
   (layer "In2.Cu" (type "copper") (thickness 0.035))
   (layer "dielectric 3" (type "prepreg") (thickness 0.1) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))
   (layer "B.Cu" (type "copper") (thickness 0.035)) (layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))
   (layer "B.Paste" (type "Bottom Solder Paste")) (layer "B.SilkS" (type "Bottom Silk Screen"))
   (copper_finish "None") (dielectric_constraints no))
  (pad_to_mask_clearance 0)
  (allow_soldermask_bridges_in_footprints no)
  (tenting front back))
'''


def build():
    parts = design.PARTS
    names = sorted(design.nets())
    unc = sorted({n for d in UNCONN.values() for n in d.values()})
    netcode = {n: i + 1 for i, n in enumerate(names + unc)}
    missing = [r for r in parts if r not in P]
    assert not missing, 'unplaced: %s' % missing
    items = []
    for ref in sorted(parts, key=lambda r: (r.rstrip('0123456789'), int(''.join(c for c in r if c.isdigit()) or 0))):
        items.append(place(ref, parts[ref], netcode))
    # outline
    items.append(['gr_rect', ['start', X0, Y0], ['end', X0 + W, Y0 + H], ['stroke', ['width', 0.1], ['type', 'default']],
                  ['fill', 'no'], ['layer', Q('Edge.Cuts')], ['uuid', uid('edge')]])
    m = 0.35
    # ground zones (fill all 4 layers)
    L4 = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
    items.append(zone('GND', netcode['GND'], L4, rect_pts(X0 + m, BAND[1], X0 + W - m, Y0 + H - m)))
    for n, (xa, xb) in ISL.items():
        items.append(zone(n, netcode[n], L4, rect_pts(max(xa, X0 + m), Y0 + m, min(xb, X0 + W - m), BAND[0])))
    # shunt current path: J3 pin -> R2 pad pours on F/B (Kelvin taps FB1/FB2 are routed thin from R2 pads)
    items.append(zone('I_SHP', netcode['I_SHP'], ['F.Cu', 'B.Cu'], rect_pts(195.0, 101.0, 198.6, 113.5), prio=2))
    items.append(zone('I_SHN', netcode['I_SHN'], ['F.Cu', 'B.Cu'], rect_pts(189.6, 101.0, 193.0, 113.5), prio=2))
    # isolation keepouts
    items.append(keepout('ISO_BARRIER', rect_pts(X0, BAND[0], X0 + W, BAND[1])))
    items.append(keepout('GAP_V_Z', rect_pts(127.0, Y0, 131.0, BAND[0])))
    items.append(keepout('GAP_Z_I', rect_pts(179.0, Y0, 183.0, BAND[0])))
    # silkscreen
    for x in (X0 + 1, X0 + W - 1):
        pass
    items.append(gline(X0 + 0.5, BAND[0], X0 + W - 0.5, BAND[0]))
    items.append(gline(X0 + 0.5, BAND[1], X0 + W - 0.5, BAND[1]))
    items.append(gtext('ISOLATION BARRIER - NO COPPER', 152.0, 129.2, 0.8))
    items.append(gtext('+/-200V MAX', 122.5, 101.2 + 0.0, 0.9))
    items.append(gtext('HI-Z +/-10V', 155.4, 101.2, 0.9))
    items.append(gtext('CURRENT 10m', 194.0, 101.2, 0.9))
    items.append(gtext('IsoOpAmpShield rev A', 114.0, 146.6, 1.2, bold=True))
    items.append(gtext('A0/A1=V A3/A2=Z A4/A5=I A6=FLT', 159.7, 148.4, 0.8))
    items.append(gtext('VDD 3V3|5V', 130.5, 143.9, 0.8))

    txt = HEADER
    txt += ' (net 0 "")' + chr(10) + ''.join(' (net %d "/%s")' % (netcode[n], n) + chr(10) for n in names) + ''.join(' (net %d "%s")' % (netcode[n], n) + chr(10) for n in unc)
    txt += ''.join(' ' + sexp.dump(it, 1) + '\n' for it in items)
    txt += ' (embedded_fonts no)\n)\n'
    with open(OUT, 'w', encoding='utf-8', newline='') as f:
        f.write(txt)
    print('wrote', os.path.abspath(OUT), len(items), 'items')


if __name__ == '__main__':
    build()
