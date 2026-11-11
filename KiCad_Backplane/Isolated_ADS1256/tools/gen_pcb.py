"""Generate Isolated_ADS1256.kicad_pcb (placement, nets, outline, zones, isolation keepouts) from design.py.

Board 95 x 62 mm, top-left at KiCad (100, 100), 4 layers (all four layers poured with the local ground).
Host side x 100-119.2, ISOLATION BARRIER x 119.2-124.8 (5.6 mm; the MEJ1 module pin gap is 5.9 mm),
isolated side x 124.8-195.  HV divider in the bottom-right corner with no inner/back-layer copper under it.

Routing is NOT done here (stitch_planes.py + route_all.py); re-running this wipes the routing.
"""
import copy
import os
import uuid

import sexp
from sexp import Q
import design

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'Isolated_ADS1256.kicad_pcb')
FPROOT = 'C:/Program Files/KiCad/10.0/share/kicad/footprints'
NS = uuid.UUID('9e3b6d2a-1c4f-4a8e-b7d5-2f6a8c0e4b19')

X0, Y0, W, H = 100.0, 100.0, 95.0, 62.0
BAR = (119.2, 124.8)


def uid(*k):
    return Q(str(uuid.uuid5(NS, '/'.join(map(str, k)))))


P = {}
# ---------------- host side
P['J1'] = (102.6, 115.0, 0)
P['H1'] = (103.5, 103.5, 0)
P['H2'] = (103.5, 158.5, 0)
P['FB1'] = (107.5, 111.6, 0)
P['C2'] = (109.2, 108.2, 90)
P['C1'] = (111.3, 108.2, 90)
P['C3'] = (113.6, 118.4, 90)
P['C4'] = (111.6, 118.4, 90)
# ---------------- barrier parts
P['PS1'] = (115.65, 103.4, 0)          # pins 1,2 host | 5,7 isolated; body y 100.5-110.8
P['U1'] = (122.0, 127.0, 0)
# ---------------- isolated: supply strip (top)
P['C7'] = (134.4, 103.5, 90)
P['C8'] = (136.2, 103.5, 90)


def ldo(u, x, y, rtop, rbot, cff, cnr, cout, cin):
    # OUT/FB network all on the left (FB never has to wrap round OUT), IN/NR on the right
    P[u] = (x, y, 0)
    P[cout] = (x - 3.9, y - 2.75, 180)
    P[rtop] = (x - 4.5, y + 0.7, 270)
    P[cff] = (x - 6.3, y + 0.7, 270)
    P[rbot] = (x - 5.4, y + 3.4, 0)
    P[cin] = (x + 4.3, y - 1.0, 270)
    P[cnr] = (x + 3.4, y + 2.7, 0)


ldo('U3', 144.5, 104.5, 'R3', 'R4', 'C13', 'C14', 'C15', 'C16')    # DVDD 3.3 V
ldo('U2', 158.5, 104.5, 'R1', 'R2', 'C9', 'C10', 'C11', 'C12')      # AVDD 5.0 V
# ---------------- isolator iso-side decoupling, crystal
P['C6'] = (129.4, 119.2, 0)
P['C5'] = (129.4, 121.0, 0)
P['Y1'] = (135.5, 114.4, 0)
P['C28'] = (127.2, 114.4, 90)
P['C29'] = (143.8, 114.4, 90)
P['R9'] = (131.5, 127.6, 0)
# ---------------- ADS1256 (rot 180: digital column faces the isolator, analog column faces the inputs)
P['U6'] = (147.0, 124.0, 180)
P['R8'] = (146.2, 115.9, 0)
P['C27'] = (146.2, 117.6, 0)
P['C26'] = (149.6, 117.6, 0)
P['C20'] = (153.9, 127.0, 0)
P['C19'] = (157.8, 127.0, 0)
P['C25'] = (153.9, 129.4, 0)
P['C24'] = (157.8, 129.4, 0)
# ---------------- reference + COM buffer
P['U4'] = (155.5, 135.5, 0)
P['C17'] = (151.0, 133.6, 90)
P['C18'] = (160.8, 137.0, 90)
P['R5'] = (151.5, 140.0, 0)
P['R6'] = (151.5, 141.8, 0)
P['C21'] = (151.5, 143.6, 0)
P['U5'] = (156.0, 142.0, 0)
P['C22'] = (156.0, 145.0, 0)
P['R7'] = (160.6, 142.0, 90)
P['C23'] = (160.6, 145.6, 0)
# ---------------- GPIO header (bottom)
P['J5'] = (139.0, 159.5, 90)
# ---------------- analog breakout (top right); rot 270 so pin1 (A4) is rightmost and the lanes to U6 nest
P['J4'] = (185.28, 102.8, 270)
for i, r in enumerate(['R20', 'R21', 'R22', 'R23', 'R24']):
    P[r] = (185.28 - 2.54 * i, 107.6, 270)
P['H3'] = (191.5, 103.5, 0)
P['H4'] = (191.5, 158.5, 0)
# ---------------- shunt (right edge, middle); terminal blocks rot 270 = wire entry at the board edge
P['J3'] = (189.9, 127.0, 270)          # pin1 I+ (y 127.0), pin2 I- (y 132.08)
P['RS1'] = (181.5, 129.5, 270)         # pad1 I+ y 127.1, pad2 I- y 131.9
P['R18'] = (177.0, 126.5, 180)
P['R19'] = (177.0, 132.5, 180)
P['C31'] = (173.5, 129.5, 270)
P['JP1'] = (181.5, 135.5, 0)
# ---------------- +/-250 V divider (bottom right)
P['J2'] = (189.9, 146.0, 270)          # pin1 HV+ (y 146.0), pin2 HV- (y 151.08)
for r, x in (('R10', 182.8), ('R11', 177.0), ('R12', 171.2), ('R13', 166.0)):
    P[r] = (x, 143.0, 180)
for r, x in (('R14', 182.8), ('R15', 177.0), ('R16', 171.2), ('R17', 166.0)):
    P[r] = (x, 154.0, 180)
P['C30'] = (166.5, 148.5, 270)

SILK_REFS = ('J', 'JP', 'PS', 'U', 'Y')
FAB_REFS = ()


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
    if lib == 'Isolated_ADS1256':
        path = os.path.join(HERE, '..', 'Isolated_ADS1256.pretty', name + '.kicad_mod')
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
            ['sheetfile', Q('Isolated_ADS1256.kicad_sch')]]
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
 (title_block (title "Isolated_ADS1256") (date "2026-11-10") (rev "A") (company "OIH Designs") (comment 1 "Isolated ADS1256: +-250 VDC divider, 0.1 ohm shunt, 4 spare inputs"))
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

def keepout_pour(name, layers, pts):
    return ['zone', ['net', 0], ['net_name', Q('')], ['layers'] + [Q(l) for l in layers],
            ['uuid', uid('keepout', name)], ['name', Q(name)], ['hatch', 'edge', 0.5], ['connect_pads', ['clearance', 0]],
            ['min_thickness', 0.25], ['filled_areas_thickness', 'no'],
            ['keepout', ['tracks', 'allowed'], ['vias', 'not_allowed'], ['pads', 'allowed'],
             ['copperpour', 'not_allowed'], ['footprints', 'allowed']],
            ['fill', ['thermal_gap', 0.5], ['thermal_bridge_width', 0.5]], ['polygon', pts]]


HV_BOX = (168.4, 140.4, X0 + W, 157.4)
J1_LABELS = ['5V', 'GND', 'VIO', 'SYNC', 'CS', 'DIN', 'SCLK', 'DRDY', 'DOUT', 'GND']
J4_LABELS = ['A4', 'A5', 'A6', 'A7', 'AC', 'COM', 'GND', '5VA']
J5_LABELS = ['D0', 'D1', 'D2', 'D3', '3V3', 'GND']


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
    items.append(['gr_rect', ['start', X0, Y0], ['end', X0 + W, Y0 + H], ['stroke', ['width', 0.1], ['type', 'default']],
                  ['fill', 'no'], ['layer', Q('Edge.Cuts')], ['uuid', uid('edge')]])
    m = 0.35
    L4 = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
    items.append(zone('GND', netcode['GND'], L4, rect_pts(X0 + m, Y0 + m, BAR[0], Y0 + H - m)))
    items.append(zone('GNDI', netcode['GNDI'], L4, rect_pts(BAR[1], Y0 + m, X0 + W - m, Y0 + H - m)))
    # shunt current path pours (terminal -> RS1 pads)
    items.append(zone('ISH_P', netcode['ISH_P'], ['F.Cu', 'B.Cu'], rect_pts(180.2, 125.1, 191.6, 128.7), prio=2))
    items.append(zone('ISH_N', netcode['ISH_N'], ['F.Cu', 'B.Cu'], rect_pts(180.2, 130.4, 191.6, 133.8), prio=2))
    # keep foreign tracks out of the shunt current path between RS1 and J3 (pours only)
    items.append(['zone', ['net', 0], ['net_name', Q('')], ['layers', Q('F.Cu'), Q('B.Cu')],
                  ['uuid', uid('keepout', 'SHUNT_PATH')], ['name', Q('SHUNT_PATH')], ['hatch', 'edge', 0.5],
                  ['connect_pads', ['clearance', 0]], ['min_thickness', 0.25], ['filled_areas_thickness', 'no'],
                  ['keepout', ['tracks', 'not_allowed'], ['vias', 'not_allowed'], ['pads', 'allowed'],
                   ['copperpour', 'allowed'], ['footprints', 'allowed']],
                  ['fill', ['thermal_gap', 0.5], ['thermal_bridge_width', 0.5]],
                  ['polygon', rect_pts(182.9, 124.5, 192.0, 134.4)]])
    # isolation barrier: nothing but the straddling parts' pads
    items.append(keepout('ISO_BARRIER', rect_pts(BAR[0], Y0, BAR[1], Y0 + H)))
    # no plane copper under the high-voltage divider (only F.Cu, which keeps the HV netclass clearance)
    items.append(keepout_pour('HV_NO_PLANES', ['In1.Cu', 'In2.Cu', 'B.Cu'], rect_pts(*HV_BOX)))
    # silkscreen
    for x in BAR:
        for ya, yb in ((111.6, 120.6), (133.4, 161.5)):
            items.append(gline(x, ya, x, yb))
    items.append(gtext('ISOLATION', 122.0, 140.0, 0.9, ang=90))
    items.append(gtext('HOST', 110.0, 160.2, 1.0))
    items.append(gtext('ISOLATED  (COM / GNDI)', 165.0, 160.6, 1.0))
    items.append(gtext('Isolated ADS1256 rev A', 157.5, 112.6, 1.0, bold=True))
    items.append(gtext('HV +/-250V', 178.0, 140.6, 1.0, bold=True))
    items.append(gtext('+', 184.5, 146.0, 1.2, bold=True))
    items.append(gtext('-', 184.5, 151.1, 1.2, bold=True))
    items.append(gtext('SHUNT 0.1R', 186.0, 122.6, 0.9))
    items.append(gtext('I+', 184.4, 127.0, 0.8))
    items.append(gtext('I-', 184.4, 132.1, 0.8))
    items.append(gtext('JP1: I- = COM', 181.0, 137.4, 0.8))
    for i, t in enumerate(J1_LABELS):
        items.append(gtext(t, 106.2, 115.0 + 2.54 * i, 0.8))
    for i, t in enumerate(J4_LABELS):
        items.append(gtext(t, 185.28 - 2.54 * i, 105.4, 0.8))
    for i, t in enumerate(J5_LABELS):
        items.append(gtext(t, 139.0 + 2.54 * i, 157.0, 0.8))

    txt = HEADER
    txt += ' (net 0 "")' + chr(10) + ''.join(' (net %d "/%s")' % (netcode[n], n) + chr(10) for n in names) + ''.join(' (net %d "%s")' % (netcode[n], n) + chr(10) for n in unc)
    txt += ''.join(' ' + sexp.dump(it, 1) + '\n' for it in items)
    txt += ' (embedded_fonts no)\n)\n'
    with open(OUT, 'w', encoding='utf-8', newline='') as f:
        f.write(txt)
    print('wrote', os.path.abspath(OUT), len(items), 'items')


if __name__ == '__main__':
    build()
