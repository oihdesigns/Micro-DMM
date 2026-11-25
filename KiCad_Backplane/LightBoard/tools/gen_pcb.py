"""Generate LightBoard.kicad_pcb (placement, nets, outline, zones, tracks, vias) from design.py.

56 x 36 mm, 4 layers: F.Cu parts + pours | In1 GND | In2 +5V | B.Cu GND (heat spreader, faces the heatsink).
The board is fully routed here (it is small): explicit tracks for the signal nets, and an automatic
via placer drops a plane via next to every SMD +5V / GND pad.  Zones are filled by kicad-cli (drc --refill-zones).

Layout: 2 x 2 LED cluster in the middle (8 mm pitch), anodes facing the centre +5V strip, cathodes facing
outward to their 2.7R dropper and MIC4801 (left pair rot 0, right pair rot 180).  Connectors on the bottom edge.
EN runs as a B.Cu ring along the left/bottom/right edges.
"""
import copy
import math
import os
import uuid

import sexp
from sexp import Q
import design

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'LightBoard.kicad_pcb')
FPROOT = 'C:/Program Files/KiCad/10.0/share/kicad/footprints'
NS = uuid.UUID('a71c3e90-2b5d-4c8f-9e16-3d0b7f2a6c48')

X0, Y0, W, H = 100.0, 100.0, 56.0, 36.0
CX = X0 + W / 2          # 128


def uid(*k):
    return Q(str(uuid.uuid5(NS, '/'.join(map(str, k)))))


def mx(x):
    return 2 * CX - x


P = {}
# ---------------- LED cluster
P['D1'] = (124.0, 114.0, 270)
P['D3'] = (124.0, 122.0, 270)
P['D2'] = (mx(124.0), 114.0, 90)
P['D4'] = (mx(124.0), 122.0, 90)
# ---------------- channels: left (rot 0) and mirrored right (rot 180)
for n, y_led, y_u in ((1, 114.0, 113.0), (3, 122.0, 123.0)):
    P['R%d' % n] = (117.2, y_led, 0)
    P['U%d' % n] = (108.4, y_u, 0)
    P['C%d' % n] = (102.6, y_u - 3.0, 90)       # VIN pad (pad 1) at the bottom, next to U pin 1
    P['R%d' % (n + 4)] = (102.6, y_u + 1.6, 270)  # RSET pad (pad 1) on top, next to U pin 3
for n, y_led, y_u in ((2, 114.0, 113.0), (4, 122.0, 123.0)):
    P['R%d' % n] = (mx(117.2), y_led, 180)
    P['U%d' % n] = (mx(108.4), y_u, 180)
    P['R%d' % (n + 4)] = (mx(102.6), y_u - 1.6, 90)   # RSET pad at the bottom, next to U pin 3
    P['C%d' % n] = (mx(102.6), y_u + 3.0, 270)       # VIN pad on top, next to U pin 1
# ---------------- bottom strip
P['J1'] = (117.6, 132.9, 180)    # pin1 +5V x 117.6, pin2 0V x 115.06; wire entry on the bottom edge
P['Q1'] = (110.6, 131.6, 0)
P['R9'] = (110.6, 127.5, 0)
P['C5'] = (120.85, 131.4, 90)
P['C6'] = (110.6, 134.6, 0)
P['SW1'] = (126.0, 133.6, 0)     # pads x 126/128/130; actuator overhangs the bottom edge
P['R11'] = (126.0, 128.9, 270)
P['R13'] = (128.0, 128.7, 90)
P['R12'] = (135.1, 131.2, 90)
P['J2'] = (141.6, 132.9, 180)    # pin1 GND x 141.6, pin2 PWM x 139.06
P['D5'] = (147.0, 130.3, 0)
P['R10'] = (147.0, 132.9, 0)
P['H1'] = (103.5, 103.5, 0)
P['H2'] = (152.5, 103.5, 0)
P['H3'] = (103.5, 132.5, 0)
P['H4'] = (152.5, 132.5, 0)

SILK_REFS = ('J', 'U', 'D', 'Q')
EN_X_L, EN_X_R, EN_Y = 101.0, 155.0, 127.9


# ------------------------------------------------------------------ footprint loading / pad geometry
def load_fp(fpid):
    lib, name = fpid.split(':')
    if lib == 'LightBoard':
        path = os.path.join(HERE, '..', 'LightBoard.pretty', name + '.kicad_mod')
    else:
        path = os.path.join(FPROOT, lib + '.pretty', name + '.kicad_mod')
    return sexp.parse(open(path, encoding='utf-8').read())[0]


def rot(dx, dy, a):
    r = math.radians(a)
    return dx * math.cos(r) + dy * math.sin(r), -dx * math.sin(r) + dy * math.cos(r)


def pads_of(ref):
    """[(num, x, y, w, h, kind, drill)] absolute, w/h already rotated (axis-aligned rotations only)."""
    part = design.PARTS[ref]
    x, y, a = P[ref]
    fp = load_fp(part['fp'])
    out = []
    for p in sexp.find(fp, 'pad'):
        at = sexp.find1(p, 'at')
        sz = sexp.find1(p, 'size')
        dx, dy = float(at[1]), float(at[2])
        rx, ry = rot(dx, dy, a)
        w, h = float(sz[1]), float(sz[2])
        pa = (float(at[3]) if len(at) > 3 else 0) + a
        if round(pa) % 180 == 90:
            w, h = h, w
        dr = sexp.find1(p, 'drill')
        drill = float(dr[1]) if dr and not isinstance(dr[1], list) and dr[1] != 'oval' else (float(dr[2]) if dr else 0)
        out.append((str(p[1]), x + rx, y + ry, w, h, p[2], drill))
    return out


def pad_xy(ref, num):
    for (n, x, y, *_r) in pads_of(ref):
        if n == num:
            return x, y
    raise KeyError((ref, num))


def place(ref, part, netcode):
    import gen_sch
    import symlib
    x, y, rot_ = P[ref]
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
            if e[1] in ('Datasheet', 'Description'):
                continue
            if e[1] == 'Reference':
                e[2] = Q(ref)
                if not any(ref.startswith(s) for s in SILK_REFS) or ref.startswith('DNP'):
                    lay = sexp.find1(e, 'layer')
                    if lay:
                        lay[1] = Q('F.Fab')
                eff = sexp.find1(e, 'effects')
                fnt = sexp.find1(eff, 'font') if eff else None
                if fnt:
                    for sub in fnt:
                        if isinstance(sub, list) and sub[0] == 'size':
                            sub[1], sub[2] = 0.8, 0.8
                        if isinstance(sub, list) and sub[0] == 'thickness':
                            sub[1] = 0.12
            elif e[1] == 'Value':
                e[2] = Q(part['value'])
            at = sexp.find1(e, 'at')
            if at is not None:
                while len(at) < 4:
                    at.append(0)
                at[3] = (float(at[3]) + rot_) % 360
            if sexp.find1(e, 'uuid') is None:
                e.insert(3, ['uuid', uid(ref, 'prop', e[1])])
        if k in ('fp_text',):
            at = sexp.find1(e, 'at')
            if at is not None:
                while len(at) < 4:
                    at.append(0)
                at[3] = (float(at[3]) + rot_) % 360
        if k == 'pad':
            at = sexp.find1(e, 'at')
            ang = (float(at[3]) if len(at) > 3 else 0.0) + rot_
            ang %= 360
            if len(at) > 3:
                at[3] = ang
            elif ang:
                at.append(ang)
            num = str(e[1])
            net = part['pins'].get(num)
            for sub in list(e):
                if isinstance(sub, list) and sub and sub[0] in ('net', 'uuid'):
                    e.remove(sub)
            if net:
                e.append(['net', netcode[net], Q('/' + net)])
            if net in ZONE_SOLID_NETS and e[2] == 'smd':
                e.append(['zone_connect', 2])
            e.append(['uuid', uid(ref, 'pad', num, sexp.dump(at))])
        body.append(e)
    out += [['layer', Q('F.Cu')], ['uuid', uid(ref, 'fp')], ['at', x, y] + ([rot_] if rot_ else [])]
    tail = [['path', Q('/' + str(gen_sch.uid('sym', ref, 1)))], ['sheetname', Q('/')],
            ['sheetfile', Q('LightBoard.kicad_sch')]]
    sym = symlib.get(part['lib'])
    desc = part['desc'] or symlib.prop(sym, 'Description')
    ds = symlib.prop(sym, 'Datasheet')
    props = [['property', Q('Datasheet'), Q(ds if ds != '~' else ''), ['at', 0, 0, 0], ['layer', Q('F.Fab')], ['hide', 'yes'],
              ['uuid', uid(ref, 'ds')], ['effects', ['font', ['size', 1, 1], ['thickness', 0.15]]]],
             ['property', Q('Description'), Q(desc), ['at', 0, 0, 0], ['layer', Q('F.Fab')], ['hide', 'yes'],
              ['uuid', uid(ref, 'desc')], ['effects', ['font', ['size', 1, 1], ['thickness', 0.15]]]]]
    if part['pn']:
        props.append(['property', Q('PN'), Q(part['pn']), ['at', 0, 0, 0], ['layer', Q('F.Fab')], ['hide', 'yes'],
                      ['uuid', uid(ref, 'pn')], ['effects', ['font', ['size', 1, 1], ['thickness', 0.15]]]])
    lib_props = [b for b in body if b[0] == 'property']
    rest = [b for b in body if b[0] != 'property']
    return out + lib_props + props + tail + rest


# ------------------------------------------------------------------ copper primitives
def seg(x0, y0, x1, y1, w, layer, net, netcode):
    return ['segment', ['start', round(x0, 4), round(y0, 4)], ['end', round(x1, 4), round(y1, 4)], ['width', w],
            ['layer', Q(layer)], ['net', netcode[net]], ['uuid', uid('seg', layer, x0, y0, x1, y1)]]


def via(x, y, net, netcode, size=0.6, drill=0.3):
    return ['via', ['at', round(x, 4), round(y, 4)], ['size', size], ['drill', drill], ['layers', Q('F.Cu'), Q('B.Cu')],
            ['net', netcode[net]], ['uuid', uid('via', x, y)]]


def rect_pts(x0, y0, x1, y1):
    return ['pts', ['xy', x0, y0], ['xy', x1, y0], ['xy', x1, y1], ['xy', x0, y1]]


ZONE_SOLID_NETS = {'GND', '+5V', 'K1', 'K2', 'K3', 'K4', 'DRV1', 'DRV2', 'DRV3', 'DRV4'}


def zone(net, code, layers, pts, prio=0, solid=True, name=None):
    z = ['zone', ['net', code], ['net_name', Q('/' + net)], ['layers'] + [Q(l) for l in layers],
         ['uuid', uid('zone', net, *layers, sexp.dump(pts))]]
    if name:
        z.append(['name', Q(name)])
    z += [['hatch', 'edge', 0.5], ['priority', prio],
          ['connect_pads'] + (['yes'] if solid else []) + [['clearance', 0.25]], ['min_thickness', 0.25],
          ['filled_areas_thickness', 'no'], ['fill', 'yes', ['thermal_gap', 0.4], ['thermal_bridge_width', 0.5]],
          ['polygon', pts]]
    return z


def gtext(s, x, y, size=1.0, layer='F.SilkS', ang=0, bold=False, just=None):
    f = ['font', ['size', size, size], ['thickness', 0.15 if not bold else 0.22]]
    eff = ['effects', f] + ([['justify', just]] if just else [])
    return ['gr_text', Q(s), ['at', x, y, ang], ['layer', Q(layer)], ['uuid', uid('txt', s, x, y)], eff]


def gline(x0, y0, x1, y1, layer='F.SilkS', w=0.15):
    return ['gr_line', ['start', x0, y0], ['end', x1, y1], ['stroke', ['width', w], ['type', 'default']],
            ['layer', Q(layer)], ['uuid', uid('ln', layer, x0, y0, x1, y1)]]


HEADER = r'''(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")
 (general (thickness 1.6) (legacy_teardrops no))
 (paper "A4")
 (title_block (title "LightBoard") (date "2026-11-22") (rev "A") (company "OIH Designs") (comment 1 "4 x LH351D + MIC4801 bright white LED driver, 5 V"))
 (layers
  (0 "F.Cu" signal) (4 "In1.Cu" power) (6 "In2.Cu" power) (2 "B.Cu" signal)
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


# ------------------------------------------------------------------ routing
def routes(nc):
    """Explicit tracks + fixed vias.  Returns (items, fixed_via_points[(x, y, net)], track_list for the via placer)."""
    T = []          # (x0, y0, x1, y1, w, layer, net)
    V = []          # (x, y, net)

    def path(pts, w, layer, net):
        for (a, b) in zip(pts, pts[1:]):
            T.append((a[0], a[1], b[0], b[1], w, layer, net))

    pp = pad_xy
    for n in (1, 2, 3, 4):
        left = n in (1, 3)
        u = 'U%d' % n
        vx = 104.1 if left else mx(104.1)
        # MIC4801 VIN <- local 2.2 uF, plus a plane via between them
        c1 = pp('C%d' % n, '1')
        u1 = pp(u, '1')
        path([c1, u1], 0.6, 'F.Cu', '+5V')
        V.append((vx, u1[1], '+5V'))
        # RSET
        path([pp('R%d' % (n + 4), '1'), pp(u, '3')], 0.3, 'F.Cu', 'RSET%d' % n)
        # GND pin 4 -> via
        u4 = pp(u, '4')
        path([u4, (vx, u4[1])], 0.6, 'F.Cu', 'GND')
        V.append((vx, u4[1], 'GND'))
        # EN: pin 2 -> via -> B.Cu stub to the edge ring
        u2 = pp(u, '2')
        path([u2, (vx, u2[1])], 0.3, 'F.Cu', 'EN')
        V.append((vx, u2[1], 'EN'))
        ex = EN_X_L if left else EN_X_R
        path([(vx, u2[1]), (ex, u2[1])], 0.3, 'B.Cu', 'EN')
    # EN ring on B.Cu: left / right edges down to EN_Y, across to SW1 pin 2
    y_top_l = pad_xy('U1', '2')[1]
    y_top_r = pad_xy('U2', '2')[1]
    sw2 = pp('SW1', '2')
    path([(EN_X_L, y_top_l), (EN_X_L, EN_Y), (EN_X_R, EN_Y), (EN_X_R, y_top_r)], 0.3, 'B.Cu', 'EN')
    path([(sw2[0], EN_Y), sw2], 0.3, 'B.Cu', 'EN')
    # EN pull-down sits right above SW1 pin 2
    path([sw2, pp('R13', '1')], 0.3, 'F.Cu', 'EN')
    # SW1 throws
    path([pp('R11', '2'), pp('SW1', '1')], 0.3, 'F.Cu', 'EN_ON')
    r12b = pp('R12', '2')
    sw3 = pp('SW1', '3')
    path([r12b, (sw3[0] + 1.0, r12b[1]), (sw3[0], r12b[1] + 1.0), sw3], 0.3, 'F.Cu', 'PWM_SW')
    r12a = pp('R12', '1')
    j22 = pp('J2', '2')
    path([r12a, (r12a[0] + 1.2, j22[1]), j22], 0.3, 'F.Cu', 'PWM_IN')
    # reverse-polarity FET
    q3 = pp('Q1', '3')
    j12 = pp('J1', '2')
    path([q3, (q3[0] + 1.3, q3[1]), (q3[0] + 1.3 + (j12[1] - q3[1]), j12[1]), j12], 0.8, 'F.Cu', 'GND_IN')
    r92 = pp('R9', '2')
    q1 = pp('Q1', '1')
    path([r92, (r92[0], q1[1]), q1], 0.3, 'F.Cu', 'QG')
    # power LED
    path([pp('R10', '2'), pp('D5', '2')], 0.3, 'F.Cu', 'PLED')
    # bulk caps to J1 +5V
    path([pp('C5', '1'), (pp('C5', '1')[0], pp('J1', '1')[1]), pp('J1', '1')], 0.8, 'F.Cu', '+5V')
    return T, V


# ------------------------------------------------------------------ automatic plane vias
def build_obstacles(T, V):
    obs = []   # (kind, geom, net, layers)
    for ref in design.PARTS:
        part = design.PARTS[ref]
        for (num, x, y, w, h, kind, drill) in pads_of(ref):
            net = part['pins'].get(num, '')
            if kind == 'np_thru_hole':
                obs.append(('box', (x - w / 2, y - h / 2, x + w / 2, y + h / 2), '#NPTH', 'all'))
            elif kind == 'thru_hole':
                obs.append(('box', (x - w / 2, y - h / 2, x + w / 2, y + h / 2), net, 'all'))
            else:
                obs.append(('box', (x - w / 2, y - h / 2, x + w / 2, y + h / 2), net, 'F.Cu'))
    for (x0, y0, x1, y1, w, layer, net) in T:
        obs.append(('seg', (x0, y0, x1, y1, w / 2), net, layer))
    for (x, y, net) in V:
        obs.append(('box', (x - 0.3, y - 0.3, x + 0.3, y + 0.3), '#VIA', 'all'))
    for h in ('H1', 'H2', 'H3', 'H4'):
        x, y, _ = P[h]
        obs.append(('box', (x - 2.9, y - 2.9, x + 2.9, y + 2.9), '#HOLE', 'all'))
    return obs


def d_pt_box(px, py, b):
    dx = max(b[0] - px, 0, px - b[2])
    dy = max(b[1] - py, 0, py - b[3])
    return math.hypot(dx, dy)


def d_pt_seg(px, py, s):
    x0, y0, x1, y1, r = s
    vx, vy = x1 - x0, y1 - y0
    L = vx * vx + vy * vy
    t = 0 if L == 0 else max(0, min(1, ((px - x0) * vx + (py - y0) * vy) / L))
    return math.hypot(px - x0 - t * vx, py - y0 - t * vy) - r


def via_ok(x, y, net, obs, r=0.3, clr=0.25):
    if not (X0 + 0.9 < x < X0 + W - 0.9 and Y0 + 0.9 < y < Y0 + H - 0.9):
        return False
    for kind, g, onet, layers in obs:
        if onet == net and not onet.startswith('#'):
            continue
        d = d_pt_box(x, y, g) if kind == 'box' else d_pt_seg(x, y, g)
        need = r + clr + (0.2 if onet == '#HOLE' else 0.1 if onet == '#VIA' else 0)
        if d < need:
            return False
    return True


def auto_vias(T, V):
    obs = build_obstacles(T, V)
    out = []
    fails = []
    for ref in sorted(design.PARTS):
        part = design.PARTS[ref]
        for (num, x, y, w, h, kind, drill) in pads_of(ref):
            net = part['pins'].get(num)
            if kind != 'smd' or net not in ('+5V', 'GND'):
                continue
            if ref.startswith('U') or (ref.startswith('C') and ref[1:].isdigit() and int(ref[1:]) <= 4):
                continue   # MIC4801 pins + local caps: fixed vias in routes()
            if ref.startswith('D') and ref != 'D5':
                continue   # LED anodes / thermal pads: centre strip + thermal arrays
            # already has a fixed via within 1.5 mm?
            if any(n2 == net and math.hypot(vx - x, vy - y) < 2.2 for (vx, vy, n2) in V + out):
                continue
            done = False
            for dist in (0.75, 1.0, 1.3, 1.7, 2.1):
                for ang in range(0, 360, 30):
                    ox = x + math.cos(math.radians(ang)) * (w / 2 + dist) if True else 0
                    oy = y + math.sin(math.radians(ang)) * (h / 2 + dist)
                    if via_ok(ox, oy, net, obs):
                        out.append((ox, oy, net))
                        obs.append(('box', (ox - 0.3, oy - 0.3, ox + 0.3, oy + 0.3), '#VIA', 'all'))
                        # stub so the via is tied to the pad even if the pour is cut
                        T.append((x, y, ox, oy, 0.5, 'F.Cu', net))
                        obs.append(('seg', (x, y, ox, oy, 0.25), net, 'F.Cu'))
                        done = True
                        break
                if done:
                    break
            if not done:
                fails.append((ref, num, net))
    return out, fails


def thermal_vias():
    """GND via arrays above / between / below the LED rows + the +5V centre strip."""
    V = []
    for xc in (124.0, mx(124.0)):
        o = -1 if xc < CX else 1          # outward (towards the cathode side)
        for y in (110.0, 111.2, 117.2, 118.8, 124.8, 126.0):
            V.append((xc, y, 'GND'))
            V.append((xc + o * 1.4, y, 'GND'))
        for y in (117.2, 118.8):
            for k in (2.8, 4.2, 5.6):
                V.append((xc + o * k, y, 'GND'))
        for y in (108.8, 110.0, 111.2):
            for k in (2.8, 4.2):
                V.append((xc + o * k, y, 'GND'))
    for y in (111.0, 112.6, 114.2, 115.8, 117.4, 119.0, 120.6, 122.2, 123.8, 125.4):
        V.append((CX, y, '+5V'))
    return V


def build():
    parts = design.PARTS
    names = sorted(design.nets())
    netcode = {n: i + 1 for i, n in enumerate(names)}
    missing = [r for r in parts if r not in P]
    assert not missing, 'unplaced: %s' % missing
    T, V = routes(netcode)
    V += thermal_vias()
    AV, fails = auto_vias(T, V)
    V += AV
    items = []
    for ref in sorted(parts, key=lambda r: (r.rstrip('0123456789'), int(''.join(c for c in r if c.isdigit()) or 0))):
        items.append(place(ref, parts[ref], netcode))
    for (x0, y0, x1, y1, w, layer, net) in T:
        items.append(seg(x0, y0, x1, y1, w, layer, net, netcode))
    for (x, y, net) in V:
        items.append(via(x, y, net, netcode))
    items.append(['gr_rect', ['start', X0, Y0], ['end', X0 + W, Y0 + H], ['stroke', ['width', 0.1], ['type', 'default']],
                  ['fill', 'no'], ['layer', Q('Edge.Cuts')], ['uuid', uid('edge')]])
    m = 0.3
    full = rect_pts(X0 + m, Y0 + m, X0 + W - m, Y0 + H - m)
    items.append(zone('GND', netcode['GND'], ['F.Cu'], full, prio=0))
    items.append(zone('GND', netcode['GND'], ['In1.Cu'], full, prio=0, solid=False))
    items.append(zone('+5V', netcode['+5V'], ['In2.Cu'], full, prio=0, solid=False))
    items.append(zone('GND', netcode['GND'], ['B.Cu'], full, prio=0, solid=False))
    # +5V centre strip joining the four anodes
    items.append(zone('+5V', netcode['+5V'], ['F.Cu'], rect_pts(125.15, 110.4, mx(125.15), 125.6), prio=2))
    # cathode (K) and driver (DRV) heat-spreading pours
    for n, (yl, ) in ((1, (114.0,)), (3, (122.0,))):
        for nn, flip in ((n, False), (n + 1, True)):
            def X(v):
                return mx(v) if flip else v

            def R(x0, y0, x1, y1):
                xa, xb = sorted((X(x0), X(x1)))
                return rect_pts(xa, y0, xb, y1)
            items.append(zone('K%d' % nn, netcode['K%d' % nn], ['F.Cu'], R(119.4, yl - 1.8, 122.85, yl + 1.8), prio=2))
            if n == 1:
                items.append(zone('DRV%d' % nn, netcode['DRV%d' % nn], ['F.Cu'], R(109.75, 106.0, 114.95, 115.8), prio=2))
            else:
                items.append(zone('DRV%d' % nn, netcode['DRV%d' % nn], ['F.Cu'], R(109.75, 120.2, 114.95, 126.0), prio=2))
    # silkscreen
    items.append(gtext('LightBoard rev A', CX, 101.7, 1.0, bold=True))
    items.append(gtext('4x LH351D / MIC4801   5V ONLY', CX, 103.4, 0.8))
    j11 = pad_xy('J1', '1')
    j12 = pad_xy('J1', '2')
    items.append(gtext('+5V', j11[0] + 0.4, 128.2, 0.8, bold=True))
    items.append(gtext('0V', j12[0] - 0.4, 128.2, 0.8, bold=True))
    j21 = pad_xy('J2', '1')
    j22 = pad_xy('J2', '2')
    items.append(gtext('GND', j21[0] + 0.4, 128.2, 0.8))
    items.append(gtext('PWM', j22[0] - 0.4, 128.2, 0.8))
    items.append(gtext('ON', 123.4, 130.6, 0.8, bold=True))
    items.append(gtext('EXT', 132.7, 130.6, 0.8, bold=True))
    items.append(gtext('B.Cu = GND: mount on heatsink with thermal pad', CX, 104.0, 0.8, layer='B.SilkS'))
    items.append(gtext('LightBoard rev A  2026-11', CX, 102.0, 0.8, layer='B.SilkS'))
    for it in items:
        if it[0] == 'gr_text' and sexp.find1(it, 'layer')[1] == 'B.SilkS':
            eff = sexp.find1(it, 'effects')
            eff.append(['justify', 'mirror'])

    txt = HEADER
    txt += ' (net 0 "")' + chr(10) + ''.join(' (net %d "/%s")' % (netcode[n], n) + chr(10) for n in names)
    txt += ''.join(' ' + sexp.dump(it, 1) + '\n' for it in items)
    txt += ' (embedded_fonts no)\n)\n'
    with open(OUT, 'w', encoding='utf-8', newline='') as f:
        f.write(txt)
    print('wrote', os.path.abspath(OUT), len(items), 'items;', len(V), 'vias;', len(T), 'segments; via fails:', fails)


if __name__ == '__main__':
    build()
