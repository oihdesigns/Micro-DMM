"""Drop one stitching via (+ short F.Cu stub) next to every SMD pad of a plane net (GND / HGND_*),
so every ground pad reaches the In1/In2/B planes regardless of how the top-layer pour gets cut up.
Run BEFORE signal routing so the vias claim their spot first.  Exposed pads get a via in the pad.
"""
import math
import kipc

DOC = {'type': 'DOCTYPE_PCB', 'board_filename': 'Isolated_ADS1256.kicad_pcb'}
NM = 1e6
REGION = {  # allowed via area per plane net (KiCad mm), inset from zone edges
    '/GND': (100.9, 100.9, 118.6, 161.1),
    '/GNDI': (125.4, 100.9, 194.1, 161.1),
}
HV = {'/HV_P', '/HV_N', '/HV_PA', '/HV_NA', '/HV_PB', '/HV_NB'}
HV_BOX = (168.4, 140.4, 195.0, 157.4)
VIA_R = 0.3
CLR = 0.2
HOLES = [(103.5, 103.5), (103.5, 158.5), (191.5, 103.5), (191.5, 158.5)]


def mm(v):
    return int(v.get('x_nm', 0)) / NM, int(v.get('y_nm', 0)) / NM


def pad_box(p):
    x, y = mm(p['position'])
    cl = p['pad_stack']['copper_layers'][0]
    w, h = mm(cl['size'])
    a = p['pad_stack'].get('angle', {}).get('value_degrees', 0.0) % 180
    if 45 < a < 135:
        w, h = h, w
    return (x - w / 2, y - h / 2, x + w / 2, y + h / 2)


def d_pt_box(px, py, b):
    dx = max(b[0] - px, 0, px - b[2])
    dy = max(b[1] - py, 0, py - b[3])
    return math.hypot(dx, dy)


def d_seg_box(ax, ay, bx, by, b, n=12):
    return min(d_pt_box(ax + (bx - ax) * t / n, ay + (by - ay) * t / n, b) for t in range(n + 1))


def main():
    c = kipc.Client()
    pads = c.call('kiapi.common.commands.GetItems', {'header': {'document': DOC}, 'types': ['KOT_PCB_PAD']})['items']
    fps = c.call('kiapi.common.commands.GetItems', {'header': {'document': DOC}, 'types': ['KOT_PCB_FOOTPRINT']})['items']
    fpos = {f['id']['value']: mm(f['position']) for f in fps}
    boxes = [(pad_box(p), p.get('net', {}).get('name', ''), p['id']['value']) for p in pads]
    placed = []  # (x, y, net)
    todo = [p for p in pads if p.get('net', {}).get('name') in REGION and p['type'] == 'PT_SMD']
    created = 0
    fails = []
    cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': DOC}})
    items = []
    for p in todo:
        net = p['net']['name']
        px, py = mm(p['position'])
        b = pad_box(p)
        hx, hy = (b[2] - b[0]) / 2, (b[3] - b[1]) / 2
        fx, fy = fpos.get(p['parent']['value'], (px, py))
        dx, dy = px - fx, py - fy
        cands = []
        if math.hypot(dx, dy) < 0.05:              # exposed / centre pad: via in pad
            cands.append((px, py))
        else:
            if abs(dx) >= abs(dy):
                out = (math.copysign(1, dx), 0)
            else:
                out = (0, math.copysign(1, dy))
            perp = (out[1], out[0])
            for k in (0.45, 0.75, 1.1, 1.5):
                ext = hx if out[0] else hy
                cands.append((px + out[0] * (ext + VIA_R + k * 0.5), py + out[1] * (ext + VIA_R + k * 0.5)))
            for s in (1, -1):
                for k in (0.45, 0.8):
                    ext = hx if perp[0] else hy
                    cands.append((px + s * perp[0] * (ext + VIA_R + k), py + s * perp[1] * (ext + VIA_R + k)))
            for sx in (1, -1):
                for sy in (1, -1):
                    cands.append((px + sx * (hx + 0.7), py + sy * (hy + 0.7)))
        R = REGION[net]
        ok = None
        for (vx, vy) in cands:
            if not (R[0] <= vx <= R[2] and R[1] <= vy <= R[3]):
                continue
            if any(math.hypot(vx - hx_, vy - hy_) < 1.6 + VIA_R + 0.3 for hx_, hy_ in HOLES):
                continue
            if HV_BOX[0] <= vx <= HV_BOX[2] and HV_BOX[1] <= vy <= HV_BOX[3]:
                continue
            bad = False
            for (bb, bn, bid) in boxes:
                if bn == net:
                    continue
                clr = 1.6 if bn in HV else CLR
                if d_pt_box(vx, vy, bb) < VIA_R + clr + 0.02:
                    bad = True
                    break
                if (vx, vy) != (px, py) and d_seg_box(px, py, vx, vy, bb) < 0.15 + clr + 0.02:
                    bad = True
                    break
            if bad:
                continue
            if any(math.hypot(vx - x2, vy - y2) < 2 * VIA_R + (CLR if n2 != net else 0.1) + 0.02 for x2, y2, n2 in placed):
                continue
            ok = (vx, vy)
            break
        if not ok:
            fails.append((p['parent']['value'], p['number'], net, round(px, 2), round(py, 2)))
            continue
        vx, vy = ok
        placed.append((vx, vy, net))
        items.append({'@type': 'type.googleapis.com/kiapi.board.types.Via',
                      'position': {'x_nm': str(int(vx * NM)), 'y_nm': str(int(vy * NM))},
                      'pad_stack': {'type': 'PST_NORMAL', 'layers': ['BL_F_Cu', 'BL_B_Cu'],
                                    'drill': {'start_layer': 'BL_F_Cu', 'end_layer': 'BL_B_Cu',
                                              'diameter': {'x_nm': '300000', 'y_nm': '300000'}, 'shape': 'DS_CIRCLE'},
                                    'copper_layers': [{'layer': 'BL_F_Cu', 'shape': 'PSS_CIRCLE',
                                                       'size': {'x_nm': '600000', 'y_nm': '600000'}}]},
                      'net': {'name': net}, 'type': 'VT_THROUGH'})
        if (vx, vy) != (px, py):
            items.append({'@type': 'type.googleapis.com/kiapi.board.types.Track',
                          'start': {'x_nm': str(int(px * NM)), 'y_nm': str(int(py * NM))},
                          'end': {'x_nm': str(int(vx * NM)), 'y_nm': str(int(vy * NM))},
                          'width': {'value_nm': '300000'}, 'layer': 'BL_F_Cu', 'net': {'name': net}})
        created += 1
    r = c.call('kiapi.common.commands.CreateItems', {'header': {'document': DOC}, 'items': items})
    st = [str(x.get('status')) for x in r.get('created_items', [])]
    c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': 'stitch planes',
                                              'header': {'document': DOC}})
    print('plane pads', len(todo), 'vias', created, 'create status', {s: st.count(s) for s in set(st)})
    ref = {f['id']['value']: [x for x in f.get('reference_field', {}).get('text', {}).values()] for f in fps}
    for f in fails:
        print('NO SPOT:', f)


if __name__ == '__main__':
    main()
