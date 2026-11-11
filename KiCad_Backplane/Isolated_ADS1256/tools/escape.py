"""Give boxed-in pads an escape via (collision-checked against pads, tracks and vias), then SketchRoute the net.
usage: escape.py REF:PAD [REF:PAD ...]
"""
import math
import sys
import kipc
from stitch_planes import mm, pad_box, d_pt_box, NM

DOC = {'type': 'DOCTYPE_PCB', 'board_filename': 'Isolated_ADS1256.kicad_pcb'}
CLR = 0.2


def d_pt_seg(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    L = vx * vx + vy * vy
    t = 0 if L == 0 else max(0, min(1, ((px - ax) * vx + (py - ay) * vy) / L))
    return math.hypot(px - ax - t * vx, py - ay - t * vy)


def main(targets):
    c = kipc.Client()
    pads = c.call('kiapi.common.commands.GetItems', {'header': {'document': DOC}, 'types': ['KOT_PCB_PAD']})['items']
    fps = c.call('kiapi.common.commands.GetItems', {'header': {'document': DOC}, 'types': ['KOT_PCB_FOOTPRINT']})['items']
    tracks = c.call('kiapi.common.commands.GetItems', {'header': {'document': DOC}, 'types': ['KOT_PCB_TRACE']}).get('items', [])
    vias = c.call('kiapi.common.commands.GetItems', {'header': {'document': DOC}, 'types': ['KOT_PCB_VIA']}).get('items', [])
    refof = {}
    for f in fps:
        refof[f['id']['value']] = (f['reference_field']['text']['text'].get('text', ''), mm(f['position']))
    for tgt in targets:
        ref, num = tgt.split(':')
        p = [p for p in pads if refof[p['parent']['value']][0] == ref and p['number'] == num][0]
        net = p['net']['name']
        px, py = mm(p['position'])
        fx, fy = refof[p['parent']['value']][1]
        b = pad_box(p)
        out = (0, math.copysign(1, py - fy)) if abs(py - fy) > abs(px - fx) else (math.copysign(1, px - fx), 0)
        ext = (b[3] - b[1]) / 2 if out[1] else (b[2] - b[0]) / 2
        best = None
        for k in [0.6, 0.8, 1.0, 1.3, 1.6, 2.0]:
            for side in [0, 0.35, -0.35, 0.6, -0.6]:
                vx = px + out[0] * (ext + k) + (side if out[1] else 0)
                vy = py + out[1] * (ext + k) + (side if out[0] else 0)
                ok = True
                for q in pads:
                    if q.get('net', {}).get('name') == net:
                        continue
                    bb = pad_box(q)
                    if d_pt_box(vx, vy, bb) < 0.3 + CLR + 0.02:
                        ok = False
                        break
                    # stub from pad edge to via
                    for t in range(11):
                        sx = px + (vx - px) * t / 10
                        sy = py + (vy - py) * t / 10
                        if d_pt_box(sx, sy, bb) < 0.1 + CLR + 0.02:
                            ok = False
                            break
                    if not ok:
                        break
                if not ok:
                    continue
                for tr in tracks:
                    if tr.get('net', {}).get('name') == net:
                        continue
                    ax, ay = mm(tr['start'])
                    bx, by = mm(tr['end'])
                    w = int(tr['width']['value_nm']) / NM / 2
                    if d_pt_seg(vx, vy, ax, ay, bx, by) < 0.3 + w + CLR + 0.02:
                        ok = False
                        break
                    if tr.get('layer') == 'BL_F_Cu':
                        for t in range(11):
                            sx = px + (vx - px) * t / 10
                            sy = py + (vy - py) * t / 10
                            if d_pt_seg(sx, sy, ax, ay, bx, by) < 0.1 + w + CLR + 0.02:
                                ok = False
                                break
                    if not ok:
                        break
                if not ok:
                    continue
                for v in vias:
                    if v.get('net', {}).get('name') == net:
                        continue
                    if math.hypot(vx - mm(v['position'])[0], vy - mm(v['position'])[1]) < 0.6 + CLR + 0.02:
                        ok = False
                        break
                if ok:
                    best = (vx, vy)
                    break
            if best:
                break
        if not best:
            print(tgt, net, 'NO ESCAPE SPOT')
            continue
        vx, vy = best
        cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': DOC}})
        r = c.call('kiapi.common.commands.CreateItems', {'header': {'document': DOC}, 'items': [
            {'@type': 'type.googleapis.com/kiapi.board.types.Via',
             'position': {'x_nm': str(int(vx * NM)), 'y_nm': str(int(vy * NM))},
             'pad_stack': {'type': 'PST_NORMAL', 'layers': ['BL_F_Cu', 'BL_B_Cu'],
                           'drill': {'start_layer': 'BL_F_Cu', 'end_layer': 'BL_B_Cu',
                                     'diameter': {'x_nm': '300000', 'y_nm': '300000'}, 'shape': 'DS_CIRCLE'},
                           'copper_layers': [{'layer': 'BL_F_Cu', 'shape': 'PSS_CIRCLE', 'size': {'x_nm': '600000', 'y_nm': '600000'}}]},
             'net': {'name': net}, 'type': 'VT_THROUGH'},
            {'@type': 'type.googleapis.com/kiapi.board.types.Track',
             'start': {'x_nm': str(int(px * NM)), 'y_nm': str(int(py * NM))},
             'end': {'x_nm': str(int(vx * NM)), 'y_nm': str(int(vy * NM))},
             'width': {'value_nm': '200000'}, 'layer': 'BL_F_Cu', 'net': {'name': net}}]})
        c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': 'escape ' + tgt,
                                                  'header': {'document': DOC}})
        vias.append({'position': {'x_nm': str(int(vx * NM)), 'y_nm': str(int(vy * NM))}, 'net': {'name': net}})
        print(tgt, net, 'escape via at (%.2f, %.2f)' % best)


if __name__ == '__main__':
    main(sys.argv[1:])
