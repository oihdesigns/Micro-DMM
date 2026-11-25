"""Rev B rework on the routed board (after ImportNetlist): generic R3 headers + corner holes.
  * J1 (now 1x04: 3V3 5V GND GND) anchored on the old pin-4 position, J6 (A6) on the old J2 pin 7 position
  * H1..H4 in the corners (3.5 mm inset)
  * J3 / R2 / FB1-3 moved 4.7 mm left to clear the top-right hole; shunt pours follow; that corner rerouted
"""
import math
import kipc
import route_all
import stitch_planes
from stitch_planes import mm

DOC = {'type': 'DOCTYPE_PCB', 'board_filename': 'IsoOpAmpShield.kicad_pcb'}
NM = 1e6
MOVES = {  # ref: (x, y, rot)
    'J1': (135.56, 150.8, 90), 'J6': (166.04, 150.8, 90),
    'H1': (103.5, 103.5, 0), 'H2': (198.1, 103.5, 0), 'H3': (103.5, 149.84, 0), 'H4': (198.1, 149.84, 0),
    'J3': (191.8, 105.2, 180), 'R2': (189.3, 112.2, 0),
    'FB1': (191.7, 116.0, 270), 'FB2': (186.9, 116.0, 270), 'FB3': (185.0, 116.0, 270),
}
SHUNT_ZONES = {'/I_SHP': (190.1, 101.0, 193.5, 113.5), '/I_SHN': (185.2, 101.0, 188.5, 113.5)}
RIP_NETS = {'/I_SHP', '/I_SHN', '/I_FP', '/I_FN'}
OLD_FB3_PAD2 = (189.6, 116.5)


def P(x, y):
    return {'x_nm': str(int(round(x * NM))), 'y_nm': str(int(round(y * NM)))}


def get(c, t):
    return c.call('kiapi.common.commands.GetItems', {'header': {'document': DOC}, 'types': [t]}).get('items', [])


def commit(c, msg, fn):
    cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': DOC}})
    try:
        out = fn()
    except Exception:
        c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_DROP', 'header': {'document': DOC}})
        raise
    c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': msg,
                                              'header': {'document': DOC}})
    return out


def main():
    c = kipc.Client(timeout_ms=600000)
    # 1. rip up the shunt-corner copper
    rip = []
    for t in ('KOT_PCB_TRACE', 'KOT_PCB_VIA'):
        for it in get(c, t):
            n = it.get('net', {}).get('name')
            pts = [mm(it['start']), mm(it['end'])] if t == 'KOT_PCB_TRACE' else [mm(it['position'])]
            if n in RIP_NETS:
                rip.append(it['id'])
            elif n == '/HGND_I' and any(math.hypot(x - OLD_FB3_PAD2[0], y - OLD_FB3_PAD2[1]) < 1.6 for x, y in pts):
                rip.append(it['id'])
    commit(c, 'rip shunt corner', lambda: c.call('kiapi.common.commands.DeleteItems',
                                                 {'header': {'document': DOC}, 'item_ids': rip}))
    print('ripped', len(rip))
    # 2. move footprints (partial FootprintInstance update)
    fps = {f['reference_field']['text']['text'].get('text', ''): f for f in get(c, 'KOT_PCB_FOOTPRINT')}
    upd = []
    for ref, (x, y, rot) in MOVES.items():
        f = fps[ref]
        upd.append({'@type': 'type.googleapis.com/kiapi.board.types.FootprintInstance', 'id': f['id'],
                    'position': P(x, y), 'orientation': {'value_degrees': rot}, 'layer': f.get('layer', 'BL_F_Cu')})
    r = commit(c, 'rev B moves', lambda: c.call('kiapi.common.commands.UpdateItems', {'header': {'document': DOC}, 'items': upd}))
    print('moved', [str(x.get('status')) for x in r.get('updated_items', [])].count("{'code': 'ISC_OK'}"), 'of', len(upd))
    # 3. shunt pours follow the shunt
    zs = []
    for z in get(c, 'KOT_PCB_ZONE'):
        n = z.get('copper_settings', {}).get('net', {}).get('name')
        if n in SHUNT_ZONES:
            x0, y0, x1, y1 = SHUNT_ZONES[n]
            z['outline'] = {'polygons': [{'outline': {'nodes': [{'point': P(x0, y0)}, {'point': P(x1, y0)},
                                                                {'point': P(x1, y1)}, {'point': P(x0, y1)}], 'closed': True}}]}
            z.pop('filled_polygons', None)
            zs.append(z)
    r = commit(c, 'shunt pours', lambda: c.call('kiapi.common.commands.UpdateItems', {'header': {'document': DOC}, 'items': zs}))
    print('zones', len(zs), [str(x.get('status')) for x in r.get('updated_items', [])])
    c.call('kiapi.common.commands.SaveDocument', {'document': DOC})


if __name__ == '__main__':
    main()
