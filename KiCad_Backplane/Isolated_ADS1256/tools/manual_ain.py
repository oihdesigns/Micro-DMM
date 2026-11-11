"""Hand-route the AIN4..7 breakout lanes (nested 45-degree lanes on F.Cu) and the shunt Kelvin taps.

1. delete every track/via of AIN4..7, COM and +5VA (the router had walled U6's analog column in with +5VA)
2. draw the AIN lanes + Kelvin taps
3. SketchRoute +5VA and COM again (prefers B.Cu), refill, save
"""
import kipc
import route_all

DOC = {'type': 'DOCTYPE_PCB', 'board_filename': 'Isolated_ADS1256.kicad_pcb'}
NM = 1e6
RIP = {'/AIN4', '/AIN5', '/AIN6', '/AIN7', '/COM', '/+5VA'}


def P(x, y):
    return {'x_nm': str(int(round(x * NM))), 'y_nm': str(int(round(y * NM)))}


def track(a, b, net, w=0.2, layer='BL_F_Cu'):
    return {'@type': 'type.googleapis.com/kiapi.board.types.Track', 'start': P(*a), 'end': P(*b),
            'width': {'value_nm': str(int(w * NM))}, 'layer': layer, 'net': {'name': net}}


def lanes():
    out = []
    pins = {'/AIN7': 120.425, '/AIN6': 121.075, '/AIN5': 121.725, '/AIN4': 122.375}
    cols = {'/AIN7': 177.66, '/AIN6': 180.2, '/AIN5': 182.74, '/AIN4': 185.28}
    yv = 112.0
    for n, yp in pins.items():
        xc = cols[n]
        xa = xc - (yp - yv)
        pts = [(150.5, yp), (xa, yp), (xc, yv), (xc, 108.5125)]
        out += [track(pts[i], pts[i + 1], n) for i in range(3)]
    return out


def kelvin():
    return [track((177.9125, 126.5), (180.6, 126.8), '/ISH_P', 0.25),
            track((177.9125, 132.5), (180.6, 132.2), '/ISH_N', 0.25),
            track((180.85, 135.5), (180.85, 132.4), '/ISH_N', 0.25)]


def main():
    c = kipc.Client()
    items = c.call('kiapi.common.commands.GetItems', {'header': {'document': DOC},
                                                      'types': ['KOT_PCB_TRACE', 'KOT_PCB_VIA', 'KOT_PCB_ARC']}).get('items', [])
    ids = [it['id'] for it in items if it.get('net', {}).get('name') in RIP]
    cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': DOC}})
    c.call('kiapi.common.commands.DeleteItems', {'header': {'document': DOC}, 'item_ids': ids})
    r = c.call('kiapi.common.commands.CreateItems', {'header': {'document': DOC}, 'items': lanes() + kelvin()})
    c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': 'manual AIN lanes',
                                              'header': {'document': DOC}})
    st = [str(x.get('status')) for x in r.get('created_items', [])]
    print('deleted', len(ids), 'created', {s: st.count(s) for s in set(st)})


if __name__ == '__main__':
    main()
