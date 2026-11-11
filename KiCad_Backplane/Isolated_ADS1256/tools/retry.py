"""Retry routing for given nets (temporary In1/In2 track keepout up), add extra vias, refill, save.
usage: retry.py [--via x,y,NET ...] [--time ms] NET [NET ...]
"""
import sys
import kipc
import route_all

DOC = {'type': 'DOCTYPE_PCB', 'board_filename': 'Isolated_ADS1256.kicad_pcb'}
NM = 1e6


def pt(x, y):
    return {'point': {'x_nm': str(int(x * NM)), 'y_nm': str(int(y * NM))}}


def via(x, y, net):
    return {'@type': 'type.googleapis.com/kiapi.board.types.Via',
            'position': {'x_nm': str(int(x * NM)), 'y_nm': str(int(y * NM))},
            'pad_stack': {'type': 'PST_NORMAL', 'layers': ['BL_F_Cu', 'BL_B_Cu'],
                          'drill': {'start_layer': 'BL_F_Cu', 'end_layer': 'BL_B_Cu',
                                    'diameter': {'x_nm': '300000', 'y_nm': '300000'}, 'shape': 'DS_CIRCLE'},
                          'copper_layers': [{'layer': 'BL_F_Cu', 'shape': 'PSS_CIRCLE',
                                             'size': {'x_nm': '600000', 'y_nm': '600000'}}]},
            'net': {'name': net}, 'type': 'VT_THROUGH'}


def main(argv):
    vias, nets, t = [], [], 20000
    it = iter(argv)
    for a in it:
        if a == '--via':
            x, y, n = next(it).split(',')
            vias.append(via(float(x), float(y), n))
        elif a == '--time':
            t = int(next(it))
        else:
            nets.append(a)
    c = kipc.Client()
    if vias:
        cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': DOC}})
        r = c.call('kiapi.common.commands.CreateItems', {'header': {'document': DOC}, 'items': vias})
        c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': 'vias',
                                                  'header': {'document': DOC}})
        print('vias', [x.get('status') for x in r.get('created_items', [])])
    if nets:
        cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': DOC}})
        r = c.call('kiapi.common.commands.CreateItems', {'header': {'document': DOC}, 'items': [{
            '@type': 'type.googleapis.com/kiapi.board.types.Zone', 'type': 'ZT_RULE_AREA',
            'layers': ['BL_In1_Cu', 'BL_In2_Cu'], 'name': 'TMP_NO_INNER_TRACKS',
            'outline': {'polygons': [{'outline': {'nodes': [pt(99, 99), pt(196, 99), pt(196, 163), pt(99, 163)], 'closed': True}}]},
            'rule_area_settings': {'keepout_copper': False, 'keepout_vias': False, 'keepout_tracks': True}}]})
        tmp_id = r['created_items'][0]['item']['id']
        c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': 'tmp keepout',
                                                  'header': {'document': DOC}})
        for layer in ('BL_F_Cu', 'BL_B_Cu'):
            route_all.LAYER = layer
            route_all.main(only=set(nets), time_ms=t)
        cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': DOC}})
        c.call('kiapi.common.commands.DeleteItems', {'header': {'document': DOC}, 'item_ids': [tmp_id]})
        c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': 'drop tmp keepout',
                                                  'header': {'document': DOC}})
    c.call('kiapi.board.commands.RefillZones', {'board': DOC})
    c.call('kiapi.common.commands.SaveDocument', {'document': DOC})
    print('saved')


if __name__ == '__main__':
    main(sys.argv[1:])
