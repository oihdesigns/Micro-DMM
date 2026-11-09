"""Full routing pass (run on the stitched, unrouted board):
  1. temporary In1/In2 track keepouts (vias still allowed) so SketchRoute never cuts the inner planes
  2. escape vias for U2 OUTP/OUTN (they must pass under U2's own low-side pins to reach J2)
  3. route_all ORDER on F.Cu, retry failures on B.Cu
  4. remove the temporary keepouts, refill, save
"""
import kipc
import escape
import route_all

DOC = {'type': 'DOCTYPE_PCB', 'board_filename': 'IsoOpAmpShield.kicad_pcb'}


def pt(x, y):
    return {'point': {'x_nm': str(int(x * 1e6)), 'y_nm': str(int(y * 1e6))}}


def main():
    c = kipc.Client()
    cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': DOC}})
    r = c.call('kiapi.common.commands.CreateItems', {'header': {'document': DOC}, 'items': [{
        '@type': 'type.googleapis.com/kiapi.board.types.Zone', 'type': 'ZT_RULE_AREA',
        'layers': ['BL_In1_Cu', 'BL_In2_Cu'], 'name': 'TMP_NO_INNER_TRACKS',
        'outline': {'polygons': [{'outline': {'nodes': [pt(99, 99), pt(203, 99), pt(203, 155), pt(99, 155)], 'closed': True}}]},
        'rule_area_settings': {'keepout_copper': False, 'keepout_vias': False, 'keepout_tracks': True}}]})
    tmp_id = r['created_items'][0]['item']['id']
    c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': 'tmp keepout',
                                              'header': {'document': DOC}})
    print('tmp keepout', tmp_id)
    escape.main(['U2:10', 'U2:11'])
    route_all.LAYER = 'BL_B_Cu'
    route_all.main(only={'/V_OUTP', '/V_OUTN'}, time_ms=8000)
    route_all.LAYER = 'BL_F_Cu'
    route_all.main(time_ms=6000)
    route_all.LAYER = 'BL_B_Cu'
    route_all.main(time_ms=20000)            # second pass: anything left, prefer B.Cu
    cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': DOC}})
    c.call('kiapi.common.commands.DeleteItems', {'header': {'document': DOC}, 'item_ids': [tmp_id]})
    c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': 'drop tmp keepout',
                                              'header': {'document': DOC}})
    c.call('kiapi.board.commands.RefillZones', {'board': DOC})
    c.call('kiapi.common.commands.SaveDocument', {'document': DOC})
    print('saved')


if __name__ == '__main__':
    main()
