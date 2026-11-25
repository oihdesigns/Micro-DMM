"""Re-stitch FB3 and reroute the shunt-corner nets with inner layers blocked (rev B)."""
import kipc, route_all, stitch_planes
from route_pass import pt
DOC = {'type': 'DOCTYPE_PCB', 'board_filename': 'IsoOpAmpShield.kicad_pcb'}
c = kipc.Client(timeout_ms=600000)
cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': DOC}})
r = c.call('kiapi.common.commands.CreateItems', {'header': {'document': DOC}, 'items': [{
    '@type': 'type.googleapis.com/kiapi.board.types.Zone', 'type': 'ZT_RULE_AREA',
    'layers': ['BL_In1_Cu', 'BL_In2_Cu'], 'name': 'TMP_NO_INNER_TRACKS',
    'outline': {'polygons': [{'outline': {'nodes': [pt(99, 99), pt(203, 99), pt(203, 155), pt(99, 155)], 'closed': True}}]},
    'rule_area_settings': {'keepout_copper': False, 'keepout_vias': False, 'keepout_tracks': True}}]})
tmp = r['created_items'][0]['item']['id']
c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': 'tmp keepout', 'header': {'document': DOC}})
stitch_planes.main({'FB3'})
route_all.LAYER = 'BL_F_Cu'
route_all.main(only={'/I_SHP', '/I_SHN', '/I_FP', '/I_FN'}, time_ms=8000)
cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': DOC}})
c.call('kiapi.common.commands.DeleteItems', {'header': {'document': DOC}, 'item_ids': [tmp]})
c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': 'drop tmp keepout', 'header': {'document': DOC}})
c.call('kiapi.board.commands.RefillZones', {'board': DOC})
c.call('kiapi.common.commands.SaveDocument', {'document': DOC})
print('saved')
