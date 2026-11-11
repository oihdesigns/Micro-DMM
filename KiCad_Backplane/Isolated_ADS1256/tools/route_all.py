"""Route IsoOpAmpShield signal nets with Backplane SketchRoute over IPC (one commit per net), refill zones, save.

Planes (GND, HGND_*) are left to the pours (+ stitch_planes.py).  Run against a running api-server
(see kipc.SOCKET) that has Isolated_ADS1256.kicad_pcb open.
"""
import sys
import json
import kipc

DOC = {'type': 'DOCTYPE_PCB', 'board_filename': 'Isolated_ADS1256.kicad_pcb'}
LAYER = 'BL_F_Cu'
PLANES = {'/GND', '/GNDI'}

ORDER = [
    # LDO locals + rails first (LDO OUT pins get boxed in otherwise)
    '/FB_A', '/NR_A', '/FB_D', '/NR_D', '/+5VA', '/+3V3I', '/+9VI', '/+5V_F', '/+5V', '/VIO',
    # breakout lanes (nest into the top of U6's analog column)
    '/AIN7', '/AIN6', '/AIN5', '/AIN4', '/X_AIN4', '/X_AIN5', '/X_AIN6', '/X_AIN7', '/X_AINCOM',
    # crystal, GPIO, SPI
    '/XTAL2', '/XTAL1', '/RESET', '/D0', '/D1', '/D2', '/D3',
    '/DIN', '/SCLK', '/SCLK_I', '/CS', '/DRDY', '/DOUT', '/SYNC',
    '/SYNC_H', '/CS_H', '/DIN_H', '/SCLK_H', '/DRDY_H', '/DOUT_H',
    # analog inputs
    '/AIN2', '/AIN3', '/AIN0', '/AIN1', '/HV_P', '/HV_PA', '/HV_PB', '/HV_N', '/HV_NA', '/HV_NB',
    '/ISH_P', '/ISH_N', '/AINCOM',
    # reference / COM
    '/VREF', '/NR_REF', '/VCM_SET', '/COM_DRV', '/COM',
]


def main(only=None, time_ms=6000):
    c = kipc.Client()
    pads = c.call('kiapi.common.commands.GetItems', {'header': {'document': DOC}, 'types': ['KOT_PCB_PAD']}).get('items', [])
    bynet = {}
    for p in pads:
        n = p.get('net', {}).get('name', '')
        if n and not n.startswith('unconnected-'):
            bynet.setdefault(n, []).append(p['id'])
    nets = [n for n in ORDER if n in bynet]
    missing = sorted(set(bynet) - set(ORDER) - PLANES)
    if missing:
        print('NOT IN ORDER (appended):', missing)
        nets += missing
    if only:
        nets = [n for n in nets if n in only]
    report = {}
    for n in nets:
        if len(bynet[n]) < 2:
            continue
        cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': DOC}})
        try:
            r = c.call('kiapi.backplane.SketchRoute', {'board': DOC, 'items': bynet[n], 'preferred_layer': LAYER,
                                                     'corner_mode': 'SRCM_MITERED_45', 'time_limit_ms': time_ms})
            c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT',
                                                      'message': 'route ' + n, 'header': {'document': DOC}})
        except kipc.ApiError as e:
            c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_DROP', 'header': {'document': DOC}})
            r = {'error': str(e)}
        report[n] = {k: r.get(k) for k in ('connections', 'routed', 'failed', 'vias', 'timed_out', 'error')}
        print('%-14s %s' % (n, report[n]), flush=True)
    c.call('kiapi.board.commands.RefillZones', {'board': DOC})
    c.call('kiapi.common.commands.SaveDocument', {'document': DOC})
    bad = {n: r for n, r in report.items() if r.get('failed') or r.get('error') or (r.get('routed') or 0) < (r.get('connections') or 0)}
    print('FAILED/PARTIAL:', json.dumps(bad, indent=0))


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if '--back' in sys.argv:
        LAYER = 'BL_B_Cu'
    main(only=set(args) or None, time_ms=20000 if '--slow' in sys.argv else 6000)
