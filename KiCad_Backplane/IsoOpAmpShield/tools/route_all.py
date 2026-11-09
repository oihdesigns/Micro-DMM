"""Route IsoOpAmpShield signal nets with Backplane SketchRoute over IPC (one commit per net), refill zones, save.

Planes (GND, HGND_*) are left to the pours (+ stitch_planes.py).  Run against a running api-server
(see kipc.SOCKET) that has IsoOpAmpShield.kicad_pcb open.
"""
import sys
import json
import kipc

DOC = {'type': 'DOCTYPE_PCB', 'board_filename': 'IsoOpAmpShield.kicad_pcb'}
LAYER = 'BL_F_Cu'
PLANES = {'/GND', '/HGND_I', '/HGND_V', '/HGND_Z'}

ORDER = [
    # pins that get boxed in by their neighbours' nets go first
    '/NR_P', '/NR_N', '/FB_P', '/FB_N',
    '/I_OUTP', '/I_OUTN', '/V_OUTP', '/V_OUTN', '/Z_OUTP', '/Z_OUTN', '/ISO_FAULT',
    # analog front ends (short, sensitive)
    '/IAMC_P', '/IAMC_N', '/I_FP', '/I_FN', '/I_SHP', '/I_SHN',
    '/VAMC_P', '/VAMC_N', '/V_DP', '/V_DN', '/V_MP', '/V_MN', '/VIN_P', '/VIN_N',
    '/ZAMC_P', '/ZAMC_N', '/Z_DIFF', '/Z_DN', '/Z_DP', '/Z_PFB', '/Z_NFB', '/Z_PBUF', '/Z_NBUF',
    '/Z_P', '/Z_N', '/ZIN_P', '/ZIN_N', '/U4B_OUT',
    # AMC local supplies
    '/I_DCDC_OUT', '/I_HLDO', '/V_DCDC_OUT', '/V_HLDO', '/Z_DCDC_OUT', '/Z_HLDO',
    '/I_DCDC_IN', '/V_DCDC_IN', '/Z_DCDC_IN',
    # isolated +/-11 V
    '/P12_RAW', '/N12_RAW', '/P12_F', '/N12_F', '/P11', '/N11',
    # low side
    '/P5_DC', '/+5V', '/VDD_AMC', '/+3V3',
    '/ADC_A0', '/ADC_A1', '/ADC_A2', '/ADC_A3', '/ADC_A4', '/ADC_A5',
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
