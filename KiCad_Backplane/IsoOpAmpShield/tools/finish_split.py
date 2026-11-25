"""Finish a board written by split_boards.py: prune dangling copper (DRC-driven), SketchRoute the
unconnected nets over a self-started headless api-server, refill, save, DRC.

    py -3 tools/finish_split.py IsoVIShield [--no-route]
"""
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, HERE)
ROUTE_MS = (8000, 20000)
FIRST_NETS = ['/VDD_AMC', '/ISO_FAULT']   # boxed in under U2's low side once the other low-side nets are down
KCLI = 'C:/Users/Nick/AppData/Local/Programs/Backplane-KiCad/10.99.0-bp.6/bin/kicad-cli.exe'


def drc(pcb, refill=True):
    rep = pcb.replace('.kicad_pcb', '.drc.json')
    args = [KCLI, 'pcb', 'drc', '--schematic-parity', '--severity-all', '--format', 'json', '-o', rep, pcb]
    if refill:
        args[3:3] = ['--refill-zones', '--save-board']
    subprocess.run(args, capture_output=True)
    return json.load(open(rep, encoding='utf-8'))


def remove_uuids(pcb, ids):
    t = open(pcb, encoding='utf-8').read()
    n = 0
    for u in ids:
        m = re.search(r'\n\t\((segment|via|arc)\n(?:(?!\n\t\()[\s\S])*?\(uuid "%s"\)\n\t\)' % re.escape(u), t)
        if m:
            t = t[:m.start()] + t[m.end():]
            n += 1
    open(pcb, 'w', encoding='utf-8', newline='').write(t)
    return n


def prune(pcb):
    for it in range(20):
        d = drc(pcb)            # zones must be filled, or stitching vias look dangling
        ids = [i['uuid'] for v in d['violations'] if v['type'] in ('track_dangling', 'via_dangling') for i in v['items'][:1]]
        if not ids:
            return
        print('prune pass', it, 'removed', remove_uuids(pcb, ids))


def route(name, pcb, stitch=()):
    sock = os.path.join(os.environ.get('TEMP', '/tmp'), 'split_%s_api.sock' % name).replace('\\', '/')
    os.environ['KIPC_SOCKET'] = sock
    if os.path.exists(sock):
        os.remove(sock)
    proc = subprocess.Popen([KCLI, 'api-server', '--socket', sock, '--idle-exit', '600', pcb],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    for _ in range(120):
        if os.path.exists(sock):
            break
        time.sleep(0.5)
    time.sleep(2)
    import importlib
    import kipc
    importlib.reload(kipc)
    import route_all
    importlib.reload(route_all)
    route_all.ORDER = FIRST_NETS + [n for n in route_all.ORDER if n not in FIRST_NETS]
    import route_pass
    doc = {'type': 'DOCTYPE_PCB', 'board_filename': os.path.basename(pcb)}
    route_all.DOC = doc
    c = kipc.Client(socket=sock)
    pt = route_pass.pt
    cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': doc}})
    r = c.call('kiapi.common.commands.CreateItems', {'header': {'document': doc}, 'items': [{
        '@type': 'type.googleapis.com/kiapi.board.types.Zone', 'type': 'ZT_RULE_AREA',
        'layers': ['BL_In1_Cu', 'BL_In2_Cu'], 'name': 'TMP_NO_INNER_TRACKS',
        'outline': {'polygons': [{'outline': {'nodes': [pt(99, 99), pt(203, 99), pt(203, 155), pt(99, 155)], 'closed': True}}]},
        'rule_area_settings': {'keepout_copper': False, 'keepout_vias': False, 'keepout_tracks': True}}]})
    tmp = r['created_items'][0]['item']['id']
    c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': 'tmp keepout', 'header': {'document': doc}})
    if stitch:
        import stitch_planes
        importlib.reload(stitch_planes)
        stitch_planes.DOC = doc
        stitch_planes.main(set(stitch))
    d = json.load(open(pcb.replace('.kicad_pcb', '.drc.json'), encoding='utf-8'))
    nets = set()
    for u in d['unconnected_items']:
        for i in u['items']:
            m = re.search(r'\[(/[^\]]+)\]', i['description'])
            if m:
                nets.add(m.group(1))
    print('routing', sorted(nets))
    route_all.LAYER = 'BL_F_Cu'
    route_all.main(only=nets, time_ms=ROUTE_MS[0])
    route_all.LAYER = 'BL_B_Cu'
    route_all.main(only=nets, time_ms=ROUTE_MS[1])
    cm = c.call('kiapi.common.commands.BeginCommit', {'header': {'document': doc}})
    c.call('kiapi.common.commands.DeleteItems', {'header': {'document': doc}, 'item_ids': [tmp]})
    c.call('kiapi.common.commands.EndCommit', {'id': cm['id'], 'action': 'CMA_COMMIT', 'message': 'drop tmp keepout', 'header': {'document': doc}})
    c.call('kiapi.board.commands.RefillZones', {'board': doc})
    c.call('kiapi.common.commands.SaveDocument', {'document': doc})
    proc.terminate()
    proc.wait(timeout=30)


def starved_refs(d):
    """Refs whose plane-net (GND / HGND_*) pads lost their stitching via in the carve-out."""
    refs = set()
    for u in d['unconnected_items'] + [v for v in d['violations'] if v['type'] == 'starved_thermal']:
        for i in u['items']:
            m = re.search(r'\[/(?:GND|HGND_\w+)\] of (\w+)', i['description'])
            if m:
                refs.add(m.group(1))
    return refs


def main(name, do_route=True, do_prune=True):
    pcb = os.path.join(ROOT, 'boards', name, name + '.kicad_pcb').replace('\\', '/')
    if do_prune:
        prune(pcb)
    d = drc(pcb)
    import split_boards
    first = set(split_boards.BOARDS[name].get('stitch', ()))
    for _ in range(3):                      # no pruning after routing: it can cut fresh routes at shared stubs
        if not (do_route and (d['unconnected_items'] or starved_refs(d) or first)):
            break
        route(name, pcb, stitch=starved_refs(d) | first)
        first = set()
        d = drc(pcb)
    import collections
    print(name, collections.Counter(v['type'] for v in d['violations']), 'unconnected', len(d['unconnected_items']),
          'parity', len(d['schematic_parity']))


if __name__ == '__main__':
    main(sys.argv[1], '--no-route' not in sys.argv, '--no-prune' not in sys.argv)
