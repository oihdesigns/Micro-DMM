"""Export an order folder mfg/<YYYYMMDD-HHMM>[-tag]/ for one or more boards (gerbers + Excellon drills,
bom.csv with the fab's LCSC codes, positions.csv at pad centres) and write manifest.json.

    py -3 tools/export_order.py <tag> <board.kicad_pcb>[:qty:assemble] ...

Customer-fitted parts (CCG3 PS1 and the R3 headers J1/J2) and DNP / exclude-from-BOM parts are left off
the assembly. Each board's fab.json (beside the .kicad_pcb) supplies name/options/features.
"""
import csv
import datetime
import hashlib
import json
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, HERE)
import sexp  # noqa: E402
KCLI = 'C:/Users/Nick/AppData/Local/Programs/Backplane-KiCad/10.99.0-bp.6/bin/kicad-cli.exe'
SKIP = {'PS1', 'J1', 'J2'}     # fitted by the customer

# design part number -> (MPN ordered, manufacturer, LCSC). Fab-stock substitutes (2026-11-22); generic R/C mapped to the
# fab's no-fee (basic/preferred) parts to avoid the per-part extended-loading fee.
M = {
    '12065C102KAT2A': ('1206B102K202NT', 'FH (Fenghua)', 'C9196'),
    'AMC3301DWER': ('AMC3301DWER', 'Texas Instruments', 'C2869778'),
    'AMC3330DWER': ('AMC3330DWER', 'Texas Instruments', 'C2999368'),
    'OPA2192IDR': ('OPA2192IDR', 'Texas Instruments', 'C110074'),
    'C0603C102K5RACTU': ('CL10B102KB8NNNC', 'Samsung Electro-Mechanics', 'C1588'),
    'C0603C103K5RACTU': ('0603B103K500NT', 'FH (Fenghua)', 'C57112'),
    'C0603C104K5RACAUTO': ('CC0603KRX7R9BB104', 'YAGEO', 'C14663'),
    'C0603C331J5GACTU': ('CL10C331JB8NNNC', 'Samsung Electro-Mechanics', 'C1664'),
    'C0805C102J5GACTU': ('C0805C102J5GACTU', 'KEMET', 'C2167084'),
    'CL21A226MAYNNNE': ('CL21A226MAQNNNE', 'Samsung Electro-Mechanics', 'C45783'),
    'CL21A475KAQNNNG': ('CL21A475KAQNNNE', 'Samsung Electro-Mechanics', 'C1779'),
    'ERJ-6ENF1000V': ('0805W8F1000T5E', 'UNI-ROYAL', 'C17408'),
    'ERJ-6ENF1002V': ('0805W8F1002T5E', 'UNI-ROYAL', 'C17414'),
    '0805W8F8202T5E': ('0805W8F8202T5E', 'UNI-ROYAL', 'C17840'),
    'LQH32PN100MN0L': ('LQH32PN100MN0L', 'Murata', 'C98020'),
    'TPS7A3001DGNR': ('TPS7A3001DGNR', 'Texas Instruments', 'C33835'),
    'TPS7A4901DGNR': ('TPS7A4901DGNR', 'Texas Instruments', 'C16430'),
    '12063C105KAT2A': ('CL31B105KBHNNNE', 'Samsung Electro-Mechanics', 'C1848'),
    '74269244182': ('BLM15HD182SN1D', 'Murata', 'C76888'),
    'C0603C103J5GACTU': ('GRM1885C1H103JA01D', 'Murata', 'C85973'),
    'C0805X103JCRAC3316': ('0805CG103J500NT', 'FH (Fenghua)', 'C2971995'),
    'CGA3E1X7R1E105K080AC': ('CL10A105KB8NNNC', 'Samsung Electro-Mechanics', 'C15849'),
    'CGA5L2NP01H104J160AA': ('GRM31C5C1H104JA01L', 'Murata', 'C97946'),
    'CRT0805ABY-10R0ELF': ('0805W8F100JT5E', 'UNI-ROYAL', 'C17415'),
    'ERA-6AED4993V': ('RT0805BRD07499KL', 'YAGEO', 'C706243'),
    'ERJ-8ENF1003V': ('1206W4F1003T5E', 'UNI-ROYAL', 'C17900'),
    'HVC1206Z1008KET': ('1206W4F1006T5E', 'UNI-ROYAL', 'C59781'),
    'RG2012P-103-B-T5': ('RT0805BRD0710KL', 'YAGEO', 'C110775'),
    'RG2012P-104-B-T5': ('RT0805BRD07100KL', 'YAGEO', 'C122537'),
    'RG2012P-153-B-T5': ('RT0805BRD0715KL', 'YAGEO', 'C728642'),
    'RG2012P-4991-B-T5': ('RT0805BRD074K99L', 'YAGEO', 'C706246'),
    'TBJ05-02-1-G-G': ('DB128L-5.08-2P-GN-S', 'DORABO', 'C395868'),
}


def kids(n, name):
    return [c for c in n if isinstance(c, list) and c and c[0] == name]


def one(n, name):
    k = kids(n, name)
    return k[0] if k else None


def footprints(pcb):
    root = sexp.parse(open(pcb, encoding='utf-8').read())
    root = root[0] if isinstance(root[0], list) else root
    out = {}
    for f in kids(root, 'footprint'):
        pr = {p[1]: p[2] for p in kids(f, 'property') if len(p) > 2}
        attr = one(f, 'attr')
        tr = one(f, 'transform')
        fx, fy = map(float, one(tr, 'translate')[1:3])
        rt = one(tr, 'rotate')
        fr = float(rt[1]) if rt else 0.0
        pads = []
        for p in kids(f, 'pad'):
            at = one(p, 'at') or ['at', 0, 0]
            px, py = float(at[1]), float(at[2])
            r = math.radians(fr)
            pads.append((fx + px * math.cos(r) + py * math.sin(r), fy - px * math.sin(r) + py * math.cos(r)))
        out[pr['Reference']] = dict(val=pr.get('Value', ''), fp=str(f[1]).split(':')[-1], pn=pr.get('PN', ''),
                                    attr=attr[1:] if attr else [], pads=pads)
    return out


def refkey(r):
    return re.match(r'[A-Z]+', r).group(), int(re.search(r'\d+', r).group())


def export_board(pcb, d):
    os.makedirs(os.path.join(d, 'gerbers'), exist_ok=True)
    run = lambda *a: subprocess.run([KCLI] + list(a), capture_output=True, text=True)
    run('pcb', 'export', 'gerbers', '--layers', 'F.Cu,In1.Cu,In2.Cu,B.Cu,F.Mask,B.Mask,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,Edge.Cuts',
        '--subtract-soldermask', '--no-x2', '-o', os.path.join(d, 'gerbers') + '/', pcb)
    run('pcb', 'export', 'drill', '--format', 'excellon', '--excellon-units', 'mm', '--excellon-separate-th',
        '--generate-map', '--map-format', 'gerberx2', '-o', os.path.join(d, 'gerbers') + '/', pcb)
    raw = os.path.join(d, '_pos_raw.csv')
    run('pcb', 'export', 'pos', '--format', 'csv', '--units', 'mm', '--side', 'both', '--exclude-dnp', '-o', raw, pcb)
    fps = footprints(pcb)
    rows, fitted, skipped = {}, set(), []
    for ref in sorted(fps, key=refkey):
        i = fps[ref]
        if 'dnp' in i['attr'] or 'exclude_from_bom' in i['attr'] or ref in SKIP or not i['pads']:
            skipped.append(ref)
            continue
        mpn, mfr, lcsc = M[i['pn']]
        r = rows.setdefault(mpn, dict(refs=[], val=i['val'], fp=i['fp'], mfr=mfr, lcsc=lcsc))
        r['refs'].append(ref)
        fitted.add(ref)
    with open(os.path.join(d, 'bom.csv'), 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['Designator', 'Quantity', 'Value', 'Footprint', 'MPN', 'Manufacturer', 'LCSC'])
        for mpn, r in rows.items():
            w.writerow([','.join(r['refs']), len(r['refs']), r['val'], r['fp'], mpn, r['mfr'], r['lcsc']])
    n = 0
    with open(raw, encoding='utf-8') as fi, open(os.path.join(d, 'positions.csv'), 'w', newline='', encoding='utf-8') as fo:
        w = csv.writer(fo)
        w.writerow(['Designator', 'Mid X', 'Mid Y', 'Rotation', 'Layer'])
        for r in csv.DictReader(fi):
            if r['Ref'] in fitted:
                ps = fps[r['Ref']]['pads']          # pad centre: the fab's part models are centred
                cx = sum(p[0] for p in ps) / len(ps)
                cy = sum(p[1] for p in ps) / len(ps)
                w.writerow([r['Ref'], '%.4f' % cx, '%.4f' % -cy, r['Rot'], 'Top' if r['Side'] == 'top' else 'Bottom'])
                n += 1
    os.remove(raw)
    return len(rows), len(fitted), n, skipped


def main(tag, specs):
    ts = datetime.datetime.now().strftime('%Y%m%d-%H%M')
    out = os.path.join(ROOT, 'mfg', ts + ('-' + tag if tag else ''))
    parts = []
    for spec in specs:
        pcb, *qa = spec.split(':')
        pcb = os.path.abspath(pcb)
        qty, asm = (int(qa[0]), int(qa[1])) if qa else (5, 5)
        fab = json.load(open(pcb.replace('.kicad_pcb', '.fab.json'), encoding='utf-8'))
        name = fab['name']
        lines, fitted, npos, skipped = export_board(pcb, os.path.join(out, name))
        print('%-14s bom lines %d, fitted %d, positions %d, not assembled %s' % (name, lines, fitted, npos, skipped))
        parts.append({'name': name, 'kind': fab['kind'], 'qty': qty, 'options': fab['options'], 'features': fab['features'],
                      'files': {'gerbers': name + '/gerbers', 'bom': name + '/bom.csv', 'positions': name + '/positions.csv'},
                      'assemble': asm})
    files = {}
    for dp, dn, fn in os.walk(out):
        for f in sorted(fn):
            p = os.path.relpath(os.path.join(dp, f), out).replace(os.sep, '/')
            if p != 'manifest.json':
                files[p] = hashlib.sha256(open(os.path.join(dp, f), 'rb').read()).hexdigest()
    title = 'IsoOpAmpShield split into ' + ' + '.join(p['name'] for p in parts) if len(parts) > 1 else \
        parts[0]['name'] + ' - isolated measurement shield for Arduino R3 headers'
    m = {'spec': 1, 'title': title, 'ship_to': 'US', 'parts': parts, 'files': dict(sorted(files.items()))}
    json.dump(m, open(os.path.join(out, 'manifest.json'), 'w', encoding='utf-8'), indent=2)
    print('wrote', out, len(files), 'files')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2:])
