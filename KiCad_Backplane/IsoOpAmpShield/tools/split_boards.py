"""Split IsoOpAmpShield into two separate shields (cost-comparison experiment, 2026-11-22):

  boards/IsoHiZShield : HI-Z channel + its CCG3 / LDO supply      -> A3(+)/A2(-), A6 fault
  boards/IsoVIShield  : +/-200 V channel + current channel         -> A0/A1, A4/A5, A6 fault

Same Arduino R3 header positions as the combined board, so either or both stack on one host and the
firmware sees the same pins.  Each board gets its own schematic (gen_sch with the part subset) and a
placed board (gen_pcb's footprint/zone helpers); routing inside the kept islands is copied from the
routed combined board (the current island is moved 52 mm left on the V/I board), then finish_split.py
prunes dangling copper and SketchRoutes whatever is left.

    py -3 tools/split_boards.py            # writes both projects (wipes their routing!)
"""
import copy
import importlib
import json
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, HERE)
import sexp  # noqa: E402
from sexp import Q  # noqa: E402
import design  # noqa: E402

FULL = copy.deepcopy(design.PARTS)
SRC_PCB = os.path.join(ROOT, 'IsoOpAmpShield.kicad_pcb')
HDR = {'J1', 'J2', 'JP1', 'C1', 'R1', 'H1', 'H2', 'H3', 'H4'}
CUR = ['J3', 'R2', 'FB1', 'FB2', 'FB3', 'R3', 'R4', 'C4', 'C5', 'C6', 'C7', 'C8', 'C9', 'U1', 'C10', 'C11', 'C12']
DX_I = -52.0   # current island 183-201.3 -> 131-149.3

BOARDS = {
    'IsoHiZShield': dict(
        title='IsoHiZShield - isolated Hi-Z voltmeter channel, Arduino R3-header shield (split from IsoOpAmpShield rev B)',
        refs={r for r, p in FULL.items() if p['block'] in ('hiz', 'psu')} | HDR | {'R27', 'C37', 'R28', 'C38'},
        X0=124.5, W=56.5,
        isl={'HGND_Z': (131.0, 179.0)},
        gaps=[('GAP_L', 124.5, 131.0), ('GAP_R', 179.0, 181.0)],
        holes={'H1': (128.0, 103.5), 'H2': (177.5, 103.5), 'H3': (127.5, 139.5), 'H4': (177.5, 149.84)},
        moves={},
        shift=set(),
        keep_x=(124.8, 180.7),
        amc_x=[172.0],
        zones={'GND': 'GND', 'HGND_Z': (131.0, 100.35, 179.0, 127.5), 'ISO_BARRIER': (124.5, 127.5, 181.0, 134.5),
               'GAP_V_Z': (124.5, 100.0, 131.0, 127.5), 'GAP_Z_I': (179.0, 100.0, 181.0, 127.5)},
        texts=[('HI-Z 10V', 166.8, 104, 0.8), ('RG', 159.9, 123.3, 0.8), ('ISOLATION BARRIER - NO COPPER', 154.4, 131, 0.8),
               ('IsoHiZShield rev A', 164.9, 143.5, 1.0, True), ('A3/A2=Z A6=FLT', 159.7, 148.4, 0.8),
               ('VDD 3V3|5V', 130.5, 143.9, 0.8)],
        note='Split from IsoOpAmpShield rev B: HI-Z channel only (A3+/A2-, A6 = ISO_FAULT). Stacks with IsoVIShield on the same host.',
    ),
    'IsoVIShield': dict(
        title='IsoVIShield - isolated +/-200 V and current channels, Arduino R3-header shield (split from IsoOpAmpShield rev B)',
        refs={r for r, p in FULL.items() if p['block'] in ('v200', 'cur')} | HDR |
             {'R5', 'C13', 'R6', 'C14', 'R12', 'C24', 'R13', 'C25'},
        X0=100.0, W=68.58,
        isl={'HGND_V': (100.3, 127.0), 'HGND_I': (131.0, 149.3)},
        gaps=[('GAP_V_I', 127.0, 131.0)],
        holes={'H1': (103.5, 103.5), 'H2': (165.08, 103.5), 'H3': (103.5, 149.84), 'H4': (165.08, 146.0)},
        moves={'R1': (155.0, 146.2, 0)},
        shift=set(CUR),
        keep_x=(100.0, 127.5),
        lv_keep={'GND', 'V_DCDC_IN', 'I_DCDC_IN'},
        keep_paths={'ISO_FAULT': 166.1},
        stitch=['C13', 'C14', 'C24', 'C25'],   # ADC RC caps whose GND vias sat outside the carve-out
        amc_x=[113.5, 192.0 + DX_I],
        zones={'GND': 'GND', 'HGND_V': (100.35, 100.35, 127.0, 127.5), 'HGND_I': (131.0, 100.35, 149.3, 127.5),
               'I_SHP': (190.1 + DX_I, 101.0, 193.5 + DX_I, 113.5), 'I_SHN': (185.2 + DX_I, 101.0, 188.5 + DX_I, 113.5),
               'ISO_BARRIER': (100.0, 127.5, 168.58, 134.5), 'GAP_V_Z': (127.0, 100.0, 131.0, 127.5)},
        texts=[('+/-200V MAX', 112.4, 102.4, 0.8), ('10 mOhm', 200.3 + DX_I, 109.4, 0.8),
               ('ISOLATION BARRIER', 157.5, 131, 0.8),
               ('IsoVIShield rev A', 116, 151.6, 1.0, True), ('A0/A1=V A4/A5=I A6=FLT', 157.0, 148.4, 0.8),
               ('VDD 3V3|5V', 130.5, 143.9, 0.8)],
        note='Split from IsoOpAmpShield rev B: +/-200 V (A0+/A1-) and current (A4+/A5-) channels, A6 = ISO_FAULT. Stacks with IsoHiZShield.',
    ),
}


def subset(refs):
    parts = {r: copy.deepcopy(FULL[r]) for r in refs}
    used = {}
    for r, p in parts.items():
        for n in p['pins'].values():
            used.setdefault(n, set()).add(r)
    # J2 keeps only the analog pins this board drives (the rest become no-connects)
    parts['J2']['pins'] = {k: n for k, n in parts['J2']['pins'].items() if used[n] - {'J2'}}
    return parts


def project_files(name, out):
    os.makedirs(out, exist_ok=True)
    pro = json.load(open(os.path.join(ROOT, 'IsoOpAmpShield.kicad_pro'), encoding='utf-8'))
    pro['meta']['filename'] = name + '.kicad_pro'
    for s in pro.get('sheets', []):
        pass
    pro['sheets'] = [[s[0], name] if isinstance(s, list) else s for s in pro.get('sheets', [])]
    pro['board']['design_settings'].setdefault('drc_exclusions', [])
    json.dump(pro, open(os.path.join(out, name + '.kicad_pro'), 'w', encoding='utf-8'), indent=2)
    open(os.path.join(out, 'fp-lib-table'), 'w').write(
        '(fp_lib_table\n  (version 7)\n  (lib (name "IsoOpAmpShield")(type "KiCad")(uri "${KIPRJMOD}/../../IsoOpAmpShield.pretty")(options "")(descr "Shared project footprints"))\n)\n')
    open(os.path.join(out, 'sym-lib-table'), 'w').write(
        '(sym_lib_table\n  (version 7)\n  (lib (name "IsoOpAmpShield")(type "KiCad")(uri "${KIPRJMOD}/../../IsoOpAmpShield.kicad_sym")(options "")(descr "Shared project symbols"))\n)\n')


def gen_schematic(name, cfg, out):
    import gen_sch
    importlib.reload(gen_sch)
    present = {p['block'] for p in design.PARTS.values()}
    gen_sch.BLOCKS = [b for b in gen_sch.BLOCKS if b[0] in present]
    gen_sch.OUT = os.path.join(out, name + '.kicad_sch')
    gen_sch.build()
    t = open(gen_sch.OUT, encoding='utf-8').read()
    t = t.replace('IsoOpAmpShield - isolated V/I/Hi-Z front end, Arduino R3-header shield', cfg['title'])
    t = re.sub(r'"IsoOpAmpShield rev B -- derived from DualAMCBoard[^"]*"', '"%s"' % cfg['note'], t)
    open(gen_sch.OUT, 'w', encoding='utf-8', newline='').write(t)


# ------------------------------------------------------------------ board: carved out of the ROUTED combined board
def blocks(text):
    """Top-level '(kind ...' blocks of a .kicad_pcb: list of (kind, start, end)."""
    out = []
    for m in re.finditer(r'\n\t\((\w+)', text):
        s = m.start() + 1
        d = 0
        for j in range(s, len(text)):
            c = text[j]
            if c == '(':
                d += 1
            elif c == ')':
                d -= 1
                if d == 0:
                    out.append((m.group(1), s, j + 1))
                    break
    return out


def first_xy(blk, key):
    m = re.search(r'\(%s ([-\d.]+) ([-\d.]+)' % key, blk)
    return (float(m.group(1)), float(m.group(2))) if m else None


def item_pts(blk):
    return [first_xy(blk, k) for k in ('start', 'mid', 'end', 'at') if first_xy(blk, k)]


def shift_item(blk, dx):
    def f(m):
        return '(%s %s %s' % (m.group(1), repr(round(float(m.group(2)) + dx, 6)), m.group(3))
    return re.sub(r'\((start|mid|end|at) ([-\d.]+) ([-\d.]+)', f, blk)


def set_translate(blk, x, y):
    return re.sub(r'\(translate [-\d.]+ [-\d.]+\)', '(translate %s %s)' % (x, y), blk, count=1)


def rect_zone(blk, x0, y0, x1, y1):
    blk = re.sub(r'\n\t\t\(filled_polygon[\s\S]*?\n\t\t\)', '', blk)
    return re.sub(r'\(polygon\s*\(pts[^()]*(?:\([^()]*\)[^()]*)*\)\s*\)', '(polygon (pts (xy %s %s) (xy %s %s) (xy %s %s) (xy %s %s)))'
                  % (x0, y0, x1, y0, x1, y1, x0, y1), blk, count=1)


def zone_name(blk):
    m = re.search(r'\(name "([^"]+)"\)', blk)
    if m:
        return m.group(1)
    m = re.search(r'\(net "/?([^"]*)"\)', blk)
    return m.group(1) if m else ''


def gr_text(s, x, y, size=0.8, bold=False, uid=''):
    th = 0.2 if bold else 0.15
    return ('\t(gr_text "%s"\n\t\t(at %s %s 0)\n\t\t(layer "F.SilkS")\n\t\t(uuid "%s")\n\t\t(effects\n\t\t\t(font\n'
            '\t\t\t\t(size %s %s)\n\t\t\t\t(thickness %s)%s\n\t\t\t)\n\t\t)\n\t)') % (
        s, x, y, uid, size, size, th, '\n\t\t\t\t(bold yes)' if bold else '')


def gr_line(x0, y, x1, uid):
    return ('\t(gr_line\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(stroke\n\t\t\t(width 0.15)\n\t\t\t(type default)\n\t\t)\n'
            '\t\t(layer "F.SilkS")\n\t\t(uuid "%s")\n\t)') % (x0, y, x1, y, uid)


def gen_board(name, cfg, out):
    import uuid as _uuid
    import gen_pcb
    importlib.reload(gen_pcb)
    ns = _uuid.UUID('9e3a5c1b-7d24-4f0e-b6a8-2c51d0e4f713')

    def uid(*k):
        return str(_uuid.uuid5(ns, '/'.join(map(str, (name,) + k))))

    src = open(SRC_PCB, encoding='utf-8').read()
    bl = blocks(src)
    head_end = min(s for k, s, e in bl if k == 'footprint')
    head = src[:head_end]
    head = re.sub(r'\(title "[^"]*"\)', '(title "%s")' % name, head)
    head = re.sub(r'\(rev "[^"]*"\)', '(rev "A")', head)
    head = re.sub(r'\(comment 1 "[^"]*"\)', '(comment 1 "%s")' % cfg['title'].split(' - ')[1], head)
    netnames = set(design.nets())
    unc = gen_pcb.UNCONN
    X0, W, Y0, H = cfg['X0'], cfg['W'], 100.0, 53.34
    BAND = (127.5, 134.5)
    out_items = []
    # footprints, verbatim from the routed board (keeps the silkscreen clean-up)
    for k, s, e in bl:
        if k != 'footprint':
            continue
        blk = src[s:e]
        ref = re.search(r'\(property "Reference" "([^"]+)"', blk).group(1)
        if ref not in design.PARTS:
            continue
        if ref in cfg['shift']:
            x, y = first_xy(blk, 'translate')
            blk = set_translate(blk, round(x + DX_I, 6), y)
        if ref in cfg['moves']:
            blk = set_translate(blk, *cfg['moves'][ref][:2])
        if ref in cfg['holes']:
            blk = set_translate(blk, *cfg['holes'][ref])
        for pin, nn in unc.get(ref, {}).items():       # pins left without a net on this board (J2)
            blk = re.sub(r'(\(pad "%s" [\s\S]*?\(net )"[^"]*"' % re.escape(pin), lambda m: m.group(1) + '"%s"' % nn, blk, count=1)
        blk = blk.replace('(sheetfile "IsoOpAmpShield.kicad_sch")', '(sheetfile "%s.kicad_sch")' % name)
        out_items.append(blk)
    # outline
    out_items.append('\t(gr_rect\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n'
                     '\t\t(fill no)\n\t\t(layer "Edge.Cuts")\n\t\t(uuid "%s")\n\t)' % (X0, Y0, round(X0 + W, 4), Y0 + H, uid('edge')))
    # barrier silk lines, broken around the parts that straddle the band
    gaps = sorted([(round(ax - 5.6, 3), round(ax + 5.6, 3)) for ax in cfg['amc_x']] +
                  ([(131.5, 142.6)] if 'PS1' in design.PARTS else []))
    xs, x = [], X0 + 0.5
    for a, b in gaps:
        if a > x:
            xs.append((x, a))
        x = max(x, b)
    if x < X0 + W - 0.5:
        xs.append((x, round(X0 + W - 0.5, 3)))
    for a, b in xs:
        for yy in BAND:
            out_items.append(gr_line(a, yy, b, uid('band', a, yy)))
    for t in cfg['texts']:
        out_items.append(gr_text(t[0], t[1], t[2], t[3], len(t) > 4, uid('txt', t[0])))
    # zones (fills dropped; refilled by kicad-cli)
    m = 0.35
    for k, s, e in bl:
        if k != 'zone':
            continue
        blk = src[s:e]
        rect = cfg['zones'].get(zone_name(blk))
        if rect is None:
            continue
        if rect == 'GND':
            rect = (X0 + m, BAND[1], round(X0 + W - m, 4), Y0 + H - m)
        out_items.append(rect_zone(blk, *rect))
    # tracks / vias
    for k, s, e in bl:
        if k not in ('segment', 'arc', 'via'):
            continue
        blk = src[s:e]
        net = re.search(r'\(net "/?([^"]*)"\)', blk).group(1)
        if net not in netnames:
            continue
        p = item_pts(blk)
        if cfg['shift'] and all(x >= 181.0 for x, _ in p):          # moves with U1
            if any(y > 134.5 for _, y in p) and net not in cfg['lv_keep']:
                continue                                              # its low-side signal stubs get rerouted
            out_items.append(shift_item(blk, DX_I))
            continue
        if net in cfg.get('keep_paths', {}) and all(x <= cfg['keep_paths'][net] for x, _ in p):
            out_items.append(blk)       # e.g. U2's ISO_FAULT escape + B.Cu run to J2 pin 7 (same spot on both boards)
            continue
        if any(y > 134.5 for _, y in p) and 'lv_keep' in cfg and net not in cfg['lv_keep']:
            continue        # low-side nets that get rerouted: drop their old stubs so the router starts at the pads
        if False:
            out_items.append(shift_item(blk, DX_I))
        elif all(cfg['keep_x'][0] <= x <= cfg['keep_x'][1] for x, _ in p):
            out_items.append(blk)
    txt = head + '\n'.join(out_items) + '\n\t(embedded_fonts no)\n)\n'
    path = os.path.join(out, name + '.kicad_pcb')
    open(path, 'w', encoding='utf-8', newline='').write(txt)
    print('wrote', path, len(out_items), 'items')


def main(which=None):
    for name, cfg in BOARDS.items():
        if which and name not in which:
            continue
        out = os.path.join(ROOT, 'boards', name)
        design.PARTS.clear()
        design.PARTS.update(subset(cfg['refs']))
        project_files(name, out)
        gen_schematic(name, cfg, out)
        gen_board(name, cfg, out)
    design.PARTS.clear()
    design.PARTS.update(FULL)


if __name__ == '__main__':
    main(set(sys.argv[1:]) or None)
