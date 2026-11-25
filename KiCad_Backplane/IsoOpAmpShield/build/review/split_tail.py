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
    return re.sub(r'\(polygon\s*\(pts[\s\S]*?\)\s*\)', '(polygon (pts (xy %s %s) (xy %s %s) (xy %s %s) (xy %s %s)))'
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
        if cfg['shift'] and all(x >= 181.0 for x, _ in p):
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
