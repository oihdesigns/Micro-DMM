"""Silkscreen clean-up applied to the ROUTED board file (targeted text edits, nothing else touched).
Run with the IPC server's copy saved, then reopen it without saving (reopen.py).
"""
import re
import uuid

PCB = 'C:/Users/Nick/Documents/GitHub/Micro-DMM/KiCad_Backplane/IsoOpAmpShield/IsoOpAmpShield.kicad_pcb'
NS = uuid.UUID('5d0c1f7e-9a51-4b3a-a0e4-6f1a7c3d2b90')

s = open(PCB, encoding='utf-8').read()


def block_at(s, start):
    d = 0
    for i in range(start, len(s)):
        if s[i] == '(':
            d += 1
        elif s[i] == ')':
            d -= 1
            if d == 0:
                return i + 1
    raise ValueError


def remove_top_items(s, pred):
    out, i, n = [], 0, 0
    for m in list(re.finditer(r'\n\t\((gr_line|gr_text)\b', s)):
        pass
    pos = 0
    res = []
    while True:
        m = re.search(r'\n\t\((gr_line|gr_text)\b', s[pos:])
        if not m:
            res.append(s[pos:])
            break
        a = pos + m.start() + 1
        b = block_at(s, a + 1)
        blk = s[a:b]
        if pred(blk):
            res.append(s[pos:a - 1])
            n += 1
        else:
            res.append(s[pos:b])
        pos = b
    return ''.join(res), n


# 1. barrier lines + labels we are replacing
def is_old(blk):
    blk = blk.strip()
    if blk.startswith('(gr_line') and '"F.SilkS"' in blk and re.search(r'\(start [\d.]+ (127\.5|134\.5)\)', blk):
        return True
    if blk.startswith('(gr_line') and '"F.SilkS"' in blk and re.search(r'\(start 100\.5 (127\.5|134\.5)\)', blk):
        return True
    if blk.startswith('(gr_text') and any(t in blk for t in ('"+/-200V MAX"', '"HI-Z +/-10V"', '"CURRENT 10m"',
                                                             '"ISOLATION BARRIER - NO COPPER"', '"VDD 3V3|5V"', '"RG"',
                                                             '"HI-Z 10V"', '"I 10 mOhm"')):
        return True
    return False


s, n = remove_top_items(s, is_old)
print('removed', n)


def gl(x0, y0, x1, y1):
    u = uuid.uuid5(NS, 'gl%.2f%.2f%.2f%.2f' % (x0, y0, x1, y1))
    return ('\t(gr_line\n\t\t(start %g %g)\n\t\t(end %g %g)\n\t\t(stroke\n\t\t\t(width 0.15)\n\t\t\t(type dash)\n\t\t)\n'
            '\t\t(layer "F.SilkS")\n\t\t(uuid "%s")\n\t)\n') % (x0, y0, x1, y1, u)


def gt(txt, x, y, size=0.8, ang=0):
    u = uuid.uuid5(NS, 'gt' + txt)
    return ('\t(gr_text "%s"\n\t\t(at %g %g %g)\n\t\t(layer "F.SilkS")\n\t\t(uuid "%s")\n\t\t(effects\n\t\t\t(font\n'
            '\t\t\t\t(size %g %g)\n\t\t\t\t(thickness 0.15)\n\t\t\t)\n\t\t)\n\t)\n') % (txt, x, y, ang, u, size, size)


new = ''
# barrier edges, broken around U2 / PS1 / U5 / U1 bodies
for y in (127.5, 134.5):
    for x0, x1 in [(100.5, 107.9), (119.1, 131.5), (142.6, 166.4), (177.6, 186.4), (197.6, 201.1)]:
        new += gl(x0, y, x1, y)
new += gt('ISOLATION BARRIER - NO COPPER', 154.4, 131.0, 0.8)
new += gt('+/-200V MAX', 109.5, 102.4, 0.9)
new += gt('HI-Z 10V', 164.0, 104.0, 0.8, 90)
new += gt('I 10 mOhm', 186.4, 105.2, 0.8, 90)
new += gt('VDD 3V3|5V', 130.5, 143.9, 0.8)
new += gt('RG', 159.9, 123.3, 0.8)
i = s.rindex('\t(embedded_fonts')
s = s[:i] + new + s[i:]

# 2. move some reference designators to F.Fab
for ref in ('J3', 'J4', 'J5', 'U3', 'U4', 'R20', 'JP1'):
    m = re.search(r'\(property "Reference" "%s"\n(\t+)\(at [^)]*\)\n\t+\(layer "F\.SilkS"\)' % ref, s)
    if m:
        seg = m.group(0).replace('(layer "F.SilkS")', '(layer "F.Fab")')
        s = s[:m.start()] + seg + s[m.end():]
    else:
        print('ref not found / already fab:', ref)

# 3. PS1 'ISO OUT' / 'IN' footprint text clear of the pads
s = s.replace('(fp_text user "ISO OUT"\n\t\t\t(at 11.7 1.52 ', '(fp_text user "ISO OUT"\n\t\t\t(at 10.4 1.52 ')
open(PCB, 'w', encoding='utf-8', newline='').write(s)
print('ok')
