"""Silkscreen clean-up on the ROUTED board file (targeted text edits only; nothing else touched; idempotent).
Run after the IPC server has saved; then reopen the board on the server without saving (reload.py -).
"""
import os
import re

PCB = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Isolated_ADS1256.kicad_pcb')
s = open(PCB, encoding='utf-8').read()

# reference designators that sit on the board edge -> F.Fab (the board labels say what they are)
for ref in ('J2', 'J3'):
    m = re.search(r'\(property "Reference" "%s"\n(\t+)\(at [^)]*\)\n\t+\(layer "F\.SilkS"\)' % ref, s)
    if m:
        seg = m.group(0).replace('(layer "F.SilkS")', '(layer "F.Fab")')
        s = s[:m.start()] + seg + s[m.end():]
    else:
        print('not found / already fab:', ref)

# PS1 reference: move from above the board edge onto the module body (footprint-relative coords)
s, n = re.subn(r'(\(property "Reference" "PS1"\n\t+)\(at [^)]*\)', r'\g<1>(at 5.08 2.6 0)', s)
print('PS1 ref moved', n)

# '+' / '-' HV polarity marks clear of J2's silk
s = s.replace('(gr_text "+"\n\t\t(at 184.5 146 0)', '(gr_text "+"\n\t\t(at 183.8 146 0)')
s = s.replace('(gr_text "-"\n\t\t(at 184.5 151.1 0)', '(gr_text "-"\n\t\t(at 183.8 151.1 0)')
open(PCB, 'w', encoding='utf-8', newline='').write(s)
print('ok')
