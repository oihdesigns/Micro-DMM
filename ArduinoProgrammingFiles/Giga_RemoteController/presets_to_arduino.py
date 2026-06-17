#!/usr/bin/env python3
"""
presets_to_arduino.py
Reads giga_presets.json and prints the PRESETS[] block ready to paste into
Giga_RemoteController.ino.

Usage:
    python presets_to_arduino.py [path/to/giga_presets.json]

If no path is given, looks for giga_presets.json in the same directory.
"""
import json
import os
import sys

RATE_OPTIONS = [10000, 44100, 50000, 100000, 250000, 500000, 1000000]
# 0=10k  1=44.1k  2=50k  3=100k  4=250k  5=500k  6=1M


def rate_idx(rate_str, name=''):
    r = int(rate_str)
    for i, ro in enumerate(RATE_OPTIONS):
        if ro == r:
            return i
    best = min(range(len(RATE_OPTIONS)), key=lambda i: abs(RATE_OPTIONS[i] - r))
    print(f'// WARNING: "{name}" rate {r} Hz not in Arduino RATE_OPTIONS; '
          f'using closest: {RATE_OPTIONS[best]} Hz (idx {best})', file=sys.stderr)
    return best


def flt(s):
    """Format a value for a C++ float field: keep decimals with 'f', else write as int."""
    s = str(s).strip()
    if '.' in s:
        return s + 'f'
    return str(int(float(s)))


def typ_int(t):
    return 1 if str(t).strip().upper() == 'V' else 2


def pin_idx(s):
    return int(str(s).strip()[1:])


def pchan_str(cfg):
    act = 'true' if cfg.get('active', False) else 'false'
    t   = typ_int(cfg.get('type', 'V'))
    off = flt(cfg.get('offset', '0'))
    scl = flt(cfg.get('scale', '1'))
    return f'{{{act},{t},{off},{scl}}}'


def diff_str(en, pos, neg, typ, off, scl):
    e = 'true' if en else 'false'
    return f'{{{e},{pin_idx(pos)},{pin_idx(neg)},{typ_int(typ)},{flt(off)},{flt(scl)}}}'


def display_name(key):
    """Convert JSON key to a human-readable display name (underscores → spaces)."""
    return key.replace('_', ' ')


def emit_preset(name, p):
    ri     = rate_idx(p.get('rate', '500000'), name)
    bits   = int(p.get('bits', '12'))
    time_  = int(p.get('time', '1000'))
    smooth = max(1, int(p.get('smooth', '1')))
    pts    = int(p.get('log', '1000'))

    pins_cfg = p.get('pins', {})
    chs = []
    for i in range(8):
        cfg = pins_cfg.get(f'A{i}', {'active': False, 'type': 'V', 'offset': '0', 'scale': '1'})
        chs.append(pchan_str(cfg))

    d1 = diff_str(
        p.get('diff_enable', False),
        p.get('diff_pos', 'A4'), p.get('diff_neg', 'A5'),
        p.get('diff_type', 'V'),
        p.get('diff_offset', '0'), p.get('diff_scale', '1'),
    )
    d2 = diff_str(
        p.get('diff2_enable', False),
        p.get('diff2_pos', 'A2'), p.get('diff2_neg', 'A3'),
        p.get('diff2_type', 'V'),
        p.get('diff2_offset', '0'), p.get('diff2_scale', '1'),
    )

    ch_line1 = ','.join(chs[:4])
    ch_line2 = ','.join(chs[4:])

    return (
        f'  {{ "{display_name(name)}",\n'
        f'    {ri},{bits},{time_},{smooth},{pts},\n'
        f'    {{ {ch_line1},\n'
        f'      {ch_line2} }},\n'
        f'    {d1}, {d2} }},'
    )


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'giga_presets.json')

    with open(path) as f:
        presets = json.load(f)

    n = len(presets)
    print(f'// Generated from {os.path.basename(path)} — {n} preset(s)')
    print('static const Preset PRESETS[] = {')
    for name, p in presets.items():
        print(emit_preset(name, p))
    print('};')
    print(f'static const int N_PRESETS = {n};')

    if n > 5:
        print(f'\n// NOTE: {n} presets — the on-device preset screen may need scrolling support.')


if __name__ == '__main__':
    main()
