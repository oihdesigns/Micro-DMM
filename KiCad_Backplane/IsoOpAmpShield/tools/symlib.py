"""Load KiCad library symbols (stock + project) and flatten `extends`."""
import copy
import os
import sexp
from sexp import Q

STOCK = 'C:/Program Files/KiCad/10.0/share/kicad/symbols'
HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_LIB = os.path.join(HERE, '..', 'IsoOpAmpShield.kicad_sym')

_cache = {}


def _lib(libname):
    if libname not in _cache:
        path = PROJECT_LIB if libname == 'IsoOpAmpShield' else os.path.join(STOCK, libname + '.kicad_sym')
        t = sexp.parse(open(path, encoding='utf-8').read())[0]
        _cache[libname] = {s[1]: s for s in sexp.find(t, 'symbol')}
    return _cache[libname]


def _rename_units(sym, old, new):
    for e in sym:
        if isinstance(e, list) and e and e[0] == 'symbol' and isinstance(e[1], str) and e[1].startswith(old + '_'):
            e[1] = Q(new + e[1][len(old):])


def get(lib_id):
    """Return a flattened deep copy of the symbol, top-level name = lib_id."""
    libname, name = lib_id.split(':')
    lib = _lib(libname)
    s = copy.deepcopy(lib[name])
    ext = sexp.find1(s, 'extends')
    if ext:
        parent = copy.deepcopy(lib[ext[1]])
        _rename_units(parent, ext[1], name)
        # child properties override parent's
        cprops = {p[1]: p for p in sexp.find(s, 'property')}
        out = []
        for e in parent:
            if isinstance(e, list) and e and e[0] == 'property' and e[1] in cprops:
                out.append(cprops.pop(e[1]))
            else:
                out.append(e)
        # append child-only props after the last property
        idx = max(i for i, e in enumerate(out) if isinstance(e, list) and e and e[0] == 'property')
        for p in cprops.values():
            idx += 1
            out.insert(idx, p)
        s = out
    s[1] = Q(lib_id)
    if lib_id == 'IsoOpAmpShield:AMC3330':
        # DIAG is open-drain (datasheet); stock symbol calls it 'output', which makes the
        # wired-OR ISO_FAULT net an ERC output-output conflict.
        for sub in sexp.find(s, 'symbol'):
            for p in sexp.find(sub, 'pin'):
                if str(sexp.find1(p, 'name')[1]) == 'DIAG':
                    p[1] = 'open_collector'
    return s


def pins(sym):
    """[(unit, number, name, etype, x, y, angle, hidden)] from a flattened symbol."""
    res = []
    for sub in sexp.find(sym, 'symbol'):
        parts = sub[1].split('_')
        unit = int(parts[-2])
        for p in sexp.find(sub, 'pin'):
            at = sexp.find1(p, 'at')
            hidden = 'hide' in p or any(isinstance(e, list) and e[:2] == ['hide', 'yes'] for e in p)
            res.append((unit, str(sexp.find1(p, 'number')[1]), str(sexp.find1(p, 'name')[1]), p[1],
                        float(at[1]), float(at[2]), int(float(at[3])), hidden))
    return res


def units(sym):
    us = sorted({u for (u, *_r) in pins(sym)})
    return [u for u in us if u != 0] or [1]


def prop(sym, name):
    for p in sexp.find(sym, 'property'):
        if p[1] == name:
            return str(p[2])
    return ''
