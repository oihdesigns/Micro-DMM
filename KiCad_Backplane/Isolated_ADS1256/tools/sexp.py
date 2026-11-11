"""Tiny s-expression reader/writer for KiCad files (good enough for symbols/footprints)."""
import re

_tok = re.compile(r'\s*(?:(\()|(\))|("(?:[^"\\]|\\.)*")|([^\s()"]+))', re.S)


class Q(str):
    """A quoted string atom."""
    pass


def _unq(s):
    out = []
    i = 0
    while i < len(s):
        c = s[i]
        if c == '\\' and i + 1 < len(s):
            n = s[i + 1]
            out.append({'n': '\n', 't': '\t'}.get(n, n))
            i += 2
        else:
            out.append(c)
            i += 1
    return ''.join(out)


def parse(text):
    stack = [[]]
    pos = 0
    n = len(text)
    while pos < n:
        m = _tok.match(text, pos)
        if not m:
            if text[pos:].strip() == '':
                break
            raise ValueError('parse error at %d: %r' % (pos, text[pos:pos + 40]))
        pos = m.end()
        if m.group(1):
            stack.append([])
        elif m.group(2):
            item = stack.pop()
            stack[-1].append(item)
        elif m.group(3) is not None:
            stack[-1].append(Q(_unq(m.group(3)[1:-1])))
        else:
            stack[-1].append(m.group(4))
    return stack[0]


def _q(s):
    return '"' + s.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n') + '"'


def dump(x, indent=0):
    if isinstance(x, list):
        if not x:
            return '()'
        if all(not isinstance(e, list) for e in x):
            return '(' + ' '.join(dump(e) for e in x) + ')'
        head = []
        i = 0
        while i < len(x) and not isinstance(x[i], list):
            head.append(dump(x[i]))
            i += 1
        s = '(' + ' '.join(head)
        for e in x[i:]:
            s += '\n' + '\t' * (indent + 1) + dump(e, indent + 1)
        s += '\n' + '\t' * indent + ')'
        return s
    if isinstance(x, Q):
        return _q(x)
    if isinstance(x, bool):
        return 'yes' if x else 'no'
    if isinstance(x, float):
        s = ('%.4f' % x).rstrip('0').rstrip('.')
        return '0' if s in ('-0', '') else s
    return str(x)


def find(x, key):
    return [e for e in x if isinstance(e, list) and e and e[0] == key]


def find1(x, key):
    r = find(x, key)
    return r[0] if r else None
