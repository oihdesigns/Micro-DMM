p = "tools/split_boards.py"
t = open(p, encoding="utf8").read()
i = t.index("# ------------------------------------------------------------------ routing carried over")
new_tail = open("build/review/split_tail.py", encoding="utf8").read()
t = t[:i] + new_tail
old_h = """        holes={'H1': (128.0, 103.5), 'H2': (177.5, 103.5), 'H3': (128.0, 149.84), 'H4': (177.5, 149.84)},
        moves={},
        shift=set(),
        keep_x=(124.8, 180.7),
        texts=[('HI-Z +/-10V', 155.4, 101.2, 0.9), ('ISOLATION BARRIER - NO COPPER', 152.0, 129.2, 0.8),
               ('IsoHiZShield rev A', 164.0, 143.0, 1.2, True), ('A3/A2=Z A6=FLT', 159.7, 148.4, 0.8),
               ('VDD 3V3|5V', 130.5, 143.9, 0.8)],"""
new_h = """        holes={'H1': (128.0, 103.5), 'H2': (177.5, 103.5), 'H3': (127.5, 139.5), 'H4': (177.5, 149.84)},
        moves={},
        shift=set(),
        keep_x=(124.8, 180.7),
        amc_x=[172.0],
        zones={'GND': 'GND', 'HGND_Z': (131.0, 100.35, 179.0, 127.5), 'ISO_BARRIER': (124.5, 127.5, 181.0, 134.5),
               'GAP_V_Z': (124.5, 100.0, 131.0, 127.5), 'GAP_Z_I': (179.0, 100.0, 181.0, 127.5)},
        texts=[('HI-Z 10V', 164, 104, 0.8), ('RG', 159.9, 123.3, 0.8), ('ISOLATION BARRIER - NO COPPER', 154.4, 131, 0.8),
               ('IsoHiZShield rev A', 164.9, 143.5, 1.0, True), ('A3/A2=Z A6=FLT', 159.7, 148.4, 0.8),
               ('VDD 3V3|5V', 130.5, 143.9, 0.8)],"""
old_v = """        moves={'R1': (155.0, 146.2, 0)},
        shift=set(CUR),
        keep_x=(100.3, 168.3),
        texts=[('+/-200V MAX', 122.5, 101.2, 0.9), ('CURRENT', 142.0, 101.2, 0.9),
               ('ISOLATION BARRIER - NO COPPER', 134.3, 129.2, 0.8),
               ('IsoVIShield rev A', 114.0, 146.6, 1.2, True), ('A0/A1=V A4/A5=I A6=FLT', 157.0, 148.4, 0.8),
               ('VDD 3V3|5V', 130.5, 143.9, 0.8)],"""
new_v = """        moves={'R1': (155.0, 146.2, 0)},
        shift=set(CUR),
        keep_x=(100.0, 127.5),
        amc_x=[113.5, 192.0 + DX_I],
        zones={'GND': 'GND', 'HGND_V': (100.35, 100.35, 127.0, 127.5), 'HGND_I': (131.0, 100.35, 149.3, 127.5),
               'I_SHP': (190.1 + DX_I, 101.0, 193.5 + DX_I, 113.5), 'I_SHN': (185.2 + DX_I, 101.0, 188.5 + DX_I, 113.5),
               'ISO_BARRIER': (100.0, 127.5, 168.58, 134.5), 'GAP_V_Z': (127.0, 100.0, 131.0, 127.5)},
        texts=[('+/-200V MAX', 112.4, 102.4, 0.8), ('10 mOhm', 200.3 + DX_I, 109.4, 0.8),
               ('ISOLATION BARRIER - NO COPPER', 157.0, 131, 0.8),
               ('IsoVIShield rev A', 116, 151.6, 1.0, True), ('A0/A1=V A4/A5=I A6=FLT', 157.0, 148.4, 0.8),
               ('VDD 3V3|5V', 130.5, 143.9, 0.8)],"""
assert old_h in t and old_v in t
t = t.replace(old_h, new_h).replace(old_v, new_v)
open(p, "w", encoding="utf8").write(t)
print("patched")
