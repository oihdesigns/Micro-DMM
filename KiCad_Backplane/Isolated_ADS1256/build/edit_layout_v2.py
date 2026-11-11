import os
os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))

s = open('design.py', encoding='utf-8').read()
old = """for i, (r, c, n) in enumerate([('R20', 'C32', 'AIN4'), ('R21', 'C33', 'AIN5'), ('R22', 'C34', 'AIN6'),
                               ('R23', 'C35', 'AIN7'), ('R24', 'C36', 'AINCOM')]):
    R(r, '1k', 'X_' + n, n, pn='ERJ-3EKF1001V', desc='Input protection', block=B)
    C(c, 'DNP', n, 'GNDI', pn='', dnp=True, desc='Optional filter cap (C0G)', block=B)"""
new = """for r, n in [('R20', 'AIN4'), ('R21', 'AIN5'), ('R22', 'AIN6'), ('R23', 'AIN7'), ('R24', 'AINCOM')]:
    R(r, '1k', 'X_' + n, n, pn='ERJ-3EKF1001V', desc='Input protection / current limit', block=B)"""
assert old in s
s = s.replace(old, new)
s = s.replace("  AIN4..7, AINCOM         broken out on J4 through 1k (cap footprints DNP)", "  AIN4..7, AINCOM         broken out on J4 through 1k")
open('design.py', 'w', encoding='utf-8', newline='').write(s)

g = open('gen_pcb.py', encoding='utf-8').read()
rep = [
    ("X0, Y0, W, H = 100.0, 100.0, 95.0, 52.0", "X0, Y0, W, H = 100.0, 100.0, 95.0, 62.0"),
    ("Board 95 x 52 mm", "Board 95 x 62 mm"),
    ("P['H2'] = (103.5, 148.5, 0)", "P['H2'] = (103.5, 158.5, 0)"),
    ("P['J5'] = (139.0, 149.5, 90)", "P['J5'] = (139.0, 159.5, 90)"),
    ("rect_pts(180.2, 112.6, 191.6, 116.2), prio=2))", "rect_pts(180.2, 125.1, 191.6, 128.7), prio=2))"),
    ("rect_pts(180.2, 117.9, 191.6, 121.3), prio=2))", "rect_pts(180.2, 130.4, 191.6, 133.8), prio=2))"),
    ("['polygon', rect_pts(182.9, 112.0, 192.0, 121.9)]])", "['polygon', rect_pts(182.9, 124.5, 192.0, 134.4)]])"),
    ("HV_BOX = (168.4, 131.4, X0 + W, 148.0)", "HV_BOX = (168.4, 140.4, X0 + W, 157.4)"),
    ("for ya, yb in ((111.6, 120.6), (133.4, 151.5)):", "for ya, yb in ((111.6, 120.6), (133.4, 161.5)):"),
    ("gtext('HOST', 110.0, 150.2, 1.0)", "gtext('HOST', 110.0, 160.2, 1.0)"),
    ("gtext('ISOLATED  (COM / GNDI)', 160.0, 150.6, 1.0)", "gtext('ISOLATED  (COM / GNDI)', 165.0, 160.6, 1.0)"),
    ("gtext('HV +/-250V', 178.0, 131.6, 1.0, bold=True)", "gtext('HV +/-250V', 178.0, 140.6, 1.0, bold=True)"),
    ("gtext('+', 184.5, 136.9, 1.2, bold=True)", "gtext('+', 184.5, 146.0, 1.2, bold=True)"),
    ("gtext('-', 184.5, 142.0, 1.2, bold=True)", "gtext('-', 184.5, 151.1, 1.2, bold=True)"),
    ("gtext('SHUNT 0.1R', 178.5, 110.6, 0.9)", "gtext('SHUNT 0.1R', 186.0, 122.6, 0.9)"),
    ("gtext('I+', 184.4, 114.5, 0.8)", "gtext('I+', 184.4, 127.0, 0.8)"),
    ("gtext('I-', 184.4, 119.6, 0.8)", "gtext('I-', 184.4, 132.1, 0.8)"),
    ("gtext('JP1: I- = COM', 181.5, 125.2, 0.8)", "gtext('JP1: I- = COM', 181.0, 137.4, 0.8)"),
    ("items.append(gtext(t, 167.5 + 2.54 * i, 105.4, 0.8))", "items.append(gtext(t, 185.28 - 2.54 * i, 105.4, 0.8))"),
    ("items.append(gtext(t, 139.0 + 2.54 * i, 147.0, 0.8))", "items.append(gtext(t, 139.0 + 2.54 * i, 157.0, 0.8))"),
]
for a, b in rep:
    assert a in g, a
    g = g.replace(a, b)
a = g.index("# ---------------- analog breakout (top right)")
b = g.index("SILK_REFS")
g = g[:a] + """# ---------------- analog breakout (top right); rot 270 so pin1 (A4) is rightmost and the lanes to U6 nest
P['J4'] = (185.28, 102.8, 270)
for i, r in enumerate(['R20', 'R21', 'R22', 'R23', 'R24']):
    P[r] = (185.28 - 2.54 * i, 107.6, 270)
P['H3'] = (191.5, 103.5, 0)
P['H4'] = (191.5, 158.5, 0)
# ---------------- shunt (right edge, middle); terminal blocks rot 270 = wire entry at the board edge
P['J3'] = (189.9, 127.0, 270)          # pin1 I+ (y 127.0), pin2 I- (y 132.08)
P['RS1'] = (181.5, 129.5, 270)         # pad1 I+ y 127.1, pad2 I- y 131.9
P['R18'] = (177.0, 126.5, 180)
P['R19'] = (177.0, 132.5, 180)
P['C31'] = (173.5, 129.5, 270)
P['JP1'] = (181.5, 135.5, 0)
# ---------------- +/-250 V divider (bottom right)
P['J2'] = (189.9, 146.0, 270)          # pin1 HV+ (y 146.0), pin2 HV- (y 151.08)
for r, x in (('R10', 182.8), ('R11', 177.0), ('R12', 171.2), ('R13', 166.0)):
    P[r] = (x, 143.0, 180)
for r, x in (('R14', 182.8), ('R15', 177.0), ('R16', 171.2), ('R17', 166.0)):
    P[r] = (x, 154.0, 180)
P['C30'] = (166.5, 148.5, 270)

""" + g[b:]
open('gen_pcb.py', 'w', encoding='utf-8', newline='').write(g)

st = open('stitch_planes.py', encoding='utf-8').read()
for a, b in [("'/GND': (100.9, 100.9, 118.6, 151.1),", "'/GND': (100.9, 100.9, 118.6, 161.1),"),
             ("'/GNDI': (125.4, 100.9, 194.1, 151.1),", "'/GNDI': (125.4, 100.9, 194.1, 161.1),"),
             ("HV_BOX = (168.4, 131.4, 195.0, 148.0)", "HV_BOX = (168.4, 140.4, 195.0, 157.4)"),
             ("HOLES = [(103.5, 103.5), (103.5, 148.5), (191.5, 103.5), (191.5, 148.5)]",
              "HOLES = [(103.5, 103.5), (103.5, 158.5), (191.5, 103.5), (191.5, 158.5)]")]:
    assert a in st, a
    st = st.replace(a, b)
open('stitch_planes.py', 'w', encoding='utf-8', newline='').write(st)
for f in ('route_pass.py', 'retry.py'):
    t = open(f, encoding='utf-8').read()
    assert "pt(196, 153), pt(99, 153)" in t, f
    open(f, 'w', encoding='utf-8', newline='').write(t.replace("pt(196, 153), pt(99, 153)", "pt(196, 163), pt(99, 163)"))
r = open('route_all.py', encoding='utf-8').read()
a = r.index("ORDER = [")
b = r.index("]\n", a) + 2
r = r[:a] + """ORDER = [
    # LDO locals + rails first (LDO OUT pins get boxed in otherwise)
    '/FB_A', '/NR_A', '/FB_D', '/NR_D', '/+5VA', '/+3V3I', '/+9VI', '/+5V_F', '/+5V', '/VIO',
    # breakout lanes (nest into the top of U6's analog column)
    '/AIN7', '/AIN6', '/AIN5', '/AIN4', '/X_AIN4', '/X_AIN5', '/X_AIN6', '/X_AIN7', '/X_AINCOM',
    # crystal, GPIO, SPI
    '/XTAL2', '/XTAL1', '/RESET', '/D0', '/D1', '/D2', '/D3',
    '/DIN', '/SCLK', '/SCLK_I', '/CS', '/DRDY', '/DOUT', '/SYNC',
    '/SYNC_H', '/CS_H', '/DIN_H', '/SCLK_H', '/DRDY_H', '/DOUT_H',
    # analog inputs
    '/AIN2', '/AIN3', '/AIN0', '/AIN1', '/HV_P', '/HV_PA', '/HV_PB', '/HV_N', '/HV_NA', '/HV_NB',
    '/ISH_P', '/ISH_N', '/AINCOM',
    # reference / COM
    '/VREF', '/NR_REF', '/VCM_SET', '/COM_DRV', '/COM',
]
""" + r[b:]
open('route_all.py', 'w', encoding='utf-8', newline='').write(r)
print('ok')
