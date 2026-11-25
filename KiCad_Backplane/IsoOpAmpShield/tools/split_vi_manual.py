"""Manual ISO_FAULT link on IsoVIShield: R1 pad 1 -> via -> B.Cu down to the kept U2->J2 fault run."""
import uuid, sys
f = sys.argv[1]
t = open(f, encoding='utf8').read()
ns = uuid.UUID('9e3a5c1b-7d24-4f0e-b6a8-2c51d0e4f713')
u = lambda k: str(uuid.uuid5(ns, 'IsoVIShield/manual/' + k))
seg = '\t(segment\n\t\t(start %s %s)\n\t\t(end %s %s)\n\t\t(width 0.2)\n\t\t(layer "%s")\n\t\t(net "/ISO_FAULT")\n\t\t(uuid "%s")\n\t)\n'
via = '\t(via\n\t\t(at %s %s)\n\t\t(size 0.6)\n\t\t(drill 0.3)\n\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "/ISO_FAULT")\n\t\t(uuid "%s")\n\t)\n'
add = seg % (154, 146.2, 154, 147.0, 'F.Cu', u('s1')) + via % (154, 147.0, u('v1')) + seg % (154, 147.0, 154, 149.475, 'B.Cu', u('s2'))
if u('v1') not in t:
    i = t.rindex('\t(embedded_fonts')
    t = t[:i] + add + t[i:]
    open(f, 'w', encoding='utf8', newline='').write(t)
print('added')
