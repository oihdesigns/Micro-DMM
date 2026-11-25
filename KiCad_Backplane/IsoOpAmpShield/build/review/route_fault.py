import os, sys
sys.path.insert(0, 'tools')
import finish_split as f
f.ROUTE_MS = (60000, 60000)
pcb = os.path.join(f.ROOT, 'boards', 'IsoVIShield', 'IsoVIShield.kicad_pcb').replace(os.sep, '/')
f.route('IsoVIShield', pcb)
d = f.drc(pcb)
print('unconnected', len(d['unconnected_items']), [v['type'] for v in d['violations']])
