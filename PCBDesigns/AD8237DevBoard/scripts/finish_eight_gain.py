"""Local DRC cleanup of the incremental draft; preserves the source layout."""
import json, math
from pathlib import Path
import pcbnew as k
ROOT=Path(__file__).resolve().parents[1];folder=ROOT/'tmp/eight-gain'
path=folder/'AD8237DevBoard.kicad_pcb'
b=k.LoadBoard(str(path));removed=[]
def xy(p):return k.ToMM(p.x),k.ToMM(p.y)
def same(a,c):return math.dist(xy(a),xy(c))<.00001
report=json.loads((folder/'draft-drc.json').read_text())
items={t.m_Uuid.AsString():t for t in b.GetTracks()}
for violation in report['violations']:
    if violation['type'] in ['track_dangling','via_dangling']:
        ident=violation['items'][0]['uuid']
        if ident in items:
            t=items.pop(ident);b.Remove(t);removed.append(t)
    if violation['type']=='hole_to_hole':
        va,vb=[items.get(i['uuid']) for i in violation['items']]
        if va is None or vb is None or va.GetNetname()!=vb.GetNetname():continue
        # Preserve original fanout vias at y ending in .025 or .075 mm.
        a,bpos=xy(va.GetPosition()),xy(vb.GetPosition())
        a_fixed=abs(a[1]*40-round(a[1]*40))<.0001 and abs(a[1]*10-round(a[1]*10))>.001
        keep,drop=(va,vb) if a_fixed else (vb,va)
        src,dst=drop.GetPosition(),keep.GetPosition()
        for t in b.GetTracks():
            if isinstance(t,k.PCB_VIA):continue
            if t.GetNetname()!=drop.GetNetname():continue
            if same(t.GetStart(),src):t.SetStart(dst)
            if same(t.GetEnd(),src):t.SetEnd(dst)
        items.pop(drop.m_Uuid.AsString(),None);b.Remove(drop);removed.append(drop)
fp={f.GetReference():f for f in b.GetFootprints()}
# A local footprint clearance accurately reflects this package's 0.15 mm gaps.
fp['U3'].SetLocalClearance(k.FromMM(.15))
for pad in fp['U3'].Pads():
    if pad.GetNumber()=='5':
        pad.GetNet().SetNetname('unconnected-(U3-P3{slash}INT-Pad5)')
for ref,x,y in [('R29',143,116.1),('R22',137.5,132),('R27',128,146.7),('U2',133,116.0)]:
    fp[ref].Reference().SetPosition(k.VECTOR2I(k.FromMM(x),k.FromMM(y)))
k.SaveBoard(str(path),b)
print('Cleaned',len(removed),'redundant tracks/vias')
