"""One-time, incremental Rev B -> C board migration from the saved user layout.

Writes a draft under tmp/eight-gain, never regenerates the user's PCB.
Run with KiCad's bundled Python. Review and DRC before promoting the draft.
"""
from pathlib import Path
import ast, json, math, heapq, itertools, os, shutil
import xml.etree.ElementTree as ET
import numpy as np
import pcbnew as k
from design import ROOT, NAME, SHEET, uid, datasheets

os.environ['KICAD_CONFIG_HOME']=str(ROOT/'tmp/kicad-config')
DEST=ROOT/'tmp/eight-gain'
b=k.LoadBoard(str(ROOT/'archive/pre-eight-gain'/(NAME+'.kicad_pcb')))
mm=k.FromMM
def pos(x,y):return k.VECTOR2I(mm(float(x)),mm(float(y)))
def key(item):return item.m_Uuid.AsString()
def xy(item):
    p=item.GetPosition();return k.ToMM(p.x),k.ToMM(p.y)
parts={p['ref']:p for p in json.loads((ROOT/'scripts/components.json').read_text())}
fps={f.GetReference():f for f in b.GetFootprints()}
nets={n.GetNetname():n for n in b.GetNetsByNetcode().values()}
root=ET.parse(DEST/'netlist.xml').getroot()
pin_nets={}
for net in root.findall('nets/net'):
    name=net.get('name')
    if name not in nets:
        n=k.NETINFO_ITEM(b,name);b.Add(n);nets[name]=n
    for node in net.findall('node'):pin_nets[node.get('ref'),node.get('pin')]=name

original_tracks={key(t) for t in b.GetTracks()}
original_positions={r:[*xy(f),f.GetOrientationDegrees()] for r,f in fps.items()}
old_mux=fps['U2'];old_id=old_mux.m_Uuid
b.Remove(old_mux)
placements={'U2':(133,119.5,0),'R21':(129.5,132,0),'R22':(134.5,132,0),
 'R23':(129.5,135,0),'R24':(134.5,135,0),'R25':(129.5,138,0),'R26':(134.5,138,0),
 'R27':(128,145,0),'R28':(133,145,0),'R29':(143,114.5,0)}
for ref,(x,y,a) in placements.items():
    p=parts[ref]
    f=k.FootprintLoad(str(ROOT/'AD8237DevBoard.pretty'),p['fp'].split(':')[1])
    f.SetFPID(k.LIB_ID('AD8237DevBoard',p['fp'].split(':')[1]))
    f.SetReference(ref);f.SetValue(p['value'])
    path=k.KIID_PATH();path.push_back(k.KIID(SHEET));path.push_back(k.KIID(p['uuid']));f.SetPath(path)
    f.SetSheetfile(NAME+'.kicad_sch');f.SetSheetname('')
    f.SetField('Assembly',p['notes']);f.GetField('Assembly').SetVisible(False)
    f.SetPosition(pos(x,y));f.SetOrientationDegrees(a)
    f.Value().SetVisible(False)
    f.Reference().SetTextSize(pos(.8,.8));f.Reference().SetTextThickness(mm(.12))
    f.Reference().SetPosition(pos(x,y-1.6 if ref!='U2' else y-3.2))
    b.Add(f);fps[ref]=f
# Keep the same decoupler, close to the new VDD pin, with enough courtyard space.
fps['C10'].Move(pos(.5,1.3))

pads={}
for ref,f in fps.items():
    p=parts[ref]
    for field,value in [('Value',p['value']),('Datasheet',datasheets.get(p['kind'],'')),('Assembly',p['notes'])]:
        f.SetField(field,value)
        if field!='Value':f.GetField(field).SetVisible(False)
    for pad in f.Pads():
        pads[ref,pad.GetNumber()]=pad
        name=pin_nets.get((ref,pad.GetNumber()))
        if name:pad.SetNet(nets[name])
    if ref in ['U2','U3']:
        for pad in f.Pads():
            if pad.GetNetname()=='/GND':pad.SetLocalZoneConnection(k.ZONE_CONNECTION_FULL)
for z in b.Zones():z.UnFill()

# Remove old mux fanout locally and tracks that collide with the added pads.
# All remaining tracks retain their original UUIDs and geometry.
local_box=k.SHAPE_RECT(mm(128.3),mm(116.2),mm(9.4),mm(6.6))
changed_pads=[p for (r,_),p in pads.items() if r in placements or r=='C10']
removed=[]
removed_objects=[]
print('Before local track removal',len(b.GetDrawings()),flush=True)
for t in list(b.GetTracks()):
    via=isinstance(t,k.PCB_VIA)
    shape=t.GetEffectiveShape(t.GetLayer())
    remove=local_box.Collide(shape,mm(.05))
    if not remove and (via or t.GetLayer()==k.F_Cu):
        for p in changed_pads:
            if p.GetNetname()!=t.GetNetname() and p.GetEffectiveShape(k.F_Cu).Collide(shape,mm(.215)):
                remove=True;break
    # Existing SCL via violates the U3 pad-5 clearance; reroute its layer change.
    if via and t.GetNetname()=='/SCL' and math.dist(xy(t),(149.9,114.173))<.01:remove=True
    if remove:removed.append(key(t));b.Remove(t);removed_objects.append(t)

for t in b.GetDrawings():
    if not isinstance(t,k.PCB_TEXT):continue
    txt=t.GetText()
    if 'Rev B' in txt:t.SetText(txt.replace('Rev B','Rev C'))
    if txt.startswith('GAIN 0/'):
        t.SetText('GAIN 0..7: 1 / 2.5 / 5.02 / 10.09\n25.05 / 49.78 / 101 / 1001\nPower-up gain: 1; use LOW BW\nJP1: EXT / VIO analog supply\nJP3: GND / MID / EXT reference\nJ4: GND / VIO / SDA / SCL\nVIO must match controller voltage')
        t.SetTextSize(pos(.9,.9))

def track(net,a,c,layer=k.F_Cu,width=.2):
    if a==c:return
    t=k.PCB_TRACK(b);t.SetStart(pos(*a));t.SetEnd(pos(*c));t.SetWidth(mm(width));t.SetLayer(layer);t.SetNet(nets[net]);b.Add(t)
def via(net,x,y):
    v=k.PCB_VIA(b);v.SetPosition(pos(x,y));v.SetWidth(mm(.6));v.SetDrill(mm(.3));v.SetLayerPair(k.F_Cu,k.B_Cu);v.SetNet(nets[net]);b.Add(v)

# Reuse just the geometry-aware search functions; never execute the old builder.
router=ast.parse((ROOT/'scripts/build_pcb.py').read_text())
names={'grid','world','layers','circle','rect','obstacles','findpath','pathdraw'}
exec(compile(ast.Module(body=[n for n in router.body if isinstance(n,ast.FunctionDef) and n.name in names],type_ignores=[]),'router-functions','exec'))
STEP=.1;NX=601;NY=501;CLEAR=.215;W=.2

def components(net):
    b.BuildConnectivity();conn=b.GetConnectivity()
    items=[p for p in pads.values() if p.GetNetname()==net]+[t for t in b.GetTracks() if t.GetNetname()==net]
    lookup={key(i):i for i in items};unseen=set(lookup);groups=[]
    while unseen:
        todo=[unseen.pop()];group=[]
        while todo:
            ident=todo.pop();item=lookup[ident];group.append(item)
            for other in conn.GetConnectedItems(item):
                oid=key(other)
                if oid in unseen:unseen.remove(oid);todo.append(oid)
        groups.append(group)
    return groups
def anchors(group):
    out=set()
    for item in group:
        if isinstance(item,k.PAD):
            pp=[item.GetPosition()];ls=layers(item)
        elif isinstance(item,k.PCB_VIA):pp=[item.GetPosition()];ls=[0,1]
        else:pp=[item.GetStart(),item.GetEnd()];ls=[0 if item.GetLayer()==k.F_Cu else 1]
        for p in pp:
            for l in ls:out.add((k.ToMM(p.x),k.ToMM(p.y),l))
    return out
def connect(net):
    for iteration in range(100):
        groups=components(net)
        if len(groups)<=1:return
        sets=[anchors(g) for g in groups]
        choices=[]
        for i in range(len(sets)):
            for j in range(i):
                # Short candidate pairs favor preserving existing trace branches.
                pairs=sorted(((math.hypot(a[0]-c[0],a[1]-c[1])+(0 if a[2]==c[2] else 2),a,c) for a in sets[i] for c in sets[j]),key=lambda x:x[0])
                for al,cl in [(0,0),(1,1),(0,1),(1,0)]:
                    choices.extend([p for p in pairs if p[1][2]==al and p[2][2]==cl][:3])
        choices.sort(key=lambda x:x[0]);last=None
        for _,a,c in choices[:80]:
            start=(*grid(*a[:2]),a[2]);goal=(*grid(*c[:2]),c[2])
            try:path=findpath(net,[start],[goal])
            except RuntimeError as e:last=e;continue
            pathdraw(net,path)
            track(net,a[:2],world(*start[:2]),k.F_Cu if a[2]==0 else k.B_Cu)
            track(net,c[:2],world(*goal[:2]),k.F_Cu if c[2]==0 else k.B_Cu)
            break
        else:raise RuntimeError(f'Unable to connect {net}: {last}')
    raise RuntimeError('No connectivity convergence '+net)

if __name__=='__main__':
    print('Retained tracks',len(b.GetTracks()),'removed locally',len(removed),flush=True)
    # Reserve an escape for every mux pin before any longer route can box it in.
    for pad in fps['U2'].Pads():
        x,y=xy(pad);n=int(pad.GetNumber())
        ex=(131.5 if n%2 else 132.5) if x<133 else (133.5 if n%2 else 134.5)
        track(pad.GetNetname(),(x,y),(ex,y));via(pad.GetNetname(),ex,y)
        track(pad.GetNetname(),(ex,y),(128.5 if x<133 else 137.5,y),k.B_Cu)
    k.SaveBoard(str(DEST/'unrouted.kicad_pcb'),b)
    order=['GAIN10','GAIN101','GAIN1001','GAIN25','GAIN50','GAIN5','GAIN2_5','FB','UNITY','SEL2','SEL0','SEL1','VS','OUT_RAW','REF','SCL']
    order+=sorted(n.lstrip('/') for n in nets if n.startswith('/') and n.lstrip('/') not in order and n!='/GND')
    for name in order:
        net='/'+name
        if net not in nets:continue
        connect(net);print('Connected',net,flush=True)
        k.SaveBoard(str(DEST/'routing-progress.kicad_pcb'),b)
    # Ground pads join the existing two planes after refill. Add a local return
    # via at each newly added / moved IC ground pad where clear.
    for ref,num in [('C10','2'),('R29','2')]:
        p=pads[ref,num];x,y=xy(p);vb=obstacles('/GND',.3);done=False
        for radius in [1,1.4,1.8,2.3]:
            for dx,dy in [(-1,0),(1,0),(0,1),(0,-1),(-1,1),(1,1)]:
                gx,gy=grid(x+dx*radius,y+dy*radius)
                if not(0<gx<NX and 0<gy<NY) or vb[0,gx,gy] or vb[1,gx,gy]:continue
                try:path=findpath('/GND',[(*grid(x,y),0)],[(gx,gy,0)])
                except RuntimeError:continue
                pathdraw('/GND',path);track('/GND',(x,y),world(*grid(x,y)));via('/GND',*world(gx,gy));done=True;break
            if done:break
        if not done:print('Ground via deferred to plane',ref,num,flush=True)
    draft=DEST/(NAME+'.kicad_pcb');k.SaveBoard(str(draft),b)
    shutil.copyfile(ROOT/(NAME+'.kicad_pro'),DEST/(NAME+'.kicad_pro'))
    shutil.copyfile(ROOT/(NAME+'.kicad_sch'),DEST/(NAME+'.kicad_sch'))
    shutil.copyfile(ROOT/(NAME+'.kicad_dru'),DEST/(NAME+'.kicad_dru'))
    report={'original_tracks':len(original_tracks),'retained_tracks':sum(key(t) in original_tracks for t in b.GetTracks()),'removed_track_uuids':removed,
        'moved_existing_footprints':{r:{'before':old,'after':[*xy(fps[r]),fps[r].GetOrientationDegrees()]} for r,old in original_positions.items() if old!=[*xy(fps[r]),fps[r].GetOrientationDegrees()]},'added_footprints':sorted(set(fps)-set(original_positions))}
    (DEST/'preservation.json').write_text(json.dumps(report,indent=2))
    print('Draft saved:',draft,flush=True)
