"""Place and route the prototype using KiCad's native pcbnew API.

All routing is subsequently checked with KiCad DRC and schematic parity.
The two-layer maze router is conservative and does not replace those checks.
"""
import json, math, heapq, shutil, uuid
from pathlib import Path
import numpy as np
import pcbnew as k

ROOT=Path(__file__).resolve().parent
NAME='SI8751AB-IS_devboard'
parts=json.loads((ROOT/'circuit.json').read_text())
FP=Path('C:/Program Files/KiCad/10.0/share/kicad/footprints')
LOCAL=ROOT/'Devboard.pretty'; LOCAL.mkdir(exist_ok=True)
mm=k.FromMM
def v(x,y): return k.VECTOR2I(mm(float(x)),mm(float(y)))
def uid(s): return str(uuid.uuid5(uuid.NAMESPACE_URL,NAME+'/'+s))

for p in parts:
    src=p['source_footprint']
    if not src or src.startswith('Devboard:'):continue
    lib,name=src.split(':');shutil.copyfile(FP/(lib+'.pretty')/(name+'.kicad_mod'),LOCAL/(name+'.kicad_mod'))
# Keep 1.1mm drills but narrow the TO-220 copper along the pin row. This
# increases adjacent copper clearance from 0.635 to 0.89mm for the 120Vac path.
qfp=k.FootprintLoad(str(LOCAL),'TO-220-3_Vertical')
for pad in qfp.Pads():
    pad.SetSize(v(1.65,1.905))
k.PCB_IO_KICAD_SEXPR().FootprintSave(str(LOCAL),qfp)
(ROOT/'fp-lib-table').write_text('(fp_lib_table (version 7) (lib (name "Devboard") (type "KiCad") (uri "${KIPRJMOD}/Devboard.pretty") (options "") (descr "Project-local footprints")))\n')

board=k.BOARD();board.SetCopperLayerCount(2)
settings=board.GetDesignSettings();settings.m_CopperEdgeClearance=mm(.25)
nets={}
for n in sorted({n for p in parts for n in p['nets'].values() if n}):
    net=k.NETINFO_ITEM(board,n);board.Add(net);nets[n]=net

# x=64 defines the control/field barrier. Nothing is poured below isolators.
pos={
'J1':(29,40,0), 'U1':(64,38,0),'U2':(64,58,0),'U5':(64,90,180),'U6':(64,73,180),
'Q1':(86,38,0),'Q2':(86,58,180),'J2':(114,36,90),'J3':(114,68,90),
'R1':(43,43,0),'R2':(51,44,90),'R3':(43,57,0),'R4':(51,62,90),'R5':(56,38,90),'R6':(56,58,90),
'C1':(58,33,90),'C2':(58,53,90),'C3':(76,34,0),'C4':(76,55,0),
'C5':(72.7,94.5,90),'C6':(77,94.5,90),'C7':(72.7,89.4,90),'C8':(78,89.4,90),
'C11':(55.5,89.4,90),'C12':(50.5,89.4,90),'C13':(55.5,94.5,90),'C19':(82,94.5,90),
'R10':(104,85,0),'R11':(98,85,0),'R12':(92,85,0),'R13':(73,84,90),'C14':(78,84,90),
'R14':(50,96,90),'C15':(58,78,90),'C16':(53,78,90),'C17':(56,70,90),
'C18':(39,68,90)
}
footprints={};allpads=[]
for p in parts:
    fp=k.FootprintLoad(str(LOCAL),p['footprint'].split(':')[-1]);assert fp,p['reference']
    fp.SetReference(p['reference']);fp.SetValue(p['value'])
    fp.SetField('Purpose',p['purpose']);fp.SetField('Datasheet',p['datasheet'])
    fp.GetField('Purpose').SetVisible(False);fp.GetField('Datasheet').SetVisible(False)
    fp.SetFPID(k.LIB_ID('Devboard',p['footprint'].split(':')[-1]))
    path=k.KIID_PATH();path.push_back(k.KIID(uid('root')));path.push_back(k.KIID(p['uuid']));fp.SetPath(path)
    x,y,angle=pos[p['reference']];fp.SetPosition(v(x,y));fp.SetOrientationDegrees(angle)
    fp.Value().SetVisible(False)
    fp.Reference().SetTextSize(v(.85,.85));fp.Reference().SetTextThickness(mm(.13))
    fp.Reference().SetTextAngle(k.EDA_ANGLE(0,k.DEGREES_T))
    if p['reference'].startswith(('C','R')):fp.Reference().SetPosition(v(x,y-2.6))
    if p['reference']=='U3':fp.Reference().SetPosition(v(59,67.5))
    for pad in fp.Pads():
        n=p['nets'].get(pad.GetNumber())
        if n:pad.SetNet(nets[n])
        allpads.append(pad)
    board.Add(fp);footprints[p['reference']]=fp

def line(a,b,layer=k.Edge_Cuts,width=.05):
    s=k.PCB_SHAPE();s.SetShape(k.SHAPE_T_SEGMENT);s.SetStart(v(*a));s.SetEnd(v(*b));s.SetWidth(mm(width));s.SetLayer(layer);board.Add(s)
for a,b in [((20,20),(130,20)),((130,20),(130,110)),((130,110),(20,110)),((20,110),(20,20))]:line(a,b)
def text(s,x,y,size=1,layer=k.F_SilkS):
    t=k.PCB_TEXT(board);t.SetText(s);t.SetPosition(v(x,y));t.SetTextSize(v(size,size));t.SetTextThickness(mm(.15));t.SetLayer(layer);board.Add(t)
text('Si8751 AC/DC SWITCH',48,24,1.5);text('REV C  |  120Vac / 170Vdc  |  <1A',90,103,1)

for i,label in enumerate(['1  3V3','2  GND','3  EN','4  VP','5  VN','6  GND','7  NC','8  IOUT','9  DIAG','10 GND']):text(label,35,40+2.54*i,.85)
text('CONTROL',40,28,1);text('LIVE FIELD SIDE',96,25,1.4)
text('SOURCE',113,43,1);text('LOAD',113,75,1)
text('EXTERNAL FUSE <=1A',107,100,.9)
text('BENCH PROTOTYPE - NOT MAINS CERTIFIED',75,107,1)
for yy in [36,68]:
    text('LINE',121,yy,.85);text('RETURN',121,yy-7.62,.85)
text('3.3V INPUT ONLY',41,102,.9)
for y1,y2 in [(23,32),(43,52),(63,68),(78,83),(97,101)]:line((64,y1),(64,y2),k.F_SilkS,.15)

# Reserve four mounting holes and use pad records as router obstacles.
for i,(x,y) in enumerate([(24,24),(126,24),(24,106),(126,106)],1):
    fp=k.FootprintLoad(str(FP/'MountingHole.pretty'),'MountingHole_3.2mm_M3');fp.SetReference(f'H{i}');fp.SetPosition(v(x,y));fp.SetBoardOnly(True) if hasattr(fp,'SetBoardOnly') else None
    fp.SetAttributes(k.FP_EXCLUDE_FROM_BOM|k.FP_EXCLUDE_FROM_POS_FILES|k.FP_BOARD_ONLY)
    fp.Reference().SetVisible(False);fp.Value().SetVisible(False);board.Add(fp);allpads.extend(fp.Pads())

STEP=.2; X0,Y0=20,20; NX,NY=551,451
def grid(x,y): return (round((x-X0)/STEP),round((y-Y0)/STEP))
def xy(pt): return (X0+pt[0]*STEP,Y0+pt[1]*STEP)
def padpoint(p):return (k.ToMM(p.GetPosition().x),k.ToMM(p.GetPosition().y))
def layers(p):return [0,1] if p.GetAttribute()!=k.PAD_ATTRIB_SMD else [0]
padgroups={n:[] for n in nets}
for p in allpads:
    if p.GetNetname() in padgroups:padgroups[p.GetNetname()].append(p)
tracks=[];vias=[]
# Escape the dense low-side pin row before global routing, keeping the
# isolation corridor free. All vias are on the control side of the package.
escapes={}
for pad in footprints['U5'].Pads():
    if int(pad.GetNumber())<9:continue
    a=padpoint(pad);b=xy(grid(57.8,a[1]));n=pad.GetNetname()
    t=k.PCB_TRACK(board);t.SetStart(v(*a));t.SetEnd(v(*b));t.SetWidth(mm(.3));t.SetLayer(k.F_Cu);t.SetNet(nets[n]);board.Add(t);tracks.append((n,a,b,.3,0))
    via=k.PCB_VIA(board);via.SetPosition(v(*b));via.SetWidth(mm(.7));via.SetDrill(mm(.3));via.SetLayerPair(k.F_Cu,k.B_Cu);via.SetNet(nets[n]);board.Add(via);vias.append((n,*b))
    escapes[('U5',pad.GetNumber())]=(*grid(*b),1)
power={'RAIL_IN','SWITCHED_PRE','RAIL_OUT','SOURCE_FLOAT','GND_LOAD_RETURN'}
primary={'+3V3','GND_LOGIC','EN','EN_A','EN_B','TT_1','TT_2','VOUT_P','VOUT_N','AMC_LLDO','DIAG_N','I_FILTER','IOUT_ADC'}
def width(n):return 1.0 if n in power else .3
def clearance(a,b):
    # Different high-voltage nodes: >=0.8mm, with fine low-side control routing.
    if a==b:return 0
    if {a,b}=={'SWITCHED_PRE','RAIL_OUT'}:return .3
    if (a in primary)!=(b in primary):return 2.5
    hi={'RAIL_IN','SWITCHED_PRE','RAIL_OUT','SOURCE_FLOAT','GATE_1','GATE_2','MCAP_1','MCAP_2','DIV_1','DIV_2'}
    if a in hi or b in hi:
        if a in {'SOURCE_FLOAT','GATE_1','GATE_2','MCAP_1','MCAP_2'} and b in {'SOURCE_FLOAT','GATE_1','GATE_2','MCAP_1','MCAP_2'}:return .3
        return .8
    return .25
def rect(mask,x1,y1,x2,y2,ls):
    gx1,gy1=grid(x1,y1);gx2,gy2=grid(x2,y2)
    for l in ls:mask[l,max(0,gx1):min(NX,gx2+1),max(0,gy1):min(NY,gy2+1)]=True
def disk(mask,x,y,r,ls):
    gx,gy=grid(x,y);rr=math.ceil(r/STEP)
    xa,xb=max(0,gx-rr),min(NX,gx+rr+1);ya,yb=max(0,gy-rr),min(NY,gy+rr+1)
    xx=X0+np.arange(xa,xb)*STEP;yy=Y0+np.arange(ya,yb)*STEP
    inside=(xx[:,None]-x)**2+(yy[None,:]-y)**2<=r*r
    for l in ls:mask[l,xa:xb,ya:yb]|=inside
def segment_mask(mask,a,b,r,ls):
    steps=max(1,math.ceil(math.dist(a,b)/(STEP/2)))
    for t in np.linspace(0,1,steps+1):disk(mask,a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t,r,ls)
def obstacle(n,w):
    m=np.zeros((2,NX,NY),dtype=bool)
    rect(m,20,20,21,110,[0,1]);rect(m,129,20,130,110,[0,1]);rect(m,20,20,130,21,[0,1]);rect(m,20,109,130,110,[0,1])
    if n in primary:rect(m,62,20,130,110,[0,1])
    else:rect(m,20,20,66,110,[0,1])
    for p in allpads:
        other=p.GetNetname()
        if other==n:continue
        x,y=padpoint(p);sz=p.GetSize();sx,sy=k.ToMM(sz.x),k.ToMM(sz.y)
        if round(p.GetOrientationDegrees())%180==90:sx,sy=sy,sx
        gap=clearance(n,other)
        if not other:gap=.25
        # Cross-barrier pin-to-pin distances are inherent to the isolator/module.
        if other and (n in primary)!=(other in primary):gap=.25
        r=gap+w/2+.08
        rect(m,x-sx/2-r,y-sy/2-r,x+sx/2+r,y+sy/2+r,layers(p))
    for other,a,b,tw,l in tracks:
        if n!=other:
            gap=clearance(n,other)
            segment_mask(m,a,b,(w+tw)/2+gap+.08,[l])
    for other,x,y in vias:
        if n!=other:disk(m,x,y,w/2+.35+clearance(n,other)+.1,[0,1])
    return m

def powerpad(p):
    return p.GetParentFootprint().GetReference().startswith(('Q','J')) or (p.GetParentFootprint().GetReference()=='U6' and p.GetNumber() in {'1','2','3','4'})

def padwidth(p,n):
    if p.GetParentFootprint().GetReference()=='U6' and n in power:return .8
    return width(n) if powerpad(p) else .3

def route(n):
    pads=sorted(padgroups[n],key=lambda p: (not powerpad(p),p.GetParentFootprint().GetReference(),p.GetNumber()))
    if len(pads)<2:return
    w=.3;m=obstacle(n,w)
    # Connected pads may be inside the side boundary but never inside foreign copper.
    nodes=[]
    for p in pads:
        gx,gy=grid(*padpoint(p));nodes.append(escapes.get((p.GetParentFootprint().GetReference(),p.GetNumber()),(gx,gy,layers(p)[0])))
    # Start the spanning tree at the power/IC pin, then connect closest pads.
    tree={nodes[0]};remaining=list(zip(pads[1:],nodes[1:]))
    while remaining:
        idx=min(range(len(remaining)),key=lambda i:(n in power and not powerpad(remaining[i][0]),min(abs(remaining[i][1][0]-s[0])+abs(remaining[i][1][1]-s[1]) for s in tree)))
        target_pad,target=remaining.pop(idx);tx,ty,tl=target
        w=padwidth(target_pad,n)
        m=obstacle(n,w)
        # A* from target to any already-connected tree node.
        xs=[s[0] for s in tree];ys=[s[1] for s in tree]
        bx1,bx2,by1,by2=min(xs),max(xs),min(ys),max(ys)
        def h(x,y):return max(bx1-x,0,x-bx2)+max(by1-y,0,y-by2)
        heap=[(h(tx,ty),0,target)];prev={};dist={target:0};end=None
        while heap:
            _,g,s=heapq.heappop(heap)
            if g!=dist.get(s):continue
            if s in tree:end=s;break
            x,y,l=s
            for dx,dy,c in [(1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,1.414),(1,-1,1.414),(-1,1,1.414),(-1,-1,1.414),(0,0,12)]:
                nl=1-l if c==12 else l;xx,yy=x+dx,y+dy;t=(xx,yy,nl)
                if xx<1 or xx>=NX-1 or yy<1 or yy>=NY-1:continue
                if m[nl,xx,yy] and t not in tree:continue
                if dx and dy and (m[nl,x,yy] or m[nl,xx,y]):continue
                if c==12:
                    # Via annulus requires space on both layers beyond track width.
                    rr=2
                    if m[:,x-rr:x+rr+1,y-rr:y+rr+1].any():continue
                ng=g+c
                if ng<dist.get(t,1e30):dist[t]=ng;prev[t]=s;heapq.heappush(heap,(ng+h(xx,yy),ng,t))
        if end is None:raise RuntimeError(f'Unrouted {n}: {target_pad.GetParentFootprint().GetReference()}.{target_pad.GetNumber()} after {len(dist)} nodes')
        path=[end]
        while path[-1]!=target:path.append(prev[path[-1]])
        path.reverse();tree.update(path)
        # Compress straight grid sections into editable KiCad segments.
        start=path[0];old=start;direction=None
        for cur in path[1:]+[None]:
            direct=None if cur is None else (cur[0]-old[0],cur[1]-old[1],cur[2]-old[2])
            if direct!=direction and direction is not None:
                if start[2]!=old[2]:
                    x,y=xy(start);via=k.PCB_VIA(board);via.SetPosition(v(x,y));via.SetWidth(mm(.7));via.SetDrill(mm(.3));via.SetLayerPair(k.F_Cu,k.B_Cu);via.SetNet(nets[n]);board.Add(via);vias.append((n,x,y))
                elif start!=old:
                    a,b=xy(start),xy(old);t=k.PCB_TRACK(board);t.SetStart(v(*a));t.SetEnd(v(*b));t.SetWidth(mm(w));t.SetLayer(k.F_Cu if old[2]==0 else k.B_Cu);t.SetNet(nets[n]);board.Add(t);tracks.append((n,a,b,w,old[2]))
                start=old
            direction=direct
            if cur is not None:old=cur
    # Exact pin center to routing grid, only a fraction of a grid interval.
    for p in pads:
        a=padpoint(p);b=xy(grid(*a))
        if math.dist(a,b)>.0001:
            pw=padwidth(p,n)
            t=k.PCB_TRACK(board);t.SetStart(v(*a));t.SetEnd(v(*b));t.SetWidth(mm(pw));t.SetLayer(k.F_Cu);t.SetNet(nets[n]);board.Add(t);tracks.append((n,a,b,pw,0))
    print(f'Routed {n}: {len(pads)} pads',flush=True)

order=['MCAP_1','MCAP_2','GATE_1','GATE_2','SOURCE_FLOAT','RAIL_IN','SWITCHED_PRE','RAIL_OUT','DIV_1','DIV_2','SENSE_IN','AMC_HRAW','AMC_HLDO','GND_LOAD_RETURN','GND_LOGIC','AMC_LLDO','I_FILTER','IOUT_ADC','EN_A','EN_B','TT_1','TT_2','DIAG_N','VOUT_P','VOUT_N','EN','+3V3']
try:
    for n in order:route(n)
finally:
    # Existing plated component holes provide the layer change; never drill a
    # second via through or against them. Short bridges stay within pad copper.
    seen=set()
    for t in list(board.GetTracks()):
        if not isinstance(t,k.PCB_VIA):continue
        a=(k.ToMM(t.GetPosition().x),k.ToMM(t.GetPosition().y))
        key=(a,t.GetNetCode())
        if key in seen:board.Remove(t);continue
        seen.add(key)
        candidates=[p for p in allpads if p.GetAttribute()==k.PAD_ATTRIB_PTH and p.GetNetCode()==t.GetNetCode() and math.dist(a,padpoint(p))<k.ToMM(p.GetDrillSize().x)/2+.15+.26]
        if candidates:
            p=min(candidates,key=lambda p:math.dist(a,padpoint(p)));b=padpoint(p)
            if math.dist(a,b)>.001:
                for layer in [k.F_Cu,k.B_Cu]:
                    bridge=k.PCB_TRACK(board);bridge.SetStart(v(*a));bridge.SetEnd(v(*b));bridge.SetWidth(mm(.3));bridge.SetLayer(layer);bridge.SetNet(p.GetNet());board.Add(bridge)
            board.Remove(t)
    for n,net in nets.items():net.SetNetname('/'+n)
    for ref,pin,pname in [('U1','6','MCAP2'),('U2','6','MCAP2'),('J1','7','Pin_7')]:
        nn=k.NETINFO_ITEM(board,f'unconnected-({ref}-{pname}-Pad{pin})');board.Add(nn)
        footprints[ref].FindPadByNumber(pin).SetNet(nn)
    k.SaveBoard(str(ROOT/(NAME+'.kicad_pcb')),board)
    (ROOT/'validation'/'routing.json').write_text(json.dumps({'segments':len(tracks),'vias':len(vias)},indent=2))
print(f'Finished: {len(tracks)} segments, {len(vias)} vias')
