"""Build and route the PCB using KiCad's bundled Python / pcbnew.

Creates a two-layer PCB, retaining schematic UUIDs for update/parity checks.
"""
raise SystemExit('Full-board regeneration is disabled: the edited PCB is authoritative. Use KiCad to update the board, or scripts/export.ps1 to check/export it.')
from pathlib import Path
import json, math, heapq, itertools, os
import numpy as np
import pcbnew as k
from design import ROOT, NAME, SHEET, uid, datasheets

os.environ['KICAD_CONFIG_HOME']=str(ROOT/'tmp/kicad-config')
parts=json.loads((ROOT/'scripts/components.json').read_text())
b=k.BOARD()
mm=k.FromMM
def pos(x,y):return k.VECTOR2I(mm(float(x)),mm(float(y)))
netnames=sorted({v for p in parts for v in p['nets'].values() if v is not None})
nets={}
for name in netnames:
    n=k.NETINFO_ITEM(b,name);b.Add(n);nets[name]=n
fps={};pads={}
for p in parts:
    f=k.FootprintLoad(str(ROOT/'AD8237DevBoard.pretty'),p['fp'].split(':')[1])
    f.SetReference(p['ref']);f.SetValue(p['value'])
    f.SetFPID(k.LIB_ID('AD8237DevBoard',p['fp'].split(':')[1]))
    path=k.KIID_PATH();path.push_back(k.KIID(SHEET));path.push_back(k.KIID(p['uuid']));f.SetPath(path)
    f.SetSheetfile(NAME+'.kicad_sch');f.SetSheetname('')
    f.SetField('Assembly',p['notes'])
    f.GetField('Assembly').SetVisible(False)
    if p['kind'] in datasheets:f.SetField('Datasheet',datasheets[p['kind']])
    if p['kind'] in datasheets:f.GetField('Datasheet').SetVisible(False)
    f.SetAttributes(f.GetAttributes() & ~k.FP_EXCLUDE_FROM_BOM)
    if p['dnp']:f.SetAttributes(f.GetAttributes() | k.FP_DNP)
    x,y,a=p['pcb'];f.SetPosition(pos(x,y));f.SetOrientationDegrees(a)
    f.Reference().SetTextSize(pos(.8,.8));f.Reference().SetTextThickness(mm(.12))
    f.Reference().SetTextAngle(k.EDA_ANGLE(0,k.DEGREES_T));f.Reference().SetPosition(pos(x,y-1.65))
    f.Value().SetVisible(False)
    if p['ref'].startswith(('H','TP','JP','J')):f.Reference().SetVisible(False)
    if p['ref'] in ['U1','U2','U3','U4']:f.Reference().SetPosition(pos(x,y-2.5))
    if p['ref']=='C3':f.Reference().SetPosition(pos(x-2,y))
    for pad in f.Pads():
        number=pad.GetNumber()
        if p['nets'].get(number) is not None:pad.SetNet(nets[p['nets'][number]])
        pads[p['ref'],number]=pad
    b.Add(f);fps[p['ref']]=f

def line(a,c,layer=k.Edge_Cuts,width=.05):
    s=k.PCB_SHAPE();s.SetShape(k.SHAPE_T_SEGMENT);s.SetStart(pos(*a));s.SetEnd(pos(*c));s.SetLayer(layer);s.SetWidth(mm(width));b.Add(s)
for a,c in [((100,100),(160,100)),((160,100),(160,150)),((160,150),(100,150)),((100,150),(100,100))]:line(a,c)
def txt(t,x,y,size=1,layer=k.F_SilkS):
    v=k.PCB_TEXT(b);v.SetText(t);v.SetPosition(pos(x,y));v.SetTextSize(pos(size,size));v.SetTextThickness(mm(.15 if size>=1 else .12));v.SetLayer(layer)
    if layer==k.B_SilkS:v.SetMirrored(True)
    b.Add(v)
txt('AD8237  I2C',128,102.5,1.3);txt('Rev B',142,102.5,.8)
txt('POWER',112.5,103,.8);txt('EXT',109,108.3,.8);txt('VIO',115.5,108.3,.8)
txt('+',102.7,109.5,1);txt('-',102.7,112.04,1)
for x,t in [(143,'G'),(145.54,'V'),(148.08,'DA'),(150.62,'CL')]:txt(t,x,108.3,.8)
txt('I2C',150.5,103.5,.8)
for y,t in [(118,'IN+'),(120.54,'IN-'),(123.08,'GND')]:txt(t,102,y,.8)
txt('L',122,105.8,.8);txt('H',127.08,105.8,.8);txt('BW',125,110.4,.8)
for y,t in [(129,'OUT'),(131.54,'GND'),(134.08,'REF')]:txt(t,158.7,y,.8)
txt('REF SELECT',141,148,.85)
for y,t in [(140,'GND'),(142.54,'MID'),(145.08,'EXT')]:txt(t,144.5,y,.8)
txt('PULL',155.5,111,.8)
for t,x,y in [('VS',124,115),('GND',107,139),('+IN',120,129),('-IN',120,137),('RAW',134,107),('FB',135,130),('REF',129,143),('MID',124,143)]:txt(t,x,y,.8)
txt('AD8237 | I2C 0x41 | Rev B',130,109,1.1,k.B_SilkS)
txt('GAIN 0/1/2/3: 1/10.09/101/1001\nPower-up gain: 1, use LOW BW\nJP1: EXT / VIO analog supply\nJP3: GND / MID / EXT reference\nJ4: GND / VIO / SDA / SCL\nVIO must match controller voltage',130,120,1,k.B_SilkS)
txt('C3-C6, R9-R10: DNP\nC7: fitted 470pF\nPrototype 2026-09',130,138,1,k.B_SilkS)

def track(net,a,c,layer=k.F_Cu,width=.2):
    if a==c:return
    t=k.PCB_TRACK(b);t.SetStart(pos(*a));t.SetEnd(pos(*c));t.SetWidth(mm(width));t.SetLayer(layer);t.SetNet(nets[net]);b.Add(t)
def via(net,x,y):
    v=k.PCB_VIA(b);v.SetPosition(pos(x,y));v.SetWidth(mm(.6));v.SetDrill(mm(.3));v.SetLayerPair(k.F_Cu,k.B_Cu);v.SetNet(nets[net]);b.Add(v)
def padxy(ref,num):
    p=pads[ref,str(num)].GetPosition();return k.ToMM(p.x),k.ToMM(p.y)

# Short, deliberate analog connections at the IC.
def manual(net,points,width=.2):
    for a,c in zip(points,points[1:]):track(net,a,c,width=width)
manual('IN_P_F',[padxy('R1',2),(117.25,118.8),(119.125,120.675),padxy('U1',2)])
manual('IN_N_F',[padxy('R2',2),(117.25,123.2),(119.125,121.325),padxy('U1',3)])
manual('VS',[padxy('U1',5),(125.95,121.975),(125.95,123.5),(124.05,123.5),padxy('C2',1)],.25)
manual('GND',[padxy('U1',4),(119.8875,123.5)])
via('GND',119.8875,123.5)
pads['U1','4'].SetLocalZoneConnection(k.ZONE_CONNECTION_FULL)
manual('GND',[padxy('C2',2),(126.8,125)],.3);via('GND',126.8,125)
# Fan out the mux's 0.5 mm pitch pins before routing the larger networks.
# Expanding to 1 mm pitch prevents later routes from boxing in adjacent pins.
anchors={}
for num in range(1,11):
    pad=pads['U2',str(num)];x,y=padxy('U2',num)
    idx=(num-1) if num<=5 else (10-num)
    yy=117.5+idx;side=-1 if num<=5 else 1
    xx=129 if side<0 else 137;neck=130 if side<0 else 136
    midx=neck+side*abs(yy-y)
    manual(pad.GetNetname(),[(x,y),(neck,y),(midx,yy),(xx,yy)])
    anchors['U2',str(num)]=(xx,yy)
via('GND',129,119.5)
# Grid router. Explicit pad/track/via obstacles include clearance plus a margin.
# Ground is connected through the plane; through-hole pads provide layer access.
STEP=.1;NX=601;NY=501;CLEAR=.21;W=.2
def grid(x,y):return int(round((x-100)/STEP)),int(round((y-100)/STEP))
def world(x,y):return round(100+x*STEP,4),round(100+y*STEP,4)
def layers(pad):return [0,1] if pad.GetAttribute()==k.PAD_ATTRIB_PTH else [0]
def circle(mask,x,y,r,ls):
    ix,iy=grid(x,y);ir=math.ceil(r/STEP)+1
    xa=max(0,ix-ir);xb=min(NX,ix+ir+1);ya=max(0,iy-ir);yb=min(NY,iy+ir+1)
    xx=100+np.arange(xa,xb)*STEP;yy=100+np.arange(ya,yb)*STEP
    sel=(xx[:,None]-x)**2+(yy[None,:]-y)**2 <=r*r
    for l in ls:mask[l,xa:xb,ya:yb] |= sel
def rect(mask,x,y,sx,sy,margin,ls):
    xa=max(0,math.ceil((x-sx/2-margin-100)/STEP));xb=min(NX,math.floor((x+sx/2+margin-100)/STEP)+1)
    ya=max(0,math.ceil((y-sy/2-margin-100)/STEP));yb=min(NY,math.floor((y+sy/2+margin-100)/STEP)+1)
    for l in ls:mask[l,xa:xb,ya:yb]=True
def obstacles(net,radius=.1):
    mask=np.zeros((2,NX,NY),dtype=bool)
    edge=math.ceil((.3+radius)/STEP)
    mask[:,:edge,:]=True;mask[:,-edge:,:]=True;mask[:,:,:edge]=True;mask[:,:,-edge:]=True
    for f in b.GetFootprints():
        for p in f.Pads():
            if p.GetNetname()==net:continue
            xy=p.GetPosition();sz=p.GetSize();x,y=k.ToMM(xy.x),k.ToMM(xy.y);sx,sy=k.ToMM(sz.x),k.ToMM(sz.y)
            a=p.GetOrientationDegrees()%180
            if abs(a-90)<1:sx,sy=sy,sx
            ls=layers(p)
            if p.GetAttribute()==k.PAD_ATTRIB_NPTH:ls=[0,1]
            if p.GetShape()==k.PAD_SHAPE_CIRCLE:circle(mask,x,y,sx/2+CLEAR+radius,ls)
            else:rect(mask,x,y,sx,sy,CLEAR+radius,ls)
    for t in b.GetTracks():
        if t.GetNetname()==net:continue
        a=t.GetStart();c=t.GetEnd();x,y=k.ToMM(a.x),k.ToMM(a.y);xx,yy=k.ToMM(c.x),k.ToMM(c.y)
        ls=[0,1] if isinstance(t,k.PCB_VIA) else [0 if t.GetLayer()==k.F_Cu else 1]
        rr=k.ToMM(t.GetWidth(k.F_Cu) if isinstance(t,k.PCB_VIA) else t.GetWidth())/2+CLEAR+radius
        dist=math.hypot(xx-x,yy-y);steps=max(1,math.ceil(dist/(STEP*.65)))
        for j in range(steps+1):circle(mask,x+(xx-x)*j/steps,y+(yy-y)*j/steps,rr,ls)
    return mask

def findpath(net,start,goals):
    blocked=obstacles(net);vblock=obstacles(net,.3)
    goalset=set(goals)
    def heuristic(s):
        x,y,l=s;return min(max(abs(x-a),abs(y-c))+.41421356*min(abs(x-a),abs(y-c))+(0 if l==ll else 10) for a,c,ll in goals)
    pq=[];dist={};prev={};counter=itertools.count()
    for s in start:dist[s]=0;heapq.heappush(pq,(heuristic(s),next(counter),s))
    moves=[(1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,1.41421356),(-1,1,1.41421356),(1,-1,1.41421356),(-1,-1,1.41421356)]
    count=0
    while pq:
        _,_,s=heapq.heappop(pq);x,y,l=s;cost=dist[s];count+=1
        if s in goalset:
            result=[s]
            while s in prev:s=prev[s];result.append(s)
            return result[::-1]
        if count>450000:break
        for dx,dy,dc in moves:
            xx,yy=x+dx,y+dy
            if not(0<=xx<NX and 0<=yy<NY) or blocked[l,xx,yy]:continue
            if dx and dy and (blocked[l,x+dx,y] or blocked[l,x,y+dy]):continue
            n=(xx,yy,l);nc=cost+dc*(1.025 if l else 1)
            if nc<dist.get(n,1e30):dist[n]=nc;prev[n]=s;heapq.heappush(pq,(nc+heuristic(n),next(counter),n))
        if not vblock[0,x,y] and not vblock[1,x,y]:
            n=(x,y,1-l);nc=cost+24
            if nc<dist.get(n,1e30):dist[n]=nc;prev[n]=s;heapq.heappush(pq,(nc+heuristic(n),next(counter),n))
    raise RuntimeError(f'Cannot route {net}: {start} -> {goals}; explored {count}')

def pathdraw(net,path):
    if len(path)<2:return
    # Keep only corners and layer transitions.
    simple=[path[0]]
    for j in range(1,len(path)-1):
        d1=tuple(path[j][v]-path[j-1][v] for v in range(3));d2=tuple(path[j+1][v]-path[j][v] for v in range(3))
        if d1!=d2:simple.append(path[j])
    simple.append(path[-1])
    for (x,y,l),(xx,yy,ll) in zip(simple,simple[1:]):
        if l!=ll:via(net,*world(x,y))
        else:track(net,world(x,y),world(xx,yy),k.F_Cu if l==0 else k.B_Cu)

# Each net is routed as a nearest-neighbour tree; all physical pads, including
# optional components and probe pads, are in the routing graph.
order=['FB','VS','OUT_RAW','UNITY','GAIN10','GAIN101','GAIN1001','REF_PIN','MID_DIV','VMID','IN_P_F','IN_N_F','IN_P','IN_N','SEL0','SEL1','SDA','SCL','VIO','PULL_V','VS_EXT','BW','REF','REF_EXT','OUT']
for net in order:
    ps=[p for p in pads.values() if p.GetNetname()==net]
    connected=[ps.pop(0)]
    while ps:
        pairs=[(math.hypot(k.ToMM(p.GetPosition().x-q.GetPosition().x),k.ToMM(p.GetPosition().y-q.GetPosition().y)),i,q) for i,p in enumerate(ps) for q in connected]
        _,i,q=min(pairs,key=lambda z:z[0]);p=ps.pop(i)
        a=p.GetPosition();c=q.GetPosition()
        pxy=anchors.get((p.GetParentFootprint().GetReference(),p.GetNumber()),(k.ToMM(a.x),k.ToMM(a.y)))
        qxy=anchors.get((q.GetParentFootprint().GetReference(),q.GetNumber()),(k.ToMM(c.x),k.ToMM(c.y)))
        aa=grid(*pxy);cc=grid(*qxy)
        start=[(*aa,l) for l in layers(p)];goals=[(*cc,l) for l in layers(q)]
        path=findpath(net,start,goals);pathdraw(net,path)
        # Join snapped grid coordinates to the true pad centers, inside each pad.
        track(net,pxy,world(*aa),k.F_Cu if path[0][2]==0 else k.B_Cu)
        track(net,qxy,world(*cc),k.F_Cu if path[-1][2]==0 else k.B_Cu)
        connected.append(p)
    print('Routed',net,flush=True)

# Local ground vias at SMD ground pads, with top copper to connect to the plane.
for ref,num in [(ref,num) for (ref,num),p in pads.items() if p.GetNetname()=='GND' and p.GetAttribute()==k.PAD_ATTRIB_SMD and (ref,num) not in [('U1','4'),('U2','3'),('C2','2')]]:
    x,y=padxy(ref,num)
    # Find a nearby clear ground via location and route the short connection.
    vblock=obstacles('GND',.3);found=False
    for radius in [.8,1,1.3,1.6,2,2.5]:
        for dx,dy in [(1,0),(0,1),(0,-1),(-1,0),(1,1),(-1,1),(1,-1),(-1,-1)]:
            gx,gy=grid(x+dx*radius,y+dy*radius)
            if not(0<gx<NX and 0<gy<NY) or vblock[0,gx,gy] or vblock[1,gx,gy]:continue
            path=findpath('GND',[(*grid(x,y),0)],[(gx,gy,0)]);pathdraw('GND',path);via('GND',*world(gx,gy));found=True;break
        if found:break
    if not found:raise RuntimeError('No ground via '+ref)

for x,y in [(107,104),(106,144),(154,105),(155,143),(105,127),(153,137),(126,147)]:
    bx,by=grid(x,y);vb=obstacles('GND',.3)
    if not vb[0,bx,by] and not vb[1,bx,by]:via('GND',x,y)

for layer in [k.F_Cu,k.B_Cu]:
    z=k.ZONE(b);z.SetLayer(layer);z.SetNet(nets['GND']);z.SetLocalClearance(mm(.2));z.SetThermalReliefGap(mm(.25));z.SetThermalReliefSpokeWidth(mm(.3));z.SetPadConnection(k.ZONE_CONNECTION_THERMAL);z.SetMinThickness(mm(.2));z.SetIslandRemovalMode(k.ISLAND_REMOVAL_MODE_ALWAYS)
    poly=z.Outline();poly.NewOutline()
    for x,y in [(100.5,100.5),(159.5,100.5),(159.5,149.5),(100.5,149.5)]:poly.Append(mm(x),mm(y))
    b.Add(z)
for key in [('U2','3'),('U3','4'),('U4','2')]:pads[key].SetLocalZoneConnection(k.ZONE_CONNECTION_FULL)
b.BuildConnectivity()
for name,n in nets.items():n.SetNetname('/'+name)
k.SaveBoard(str(ROOT/(NAME+'.kicad_pcb')),b)
print('Saved board; run kicad-cli pcb drc --refill-zones --save-board for final filling')

