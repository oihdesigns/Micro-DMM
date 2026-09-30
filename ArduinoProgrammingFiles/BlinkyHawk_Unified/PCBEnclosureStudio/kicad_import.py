"""KiCad 6+ mechanical import. All output coordinates are right-handed millimeters."""
import math
import re
import uuid
import numpy as np
from shapely.geometry import Polygon, LineString, Point, box
from shapely.ops import polygonize, unary_union

def parse_sexpr(text):
    tokens = re.findall(r'"(?:\\.|[^"\\])*"|[()]|[^\s()]+', text)
    stack=[]; roots=[]
    for token in tokens:
        if token=='(':
            node=[]; (stack[-1] if stack else roots).append(node); stack.append(node)
        elif token==')':
            if not stack: raise ValueError('Malformed KiCad file: unmatched parenthesis.')
            stack.pop()
        else:
            if not stack: raise ValueError('Malformed KiCad file.')
            stack[-1].append(token[1:-1].replace('\\"','"').replace('\\\\','\\') if token.startswith('"') else token)
    if stack or len(roots)!=1: raise ValueError('Malformed KiCad file.')
    return roots[0]

def children(node,tag): return [a for a in node if isinstance(a,list) and a and a[0]==tag]
def one(node,tag,default=None): return next(iter(children(node,tag)),default)
def values(node): return [float(v) for v in node[1:]]
def point(node,tag): return values(one(node,tag))[:2]

def arc_points(start,mid,end,tolerance=.015):
    a,b,c=np.asarray(start),np.asarray(mid),np.asarray(end)
    mat=2*np.array([b-a,c-a]); rhs=np.array([b@b-a@a,c@c-a@a])
    if abs(np.linalg.det(mat))<1e-9: return [start,end]
    center=np.linalg.solve(mat,rhs); radius=np.linalg.norm(a-center)
    t0,tm,t1=[math.atan2(p[1]-center[1],p[0]-center[0]) for p in [a,b,c]]
    sweep=(t1-t0)%(2*math.pi)
    if (tm-t0)%(2*math.pi)>sweep: sweep-=2*math.pi
    step=2*math.acos(max(-1,min(1,1-tolerance/max(radius,tolerance))))
    angles=np.linspace(t0,t0+sweep,max(3,int(abs(sweep)/max(step,.01))+1))
    pts=[(center+radius*np.array([math.cos(t),math.sin(t)])).tolist() for t in angles]
    pts[0]=start;pts[-1]=end;return pts

def graphic_points(node):
    kind=node[0].split('_',1)[-1]
    if kind=='line': return [point(node,'start'),point(node,'end')],False
    if kind=='arc':
        if one(node,'mid'): return arc_points(point(node,'start'),point(node,'mid'),point(node,'end')),False
        raise ValueError('Legacy KiCad arcs require resaving the board in KiCad 6 or later.')
    if kind=='circle':
        center=point(node,'center');r=math.dist(center,point(node,'end'))
        return list(Point(center).buffer(r,quad_segs=32).exterior.coords),True
    if kind=='rect':
        a=point(node,'start');b=point(node,'end');lo=np.minimum(a,b);hi=np.maximum(a,b)
        rad=float(one(node,'radius',['radius',0])[1]);rad=min(rad,*((hi-lo)/2))
        poly=box(*lo,*hi) if rad<=0 else box(*(lo+rad),*(hi-rad)).buffer(rad,quad_segs=24)
        return list(poly.exterior.coords),True
    if kind=='poly':return [values(xy)[:2] for xy in children(one(node,'pts',[]),'xy')],True
    if kind=='curve':raise ValueError('Bezier Edge.Cuts are not supported yet. Convert the board outline to lines/arcs first.')
    return [],False

def transform_local(pts,at):
    theta=math.radians(at[2] if len(at)>2 else 0);c,s=math.cos(theta),math.sin(theta)
    return [[at[0]+x*c+y*s,at[1]-x*s+y*c] for x,y in pts]

def guess_height(name):
    name=name.lower()
    for key,h in [('xiao',4.8),('usb',3.5),('pinsocket',8.5),('pinheader',6),('buzzer',3.1),('terminalblock',10),('battery',5),('led',2.1),('sot-23',1.2),('soic',1.8),('tqfp',1.6),('mountinghole',0),('testpoint',.1),('solderwire',.5),('0402',.55),('0603',.8),('0805',1.0),('dsbga',.6),('qfn',1.0)]:
        if key in name:return h
    return 2.0

def import_board(text,filename):
    root=parse_sexpr(text)
    if root[0]!='kicad_pcb':raise ValueError('Choose a .kicad_pcb board file, not a schematic or footprint.')
    edges=[n for n in root if isinstance(n,list) and n and n[0].startswith('gr_') and one(n,'layer')==['layer','Edge.Cuts']]
    footprints=children(root,'footprint')+children(root,'module')
    segments=[];closed=[];edge_holes=[]
    def append_edge(node,at=None):
        pts,isclosed=graphic_points(node)
        if not pts:return
        if at:pts=transform_local(pts,at)
        pts=[[round(float(x),4),round(float(y),4)] for x,y in pts]
        if isclosed:
            if pts[-1]!=pts[0]:pts.append(pts[0])
            poly=Polygon(pts)
            if not poly.is_valid or poly.area<=1e-6:raise ValueError('The PCB outline includes a self-intersecting or degenerate loop.')
            closed.append(poly)
            if node[0].endswith('circle'):
                center=point(node,'center');center=transform_local([center],at)[0] if at else center
                edge_holes.append({'x':center[0],'y':center[1],'diameter':2*math.dist(point(node,'center'),point(node,'end')),'source':'Edge.Cuts'})
        else:segments.append(LineString(pts))
    for e in edges:append_edge(e)
    for f in footprints:
        at=values(one(f,'at',['at',0,0]))
        for e in f:
            if isinstance(e,list) and e and e[0].startswith('fp_') and one(e,'layer')==['layer','Edge.Cuts']:append_edge(e,at)
    if segments:
        merged=unary_union(segments);polys=list(polygonize(merged))
        covered=unary_union([p.boundary for p in polys]) if polys else LineString()
        dangling=merged.difference(covered.buffer(.002)).length
        if dangling>.02:raise ValueError('The Edge.Cuts outline has an open contour. Close the outline in KiCad and retry.')
        closed.extend(Polygon(p.exterior) for p in polys)
    if not closed:raise ValueError('No closed board outline was found on Edge.Cuts.')
    outer=max(closed,key=lambda p:p.area)
    if any(not outer.buffer(.02).covers(p) for p in closed):raise ValueError('Multiple separate boards were found. Import one board at a time.')
    holes=[]
    for p in sorted(closed,key=lambda p:-p.area):
        if p.equals(outer):continue
        if not any(q.buffer(.001).covers(p) for q in holes):holes.append(p)
    x0,y0,x1,y1=outer.bounds
    conv=lambda p:[round(float(p[0]-x0),5),round(float(y1-p[1]),5)]
    thickness=float(one(one(root,'general',[]),'thickness',['thickness',1.6])[1])
    board={'name':filename,'thickness':thickness,'outline':[conv(p) for p in list(outer.exterior.coords)[:-1]],'cutouts':[[conv(p) for p in list(h.exterior.coords)[:-1]] for h in holes], 'holes':[],'components':[], 'sourceOrigin':[x0,y1], 'width':x1-x0,'length':y1-y0,'warnings':['KiCad component bodies are editable courtyard/fabrication envelopes; component heights are estimates. Add STEP references for exact surfaces.','Outline arcs are sampled to approximately 0.015 mm chord tolerance.']}
    for i,h in enumerate(edge_holes):
        if any(p.buffer(.01).covers(Point(h['x'],h['y'])) for p in holes):board['holes'].append({'id':f'edge-hole-{i}','xy':conv([h['x'],h['y']]),'diameter':h['diameter'],'mounting':True,'source':'Edge.Cuts'})
    for i,f in enumerate(footprints):
        props={p[1]:p[2] for p in children(f,'property')}
        for p in children(f,'fp_text'):
            if p[1]=='reference':props.setdefault('Reference',p[2])
            if p[1]=='value':props.setdefault('Value',p[2])
        ref=props.get('Reference',f'F{i+1}');name=f[1];at=values(one(f,'at',['at',0,0]));angle=at[2] if len(at)>2 else 0
        side='bottom' if one(f,'layer',['layer','F.Cu'])[1].startswith('B.') else 'top'
        cid=one(f,'uuid',one(f,'tstamp',['uuid',ref]))[1]
        outlines=[]
        for layer in [('B.' if side=='bottom' else 'F.')+'CrtYd',('B.' if side=='bottom' else 'F.')+'Fab']:
            for g in f:
                if isinstance(g,list) and g and g[0] in ['fp_rect','fp_line','fp_circle','fp_arc','fp_poly'] and one(g,'layer')==['layer',layer]:outlines.extend(graphic_points(g)[0])
            if outlines:break
        graphics_found=bool(outlines)
        pads=[]
        for pad_index,p in enumerate(children(f,'pad')):
            pa=values(one(p,'at',['at',0,0]));size=values(one(p,'size',['size',1,1]));local=pa[:2]
            pos=conv(transform_local([local],at)[0]);drill=one(p,'drill')
            nums=[float(v) for v in drill[1:] if isinstance(v,str) and re.fullmatch(r'[0-9.]+',v)] if drill else []
            pad_angle=pa[2] if len(pa)>2 else 0
            pd={'number':p[1],'xy':pos,'localXY':[local[0],-local[1]],'size':size[:2],'drill':min(nums) if nums else 0,'kind':p[2]}
            pads.append(pd)
            if nums:
                # KiCad stores pad angles in board coordinates. Drill X/Y are full extents;
                # the optional drill offset moves copper relative to the hole, not the hole.
                hole_angle=pad_angle+(90 if len(nums)>1 and nums[1]>nums[0] else 0)
                pid=one(p,'uuid',one(p,'tstamp',['uuid',f'{p[1]}:{pad_index}']))[1]
                board['holes'].append({'id':f'{cid}:pad:{pid}','xy':pos,'diameter':min(nums),'slotLength':max(nums),'angle':hole_angle,'mounting':'mountinghole' in name.lower() or (p[2]=='np_thru_hole' and min(nums)>=1.8),'source':ref+' pad '+p[1]})
            if not graphics_found:
                offset=values(one(drill or [],'offset',['offset',0,0]))
                corners=[[x*size[0]/2+offset[0],y*size[1]/2+offset[1]] for x,y in [(-1,-1),(-1,1),(1,-1),(1,1)]]
                outlines.extend(transform_local(corners,[*local,pad_angle-angle]))
        if not outlines:outlines=[[-1,-1],[1,1]]
        v=np.asarray(outlines);lo=v.min(0);hi=v.max(0);center=(lo+hi)/2
        # Local Y is inverted when changing from KiCad's screen coordinates to CAD.
        board['components'].append({'id':str(cid),'ref':ref,'value':props.get('Value',''),'footprint':name,'xy':conv(at[:2]),'rotation':angle,'side':side,'center':[float(center[0]),float(-center[1])],'width':max(.1,float(hi[0]-lo[0])),'depth':max(.1,float(hi[1]-lo[1])),'height':guess_height(name),'estimated':True,'pads':pads})
    if not .1<=thickness<=20:raise ValueError('Board thickness must be between 0.1 and 20 mm.')
    return board
