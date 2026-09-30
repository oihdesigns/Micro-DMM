"""PCB assemblies from STEP: actual solids, assembly names, and planar face anchors."""
import hashlib
from collections import defaultdict
from functools import lru_cache
from pathlib import Path
import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf
from shapely.geometry import Polygon
from geometry import frame, mesh_data, new_project
from references import asset_path, store_asset


def bounds(shape):
    b=shape.BoundingBox()
    return np.array([[b.xmin,b.ymin,b.zmin],[b.xmax,b.ymax,b.zmax]])


def transform(shape,rotation,offset):
    trsf=gp_Trsf();trsf.SetValues(*np.column_stack([rotation,offset]).ravel().tolist())
    return shape.moved(cq.Location(trsf))


@lru_cache(maxsize=4)
def source_bodies(asset):
    path=asset_path(asset)
    if path.suffix not in ['.step','.stp']:raise ValueError('A PCB assembly must be STEP or STP.')
    records=[]
    try:
        assembly=cq.Assembly.importStep(str(path))
    except ValueError as exc:
        if 'does not contain an assembly' not in str(exc):raise
        assembly=None
    if assembly is None:
        leaves=[(s,f'Body {i+1}',cq.Location(),None) for i,s in enumerate(cq.importers.importStep(str(path)).vals())]
    else:leaves=list(assembly)
    for shape,name,location,color in leaves:
        parts=name.split('/')[1:] if assembly else [name]
        group=' / '.join(parts[:2]) or name
        # Keep instance transforms and the distinct solids inside each assembly leaf.
        solids=shape.moved(location).Solids()
        for j,solid in enumerate(solids):
            if solid.Volume()<1e-8:continue
            rgb=color.toTuple()[:3] if color else None
            records.append({'shape':solid,'name':' / '.join(parts) or name,'group':group,
                            'color':'#'+''.join(f'{round(max(0,min(1,x))*255):02x}' for x in rgb) if rgb else None})
    if not records:raise ValueError('The STEP file contains no solid bodies.')
    if len(records)>1500:raise ValueError('The PCB assembly exceeds 1,500 solids. Export a simplified assembly.')
    return records


def board_basis(shape,flip=False):
    faces=[f for f in shape.Faces() if f.geomType()=='PLANE']
    if not faces:raise ValueError('Choose a flat PCB substrate body with planar top and bottom faces.')
    face=max(faces,key=lambda f:f.Area());n=np.array(face.normalAt().toTuple())
    if n[np.argmax(abs(n))]<0:n=-n
    if flip:n=-n
    _,basis=frame(normal=n)
    return basis.T


@lru_cache(maxsize=4)
def candidates(asset):
    result=[]
    for i,rec in enumerate(source_bodies(asset)):
        try:
            r=board_basis(rec['shape']);b=bounds(transform(rec['shape'],r,np.zeros(3)));d=b[1]-b[0]
            if .1<=d[2]<=20 and min(d[:2])>=5 and max(d[:2])/d[2]>=3:
                face_area=max(f.Area() for f in rec['shape'].Faces() if f.geomType()=='PLANE')
                result.append({'body':i,'name':rec['name'],'dimensions':np.round(d,4).tolist(),'area':round(face_area,3)})
        except ValueError:continue
    return sorted(result,key=lambda x:-x['area'])


@lru_cache(maxsize=6)
def normalized_bodies(asset,substrate,flip=False):
    records=source_bodies(asset)
    if not isinstance(substrate,int) or not 0<=substrate<len(records):raise ValueError('Invalid PCB substrate selection.')
    r=board_basis(records[substrate]['shape'],flip)
    b=bounds(transform(records[substrate]['shape'],r,np.zeros(3)))
    t=-b[0]
    return [{**rec,'shape':transform(rec['shape'],r,t)} for rec in records],r,t


def outline_of(shape):
    zfaces=[f for f in shape.Faces() if f.geomType()=='PLANE' and abs(f.normalAt().z)>.99999]
    face=max(zfaces,key=lambda f:f.Area())
    def polygon(wire):
        pts,_=wire.sample(.015)
        result=[[round(p.x,6),round(p.y,6)] for p in pts]
        if result and result[-1]==result[0]:result.pop()
        return result
    outline=polygon(face.outerWire())
    if len(outline)<3 or not Polygon(outline).is_valid:raise ValueError('Unable to extract a closed substrate outline. Choose another PCB body.')
    holes=[]
    for i,w in enumerate(face.innerWires()):
        edges=w.Edges()
        if len(edges)==1 and edges[0].geomType()=='CIRCLE':
            c=w.Center();d=2*edges[0].radius()
            holes.append({'id':f'step-hole-{i}','xy':[c.x,c.y],'diameter':d,'mounting':d>=1.8,'source':f'STEP hole {i+1}'})
    return outline,[polygon(w) for w in face.innerWires()],holes


def from_asset(asset,name,substrate=None,flip=False):
    options=candidates(asset)
    if not options:raise ValueError('No flat substrate was identified. Export the PCB as a separate solid in the STEP assembly.')
    substrate=options[0]['body'] if substrate is None else substrate
    records,r,t=normalized_bodies(asset,substrate,flip)
    substrate_shape=records[substrate]['shape'];b=bounds(substrate_shape);d=b[1]-b[0]
    if not .1<=d[2]<=20:raise ValueError('The selected substrate must be between 0.1 and 20 mm thick.')
    outline,cutouts,holes=outline_of(substrate_shape)
    groups=defaultdict(list)
    for i,rec in enumerate(records):
        if i!=substrate:groups[rec['group']].append(i)
    components=[]
    for group,indices in groups.items():
        allbounds=np.array([bounds(records[i]['shape']) for i in indices]);lo=allbounds[:,0].min(0);hi=allbounds[:,1].max(0)
        cid='step:'+asset[:16]+':'+hashlib.sha256(group.encode()).hexdigest()[:16]
        components.append({'id':cid,'ref':group.split(' / ')[-1],'value':group,'footprint':'STEP solid geometry','bodies':indices,
                           'xy':((lo[:2]+hi[:2])/2).tolist(),'center':[0,0],'rotation':0,'side':'bottom' if hi[2]<=0 else 'top',
                           'width':max(.05,float(hi[0]-lo[0])),'depth':max(.05,float(hi[1]-lo[1])),
                           'height':float(hi[2]-lo[2]),'zMin':float(lo[2]),'zMax':float(hi[2]),'estimated':False,'pads':[]})
    board={'name':Path(name).name,'source':'step','asset':asset,'substrate':substrate,'flip':flip,
           'substrateName':records[substrate]['name'],'bodyCount':len(records),'substrateOptions':options,
           'transform':{'rotation':r.tolist(),'offset':t.tolist()},'thickness':float(d[2]),'width':float(d[0]),'length':float(d[1]),
           'outline':outline,'cutouts':cutouts,'holes':holes,'components':components,
           'warnings':['The substrate is detected from planar solids. Verify its selection and thickness below.',
                       'Components retain the geometry present in STEP. Missing models in the source cannot be recovered.',
                       'Round openings ≥ 1.8 mm are offered as mounting holes; verify them before adding posts.']}
    p=new_project(board)
    if components:p['enclosure']['below']=max(3.,round(-min(c['zMin'] for c in components)+1.,2))
    return p


def import_step_board(data,name):
    return from_asset(store_asset(data,name),name)


def board_shapes(board):
    return normalized_bodies(board['asset'],board['substrate'],board.get('flip',False))[0]


@lru_cache(maxsize=4)
def board_meshes(asset,substrate,flip=False):
    records,_,_=normalized_bodies(asset,substrate,flip)
    # Meshes and face frames share normalized board coordinates. Original B-reps stay exact.
    return [{**mesh_data(rec['shape'],with_faces=True,tolerance=.08),'body':i,'color':rec['color']}
            for i,rec in enumerate(records)]


def render_board(board):
    owners={i:c for c in board['components'] for i in c['bodies']}
    meshes=[]
    for source in board_meshes(board['asset'],board['substrate'],board.get('flip',False)):
        i=source['body'];is_pcb=i==board['substrate'];c=owners.get(i)
        if not is_pcb and c is None:continue
        meshes.append({**source,'id':f'pcb:{i}','componentId':c['id'] if c else None,
                       'name':c['ref'] if c else 'PCB substrate','kind':'pcb' if is_pcb else 'component',
                       'boardAsset':board['asset'],'color':source['color'] or ('#327653' if is_pcb else '#66717d')})
    return meshes


def face_frame(board,anchor):
    if anchor.get('ref')!=board.get('asset'):raise ValueError('The attached STEP PCB was replaced. Pick a face on the new assembly.')
    # Resolve from the solid, not user-supplied face coordinates; changing the substrate keeps the face attached.
    shapes=board_shapes(board);body=anchor.get('body');fid=int(anchor.get('face',{}).get('id',-1))
    if not isinstance(body,int) or not 0<=body<len(shapes):raise ValueError('The attached STEP body is missing.')
    faces=shapes[body]['shape'].Faces()
    if not 0<=fid<len(faces) or faces[fid].geomType()!='PLANE':raise ValueError('Select a planar STEP PCB face.')
    face=faces[fid]
    return frame(face.Center().toTuple(),face.normalAt().toTuple())


def collision_shapes(board):
    records=board_shapes(board)
    yield 'PCB substrate',records[board['substrate']]['shape'],False
    for c in board['components']:
        yield c['ref'],cq.Compound.makeCompound([records[i]['shape'] for i in c['bodies']]),False
