"""Immutable imported references. STEP faces keep their local attachment frames."""
import hashlib,uuid,re,copy
from pathlib import Path
from functools import lru_cache
import cadquery as cq
import numpy as np
import trimesh
from geometry import mesh_data,euler,reference_transform,metrics,validate_project,numeric

ROOT=Path(__file__).resolve().parent
ASSETS=ROOT/'data'/'assets'
ASSETS.mkdir(parents=True,exist_ok=True)
EXTENSIONS={'.step','.stp','.stl','.3mf'}

def store_asset(data,name):
    suffix=Path(name).suffix.lower()
    if suffix not in EXTENSIONS:raise ValueError('Reference formats: STEP, STP, STL, and 3MF.')
    if not 0<len(data)<=64*1024*1024:raise ValueError('Reference files must be between 1 byte and 64 MB.')
    asset=hashlib.sha256(data).hexdigest()+suffix
    path=ASSETS/asset
    if not path.exists():path.write_bytes(data)
    return asset

def asset_path(asset):
    if not re.fullmatch(r'[0-9a-f]{64}\.(step|stp|stl|3mf)',asset):raise ValueError('Invalid reference asset identifier.')
    path=ASSETS/asset
    if not path.is_file():raise ValueError('Reference data is missing. Reimport the model or open the full .pcbshell project.')
    return path

@lru_cache(maxsize=12)
def original_shapes(asset):
    path=asset_path(asset)
    roots=cq.importers.importStep(str(path)).vals()
    solids=tuple(s for root in roots for s in root.Solids())
    if not solids:raise ValueError('This STEP reference has no closed solid bodies.')
    if len(solids)>1500:raise ValueError('This reference has more than 1,500 bodies; export a simplified mechanical model.')
    return solids

@lru_cache(maxsize=12)
def original_meshes(asset):
    path=asset_path(asset);meshes=[]
    if path.suffix in ['.step','.stp']:
        solids=original_shapes(asset)
        for i,s in enumerate(solids):meshes.append({'body':i,**mesh_data(s,with_faces=True,tolerance=.12)})
    else:
        scene=trimesh.load_scene(path)
        # STL is unitless and interpreted as mm. 3MF declares its units.
        if scene.units and scene.units!='millimeter':scene.convert_units('mm')
        for i,m in enumerate(scene.dump()):
            if len(m.faces)>400000:raise ValueError('A mesh reference exceeds 400,000 triangles. Simplify it before importing.')
            meshes.append({'body':i,'vertices':np.round(m.vertices,5).tolist(),'triangles':m.faces.tolist(),'faces':[]})
    if not meshes:raise ValueError('No reference geometry was found.')
    return meshes

def import_reference(data,name):
    asset=store_asset(data,name);meshes=original_meshes(asset)
    coords=np.vstack([m['vertices'] for m in meshes])
    return {'id':str(uuid.uuid4()),'name':Path(name).name,'asset':asset,'format':Path(name).suffix[1:].upper(),'offset':[0,0,0],'rotation':[0,0,0],'visible':True,'fitCheck':True,'spaceBody':'all','spaceMargin':1.,'bounds':[coords.min(0).tolist(),coords.max(0).tolist()],'bodies':[{'id':m['body'],'bounds':[np.min(m['vertices'],axis=0).tolist(),np.max(m['vertices'],axis=0).tolist()]} for m in meshes],'planarFaces':sum(1 for m in meshes for f in m['faces'] if f['type']=='PLANE')}

def render_reference(ref,pose=None):
    result=[];offset,r=pose if pose else (np.asarray(ref.get('offset',[0,0,0])),euler(ref.get('rotation',[0,0,0])))
    for source in original_meshes(ref['asset']):
        m=dict(source);m['vertices']=np.round(np.asarray(source['vertices'])@r.T+offset,5).tolist()
        m.update(id=f"{ref['id']}:{source['body']}",refId=ref['id'],name=ref['name'],kind='reference',visible=ref.get('visible',True))
        # Face descriptors deliberately remain in the asset's local coordinate frame.
        result.append(m)
    return result

def placed_shapes(project,ref,met):
    from OCP.gp import gp_Trsf
    offset,r=reference_transform(project,ref,met)
    if asset_path(ref['asset']).suffix in ['.step','.stp']:
        t=gp_Trsf();t.SetValues(*np.column_stack([r,offset]).ravel().tolist())
        return cq.Compound.makeCompound([s.moved(cq.Location(t)) for s in original_shapes(ref['asset'])]),False
    coords=np.vstack([np.asarray(m['vertices'])@r.T+offset for m in original_meshes(ref['asset'])])
    lo=coords.min(0);hi=coords.max(0);dims=np.maximum(hi-lo,.001)
    return cq.Workplane('XY').box(*dims).translate(tuple((lo+hi)/2)).val(),True

def reserve_space(project,ref_id,body='all',margin=1):
    """One-time sizing operation for a freely positioned hardware body."""
    validate_project(project);margin=numeric(margin,'Hardware space allowance',0,30)
    ref=next((r for r in project.get('references',[]) if r['id']==ref_id),None)
    if not ref:raise ValueError('The hardware model is missing.')
    if ref.get('mount'):raise ValueError('Reserve space in World coordinates first, then mount the model to a wall or plane.')
    offset,r=reference_transform(project,ref,metrics(project))
    meshes=[m for m in original_meshes(ref['asset']) if body=='all' or str(m['body'])==str(body)]
    if not meshes:raise ValueError('The selected hardware body is missing.')
    coords=np.vstack([np.asarray(m['vertices'])@r.T+offset for m in meshes]);lo=coords.min(0);hi=coords.max(0)
    c=project['enclosure'];b=np.asarray(project['board']['outline']);blo=b.min(0);bhi=b.max(0)
    pad=margin+max(0,c['corner']-c['wall'])
    for key,amount in [('extraLeft',blo[0]-c['clearance']-lo[0]+pad),('extraRight',hi[0]+pad-bhi[0]-c['clearance']),('extraFront',blo[1]-c['clearance']-lo[1]+pad),('extraBack',hi[1]+pad-bhi[1]-c['clearance'])]:c[key]=max(c[key],round(float(amount),5))
    c['below']=max(c['below'],float(-lo[2]+margin));c['above']=max(c['above'],float(hi[2]+margin-project['board']['thickness']))
    c['shape']='rectangle';validate_project(project)
    return {'enclosure':c,'bounds':[lo.tolist(),hi.tolist()]}
