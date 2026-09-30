"""Immutable imported references. STEP faces keep their local attachment frames."""
import hashlib,uuid,re,copy
from pathlib import Path
from functools import lru_cache
import cadquery as cq
import numpy as np
import trimesh
from geometry import mesh_data,euler

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
def original_meshes(asset):
    path=asset_path(asset);meshes=[]
    if path.suffix in ['.step','.stp']:
        shape=cq.importers.importStep(str(path)).val()
        solids=shape.Solids() or [shape]
        if len(solids)>1500:raise ValueError('This reference has more than 1,500 bodies; export a simplified mechanical model.')
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
    return {'id':str(uuid.uuid4()),'name':Path(name).name,'asset':asset,'format':Path(name).suffix[1:].upper(),'offset':[0,0,0],'rotation':[0,0,0],'visible':True,'bounds':[coords.min(0).tolist(),coords.max(0).tolist()],'planarFaces':sum(1 for m in meshes for f in m['faces'] if f['type']=='PLANE')}

def render_reference(ref):
    result=[];r=euler(ref.get('rotation',[0,0,0]));offset=np.asarray(ref.get('offset',[0,0,0]))
    for source in original_meshes(ref['asset']):
        m=dict(source);m['vertices']=np.round(np.asarray(source['vertices'])@r.T+offset,5).tolist()
        m.update(id=f"{ref['id']}:{source['body']}",refId=ref['id'],name=ref['name'],kind='reference',visible=ref.get('visible',True))
        # Face descriptors deliberately remain in the asset's local coordinate frame.
        result.append(m)
    return result
