"""Adapt the supplied V3 3MF meshes to the measured V3b PCB geometry.

Run with Python + numpy, trimesh, manifold3d, shapely, mapbox-earcut,
matplotlib and lxml. Originals are read only; outputs go one directory up.
"""
import sys, json, zipfile, hashlib
from pathlib import Path
sys.path.insert(0,r'C:\Users\Nick\.codex\visualizations\2026\09\29\01a0ece9-af29-7560-aa5b-b8634a1263b9\geometry-deps')
import numpy as np
import trimesh
from lxml import etree as ET
from shapely.geometry import box, LineString
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

WORK=Path(__file__).resolve().parent
OUT=WORK.parent
SW=Path(r'C:\Users\Nick\Dropbox (Personal)\Solidworks\STL Outputs')
NS='http://schemas.microsoft.com/3dmanufacturing/core/2015/02'
Q=lambda tag:'{'+NS+'}'+tag
EXTENSION=1.397
USB_DX=.2386
USB_DZ=.186
LID_OFFSET=np.array([3.778341,17.34274,0])
FLOOR_REMAINDER=1.2

def bounds_box(lo,hi):
    lo,hi=np.asarray(lo),np.asarray(hi)
    return trimesh.creation.box(extents=hi-lo,transform=trimesh.transformations.translation_matrix((hi+lo)/2))

def rounded_rect(cx,cy,w,h,r):
    return box(cx-w/2+r,cy-h/2+r,cx+w/2-r,cy+h/2-r).buffer(r,quad_segs=12)

def cutter(poly):
    m=trimesh.creation.extrude_polygon(poly,2.01-FLOOR_REMAINDER,engine='earcut')
    return m.apply_translation([0,0,FLOOR_REMAINDER])

def read_xml(path):
    with zipfile.ZipFile(path) as z:return ET.fromstring(z.read('3D/3dmodel.model'))

def write_mesh(obj,m):
    node=obj.find(Q('mesh'));obj.remove(node)
    node=ET.SubElement(obj,Q('mesh')); vv=ET.SubElement(node,Q('vertices'));ff=ET.SubElement(node,Q('triangles'))
    for v in m.vertices:ET.SubElement(vv,Q('vertex'),**{k:format(float(x),'.8f') for k,x in zip('xyz',v)})
    for f in m.faces:ET.SubElement(ff,Q('triangle'),**{k:str(int(x)) for k,x in zip(['v1','v2','v3'],f)})

def save3mf(source,path,root,thumbnail):
    for m in root.findall(Q('metadata')):
        if m.get('name')=='ModificationDate':m.text='2026-09-29'
    ET.SubElement(root,Q('metadata'),name='Description').text='V3b: USB-end extension 1.397 mm; see accompanying fit notes.'
    # The original empty color resources are unused and omitted for spec compliance.
    resources=root.find(Q('resources'))
    for resource in list(resources):
        if resource.tag.endswith('colorgroup') and len(resource)==0:resources.remove(resource)
    with zipfile.ZipFile(source) as src,zipfile.ZipFile(path,'w',compression=zipfile.ZIP_DEFLATED) as dest:
        for item in src.infolist():
            if item.filename=='3D/3dmodel.model':data=ET.tostring(root,xml_declaration=True,encoding='UTF-8')
            elif item.filename=='Metadata/thumbnail.png':data=thumbnail.read_bytes()
            else:data=src.read(item.filename)
            dest.writestr(item.filename,data)

def thumbnail(meshes,path,colors):
    fig=plt.figure(figsize=(5,5));ax=fig.add_subplot(projection='3d')
    for m,c in zip(meshes,colors):
        # Flat lighting preserves real mesh geometry without displaying tessellation edges.
        n=m.face_normals; light=np.array([.2,-.4,1]);light/=np.linalg.norm(light)
        shade=.55+.45*np.clip(n@light,0,1)
        rgb=np.asarray(matplotlib.colors.to_rgb(c))
        ax.add_collection3d(Poly3DCollection(m.triangles,facecolors=shade[:,None]*rgb,edgecolors='none'))
    b=np.array([np.min([m.bounds[0] for m in meshes],axis=0),np.max([m.bounds[1] for m in meshes],axis=0)])
    center=b.mean(0);span=max(b[1]-b[0]);ax.set_xlim(center[0]-span/2,center[0]+span/2);ax.set_ylim(center[1]-span/2,center[1]+span/2);ax.set_zlim(center[2]-span/2,center[2]+span/2)
    ax.set_box_aspect([1,1,1]);ax.view_init(52,-48);ax.set_axis_off();fig.subplots_adjust(0,0,1,1);fig.savefig(path,dpi=150,transparent=True);plt.close(fig)

shellsrc=SW/'BlinkyHawk_V3_Shell.3MF'
lidsrc=SW/'BlinkyHawkV3_Lid.3MF'
original=trimesh.load_scene(shellsrc).to_mesh()
shell=original.copy();v=shell.vertices.copy()
# No source vertices fall in Y=66..69; all cross-boundary faces are planar
# wall/floor strips. This inserts length while rigidly translating end features.
v[v[:,1]>68,1]+=EXTENSION
usb=(v[:,1]>75)&(v[:,0]>10.8)&(v[:,0]<20.5)&(v[:,2]>5)&(v[:,2]<9)
assert usb.sum()==88
v[usb,0]+=USB_DX;v[usb,2]+=USB_DZ;shell.vertices=v
# Relief under relocated J4, connected to a shallow J5 underside wire route.
j4_relief=rounded_rect(12.706,50.759,3.5,4.0,.6)
j5_route=LineString([(15.246,18.8),(15.246,51.648)]).buffer(1.2,quad_segs=12)
relief=cutter(j4_relief.union(j5_route))
shell=trimesh.boolean.difference([shell,relief],engine='manifold')
assert shell.is_volume and len(shell.split())==1
shellroot=read_xml(shellsrc)
write_mesh(shellroot.find('.//'+Q('object')+'[@id="2"]'),shell)
shellroot.find('.//'+Q('object')+'[@id="1"]').set('name','BlinkyHawk_V3b_Shell')
thumbnail([shell],WORK/'shell_thumbnail.png',['#687887'])
save3mf(shellsrc,OUT/'BlinkyHawk_V3b_Shell.3mf',shellroot,WORK/'shell_thumbnail.png')
shell.export(OUT/'BlinkyHawk_V3b_Shell.stl')

lidroot=read_xml(lidsrc)
for vertex in lidroot.findall('.//'+Q('vertex')):
    p=np.array([float(vertex.get(c)) for c in 'xyz'])+LID_OFFSET
    if p[1]>68:p[1]+=EXTENSION
    for c,x in zip('xyz',p):vertex.set(c,format(float(x),'.8f'))
lidroot.find('.//'+Q('object')+'[@id="1"]').set('name','BlinkyHawk_V3b_Lid')
lidparts=[]
for obj in lidroot.findall('.//'+Q('object')):
    vs=obj.findall('.//'+Q('vertex'));fs=obj.findall('.//'+Q('triangle'))
    if not vs:continue
    lidparts.append(trimesh.Trimesh(vertices=[[float(v.get(c)) for c in 'xyz'] for v in vs],faces=[[int(f.get(c)) for c in ['v1','v2','v3']] for f in fs],process=True))
assert len(lidparts)==15 and all(m.is_volume for m in lidparts)
thumbnail(lidparts,WORK/'lid_thumbnail.png',['#b17932' if i not in [10,14] else '#687887' for i in range(15)])
save3mf(lidsrc,OUT/'BlinkyHawk_V3b_Lid_Multipart.3mf',lidroot,WORK/'lid_thumbnail.png')
# The STL is a single fused material. The 3MF retains all 15 original bodies.
lid=trimesh.boolean.union(lidparts,engine='manifold')
assert lid.is_volume and len(lid.split())==1
lid.export(OUT/'BlinkyHawk_V3b_Lid.stl')
single_root=read_xml(lidsrc)
resources=single_root.find(Q('resources'))
for item in list(resources):resources.remove(item)
obj=ET.SubElement(resources,Q('object'),id='1',type='model',name='BlinkyHawk_V3b_Lid')
ET.SubElement(obj,Q('mesh'))
write_mesh(obj,lid)
save3mf(lidsrc,OUT/'BlinkyHawk_V3b_Lid.3mf',single_root,WORK/'lid_thumbnail.png')

report={
 'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [shellsrc,lidsrc]},
 'changes_mm':{'length_increase':EXTENSION,'USB_dx':USB_DX,'USB_dz':USB_DZ,'minimum_floor_under_relief':FLOOR_REMAINDER},
 'shell':{'bounds':shell.bounds.tolist(),'volume_mm3':shell.volume,'watertight':shell.is_watertight,'connected_solids':len(shell.split())},
 'lid':{'bounds':lid.bounds.tolist(),'volume_mm3':lid.volume,'watertight':lid.is_watertight,'connected_solids_stl':len(lid.split()),'bodies_3mf':len(lidparts)},
}
(WORK/'build_report.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
