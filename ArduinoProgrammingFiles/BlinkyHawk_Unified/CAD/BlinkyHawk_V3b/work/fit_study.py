exec(open(__file__.replace('fit_study.py','inspect_meshes.py')).read().split('meshes={}')[0])
from collections import defaultdict
import re
shell=trimesh.load_scene(SW/'BlinkyHawk_V3_Shell.3MF').to_mesh()
scene=trimesh.load_scene(OUT/'new_pcb.glb')
groups=defaultdict(list)
T=np.array([[1000,0,0,-142.742],[0,0,-1000,138.008],[0,1000,0,2.55],[0,0,0,1.]])
for name in scene.graph.nodes_geometry:
    t,g=scene.graph[name]; groups[name.rsplit('_',1)[0]].append(scene.geometry[g].copy().apply_transform(T@t))
meshes={}
for name,parts in groups.items():
    m=trimesh.util.concatenate(parts);m.merge_vertices();m.remove_unreferenced_vertices()
    if m.is_watertight: m.fix_normals(multibody=True)
    meshes[name]=m
new=shell.copy();v=new.vertices.copy();v[v[:,1]>68,1]+=1.397
port=(v[:,1]>75)&(v[:,0]>10.8)&(v[:,0]<20.5)&(v[:,2]>5)&(v[:,2]<9)
v[port,0]+=.23866;v[port,2]+=.186;new.vertices=v
new.export(OUT/'shell_candidate.stl')
print('shell closed',new.is_watertight, 'volume',new.volume,'changed port vertices',int(port.sum()))
summary=[]
for name,m in meshes.items():
    bounds=m.bounds
    row=dict(name=name,bounds=bounds.tolist(),closed=m.is_watertight)
    # A closed bounding box conservatively checks even components with incomplete display meshes.
    bbox=m.bounding_box.to_mesh()
    row['bbox_overlap_old_mm3']=float(trimesh.boolean.intersection([shell,bbox],engine='manifold').volume)
    row['bbox_overlap_new_mm3']=float(trimesh.boolean.intersection([new,bbox],engine='manifold').volume)
    if m.is_volume:
        row['exact_overlap_old_mm3']=float(trimesh.boolean.intersection([shell,m],engine='manifold').volume)
        row['exact_overlap_new_mm3']=float(trimesh.boolean.intersection([new,m],engine='manifold').volume)
    if row['bbox_overlap_old_mm3']>.0001 or row['bbox_overlap_new_mm3']>.0001: print(json.dumps(row))
    summary.append(row)
    m.export(OUT/('pcb_part_'+re.sub(r'[^a-zA-Z0-9_-]','_',name)+'.ply'))
(OUT/'fit_study.json').write_text(json.dumps(summary,indent=2))
fig,axs=plt.subplots(1,2,figsize=(12,10))
for ax,case,title in zip(axs,[shell,new],['Original shell / new PCB','Extended shell / new PCB']):
    for z,color in [(3,'#526578'),(6.9,'#778fa9'),(15,'#b7c2cc')]:
        p=case.section(plane_origin=[0,0,z],plane_normal=[0,0,1])
        for pts in p.discrete: ax.plot(pts[:,0],pts[:,1],color=color,lw=1)
    p=meshes['=>[0:1:1:11]'].section(plane_origin=[0,0,3],plane_normal=[0,0,1])
    for pts in p.discrete:ax.plot(pts[:,0],pts[:,1],color='#019773',lw=1.5)
    for name,color in [('CHAMFER9:1','#cd803b'),('BZ1','#a46298'),('J4','#dd3e4c')]:
        b=meshes[name].bounds; ax.add_patch(plt.Rectangle(b[0,:2],*(b[1,:2]-b[0,:2]),fill=False,color=color,lw=1.4));ax.text(b[0,0],b[0,1]-1,name,fontsize=8,color=color)
    ax.set_aspect('equal');ax.set_title(title);ax.grid(alpha=.2);ax.set_xlabel('mm');ax.set_ylabel('mm')
fig.tight_layout();fig.savefig(OUT/'fit_study.png',dpi=140)
