import sys, json, zipfile
from pathlib import Path
sys.path.insert(0,r'C:\Users\Nick\.codex\visualizations\2026\09\29\01a0ece9-af29-7560-aa5b-b8634a1263b9\geometry-deps')
import trimesh, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
OUT=Path(__file__).resolve().parent
SW=Path(r'C:\Users\Nick\Dropbox (Personal)\Solidworks\STL Outputs')
meshes={}
for filename in ['BlinkyHawkV3_Lid.3MF','BlinkyHawk_V3_Shell.3MF']:
    s=trimesh.load_scene(SW/filename)
    m=s.to_mesh(); meshes[filename]=m
    print(filename,'bounds',m.bounds,'watertight',m.is_watertight,'volume',m.volume)
    for name,g in s.geometry.items(): print(name,'bounds',g.bounds.tolist(),'watertight',g.is_watertight,'volume',g.volume)
    with zipfile.ZipFile(SW/filename) as z: (OUT/(Path(filename).stem+'_thumbnail.png')).write_bytes(z.read('Metadata/thumbnail.png'))
    m.export(OUT/(Path(filename).stem+'_original.stl'))
    fig=plt.figure(figsize=(15,10))
    for i,(elev,azim) in enumerate([(70,-60),(-60,-60),(0,-90),(0,0)]):
        ax=fig.add_subplot(2,2,i+1,projection='3d')
        ax.add_collection3d(Poly3DCollection(m.triangles,facecolor='#96b6cc',edgecolor='#223344',linewidth=.18))
        ax.set_xlim(m.bounds[:,0]); ax.set_ylim(m.bounds[:,1]); ax.set_zlim(m.bounds[0,2]-1,m.bounds[1,2]+1)
        ax.set_box_aspect(m.extents+2); ax.view_init(elev,azim); ax.set_xlabel('X');ax.set_ylabel('Y');ax.set_zlabel('Z')
    fig.suptitle(filename); fig.tight_layout();fig.savefig(OUT/(Path(filename).stem+'_views.png'),dpi=140);plt.close(fig)

rows=[]
for p in SW.glob('BlinkyHawk_V3 - *.STL'):
    if any(k in p.name for k in ['PCB_3','SmallToggle','12mmBuzzer','LED-SMD','BlinkyHawkV3_Lid','BlinkyHawk_V3_Shell','USB','TYPE C','Shield','PCB (1)-14','PCB (1)-0']):
        m=trimesh.load_mesh(p)
        r=dict(name=p.name,bounds=m.bounds.tolist(),extent=m.extents.tolist(),watertight=m.is_watertight,volume=m.volume)
        rows.append(r); print(json.dumps(r))
(OUT/'assembly_mesh_bounds.json').write_text(json.dumps(rows,indent=2))
