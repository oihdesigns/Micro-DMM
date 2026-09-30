exec(open(__file__.replace('detail_geometry.py','inspect_meshes.py')).read().split('meshes={}')[0])
scene=trimesh.load_scene(OUT/'new_pcb.glb')
print('NEW PCB bounds',scene.bounds, 'units',scene.units)
groups={}
for name in scene.graph.nodes_geometry:
    t,g=scene.graph[name]; m=scene.geometry[g].copy().apply_transform(t)
    groups.setdefault(name.rsplit('_',1)[0],[]).append(m)
for name,parts in groups.items():
    m=trimesh.util.concatenate(parts); m.merge_vertices()
    print(name, np.round(m.bounds*1000,4).tolist(), 'closed',m.is_watertight)
(OUT/'new_pcb_group_bounds.json').write_text(json.dumps({n:trimesh.util.concatenate(ms).bounds.tolist() for n,ms in groups.items()},indent=2))

shell=trimesh.load_scene(SW/'BlinkyHawk_V3_Shell.3MF').to_mesh()
fig,axs=plt.subplots(2,3,figsize=(15,12))
for ax,z in zip(axs.flat,[1.,2.6,4.0,5.5,8.5,15.]):
    p=shell.section(plane_origin=[0,0,z],plane_normal=[0,0,1])
    for pts in p.discrete: ax.plot(pts[:,0],pts[:,1],color='#193c57',lw=1)
    ax.set_aspect('equal');ax.set_title(f'Shell at Z={z} mm');ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(OUT/'shell_sections.png',dpi=160)
for axis,value in [(2,2.6),(2,4.5),(1,75),(0,30)]:
    origin=np.zeros(3);origin[axis]=value;norm=np.zeros(3);norm[axis]=1
    p=shell.section(plane_origin=origin,plane_normal=norm)
    print('SECTION',axis,value)
    for pts in p.discrete: print('bounds',np.round([pts.min(0),pts.max(0)],5).tolist())

