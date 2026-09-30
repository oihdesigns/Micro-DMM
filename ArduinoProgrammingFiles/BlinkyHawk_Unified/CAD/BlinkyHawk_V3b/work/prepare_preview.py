import sys,json
from pathlib import Path
sys.path.insert(0,r'C:\Users\Nick\.codex\visualizations\2026\09\29\01a0ece9-af29-7560-aa5b-b8634a1263b9\geometry-deps')
import trimesh,numpy as np
W=Path(__file__).resolve().parent;O=W.parent
V=Path(r'C:\Users\Nick\.codex\visualizations\2026\09\29\01a0ece9-af29-7560-aa5b-b8634a1263b9\blinkyhawk-v3b.html')
parts=[]
def add(name,m,group,color):
    m.merge_vertices()
    parts.append(dict(name=name,group=group,color=color,v=np.round(m.vertices,4).ravel().tolist(),f=m.faces.ravel().tolist()))
add('Shell',trimesh.load_mesh(O/'BlinkyHawk_V3b_Shell.stl'),'shell','--muted-foreground')
lid=trimesh.load_scene(O/'BlinkyHawk_V3b_Lid_Multipart.3mf')
for name,m in lid.geometry.items():add(name,m,'lid','--muted-foreground' if name in ['body26125432','body26149282'] else '--viz-series-2')
for name,file,token in [('Main PCB','___0_1_1_11_','--viz-series-1'),('XIAO PCB','___0_1_1_39_','--viz-series-1'),('USB','CHAMFER9_1','--foreground'),('Shield','Shield_1','--muted-foreground'),('Buzzer','BZ1','--foreground'),('Side LED','LED1','--viz-series-2'),('Battery connection','J4','--foreground')]:
    add(name,trimesh.load_mesh(W/('pcb_part_'+file+'.ply')),'pcb',token)
text=V.read_text(encoding='utf8');data=json.dumps(parts,separators=(',',':'))
if 'GEOMETRY_DATA' in text:text=text.replace('GEOMETRY_DATA',data)
else:
    start=text.index('id="blinkyhawk-v3b-data">')+len('id="blinkyhawk-v3b-data">');end=text.index('</script>',start);text=text[:start]+data+text[end:]
V.write_text(text,encoding='utf8')
assert V.stat().st_size<1000000
print('Preview bytes',V.stat().st_size)
