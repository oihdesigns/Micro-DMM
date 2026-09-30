import sys, json, re, zipfile, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = Path(r'C:\Users\Nick\Documents\GitHub\Micro-DMM')
SW = Path(r'C:\Users\Nick\Dropbox (Personal)\Solidworks\STL Outputs')

def sexp(path):
    toks = re.findall(r'"(?:\\.|[^"\\])*"|[()]|[^\s()]+', path.read_text(encoding='utf8'))
    stack=[]; result=[]
    for t in toks:
        if t=='(':
            n=[]
            (stack[-1] if stack else result).append(n); stack.append(n)
        elif t==')': stack.pop()
        else: stack[-1].append(t[1:-1] if t.startswith('"') else t)
    return result[0]
def children(n, tag): return [a for a in n if isinstance(a,list) and a and a[0]==tag]
def one(n, tag, default=None): return next(iter(children(n,tag)), default)
def nums(n): return [float(v) for v in n[1:]] if n else None
def inspect_board(path):
    b=sexp(path); fps={}
    for f in children(b,'footprint'):
        props={p[1]:p[2] for p in children(f,'property')}
        pads=[]
        for p in children(f,'pad'):
            pads.append(dict(id=p[1], kind=p[2], shape=p[3], at=nums(one(p,'at')),size=nums(one(p,'size')),drill=one(p,'drill'),layers=one(p,'layers'),net=one(p,'net')))
        graphics=[a for a in f if isinstance(a,list) and a[0] in ['fp_rect','fp_line','fp_circle','fp_arc','fp_poly']]
        fps[props.get('Reference','?')] = dict(name=f[1],value=props.get('Value'),at=nums(one(f,'at')),layer=one(f,'layer')[1],pads=pads,models=children(f,'model'),graphics=graphics)
    edges=[a for a in b if isinstance(a,list) and a[0].startswith('gr_') and one(a,'layer')==['layer','Edge.Cuts']]
    return dict(path=str(path),thickness=nums(one(one(b,'general'),'thickness'))[0],edges=edges,footprints=fps)

data={}
for label, name in [('old','OpenLead_Headless_V3'),('new','BlinkyHawk_V3b')]:
    data[label]=inspect_board(ROOT/'PCBDesigns'/name/(name+'.kicad_pcb'))
    print(label, 'thickness',data[label]['thickness'],'EDGES',json.dumps(data[label]['edges']))
    for ref,f in sorted(data[label]['footprints'].items()):
        print(ref,f['name'],f['value'],f['at'],f['layer'])
        if ref.startswith(('J','SW','BZ','U')) or 'MountingHole' in f['name']: print('  models',f['models'])
(OUT/'boards.json').write_text(json.dumps(data,indent=2))
print('CHANGES')
for ref in sorted(set(data['old']['footprints'])|set(data['new']['footprints'])):
    a=data['old']['footprints'].get(ref); b=data['new']['footprints'].get(ref)
    if not a or not b: print(ref,'ADDED' if b else 'REMOVED'); continue
    changes={k:[a[k],b[k]] for k in ['name','value','at','layer'] if a[k]!=b[k]}
    if changes: print(ref,json.dumps(changes))

for filename in ['BlinkyHawkV3_Lid.3MF','BlinkyHawk_V3_Shell.3MF']:
    with zipfile.ZipFile(SW/filename) as z:
        print(filename,z.namelist())
        for p in z.namelist():
            if p.endswith('.model'):
                xml=ET.fromstring(z.read(p)); ns={'m':xml.tag.split('}')[0][1:]}
                print('root',xml.attrib)
                for obj in xml.findall('.//m:object',ns):
                    v=np.array([[float(a.attrib[c]) for c in 'xyz'] for a in obj.findall('.//m:vertex',ns)])
                    tris=np.array([[int(a.attrib[c]) for c in ['v1','v2','v3']] for a in obj.findall('.//m:triangle',ns)])
                    print('object',obj.attrib,'vertices',v.shape,'tris',tris.shape,'bounds',v.min(0) if len(v) else None,v.max(0) if len(v) else None)
                print('build',[e.attrib for e in xml.findall('.//m:build/m:item',ns)])
