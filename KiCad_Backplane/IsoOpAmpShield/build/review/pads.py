import sys,math; sys.path.insert(0,"tools")
from sexp import parse
from collections import defaultdict
t=open(sys.argv[1] if len(sys.argv)>1 else "IsoOpAmpShield.kicad_pcb",encoding="utf8").read()
root=parse(t); root=root[0] if isinstance(root[0],list) else root
def kids(n,name): return [c for c in n if isinstance(c,list) and c and c[0]==name]
def one(n,name):
    k=kids(n,name); return k[0] if k else None
pads=[]; fps={}
for f in kids(root,"footprint"):
    ref=[p[2] for p in kids(f,"property") if p[1]=="Reference"][0]
    tr=one(f,"transform"); fx,fy=map(float,one(tr,"translate")[1:3]); rt=one(tr,"rotate"); fr=float(rt[1]) if rt else 0
    fps[ref]=(fx,fy,fr,one(f,"layer")[1])
    for p in kids(f,"pad"):
        at=one(p,"at") or ["at",0,0]
        if one(p,"transform"): at=["at"]+one(one(p,"transform"),"translate")[1:3]
        px,py=float(at[1]),float(at[2])
        r=math.radians(fr); x=fx+px*math.cos(r)+py*math.sin(r); y=fy-px*math.sin(r)+py*math.cos(r)
        net=one(p,"net"); net=net[-1] if net else ""
        pads.append(dict(ref=ref,num=p[1],typ=p[2],x=x,y=y,net=net,drill=one(p,"drill"),size=one(p,"size")))
