"""Parametric enclosure solids and local attachment frames, backed by Open CASCADE."""
import io,json,math,uuid
from pathlib import Path
import numpy as np
import cadquery as cq
from shapely.geometry import Polygon,box
from shapely.affinity import translate
from shapely.ops import unary_union

DEFAULTS={'clearance':.6,'wall':2.,'floor':2.,'below':3.,'above':8.,'autoHeight':True,'topMargin':2.,'corner':3.,'lid':2.,'lip':1.5,'lipWidth':1.2,'lipClearance':.25,'shape':'rectangle','extraLeft':0.,'extraRight':0.,'extraFront':0.,'extraBack':0.}

def vec(v):return np.asarray(v,dtype=float)
def unit(v):
    v=vec(v);n=np.linalg.norm(v)
    if n<1e-8:raise ValueError('A plane direction cannot be zero.')
    return v/n
def frame(origin=(0,0,0),normal=(0,0,1),x=(1,0,0)):
    n=unit(normal);x=vec(x)-np.dot(x,n)*n
    if np.linalg.norm(x)<1e-8:x=np.cross([0,1,0] if abs(n[1])<.9 else [1,0,0],n)
    x=unit(x);return np.asarray(origin,dtype=float),np.column_stack([x,np.cross(n,x),n])
def euler(angles):
    x,y,z=np.radians(angles);cx,sx=np.cos(x),np.sin(x);cy,sy=np.cos(y),np.sin(y);cz,sz=np.cos(z),np.sin(z)
    return np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])@np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])@np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
AXES={'XY':((0,0,1),(1,0,0)),'XZ':((0,-1,0),(1,0,0)),'YZ':((1,0,0),(0,1,0)),'-XY':((0,0,-1),(1,0,0)),'-XZ':((0,1,0),(-1,0,0)),'-YZ':((-1,0,0),(0,-1,0))}

def numeric(value,label,lo,hi):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lo<=value<=hi:raise ValueError(f'{label} must be a finite number between {lo:g} and {hi:g} mm.')
    return float(value)

def validate_project(project):
    if not isinstance(project,dict) or project.get('version')!=1:raise ValueError('Unsupported project format.')
    b=project.get('board')
    if not b:raise ValueError('Import a KiCad board or STEP PCB assembly first.')
    numeric(b.get('thickness'), 'PCB thickness',.1,20)
    if not 3<=len(b.get('outline',[]))<=15000:raise ValueError('Invalid PCB outline.')
    for p in b['outline']:
        if len(p)!=2:raise ValueError('PCB outline vertices must have two coordinates.')
        for v in p:numeric(v,'PCB coordinate',-10000,10000)
    if not Polygon(b['outline']).is_valid:raise ValueError('The PCB outline is self-intersecting.')
    c=DEFAULTS|project.get('enclosure',{})
    for key in DEFAULTS:
        if key not in ['shape','autoHeight']:numeric(c[key],key,0 if key.startswith('extra') or key in ['corner','lip','below','above','lipClearance'] else .05,300)
    if c['lipClearance']>=min(c['wall'],c['lipWidth'])*2:raise ValueError('The lid lip clearance is too large for the lip width.')
    for comp in b.get('components',[]):
        numeric(comp.get('height',0),'Component height',0,300)
        for k in ['width','depth']:numeric(comp.get(k),f'Component {k}',.05,1000)
        for v in comp.get('xy',[]):numeric(v,'Component position',-10000,10000)
    if len(project.get('features',[]))>150 or len(project.get('planes',[]))>40:raise ValueError('This project exceeds the 150-feature/40-plane limit.')
    for f in project.get('features',[])+project.get('planes',[])+project.get('references',[]):
        for key in ['offset','rotation']:
            xyz=f.get(key,[0,0,0])
            if len(xyz)!=3:raise ValueError(f'{key} requires three numbers.')
            for v in xyz:numeric(v,key,-10000,10000)
    project['enclosure']=c
    return project

def component_frame(board,c,point='center'):
    rotation=euler([0,0,c.get('rotation',0)])
    center2=vec(c.get('center',[0,0]));xy=vec(c['xy'])+(rotation@np.r_[center2,0])[:2]
    height=c.get('height',2);z0=c.get('zMin',board['thickness'] if c.get('side')!='bottom' else -height)
    center=np.r_[xy,z0+height/2]
    if point=='origin':return frame(np.r_[c['xy'],board['thickness'] if c.get('side')!='bottom' else 0],x=rotation[:,0])
    axis={'top':2,'bottom':2,'right':0,'left':0,'back':1,'front':1}.get(point)
    if axis is None:return center,rotation
    sign=-1 if point in ['bottom','left','front'] else 1
    dims=[c['width'],c['depth'],height];normal=rotation[:,axis]*sign
    return frame(center+normal*dims[axis]/2,normal,rotation[:,0] if axis!=0 else rotation[:,1])

def component_shape(board,c):
    p,r=component_frame(board,c)
    return cq.Workplane(cq.Plane(tuple(p),tuple(r[:,0]),tuple(r[:,2]))).box(c['width'],c['depth'],max(.03,c.get('height',2))).val()

def metrics(project):
    b=project['board'];c=project['enclosure'];bounds=Polygon(b['outline']).bounds
    tops=[b['thickness']]+[x['zMax'] if 'zMax' in x else component_shape(b,x).BoundingBox().zmax for x in b.get('components',[]) if x.get('height',0)>.01]
    top=max(b['thickness']+c['above'],max(tops)+c['topMargin']) if c['autoHeight'] else b['thickness']+c['above']
    xmin=bounds[0]-c['clearance']-c['wall']-c.get('extraLeft',0);xmax=bounds[2]+c['clearance']+c['wall']+c.get('extraRight',0)
    ymin=bounds[1]-c['clearance']-c['wall']-c.get('extraFront',0);ymax=bounds[3]+c['clearance']+c['wall']+c.get('extraBack',0)
    return {'xmin':xmin,'ymin':ymin,'xmax':xmax,'ymax':ymax,'floor':-c['below'],'bottom':-c['below']-c['floor'],'top':top,'width':xmax-xmin,'length':ymax-ymin,'height':top+c['lid']+c['below']+c['floor']}

def space_outline(project):
    """Sweep the outline along each extra-space interval without moving the PCB."""
    c=project['enclosure'];poly=translate(Polygon(project['board']['outline']),-c.get('extraLeft',0),-c.get('extraFront',0))
    for dx,dy in [(c.get('extraLeft',0)+c.get('extraRight',0),0),(0,c.get('extraFront',0)+c.get('extraBack',0))]:
        if dx+dy<=0:continue
        pieces=[poly,translate(poly,dx,dy)];points=list(poly.exterior.coords)
        for a,b in zip(points,points[1:]):
            q=Polygon([a,b,(b[0]+dx,b[1]+dy),(a[0]+dx,a[1]+dy)])
            if q.area>1e-9:pieces.append(q)
        poly=unary_union(pieces)
    return poly

def footprint(project,met,inset=0):
    c=project['enclosure']
    if c['shape']=='outline':return space_outline(project).buffer(c['clearance']+c['wall']-inset,quad_segs=12)
    x0,y0,x1,y1=[met[k] for k in ['xmin','ymin','xmax','ymax']];x0+=inset;y0+=inset;x1-=inset;y1-=inset
    radius=max(0,min(c['corner']-inset,(x1-x0)/2-.001,(y1-y0)/2-.001))
    return box(x0+radius,y0+radius,x1-radius,y1-radius).buffer(radius,quad_segs=24) if radius else box(x0,y0,x1,y1)

def reference_transform(project,ref,met,stack=()):
    if not ref.get('mount'):return vec(ref.get('offset',[0,0,0])),euler(ref.get('rotation',[0,0,0]))
    token='reference:'+ref['id']
    if token in stack:raise ValueError('Hardware and construction planes have a circular attachment.')
    p,r=attach_frame(project,ref['mount'],met,stack+(token,))
    face=ref.get('mountFace')
    if face:q,s=frame(face['origin'],face['normal'],face['xDir'])
    else:
        bounds=np.asarray(ref['bounds']);q,s=frame([(bounds[0][0]+bounds[1][0])/2,(bounds[0][1]+bounds[1][1])/2,bounds[1][2]])
    rr=r@euler(ref.get('rotation',[0,0,0]))@s.T
    return p+r@vec(ref.get('offset',[0,0,0]))-rr@q,rr

def attach_frame(project,spec,met,stack=()):
    a=spec.get('anchor',{'kind':'origin','plane':'XY'});kind=a.get('kind','origin')
    if kind=='component':
        c=next((x for x in project['board']['components'] if x['id']==a.get('ref')),None)
        if c is None:raise ValueError('The attached PCB component is missing. Select a replacement attachment.')
        p,r=component_frame(project['board'],c,a.get('point','center'))
    elif kind=='pad':
        c=next((x for x in project['board']['components'] if x['id']==a.get('ref')),None)
        pad=next((x for x in c.get('pads',[]) if x['number']==a.get('pad')),None) if c else None
        if not pad:raise ValueError('The attached component pad is missing.')
        rotation=euler([0,0,c.get('rotation',0)])
        xy=(vec(c['xy'])+(rotation@np.r_[pad['localXY'],0])[:2]) if 'localXY' in pad else pad['xy']
        p,r=frame([*xy,project['board']['thickness'] if c['side']=='top' else 0],normal=(0,0,1 if c['side']=='top' else -1),x=rotation[:,0])
    elif kind=='hole':
        h=next((x for x in project['board'].get('holes',[]) if x['id']==a.get('ref')),None)
        if h is None:raise ValueError('The attached PCB hole is missing.')
        p,r=frame([*h['xy'],met['floor']])
    elif kind=='plane':
        pid=a.get('ref')
        if pid in stack:raise ValueError('The construction planes have a circular attachment.')
        plane=next((x for x in project.get('planes',[]) if x['id']==pid),None)
        if not plane:raise ValueError('The attached construction plane is missing.')
        p,r=attach_frame(project,plane,met,stack+(pid,))
    elif kind in ['face','reference']:
        ref=next((x for x in project.get('references',[]) if x['id']==a.get('ref')),None)
        if not ref:raise ValueError('The attached reference model is missing.')
        if kind=='face':
            if 'face' not in a:raise ValueError('Pick a planar face on a STEP reference first.')
            f=a['face'];p,r=frame(f['origin'],f['normal'],f['xDir'])
        else:
            lo,hi=np.asarray(ref['bounds']);center=(lo+hi)/2;point=a.get('point','center')
            if point=='origin':p,r=frame()
            else:
                axis={'left':0,'right':0,'front':1,'back':1,'bottom':2,'top':2}.get(point)
                if axis is None:p,r=frame(center)
                else:
                    sign=-1 if point in ['left','front','bottom'] else 1;normal=np.zeros(3);normal[axis]=sign
                    center[axis]=(lo if sign<0 else hi)[axis];p,r=frame(center,normal)
        offset,rr=reference_transform(project,ref,met,stack);p=rr@p+offset;r=rr@r
    elif kind=='boardFace':
        from step_import import face_frame
        p,r=face_frame(project['board'],a)
    elif kind=='case':
        center=[(met['xmin']+met['xmax'])/2,(met['ymin']+met['ymax'])/2,(met['bottom']+met['top'])/2]
        point=a.get('point','floor')
        if point=='floor':center[2]=met['floor'];p,r=frame(center)
        elif point=='lid':center[2]=met['top']+project['enclosure']['lid'];p,r=frame(center)
        elif point=='rim':center[2]=met['top'];p,r=frame(center)
        else:
            axis=0 if point in ['left','right'] else 1;center[axis]=met[{'left':'xmin','right':'xmax','front':'ymin','back':'ymax'}[point]]
            normal=np.zeros(3);normal[axis]=-1 if point in ['left','front'] else 1;p,r=frame(center,normal,[0,1,0] if axis==0 else [1,0,0])
        if a.get('surface')=='inside':
            if point in ['left','right','front','back']:p=p-r[:,2]*project['enclosure']['wall']
            elif point=='lid':p[2]=met['top']
    else:
        if kind!='origin':raise ValueError('Unknown attachment type.')
        p,r=frame()
    projection=a.get('project','none')
    if projection!='none':
        if projection in ['front','back','left','right']:
            axis=0 if projection in ['left','right'] else 1
            p[axis]=met[{'left':'xmin','right':'xmax','front':'ymin','back':'ymax'}[projection]]
        elif projection=='floor':p[2]=met['floor']
        elif projection=='lid':p[2]=met['top']+project['enclosure']['lid']
        else:raise ValueError('Unknown projection surface.')
    if a.get('plane','face') in AXES:
        n,x=AXES[a['plane']];_,r=frame(normal=n,x=x)
    p=p+r@vec(spec.get('offset',[0,0,0]));r=r@euler(spec.get('rotation',[0,0,0]))
    return p,r

def poly_prism(poly,z,height):
    if poly.is_empty or poly.geom_type!='Polygon':raise ValueError('The outline offset split into multiple regions. Increase clearances or use the rectangular enclosure.')
    pts=list(poly.exterior.coords)[:-1]
    wp=cq.Workplane('XY').workplane(offset=z).polyline(pts).close()
    for h in poly.interiors:wp=wp.polyline(list(h.coords)[:-1]).close()
    return wp.extrude(height).val()

def rounded_prism(bounds,z,height,radius):
    x0,y0,x1,y1=bounds;w=x1-x0;d=y1-y0
    if w<=0 or d<=0 or height<=0:raise ValueError('The enclosure has a zero or negative dimension.')
    wp=cq.Workplane('XY').workplane(offset=z).center((x0+x1)/2,(y0+y1)/2).rect(w,d).extrude(height)
    r=min(radius,w/2-.001,d/2-.001)
    if r>.01:wp=wp.edges('|Z').fillet(r)
    return wp.val()

def make_board(b):
    if b.get('source')=='step':
        from step_import import board_shapes
        return board_shapes(b)[b['substrate']]['shape']
    poly=Polygon(b['outline'],b.get('cutouts',[]));solid=poly_prism(poly,0,b['thickness'])
    # Cut holes only if they are inside the substrate and not already Edge.Cuts loops.
    for h in b.get('holes',[]):
        if h.get('source')=='Edge.Cuts':continue
        x,y=h['xy'];d=h['diameter'];length=h.get('slotLength',d)
        wp=cq.Workplane('XY').workplane(offset=-.1).center(x,y)
        wp=wp.slot2D(length,d,h.get('angle',0)) if length>d+.001 else wp.circle(d/2)
        solid=solid.cut(wp.extrude(b['thickness']+.2).val())
    return solid

def make_base(project,met):
    c=project['enclosure'];b=project['board'];outer=[met[k] for k in ['xmin','ymin','xmax','ymax']]
    inner=[outer[0]+c['wall'],outer[1]+c['wall'],outer[2]-c['wall'],outer[3]-c['wall']]
    if c['shape']=='outline':
        poly=space_outline(project);outerpoly=poly.buffer(c['clearance']+c['wall'],quad_segs=12);innerpoly=poly.buffer(c['clearance'],quad_segs=12)
        prism=lambda inset,z,h:poly_prism(innerpoly.buffer(-inset,quad_segs=12),z,h)
        shell=poly_prism(outerpoly,met['bottom'],met['top']-met['bottom']).cut(poly_prism(innerpoly,met['floor'],met['top']-met['floor']+.1))
        lid=poly_prism(outerpoly,met['top'],c['lid'])
    elif c['shape']=='rectangle':
        prism=lambda inset,z,h:rounded_prism([inner[0]+inset,inner[1]+inset,inner[2]-inset,inner[3]-inset],z,h,max(0,c['corner']-c['wall']-inset))
        shell=rounded_prism(outer,met['bottom'],met['top']-met['bottom'],c['corner']).cut(prism(0,met['floor'],met['top']-met['floor']+.1))
        lid=rounded_prism(outer,met['top'],c['lid'],c['corner'])
    else:raise ValueError('Unknown enclosure outline style.')
    if c['lip']>.01:
        if c['lip']>=met['top']-met['floor']:raise ValueError('The lid lip extends to the enclosure floor.')
        lip=prism(c['lipClearance'],met['top']-c['lip'],c['lip']+.001)
        lip=lip.cut(prism(c['lipClearance']+c['lipWidth'],met['top']-c['lip']-.01,c['lip']+.02))
        lid=lid.fuse(lip).clean()
    return {'shell':shell.clean(),'lid':lid.clean()}

def feature_tool(f,p,r):
    d=f.get('dimensions',{});kind=f.get('type','box');depth=numeric(d.get('depth',5),'Feature extrusion depth',.05,1000)
    plane=cq.Plane(tuple(p),tuple(r[:,0]),tuple(r[:,2]));wp=cq.Workplane(plane)
    def size(key,default=5):return numeric(d.get(key,default),key,.05,1000)
    through=f.get('extent','symmetric')=='symmetric'
    if kind in ['box','window']:
        w=size('width',10);h=size('height',6);rad=numeric(d.get('radius',0),'Corner radius',0,min(w,h)/2-.001)
        # Sketch fillets produce genuine analytic arcs.
        profile=wp.sketch().rect(w,h)
        if rad>.001:profile=profile.vertices().fillet(rad)
        return profile.finalize().extrude(depth/2 if through else depth,both=through).val()
    if kind in ['cylinder','hole']:
        return wp.circle(size('diameter',3)/2).extrude(depth/2 if through else depth,both=through).val()
    if kind=='standoff':
        outer=size('diameter',6);inner=numeric(d.get('bore',2.2),'Screw bore',0,outer-.3)
        solid=wp.circle(outer/2).extrude(depth).val()
        if inner>0:solid=solid.cut(wp.circle(inner/2).extrude(depth+.02).val())
        return solid
    if kind=='vent':
        count=int(numeric(d.get('count',5),'Vent count',1,40));pitch=size('pitch',4);w=size('width',12);h=size('height',1.5)
        solids=[]
        for i in range(count):
            center=p+r[:,1]*((i-(count-1)/2)*pitch)
            solids.append(cq.Workplane(cq.Plane(tuple(center),tuple(r[:,0]),tuple(r[:,2]))).slot2D(max(w,h),min(w,h)).extrude(depth/2 if through else depth,both=through).val())
        return cq.Compound.makeCompound(solids)
    if kind=='text':
        text=str(d.get('text','PCB'))[:80]
        if not text.strip():raise ValueError('Enter lettering text.')
        return wp.text(text,size('height',4),depth,combine=False,font='Arial',kind='bold').val()
    if kind=='sketch':
        pts=d.get('points',[])
        if not 3<=len(pts)<=300:raise ValueError('A sketch needs 3–300 polygon vertices.')
        if not Polygon(pts).is_valid or Polygon(pts).area<.01:raise ValueError('The sketch is self-intersecting or has zero area.')
        return wp.polyline(pts).close().extrude(depth/2 if through else depth,both=through).val()
    raise ValueError('Unknown feature type.')

def mesh_data(shape,with_faces=False,tolerance=.12):
    vertices=[];triangles=[];faces=[]
    if with_faces:
        for i,face in enumerate(shape.Faces()):
            vv,tt=face.tessellate(tolerance,.2);start=len(triangles);base=len(vertices)
            vertices.extend([[round(v.x,5),round(v.y,5),round(v.z,5)] for v in vv]);triangles.extend([[a+base,b+base,c+base] for a,b,c in tt])
            record={'id':str(i),'start':start,'count':len(tt),'type':face.geomType()}
            if face.geomType()=='PLANE':
                origin=face.Center();normal=face.normalAt();p,r=frame(origin.toTuple(),normal.toTuple())
                record.update(origin=p.tolist(),normal=r[:,2].tolist(),xDir=r[:,0].tolist(),area=face.Area())
            faces.append(record)
    else:
        vv,tt=shape.tessellate(tolerance,.2);vertices=[[round(v.x,5),round(v.y,5),round(v.z,5)] for v in vv];triangles=[list(t) for t in tt]
    return {'vertices':vertices,'triangles':triangles,'faces':faces}

def build(project,reference_loader=None):
    validate_project(project);met=metrics(project);board=make_board(project['board']);bodies=make_base(project,met)
    feature_results=[];errors=[];warnings=[];tools={}
    for f in project.get('features',[]):
        if f.get('suppressed'):
            feature_results.append({'id':f['id'],'status':'suppressed'});continue
        try:
            if f.get('type')=='lidScrews':
                from fasteners import apply_fasteners
                changed,centers=apply_fasteners(project,met,bodies,f)
                delta=sum(abs(changed[k].Volume()-bodies[k].Volume()) for k in bodies);bodies.update(changed)
                feature_results.append({'id':f['id'],'status':'ok','centers':centers,'changedVolume':delta});continue
            p,r=attach_frame(project,f,met);tool=feature_tool(f,p,r);tools[f['id']]=tool
            if not tool.isValid():raise ValueError('This feature produces an invalid solid.')
            target=f.get('target','shell');op=f.get('operation','cut')
            if target not in ['shell','lid','both'] or op not in ['cut','add']:raise ValueError('Unknown feature target or operation.')
            changed={};delta=0
            for key in (['shell','lid'] if target=='both' else [target]):
                prev=bodies[key]
                nxt=prev.cut(tool).clean() if op=='cut' else prev.fuse(tool).clean()
                if not nxt.isValid() or not nxt.Solids():raise ValueError('This operation removes the entire part or creates an invalid solid.')
                if len(nxt.Solids())!=1:raise ValueError('This feature leaves disconnected solids. Extend it until it joins the part, or reduce the cut.')
                delta+=abs(nxt.Volume()-prev.Volume());changed[key]=nxt
            bodies.update(changed)
            msg='No material changed; the feature misses its target.' if delta<1e-6 else ''
            if msg:warnings.append(f"{f['name']}: {msg}")
            feature_results.append({'id':f['id'],'status':'warning' if msg else 'ok','message':msg,'origin':p.tolist(),'axes':r.tolist(),'changedVolume':delta})
        except Exception as exc:
            msg=str(exc);errors.append(f"{f.get('name','Feature')}: {msg}");feature_results.append({'id':f['id'],'status':'error','message':msg})
    planes=[]
    for plane in project.get('planes',[]):
        try:
            p,r=attach_frame(project,plane,met);planes.append({'id':plane['id'],'name':plane['name'],'origin':p.tolist(),'axes':r.tolist()})
        except Exception as exc:errors.append(f"{plane['name']}: {exc}")
    collisions=[]
    if project['board'].get('source')=='step':
        from step_import import collision_shapes
        checks=list(collision_shapes(project['board']))
    else:checks=[('PCB substrate',board,False)]+[(c['ref'],component_shape(project['board'],c),True) for c in project['board'].get('components',[]) if c.get('height',0)>.1]
    for name,shape,estimated in checks:
        for key,body in bodies.items():
            a,b=shape.BoundingBox(),body.BoundingBox()
            if a.xmax<b.xmin or a.xmin>b.xmax or a.ymax<b.ymin or a.ymin>b.ymax or a.zmax<b.zmin or a.zmin>b.zmax:continue
            volume=shape.intersect(body).Volume()
            if volume>.005:collisions.append({'name':name,'part':key,'volume':round(volume,4),'estimated':estimated})
    for ref in project.get('references',[]):
        if not ref.get('fitCheck',False):continue
        try:
            from references import placed_shapes
            shape,estimated=placed_shapes(project,ref,met)
            for key,body in list(bodies.items())+[('PCB assembly',cq.Compound.makeCompound([s for _,s,_ in checks]))]:
                a,b=shape.BoundingBox(),body.BoundingBox()
                if a.xmax<b.xmin or a.xmin>b.xmax or a.ymax<b.ymin or a.ymin>b.ymax or a.zmax<b.zmin or a.zmin>b.zmax:continue
                volume=shape.intersect(body).Volume()
                if volume>.005:collisions.append({'name':ref['name'],'part':key,'volume':round(volume,4),'estimated':estimated or (key=='PCB assembly' and project['board'].get('source')!='step')})
        except Exception as exc:errors.append(f"Hardware {ref['name']}: {exc}")
    overlap=bodies['shell'].intersect(bodies['lid']).Volume()
    if overlap>.001:errors.append('The shell and seated lid overlap. Reduce the lip or revise the features.')
    renders=[{'id':key,'kind':key,**mesh_data(body)} for key,body in bodies.items()]
    if project['board'].get('source')=='step':
        from step_import import render_board
        renders.extend(render_board(project['board']))
    else:
        renders.append({'id':'pcb','kind':'pcb',**mesh_data(board)})
        for c in project['board'].get('components',[]):
            if c.get('height',0)>.01:renders.append({'id':c['id'],'name':c['ref'],'kind':'component',**mesh_data(component_shape(project['board'],c))})
    if reference_loader:
        for ref in project.get('references',[]):
            try:
                for m in reference_loader(ref,reference_transform(project,ref,met)):renders.append(m)
            except Exception as exc:errors.append(f"Reference {ref['name']}: {exc}")
    report={'metrics':met,'features':feature_results,'planes':planes,'errors':errors,'warnings':warnings,'collisions':collisions,'volumes':{k:round(s.Volume(),2) for k,s in bodies.items()},'valid':not errors,'meshes':renders,'kernel':'Open CASCADE / CadQuery'}
    return report,bodies,tools

def new_project(board,name=None):
    return {'version':1,'name':name or Path(board['name']).stem,'board':board,'enclosure':dict(DEFAULTS),'features':[],'planes':[],'references':[]}
