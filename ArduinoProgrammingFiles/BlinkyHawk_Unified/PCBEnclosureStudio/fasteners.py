"""Paired lid clearance holes and blind, wall-supported screw bosses."""
import math
import numpy as np
import cadquery as cq
from shapely.geometry import Point
from shapely.ops import nearest_points
from geometry import numeric,footprint,poly_prism

DEFAULTS={'pattern':'four','insetX':3.5,'insetY':3.5,'diameter':6.,'pilot':1.8,'clearance':2.4,
          'pilotDepth':4.,'blindFloor':1.2,'supportAngle':45.,'headStyle':'none','headDiameter':4.4,'headDepth':.8,'points':[]}

def apply_fasteners(project,met,bodies,feature):
    d=DEFAULTS|feature.get('dimensions',{});c=project['enclosure']
    radius=numeric(d['diameter'],'Boss diameter',2,40)/2
    pilot=numeric(d['pilot'],'Pilot diameter',.5,2*radius-1)/2
    clearance=numeric(d['clearance'],'Lid clearance diameter',2*pilot,2*radius-.6)/2
    depth=numeric(d['pilotDepth'],'Blind pilot depth',.5,100)
    cap=numeric(d['blindFloor'],'Material below the pilot',.6,20)
    angle=numeric(d['supportAngle'],'Support angle from vertical',20,60)
    pattern=d['pattern'];x=numeric(d['insetX'],'X inset',.5,met['width']/2-.1);y=numeric(d['insetY'],'Y inset',.5,met['length']/2-.1)
    xs=[met['xmin']+x,met['xmax']-x];ys=[met['ymin']+y,met['ymax']-y]
    layouts={'four':[(a,b) for a in xs for b in ys],'front':[(a,ys[0]) for a in xs],
             'back':[(a,ys[1]) for a in xs],'left':[(xs[0],b) for b in ys],'right':[(xs[1],b) for b in ys]}
    if pattern=='custom':
        points=d['points']
        if not isinstance(points,list) or not 1<=len(points)<=20:raise ValueError('Enter 1–20 custom screw centers in world X, Y.')
        for xy in points:
            if len(xy)!=2:raise ValueError('Each screw center needs X and Y.')
            for v in xy:numeric(v,'Screw center',-10000,10000)
    elif pattern in layouts:points=layouts[pattern]
    else:raise ValueError('Unknown lid screw pattern.')
    for i,a in enumerate(points):
        if any(np.linalg.norm(np.asarray(a)-b)<2*radius+.2 for b in points[:i]):raise ValueError('The screw supports overlap. Reduce their diameter or spread the centers apart.')
    outer=footprint(project,met);inner=footprint(project,met,c['wall'])
    envelope=poly_prism(outer,met['floor'],met['top']-met['floor'])
    shell=bodies['shell'];lid=bodies['lid'];centers=[]
    for x,y in points:
        point=Point(x,y)
        if not inner.contains(point):raise ValueError('A screw center is outside the cavity. Increase the pattern inset or use custom centers.')
        if not outer.contains(point.buffer(pilot+.6)):raise ValueError('A pilot hole would break through a side wall. Move its center inward.')
        wall=np.array(nearest_points(inner.boundary,point)[0].coords[0]);axis=np.array([x,y])-wall;distance=np.linalg.norm(axis)
        if distance>=radius-.3:raise ValueError('A screw boss misses the wall. Reduce its inset or increase its diameter.')
        axis/=distance;reach=distance+radius;support_height=reach/math.tan(math.radians(angle))
        barrel_bottom=met['top']-depth-cap;bottom=barrel_bottom-support_height
        if bottom<=met['floor']+.2:raise ValueError('A tapered support reaches the floor. Reduce pilot depth or boss diameter, or increase enclosure height.')
        # The ramp starts on the wall, not underneath the screw center: every printed layer joins the wall.
        plane=cq.Plane((float(wall[0]),float(wall[1]),barrel_bottom),(float(axis[0]),float(axis[1]),0),(float(axis[1]),float(-axis[0]),0))
        ramp=cq.Workplane(plane).polyline([(-c['wall'],-support_height),(0,-support_height),(reach,0),(-c['wall'],0)]).close().extrude(radius+.1,both=True).val()
        lower=cq.Solid.makeCylinder(radius,support_height,(x,y,bottom)).intersect(ramp)
        barrel=cq.Solid.makeCylinder(radius,depth+cap,(x,y,barrel_bottom))
        boss=barrel.fuse(lower).intersect(envelope).clean()
        if shell.intersect(boss).Volume()<.01:raise ValueError('The support does not join the shell at this location.')
        shell=shell.fuse(boss).cut(cq.Solid.makeCylinder(pilot,depth+.02,(x,y,met['top']-depth))).clean()
        # Relieve only the locating lip, leaving the lid plate seated on the boss.
        if c['lip']>0:
            lip_relief=cq.Solid.makeCylinder(radius+c['lipClearance'],c['lip']+.02,(x,y,met['top']-c['lip']-.02))
            lid=lid.cut(lip_relief)
        lid=lid.cut(cq.Solid.makeCylinder(clearance,c['lid']+c['lip']+.2,(x,y,met['top']-c['lip']-.1)))
        style=d['headStyle']
        if style not in ['none','counterbore','countersink']:raise ValueError('Unknown screw head recess.')
        if style!='none':
            head=numeric(d['headDiameter'],'Head recess diameter',clearance*2+.1,radius*2-.2)/2
            h=(head-clearance) if style=='countersink' else numeric(d['headDepth'],'Head recess depth',.1,100)
            if h>c['lid']-.6:raise ValueError('The head recess leaves less than 0.6 mm of lid. Reduce it or thicken the lid.')
            z=met['top']+c['lid']-h
            cutter=cq.Solid.makeCylinder(head,h+.02,(x,y,z)) if style=='counterbore' else cq.Solid.makeCone(clearance,head+.01,h+.01,(x,y,z))
            lid=lid.cut(cutter)
        centers.append([x,y,met['top']])
    result={'shell':shell.clean(),'lid':lid.clean()}
    for part in result.values():
        if not part.isValid() or len(part.Solids())!=1:raise ValueError('The screw set leaves a disconnected or invalid part. Adjust the pattern or lip dimensions.')
    return result,centers
