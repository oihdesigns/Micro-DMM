"""Run with FreeCAD's bundled Python. All dimensions in mm."""
from pathlib import Path
import json
import FreeCAD as App
import Part
import MeshPart

OUT = Path(__file__).resolve().parent
# Change SOCKET_D after testing the coupon. These are individual blind sockets,
# not replicas of LEGO underside tubes. Pitch is preserved at 8 mm.
SOCKET_D = 4.90  # User confirmed Fit_Test_4p90 fits best.
SOCKET_DEPTH = 2.10
SCREW_D = 2.40
CLEARANCE_D = 22.50
V = App.Vector

def sockets(shape, nx, ny, width, depth, diameter):
    for i in range(nx):
        for j in range(ny):
            x = -width/2 + 3.9 + i*8
            y = 3.9 + j*8
            # Small lead-in to avoid a sharp press-fit entrance.
            cut = Part.makeCylinder(diameter/2, SOCKET_DEPTH+0.02, V(x,y,-0.02))
            cut = cut.fuse(Part.makeCone(diameter/2+0.20, diameter/2, 0.30, V(x,y,0)))
            shape = shape.cut(cut)
    return shape.removeSplitter()

def bracket():
    shape = Part.makeBox(47.8,47.8,5,V(-23.9,0,0))
    shape = shape.fuse(Part.makeBox(47.8,4,36,V(-23.9,0,5)))
    for x in [-22.9,19.9]:
        wire = Part.makePolygon([V(x,3.5,4.5),V(x,36,4.5),V(x,3.5,35),V(x,3.5,4.5)])
        shape = shape.fuse(Part.Face(wire).extrude(V(3,0,0)))
    shape = shape.cut(Part.makeCylinder(CLEARANCE_D/2,6,V(0,-1,24),V(0,1,0)))
    for x in [-11.75,11.75]:
        for z in [12.25,35.75]:
            shape = shape.cut(Part.makeCylinder(SCREW_D/2,6,V(x,-1,z),V(0,1,0)))
    return sockets(shape,6,6,47.8,47.8,SOCKET_D)

def export(shape, name):
    assert shape.isValid(), name + ': invalid BREP'
    assert len(shape.Solids)==1, name + ': expected one connected solid'
    doc = App.newDocument(name)
    obj = doc.addObject('PartDesign::Feature',name)
    obj.Shape = shape
    doc.recompute()
    doc.saveAs(str(OUT/(name+'.FCStd')))
    Part.export([obj],str(OUT/(name+'.step')))
    mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.05, AngularDeflection=0.15, Relative=False)
    mesh.write(str(OUT/(name+'.stl')))
    result = {'valid':shape.isValid(),'solids':len(shape.Solids),'volume_mm3':shape.Volume,'triangles':mesh.CountFacets,'closed_mesh':mesh.isSolid()}
    assert result['closed_mesh'], name + ': mesh is not closed'
    App.closeDocument(doc.Name)
    return result

results = {'mount':export(bracket(),'Stepper_LEGO_Mount')}
# Separate 2 x 2 coupons, each identified by filename.
for d in [4.70,4.80,4.90,5.00,5.10,5.20]:
    coupon = sockets(Part.makeBox(15.8,15.8,5,V(-7.9,0,0)),2,2,15.8,15.8,d)
    name = 'Fit_Test_' + ('%.2f'%d).replace('.','p')
    results[name] = export(coupon,name)
(OUT/'validation.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
