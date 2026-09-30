import copy,io,zipfile
import cadquery as cq
import numpy as np
import pytest
import trimesh
from fastapi.testclient import TestClient
from geometry import new_project,build,metrics,attach_frame,reference_transform
from references import import_reference,original_meshes,render_reference,reserve_space,placed_shapes,asset_path
from server import app,export_shape

@pytest.fixture
def project():
    return new_project({'name':'Hardware enclosure','outline':[[0,0],[30,0],[30,50],[0,50]],
        'width':30,'length':50,'thickness':1.6,'components':[],'holes':[],'cutouts':[],'warnings':[]})

def screw_set(**dims):return {'id':'screws','name':'Lid screws','type':'lidScrews','dimensions':dims}

@pytest.fixture
def switch(tmp_path):
    # Illustrative hardware: shoulder at Z=0, internal body below it, shaft above it.
    model=cq.Assembly();model.add(cq.Workplane('XY').box(8,10,7).translate((0,0,-3.5)),name='Internal body')
    model.add(cq.Workplane('XY').circle(3).extrude(8),name='Mounting shaft')
    model.add(cq.Workplane('XY').workplane(offset=8).circle(1).extrude(7),name='Lever')
    path=tmp_path/'Illustrative_switch.step';model.export(str(path))
    return import_reference(path.read_bytes(),path.name)

@pytest.mark.parametrize('style',['rectangle','outline'])
def test_asymmetric_space_preserves_board_and_wall_thickness(project,style):
    project['enclosure'].update(shape=style,extraFront=17,extraRight=8)
    report,bodies,_=build(project);m=report['metrics'];assert report['valid']
    assert m['xmin']==pytest.approx(-2.6);assert m['ymin']==pytest.approx(-19.6)
    assert m['xmax']==pytest.approx(40.6);assert m['ymax']==pytest.approx(52.6)
    shell=bodies['shell'];assert shell.isInside((15,-18.6,3)) and not shell.isInside((15,-16.6,3))
    assert shell.isInside((39.6,25,3)) and not shell.isInside((37.6,25,3))
    substrate=next(m for m in report['meshes'] if m['kind']=='pcb');assert np.min(substrate['vertices'],axis=0)==pytest.approx([0,0,0])

def test_paired_holes_blind_bottom_and_printable_taper(project):
    project['features']=[screw_set()];report,bodies,_=build(project)
    assert report['valid'],report['errors'];m=report['metrics'];shell,lid=bodies['shell'],bodies['lid']
    assert len(report['features'][0]['centers'])==4
    for x,y,z in report['features'][0]['centers']:
        assert not shell.isInside((x,y,z-2))                 # pilot hole
        assert shell.isInside((x,y,z-4.5))                  # closed beneath pilot
        assert not shell.isInside((x,y,m['floor']+1))       # no full-height post
        assert not lid.isInside((x,y,z+1))                 # coaxial lid hole
        assert lid.isInside((x+1.5,y,z+1))                 # lid remains around it
    x,y,z=report['features'][0]['centers'][0]
    # The nearest wall at this corner is x=-0.6 or y=-0.6. Points close to the wall
    # become supported lower down than points at the free edge of the boss.
    assert shell.isInside((x,y,z-7))
    assert not shell.isInside((x+1.8,y+1.8,z-7))
    assert shell.intersect(lid).Volume()<1e-6
    for body in bodies.values():
        mesh=trimesh.load(io.BytesIO(export_shape(body,'stl')),file_type='stl')
        assert mesh.is_watertight and len(body.Solids())==1
    project['enclosure']['extraFront']=12
    next_report,_,_=build(project);assert next_report['valid']
    old=report['features'][0]['centers'];new=next_report['features'][0]['centers']
    assert new[0][1]==pytest.approx(old[0][1]-12);assert new[1][1]==pytest.approx(old[1][1])

@pytest.mark.parametrize('style',['counterbore','countersink'])
def test_head_recess_and_suppression(project,style):
    project['features']=[screw_set(pattern='front',headStyle=style)]
    report,bodies,_=build(project);assert report['valid'],report['errors']
    x,y,z=report['features'][0]['centers'][0]
    assert not bodies['lid'].isInside((x+1.7,y,z+1.9))
    assert bodies['lid'].isInside((x+1.7,y,z+.3))
    project['features'][0]['suppressed']=True
    _,suppressed,_=build(project);project['features']=[];_,base,_=build(project)
    for k in base:assert suppressed[k].Volume()==pytest.approx(base[k].Volume())

@pytest.mark.parametrize('dims,reason',[
    ({'pilotDepth':20},'floor'),({'insetX':8,'insetY':8},'misses the wall'),
    ({'headStyle':'counterbore','headDepth':2},'lid'),
    ({'pattern':'custom','points':[[.9,.9],[.9,.9]]},'overlap'),
])
def test_bad_screws_fail_atomically_and_block_exports(project,dims,reason):
    _,base,_=build(copy.deepcopy(project));project['features']=[screw_set(**dims)]
    report,bodies,_=build(project);assert not report['valid'] and reason in report['errors'][0]
    for k in base:assert bodies[k].Volume()==pytest.approx(base[k].Volume())
    assert TestClient(app).post('/api/export/stl/pair',json=project).status_code==422

def test_hardware_space_mount_hole_and_portable_project(project,switch):
    project['references']=[switch];switch.update(offset=[15,-8,4],rotation=[90,0,0])
    # Reserve the internal body, excluding the external shaft and lever.
    result=reserve_space(project,switch['id'],body=0,margin=1)
    assert result['enclosure']['extraFront']>8
    assert result['enclosure']['extraBack']==0
    assert result['enclosure']['above']==pytest.approx(8.4)
    face=next(f for f in original_meshes(switch['asset'])[0]['faces'] if f.get('normal',[0,0,0])[2]>.99)
    switch.update(mount={'anchor':{'kind':'case','point':'front','surface':'inside'}},mountFace={**face,'body':0},offset=[0,0,0],rotation=[0,0,0])
    with pytest.raises(ValueError,match='World coordinates'):reserve_space(project,switch['id'])
    report,_,_=build(project,render_reference)
    assert any(c['name']==switch['name'] and c['part']=='shell' and not c['estimated'] for c in report['collisions'])
    hole={'id':'switch-hole','name':'Switch hole','type':'hole','operation':'cut','target':'shell','extent':'symmetric',
        'anchor':{'kind':'face','ref':switch['id'],'face':{**face,'body':0}},'dimensions':{'diameter':6.6,'depth':6}}
    project['features']=[hole];report,bodies,_=build(project,render_reference)
    assert report['valid'],report['errors'];assert not report['collisions'],report['collisions']
    p,r=attach_frame(project,hole,metrics(project));assert np.allclose(r[:,2],[0,-1,0])
    project['enclosure']['extraFront']+=9
    q,_=attach_frame(project,hole,metrics(project));assert q-p==pytest.approx([0,-9,0])
    switch['offset'][0]=3;q2,_=attach_frame(project,hole,metrics(project));assert q2-q==pytest.approx([3,0,0])
    client=TestClient(app);saved=client.post('/api/project/save',json=project);assert saved.status_code==200
    reopened=client.post('/api/project/open',files={'file':('hardware.pcbshell',saved.content)}).json();assert reopened==project
    rebuilt=client.post('/api/build',json=reopened).json();assert rebuilt['valid'] and not rebuilt['collisions']
    assert client.post('/api/export/3mf/pair',json=reopened).status_code==200

def test_mounted_hardware_plane_cycle_is_reported(project,switch):
    project['references']=[switch];switch['mount']={'anchor':{'kind':'plane','ref':'plane'}}
    project['planes']=[{'id':'plane','name':'Circular plane','anchor':{'kind':'reference','ref':switch['id']}}]
    report,_,_=build(project,render_reference);assert not report['valid'] and any('circular' in e for e in report['errors'])

def test_rotated_hardware_anchor_follows_model(project,switch):
    project['references']=[switch];switch.update(offset=[10,20,5],rotation=[0,90,30])
    spec={'anchor':{'kind':'reference','ref':switch['id'],'point':'origin'}}
    p,r=attach_frame(project,spec,metrics(project));assert p==pytest.approx([10,20,5])
    shape,estimated=placed_shapes(project,switch,metrics(project));assert not estimated
    model=render_reference(switch);v=np.vstack([m['vertices'] for m in model]);b=shape.BoundingBox()
    assert v.min(0)==pytest.approx([b.xmin,b.ymin,b.zmin],abs=.1)
    assert v.max(0)==pytest.approx([b.xmax,b.ymax,b.zmax],abs=.1)

def test_hardware_import_and_space_api(project,switch):
    client=TestClient(app)
    response=client.post('/api/import/reference',files={'file':('switch.step',asset_path(switch['asset']).read_bytes())})
    assert response.status_code==200
    ref=response.json();assert len(ref['bodies'])==3 and ref['fitCheck']
    ref['offset']=[15,-15,8];project['references']=[ref]
    result=client.post('/api/reference/space',json={'project':project,'refId':ref['id'],'body':'0','margin':1})
    assert result.status_code==200 and result.json()['enclosure']['extraFront']>15
    assert project['enclosure']['extraFront']==0  # Endpoint returns a proposed size; caller applies it.
