import copy, io, json, math, zipfile
from pathlib import Path
import cadquery as cq
import numpy as np
import pytest
import trimesh
from fastapi.testclient import TestClient
from geometry import new_project, build, attach_frame, metrics, component_frame, validate_project
from kicad_import import import_board
from references import import_reference, original_meshes, render_reference
from server import app, export_shape

@pytest.fixture
def project():
    return new_project({'name':'test.kicad_pcb','width':30,'length':50,'thickness':1.6,
        'outline':[[0,0],[30,0],[30,50],[0,50]],'cutouts':[],
        'holes':[{'id':'mount','xy':[5,5],'diameter':2.5,'mounting':True,'source':'H1'}],
        'components':[{'id':'usb','ref':'J1','xy':[15,2],'rotation':0,'side':'top','center':[0,0],
                       'width':8,'depth':5,'height':3,'pads':[{'number':'1','xy':[14,3]}]}]})

def feature(kind='window',**kwargs):
    f={'id':'feature','name':kind,'type':kind,'operation':'cut','target':'shell',
       'anchor':{'kind':'case','point':'front'},'offset':[0,0,0],'rotation':[0,0,0],
       'extent':'symmetric','dimensions':{'width':8,'height':4,'radius':1,'depth':10}}
    f.update(kwargs);return f

def test_actual_blinkyhawk_import_and_export():
    board=import_board((Path(__file__).parents[1]/'examples/BlinkyHawk_V3b.kicad_pcb').read_text(encoding='utf8'),'BlinkyHawk_V3b.kicad_pcb')
    assert board['width']==pytest.approx(25.4)
    assert board['length']==pytest.approx(53.34)
    assert len(board['components'])==40
    result,bodies,_=build(new_project(board))
    assert result['valid'],result['errors']
    for body in bodies.values():
        assert body.isValid() and len(body.Solids())==1
        mesh=trimesh.load(io.BytesIO(export_shape(body,'stl')),file_type='stl')
        assert mesh.is_watertight and mesh.volume>0
        assert mesh.volume==pytest.approx(body.Volume(),rel=.002)

def test_component_attachment_follows_location_and_rotation(project):
    f=feature(anchor={'kind':'component','ref':'usb','point':'center','project':'front','plane':'XZ'})
    f['offset']=[2,1,0];project['features']=[f]
    r,_,_=build(project);assert r['features'][0]['status']=='ok'
    p=np.array(r['features'][0]['origin'])
    project['board']['components'][0]['xy'][0]+=7
    r,_,_=build(project)
    assert np.allclose(np.array(r['features'][0]['origin'])-p,[7,0,0])
    # A local side frame rotates with its footprint.
    c=project['board']['components'][0];c['rotation']=90
    p,r=component_frame(project['board'],c,'right')
    assert np.allclose(r[:,2],[0,1,0],atol=1e-9)
    assert np.allclose(p[:2],[22,6])

def test_features_produce_connected_solids(project):
    base,before,_=build(copy.deepcopy(project))
    project['features']=[feature(),feature('standoff',id='post',operation='add',anchor={'kind':'hole','ref':'mount'},dimensions={'diameter':6,'bore':2.3,'depth':3}),
        feature('vent',id='vents',target='lid',anchor={'kind':'case','point':'lid'},dimensions={'width':12,'height':1.5,'pitch':4,'count':3,'depth':8})]
    r,parts,_=build(project)
    assert r['valid'],r['errors']
    assert all(f['status']=='ok' for f in r['features'])
    assert parts['lid'].Volume()<before['lid'].Volume()
    for part in parts.values():assert part.isValid() and len(part.Solids())==1

@pytest.mark.parametrize('kind,dimensions',[
    ('hole',{'diameter':3,'depth':10}),
    ('sketch',{'points':[[-3,-2],[3,-2],[5,0],[3,2],[-3,2]],'depth':10}),
    ('box',{'width':8,'height':4,'depth':10,'radius':0}),
    ('text',{'text':'PCB','height':4,'depth':.8}),
])
def test_other_feature_profiles(project,kind,dimensions):
    f=feature(kind,dimensions=dimensions)
    if kind=='text':f.update(operation='add',target='lid',anchor={'kind':'case','point':'lid'},offset=[0,0,-.1])
    project['features']=[f]
    r,_,_=build(project)
    assert r['valid'] and r['features'][0]['status']=='ok',r['errors']

def test_plane_chain_and_cycle(project):
    project['planes']=[{'id':'a','name':'A','anchor':{'kind':'component','ref':'usb','point':'top'},'offset':[0,0,2]},
                       {'id':'b','name':'B','anchor':{'kind':'plane','ref':'a'},'offset':[1,0,3]}]
    p,r=attach_frame(project,{'anchor':{'kind':'plane','ref':'b'}},metrics(project))
    assert np.allclose(p,[16,2,9.6])
    project['planes'][0]['anchor']={'kind':'plane','ref':'b'}
    with pytest.raises(ValueError,match='circular'):attach_frame(project,{'anchor':{'kind':'plane','ref':'b'}},metrics(project))

def test_failed_feature_blocks_export_and_suppression_recovers(project):
    project['features']=[feature('box',operation='add',offset=[0,0,100])]
    client=TestClient(app)
    r=client.post('/api/build',json=project)
    assert r.status_code==200 and not r.json()['valid']
    assert client.post('/api/export/stl/shell',json=project).status_code==422
    project['features'][0]['suppressed']=True
    assert client.post('/api/export/stl/shell',json=project).status_code==200

def test_cut_miss_warning(project):
    project['features']=[feature(offset=[100,0,0])]
    r,_,_=build(project)
    assert r['valid'] and r['warnings'] and r['features'][0]['status']=='warning'

def test_step_face_attachment_and_portable_roundtrip(project,tmp_path):
    step=tmp_path/'connector.step';cq.exporters.export(cq.Workplane('XY').box(8,5,3),str(step))
    ref=import_reference(step.read_bytes(),step.name);project['references']=[ref]
    faces=original_meshes(ref['asset'])[0]['faces'];assert len(faces)==6
    f=next(f for f in faces if f['normal'][2]>.9)
    anchor={'anchor':{'kind':'face','ref':ref['id'],'face':f}}
    p,_=attach_frame(project,anchor,metrics(project));assert np.allclose(p,[0,0,1.5])
    ref['offset']=[10,20,5];ref['rotation']=[0,90,0]
    p,r=attach_frame(project,anchor,metrics(project));assert np.allclose(p,[11.5,20,5])
    assert np.allclose(r[:,2],[1,0,0],atol=1e-9)
    client=TestClient(app)
    saved=client.post('/api/project/save',json=project)
    assert saved.status_code==200
    opened=client.post('/api/project/open',files={'file':('test.pcbshell',saved.content)})
    assert opened.status_code==200 and opened.json()==project
    assert client.post('/api/build',json=opened.json()).json()['valid']

@pytest.mark.parametrize('fmt',['3mf','step','stl'])
def test_pair_export(project,fmt):
    response=TestClient(app).post(f'/api/export/{fmt}/pair',json=project)
    assert response.status_code==200,response.text[:300]
    with zipfile.ZipFile(io.BytesIO(response.content)) as z:
        names=[n for n in z.namelist() if n.endswith('.'+fmt)]
        assert len(names)==2
        for n in names:
            data=z.read(n)
            if fmt=='step':assert b'ISO-10303-21' in data
            else:
                scene=trimesh.load_scene(io.BytesIO(data),file_type=fmt)
                mesh=scene.to_mesh()
                assert mesh.is_watertight
                assert mesh.bounds[0][2]==pytest.approx(0,abs=1e-5)

def test_import_outline_validation_and_all_pad_bounds():
    source='''(kicad_pcb (general (thickness 1.6))
      (gr_rect (start 10 20) (end 40 70) (layer "Edge.Cuts"))
      (footprint "Test" (layer "F.Cu") (at 15 25 90) (uuid "stable-id")
       (property "Reference" "J1")
       (pad "1" thru_hole circle (at -2 0 90) (size 2 2) (drill 1))
       (pad "2" thru_hole circle (at 2 0 90) (size 2 2) (drill 1))))'''
    b=import_board(source,'test.kicad_pcb');c=b['components'][0]
    assert c['width']==6 and c['depth']==2
    assert c['pads'][0]['xy']==[5,43] and c['pads'][1]['xy']==[5,47]
    with pytest.raises(ValueError,match='open contour'):
        import_board('(kicad_pcb (gr_line (start 0 0) (end 20 0) (layer "Edge.Cuts")))','open.kicad_pcb')

def test_outline_shell_and_collision_check(project):
    project['enclosure']['shape']='outline'
    report,_,_=build(project);assert report['valid']
    project['board']['components'][0]['height']=25;project['enclosure']['autoHeight']=False
    report,_,_=build(project)
    assert any(x['name']=='J1' and x['part']=='lid' for x in report['collisions'])

def test_local_origin_and_corrupt_project(project):
    client=TestClient(app)
    assert client.post('/api/build',json=project,headers={'Origin':'https://example.com'}).status_code==403
    assert client.post('/api/project/open',files={'file':('bad.pcbshell',b'not a zip')}).status_code==422

def test_slotted_holes_match_kicad_and_pad_moves():
    source='''(kicad_pcb (general (thickness 1.6))
      (gr_rect (start 10 20) (end 40 70) (layer "Edge.Cuts"))
      (footprint "Test" (layer "F.Cu") (at 15 25 45) (uuid "stable-id")
       (property "Reference" "J1")
       (pad "1" thru_hole oval (at 0 0 45) (size 4 2) (drill oval 3 1 (offset 0.5 0)))))'''
    board=import_board(source,'slots.kicad_pcb');hole=board['holes'][0]
    assert hole['xy']==[5,45] and hole['diameter']==1 and hole['slotLength']==3 and hole['angle']==45
    project=new_project(board);f={'anchor':{'kind':'pad','ref':'stable-id','pad':'1'}}
    p,_=attach_frame(project,f,metrics(project));assert np.allclose(p,[5,45,1.6])
    board['components'][0]['xy'][0]+=7
    p,_=attach_frame(project,f,metrics(project));assert np.allclose(p,[12,45,1.6])

@pytest.mark.parametrize('fmt',['3mf','stl'])
def test_mesh_reference_import(project,fmt):
    mesh=trimesh.creation.box([5,10,2])
    blob=trimesh.exchange.threemf.export_3MF(trimesh.Scene(mesh)) if fmt=='3mf' else mesh.export(file_type='stl')
    ref=import_reference(blob,'reference.'+fmt);project['references']=[ref]
    assert ref['planarFaces']==0
    assert np.allclose(np.array(ref['bounds'])[1]-np.array(ref['bounds'])[0],[5,10,2])
    assert build(project,render_reference)[0]['valid']
