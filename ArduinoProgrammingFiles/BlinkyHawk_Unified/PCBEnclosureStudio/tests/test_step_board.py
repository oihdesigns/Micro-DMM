import io,json,zipfile
from pathlib import Path
import cadquery as cq
import numpy as np
import pytest
from fastapi.testclient import TestClient
from geometry import build,attach_frame,metrics
from server import app
from step_import import import_step_board,from_asset,board_meshes,source_bodies,board_shapes

@pytest.fixture(scope='module')
def step_file(tmp_path_factory):
    dest=tmp_path_factory.mktemp('step')/'board.step'
    pcb=cq.Workplane('XY').rect(20,30).extrude(1.6).faces('>Z').workplane().center(5,5).hole(2.5)
    usb=cq.Workplane('XY').box(8,6,3).translate((0,14,3.1)).cut(cq.Workplane('XY').box(6,5,1.6).translate((0,15,3.1)))
    assy=cq.Assembly(name='PCB assembly',loc=cq.Location((100,-200,5)))
    assy.add(pcb,name='Main PCB',color=cq.Color('green'));assy.add(usb,name='USB-C',color=cq.Color(.7,.7,.7))
    assy.export(str(dest));return dest

def test_standalone_step_import_retains_solids_and_connector(step_file):
    p=import_step_board(step_file.read_bytes(),step_file.name);b=p['board']
    assert b['source']=='step' and b['bodyCount']==2
    assert b['thickness']==pytest.approx(1.6) and b['width']==pytest.approx(20) and b['length']==pytest.approx(30)
    assert len(b['holes'])==1 and b['holes'][0]['diameter']==pytest.approx(2.5)
    usb=next(c for c in b['components'] if c['ref']=='USB-C');assert usb['zMin']==pytest.approx(1.6)
    report,_,_=build(p)
    assert report['valid'] and any(c['name']=='USB-C' and not c['estimated'] for c in report['collisions'])
    assert len(report['meshes'])==4
    exact=next(m for m in report['meshes'] if m.get('componentId')==usb['id'])
    assert exact['faces'] and exact['boardAsset']==b['asset'] and exact['color']
    # The modeled opening is retained: the imported component volume is not its bounding box.
    shape=board_shapes(b)[usb['bodies'][0]]['shape']
    assert shape.Volume()<usb['width']*usb['depth']*usb['height']

def test_step_planar_face_cut_tracks_board_frame(step_file):
    p=import_step_board(step_file.read_bytes(),step_file.name);b=p['board'];usb=b['components'][0]
    body=usb['bodies'][0];mesh=board_meshes(b['asset'],b['substrate'])[body]
    face=next(f for f in mesh['faces'] if f['type']=='PLANE' and f['normal'][1]>.9)
    a={'kind':'boardFace','ref':b['asset'],'body':body,'face':{'id':face['id']},'project':'back','plane':'-XZ'}
    f={'id':'usb-opening','name':'USB opening','type':'window','operation':'cut','target':'shell','anchor':a,
       'dimensions':{'width':10,'height':5,'depth':12,'radius':.6},'offset':[0,0,0],'rotation':[0,0,0]}
    p['features']=[f];report,_,_=build(p)
    assert report['valid'] and report['features'][0]['status']=='ok'
    assert not report['collisions']
    unprojected={'anchor':dict(a,project='none',plane='face')}
    old,axes=attach_frame(p,unprojected,metrics(p))
    p['board']=from_asset(b['asset'],b['name'],b['substrate'],True)['board']
    new,axes2=attach_frame(p,unprojected,metrics(p))
    assert not np.allclose(old,new)
    p['features'][0]['anchor']['ref']='different-file.step'
    assert 'replaced' in build(p)[0]['errors'][0]

def test_step_portable_save_open_and_reimport_selection(step_file):
    client=TestClient(app)
    imported=client.post('/api/import/board',files={'file':('board.STEP',step_file.read_bytes())})
    assert imported.status_code==200,imported.text
    p=imported.json();saved=client.post('/api/project/save',json=p)
    with zipfile.ZipFile(io.BytesIO(saved.content)) as z:assert 'assets/'+p['board']['asset'] in z.namelist()
    reopened=client.post('/api/project/open',files={'file':('board.pcbshell',saved.content)})
    assert reopened.status_code==200 and reopened.json()==p
    assert client.post('/api/build',json=reopened.json()).json()['valid']
    selected=client.post('/api/board/substrate',json={'asset':p['board']['asset'],'name':'board.step','substrate':p['board']['substrate'],'flip':True})
    assert selected.status_code==200 and selected.json()['board']['flip']
    assert client.post('/api/import/board',files={'file':('broken.step',b'not STEP')}).status_code==422

def test_single_solid_step_is_valid_board(tmp_path):
    path=tmp_path/'single.stp';cq.exporters.export(cq.Workplane('XY').box(20,30,1.6),str(path),exportType='STEP')
    p=import_step_board(path.read_bytes(),path.name)
    assert not p['board']['components'] and p['board']['thickness']==pytest.approx(1.6)
    assert build(p)[0]['valid']

def test_vertical_pcb_normalizes_to_xy(tmp_path):
    path=tmp_path/'vertical.step'
    a=cq.Assembly(name='Vertical PCB',loc=cq.Location((100,200,300),(1,0,0),90))
    a.add(cq.Workplane('XY').rect(20,30).extrude(1.6),name='PCB')
    a.add(cq.Workplane('XY').box(8,5,3).translate((0,0,3.1)),name='Connector')
    a.export(str(path));p=import_step_board(path.read_bytes(),path.name);b=p['board']
    assert sorted([b['width'],b['length']])==pytest.approx([20,30])
    assert b['thickness']==pytest.approx(1.6)
    assert np.linalg.det(np.array(b['transform']['rotation']))==pytest.approx(1)
    assert build(p)[0]['valid']

@pytest.mark.skipif(not Path(r'C:\Users\Nick\Documents\GitHub\Micro-DMM\PCBDesigns\BlinkyHawk_V3b\BlinkyHawkModel_v3b.step').exists(),reason='Local V3b STEP fixture is not present')
def test_actual_blinkyhawk_step_contains_usb_and_exact_board():
    path=Path(r'C:\Users\Nick\Documents\GitHub\Micro-DMM\PCBDesigns\BlinkyHawk_V3b\BlinkyHawkModel_v3b.step')
    p=import_step_board(path.read_bytes(),path.name);b=p['board']
    assert b['bodyCount']==301 and len(b['components'])==41
    assert b['width']==pytest.approx(25.4) and b['length']==pytest.approx(53.34)
    assert b['thickness']==pytest.approx(1.51)
    usb=next(c for c in b['components'] if c['ref']=='USB TYPE C PORT')
    assert len(usb['bodies'])==24
    report,_,_=build(p)
    assert report['valid'] and len(report['meshes'])==303
    assert any(c['name']=='USB TYPE C PORT' and not c['estimated'] for c in report['collisions'])
    # A projected opening clears the actual USB body, not just its courtyard.
    p['features']=[{'id':'usb','name':'USB window','type':'window','target':'shell','operation':'cut',
        'anchor':{'kind':'component','ref':usb['id'],'point':'center','project':'back','plane':'-XZ'},
        'dimensions':{'width':12,'height':6,'depth':12,'radius':1},'offset':[0,0,0],'rotation':[0,0,0]}]
    result,_,_=build(p)
    assert result['valid'] and not result['collisions']
