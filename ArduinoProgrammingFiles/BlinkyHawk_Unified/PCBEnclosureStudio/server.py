import bootstrap
import io,json,time,zipfile,hashlib,threading,uuid,copy,re,os
from pathlib import Path
from functools import lru_cache
import cadquery as cq
import trimesh
import numpy as np
from fastapi import FastAPI,UploadFile,HTTPException,Request
from fastapi.responses import FileResponse,Response,JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from starlette.concurrency import run_in_threadpool
from geometry import build,new_project,validate_project
from kicad_import import import_board
from references import import_reference,render_reference,asset_path,store_asset,reserve_space
from step_import import import_step_board,from_asset

ROOT=Path(__file__).resolve().parent
DATA=ROOT/'data';DATA.mkdir(exist_ok=True)
app=FastAPI(title='PCB Enclosure Studio',docs_url=None,redoc_url=None)
app.add_middleware(TrustedHostMiddleware,allowed_hosts=['127.0.0.1','localhost','testserver'])
LOCK=threading.RLock()
MAX_UPLOAD=64*1024*1024

@app.middleware('http')
async def local_only(request:Request,call_next):
    # Cross-site browser pages cannot send state-changing requests to the local editor.
    origin=request.headers.get('origin')
    if request.method not in ['GET','HEAD','OPTIONS'] and origin and origin!=str(request.base_url).rstrip('/'):
        return JSONResponse({'detail':'Cross-origin requests are disabled.'},status_code=403)
    response=await call_next(request)
    response.headers['X-Content-Type-Options']='nosniff'
    response.headers['Cache-Control']='no-store' if request.url.path.startswith('/api/') else 'no-cache'
    return response

@app.exception_handler(ValueError)
async def invalid(request,exc):return JSONResponse({'detail':str(exc)},status_code=422)

def canonical(p):return json.dumps(p,sort_keys=True,separators=(',',':'),allow_nan=False)
@lru_cache(maxsize=3)
def cached_build(key):
    with LOCK:
        t=time.monotonic();report,bodies,tools=build(json.loads(key),render_reference);report['buildSeconds']=round(time.monotonic()-t,2)
        return report,bodies,tools

async def read_upload(file):
    data=await file.read(MAX_UPLOAD+1)
    if len(data)>MAX_UPLOAD:raise ValueError('Files are limited to 64 MB.')
    return data

def import_step_locked(data,name):
    with LOCK:return import_step_board(data,name)

@app.get('/api/health')
def health():return {'app':'PCB Enclosure Studio','version':'0.3.0','kernel':cq.__version__}

@app.get('/api/example')
def example():
    p=ROOT/'examples'/'BlinkyHawk_V3b.kicad_pcb'
    return new_project(import_board(p.read_text(encoding='utf8'),p.name),'BlinkyHawk V3b enclosure')

@app.get('/api/example/step')
def step_example():
    return open_archive((ROOT/'examples'/'BlinkyHawk_V3b_STEP.pcbshell').read_bytes())

@app.post('/api/import/board')
async def upload_board(file:UploadFile):
    name=file.filename or ''
    if not name.lower().endswith(('.kicad_pcb','.step','.stp')):raise ValueError('Choose a KiCad board or STEP/STP PCB assembly.')
    data=await read_upload(file)
    if name.lower().endswith(('.step','.stp')):
        try:
            return await run_in_threadpool(import_step_locked,data,name)
        except ValueError:raise
        except Exception as exc:raise ValueError(f'The STEP PCB could not be imported: {exc}')
    try:text=data.decode('utf-8-sig')
    except UnicodeDecodeError:raise ValueError('The board is not a UTF-8 KiCad board file.')
    return new_project(import_board(text,Path(file.filename).name))

@app.post('/api/board/substrate')
def choose_substrate(payload:dict):
    with LOCK:return from_asset(payload['asset'],payload['name'],payload.get('substrate'),bool(payload.get('flip',False)))

def project_assets(project):
    assets={r['asset'] for r in project.get('references',[])}
    if project.get('board',{}).get('source')=='step':assets.add(project['board']['asset'])
    return assets

@app.post('/api/import/reference')
async def upload_reference(file:UploadFile):
    data=await read_upload(file)
    try:
        def load():
            with LOCK:return import_reference(data,file.filename or 'reference.step')
        return await run_in_threadpool(load)
    except ValueError:raise
    except Exception as exc:raise ValueError(f'The reference model could not be imported: {exc}')

@app.post('/api/reference/space')
def hardware_space(payload:dict):
    with LOCK:return reserve_space(payload['project'],payload['refId'],payload.get('body','all'),payload.get('margin',1))

@app.post('/api/build')
def rebuild(project:dict):
    try:return cached_build(canonical(project))[0]
    except ValueError:raise
    except Exception as exc:raise HTTPException(422,detail=f'CAD rebuild failed: {exc}')

def export_shape(shape,fmt,bed=False,flip=False):
    if fmt=='step':
        temp=DATA/(str(uuid.uuid4())+'.step')
        try:cq.exporters.export(shape,str(temp));return temp.read_bytes()
        finally:temp.unlink(missing_ok=True)
    vertices,triangles=shape.tessellate(.035,.12)
    mesh=trimesh.Trimesh(vertices=[v.toTuple() for v in vertices],faces=triangles,process=True)
    mesh.merge_vertices();mesh.fix_normals(multibody=True)
    if bed:
        if flip:mesh.apply_transform(trimesh.transformations.rotation_matrix(3.141592653589793,[1,0,0]))
        # Use the finished tessellation bounds: OCC cached bounds include tolerance margins.
        mesh.apply_translation([0,0,-mesh.bounds[0][2]])
    if not mesh.is_watertight:raise ValueError('The export mesh is not watertight; revise the feature or export STEP for inspection.')
    if fmt=='stl':return mesh.export(file_type='stl')
    if fmt=='3mf':return trimesh.exchange.threemf.export_3MF(trimesh.Scene(mesh))
    raise ValueError('Choose STL, 3MF, or STEP.')

@app.post('/api/export/{fmt}/{part}')
def export(fmt:str,part:str,project:dict):
    if fmt not in ['stl','3mf','step'] or part not in ['shell','lid','pair']:raise ValueError('Invalid export format or part.')
    report,bodies,_=cached_build(canonical(project))
    if report['errors']:raise ValueError('Fix or suppress the failed features before exporting: '+report['errors'][0])
    name=re.sub(r'[^a-zA-Z0-9_-]+','_',project.get('name','Enclosure'))[:80] or 'Enclosure'
    with LOCK:
        if part=='pair':
            buf=io.BytesIO()
            with zipfile.ZipFile(buf,'w',zipfile.ZIP_DEFLATED) as z:
                for k,s in bodies.items():
                    # Print meshes lie on the bed; STEP retains assembly coordinates.
                    z.writestr(f'{name}_{k}.{fmt}',export_shape(s,fmt,bed=fmt!='step',flip=k=='lid'))
                z.writestr('Fit-check.json',json.dumps({k:report[k] for k in ['metrics','errors','warnings','collisions','volumes']},indent=2))
            data=buf.getvalue();filename=f'{name}_{fmt}.zip'
        else:
            data=export_shape(bodies[part],fmt,bed=fmt!='step',flip=part=='lid');filename=f'{name}_{part}.{fmt}'
    return Response(data,media_type='application/octet-stream',headers={'Content-Disposition':f'attachment; filename="{filename}"'})

@app.post('/api/project/save')
def save_project(project:dict):
    validate_project(project);buf=io.BytesIO()
    with zipfile.ZipFile(buf,'w',zipfile.ZIP_DEFLATED) as z:
        z.writestr('project.json',json.dumps(project,indent=2,allow_nan=False))
        for asset in project_assets(project):z.writestr('assets/'+asset,asset_path(asset).read_bytes())
    name=re.sub(r'[^a-zA-Z0-9_-]+','_',project.get('name','Enclosure'))[:80]
    return Response(buf.getvalue(),media_type='application/octet-stream',headers={'Content-Disposition':f'attachment; filename="{name}.pcbshell"'})

@app.post('/api/project/open')
async def open_project(file:UploadFile):
    return open_archive(await read_upload(file))

def open_archive(data):
    try:
        with zipfile.ZipFile(io.BytesIO(data)) as z:
            if sum(i.file_size for i in z.infolist())>256*1024*1024:raise ValueError('The expanded project exceeds 256 MB.')
            if z.getinfo('project.json').file_size>8*1024*1024:raise ValueError('Project metadata exceeds 8 MB.')
            p=json.loads(z.read('project.json'));validate_project(p)
            for asset in project_assets(p):
                if not re.fullmatch(r'[0-9a-f]{64}\.(step|stp|stl|3mf)',asset):raise ValueError('Invalid reference identifier in project.')
                blob=z.read('assets/'+asset)
                if store_asset(blob,asset)!=asset:raise ValueError('Reference checksum mismatch.')
            return p
    except (zipfile.BadZipFile,KeyError,json.JSONDecodeError):raise ValueError('This is not a complete .pcbshell project.')

@app.get('/')
def index():return FileResponse(ROOT/'static'/'index.html')
app.mount('/static',StaticFiles(directory=ROOT/'static'),name='static')

if __name__=='__main__':
    import uvicorn
    uvicorn.run(app,host='127.0.0.1',port=int(os.environ.get('PCB_STUDIO_PORT','8766')),log_level='warning')
