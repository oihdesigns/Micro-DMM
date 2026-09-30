"""Launch the loopback-only service with no background console window."""
import json, os, subprocess, sys, time, urllib.request, webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PORT = int(os.environ.get('PCB_STUDIO_PORT', '8766'))
URL = f'http://127.0.0.1:{PORT}'

def healthy():
    try:
        with urllib.request.urlopen(URL+'/api/health', timeout=2) as r:
            return json.load(r).get('app') == 'PCB Enclosure Studio'
    except Exception:
        return False

if not healthy():
    log = open(ROOT/'server.log', 'a', encoding='utf8')
    process = subprocess.Popen([sys.executable, str(ROOT/'server.py')], cwd=ROOT,
        stdin=subprocess.DEVNULL, stdout=log, stderr=log,
        creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
    (ROOT/'.server.json').write_text(json.dumps({'pid':process.pid,'port':PORT}), encoding='utf8')
    for _ in range(90):
        if healthy(): break
        if process.poll() is not None: break
        time.sleep(.5)
    if not healthy():
        print('The CAD service could not start. Run Install dependencies.cmd, then try again. Details: server.log')
        sys.exit(1)
if '--no-browser' not in sys.argv: webbrowser.open(URL)
print('PCB Enclosure Studio is running at '+URL)
