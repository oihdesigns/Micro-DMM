"""Download pinned, local frontend dependencies; no CDNs are used at runtime."""
import io,tarfile,urllib.request
from pathlib import Path
root=Path(__file__).resolve().parent/'static'/'vendor'
root.mkdir(parents=True,exist_ok=True)
with urllib.request.urlopen('https://registry.npmjs.org/three/-/three-0.169.0.tgz',timeout=60) as r:
    archive=tarfile.open(fileobj=io.BytesIO(r.read()),mode='r:gz')
for src,dst in [('package/build/three.module.js','three.module.js'),('package/examples/jsm/controls/OrbitControls.js','OrbitControls.js'),('package/LICENSE','THREE-LICENSE.txt')]:
    (root/dst).write_bytes(archive.extractfile(src).read())
print('Installed pinned Three.js 0.169.0 viewer files.')
