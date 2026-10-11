"""
virtual_bench.py  --  browser dashboard for the bench instruments.

    py -3 virtual_bench.py            then http://127.0.0.1:8765 opens by itself
    py -3 virtual_bench.py --no-browser --port 8800

Standard library only.  Serves web/index.html and a small JSON API; each
instrument has its own poller thread (vb_drivers.py).  Listens on 127.0.0.1
only: this drives power supplies, so it is not offered to the network.

Opening the bench connects to every instrument but changes nothing on them.
Quitting (Ctrl+C) leaves every output exactly as it is -- use ALL OUTPUTS OFF
on the dashboard first if that is what you want.

While the bench is connected to an instrument, nothing else (the BlinkyHawk
test suite, Capture-Scope.ps1) can talk to it: they serve one SCPI client at a
time.  Disconnect it on its panel first.

API
  GET  /api/bench                         every instrument's snapshot
  GET  /api/inst/<key>?since=<t>          one snapshot + history newer than t
  GET  /api/inst/scope/screenshot         PNG of the scope screen
  POST /api/inst/<key>/connect | /disconnect
  POST /api/inst/<key>/act   {"action": ..., "args": {...}}
  POST /api/inst/<key>/scpi  {"cmd": "..."}
  POST /api/alloff                        every output off, every instrument
"""

import argparse
import json
import math
import os
import re
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import vb_drivers as d

HERE = os.path.dirname(os.path.abspath(__file__))
INDEX = os.path.join(HERE, "web", "index.html")

# key, display name, driver, host, port -- static IPs on the bench switch
# (PC = 192.168.10.1/24); the GigaMeter is on the WiFi LAN.
BENCH = [
    ("scope", "Oscilloscope",       d.SiglentScope,   "192.168.10.2", 5025),
    ("fgen",  "Function generator", d.RigolDG800,     "192.168.10.3", 5025),
    ("load",  "Electronic load",    d.RigolDL3000,    "192.168.10.4", 5555),
    ("dmm",   "Multimeter",         d.RigolDM858,     "192.168.10.5", 5025),
    ("psu",   "Power supply",       d.SiglentSPD4000, "192.168.10.6", 5025),
    ("giga",  "GigaMeter",          d.GigaBench,      "192.168.40.152", 8080),
]

INSTRUMENTS = {}


def clean(x):
    """JSON without NaN/Infinity, which JSON.parse rejects."""
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {k: clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [clean(v) for v in x]
    return x


class Handler(BaseHTTPRequestHandler):
    server_version = "VirtualBench/1.0"

    def log_message(self, fmt, *args):
        pass

    # --- responses ---
    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(clean(obj)).encode(), "application/json")

    def _err(self, e, code=400):
        self._json({"ok": False, "error": d._msg(e)}, code)

    def _inst(self, key):
        inst = INSTRUMENTS.get(key)
        if inst is None:
            raise KeyError(key)
        return inst

    # --- GET ---
    def do_GET(self):
        url = urlsplit(self.path)
        path, qs = url.path, parse_qs(url.query)
        try:
            if path in ("/", "/index.html"):
                with open(INDEX, "rb") as fh:
                    self._send(200, fh.read(), "text/html; charset=utf-8")
            elif path == "/api/bench":
                self._json({"ok": True, "instruments": [i.snapshot() for i in INSTRUMENTS.values()]})
            elif m := re.fullmatch(r"/api/inst/(\w+)/screenshot", path):
                inst = self._inst(m.group(1))
                if not hasattr(inst, "screenshot"):
                    raise ValueError("no screenshot for this instrument")
                self._send(200, inst.screenshot(), "image/png")
            elif m := re.fullmatch(r"/api/inst/(\w+)", path):
                inst = self._inst(m.group(1))
                since = float(qs.get("since", ["0"])[0])
                snap = inst.snapshot()
                snap["history"] = inst.history_since(since)
                snap["ok"] = True
                self._json(snap)
            else:
                self._send(404, b"not found", "text/plain")
        except KeyError as e:
            self._err(f"no instrument {e}", 404)
        except (ValueError, d.ScpiError) as e:
            self._err(e)
        except Exception as e:                         # noqa: BLE001
            self._err(e, 500)

    # --- POST ---
    def do_POST(self):
        path = urlsplit(self.path).path
        try:
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
            if path == "/api/alloff":
                self._json(all_off())
                return
            m = re.fullmatch(r"/api/inst/(\w+)/(connect|disconnect|act|scpi)", path)
            if not m:
                self._send(404, b"not found", "text/plain")
                return
            inst, op = self._inst(m.group(1)), m.group(2)
            if op == "connect":
                inst.connect()
                self._json({"ok": True, "ident": inst.ident})
            elif op == "disconnect":
                inst.disconnect()
                self._json({"ok": True})
            elif op == "act":
                res = inst.act(body.get("action", ""), body.get("args") or {})
                self._json({"ok": True, "result": res})
            else:
                if not hasattr(inst, "raw"):
                    raise ValueError("this instrument has no SCPI console")
                self._json({"ok": True, "reply": inst.raw(body.get("cmd", ""))})
        except KeyError as e:
            self._err(f"no instrument {e}", 404)
        except (ValueError, TypeError, d.ScpiError, d.InstrumentError) as e:
            self._err(e)
        except Exception as e:                         # noqa: BLE001
            self._err(e, 500)


def all_off():
    """Every output off, in parallel so one slow instrument doesn't hold up the rest."""
    results = {}

    def off(inst):
        if not inst.connected or type(inst).safe_off is d.Instrument.safe_off:
            return
        try:
            with inst.lock:
                inst.safe_off()
            results[inst.key] = "off"
        except Exception as e:                         # noqa: BLE001
            results[inst.key] = "FAILED: " + d._msg(e)
        inst._wake.set()

    threads = [threading.Thread(target=off, args=(i,)) for i in INSTRUMENTS.values()]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=10)
    failed = {k: v for k, v in results.items() if v != "off"}
    return {"ok": not failed, "results": results,
            "error": "; ".join(f"{k}: {v}" for k, v in failed.items()) or None}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--trace", metavar="FILE", help="log every SCPI command sent to FILE")
    a = ap.parse_args()
    if a.trace:
        d.TRACE = open(a.trace, "a", encoding="utf-8")

    for key, name, cls, host, port in BENCH:
        INSTRUMENTS[key] = cls(key, name, host, port)
    try:
        srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    except OSError as e:
        sys.exit(f"cannot listen on 127.0.0.1:{a.port} ({e}) -- is the bench already running? "
                 "Try --port")
    srv.daemon_threads = True
    for inst in INSTRUMENTS.values():
        inst.start()
    url = f"http://127.0.0.1:{a.port}/"
    print(f"Virtual Bench on {url}   (Ctrl+C to quit; outputs are left as they are)")
    if not a.no_browser:
        threading.Timer(0.5, webbrowser.open, args=(url,)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        for inst in INSTRUMENTS.values():
            try:
                inst.disconnect()
            except Exception:                          # noqa: BLE001
                pass


if __name__ == "__main__":
    main()
