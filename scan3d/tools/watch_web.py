#!/usr/bin/env python3
"""Watchdog web page: http://localhost:8792/  (live watchdog.sh + log tails, refresh every 15 s); /docs/<file> serves docs/.
  nohup python3 tools/watch_web.py > runs/watch_web.log 2>&1 &
"""
import html
import subprocess
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LOGS = ["runs/spline_smoke.log", "runs/vio_morning.log", "runs/calibonly_hs_20261019_131326.log",
        "runs/calibonly_hs_20261019_145816.log"]


def snapshot():
    try:
        wd = subprocess.run(["bash", str(ROOT / "tools/watchdog.sh")], capture_output=True, text=True, timeout=20).stdout
    except Exception as e:                       # noqa: BLE001
        wd = f"watchdog.sh failed: {e}"
    parts = [("watchdog.sh", wd)]
    extra = sorted((ROOT / "runs").glob("*.log"), key=lambda p: p.stat().st_mtime, reverse=True)[:6]
    for p in dict.fromkeys([ROOT / l for l in LOGS] + extra):
        if p.exists():
            lines = p.read_text(errors="replace").splitlines()[-12:]
            parts.append((p.relative_to(ROOT).as_posix(), "\n".join(l[:200] for l in lines)))
    return parts


PAGE = """<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>감시견</title><meta http-equiv=refresh content=15>
<style>body{{font:14px system-ui;background:#111;color:#ddd;margin:16px}}h2{{font-size:15px;color:#8cf;margin:18px 0 4px}}
pre{{background:#1b1b1b;padding:8px;border-radius:6px;white-space:pre-wrap;word-break:break-all;margin:0}}</style>
<h1 style="font-size:18px">감시견 — 15초마다 새로고침</h1>{body}"""


class H(SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=str(ROOT), **k)

    def do_GET(self):
        if self.path in ("/", "/watch"):
            body = "".join(f"<h2>{html.escape(n)}</h2><pre>{html.escape(t)}</pre>" for n, t in snapshot())
            b = PAGE.format(body=body).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        elif self.path.startswith("/docs/"):
            super().do_GET()
        else:
            self.send_error(404)

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    ThreadingHTTPServer(("0.0.0.0", 8792), H).serve_forever()
