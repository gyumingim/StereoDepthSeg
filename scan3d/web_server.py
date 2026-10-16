"""내장 HTTP 서버 (표준 라이브러리). 정적 파일 scan3d/web/ + JSON/바이너리 API.

GET  /api/state                  상태 JSON (VIO, 폰, 깊이, 맵, 물체, 클래스 팔레트)
GET  /api/voxels?since=V&epoch=E 바뀐 복셀만: 헤더 <IIIf (epoch, version, M, voxel_m) + M × 17 B
                                 (<hhhBBBhiH: ix,iy,iz, r,g,b, cls, inst, hits). epoch 가 다르면 전체.
GET  /api/trajectory             [[t, x, y, z], ...] (rect0 원점, 솎음) + 현재 자세
GET  /api/frame.jpg?kind=rgb|overlay|depth   최근 키프레임 2D 패널
GET  /api/recordings?refresh=1   폰 녹화 목록(adb) + 가져온 녹화(recordings/) 목록
POST /api/cmd  {"cmd": pause|resume|reset_map|reset_vio|save|set, ...}
               {"cmd": "mode", "mode": live|record|both, "start_app": true}   모드 시작 (앱 --es mode)
               {"cmd": "stop", "stop_app": true}                              세션·재생 정지 (녹화 마무리 후 앱 정지)
               {"cmd": "pull", "name": ..., "replay": true, "every_frame": false, "iters": 8}   가져와서 복원
               {"cmd": "replay", "name": ..., "every_frame": false, "iters": 8}               가져온 녹화 복원
"""
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import json
import struct
import threading
from pathlib import Path
from urllib.parse import urlparse, parse_qs

WEB = Path(__file__).parent / "web"
CTYPE = {".html": "text/html; charset=utf-8", ".js": "application/javascript", ".css": "text/css",
         ".json": "application/json", ".png": "image/png", ".ico": "image/x-icon"}


def make_handler(scanner):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json"):
            if isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, default=_default, separators=(",", ":")))

        def do_GET(self):
            u = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(u.query).items()}
            try:
                if u.path == "/api/state":
                    return self._json(scanner.state())
                if u.path == "/api/voxels":
                    epoch, ver, rec = scanner.vmap.delta(int(q.get("since", 0)),
                                                         int(q["epoch"]) if "epoch" in q else None)
                    body = struct.pack("<IIIf", epoch, ver, len(rec), scanner.vmap.voxel_m) + rec.tobytes()
                    return self._send(200, body, "application/octet-stream")
                if u.path == "/api/recordings":
                    return self._json(scanner.recordings_json(refresh=q.get("refresh") == "1"))
                if u.path == "/api/trajectory":
                    return self._json(scanner.trajectory_json())
                if u.path == "/api/frame.jpg":
                    jpg = scanner.frame_jpeg(q.get("kind", "overlay"))
                    return self._send(200, jpg, "image/jpeg") if jpg else self._send(204, b"", "text/plain")
                path = "/index.html" if u.path in ("/", "") else u.path
                f = (WEB / path.lstrip("/")).resolve()
                if WEB.resolve() not in f.parents or not f.is_file():
                    return self._send(404, "not found", "text/plain")
                return self._send(200, f.read_bytes(), CTYPE.get(f.suffix, "application/octet-stream"))
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as e:
                try:
                    self._json({"error": repr(e)}, 500)
                except Exception:
                    pass

        def do_POST(self):
            u = urlparse(self.path)
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self._json({"error": "bad json"}, 400)
            if u.path != "/api/cmd":
                return self._json({"error": "unknown"}, 404)
            try:
                return self._json(scanner.command(body))
            except Exception as e:
                return self._json({"error": repr(e)}, 500)

    return H


def _default(o):
    import numpy as np
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    return str(o)


def serve(scanner, host="0.0.0.0", port=8795):
    srv = ThreadingHTTPServer((host, port), make_handler(scanner))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True, name="http").start()
    return srv
