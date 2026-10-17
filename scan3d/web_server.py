"""내장 HTTP 서버 (표준 라이브러리). 정적 파일 scan3d/web/ + JSON/바이너리 API.

GET  /api/state                  상태 JSON (VIO, 폰, 깊이, 맵, 물체, 클래스 팔레트)
GET  /api/voxels?since=V&epoch=E 바뀐 복셀만: 헤더 <IIIf (epoch, version, M, voxel_m) + M × 17 B
                                 (<hhhBBBhiH: ix,iy,iz, r,g,b, cls, inst, hits). epoch 가 다르면 전체.
GET  /api/mesh?since=V&epoch=E   TSDF 메시 바뀐 청크만 (tsdf_map.mesh_bytes 형식 S3M1). epoch 가 다르면 전체, 지도가 없으면 epoch 0 빈 것
GET  /api/mesh.ply · /api/mesh.obj   지금 메시 내려받기 (색 포함)
GET  /api/replay/mesh?rec=NAME[&recon=R]   복원 결과 mesh_web.bin (S3M1 한 청크) · /api/replay/mesh.ply|mesh.obj 내려받기
GET  /api/trajectory             [[t, x, y, z], ...] (rect0 원점, 솎음) + 현재 자세
GET  /api/frame.jpg?kind=rgb|overlay|depth   최근 키프레임 2D 패널 (웹은 depth 만 씀)
GET  /api/cam.jpg?lens=0|1&overlay=0|1        가장 최근 수신 프레임 (0 = 초광각 렌즈 2 = 정류 왼쪽, 1 = 광각 렌즈 5).
                                 JPEG 스트림이면 받은 바이트 그대로, overlay=1 이면 렌즈 0 에 최근 YOLO 마스크·박스. 없으면 204
GET  /api/recordings?refresh=1   폰 녹화 목록(adb) + 가져온 녹화(recordings/) 목록 + items (녹화마다 상태 한 줄, rec_sync.py)
GET  /api/replay/info?rec=NAME[&recon=R|&pair=2+6]  복원 리플레이 timeline.json (+ thumbs 준비 여부, 복원 목록,
                                              3렌즈 녹화면 pair_recons {쌍: 복원} · pair_compare 같은 키프레임 비교) — replay_view.py
GET  /api/replay/voxels?rec=NAME[&recon=R]    voxels_t.bin (S3VT: 17 B 레코드 처음 보인 시각 순 + float32 시각 + int32 키프레임)
GET  /api/replay/frame.jpg?rec=NAME&lens=0|1&t=SEC[&phys=2|5|6]   그 시각(녹화 기준 초)의 녹화 프레임 (thumbs/, 없으면
                                              만들기 시작하고 204). phys = 물리 렌즈 (3렌즈 녹화에서 보는 쌍의 렌즈)
GET  /api/replay/depth.jpg?rec=NAME&recon=R&kf=I     키프레임 I 의 깊이 그림
GET  /api/combos?mode=live|record|both&kf=3&fmt=h264|jpeg|nv21[&lenses=3]   (lenses=3 = 3개 모두 녹화, 녹화 인코더 3개)
                                 화질(4:3 해상도)·fps 조합별 가능 여부·이유·예상 부하 (load_model.py). fmt = 실시간 영상 형식 (기본 h264)
                                 state.ui.stream = 지금 크기·fps·형식, state.ui.load = 실행 중 예상 대비 실측·과부하 신호·한 단계 낮추기 제안
POST /api/cmd  {"cmd": pause|resume|reset_map|reset_vio|save|set, ...}
               {"cmd": "mode", "mode": live|record|both, "start_app": true,
                "size": "WxH", "fps": 15, "stream_fmt": "h264"}
                                            모드 시작 (앱 --es mode). size·fps·형식은 부하 모델로 검사 (불가능하면 ok:false + reasons,
                                            "force": true 면 그래도 시작). scanner.set_stream(w, h, fps, rec_kbps=, stream_fmt=,
                                            live_kbps=) 가 있으면 그것으로 적용 (stream_fmt 인자를 못 받는 옛 set_stream 이면 빼고 부름)
               {"cmd": "load_reset"}        부하 모델 계수 초기화 (runs/load_coeffs.json 지움)
               {"cmd": "caps_refresh"}      폰 능력표 caps.json 다시 읽기 (adb)
               {"cmd": "stop", "stop_app": true}                              세션·재생 정지 (녹화 마무리 후 앱 정지)
               {"cmd": "pull", "name": ..., "replay": true, "every_frame": false, "iters": 8}   가져와서 복원
               {"cmd": "replay", "name": ..., "every_frame": false, "iters": 8}               가져온 녹화 복원
               {"cmd": "rec_auto", "pull": bool, "recon": bool}   자동 가져오기·자동 복원 켜고 끔 (runs/rec_auto.json)
               {"cmd": "rec_retry", "name": ...}                  실패·멈춘 복원 다시
               {"cmd": "rec_delete_phone", "name": ...}           PC 사본이 똑같을 때만 폰에서 지움 (웹이 먼저 물어봄)
               {"cmd": "mesh_set", "voxel_m": 0.02, "z_max": 2.5}   TSDF 메시 설정 (바꾸면 메시 지도를 새로 시작)
               {"cmd": "sfm_recon", "name": REC}   정밀 복원 (sfm_recon.py 별도 프로세스, 수 분~수십 분) → REC/recon_<시각>_sfm.
                                            대기 중(실시간·녹화·복원 아님)일 때만. 도는 동안 실시간·복원·다른 정밀 복원은 거절,
                                            자동 복원은 쉼. 진행률 state.ui.sfm (SfmJob), 녹화 목록 줄에도 '3D 복원 중 %'
               {"cmd": "sfm_stop"}  정밀 복원 멈춤 (만들던 폴더 지움)
               {"cmd": "restart"}   [처음부터 다시]: VIO 가 죽었으면 세션 다시 시작(브리지 새로), 아니면 VIO 재시작 + 맵 초기화

쉬운 UI 도우미 (Helper, 이 파일 안의 별도 스레드, scan3d.py 로직은 안 건드림):
  - 3 s 마다 adb devices -l → state.ui.usb (none | device | unauthorized | offline | noadb).
    transport_id 가 바뀌면 3 s 사이에 뺐다 꽂은 것도 "다시 꽂음" 으로 본다 (REVIEW4 B2)
  - 10 s 마다 adb shell dumpsys thermalservice 의 SKIN 온도 → state.ui.temp_c / temp_level (ok | warm >42 | hot >45 °C)
  - WSL + Windows adb 면 [스캔 시작](live/both) 때 Windows 쪽 win_relay.py --no-start-app 을 interop
    (powershell.exe Start-Process, 창 없음)으로 띄우고 [정지] 때 끈다. 죽거나 USB 를 다시 꽂으면 다시 띄운다
    (adb forward 다시; 다시 꽂음은 래치해 두었다가 relay_next 가 지나면 처리, 연속 실패는 8→16→32→60 s 로 늘림).
    win_relay 자신도 폰 포트 접속이 실패하면 adb forward 를 다시 한다. 로그 scan3d/runs/win_relay.log
"""
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import inspect
import json
import os
import re
import shutil
import struct
import subprocess
import threading
import time
from pathlib import Path
from urllib.parse import urlparse, parse_qs

try:
    import load_model
    import replay_view
    import tsdf_map
    import voxel_map
except ImportError:                     # 다른 위치에서 import 될 때
    from scan3d import load_model       # type: ignore
    from scan3d import replay_view      # type: ignore
    from scan3d import tsdf_map         # type: ignore
    from scan3d import voxel_map        # type: ignore

WEB = Path(__file__).parent / "web"
CTYPE = {".html": "text/html; charset=utf-8", ".js": "application/javascript", ".css": "text/css",
         ".json": "application/json", ".png": "image/png", ".ico": "image/x-icon"}


def make_handler(scanner, helper):
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
                    st = scanner.state()
                    st["ui"] = helper.ui()
                    st["mesh"] = _mesh_state(scanner)
                    return self._json(st)
                if u.path == "/api/mesh":
                    ts = scanner.vmap.tsdf
                    if ts is None:
                        return self._send(200, _EMPTY_MESH, "application/octet-stream")
                    body = ts.mesh_bytes(int(q.get("since", 0)), int(q["epoch"]) if "epoch" in q else None)
                    return self._send(200, body, "application/octet-stream")
                if u.path in ("/api/mesh.ply", "/api/mesh.obj"):
                    ts = scanner.vmap.tsdf
                    if ts is None:
                        return self._json({"error": "메시가 아직 없습니다 (스캔이나 복원을 먼저)"}, 404)
                    return self._download(*ts.mesh_arrays(), kind=u.path[-3:], name=time.strftime("scan3d_mesh_%Y%m%d-%H%M%S"))
                if u.path == "/api/voxels":
                    epoch, ver, rec = scanner.vmap.delta(int(q.get("since", 0)),
                                                         int(q["epoch"]) if "epoch" in q else None)
                    body = struct.pack("<IIIf", epoch, ver, len(rec), scanner.vmap.voxel_m) + rec.tobytes()
                    return self._send(200, body, "application/octet-stream")
                if u.path == "/api/recordings":
                    return self._json(scanner.recordings_json(refresh=q.get("refresh") == "1"))
                if u.path == "/api/combos":
                    try:
                        kf = float(q["kf"]) if q.get("kf") else None
                    except ValueError:
                        kf = None
                    n_rec = 3 if q.get("lenses") == "3" else 2
                    return self._json(helper.combos(q.get("mode", "live"), kf, q.get("fmt"), n_rec))
                if u.path == "/api/trajectory":
                    return self._json(scanner.trajectory_json())
                if u.path == "/api/frame.jpg":
                    jpg = scanner.frame_jpeg(q.get("kind", "overlay"))
                    return self._send(200, jpg, "image/jpeg") if jpg else self._send(204, b"", "text/plain")
                if u.path == "/api/cam.jpg":
                    try:
                        lens = int(q.get("lens", 0))
                    except ValueError:
                        lens = 0
                    jpg = scanner.cam_jpeg(lens, q.get("overlay") == "1") if hasattr(scanner, "cam_jpeg") else None
                    return self._send(200, jpg, "image/jpeg") if jpg else self._send(204, b"", "text/plain")
                if u.path.startswith("/api/replay/"):
                    return self._replay(u.path[len("/api/replay/"):], q)
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

        def _download(self, V, C, F, kind, name):
            if kind == "ply":
                import io
                buf = io.BytesIO()
                tsdf_map.write_mesh_ply(buf, V, C, F)
                body = buf.getvalue()
            else:
                body = tsdf_map.mesh_obj_bytes(V, C, F)
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Content-Disposition", f'attachment; filename="{name}.{kind}"')
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _replay(self, what, q):
            """복원 리플레이 (replay_view.py 형식). rec = 녹화 이름, recon = 복원 폴더 이름 (없으면 가장 최근 리플레이 가능한 것)."""
            try:
                rec = replay_view.safe_name(q.get("rec"))
                rdir = Path(scanner.a.rec_root) / rec
                recon = q.get("recon") or replay_view.best_recon(rdir, q.get("pair") or None)
                cdir = rdir / replay_view.safe_name(recon) if recon else None
            except ValueError as e:
                return self._json({"error": str(e)}, 400)
            if not rdir.is_dir():
                return self._json({"error": f"녹화 없음: {rec}"}, 404)
            if what == "frame.jpg":
                try:
                    lens, t = int(q.get("lens", 0)), float(q.get("t", 0))
                except ValueError:
                    return self._json({"error": "lens·t 값"}, 400)
                if not replay_view.thumbs_ready(rdir):
                    _thumbs_bg(rdir, scanner.log)
                    return self._send(204, b"", "text/plain")
                jpg, ft = replay_view.thumb_at(rdir, lens, t, q.get("phys") or None)
                if not jpg:
                    return self._send(204, b"", "text/plain")
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg)))
                self.send_header("Cache-Control", "max-age=3600")
                self.send_header("X-Frame-T", f"{ft:.4f}")
                self.end_headers()
                self.wfile.write(jpg)
                return
            if cdir is None or not cdir.is_dir():
                return self._json({"error": "리플레이할 복원본이 없음 (시간 정보가 있는 복원이 필요)"}, 404)
            if what == "info":
                tl = replay_view.read_timeline(cdir)
                if tl is None:
                    return self._json({"error": f"{recon} 에 timeline.json 없음 (옛 복원)"}, 404)
                if not replay_view.thumbs_ready(rdir):
                    _thumbs_bg(rdir, scanner.log)
                return self._json(dict(tl, thumbs=replay_view.thumbs_ready(rdir), thumbs_busy=str(rdir) in _thumb_busy,
                                       recons=replay_view.list_recons(rdir), pair_recons=replay_view.pair_recons(rdir),
                                       pair_compare=replay_view.pair_compare(rdir)))
            if what == "voxels":
                p = cdir / "voxels_t.bin"
                if not p.is_file():
                    return self._json({"error": "voxels_t.bin 없음"}, 404)
                return self._send(200, p.read_bytes(), "application/octet-stream")
            if what == "mesh":
                p = cdir / "mesh_web.bin"
                if not p.is_file():
                    return self._json({"error": "이 복원에는 메시가 없습니다 (옛 복원)"}, 404)
                return self._send(200, p.read_bytes(), "application/octet-stream")
            if what in ("mesh.ply", "mesh.obj"):
                p = cdir / "mesh.ply"
                if not p.is_file():
                    return self._json({"error": "이 복원에는 메시가 없습니다 (옛 복원)"}, 404)
                return self._download(*tsdf_map.read_mesh_ply(p), kind=what[-3:], name=f"{rec}_{cdir.name}_mesh")
            if what == "depth.jpg":
                try:
                    p = cdir / "kf" / f"d_{int(q.get('kf', 0)):05d}.jpg"
                except ValueError:
                    return self._json({"error": "kf 값"}, 400)
                return self._send(200, p.read_bytes(), "image/jpeg") if p.is_file() else self._send(204, b"", "text/plain")
            return self._json({"error": "unknown"}, 404)

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
                return self._json(helper.command(body))
            except Exception as e:
                return self._json({"error": repr(e)}, 500)

    return H


_EMPTY_MESH = tsdf_map.mesh_bin_static([], [], [])    # 지도 없음: epoch 0, 청크 없음 (웹이 지운다)


def _mesh_state(scanner):
    """/api/state 의 mesh: 설정 (cfg) + 지금 TSDF 숫자 (없으면 None)."""
    try:
        cfg = voxel_map.tsdf_cfg()
    except Exception as e:                      # torch·tsdf_map 문제여도 상태 JSON 은 나가게
        return dict(error=repr(e))
    vm = getattr(scanner, "vmap", None)
    ts = getattr(vm, "tsdf", None) if vm is not None else None
    return dict(cfg=dict(enabled=cfg["enabled"], voxel_m=cfg["voxel_m"], z_min=cfg["z_min"], z_max=cfg["z_max"],
                         sigma_max=cfg["sigma_max"]), stats=ts.stats() if ts is not None else None)


_thumb_busy = set()
ALL_LENS_PAIRS = ("all", "2+5+6", "2,5,6")      # 웹 '깊이 렌즈' = 3개 모두 녹화 (scan3d.ALL_LENSES 와 같은 값)


def _thumbs_bg(rdir, log):
    """녹화 카메라 화면 그림(thumbs/)이 없으면 뒤에서 한 번 만든다 (옛 복원·직접 가져온 녹화)."""
    k = str(rdir)
    if k in _thumb_busy:
        return
    _thumb_busy.add(k)

    def run():
        try:
            replay_view.ensure_thumbs(rdir, log)
        except Exception as e:
            log(f"[replay] 카메라 화면 그림 실패 {rdir.name}: {e!r}")
        finally:
            _thumb_busy.discard(k)
    threading.Thread(target=run, daemon=True, name="thumbs").start()


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
    helper = Helper(scanner)
    srv = ThreadingHTTPServer((host, port), make_handler(scanner, helper))
    srv.helper = helper
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True, name="http").start()
    return srv


# ---------------- 쉬운 UI 도우미 ----------------
TEMP_WARM, TEMP_HOT = 42.0, 45.0
HERE = Path(__file__).resolve().parent


def parse_skin_temp(text):
    """dumpsys thermalservice → SKIN(mType=3) 온도 °C (HAL 현재값 구역 우선). 없으면 None."""
    sec = text
    i = text.find("Current temperatures from HAL")
    if i >= 0:
        sec = text[i:]
        j = sec.find("Current cooling devices")
        if j > 0:
            sec = sec[:j]
    vals = []
    for m in re.finditer(r"Temperature\{mValue=([-\d.]+), mType=(\d+), mName=([^,}]+)", sec):
        if m.group(2) == "3" or "skin" in m.group(3).lower():
            try:
                vals.append(float(m.group(1)))
            except ValueError:
                pass
    vals = [v for v in vals if -20 < v < 120]
    return max(vals) if vals else None


def _wslpath_w(p):
    if not p:
        return None
    try:
        return subprocess.run(["wslpath", "-w", str(p)], capture_output=True, text=True, timeout=5).stdout.strip() or None
    except (OSError, subprocess.TimeoutExpired):
        return None


def _ps_exe():
    for c in (shutil.which("powershell.exe"), "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"):
        if c and os.path.exists(c):
            return c
    return None


def _win_python():
    """(windows_path, prefix) of py.exe / python.exe found from WSL, or (None, None).
    Interop PowerShell from the logon wsl.exe has no LOCALAPPDATA/TEMP and a short PATH, so it cannot find them itself."""
    import glob
    pats = [("/mnt/c/Users/*/AppData/Local/Programs/Python/Launcher/py.exe", "-3 "), ("/mnt/c/Windows/py.exe", "-3 "),
            ("/mnt/c/Users/*/AppData/Local/Programs/Python/Python3*/python.exe", ""),
            ("/mnt/c/Program Files/Python3*/python.exe", "")]
    for pat, pre in pats:
        for c in sorted(glob.glob(pat)):
            if "/Users/Public/" in c or "/Users/Default" in c:
                continue
            w = _wslpath_w(Path(c))
            if w:
                return w, pre
    return None, None


def _win_temp(py_w):
    """A writable Windows working dir next to the found Python's user profile (C:\\Users\\<u>\\AppData\\Local\\Temp)."""
    import re as _re
    m = _re.match(r"([A-Za-z]:\\Users\\[^\\]+)\\", py_w or "")
    return (m.group(1) + "\\AppData\\Local\\Temp") if m else "C:\\Windows\\Temp"


def _psq(s):
    return "'" + str(s).replace("'", "''") + "'"


class SfmJob:
    """정밀 복원 (sfm_recon.py) 별도 프로세스 하나. rec_sync 의 job 자리에 넣어 자동 복원이 겹치지 않게 한다
    (rec_sync 는 job['rp'].thread.is_alive() · .progress 만 본다 — replay.Replayer 와 같은 꼴)."""

    def __init__(self, sc, rec_dir, log):
        import sys
        self.sc, self.rec_dir, self.log = sc, Path(rec_dir), log
        self.out = self.rec_dir / time.strftime("recon_%Y%m%d-%H%M%S_sfm")
        self.pfile = self.out / "sfm_progress.json"
        self.stopped = False
        self.rc = None
        self.t0 = time.time()
        logp = HERE / "runs" / time.strftime("sfm_%Y%m%d-%H%M%S.log")
        self.logp = logp
        cmd = ["nice", "-n", "10", sys.executable, str(HERE / "sfm_recon.py"), str(self.rec_dir), "--out", str(self.out)]
        self.fo = open(logp, "w")
        self.proc = subprocess.Popen(cmd, stdout=self.fo, stderr=subprocess.STDOUT, cwd=str(HERE.parent))
        self.thread = threading.Thread(target=self._wait, daemon=True, name="sfm")
        self.thread.start()
        log(f"[sfm] 정밀 복원 시작 {self.rec_dir.name} → {self.out.name} (pid {self.proc.pid}, 로그 {logp})")

    def _wait(self):
        self.rc = self.proc.wait()
        self.fo.close()
        if self.stopped:
            shutil.rmtree(self.out, ignore_errors=True)
        self.log(f"[sfm] 정밀 복원 끝 rc={self.rc} {'(멈춤)' if self.stopped else ''} {self.rec_dir.name}")

    def stop(self):
        self.stopped = True
        if self.proc.poll() is None:
            self.proc.terminate()

    def alive(self):
        return self.thread.is_alive()

    def file(self):
        try:
            return json.loads(self.pfile.read_text())
        except (OSError, ValueError):
            return {}

    @property
    def progress(self):
        """rec_sync 가 읽는 꼴: state (완료 | 정지됨 | 단계), pos_s/total_s (= %), timeline, out, error."""
        f = self.file()
        done = not self.alive()
        if done and self.rc == 0 and f.get("state") == "done":
            st = "완료"
        elif done:
            st = "정지됨"                     # 실패도 '쉼' 으로 (자동 복원 실패 목록에 넣지 않음, 로그·웹 sfm.error 로 알림)
        else:
            st = f"정밀 복원 · {f.get('stage') or '시작'}"
        return dict(state=st, pos_s=float(f.get("pct") or 0.0), total_s=100.0, timeline=st == "완료", out=str(self.out),
                    error=None, pair="2+5", keyframes=1)

    def snapshot(self):
        f = self.file()
        return dict(name=self.rec_dir.name, out=self.out.name, running=self.alive(), rc=self.rc, stopped=self.stopped,
                    stage=f.get("stage"), pct=f.get("pct"), msg=f.get("msg"), error=f.get("error"),
                    elapsed_s=round(time.time() - self.t0, 1), log=str(self.logp))


class Helper:
    """폰 USB·온도 감시 + Windows 중계(win_relay.py) 켜기/끄기. 실패해도 스캐너는 그대로 돈다."""

    KILL_ALL = ("Get-CimInstance Win32_Process | Where-Object { $_.ProcessId -ne $PID -and $_.Name -notmatch '^(powershell|pwsh)' -and $_.CommandLine -like '*win_relay.py*' } | "
                "ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }; ")

    def __init__(self, scanner):
        self.sc = scanner
        self.lock = threading.RLock()
        self.usb = "모름"
        self.usb_t = 0.0
        self.temp_c = None
        self.temp_t = 0.0
        self.want_relay = False
        self.relay_pid = None
        self.relay_state = "꺼짐"          # 꺼짐 | 켜는 중 | 켜짐 | 오류 | 해당 없음
        self.relay_err = None
        self.relay_starts = 0
        self.relay_next = 0.0
        self.relay_t = 0.0                 # 마지막 중계 시작 시각 (monotonic)
        self.relay_fails = 0               # 연속 시작 실패·곧 죽음 수 → 다시 시도 간격 (B11)
        self.replug_pending = False        # 다시 꽂음 래치 (B2: relay_next 창 안이면 나중에 처리)
        self.transport = None              # adb devices -l 의 transport_id (바뀌면 다시 꽂음)
        self.ps = _ps_exe()
        link = getattr(scanner, "link", None)
        a = getattr(scanner, "a", None)
        self.relay_ok = bool(self.ps and link is not None and link.kind == "windows"
                             and getattr(a, "link", "relay") in ("relay", "auto"))
        if not self.relay_ok:
            self.relay_state = "해당 없음"
        self.relay_log = HERE / "runs" / "win_relay.log"
        # 화질·fps 부하 모델 (load_model.py): 실패해도 스캐너는 돈다
        self.lm = self.mon = None
        try:
            self.lm = load_model.LoadModel(log=scanner.log)
            self.mon = load_model.LoadMonitor(self.lm)
            # FFS 입력 축소: scan3d 와 같은 규칙 (--scale 고정, 없으면 --ffs-max-px 기준 자동 → 크기마다 다름)
            self.lm.ffs_scale_fixed = getattr(a, "scale", None)
            self.lm.ffs_max_px = int(getattr(a, "ffs_max_px", 640 * 480) or 640 * 480)
            # 고정 VIO 설정(--ov-config)·외부 브리지면 그 크기 말고는 실시간 조합을 막는다 (REVIEW6 R1)
            vf = getattr(scanner, "vio_fixed_size", None)
            if callable(vf):
                try:
                    self.lm.vio_fixed_size = vf()
                except Exception as e:
                    scanner.log(f"[load] 고정 VIO 설정 크기를 못 읽음: {e!r}")
        except Exception as e:
            scanner.log(f"[load] 부하 모델을 못 켬: {e!r}")
        self.perf = None                   # 폰 metrics.json (adb, SPEC 14.3) 또는 receiver/scanner 의 perf
        self.perf_t = 0.0
        self.perf_wall = None              # 지난번 metrics.json 의 t_wall_ms (안 바뀌면 낡은 값 → 안 씀)
        self.sess_wall_ms = None           # 이번 세션 시작 시각 (ms) — 이보다 오래된 metrics.json 은 지난 세션 것 (REVIEW6 R4)
        self.caps_idle_t = 0.0             # 대기 중 caps.json 을 다시 찾아볼 시각 (폰 값이 아직 없을 때만)
        self.caps_try_t = 0.0              # 세션 시작 뒤 caps.json 을 읽어 볼 시각 (monotonic, 0 = 안 함)
        self.kf_min = None                 # 웹이 고른 키프레임 목표 (없으면 load_model.LIMITS)
        self.stream_fmt = None             # 웹이 고른 실시간 영상 형식 (없으면 scanner 값 → 기본 h264)
        self.h264_err = None               # 폰 metrics.json 의 h264.error (실시간 압축기 오류, REVIEW7 H2) — 세션 끝나면 지움
        self.h264_err_gen = None           # 그 오류를 낸 인코더 세대 (metrics h264.gen) — 세대가 바뀌면 지움
        self.phone_sync = None             # 폰 metrics.json 의 sync (렌즈 위상 맞추기, SPEC 15) — 세션 끝나면 지움
        self.sfm = None                    # 정밀 복원 SfmJob (마지막 것)
        threading.Thread(target=self._loop, daemon=True, name="ui-helper").start()

    # ---- 상태 ----
    def ui(self):
        t = self.temp_c
        lvl = None if t is None else ("hot" if t > TEMP_HOT else "warm" if t > TEMP_WARM else "ok")
        return dict(usb=self.usb, usb_age_s=round(time.time() - self.usb_t, 1) if self.usb_t else None,
                    temp_c=t, temp_level=lvl, temp_age_s=round(time.time() - self.temp_t, 1) if self.temp_t else None,
                    stream=self.stream(), load=self.mon.snapshot if self.mon else None, h264_error=self.h264_err,
                    sync=self.phone_sync, sfm=self.sfm.snapshot() if self.sfm else None,
                    relay=dict(state=self.relay_state, pid=self.relay_pid, error=self.relay_err,
                               starts=self.relay_starts, fails=self.relay_fails, wanted=self.want_relay,
                               log_tail=self._relay_tail()))

    def _relay_tail(self, n=3):
        try:
            with open(self.relay_log, "rb") as f:
                f.seek(0, 2)
                f.seek(max(0, f.tell() - 2000))
                lines = f.read().decode("utf-8", "replace").splitlines()
            return [ln for ln in lines if ln.strip()][-n:]
        except OSError:
            return []

    # ---- 화질·fps (load_model) ----
    def stream(self):
        """지금 세션(또는 다음 세션)의 크기·fps. scanner.stream dict (통합 뒤) > scanner.a.size / app_fps."""
        st = getattr(self.sc, "stream", None)
        a = getattr(self.sc, "a", None)
        if isinstance(st, dict) and st.get("w"):
            w, h, fps = int(st["w"]), int(st["h"]), int(st.get("fps") or getattr(a, "app_fps", 15))
        else:
            size = getattr(a, "size", None) or load_model.DEFAULT_SIZE
            w, h, fps = int(size[0]), int(size[1]), int(getattr(a, "app_fps", load_model.DEFAULT_FPS))
        calib = tuple(self.lm.calib) if self.lm else load_model.DEFAULT_SIZE
        sfmt = (st.get("stream_fmt") if isinstance(st, dict) else None) or getattr(a, "app_stream", None)
        d = dict(size=load_model.size_str(w, h), w=w, h=h, fps=fps, calib=load_model.size_str(*calib),
                 scaled_calib=(w, h) != calib, stream_fmt=load_model.norm_fmt(sfmt))
        if isinstance(st, dict):
            # 지금 세션의 실제 FFS 축소 비율 (부하 실측 환산용) · 보정 변환 문구 · 실시간 비트레이트
            d.update(ffs_scale=st.get("ffs_scale"), calib_note=st.get("calib_note"), epi_gate_px=st.get("epi_gate_px"),
                     live_kbps=st.get("live_kbps"))
        return d

    def _fmt_scale(self, running=False):
        """(형식, FFS 축소). running=True = 지금 도는 세션의 형식 (실측 비교용), 아니면 웹이 고른 형식 (다음 시작용)."""
        a = getattr(self.sc, "a", None)
        sc_ = getattr(a, "scale", None)                 # None = 자동 (크기마다, load_model.LoadModel.ffs_scale)
        fmt = self.stream()["stream_fmt"] if running or not self.stream_fmt else self.stream_fmt
        return load_model.norm_fmt(fmt), (float(sc_) if sc_ else None)

    def combos(self, mode, kf=None, fmt=None, n_rec=2):
        if self.lm is None:
            return dict(error="부하 모델 없음")
        if kf is not None:
            self.kf_min = kf
        if fmt:
            self.stream_fmt = load_model.norm_fmt(fmt)
        fmt, scale = self._fmt_scale()
        st = self.stream()
        return self.lm.table(mode, kf_min=self.kf_min, stream_fmt=fmt, scale=scale,
                             current=dict(size=st["size"], fps=st["fps"], stream_fmt=st["stream_fmt"]), n_rec=n_rec)

    def _apply_stream(self, body, mode):
        """[스캔 시작] 의 size·fps → 검사 후 적용. 반환 (거절 dict 또는 None, 경고 문자열 또는 None)."""
        if body.get("stream_fmt"):
            self.stream_fmt = load_model.norm_fmt(body["stream_fmt"])
        if self.lm is None or (body.get("size") is None and body.get("fps") is None and not body.get("stream_fmt")):
            return None, None
        st = self.stream()
        try:
            w, h = load_model.parse_size(body.get("size") or st["size"])
            fps = int(body.get("fps") or st["fps"])
        except (TypeError, ValueError):
            return dict(ok=False, error="화질·fps 값이 잘못됐습니다"), None
        fmt, scale = self._fmt_scale()
        n_rec = 3 if str(body.get("pair") or "") in ALL_LENS_PAIRS else 2   # '3개 모두 녹화 (비교용)' = 녹화 인코더 3개
        c = self.lm.combo(mode, w, h, fps, self.kf_min, fmt, scale, n_rec)
        warn = None
        if not c["ok"] and c.get("load_only"):
            # 부하 추정만 넘음 (장치 한계 아님) → 시작은 하고 경고만 (REVIEW6 R2: 튄 실측 하나로 시작이 막히지 않게)
            warn = "예상 부하가 높습니다 (" + " · ".join(c["reasons"]) + ") — 느리거나 끊기면 화질·fps 를 낮추세요"
        elif not c["ok"] and not body.get("force"):
            return dict(ok=False, error=f"{w}x{h} · {fps} fps ({load_model.FMT_INFO[fmt]}) 는 지금 쓸 수 없습니다: "
                        + " · ".join(c["reasons"]), reasons=c["reasons"]), None
        sc = self.sc
        a = getattr(sc, "a", None)
        if hasattr(sc, "set_stream"):
            # 통합(담당 2): 보정 비례 변환·정류 맵·수신 크기·앱 --ei width/height/fps/rec_kbps (+ --es stream·live_kbps)
            try:
                params = inspect.signature(sc.set_stream).parameters
            except (TypeError, ValueError):
                params = {}
            kw = dict(rec_kbps=c.get("rec_kbps"))
            if "stream_fmt" in params or any(p.kind == p.VAR_KEYWORD for p in params.values()):
                kw.update(stream_fmt=fmt, live_kbps=c.get("live_kbps"))
                if "live_kbps" not in params and not any(p.kind == p.VAR_KEYWORD for p in params.values()):
                    kw.pop("live_kbps")
                elif "live_kbps_auto" in params:
                    kw["live_kbps_auto"] = True        # 부하 모델이 계산한 기본값 (사용자가 직접 준 값 아님, REVIEW7 H4)
                sc.set_stream(w, h, fps, **kw)
            else:                                      # 옛 set_stream (형식 인자 없음)
                sc.set_stream(w, h, fps, **kw)
                if a is not None and hasattr(a, "app_stream"):
                    a.app_stream = fmt
        else:
            if a is not None:
                a.app_fps = fps
                if hasattr(a, "app_stream"):
                    a.app_stream = fmt
            if getattr(sc, "link", None) is not None:
                sc.link.fps = fps
            if (w, h) != (st["w"], st["h"]):
                warn = (warn + " / " if warn else "") + f"크기 바꾸기는 아직 스캐너에 연결되지 않아 {st['size']} 로 시작합니다 (fps 만 적용)"
        try:
            ph = (f"실시간 H.264 {c['live_kbps']} kbps × 2 · 폰 복사 {c['phone_cpu_pct']}%" if fmt == "h264"
                  else f"폰 부하 {c['phone_cpu_pct']}%")
            sc.log(f"[load] 시작 화질 {w}x{h} · {fps} fps · {load_model.FMT_INFO[fmt]} · 예상 {ph} · USB {c['usb_mbs']} MB/s"
                   f" · PC 깊이 {c['pc_ffs_fps']} /s" + (f" — {warn}" if warn else ""))
        except Exception:
            pass
        return None, warn

    def _load_tick(self):
        """감시 루프마다 (3 s): 폰 perf·PC 깊이 지표 → LoadMonitor (계수 EWMA, 과부하 신호).
        부하 모델이 꺼져 있어도 metrics.json 은 읽는다 (렌즈 위상·압축기 오류 표시, REVIEW8 L3)."""
        sc = self.sc
        mode = getattr(sc, "mode", "idle")
        link = getattr(sc, "link", None)
        now = time.monotonic()
        if mode not in load_model.MODES:
            self.perf = self.perf_wall = None
            self.sess_wall_ms = None
            self.h264_err = self.h264_err_gen = None
            self.phone_sync = None
            # 대기 중 1분마다 폰 caps.json 을 본다: 아직 없으면 찾고, 있으면 앱 버전·쓴 시각이 바뀌었을 때만 캐시 갱신
            # (앱을 새로 설치해도 옛 능력표를 계속 쓰지 않게, REVIEW6 R8)
            if self.mon is None:
                return
            if (self.usb == "device" and link is not None
                    and getattr(link, "available", False) and now >= self.caps_idle_t):
                self.caps_idle_t = now + 60.0
                self.lm.refresh_caps(link)
            self.lm.save()
            self.mon.update("idle", 0, 0, 0)
            return
        # 세션 시작 뒤 앱이 caps.json 을 썼으면 한 번 읽는다 (읽고 나면 다음 세션부터 폰 값으로)
        if self.lm is not None and self.caps_try_t and now >= self.caps_try_t and self.usb == "device" and link is not None:
            self.caps_try_t = 0.0
            self.lm.refresh_caps(link)
        rx = getattr(sc, "receiver", None)
        phone_st = {}
        if rx is not None:
            try:
                ps = rx.stats()
                # decode_ms / decode_ms_max = H.264 한 장 풀기 (가정한 이름, 수신 담당과 통합 때 맞춤)
                phone_st = dict(fps=ps.get("sensor_fps"), connected=True, decode_ms=ps.get("decode_ms"),
                                decode_ms_max=ps.get("decode_ms_max"))
                if isinstance(ps.get("perf"), dict) and ps["perf"]:
                    self.perf, self.perf_t = ps["perf"], now
            except Exception:
                pass
        sp = getattr(sc, "phone_perf", None)              # 통합 뒤 scanner 가 직접 들고 있으면 그것
        if isinstance(sp, dict) and sp:
            self.perf, self.perf_t = sp, now
        elif now - self.perf_t > 2.5 and self.usb == "device" and link is not None and getattr(link, "available", False):
            # 앱 1.3-caps metrics.json (1 s 마다 갱신, SPEC 14.3). t_wall_ms 가 지난번과 같으면 앱이 멈춘 것 → 안 씀
            j = load_model._read_phone_json(link, "metrics.json", timeout=5)
            tw = j.get("t_wall_ms") if isinstance(j, dict) else None
            if self.sess_wall_ms is None:
                self.sess_wall_ms = time.time() * 1000.0
            # 세션 시작 전에 쓴 파일 = 지난 세션 값 → 안 씀 (폰·PC 시계 차이 5 s 여유, REVIEW6 R4)
            old = isinstance(tw, (int, float)) and tw < self.sess_wall_ms - 5000.0
            self.perf = j if j and not old and (tw is None or tw != self.perf_wall) else None
            self.perf_wall = tw
            self.perf_t = now
        # 폰 실시간 압축기 오류 (metrics.json h264.error) → 웹 단계 표시줄에 진짜 원인 (REVIEW7 H2)
        # 새 metrics 에 h264.error 가 없거나 인코더 세대(h264.gen)가 바뀌면 지움 (REVIEW7 확인 H2-2)
        if isinstance(self.perf, dict):
            hj = self.perf.get("h264")
            if isinstance(hj, dict):
                err, gen = hj.get("error"), hj.get("gen")
                if self.h264_err is not None and gen is not None and gen != self.h264_err_gen:
                    self.h264_err = None
                if err:
                    self.h264_err, self.h264_err_gen = str(err)[:200], gen
                else:
                    self.h264_err = None
        # 렌즈 위상 맞추기 상태 (앱 1.5-sync metrics.json sync, SPEC 15.6) → 웹 단계 표시줄·고급
        sj = self.perf.get("sync") if isinstance(self.perf, dict) else None
        if isinstance(sj, dict):
            # events(긴 글) 만 빼고 그대로 (1.6-sync: 방법 C 의 다시 열기 횟수·한 번 걸린 시간·녹화 대기 등)
            self.phone_sync = {k: v for k, v in sj.items() if k != "events"}
            self.phone_sync["src"] = "metrics"
        else:
            # metrics.json 을 못 읽음·멈춤, 또는 sync 가 없는 옛 앱 → 낡은 위상 값은 안 보여 줌 (REVIEW8 L3)
            self.phone_sync = None
        if self.mon is None:
            return
        sizes = []
        if rx is not None:
            for f in list(getattr(rx, "latest", None) or []):
                j = getattr(f, "jpeg", None) if f is not None else None
                if j is not None:
                    sizes.append(len(j))
        st = self.stream()
        fmt, scale = self._fmt_scale(running=True)
        t = self.temp_c
        lvl = None if t is None else ("hot" if t > TEMP_HOT else "warm" if t > TEMP_WARM else "ok")
        if st.get("ffs_scale"):
            scale = float(st["ffs_scale"])             # 세션이 실제로 쓰는 FFS 축소 비율
        self.mon.update(mode, st["w"], st["h"], st["fps"], perf=self.perf, depth=dict(getattr(sc, "dstats", {}) or {}),
                        phone=phone_st, temp_level=lvl, jpeg_sizes=sizes if fmt == "jpeg" else None,
                        scale=scale, kf_min=self.kf_min, stream_fmt=fmt)

    # ---- 명령 ----
    def _sfm_start(self, name):
        sc = self.sc
        if self.sfm is not None and self.sfm.alive():
            return dict(ok=False, error=f"이미 정밀 복원 중 ({self.sfm.rec_dir.name})")
        if getattr(sc, "mode", "idle") != "idle":
            return dict(ok=False, error="스캔·녹화·복원 중에는 정밀 복원을 못 합니다 — 먼저 [■ 정지] 또는 복원이 끝나기를 기다리세요")
        rp = getattr(sc, "replayer", None)
        rs = getattr(sc, "recsync", None)
        if (rp is not None and rp.thread.is_alive()) or (rs is not None and rs.job is not None):
            return dict(ok=False, error="다른 복원이 도는 중 — 끝난 뒤 다시 누르세요")
        try:
            path = sc.resolve_rec(replay_view.safe_name(name))
        except ValueError as e:
            return dict(ok=False, error=str(e))
        if not path.is_dir():
            return dict(ok=False, error=f"녹화 폴더 없음: {name}")
        self.sfm = SfmJob(sc, path, sc.log)
        if rs is not None and rs.job is None:
            rs.job = dict(name=path.name, rp=self.sfm, t0=time.time(), pair=None)   # 자동 복원이 끝날 때까지 쉼
        return dict(ok=True, out=self.sfm.out.name, note="정밀 복원은 오래 걸립니다 (26 초 녹화에 약 5~10 분)")

    def command(self, body):
        cmd = body.get("cmd")
        sc = self.sc
        if cmd == "sfm_recon":
            return self._sfm_start(str(body.get("name") or ""))
        if cmd == "sfm_stop":
            if self.sfm is None or not self.sfm.alive():
                return dict(ok=False, error="정밀 복원 중이 아님")
            self.sfm.stop()
            return dict(ok=True)
        if cmd in ("mode", "replay") and self.sfm is not None and self.sfm.alive():
            return dict(ok=False, error="정밀 복원 중 (GPU 를 씀) — 끝나기를 기다리거나 [정밀 복원 멈춤]")
        if cmd == "load_reset":
            if self.lm is None:
                return dict(ok=False, error="부하 모델 없음")
            self.lm.reset()
            return dict(ok=True, coeffs=self.lm.coeffs_json())
        if cmd == "caps_refresh":
            if self.lm is None:
                return dict(ok=False, error="부하 모델 없음")
            link = getattr(sc, "link", None)
            src = self.lm.refresh_caps(link) if link is not None else self.lm.caps_src
            return dict(ok=True, caps_src=src)
        if cmd == "mode":
            mode = body.get("mode", "live")
            if str(body.get("pair") or "") in ALL_LENS_PAIRS and mode != "record":
                return dict(ok=False, error="렌즈 3개 모두 녹화는 '폰에 녹화만' 에서만 됩니다 — 실시간 깊이는 한 쌍을 고르세요")
            bad, warn = self._apply_stream(body, mode)
            if bad:
                return bad
            self.sess_wall_ms = time.time() * 1000.0     # 새 세션: 이보다 오래된 metrics.json 은 안 씀 (REVIEW6 R4)
            if self.lm is not None and not self.lm.caps_src.startswith("폰"):
                self.caps_try_t = time.monotonic() + 15.0
            if mode in ("live", "both"):
                self.relay_on()
            else:
                self.relay_off()
            r = sc.command(body)
            return dict(r, warn=warn) if warn and isinstance(r, dict) else r
        if cmd == "stop":
            r = sc.command(body)
            self.relay_off()
            return r
        if cmd == "mesh_set":
            kw = {}
            try:
                if body.get("voxel_m") is not None:
                    kw["voxel_m"] = float(min(max(float(body["voxel_m"]), 0.01), 0.05))
                if body.get("z_max") is not None:
                    kw["z_max"] = float(min(max(float(body["z_max"]), 0.6), 3.0))
                if body.get("enabled") is not None:
                    kw["enabled"] = bool(body["enabled"])
            except (TypeError, ValueError):
                return dict(ok=False, error="값이 숫자가 아닙니다")
            cfg = voxel_map.configure_tsdf(**kw)
            sc.vmap.reset_tsdf()                         # 새 설정은 새 메시 지도부터 (이미 넣은 깊이는 다시 안 넣음)
            return dict(ok=True, cfg=cfg)
        if cmd == "restart":
            # [처음부터 다시]: VIO(브리지)가 죽었으면 세션을 새로 (pipe 브리지 새 프로세스), 아니면 VIO 재시작 + 맵 초기화
            mode = getattr(sc, "mode", "idle")
            vio_err = None
            try:
                vio_err = sc.vio.stats().get("error") or getattr(sc, "vio_error", None)
            except Exception:
                pass
            if mode in ("live", "both") and vio_err:
                if self.relay_ok and self.relay_state != "켜짐":
                    self.relay_on()
                r = sc.start_session(mode, start_app=False)
                return dict(r, how="session")
            if mode in ("live", "both"):
                return dict(sc.command({"cmd": "reset_vio"}), how="reset_vio")
            return dict(sc.command({"cmd": "reset_map"}), how="reset_map")
        return sc.command(body)

    # ---- Windows 중계 ----
    def _ps(self, script, timeout=30):
        try:
            r = subprocess.run([self.ps, "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-Command", script],
                               capture_output=True, timeout=timeout, cwd="/mnt/c" if os.path.isdir("/mnt/c") else None)
            return r.returncode, (r.stdout + r.stderr).decode("utf-8", "replace").strip()
        except (OSError, subprocess.TimeoutExpired) as e:
            return -1, repr(e)

    def relay_on(self):
        self.want_relay = True
        self.relay_fails = 0
        if self.relay_ok:
            threading.Thread(target=self._relay_start, daemon=True, name="relay-start").start()

    def relay_off(self):
        self.want_relay = False
        if self.relay_ok:
            threading.Thread(target=self._relay_stop, daemon=True, name="relay-stop").start()

    def _relay_start(self):
        with self.lock:
            if not self.want_relay:
                return
            self.relay_state, self.relay_err = "켜는 중", None
            self.relay_next = time.monotonic() + 8.0
            script = _wslpath_w(HERE / "win_relay.py")
            adb = _wslpath_w(self.sc.link.adb)
            out = _wslpath_w(self.relay_log)
            if not (script and adb and out):
                self.relay_state, self.relay_err = "오류", "wslpath 실패 (WSL interop 확인)"
                return
            args = " ".join('"%s"' % x for x in ("-u", script, "--no-start-app", "--adb", adb))
            py_w, py_pre = _win_python()
            if not py_w:
                self.relay_state, self.relay_err = "오류", "Windows 에 Python 이 없습니다 (py.exe / python.exe)"
                self.relay_fails += 1
                self.relay_next = time.monotonic() + self._backoff()
                return
            ps = (self.KILL_ALL +
                  f"$py = {_psq(py_w)}; $pre = {_psq(py_pre)}; "
                  "$env:PYTHONIOENCODING = 'utf-8'; "
                  f"$p = Start-Process -FilePath $py -ArgumentList ($pre + {_psq(args)}) -WindowStyle Hidden "
                  f"-WorkingDirectory {_psq(_win_temp(py_w))} -RedirectStandardOutput {_psq(out)} "
                  f"-RedirectStandardError {_psq(out[:-4] + '_err.log')} -PassThru; "
                  "Write-Output ('PID ' + $p.Id)")
            rc, txt = self._ps(ps)
            m = re.search(r"PID (\d+)", txt)
            if m:
                self.relay_pid = int(m.group(1))
                self.relay_state = "켜짐"
                self.relay_starts += 1
                self.relay_t = time.monotonic()
                self.sc.log(f"[relay] Windows win_relay.py 시작 pid={self.relay_pid}")
            else:
                self.relay_pid = None
                self.relay_state = "오류"
                self.relay_fails += 1
                wait = self._backoff()
                self.relay_next = time.monotonic() + wait
                self.relay_err = ("Windows 에 Python 이 없습니다 (py.exe / python.exe)" if "no-python" in txt
                                  else f"중계 시작 실패: {txt[-200:]}")
                self.sc.log(f"[relay] {self.relay_err} — {wait:.0f} 초 뒤 다시 시도")

    def _backoff(self):
        """연속 실패 수 → 다음 시도까지 (s): 8 → 16 → 32 → 60 (B11)."""
        return min(60.0, 8.0 * (2 ** max(0, min(self.relay_fails - 1, 3))))

    def _relay_stop(self):
        with self.lock:
            if self.want_relay:
                return
            self._ps(self.KILL_ALL)
            if self.relay_pid:
                self.sc.log(f"[relay] Windows win_relay.py 정지 pid={self.relay_pid}")
            self.relay_pid = None
            self.relay_state = "꺼짐"

    def _relay_alive(self):
        if not self.relay_pid:
            return False
        rc, txt = self._ps(f"if (Get-Process -Id {self.relay_pid} -ErrorAction SilentlyContinue) "
                           "{ 'ALIVE' } else { 'DEAD' }", timeout=15)
        if "ALIVE" in txt:
            return True
        if "DEAD" in txt:
            return False
        return None

    # ---- 감시 루프 ----
    def _usb_poll(self):
        link = self.sc.link
        if link is None or not link.available:
            return "noadb"
        rc, out = link.run("devices", "-l", timeout=8)
        if rc != 0:
            return "모름"
        rows = [ln.split() for ln in out.splitlines()[1:] if len(ln.split()) >= 2]
        states = [r[1] for r in rows]
        if "device" in states:
            r = next(r for r in rows if r[1] == "device")
            tid = next((x.split(":", 1)[1] for x in r if x.startswith("transport_id:")), None)
            if tid is not None and self.transport is not None and tid != self.transport:
                self.replug_pending = True                # 3 s 사이에 뺐다 꽂음 (B2 a)
            self.transport = tid
            return "device"
        if "unauthorized" in states:
            return "unauthorized"
        return "offline" if states else "none"

    def _temp_poll(self):
        rc, out = self.sc.link.run("shell", "dumpsys", "thermalservice", timeout=10)
        if rc == 0:
            t = parse_skin_temp(out)
            if t is not None:
                self.temp_c = round(t, 1)
                self.temp_t = time.time()

    def _loop(self):
        last_temp = last_alive = 0.0
        while not getattr(self.sc, "stopped", False):
            try:
                self._load_tick()
            except Exception as e:
                try:
                    self.sc.log(f"[load] {e!r}")
                except Exception:
                    pass
            try:
                prev = self.usb
                self.usb = self._usb_poll()
                self.usb_t = time.time()
                if self.usb != "device":
                    if self.temp_t and time.time() - self.temp_t > 60:
                        self.temp_c = None
                else:
                    now = time.monotonic()
                    if now - last_temp >= 10.0:
                        last_temp = now
                        self._temp_poll()
                    if prev in ("none", "offline", "unauthorized") and self.usb == "device":
                        self.replug_pending = True                # 래치: relay_next 창 안이어도 잃지 않음 (B2 b)
                    if not (self.relay_ok and self.want_relay):
                        self.replug_pending = False
                    elif self.relay_state != "켜는 중":
                        replug = self.replug_pending
                        dead = False
                        if now - last_alive >= 12.0 and self.relay_state == "켜짐":   # powershell 을 덜 띄움 (B11)
                            last_alive = now
                            dead = self._relay_alive() is False
                            if dead:
                                # 막 켠 중계가 곧(30 s 안) 죽으면 연속 실패로 세고 간격을 늘린다 (B11).
                                # 오래 돌다 죽은 것은 바로 다시 띄운다.
                                if now - self.relay_t < 30.0:
                                    self.relay_fails += 1
                                    self.relay_next = max(self.relay_next, now + self._backoff())
                                else:
                                    self.relay_fails = 0
                                self.relay_pid = None
                                self.relay_state, self.relay_err = "오류", "중계가 꺼졌습니다 (로그 scan3d/runs/win_relay.log)"
                        if (replug or dead or self.relay_state == "오류") and now >= self.relay_next:
                            self.replug_pending = False
                            self.sc.log("[relay] " + ("USB 다시 연결 → 중계 다시 시작" if replug
                                                      else "중계가 꺼져 있음 → 다시 시작"))
                            self._relay_start()
            except Exception as e:
                try:
                    self.sc.log(f"[ui-helper] {e!r}")
                except Exception:
                    pass
            time.sleep(3.0)
