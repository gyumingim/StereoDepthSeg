#!/usr/bin/env python3
r"""VIO bench: live ARCore vs OpenVINS (on the phone) latency and pose page on the PC.

Runs on Windows Python (adb is a Windows exe); stdlib only:
  python \\\\wsl.localhost\\Ubuntu-22.04\\home\\system\\scanner\\StereoDepthSeg\\scan3d\\tools\\vio_bench_server.py
  -> http://localhost:8793

- Phone app: phone_stereo/arlat (com.scan3d.arlat). mode arcore | ov. It streams one JSON line per frame to
  127.0.0.1:8798 on the phone; `adb reverse tcp:8798 tcp:8798` (done here) brings it to this server.
- Start/Stop buttons run `adb shell am start/force-stop`. OpenVINS config + undistort map (make_phone_ov.py output)
  are pushed to the app folder the first time OpenVINS starts (or with the "설정 다시 보내기" button).
- Latency = t_out - t_sensor on the phone clock (sensor timestamp = start of exposure -> pose available in the app).
- Every run is saved to %LOCALAPPDATA%\scan3d_vio_bench\vio_bench_<time>_<mode>.jsonl (+ .summary.json); the page
  lists past runs. Not on the \\wsl.localhost share: per-line writes there stalled and froze the whole server
  (2026-10-18), so files are local and only the writer thread touches the disk, never under the hub lock.
"""
import argparse
import json
import os
import math
import queue
import statistics
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUNS = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "scan3d_vio_bench"
WRITES = queue.Queue()      # (path, line) appends, or (path, None, text) whole-file writes -> writer_loop
PKG = "com.scan3d.arlat"
PHONE_OV = f"/sdcard/Android/data/{PKG}/files/ov/"
OV_FILES = ["estimator_config.yaml", "kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml", "lens0u_remap.bin"]


def pct(v, q):
    if not v:
        return None
    s = sorted(v)
    return s[min(len(s) - 1, int(q * len(s)))]


class Run:
    """One phone run (mode arcore | ov): raw lines on disk + rolling stats."""

    def __init__(self, mode):
        self.mode = mode
        self.t0 = time.time()
        self.name = time.strftime("vio_bench_%Y%m%d-%H%M%S_", time.localtime(self.t0)) + mode
        RUNS.mkdir(parents=True, exist_ok=True)
        self.path_jsonl = RUNS / (self.name + ".jsonl")
        self.lat = []            # ms, whole run (tracking frames only)
        self.lat_all = []        # ms, every frame
        self.ms = []             # OpenVINS processing ms
        self.n = 0
        self.n_track = 0
        self.t_first = None
        self.t_last = None
        self.path = 0.0
        self.p_first = None
        self.p_last = None
        self.status = "-"
        self.events = deque(maxlen=20)

    def add(self, d):
        WRITES.put((self.path_jsonl, json.dumps(d) + "\n"))
        if "event" in d:
            self.events.append(d["event"])
            return
        self.n += 1
        st = d.get("status", "")
        self.status = st
        tracking = st in ("TRACKING", "tracking")
        if d.get("t_sensor") and d.get("t_out"):
            l = (d["t_out"] - d["t_sensor"]) / 1e6
            self.lat_all.append(l)
            if tracking:
                self.lat.append(l)
        if d.get("ms"):
            self.ms.append(d["ms"])
        t = d.get("t_sensor") or d.get("t_out")
        if t:
            self.t_first = self.t_first or t
            self.t_last = t
        p = d.get("p")
        if tracking and p and all(isinstance(x, (int, float)) and math.isfinite(x) for x in p):
            self.n_track += 1
            if self.p_last is not None:
                step = math.dist(p, self.p_last)
                if step < 0.5:
                    self.path += step
            self.p_first = self.p_first or p
            self.p_last = p
        pc = d.get("pc")      # ARCore pose after loop closure (arloop)
        if tracking and pc:
            self.pc_first = self.pc_first or pc
            self.pc_last = pc
        if d.get("loop"):
            self.loop = d["loop"]
            self.loop_mode = d.get("loop_mode")

    pc_first = pc_last = loop = loop_mode = None

    def stats(self):
        span = (self.t_last - self.t_first) / 1e9 if self.t_first and self.t_last and self.t_last > self.t_first else 0
        L = self.lat
        r = dict(mode=self.mode, name=self.name, frames=self.n, tracking=self.n_track,
                 tracking_pct=round(100 * self.n_track / self.n, 1) if self.n else None,
                 fps=round(self.n / span, 1) if span else None, secs=round(span, 1), status=self.status,
                 lat_med=round(statistics.median(L), 1) if L else None, lat_p90=round(pct(L, 0.9), 1) if L else None,
                 lat_max=round(max(L), 1) if L else None,
                 lat_recent_med=round(statistics.median(L[-90:]), 1) if L else None,
                 lat_all_med=round(statistics.median(self.lat_all), 1) if self.lat_all else None,
                 lat_all_recent_med=round(statistics.median(self.lat_all[-90:]), 1) if self.lat_all else None,
                 proc_med=round(statistics.median(self.ms), 1) if self.ms else None,
                 proc_p90=round(pct(self.ms, 0.9), 1) if self.ms else None,
                 path_m=round(self.path, 2),
                 end_from_start_m=round(math.dist(self.p_first, self.p_last), 3) if self.p_first and self.p_last else None,
                 corr_end_from_start_m=round(math.dist(self.pc_first, self.pc_last), 3) if self.pc_first and self.pc_last else None,
                 loop_mode=self.loop_mode, loop=self.loop,
                 events=list(self.events)[-5:])
        return r

    def close(self):
        s = self.stats()
        WRITES.put((RUNS / (self.name + ".summary.json"), None, json.dumps(s, ensure_ascii=False, indent=1)))
        return s


def writer_loop():
    """The only thread that touches the disk: appends run lines, writes summaries (and closes that run's file)."""
    files = {}
    while True:
        item = WRITES.get()
        try:
            if len(item) == 3:
                item[0].write_text(item[2], encoding="utf-8")
                f = files.pop(str(item[0]).replace(".summary.json", ".jsonl"), None)
                if f:
                    f.close()
                continue
            path, line = item
            f = files.get(str(path))
            if f is None:
                f = files[str(path)] = open(path, "a", encoding="utf-8")
            f.write(line)
            if WRITES.empty():
                f.flush()
        except Exception as e:
            print("writer:", e, flush=True)


class Hub:
    def __init__(self, a):
        self.a = a
        self.run = None
        self.lock = threading.Lock()
        self.cmd_lock = threading.Lock()
        self.clients = []
        self.phone_connected = False
        self.ov_pushed = False
        self.log = deque(maxlen=50)

    def background(self, fn, *args):
        """adb commands off the HTTP thread, one at a time (double clicks queue up instead of racing)."""
        def run():
            with self.cmd_lock:
                try:
                    out = fn(*args)
                    if out:
                        self.say(out.splitlines()[-1][:200])
                except Exception as e:
                    self.say(f"오류: {e}")
        threading.Thread(target=run, daemon=True).start()

    def say(self, msg):
        self.log.append(time.strftime("%H:%M:%S ") + msg)
        self.broadcast({"type": "log", "msg": msg})

    def adb(self, *args, timeout=60):
        r = subprocess.run([self.a.adb, *args], capture_output=True, text=True, timeout=timeout)
        return (r.stdout + r.stderr).strip()

    def broadcast(self, d):
        s = json.dumps(d)
        for q in list(self.clients):
            try:
                q.put_nowait(s)
            except queue.Full:
                pass

    def on_line(self, d):
        with self.lock:
            if self.run is None or self.run.mode != d.get("src"):
                if self.run is not None:
                    self.run.close()
                self.run = Run(d.get("src", "?"))
            self.run.add(d)
        d = dict(d, type="frame")
        self.broadcast(d)

    def start(self, mode, secs=0, loop="both", mlen=0.15, mdict=0, dump=0, exp_ms=0.0):
        """loop = both | marker | feature | off (ARCore loop closure, arlat ArLoop); mlen = printed marker side (m)."""
        self.stop(quiet=True)
        out = [self.adb("reverse", "tcp:8798", "tcp:8798")]
        if mode == "ov" and not self.ov_pushed:
            out.append(self.push_ov())
        self.adb("shell", "input", "keyevent", "KEYCODE_WAKEUP")
        out.append(self.adb("shell", "am", "start", "-n", f"{PKG}/.MainActivity", "--es", "mode", mode, "--ei", "secs", str(secs),
                            "--es", "loop", str(loop), "--ef", "mlen", str(float(mlen)), "--ei", "mdict", str(int(mdict)),
                            "--ei", "dump", str(int(dump)), "--ei", "exp_cap_us", str(int(float(exp_ms) * 1000))))
        self.say(f"시작: {mode}" + (f" (루프 {loop}, 마커 {mlen} m)" if mode == "arcore" else ""))
        return "\n".join(o for o in out if o)

    def stop(self, quiet=False):
        o = self.adb("shell", "am", "force-stop", PKG)
        s = None
        with self.lock:
            if self.run is not None:
                s = self.run.close()
                self.run = None
        if s is not None:
            self.broadcast({"type": "run_end", "summary": s})
        if not quiet:
            self.say("멈춤")
        return o

    def hs_start(self, fps=240):
        """고속 정답 녹화 (arlat mode hs): 주 카메라 1920x1080 fps + IMU → 폰 files/hs_<시각>/."""
        self.stop(quiet=True)
        self.adb("shell", "input", "keyevent", "KEYCODE_WAKEUP")
        o = self.adb("shell", "am", "start", "-n", f"{PKG}/.MainActivity", "--es", "mode", "hs", "--ei", "fps", str(int(fps)))
        self.say(f"고속 녹화 시작: {fps} fps (멈추면 PC 로 가져옴)")
        return o

    def hs_stop(self):
        """홈 키 → 앱이 onPause 에서 파일을 닫음 → 끝낸 뒤 가장 최근 hs_ 폴더를 PC recordings/ 로 가져옴."""
        self.adb("shell", "input", "keyevent", "KEYCODE_HOME")
        time.sleep(3)
        self.adb("shell", "am", "force-stop", PKG)
        base = f"/sdcard/Android/data/{PKG}/files"
        name = self.adb("shell", f"ls -d {base}/hs_* | tail -1").strip().split("/")[-1]
        if not name.startswith("hs_"):
            self.say("고속 녹화 폴더를 못 찾음")
            return ""
        dst = Path(self.a.rec_dir)
        self.say(f"가져오는 중: {name} → {dst}")
        o = self.adb("pull", f"{base}/{name}", str(dst), timeout=3600)
        self.say(f"가져옴: {name} ({o.splitlines()[-1][:120] if o else ''})")
        return o

    def push_ov(self):
        self.adb("shell", "mkdir", "-p", PHONE_OV)
        outs = []
        for f in OV_FILES:
            outs.append(self.adb("push", str(Path(self.a.ov_dir) / f), PHONE_OV, timeout=300).splitlines()[-1:])
        self.ov_pushed = True
        self.say("OpenVINS 설정 보냄")
        return "\n".join(o[0] for o in outs if o)

    def runs(self):
        out = []
        for p in sorted(RUNS.glob("vio_bench_*.summary.json"), reverse=True)[:30]:
            try:
                out.append(json.loads(p.read_text(encoding="utf-8")))
            except Exception:
                pass
        return out


def phone_listener(hub, port):
    import socket
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(2)
    while True:
        c, _ = srv.accept()   # one thread per connection: an old half-dead phone socket must not block the new one
        threading.Thread(target=phone_conn, args=(hub, c), daemon=True).start()


def phone_conn(hub, c):
    hub.phone_connected = True
    hub.say("폰 연결됨")
    c.settimeout(10)
    buf = b""
    try:
        while True:
            b = c.recv(65536)
            if not b:
                break
            buf += b
            *lines, buf = buf.split(b"\n")
            for l in lines:
                if l.strip():
                    try:
                        hub.on_line(json.loads(l))
                    except Exception as e:
                        hub.say(f"잘못된 줄: {e}")
    except OSError:
        pass
    finally:
        c.close()
        hub.phone_connected = False
        hub.say("폰 연결 끊김")


def stats_ticker(hub):
    while True:
        time.sleep(1.0)
        try:
            with hub.lock:
                s = hub.run.stats() if hub.run else None
            hub.broadcast({"type": "stats", "stats": s, "phone": hub.phone_connected})
        except Exception as e:
            print("stats:", e, flush=True)


def make_handler(hub):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            b = json.dumps(obj, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path == "/":
                b = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path == "/api/runs":
                self._json(hub.runs())
            elif self.path == "/events":
                q = queue.Queue(maxsize=2000)
                hub.clients.append(q)
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    for m in list(hub.log)[-10:]:
                        self.wfile.write(f"data: {json.dumps({'type': 'log', 'msg': m})}\n\n".encode())
                    while True:
                        try:
                            s = q.get(timeout=15)
                        except queue.Empty:
                            s = '{"type":"ping"}'
                        self.wfile.write(f"data: {s}\n\n".encode())
                        self.wfile.flush()
                except OSError:
                    pass
                finally:
                    hub.clients.remove(q)
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
            try:
                if self.path == "/api/start":
                    hub.background(hub.start, body.get("mode", "arcore"), int(body.get("secs", 0)), body.get("loop", "both"),
                                   float(body.get("mlen", 0.15)), int(body.get("mdict", 0)), int(body.get("dump", 0)),
                                   float(body.get("exp_ms", 0)))
                    self._json({"ok": True})
                elif self.path == "/api/stop":
                    hub.background(hub.stop)
                    self._json({"ok": True})
                elif self.path == "/api/hs_start":
                    hub.background(hub.hs_start, int(body.get("fps", 240)))
                    self._json({"ok": True})
                elif self.path == "/api/hs_stop":
                    hub.background(hub.hs_stop)
                    self._json({"ok": True})
                elif self.path == "/api/push_ov":
                    hub.background(hub.push_ov)
                    self._json({"ok": True})
                else:
                    self._json({"error": "not found"}, 404)
            except Exception as e:
                self._json({"ok": False, "error": str(e)}, 500)
    return H


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>VIO 지연 비교</title>
<style>
:root{--bg:#f6f7f9;--fg:#1d2128;--mut:#6b7280;--card:#fff;--line:#e3e6ea;--ar:#2563eb;--ov:#dc2626;--lc:#16a34a}
@media (prefers-color-scheme:dark){:root{--bg:#14161a;--fg:#e8eaed;--mut:#9aa0a6;--card:#1d2026;--line:#2c3038;--ar:#60a5fa;--ov:#f87171;--lc:#4ade80}}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,sans-serif}
main{max-width:1200px;margin:0 auto;padding:16px}
h1{font-size:20px;margin:0 0 12px}
.row{display:flex;gap:12px;flex-wrap:wrap}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px;flex:1;min-width:280px}
button{font:inherit;padding:8px 14px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--fg);cursor:pointer}
button.ar{border-color:var(--ar);color:var(--ar)} button.ov{border-color:var(--ov);color:var(--ov)}
.big{font-size:32px;font-weight:700} .mut{color:var(--mut)} canvas{width:100%;height:260px;display:block}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums} th,td{text-align:right;padding:4px 6px;border-bottom:1px solid var(--line)} th:first-child,td:first-child{text-align:left}
#log{font:12px ui-monospace,monospace;color:var(--mut);max-height:120px;overflow:auto;white-space:pre-wrap}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px;vertical-align:middle}
</style></head><body><main>
<h1>VIO 지연 비교 — ARCore vs OpenVINS (폰에서 실시간)</h1>
<div class="row" style="margin-bottom:12px">
 <div class="card" style="flex:2">
  <button class="ar" onclick="start('arcore')">ARCore 시작</button>
  <button class="ov" onclick="start('ov')">OpenVINS 시작</button>
  <button onclick="post('/api/stop')">멈춤</button>
  <button onclick="post('/api/push_ov')">OpenVINS 설정 다시 보내기</button>
  <div style="margin-top:8px">보정판 정답 녹화 (고속, 주 카메라 1920x1080 + IMU)
   <select id="hsfps"><option>240</option><option>120</option></select> fps
   <button class="ar" onclick="post('/api/hs_start',{fps:+document.getElementById('hsfps').value})">● 고속 녹화 시작</button>
   <button onclick="post('/api/hs_stop')">■ 멈추고 PC로 가져오기</button>
   <span class="mut">고속 모드는 폰 화면 미리보기·마커 표시 없음</span></div>
  <span id="phone" class="mut" style="margin-left:8px">폰: -</span>
  <div style="margin-top:8px">ARCore 루프 클로저
   <select id="loopmode"><option value="both">둘 다 (마커 + 특징점)</option><option value="marker">마커만</option><option value="feature">특징점만 (마커 없음)</option><option value="off">끔</option></select>
   마커 한 변 <input id="mlen" type="number" step="0.005" value="0.15" style="width:70px"> m (ArUco 4x4_50)
   <label><input id="dump" type="checkbox"> 키프레임 저장 (PC 재생용)</label>
   · OpenVINS 노출 상한 <input id="expms" type="number" step="0.5" min="0" value="2" style="width:56px"> ms (0 = 자동, 짧을수록 덜 번짐)</div>
  <div class="mut" style="margin-top:6px">지연 = 촬영(센서 시각) → 앱에서 위치가 나온 시각, 폰 시계 하나로 잼. 한 번에 하나만 돌아감 (카메라 공유 안 됨).</div>
 </div>
</div>
<div class="row">
 <div class="card"><div class="mut">지금 모드</div><div class="big" id="mode">-</div><div id="status" class="mut">-</div></div>
 <div class="card"><div class="mut">지연 (최근 3초 중앙값)</div><div class="big" id="latnow">-</div><div class="mut" id="latall">-</div></div>
 <div class="card"><div class="mut">추적 · 속도</div><div class="big" id="trk">-</div><div class="mut" id="fps">-</div></div>
</div>
<div class="row" style="margin-top:12px">
 <div class="card" style="flex:2"><div class="mut"><span class="dot" style="background:var(--ar)"></span>ARCore <span class="dot" style="background:var(--ov);margin-left:10px"></span>OpenVINS — 프레임별 지연 (ms, 최근 20초)</div><canvas id="lat"></canvas></div>
 <div class="card"><div class="mut">궤적 (위에서 본 것, m) — <span style="color:var(--ar)">ARCore</span> · <span style="color:var(--lc)">루프 보정</span> · <span style="color:var(--ov)">OpenVINS</span></div><canvas id="traj"></canvas><div class="mut" id="path">-</div></div>
</div>
<div class="card" style="margin-top:12px"><div class="mut">루프 클로저</div><div id="loopst" style="font-variant-numeric:tabular-nums">-</div><div id="loopev" class="mut" style="font:12px ui-monospace,monospace;white-space:pre-wrap"></div></div>
<div class="card" style="margin-top:12px"><div class="mut">기록 (끝난 측정)</div>
 <table id="runs"><thead><tr><th>측정</th><th>초</th><th>프레임</th><th>추적 %</th><th>fps</th><th>지연 중앙</th><th>p90</th><th>최대</th><th>계산 ms</th><th>이동 m</th><th>시작↔끝 m</th><th>보정 시작↔끝 m</th><th>루프</th></tr></thead><tbody></tbody></table></div>
<div class="card" style="margin-top:12px"><div class="mut">로그</div><div id="log"></div></div>
</main><script>
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const pts={arcore:[],ov:[]}, traj={arcore:[],ov:[],lc:[]};
function post(u,b){return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})}).then(r=>r.json()).then(j=>{if(j.out)log(j.out);if(j.error)log('오류: '+j.error);loadRuns();});}
function start(m){traj[m]=[];if(m==='arcore')traj.lc=[];post('/api/start',{mode:m,loop:document.getElementById('loopmode').value,mlen:parseFloat(document.getElementById('mlen').value)||0.15,dump:document.getElementById('dump').checked?1:0,exp_ms:parseFloat(document.getElementById('expms').value)||0});}
function log(s){const e=document.getElementById('log');e.textContent=(s+'\n'+e.textContent).slice(0,6000);}
function fmt(v,u=''){return v==null?'-':v+u}
const es=new EventSource('/events');
es.onmessage=ev=>{const d=JSON.parse(ev.data);
 if(d.type==='frame'&&!d.event){const now=performance.now()/1000;
  if(d.t_sensor&&d.t_out){pts[d.src].push([now,(d.t_out-d.t_sensor)/1e6,/track/i.test(d.status)]);}
  if(d.p&&/track/i.test(d.status)){const p=d.p; traj[d.src].push(d.src==='arcore'?[p[0],-p[2]]:[p[0],p[1]]); if(traj[d.src].length>20000)traj[d.src].shift();}
  if(d.pc&&/track/i.test(d.status)){traj.lc.push([d.pc[0],-d.pc[2]]); if(traj.lc.length>20000)traj.lc.shift();}
 } else if(d.type==='frame'&&d.event){log('OpenVINS: '+JSON.stringify(d.event).slice(0,200));}
 else if(d.type==='stats'){const s=d.stats; document.getElementById('phone').textContent='폰: '+(d.phone?'연결됨':'연결 안 됨');
  if(!s)return; document.getElementById('mode').textContent=s.mode==='ov'?'OpenVINS':'ARCore';
  document.getElementById('mode').style.color=css(s.mode==='ov'?'--ov':'--ar');
  const ev=(s.events||[]).filter(e=>e.type!=='ready');
  document.getElementById('status').textContent=(s.status==='init'?'초기화 대기 (가만히 1~2초 뒤 움직이기)':s.status)+(ev.length?' · '+JSON.stringify(ev[ev.length-1]).slice(0,80):'');
  document.getElementById('latnow').textContent=s.lat_recent_med!=null?s.lat_recent_med+' ms':(s.lat_all_recent_med!=null?s.lat_all_recent_med+' ms (추적 전)':'-');
  document.getElementById('latall').textContent='전체 중앙 '+fmt(s.lat_med)+' · p90 '+fmt(s.lat_p90)+' · 최대 '+fmt(s.lat_max)+' ms'+(s.proc_med!=null?' · 계산 '+s.proc_med+' ms':'');
  document.getElementById('trk').textContent=fmt(s.tracking_pct,' %');
  document.getElementById('fps').textContent=fmt(s.fps,' fps')+' · '+s.frames+' 프레임 · '+s.secs+' 초';
  document.getElementById('path').textContent='이동 '+s.path_m+' m · 시작↔지금 '+fmt(s.end_from_start_m,' m')+(s.corr_end_from_start_m!=null?' · 보정 시작↔지금 '+s.corr_end_from_start_m+' m':'');
  const L=s.loop; if(L){document.getElementById('loopst').textContent=(s.loop_mode||'')+' · 키프레임 '+L.kf+' (깊이 있음 '+L.depth_kf+') · 특징점 루프 '+L.loops+' · 마커 '+L.markers+'개 / 관측 '+L.marker_edges+' · 최적화 '+L.opt+'회 ('+L.opt_ms+' ms) · 키프레임 처리 '+L.proc_ms+' ms · 보정 yaw '+L.corr_yaw_deg.toFixed(2)+'° 이동 '+L.corr_t.map(v=>v.toFixed(2)).join(', ')+' m';
   document.getElementById('loopev').textContent=(L.events||[]).slice().reverse().join('\n');}}
 else if(d.type==='run_end'){loadRuns();}
 else if(d.type==='log'){log(d.msg);}
};
function draw(){const c=document.getElementById('lat'),x=c.getContext('2d');const W=c.width=c.clientWidth*devicePixelRatio,H=c.height=c.clientHeight*devicePixelRatio;
 const now=performance.now()/1000,span=20; let ymax=150; for(const k in pts){pts[k]=pts[k].filter(p=>now-p[0]<span);for(const p of pts[k])ymax=Math.max(ymax,p[1]*1.1);}
 x.clearRect(0,0,W,H);x.strokeStyle=css('--line');x.fillStyle=css('--mut');x.font=(11*devicePixelRatio)+'px sans-serif';
 for(let v=0;v<=ymax;v+=50){const y=H-v/ymax*H;x.beginPath();x.moveTo(0,y);x.lineTo(W,y);x.stroke();x.fillText(v+'',4,y-2);}
 for(const k of ['arcore','ov']){x.fillStyle=css(k==='ov'?'--ov':'--ar');for(const p of pts[k]){const X=W-(now-p[0])/span*W,Y=H-p[1]/ymax*H;x.globalAlpha=p[2]?1:.3;x.fillRect(X-2,Y-2,4,4);}}
 x.globalAlpha=1;
 const t=document.getElementById('traj'),y=t.getContext('2d');const TW=t.width=t.clientWidth*devicePixelRatio,TH=t.height=t.clientHeight*devicePixelRatio;
 y.clearRect(0,0,TW,TH);let all=[...traj.arcore,...traj.ov,...traj.lc];if(all.length){let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;for(const p of all){x0=Math.min(x0,p[0]);x1=Math.max(x1,p[0]);y0=Math.min(y0,p[1]);y1=Math.max(y1,p[1]);}
  const s=Math.min(TW/Math.max(x1-x0,1),TH/Math.max(y1-y0,1))*0.9,cx=(x0+x1)/2,cy=(y0+y1)/2;
  y.strokeStyle=css('--line');y.strokeRect(TW/2+(Math.floor(x0)-cx)*s,TH/2-(Math.ceil(y1)-cy)*s,(Math.ceil(x1)-Math.floor(x0))*s,(Math.ceil(y1)-Math.floor(y0))*s);
  for(const k of ['arcore','ov','lc']){const P=traj[k];if(!P.length)continue;y.strokeStyle=css(k==='ov'?'--ov':k==='lc'?'--lc':'--ar');y.lineWidth=2*devicePixelRatio;y.beginPath();P.forEach((p,i)=>{const X=TW/2+(p[0]-cx)*s,Y=TH/2-(p[1]-cy)*s;i?y.lineTo(X,Y):y.moveTo(X,Y);});y.stroke();}}
 requestAnimationFrame(draw);}
requestAnimationFrame(draw);
function loadRuns(){fetch('/api/runs').then(r=>r.json()).then(R=>{const b=document.querySelector('#runs tbody');b.innerHTML='';for(const s of R){const tr=document.createElement('tr');
 const c=[(s.mode==='ov'?'OpenVINS ':'ARCore ')+s.name.slice(10,25),s.secs,s.frames,s.tracking_pct,s.fps,s.lat_med,s.lat_p90,s.lat_max,s.proc_med,s.path_m,s.end_from_start_m,s.corr_end_from_start_m,s.loop?((s.loop_mode||'')+' '+s.loop.loops+'/'+s.loop.marker_edges):null];
 c.forEach((v,i)=>{const td=document.createElement('td');td.textContent=v==null?'-':v;if(i===0)td.style.color=css(s.mode==='ov'?'--ov':'--ar');tr.appendChild(td);});b.appendChild(tr);}});}
loadRuns();
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8793)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--phone-port", type=int, default=8798)
    ap.add_argument("--adb", default=r"C:\Users\a3162\platform-tools\adb.exe")
    ap.add_argument("--rec-dir", default=r"\\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg\scan3d\recordings", help="고속 녹화를 가져올 곳")
    ap.add_argument("--ov-dir", default=r"\\wsl.localhost\Ubuntu-22.04\home\system\scanner\android\phone_ov",
                    help="make_phone_ov.py output (config + lens0u_remap.bin)")
    a = ap.parse_args()
    hub = Hub(a)
    threading.Thread(target=phone_listener, args=(hub, a.phone_port), daemon=True).start()
    threading.Thread(target=stats_ticker, args=(hub,), daemon=True).start()
    threading.Thread(target=writer_loop, daemon=True).start()
    print(hub.adb("reverse", "tcp:8798", "tcp:8798"))
    print(f"http://localhost:{a.port}")
    ThreadingHTTPServer((a.host, a.port), make_handler(hub)).serve_forever()


if __name__ == "__main__":
    main()
