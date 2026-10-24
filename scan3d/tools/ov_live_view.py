#!/usr/bin/env python3
r"""Live OpenVINS path viewer on the PC (phone app arlat, mode ov).

Runs on Windows Python (adb is a Windows exe); stdlib only:
  C:\Users\a3162\AppData\Local\Programs\Python\Python312\pythonw.exe \\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg\scan3d\tools\ov_live_view.py
  -> http://100.92.214.97:8798   (Tailscale IP of this PC; --host / --port to change)
  WSL: LIVE=1 bash scan3d/tools/ov_phone_run.sh run <ip:port> <cfg> <secs>  starts it if needed (see that script).

- The phone's Streamer.java sends one JSON line per pose to 127.0.0.1:8798 on the phone. 127.0.0.1:8798 on the PC
  is already taken by vio_bench_server.py, so this viewer listens on 127.0.0.1:8794 (--stream-port) and the phone
  port is mapped with `adb reverse tcp:8798 tcp:8794` (done at start for --serial, by /api/reverse, and by
  ov_phone_run.sh LIVE=1). Note: vio_bench_server.py sets `tcp:8798 tcp:8798` again when it starts the phone.
- Several / repeated Streamer connections are fine (reconnects every second). A new session (cleared path, new file)
  starts on /api/new?tag=..., or by itself when t_out jumps back or forward by > 10 s.
- Every received line is saved as is to %USERPROFILE%\scan3d_ov_live\ov_live_<time>[_tag].jsonl (local disk: writes
  on the \\wsl.localhost share froze vio_bench_server, 2026-10-18). /api/session gives the path; ov_phone_run.sh
  LIVE=1 copies it into the run folder as ov_live.jsonl (poses.csv has no positions).
- Page: top view (x-y) and height (z) over time, metre grid, colored by status; polls /api/state ~10 Hz.
"""
import argparse
import json
import math
import os
import queue
import socketserver
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

# %USERPROFILE%\scan3d_ov_live, not under AppData: Microsoft Store Python silently redirects AppData writes into its
# package folder (2026-10-24: saved files "missing"). Started from WSL the Windows env can be thin -> fixed fallback.
RUNS = Path(os.environ.get("OV_LIVE_DIR") or os.environ.get("USERPROFILE", r"C:\Users\a3162")) / "scan3d_ov_live"
ADB_DEFAULT = r"C:\Users\a3162\platform-tools\adb.exe"
GAP_NS = 10e9            # t_out jump that starts a new session by itself
LOG = None


def log(*a):
    s = time.strftime("%H:%M:%S ") + " ".join(str(x) for x in a)
    if sys.stdout is not None:
        try:
            print(s, flush=True)
        except Exception:
            pass
    try:
        RUNS.mkdir(parents=True, exist_ok=True)
        with open(RUNS / "ov_live_view.log", "a", encoding="utf-8") as f:
            f.write(s + "\n")
    except Exception:
        pass


def finite(v):
    return isinstance(v, (int, float)) and math.isfinite(v)


class Writer(threading.Thread):
    """Only this thread touches the disk: ('open', path) / ('line', text) / ('close',) / ('flush', Event)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.q = queue.Queue()
        self.f = None

    def run(self):
        last_flush = time.time()
        while True:
            try:
                m = self.q.get(timeout=0.5)
            except queue.Empty:
                m = None
            try:
                if m is not None:
                    if m[0] == "open":
                        if self.f:
                            self.f.close()
                        m[1].parent.mkdir(parents=True, exist_ok=True)
                        self.f = open(m[1], "a", encoding="utf-8", newline="\n")
                    elif m[0] == "close":
                        if self.f:
                            self.f.close()
                        self.f = None
                    elif m[0] == "line" and self.f:
                        self.f.write(m[1] + "\n")
                    elif m[0] == "flush":
                        if self.f:
                            self.f.flush()
                        m[1].set()
                if self.f and time.time() - last_flush > 0.5:
                    self.f.flush()
                    last_flush = time.time()
            except Exception as e:
                log("writer:", e)


class Hub:
    def __init__(self, writer):
        self.lock = threading.Lock()
        self.w = writer
        self.sid = 0
        self.conns = 0
        self.n_conn_total = 0
        self.new_session("")

    def new_session(self, tag):
        with self.lock:
            self._new(tag)

    def _new(self, tag):
        self.sid += 1
        self.tag = "".join(c for c in tag if c.isalnum() or c in "-_.")[:40]
        self.path = None           # file opened at the first line
        self.n_lines = 0
        self.poses = []            # [t_rel_s, x, y, z, status_code]  status: 0 init 1 tracking 2 lost 3 other
        self.ms = []               # (t_out_ns, ms)
        self.t_first = None
        self.t_last = None
        self.pc_last = None
        self.last = {}
        self.events = []
        self.path_len = 0.0
        self.start_xyz = None
        self.prev_track = None
        self.counts = {"init": 0, "tracking": 0, "lost": 0, "other": 0}
        self.w.q.put(("close",))

    def add(self, line):
        line = line.strip()
        if not line:
            return
        try:
            j = json.loads(line)
        except Exception:
            j = None
        with self.lock:
            t = j.get("t_out") if isinstance(j, dict) else None
            if finite(t) and self.t_last is not None and (t < self.t_last - 1e9 or t > self.t_last + GAP_NS):
                log(f"t_out jump {(t - self.t_last) / 1e9:+.1f} s -> new session")
                self._new(self.tag)
            if self.path is None:
                name = time.strftime("ov_live_%Y%m%d-%H%M%S") + (f"_{self.tag}" if self.tag else "") + ".jsonl"
                self.path = RUNS / name
                self.w.q.put(("open", self.path))
                log("saving ->", self.path)
            self.w.q.put(("line", line))
            self.n_lines += 1
            self.pc_last = time.time()
            if not isinstance(j, dict):
                return
            if "event" in j:
                ev = j["event"]
                self.events.append([time.strftime("%H:%M:%S"), json.dumps(ev, ensure_ascii=False)[:200]])
                self.events = self.events[-20:]
                return
            if not finite(t):
                return
            if self.t_first is None:
                self.t_first = t
            self.t_last = t
            st = str(j.get("status", ""))
            code = {"init": 0, "tracking": 1, "lost": 2}.get(st, 3)
            self.counts[st if st in self.counts else "other"] += 1
            ms = j.get("ms")
            if finite(ms):
                self.ms.append((t, float(ms)))
            self.last = {k: j.get(k) for k in ("status", "ms", "n_slam", "n_msckf", "cam_q_drop", "pool_drop", "dt_cam_imu")}
            p = j.get("p")
            if isinstance(p, list) and len(p) >= 3 and all(finite(v) for v in p[:3]):
                x, y, z = (float(v) for v in p[:3])
                self.poses.append([round((t - self.t_first) / 1e9, 3), round(x, 4), round(y, 4), round(z, 4), code])
                if code == 1:
                    if self.start_xyz is None:
                        self.start_xyz = (x, y, z)
                    if self.prev_track is not None:
                        self.path_len += math.dist(self.prev_track, (x, y, z))
                    self.prev_track = (x, y, z)
                else:
                    self.prev_track = None

    def state(self, since, sid):
        with self.lock:
            if sid != self.sid:
                since = 0
            since = max(0, min(since, len(self.poses)))
            now_t = self.t_last
            ms5 = [m for (tt, m) in self.ms if now_t is not None and tt >= now_t - 5e9]
            p95 = sorted(ms5)[min(len(ms5) - 1, int(0.95 * len(ms5)))] if ms5 else None
            rate = None
            if len(self.ms) >= 2:
                k = [tt for (tt, _) in self.ms if tt >= now_t - 5e9]
                if len(k) >= 2 and k[-1] > k[0]:
                    rate = (len(k) - 1) / ((k[-1] - k[0]) / 1e9)
            dist = None
            cur = self.poses[-1] if self.poses else None
            if self.start_xyz and cur:
                dist = math.dist(self.start_xyz, cur[1:4])
            return {
                "sid": self.sid, "tag": self.tag, "n": len(self.poses), "since": since,
                "poses": self.poses[since:since + 20000],
                "conns": self.conns, "n_conn_total": self.n_conn_total, "n_lines": self.n_lines,
                "age_s": None if self.pc_last is None else round(time.time() - self.pc_last, 1),
                "elapsed_s": None if self.t_first is None else round((self.t_last - self.t_first) / 1e9, 2),
                "last": self.last, "ms_p95_5s": p95, "rate_hz": None if rate is None else round(rate, 1),
                "path_len": round(self.path_len, 3), "dist_start": None if dist is None else round(dist, 3),
                "counts": self.counts, "events": self.events[-5:], "file": str(self.path) if self.path else None,
            }


class StreamHandler(socketserver.StreamRequestHandler):
    def handle(self):
        hub = self.server.hub
        with hub.lock:
            hub.conns += 1
            hub.n_conn_total += 1
        log("phone connected", self.client_address)
        try:
            for raw in self.rfile:
                hub.add(raw.decode("utf-8", "replace"))
        except Exception as e:
            log("stream:", e)
        finally:
            with hub.lock:
                hub.conns -= 1
            log("phone disconnected", self.client_address)


class StreamServer(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = False     # Windows SO_REUSEADDR would let a second copy bind the same port


class HttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = False


def adb_reverse(adb, serial, phone_port, stream_port):
    cmd = [adb] + (["-s", serial] if serial else []) + ["reverse", f"tcp:{phone_port}", f"tcp:{stream_port}"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=8,
                           creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        out = (r.stdout + r.stderr).strip()
        ok = r.returncode == 0
    except Exception as e:
        out, ok = str(e), False
    log("adb reverse", f"tcp:{phone_port} tcp:{stream_port}", "ok" if ok else "FAILED", out)
    return ok, out


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False, separators=(",", ":"))
        b = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path)
        qs = parse_qs(u.query)
        hub, cfg = self.server.hub, self.server.cfg
        try:
            if u.path in ("/", "/index.html"):
                return self.send(200, PAGE, "text/html; charset=utf-8")
            if u.path == "/api/ping":
                return self.send(200, {"ok": True, "app": "ov_live_view", "stream_port": cfg.stream_port})
            if u.path == "/api/state":
                since = int(qs.get("since", ["0"])[0] or 0)
                sid = int(qs.get("sid", ["-1"])[0] or -1)
                return self.send(200, hub.state(since, sid))
            if u.path == "/api/new":
                hub.new_session(qs.get("tag", [""])[0])
                log("new session", hub.sid, hub.tag)
                return self.send(200, {"ok": True, "sid": hub.sid, "tag": hub.tag})
            if u.path == "/api/session":          # flush first: the caller copies the file next
                ev = threading.Event()
                hub.w.q.put(("flush", ev))
                ev.wait(3)
                with hub.lock:
                    return self.send(200, {"sid": hub.sid, "tag": hub.tag, "file": str(hub.path) if hub.path else None,
                                           "n_lines": hub.n_lines})
            if u.path == "/api/reverse":
                serial = qs.get("serial", [cfg.serial])[0]
                ok, out = adb_reverse(cfg.adb, serial, cfg.phone_port, cfg.stream_port)
                return self.send(200, {"ok": ok, "out": out})
            return self.send(404, {"error": "not found"})
        except Exception as e:
            log("http:", e)
            try:
                self.send(500, {"error": str(e)})
            except Exception:
                pass


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>VIO 실시간 경로</title>
<style>
:root{--bg:#f6f7f9;--panel:#fff;--fg:#1b1f24;--mut:#5f6b7a;--grid:#e3e7ec;--grid2:#c9d0d8;--axis:#8a96a3;
 --tr:#1a8f4c;--init:#8a96a3;--lost:#d23b3b;--oth:#c78a00;--cur:#1f6feb;--start:#6b4fd8;--bd:#dde2e8}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#0f1216;--panel:#171b21;--fg:#e6e9ee;--mut:#97a3b0;
 --grid:#232a33;--grid2:#34404d;--axis:#6b7785;--tr:#3ccf7a;--init:#7b8794;--lost:#ff6b6b;--oth:#e0a92e;--cur:#58a6ff;--start:#a68cff;--bd:#2a313b}}
:root[data-theme="dark"]{--bg:#0f1216;--panel:#171b21;--fg:#e6e9ee;--mut:#97a3b0;--grid:#232a33;--grid2:#34404d;--axis:#6b7785;
 --tr:#3ccf7a;--init:#7b8794;--lost:#ff6b6b;--oth:#e0a92e;--cur:#58a6ff;--start:#a68cff;--bd:#2a313b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.4 system-ui,-apple-system,"Segoe UI","Malgun Gothic",sans-serif}
main{max-width:1200px;margin:0 auto;padding:12px 16px 24px}
h1{font-size:18px;margin:0 0 8px;display:flex;gap:10px;align-items:center;flex-wrap:wrap}
.badge{font-size:13px;padding:2px 10px;border-radius:999px;color:#fff;background:var(--init)}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(104px,1fr));gap:8px;margin:8px 0 12px}
.st{background:var(--panel);border:1px solid var(--bd);border-radius:8px;padding:6px 10px}
.st b{display:block;font-size:18px;font-variant-numeric:tabular-nums}.st span{color:var(--mut);font-size:12px}
.views{display:grid;grid-template-columns:minmax(0,3fr) minmax(0,2fr);gap:12px}@media (max-width:760px){.views{grid-template-columns:minmax(0,1fr)}}
.card{background:var(--panel);border:1px solid var(--bd);border-radius:8px;padding:8px;min-width:0}
.card h2{font-size:13px;margin:0 0 4px;color:var(--mut);font-weight:600}
canvas{width:100%;max-width:100%;display:block;touch-action:none}
#top{aspect-ratio:1/1}#side{aspect-ratio:4/3}
.leg{display:flex;gap:12px;flex-wrap:wrap;color:var(--mut);font-size:12px;margin-top:6px}
.leg i{display:inline-block;width:12px;height:3px;vertical-align:middle;margin-right:4px}
.foot{color:var(--mut);font-size:12px;margin-top:10px;word-break:break-all}
button{font:inherit;font-size:12px;background:var(--panel);color:var(--fg);border:1px solid var(--bd);border-radius:6px;padding:3px 10px;cursor:pointer}
</style></head><body><main>
<h1>VIO 실시간 경로 <span id="badge" class="badge">대기</span> <span id="conn" style="font-size:12px;color:var(--mut)"></span>
 <span style="flex:1"></span><button id="follow">현재 위치 따라가기: 끔</button><button id="theme">화면 밝기</button></h1>
<div class="stats">
 <div class="st"><span>경과</span><b id="s_el">-</b></div>
 <div class="st"><span>프레임 처리 (마지막)</span><b id="s_ms">-</b></div>
 <div class="st"><span>처리 p95 (최근 5초)</span><b id="s_p95">-</b></div>
 <div class="st"><span>포즈 속도</span><b id="s_hz">-</b></div>
 <div class="st"><span>SLAM 특징점</span><b id="s_slam">-</b></div>
 <div class="st"><span>MSCKF 특징점</span><b id="s_msckf">-</b></div>
 <div class="st"><span>이동 경로 길이</span><b id="s_len">-</b></div>
 <div class="st"><span>시작점에서 거리</span><b id="s_dist">-</b></div>
 <div class="st"><span>현재 위치 x, y, z</span><b id="s_xyz" style="font-size:14px">-</b></div>
</div>
<div class="views">
 <div class="card"><h2>위에서 본 경로 (x-y, 격자 1 m)</h2><canvas id="top"></canvas></div>
 <div class="card"><h2>높이 z - 시간 (격자 자동 / 10 s)</h2><canvas id="side"></canvas></div>
</div>
<div class="leg"><span><i style="background:var(--tr)"></i>추적 중</span><span><i style="background:var(--init)"></i>초기화</span>
 <span><i style="background:var(--lost)"></i>잃음</span><span><i style="background:var(--start);height:8px;width:8px;border-radius:50%"></i>시작점</span>
 <span><i style="background:var(--cur);height:8px;width:8px;border-radius:50%"></i>현재</span></div>
<div class="foot" id="foot"></div>
</main><script>
const R=document.documentElement;
try{const t=localStorage.getItem("ovlv_theme");if(t)R.dataset.theme=t}catch(e){}
document.getElementById("theme").onclick=()=>{const dark=getComputedStyle(R).getPropertyValue("--bg").trim()==="#0f1216";
 R.dataset.theme=dark?"light":"dark";try{localStorage.setItem("ovlv_theme",R.dataset.theme)}catch(e){};draw()};
let follow=false;document.getElementById("follow").onclick=e=>{follow=!follow;e.target.textContent="현재 위치 따라가기: "+(follow?"켬":"끔");draw()};
let sid=-1,P=[],S=null,busy=false;
const C=n=>getComputedStyle(R).getPropertyValue(n).trim();
const ST=["init","tracking","lost","other"],SK={init:"초기화",tracking:"추적 중",lost:"잃음"};
function colOf(c){return [C("--init"),C("--tr"),C("--lost"),C("--oth")][c]}
function fmt(v,d,u){return v==null||!isFinite(v)?"-":v.toFixed(d)+(u||"")}
function setup(cv){const r=cv.getBoundingClientRect(),d=window.devicePixelRatio||1;const w=Math.max(10,Math.round(r.width*d)),h=Math.max(10,Math.round(r.height*d));
 if(cv.width!==w||cv.height!==h){cv.width=w;cv.height=h}const g=cv.getContext("2d");g.setTransform(d,0,0,d,0,0);g.clearRect(0,0,r.width,r.height);return [g,r.width,r.height]}
function niceStep(span,px){const t=[0.1,0.2,0.5,1,2,5,10,20,50,100];for(const s of t){if(px*s/span>=40)return s}return 100}
// generic view: data coords -> canvas, equal scale if eq
function view(cv,pts,fx,fy,opt){
 const [g,W,H]=setup(cv);const pad=34;
 let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;
 for(const p of pts){const a=fx(p),b=fy(p);if(a<x0)x0=a;if(a>x1)x1=a;if(b<y0)y0=b;if(b>y1)y1=b}
 if(!pts.length){x0=-1;x1=1;y0=-1;y1=1}
 if(opt.follow&&pts.length){const l=pts[pts.length-1],cx=fx(l),cy=fy(l);x0=cx-2;x1=cx+2;y0=cy-2;y1=cy+2}
 let mx=Math.max(opt.minx,x1-x0),my=Math.max(opt.miny,y1-y0);const cx=(x0+x1)/2,cy=(y0+y1)/2;
 mx*=1.1;my*=1.1;let sx=(W-2*pad)/mx,sy=(H-2*pad)/my;if(opt.eq){const s=Math.min(sx,sy);sx=sy=s}
 const X=a=>W/2+(a-cx)*sx,Y=b=>H/2-(b-cy)*sy;
 // grid
 const gx=opt.gx||niceStep(mx,sx),gy=opt.gy||niceStep(my,sy);
 g.lineWidth=1;g.font="11px system-ui,sans-serif";g.fillStyle=C("--mut");
 const ax0=cx-(W/2)/sx,ax1=cx+(W/2)/sx,ay0=cy-(H/2)/sy,ay1=cy+(H/2)/sy;
 for(let a=Math.ceil(ax0/gx)*gx;a<=ax1;a+=gx){const px=Math.round(X(a))+.5;g.strokeStyle=Math.abs(a)<1e-9?C("--axis"):C("--grid");g.beginPath();g.moveTo(px,0);g.lineTo(px,H);g.stroke();
  g.fillText(opt.lx(a),px+2,H-4)}
 for(let b=Math.ceil(ay0/gy)*gy;b<=ay1;b+=gy){const py=Math.round(Y(b))+.5;g.strokeStyle=Math.abs(b)<1e-9?C("--axis"):C("--grid");g.beginPath();g.moveTo(0,py);g.lineTo(W,py);g.stroke();
  g.fillText(opt.ly(b),2,py-2)}
 // path, colored by status of the newer point
 g.lineWidth=2;g.lineJoin="round";g.lineCap="round";
 let i=1;while(i<pts.length){const c=pts[i][4];g.strokeStyle=colOf(c);g.beginPath();g.moveTo(X(fx(pts[i-1])),Y(fy(pts[i-1])));
  while(i<pts.length&&pts[i][4]===c){g.lineTo(X(fx(pts[i])),Y(fy(pts[i])));i++}g.stroke()}
 const dot=(p,col,r)=>{g.fillStyle=col;g.beginPath();g.arc(X(fx(p)),Y(fy(p)),r,0,7);g.fill()};
 const s0=pts.find(p=>p[4]===1);if(s0)dot(s0,C("--start"),5);
 if(pts.length){const l=pts[pts.length-1];g.strokeStyle=C("--panel");g.lineWidth=2;dot(l,C("--cur"),6);g.beginPath();g.arc(X(fx(l)),Y(fy(l)),6,0,7);g.stroke()}
 g.fillStyle=C("--mut");g.fillText(opt.title,W-g.measureText(opt.title).width-4,12);
}
function draw(){
 view(document.getElementById("top"),P,p=>p[1],p=>p[2],{eq:true,minx:2,miny:2,gx:1,gy:1,follow,lx:a=>a.toFixed(0)+" m",ly:b=>b.toFixed(0),title:"x →, y ↑"});
 view(document.getElementById("side"),P,p=>p[0],p=>p[3],{eq:false,minx:10,miny:0.5,gx:10,follow:false,lx:a=>a.toFixed(0)+" s",ly:b=>+b.toFixed(1)+" m",title:"z (m)"});
}
function txt(id,v){document.getElementById(id).textContent=v}
function show(){
 if(!S)return;const L=S.last||{},st=L.status||"";const b=document.getElementById("badge");
 const stale=S.age_s==null||S.age_s>3;b.textContent=S.n_lines?(SK[st]||st||"?")+(stale?" (멈춤)":""):"대기";
 b.style.background=!S.n_lines||stale?C("--init"):st==="tracking"?C("--tr"):st==="lost"?C("--lost"):C("--init");
 txt("conn",(S.conns?"폰 연결됨":"폰 연결 없음")+" · 받은 줄 "+S.n_lines+(S.age_s!=null?" · 마지막 "+S.age_s+" s 전":""));
 txt("s_el",fmt(S.elapsed_s,1," s"));txt("s_ms",fmt(L.ms,1," ms"));txt("s_p95",fmt(S.ms_p95_5s,1," ms"));txt("s_hz",fmt(S.rate_hz,1," Hz"));
 txt("s_slam",L.n_slam??"-");txt("s_msckf",L.n_msckf??"-");txt("s_len",fmt(S.path_len,2," m"));txt("s_dist",fmt(S.dist_start,2," m"));
 const l=P[P.length-1];txt("s_xyz",l?l[1].toFixed(2)+", "+l[2].toFixed(2)+", "+l[3].toFixed(2):"-");
 const c=S.counts||{};const ev=(S.events||[]).map(e=>e[0]+" "+e[1]).join(" | ");
 txt("foot","포즈 "+S.n+" (추적 "+(c.tracking||0)+" · 초기화 "+(c.init||0)+" · 잃음 "+(c.lost||0)+") · 세션 "+S.sid+(S.tag?" ["+S.tag+"]":"")+
  " · 저장: "+(S.file||"(아직 없음)")+(ev?" · 이벤트: "+ev:""));
}
async function poll(){if(busy)return;busy=true;try{
 const r=await fetch("/api/state?sid="+sid+"&since="+(sid<0?0:P.length),{cache:"no-store"});const j=await r.json();
 if(j.sid!==sid||j.since===0&&P.length&&j.since!==P.length){sid=j.sid;P=[]}
 if(j.since===P.length)P=P.concat(j.poses);S=j;show();draw();
}catch(e){txt("conn","서버 응답 없음")}finally{busy=false}}
setInterval(poll,100);poll();window.addEventListener("resize",draw);
matchMedia("(prefers-color-scheme: dark)").addEventListener("change",draw);
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="100.92.214.97", help="page address (Tailscale IP of this PC)")
    ap.add_argument("--port", type=int, default=8798, help="page port")
    ap.add_argument("--stream-port", type=int, default=8794, help="PC port the phone stream arrives on (127.0.0.1)")
    ap.add_argument("--phone-port", type=int, default=8798, help="Streamer.PORT on the phone")
    ap.add_argument("--serial", default=os.environ.get("OV_SERIAL", "100.79.156.20:5555"), help="adb serial ('' = none)")
    ap.add_argument("--adb", default=os.environ.get("ADB_EXE", ADB_DEFAULT))
    ap.add_argument("--no-reverse", action="store_true", help="do not run adb reverse at start")
    a = ap.parse_args()

    w = Writer()
    w.start()
    hub = Hub(w)
    ss = StreamServer(("127.0.0.1", a.stream_port), StreamHandler)
    ss.hub = hub
    hs = HttpServer((a.host, a.port), Handler)
    hs.hub, hs.cfg = hub, a
    threading.Thread(target=ss.serve_forever, daemon=True).start()
    log(f"ov_live_view: page http://{a.host}:{a.port}  stream 127.0.0.1:{a.stream_port}  files {RUNS}")
    if not a.no_reverse:
        threading.Thread(target=adb_reverse, args=(a.adb, a.serial, a.phone_port, a.stream_port), daemon=True).start()
    try:
        hs.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
