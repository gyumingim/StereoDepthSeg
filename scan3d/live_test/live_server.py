#!/usr/bin/env python3
r"""Live scan test (map mode) - PC page with START / STOP, phone capture, laptop scanner, mesh view. 2026-11-15.

Runs on Windows Python (adb.exe and wsl.exe are Windows programs); stdlib only; never opens a window:
  C:\Users\a3162\AppData\Local\Programs\Python\Python312\python.exe \\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg-livetest\scan3d\live_test\live_server.py
  -> http://localhost:8797

What it does (the phone app cannot send keyframes live yet, so "live" = capture -> STOP -> mesh a few minutes later):
  START  preflight (adb device, laptop reachable + GPU lock free, code pushed to the laptop) -> phone app arlat
         `mode hs, session std` = 1920x1080 60 fps, focus 0.5 dpt, the recording format the scanner defaults were set on;
         on-screen guide (still -> move -> still) that ends the recording by itself after N s (heat cap).
  STOP   recording: HOME + force-stop (the app closes the mp4 in onPause), then the build continues on what was recorded.
         pulling:   cancels the adb pull (recording stays on the phone).
         building:  kills the laptop job; remote_laptop.sh then pulls whatever was built (partial outputs kept).
         always:    force-stop of the phone app (idempotent safety).
  build  adb pull -> C:\Users\a3162\scan3d_live\recordings -> WSL scan3d/recordings -> remote_laptop.sh job <rec> scan --tag live
         (= scan3d/scanA/run_scanner.sh defaults on the laptop: VIO c4f60 replay -> motion keyframes as 1920 JPEG q90
         -> 960 compute, kf_ba GPU BA -> FFS + TSDF 5 mm) -> mesh.ply copied to C:\Users\a3162\scan3d_live\meshes\<rec>\.
  replay /api/replay {rec}: build only, from a recording already in WSL scan3d/recordings (no phone) - the dry check.
"""
import argparse
import json
import os
import re
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG = "com.scan3d.arlat"
PHONE_FILES = f"/sdcard/Android/data/{PKG}/files"
DISTRO, WSL_USER = "Ubuntu-22.04", "system"
WT = "/home/system/scanner/StereoDepthSeg-livetest"            # worktree pushed to the laptop (run_scanner.sh + remote_laptop.sh)
RECS = "/home/system/scanner/StereoDepthSeg/scan3d/recordings"  # remote_laptop.sh resolves recordings here
WLOG = "/home/system/scanner/live_test_logs"                    # build logs (WSL side)
LOCAL = Path(r"C:\Users\a3162\scan3d_live")                      # Windows side: pulled recordings (temporary), meshes, server log
NOWIN = getattr(subprocess, "CREATE_NO_WINDOW", 0)
GUIDE = "가만히 (바닥에 둔 채):{a};천천히 둘러보기 (조금씩 걸으며):{b};가만히:{a}"


def wpath(p):
    """C:\\x\\y -> /mnt/c/x/y"""
    p = str(p)
    return "/mnt/" + p[0].lower() + p[2:].replace("\\", "/")


class Hub:
    def __init__(self, a):
        self.a = a
        self.lock = threading.Lock()
        self.log = deque(maxlen=200)
        self.st = dict(phase="idle", msg="대기 중", rec=None, t0=None, secs=0, pull=None, job=None, step=None,
                       mesh=None, temp=None, serial=None, link=None)
        self.cancel = threading.Event()
        self.pull_proc = None
        self.worker = None
        for d in ("recordings", "meshes", "logs"):
            (LOCAL / d).mkdir(parents=True, exist_ok=True)
        self.logf = LOCAL / "logs" / time.strftime("server_%Y%m%d.log")

    # ---------- helpers ----------
    def say(self, msg):
        line = time.strftime("%H:%M:%S ") + msg
        self.log.append(line)
        print(line, flush=True)
        try:
            with open(self.logf, "a", encoding="utf-8") as f:
                f.write(time.strftime("%Y-%m-%d ") + line + "\n")
        except OSError:
            pass

    def set(self, **kw):
        with self.lock:
            self.st.update(kw)

    def phase(self):
        with self.lock:
            return self.st["phase"]

    def adb(self, *args, timeout=60, serial=True):
        cmd = [self.a.adb] + (["-s", self.st["serial"]] if serial and self.st.get("serial") else []) + list(args)
        r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=timeout,
                           creationflags=NOWIN)
        return (r.stdout + r.stderr).strip()

    def wsl(self, script, timeout=600):
        """bash script on stdin (wsl.exe would expand $VAR in a command line)."""
        r = subprocess.run(["wsl.exe", "-d", DISTRO, "-u", WSL_USER, "--", "bash", "-s"], input=script, capture_output=True,
                           text=True, encoding="utf-8", errors="replace", timeout=timeout, creationflags=NOWIN)
        return r.returncode, (r.stdout + r.stderr).replace("\0", "").strip()

    def pick_serial(self):
        """USB device first (fast pull), else the wireless one. None = no phone."""
        out = self.adb("devices", serial=False, timeout=20)
        devs = [l.split()[0] for l in out.splitlines()[1:] if l.strip().endswith("device")]
        usb = [d for d in devs if ":" not in d and not d.startswith("adb-")]
        s = (usb or devs or [None])[0]
        self.set(serial=s, link=None if s is None else ("USB" if s in usb else "무선"))
        return s

    def phone_temp(self):
        out = self.adb("shell", "dumpsys battery | grep -E '^ *temperature:'", timeout=15)
        m = re.search(r"temperature: (\d+)", out)
        return int(m.group(1)) / 10 if m else None

    def phone_dirs(self):
        out = self.adb("shell", f"ls -d {PHONE_FILES}/hs_* 2>/dev/null", timeout=20)
        return sorted(l.strip().split("/")[-1] for l in out.splitlines() if "/hs_" in l)

    def phone_stop(self):
        """HOME -> the app stops the recorder in onPause (mp4 closed, meta.json written) -> force-stop."""
        self.adb("shell", "input", "keyevent", "KEYCODE_HOME", timeout=15)
        time.sleep(3)
        self.adb("shell", "am", "force-stop", PKG, timeout=15)
        self.say("폰 앱 멈춤 (HOME + force-stop)")

    # ---------- commands from the page ----------
    def start(self, secs, kbps):
        if self.phase() not in ("idle", "done", "failed", "canceled"):
            return "이미 진행 중"
        self.cancel.clear()
        self.set(phase="preflight", msg="준비 확인 중…", rec=None, t0=None, secs=secs, pull=None, job=None, step=None, mesh=None)
        self.worker = threading.Thread(target=self._run_capture, args=(secs, kbps), daemon=True)
        self.worker.start()
        return "시작"

    def replay(self, rec):
        if self.phase() not in ("idle", "done", "failed", "canceled"):
            return "이미 진행 중"
        self.cancel.clear()
        self.set(phase="building", msg=f"녹화본으로 시험: {rec}", rec=rec, t0=None, pull=None, job=None, step=None, mesh=None)
        self.worker = threading.Thread(target=self._run_build_safe, args=(rec,), daemon=True)
        self.worker.start()
        return "시작 (폰 없이)"

    def stop(self):
        ph = self.phase()
        self.say(f"STOP 눌림 (단계: {ph})")
        if ph == "preflight":                                       # phone not started yet: just do not start it
            self.cancel.set()
            self.set(phase="stopping", msg="시작 취소 중…")
            return "시작 취소"
        if ph in ("starting", "recording"):
            self.set(phase="stopping", msg="폰 녹화 멈추는 중…")
            threading.Thread(target=self.phone_stop, daemon=True).start()   # capture thread sees it and continues to the build
            return "폰 녹화 멈춤 → 찍은 만큼 메시로 만듦"
        if ph == "pulling":
            self.cancel.set()
            p = self.pull_proc
            if p and p.poll() is None:
                p.kill()
            threading.Thread(target=lambda: self.adb("shell", "am", "force-stop", PKG, timeout=15), daemon=True).start()
            return "가져오기 취소 (녹화는 폰에 그대로)"
        if ph == "building":
            self.cancel.set()
            threading.Thread(target=self._cancel_laptop, daemon=True).start()
            return "노트북 작업 멈춤 → 만든 데까지 가져옴"
        if self.st.get("serial"):
            threading.Thread(target=lambda: self.adb("shell", "am", "force-stop", PKG, timeout=15), daemon=True).start()
        return "진행 중인 일 없음 (폰 앱 force-stop 만)"

    # ---------- capture ----------
    def _run_capture(self, secs, kbps):
        try:
            rec = self._capture(secs, kbps)
            if rec:
                self._pull(rec)
                if not self.cancel.is_set():
                    self._build(rec)
        except Exception as e:
            self.set(phase="failed", msg=f"오류: {e}")
            self.say(f"오류: {e}")
            try:
                self.adb("shell", "am", "force-stop", PKG, timeout=15)
            except Exception:
                pass

    def _capture(self, secs, kbps):
        if not self.pick_serial():
            raise RuntimeError("폰이 adb 에 안 보임 (adb devices)")
        self.say(f"폰: {self.st['serial']} ({self.st['link']})")
        ver = self.adb("shell", f"dumpsys package {PKG} | grep -m1 versionName", timeout=20)
        if "versionName" not in ver:
            raise RuntimeError(f"폰에 {PKG} 앱이 없음")
        self.say("앱 " + ver.strip())
        self.preflight_laptop()
        if self.cancel.is_set():
            self.set(phase="canceled", msg="시작 전에 멈춤 (폰 카메라 안 켬)")
            return None
        before = set(self.phone_dirs())
        for perm in ("android.permission.DUMP", "android.permission.PACKAGE_USAGE_STATS"):
            self.adb("shell", "pm", "grant", PKG, perm, timeout=15)
        a = 5 if secs >= 20 else 2
        guide = GUIDE.format(a=a, b=max(1, secs - 2 * a))
        self.adb("shell", "input", "keyevent", "KEYCODE_WAKEUP", timeout=15)
        cmd = ["shell", "am", "start", "-n", f"{PKG}/.MainActivity", "--es", "mode", "hs", "--es", "session", "std",
               "--ei", "fps", "60", "--ef", "focus_dpt", "0.5", "--ei", "kbps", str(int(kbps)),
               "--es", "guide", f"'{guide}'", "--ei", "guide_secs", str(int(secs))]
        self.set(phase="starting", msg="폰 카메라 켜는 중…")
        self.say("폰 녹화 시작: 1920x1080 60 fps, 초점 0.5 dpt, " + f"{kbps // 1000} Mbps, {secs} s 안내 후 자동 멈춤")
        self.say(self.adb(*cmd, timeout=30)[-200:])
        t_start = time.time()
        rec = None
        while time.time() - t_start < 25 and rec is None:          # the app makes files/hs_<time>_std60 when recording starts
            if self.phase() == "stopping":
                break
            new = [d for d in self.phone_dirs() if d not in before]
            rec = new[-1] if new else None
            if rec is None:
                time.sleep(1.5)
        if rec is None:
            if self.phase() != "stopping":
                self.phone_stop()
            # STOP during start: the folder may still have appeared
            new = [d for d in self.phone_dirs() if d not in before]
            if not new:
                self.set(phase="failed" if self.phase() != "stopping" else "canceled", msg="녹화 폴더가 안 생김 (폰 화면 잠김? 앱 오류?)")
                return None
            rec = new[-1]
        self.set(rec=rec, t0=time.time())
        if self.phase() != "stopping":
            self.set(phase="recording", msg="녹화 중 — 폰 화면 안내대로 움직이세요")
        self.say(f"녹화 폴더: {rec}")
        last_t = 0
        while True:                                                 # until STOP, guide end (app stops itself) or the PC cap
            meta = self.adb("shell", f"grep -o '\"stop_reason\": \"[^\"]*' {PHONE_FILES}/{rec}/meta.json 2>/dev/null", timeout=15)
            if '"stop_reason"' in meta:
                self.say("폰이 녹화를 마침: " + meta.split('"stop_reason": "')[-1])
                break
            el = time.time() - self.st["t0"]
            if self.phase() == "recording" and el > secs + 25:
                self.say(f"안내 시간 {secs} s + 25 s 를 넘김 → PC 가 멈춤")
                self.set(phase="stopping", msg="시간 초과 — 멈추는 중")
                self.phone_stop()
            if self.phase() == "stopping" and el > secs + 60:
                break
            if time.time() - last_t > 10:
                last_t = time.time()
                t = self.phone_temp()
                self.set(temp=t)
            time.sleep(2)
        self.adb("shell", "am", "force-stop", PKG, timeout=15)       # camera surely off (idempotent)
        time.sleep(1)
        return rec

    def preflight_laptop(self):
        self.set(msg="노트북 확인 + 코드 보내는 중…")
        rc, out = self.wsl(f"cd {WT}/scan3d && timeout 90 tools/remote_laptop.sh status 2>&1 | head -3", timeout=120)
        self.say("노트북: " + out.replace("\n", " | ")[:200])
        if "unreachable" in out or rc != 0:
            raise RuntimeError("노트북에 연결 안 됨: " + out[-200:])
        if "lock free" not in out:
            raise RuntimeError("노트북 GPU 가 다른 작업 중 (lock): " + out.splitlines()[0][:150])
        rc, out = self.wsl(f"cd {WT}/scan3d && timeout 300 tools/remote_laptop.sh push {WT} 2>&1 | tail -1", timeout=320)
        self.say("노트북 코드: " + out[-150:])
        if rc != 0:
            raise RuntimeError("노트북에 코드 보내기 실패: " + out[-200:])

    # ---------- pull ----------
    def _pull(self, rec):
        out = self.adb("shell", f"stat -c %s {PHONE_FILES}/{rec}/cam5.mp4; grep -c '\"complete\": true' {PHONE_FILES}/{rec}/meta.json",
                       timeout=20).split()
        total = int(out[0]) if out and out[0].isdigit() else 0
        if total < 1_000_000:
            raise RuntimeError(f"녹화 영상이 없음/너무 작음 ({total} B): {rec}")
        if len(out) < 2 or out[1] != "1":
            self.say("주의: meta.json 에 complete:true 가 없음 (그래도 가져옴)")
        dst = LOCAL / "recordings"
        self.set(phase="pulling", msg=f"PC 로 가져오는 중 ({self.st['link']})", pull=dict(total=total, done=0, rate=0, eta=None))
        self.say(f"가져오기: {rec} ({total / 1e6:.0f} MB, {self.st['link']})")
        t0 = time.time()
        self.pull_proc = subprocess.Popen([self.a.adb, "-s", self.st["serial"], "pull", f"{PHONE_FILES}/{rec}", str(dst)],
                                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, creationflags=NOWIN)
        f = dst / rec / "cam5.mp4"
        last = (t0, 0)
        while self.pull_proc.poll() is None:
            time.sleep(2)
            try:
                n = f.stat().st_size
            except OSError:
                continue
            t = time.time()
            r = (n - last[1]) / max(t - last[0], 1e-3)
            p = self.st["pull"]
            rate = r if not p["rate"] else 0.7 * p["rate"] + 0.3 * r
            self.set(pull=dict(total=total, done=n, rate=rate, eta=(total - n) / rate if rate > 1e4 else None))
            last = (t, n)
        if self.cancel.is_set():
            self.set(phase="canceled", msg=f"가져오기 취소됨 — 녹화는 폰에 그대로 ({rec})")
            return
        if self.pull_proc.returncode != 0 or not f.exists() or f.stat().st_size != total:
            raise RuntimeError(f"가져오기 실패 (rc {self.pull_proc.returncode}, {f.stat().st_size if f.exists() else 0}/{total} B)")
        self.say(f"가져옴 {total / 1e6:.0f} MB, {time.time() - t0:.0f} s ({total / 1e6 / max(time.time() - t0, 1):.2f} MB/s); 폰 사본은 그대로 둠")
        rc, out = self.wsl(f"set -e; [ -e {RECS}/{rec} ] && {{ echo exists; exit 0; }}; cp -r {wpath(dst / rec)} {RECS}/{rec}.part && "
                           f"mv {RECS}/{rec}.part {RECS}/{rec}; stat -c %s {RECS}/{rec}/cam5.mp4", timeout=900)
        if rc != 0:
            raise RuntimeError("WSL 로 복사 실패: " + out[-200:])
        if out.strip().split()[-1] == str(total):
            import shutil
            shutil.rmtree(dst / rec, ignore_errors=True)            # the WSL copy is the kept one
        self.say(f"WSL 저장: {RECS}/{rec}")

    # ---------- build on the laptop ----------
    def _run_build_safe(self, rec):
        try:
            self._build(rec)
        except Exception as e:
            self.set(phase="failed", msg=f"오류: {e}")
            self.say(f"오류: {e}")

    def _build(self, rec):
        self.set(phase="building", msg="노트북에서 메시 만드는 중 (보통 4~6 분)", step="보내는 중")
        self.say(f"노트북 작업 시작: {rec} (run_scanner.sh 기본값)")
        lg, rcf = f"{WLOG}/{rec}.log", f"{WLOG}/{rec}.rc"
        rc, out = self.wsl(f"mkdir -p {WLOG}; [ -d {RECS}/{rec} ] || {{ echo 'no recording {RECS}/{rec}'; exit 3; }}; rm -f {rcf}; cd {WT}/scan3d && "
                           f"setsid -f bash -c 'WT={WT} tools/remote_laptop.sh job {rec} scan --tag live > {lg} 2>&1; echo $? > {rcf}' "
                           f"< /dev/null > /dev/null 2>&1; echo started", timeout=60)
        if rc != 0:
            raise RuntimeError(out[-200:])
        t0, last_q = time.time(), 0
        rid = rec[3:] if rec.startswith("hs_") else rec
        while True:
            time.sleep(4)
            rc, out = self.wsl(f"cat {rcf} 2>/dev/null || echo RUNNING; grep -m1 -o '== job [^ ]*' {lg}; tail -n 2 {lg}", timeout=60)
            lines = out.splitlines()
            jid = next((l.split()[2] for l in lines if l.startswith("== job ")), None)
            if jid and not self.st.get("job"):
                self.set(job=jid)
            if lines and lines[0].strip().isdigit():
                brc = int(lines[0])
                break
            if jid and time.time() - last_q > 20:                   # laptop step (vio / reduce / kfba / mesh)
                last_q = time.time()
                _, s = self.wsl(f"cd {WT}/scan3d && timeout 60 tools/remote_laptop.sh sh 'tail -n 40 $(ls -d runs/laptop/{rid}/live/scan.log 2>/dev/null | head -1) 2>/dev/null "
                                f"| grep -o \"step [a-z_]* [a-z]*\" | tail -1'", timeout=90)
                stp = next((l for l in reversed(s.splitlines()) if l.startswith("step ")), None)
                if stp:
                    self.set(step=stp, msg=f"노트북: {stp} ({(time.time() - t0) / 60:.1f} 분)")
            elif not jid:
                self.set(msg=f"노트북 차례 기다리는 중 / 보내는 중 ({(time.time() - t0) / 60:.1f} 분)")
        _, tail = self.wsl(f"tail -n 25 {lg}")
        self.say(f"노트북 작업 끝 rc {brc} ({(time.time() - t0) / 60:.1f} 분)")
        # results: WT/scan3d/runs/laptop/<rid>/live/scan/{mesh/mesh.ply, scan_summary.json}
        dst = LOCAL / "meshes" / rec
        rc, out = self.wsl(f"d=$(ls -d {WT}/scan3d/runs/laptop/{rid}/live/scan 2>/dev/null | head -1); [ -n \"$d\" ] || {{ echo NODIR; exit 0; }}; "
                           f"mkdir -p {wpath(dst)}; for f in scan_summary.json mesh/mesh.ply mesh/summary.json; do "
                           f"[ -f $d/$f ] && cp $d/$f {wpath(dst)}/$(echo $f | tr / _); done; echo $d; ls {wpath(dst)}", timeout=300)
        self.say("결과: " + out.replace("\n", " ")[:250])
        has_mesh = (dst / "mesh_mesh.ply").exists()
        summ = {}
        try:
            summ = json.loads((dst / "scan_summary.json").read_text(encoding="utf-8"))
        except Exception:
            pass
        if has_mesh:
            self.set(mesh=f"/mesh/{rec}/mesh_mesh.ply")
        if self.cancel.is_set():
            self.set(phase="canceled", msg="노트북 작업 멈춤 — " + ("만든 메시 있음" if has_mesh else "메시 전에 멈춤 (중간 결과는 runs/laptop 에)"))
        elif brc == 0 and has_mesh:
            m = summ.get("mesh") or {}
            self.set(phase="done", msg=f"완료 — 메시 {m.get('mesh_vertices') or '?'} 꼭짓점, 키프레임 {m.get('n_kf') or '?'}, "
                                       f"총 {summ.get('total_s', '?')} s")
        else:
            self.set(phase="failed", msg=f"노트북 작업 실패 rc {brc}: " + " / ".join(tail.splitlines()[-3:])[:300])

    def _cancel_laptop(self):
        jid = self.st.get("job")
        if not jid:
            self.say("작업 id 아직 없음 → 노트북 차례 대기/전송 중; remote_laptop.sh 를 끔")
            self.wsl(f"pkill -INT -f 'remote_laptop.sh job {self.st.get('rec')}' || true", timeout=30)
            return
        rc, out = self.wsl(f"cd {WT}/scan3d && timeout 90 tools/remote_laptop.sh sh 'pid=$(pgrep -f \"laptop_jobs/{jid}/run.sh\" | head -1); "
                           f"if [ -n \"$pid\" ]; then kill -TERM -- -$(ps -o pgid= -p $pid | tr -d \" \"); echo killed $pid; else echo no-job; fi'",
                           timeout=120)
        self.say("노트북 작업 멈춤: " + out[-150:] + " (remote_laptop.sh 가 만든 데까지 가져옴)")

    def recordings(self):
        rc, out = self.wsl(f"cd {RECS} && ls -d hs_*_std60 2>/dev/null | sort | tail -12", timeout=60)
        return out.split() if rc == 0 else []

    def meshes(self):
        r = []
        for d in sorted((LOCAL / "meshes").glob("*/mesh_mesh.ply"), key=lambda p: p.stat().st_mtime, reverse=True):
            r.append(dict(rec=d.parent.name, url=f"/mesh/{d.parent.name}/mesh_mesh.ply", mb=round(d.stat().st_size / 1e6, 1),
                          t=time.strftime("%m-%d %H:%M", time.localtime(d.stat().st_mtime))))
        return r

    def state(self):
        with self.lock:
            s = dict(self.st)
        if s.get("t0"):
            s["elapsed"] = round(time.time() - s["t0"], 1)
        s["log"] = list(self.log)[-30:]
        return s


def make_handler(hub):
    page = (HERE / "live.html").read_bytes()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, code=200):
            b = json.dumps(obj, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif self.path == "/api/state":
                self._json(hub.state())
            elif self.path == "/api/meshes":
                self._json(hub.meshes())
            elif self.path == "/api/recordings":
                self._json(hub.recordings())
            elif self.path.startswith("/mesh/"):
                parts = self.path.split("?")[0].split("/")
                if len(parts) != 4 or ".." in parts:
                    return self._json({"err": "bad path"}, 400)
                f = LOCAL / "meshes" / parts[2] / parts[3]
                if not f.is_file():
                    return self._json({"err": "no file"}, 404)
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(f.stat().st_size))
                self.end_headers()
                with open(f, "rb") as fh:
                    while True:
                        b = fh.read(1 << 20)
                        if not b:
                            break
                        self.wfile.write(b)
            else:
                self._json({"err": "not found"}, 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                body = {}
            if self.path == "/api/start":
                secs = max(10, min(120, int(body.get("secs", 30))))
                kbps = int(body.get("kbps", 160000))
                kbps = kbps if kbps in (40000, 80000, 160000) else 160000
                self._json({"msg": hub.start(secs, kbps)})
            elif self.path == "/api/stop":
                self._json({"msg": hub.stop()})
            elif self.path == "/api/replay":
                rec = str(body.get("rec", ""))
                if not re.fullmatch(r"hs_\d{8}_\d{6}_std60", rec):
                    return self._json({"msg": "녹화 이름이 이상함"}, 400)
                self._json({"msg": hub.replay(rec)})
            else:
                self._json({"err": "not found"}, 404)
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8797)
    ap.add_argument("--adb", default=r"C:\Users\a3162\AppData\Local\Microsoft\WinGet\Packages\Genymobile.scrcpy_Microsoft.Winget.Source_8wekyb3d8bbwe"
                                     r"\scrcpy-win64-v4.1\adb.exe")
    a = ap.parse_args()
    hub = Hub(a)
    hub.say(f"live_server 켜짐: http://localhost:{a.port}  (adb {a.adb})")
    ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(hub)).serve_forever()


if __name__ == "__main__":
    main()
