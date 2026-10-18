#!/usr/bin/env python3
"""폰 녹화 업로드 수신 서버 + Cloudflare Tunnel (표준 라이브러리만, scan3d 웹 서버와 따로 도는 프로세스).

왜: 집(폰) → 회사 PC 는 들어오는 연결이 막혀 Tailscale 이 도쿄 DERP 중계(adb pull 약 0.7 MB/s)로만 된다.
PC 가 cloudflared 로 바깥(Cloudflare)에 나가는 연결을 열면 폰이 HTTPS 로 녹화를 올릴 수 있다
(2026-10-18 실측: 48 MB, 1 줄 2.4 MB/s · 3 줄 3.1 MB/s, adb pull 0.68 MB/s).

    python3 scan3d/upload_server.py            # 포트 8799 (127.0.0.1) + quick tunnel
    python3 scan3d/upload_server.py --push-now # 폰에 설정(upload.json)만 한 번 넣고 끝

serve_idle.sh 가 서버를 띄울 때 이것도 (없으면) 같이 띄운다. scan3d 서버를 다시 켜도 이 프로세스는 남아서
터널 주소가 바뀌지 않는다. 로그 runs/upload_server.log, cloudflared 로그 runs/cloudflared.log.

설정 runs/upload.json (없으면 기본값으로 만듦):
  enabled      true   — false 면 폰 설정에 enabled=false 를 넣어 앱이 안 올림 (서버는 계속 받음)
  protocol     "http2" — cloudflared 와 Cloudflare 사이 (WSL 은 UDP 버퍼를 못 키워 quic 이 느림: 2026-10-18 1 MB/s 대)
  tunnel       "quick" (계정 없는 trycloudflare, 주소가 cloudflared 시작마다 바뀜) | "none" (고정 주소를 따로 둘 때)
  public_url   null 또는 "https://scan.example.com" (Cloudflare 계정·도메인의 이름 있는 터널 → 이 주소를 폰에 넣음)
  wifi_only    true   — 앱이 Wi-Fi(데이터 요금 없는 망)에서만 올림
  delete_after false  — PC 가 sha256 까지 확인한 녹화를 폰에서 지움
  max_rec_mb   4096   — 녹화 하나 최대 크기 (넘으면 거절)
토큰 runs/upload_token.txt (처음에 만듦, 0600). 폰에는 adb 로 한 번 넣는다 (아래 '폰 설정 넣기').

API (모든 요청에 Authorization: Bearer <토큰>, 없거나 틀리면 401):
  GET  /api/up/ping                                    {ok, t}
  GET  /api/up/status?rec=NAME                         {ok, done, have:{파일: 받은 바이트}}  (done = PC recordings/ 에 이미 있음)
  PUT  /api/up/chunk?rec=NAME&file=F&off=O&total=T     본문 = 파일 F 의 O 바이트부터 (최대 32 MB). 헤더 X-Rec-Total = 녹화 전체 크기.
                                                       O 가 PC 에 받은 크기와 다르면 409 {have} → 앱이 그 자리부터 다시
  POST /api/up/finish?rec=NAME  {"files": {F: {"size": N, "sha256": "..."}}}
       크기·sha256 이 모두 맞고 meta.json 이 complete=true 면 recordings/.uploading/NAME → recordings/NAME 로 옮김
       (옛 PC 사본의 recon_* 은 새 폴더로 옮겨 보존). 그 뒤 rec_sync 의 자동 복원이 집어 간다. 응답 {ok, moved}
상태 파일 runs/upload_state.json (웹 /api/state 의 ui.upload): 터널 주소, 지금 받는 녹화·진행률·속도, 최근 끝난 것.

폰 설정 넣기 (adb 가 붙어 있을 때 30 s 마다 확인, 다르면 다시 씀):
  /sdcard/Android/data/com.camera.dualstream/files/upload.json = {url, token, enabled, wifi_only, delete_after}
  앱(1.9-upload)은 녹화가 끝나면 UploadService 로 이 주소에 올린다.
"""
import argparse
import base64
import hashlib
import json
import os
import re
import secrets
import shutil
import subprocess
import sys
import threading
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse, parse_qs

import phone_link

HERE = Path(__file__).resolve().parent
RUNS = HERE / "runs"
CFG_FILE = RUNS / "upload.json"
TOKEN_FILE = RUNS / "upload_token.txt"
STATE_FILE = RUNS / "upload_state.json"
URL_FILE = RUNS / "tunnel_url.txt"
LOG_FILE = RUNS / "upload_server.log"
CF_LOG = RUNS / "cloudflared.log"
PORT = 8799                                  # 8795 웹 · 8796 미리보기 · 8797 adb_keeper · 8798 rt_depth
CHUNK_MAX = 32 << 20
REC_RE = re.compile(r"^rec_[\w.\-]{1,80}$")
FILE_RE = re.compile(r"^[A-Za-z0-9][\w.\-]{0,120}$")
PHONE_CFG = f"/sdcard/Android/data/{phone_link.APP}/files/upload.json"
DEFAULT_CFG = dict(enabled=True, tunnel="quick", protocol="http2", public_url=None, wifi_only=True, delete_after=False, max_rec_mb=4096)
CF_URL_RE = re.compile(r"https://[a-z0-9-]+\.trycloudflare\.com")


def log(msg):
    line = time.strftime("%Y-%m-%d %H:%M:%S ") + msg
    if sys.stdout.isatty():
        print(line, flush=True)
    try:
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except OSError:
        pass


def write_json(path, obj):
    tmp = path.with_name(path.name + ".part")
    tmp.write_text(json.dumps(obj, ensure_ascii=False, indent=1))
    tmp.replace(path)


def load_cfg():
    cfg = dict(DEFAULT_CFG)
    try:
        cfg.update(json.loads(CFG_FILE.read_text()))
    except (OSError, ValueError):
        RUNS.mkdir(parents=True, exist_ok=True)
        write_json(CFG_FILE, cfg)
    return cfg


def load_token():
    try:
        t = TOKEN_FILE.read_text().strip()
        if len(t) >= 24:
            return t
    except OSError:
        pass
    t = secrets.token_hex(24)
    RUNS.mkdir(parents=True, exist_ok=True)
    TOKEN_FILE.write_text(t + "\n")
    os.chmod(TOKEN_FILE, 0o600)
    log("새 업로드 토큰을 만들었습니다 (runs/upload_token.txt)")
    return t


def upload_enabled():
    """phone_link·rec_sync 가 부름: 업로드 방식이 켜져 있나 (runs/upload.json enabled)."""
    try:
        return bool(json.loads(CFG_FILE.read_text()).get("enabled", True))
    except (OSError, ValueError):
        return False


class State:
    def __init__(self, rec_root):
        self.rec_root = Path(rec_root)
        self.stage = self.rec_root / ".uploading"
        self.lock = threading.Lock()
        self.url = None
        self.url_t = None
        self.tunnel = "off"
        self.active = {}            # name → dict(done, total, t0, t, bytes_win)
        self.recent = []            # [{name, mb, s, mbps, at, ok, error}]
        self.pushed = None          # 폰에 마지막으로 넣은 설정 (문자열)
        self.phone_cfg_ok = None
        self.dirty = True

    def save(self):
        with self.lock:
            now = time.time()
            act = []
            for n, a in self.active.items():
                if now - a["t"] > 600:
                    continue
                el = max(1e-3, a["t"] - a["t0"])
                act.append(dict(name=n, done=a["done"], total=a["total"], session_bytes=a["sess"],
                                mbps=round(a["sess"] / 1048576 / el, 2), idle_s=round(now - a["t"], 1)))
            st = dict(t=now, url=self.url, url_t=self.url_t, tunnel=self.tunnel, phone_cfg_ok=self.phone_cfg_ok,
                      active=act, recent=self.recent[-10:], enabled=upload_enabled())
            self.dirty = False
        try:
            write_json(STATE_FILE, st)
        except OSError:
            pass

    def note(self, name, nbytes, total):
        with self.lock:
            now = time.time()
            a = self.active.get(name)
            if a is None or now - a["t"] > 120:
                a = self.active[name] = dict(done=0, total=total, t0=now, t=now, sess=0)
            a["sess"] += nbytes
            a["t"] = now
            if total:
                a["total"] = total
            a["done"] = staged_bytes(self.stage / name)
            self.dirty = True


def staged_bytes(d):
    try:
        return sum(p.stat().st_size for p in d.iterdir() if p.is_file())
    except OSError:
        return 0


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        while True:
            b = f.read(1 << 20)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def make_handler(S, token, cfg_get):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "scan3d-upload/1"

        def log_message(self, *a):
            pass

        def _json(self, code, obj):
            b = json.dumps(obj, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def _auth(self):
            h = self.headers.get("Authorization", "")
            ok = h.startswith("Bearer ") and secrets.compare_digest(h[7:].strip(), token)
            if not ok:
                self._drain()
                self._json(401, dict(ok=False, error="token"))
            return ok

        def _drain(self):
            n = int(self.headers.get("Content-Length") or 0)
            if 0 < n <= CHUNK_MAX:
                try:
                    self.rfile.read(n)
                except OSError:
                    pass
            elif n > CHUNK_MAX:
                self.close_connection = True

        def _q(self):
            u = urlparse(self.path)
            return u.path, {k: v[0] for k, v in parse_qs(u.query).items()}

        def _rec(self, q):
            n = q.get("rec", "")
            if not REC_RE.match(n):
                self._drain()
                self._json(400, dict(ok=False, error="rec name"))
                return None
            return n

        def do_GET(self):
            path, q = self._q()
            if not self._auth():
                return
            if path == "/api/up/ping":
                return self._json(200, dict(ok=True, t=time.time()))
            if path == "/api/up/status":
                n = self._rec(q)
                if n is None:
                    return
                final = S.rec_root / n
                done = (final / "meta.json").is_file() and (final / ".uploaded.json").is_file()
                files = {}
                d = final if done else S.stage / n
                if d.is_dir():
                    files = {p.name: p.stat().st_size for p in d.iterdir() if p.is_file() and not p.name.startswith(".")}
                if not done and final.is_dir():
                    # 이미 adb 로 가져온 사본 → 앱이 크기를 비교해 같으면 올리지 않음
                    pc = {p.name: p.stat().st_size for p in final.iterdir() if p.is_file() and not p.name.startswith(".")}
                    return self._json(200, dict(ok=True, done=False, have=files, pc=pc))
                return self._json(200, dict(ok=True, done=done, have=files))
            return self._json(404, dict(ok=False, error="path"))

        def do_PUT(self):
            path, q = self._q()
            if not self._auth():
                return
            if path != "/api/up/chunk":
                self._drain()
                return self._json(404, dict(ok=False, error="path"))
            n = self._rec(q)
            if n is None:
                return
            fn = q.get("file", "")
            try:
                off, total = int(q.get("off", "-1")), int(q.get("total", "-1"))
                rec_total = int(self.headers.get("X-Rec-Total") or 0)
                ln = int(self.headers.get("Content-Length") or -1)
            except ValueError:
                self._drain()
                return self._json(400, dict(ok=False, error="numbers"))
            cfg = cfg_get()
            lim = int(cfg.get("max_rec_mb", 4096)) << 20
            if not FILE_RE.match(fn) or ln < 0 or ln > CHUNK_MAX or off < 0 or total < 0 or off + ln > total:
                self._drain()
                return self._json(400, dict(ok=False, error="file/off/total/length"))
            if total > lim or rec_total > lim:
                self._drain()
                return self._json(413, dict(ok=False, error=f"recording larger than {lim >> 20} MB"))
            d = S.stage / n
            d.mkdir(parents=True, exist_ok=True)
            if staged_bytes(d) + ln > lim:
                self._drain()
                return self._json(413, dict(ok=False, error="recording too large"))
            if shutil.disk_usage(S.rec_root).free < ln + (2 << 30):
                self._drain()
                return self._json(507, dict(ok=False, error="PC disk full"))
            p = d / fn
            have = p.stat().st_size if p.exists() else 0
            if off != have:
                self._drain()
                return self._json(409, dict(ok=False, have=have))
            got = 0
            with open(p, "ab") as f:
                while got < ln:
                    b = self.rfile.read(min(1 << 20, ln - got))
                    if not b:
                        break
                    f.write(b)
                    got += len(b)
            os.utime(d)                                  # rec_sync: 최근에 받는 중 → adb 자동 가져오기 미룸
            S.note(n, got, rec_total)
            if got < ln:
                self.close_connection = True
                return self._json(400, dict(ok=False, error="short body", have=have + got))
            return self._json(200, dict(ok=True, have=have + got))

        def do_POST(self):
            path, q = self._q()
            if not self._auth():
                return
            if path != "/api/up/finish":
                self._drain()
                return self._json(404, dict(ok=False, error="path"))
            n = self._rec(q)
            if n is None:
                return
            ln = int(self.headers.get("Content-Length") or 0)
            if ln > 1 << 20:
                self._drain()
                return self._json(413, dict(ok=False, error="body"))
            try:
                want = json.loads(self.rfile.read(ln) or b"{}")["files"]
                assert isinstance(want, dict) and want and all(FILE_RE.match(k) for k in want)
            except (ValueError, KeyError, AssertionError, TypeError):
                return self._json(400, dict(ok=False, error="files"))
            return self._json(*finish(S, n, want))

    return H


def finish(S, n, want):
    d, final = S.stage / n, S.rec_root / n
    if (final / ".uploaded.json").is_file() and not d.is_dir():
        return 200, dict(ok=True, moved=False, note="already")
    bad = []
    t0 = time.time()
    for fn, w in want.items():
        p = d / fn
        if not p.is_file() or p.stat().st_size != int(w.get("size", -1)):
            bad.append(dict(file=fn, have=p.stat().st_size if p.is_file() else None, want=w.get("size")))
        elif sha256_file(p) != str(w.get("sha256", "")).lower():
            bad.append(dict(file=fn, error="sha256"))
            p.unlink()                                       # 내용이 틀림 → 처음부터 다시 받음
    extra = [p.name for p in d.iterdir() if p.is_file() and p.name not in want] if d.is_dir() else []
    for fn in extra:
        (d / fn).unlink()
    if bad:
        log(f"확인 실패 {n}: {bad}")
        return 409, dict(ok=False, error="mismatch", bad=bad)
    try:
        complete = bool(json.loads((d / "meta.json").read_text()).get("complete"))
    except (OSError, ValueError, AttributeError):
        complete = False
    if not complete:
        return 409, dict(ok=False, error="meta.json complete=false")
    total = sum(int(w["size"]) for w in want.values())
    (d / ".uploaded.json").write_text(json.dumps(dict(at=time.strftime("%Y-%m-%d %H:%M:%S"), files=want), indent=1))
    if final.exists():
        for sub in final.iterdir():                          # 옛 복원본 보존 (rec_sync._pull_safe 와 같은 규칙)
            if sub.is_dir() and sub.name.startswith("recon_"):
                shutil.move(str(sub), str(d / sub.name))
        shutil.rmtree(final)
    d.rename(final)
    with S.lock:
        a = S.active.pop(n, None)
        el = (a["t"] - a["t0"]) if a else 0
        S.recent.append(dict(name=n, mb=round(total / 1048576, 1), s=round(el, 1),
                             mbps=round(a["sess"] / 1048576 / max(el, 1e-3), 2) if a else None,
                             at=time.strftime("%H:%M:%S"), ok=True))
        S.dirty = True
    log(f"받음 {n}: {total / 1048576:.1f} MB, sha256 확인 {time.time() - t0:.1f} s → recordings/{n}")
    return 200, dict(ok=True, moved=True)


# ---------------- cloudflared ----------------
def find_cloudflared():
    for c in (os.environ.get("CLOUDFLARED"), os.path.expanduser("~/scanner/local/bin/cloudflared"),
              shutil.which("cloudflared")):
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return None


def tunnel_loop(S, port, stop):
    exe = find_cloudflared()
    if not exe:
        S.tunnel = "no cloudflared (~/scanner/local/bin/cloudflared)"
        log("cloudflared 없음 → 터널 없이 (폰은 업로드 못 함)")
        S.dirty = True
        return
    backoff = 5
    while not stop.is_set():
        t0 = time.time()
        S.tunnel = "starting"
        S.dirty = True
        log(f"cloudflared quick tunnel 시작 → http://127.0.0.1:{port}")
        with open(CF_LOG, "a", encoding="utf-8") as lf:
            proto = str(load_cfg().get("protocol") or "http2")
            p = subprocess.Popen([exe, "tunnel", "--no-autoupdate", "--protocol", proto, "--url", f"http://127.0.0.1:{port}"],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True, bufsize=1)
            for line in p.stderr:
                lf.write(line)
                lf.flush()
                m = CF_URL_RE.search(line)
                if m and m.group(0) != S.url:
                    S.url, S.url_t = m.group(0), time.strftime("%Y-%m-%d %H:%M:%S")
                    URL_FILE.write_text(S.url + "\n")
                    log(f"터널 주소: {S.url}")
                    S.pushed = None                              # 폰 설정 다시 넣기
                    S.dirty = True
                if "Registered tunnel connection" in line:
                    S.tunnel = "up"
                    S.dirty = True
                if stop.is_set():
                    p.terminate()
            p.wait()
        S.tunnel = f"exited rc={p.returncode}"
        S.dirty = True
        log(f"cloudflared 끝남 rc={p.returncode} → {backoff} s 뒤 다시 (주소가 바뀜)")
        backoff = 5 if time.time() - t0 > 300 else min(backoff * 2, 300)
        stop.wait(backoff)


# ---------------- 폰 설정 (adb) ----------------
def phone_cfg_text(S, token, cfg):
    url = cfg.get("public_url") or S.url
    if not url:
        return None
    return json.dumps(dict(url=url.rstrip("/"), token=token, enabled=bool(cfg.get("enabled", True)),
                           wifi_only=bool(cfg.get("wifi_only", True)), delete_after=bool(cfg.get("delete_after", False)),
                           chunk_mb=16, parallel=2, set_at=time.strftime("%Y-%m-%d %H:%M:%S")), separators=(",", ":"))


def push_phone(S, link, token, cfg, force=False):
    """폰의 upload.json 이 지금 설정과 다르면 다시 씀. 반환 True(맞음)/False(못 넣음)/None(폰 없음)."""
    txt = phone_cfg_text(S, token, cfg)
    if txt is None or not link.available or not link.devices():
        return None
    rc, cur = link.run("shell", f"cat {PHONE_CFG} 2>/dev/null", timeout=10)
    core = lambda s: {k: v for k, v in (json.loads(s) if s.strip().startswith("{") else {}).items() if k != "set_at"}
    try:
        same = core(cur) == core(txt)
    except ValueError:
        same = False
    if same and not force:
        return True
    b64 = base64.b64encode(txt.encode()).decode()
    d = PHONE_CFG.rsplit("/", 1)[0]
    rc, out = link.run("shell", f"mkdir -p {d} && echo {b64} | base64 -d > {PHONE_CFG}.part && mv {PHONE_CFG}.part {PHONE_CFG}",
                       timeout=15)
    if rc != 0:
        log(f"폰 설정 넣기 실패: {out}")
        return False
    log(f"폰에 업로드 설정을 넣음 ({json.loads(txt)['url']})")
    # 앱에 바로 알림 (앱이 안 떠 있으면 무시됨 — 다음 실행 때 읽음)
    link.run("shell", "am", "broadcast", "-a", phone_link.APP + ".UPLOAD", "-p", phone_link.APP, timeout=10)
    return True


def phone_loop(S, token, stop):
    link = phone_link.PhoneLink(log=log)
    while not stop.is_set():
        try:
            ok = push_phone(S, link, token, load_cfg())
            if ok is not None and ok != S.phone_cfg_ok:
                S.phone_cfg_ok = ok
                S.dirty = True
        except Exception as e:
            log(f"폰 설정 확인 오류 {e!r}")
        stop.wait(30)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--rec-root", default=str(HERE / "recordings"))
    ap.add_argument("--no-tunnel", action="store_true")
    ap.add_argument("--push-now", action="store_true", help="폰에 upload.json 만 넣고 끝 (runs/tunnel_url.txt 주소)")
    a = ap.parse_args()
    RUNS.mkdir(parents=True, exist_ok=True)
    cfg = load_cfg()
    token = load_token()
    S = State(a.rec_root)
    if a.push_now:
        try:
            S.url = URL_FILE.read_text().strip() or None
        except OSError:
            pass
        r = push_phone(S, phone_link.PhoneLink(log=log), token, cfg, force=True)
        print("ok" if r else f"실패 ({r})")
        return 0 if r else 1
    S.stage.mkdir(parents=True, exist_ok=True)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(S, token, load_cfg))
    srv.daemon_threads = True
    stop = threading.Event()
    log(f"업로드 서버 시작 127.0.0.1:{a.port}, 받는 곳 {S.stage}")
    if cfg.get("public_url"):
        S.url, S.tunnel = cfg["public_url"], "fixed"
    if not a.no_tunnel and cfg.get("tunnel", "quick") == "quick" and not cfg.get("public_url"):
        threading.Thread(target=tunnel_loop, args=(S, a.port, stop), daemon=True, name="tunnel").start()
    threading.Thread(target=phone_loop, args=(S, token, stop), daemon=True, name="phone").start()

    def saver():
        last = 0
        while not stop.is_set():
            if S.dirty or time.time() - last > 10:
                S.save()
                last = time.time()
            stop.wait(1)
    threading.Thread(target=saver, daemon=True, name="state").start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
