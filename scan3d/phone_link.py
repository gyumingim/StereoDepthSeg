"""폰 제어 (adb) — 앱 시작(모드 인자)·정지·status.txt·녹화 목록·녹화 가져오기. 표준 라이브러리만 (Windows 에서도 import 가능).

adb 찾기 (find_adb):
  1) 인자 / 환경변수 SCAN3D_ADB (Windows 경로 C:\\... 도 WSL 에서는 /mnt/c/... 로 바꿈)
  2) 우분투 직접: PATH 의 adb, 저장소 tools/platform-tools/adb, ~/Android/Sdk/platform-tools/adb
  3) WSL: /mnt/c/Users/a3162/platform-tools/adb.exe (이 PC 의 고정 위치, REVIEW4 B5), PATH 의 adb.exe,
     /mnt/c/Users/*/platform-tools/adb.exe, /mnt/c/Users/*/AppData/Local/Android/Sdk/platform-tools/adb.exe
     (WSL interop 로 Windows adb 실행)
  WSL 안의 리눅스 adb 는 USB 가 안 보이므로(usbipd 없으면) WSL 에서는 Windows adb.exe 를 먼저 쓴다.

앱 시작 인자 (SPEC 1.5 + 모드): am start -S -n com.camera.dualstream/.MainActivity --es logical 0 --es physical 2,5  (깊이 렌즈 쌍 2+6 이면 --es physical 2,6, scan3d.apply_geometry 가 바꿈)
  --ei width <W> --ei height <H> --ei fps <fps> (기본 640 480 15; scan3d 가 세션마다 width/height/fps 를 바꿈) --ef focus_diopters 1.0 --ez raw_ts false --ei imu_us 2500 --es mode <live|record|both> --es stream_fmt h264|jpeg|nv21 (기본 h264) --ei jpeg_q 90 [--ei live_kbps N] (SPEC 1.2)
  [--ei save_fps <fps>] (앱 1.8-subsample: capture_fps > fps 면 --ei fps <capture_fps> --ei save_fps <fps> = 빠르게 찍고 1/N 만
  저장·전송 → 두 렌즈 시각차 ≤ 반 촬영 주기, 녹화·스트림은 fps 그대로. 옛 앱은 save_fps 를 몰라 capture_fps 로 전부 보냄)
  --es sync off|auto|a|b (기본 off = 안 맞춤: S24 는 다시 열어도 위상이 안 맞아 PC 가 깊이에서 움직임 보정; auto = 카메라 다시 열기, 옵션) --ei rec_wait_sync_s 40 (녹화는 맞춘 뒤 시작, SPEC 15)
녹화 위치 (RECORDING_FORMAT.md 2절): /sdcard/Android/data/com.camera.dualstream/files/rec_YYYYMMDD_HHMMSS/ (adb 로 바로 읽힘),
  예비로 앱 내부 files/rec_* (run-as). 환경변수 SCAN3D_REC_DIRS(':' 구분)로 덮어씀. 폴더 이름이 rec_ 로 시작하는 것만.
status.txt (1절): STREAMING … | RECORDING rec=… | RECORDING+STREAMING rec=… | RECORD_DONE rec=… reason=… | STOPPED …
녹화 정지 (1절): am broadcast -a com.camera.dualstream.STOP → status 첫 낱말이 RECORD_DONE / STREAMING / STOPPED / *ERROR* 가
  될 때까지(최대 8 s, mp4 마무리 약 3 s; 빈 값·모르는 값이면 계속 읽음) → 그래도 안 되면 BACK 키(onStop 도 같은 마무리) → force-stop.
  force-stop 을 먼저 하면 mp4 moov 가 없어 재생 불가. start_app 도 앱이 녹화 중이면 이 정지를 먼저 한다 (둘은 lock 으로 직렬화).
"""
import glob
import io
import json
import os
import re
import shutil
import subprocess
import tarfile
import threading
import time
from pathlib import Path

APP = "com.camera.dualstream"
ACTIVITY = APP + "/.MainActivity"
STOP_ACTION = APP + ".STOP"
MODES = ("live", "record", "both")          # --es mode 값
REPO = Path(__file__).resolve().parent.parent
DEFAULT_REC_DIRS = (f"/sdcard/Android/data/{APP}/files", "files")
REC_PREFIX = "rec_"
ADB_FIXED_WIN = r"C:\Users\a3162\platform-tools\adb.exe"    # 오래 남는 위치 (Temp 정리에 안 지워짐, REVIEW4 B5)
OK_STATES = ("STREAMING", "RECORDING", "READY", "WAITING", "IDLE")


def state_ok(first):
    """'RECORDING+STREAMING' 처럼 + 로 이은 상태도 받는다."""
    return bool(first) and all(part in OK_STATES for part in first.split("+"))


BAD_WORDS = ("FAILED", "ERROR", "DISCONNECTED", "STOPPED")
DONE_STATES = ("RECORD_DONE", "STREAMING", "STOPPED")   # 녹화가 끝났다고 확실히 말하는 첫 낱말 (+ *ERROR*)


def is_recording(raw):
    """status 줄이 '녹화 중'인가: 첫 낱말의 + 조각에 RECORDING, 또는 OPEN mode=record|both (onCreate 가 녹화를 먼저 연다)."""
    st = parse_status(raw)
    first = st["state"]
    if "RECORDING" in first.split("+"):
        return True
    return first == "OPEN" and st.get("mode") in ("record", "both")


def is_done_state(first):
    """stop 뒤 기다림을 끝내도 되는 명시적 상태만 True. 빈 값·모르는 값(OPEN 등)은 False → 계속 기다림 (REVIEW2 4)."""
    if not first or "RECORDING" in first.split("+"):
        return False
    return first.startswith(DONE_STATES) or "ERROR" in first


def upload_on():
    """runs/upload.json 의 enabled (upload_server.py). 파일이 없으면 False = 옛 동작 (정지 때 force-stop)."""
    try:
        return bool(json.loads((Path(__file__).resolve().parent / "runs" / "upload.json").read_text()).get("enabled", True))
    except (OSError, ValueError, AttributeError):
        return False


def is_wsl():
    try:
        return "microsoft" in Path("/proc/version").read_text().lower()
    except OSError:
        return False


def _win_to_wsl(p):
    m = re.match(r"^([A-Za-z]):[\\/](.*)$", p or "")
    if not m or os.name == "nt":
        return p
    return f"/mnt/{m.group(1).lower()}/" + m.group(2).replace("\\", "/")


def find_adb(explicit=None):
    """(경로, 종류) 종류 = native | windows | None."""
    wsl = is_wsl()
    cands = []
    for c in (explicit, os.environ.get("SCAN3D_ADB")):
        if c:
            cands.append(_win_to_wsl(c))
    if os.name == "nt":
        cands += [ADB_FIXED_WIN, shutil.which("adb")]
        la = os.environ.get("LOCALAPPDATA")
        if la:
            cands.append(os.path.join(la, "Android", "Sdk", "platform-tools", "adb.exe"))
    else:
        linux = [shutil.which("adb"), str(REPO / "tools" / "platform-tools" / "adb"),
                 os.path.expanduser("~/Android/Sdk/platform-tools/adb")]
        win = []
        if wsl:
            win = [_win_to_wsl(ADB_FIXED_WIN), shutil.which("adb.exe")]
            win += sorted(glob.glob("/mnt/c/Users/*/platform-tools/adb.exe"))
            win += sorted(glob.glob("/mnt/c/Users/*/AppData/Local/Android/Sdk/platform-tools/adb.exe"))
            win += ["/mnt/c/platform-tools/adb.exe", "/mnt/c/Android/platform-tools/adb.exe"]
        cands += (win + linux) if wsl else linux
    for c in cands:
        if c and os.path.isfile(c) and (c.lower().endswith(".exe") or os.access(c, os.X_OK)):
            return c, ("windows" if c.lower().endswith(".exe") else "native")
    return None, None


def parse_status(raw):
    """'RECORDING t=12.3 frames=370 dir=rec_x' → {'state': 'RECORDING', 't': '12.3', ...}."""
    raw = (raw or "").strip()
    line = raw.splitlines()[-1] if raw else ""
    out = dict(raw=line, state=(line.split()[0] if line.split() else ""))
    for k, v in re.findall(r"([A-Za-z_][\w]*)=([^\s,]+)", line):
        out[k] = v
    return out


class PhoneLink:
    def __init__(self, adb=None, log=print, physical="2,5", fps=15, focus=1.0, imu_us=2500, rec_dirs=None,
                 stream_fmt="h264", jpeg_q=90, width=1440, height=1080, live_kbps=None, sync="off", rec_wait_sync_s=40):
        self.adb, self.kind = find_adb(adb)
        self.log = log
        self.physical, self.fps, self.focus, self.imu_us = physical, fps, focus, imu_us
        self.capture_fps = None                                     # 촬영 fps (None/0 = fps 그대로). fps 의 배수일 때만 씀
        self.exp_cap_us = 0                                         # 노출 상한 µs (0 = 자동 노출 그대로, 앱 1.12-shortexp)
        self.width, self.height = int(width), int(height)          # 스트림·녹화 크기 (4:3, scan3d.apply_geometry 가 바꿈)
        self.rec_kbps = None                                        # 녹화 H.264 kbps (None = 앱 자동, SPEC 14.5: 640x480·15 fps 에서 4000)
        self.stream_fmt, self.jpeg_q = stream_fmt, int(jpeg_q)      # 스트림 형식 (SPEC 1.2: 기본 h264, jpeg·nv21 은 고급)
        self.live_kbps = live_kbps                              # 렌즈당 H.264 스트림 kbps (None = 앱이 크기·fps 비례로 정함)
        self.sync = sync or "off"                               # 렌즈 위상 맞추기 (앱 --es sync, SPEC 15.3: 기본 off, auto = 방법 C)
        self.rec_wait_sync_s = int(rec_wait_sync_s)               # 녹화가 위상 맞춤을 기다리는 최대 초 (앱 --ei rec_wait_sync_s)
        env = os.environ.get("SCAN3D_REC_DIRS")
        self.rec_dirs = tuple(rec_dirs or (env.split(":") if env else DEFAULT_REC_DIRS))
        self.app_mode = None
        self.app_started = None
        self._app_lock = threading.RLock()       # start_app / stop_app 직렬화: 정지(mp4 마무리) 중에 force-stop 안 함

    @property
    def available(self):
        return self.adb is not None

    def info(self):
        return dict(available=self.available, kind=self.kind, adb=self.adb, app_mode=self.app_mode)

    # ---- adb 기본 ----
    def run(self, *args, timeout=15, text=True):
        if not self.adb:
            return -1, "adb 없음 (SCAN3D_ADB 또는 tools/platform-tools/adb)"
        try:
            r = subprocess.run([self.adb, *args], capture_output=True, timeout=timeout)
        except (OSError, subprocess.TimeoutExpired) as e:
            return -1, str(e)
        if not text:
            return r.returncode, r.stdout
        out = (r.stdout + r.stderr).decode("utf-8", "replace").replace("\r\n", "\n").strip()
        return r.returncode, out

    def shell(self, *args, timeout=15):
        return self.run("shell", *args, timeout=timeout)

    def devices(self):
        rc, out = self.run("devices")
        return [ln.split()[0] for ln in out.splitlines()[1:] if ln.strip().endswith("device")] if rc == 0 else []

    def forward(self, local=8765, remote=8765):
        rc, out = self.run("forward", f"tcp:{local}", f"tcp:{remote}")
        if rc:
            self.log(f"[adb] forward 실패 rc={rc} {out}")
        return rc == 0

    # ---- 앱 ----
    def status_raw(self):
        rc, out = self.shell("run-as", APP, "cat", "files/status.txt", timeout=6)
        return out if rc == 0 else ""

    def status(self):
        return parse_status(self.status_raw())

    def _rec_kbps(self):
        """녹화 비트레이트: 세션이 정한 값, 없으면 load_model 기준 (1440x1080·15 = 14 Mbps, 픽셀률 비례).
        앱 자체 기본식(640x480·15 = 4 Mbps 비례 → 1440 에서 20.25 Mbps)을 쓰지 않게 항상 넘긴다
        (2026-10-18: 웹 밖에서 시작한 녹화가 20 Mbps 로 찍혀 업로드가 밀림)."""
        if self.rec_kbps:
            return int(self.rec_kbps)
        try:
            import load_model
            return int(load_model.rec_kbps(dict(load_model.DEFAULT_COEFFS), {}, int(self.width), int(self.height),
                                           int(self.save_fps or self.fps) if hasattr(self, "save_fps") else int(self.fps)))
        except Exception:
            return 14000

    def start_app(self, mode="live", wait_s=12.0):
        """SPEC 1.5 + --es mode. 반환 (ok, status 줄)."""
        if mode not in MODES:
            raise ValueError(f"mode {mode} (live|record|both)")
        if not self.available:
            return False, "adb 없음"
        with self._app_lock:
            return self._start_app_locked(mode, wait_s)

    def app_running(self):
        """True / False / None(모름). pidof 는 프로세스가 없으면 rc=1 + 빈 출력."""
        rc, out = self.shell("pidof", APP, timeout=6)
        if rc == 0 and out.strip():
            return True
        if rc == 1 and not out.strip():
            return False
        return None

    def _start_app_locked(self, mode, wait_s):
        # REVIEW2 1: 녹화 중(또는 status 를 못 읽는데 앱이 살아 있음)이면 먼저 정상 정지 → mp4 마무리 뒤 force-stop
        raw = self.status_raw()
        if is_recording(raw) or (not raw.strip() and self.app_running() is not False):
            self.log(f"[adb] 앱이 녹화 중이거나 상태 모름 ({parse_status(raw)['raw'] or '-'}) → 먼저 녹화 정지")
            self._stop_app_locked(8.0)
        self.shell("pm", "enable", APP)
        self.shell("am", "force-stop", APP)
        self.shell("run-as", APP, "rm", "-f", "files/status.txt", "files/cameras.json")
        cap = self.capture_fps_used()
        rc, out = self.shell("am", "start", "-S", "-n", ACTIVITY,
                             "--es", "logical", "0", "--es", "physical", str(self.physical),
                             "--ei", "width", str(self.width), "--ei", "height", str(self.height),
                             "--ei", "fps", str(cap or self.fps),
                             *(("--ei", "save_fps", str(int(self.fps))) if cap else ()),   # 앱 1.8: cap fps 로 찍고 fps 만 저장
                             "--ef", "focus_diopters", str(self.focus), "--ez", "raw_ts", "false",
                             "--ei", "imu_us", str(self.imu_us), "--es", "mode", mode,
                             "--es", "stream_fmt", str(self.stream_fmt), "--ei", "jpeg_q", str(self.jpeg_q),
                             "--es", "sync", str(self.sync), "--ei", "rec_wait_sync_s", str(int(self.rec_wait_sync_s)),
                             *(("--ei", "live_kbps", str(int(self.live_kbps))) if self.live_kbps else ()),   # 앱 MainActivity live_kbps
                             "--ei", "rec_kbps", str(int(self._rec_kbps())),
                             *(("--ei", "exp_cap_us", str(int(self.exp_cap_us))) if self.exp_cap_us else ()),   # 앱 1.12: 짧은 노출
                             timeout=20)
        self.log(f"[adb] am start mode={mode} {self.width}x{self.height} "
                 f"{f'{cap} fps 촬영 → {self.fps} fps 저장' if cap else f'{self.fps} fps'} sync={self.sync} rc={rc} "
                 f"{out.splitlines()[-1] if out else ''}")
        self.app_mode, self.app_started = mode, time.time()
        t_end = time.monotonic() + wait_s
        st = ""
        while time.monotonic() < t_end:
            st = self.status_raw()
            first = st.split()[0] if st.split() else ""
            if state_ok(first):
                self.log(f"[adb] 앱: {st.splitlines()[-1]}")
                return True, st
            if any(k in st for k in BAD_WORDS):
                break
            time.sleep(0.2)
        self.log(f"[adb] 앱 시작 확인 실패 (화면 켜짐·카메라 권한 확인): {st}")
        return False, st

    def capture_fps_used(self):
        """앱에 줄 촬영 fps (save_fps 와 다를 때만), 아니면 None. fps 의 정수배이고 fps 보다 클 때만."""
        try:
            c, f = int(self.capture_fps or 0), int(self.fps)
        except (TypeError, ValueError):
            return None
        return c if c > f > 0 and c % f == 0 else None

    def stop_app(self, wait_s=8.0):
        """녹화를 닫고(MP4 마무리) 앱을 끈다."""
        if not self.available:
            return False
        with self._app_lock:
            return self._stop_app_locked(wait_s)

    def _stop_app_locked(self, wait_s):
        self.shell("am", "broadcast", "-a", STOP_ACTION, "-p", APP)
        t_end = time.monotonic() + wait_s
        sent_back = False
        st = raw = ""
        while time.monotonic() < t_end:
            raw = self.status_raw()
            st = parse_status(raw)["state"]
            # REVIEW2 4: 명시적 끝 상태(RECORD_DONE / STREAMING / STOPPED / *ERROR*)에서만 끝냄.
            # 빈 값(run-as 실패·시간 초과·앱이 status.txt 를 막 비움)이나 모르는 값이면 다시 읽으며 기다린다.
            if is_done_state(st):
                break
            if not raw.strip() and self.app_running() is False:
                st = "(앱 프로세스 없음)"
                break
            if not sent_back and time.monotonic() > t_end - wait_s / 2:
                self.shell("input", "keyevent", "KEYCODE_BACK")   # onStop 도 같은 마무리
                sent_back = True
            time.sleep(0.3)
        else:
            self.log(f"[adb] 정지 확인 시간 초과 ({wait_s:.0f} s, 마지막 상태 {st or '-'}) → force-stop")
        if (st.startswith("RECORD_DONE") or st.startswith("STOPPED")) and upload_on():
            # 업로드 방식 (upload_server.py): 앱이 녹화 끝에 UploadService 로 올리는 중 → force-stop 하면 끊긴다.
            # 녹화만(record)이면 앱이 카메라를 이미 닫음. 녹화+실시간(both)이면 BACK 으로 화면만 닫음 (onStop → 카메라 정지).
            if "STREAMING continues" in (raw or ""):
                self.shell("input", "keyevent", "KEYCODE_BACK")
            self.app_mode = None
            self.log("[adb] 앱 정지 (업로드 중일 수 있어 force-stop 안 함)")
            return True
        self.shell("am", "force-stop", APP)
        self.app_mode = None
        self.log("[adb] 앱 정지")
        return True

    # ---- 녹화 목록·가져오기 ----
    def _is_private(self, base):
        return not base.startswith("/")

    def _sh(self, base, *cmd, timeout=20, text=True):
        if self._is_private(base):
            return self.run("exec-out" if not text else "shell", "run-as", APP, *cmd, timeout=timeout, text=text)
        return self.run("exec-out" if not text else "shell", *cmd, timeout=timeout, text=text)

    def list_recordings(self):
        """[{name, base, size_kb}] (최근 이름순 내림차순)."""
        out = []
        for base in self.rec_dirs:
            rc, txt = self._sh(base, "du", "-k", "-d", "1", base)
            if rc != 0 or "No such file" in txt:
                continue
            for ln in txt.splitlines():
                parts = ln.split(None, 1)
                if len(parts) != 2 or not parts[0].isdigit():
                    continue
                path = parts[1].strip().rstrip("/")
                if path.rstrip("/") == base.rstrip("/"):
                    continue
                name = path.rsplit("/", 1)[-1]
                if not name.startswith(REC_PREFIX):
                    continue
                out.append(dict(name=name, base=base, size_kb=int(parts[0])))
            if out:
                break                                        # 첫 위치에 있으면 예비 위치는 안 본다
        out.sort(key=lambda r: r["name"], reverse=True)
        return out

    def _find(self, name):
        for r in self.list_recordings():
            if r["name"] == name:
                return r
        return None

    def pull(self, name, dest_root, progress=None):
        """녹화 폴더 name 을 dest_root/name 으로 복사. progress(done_bytes, total_bytes). 반환 Path."""
        if not re.match(r"^[\w.\-]+$", name):
            raise ValueError(f"녹화 이름이 이상함: {name!r}")
        rec = self._find(name)
        if rec is None:
            raise FileNotFoundError(f"폰에 녹화 {name} 없음 (위치 {self.rec_dirs})")
        dest_root = Path(dest_root)
        dest_root.mkdir(parents=True, exist_ok=True)
        tmp = dest_root / f".{name}.part"
        if tmp.exists():
            shutil.rmtree(tmp)
        tmp.mkdir()
        total = rec["size_kb"] * 1024
        try:
            ok = self._pull_tar(rec, tmp, total, progress)
            if not ok:
                self.log("[adb] tar 전송 실패 → 파일별 cat 으로 다시")
                shutil.rmtree(tmp); tmp.mkdir()
                self._pull_cat(rec, tmp, total, progress)
            src = tmp / name
            if not src.is_dir():
                raise RuntimeError("가져온 폴더가 비어 있음")
            final = dest_root / name
            if final.exists():
                shutil.rmtree(final)
            src.rename(final)
        finally:
            if tmp.exists():
                shutil.rmtree(tmp, ignore_errors=True)
        files = {p.name: p.stat().st_size for p in final.iterdir() if p.is_file()}
        (final / ".pulled.json").write_text(json.dumps(dict(
            pulled=time.strftime("%Y-%m-%d %H:%M:%S"), source=f"{rec['base']}/{name}", files=files), indent=1))
        self.log(f"[adb] 가져옴 {final} ({sum(files.values()) / 1e6:.1f} MB)")
        return final

    def _pull_tar(self, rec, tmp, total, progress):
        cmd = [self.adb, "exec-out"] + (["run-as", APP] if self._is_private(rec["base"]) else []) + \
              ["tar", "-cf", "-", "-C", rec["base"], rec["name"]]
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        except OSError as e:
            self.log(f"[adb] {e}")
            return False
        reader = _CountingReader(p.stdout, total, progress)
        n_files = 0
        try:
            with tarfile.open(fileobj=reader, mode="r|") as tf:
                for m in tf:
                    nm = m.name.lstrip("./")
                    if not (nm == rec["name"] or nm.startswith(rec["name"] + "/")) or ".." in nm.split("/"):
                        continue
                    if not (m.isdir() or m.isfile()):
                        continue
                    target = tmp / nm
                    if m.isdir():
                        target.mkdir(parents=True, exist_ok=True)
                        continue
                    target.parent.mkdir(parents=True, exist_ok=True)
                    f = tf.extractfile(m)
                    with open(target, "wb") as fo:
                        shutil.copyfileobj(f, fo, 1 << 20)
                    n_files += 1
        except (tarfile.TarError, OSError) as e:
            self.log(f"[adb] tar 해제 오류: {e}")
            n_files = 0
        finally:
            try:
                p.stdout.close()
            except OSError:
                pass
            p.wait(timeout=30)
        return n_files > 0 and p.returncode == 0

    def _pull_cat(self, rec, tmp, total, progress):
        base, name = rec["base"], rec["name"]
        rc, txt = self._sh(base, "ls", f"{base}/{name}")
        if rc:
            raise RuntimeError(f"ls 실패: {txt}")
        (tmp / name).mkdir(parents=True, exist_ok=True)
        done = 0
        for fn in txt.split():
            if not re.match(r"^[\w.\-]+$", fn):
                continue
            rc, data = self._sh(base, "cat", f"{base}/{name}/{fn}", timeout=600, text=False)
            if rc:
                raise RuntimeError(f"cat {fn} 실패")
            (tmp / name / fn).write_bytes(data)
            done += len(data)
            if progress:
                progress(done, total)


class _CountingReader(io.RawIOBase):
    def __init__(self, f, total, cb):
        self.f, self.total, self.cb, self.done = f, total, cb, 0
        self._t = 0.0

    def readable(self):
        return True

    def readinto(self, b):
        data = self.f.read(len(b))
        n = len(data)
        b[:n] = data
        self.done += n
        now = time.monotonic()
        if self.cb and (now - self._t > 0.3 or n == 0):
            self._t = now
            self.cb(self.done, self.total)
        return n

    def read(self, n=-1):
        if n is None or n < 0:
            data = self.f.read()
        else:
            data = self.f.read(n)
        self.done += len(data)
        now = time.monotonic()
        if self.cb and (now - self._t > 0.3 or not data):
            self._t = now
            self.cb(self.done, self.total)
        return data


def _meta_complete(d):
    try:
        return bool(json.loads((Path(d) / "meta.json").read_text()).get("complete"))
    except (OSError, ValueError, AttributeError):
        return None


def list_local(root):
    """저장소 recordings/ 아래 녹화 폴더 [{name, path, size_mb, has_video, recon:[...]}]."""
    root = Path(root)
    out = []
    if not root.is_dir():
        return out
    for d in sorted(root.iterdir(), reverse=True):
        if not d.is_dir() or d.name.startswith("."):
            continue
        files = [p for p in d.iterdir() if p.is_file()]
        recon = sorted([p.name for p in d.iterdir() if p.is_dir() and p.name.startswith("recon_")], reverse=True)
        out.append(dict(name=d.name, path=str(d), size_mb=round(sum(p.stat().st_size for p in files) / 1e6, 1),
                        has_video=any(p.suffix.lower() == ".mp4" for p in files), recon=recon,
                        complete=_meta_complete(d)))
    return out
