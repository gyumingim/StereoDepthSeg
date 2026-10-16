#!/usr/bin/env python3
"""Windows TCP relay: phone (adb forward 127.0.0.1:8765) <-> WSL2 (NAT).

WSL(NAT) 안에서는 Windows 127.0.0.1 의 adb forward 포트가 안 보인다. 이 스크립트를 Windows Python 3
(표준 라이브러리만)으로 돌려 바이트를 그대로 중계한다. 해석·재조립 없음 → WSL 쪽 소켓은 폰 소켓과 바이트 단위로 같다.

두 모드 (SPEC.md 8.3):

1) reverse (기본, 방화벽·관리자 권한 불필요)
   릴레이가 양쪽 모두에 "바깥으로" connect 한다. WSL2 localhost forwarding 은 Windows→WSL 방향만 되므로
   WSL 이 0.0.0.0:<wsl-port> 에 listen 하면 Windows 127.0.0.1:<wsl-port> 로 붙을 수 있다.
     a. 127.0.0.1:<wsl-port> 에 connect (0.5 s 간격 재시도, --connect-timeout 초)
     b. WSL 이 1 바이트 b"G" 를 보내면 127.0.0.1:<phone-port> (adb forward) 에 connect → 폰→WSL 복사,
        WSL→폰 방향은 그대로 폰에 전달 (폰 앱은 읽지 않음).  b"Q" 면 릴레이 종료.
     c. 어느 쪽이든 끊기면 양쪽 닫고 a 로 돌아감 (재연결 무제한).
   py -3 win_relay.py --wsl-port 8766 --phone-port 8765

2) listen (대안, 정방향)
   Windows 쪽 WSL 가상 어댑터 IP (vEthernet (WSL ...), 예 172.29.128.1) 의 <listen-port>(기본 18765) 에서 받아
   127.0.0.1:<phone-port> 로 양방향 중계. WSL 에서는 `ip route show default | awk '{print $3}'`:18765 로 connect.
   어댑터 IP 에만 bind 하는 이유: LAN 에 폰 스트림을 노출하지 않기 위해. 어댑터를 못 찾으면 --listen-host 로 지정,
   최후 수단 0.0.0.0 (모든 인터페이스 = LAN 노출, 방화벽이 막아줄 때만 사용).
   Windows Defender 방화벽 인바운드 허용(첫 실행 시 프롬프트) 또는 Hyper-V 방화벽 규칙이 필요할 수 있다.
   py -3 win_relay.py --mode listen --listen-port 18765

시작 시 앱을 띄우고(SPEC 1.5, --no-start-app 로 생략) adb forward tcp:<phone-port> tcp:8765 를 한다 (--no-forward 로 끔).
앱 모드: --app-mode live|record|both → am start ... --es mode <값> (record 는 폰에만 녹화, 스트림 없음 → 중계 안 함).
WSL 안에서는 USB 가 안 보이므로 forward 는 이 릴레이가 맡는다. 앱 시작·정지·녹화 목록은 WSL scan3d 가
interop 으로 Windows adb.exe 를 불러 직접 할 수도 있다 (웹 뷰어 모드 버튼; 그때는 --no-start-app 로 띄워도 됨).
폰 쪽이 바로 끊기면(앱이 죽음) 앱을 다시 시작하고 forward 를 다시 한다 (앱을 이 릴레이가 시작했을 때, 백오프).
Ctrl+C 로 언제든 끝난다 (모든 대기에 1 s 타임아웃).
adb 경로: --adb > 환경변수 SCAN3D_ADB > PATH 의 adb > %LOCALAPPDATA%\\Android\\Sdk\\platform-tools\\adb.exe.
짝: scan3d.py 기본 --phone listen:8766 (reverse). listen 모드면 scan3d 쪽 --phone :18765.
"""
import argparse
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time

BUF = 1 << 20


def log(*a):
    print(time.strftime("%H:%M:%S"), "[win_relay]", *a, flush=True)


APP = "com.camera.dualstream"


def adb_exe(adb):
    """--adb > SCAN3D_ADB > PATH > Android SDK 기본 위치."""
    cands = [adb, os.environ.get("SCAN3D_ADB"), shutil.which("adb")]
    la = os.environ.get("LOCALAPPDATA")
    if la:
        cands.append(os.path.join(la, "Android", "Sdk", "platform-tools", "adb.exe"))
    for c in cands:
        if c and (os.path.isfile(c) or shutil.which(c)):
            return c
    return "adb"


def adb_run(exe, *cmd, timeout=15):
    try:
        r = subprocess.run([exe, *cmd], capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr).strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        return -1, str(e)


def adb_forward(exe, phone_port, device_port):
    rc, out = adb_run(exe, "forward", f"tcp:{phone_port}", f"tcp:{device_port}")
    log("adb forward", phone_port, "->", device_port, "rc", rc, out)
    if rc:
        raise SystemExit("adb forward failed (USB 연결·adb 경로 확인: --adb 또는 SCAN3D_ADB)")


OK_STATES = ("STREAMING", "RECORDING", "READY", "WAITING", "IDLE")


def state_ok(st):
    """status.txt 첫 낱말이 STREAMING / RECORDING / RECORDING+STREAMING … 이면 True (RECORDING_FORMAT.md 1절)."""
    first = st.split()[0] if st.split() else ""
    return bool(first) and all(p in OK_STATES for p in first.split("+"))


def app_status(exe):
    """status.txt 마지막 줄. 못 읽으면(run-as 실패·시간 초과) "" — 오류 문구를 상태로 오해하지 않게."""
    rc, st = adb_run(exe, "shell", "run-as", APP, "cat", "files/status.txt", timeout=5)
    if rc != 0:
        return ""
    lines = st.strip().splitlines()
    return lines[-1].strip() if lines else ""


def first_word(st):
    return st.split()[0] if st and st.split() else ""


def is_recording(st):
    """첫 낱말의 + 조각에 RECORDING, 또는 OPEN mode=record|both (앱이 녹화를 먼저 연다). phone_link.is_recording 과 같은 규칙."""
    first = first_word(st)
    if "RECORDING" in first.split("+"):
        return True
    return first == "OPEN" and re.search(r"\bmode=(record|both)\b", st or "") is not None


def is_done_state(first):
    """정지 대기를 끝내도 되는 명시적 상태 (RECORD_DONE / STREAMING / STOPPED / *ERROR*). 빈 값·모르는 값은 False."""
    if not first or "RECORDING" in first.split("+"):
        return False
    return first.startswith(("RECORD_DONE", "STREAMING", "STOPPED")) or "ERROR" in first


def app_running(exe):
    """True / False / None(모름)."""
    rc, out = adb_run(exe, "shell", "pidof", APP, timeout=5)
    if rc == 0 and out.strip():
        return True
    if rc == 1 and not out.strip():
        return False
    return None


def stop_recording(exe, wait_s=8.0):
    """녹화를 정상 마무리(STOP broadcast → 끝 상태 대기, 절반 지나면 BACK). force-stop 은 부르는 쪽이 한다 (REVIEW2 1·4)."""
    adb_run(exe, "shell", "am", "broadcast", "-a", APP + ".STOP", "-p", APP)
    t_end = time.monotonic() + wait_s
    sent_back = False
    st = ""
    while time.monotonic() < t_end:
        st = app_status(exe)
        if is_done_state(first_word(st)):
            log("recording finished:", st)
            return True
        if not st and app_running(exe) is False:
            return True
        if not sent_back and time.monotonic() > t_end - wait_s / 2:
            adb_run(exe, "shell", "input", "keyevent", "KEYCODE_BACK")   # onStop 도 같은 마무리
            sent_back = True
        time.sleep(0.3)
    log("recording stop not confirmed in %.0f s (last status: %s)" % (wait_s, st or "-"))
    return False


def start_app(exe, args):
    """SPEC 1.5: (녹화 중이면 먼저 정상 정지) → pm enable → force-stop → status.txt 지움 → am start (--es mode) → STREAMING/RECORDING 대기."""
    st0 = app_status(exe)
    if is_recording(st0) or (not st0 and app_running(exe) is not False):
        log("app is recording or status unknown (%s) -> finish recording before restart" % (st0 or "-"))
        stop_recording(exe)
    adb_run(exe, "shell", "pm", "enable", APP)
    adb_run(exe, "shell", "am", "force-stop", APP)
    adb_run(exe, "shell", "run-as", APP, "rm", "-f", "files/status.txt", "files/cameras.json")
    rc, out = adb_run(exe, "shell", "am", "start", "-S", "-n", APP + "/.MainActivity",
                      "--es", "logical", "0", "--es", "physical", args.physical,
                      "--ei", "width", "640", "--ei", "height", "480", "--ei", "fps", str(args.fps),
                      "--ef", "focus_diopters", str(args.focus), "--ez", "raw_ts", "false",
                      "--ei", "imu_us", str(args.imu_us), "--es", "mode", args.app_mode)
    log("am start rc", rc, out.splitlines()[-1] if out else "")
    t_end = time.monotonic() + 12.0
    st = ""
    while time.monotonic() < t_end:
        st = app_status(exe)
        if state_ok(st):
            log("app:", st)
            return True
        if any(k in st for k in ("FAILED", "ERROR", "DISCONNECTED", "STOPPED")):
            break
        time.sleep(0.15)
    log("app not STREAMING (화면 켜짐·카메라 권한 확인):", st)
    return False


def wsl_adapter_ip():
    """ipconfig 에서 'vEthernet (WSL...)' 어댑터의 IPv4. 한국어 ipconfig 도 'IPv4' 문자열은 같다."""
    try:
        out = subprocess.run(["ipconfig"], capture_output=True).stdout
    except OSError:
        return None
    text = out.decode("mbcs" if os.name == "nt" else "utf-8", errors="replace")
    in_wsl = False
    for line in text.splitlines():
        if line and not line[0].isspace():          # 어댑터 제목 줄
            in_wsl = "WSL" in line
        elif in_wsl and "IPv4" in line:
            m = re.search(r"(\d+\.\d+\.\d+\.\d+)", line)
            if m:
                return m.group(1)
    return None


def nodelay(s):
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return s


def pump(src, dst, name, stop, counts=None):
    """src → dst 바이트 복사. 끝나면 stop 을 세우고 양쪽을 깨운다. counts[name] = 복사한 바이트."""
    buf = bytearray(BUF)
    view = memoryview(buf)
    n_total = 0
    try:
        while not stop.is_set():
            n = src.recv_into(buf)
            if n == 0:
                break
            dst.sendall(view[:n])
            n_total += n
            if counts is not None:
                counts[name] = n_total
    except OSError:
        pass
    finally:
        stop.set()
        for s in (src, dst):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        log(name, "closed after", n_total, "bytes")


def bridge(a, b, name_ab, name_ba):
    """양방향 복사, 한쪽이 끝나면 둘 다 닫고 반환 → {name: 바이트}. join 을 짧게 끊어 Ctrl+C 가 먹게."""
    stop = threading.Event()
    counts = {name_ab: 0, name_ba: 0}
    t1 = threading.Thread(target=pump, args=(a, b, name_ab, stop, counts), daemon=True)
    t2 = threading.Thread(target=pump, args=(b, a, name_ba, stop, counts), daemon=True)
    t1.start(); t2.start()
    try:
        while t1.is_alive() or t2.is_alive():
            t1.join(0.5); t2.join(0.5)
    finally:
        stop.set()
        a.close(); b.close()
    return counts


class AppKeeper:
    """폰 쪽이 바로 끊기면(앱 죽음) 앱 재시작 + forward 다시 (REVIEW 11). 앱을 이 릴레이가 띄웠을 때만."""

    def __init__(self, exe, args):
        self.exe, self.args = exe, args
        self.fails = 0
        self.next_try = 0.0

    def after_session(self, phone_bytes):
        if phone_bytes > 4096:
            self.fails = 0
            return
        self.fails += 1
        if self.args.no_start_app or self.args.app_mode == "record":
            return
        st = app_status(self.exe)
        m = re.search(r"\bmode=(\w+)", st or "")
        if m and m.group(1) != self.args.app_mode:
            # 웹 뷰어(scan3d 의 phone_link)가 다른 모드로 앱을 띄움 → 재시작은 그쪽이 맡는다 (녹화를 덮어쓰지 않게)
            log("app mode=%s (relay --app-mode %s) -> not restarting" % (m.group(1), self.args.app_mode))
            return
        first = first_word(st)
        if is_recording(st):
            # REVIEW2 5: RECORDING / RECORDING+STREAMING(both) → 재시작하면 녹화가 끊김. 접속만 다시 시도.
            if "STREAMING" in first.split("+") or first == "OPEN":
                log("phone side closed but app is recording (%s) -> not restarting" % st)
            return
        if first.startswith("RECORD_DONE"):
            return                                           # 녹화 정지 직후 → 스트림 없음이 정상
        if state_ok(st) and self.fails < 3:
            return                                           # 앱은 살아 있음 → 한두 번은 그냥 다시 시도
        now = time.monotonic()
        if now < self.next_try:
            return
        backoff = min(30.0, 2.0 * (2 ** min(self.fails, 4)))
        self.next_try = now + backoff
        log("phone side closed immediately (status: %s) -> restart app (next retry >= %.0f s)" % (st or "-", backoff))
        start_app(self.exe, self.args)
        if self.args.forward:
            try:
                adb_forward(self.exe, self.args.phone_port, self.args.device_port)
            except SystemExit as e:
                log(e)


def connect_retry(host, port, timeout_s):
    t_end = time.monotonic() + timeout_s if timeout_s > 0 else float("inf")   # 0 = 무한 재시도
    while True:
        try:
            return nodelay(socket.create_connection((host, port), timeout=2.0))
        except OSError as e:
            if time.monotonic() > t_end:
                raise TimeoutError(f"connect {host}:{port} failed: {e}")
            time.sleep(0.5)


def wait_cmd(w):
    """WSL 의 G/Q 1 바이트를 1 s 타임아웃 루프로 기다림 (Ctrl+C 가 먹게)."""
    w.settimeout(1.0)
    while True:
        try:
            return w.recv(1)
        except socket.timeout:
            continue
        except OSError:
            return b""


def run_reverse(args, keeper=None):
    while True:
        try:
            w = connect_retry("127.0.0.1", args.wsl_port, args.connect_timeout)
        except TimeoutError as e:
            log(e); return 1
        log("connected to WSL 127.0.0.1:%d, waiting for G/Q" % args.wsl_port)
        cmd = wait_cmd(w)
        w.settimeout(None)
        if cmd == b"Q":
            log("Q received, exit"); w.close(); return 0
        if cmd != b"G":
            log("WSL closed or bad command", cmd); w.close(); time.sleep(0.5); continue
        try:
            p = connect_retry("127.0.0.1", args.phone_port, 5.0)
            p.settimeout(None)
        except TimeoutError as e:
            log(e); w.close(); time.sleep(0.5); continue
        log("phone 127.0.0.1:%d <-> WSL relay running" % args.phone_port)
        counts = bridge(p, w, "phone->wsl", "wsl->phone")
        if keeper:
            keeper.after_session(counts.get("phone->wsl", 0))


def run_listen(args, keeper=None):
    host = args.listen_host or wsl_adapter_ip()
    if not host:
        log("WSL adapter IPv4 not found in ipconfig; use --listen-host (0.0.0.0 exposes the stream to the LAN)")
        return 1
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.bind((host, args.listen_port))
    srv.listen(1)
    srv.settimeout(1.0)
    log("listening on %s:%d -> phone 127.0.0.1:%d" % (host, args.listen_port, args.phone_port))
    while True:
        try:
            w, addr = srv.accept()
        except socket.timeout:
            continue
        w.settimeout(None)
        nodelay(w)
        log("WSL client", addr)
        try:
            p = connect_retry("127.0.0.1", args.phone_port, 5.0)
            p.settimeout(None)
        except TimeoutError as e:
            log(e); w.close(); continue
        counts = bridge(p, w, "phone->wsl", "wsl->phone")     # 한 번에 하나; 끝나면 다음 accept
        if keeper:
            keeper.after_session(counts.get("phone->wsl", 0))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["reverse", "listen"], default="reverse")
    ap.add_argument("--phone-port", type=int, default=8765, help="Windows-side adb forward port")
    ap.add_argument("--device-port", type=int, default=8765, help="app ServerSocket port on the phone")
    ap.add_argument("--wsl-port", type=int, default=8766, help="reverse: WSL listen port")
    ap.add_argument("--connect-timeout", type=float, default=0.0,
                    help="reverse: give up connecting to WSL after s (0 = retry forever)")
    ap.add_argument("--listen-host", default=None, help="listen: bind address (default: WSL vEthernet IPv4)")
    ap.add_argument("--listen-port", type=int, default=18765)
    ap.add_argument("--adb", default=None)
    ap.add_argument("--no-start-app", action="store_true", help="앱 시작(pm enable, am start, status 대기) 생략")
    ap.add_argument("--physical", default="2,5", help="앱 physical 인자 (스트림 0 = 왼쪽 렌즈 2)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--focus", type=float, default=1.0, help="focus_diopters (0 = AF)")
    ap.add_argument("--imu-us", type=int, default=2500, help="가속도·자이로 주기 µs (0 = FASTEST)")
    ap.add_argument("--app-mode", choices=["live", "record", "both"], default="live",
                    help="앱 --es mode: live = 스트림만, record = 폰에 녹화만, both = 녹화 + 스트림")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--forward", dest="forward", action="store_true", default=True, help="adb forward (기본)")
    g.add_argument("--no-forward", dest="forward", action="store_false")
    args = ap.parse_args()
    exe = adb_exe(args.adb)
    log("adb:", exe)
    if not args.no_start_app:
        start_app(exe, args)
    if args.forward:
        adb_forward(exe, args.phone_port, args.device_port)
    if args.app_mode == "record":
        log("app-mode record: 폰에만 녹화 (스트림 없음). 정지는 WSL 웹 뷰어 '정지' 또는 "
            "adb shell am broadcast -a %s.STOP" % APP)
    keeper = AppKeeper(exe, args)
    try:
        return run_reverse(args, keeper) if args.mode == "reverse" else run_listen(args, keeper)
    except KeyboardInterrupt:
        log("Ctrl+C")
        return 0


if __name__ == "__main__":
    sys.exit(main())
