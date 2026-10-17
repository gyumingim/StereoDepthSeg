#!/usr/bin/env python3
"""폰 무선 adb 자동 유지 (Windows Python). 15 s 마다 확인해서 Tailscale 무선 adb 가 끊겼으면 다시 붙인다.

  pythonw adb_keeper.py                 # 숨김 실행 (로그만): scan3d/runs/adb_keeper.log
  python  adb_keeper.py --once          # 한 번만 확인하고 결과 출력
  python  adb_keeper.py --once --dry-run   # 확인·포트 훑기만, adb connect 안 함
  python  adb_keeper.py --device s24 --interval 15

한 번 확인하는 순서
  1) Windows adb devices 에 기기가 있으면 (USB 든 무선이든, unauthorized 여도) → 아무것도 안 함.
  2) WSL 의 리눅스 adb 데몬이 떠 있고 거기에 기기가 있으면 (usbipd 로 WSL 에 붙은 상태) → 아무것도 안 함.
     (데몬이 안 떠 있으면 새로 띄우지 않는다.)
  3) tailscale status --json 에서 폰(이름에 --device 글자, 없으면 첫 android)이 온라인이면
     5555 와 30000~49999 TCP 포트를 비동기로 훑어 열린 포트마다 adb connect.
     offline 으로 붙으면 그 주소만 끊고 다음 포트. device 가 되면 성공 로그.
USB 로 연결된 기기·다른 주소는 절대 끊지 않는다. 무선 디버깅 '페어링'은 폰에서 한 번 해 둬야 한다 (OPERATIONS.md).
표준 라이브러리만 쓴다. 이미 하나가 돌고 있으면 (127.0.0.1:8797 잠금) 두 번째는 바로 끝난다.
"""
import argparse
import asyncio
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOG_DEFAULT = HERE.parent / "runs" / "adb_keeper.log"
DISTRO = "Ubuntu-22.04"
WSL_ADB = "/home/system/scanner/StereoDepthSeg/tools/platform-tools/adb"
LOCK_PORT = 8797
NOWIN = 0x08000000 if os.name == "nt" else 0          # CREATE_NO_WINDOW: pythonw 에서 콘솔 창이 번쩍이지 않게


def find_exe(cands):
    for p in cands:
        if p and Path(p).is_file():
            return str(p)
    return None


def run(cmd, timeout=15):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, creationflags=NOWIN,
                           encoding="utf-8", errors="replace")
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except (OSError, subprocess.SubprocessError) as e:
        return -1, str(e)


class Keeper:
    def __init__(self, a, log):
        self.a, self.log = a, log
        home = os.environ.get("USERPROFILE", r"C:\Users\a3162")
        self.adb = a.adb or find_exe([os.environ.get("SCAN3D_ADB"), r"C:\Users\a3162\platform-tools\adb.exe",
                                      os.path.join(home, "platform-tools", "adb.exe"),
                                      os.path.join(os.environ.get("LOCALAPPDATA", ""), "Android", "Sdk",
                                                   "platform-tools", "adb.exe")])
        self.ts = a.tailscale
        self.last_state = None

    # ---- 지금 상태 ----
    def win_devices(self):
        """Windows adb devices → [(serial, state)]."""
        rc, out = run([self.adb, "devices"])
        devs = []
        for line in out.splitlines()[1:]:
            parts = line.split()
            if len(parts) >= 2 and not line.startswith("*"):
                devs.append((parts[0], parts[1]))
        return devs

    def wsl_devices(self):
        """WSL 리눅스 adb 데몬이 이미 떠 있을 때만 그 기기 목록 (데몬을 새로 띄우지 않음)."""
        if os.name != "nt":
            return []
        sh = f"pgrep -x adb >/dev/null && {WSL_ADB} devices || true"
        rc, out = run(["wsl.exe", "-d", DISTRO, "-e", "bash", "-c", sh], timeout=20)
        return [ln.split()[0] for ln in out.splitlines()[1:] if ln.strip().endswith("device")]

    def phone_ip(self):
        """Tailscale 에서 폰 찾기 → (이름, IP, 온라인 여부) 또는 None."""
        rc, out = run([self.ts, "status", "--json"], timeout=15)
        try:
            st = json.loads(out[out.index("{"):])
        except ValueError:
            return None
        peers = [p for p in (st.get("Peer") or {}).values() if str(p.get("OS", "")).lower() == "android"]
        want = (self.a.device or "s24").lower()
        pick = [p for p in peers if want in str(p.get("HostName", "")).lower() or want in str(p.get("DNSName", "")).lower()]
        if not pick and not self.a.device:
            pick = peers[:1]
        if not pick:
            return None
        p = pick[0]
        ips = [ip for ip in (p.get("TailscaleIPs") or []) if "." in ip]
        return (p.get("HostName"), ips[0] if ips else None, bool(p.get("Online")))

    # ---- 포트 훑기·연결 ----
    async def _scan(self, ip, ports):
        sem = asyncio.Semaphore(self.a.conc)
        found = []

        async def probe(port):
            async with sem:
                try:
                    _, w = await asyncio.wait_for(asyncio.open_connection(ip, port), self.a.probe_timeout)
                    found.append(port)
                    w.close()
                except (OSError, asyncio.TimeoutError):
                    pass
        await asyncio.gather(*(probe(p) for p in ports))
        return sorted(found)

    def scan(self, ip):
        ports = [5555] + list(range(self.a.port_min, self.a.port_max + 1))
        t0 = time.time()
        found = asyncio.run(self._scan(ip, ports))
        self.log(f"포트 훑기 {ip}: {len(ports)} 개 {time.time() - t0:.1f} s → 열림 {found or '없음'}")
        return found

    def connect(self, ip, ports):
        for port in ports:
            addr = f"{ip}:{port}"
            rc, out = run([self.adb, "connect", addr], timeout=15)
            out = out.strip().replace("\n", " ")
            time.sleep(1.0)
            state = dict(self.win_devices()).get(addr)
            if state == "device":
                self.log(f"연결 성공 {addr} ({out})")
                return addr
            self.log(f"{addr}: {out or 'no output'} state={state}")
            if state in ("offline", "unauthorized", "connecting") or state is None and "connected" in out:
                run([self.adb, "disconnect", addr])      # 방금 우리가 붙인 주소만 끊음
        return None

    # ---- 한 번 확인 ----
    def check(self):
        if not self.adb:
            return self.report("no_adb", "Windows adb.exe 를 찾지 못함 (--adb 로 지정)")
        devs = self.win_devices()
        if devs:
            usb = [s for s, _ in devs if ":" not in s]
            kind = "USB" if usb else "무선"
            return self.report("connected", f"이미 연결됨 ({kind}): " + ", ".join(f"{s} {st}" for s, st in devs))
        wsl = self.wsl_devices()
        if wsl:
            return self.report("usbipd", f"WSL adb 에 연결됨 (usbipd): {', '.join(wsl)} → 할 일 없음")
        ph = self.phone_ip()
        if ph is None:
            return self.report("no_phone", "Tailscale 에 안드로이드 기기 없음 (또는 tailscale 실행 안 됨)")
        name, ip, online = ph
        if not online or not ip:
            return self.report("phone_offline", f"Tailscale 의 {name} 오프라인 (폰 화면 켜기·Tailscale 켜기)")
        self.log(f"Windows adb 에 기기 없음, Tailscale {name} {ip} 온라인 → 무선 디버깅 포트 찾기")
        ports = self.scan(ip)
        if not ports:
            return self.report("no_port", "열린 포트 없음 (폰 '무선 디버깅' 꺼짐?)", force=True)
        if self.a.dry_run:
            return self.report("dry_run", f"--dry-run: 연결 안 함 (후보 {ports})", force=True)
        addr = self.connect(ip, ports)
        if addr:
            return self.report("reconnected", f"다시 연결함 {addr}", force=True)
        return self.report("failed", "열린 포트에 adb 연결 실패 (폰에서 무선 디버깅 페어링 필요?)", force=True)

    def report(self, state, msg, force=False):
        """상태가 바뀔 때만 로그 (15 s 마다 같은 줄이 쌓이지 않게)."""
        if force or state != self.last_state or self.a.once:
            self.log(msg)
        self.last_state = state
        return state


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", help="Tailscale 기기 이름 일부 (기본: s24, 없으면 첫 android)")
    ap.add_argument("--interval", type=float, default=15.0)
    ap.add_argument("--once", action="store_true", help="한 번만 확인하고 끝")
    ap.add_argument("--dry-run", action="store_true", help="adb connect/disconnect 안 함")
    ap.add_argument("--adb", help="adb.exe 경로")
    ap.add_argument("--tailscale", default=r"C:\Program Files\Tailscale\tailscale.exe")
    ap.add_argument("--port-min", type=int, default=30000)
    ap.add_argument("--port-max", type=int, default=49999)
    ap.add_argument("--conc", type=int, default=800, help="동시 TCP 시도 수")
    ap.add_argument("--probe-timeout", type=float, default=0.8)
    ap.add_argument("--log", default=str(LOG_DEFAULT))
    a = ap.parse_args()

    logf = Path(a.log)
    logf.parent.mkdir(parents=True, exist_ok=True)
    quiet = sys.stdout is None or "pythonw" in Path(sys.executable).name.lower()

    def log(msg):
        line = time.strftime("%Y-%m-%d %H:%M:%S ") + msg
        try:
            with open(logf, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except OSError:
            pass
        if not quiet:
            try:
                print(line, flush=True)
            except (OSError, UnicodeEncodeError):
                pass

    if not a.once:
        lock = socket.socket()
        try:
            lock.bind(("127.0.0.1", LOCK_PORT))       # 한 번에 하나만 (프로세스가 끝나면 OS 가 풀어 줌)
        except OSError:
            log("이미 실행 중 (다른 adb_keeper) → 끝")
            return 0
        log(f"adb_keeper 시작 (간격 {a.interval:.0f} s, pid {os.getpid()})")
    k = Keeper(a, log)
    while True:
        try:
            st = k.check()
        except Exception as e:                          # 한 번 실패해도 계속 돈다
            log(f"오류: {e!r}")
            st = "error"
        if a.once:
            return 0 if st in ("connected", "usbipd", "reconnected", "dry_run") else 1
        time.sleep(a.interval)


if __name__ == "__main__":
    sys.exit(main())
