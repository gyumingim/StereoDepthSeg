#!/usr/bin/env python3
"""폰 녹화본 목록·가져오기 (표준 라이브러리만 — WSL/우분투/Windows 어디서나).

  python3 scan3d/pull_rec.py list                       # 폰 녹화 목록 (+ 이미 가져온 것 표시)
  python3 scan3d/pull_rec.py pull rec_20261016_120000   # → <저장소>/recordings/rec_20261016_120000/
  python3 scan3d/pull_rec.py pull --latest              # 가장 최근 것
  python3 scan3d/pull_rec.py pull --all                 # 아직 안 가져온 것 전부
  python3 scan3d/pull_rec.py local                      # 가져온 녹화 목록
옵션: --adb PATH (기본 SCAN3D_ADB > PATH > tools/platform-tools/adb > WSL 이면 Windows adb.exe 를 interop 으로),
      --dest DIR (기본 <저장소>/recordings). Windows 에서 실행하면 \\\\wsl.localhost\\Ubuntu-22.04\\... 경로로 바로 들어간다.
전송: adb exec-out [run-as com.camera.dualstream] tar -cf - -C <녹화 위치> <이름> 을 풀어서 저장 (실패하면 파일별 cat).
녹화 위치: /sdcard/Android/data/com.camera.dualstream/files/rec_*/ (RECORDING_FORMAT.md), 예비 앱 내부 files/rec_* — phone_link.py.
녹화 중인 폴더는 먼저 정지(웹 '정지' 또는 adb shell am broadcast -a com.camera.dualstream.STOP)한 뒤 가져올 것.
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import phone_link   # noqa: E402

DEFAULT_DEST = phone_link.REPO / "recordings"


def _progress(name):
    t0 = time.monotonic()

    def cb(done, total):
        el = max(1e-3, time.monotonic() - t0)
        pct = f"{100 * done / total:5.1f}%" if total else "  ?  "
        print(f"\r  {name}: {done / 1e6:8.1f} / {total / 1e6:.1f} MB {pct}  {done / 1e6 / el:5.1f} MB/s", end="", flush=True)
    return cb


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["list", "pull", "local"])
    ap.add_argument("names", nargs="*")
    ap.add_argument("--latest", action="store_true")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--adb", default=None)
    ap.add_argument("--dest", default=str(DEFAULT_DEST))
    a = ap.parse_args(argv)
    if a.action == "local":
        for r in phone_link.list_local(a.dest):
            print(f"{r['name']:32s} {r['size_mb']:8.1f} MB  video={r['has_video']}  complete={r['complete']}  "
                  f"recon={','.join(r['recon']) or '-'}")
        return 0
    link = phone_link.PhoneLink(a.adb, log=print)
    if not link.available:
        print("adb 를 찾지 못함: --adb 또는 SCAN3D_ADB 지정 (우분투: tools/platform-tools/adb, WSL: Windows adb.exe)")
        return 1
    print(f"adb: {link.adb} ({link.kind})")
    devs = link.devices()
    if not devs:
        print("연결된 기기 없음 (USB 디버깅 허용 확인)")
        return 1
    recs = link.list_recordings()
    local = {r["name"] for r in phone_link.list_local(a.dest)}
    if a.action == "list":
        if not recs:
            print(f"폰에 녹화 없음 (찾은 위치: {', '.join(link.rec_dirs)})")
        for r in recs:
            print(f"{r['name']:32s} {r['size_kb'] / 1024:8.1f} MB  {'(가져옴)' if r['name'] in local else ''}  [{r['base']}]")
        return 0
    if a.all:
        names = [r["name"] for r in recs if r["name"] not in local]
    elif a.latest:
        names = [recs[0]["name"]] if recs else []
    else:
        names = a.names
    if not names:
        print("가져올 녹화가 없음 (이름, --latest 또는 --all)")
        return 1
    rc = 0
    for n in names:
        try:
            p = link.pull(n, a.dest, progress=_progress(n))
            print(f"\n  → {p}")
        except Exception as e:
            print(f"\n  {n}: 실패 {e!r}")
            rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
