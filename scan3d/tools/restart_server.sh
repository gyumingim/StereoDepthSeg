#!/usr/bin/env bash
# scan3d 대기 서버 다시 켜기 (WSL 쪽에서 부를 때). 하는 일은 restart_server.ps1 과 같다 — 그 파일을 Windows PowerShell 로 부른다.
#   bash scan3d/tools/restart_server.sh            # 스캔·복원 중이면 거절
#   bash scan3d/tools/restart_server.sh -Force     # 그래도 끔 (인자는 ps1 로 그대로)
# 왜 Windows 를 거치나: 서버는 로그인 때처럼 시작프로그램의 scan3d_server.vbs(창 없는 wsl.exe)로 띄워야
#   이 셸이 끝나도 같이 죽지 않는다. WSL interop 로 부른 PowerShell 은 환경 변수가 거의 없어서
#   (APPDATA·TEMP 없음) 경로는 여기 WSL 쪽에서 찾아 넘긴다.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PS=/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe
[ -x "$PS" ] || { echo "powershell.exe 없음: $PS" >&2; exit 1; }

VBS=""
for f in /mnt/c/Users/*/AppData/Roaming/Microsoft/Windows/Start\ Menu/Programs/Startup/scan3d_server.vbs; do
  [ -f "$f" ] && { VBS="$f"; break; }
done
[ -n "$VBS" ] || VBS="$(dirname "$HERE")/scan3d_server.vbs"
VBS_WIN="$(wslpath -w "$VBS")"
PS1_WIN="$(wslpath -w "$HERE/restart_server.ps1")"

# Windows 가 UNC 스크립트를 막지 않게 Bypass. 출력은 UTF-8 (ps1 이 OutputEncoding 을 맞춤), CR 은 지움.
"$PS" -NoProfile -ExecutionPolicy Bypass -File "$PS1_WIN" -Vbs "$VBS_WIN" "$@" | tr -d '\r'
exit "${PIPESTATUS[0]}"
