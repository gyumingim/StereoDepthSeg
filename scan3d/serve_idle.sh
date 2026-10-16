#!/usr/bin/env bash
# 대기 서버: 웹(http://localhost:8795)만 띄워 두고, 웹의 [스캔 시작] 때 OpenVINS 브리지·Windows 중계·폰 앱을 켠다.
# Windows 로그인 때 시작프로그램의 scan3d_server.vbs 가 창 없이 이 스크립트를 부른다.
#   bash scan3d/serve_idle.sh            # 이미 떠 있으면 아무것도 안 하고 끝남
#   bash scan3d/serve_idle.sh --voxel 0.02   # 나머지 인자는 scan3d.py 로
# 로그: scan3d/runs/server_<날짜>.log, OpenVINS 로그는 scan3d/runs/ov_bridge_live_<시각>.log
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
cd "$ROOT"
PY="${PY:-$ROOT/venv_ffs/bin/python}"
PORT="${PORT:-8795}"
export LD_LIBRARY_PATH="$HOME/scanner/local/lib:$HOME/scanner/src/ovbuild:${LD_LIBRARY_PATH:-}"
export PATH="$ROOT/tools/platform-tools:$PATH"

if ss -ltn "sport = :$PORT" 2>/dev/null | grep -q LISTEN; then
  echo "이미 실행 중 (포트 $PORT)"; exit 0
fi

OV_BRIDGE="${OV_BRIDGE:-$HERE/ov_bridge/build/ov_bridge}"
# OpenVINS 설정: 기본은 scan3d 가 세션 크기·fps 로 자동 (config/<W>x<H>_<fps>/). OV_CONFIG 를 주면 그 파일 고정.
OV_CFG_ARGS=()
if [ -n "${OV_CONFIG:-}" ]; then OV_CFG_ARGS=(--ov-config "$OV_CONFIG"); fi
if [ "${NO_VIO:-0}" = "1" ] || [ ! -x "$OV_BRIDGE" ] || { [ -n "${OV_CONFIG:-}" ] && [ ! -f "$OV_CONFIG" ]; }; then
  OV="none"
  echo "주의: 브리지 또는 설정 없음 → VIO 없이 (고정 자세)"
else
  # auto = [스캔 시작] 때 scan3d 가 브리지를 자식 프로세스로 띄움 (대기 중에는 안 돎, 죽으면 2 초 뒤 자동으로 다시)
  OV="auto"
fi

mkdir -p "$HERE/runs"
LOG="$HERE/runs/server_$(date +%Y%m%d).log"
echo "===== $(date '+%F %T') serve_idle 시작 =====" >>"$LOG"
# LINK=native (usbipd 로 폰 USB 를 WSL 에 붙였거나 우분투 직접): 리눅스 adb + adb forward, Windows 중계 없음.
# 설정 파일 runs/link.conf 에 LINK=native 한 줄을 두면 로그인 자동 시작도 그 모드로.
[ -f "$HERE/runs/link.conf" ] && . "$HERE/runs/link.conf"
LINK="${LINK:-relay}"
if [ "$LINK" = native ]; then PHONE="${PHONE:-127.0.0.1:8765}"; else PHONE="${PHONE:-listen:8766}"; fi
echo "link=$LINK phone=$PHONE" >>"$LOG"
exec "$PY" "$HERE/scan3d.py" --link "$LINK" --phone "$PHONE" --mode idle \
  --ov "$OV" ${OV_ARGS:+"--live-bridge-args=$OV_ARGS"} --ov-bridge "$OV_BRIDGE" ${OV_CFG_ARGS[@]+"${OV_CFG_ARGS[@]}"} \
  --port "$PORT" "$@" >>"$LOG" 2>&1
