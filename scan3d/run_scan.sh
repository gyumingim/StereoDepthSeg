#!/usr/bin/env bash
# WSL/우분투에서 실행: scan3d.py (OpenVINS 브리지는 scan3d 가 자식 프로세스로 띄움, 웹 뷰어 http://localhost:8795).
#
#   bash scan3d/run_scan.sh                    # 연결 자동: WSL 이면 relay (Windows 에서 win_relay.py 먼저), 아니면 native
#   bash scan3d/run_scan.sh --native           # 우분투 직접: adb forward tcp:8765 → 127.0.0.1:8765, 앱 시작도 직접
#   bash scan3d/run_scan.sh --relay            # WSL + Windows win_relay.py 중계 (phone listen:8766)
#   bash scan3d/run_scan.sh --replay recordings/rec_X [--every-frame]   # 녹화 재생만 하고 종료 (실시간 브리지 안 띄움)
#   bash scan3d/run_scan.sh --mode idle        # 웹에서 모드 고르기 (실시간 / 녹화 / 녹화+실시간 / 복원)
#   NO_VIO=1 bash scan3d/run_scan.sh           # 브리지 없이 고정 포즈로 깊이·분할만
#   bash scan3d/run_scan.sh --voxel 0.02 --yolo-rotate --iters 4 --stride 3   # 나머지 인자는 scan3d.py 로
#
# 환경변수로 덮어쓰기:
#   OV_BRIDGE   브리지 실행 파일 (기본: scan3d/ov_bridge/build/ov_bridge)
#   OV_CONFIG   estimator_config.yaml 고정 (기본: 없음 = scan3d 가 세션 크기·fps 로 config/<W>x<H>_<fps>/ 자동 생성)
#   OV_ARGS     실시간 브리지 추가 인자 (기본 "--auto-reset"; scan3d 가 "<config> --stdio" 앞에 붙여 띄움)
#   OV_ADDR     따로 띄운 TCP 브리지 주소 (예 127.0.0.1:7600). 비우면 scan3d 가 브리지를 직접 띄우고 죽으면 재시작
#   VIO 모드    --vio-mode stereo (기본, 두 렌즈 쌍만) | --vio-mode mono (왼쪽 렌즈만, config/mono 설정) — scan3d.py 인자
#   PHONE       폰 주소 (relay 기본 "listen:8766", native 기본 "127.0.0.1:8765")
#   SCAN3D_ADB  adb 경로 (native: PATH 의 adb 또는 tools/platform-tools/adb, WSL: Windows adb.exe 를 interop 으로)
#   NO_VIO=1    브리지 없이 고정 포즈로 깊이·분할만
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
cd "$ROOT"
PY="${PY:-$ROOT/venv_ffs/bin/python}"
OV_PORT="${OV_PORT:-7600}"
export LD_LIBRARY_PATH="$HOME/scanner/local/lib:$HOME/scanner/src/ovbuild:${LD_LIBRARY_PATH:-}"
export PATH="$ROOT/tools/platform-tools:$PATH"

first_existing() { for f in "$@"; do [ -e "$f" ] && { echo "$f"; return 0; }; done; echo "$1"; }

# --- 이 스크립트 인자 (나머지는 scan3d.py 로) ---
LINK=auto
REPLAY=""
PASS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --native) LINK=native ;;
    --relay) LINK=relay ;;
    --replay) REPLAY="${2:?--replay DIR}"; shift ;;
    --replay=*) REPLAY="${1#--replay=}" ;;
    *) PASS+=("$1") ;;
  esac
  shift
done
if [ "$LINK" = auto ]; then
  if grep -qi microsoft /proc/version 2>/dev/null; then LINK=relay; else LINK=native; fi
fi
if [ -z "${PHONE:-}" ]; then
  if [ "$LINK" = relay ]; then PHONE="listen:8766"; else PHONE="127.0.0.1:8765"; fi
fi
if [ "$LINK" = native ] && ! command -v adb >/dev/null 2>&1 && [ -z "${SCAN3D_ADB:-}" ]; then
  echo "주의: adb 없음 — 'bash scan3d/setup.sh adb' 로 tools/platform-tools 를 받거나 SCAN3D_ADB 지정 (앱 시작·forward 불가)"
fi
echo "link=$LINK phone=$PHONE"
if [ -z "$REPLAY" ] && ss -ltn "sport = :${PORT:-8795}" 2>/dev/null | grep -q LISTEN; then
  echo "이미 스캐너 서버가 켜져 있습니다 (포트 ${PORT:-8795}, 보통 로그인 때 켜지는 대기 서버)."
  echo "→ 브라우저에서 http://localhost:${PORT:-8795} 를 쓰거나, 먼저 scan3d\\stop_scanner.bat 로 끄세요."
  exit 1
fi

OV_BRIDGE="${OV_BRIDGE:-$(first_existing "$HERE/ov_bridge/build/ov_bridge")}"
COMMON=(--link "$LINK" --phone "$PHONE" --ov-bridge "$OV_BRIDGE")
if [ -n "${OV_CONFIG:-}" ]; then COMMON+=(--ov-config "$OV_CONFIG"); fi

if [ "${NO_VIO:-0}" = "1" ]; then
  if [ -n "$REPLAY" ]; then
    exec "$PY" "$HERE/scan3d.py" "${COMMON[@]}" --ov none --mode idle --replay "$REPLAY" --exit-after-replay ${PASS[@]+"${PASS[@]}"}
  fi
  exec "$PY" "$HERE/scan3d.py" "${COMMON[@]}" --ov none ${PASS[@]+"${PASS[@]}"}
fi

if [ ! -x "$OV_BRIDGE" ]; then echo "브리지 실행 파일 없음: $OV_BRIDGE (OV_BRIDGE 로 지정, bash scan3d/setup.sh bridge, 또는 NO_VIO=1)"; exit 1; fi
if [ -n "${OV_CONFIG:-}" ] && [ ! -f "$OV_CONFIG" ]; then echo "OpenVINS 설정 없음: $OV_CONFIG (venv_ffs/bin/python scan3d/make_ov_config.py 로 생성)"; exit 1; fi

if [ -n "$REPLAY" ]; then
  # 재생은 scan3d 가 전용 브리지(pipe, 새 상태)를 직접 띄운다
  exec "$PY" "$HERE/scan3d.py" "${COMMON[@]}" --mode idle --replay "$REPLAY" --exit-after-replay ${PASS[@]+"${PASS[@]}"}
fi

# 실시간 브리지도 scan3d 가 자식 프로세스로 직접 띄운다 (--ov auto): 죽으면 몇 초 안에 다시 띄우고 VIO 리셋 규칙을 탄다.
# 로그: scan3d/runs/ov_bridge_live_<시각>.log. 따로 띄운 TCP 브리지를 쓰려면 OV_ADDR=127.0.0.1:7600.
if [ -n "${OV_ADDR:-}" ]; then
  exec "$PY" "$HERE/scan3d.py" "${COMMON[@]}" --ov "$OV_ADDR" ${PASS[@]+"${PASS[@]}"}
fi
exec "$PY" "$HERE/scan3d.py" "${COMMON[@]}" --ov auto ${OV_ARGS:+"--live-bridge-args=$OV_ARGS"} ${PASS[@]+"${PASS[@]}"}
