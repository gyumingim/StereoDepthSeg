#!/usr/bin/env bash
# scan3d 설치 (Ubuntu 22.04 / WSL Ubuntu-22.04, sudo 없이). 이 PC 에서 한 과정(/home/system/setup_scanner_wsl.sh)을 저장소 안에서 재현한다.
#
#   bash scan3d/setup.sh            # 전부: check python ffs ceres openvins bridge config
#   bash scan3d/setup.sh adb        # 우분투 직접 모드용 android platform-tools → tools/platform-tools/adb
#   bash scan3d/setup.sh bridge config   # 단계만 골라서
# 단계: check | python | ffs | ceres | openvins | bridge | config | adb | all
#
# 먼저 (관리자, 한 번): sudo apt install -y git build-essential cmake libeigen3-dev libopencv-dev libboost-all-dev curl unzip
# 환경변수:
#   SCANNER_HOME  Ceres/OpenVINS 소스·설치 위치 (기본 ~/scanner; ov_bridge/CMakeLists.txt 기본값과 같음)
#   TORCH_INDEX   torch 휠 주소 (기본 https://download.pytorch.org/whl/cu128 — RTX 50xx(sm_120) 용. 다른 GPU 는 cu124 등)
#   OPENVINS_REF  OpenVINS 커밋 (기본 이 PC 에서 쓴 6948812 = 69488123ed9362dd44b6f28e7f4680abbff1442b)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
SCANNER_HOME="${SCANNER_HOME:-$HOME/scanner}"
PREFIX="$SCANNER_HOME/local"
SRC="$SCANNER_HOME/src"
TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/cu128}"
OPENVINS_REF="${OPENVINS_REF:-69488123ed9362dd44b6f28e7f4680abbff1442b}"
CERES_TAG="${CERES_TAG:-2.2.0}"
PY="$ROOT/venv_ffs/bin/python"
JOBS="$(nproc 2>/dev/null || echo 4)"
export PATH="$HOME/.local/bin:$PATH"

say() { printf '\n=== %s ===\n' "$*"; }

step_check() {
  say "apt 패키지 확인 (설치는 sudo 로 직접)"
  local miss=()
  for p in git build-essential cmake libeigen3-dev libopencv-dev libboost-all-dev curl unzip; do
    dpkg -s "$p" >/dev/null 2>&1 || miss+=("$p")
  done
  if [ ${#miss[@]} -gt 0 ]; then
    echo "없는 패키지: ${miss[*]}"
    echo "  sudo apt install -y ${miss[*]}"
    return 1
  fi
  echo "OK"
  command -v nvidia-smi >/dev/null && nvidia-smi --query-gpu=name,driver_version --format=csv,noheader || echo "주의: nvidia-smi 없음 (GPU 드라이버 / WSL CUDA 확인)"
}

step_python() {
  say "uv + Python 3.12 + venv_ffs + torch ($TORCH_INDEX)"
  command -v uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | sh
  cd "$ROOT"
  uv python install 3.12
  [ -x "$PY" ] || uv venv --python 3.12 venv_ffs
  uv pip install --python "$PY" torch torchvision --index-url "$TORCH_INDEX"
}

step_ffs() {
  say "Fast-FoundationStereo 클론 + 가중치 + 파이썬 패키지"
  cd "$ROOT/tools"
  [ -d Fast-FoundationStereo/core ] || git clone --depth 1 https://github.com/NVlabs/Fast-FoundationStereo.git
  cd "$ROOT"
  local req
  req="$(mktemp)"
  grep -viE '^(torch|torchvision|xformers)' tools/Fast-FoundationStereo/requirements.txt > "$req" || true
  uv pip install --python "$PY" -r "$req" ultralytics open3d opencv-python scipy
  rm -f "$req"
  uv pip install --python "$PY" xformers --index-url "$TORCH_INDEX" || echo "xformers 생략 (없어도 동작)"
  mkdir -p tools/Fast-FoundationStereo/weights/c
  for f in cfg.yaml model_best_bp2_serialize.pth; do
    [ -s "tools/Fast-FoundationStereo/weights/c/$f" ] || \
      curl -L "https://huggingface.co/nvidia/c-fast-foundationstereo/resolve/main/$f" -o "tools/Fast-FoundationStereo/weights/c/$f"
  done
  "$PY" -c "import torch, cv2; print('torch', torch.__version__, 'cuda', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else '-'); print('opencv', cv2.__version__, 'ffmpeg', 'FFMPEG' in cv2.getBuildInformation())"
}

step_ceres() {
  say "Ceres $CERES_TAG → $PREFIX (사용자 prefix)"
  mkdir -p "$PREFIX" "$SRC"
  cd "$SRC"
  if [ -f "$PREFIX/lib/cmake/Ceres/CeresConfig.cmake" ]; then echo "이미 설치됨"; return 0; fi
  [ -d ceres-solver ] || git clone --depth 1 --branch "$CERES_TAG" https://github.com/ceres-solver/ceres-solver.git
  cmake -S ceres-solver -B ceres-build -DCMAKE_INSTALL_PREFIX="$PREFIX" -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    -DBUILD_BENCHMARKS=OFF -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DUSE_CUDA=OFF -DCMAKE_BUILD_TYPE=Release
  cmake --build ceres-build -j"$JOBS"
  cmake --install ceres-build
}

step_openvins() {
  say "OpenVINS (ROS 없음) $OPENVINS_REF → $SRC/ovbuild"
  mkdir -p "$SRC"
  cd "$SRC"
  if [ ! -d open_vins ]; then
    git clone https://github.com/rpng/open_vins.git
  fi
  git -C open_vins fetch --depth 1 origin "$OPENVINS_REF" 2>/dev/null || true
  git -C open_vins checkout -q "$OPENVINS_REF" || echo "주의: $OPENVINS_REF 체크아웃 실패 — 현재 커밋으로 빌드"
  cmake -S open_vins/ov_msckf -B ovbuild -DENABLE_ROS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PREFIX"
  cmake --build ovbuild -j"$JOBS"
  ls "$SRC/ovbuild"/libov_msckf_lib.so
}

step_bridge() {
  say "ov_bridge 빌드"
  cd "$HERE/ov_bridge"
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DOV_SRC="$SRC/open_vins" -DOV_BUILD="$SRC/ovbuild" -DOV_LOCAL="$PREFIX"
  cmake --build build -j"$JOBS"
  ls -la build/ov_bridge
}

step_config() {
  say "OpenVINS 설정 생성 (scan3d/config/*.yaml)"
  cd "$ROOT"
  [ -f calib_phone_pair.json ] || { echo "calib_phone_pair.json 없음 (스테레오 보정 먼저)"; return 1; }
  "$PY" scan3d/make_ov_config.py
  mkdir -p recordings
}

step_adb() {
  say "android platform-tools (리눅스) → tools/platform-tools"
  cd "$ROOT/tools"
  if [ ! -x platform-tools/adb ]; then
    curl -L -o platform-tools.zip https://dl.google.com/android/repository/platform-tools-latest-linux.zip
    unzip -q -o platform-tools.zip && rm -f platform-tools.zip
  fi
  platform-tools/adb version | head -1
  echo "USB 권한: 'adb devices' 가 no permissions 면 udev 규칙 필요 (README 참고). WSL 에서는 USB 가 안 보이므로 Windows adb.exe 를 쓴다."
}

STEPS=("$@")
[ ${#STEPS[@]} -eq 0 ] && STEPS=(all)
for s in "${STEPS[@]}"; do
  case "$s" in
    all) step_check || { echo "apt 패키지부터 설치"; exit 1; }
         step_python; step_ffs; step_ceres; step_openvins; step_bridge; step_config ;;
    check) step_check ;;
    python) step_python ;;
    ffs) step_ffs ;;
    ceres) step_ceres ;;
    openvins) step_openvins ;;
    bridge) step_bridge ;;
    config) step_config ;;
    adb) step_adb ;;
    *) echo "모르는 단계: $s (check|python|ffs|ceres|openvins|bridge|config|adb|all)"; exit 2 ;;
  esac
done
say "끝"
