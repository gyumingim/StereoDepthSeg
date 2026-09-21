#!/usr/bin/env bash
# setup_ffs.sh — Fast-FoundationStereo 실행 환경(venv_ffs) 재현 스크립트
# 근거: 저장소 README (python 3.12, torch 2.6.0+cu124, xformers) + requirements.txt
# 메인 venv(3.10, torch 2.14)와 고정 버전이 달라 별도 venv 를 쓴다. 가중치는 HF C 버전(비게이트).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"; ROOT="$(dirname "$HERE")"
cd "$HERE"
[ -d Fast-FoundationStereo/core ] || git clone --depth 1 https://github.com/NVlabs/Fast-FoundationStereo.git
[ -x "$ROOT/venv_ffs/bin/pip" ] || python3.12 -m venv "$ROOT/venv_ffs"
"$ROOT/venv_ffs/bin/pip" install torch==2.6.0 torchvision==0.21.0 xformers --index-url https://download.pytorch.org/whl/cu124
"$ROOT/venv_ffs/bin/pip" install -r Fast-FoundationStereo/requirements.txt ultralytics
mkdir -p Fast-FoundationStereo/weights/c && cd Fast-FoundationStereo/weights/c
for f in cfg.yaml model_best_bp2_serialize.pth; do
  [ -s "$f" ] || curl -L "https://huggingface.co/nvidia/c-fast-foundationstereo/resolve/main/$f" -o "$f"
done
"$ROOT/venv_ffs/bin/python" -c "import torch, xformers, ultralytics; print('OK torch', torch.__version__, 'cuda', torch.cuda.is_available())"
[ -f /usr/include/python3.12/Python.h ] || echo "주의: python3.12-dev 없음 -> triton 컴파일 불가, 순수 PyTorch 경로(--scale 0.5 권장). sudo apt install python3.12-dev"
echo "완료. 실행: $ROOT/venv_ffs/bin/python $ROOT/z_ffs_smoke.py"
