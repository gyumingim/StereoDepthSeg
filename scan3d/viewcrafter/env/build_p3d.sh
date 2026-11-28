#!/bin/bash
# build pytorch3d from source for torch 2.11 cu128 / sm_120 (own venv only)
export CUDA_HOME=/home/system/scanner/gsplat_env/cuda128
export PATH=$CUDA_HOME/bin:/home/system/.local/bin:/usr/local/bin:/usr/bin:/bin
export TORCH_CUDA_ARCH_LIST=12.0
export FORCE_CUDA=1
export MAX_JOBS=6
NV=/home/system/scanner/viewcrafter_env/venv/lib/python3.12/site-packages/nvidia
export CPATH=$CUDA_HOME/targets/x86_64-linux/include:$(ls -d $NV/*/include | tr '\n' ':')
export NVCC_FLAGS="$(ls -d $NV/*/include | sed 's/^/-I/' | paste -sd' ')"
grep -n NVCC_FLAGS setup.py pytorch3d/setup.py 2>/dev/null
cd /home/system/scanner/viewcrafter_env || exit 1
date
[ -d pytorch3d ] || git clone -q https://github.com/facebookresearch/pytorch3d.git
cd pytorch3d && git log -1 --oneline
uv pip install --python ../venv/bin/python --no-build-isolation -v . > ../logs/p3d_full.log 2>&1
date
cd /tmp && /home/system/scanner/viewcrafter_env/venv/bin/python -c "import torch, pytorch3d, pytorch3d._C; print('p3d ok', pytorch3d.__version__)"
