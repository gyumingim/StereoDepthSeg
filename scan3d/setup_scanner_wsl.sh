#!/bin/bash
# WSL (Ubuntu-22.04, no sudo): StereoDepthSeg + FFS env (Python 3.12, Blackwell torch) + Ceres + OpenVINS (ROS-free).
set -x
LOG=~/scanner_setup.log
exec > >(tee -a "$LOG") 2>&1
mkdir -p ~/scanner && cd ~/scanner

# 1) StereoDepthSeg
[ -d StereoDepthSeg ] || git clone https://github.com/gyumingim/StereoDepthSeg.git
cd ~/scanner/StereoDepthSeg

# 2) uv + Python 3.12 + venv_ffs (RTX 5080 = sm_120 -> torch cu128 build)
if ! command -v uv >/dev/null; then curl -LsSf https://astral.sh/uv/install.sh | sh; fi
export PATH=$HOME/.local/bin:$PATH
uv python install 3.12
[ -x venv_ffs/bin/python ] || uv venv --python 3.12 venv_ffs
uv pip install --python venv_ffs/bin/python torch torchvision --index-url https://download.pytorch.org/whl/cu126
cd tools
[ -d Fast-FoundationStereo/core ] || git clone --depth 1 https://github.com/NVlabs/Fast-FoundationStereo.git
cd ~/scanner/StereoDepthSeg
grep -viE '^(torch|torchvision|xformers)' tools/Fast-FoundationStereo/requirements.txt > /tmp/ffs_req.txt || true
uv pip install --python venv_ffs/bin/python -r /tmp/ffs_req.txt ultralytics open3d opencv-python
uv pip install --python venv_ffs/bin/python xformers --index-url https://download.pytorch.org/whl/cu126 || echo "xformers skipped"
mkdir -p tools/Fast-FoundationStereo/weights/c && cd tools/Fast-FoundationStereo/weights/c
for f in cfg.yaml model_best_bp2_serialize.pth; do
  [ -s "$f" ] || curl -L "https://huggingface.co/nvidia/c-fast-foundationstereo/resolve/main/$f" -o "$f"
done
cd ~/scanner/StereoDepthSeg
venv_ffs/bin/python -c "import torch; print('TORCH', torch.__version__, 'cuda', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else '-')"

# 3) Ceres 2.2 (user prefix)
PREFIX=~/scanner/local
mkdir -p $PREFIX ~/scanner/src && cd ~/scanner/src
if [ ! -f $PREFIX/lib/cmake/Ceres/CeresConfig.cmake ]; then
  [ -d ceres-solver ] || git clone --depth 1 --branch 2.2.0 https://github.com/ceres-solver/ceres-solver.git
  cmake -S ceres-solver -B ceres-build -DCMAKE_INSTALL_PREFIX=$PREFIX -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DUSE_CUDA=OFF -DCMAKE_BUILD_TYPE=Release
  cmake --build ceres-build -j"$(nproc)" && cmake --install ceres-build
fi

# 4) OpenVINS (ROS-free)
[ -d open_vins ] || git clone --depth 1 https://github.com/rpng/open_vins.git
cmake -S open_vins/ov_msckf -B ovbuild -DENABLE_ROS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$PREFIX
cmake --build ovbuild -j"$(nproc)"
ls -la ovbuild | head -30
echo SETUP_DONE
