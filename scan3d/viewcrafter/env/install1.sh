#!/bin/bash
# ViewCrafter reinstall 2026-11-28, step 1: clone @97cdb86, venv py3.12, pip deps (network-bound, nice)
# --no-deps: the freeze is complete and has protobuf 3.20.3 next to tensorboard 2.21 (resolver refuses; works at runtime)
set -x
E=/home/system/scanner/viewcrafter_env
U=/home/system/.local/bin/uv
REQ=/home/system/scanner/StereoDepthSeg-gsmap/scan3d/viewcrafter/env
mkdir -p $E/logs $E/work
cd $E || exit 1
date
[ -d ViewCrafter ] || git clone -q https://github.com/Drexubery/ViewCrafter.git
cd ViewCrafter && git checkout -q 97cdb86 && git log -1 --format='%H %ci %s'
git apply --check $REQ/viewcrafter_local.patch && git apply $REQ/viewcrafter_local.patch && echo PATCH_OK
git status --short
cd $E
[ -d venv ] || $U venv -q -p 3.12 venv
grep -v -E '^(pytorch3d|torch==|torchvision|triton|nvidia-|cuda-)' $REQ/requirements_freeze.txt > $E/logs/req_nontorch.txt
nice -n 10 $U pip install --python venv/bin/python torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu128 2>&1 | tail -5
nice -n 10 $U pip install --python venv/bin/python --no-deps -r $E/logs/req_nontorch.txt 2>&1 | tail -5
venv/bin/python -c "import torch,torchvision;print(torch.__version__,torchvision.__version__,torch.version.cuda,torch.cuda.is_available(),torch.cuda.get_device_name(0),torch.cuda.get_arch_list())"
$U pip freeze --python venv/bin/python > $E/logs/freeze_after_step1.txt
diff <(grep -v pytorch3d $REQ/requirements_freeze.txt | sort) <(sort $E/logs/freeze_after_step1.txt) && echo FREEZE_SAME
du -sh $E/venv
date
echo STEP1_DONE
