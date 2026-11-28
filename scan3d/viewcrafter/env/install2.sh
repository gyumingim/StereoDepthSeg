#!/bin/bash
# ViewCrafter reinstall 2026-11-28, step 2: pytorch3d from source (sm_120, nice, 6 jobs) + checkpoints into ViewCrafter/checkpoints
E=/home/system/scanner/viewcrafter_env
REQ=/home/system/scanner/StereoDepthSeg-gsmap/scan3d/viewcrafter/env
date
cd $E
if [ ! -d pytorch3d ]; then
  git clone -q https://github.com/facebookresearch/pytorch3d.git
  (cd pytorch3d && (git checkout -q V0.7.9 2>/dev/null || git checkout -q v0.7.9 2>/dev/null || echo "no 0.7.9 tag, HEAD used"))
fi
(cd pytorch3d && git log -1 --format='p3d %H %ci %s' && git describe --tags 2>/dev/null)
mkdir -p $E/logs
nice -n 10 bash $REQ/build_p3d.sh
echo "p3d build exit $?"
until grep -q DL_DONE $E/logs/dl.log; do sleep 20; done
# checkpoints: move downloads into the paths vc_run.py expects
mkdir -p $E/ViewCrafter/checkpoints/dust3r_hf
cd $E/ckpt_dl
[ -f model_512.ckpt ] && mv model_512.ckpt $E/ViewCrafter/checkpoints/model_512.ckpt
[ -f dust3r_model.safetensors ] && mv dust3r_model.safetensors $E/ViewCrafter/checkpoints/dust3r_hf/model.safetensors
cp dust3r_config.json $E/ViewCrafter/checkpoints/dust3r_hf/config.json
cd $E/ViewCrafter
nice -n 10 ../venv/bin/python $REQ/hf_dust3r_to_pth.py && sha256sum checkpoints/dust3r_hf/DUSt3R_512_dpt_from_hf.pth | tee -a $E/ckpt_dl/sha256.txt
ls -la checkpoints checkpoints/dust3r_hf
date
echo STEP2_DONE
