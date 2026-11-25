#!/bin/bash
# viewcrafter 2026-11-25: 4 ViewCrafter-512 runs on 165506 along the same side orbit (sequential, one GPU job at a time).
#   setsid nohup bash scan3d/viewcrafter/run_vc.sh > ~/scanner/viewcrafter_env/work/run_vc.log 2>&1 &
# Skips runs whose info.json exists. RUNS env var limits the list (e.g. RUNS="sparse_dust3r").
G=$HOME/scanner/StereoDepthSeg-gsmap
R=$G/scan3d/runs/gsmap_20261125
WK=$HOME/scanner/viewcrafter_env/work
PY=$HOME/scanner/viewcrafter_env/venv/bin/python
export SCAN3D_GPU_FRAC=${SCAN3D_GPU_FRAC:-0.85}
D3=${D3:-$HOME/scanner/viewcrafter_env/ViewCrafter/checkpoints/dust3r_hf/DUSt3R_512_dpt_from_hf.pth}
cd $HOME/scanner/viewcrafter_env/ViewCrafter || exit 1
for r in ${RUNS:-sparse_dust3r sparse_ours single_dust3r single_ours}; do
  m=${r%%_*}; pc=${r##*_}
  [ -f $WK/$r/info.json ] && { echo "skip $r"; continue; }
  extra=""; [ $pc = ours ] && extra="--npts-from $WK/${m}_dust3r/info.json"
  echo "== $r $(date +%T)"
  $PY -W ignore $G/scan3d/viewcrafter/vc_run.py --traj $WK/traj.npz --dump $R/dump_ba --fuse $R/fuse/da3 --mode $m --pc $pc \
      --out $WK/$r --dust3r $D3 $extra > $WK/$r.log 2>&1
  echo "   exit $? $(date +%T)"; tail -n 3 $WK/$r.log
done
echo ALLDONE $(date +%T)
