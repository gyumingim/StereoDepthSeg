#!/bin/bash
# gsmap 2026-11-26 FFS pairs3 = pairs2 + low-angle tier (partial rectification) on 121801. usage: run_ffs_pairs3.sh plan|ffs|fuse|measure
# Power safety: one heavy job at a time; CPU under nice 19, 8 threads.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8
R=scan3d/runs/gsmap_20261125_121801; V2=scan3d/runs/gsmap_20261125_121801_v2; P2O=$V2/ffs_pairs2; O=$V2/ffs_pairs3
VIO=/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/vio/B_o0.poses.jsonl.gz
P="nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
A="$R --vio $VIO --rec $R/rec --p2 $P2O --out $O --min-perp 40 --fb-perp 30 --dir-deg 10 --n-pairs 4 --k 2"
mkdir -p $O
case "$1" in
  plan) $P scan3d/gsmap/gsm_ffs_pairs3.py plan $A ;;
  ffs)  FFS_ROOT=$HOME/scanner/StereoDepthSeg nice -n 19 $HOME/scanner/StereoDepthSeg/venv_ffs/bin/python -W ignore scan3d/gsmap/gsm_ffs_pairs3.py ffs $A ;;
  fuse) $P scan3d/gsmap/gsm_ffs_pairs3.py fuse $A ;;
  measure) $P scan3d/gsmap/gsm_ffs_pairs3.py measure $A --gsmap-run $PWD/$V2 \
           --fig /mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo/ffs_lowangle_cmp.jpg --fig-kf 762 ;;
esac
