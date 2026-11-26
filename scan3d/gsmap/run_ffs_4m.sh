#!/bin/bash
# gsmap 2026-11-26 FFS 4 m (wide pairs + error cut) on top of the adopted pairs3 L set, 121801. usage: run_ffs_4m.sh plan|ffs|fuse|measure [extra args]
# Power safety: one heavy job at a time; CPU under nice 19, 8 threads.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8
R=scan3d/runs/gsmap_20261125_121801; V2=scan3d/runs/gsmap_20261125_121801_v2; O=$V2/ffs_4m
VIO=/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/vio/B_o0.poses.jsonl.gz
P="nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
A="$R --vio $VIO --rec $R/rec --p2 $V2/ffs_pairs2 --p3 $V2/ffs_pairs3 --out $O --min-perp 40 --fb-perp 30 --dir-deg 10 --n-pairs 4 --k 2"
FIG=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo/ffs_4m_cmp.jpg
mkdir -p $O
cmd=$1; shift
case "$cmd" in
  plan) $P scan3d/gsmap/gsm_ffs_4m.py plan $A "$@" ;;
  ffs)  FFS_ROOT=$HOME/scanner/StereoDepthSeg nice -n 19 $HOME/scanner/StereoDepthSeg/venv_ffs/bin/python -W ignore scan3d/gsmap/gsm_ffs_4m.py ffs $A "$@" ;;
  fuse) $P scan3d/gsmap/gsm_ffs_4m.py fuse $A "$@" ;;
  measure) $P scan3d/gsmap/gsm_ffs_4m.py measure $A --gsmap-run $PWD/$V2 "$@" ;;
esac
echo EXIT $?
