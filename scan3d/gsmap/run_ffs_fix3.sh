#!/bin/bash
# gsmap 2026-11-26 FFS fusion fixes (per-kf scale, partial BA/VIO rescale, dark pixels) on 121801. CPU only, 1 thread, nice 19 (power safety).
# usage: run_ffs_fix3.sh fuse|kfit|judge [args]   (OUT = ffs_fix3; plan + wide pairs read from ffs_4m)
cd "$HOME/scanner/StereoDepthSeg-gsmap"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 NUMBA_NUM_THREADS=1 CUDA_VISIBLE_DEVICES=
R=scan3d/runs/gsmap_20261125_121801; V2=scan3d/runs/gsmap_20261125_121801_v2; S=$V2/ffs_4m; O=${FIX3_OUT:-$V2/ffs_fix3}
VIO=/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/vio/B_o0.poses.jsonl.gz
P="nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore scan3d/gsmap/gsm_ffs_fix3.py"
mkdir -p $O
cmd=$1; shift
case "$cmd" in
  fuse)  $P fuse $R --vio $VIO --rec $R/rec --p2 $V2/ffs_pairs2 --p3 $V2/ffs_pairs3 --src $S --out $O --k 2 "$@" ;;
  kfit)  $P kfit $R "$@" ;;
  judge) $P judge $R --ref-set $S/s4m --gold $V2/eval2/ref/gold_ffs.npz --json $O/judge.json "$@" ;;
esac
echo EXIT $?
