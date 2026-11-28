#!/bin/bash
# viewcrafter 2026-11-28 smoke test after reinstall: ONE ViewCrafter-512 clip (25 frames, 320x512) on recording 165506,
# single anchor kf (idx 24), our fused FFS+DA3 point cloud (no DUSt3R), same side orbit as 11-25 (vc_traj.py defaults).
#   setsid nohup bash scan3d/viewcrafter/run_smoke.sh > ~/scanner/viewcrafter_env/work/smoke.log 2>&1 &
# Expect (11-25 reference, n=1): ~45 s model load + ~41 s diffusion, peak ~10.9 GB (nvidia-smi).
G=$HOME/scanner/StereoDepthSeg-gsmap
R=$G/scan3d/runs/gsmap_20261125
WK=$HOME/scanner/viewcrafter_env/work
PY=$HOME/scanner/viewcrafter_env/venv/bin/python
OUT=${OUT:-$WK/smoke_165506_single_ours}
export SCAN3D_GPU_FRAC=${SCAN3D_GPU_FRAC:-0.85}
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8
mkdir -p $WK
cd $G || exit 1
[ -f $WK/traj.npz ] || nice -n 10 $HOME/scanner/gsmap_env/venv/bin/python scan3d/viewcrafter/vc_traj.py --dump $R/dump_ba --fuse $R/fuse/da3 --out $WK/traj.npz
cd $HOME/scanner/viewcrafter_env/ViewCrafter || exit 1
echo "== smoke $(date +%T)"
/usr/bin/time -v nice -n 10 $PY -W ignore $G/scan3d/viewcrafter/vc_run.py --traj $WK/traj.npz --dump $R/dump_ba --fuse $R/fuse/da3 \
    --mode single --pc ours --out $OUT
echo "exit $? $(date +%T)"
ls -la $OUT
