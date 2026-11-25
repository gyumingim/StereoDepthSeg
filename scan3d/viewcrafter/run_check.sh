#!/bin/bash
# viewcrafter 2026-11-25 check with a known answer: path anchor kf 540 (idx 24) -> real held-out keyframe idx 64 (kf 970, splat
# test view, not a ViewCrafter input; idx 63-65 kept out of our point cloud). Last frame is compared with the real photo.
G=$HOME/scanner/StereoDepthSeg-gsmap
R=$G/scan3d/runs/gsmap_20261125
WK=$HOME/scanner/viewcrafter_env/work
PY=$HOME/scanner/viewcrafter_env/venv/bin/python
export SCAN3D_GPU_FRAC=${SCAN3D_GPU_FRAC:-0.85}
D3=$HOME/scanner/viewcrafter_env/ViewCrafter/checkpoints/dust3r_hf/DUSt3R_512_dpt_from_hf.pth
cd $G
$HOME/scanner/gsmap_env/venv/bin/python scan3d/viewcrafter/vc_traj.py --dump $R/dump_ba --fuse $R/fuse/da3 --out $WK/traj_check.npz --target 64 | grep -E "target|last"
source $HOME/scanner/gsplat_env/env.sh
$HOME/scanner/gsmap_env/venv/bin/python -W ignore scan3d/viewcrafter/vc_splat.py $R/gsA2 $WK/traj_check.npz $WK/chk_splat_gsA2 2>&1 | tail -1
cd $HOME/scanner/viewcrafter_env/ViewCrafter
for pc in dust3r ours; do
  [ -f $WK/chk_sparse_$pc/info.json ] && continue
  extra=""; [ $pc = ours ] && extra="--npts-from $WK/chk_sparse_dust3r/info.json --exclude 63 64 65"
  echo "== chk $pc $(date +%T)"
  $PY -W ignore $G/scan3d/viewcrafter/vc_run.py --traj $WK/traj_check.npz --dump $R/dump_ba --fuse $R/fuse/da3 --mode sparse --pc $pc \
      --out $WK/chk_sparse_$pc --dust3r $D3 $extra > $WK/chk_sparse_$pc.log 2>&1
  echo "   exit $? $(date +%T)"
done
echo ALLDONE $(date +%T)
