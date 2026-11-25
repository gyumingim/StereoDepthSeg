#!/bin/bash
# gsmap 2026-11-25: mesh-mode pipeline on hs_20261112_121801_std60 (user order). Same steps as gsmap_20261125/run_2dm.sh on 165506.
# Poses = BA keyframe poses of fixes fix_u8 (494 kf). lensonly "GT" there = raw VIO poses + 1000 m z (stand-in, NOT a GT).
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
source $HOME/scanner/gsplat_env/env.sh
P=$HOME/scanner/gsmap_env/venv/bin/python; PF=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
R=scan3d/runs/gsmap_20261125_121801; export GSMAP_RUN=$PWD/$R; export FFS_ROOT=$HOME/scanner/StereoDepthSeg
D=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo
BA=$HOME/scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/ba/poses_ba.jsonl.gz
t(){ echo "== $1 $(date +%T)"; }
set -o pipefail
if [ ! -f $R/dump_ba/cams_lensonly.npz ]; then
  t dump_ffs; $PF scan3d/gsplat/scan_offline_dump.py $R/rec --gt lensonly --kf-step 1 --out $R/tsdf_dump --dump $R/dump_ba --vio $BA > $R/dump.log 2>&1 || { echo FAIL dump; exit 1; }
  $P scan3d/gsmap/gsm_reframe.py $R/dump_ba || exit 1
fi
[ -f $R/depth_moge/meta.json ] || { t moge; $P -W ignore scan3d/gsmap/depth_ai.py $R/dump_ba --model moge --out $R/depth_moge > $R/moge.log 2>&1 || { echo FAIL moge; exit 1; }; }
[ -f $R/fuse/align_stats.json ] || { t align; $P scan3d/gsmap/align_fuse.py --dump $R/dump_ba --ai moge=$R/depth_moge --out $R/fuse > $R/align.log 2>&1 || { echo FAIL align; exit 1; }; }
[ -f $R/fuse/moge_f/mesh_info.json ] || { t filter; GSMAP_NO_MESH=1 $P -W ignore scan3d/gsmap/gsm_filtmesh.py $R/fuse/none.ply > $R/filter.log 2>&1 || { echo FAIL filter; exit 1; }; }
[ -f $R/gs2DM/summary.json ] || { t train; $P -W ignore scan3d/gsmap/gsm_train.py $R/dump_ba --fused $R/fuse/moge_f --out $R/gs2DM --surfel 1 --mask-border 1 --fill-w 0.1 --aniso-max 5 --aniso-w 1.0 --max-scale 0.05 --voxel 0.025 --max-gs 2500000 > $R/train.log 2>&1 || { echo FAIL train; exit 1; }; }
[ -f $R/gs2DM/mesh2d.json ] || { t mesh; $P -W ignore scan3d/gsmap/gsm_mesh2d.py $R/gs2DM $R/gs2DM/mesh2d.ply > $R/mesh.log 2>&1 || { echo FAIL mesh; exit 1; }; }
t planefix; $P -W ignore scan3d/gsmap/gsm_planefix.py $R/gs2DM/mesh2d.ply $R/gs2DM/mesh2d_pf.ply --min-inl 1500 --min-fill 0.8 --min-area 0.09 --max-behind 0 > $R/planefix.log 2>&1 || { echo FAIL planefix; exit 1; }
t texture; OMP_NUM_THREADS=8 $P -W ignore scan3d/gsmap/gsm_texture2.py $R/gs2DM/mesh2d_pf.ply $R/tex_121801.glb --dump $R/dump_ba --rec $R/rec --gt lensonly > $R/texture.log 2>&1 || { echo FAIL texture; exit 1; }
t resize; $P -W ignore scan3d/gsmap/gsm_atlas8k.py $R/tex_121801.glb >> $R/texture.log 2>&1
t check; $P -W ignore scan3d/gsmap/gsm_check.py $R $R/tex_121801.glb $R/check_121801.jpg > $R/check.log 2>&1
cp $R/tex_121801.glb $R/check_121801.jpg $D/ && echo COPIED
echo ALLDONE $(date +%T)
