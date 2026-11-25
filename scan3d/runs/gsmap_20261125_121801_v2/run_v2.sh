#!/bin/bash
# gsmap 2026-11-25 121801 v2 (diag fixes): FFS hybrid (old 361 + 50 new dense-partner kf), MoGe for no-FFS kf aligned to
# neighbour FFS (weak -> dropped), multi-view filter, 2DGS 12k steps (caps), mesh weld + trunc 4 cm and 6 cm (eval picks).
# Resumable: each stage skips when its output exists. Strictly one heavy job (power-offs).
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
source $HOME/scanner/gsplat_env/env.sh
P="nice -n 10 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"; export OMP_NUM_THREADS=8
R=scan3d/runs/gsmap_20261125_121801_v2; export GSMAP_RUN=$PWD/$R
t(){ echo "== $1 $(date +%T)"; }
[ -f $R/fuse/align_stats.json ] || { t align; $P scan3d/gsmap/align_fuse.py --dump $R/dump_ba --ai moge=$R/depth_moge --out $R/fuse > $R/align.log 2>&1 || { echo FAIL align; exit 1; }; }
[ -f $R/fuse/align_nb.json ] || { t align_nb; $P scan3d/gsmap/gsm_align_nb.py > $R/align_nb.log 2>&1 || { echo FAIL align_nb; exit 1; }; }
[ -f $R/fuse/moge_f/mesh_info.json ] || { t filter; GSMAP_NO_MESH=1 $P scan3d/gsmap/gsm_filtmesh.py $R/fuse/none.ply > $R/filter.log 2>&1 || { echo FAIL filter; exit 1; }; }
[ -f $R/gs2DM/summary.json ] || { t train; $P scan3d/gsmap/gsm_train.py $R/dump_ba --fused $R/fuse/moge_f --out $R/gs2DM --surfel 1 --mask-border 1 --fill-w 0.1 --aniso-max 5 --aniso-w 1.0 --max-scale 0.05 --voxel 0.025 --max-gs 2500000 --steps 12000 > $R/train.log 2>&1 || { echo FAIL train; exit 1; }; }
for tr in 0.04 0.06; do
  n=mesh_t${tr#0.0}
  [ -f $R/gs2DM/$n.json ] || { t mesh$tr; GSMAP_WELD=1 GSMAP_TRUNC=$tr GSMAP_CACHE=$R/gs2DM/render_cache.npz $P scan3d/gsmap/gsm_mesh2d.py $R/gs2DM $R/gs2DM/$n.ply > $R/$n.log 2>&1 || { echo FAIL mesh$tr; exit 1; }; }
done
echo PART1DONE $(date +%T)
