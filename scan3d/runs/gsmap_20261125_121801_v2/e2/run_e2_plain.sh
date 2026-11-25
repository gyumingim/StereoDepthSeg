#!/bin/bash
# E2: TSDF re-extraction (min_obs 4 / 8 / 12, alpha 0.7, depth-edge mask, flagged Gaussians dropped) -> v3fix -> carve (mv5, a only)
# -> held-out texture -> eval2. Strictly sequential: the GPU mesh step never overlaps a CPU job.
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; export GSMAP_RUN=$N
source $HOME/scanner/gsplat_env/env.sh
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
TX="scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0"
for mo in ${MOS:-4 8 12}; do
  n=mo${mo}_plain; d=$N/e2/$n; mkdir -p $d
  [ -f $d/mesh.json ] || { echo "== mesh $mo $(date +%T)"; nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader; GSMAP_WELD=1 GSMAP_TRUNC=0.04 GSMAP_MINOBS=$mo GSMAP_ALPHA=0.5 GSMAP_EDGE=0 GSMAP_DROP_FLOATERS=1 $P scan3d/gsmap/gsm_mesh2d.py $N/gs2DM $d/mesh.ply > $d/mesh.log 2>&1 || { echo "FAIL mesh $mo"; continue; }; }
  [ -f $d/v3fix.json ] || { echo "== v3fix $mo $(date +%T)"; $P scan3d/gsmap/gsm_v3fix.py $d/mesh.ply $d/v3fix.ply > $d/v3fix.log 2>&1 || { echo "FAIL v3fix $mo"; continue; }; }
  [ -f $d/carve.json ] || { echo "== carve $mo $(date +%T)"; $P scan3d/gsmap/gsm_carve.py $d/v3fix.ply $d/carve.ply --min-viol 5 --island-area 0 --min-tri 0 > $d/carve.log 2>&1 || { echo "FAIL carve $mo"; continue; }; }
  [ -f $d/held.glb ] || { echo "== tex $mo $(date +%T)"; $P $TX --skip-held 1 $d/carve.ply $d/held.glb > $d/held.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $d/held.glb >> $d/held.log 2>&1 || { echo "FAIL tex $mo"; continue; }; }
  [ -f $d/eval.json ] || { echo "== eval $mo $(date +%T)"; $P scan3d/gsmap/gsm_eval2.py $d/held.glb $d/eval.json --heldout-glb $d/held.glb --tex-json $d/held.json > $d/eval.log 2>&1 || echo "FAIL eval $mo"; }
done
echo E2PLAINDONE $(date +%T)
