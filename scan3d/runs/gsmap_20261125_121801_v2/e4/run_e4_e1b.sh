#!/bin/bash
# E4: v4 pipeline with meshing range 5 m + depth-weighted sigma; E1b: support-based fragment removal on v4. Sequential.
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; export GSMAP_RUN=$N
source $HOME/scanner/gsplat_env/env.sh
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
TX="scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0"
d=$N/e4
[ -f $d/mesh.json ] || { echo "== e4 mesh $(date +%T)"; GSMAP_WELD=1 GSMAP_TRUNC=0.04 GSMAP_ZMAX=5.0 GSMAP_ZSIG=1 $P scan3d/gsmap/gsm_mesh2d.py $N/gs2DM $d/mesh.ply > $d/mesh.log 2>&1 || echo FAIL e4 mesh; }
[ -f $d/v3fix.json ] || { echo "== e4 v3fix $(date +%T)"; $P scan3d/gsmap/gsm_v3fix.py $d/mesh.ply $d/v3fix.ply > $d/v3fix.log 2>&1 || echo FAIL; }
[ -f $d/carve.json ] || { echo "== e4 carve $(date +%T)"; $P scan3d/gsmap/gsm_carve.py $d/v3fix.ply $d/carve.ply --min-viol 5 --island-area 0 --min-tri 0 > $d/carve.log 2>&1 || echo FAIL; }
[ -f $d/held.glb ] || { echo "== e4 tex $(date +%T)"; $P $TX --skip-held 1 $d/carve.ply $d/held.glb > $d/held.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $d/held.glb >> $d/held.log 2>&1 || echo FAIL; }
[ -f $d/eval.json ] || { echo "== e4 eval $(date +%T)"; $P scan3d/gsmap/gsm_eval2.py $d/held.glb $d/eval.json --heldout-glb $d/held.glb --tex-json $d/held.json > $d/eval.log 2>&1 || echo FAIL; }
d=$N/e1b
[ -f $d/carve.json ] || { echo "== e1b carve $(date +%T)"; $P scan3d/gsmap/gsm_carve.py $N/v3/mesh_v3.ply $d/carve.ply --min-viol 5 --island-area 0 --min-tri 0 --e1b 0.25 > $d/carve.log 2>&1 || echo FAIL; }
[ -f $d/held.glb ] || { echo "== e1b tex $(date +%T)"; $P $TX --skip-held 1 $d/carve.ply $d/held.glb > $d/held.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $d/held.glb >> $d/held.log 2>&1 || echo FAIL; }
[ -f $d/eval.json ] || { echo "== e1b eval $(date +%T)"; $P scan3d/gsmap/gsm_eval2.py $d/held.glb $d/eval.json --heldout-glb $d/held.glb --tex-json $d/held.json > $d/eval.log 2>&1 || echo FAIL; }
echo E4E1BDONE $(date +%T)
