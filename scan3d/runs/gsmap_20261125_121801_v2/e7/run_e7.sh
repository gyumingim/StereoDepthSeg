#!/bin/bash
# E7: MoGe normals -> 2DGS 30k steps + per-image exposure + MoGe normal prior (0.05) on FUSE ($1) -> mesh ($2 = mesh env flags)
# -> v3fix -> carve -> held-out texture -> eval2. Sequential, resumable.
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; export GSMAP_RUN=$N; source $HOME/scanner/gsplat_env/env.sh
FUSE=$1; MESHENV=$2
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
TX="scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0"
d=$N/e7
[ -f $N/depth_moge_n/meta.json ] || { echo "== moge normals $(date +%T)"; $P scan3d/gsmap/depth_ai.py $N/dump_ba --model moge --out $N/depth_moge_n > $d/moge.log 2>&1 || { echo FAIL moge; exit 1; }; }
[ -f $d/gs/summary.json ] || { echo "== train $(date +%T)"; $P scan3d/gsmap/gsm_train.py $N/dump_ba --fused $FUSE --out $d/gs --surfel 1 --mask-border 1 --fill-w 0.1 --aniso-max 5 --aniso-w 1.0 --max-scale 0.05 --voxel 0.025 --max-gs 2500000 --steps 30000 --exposure 1 --normals $N/depth_moge_n --normal-w 0.05 > $d/train.log 2>&1 || { echo FAIL train; exit 1; }; }
[ -f $d/gs/mesh.json ] || { echo "== mesh $(date +%T)"; env GSMAP_WELD=1 GSMAP_TRUNC=0.04 $MESHENV $P scan3d/gsmap/gsm_mesh2d.py $d/gs $d/gs/mesh.ply > $d/mesh.log 2>&1 || { echo FAIL mesh; exit 1; }; }
[ -f $d/v3fix.json ] || { echo "== v3fix $(date +%T)"; $P scan3d/gsmap/gsm_v3fix.py $d/gs/mesh.ply $d/v3fix.ply > $d/v3fix.log 2>&1 || { echo FAIL v3fix; exit 1; }; }
[ -f $d/carve.json ] || { echo "== carve $(date +%T)"; $P scan3d/gsmap/gsm_carve.py $d/v3fix.ply $d/carve.ply --min-viol 5 --island-area 0 --min-tri 0 > $d/carve.log 2>&1 || { echo FAIL carve; exit 1; }; }
[ -f $d/held.glb ] || { echo "== tex $(date +%T)"; $P $TX --skip-held 1 $d/carve.ply $d/held.glb > $d/held.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $d/held.glb >> $d/held.log 2>&1 || { echo FAIL tex; exit 1; }; }
[ -f $d/eval.json ] || { echo "== eval $(date +%T)"; $P scan3d/gsmap/gsm_eval2.py $d/held.glb $d/eval.json --heldout-glb $d/held.glb --tex-json $d/held.json > $d/eval.log 2>&1 || echo FAIL eval; }
echo E7DONE $(date +%T)
