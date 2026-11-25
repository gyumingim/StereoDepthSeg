#!/bin/bash
# E6: re-fuse (filtered FFS + DA3 pose-conditioned raw fill, multi-view gate >= 2 kf with >= 10 cm baseline within 3 %, zmax 5 m)
# -> retrain 2DGS 12k (same flags as run_v2.sh) -> mesh t4 (zmax 5, depth-weighted sigma) -> v3fix -> carve mv5 -> held-out texture -> eval2.
# Strictly sequential (GPU steps never overlap CPU-heavy steps). Resumable.
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; export GSMAP_RUN=$N; source $HOME/scanner/gsplat_env/env.sh
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
TX="scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0"
d=$N/e6
[ -f $d/fuse/fuse_info.json ] || { echo "== fuse $(date +%T)"; $P scan3d/gsmap/gsm_e6fuse.py $d/fuse --src da3 --da3 $N/depth_da3 --zmax 5 --gate 2 > $d/fuse.log 2>&1 || { echo FAIL fuse; exit 1; }; }
[ -f $d/gs/summary.json ] || { echo "== train $(date +%T)"; $P scan3d/gsmap/gsm_train.py $N/dump_ba --fused $d/fuse --out $d/gs --surfel 1 --mask-border 1 --fill-w 0.1 --aniso-max 5 --aniso-w 1.0 --max-scale 0.05 --voxel 0.025 --max-gs 2500000 --steps 12000 > $d/train.log 2>&1 || { echo FAIL train; exit 1; }; }
[ -f $d/gs/mesh.json ] || { echo "== mesh $(date +%T)"; GSMAP_WELD=1 GSMAP_TRUNC=0.04 GSMAP_ZMAX=5.0 GSMAP_ZSIG=1 $P scan3d/gsmap/gsm_mesh2d.py $d/gs $d/gs/mesh.ply > $d/mesh.log 2>&1 || { echo FAIL mesh; exit 1; }; }
[ -f $d/v3fix.json ] || { echo "== v3fix $(date +%T)"; $P scan3d/gsmap/gsm_v3fix.py $d/gs/mesh.ply $d/v3fix.ply > $d/v3fix.log 2>&1 || { echo FAIL v3fix; exit 1; }; }
[ -f $d/carve.json ] || { echo "== carve $(date +%T)"; $P scan3d/gsmap/gsm_carve.py $d/v3fix.ply $d/carve.ply --min-viol 5 --island-area 0 --min-tri 0 > $d/carve.log 2>&1 || { echo FAIL carve; exit 1; }; }
[ -f $d/held.glb ] || { echo "== tex $(date +%T)"; $P $TX --skip-held 1 $d/carve.ply $d/held.glb > $d/held.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $d/held.glb >> $d/held.log 2>&1 || { echo FAIL tex; exit 1; }; }
[ -f $d/eval.json ] || { echo "== eval $(date +%T)"; $P scan3d/gsmap/gsm_eval2.py $d/held.glb $d/eval.json --heldout-glb $d/held.glb --tex-json $d/held.json > $d/eval.log 2>&1 || echo FAIL eval; }
echo E6DONE $(date +%T)
