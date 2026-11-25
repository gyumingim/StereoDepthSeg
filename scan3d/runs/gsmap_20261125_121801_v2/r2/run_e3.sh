#!/bin/bash
# E3 on v4: full-room planes -> held-out texture -> eval2. Waits for the v4 re-baseline.
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; export GSMAP_RUN=$N
until grep -q V4BASEDONE $N/r2/v4base.log; do sleep 15; done
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
TX="scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0"
[ -f $N/r2/e3.json ] || { echo "== e3 $(date +%T)"; $P scan3d/gsmap/gsm_e3planes.py $N/e1/carve_mv5_aonly.ply $N/r2/e3.ply > $N/r2/e3.log 2>&1 || { echo FAIL e3; exit 1; }; }
[ -f $N/r2/held_e3.glb ] || { echo "== tex $(date +%T)"; $P $TX --skip-held 1 $N/r2/e3.ply $N/r2/held_e3.glb > $N/r2/held_e3.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $N/r2/held_e3.glb >> $N/r2/held_e3.log 2>&1 || { echo FAIL tex; exit 1; }; }
echo "== eval $(date +%T)"; $P scan3d/gsmap/gsm_eval2.py $N/r2/held_e3.glb $N/r2/eval_e3.json --heldout-glb $N/r2/held_e3.glb --tex-json $N/r2/held_e3.json > $N/r2/eval_e3.log 2>&1 || echo FAIL eval
echo E3DONE $(date +%T)
