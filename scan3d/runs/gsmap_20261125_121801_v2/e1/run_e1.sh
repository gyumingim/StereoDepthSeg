#!/bin/bash
# E1 sweep: carve v3 mesh with min-viol 3 / 5 / 10, held-out texture, eval2. Waits for E0. One heavy job at a time.
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; export GSMAP_RUN=$N
until grep -q E0DONE $N/eval2/e0.log; do sleep 15; done
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
TX="scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1"
for mv in 3 5 10; do
  n=carve_mv$mv
  [ -f $N/e1/$n.json ] || { echo "== carve $mv $(date +%T)"; $P scan3d/gsmap/gsm_carve.py $N/v3/mesh_v3.ply $N/e1/$n.ply --min-viol $mv > $N/e1/$n.log 2>&1 || { echo "FAIL carve $mv"; continue; }; }
  [ -f $N/e1/held_$n.glb ] || { echo "== tex $mv $(date +%T)"; $P $TX --skip-held 1 $N/e1/$n.ply $N/e1/held_$n.glb > $N/e1/held_$n.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $N/e1/held_$n.glb >> $N/e1/held_$n.log 2>&1 || echo "FAIL tex $mv"; }
  [ -f $N/e1/eval_$n.json ] || { echo "== eval $mv $(date +%T)"; $P scan3d/gsmap/gsm_eval2.py $N/e1/held_$n.glb $N/e1/eval_$n.json --heldout-glb $N/e1/held_$n.glb --tex-json $N/e1/held_$n.json > $N/e1/eval_$n.log 2>&1 || echo "FAIL eval $mv"; }
done
echo E1SWEEPDONE $(date +%T)
