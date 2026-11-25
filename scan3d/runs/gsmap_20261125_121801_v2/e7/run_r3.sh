#!/bin/bash
# judge round 3: after E7-MoGe, run E7 on the E6 DA3 fuse (mesh zmax 5 + depth sigma), then no-gain held-out textures + eval2 for both.
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; export GSMAP_RUN=$N
until grep -q "E7DONE\|FAIL" $N/e7/run.log; do sleep 20; done
OUT=$N/e7da3 bash $N/e7/run_e7b.sh $N/e6/fuse "GSMAP_ZMAX=5.0 GSMAP_ZSIG=1" > $N/e7da3/run.log 2>&1
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
TX="scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --skip-held 1 --no-gain 1"
for d in e7 e7da3; do
  [ -f $N/$d/held_ng.glb ] || { echo "== tex ng $d $(date +%T)"; $P $TX $N/$d/carve.ply $N/$d/held_ng.glb > $N/$d/held_ng.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $N/$d/held_ng.glb >> $N/$d/held_ng.log 2>&1; }
  [ -f $N/$d/eval_ng.json ] || { echo "== eval ng $d $(date +%T)"; $P scan3d/gsmap/gsm_eval2.py $N/$d/held_ng.glb $N/$d/eval_ng.json --heldout-glb $N/$d/held_ng.glb --tex-json $N/$d/held_ng.json > $N/$d/eval_ng.log 2>&1; }
done
echo R3DONE $(date +%T)
