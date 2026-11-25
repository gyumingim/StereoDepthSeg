#!/bin/bash
# diagnostic: held-out textures WITHOUT per-view gain for v4 and E6 (separates texture gain from geometry in P4)
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; export GSMAP_RUN=$N; D=$N/e6/nogain
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
TX="scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --skip-held 1 --no-gain 1"
for spec in "v4:$N/e1/carve_mv5_aonly.ply" "e6:$N/e6/carve.ply"; do
  n=${spec%%:*}; m=${spec#*:}
  [ -f $D/$n.glb ] || { $P $TX $m $D/$n.glb > $D/$n.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $D/$n.glb >> $D/$n.log 2>&1; }
  $P scan3d/gsmap/gsm_eval2.py $D/$n.glb $D/eval_$n.json --heldout-glb $D/$n.glb > $D/eval_$n.log 2>&1
done
echo NOGAINDONE
