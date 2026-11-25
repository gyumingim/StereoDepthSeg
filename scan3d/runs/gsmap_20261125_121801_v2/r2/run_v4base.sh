#!/bin/bash
# judge round 1: re-baseline v4 with the new eval2 (P2 inside defects, P4 960 render, kf 0 dropped from P4 and texturing)
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; export GSMAP_RUN=$N
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
TX="scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0"
[ -f $N/r2/held_v4.glb ] || { $P $TX --skip-held 1 $N/e1/carve_mv5_aonly.ply $N/r2/held_v4.glb > $N/r2/held_v4.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $N/r2/held_v4.glb >> $N/r2/held_v4.log 2>&1; }
$P scan3d/gsmap/gsm_eval2.py $N/r2/held_v4.glb $N/r2/eval_v4.json --heldout-glb $N/r2/held_v4.glb --tex-json $N/r2/held_v4.json > $N/r2/eval_v4.log 2>&1
echo V4BASEDONE $(date +%T)
