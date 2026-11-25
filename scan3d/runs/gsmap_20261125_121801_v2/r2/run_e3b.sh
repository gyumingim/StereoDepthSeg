#!/bin/bash
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; export GSMAP_RUN=$N
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
$P scan3d/gsmap/gsm_e3planes.py $N/e1/carve_mv5_aonly.ply $N/r2/e3b.ply --ceil-y -2.117 > $N/r2/e3b.log 2>&1 || { echo FAIL; exit 1; }
$P scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --skip-held 1 $N/r2/e3b.ply $N/r2/held_e3b.glb > $N/r2/held_e3b.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $N/r2/held_e3b.glb >> $N/r2/held_e3b.log 2>&1 || { echo FAIL; exit 1; }
$P scan3d/gsmap/gsm_eval2.py $N/r2/held_e3b.glb $N/r2/eval_e3b.json --heldout-glb $N/r2/held_e3b.glb --tex-json $N/r2/held_e3b.json > $N/r2/eval_e3b.log 2>&1
echo E3BDONE
