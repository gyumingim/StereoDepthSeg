#!/bin/bash
# eval2 with the NEW BA keyframe poses as judge cams (refs unchanged: v2 eval2/ref), for e8 final and ffsonly -> fairness check of the ruler
cd $HOME/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/ffsonly_ba; E=$O/evalrun_newcams
mkdir -p $E/dump_ba $E/eval2/ref
ln -sfn $N/ba_partners/run/dump_ba/cams.npz $E/dump_ba/cams.npz; ln -sfn $N/dump_ba/img $E/dump_ba/img
for f in $N/eval2/ref/*; do ln -sfn $f $E/eval2/ref/$(basename $f); done
P="taskset -c 0-7 nice -n 19 env OMP_NUM_THREADS=8 GSMAP_RUN=$E $HOME/scanner/gsmap_env/venv/bin/python -W ignore scan3d/gsmap/gsm_eval2.py"
$P $N/e8/c1855/tex_121801_final.glb $O/eval_newcams_e8final.json --tex-json $N/e8/c1855/tex_121801_final.json > $O/logs/eval_newcams_e8.log 2>&1; echo e8 $?
$P $O/b/tex_121801_ffsonly.glb $O/eval_newcams_ffsonly.json --tex-json $O/b/tex_121801_ffsonly.json > $O/logs/eval_newcams_ffsonly.log 2>&1; echo ffs $?
