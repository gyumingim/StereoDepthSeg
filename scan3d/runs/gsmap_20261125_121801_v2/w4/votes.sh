#!/bin/bash
V2=~/scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2; G=~/scanner/StereoDepthSeg-gsmap/scan3d/gsmap; PY=~/scanner/gsmap_env/venv/bin/python
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 NUMEXPR_NUM_THREADS=8
L=$V2/w4/run.log
echo "$(date '+%F %T') START votes_w3A" >> $L
taskset -c 0-7 nice -n 19 $PY $G/gsm_ffs_support.py $V2/w3/fill/w3.ply $V2/w4/votes_w3A --fused $V2/ffsonly_ba2/fused --cams $V2/ba_partners2/run/dump_ba/cams.npz --n 2 > $V2/w4/logs/votes_w3A.log 2>&1
echo "$(date '+%F %T') END votes_w3A rc $?" >> $L
echo "$(date '+%F %T') START votes_nC" >> $L
taskset -c 0-7 nice -n 19 $PY $G/gsm_ffs_support.py $V2/ffsonly_ba/support_test/nC/e3c.ply $V2/w4/votes_nC --fused $V2/ffsonly_ba/fused --cams $V2/ba_partners/run/dump_ba/cams.npz --n 2 > $V2/w4/logs/votes_nC.log 2>&1
echo "$(date '+%F %T') END votes_nC rc $?" >> $L
