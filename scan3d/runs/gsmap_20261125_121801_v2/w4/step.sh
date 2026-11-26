#!/bin/bash
# usage: step.sh <name> <args...>   -> runs gsm_f_ruler.py <args>, writes logs/<name>.log and marker logs/<name>.rc
V2=~/scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2; G=~/scanner/StereoDepthSeg-gsmap/scan3d/gsmap; PY=~/scanner/gsmap_env/venv/bin/python
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 NUMEXPR_NUM_THREADS=8
n=$1; shift; rm -f $V2/w4/logs/$n.rc
cd $G; taskset -c 0-7 nice -n 19 $PY -W ignore $G/gsm_f_ruler.py "$@" > $V2/w4/logs/$n.log 2>&1; rc=$?
echo $rc > $V2/w4/logs/$n.rc; echo "$(date '+%F %T') step $n rc $rc" >> $V2/w4/run.log
