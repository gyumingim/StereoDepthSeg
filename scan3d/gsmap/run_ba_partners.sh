#!/bin/bash
# gsmap 2026-11-26: BA with FFS partner frames + FFS re-run with the new poses (gsm_ba_partners.py). usage: run_ba_partners.sh [step ...]
# Power safety: before every GPU / heavy CPU step wait (poll 60 s) until no other GPU compute process and no other heavy python
# (gsm_*.py, train, ffs scripts) runs; one step at a time; CPU under nice 19 on 8 cores (taskset 0-7, 8 BLAS threads).
# A step that fails twice stops the chain.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
O=scan3d/runs/gsmap_20261125_121801_v2/ba_partners; mkdir -p $O
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 PYTHONUNBUFFERED=1
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
FF="taskset -c 0-7 nice -n 19 $HOME/scanner/StereoDepthSeg/venv_ffs/bin/python -W ignore"
S=scan3d/gsmap/gsm_ba_partners.py

wait_free() {
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|ffs[a-z0-9_]*\.py|lib_ffs|kf_ba' | grep -v gsm_ba_partners | grep -c .)
    [ "$g" = 0 ] && [ "$p" = 0 ] && return
    echo "$(date '+%F %T') wait: gpu procs $g, heavy python $p" >> $O/wait.log; sleep 60
  done
}

run_step() {
  local st=$1; shift
  for try in 1 2; do
    wait_free
    echo "$(date '+%F %T') START $st try $try" >> $O/run.log
    "$@" >> $O/$st.log 2>&1; rc=$?
    echo "$(date '+%F %T') END $st rc $rc" >> $O/run.log
    [ $rc = 0 ] && return 0
  done
  echo "$(date '+%F %T') STOP: $st failed twice" >> $O/run.log; exit 1
}

steps=${@:-frames ba post ffs2 ffs3 fuse measure}
for st in $steps; do
  case $st in
    ba) run_step ba $FF $S ba ;;
    ffs2|ffs3) run_step $st env FFS_ROOT=$HOME/scanner/StereoDepthSeg $FF $S $st ;;
    *) run_step $st $GS $S $st ;;
  esac
done
echo "$(date '+%F %T') ALL DONE" >> $O/run.log
