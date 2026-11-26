#!/bin/bash
# gsmap 2026-11-26 overnight task R (fair judge): run_rjudge.sh step [step ...]; steps = claims xrep geo photo mv. One step at a time,
# nice 19, 8 threads; waits until no other heavy python / GPU compute job runs. Output only in runs/gsmap_20261125_121801_v2/r_judge.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
O=$PWD/scan3d/runs/gsmap_20261125_121801_v2/r_judge; mkdir -p $O/logs
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 NUMEXPR_NUM_THREADS=8 PYTHONUNBUFFERED=1
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
wait_free() {
  local n=0
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|ffs[a-z0-9_]*\.py|kf_ba' | grep -c .)
    [ "$g" = 0 ] && [ "$p" = 0 ] && return
    sleep 30; n=$((n+1)); [ $n -gt 60 ] && { echo "$(date '+%F %T') FAIL r wait_free > 30 min" >> $O/run.log; exit 1; }
  done
}
for st in "$@"; do
  wait_free
  echo "$(date '+%F %T') START $st" >> $O/run.log
  [ $st = mv ] && source $HOME/scanner/gsplat_env/env.sh
  $GS scan3d/gsmap/gsm_rjudge.py $st > $O/logs/$st.log 2>&1; rc=$?
  echo "$(date '+%F %T') END $st rc $rc" >> $O/run.log
  [ $rc = 0 ] || { echo "$(date '+%F %T') FAIL r $st rc $rc" >> $O/run.log; exit 1; }
done
echo "$(date '+%F %T') STEPS_OK $*" >> $O/run.log
