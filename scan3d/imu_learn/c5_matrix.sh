#!/bin/bash
# C5 test matrix (docs/c5_test_plan.md). 4 runs at a time (fixed), then the determinism repeats one at a time.
# Start detached: (setsid nohup bash imu_learn/c5_matrix.sh > runs/c5/matrix.out 2>&1 &)
cd "$(dirname "$0")/.." || exit 1
PY=../venv_ffs/bin/python
OUT=runs/c5
LOG=runs/c5_progress.log
mkdir -p $OUT
RECS="114000 115620 125714 131326 132803 145816 151358 152631 154320 161035 163345"
log() { echo "$(date '+%F %T') c5: $*" >> $LOG; }
job() {   # name init clones noise scale rec off
  local name=$1 init=$2 cl=$3 noise=$4 sc=$5 rec=$6 off=$7
  [ -f $OUT/$name.json ] && return 0
  local r
  r=$($PY imu_learn/c5_run.py recordings/hs_20261019_$rec --init-cfg $init --clones $cl --noise $noise --noise-scale $sc \
      --start-off $off --max-secs 60 --out $OUT/$name 2> $OUT/$name.err | tail -1)
  log "done $name load $(cut -d' ' -f1-3 /proc/loadavg) $r"
}
export -f job log
export PY OUT LOG
{
  # primary noise (cfg): loose baseline, strict, strict + 30 clones
  for rec in $RECS; do for off in 0 1 2; do
    echo "${rec}_cfg_loose_o$off loose 11 cfg 1 $rec $off"
    echo "${rec}_cfg_strict_o$off strict 11 cfg 1 $rec $off"
    echo "${rec}_cfg_strict30_o$off strict 30 cfg 1 $rec $off"
  done; done
  # secondary noise (meas x10, tuned on eval recordings): loose, strict
  for rec in $RECS; do for off in 0 1 2; do
    echo "${rec}_m10_loose_o$off loose 11 meas 10 $rec $off"
    echo "${rec}_m10_strict_o$off strict 11 meas 10 $rec $off"
  done; done
  # secondary init: strict-dyn (make_ov_config --init dynamic values)
  for rec in $RECS; do for off in 0 1 2; do
    echo "${rec}_cfg_strictdyn_o$off strict-dyn 11 cfg 1 $rec $off"
  done; done
} > $OUT/jobs.txt
log "matrix start $(wc -l < $OUT/jobs.txt) jobs, 4 parallel, git $(git rev-parse --short HEAD) bench $(git hash-object imu_learn/vio_bench_hs.py)"
xargs -P 4 -L 1 bash -c 'job "$@"' _ < $OUT/jobs.txt
log "matrix parallel part finished"
# determinism: same runs again, one at a time (low load)
for n in "145816_cfg_loose_o0 loose 11 cfg 1 145816 0" "145816_cfg_strict_o0 strict 11 cfg 1 145816 0" \
         "161035_cfg_loose_o1 loose 11 cfg 1 161035 1" "151358_cfg_loose_o0 loose 11 cfg 1 151358 0"; do
  set -- $n
  job "rep_$1" $2 $3 $4 $5 $6 $7
done
log "matrix all finished"
