#!/bin/bash
# C5 test matrix (docs/c5_test_plan.md + amendment 15:40). 3 runs at a time (fixed), video decode 2 threads (c5_run default),
# then the determinism repeats one at a time.
# Start detached: (setsid nohup bash imu_learn/c5_matrix.sh > runs/c5_matrix.out 2>&1 &)
cd "$(dirname "$0")/.." || exit 1
PY=../venv_ffs/bin/python
OUT=runs/c5
LOG=runs/c5_progress.log
PAR=3
mkdir -p $OUT
RECS="114000 115620 125714 131326 132803 145816 151358 152631 154320 161035 163345"
log() { echo "$(date '+%F %T') c5: $*" >> $LOG; }
job() {   # name init clones noise scale rec off
  local name=$1 init=$2 cl=$3 noise=$4 sc=$5 rec=$6 off=$7
  [ -f $OUT/$name.json ] && return 0
  local l0 r
  l0=$(cut -d' ' -f1-3 /proc/loadavg)
  r=$($PY imu_learn/c5_run.py recordings/hs_20261019_$rec --init-cfg $init --clones $cl --noise $noise --noise-scale $sc \
      --start-off $off --max-secs 60 --out $OUT/$name 2> $OUT/$name.err | tail -1)
  [ -f $OUT/$name.json ] || r="FAILED (no json): $(tail -1 $OUT/$name.err)"
  log "done $name load_start $l0 load_end $(cut -d' ' -f1-3 /proc/loadavg) $r"
}
export -f job log
export PY OUT LOG
{
  for rec in $RECS; do for off in 0 1; do          # main (amendment 15:42): loose (c) vs strict-dyn (d), (d)+30 clones (e)
    echo "${rec}_cfg_loose_o$off loose 11 cfg 1 $rec $off"
    echo "${rec}_cfg_strictdyn_o$off strict-dyn 11 cfg 1 $rec $off"
  done; done
  echo "145816_cfg_strict_o0 strict 11 cfg 1 145816 0"      # (a) confirmation only
  echo "154320_cfg_strict_o0 strict 11 cfg 1 154320 0"
  for rec in $RECS; do for off in 0 1; do
    echo "${rec}_cfg_strictdyn30_o$off strict-dyn 30 cfg 1 $rec $off"
  done; done
} > $OUT/jobs.txt
log "matrix start $(wc -l < $OUT/jobs.txt) jobs, $PAR parallel, decode threads 2, git $(git rev-parse --short HEAD) bench $(git hash-object imu_learn/vio_bench_hs.py) runner $(git hash-object imu_learn/c5_run.py)"
xargs -P $PAR -L 1 bash -c 'job "$@"' _ < $OUT/jobs.txt
log "matrix parallel part finished"
for n in "145816_cfg_loose_o0 loose 11 cfg 1 145816 0" "151358_cfg_strictdyn_o0 strict-dyn 11 cfg 1 151358 0"; do
  set -- $n
  job "rep_$1" $2 $3 $4 $5 $6 $7                    # determinism: same run again, alone (low load)
done
log "matrix all finished"
