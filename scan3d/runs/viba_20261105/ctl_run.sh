#!/bin/bash
# viba README 2.10: after the 3rd PC hang. Only the remaining x1.03 positive control, ONE heavy job machine-wide
# (flock ~/scanner/heavy.lock around each job, shared with featloop), BLAS/OMP threads 8, RAM floor 8 GB + peak.
cd "$(dirname "$0")/../../.."
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
PV=$HOME/scanner/viba_work/venv/bin/python; D=scan3d/runs/viba_20261105; BL=$HOME/scanner/build_livepose; MR=$HOME/scanner/viba_work/memrun.sh
memwait() { while [ $(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo) -lt $1 ]; do sleep 30; done; }
for r in "$@"; do
  o=$D/viba/$r/viba_x103; [ -f $o/viba.json ] && continue
  if [[ $r == *165506 ]]; then R=scan3d/recordings/hs_${r}_std60; C=$BL/cfg_gen/c4f60; else R=scan3d/recordings/hs_$r; C=$BL/cfg_gen_hs222/c4f60; fi
  memwait 14; mkdir -p $o
  flock $HOME/scanner/heavy.lock $MR 11000 $D/viba/$r/viba_x103.log $PV scan3d/viba/viba.py $R --dump $D/ba/$r/viba_input.npz \
      --vio $D/vio/$r.poses.jsonl.gz --cfg $C --out $o --calib yaml --init-scale 1.03
  echo "$r viba_x103 $(tail -1 $D/viba/$r/viba_x103.log.mem) $(date +%T) $(grep -E '^\[lm\]' $D/viba/$r/viba_x103.log)"
done
echo CTLDONE $(date +%T)
