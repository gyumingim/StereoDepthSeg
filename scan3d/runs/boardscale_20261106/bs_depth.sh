#!/bin/bash
# boardscale depth stage (coordinator 11-06: one CPU + one GPU job machine-wide). GPU jobs take only ~/scanner/gpu.lock
# (gpu_guard inside scan_offline), NOT heavy.lock. Each recording starts once its board_scale.json exists. Frac 0.6, no compile,
# RAM >= 12 GB before start, own watchdog kills only its own job below 8 GB. Touches ~/scanner/boardscale.done at the end.
cd "$(dirname "$0")/../../.."
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
RD=$HOME/scanner/StereoDepthSeg/scan3d/recordings
SA=$HOME/scanner/StereoDepthSeg-scanA/scan3d/runs/scanA_20261105
O=scan3d/runs/boardscale_20261106
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 \
       TORCHINDUCTOR_COMPILE_THREADS=1 FFS_ROOT=$HOME/scanner/StereoDepthSeg
mem() { awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo; }
gjob() {  # gjob LOG cmd...
  log=$1; shift
  grep -q "^JOBDONE rc 0" $log 2>/dev/null && { echo "skip $log"; return 0; }
  for try in 1 2 3; do
    while [ "$(mem)" -lt 12 ]; do sleep 20; done
    echo "== $(date '+%F %T') start $log (try $try, mem $(mem) GB)"
    "$@" > $log 2>&1 & pid=$!; killed=0
    while kill -0 $pid 2>/dev/null; do
      if [ "$(mem)" -lt 8 ]; then echo "WATCHDOG $(date +%T) mem $(mem) GB -> kill own pid $pid"; kill $pid; killed=1; fi
      sleep 5; done
    wait $pid; rc=$?; echo "JOBDONE rc $rc $(date '+%F %T')" >> $log; echo "   rc $rc killed=$killed"
    [ $killed = 0 ] && return $rc
  done; return 1; }
recdir() { [ $1 = 165506 ] && echo $RD/hs_20261024_165506_std60 || echo $RD/hs_$1; }
gtl() { [ $1 = 165506 ] && echo imu_gt_spl || echo imu_gt_spl_cal1022; }
mkdir -p $O/depth
for r in 165506 20261022_134204 20261021_213341 20261020_222747 20261020_214138; do
  R=$(recdir $r); G=$(gtl $r); od=$O/scale/${r}_scanA
  if [ $r = 165506 ]; then B=$SA/step2/ba_o0; X=""; else B=$SA/more/ba/$r; X="--compress --kf-step 1"; fi
  until [ -f $od/board_scale.json ] || grep -q "ALLDONE" $O/bs_scale.out; do sleep 30; done
  [ -f $od/poses_board.jsonl.gz ] || { echo "no board poses $r"; continue; }
  gjob $O/depth/${r}_board.log $P scan3d/scanA/scan_offline.py $R --out $O/depth/${r}_board --gt $G --vio $od/poses_board.jsonl.gz $X
  if [ $r = 165506 ]; then
    gjob $O/depth/${r}_ba_rerun.log $P scan3d/scanA/scan_offline.py $R --out $O/depth/${r}_ba_rerun --gt $G --vio $B/poses_ba.jsonl.gz
  else
    gjob $O/depth/${r}_oracle.log $P scan3d/scanA/scan_offline.py $R --out $O/depth/${r}_oracle --gt $G --vio $B/poses_ba.jsonl.gz --free-scale $X
  fi
done
echo "DEPTH ALLDONE $(date '+%F %T')"
until grep -q "ALLDONE" $O/bs_scale.out; do sleep 30; done
touch $HOME/scanner/boardscale.done; echo "boardscale.done $(date '+%F %T')"
