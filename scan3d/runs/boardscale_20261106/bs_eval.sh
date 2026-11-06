#!/bin/bash
# boardscale 2026-11-06 (README 2): board scale for existing BA outputs, then depth + TSDF runs (board scale, oracle).
# Every heavy job: flock ~/scanner/heavy.lock (one heavy job machine-wide), threads 8, GPU frac 0.6, no torch.compile,
# RAM >= 12 GB available before start, own watchdog kills ONLY its own pid if MemAvailable < 8 GB.
#   bash scan3d/runs/boardscale_20261106/bs_eval.sh [scale|depth|all]
cd "$(dirname "$0")/../../.."
W=$(pwd)
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
RD=$HOME/scanner/StereoDepthSeg/scan3d/recordings
CB=$HOME/scanner/StereoDepthSeg/scan3d/config/charuco
SA=$HOME/scanner/StereoDepthSeg-scanA/scan3d/runs/scanA_20261105
FL=$HOME/scanner/StereoDepthSeg-featloop/scan3d/runs/featloop_20261105/rec
O=scan3d/runs/boardscale_20261106
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 \
       TORCHINDUCTOR_COMPILE_THREADS=1 FFS_ROOT=$HOME/scanner/StereoDepthSeg
mem() { awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo; }
job() {  # job LOG cmd...   (skips if LOG has DONE)
  log=$1; shift
  grep -q "^JOBDONE rc 0" $log 2>/dev/null && { echo "skip $log"; return 0; }
  for try in 1 2 3; do
    echo "== $(date '+%F %T') wait lock: $log (try $try)"
    flock $HOME/scanner/heavy.lock bash -c '
      while [ "$(awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo)" -lt 12 ]; do sleep 20; done
      echo "start $(date "+%F %T") mem $(awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo) GB"
      "$@" & pid=$!; killed=0
      while kill -0 $pid 2>/dev/null; do
        m=$(awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo)
        if [ $m -lt 8 ]; then echo "WATCHDOG $(date +%T) MemAvailable $m GB -> kill own pid $pid"; kill $pid; killed=1; fi
        sleep 5; done
      wait $pid; rc=$?; [ $killed = 1 ] && rc=99; echo "JOBDONE rc $rc $(date "+%F %T")"; exit $rc' _ "$@" > $log 2>&1
    rc=$?; echo "   rc $rc $(date +%T)"
    [ $rc != 99 ] && return $rc
  done; return 1; }
board_of() { case $1 in 165506) echo $CB/board_laptop_5x3.json;; 20261020_*) echo $CB/board_laptop_6x6.json;;
             20261021_*) echo $CB/board_laptop_fit_6x6.json;; 20261022_*) echo $CB/board_monitor_6x6.json;; esac; }
recdir() { [ $1 = 165506 ] && echo $RD/hs_20261024_165506_std60 || echo $RD/hs_$1; }
gtl() { [ $1 = 165506 ] && echo imu_gt_spl || echo imu_gt_spl_cal1022; }
gtrs() { [ $1 = 165506 ] && echo imu_gt_rs || echo imu_gt_rs_cal1022; }
bapath() { case $2 in scanA) [ $1 = 165506 ] && echo $SA/step2/ba_o0 || echo $SA/more/ba/$1;; S) echo $FL/$1/ba_S;; esac; }
MAIN="165506 20261022_134204 20261021_213341 20261020_222747 20261020_214138"
EXTRA="165506 20261022_134204 20261021_213341 20261020_224323 20261021_214550 20261022_133820 20261022_134550"
mkdir -p $O/scale $O/depth
stage=${1:-all}
if [ $stage = scale ] || [ $stage = all ]; then
  for src in scanA S; do
    L=$MAIN; [ $src = S ] && L=$EXTRA
    for r in $L; do
      B=$(bapath $r $src); [ -f $B/poses_ba.jsonl.gz ] || { echo "no BA $B"; continue; }
      R=$(recdir $r); od=$O/scale/${r}_$src
      job $od.log $P scan3d/scanA/board_scale.py $R --poses $B/poses_ba.jsonl.gz --board $(board_of $r) \
        --lens $R/$(gtl $r)/gt.npz --out $od --check-gt-corners $R/$(gtrs $r)/gt.npz
      tail -n 4 $od.log | head -n 3
    done
  done
fi
if [ $stage = depth ] || [ $stage = all ]; then
  for r in $MAIN; do
    R=$(recdir $r); G=$(gtl $r); B=$(bapath $r scanA); od=$O/scale/${r}_scanA
    [ -f $od/poses_board.jsonl.gz ] || { echo "no board poses $r"; continue; }
    if [ $r = 165506 ]; then X=""; else X="--compress --kf-step 1"; fi
    job $O/depth/${r}_board.log $P scan3d/scanA/scan_offline.py $R --out $O/depth/${r}_board --gt $G --vio $od/poses_board.jsonl.gz $X
    if [ $r = 165506 ]; then   # determinism check of the existing baseline (step3/ba) under the new caps
      job $O/depth/${r}_ba_rerun.log $P scan3d/scanA/scan_offline.py $R --out $O/depth/${r}_ba_rerun --gt $G --vio $B/poses_ba.jsonl.gz
    else
      job $O/depth/${r}_oracle.log $P scan3d/scanA/scan_offline.py $R --out $O/depth/${r}_oracle --gt $G --vio $B/poses_ba.jsonl.gz --free-scale $X
    fi
  done
fi
echo ALLDONE $(date '+%F %T')
