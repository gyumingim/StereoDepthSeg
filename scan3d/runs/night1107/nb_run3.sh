#!/bin/bash
# night1107 README 5.1 (exploratory): BA + board scale for audit-init VIO 173432 o1/o2 and 165506 o0, then depth ba + board.
cd "$(dirname "$0")/../../.."; W=$(pwd)
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
RD=$HOME/scanner/StereoDepthSeg/scan3d/recordings; CB=$HOME/scanner/StereoDepthSeg/scan3d/config/charuco; O=$W/scan3d/runs/night1107
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 \
       TORCHINDUCTOR_COMPILE_THREADS=1 FFS_ROOT=$HOME/scanner/StereoDepthSeg
mem() { awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo; }
j() { log=$1; shift; grep -q "^JOBDONE rc 0" $log 2>/dev/null && { echo "skip $log"; return 0; }
  while [ "$(mem)" -lt 12 ]; do sleep 20; done
  echo "== $(date +%T) $log"; echo "start $(date "+%F %T") mem $(mem) GB" > $log
  ( exec 9>&-; "$@" ) >> $log 2>&1 & pid=$!; k=0
  while kill -0 $pid 2>/dev/null; do [ "$(mem)" -lt 4 ] && { echo "WATCHDOG $(date +%T)" | tee -a $log; kill $pid; pkill -P $pid; k=1; }; sleep 5; done
  wait $pid; rc=$?; [ $k = 1 ] && rc=99; echo "JOBDONE rc $rc $(date "+%F %T")" >> $log; echo "   rc $rc"; return $rc; }
# name rec vio
L="173432_A_o1:hs_20261024_173432_std60 173432_A_o2:hs_20261024_173432_std60 165506_A_o0:hs_20261024_165506_std60"
echo "== $(date "+%F %T") wait heavy.lock"; exec 9>$HOME/scanner/heavy.lock; flock 9; echo "== $(date "+%F %T") got heavy.lock"
for e in $L; do n=${e%%:*}; R=$RD/${e#*:}; X=$O/rec/$n; mkdir -p $X
  j $X/ba_S.log $P scan3d/scanA/kf_ba.py $R --vio $O/vio/$n.poses.jsonl.gz --out $X/ba_S --gt imu_gt_spl --feat sift --loop none \
     --cache-root $HOME/scanner/night1107_ext/cache --seed 0 --keep-db; grep -E "^\[sim3\]" $X/ba_S.log
  [ -f $X/ba_S/poses_ba.jsonl.gz ] && j $O/scale/${n}_S.log $P scan3d/scanA/board_scale.py $R --poses $X/ba_S/poses_ba.jsonl.gz \
     --board $CB/board_laptop_5x3.json --lens $R/imu_gt_spl/gt.npz --out $O/scale/${n}_S --check-gt-corners $R/imu_gt_rs/gt.npz
  tail -n 2 $O/scale/${n}_S.log | head -1
done
flock -u 9; exec 9>&-; echo "== $(date "+%F %T") released heavy.lock"
for e in $L; do n=${e%%:*}; R=$RD/${e#*:}; X=$O/rec/$n
  [ -f $X/ba_S/poses_ba.jsonl.gz ] && j $O/depth/${n}_ba.log $P scan3d/scanA/scan_offline.py $R --out $O/depth/${n}_ba --gt imu_gt_spl --vio $X/ba_S/poses_ba.jsonl.gz
  [ -f $O/scale/${n}_S/poses_board.jsonl.gz ] && j $O/depth/${n}_board.log $P scan3d/scanA/scan_offline.py $R --out $O/depth/${n}_board --gt imu_gt_spl --vio $O/scale/${n}_S/poses_board.jsonl.gz
done
echo "ALLDONE3 $(date "+%F %T")"; touch $HOME/scanner/night1107.run3.done
