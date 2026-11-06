#!/bin/bash
# night1107 (README 1, 4): 173432 full stack + 165506 ba_S board/oracle depth. CPU-heavy = heavy.lock, GPU = gpu.lock (gpu_guard) only.
# Each job skipped if its log has "JOBDONE rc 0". Own watchdog kills only its own job when MemAvailable < 4 GB. Ends with ~/scanner/night1107.run.done
cd "$(dirname "$0")/../../.."; W=$(pwd)
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
RD=$HOME/scanner/StereoDepthSeg/scan3d/recordings; CB=$HOME/scanner/StereoDepthSeg/scan3d/config/charuco
B=$HOME/scanner/build_livepose; O=$W/scan3d/runs/night1107; FL=$HOME/scanner/StereoDepthSeg-featloop/scan3d/runs/featloop_20261105/rec
BS=$HOME/scanner/StereoDepthSeg-boardscale/scan3d/runs/boardscale_20261106/scale
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 \
       TORCHINDUCTOR_COMPILE_THREADS=1 FFS_ROOT=$HOME/scanner/StereoDepthSeg
mem() { awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo; }
run1() {  # run1 LOG LOCK(heavy|gpu) cmd...
  log=$1; lk=$2; shift 2
  grep -q "^JOBDONE rc 0" $log 2>/dev/null && { echo "skip $log"; return 0; }
  for try in 1 2 3; do
    echo "== $(date "+%F %T") wait ($lk) $log try $try"
    if [ $lk = heavy ]; then exec 9>$HOME/scanner/heavy.lock; flock 9; fi
    while [ "$(mem)" -lt 12 ]; do sleep 20; done
    echo "start $(date "+%F %T") mem $(mem) GB" > $log
    ( exec 9>&-; "$@" ) >> $log 2>&1 & pid=$!; killed=0
    while kill -0 $pid 2>/dev/null; do
      if [ "$(mem)" -lt 4 ]; then echo "WATCHDOG $(date +%T) mem $(mem) GB -> kill own $pid" | tee -a $log; kill $pid; pkill -P $pid; killed=1; fi
      sleep 5; done
    wait $pid; rc=$?; [ $killed = 1 ] && rc=99
    echo "JOBDONE rc $rc $(date "+%F %T")" >> $log
    [ $lk = heavy ] && { flock -u 9; exec 9>&-; }
    echo "   rc $rc $(date +%T)"
    [ $rc != 99 ] && return $rc
    sleep 60
  done; return 1; }
R=$RD/hs_20261024_173432_std60; R6=$RD/hs_20261024_165506_std60; X=$O/rec/173432
mkdir -p $O/vio $X $O/scale $O/depth $HOME/scanner/night1107_ext/cache
ST=${1:-all}   # all | g6 (only the 165506 GPU runs) | main (173432 only)
if [ $ST != g6 ]; then
# 1 VIO replay (B o0, default args)
( cd $W/scan3d/runs/livepose_20261030 && run1 $O/vio/173432_B_o0.log heavy $P lp_replay.py $R $B/cfg_gen/c4f60 $O/vio/173432_B_o0 \
    --bridge $B/bridge/ov_bridge --vocab $W/scan3d/ov_bridge/loop/voc_k10L4.voc )
tail -n 2 $O/vio/173432_B_o0.log
# 2 BA seed 0 SIFT
[ -f $O/vio/173432_B_o0.poses.jsonl.gz ] && run1 $X/ba_S.log heavy $P scan3d/scanA/kf_ba.py $R --vio $O/vio/173432_B_o0.poses.jsonl.gz \
    --out $X/ba_S --gt imu_gt_spl --feat sift --loop none --cache-root $HOME/scanner/night1107_ext/cache --seed 0 --keep-db
grep -E "^\[(sim3|pairs)\]" $X/ba_S.log | cut -c1-200
# 3 board scale
[ -f $X/ba_S/poses_ba.jsonl.gz ] && run1 $O/scale/173432_S.log heavy $P scan3d/scanA/board_scale.py $R --poses $X/ba_S/poses_ba.jsonl.gz \
    --board $CB/board_laptop_5x3.json --lens $R/imu_gt_spl/gt.npz --out $O/scale/173432_S --check-gt-corners $R/imu_gt_rs/gt.npz
tail -n 3 $O/scale/173432_S.log
# 4 depth (GPU lock only)
if [ -f $X/ba_S/poses_ba.jsonl.gz ]; then
  run1 $O/depth/173432_ba.log gpu $P scan3d/scanA/scan_offline.py $R --out $O/depth/173432_ba --gt imu_gt_spl --vio $X/ba_S/poses_ba.jsonl.gz
  [ -f $O/scale/173432_S/poses_board.jsonl.gz ] && \
  run1 $O/depth/173432_board.log gpu $P scan3d/scanA/scan_offline.py $R --out $O/depth/173432_board --gt imu_gt_spl --vio $O/scale/173432_S/poses_board.jsonl.gz
  run1 $O/depth/173432_oracle.log gpu $P scan3d/scanA/scan_offline.py $R --out $O/depth/173432_oracle --gt imu_gt_spl --vio $X/ba_S/poses_ba.jsonl.gz --free-scale
fi
fi
if [ $ST != main ]; then
run1 $O/depth/165506_S_board.log gpu $P scan3d/scanA/scan_offline.py $R6 --out $O/depth/165506_S_board --gt imu_gt_spl --vio $BS/165506_S/poses_board.jsonl.gz
run1 $O/depth/165506_S_oracle.log gpu $P scan3d/scanA/scan_offline.py $R6 --out $O/depth/165506_S_oracle --gt imu_gt_spl --vio $FL/165506/ba_S/poses_ba.jsonl.gz --free-scale
fi
echo "ALLDONE $ST $(date "+%F %T")"; touch $HOME/scanner/night1107.run_$ST.done
