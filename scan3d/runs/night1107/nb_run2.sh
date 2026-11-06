#!/bin/bash
# night1107 README 5 (post-hoc): CPU chain under ONE heavy.lock hold (replays + BA + board scale), then GPU depth (gpu.lock only).
cd "$(dirname "$0")/../../.."; W=$(pwd)
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
RD=$HOME/scanner/StereoDepthSeg/scan3d/recordings; CB=$HOME/scanner/StereoDepthSeg/scan3d/config/charuco
B=$HOME/scanner/build_livepose; O=$W/scan3d/runs/night1107
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 \
       TORCHINDUCTOR_COMPILE_THREADS=1 FFS_ROOT=$HOME/scanner/StereoDepthSeg
mem() { awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo; }
j() {  # j LOG cmd...  (skip if done; watchdog own job < 4 GB)
  log=$1; shift; grep -q "^JOBDONE rc 0" $log 2>/dev/null && { echo "skip $log"; return 0; }
  while [ "$(mem)" -lt 12 ]; do sleep 20; done
  echo "== $(date +%T) $log"; echo "start $(date "+%F %T") mem $(mem) GB" > $log
  ( exec 9>&-; "$@" ) >> $log 2>&1 & pid=$!; k=0
  while kill -0 $pid 2>/dev/null; do [ "$(mem)" -lt 4 ] && { echo "WATCHDOG $(date +%T)" | tee -a $log; kill $pid; pkill -P $pid; k=1; }; sleep 5; done
  wait $pid; rc=$?; [ $k = 1 ] && rc=99; echo "JOBDONE rc $rc $(date "+%F %T")" >> $log; echo "   rc $rc"; return $rc; }
R=$RD/hs_20261024_173432_std60; R6=$RD/hs_20261024_165506_std60; X=$O/rec/173432
AUD="--yaml init_window_time=1.5,init_max_disparity=2.30"
LP="--bridge $B/bridge/ov_bridge --vocab $W/scan3d/ov_bridge/loop/voc_k10L4.voc"
ntrk() { python3 -c "import gzip,json,sys;print(sum(json.loads(l).get(\"status\")==\"tracking\" for l in gzip.open(sys.argv[1],\"rt\")))" $1 2>/dev/null || echo 0; }
echo "== $(date "+%F %T") wait heavy.lock"
exec 9>$HOME/scanner/heavy.lock; flock 9; echo "== $(date "+%F %T") got heavy.lock"
cd $W/scan3d/runs/livepose_20261030
for o in 1 2; do j $O/vio/173432_B_o$o.log $P lp_replay.py $R $B/cfg_gen/c4f60 $O/vio/173432_B_o$o $LP --start-off $o; tail -n 2 $O/vio/173432_B_o$o.log | head -1 | cut -c1-200; done
for o in 0 1 2; do j $O/vio/173432_A_o$o.log $P lp_replay.py $R $B/cfg_gen/c4f60 $O/vio/173432_A_o$o $LP --start-off $o $AUD; tail -n 2 $O/vio/173432_A_o$o.log | head -1 | cut -c1-200; done
j $O/vio/165506_A_o0.log $P lp_replay.py $R6 $B/cfg_gen/c4f60 $O/vio/165506_A_o0 $LP $AUD; tail -n 2 $O/vio/165506_A_o0.log | head -1 | cut -c1-200
cd $W
V=$O/vio/173432_A_o0.poses.jsonl.gz; echo "A_o0 tracking $(ntrk $V)"
if [ "$(ntrk $V)" -gt 200 ]; then
  j $X/ba_SA.log $P scan3d/scanA/kf_ba.py $R --vio $V --out $X/ba_SA --gt imu_gt_spl --feat sift --loop none \
     --cache-root $HOME/scanner/night1107_ext/cache --seed 0 --keep-db; grep -E "^\[(sim3|pairs)\]" $X/ba_SA.log | cut -c1-200
  [ -f $X/ba_SA/poses_ba.jsonl.gz ] && j $O/scale/173432_SA.log $P scan3d/scanA/board_scale.py $R --poses $X/ba_SA/poses_ba.jsonl.gz \
     --board $CB/board_laptop_5x3.json --lens $R/imu_gt_spl/gt.npz --out $O/scale/173432_SA --check-gt-corners $R/imu_gt_rs/gt.npz
  tail -n 3 $O/scale/173432_SA.log
fi
flock -u 9; exec 9>&-; echo "== $(date "+%F %T") released heavy.lock"
if [ -f $X/ba_SA/poses_ba.jsonl.gz ]; then
  j $O/depth/173432A_ba.log $P scan3d/scanA/scan_offline.py $R --out $O/depth/173432A_ba --gt imu_gt_spl --vio $X/ba_SA/poses_ba.jsonl.gz
  [ -f $O/scale/173432_SA/poses_board.jsonl.gz ] && j $O/depth/173432A_board.log $P scan3d/scanA/scan_offline.py $R --out $O/depth/173432A_board \
     --gt imu_gt_spl --vio $O/scale/173432_SA/poses_board.jsonl.gz
  j $O/depth/173432A_oracle.log $P scan3d/scanA/scan_offline.py $R --out $O/depth/173432A_oracle --gt imu_gt_spl --vio $X/ba_SA/poses_ba.jsonl.gz --free-scale
fi
echo "ALLDONE2 $(date "+%F %T")"; touch $HOME/scanner/night1107.run2.done
