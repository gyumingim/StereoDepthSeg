#!/bin/bash
# kfonly 11-11 README 11: FFS+TSDF only, on the reduced keyframe folders, with A's BA poses. usage: job_apose.sh OUT_DIR
set -e
O=$1; L=scan3d/runs/laptop/20261022_133820
P=${P:-/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python}
MS="--gt imu_gt_spl_cal1022 --compress --kf-step 1"; AP=$L/kfonly/A/ba_S/poses_ba.jsonl.gz
T() { /usr/bin/time -f "TIME $1 %e s maxrss %M kB" "${@:2}"; }
mkdir -p $O
echo "$(date +%T) B1080-Apose"
T b1080a $P scan3d/scanA/scan_offline.py $L/kfonly1080/red1080/hs_20261022_133820_kfonly1080 --out $O/B1080_Apose $MS --vio $AP > $O/b1080a.log 2>&1
echo "$(date +%T) B960-Apose"
T b960a $P scan3d/scanA/scan_offline.py $L/kfonly/red/hs_20261022_133820_kfonly --out $O/B960_Apose $MS --vio $AP > $O/b960a.log 2>&1
echo "$(date +%T) done"
