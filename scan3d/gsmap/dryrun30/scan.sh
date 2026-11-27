#!/bin/bash
# dry run 30 fps (2026-11-27): VIO (cfg_gen/c4f60 = the 121801 config) + lensonly + reduce (3 cm / 4 deg motion keyframes), CPU only,
# for the 30 fps decimated recording and (same code, same machine) the original 60 fps recording via a symlink folder.
set -uo pipefail
D=/home/system/scanner/StereoDepthSeg-gsmap/scan3d/runs/dryrun30_121801_20261127
SRC=/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261112_121801_std60
RS=/home/system/scanner/StereoDepthSeg-fixes/scan3d/scanA/run_scanner.sh
R60=$D/rec_hs_20261112_121801_std60; mkdir -p $R60
for f in cam5.mp4 frames_cam5.csv imu.csv meta.json results_cam5.csv temps.csv; do ln -sf $SRC/$f $R60/$f; done
export OMP_NUM_THREADS=8
echo "== scan30 $(date +%T)"
nice -n 19 bash $RS $D/rec_hs_20261112_121801_std30 $D/scan30 --stop-after reduce --vio-args "--fps 30" > $D/scan30.log 2>&1; echo "scan30 rc=$?"
echo "== scan60 $(date +%T)"
nice -n 19 bash $RS $R60 $D/scan60 --stop-after reduce > $D/scan60.log 2>&1; echo "scan60 rc=$?"
echo "== done $(date +%T)"
