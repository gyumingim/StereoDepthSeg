#!/usr/bin/env bash
# fixes 11-13 unit 15: record code state, then start u15_laptop_launch.sh detached (desktop WSL orchestrates, laptop computes).
cd /home/system/scanner/StereoDepthSeg-fixes
W=${1:-both}
echo "$(date '+%m-%d %T') laptop launch ($W): git $(git rev-parse --short HEAD)-dirty, scan_offline sha $(sha256sum scan3d/scanA/scan_offline.py | cut -c1-16), motion_stereo sha $(sha256sum scan3d/motion_stereo.py | cut -c1-16)" \
  | tee -a /mnt/c/Users/a3162/scan3d_handoff/review_20261112/interim_u15.txt
setsid nohup bash scan3d/runs/fixes_20261112/u15_laptop_launch.sh "$W" > /dev/null 2>&1 < /dev/null &
echo launched
