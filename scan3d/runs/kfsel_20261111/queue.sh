#!/usr/bin/env bash
# kfsel 11-11 queue: one laptop job per variant (sequential; waits on the laptop lock)
cd /home/system/scanner/StereoDepthSeg-kfsel/scan3d
export WT=/home/system/scanner/StereoDepthSeg-kfsel
V=/home/system/scanner/StereoDepthSeg-kfsel/scan3d/runs/kfsel_20261111/vio/20261022_133820.poses.jsonl.gz
run() { local tag=$1; shift; echo "$(date '+%F %T') start $tag"; tools/remote_laptop.sh job 133820 scan --vio $V --tag kfsel_$tag --args "$*"; echo "$(date '+%F %T') end $tag rc $?"; }
run M3 --kf-select motion --kf-dist 3 --kf-rot 4 --kf-win 4 --kf-blur lap
run I --kf-select interval
run M2 --kf-select motion --kf-dist 2 --kf-rot 3 --kf-win 4 --kf-blur lap
run M4 --kf-select motion --kf-dist 4 --kf-rot 6 --kf-win 4 --kf-blur lap
run M3g --kf-select motion --kf-dist 3 --kf-rot 4 --kf-win 4 --kf-blur gyro
echo "$(date '+%F %T') queue done"
