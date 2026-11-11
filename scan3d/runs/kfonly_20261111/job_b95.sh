#!/bin/bash
# kfonly 11-11 follow-up (README 7, not the pre-registered rule): B with JPEG quality 95 (B95). Same as job_rec.sh B part, own out dir.
# usage: job_b95.sh REC_DIR OUT_DIR VIO_POSES
set -e
R=$1; O=$2; V=$3; Q=95
P=${P:-/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python}
CACHE=${CACHE:-/home/system/scanner/laptop_cache}
G=imu_gt_spl_cal1022; D=scan3d/runs/kfonly_20261111
KF="--gt $G --kf-from-vio --match-ms 1 --feat sift --loop none --seed 0"; MS="--gt $G --compress --kf-step 1"
mkdir -p $O/B95
T() { /usr/bin/time -f "TIME $1 %e s maxrss %M kB" "${@:2}"; }
RED=$O/red95/$(basename $R)_kfonly95
echo "$(date +%T) reduce q95 -> $RED"
T reduce $P $D/make_reduced.py $R $V --out $RED --quality $Q --gt $G > $O/reduce95.log 2>&1
[ -e $RED/cam5.mp4 ] && { echo "reduced folder holds a video"; exit 9; }
echo "$(date +%T) B95 kf_ba"
[ -f $O/B95/ba_S/poses_ba.jsonl.gz ] || T B95_kfba $P scan3d/scanA/kf_ba.py $RED --vio $RED/phone_live_poses.jsonl.gz --out $O/B95/ba_S $KF --cache-root $CACHE/B95 > $O/B95/kfba.log 2>&1
echo "$(date +%T) B95 mesh"
[ -f $O/B95/mesh_S/summary.json ] || T B95_mesh $P scan3d/scanA/scan_offline.py $RED --out $O/B95/mesh_S --vio $O/B95/ba_S/poses_ba.jsonl.gz $MS > $O/B95/mesh.log 2>&1
rm -f $O/B95/ba_S/db.db
echo "$(date +%T) done"
