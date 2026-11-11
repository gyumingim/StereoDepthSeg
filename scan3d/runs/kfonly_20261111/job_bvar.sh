#!/bin/bash
# kfonly 11-11 follow-up variant: NAME Q FULL(0|1) -> B<NAME> (README 8 B1080). Same as job_rec.sh B part, own out dir.
# usage: job_bvar.sh REC_DIR OUT_DIR VIO_POSES NAME Q FULL
set -e
R=$1; O=$2; V=$3; N=$4; Q=$5; FULL=$6; XR=""; [ "$FULL" = 1 ] && XR=--full-res
P=${P:-/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python}
CACHE=${CACHE:-/home/system/scanner/laptop_cache}
G=imu_gt_spl_cal1022; D=scan3d/runs/kfonly_20261111
KF="--gt $G --kf-from-vio --match-ms 1 --feat sift --loop none --seed 0"; MS="--gt $G --compress --kf-step 1"
mkdir -p $O/B$N
T() { /usr/bin/time -f "TIME $1 %e s maxrss %M kB" "${@:2}"; }
RED=$O/red$N/$(basename $R)_kfonly$N
echo "$(date +%T) reduce q$Q full $FULL -> $RED"
T reduce $P $D/make_reduced.py $R $V --out $RED --quality $Q $XR --gt $G > $O/reduce$N.log 2>&1
[ -e $RED/cam5.mp4 ] && { echo "reduced folder holds a video"; exit 9; }
echo "$(date +%T) B$N kf_ba"
[ -f $O/B$N/ba_S/poses_ba.jsonl.gz ] || T B${N}_kfba $P scan3d/scanA/kf_ba.py $RED --vio $RED/phone_live_poses.jsonl.gz --out $O/B$N/ba_S $KF --cache-root $CACHE/B$N > $O/B$N/kfba.log 2>&1
echo "$(date +%T) B$N mesh"
[ -f $O/B$N/mesh_S/summary.json ] || T B${N}_mesh $P scan3d/scanA/scan_offline.py $RED --out $O/B$N/mesh_S --vio $O/B$N/ba_S/poses_ba.jsonl.gz $MS > $O/B$N/mesh.log 2>&1
rm -f $O/B$N/ba_S/db.db
echo "$(date +%T) done"
