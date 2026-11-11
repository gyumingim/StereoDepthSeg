#!/bin/bash
# kfonly 11-11: ONE recording on the laptop worker (README.md): A = full pipeline (kf_ba SIFT seed 0 -> FFS+TSDF), then the
# reduced recording (make_reduced.py: keyframe JPEGs + poses + IMU, no video), then B = the same two steps reading ONLY the reduced folder.
# usage (cwd = worktree root, run by remote_laptop.sh 'cmd:'): job_rec.sh REC_DIR OUT_DIR VIO_POSES [JPEG_QUALITY]
set -e
R=$1; O=$2; V=$3; Q=${4:-90}
P=${P:-/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python}
CACHE=${CACHE:-/home/system/scanner/laptop_cache}
G=imu_gt_spl_cal1022
D=scan3d/runs/kfonly_20261111
KF="--gt $G --kf-from-vio --match-ms 1 --feat sift --loop none --seed 0"
MS="--gt $G --compress --kf-step 1"
mkdir -p $O/A $O/B
T() { /usr/bin/time -f "TIME $1 %e s maxrss %M kB" "${@:2}"; }
echo "$(date +%T) A kf_ba"
[ -f $O/A/ba_S/poses_ba.jsonl.gz ] || T A_kfba $P scan3d/scanA/kf_ba.py $R --vio $V --out $O/A/ba_S $KF --cache-root $CACHE/A > $O/A/kfba.log 2>&1
echo "$(date +%T) A mesh"
[ -f $O/A/mesh_S/summary.json ] || T A_mesh $P scan3d/scanA/scan_offline.py $R --out $O/A/mesh_S --vio $O/A/ba_S/poses_ba.jsonl.gz $MS > $O/A/mesh.log 2>&1
RED=$O/red/$(basename $R)_kfonly
echo "$(date +%T) reduce -> $RED"
T reduce $P $D/make_reduced.py $R $V --out $RED --quality $Q --gt $G --check $O/A/ba_S/poses_ba.jsonl.gz > $O/reduce.log 2>&1
[ -e $RED/cam5.mp4 ] && { echo "reduced folder holds a video"; exit 9; }
echo "$(date +%T) B kf_ba (reads only $RED)"
[ -f $O/B/ba_S/poses_ba.jsonl.gz ] || T B_kfba $P scan3d/scanA/kf_ba.py $RED --vio $RED/phone_live_poses.jsonl.gz --out $O/B/ba_S $KF --cache-root $CACHE/B > $O/B/kfba.log 2>&1
echo "$(date +%T) B mesh"
[ -f $O/B/mesh_S/summary.json ] || T B_mesh $P scan3d/scanA/scan_offline.py $RED --out $O/B/mesh_S --vio $O/B/ba_S/poses_ba.jsonl.gz $MS > $O/B/mesh.log 2>&1
rm -f $O/A/ba_S/db.db $O/B/ba_S/db.db
echo "$(date +%T) done"
