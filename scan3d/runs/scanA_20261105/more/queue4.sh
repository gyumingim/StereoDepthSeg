#!/bin/bash
# like queue3 (222747 meshes, then 214138 kf-step 20), but each job first waits for MemAvailable >= 14 GB (other agents share
# the 30 GB WSL RAM) and is retried up to 3 times if the RAM watchdog (< 8 GB, kills ONLY our own pid) stops it.
cd "$(dirname "$0")/../../../.."
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python; D=scan3d/runs/scanA_20261105/more; G=imu_gt_spl_cal1022
export FFS_ROOT=$HOME/scanner/StereoDepthSeg
mem() { awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo; }
job() {  # job LOG cmd...
  log=$1; shift
  for try in 1 2 3; do
    while [ "$(mem)" -lt 14 ]; do sleep 30; done
    "$@" > $log 2>&1 & local pid=$! killed=0
    while kill -0 $pid 2>/dev/null; do
      if [ "$(mem)" -lt 8 ]; then echo "WATCHDOG $(date +%T) MemAvailable $(mem) GB -> kill pid $pid (try $try)"; kill $pid; killed=1; fi
      sleep 5; done
    wait $pid; rc=$?; echo "   rc $rc $(date +%T)"
    [ $killed = 0 ] && return $rc
  done; return 1; }
mesh2() { r=$1
  for k in ba gt; do echo "== $r mesh $k $(date +%T)"
    if [ $k = ba ]; then a="--vio $D/ba/$r/poses_ba.jsonl.gz"; else a="--rows-from $D/ba/$r/poses_ba.jsonl.gz"; fi
    job $D/mesh/${r}_$k.log $P scan3d/scanA/scan_offline.py scan3d/recordings/hs_$r --out $D/mesh/${r}_$k --gt $G $a --compress --kf-step 1
  done; }
mesh2 20261020_222747
r=20261020_214138
echo "== $r BA (kf-step 20) $(date +%T)"
job $D/ba/$r.log $P scan3d/scanA/kf_ba.py scan3d/recordings/hs_$r --vio $D/vio/$r.poses.jsonl.gz --out $D/ba/$r --gt $G \
  --kf-from-vio --match-ms 1 --iters 12 --kf-step 20 && { rm -f $D/ba/$r/db.db; grep -E "^\[sim3" $D/ba/$r.log; mesh2 $r; }
echo ALLDONE $(date +%T)
