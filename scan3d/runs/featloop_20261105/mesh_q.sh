#!/bin/bash
# featloop mesh queue: depth + TSDF (scan_offline 960x540, BA poses, VIO scale) for "ID:VARIANT ..." items, sequential (GPU lock),
# waits for >= 12 GB free RAM before each, retries a job killed by the RAM watchdog (rc 143) up to 4 times.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python; D=scan3d/runs/featloop_20261105
export FFS_ROOT=$HOME/scanner/StereoDepthSeg SCAN3D_GPU_POLL_S=3
mem() { awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo; }
( while sleep 5; do a=$(mem); if [ "$a" -lt 8 ]; then for p in $(pgrep -f "scan_offline.py.*featloop_20261105/rec/"); do
    r=$(awk "/VmRSS/{print int(\$2/1048576)}" /proc/$p/status 2>/dev/null); [ "${r:-0}" -ge 3 ] && { echo "WATCHDOG $(date +%T) avail $a GB, pid $p $r GB -> kill"; kill $p; }; done; fi; done ) &
WD=$!
for it in "$@"; do
  id=${it%%:*}; v=${it#*:}; O=$D/rec/$id
  if [ "$id" = 165506 ]; then R=$W/scan3d/recordings/hs_20261024_165506_std60; G=imu_gt_spl; SOO=""
  else R=$W/scan3d/recordings/hs_$id; G=imu_gt_spl_cal1022; SOO="--compress --kf-step 1"; fi
  [ -f $O/mesh_$v/summary.json ] && continue
  [ -f $O/ba_$v/poses_ba.jsonl.gz ] || { echo "no BA $id $v"; continue; }
  for try in 1 2 3 4 5; do
    while [ $(mem) -lt 12 ]; do sleep 30; done
    echo "== mesh $id $v try $try $(date +%T)"
    $P scan3d/scanA/scan_offline.py $R --out $O/mesh_$v --gt $G --vio $O/ba_$v/poses_ba.jsonl.gz $SOO > $O/mesh_$v.log 2>&1; rc=$?
    [ $rc = 0 ] && break
    echo "   rc $rc"; [ $rc = 143 ] || break; sleep 300
  done
done
kill $WD; echo "MESHQ DONE $(date +%T)"
