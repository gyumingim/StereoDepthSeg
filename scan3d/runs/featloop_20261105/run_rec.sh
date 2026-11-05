#!/bin/bash
# featloop 11-05: per recording, 4 BA variants (S X SL XL; README 2) -> corner ruler (all variants + VIO kf, one call) -> depth+TSDF per variant.
# usage: run_rec.sh ID [VARIANTS...]   ID = 165506 or an H id (e.g. 20261022_134204). Sequential; GPU via gpu_guard lock.
# RAM watchdog: kill my running python job if MemAvailable < 8 GB.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
D=scan3d/runs/featloop_20261105; CACHE=$HOME/scanner/featloop_ext/cache
export FFS_ROOT=$HOME/scanner/StereoDepthSeg SCAN3D_GPU_POLL_S=3
id=$1; shift; VARS=${@:-S X SL XL}
if [ "$id" = 165506 ]; then
  R=$W/scan3d/recordings/hs_20261024_165506_std60; G=imu_gt_spl; GR=imu_gt_rs; V=scan3d/runs/scanA_20261105/step1/B_o0.poses.jsonl.gz
  KFO=""; CEO="--kf-step 10"; SOO=""
else
  R=$W/scan3d/recordings/hs_$id; G=imu_gt_spl_cal1022; GR=imu_gt_rs_cal1022; V=$D/vio/$id.poses.jsonl.gz
  KFO="--kf-from-vio --match-ms 1"; CEO="--kf-step 1 --tol-ms 1 --max-gap-rows 1200"; SOO="--compress --kf-step 1"
fi
O=$D/rec/$id; mkdir -p $O
# RAM: wait for >= 10 GB available before each job; watchdog kills my job only if MemAvailable < 8 GB AND my job uses >= 3 GB
# (other agents' jobs can take the machine down to 0 GB; killing my 1 GB job then does not help)
memwait() { while [ $(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo) -lt 10 ]; do sleep 30; done; }
( while sleep 5; do a=$(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo)
    if [ "$a" -lt 8 ]; then for p in $(pgrep -f "featloop_20261105/rec/$id/"); do r=$(awk '/VmRSS/{print int($2/1048576)}' /proc/$p/status 2>/dev/null)
      [ "${r:-0}" -ge 3 ] && { echo "WATCHDOG $(date +%T) MemAvailable ${a} GB, my pid $p ${r} GB -> kill"; kill $p; }; done; fi; done ) &
WD=$!
for v in $VARS; do
  case $v in S*) F=sift;; X*) F=xfeat_lg;; esac
  case $v in *L) L=salad;; *) L=none;; esac
  [ -f $O/ba_$v/poses_ba.jsonl.gz ] && { echo "== $id $v BA exists"; continue; }
  memwait
  echo "== $id $v BA ($F, loop $L) $(date +%T)"
  $P scan3d/scanA/kf_ba.py $R --vio $V --out $O/ba_$v --gt $G $KFO --feat $F --loop $L --cache-root $CACHE --seed 0 --keep-db > $O/ba_$v.log 2>&1 \
    || echo "   BA $v failed rc $?"
  grep -E "^\[(sim3|pairs)\]" $O/ba_$v.log | cut -c1-250
done
PO="--pose VIO=$O/ba_S/poses_vio_kf.jsonl.gz"
for v in $VARS; do [ -f $O/ba_$v/poses_ba.jsonl.gz ] && PO="$PO --pose $v=$O/ba_$v/poses_ba.jsonl.gz"; done
echo "== $id corner $(date +%T)"
$P scan3d/scanA/corner_eval.py $R $PO --gt $G --gt-rs $GR $CEO --out $O/corner.json > $O/corner.log 2>&1 || echo "   corner failed rc $?"
tail -n 6 $O/corner.log | cut -c1-200
for v in $VARS; do
  [ -f $O/ba_$v/poses_ba.jsonl.gz ] || continue
  [ -f $O/mesh_$v/summary.json ] && { echo "== $id $v mesh exists"; continue; }
  memwait
  echo "== $id $v mesh $(date +%T)"
  $P scan3d/scanA/scan_offline.py $R --out $O/mesh_$v --gt $G --vio $O/ba_$v/poses_ba.jsonl.gz $SOO > $O/mesh_$v.log 2>&1 \
    || echo "   mesh $v failed rc $?"
  tail -n 1 $O/mesh_$v.log | cut -c1-200
done
kill $WD
echo "DONE $id $(date +%T)"
