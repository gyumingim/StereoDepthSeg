#!/bin/bash
# featloop: seed repeats on the CPU (no GPU lock): same features + raw matches (ba_V/db.db), verification RANSAC seed 1, 2
# -> tracks -> BA; then the corner ruler over all seeds. usage: rep_rec.sh ID [VARIANTS...]
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
D=scan3d/runs/featloop_20261105; CACHE=$HOME/scanner/featloop_ext/cache
id=$1; shift; VARS=${@:-S X SL XL}
if [ "$id" = 165506 ]; then
  R=$W/scan3d/recordings/hs_20261024_165506_std60; G=imu_gt_spl; GR=imu_gt_rs; V=scan3d/runs/scanA_20261105/step1/B_o0.poses.jsonl.gz
  KFO=""; CEO="--kf-step 10"
else
  R=$W/scan3d/recordings/hs_$id; G=imu_gt_spl_cal1022; GR=imu_gt_rs_cal1022; V=$D/vio/$id.poses.jsonl.gz
  KFO="--kf-from-vio --match-ms 1"; CEO="--kf-step 1 --tol-ms 1 --max-gap-rows 1200"
fi
O=$D/rec/$id
# RAM: wait for >= 10 GB available before each job; watchdog kills my job only if MemAvailable < 8 GB AND my job uses >= 3 GB
# (other agents' jobs can take the machine down to 0 GB; killing my 1 GB job then does not help)
memwait() { while [ $(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo) -lt 10 ]; do sleep 30; done; }
( while sleep 5; do a=$(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo)
    if [ "$a" -lt 8 ]; then for p in $(pgrep -f "featloop_20261105/rec/$id/ba_.*_s[12]"); do r=$(awk '/VmRSS/{print int($2/1048576)}' /proc/$p/status 2>/dev/null)
      [ "${r:-0}" -ge 3 ] && { echo "WATCHDOG $(date +%T) MemAvailable ${a} GB, my pid $p ${r} GB -> kill"; kill $p; }; done; fi; done ) &
WD=$!
for v in $VARS; do
  case $v in S*) F=sift;; X*) F=xfeat_lg;; esac
  case $v in *L) L=salad;; *) L=none;; esac
  for s in 1 2; do
    [ -f $O/ba_${v}_s$s/poses_ba.jsonl.gz ] && continue
    [ -f $O/ba_$v/db.db ] || { echo "no db for $v"; continue; }
    memwait
    echo "== $id $v seed $s $(date +%T)"
    nice -n 5 $P scan3d/scanA/kf_ba.py $R --vio $V --out $O/ba_${v}_s$s --gt $G $KFO --feat $F --loop $L --cache-root $CACHE \
      --seed $s --db-from $O/ba_$v > $O/ba_${v}_s$s.log 2>&1 || echo "   $v s$s failed rc $?"
    grep -E "^\[sim3\]" $O/ba_${v}_s$s.log
  done
done
PO="--pose VIO=$O/ba_S/poses_vio_kf.jsonl.gz"
for d in $O/ba_*/; do n=$(basename $d); n=${n#ba_}; [ -f $d/poses_ba.jsonl.gz ] && PO="$PO --pose $n=$d/poses_ba.jsonl.gz"; done
$P scan3d/scanA/corner_eval.py $R $PO --gt $G --gt-rs $GR $CEO --out $O/corner_rep.json > $O/corner_rep.log 2>&1 || echo "corner failed"
tail -n 20 $O/corner_rep.log | cut -c1-220
kill $WD
echo "REPDONE $id $(date +%T)"
