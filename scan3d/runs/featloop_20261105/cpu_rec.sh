#!/bin/bash
# featloop: CPU reruns (no GPU lock) from the same features + raw matches (ba_V/db.db): verification with RANSAC seed s -> tracks -> BA;
# then the corner ruler over every ba_* dir. usage: [SEEDS="1 2"] [TRK=drop|greedy] rep_rec.sh ID [VARIANTS...]
# out dir: ba_V{g if greedy}{_s<seed> unless seed 0}  (seed 0 + drop = the GPU run itself: redone from its db only if its BA is missing)
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
    if [ "$a" -lt 8 ]; then for p in $(pgrep -f "kf_ba.py.*db-from.*featloop_20261105/rec/$id/"); do r=$(awk '/VmRSS/{print int($2/1048576)}' /proc/$p/status 2>/dev/null)
      [ "${r:-0}" -ge 3 ] && { echo "WATCHDOG $(date +%T) MemAvailable ${a} GB, my pid $p ${r} GB -> kill"; kill $p; }; done; fi; done ) &
WD=$!
for v in $VARS; do
  case $v in S*) F=sift;; X*) F=xfeat_lg;; esac
  case $v in *L) L=salad;; *) L=none;; esac
  # scope cut (11-06 02:10, time): on H only S and X get greedy-track reruns (SLg/XLg on 165506 only) unless ALLG=1
  if [ "${TRK:-drop}" = greedy ] && [ $id != 165506 ] && [ -z "$ALLG" ] && [ $v != S ] && [ $v != X ]; then continue; fi
  for s in ${SEEDS:-1 2}; do
    T=${TRK:-drop}; o=ba_${v}$([ $T = greedy ] && echo g)$([ $s != 0 ] && echo _s$s)
    [ -f $O/$o/poses_ba.jsonl.gz ] && continue
    SRC=ba_$v; [ -f $O/ba_${v}_gpu/db.db ] && SRC=ba_${v}_gpu
    if [ $o = ba_$v ]; then      # GPU run killed after features/matching (RAM watchdog): redo verification (same seed) + BA from its db
      [ -f $O/ba_$v/db.db ] || { echo "no db for $v"; continue; }
      rm -rf $O/ba_${v}_gpu; mv $O/ba_$v $O/ba_${v}_gpu; mv $O/ba_$v.log $O/ba_${v}_gpu.log 2>/dev/null; SRC=ba_${v}_gpu
    fi
    [ -f $O/$SRC/db.db ] || { echo "no db for $v"; continue; }
    memwait
    echo "== $id $o (seed $s, tracks $T) $(date +%T)"
    nice -n 5 $P scan3d/scanA/kf_ba.py $R --vio $V --out $O/$o --gt $G $KFO --feat $F --loop $L --cache-root $CACHE \
      --seed $s --tracks $T --db-from $O/$SRC > $O/$o.log 2>&1 || echo "   $o failed rc $?"
    grep -E "^\[(sim3|tri)\]" $O/$o.log | cut -c1-200
  done
done
PO="--pose VIO=$O/ba_S/poses_vio_kf.jsonl.gz"
for d in $O/ba_*/; do n=$(basename $d); n=${n#ba_}; [ "${n%_gpu}" != "$n" ] && continue; [ -f $d/poses_ba.jsonl.gz ] && PO="$PO --pose $n=$d/poses_ba.jsonl.gz"; done
[ -n "$CORNER" ] && $P scan3d/scanA/corner_eval.py $R $PO --gt $G --gt-rs $GR $CEO --out $O/corner_rep.json > $O/corner_rep.log 2>&1 || echo "corner failed"
tail -n 20 $O/corner_rep.log | cut -c1-220
kill $WD
echo "REPDONE $id $(date +%T)"
