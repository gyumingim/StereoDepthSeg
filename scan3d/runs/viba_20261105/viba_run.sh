#!/bin/bash
# viba 11-05: per recording -> baseline kf_ba (+dump) -> VI-BA (yaml / online calib / x1.03 init) -> depth+TSDF (ba, viba).
#   bash viba_run.sh STAGE REC_SHORT ...   STAGE = ba | viba | mesh | all
# GPU only in kf_ba features/matching and scan_offline (gpu_guard lock ~/scanner/gpu.lock). RAM guard: stop MY jobs when
# MemAvailable < 10 GB (scanA's own watchdog kills its jobs at 8 GB; mine stop first).
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python; PV=$HOME/scanner/viba_work/venv/bin/python
D=scan3d/runs/viba_20261105; BL=$HOME/scanner/build_livepose
export FFS_ROOT=$HOME/scanner/StereoDepthSeg
( while sleep 5; do a=$(awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo)
    if [ "$a" -lt 7 ]; then echo "WATCHDOG $(date +%T) MemAvailable ${a} GB (log only, my jobs are small)"; fi; done ) &
WD=$!
trap "kill $WD 2>/dev/null" EXIT
ST=$1; shift
for r in "$@"; do
  if [[ $r == *165506 ]]; then
    R=scan3d/recordings/hs_${r}_std60; G=imu_gt_spl; CFG=$BL/cfg_gen/c4f60; VIO=$D/vio/$r.poses.jsonl.gz; BAX=""; MX=""
  else
    R=scan3d/recordings/hs_$r; G=imu_gt_spl_cal1022; CFG=$BL/cfg_gen_hs222/c4f60; VIO=$D/vio/$r.poses.jsonl.gz
    BAX="--kf-from-vio --match-ms 1"; MX="--compress --kf-step 1"
  fi
  if [[ $ST == ba || $ST == all ]]; then
    echo "== $r BA $(date +%T)"
    $P scan3d/viba/kf_ba.py $R --vio $VIO --out $D/ba/$r --gt $G $BAX --dump > $D/ba/$r.log 2>&1 || { echo "   BA failed rc $?"; continue; }
    rm -f $D/ba/$r/db.db
    grep -E "^\[sim3" $D/ba/$r.log
  fi
  if [[ $ST == viba || $ST == all ]]; then
    for v in "ba_g:--vision-only" "viba:--calib yaml" "viba_on:--calib online" "viba_x103:--calib yaml --init-scale 1.03"; do
      n=${v%%:*}; args=${v#*:}; mkdir -p $D/viba/$r/$n
      echo "== $r $n $(date +%T)"
      $PV scan3d/viba/viba.py $R --dump $D/ba/$r/viba_input.npz --vio $VIO --cfg $CFG --out $D/viba/$r/$n $args > $D/viba/$r/$n.log 2>&1 \
        || echo "   $n failed rc $?"
      grep -E "^\[lm\]" $D/viba/$r/$n.log
    done
  fi
  if [[ $ST == mesh || $ST == all ]]; then
    for n in ba viba; do
      f=$D/ba/$r/poses_ba.jsonl.gz; [[ -f $f ]] || f=$D/viba/$r/ba_g/poses_viba.jsonl.gz   # kf_ba if run, else same-solver ba_g
      [[ $n == viba ]] && f=$D/viba/$r/viba/poses_viba.jsonl.gz
      [ -f $D/mesh/${r}_$n/summary.json ] && continue
      echo "== $r mesh $n $(date +%T)"
      $P scan3d/viba/scan_offline.py $R --out $D/mesh/${r}_$n --gt $G --vio $f $MX > $D/mesh/${r}_$n.log 2>&1 || echo "   mesh $n failed rc $?"
    done
  fi
done
echo ALLDONE $ST $(date +%T)
