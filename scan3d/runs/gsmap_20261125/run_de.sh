#!/bin/bash
# gsmap 2026-11-25 follow-up after seeing gsB/gsC (dark streaks, more floaters): D = DA3 fill after the multi-view free-space
# check (fuse/da3c_0.3, fill_check.py --min-frac 0.3), E = DA3 fill unchanged but no depth loss on filled pixels (--fill-w 0).
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
source $HOME/scanner/gsplat_env/env.sh
P=$HOME/scanner/gsmap_env/venv/bin/python
R=scan3d/runs/gsmap_20261125
for spec in "gsD fuse/da3c_0.3 " "gsE fuse/da3 --fill-w 0"; do
  set -- $spec; n=$1; f=$2; shift 2
  if [ ! -f $R/$n/summary.json ]; then
    echo "== train $n ($f $*) $(date +%T)"
    ( while true; do nvidia-smi --query-gpu=memory.used,power.draw --format=csv,noheader,nounits; sleep 2; done ) > $R/$n.gpu.txt & G=$!
    $P -W ignore scan3d/gsmap/gsm_train.py $R/dump_ba --fused $R/$f --out $R/$n "$@" > $R/$n.log 2>&1
    kill $G
  fi
  [ -f $R/$n/chase.mp4 ] || { echo "== render $n $(date +%T)"; $P -W ignore scan3d/gsmap/gsm_render.py $R/$n >> $R/$n.log 2>&1 || echo "render $n failed"; }
done
echo ALLDONE $(date +%T)
