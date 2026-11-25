#!/bin/bash
# gsmap 2026-11-25 v2: same 4 depth inputs, trainer now ignores the black undistortion border (--mask-border 1, default).
# A2 = FFS only, B2 = FFS + DA3 fill, C2 = FFS + MoGe-2 fill, D2 = FFS + DA3 fill after the multi-view free-space check.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
source $HOME/scanner/gsplat_env/env.sh
P=$HOME/scanner/gsmap_env/venv/bin/python
R=scan3d/runs/gsmap_20261125
for v in ${VARS:-A2:ffs B2:da3 D2:da3c_0.3 C2:moge}; do
  n=gs${v%%:*}; f=${v##*:}
  if [ ! -f $R/$n/summary.json ]; then
    echo "== train $n ($f) $(date +%T)"
    ( while true; do nvidia-smi --query-gpu=memory.used,power.draw --format=csv,noheader,nounits; sleep 2; done ) > $R/$n.gpu.txt & G=$!
    $P -W ignore scan3d/gsmap/gsm_train.py $R/dump_ba --fused $R/fuse/$f --out $R/$n ${GSARGS} > $R/$n.log 2>&1
    kill $G
  fi
  [ -f $R/$n/chase.mp4 ] || { echo "== render $n $(date +%T)"; $P -W ignore scan3d/gsmap/gsm_render.py $R/$n >> $R/$n.log 2>&1 || echo "render $n failed"; }
done
echo ALLDONE $(date +%T)
