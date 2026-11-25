#!/bin/bash
# gsmap 2026-11-25: train + render the 3 variants one at a time (A = FFS only, B = FFS + DA3 fill, C = FFS + MoGe-2 fill).
# Resume: finished variants (summary.json + chase.mp4) are skipped. Log: runs/gsmap_20261125/run_all.log
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
source $HOME/scanner/gsplat_env/env.sh
P=$HOME/scanner/gsmap_env/venv/bin/python
R=scan3d/runs/gsmap_20261125
for v in ${VARS:-A:ffs B:da3 C:moge}; do
  n=gs${v%%:*}; f=${v##*:}
  if [ ! -f $R/$n/summary.json ]; then
    echo "== train $n ($f) $(date +%T)"
    ( while true; do nvidia-smi --query-gpu=memory.used,power.draw --format=csv,noheader,nounits; sleep 2; done ) > $R/$n.gpu.txt & G=$!
    $P -W ignore scan3d/gsmap/gsm_train.py $R/dump_ba --fused $R/fuse/$f --out $R/$n ${GSARGS} > $R/$n.log 2>&1
    kill $G
    tail -n 1 $R/$n.log | head -c 600; echo
  fi
  if [ ! -f $R/$n/chase.mp4 ]; then
    echo "== render $n $(date +%T)"
    $P -W ignore scan3d/gsmap/gsm_render.py $R/$n >> $R/$n.log 2>&1 || echo "render $n failed"
  fi
done
echo ALLDONE $(date +%T)
