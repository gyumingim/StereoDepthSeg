#!/bin/bash
# gsmap 2026-11-25 v3 (user order): border mask + anti-spike (aniso penalty above 5, max scale 5 cm). A2 then B2; copy ply to splat_demo.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
source $HOME/scanner/gsplat_env/env.sh
P=$HOME/scanner/gsmap_env/venv/bin/python
R=scan3d/runs/gsmap_20261125
D=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo
for v in A2:ffs B2:da3; do
  n=gs${v%%:*}; f=${v##*:}
  echo "== train $n ($f) $(date +%T)"
  $P -W ignore scan3d/gsmap/gsm_train.py $R/dump_ba --fused $R/fuse/$f --out $R/$n --mask-border 1 --aniso-max 5 --aniso-w 1.0 --max-scale 0.05 > $R/$n.log 2>&1
  cp $R/$n/splat.ply $D/$n.ply.tmp && mv $D/$n.ply.tmp $D/$n.ply && echo "COPIED $n $(date +%T)"
done
echo ALLDONE $(date +%T)
