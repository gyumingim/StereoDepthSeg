#!/bin/bash
# gsmap 2026-11-25 (user order): 2DGS surfels, FFS only, border mask, 20000 steps; after run_v3 (B2). Copy ply as gs2D.ply.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
source $HOME/scanner/gsplat_env/env.sh
P=$HOME/scanner/gsmap_env/venv/bin/python
R=scan3d/runs/gsmap_20261125
D=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo
until grep -q ALLDONE $R/run_v3.log; do sleep 5; done
echo "== smoke $(date +%T)"
$P -W ignore scan3d/gsmap/gsm_train.py $R/dump_ba --fused $R/fuse/ffs --out $R/smoke2d --steps 3100 --surfel 1 --mask-border 1 > $R/smoke2d.log 2>&1 || { echo SMOKEFAIL; exit 1; }
echo "== train gs2D $(date +%T)"
$P -W ignore scan3d/gsmap/gsm_train.py $R/dump_ba --fused $R/fuse/ffs --out $R/gs2D --surfel 1 --mask-border 1 > $R/gs2D.log 2>&1
cp $R/gs2D/splat.ply $D/gs2D.ply.tmp && mv $D/gs2D.ply.tmp $D/gs2D.ply && echo "COPIED gs2D $(date +%T)"
echo ALLDONE $(date +%T)
