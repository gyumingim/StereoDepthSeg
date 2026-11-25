#!/bin/bash
# gsmap 2026-11-25 (user order): 2DGS on filtered FFS + MoGe-2 (fuse/moge_f, fill_w 0.1), border mask, in-plane aniso <= 5, scale cap 5 cm.
# Then mesh (gsm_mesh2d.py) + texture (gsm_texture.py); copy gs2DM.ply, mesh2d_moge.ply, tex_2dmoge.glb to splat_demo.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
source $HOME/scanner/gsplat_env/env.sh
P=$HOME/scanner/gsmap_env/venv/bin/python
R=scan3d/runs/gsmap_20261125
D=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo
echo "== train $(date +%T)"
$P -W ignore scan3d/gsmap/gsm_train.py $R/dump_ba --fused $R/fuse/moge_f --out $R/gs2DM --surfel 1 --mask-border 1 --fill-w 0.1 --aniso-max 5 --aniso-w 1.0 --max-scale 0.05 > $R/gs2DM.log 2>&1 || { echo TRAINFAIL; exit 1; }
cp $R/gs2DM/splat.ply $D/gs2DM.ply.tmp && mv $D/gs2DM.ply.tmp $D/gs2DM.ply && echo "COPIED splat $(date +%T)"
$P -W ignore scan3d/gsmap/gsm_mesh2d.py $R/gs2DM $R/gs2DM/mesh2d.ply >> $R/gs2DM.log 2>&1 && cp $R/gs2DM/mesh2d.ply $D/mesh2d_moge.ply && echo "COPIED mesh $(date +%T)"
OMP_NUM_THREADS=8 $P -W ignore scan3d/gsmap/gsm_texture.py $D/mesh2d_moge.ply $R/tex/tex_2dmoge.glb >> $R/gs2DM.log 2>&1 && cp $R/tex/tex_2dmoge.glb $D/ && echo "COPIED glb $(date +%T)"
echo ALLDONE $(date +%T)
