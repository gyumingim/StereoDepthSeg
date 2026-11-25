#!/bin/bash
# after the round-3 pick: WIN / LOSE = e7 or e7da3. Full textures (all views except kf 0, no gains) -> v5 (winner) / v6 (loser),
# inside-view comparison sheet; then E3c on the winner (held-out no-gain texture + eval2).
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$PWD/scan3d/runs/gsmap_20261125_121801; export GSMAP_RUN=$N; WIN=$1; LOSE=$2
D=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
TX="scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1"
for spec in "$WIN:v5" "$LOSE:v6"; do d=${spec%%:*}; v=${spec#*:}
  [ -f $N/final/tex_121801_$v.glb ] || { echo "== tex $v ($d) $(date +%T)"; $P $TX $N/$d/carve.ply $N/final/tex_121801_$v.glb > $N/final/tex_$v.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $N/final/tex_121801_$v.glb >> $N/final/tex_$v.log 2>&1; }
done
echo "== sheet $(date +%T)"
GSMAP_SPOTS="44:-30:25:inside 1;89:30:25:inside 2;134:-30:0:inside 3;179:30:0:inside 4;224:-30:25:inside 5;313:30:25:inside 6;358:-30:0:inside 7;448:30:0:inside 8;80:0:0:panel kf813;72:0:0:cabinet kf774" \
  $P scan3d/gsmap/gsm_crops.py $O $N/final/inside_121801_v5_v6.jpg v4=$N/e1/tex_121801_v4.glb v5=$N/final/tex_121801_v5.glb v6=$N/final/tex_121801_v6.glb > $N/final/sheet.log 2>&1
cp $N/final/tex_121801_v5.glb $N/final/tex_121801_v6.glb $N/final/inside_121801_v5_v6.jpg $D/
echo "== e3c $(date +%T)"
[ -f $N/final/e3c.json ] || $P scan3d/gsmap/gsm_e3c.py $N/$WIN/carve.ply $N/final/e3c.ply > $N/final/e3c.log 2>&1
[ -f $N/final/held_e3c.glb ] || { $P $TX --skip-held 1 $N/final/e3c.ply $N/final/held_e3c.glb > $N/final/held_e3c.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $N/final/held_e3c.glb >> $N/final/held_e3c.log 2>&1; }
[ -f $N/final/eval_e3c.json ] || $P scan3d/gsmap/gsm_eval2.py $N/final/held_e3c.glb $N/final/eval_e3c.json --heldout-glb $N/final/held_e3c.glb --tex-json $N/final/held_e3c.json > $N/final/eval_e3c.log 2>&1
echo FINALDONE $(date +%T)
