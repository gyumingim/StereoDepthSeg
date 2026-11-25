#!/bin/bash
# E8 final build: kept settings (E7 flags on the E6 DA3 fuse, mesh zmax 5 + depth sigma, v3fix, carve mv5, E3c if E3C=1),
# trained on ALL 494 keyframes (--hold 0), texture with all views except kf 0, no gains. P4 not measurable (quote E7-DA3).
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$PWD/scan3d/runs/gsmap_20261125_121801; export GSMAP_RUN=$N; source $HOME/scanner/gsplat_env/env.sh
D=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
d=$N/e8
[ -f $d/gs/summary.json ] || { echo "== train $(date +%T)"; $P scan3d/gsmap/gsm_train.py $N/dump_ba --fused $N/e6/fuse --out $d/gs --surfel 1 --mask-border 1 --fill-w 0.1 --aniso-max 5 --aniso-w 1.0 --max-scale 0.05 --voxel 0.025 --max-gs 2500000 --steps 30000 --exposure 1 --normals $N/depth_moge_n --normal-w 0.05 --hold 0 > $d/train.log 2>&1 || { echo FAIL train; exit 1; }; }
[ -f $d/gs/mesh.json ] || { echo "== mesh $(date +%T)"; GSMAP_WELD=1 GSMAP_TRUNC=0.04 GSMAP_ZMAX=5.0 GSMAP_ZSIG=1 $P scan3d/gsmap/gsm_mesh2d.py $d/gs $d/gs/mesh.ply > $d/mesh.log 2>&1 || { echo FAIL mesh; exit 1; }; }
[ -f $d/v3fix.json ] || { echo "== v3fix $(date +%T)"; $P scan3d/gsmap/gsm_v3fix.py $d/gs/mesh.ply $d/v3fix.ply > $d/v3fix.log 2>&1 || { echo FAIL; exit 1; }; }
[ -f $d/carve.json ] || { echo "== carve $(date +%T)"; $P scan3d/gsmap/gsm_carve.py $d/v3fix.ply $d/carve.ply --min-viol 5 --island-area 0 --min-tri 0 > $d/carve.log 2>&1 || { echo FAIL; exit 1; }; }
M=$d/carve.ply
if [ "${E3C:-0}" = 1 ]; then [ -f $d/e3c.json ] || $P scan3d/gsmap/gsm_e3c.py $d/carve.ply $d/e3c.ply > $d/e3c.log 2>&1; M=$d/e3c.ply; fi
[ -f $d/tex_121801_final.glb ] || { echo "== tex $(date +%T)"; $P scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 $M $d/tex_121801_final.glb > $d/tex.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $d/tex_121801_final.glb >> $d/tex.log 2>&1 || { echo FAIL tex; exit 1; }; }
[ -f $d/eval.json ] || { echo "== eval $(date +%T)"; $P scan3d/gsmap/gsm_eval2.py $d/tex_121801_final.glb $d/eval.json --tex-json $d/tex_121801_final.json > $d/eval.log 2>&1; }
GSMAP_SPOTS="72:0:0:cabinet kf774;24:0:0:bin kf458;48:0:0:wall kf625;52:0:20:ceiling +20;444:0:0:floor + chair;336:0:0:desk kf2580;72:25:0:cabinet side 25;200:0:35:inside looking up" \
  $P scan3d/gsmap/gsm_crops.py $O $d/final_121801.jpg v4=$N/e1/tex_121801_v4.glb v5=$N/final/tex_121801_v5.glb final=$d/tex_121801_final.glb > $d/sheet.log 2>&1
cp $d/tex_121801_final.glb $d/final_121801.jpg $D/
echo E8DONE $(date +%T)
