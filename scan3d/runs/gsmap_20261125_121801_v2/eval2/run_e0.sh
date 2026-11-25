#!/bin/bash
# E0 baselines: held-out textures (no held-out views) for old / v2 / v3 meshes, then eval2 on the delivered GLBs.
cd ~/scanner/StereoDepthSeg-gsmap
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$PWD/scan3d/runs/gsmap_20261125_121801; export GSMAP_RUN=$N
P="nice -n 19 env OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
TX="scan3d/gsmap/gsm_texture2.py --dump $N/dump_ba --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --skip-held 1"
for spec in "old:$O/gs2DM/mesh2d_pf.ply" "v2:$N/gs2DM/mesh_t4.ply" "v3:$N/v3/mesh_v3.ply"; do
  n=${spec%%:*}; m=${spec#*:}
  [ -f $N/eval2/held/$n.glb ] || { echo "== tex $n $(date +%T)"; $P $TX $m $N/eval2/held/$n.glb > $N/eval2/held/$n.log 2>&1 && $P scan3d/gsmap/gsm_atlas8k.py $N/eval2/held/$n.glb >> $N/eval2/held/$n.log 2>&1 || echo "FAIL tex $n"; }
done
for spec in "old:$O/tex_121801.glb:$O/tex_121801.json" "v2:$N/tex_121801_v2.glb:$N/tex_121801_v2.json" "v3:$N/v3/tex_121801_v3.glb:$N/v3/tex_121801_v3.json"; do
  IFS=: read n g j <<< "$spec"
  [ -f $N/eval2/$n.json ] || { echo "== eval $n $(date +%T)"; $P scan3d/gsmap/gsm_eval2.py $g $N/eval2/$n.json --heldout-glb $N/eval2/held/$n.glb --tex-json $j > $N/eval2/$n.log 2>&1 || echo "FAIL eval $n"; }
done
echo E0DONE $(date +%T)
