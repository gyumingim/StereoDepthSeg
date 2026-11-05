#!/bin/bash
# gsplat step 1: 2DGS A/B on the dumps (ba, ba_fs), then score GS + TSDF meshes with the same scorer. Sequential GPU jobs.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
source $HOME/scanner/gsplat_env/env.sh
P=$HOME/scanner/gsplat_env/venv/bin/python
O=scan3d/runs/gsplat_20261105
for r in ${RUNS:-ba_fs ba}; do
  for v in ${VARS:-B A}; do
    n=gs${v}_$r
    if [ ! -f $O/gs/$n/summary.json ]; then
      echo "== $n $(date +%T)"
      nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits -l 2 > $O/$n.gpu.txt & G=$!
      $P scan3d/gsplat/gs_train.py $O/dump/$r --out $O/gs/$n --variant $v ${GSARGS} > $O/$n.log 2>&1
      kill $G
      tail -n 1 $O/$n.log | head -c 400; echo
    fi
    $P scan3d/gsplat/gs_score.py --cams $O/dump/$r/cams.npz --mesh $O/gs/$n/mesh.ply --kf-json $O/tsdf/$r/keyframes.json \
       --depth-dir $O/gs/$n/depth --out $O/score/$n.json > /dev/null 2>&1 || echo "score $n failed"
  done
  $P scan3d/gsplat/gs_score.py --cams $O/dump/$r/cams.npz --mesh $O/tsdf/$r/mesh.ply --kf-json $O/tsdf/$r/keyframes.json \
     --depth-dir $O/dump/$r/ffs --out $O/score/tsdf_$r.json > /dev/null 2>&1 || echo "score tsdf_$r failed"
done
echo ALLDONE $(date +%T)
