#!/bin/bash
# scanA resolution test (README 8): depth + TSDF at 960x540 vs 1920x1080, BA poses from 960 or 1920 features. Sequential GPU jobs.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
R=$W/scan3d/recordings/hs_20261024_165506_std60
D=scan3d/runs/scanA_20261105; O=$D/res; mkdir -p $O
export FFS_ROOT=$HOME/scanner/StereoDepthSeg
echo "== BA full-res features $(date +%T)"
/usr/bin/time -v $P scan3d/scanA/kf_ba.py $R --vio $D/step1/B_o0.poses.jsonl.gz --out $D/step2/ba_o0_full --scale 1.0 \
  > $O/ba_full.log 2>&1; grep -E "^\[sim3|Maximum resident|Elapsed" $O/ba_full.log; rm -f $D/step2/ba_o0_full/db.db
run() { n=$1; shift; echo "== $n $(date +%T)"
  ( while sleep 2; do nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits; done ) > $O/$n.gpu.txt & G=$!
  $P scan3d/scanA/scan_offline.py $R --out $O/$n "$@" > $O/$n.log 2>&1; kill $G
  echo "   nvidia-smi max MiB $(sort -n $O/$n.gpu.txt | tail -n 1)"; }
for res in 0.5 1.0; do
  r=$([ $res = 0.5 ] && echo r960 || echo r1920)
  for pz in ba ba_full; do
    f=$D/step2/$([ $pz = ba ] && echo ba_o0 || echo ba_o0_full)/poses_ba.jsonl.gz
    run ${r}_$pz --vio $f --scale $res
    run ${r}_${pz}_fs --vio $f --scale $res --free-scale
  done
done
echo ALLDONE $(date +%T)
