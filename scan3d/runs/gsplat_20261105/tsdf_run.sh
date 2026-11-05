#!/bin/bash
# gsplat step 0: TSDF baseline re-run (= scanA step3 ba / ba_fs) with --dump of keyframe images, poses, FFS depth for 2DGS.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python        # shared env, used read-only
R=$HOME/scanner/StereoDepthSeg/scan3d/recordings/hs_20261024_165506_std60
S2=$HOME/scanner/StereoDepthSeg-scanA/scan3d/runs/scanA_20261105/step2
O=scan3d/runs/gsplat_20261105
export FFS_ROOT=$HOME/scanner/StereoDepthSeg
for fs in "" "--free-scale"; do
  t=${fs:+_fs}
  echo "== ba$t $(date +%T)"
  nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits -l 2 > $O/tsdf_ba$t.gpu.txt & G=$!
  $P scan3d/gsplat/scan_offline_dump.py $R --out $O/tsdf/ba$t --dump $O/dump/ba$t --vio $S2/ba_o0/poses_ba.jsonl.gz $fs > $O/tsdf_ba$t.log 2>&1
  kill $G
  tail -n 3 $O/tsdf_ba$t.log | head -c 300; echo
done
echo ALLDONE $(date +%T)
