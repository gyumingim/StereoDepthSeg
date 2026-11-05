#!/bin/bash
# scanA step 3: same depth + TSDF (5 mm) + scoring, only the poses change. Sequential (one GPU job at a time).
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
R=$W/scan3d/recordings/hs_20261024_165506_std60
O=scan3d/runs/scanA_20261105/step3; S2=scan3d/runs/scanA_20261105/step2
mkdir -p $O
export FFS_ROOT=$HOME/scanner/StereoDepthSeg
run() { n=$1; shift; echo "== $n $(date +%T)"; $P scan3d/scanA/scan_offline.py $R --out $O/$n "$@" > $O/$n.log 2>&1; tail -n 2 $O/$n.log | head -c 300; echo; }
run gt
for fs in "" "--free-scale"; do
  t=${fs:+_fs}
  run vio_all$t --vio scan3d/runs/scanA_20261105/step1/B_o0.poses.jsonl.gz $fs
  run vio_kf$t  --vio $S2/ba_o0/poses_vio_kf.jsonl.gz $fs
  run ba$t      --vio $S2/ba_o0/poses_ba.jsonl.gz $fs
  run ba_rs$t   --vio $S2/ba_o0_rs/poses_ba.jsonl.gz $fs
  run ba_mask_rs$t --vio $S2/ba_o0_mask_rs/poses_ba.jsonl.gz $fs
done
echo ALLDONE $(date +%T)
