#!/usr/bin/env bash
# laptop job (redscreen 11-11): VIO c4f60 -> lens-only GT folder -> run_scanner defaults with --vio/--gt.  Args: R O (cwd = WT)
set -euo pipefail
R=$1; O=$2; WT=$(pwd); B=/home/system/scanner/build_livepose; P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
LENS=$WT/scan3d/config/lens/std_1920x1080_f0.50_calcap20261026.json; G=lensonly_f050
mkdir -p $O/vio
( cd $WT/scan3d/runs/livepose_20261030 && $P lp_replay.py "$R" $B/cfg_gen/c4f60 $WT/$O/vio/B_o0 --bridge $B/bridge/ov_bridge \
    --vocab $WT/scan3d/ov_bridge/loop/voc_k10L4.voc > $WT/$O/vio.log 2>&1 )
echo "$(date +%T) vio done"
$P scan3d/runs/redscreen_20261111/make_lensonly_gt.py "$R" $O/vio/B_o0.poses.jsonl.gz $LENS $G
cp -r "$R/$G" $O/
rc=0
SCANNER_COMMIT=${SCANNER_COMMIT:-fa7558a+redscreen} bash scan3d/scanA/run_scanner.sh "$R" $WT/$O/scan --vio $WT/$O/vio/B_o0.poses.jsonl.gz \
  --gt $G --cfg $B/cfg_gen/c4f60 || rc=$?
echo "run_scanner rc $rc (a summary KeyError is expected: no board pooled dZ)"
ls $O/scan $O/scan/mesh
