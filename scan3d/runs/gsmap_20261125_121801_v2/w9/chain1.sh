#!/bin/bash
R=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/gsmap/run_w9.sh; L=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2/w9/run.log
D=$(dirname $L)
echo "$(date '+%F %T') START w9 chain1 (singles o1 lu ga wt wr fi + judge1 + detail1)" >> $L
for spec in "o1 --seam-occl 1" "lu --seam-mode luma" "ga --seam-mode gain" "wt --seam-target w" "wr --seam-target wr" "fi --seam-fill 1"; do
  $R var $spec
done
ok=""; for v in o1 lu ga wt wr fi; do [ -e $D/$v/tex_121801_nC_$v.glb ] && ok="$ok $v"; done
$R judge rjudge1 $ok
$R detail detail1 $ok
echo "$(date '+%F %T') END w9 chain1 ok:$ok" >> $L
