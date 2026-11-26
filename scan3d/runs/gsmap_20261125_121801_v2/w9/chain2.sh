#!/bin/bash
# chain2: combos on top of the chain1 winner o1 (--seam-occl 1): c1 = o1 + seam-fill, c2 = o1 + score-weighted target
R=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/gsmap/run_w9.sh; L=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2/w9/run.log
D=$(dirname $L)
echo "$(date '+%F %T') START w9 chain2 (combos c1 = o1+fi, c2 = o1+wt + judge2 + detail2)" >> $L
$R var c1 --seam-occl 1 --seam-fill 1
$R var c2 --seam-occl 1 --seam-target w
ok="o1"; for v in c1 c2; do [ -e $D/$v/tex_121801_nC_$v.glb ] && ok="$ok $v"; done
$R judge rjudge2 $ok
$R detail detail2 $ok
echo "$(date '+%F %T') END w9 chain2 ok:$ok" >> $L
