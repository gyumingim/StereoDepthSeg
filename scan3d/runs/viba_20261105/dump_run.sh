#!/bin/bash
# viba: H recordings -> kf_ba (CPU SIFT, no GPU lock) dump; full kf_ba BA only for the equivalence-gate recordings (README 2.3)
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python; D=scan3d/runs/viba_20261105
one() { r=$1; nb="--no-ba"; [[ $r == 20261022_134204 || $r == 20261020_221052 ]] && nb=""
  $P scan3d/viba/kf_ba.py scan3d/recordings/hs_$r --vio $D/vio/$r.poses.jsonl.gz --out $D/ba/$r --gt imu_gt_spl_cal1022 \
     --kf-from-vio --match-ms 1 --cpu-features --dump $nb > $D/ba/$r.log 2>&1; echo "$r rc $? $(date +%T)"; rm -f $D/ba/$r/db.db; }
export -f one; export W P D
printf "%s\n" "$@" | xargs -P 2 -I{} bash -c "one {}"
echo ALLDONE $(date +%T)
