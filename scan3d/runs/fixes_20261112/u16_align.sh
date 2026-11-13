#!/usr/bin/env bash
# u16 (11-13): merge_align.py only (gates, no BA / mesh) on the red folders of u11 / u10 + synthetic B x 1.05 (laptop, cwd = WT).
#   remote_laptop.sh job 121801 'cmd:bash scan3d/runs/fixes_20261112/u16_align.sh $WT/$O' --tag fix_u16align --send <red dirs>
set -u
CAL=${CAL:-}   # e.g. "--min-cells 0 --min-extent-m 0 --min-span-s 0" = calibration pass (spread gates off)
O=$1; mkdir -p "$O"
WT=$(pwd); P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
L=$WT/scan3d/runs/laptop/20261112_121801_std60
LENS=$WT/scan3d/config/lens/std_1920x1080_f0.50_calcap20261026.json
U11A=$L/fix_u11merge/merge/part1/red/hs_20261112_121801_std60_kf; U11B=$L/fix_u11merge/merge/part2/red/hs_20261112_131328_std60_kf
U10A=$L/fix_u10split/merge/part1/red/hs_20261112_121801_std60_kf; U10B=$L/fix_u10split/merge/part2/red/hs_20261112_121801_std60_kf
run() { local n=$1; shift; local t0=$SECONDS
  $P scan3d/scanA/merge_align.py "$@" --out "$O/$n/merged_kf" --lens "$LENS" ${CAL:-} ${XA:-} > "$O/$n.log" 2>&1; local rc=$?
  echo "$(date +%T) $n rc $rc $((SECONDS - t0)) s" | tee -a "$O/rc.txt"; grep '^\[part 1\] 4-DoF\|MERGE FAILED' "$O/$n.log" | tail -n 2; }
run u11 "$U11A" "$U11B"
run u10 "$U10A" "$U10B"
$P scan3d/runs/fixes_20261112/u16_scale_part.py "$U11B" "$O/u11B_x105" 1.05
$P scan3d/runs/fixes_20261112/u16_scale_part.py "$U11B" "$O/u11B_x095" 0.95238
run u11_x105 "$U11A" "$O/u11B_x105"
run u11_x095 "$U11A" "$O/u11B_x095"
XA="--scale-mode sim3" run u11_x105_sim3 "$U11A" "$O/u11B_x105"
rm -rf "$O"/u11B_x105/frames_kf "$O"/u11B_x095/frames_kf "$O"/*/merged_kf/frames_kf "$O"/*/merged_kf_align/img "$O"/*/merged_kf_align/*.db
exit 0
