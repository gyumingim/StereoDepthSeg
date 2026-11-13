#!/usr/bin/env bash
# fixes 11-13 unit 15: commit only this unit's files by explicit path (no git add -A, no push)
set -e
cd /home/system/scanner/StereoDepthSeg-fixes
F=scan3d/runs/fixes_20261112
files=(scan3d/motion_stereo.py scan3d/scanA/scan_offline.py $F/README.md
  $F/u15_job.sh $F/u15_far.sh $F/u15_eval.sh $F/u15_table.py $F/u15_guard_where.py $F/u15_where_laptop.sh
  $F/u15_laptop.sh $F/u15_laptop_launch.sh $F/u15_laptop_far.sh $F/u15_laptop_start.sh $F/u15_status.sh $F/u15_commit_list.sh $F/u15_commit.sh)
for f in $F/u15_times.txt $F/u15_wait.log $F/u15_laptop_logs/board.log $F/u15_laptop_logs/office.log $F/u15_laptop_logs/launch.txt; do [ -f $f ] && files+=($f); done
mapfile -t res < <(bash $F/u15_commit_list.sh)
git add -- "${files[@]}" "${res[@]}"
git status --short | grep -v '^??' | head -5
git diff --cached --stat | tail -3
git commit -q -F - <<'MSG'
fixes 11-13 unit 15: depth sigma uses the FFS downscale (default on) + wide-only pixel guard for --far-pairs wide

- motion_stereo.pair_depth: MSConfig.sigma_scale_aware (sigma_d_px / s) and guard_masks (left-right check, saturation mask);
  both default off for other callers
- scan_offline: --sigma-scale-aware on|off (default on; board 165506 p90 identical on/off on laptop PyTorch 2.9531 and
  desktop TRT 2.8944 cm), --far-guard lr+sat, --far-partner2 on, --far-repl-min-z on (defaults of the opt-in far path),
  far_pairs counters for rejected wide-only pixels
- laptop PyTorch comparison on 121801 + desktop TRT record, README section 15 (1 run each, unverified)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
MSG
git log --oneline -1
