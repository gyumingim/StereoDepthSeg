#!/usr/bin/env bash
# fixes 11-13 unit 15c (audit u15b_verify.md wording fixes, text + CPU table only): checks, then commit own files by explicit path
set -e
cd /home/system/scanner/StereoDepthSeg-fixes
F=scan3d/runs/fixes_20261112
bash -n $F/u15_laptop.sh; bash -n $F/u15_laptop_launch.sh; bash -n $F/u15b_thresholds.sh
/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python -m py_compile scan3d/scanA/scan_offline.py $F/u15b_thresholds.py
git add -- scan3d/scanA/scan_offline.py $F/README.md $F/u15_laptop.sh $F/u15_laptop_launch.sh $F/u15b_thresholds.py $F/u15b_thresholds.sh \
  $F/u15c_commit.sh scan3d/runs/laptop/20261112_121801_std60/fix_u15b/u15b_thresholds.json
git diff --cached --stat | tail -1
git commit -q -F - <<'MSG'
fixes 11-13 unit 15c: tone down the far LR-tolerance claim, small pieces at 3 thresholds, log code sha on laptop runs

- README 15b / scan_offline comment: tol 1.5 = less conservative, no sign of harm (not better); the < 5000 small-piece drop is a
  threshold effect (3 pieces join one 15 078-vertex component), worse at < 20 000; near p90 slightly worse; same inputs, not noise
- README 15 stale spots fixed (LR 1.5 was run, partner2 default history, 15b pointer); small-piece table at 500 / 5000 / 20000 (CPU)
- u15_laptop.sh / u15_laptop_launch.sh log the git state and the code file shas of each laptop run

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
MSG
git log --oneline -1
