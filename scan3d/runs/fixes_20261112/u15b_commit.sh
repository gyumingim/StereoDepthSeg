#!/usr/bin/env bash
# fixes 11-13 unit 15b: append README part, commit only this unit's files by explicit path (no git add -A, no push)
set -e
cd /home/system/scanner/StereoDepthSeg-fixes
F=scan3d/runs/fixes_20261112
if [ -f $F/u15b_readme_part.md ]; then cat $F/u15b_readme_part.md >> $F/README.md; rm $F/u15b_readme_part.md; fi
L=scan3d/runs/laptop/20261112_121801_std60
files=(scan3d/scanA/scan_offline.py $F/README.md $F/u15_laptop.sh $F/u15_far.sh
  $F/u15b_laptop.sh $F/u15b_p2_geom.py $F/u15b_where_show.py $F/u15b_commit.sh $L/fix_u15far/u15b_p2_geom.json
  $F/u15_laptop_logs/office.log $F/u15_laptop_logs/launch.txt)
mapfile -t res < <(find $L/fix_u15b -type f ! -name '*.ply' ! -name '*.obj' ! -name '*.npz' ! -path '*tsdf_spill*' -size -5M)
git add -- "${files[@]}" "${res[@]}"
git diff --cached --stat | tail -2
git commit -q -F - <<'MSG'
fixes 11-13 unit 15b: far guard default lr+sat with LR tolerance 1.5, partner2 off (audit u15_verify.md)

- partner2 now needs a baseline direction >= --far-p2-min-dir-deg (20) from the 1st wide partner; keyframes without one keep
  lr/sat (unit 15 took the next candidate, ~1.7 deg away, and dropped them). Found for 30/240 keyframes, no gain -> default off
- --far-lr-tol default 1.5 (laptop PyTorch 121801: small pieces < 5000 v 137 070 -> 127 735, > 1.6 m vertices 63 173 -> 69 006)
- sigma-on vs off on the far path confirmed (> 1.6 m 106 005 -> 93 318); README 15 corrections (partner2 off explicit in
  scripts, 0.83 mm = desktop TRT rebuild noise). 1 run each, unverified

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
MSG
git log --oneline -1
