#!/usr/bin/env bash
# fixes 11-13 unit 15: list the unit's small result files for an explicit-path commit (no ply / obj / npz / spill / _prev)
cd /home/system/scanner/StereoDepthSeg-fixes
for d in scan3d/runs/laptop/20261024_165506_std60/fix_u15 scan3d/runs/laptop/20261112_121801_std60/fix_u15 \
         scan3d/runs/laptop/20261112_121801_std60/fix_u15far scan3d/runs/desktop/20261024_165506_std60/fix_u15_base \
         scan3d/runs/desktop/20261024_165506_std60/fix_u15_sig scan3d/runs/desktop/20261112_121801_std60/fix_u15; do
  find "$d" -type f ! -name '*.ply' ! -name '*.obj' ! -name '*.npz' ! -path '*tsdf_spill*' ! -path '*/_prev/*' -size -5M
done
for f in scan3d/runs/desktop/20261024_165506_std60/fix_u15_base.log scan3d/runs/desktop/20261024_165506_std60/fix_u15_sig.log \
         scan3d/runs/desktop/20261024_165506_std60/u15_gpu_base.csv scan3d/runs/desktop/20261024_165506_std60/u15_gpu_sig.csv; do
  [ -f "$f" ] && echo "$f"
done
