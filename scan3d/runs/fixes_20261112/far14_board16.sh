#!/usr/bin/env bash
# fixes 11-12 unit 14: 165506 board GT again with the final default --far-min-z 1.6 (far14_job.sh ran it with 1.2), pytorch, same BA poses
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-fixes
cd "$WT"
export FFS_ROOT=/home/system/scanner/StereoDepthSeg PATH=/usr/local/cuda/bin:$PATH PYTHONUNBUFFERED=1
O=$WT/scan3d/runs/desktop/20261112_121801_std60/fix_u14far
LOGW=/mnt/c/Users/a3162/scan3d_handoff/review_20261112/interim_far.txt
note() { echo "$(date '+%m-%d %T') $*" | tee -a "$O/times.txt" >> "$LOGW"; }
while :; do
  busy=$(pgrep -f -a 'scan_offline.py|ffs_trt.py|mvsf_run|run_scanner.sh|kf_ba.py' | grep -v -e far14_ -e pgrep | wc -l)
  mem=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)
  [ "$busy" = 0 ] && [ "$mem" -lt 3000 ] && break
  echo "$(date +%T) GPU busy (procs $busy, mem $mem MiB), waiting" >> "$O/wait.log"; sleep 120
done
REC=/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261024_165506_std60
D=$WT/scan3d/runs/desktop/20261024_165506_std60
v=far16
t0=$SECONDS
SCANNER_COMMIT=$(git rev-parse HEAD)$(git diff --quiet || echo -dirty) bash scan3d/scanA/run_scanner.sh "$REC" "$D/fix_u14far_$v" --reuse "$D/fix_u13c_pt" --board auto \
  --depth-sigma ba --mesh-args "--tsdf-max-blocks 250000 --ffs-backend pytorch --far-pairs wide --far-baseline 0.35" > "$D/fix_u14far_$v.log" 2>&1; rc=$?
note "165506 board $v (--far-min-z default 1.6) rc $rc $((SECONDS - t0)) s, $(python3 -c "import json;s=json.load(open('$D/fix_u14far_$v/mesh/summary.json'));print('absdZ p90 pooled',round(s['pooled_absdZ_cm']['p90'],3),'kf p90 median',round(s['kf_board_absdZ_p90_cm_median'],3),'board z rms cm',round(s['mesh_board_z_rms_cm'],4),'vertices',s['mesh_vertices'],'min_z',s['far_pairs']['cfg']['min_z'],'far kf',s['far_pairs']['kf_with_partner'],'runtime',s['runtime_s'])" 2>&1)"
