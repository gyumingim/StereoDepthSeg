#!/usr/bin/env bash
# fixes 11-12 unit 14 follow-up: 121801 --far-pairs wide with --far-min-z 1.6 (wide pair never replaces near-pair pixels below 1.6 m)
# to see where the near-range (< 1.2 m camera depth) shift of the default far run (--far-min-z 1.2) comes from. Same command otherwise.
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-fixes
cd "$WT"
export FFS_ROOT=/home/system/scanner/StereoDepthSeg PATH=/usr/local/cuda/bin:$PATH PYTHONUNBUFFERED=1
P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
O=$WT/scan3d/runs/desktop/20261112_121801_std60/fix_u14far
LOGW=/mnt/c/Users/a3162/scan3d_handoff/review_20261112/interim_far.txt
note() { echo "$(date '+%m-%d %T') $*" | tee -a "$O/times.txt" >> "$LOGW"; }
while :; do
  busy=$(pgrep -f -a 'scan_offline.py|ffs_trt.py|mvsf_run|run_scanner.sh|kf_ba.py' | grep -v -e far14_ -e pgrep | wc -l)
  mem=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)
  [ "$busy" = 0 ] && [ "$mem" -lt 3000 ] && break
  echo "$(date +%T) GPU busy (procs $busy, mem $mem MiB), waiting" >> "$O/wait.log"; sleep 120
done
U8=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan
ARGS="--gt lensonly --vio $U8/ba/poses_ba.jsonl.gz --compress --kf-step 1 --tsdf-alloc dense --ffs-views 2 --ffs-fuse auto \
 --ffs-partners all --sigma-rot-deg 0.1 --sigma-b-rel 0.02 --tsdf-max-blocks 600000 --ffs-backend pytorch --far-pairs wide --far-baseline 0.35 --far-min-z 1.6"
t0=$SECONDS
$P scan3d/scanA/scan_offline.py "$U8/red/hs_20261112_121801_std60_kf" --out "$O/mesh_far_minz16" $ARGS > "$O/mesh_far_minz16.log" 2>&1 || note "121801 far minz1.6 FAILED"
note "121801 far --far-min-z 1.6: $((SECONDS - t0)) s wall"
$P scan3d/runs/fixes_20261112/far_eval.py "$O/mesh_base" "$O/mesh_far_minz16" "$U8/red/hs_20261112_121801_std60_kf" "$O/u14_121801_minz16" > "$O/u14_121801_minz16_eval.log" 2>&1 \
  || note "eval minz16 FAILED"
note "121801 far minz1.6 eval: $(grep '^near' "$O/u14_121801_minz16_eval.log" | cut -c1-400)"
