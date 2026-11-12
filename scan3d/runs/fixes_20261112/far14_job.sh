#!/usr/bin/env bash
# fixes 11-12 unit 14 (desktop RTX 5080): --far-pairs wide vs baseline, both --ffs-backend pytorch (explicit; host.env trt default not used).
#  1) 121801: fix_u8 keyframes + BA poses, fix_u8 mesh command (as trt13d_full.sh) but --tsdf-max-blocks 600000 for both runs
#     (baseline uses 154 606 blocks < 250 000, so the cap does not change it; the far run needs more blocks)
#  2) 165506 board GT: run_scanner --reuse fix_u13c_pt (same BA poses), baseline and --far-pairs wide
# Waits until no other GPU job runs (other agents share this GPU). Run detached: setsid nohup bash far14_job.sh > log 2>&1 &
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-fixes
cd "$WT"
export FFS_ROOT=/home/system/scanner/StereoDepthSeg PATH=/usr/local/cuda/bin:$PATH PYTHONUNBUFFERED=1
P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
O=$WT/scan3d/runs/desktop/20261112_121801_std60/fix_u14far
mkdir -p "$O"
LOGW=/mnt/c/Users/a3162/scan3d_handoff/review_20261112/interim_far.txt
note() { echo "$(date '+%m-%d %T') $*" | tee -a "$O/times.txt" >> "$LOGW"; }
gpumon() { nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 500 > "$1" 2>/dev/null & echo $!; }
peak() { awk -F', ' 'NR==1||$2>m{m=$2} END{print m}' "$1"; }
waitgpu() {   # other jobs: scan_offline / ffs_trt / mvs / run_scanner processes not ours, or GPU memory > 3 GB
  while :; do
    busy=$(pgrep -f -a 'scan_offline.py|ffs_trt.py|mvsf_run|run_scanner.sh|kf_ba.py' | grep -v -e far14_job -e pgrep | wc -l)
    mem=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)
    [ "$busy" = 0 ] && [ "$mem" -lt 3000 ] && break
    echo "$(date +%T) GPU busy (procs $busy, mem $mem MiB), waiting" >> "$O/wait.log"; sleep 120
  done
}
note "unit 14 job start, git $(git rev-parse --short HEAD)$(git diff --quiet || echo -dirty)"
U8=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan
ARGS="--gt lensonly --vio $U8/ba/poses_ba.jsonl.gz --compress --kf-step 1 --tsdf-alloc dense --ffs-views 2 --ffs-fuse auto \
 --ffs-partners all --sigma-rot-deg 0.1 --sigma-b-rel 0.02 --tsdf-max-blocks 600000 --ffs-backend pytorch"
for v in base far; do
  extra=""; [ $v = far ] && extra="--far-pairs wide --far-baseline 0.35"
  waitgpu
  m=$(gpumon "$O/gpu_$v.csv"); t0=$SECONDS
  $P scan3d/scanA/scan_offline.py "$U8/red/hs_20261112_121801_std60_kf" --out "$O/mesh_$v" $ARGS $extra > "$O/mesh_$v.log" 2>&1 \
    || note "121801 $v FAILED"
  kill $m 2>/dev/null; note "121801 $v $((SECONDS - t0)) s wall, GPU peak $(peak "$O/gpu_$v.csv") MiB, $(python3 -c "import json;s=json.load(open('$O/mesh_$v/summary.json'));print('vertices',s['mesh_vertices'],'runtime_s',s['runtime_s'],'blocks',s['tsdf_blocks'],'full',s['tsdf_full'],'far',s.get('far_pairs',{}).get('kf_with_partner'))" 2>&1)"
done
REC=/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261024_165506_std60
D=$WT/scan3d/runs/desktop/20261024_165506_std60
for v in base far; do
  ma="--tsdf-max-blocks 250000 --ffs-backend pytorch"; [ $v = far ] && ma="$ma --far-pairs wide --far-baseline 0.35"
  waitgpu
  t0=$SECONDS
  SCANNER_COMMIT=$(git rev-parse HEAD)$(git diff --quiet || echo -dirty) bash scan3d/scanA/run_scanner.sh "$REC" "$D/fix_u14far_$v" --reuse "$D/fix_u13c_pt" --board auto \
    --depth-sigma ba --mesh-args "$ma" > "$D/fix_u14far_$v.log" 2>&1; rc=$?
  note "165506 board $v rc $rc $((SECONDS - t0)) s, $(python3 -c "import json;s=json.load(open('$D/fix_u14far_$v/mesh/summary.json'));print('absdZ p90 pooled',round(s['pooled_absdZ_cm']['p90'],3),'kf p90 median',round(s['kf_board_absdZ_p90_cm_median'],3),'vertices',s['mesh_vertices'],'far',s.get('far_pairs',{}).get('kf_with_partner'))" 2>&1)"
done
note "unit 14 job done"
