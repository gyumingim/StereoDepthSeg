#!/usr/bin/env bash
# fixes 11-13 unit 15 (desktop RTX 5080, TRT for every run): sigma / FFS downscale + wide-only pixel guard.
# Usage: bash u15_job.sh board TAG "extra scan_offline args"   -> 165506 run_scanner --reuse fix_u13c_pt (same BA poses)
#        bash u15_job.sh office TAG "extra scan_offline args"  -> 121801 fix_u8 keyframes + BA poses, fix_u8 mesh command
# One GPU job at a time: waits until no other scan / FFS process runs and the GPU holds < 3 GB.
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-fixes
cd "$WT"
export FFS_ROOT=/home/system/scanner/StereoDepthSeg PATH=/usr/local/cuda/bin:$PATH PYTHONUNBUFFERED=1
P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
LOGW=/mnt/c/Users/a3162/scan3d_handoff/review_20261112/interim_u15.txt
what=$1; tag=$2; extra=${3:-}
note() { echo "$(date '+%m-%d %T') $*" | tee -a "$WT/scan3d/runs/fixes_20261112/u15_times.txt" >> "$LOGW"; }
gpumon() { nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 500 > "$1" 2>/dev/null & echo $!; }
peak() { awk -F', ' 'NR==1||$2>m{m=$2} END{print m}' "$1"; }
while :; do
  busy=$(pgrep -f -a 'scan_offline.py|ffs_trt.py|mvsf_run|run_scanner.sh|kf_ba.py' | grep -v -e u15_job -e pgrep | wc -l)
  mem=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)
  [ "$busy" = 0 ] && [ "$mem" -lt 3000 ] && break
  echo "$(date +%T) GPU busy (procs $busy, mem $mem MiB), waiting" >> "$WT/scan3d/runs/fixes_20261112/u15_wait.log"; sleep 60
done
GIT="$(git rev-parse --short HEAD)$(git diff --quiet -- scan3d/scanA/scan_offline.py scan3d/motion_stereo.py || echo -dirty)"
note "start $what $tag ($extra) git $GIT, GPU mem $(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1) MiB"
t0=$SECONDS
if [ "$what" = board ]; then
  REC=/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261024_165506_std60
  D=$WT/scan3d/runs/desktop/20261024_165506_std60
  m=$(gpumon "$D/u15_gpu_$tag.csv")
  SCANNER_COMMIT=$(git rev-parse HEAD)$(git diff --quiet || echo -dirty) bash scan3d/scanA/run_scanner.sh "$REC" "$D/fix_u15_$tag" --reuse "$D/fix_u13c_pt" \
    --board auto --depth-sigma ba --mesh-args "--tsdf-max-blocks 250000 --ffs-backend trt $extra" > "$D/fix_u15_$tag.log" 2>&1; rc=$?
  kill $m 2>/dev/null
  note "board $tag rc $rc $((SECONDS - t0)) s, GPU peak $(peak "$D/u15_gpu_$tag.csv") MiB, $(python3 -c "
import json;s=json.load(open('$D/fix_u15_$tag/mesh/summary.json'));b=json.load(open('$D/fix_u15_$tag/mesh_board/summary.json'))
print('mesh absdZ p90 pooled',round(s['pooled_absdZ_cm']['p90'],4),'kf p90 med',round(s['kf_board_absdZ_p90_cm_median'],4),
 'mesh_board p90',round(b['pooled_absdZ_cm']['p90'],4),'board z rms',round(s.get('mesh_board_z_rms_cm',-1),4),'cover',round(s.get('mesh_board_cover_frac',-1),3),
 'vertices',s['mesh_vertices'],'backend',s['ffs_backend'],'sigma',s['depth_sigma'])" 2>&1)"
else
  U8=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan
  O=$WT/scan3d/runs/desktop/20261112_121801_std60/fix_u15
  mkdir -p "$O"
  $P scan3d/ffs_trt.py check > "$O/check_$tag.log" 2>&1 || { note "office $tag: ffs_trt check FAILED"; exit 3; }
  ARGS="--gt lensonly --vio $U8/ba/poses_ba.jsonl.gz --compress --kf-step 1 --tsdf-alloc dense --ffs-views 2 --ffs-fuse auto \
 --ffs-partners all --sigma-rot-deg 0.1 --sigma-b-rel 0.02 --tsdf-max-blocks 600000 --ffs-backend trt"
  m=$(gpumon "$O/gpu_$tag.csv")
  $P scan3d/scanA/scan_offline.py "$U8/red/hs_20261112_121801_std60_kf" --out "$O/mesh_$tag" $ARGS $extra > "$O/mesh_$tag.log" 2>&1; rc=$?
  kill $m 2>/dev/null
  note "office $tag rc $rc $((SECONDS - t0)) s wall, GPU peak $(peak "$O/gpu_$tag.csv") MiB, $(python3 -c "
import json;s=json.load(open('$O/mesh_$tag/summary.json'));fp=s.get('far_pairs') or {}
print('vertices',s['mesh_vertices'],'runtime_s',s['runtime_s'],'ffs_s',s['ffs_s_total'],'blocks',s['tsdf_blocks'],'full',s['tsdf_full'],'backend',s['ffs_backend'],
 'sigma',s['depth_sigma'],'far',{k:fp.get(k) for k in ('kf_with_partner','px_added','px_replaced','px_wide_only_cand','px_wide_only_rejected','px_rej_lr','px_rej_sat','px_rej_p2','px_repl_below_min_z','kf_partner2','kf_no_partner2','ffs_s','ffs2_s')} if fp else None)" 2>&1)"
fi
