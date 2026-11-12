#!/usr/bin/env bash
# fixes 11-12 unit 13d, desktop RTX 5080: whole 121801 (fix_u8 keyframes + BA poses, all 351 keyframes, fix_u8 mesh command)
# PyTorch vs TRT export v3, depth dumps compared (all keyframes + the s<1 subset). Run detached (setsid nohup).
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-fixes
cd "$WT"
export FFS_ROOT=/home/system/scanner/StereoDepthSeg PATH=/usr/local/cuda/bin:$PATH PYTHONUNBUFFERED=1
P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
TR=/home/system/scanner/ffs_trt
O=$WT/scan3d/runs/desktop/20261112_121801_std60/fix_u13d_full
mkdir -p "$O"
U8=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan
ARGS="--gt lensonly --vio $U8/ba/poses_ba.jsonl.gz --compress --kf-step 1 --tsdf-alloc dense --ffs-views 2 --ffs-fuse auto \
 --ffs-partners all --sigma-rot-deg 0.1 --sigma-b-rel 0.02 --tsdf-max-blocks 250000"
gpumon() { nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 500 > "$1" 2>/dev/null & echo $!; }
peak() { awk -F', ' 'NR==1||$2>m{m=$2} END{print m}' "$1"; }
echo "$(date +%T) start" | tee -a "$O/times.txt"
rm -rf /home/system/scanner/ffs_trt/dump13d_full_pt /home/system/scanner/ffs_trt/dump13d_full_trt
for v in pt trt; do
  extra=""; [ $v = trt ] && extra="--ffs-backend trt --ffs-engines $TR/engines_v3"
  m=$(gpumon "$O/gpu_$v.csv"); t0=$SECONDS
  $P scan3d/scanA/scan_offline.py "$U8/red/hs_20261112_121801_std60_kf" --out "$O/mesh_$v" $ARGS $extra \
    --dump-depth "$TR/dump13d_full_$v" > "$O/mesh_$v.log" 2>&1 || echo "$v FAILED" | tee -a "$O/times.txt"
  kill $m 2>/dev/null; echo "$v $((SECONDS - t0)) s, GPU peak $(peak "$O/gpu_$v.csv") MiB" | tee -a "$O/times.txt"
  rm -f "$O/mesh_$v/mesh.obj" "$O/mesh_$v/mesh.ply"
done
$P scan3d/runs/fixes_20261112/trt_compare.py --a "$TR/dump13d_full_pt" --b "$TR/dump13d_full_trt" --sa "$O/mesh_pt/summary.json" \
  --sb "$O/mesh_trt/summary.json" --out "$O/cmp_pt_vs_trt.json" > "$O/cmp_pt_vs_trt.log" 2>&1 || echo "cmp failed" | tee -a "$O/times.txt"
$P - "$O/cmp_pt_vs_trt.json" "$WT/scan3d/runs/fixes_20261112/u13b_kf_s_lt1.json" <<'PY' | tee -a "$O/times.txt"
import json, sys
import numpy as np
d = json.load(open(sys.argv[1])); sub = set(json.load(open(sys.argv[2])))
print("full:", {k: d.get(k) for k in ("n_kf_eval", "n_kf_same_partners", "kf_absdz_med_mm_median", "kf_absdz_p95_mm_median",
      "kf_absdz_p95_mm_p90", "pooled_absdz_mm", "kf_frac_p95_gt_10mm", "mesh_vertices", "ffs_s_total", "runtime_s")})
per = d.get("per_kf") or []
for name, sel in (("s=1", [r for r in per if r.get("kf") not in sub]), ("s<1", [r for r in per if r.get("kf") in sub])):
    p95 = [r["absdz_p95_mm"] for r in sel if r.get("absdz_p95_mm") is not None]
    if p95:
        print(name, "n", len(p95), "p95 median %.1f mm, p90 of p95 %.1f mm, frac p95>10mm %.2f" % (np.median(p95), np.percentile(p95, 90), np.mean(np.array(p95) > 10)))
PY
echo "$(date +%T) done" | tee -a "$O/times.txt"
