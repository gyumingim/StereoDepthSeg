#!/usr/bin/env bash
# fixes 11-12 unit 13d (laptop or desktop job): dynamic-shape TRT engine with InstanceNorm DECOMPOSED in the ONNX (mean/var/sqrt,
# kept fp32; FFS_TRT_DECOMP_IN=1) -> exact /32 padding like lib_ffs. Probe sizes (synthetic 12 px shift), then the 40 downscaled
# keyframes of 121801 (fix_u8 keyframes + BA poses) vs the u13 full PyTorch dump. Env: WT O P [ENGD DYN SIZES NO_KF].
set -uo pipefail
cd "$WT"; mkdir -p "$O"
export PYTHONUNBUFFERED=1
TR=/home/system/scanner/ffs_trt
ENGD=${ENGD:-$TR/engines_dyn3}
DYN=${DYN:-256x256,544x960,960x960}
SIZES=${SIZES:-}
U8=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan
IN=$U8/red/hs_20261112_121801_std60_kf
POSES=$U8/ba/poses_ba.jsonl.gz
KF=$WT/scan3d/runs/fixes_20261112/u13b_kf_s_lt1.json
S13=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u11trt
gpumon() { nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 500 > "$1" 2>/dev/null & echo $!; }
peak() { awk -F', ' 'NR==1||$2>m{m=$2} END{print m}' "$1"; }
nvidia-smi --query-gpu=name --format=csv,noheader | sed "s/^/gpu: /" | tee -a "$O/times.txt"
if [ ! -f "$ENGD/manifest.json" ]; then
  m=$(gpumon "$O/gpu_build.csv"); t0=$SECONDS
  FFS_TRT_DECOMP_IN=1 $P scan3d/ffs_trt.py build --out "$ENGD" --dynamic "$DYN" ${SIZES:+--sizes $SIZES} > "$O/build.log" 2>&1 \
    || echo "build FAILED (build.log)" | tee -a "$O/times.txt"
  kill $m 2>/dev/null; echo "build $((SECONDS - t0)) s, GPU peak $(peak "$O/gpu_build.csv") MiB" | tee -a "$O/times.txt"
fi
[ -f "$ENGD/manifest.json" ] || exit 3
cp "$ENGD/manifest.json" "$O/engine_manifest.json"
for s in "540 960" "448 768" "448 800" "320 576" "247 440" "928 512"; do
  $P scan3d/runs/fixes_20261112/trt_dyn_probe.py "$ENGD" $s 2>&1 | grep -E "^TRT|rror" | head -2 | sed "s/^/probe: /" | tee -a "$O/probe.txt"
done
if grep -q -e "disp median nan" -e "FAIL" "$O/probe.txt"; then echo "probe has NaN / FAIL -> stop" | tee -a "$O/times.txt"; exit 4; fi
[ -n "${NO_KF:-}" ] && exit 0
rm -rf /home/system/scanner/ffs_trt/dump13d_trt
m=$(gpumon "$O/gpu_trt.csv"); t0=$SECONDS
$P scan3d/scanA/scan_offline.py "$IN" --out "$O/mesh_trt" --gt lensonly --vio "$POSES" --compress --kf-step 1 --tsdf-alloc dense \
  --ffs-views 2 --ffs-fuse auto --ffs-partners all --sigma-rot-deg 0.1 --sigma-b-rel 0.02 --tsdf-max-blocks 250000 --kf-only "$KF" \
  --ffs-backend trt --ffs-engines "$ENGD" --dump-depth /home/system/scanner/ffs_trt/dump13d_trt > "$O/mesh_trt.log" 2>&1 \
  || echo "trt run FAILED" | tee -a "$O/times.txt"
kill $m 2>/dev/null; echo "trt 40kf $((SECONDS - t0)) s, GPU peak $(peak "$O/gpu_trt.csv") MiB" | tee -a "$O/times.txt"
rm -f "$O/mesh_trt/mesh.obj" "$O/mesh_trt/mesh.ply"
$P scan3d/runs/fixes_20261112/trt_compare.py --a /home/system/scanner/ffs_trt/dump_pt --b /home/system/scanner/ffs_trt/dump13d_trt \
  --sa "$S13/mesh_pt/summary.json" --sb "$O/mesh_trt/summary.json" --out "$O/cmp_ptfull_vs_trt_dyn3.json" > "$O/cmp_ptfull_vs_trt_dyn3.log" 2>&1 \
  || echo "cmp failed" | tee -a "$O/times.txt"
$P - "$O/cmp_ptfull_vs_trt_dyn3.json" <<'PY' | tee -a "$O/times.txt"
import json, sys
d = json.load(open(sys.argv[1]))
print({k: d.get(k) for k in ("n_kf_eval", "kf_absdz_med_mm_median", "kf_absdz_p95_mm_median", "kf_absdz_p95_mm_p90",
                             "pooled_absdz_mm", "pooled_frac_absdz_gt_10mm", "kf_frac_p95_gt_10mm", "ffs_ms", "ffs_s_total")})
PY
