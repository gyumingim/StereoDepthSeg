#!/usr/bin/env bash
# fixes 11-12 unit 13d, desktop RTX 5080 (run detached: setsid nohup bash trt13d_desk.sh > log &):
#  1) build export-v3 engines (ffs_trt.EXPORT_VERSION notes): static 544x960 + 960x544 (= /32 of the full
#     960x540 frame, same padding as PyTorch) + two dynamic engines (landscape, portrait) for the downscaled (s<1) pairs, then synthetic probes
#  2) 121801 (fix_u8 keyframes + BA poses), the 40 downscaled keyframes: PyTorch vs TRT v2, same machine, depth dumps compared
#  3) 165506 board GT: run_scanner --reuse fix_u13c_pt (same BA poses) with pytorch and with trt, both fresh here
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-fixes
cd "$WT"
export FFS_ROOT=/home/system/scanner/StereoDepthSeg PATH=/usr/local/cuda/bin:$PATH PYTHONUNBUFFERED=1
P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
TR=/home/system/scanner/ffs_trt
ENGD=${ENGD:-$TR/engines_v3}
O=$WT/scan3d/runs/desktop/20261112_121801_std60/fix_u13d
mkdir -p "$O"
gpumon() { nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 500 > "$1" 2>/dev/null & echo $!; }
peak() { awk -F', ' 'NR==1||$2>m{m=$2} END{print m}' "$1"; }
echo "$(date +%T) start, gpu $(nvidia-smi --query-gpu=name --format=csv,noheader)" | tee -a "$O/times.txt"
if [ ! -f "$ENGD/manifest.json" ]; then
  m=$(gpumon "$O/gpu_build.csv"); t0=$SECONDS
  $P scan3d/ffs_trt.py build --out "$ENGD" --sizes 544x960,960x544 --dynamic "128x128,384x672,544x960;128x128,672x384,960x544" > "$O/build.log" 2>&1 \
    || echo "build FAILED (build.log)" | tee -a "$O/times.txt"
  kill $m 2>/dev/null; echo "build $((SECONDS - t0)) s, GPU peak $(peak "$O/gpu_build.csv") MiB" | tee -a "$O/times.txt"
fi
[ -f "$ENGD/manifest.json" ] || exit 3
cp "$ENGD/manifest.json" "$O/engine_manifest.json"
: > "$O/probe.txt"
for s in "540 960" "960 540" "448 768" "448 800" "320 576" "247 440" "928 512"; do
  $P scan3d/runs/fixes_20261112/trt_dyn_probe.py "$ENGD" $s 2>&1 | grep -E "^TRT|rror" | head -2 | sed "s/^/probe: /" | tee -a "$O/probe.txt"
done
if grep -q -e "disp median nan" -e "FAIL" "$O/probe.txt"; then echo "probe has NaN / FAIL -> stop" | tee -a "$O/times.txt"; exit 4; fi

# 2) 40 downscaled keyframes of 121801
U8=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan
ARGS="--gt lensonly --vio $U8/ba/poses_ba.jsonl.gz --compress --kf-step 1 --tsdf-alloc dense --ffs-views 2 --ffs-fuse auto \
 --ffs-partners all --sigma-rot-deg 0.1 --sigma-b-rel 0.02 --tsdf-max-blocks 250000 --kf-only $WT/scan3d/runs/fixes_20261112/u13b_kf_s_lt1.json"
rm -rf /home/system/scanner/ffs_trt/dump13d_desk_pt /home/system/scanner/ffs_trt/dump13d_desk_trt
for v in pt trt; do
  extra=""; [ $v = trt ] && extra="--ffs-backend trt --ffs-engines $ENGD"
  m=$(gpumon "$O/gpu_kf40_$v.csv"); t0=$SECONDS
  $P scan3d/scanA/scan_offline.py "$U8/red/hs_20261112_121801_std60_kf" --out "$O/kf40_$v" $ARGS $extra \
    --dump-depth "$TR/dump13d_desk_$v" > "$O/kf40_$v.log" 2>&1 || echo "kf40 $v FAILED" | tee -a "$O/times.txt"
  kill $m 2>/dev/null; echo "kf40 $v $((SECONDS - t0)) s, GPU peak $(peak "$O/gpu_kf40_$v.csv") MiB" | tee -a "$O/times.txt"
  rm -f "$O/kf40_$v/mesh.obj" "$O/kf40_$v/mesh.ply"
done
$P scan3d/runs/fixes_20261112/trt_compare.py --a "$TR/dump13d_desk_pt" --b "$TR/dump13d_desk_trt" --sa "$O/kf40_pt/summary.json" \
  --sb "$O/kf40_trt/summary.json" --out "$O/cmp_kf40_pt_vs_trtv2.json" > "$O/cmp_kf40_pt_vs_trtv2.log" 2>&1 || echo "cmp failed" | tee -a "$O/times.txt"
$P - "$O/cmp_kf40_pt_vs_trtv2.json" <<'PY' | tee -a "$O/times.txt"
import json, sys
d = json.load(open(sys.argv[1]))
print("kf40 pt vs trt v2:", {k: d.get(k) for k in ("n_kf_eval", "n_kf_same_partners", "kf_absdz_med_mm_median", "kf_absdz_p95_mm_median",
      "kf_absdz_p95_mm_p90", "pooled_absdz_mm", "pooled_frac_absdz_gt_10mm", "kf_frac_p95_gt_10mm", "ffs_ms", "ffs_s_total")})
PY

# 3) 165506 board GT, same BA poses (fix_u13c_pt), fresh pytorch and trt mesh + board here
REC=/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261024_165506_std60
D=$WT/scan3d/runs/desktop/20261024_165506_std60
for v in pt trt; do
  ma="--tsdf-max-blocks 250000"; [ $v = trt ] && ma="$ma --ffs-backend trt --ffs-engines $ENGD"
  t0=$SECONDS
  SCANNER_COMMIT=$(git rev-parse HEAD)-dirty bash scan3d/scanA/run_scanner.sh "$REC" "$D/fix_u13d_$v" --reuse "$D/fix_u13c_pt" --board auto \
    --depth-sigma ba --mesh-args "$ma" > "$D/fix_u13d_$v.log" 2>&1; echo "board165506 $v rc $? $((SECONDS - t0)) s" | tee -a "$O/times.txt"
done
echo "$(date +%T) done" | tee -a "$O/times.txt"
