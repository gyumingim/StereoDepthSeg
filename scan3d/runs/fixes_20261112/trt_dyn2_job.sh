#!/usr/bin/env bash
# unit 13b try 2: dynamic-shape engine with InstanceNorm as TRT plugin (native one gave NaN / CUDA 720), then probe sizes;
# static 544x960 engine probed as a sanity reference. Env: WT O P (ENGD, DYN optional).
set -uo pipefail
cd "$WT"; mkdir -p "$O"
ENGD=${ENGD:-/home/system/scanner/ffs_trt/engines_dyn2}
DYN=${DYN:-256x256,448x768,960x960}
if [ ! -f "$ENGD/manifest.json" ]; then
  t0=$SECONDS
  FFS_TRT_PLUGIN_INORM=1 $P scan3d/ffs_trt.py build --out "$ENGD" --dynamic "$DYN" > "$O/build_dyn2.log" 2>&1 || echo "build FAILED" >> "$O/probe.txt"
  echo "build_dyn2 $((SECONDS - t0)) s" >> "$O/probe.txt"
fi
[ -f "$ENGD/manifest.json" ] && cp "$ENGD/manifest.json" "$O/engine_dyn2_manifest.json"
$P scan3d/runs/fixes_20261112/trt_dyn_probe.py /home/system/scanner/ffs_trt/engines 540 960 2>&1 | grep -E "^TRT|rror" | head -2 | sed "s/^/static: /" >> "$O/probe.txt"
for s in "448 768" "448 800" "320 576" "928 512" "540 960"; do
  $P scan3d/runs/fixes_20261112/trt_dyn_probe.py "$ENGD" $s 2>&1 | grep -E "^TRT|rror" | head -2 | sed "s/^/dyn2: /" >> "$O/probe.txt"
done
cat "$O/probe.txt"
