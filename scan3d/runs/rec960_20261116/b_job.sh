#!/usr/bin/env bash
# rec960 2026-11-16 (B) — runs ON THE LAPTOP inside one remote_laptop.sh job (GPU lock held for the whole job):
#   tools/remote_laptop.sh job 165506 'cmd:bash scan3d/runs/rec960_20261116/b_job.sh $R $WT/$O $P $CACHE' --tag rec960
# 1. make the 960x540 emulation of REC (make960.py: INTER_AREA = 2x2 box, FFV1 lossless) next to it (REC_d960, deleted at the end)
# 2. VIO lp_replay.py c4f60 (live config, ZUPT off = base_nozupt line, lever arm, 960x540 remap, 15 clones), identical settings,
#    order 1920 r1, 960 r1, 1920 r2, 960 r2 (interleaved: thermal / background drift hits both)
# 3. scanner run_scanner.sh defaults on both (1920: keyframes 1920 JPEG -> 960 compute; 960: keyframes 960 JPEG)
# 4. b_summary.py -> OUT/b_summary.json, b_table.md, mesh_diff.png
set -uo pipefail
R=$1; OUT=$2; P=$3; CACHE=$4
WT=$(cd "$(dirname "$0")/../../.." && pwd)
B=/home/system/scanner/build_livepose
R9=${R}_d960
mkdir -p "$OUT"
log() { echo "$(date '+%F %T') $*" | tee -a "$OUT/b_progress.txt"; }
log "B start on $(hostname) WT $WT REC $R"
nproc > "$OUT/host.txt"; nvidia-smi --query-gpu=name,driver_version --format=csv,noheader >> "$OUT/host.txt" 2>&1
if [ ! -d "$R9" ]; then
  /usr/bin/time -f "TIME make960 %e s" $P "$WT/scan3d/runs/rec960_20261116/make960.py" "$R" "$R9" > "$OUT/make960.json" 2> "$OUT/make960.err" \
    || { log "make960 FAILED: $(tail -n 3 "$OUT/make960.err")"; exit 1; }
fi
log "make960 done: $(tail -n 1 "$OUT/make960.err") $(du -sh "$R9/cam5.mp4" | cut -f1)"
for k in 1 2; do for res in 1920 960; do
  rr=$R; [ $res = 960 ] && rr=$R9
  d=$OUT/vio_${res}_r$k; mkdir -p "$d"
  ( cd "$WT/scan3d/runs/livepose_20261030" && /usr/bin/time -f "TIME vio %e s maxrss %M kB" $P lp_replay.py "$rr" $B/cfg_gen/c4f60 "$d/B_o0" \
      --bridge $B/bridge/ov_bridge --vocab "$WT/scan3d/ov_bridge/loop/voc_k10L4.voc" > "$d/vio.log" 2>&1 )
  log "vio $res r$k rc $?: $(grep -m1 '^{' "$d/vio.log" | head -c 400)"
done; done
CM=${SC:-unknown}            # commit of the pushed worktree (remote_laptop.sh pushes no .git), given by b_launch.sh
for res in 1920 960; do
  rr=$R; [ $res = 960 ] && rr=$R9
  SCANNER_COMMIT=$CM bash "$WT/scan3d/scanA/run_scanner.sh" "$rr" "$OUT/scan_$res" --cache "$CACHE/scan_$res" > "$OUT/scan_$res.log" 2>&1
  log "scan $res rc $?: $(tail -n 2 "$OUT/scan_$res.log" | tr '\n' ' ' | head -c 300)"
done
$P "$WT/scan3d/runs/rec960_20261116/b_summary.py" "$OUT" > "$OUT/b_summary.log" 2>&1; log "summary rc $?"
cat "$OUT/b_table.md" >> "$OUT/b_progress.txt" 2>/dev/null
rm -rf "$R9"; log "B done (removed $R9)"
