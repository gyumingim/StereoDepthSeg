#!/usr/bin/env bash
# rec960 2026-11-16 (B) desktop side: wait until the laptop answers (it was unreachable at 10:44), then ONE remote_laptop.sh job that
# runs b_job.sh there. Compute only on the laptop (user 11-13). Progress -> C:\Users\a3162\scan3d_handoff\live_test\progress_960.txt.
#   setsid -f bash scan3d/runs/rec960_20261116/b_launch.sh > ~/scanner/rec960_b_launch.log 2>&1 < /dev/null
# WAIT_H (default 12): give up after that many hours of no laptop. Never kill a running remote_laptop.sh poller (its EXIT trap).
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-rec960
PROG=/mnt/c/Users/a3162/scan3d_handoff/live_test/progress_960.txt
log() { echo "$(date '+%F %T') [B] $*" | tee -a "$PROG"; }
cd "$WT/scan3d"
SC=$(git -C "$WT" rev-parse HEAD)$(git -C "$WT" diff --quiet HEAD -- scan3d || echo -dirty)
t0=$SECONDS; n=0
while :; do
  st=$(timeout 60 tools/remote_laptop.sh status 2>&1 | head -3)
  case "$st" in *unreachable*) ;; *"lock free"*) break;; *) log "laptop answers but busy: $(echo "$st" | head -1)";; esac
  n=$((n + 1)); [ $((n % 6)) = 1 ] && log "waiting for the laptop ($(( (SECONDS - t0) / 60 )) min): $(echo "$st" | head -1)"
  [ $((SECONDS - t0)) -gt $(( ${WAIT_H:-12} * 3600 )) ] && { log "gave up: laptop not free for ${WAIT_H:-12} h"; exit 3; }
  sleep 300
done
log "laptop free -> job 165506 rec960 (code $SC)"
WT=$WT tools/remote_laptop.sh job 165506 "cmd:SC=$SC bash scan3d/runs/rec960_20261116/b_job.sh \$R \$WT/\$O \$P \$CACHE" --tag rec960
rc=$?
O=$WT/scan3d/runs/laptop/20261024_165506_std60/rec960
log "job rc $rc, results $O"
[ -f "$O/b_table.md" ] && cat "$O/b_table.md" >> "$PROG"
mkdir -p /mnt/c/Users/a3162/scan3d_handoff/live_test/rec960_B
cp "$O"/b_summary.json "$O"/b_table.md "$O"/mesh_diff.png "$O"/b_progress.txt /mnt/c/Users/a3162/scan3d_handoff/live_test/rec960_B/ 2>/dev/null
log "copied summary to C:\\Users\\a3162\\scan3d_handoff\\live_test\\rec960_B (if made)"
