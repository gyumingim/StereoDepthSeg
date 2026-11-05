#!/bin/bash
# viba: VI-BA stage, the 4 variants of one recording in parallel (CPU), recordings as soon as their kf_ba dump is done.
#   bash viba_stage.sh REC_SHORT ...      (skips a variant whose viba.json exists)
# RAM guard: stop MY viba.py jobs when MemAvailable < 10 GB (scanA's own watchdog acts at 8 GB).
cd "$(dirname "$0")/../../.."
PV=$HOME/scanner/viba_work/venv/bin/python; D=scan3d/runs/viba_20261105; BL=$HOME/scanner/build_livepose
( while sleep 5; do a=$(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo)
    if [ "$a" -lt 10 ]; then echo "WATCHDOG $(date +%T) MemAvailable ${a} GB -> kill my viba.py"; pkill -f "[v]iba_work/venv/bin/python scan3d/viba/viba.py"; fi; done ) &
WD=$!
trap "kill $WD 2>/dev/null" EXIT
todo="$*"
while [ -n "$todo" ]; do
  left=""
  for r in $todo; do
    if [ -f $D/ba/$r/viba_input.npz ] && ! pgrep -f "[k]f_ba.py scan3d/recordings/hs_$r " > /dev/null; then
      if [[ $r == *165506 ]]; then R=scan3d/recordings/hs_${r}_std60; CFG=$BL/cfg_gen/c4f60; else R=scan3d/recordings/hs_$r; CFG=$BL/cfg_gen_hs222/c4f60; fi
      for v in "ba_g:--vision-only" "viba:--calib yaml" "viba_on:--calib online" "viba_x103:--calib yaml --init-scale 1.03"; do
        n=${v%%:*}; args=${v#*:}; mkdir -p $D/viba/$r/$n
        [ -f $D/viba/$r/$n/viba.json ] && continue
        ( $PV scan3d/viba/viba.py $R --dump $D/ba/$r/viba_input.npz --vio $D/vio/$r.poses.jsonl.gz --cfg $CFG --out $D/viba/$r/$n $args \
            > $D/viba/$r/$n.log 2>&1; echo "$r $n rc $? $(date +%T) $(grep -E '^\[lm\]' $D/viba/$r/$n.log)" ) &
      done
      wait $(jobs -p | grep -v "^$WD$")
    else left="$left $r"; fi
  done
  todo=$(echo $left); [ -n "$todo" ] && sleep 30
done
echo STAGEDONE $(date +%T)
