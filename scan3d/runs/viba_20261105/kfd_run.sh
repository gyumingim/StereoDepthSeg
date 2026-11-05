#!/bin/bash
# viba README 2.5: kf_ba BA from the dump, one recording at a time (nice 10), as soon as the dump exists and no viba.py of it runs
cd "$(dirname "$0")/../../.."; P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python; D=scan3d/runs/viba_20261105
todo="$*"
while [ -n "$todo" ]; do left=""
  for r in $todo; do
    if [ -f $D/ba/$r/viba_input.npz ] && ! pgrep -f "[k]f_ba.py scan3d/recordings/hs_$r " >/dev/null && [ $(awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo) -ge 14 ]; then
      nice -n 10 $P scan3d/viba/kfba_from_dump.py $D/ba/$r > $D/ba/$r.from_dump.out 2>&1; echo "$r kfd rc $? $(date +%T) $(grep sim3 $D/ba/$r/ba_from_dump.log)"
    else left="$left $r"; fi
  done; todo=$(echo $left); [ -n "$todo" ] && sleep 60
done; echo KFDDONE $(date +%T)
