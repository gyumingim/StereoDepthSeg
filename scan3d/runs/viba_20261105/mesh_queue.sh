#!/bin/bash
# viba M3: depth+TSDF for each H recording once both kf_ba (poses_ba) and VI-BA poses exist; one at a time (GPU lock inside)
cd "$(dirname "$0")"
todo="$*"
while [ -n "$todo" ]; do left=""
  for r in $todo; do
    if [ -f ba/$r/poses_ba.jsonl.gz ] && [ -f viba/$r/viba/viba.json ]; then bash viba_run.sh mesh $r; else left="$left $r"; fi
  done; todo=$(echo $left); [ -n "$todo" ] && sleep 60
done; echo MESHQDONE $(date +%T)
