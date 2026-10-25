#!/bin/bash
# progress every 10 min + any FAILED / finished marker
n0=0
while true; do
  f=$(grep -c FAILED /tmp/ovacc/round1.log /tmp/ovacc/round*.log 2>/dev/null | awk -F: '{s+=$2} END {print s+0}')
  d=$(cat /tmp/ovacc/round*.log 2>/dev/null | grep -c ' done ')
  last=$(tail -1 /tmp/ovacc/round1.log)
  echo "progress done=$d failed=$f load=$(cut -d' ' -f1 /proc/loadavg) last: ${last:0:80}"
  grep -h 'DONE' /tmp/ovacc/round*.log 2>/dev/null | tr '\n' ' '
  sleep 600
done
