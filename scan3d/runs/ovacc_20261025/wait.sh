#!/bin/bash
# wait.sh COND N MAXSEC: wait until COND has N result files (or MAXSEC), then print counts of all dirs
c=$1; n=$2; m=${3:-580}
t0=$(date +%s)
while [ $(ls /tmp/ovacc/ate/$c/*.json 2>/dev/null | wc -l) -lt $n ] && [ $(( $(date +%s) - t0 )) -lt $m ]; do sleep 10; done
for d in /tmp/ovacc/ate/*/; do b=$(basename $d); [ "$b" = bin ] || [ "$b" = tmp ] || [ "$b" = score_tmp ] || echo -n "$b:$(ls $d/*.json 2>/dev/null | wc -l) "; done
echo; grep -c FAILED /tmp/ovacc/round1.log; uptime
