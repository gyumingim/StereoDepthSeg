#!/bin/bash
D=$(cd $(dirname $0) && pwd); cd $D
for k in xq8 xqfp xq16; do
  for rep in 1 2; do
  t=p_${k}_r$rep; a="model=$k.nnc in=gray13.bin out=kp_${k}_r$rep.bin pipe=1 passes=10"; n=0
  while :; do
    bash run1.sh $t "$a"; rc=$?
    [ $rc -eq 0 ] && grep -q "nncrun end" runs/$t.applog.txt && break
    [ $rc -eq 3 ] && { echo "LIMIT stop"; exit 3; }
    n=$((n+1)); [ $n -ge 40 ] && { echo "gate timeout"; exit 9; }
    sleep 60
  done
  echo "ITEM $t done"
  done
done
echo BATCH_DONE
