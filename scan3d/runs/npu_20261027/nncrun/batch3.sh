#!/bin/bash
D=$(cd $(dirname $0) && pwd); cd $D
items=("u_c5_gold|model=c5.nnc in=g_c5.bin out=og_c5.bin warm=5 iters=10 reps=1"
       "u_c5_eval|model=c5.nnc in=in_c3.bin out=oe_c5.bin warm=50 iters=200 reps=2")
for it in "${items[@]}"; do
  t=${it%%|*}; a=${it#*|}; n=0
  while :; do
    bash run1.sh $t "$a"; rc=$?
    [ $rc -eq 0 ] && grep -q "nncrun end" runs/$t.applog.txt && break
    [ $rc -eq 3 ] && { echo "LIMIT stop"; exit 3; }
    n=$((n+1)); [ $n -ge 40 ] && { echo "gate timeout"; exit 9; }
    sleep 60
  done
  echo "ITEM $t done"
done
echo BATCH_DONE
