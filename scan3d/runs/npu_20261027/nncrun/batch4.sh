#!/bin/bash
D=$(cd $(dirname $0) && pwd); cd $D
items=()
for k in xq8 xq16 xqfp; do
  items+=("r_${k}_gold|model=$k.nnc in=g_$k.bin out=og_$k.bin warm=5 iters=10 reps=1 msg=1")
  items+=("r_${k}_eval|model=$k.nnc in=eval3ch.bin out=oe_$k.bin warm=50 iters=200 reps=2")
done
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
