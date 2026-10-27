#!/bin/bash
# waits for the start gate (max 40 min) before each item; stop at limits
D=$(cd $(dirname $0) && pwd); cd $D
items=("u_c3_gold|model=c3.nnc in=g_c3.bin out=og_c3.bin warm=5 iters=10 reps=1"
       "u_c3_eval|model=c3.nnc in=in_c3.bin out=oe_c3.bin warm=50 iters=200 reps=2"
       "u_c4_gold|model=c4.nnc in=g_c4.bin out=og_c4.bin warm=5 iters=10 reps=1"
       "u_c4_eval|model=c4.nnc in=in_c4.bin out=oe_c4.bin warm=50 iters=200 reps=2"
       "u_a3_gold|model=a3.nnc in=g_a3.bin out=og_a3.bin warm=5 iters=10 reps=1"
       "u_a3_eval|model=a3.nnc in=in_a3.bin out=oe_a3.bin warm=50 iters=200 reps=2")
for it in "${items[@]}"; do
  t=${it%%|*}; a=${it#*|}; n=0
  while :; do
    bash run1.sh $t "$a"; rc=$?
    [ $rc -eq 0 ] && break
    [ $rc -eq 3 ] && { echo "LIMIT stop"; exit 3; }
    n=$((n+1)); [ $n -ge 40 ] && { echo "gate timeout"; exit 9; }
    sleep 60
  done
  echo "ITEM $t done"
done
echo BATCH_DONE
