#!/bin/bash
D=$(cd $(dirname $0) && pwd); cd $D
items=()
for k in xfpmc xfpo16; do
  for rep in r1 r2; do
    items+=("v_${k}_p2t1s_$rep|model=$k.nnc in=gray13.bin out=kv_${k}_p2t1s_$rep.bin pipe=1 passes=10 post=2 threads=1 skip=1")
    items+=("v_${k}_p2t2s_$rep|model=$k.nnc in=gray13.bin out=kv_${k}_p2t2s_$rep.bin pipe=1 passes=10 post=2 threads=2 skip=1")
  done
done
for it in "${items[@]}"; do
  t=${it%%|*}; a=${it#*|}; n=0
  while :; do
    bash run1.sh $t "$a" > /dev/null; rc=$?
    [ $rc -eq 0 ] && grep -q "nncrun end" runs/$t.applog.txt && break
    [ $rc -eq 3 ] && { echo "LIMIT stop"; exit 3; }
    n=$((n+1)); [ $n -ge 40 ] && { echo "gate timeout"; exit 9; }
    sleep 60
  done
  echo "ITEM $t done $(cat runs/$t.state_before.txt)"
done
echo BATCH_DONE
