#!/bin/bash
D=$(cd $(dirname $0) && pwd); cd $D
items=()
for k in xfpmc xfpo16; do
  for rep in r1 r2; do
    items+=("w_${k}_p0_$rep|model=$k.nnc in=gray13.bin out=kw_${k}_p0_$rep.bin pipe=1 passes=10 post=0")
    items+=("w_${k}_p1_$rep|model=$k.nnc in=gray13.bin out=kw_${k}_p1_$rep.bin pipe=1 passes=10 post=1")
    items+=("w_${k}_p2t1_$rep|model=$k.nnc in=gray13.bin out=kw_${k}_p2t1_$rep.bin pipe=1 passes=10 post=2 threads=1")
    items+=("w_${k}_p2t2_$rep|model=$k.nnc in=gray13.bin out=kw_${k}_p2t2_$rep.bin pipe=1 passes=10 post=2 threads=2")
    items+=("w_${k}_p2t2c89_$rep|model=$k.nnc in=gray13.bin out=kw_${k}_p2t2c89_$rep.bin pipe=1 passes=10 post=2 threads=2 cpus=8,9")
    items+=("w_${k}_p2t3c789_$rep|model=$k.nnc in=gray13.bin out=kw_${k}_p2t3c789_$rep.bin pipe=1 passes=10 post=2 threads=3 cpus=7,8,9")
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
