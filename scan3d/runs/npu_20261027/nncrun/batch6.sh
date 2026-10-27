#!/bin/bash
D=$(cd $(dirname $0) && pwd); cd $D
items=()
for k in xq8b xq16b xf1fp xfpmc xfpo16; do
  ev=eval3ch.bin; [ $k = xf1fp ] && ev=in_a3.bin
  items+=("s_${k}_gold|model=$k.nnc in=g_$k.bin out=og_$k.bin warm=5 iters=10 reps=1")
  items+=("s_${k}_eval|model=$k.nnc in=$ev out=oe_$k.bin warm=50 iters=200 reps=2")
  items+=("q_${k}_r1|model=$k.nnc in=gray13.bin out=kp_${k}_r1.bin pipe=1 passes=10")
  items+=("q_${k}_r2|model=$k.nnc in=gray13.bin out=kp_${k}_r2.bin pipe=1 passes=10")
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
