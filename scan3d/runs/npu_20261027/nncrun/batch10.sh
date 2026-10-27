#!/bin/bash
D=$(cd $(dirname $0) && pwd); cd $D
items=()
for k in L6_a16 L4_a16 L4s256_a16; do
  if [ $k = L4s256_a16 ]; then x=lg_x13.bin; pr=lg_pairs13.bin; n=256; else x=lg_x13_n512.bin; pr=lg_pairs13_n512.bin; n=512; fi
  items+=("m_${k}_gold|model=$k.nnc in=g_$k.bin out=og_$k.bin warm=3 iters=10 reps=1")
  items+=("m_${k}_eval|model=$k.nnc in=$x out=oe_$k.bin warm=20 iters=200 reps=2")
  items+=("m_${k}_pipe_r1|model=$k.nnc in=$pr out=ml_${k}_r1.bin lgpipe=1 lgn=$n passes=5 wr=lg_wr.bin")
  items+=("m_${k}_pipe_r2|model=$k.nnc in=$pr out=ml_${k}_r2.bin lgpipe=1 lgn=$n passes=5 wr=lg_wr.bin")
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
  echo "ITEM $t done $(cat runs/$t.state_before.txt) | after $(cat runs/$t.state_after.txt)"
done
echo BATCH_DONE
