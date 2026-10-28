#!/bin/bash
D=$(cd $(dirname $0) && pwd); cd $D
items=("z_p1_gold|model=p1.nnc in=g_p1.bin out=og_p1.bin warm=5 iters=10 reps=1"
 "z_f1_gold|model=f1.nnc in=g_f1.bin out=og_f1.bin warm=5 iters=10 reps=1"
 "z_p1_eval|model=p1.nnc in=in960.bin out=oe_p1.bin warm=30 iters=200 reps=2"
 "z_f1_eval|model=f1.nnc in=in960.bin out=oe_f1.bin warm=30 iters=200 reps=2"
 "z_p1_pipe_r1|model=p1.nnc in=in960.bin out=kz_p1_r1.bin p960=1 post960=soft passes=10"
 "z_f1_pipe_r1|model=f1.nnc in=in960.bin out=kz_f1_r1.bin p960=1 post960=off passes=10"
 "z_p1_pipe_r2|model=p1.nnc in=in960.bin out=kz_p1_r2.bin p960=1 post960=soft passes=10"
 "z_f1_pipe_r2|model=f1.nnc in=in960.bin out=kz_f1_r2.bin p960=1 post960=off passes=10")
for it in "${items[@]}"; do
  t=${it%%|*}; a=${it#*|}; n=0
  while :; do
    bash run1.sh $t "$a" > /dev/null; rc=$?
    [ $rc -eq 0 ] && grep -q "nncrun end" runs/$t.applog.txt && break
    [ $rc -eq 3 ] && { echo "LIMIT stop"; exit 3; }
    n=$((n+1)); [ $n -ge 10 ] && { echo "gate timeout at $t"; exit 9; }
    sleep 60
  done
  echo "ITEM $t done $(cat runs/$t.state_before.txt) | after $(cat runs/$t.state_after.txt)"
done
echo BATCH_DONE
