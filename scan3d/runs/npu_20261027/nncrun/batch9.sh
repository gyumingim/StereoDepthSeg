#!/bin/bash
D=$(cd $(dirname $0) && pwd); cd $D
items=(
 "x_xpsoft_gold|model=xpsoft.nnc in=g_xpsoft.bin out=og_xpsoft.bin warm=5 iters=10 reps=1"
 "x_xpsoft_eval|model=xpsoft.nnc in=eval3ch.bin out=oe_xpsoft.bin warm=50 iters=200 reps=2"
 "x_xpsoft_pipe_r1|model=xpsoft.nnc in=gray13.bin out=kx_xpsoft_r1.bin pipe=1 passes=10 post=4 oc=129"
 "x_xpsoft_pipe_r2|model=xpsoft.nnc in=gray13.bin out=kx_xpsoft_r2.bin pipe=1 passes=10 post=4 oc=129"
 "x_xfpmc_pipe_r1|model=xfpmc.nnc in=gray13.bin out=kx_xfpmc_r1.bin pipe=1 passes=10 post=2 threads=1"
 "x_xfpmc_pipe_r2|model=xfpmc.nnc in=gray13.bin out=kx_xfpmc_r2.bin pipe=1 passes=10 post=2 threads=1"
 "l_lg_gold|model=lg.nnc in=g_lg.bin out=og_lg.bin warm=3 iters=10 reps=1 msg=1"
 "l_lg_eval|model=lg.nnc in=lg_x13.bin out=oe_lg.bin warm=20 iters=200 reps=2"
 "l_lg_pipe_r1|model=lg.nnc in=lg_pairs13.bin out=ml_lg_r1.bin lgpipe=1 passes=5 wr=lg_wr.bin"
 "l_lg_pipe_r2|model=lg.nnc in=lg_pairs13.bin out=ml_lg_r2.bin lgpipe=1 passes=5 wr=lg_wr.bin")
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
