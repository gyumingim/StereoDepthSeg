#!/bin/bash
# mesh lane for H: priority S SL Sg Xg for every recording, then X XL; repeats passes until the BA queues are done
D=$(cd "$(dirname "$0")/.." && pwd); cd $D
H="20261022_133820 20261022_134550 20261022_134204 20261021_215348 20261021_214550 20261021_213341 20261020_224323 20261020_223815 20261020_221925 20261020_221052 20261020_222747 20261020_214138"
items=""; for v in S SL Sg Xg; do for id in $H; do items="$items $id:$v"; done; done
items2=""; for v in X XL; do for id in $H; do items2="$items2 $id:$v"; done; done
last=0
while true; do
  done_q=0; grep -q QHDONE rec/qH.out && grep -q QRDONE rec/qR.out && ! pgrep -f "cpu_rec.sh 2026" >/dev/null && done_q=1
  bash mesh_q.sh $items >> rec/qM_passes.out 2>&1
  bash mesh_q.sh $items2 >> rec/qM_passes.out 2>&1
  [ $last = 1 ] && break
  [ $done_q = 1 ] && last=1
  sleep 300
done
echo QMDONE $(date +%T)
