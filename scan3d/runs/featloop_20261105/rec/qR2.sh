#!/bin/bash
# qR v2 (run_rec2 no longer prints the corner line): redo killed GPU-queue BAs from their db (CPU, seed 0),
# trigger = the next recording's GPU stage has started (or QHDONE for the last one)
D=$(cd "$(dirname "$0")/.." && pwd); cd $D
H="20261022_134550 20261022_134204 20261021_215348 20261021_214550 20261021_213341 20261020_224323 20261020_223815 20261020_221925 20261020_221052 20261020_222747 20261020_214138"
set -- $H
while [ $# -gt 0 ]; do
  id=$1; shift; nx=${1:-}
  if [ -n "$nx" ]; then until [ -f rec/$nx.out ]; do sleep 30; done; else until grep -q QHDONE rec/qH.out; do sleep 30; done; fi
  flock $HOME/scanner/featloop_ext/cpu.lock env SEEDS=0 TRK=drop bash cpu_rec.sh $id S X SL XL > rec/${id}_redo.out 2>&1
done
echo QRDONE $(date +%T)
