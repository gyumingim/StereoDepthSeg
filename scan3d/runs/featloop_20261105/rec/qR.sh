#!/bin/bash
# redo (CPU, from db, same seed 0) the GPU-queue BAs that the RAM watchdog killed, one recording at a time after its GPU stage
D=$(cd "$(dirname "$0")/.." && pwd); cd $D
for id in 20261022_133820 20261022_134550 20261022_134204 20261021_215348 20261021_214550 20261021_213341 20261020_224323 \
          20261020_223815 20261020_221925 20261020_221052 20261020_222747 20261020_214138; do
  until grep -q "^== $id corner" rec/$id.out 2>/dev/null; do sleep 30; done
  flock $HOME/scanner/featloop_ext/cpu.lock env SEEDS=0 TRK=drop bash cpu_rec.sh $id S X SL XL > rec/${id}_redo.out 2>&1
done
echo QRDONE $(date +%T)
