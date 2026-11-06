#!/bin/bash
# H queue: GPU feature/matching + BA for 4 variants per recording (no meshes yet), smallest recordings first;
# a CPU job per recording then adds greedy-track variants (seed 0). Meshes later (separate queue).
D=$(cd "$(dirname "$0")/.." && pwd); cd $D
until grep -q "DONE 165506" rec/165506.out; do sleep 20; done
for id in 20261022_133820 20261022_134550 20261022_134204 20261021_215348 20261021_214550 20261021_213341 20261020_224323 \
          20261020_223815 20261020_221925 20261020_221052 20261020_222747 20261020_214138; do
  NOMESH=1 bash run_rec2.sh $id S X SL XL > rec/$id.out 2>&1
  ( flock /home/system/scanner/featloop_ext/cpu.lock env SEEDS=0 TRK=greedy bash cpu_rec.sh $id S X SL XL > rec/${id}_cpu.out 2>&1 & )
done
echo QHDONE $(date +%T)
