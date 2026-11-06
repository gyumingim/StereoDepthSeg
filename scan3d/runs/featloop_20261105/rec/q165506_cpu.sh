#!/bin/bash
# queue: after S seed repeats, greedy tracks for each variant once its GPU BA exists, then seeds 1 2 for the rest
D=$(cd "$(dirname "$0")/.." && pwd); cd $D
until grep -q REPDONE rec/165506_rep.out; do sleep 20; done
for v in S X SL XL; do
  until [ -f rec/165506/ba_$v/ba.json ] || grep -q "DONE 165506" rec/165506.out; do sleep 20; done
  flock /home/system/scanner/featloop_ext/cpu.lock env SEEDS=0 TRK=greedy bash cpu_rec.sh 165506 $v
done
flock /home/system/scanner/featloop_ext/cpu.lock env SEEDS="1 2" TRK=greedy bash cpu_rec.sh 165506 S X SL XL
flock /home/system/scanner/featloop_ext/cpu.lock env SEEDS="1 2" TRK=drop bash cpu_rec.sh 165506 X SL XL
echo QDONE
