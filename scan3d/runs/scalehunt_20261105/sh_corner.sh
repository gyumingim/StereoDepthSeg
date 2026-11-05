#!/bin/bash
# scalehunt R5: board-corner ruler (scanA corner_eval.py, kf 10, RS 0) on 165506 for every condition -> ROOT/corner/<cond>.json
W=$HOME/scanner/StereoDepthSeg-scalehunt; R=$W/scan3d/recordings/hs_20261024_165506_std60; ROOT=$HOME/scanner/build_livepose/scalehunt
mkdir -p $ROOT/corner
for d in $ROOT/runs/*/; do c=$(basename $d); f=$d/165506s.poses.jsonl.gz; [ -f $f ] || continue; [ -f $ROOT/corner/$c.json ] && continue
  $HOME/scanner/StereoDepthSeg/venv_ffs/bin/python $W/scan3d/scanA/corner_eval.py $R --pose $c=$f --out $ROOT/corner/$c.json > /dev/null 2>&1 &
  while [ $(jobs -r | wc -l) -ge 4 ]; do sleep 1; done
done; wait
