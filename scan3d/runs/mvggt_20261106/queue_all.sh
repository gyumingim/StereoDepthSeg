#!/bin/bash
# mvggt 11-06: 165506 (probe + all variants) -> 134204 -> 222747 only if featloop's rebuilt ba_S exists (else skipped, README).
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
Q=scan3d/runs/mvggt_20261106
$Q/run_rec.sh 165506 probe b0 vggt v mast3r m > $Q/rec/165506.out 2>&1
$Q/run_rec.sh 20261022_134204 dump b0 vggt v mast3r m > $Q/rec/20261022_134204.out 2>&1
F=$HOME/scanner/StereoDepthSeg-featloop/scan3d/runs/featloop_20261105/rec/20261020_222747/ba_S
D=$Q/rec/20261020_222747
if [ -f $F/poses_ba.jsonl.gz ] && [ -f $F/ba.json ]; then
  mkdir -p $D && cp $F/poses_ba.jsonl.gz $F/poses_vio_kf.jsonl.gz $F/ba.json $D/ && sha256sum $F/poses_ba.jsonl.gz $D/poses_ba.jsonl.gz > $D/ba_sha.txt
  $Q/run_rec.sh 20261020_222747 dump b0 vggt v mast3r m > $Q/rec/20261020_222747.out 2>&1
else
  echo "222747 skipped $(date '+%F %T'): featloop ba_S missing" > $Q/rec/20261020_222747.skipped
fi
echo "QUEUE_ALL DONE $(date '+%F %T')" > $Q/queue_all.done
