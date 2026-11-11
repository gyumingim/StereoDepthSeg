#!/bin/bash
# gpuba 11-09: CPU reference BA (db-from featloop S db, seed 0, --ba-device cpu) on the 11 H, sequential under heavy.lock
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
D=scan3d/runs/gpuba_20261109; FLR=$HOME/scanner/StereoDepthSeg-featloop/scan3d/runs/featloop_20261105/rec
H=${H:-"20261022_133820 20261022_134550 20261022_134204 20261021_214550 20261021_213341 20261021_215348 20261020_221052 20261020_221925 20261020_222747 20261020_223815 20261020_224323"}
while pgrep -f "kf_ba.py.*gpuba_20261109/rec/20261022_133820/cpu_db" >/dev/null; do sleep 10; done
for id in $H; do
  SRC=$FLR/$id/ba_S; [ -f $SRC/db.db ] || SRC=$FLR/$id/ba_S_gpu
  [ -f $D/rec/$id/cpu_db/poses_ba.jsonl.gz ] || $D/time_rec.sh $id cpu cpu_db --db-from $SRC
done
echo "CPULANE DONE $(date +%T)"
