#!/bin/bash
# scanA more-recordings (11-05): VIO replay (live c4f60 flags) of 4 HS 240 fps recordings (2.22 dpt focus) with the
# config regenerated from the matching 2.22 dpt board lens (hs_1920x1080_f2.22_cal20261022.json, fx 1303.9), fed at 60 fps.
W=$(cd "$(dirname "$0")/../../../.." && pwd)
B=$HOME/scanner/build_livepose; P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
O=$W/scan3d/runs/scanA_20261105/more/vio; mkdir -p $O
[ -d $B/cfg_gen_hs222/c4f60 ] || (cd $W/scan3d && $P ov_bridge/android/make_phone_ov_main.py $B/cfg_gen_hs222 \
   --lens-file config/lens/hs_1920x1080_f2.22_cal20261022.json --focus-dpt 2.22)
sha256sum $B/cfg_gen_hs222/c4f60/* > $O/cfg_sha.txt
A="--bridge $B/bridge/ov_bridge --vocab $W/scan3d/ov_bridge/loop/voc_k10L4.voc"
cd $W/scan3d/runs/livepose_20261030
for r in "$@"; do
  $P lp_replay.py $W/scan3d/recordings/hs_$r $B/cfg_gen_hs222/c4f60 $O/$r $A > $O/$r.log 2>&1 &
done
wait; tail -qn1 $O/*.log
