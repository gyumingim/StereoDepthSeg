#!/bin/bash
# viba 11-05: VIO replay (live c4f60 flags, lp_replay.py unchanged) of the 12 H recordings (240 fps, 2.22 dpt) with the
# config made by scanA "more" from the matching 2.22 dpt lens (build_livepose/cfg_gen_hs222/c4f60, fx 1303.9), fed at 60 fps.
# 165506 uses scanA step1 B_o0 (cfg_gen/c4f60, 0.5 dpt). At most 5 bridges at once (machine limit 6).
W=$(cd "$(dirname "$0")/../../.." && pwd)
B=$HOME/scanner/build_livepose; P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
O=$W/scan3d/runs/viba_20261105/vio; mkdir -p $O
sha256sum $B/cfg_gen_hs222/c4f60/* $B/bridge/ov_bridge > $O/cfg_sha.txt
A="--bridge $B/bridge/ov_bridge --vocab $W/scan3d/ov_bridge/loop/voc_k10L4.voc"
cd $W/scan3d/runs/livepose_20261030
printf "%s\n" "$@" | xargs -P 5 -I{} sh -c "$P lp_replay.py $W/scan3d/recordings/hs_{} $B/cfg_gen_hs222/c4f60 $O/{} $A > $O/{}.log 2>&1; echo {} rc \$? \$(date +%T)"
echo ALLDONE $(date +%T)
