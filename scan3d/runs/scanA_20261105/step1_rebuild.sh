#!/bin/bash
# scanA 2026-11-05 step 1: rebuild the live-config PC replay (bridge + c4f60) outside ~/scanner/tmp_* and replay 165506.
# Usage (WSL): bash scan3d/runs/scanA_20261105/step1_rebuild.sh   -> ~/scanner/build_livepose/{bridge,cfg_gen,runs}
set -e
W=$(cd "$(dirname "$0")/../../.." && pwd)        # worktree root
B=$HOME/scanner/build_livepose
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
R=$W/scan3d/recordings/hs_20261024_165506_std60
mkdir -p "$B/runs"
cmake -S "$W/scan3d/ov_bridge" -B "$B/bridge" -DOV_FULL_SOURCE=ON -DCMAKE_BUILD_TYPE=Release > "$B/cmake.log"
cmake --build "$B/bridge" -j16 --target ov_bridge > "$B/build.log" 2>&1
(cd "$W/scan3d" && $P ov_bridge/android/make_phone_ov_main.py "$B/cfg_gen" \
   --lens-file config/lens/std_1920x1080_f0.50_calcap20261026.json)
sha256sum "$B/bridge/ov_bridge" "$B"/cfg_gen/c4f60/*      # expect imucam 58b9a1af46461eb3, remap 483168c9723c0ba1
A="--bridge $B/bridge/ov_bridge --vocab $W/scan3d/ov_bridge/loop/voc_k10L4.voc"
cd "$W/scan3d/runs/livepose_20261030"
for o in 0 1 2; do $P lp_replay.py "$R" "$B/cfg_gen/c4f60" "$B/runs/B_o$o" $A --start-off $o > "$B/runs/B_o$o.log" 2>&1 & done
$P lp_replay.py "$R" "$B/cfg_gen/c4f60" "$B/runs/smoke_calor" $A --imu cal --time oracle > "$B/runs/smoke.log" 2>&1 &
wait
tail -qn1 "$B"/runs/*.log                                  # expect ate_vio_cm 1.70 / 1.57 / 1.52, smoke 1.63
