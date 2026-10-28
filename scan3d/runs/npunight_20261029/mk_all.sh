#!/bin/bash
# npunight: PC replay of 5 std60 GT recordings through c4f60 + PC bridge (flags off), stdin tee -> stream.bin
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python
R=~/scanner/StereoDepthSeg/scan3d/recordings
C=~/scanner/tmp_lens05/new_gen/c4f60
B=~/scanner/tmp_appnpu/ident/ov_bridge_new
O=~/scanner/tmp_npunight/pc; mkdir -p $O
D=$(dirname $(readlink -f $0))
for r in 165506 142148 151646 173432 142610; do
  nice -n 5 $PY $D/mkstream.py $R/hs_20261024_${r}_std60 $C $B $O/$r > $O/$r.log 2>&1
  tail -1 $O/$r.log
done
