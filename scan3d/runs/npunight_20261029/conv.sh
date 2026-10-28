#!/bin/bash
# npunight: stream.bin -> side.bin + frames.mp4 (crf 18) for the phone (adb is slow; the phone rebuilds the stream with unpack)
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python
D=$(dirname $(readlink -f $0))
for r in "$@"; do
  nice -n 5 $PY $D/to_mp4.py ~/scanner/tmp_npunight/pc/$r.stream.bin ~/scanner/tmp_npunight/up/$r --crf 18
done
