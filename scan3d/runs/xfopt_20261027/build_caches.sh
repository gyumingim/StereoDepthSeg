#!/bin/bash
# xfopt: pre-build 960x540 / 60 fps frame caches with the lens05 lenses (offsets 0,1,2) so parallel jobs never race on a cache.
set -u
S=/home/system/scanner/StereoDepthSeg/scan3d
WT=/home/system/scanner/StereoDepthSeg-xfopt/scan3d
PY=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
LOG=/home/system/scanner/tmp_xfopt/caches.log
mkdir -p /home/system/scanner/tmp_xfopt
cd $S
HSL=$WT/config/lens/hs_1920x1080_f2.22_from_calcap05_20261027.json
STL=$WT/config/lens/std_1920x1080_f0.50_calcap20261026.json
one() {
  r=$1; l=$2; offs=$3
  echo "$(date '+%F %T') start $r" >> $LOG
  $PY imu_learn/frame_cache.py build recordings/$r --size 960x540 --fps 60 --offsets $offs --threads 2 --lens-file $l >> $LOG 2>&1
  echo "$(date '+%F %T') done $r exit $?" >> $LOG
}
export -f one; export PY LOG
{
for r in hs_20261020_221052 hs_20261020_222747 hs_20261020_223815 hs_20261021_214550 hs_20261022_134550 \
         hs_20261019_131326 hs_20261019_154320 hs_20261020_214138 hs_20261020_221925 hs_20261020_224323 \
         hs_20261021_213341 hs_20261021_215348 hs_20261022_133820 hs_20261022_134204; do echo "$r $HSL 0,1,2"; done
echo "hs_20261024_165506_std60 $STL 0,1,2"
echo "hs_20261024_142148_std60 $STL 0"
echo "hs_20261024_142610_std60 $STL 0"
} | xargs -P 4 -L 1 bash -c 'one "$0" "$1" "$2"'
echo "$(date '+%F %T') ALL DONE" >> $LOG
