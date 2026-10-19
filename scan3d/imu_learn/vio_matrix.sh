#!/usr/bin/env bash
# VIO comparison matrix on the board recordings (after overnight.sh: needs on_rs_* training results for the held-out
# physical calibration). 4 runs in parallel for accuracy; then timing runs one at a time on an idle machine.
set -u
cd "$(dirname "$0")/.."
PY=../venv_ffs/bin/python
export PYTHONIOENCODING=utf-8
OUT=runs/vio
mkdir -p $OUT
jobs=()
for n in hs_20261019_131326 hs_20261019_145816 hs_20261019_154320; do
  R=recordings/$n; S='--max-secs 120'
  CAL=runs/on_rs_$n/result.json                       # physical calibration trained WITHOUT this recording
  for f in 30 60 120 240; do jobs+=("$R --fps $f $S --out $OUT/${n}_fps$f.json"); done
  jobs+=("$R --fps 60 --noise meas $S --out $OUT/${n}_noisemeas.json")
  [ -f $CAL ] && jobs+=("$R --fps 60 --imu-calib $CAL $S --out $OUT/${n}_calib.json")
  [ -f $CAL ] && jobs+=("$R --fps 60 --imu-calib $CAL --noise meas $S --out $OUT/${n}_calib_noisemeas.json")
  D='--dropout 30:2,50:5,80:10'
  jobs+=("$R --fps 60 $D $S --out $OUT/${n}_drop_raw.json")
  [ -f $CAL ] && jobs+=("$R --fps 60 --imu-calib $CAL --noise meas $D $S --out $OUT/${n}_drop_calib.json")
  for sz in 480x270 320x180; do jobs+=("$R --fps 60 --size $sz $S --out $OUT/${n}_size$sz.json"); done
  for np_ in 100 150; do jobs+=("$R --fps 60 --npts $np_ $S --out $OUT/${n}_npts$np_.json"); done
  jobs+=("$R --fps 60 --clones 6 $S --out $OUT/${n}_clones6.json")
done
echo "== accuracy runs: ${#jobs[@]} $(date +%T)"
printf '%s\n' "${jobs[@]}" | xargs -P 4 -I{} bash -c "$PY imu_learn/vio_bench_hs.py {} > /dev/null 2>&1 || echo fail: {}"
echo "== timing runs (idle, one at a time) $(date +%T)"
R=recordings/hs_20261019_131326
for cfg in '--fps 30' '--fps 60' '--fps 120' '--fps 60 --size 480x270' '--fps 60 --size 320x180' '--fps 60 --npts 100' '--fps 120 --size 480x270 --npts 100'; do
  tag=$(echo $cfg | tr -d ' -')
  $PY imu_learn/vio_bench_hs.py $R $cfg --max-secs 40 --out $OUT/timing_$tag.json > /dev/null 2>&1 || echo fail timing $cfg
done
echo "== done $(date +%T)"
