#!/usr/bin/env bash
# Reduced VIO matrix for the 2026-10-20 morning report (training ran long): 2 recordings, 60 s each.
set -u
cd "$(dirname "$0")/.."
PY=../venv_ffs/bin/python
OUT=runs/vio; mkdir -p $OUT
S='--max-secs 60'
D='--dropout 20:2,30:5,42:10'
J=()
for n in hs_20261019_131326 hs_20261019_145816; do
  R=recordings/$n
  for f in 30 60 120; do J+=("$R --fps $f $S --out $OUT/${n}_fps$f.json"); done
  J+=("$R --fps 60 --noise meas $S --out $OUT/${n}_noisemeas.json")
  J+=("$R --fps 60 --size 480x270 $S --out $OUT/${n}_size480.json")
  J+=("$R --fps 60 $D $S --out $OUT/${n}_drop_raw.json")
done
echo "== part 1 (${#J[@]} runs) $(date +%T)"
printf '%s\n' "${J[@]}" | xargs -P 6 -I{} bash -c "$PY imu_learn/vio_bench_hs.py {} > /dev/null 2>&1 || echo fail: {}"
for n in hs_20261019_131326 hs_20261019_145816; do until [ -f runs/on_rs_$n/result.json ]; do sleep 20; done; done
J=()
for n in hs_20261019_131326 hs_20261019_145816; do
  R=recordings/$n; CAL=runs/on_rs_$n/result.json
  J+=("$R --fps 60 --imu-calib $CAL $S --out $OUT/${n}_calib.json")
  J+=("$R --fps 60 --imu-calib $CAL --noise meas $S --out $OUT/${n}_calib_noisemeas.json")
  J+=("$R --fps 60 --imu-calib $CAL --noise meas $D $S --out $OUT/${n}_drop_calib.json")
done
echo "== part 2 (${#J[@]} runs, held-out calibration) $(date +%T)"
printf '%s\n' "${J[@]}" | xargs -P 6 -I{} bash -c "$PY imu_learn/vio_bench_hs.py {} > /dev/null 2>&1 || echo fail: {}"
echo "== timing (one at a time) $(date +%T)"
R=recordings/hs_20261019_131326
for cfg in '--fps 30' '--fps 60' '--fps 120' '--fps 60 --size 480x270' '--fps 60 --npts 100'; do
  tag=$(echo $cfg | tr -d ' -')
  $PY imu_learn/vio_bench_hs.py $R $cfg --max-secs 30 --out $OUT/timing_$tag.json > /dev/null 2>&1 || echo fail timing $cfg
done
echo "== done $(date +%T)"
