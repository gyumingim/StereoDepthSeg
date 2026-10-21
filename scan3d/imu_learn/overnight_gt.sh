#!/usr/bin/env bash
# Overnight 2026-10-20 ground truth for laptop-screen board recordings, one recording at a time (run from scan3d/):
#   imu_gt (per-frame PnP, align) → board_shift (report only) → batch_sync (dt, R_ci, gyro bias) → imu_gt --rs --keep-corners
#   (imu_gt_rs) → spline_gt knot 0.0125 camera-only (imu_gt_spl) → scale_ls (IMU scale check) → gt_status.py
# Raw IMU (types 35/16) when the recording has them (align.json imu_raw), else Android-calibrated 1/4.
# Progress lines → runs/overnight_gt.log (watchdog), step logs → runs/overnight_gt/<name>_<step>.log.
#   bash imu_learn/overnight_gt.sh recordings/hs_20261020_214138 [...]
# BOARD=auto (default since 2026-10-21 night): imu_learn/board_pick.py picks the board json from the marker ids
# (0-21 -> laptop 42.86 mm / LOW_BOARD, 100-121 -> laptop_fit 32.03 mm). LOG / OD override the log file / step folder.
# BOARD / KNOT / PY overridable from the environment; FROM=board_shift|batch_sync|imu_gt_rs|spline|scale resumes
# (earlier outputs kept). Re-scaling later: imu_learn/rescale_gt.py (no re-run needed).
set -u
cd "$(dirname "$0")/.."
PY=${PY:-../venv_ffs/bin/python}
BOARD0=${BOARD:-auto}
KNOT=${KNOT:-0.0125}
export PYTHONIOENCODING=utf-8
LOG=${LOG:-runs/overnight_gt.log}
OD=${OD:-runs/overnight_gt}
mkdir -p $OD
say() { echo "$(date +%T) $*" | tee -a $LOG; }
step() {  # step <name> <rec> <cmd...>: run, log, record failure
  local s=$1 n=$2; shift 2
  local t0=$(date +%s)
  if "$@" > $OD/${n}_$s.log 2>&1; then
    say "$n $s ok ($(( $(date +%s) - t0 )) s)"
    return 0
  fi
  say "$n $s FAILED ($(( $(date +%s) - t0 )) s): $(grep -v '^\[' $OD/${n}_$s.log | tail -2 | tr '\n' ' ')"
  return 1
}
for R in "$@"; do
  R=${R%/}
  N=$(basename $R)
  RAW=""
  BOARD=$BOARD0
  if [ "$BOARD" = auto ]; then
    BOARD=$($PY imu_learn/board_pick.py $R 2>> $OD/${N}_board_pick.log | tail -1)
    [ -f "$BOARD" ] || { say "$N board_pick FAILED: $(tail -1 $OD/${N}_board_pick.log)"; continue; }
  fi
  if grep -q -m1 '^16,' $R/imu.csv && grep -q -m1 '^35,' $R/imu.csv; then RAW="--raw-imu"; fi
  say "$N start (board $(basename $BOARD), imu ${RAW:-calibrated})"
  if [ -z "${FROM:-}" ]; then
  step imu_gt $N $PY -u imu_learn/imu_gt.py $R --board $BOARD $RAW || { $PY imu_learn/gt_status.py $R; continue; }
  fi
  case "${FROM:-}" in ""|board_shift) RUN=1;; *) RUN=0;; esac
  [ $RUN = 1 ] && step board_shift $N $PY imu_learn/board_shift.py $R --max-gap-ms 500 --raw-jump   # diagnostic only, no trim (detector has false hits)
  [ "${FROM:-}" = batch_sync ] && RUN=1
  [ $RUN = 1 ] && { step batch_sync $N $PY -u imu_learn/batch_sync.py $R --write --json-out $OD/${N}_batch_sync.json || { $PY imu_learn/gt_status.py $R; continue; }; }
  [ "${FROM:-}" = imu_gt_rs ] && RUN=1
  [ $RUN = 1 ] && { step imu_gt_rs $N $PY -u imu_learn/imu_gt.py $R --board $BOARD --rs --keep-corners --out-dir imu_gt_rs || { $PY imu_learn/gt_status.py $R; continue; }; }
  [ "${FROM:-}" = spline ] && RUN=1
  [ $RUN = 1 ] && step spline $N $PY -u imu_learn/spline_gt.py $R --knot $KNOT
  step scale $N bash -c "$PY imu_learn/scale_ls.py $R --gt-dir imu_gt_rs --out $OD/${N}_scale_rs.json && \
                          $PY imu_learn/scale_ls.py $R --gt-dir imu_gt_spl --out $OD/${N}_scale_spl.json"
  SC=$($PY -c "import json,sys; d=json.load(open(sys.argv[1]))[0]; print(d['s'], d['gt_square_mm'], d['square_mm_raw'])" $OD/${N}_scale_spl.json 2>/dev/null)
  if [ -n "$SC" ]; then read -r S1 SQ SR <<< "$SC"
    say "$N scale check (spl): s $S1 board $SQ mm -> IMU $SR mm $(python3 -c "print('OK' if abs($S1-1)<=0.15 else 'BOARD?')")"; fi
  $PY imu_learn/gt_status.py $R >> $OD/${N}_status.log 2>&1
  say "$N done: $(tail -1 $OD/${N}_status.log)"
done
