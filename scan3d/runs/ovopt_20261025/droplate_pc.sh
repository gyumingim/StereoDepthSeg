# PC overload check of ov_bridge --drop-late (2026-10-25). The phone replay study found a "cliff": once frames take
# longer than the 33 ms period the backlog keeps growing. On the PC the bridge is fast, so the overload is made by
# pinning the bridge to ONE core (taskset, --threads 2 on that core) with the heavy config c4n400s100c20; raw80 is paced
# at real time by feeder3 (x86 build of the replay study's feeder, on another core). Compared: no drop vs --drop-late
# 50 / 100 ms, without and with --ovopt all_exact. Metrics (ate_raw80.py): ATE vs board GT, feed lag p50/p95/max
# (receive time vs schedule + processing), frames fed, frames dropped.
set -u
H=~/scanner/StereoDepthSeg-ovopt/scan3d
RD=$H/runs/ovopt_20261025
OUT=/tmp/ovopt/droplate; mkdir -p $OUT
FULL=$H/ov_bridge/build_ovopt_full/ov_bridge
FEED=$OUT/feeder3
[ -x $FEED ] || g++ -O2 -pthread $RD/feeder3.cpp -o $FEED || exit 1
CFG=/tmp/ovopt/cfg/${CFG:-c4n400s100c20}
CORE=${CORE:-3}
date
for n in ${RUNS:-A0 A50 X0 X50}; do
  case $n in
    A0) X="" ;;
    A50) X="--drop-late 50" ;;
    A100) X="--drop-late 100" ;;
    X0) X="--ovopt all_exact" ;;
    X50) X="--ovopt all_exact --drop-late 50" ;;
  esac
  echo "== $n $(date +%T) bridge on cpu $CORE ($X)"
  ( taskset -c 8 $FEED /tmp/ovopt/raw80.bin 1.0 $OUT/$n.progress 1.0 | taskset -c $CORE $FULL $CFG/estimator_config.yaml --stdio \
      --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --remap /tmp/ovopt/remap_1920x1080.bin --threads 2 \
      --timing-log $OUT/$n.tl.csv $X > $OUT/$n.jsonl 2> $OUT/$n.err ) 2> $OUT/$n.feeder.txt
  tail -1 $OUT/$n.feeder.txt; grep -E "drop-late: limit" $OUT/$n.err
  mv $CFG/ov_timing.txt $OUT/$n.ov_timing.txt 2>/dev/null
  python3 $RD/ate_raw80.py $OUT/$n
done
date
