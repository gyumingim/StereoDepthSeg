# imuprio 2026-10-25 (arlat 0.6.19): copy of scan3d/runs/ovopt_20261025/phone/phone_run.sh with its own dir
# /data/local/tmp/imuprio (own feeder3_imuprio / ov_bridge_imuprio copies, own cfg copy) - camera-free raw80 replay.
# ON-PHONE runner for the ovopt phone plan (2026-10-25). NOT RUN YET - only after the user approves phone work.
# Pushed to /data/local/tmp/ovopt/ and started with `adb shell nohup sh /data/local/tmp/ovopt/phone_run.sh LIST &`.
# Camera-free: replays the replay study's raw80 stream (read-only reuse of /data/local/tmp/replay: raw80.bin, feeder3,
# remap_1920x1080.bin) into ov_bridge binaries with OUR configs (/data/local/tmp/ovopt/cfg/<cfg>, made by
# phone_plan.sh: config/phone_ov_main/<cfg> + timeshift_cam_imu 0 like the replay study), like run_all2.sh, plus per-run
# samples every 5 s: SKIN / AP (HAL section of dumpsys thermalservice), big-core clock cap + current clocks,
# battery "current now" + status from dumpsys battery (sysfs current_now is not readable for adb shell; energy proxy),
# screen state (dumpsys power mWakefulness: the user may use the phone), bridge thread CPU times. No phone settings are changed, nothing is written or deleted
# outside /data/local/tmp/ovopt/.
# LIST lines: TAG BIN CFG IDLE_SPIN THREADS SPEED [EXTRA bridge args...]   (BIN = file name in /data/local/tmp/ovopt or
#             /data/local/tmp/replay/ov_bridge_0617; CFG = config dir name under /data/local/tmp/ovopt/cfg)
# Own process names (feeder3_imuprio, ov_bridge_0617_ref, ov_bridge_ovopt*: copies made by phone_plan.sh) so pidof /
# pkill never touch another agent's feeder3 / ov_bridge_0617.
cd /data/local/tmp/imuprio || exit 1
R=/data/local/tmp/replay
C=/data/local/tmp/imuprio/cfg
export LD_LIBRARY_PATH=/data/local/tmp/ovopt/lib
ST=out/status.txt
mkdir -p out
SKIN_MAX_START=${SKIN_MAX_START:-36.0}   # 2026-10-25 user gate: start < 36, abort >= 45 (AP >= 50)
skin() { dumpsys thermalservice | grep -A40 'Current temperatures from HAL' | grep -m1 'mName=SKIN' | sed 's/.*mValue=\([0-9.]*\).*/\1/'; }
ap() { dumpsys thermalservice | grep -A40 'Current temperatures from HAL' | grep -m1 'mName=AP,' | sed 's/.*mValue=\([0-9.]*\).*/\1/'; }
gt() { awk -v a="$1" -v b="$2" 'BEGIN{exit !(a>=b)}'; }
echo "start $(date +%T) list $1 skin_max_start $SKIN_MAX_START" >> $ST
grep -m1 Features /proc/cpuinfo >> $ST
while read TAG BIN CFG SP TH SPD EXTRA; do
  [ -z "$TAG" ] && continue
  case "$TAG" in \#*) continue;; esac
  B=./$BIN
  O=out/$TAG; mkdir -p $O
  sleep ${MIN_COOL:-0}   # fixed cool-down before every run (L1 showed the big-core cap falls 2784 -> 2016 MHz run to run)
  while gt "$(skin)" "$SKIN_MAX_START" || gt "$(ap)" "${AP_MAX_START:-200}"; do
    echo "$(date +%T) $TAG wait SKIN $(skin) AP $(ap)" >> $ST; sleep 15
  done
  S0=$(skin); A0=$(ap); T0=$(date +%T)
  CAP0=$(cat /sys/devices/system/cpu/cpu9/cpufreq/scaling_max_freq)
  WK0=$(dumpsys power | grep -m1 mWakefulness= | sed 's/.*=//'); FG0=$(dumpsys window | grep -m1 mCurrentFocus | sed 's/.*u0 //; s/}.*//')
  echo "$T0 $TAG run bin $BIN cfg $CFG spin $SP thr $TH speed $SPD extra '$EXTRA' SKIN $S0 AP $A0 cpu9_cap $CAP0 screen $WK0 fg $FG0" >> $ST
  dumpsys battery > $O/battery_start.txt
  rm -f $C/$CFG/ov_timing.txt 2>/dev/null
  ( ./feeder3_imuprio $R/raw80.bin 1.0 $O/progress.txt $SPD | $B $C/$CFG/estimator_config.yaml --stdio --max-speed 8 \
      --lost-frames 1000000 --startup-lost-frames 1000000 --remap $R/remap_1920x1080.bin --timing-log $O/tl.csv \
      --threads $TH --idle-spin $SP $EXTRA > $O/po.jsonl 2> $O/pe.txt ) 2> $O/feeder.txt &
  SMAX=$S0; AB=0
  while pidof feeder3_imuprio > /dev/null || pidof $BIN > /dev/null; do
    sleep 5
    P=$(pidof $BIN)
    { echo "t $(date +%s) skin $(skin) ap $(ap) cur $(dumpsys battery | grep -m1 'current now' | awk '{print $3}') \
bst $(dumpsys battery | grep -m1 ' status:' | awk '{print $2}') wake $(dumpsys power | grep -m1 mWakefulness= | sed 's/.*=//') \
cap9 $(cat /sys/devices/system/cpu/cpu9/cpufreq/scaling_max_freq) freq $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq | xargs)"
      [ -n "$P" ] && cat /proc/$P/task/*/stat 2>/dev/null | awk '{print "  tid " $1 " ut " $14 " stt " $15 " cpu " $39}'; } >> $O/samp.txt
    s=$(skin); gt "$s" "$SMAX" && SMAX=$s
    a=$(ap)
    if gt "$s" 45.0 || gt "$a" 50.0; then AB=1; echo "$(date +%T) $TAG ABORT SKIN $s AP $a" >> $ST; pkill feeder3_imuprio; pkill $BIN; sleep 2; fi
  done
  wait
  [ -f $C/$CFG/ov_timing.txt ] && mv $C/$CFG/ov_timing.txt $O/ov_timing.txt
  dumpsys battery > $O/battery_end.txt
  echo "tag=$TAG bin=$BIN cfg=$CFG idle_spin=$SP threads=$TH speed=$SPD extra='$EXTRA' start=$T0 end=$(date +%T) skin_start=$S0 skin_end=$(skin) skin_max=$SMAX ap_start=$A0 ap_end=$(ap) cpu9_cap_start=$CAP0 screen_start=$WK0 fg_start=$FG0 screen_end=$(dumpsys power | grep -m1 mWakefulness= | sed 's/.*=//') aborted=$AB" > $O/run.txt
  echo "$(date +%T) $TAG done SKIN $(skin) max $SMAX aborted $AB" >> $ST
done < $1
echo "ALL DONE $(date +%T) $1" >> $ST
pidof feeder3_imuprio ov_bridge_imuprio || echo "no bridge/feeder left" >> $ST
