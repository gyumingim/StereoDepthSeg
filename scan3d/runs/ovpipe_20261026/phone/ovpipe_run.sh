# ON-PHONE runner for ovpipe-20261026 (copy of integ_20261025/phone/integ_run.sh, names integ -> ovpipe). Camera-free replay.
# Pushed to /data/local/tmp/ovpipe/ and started with `adb shell nohup sh /data/local/tmp/ovpipe/ovpipe_run.sh LIST &`.
# Reads (read-only) the replay study's raw80.bin + remap in /data/local/tmp/replay; own copies of the bridge / feeder /
# configs in /data/local/tmp/ovpipe (own process names: feeder4_ovpipe, ov_bridge_ovpipe*), nothing written elsewhere.
# LIST lines: TAG BIN CFG IDLE_SPIN THREADS SPEED LOOPS SKIP_S STREAM [EXTRA bridge args...]
#   STREAM = "raw80" (/data/local/tmp/replay/raw80.bin) or a file name in /data/local/tmp/ovpipe.
#   LOOPS > 1 replays the stream again (feeder4: passes >= 1 leave out the first SKIP_S s, times shifted) - add
#   --auto-reset to EXTRA so the pose jump at each seam resets + re-initialises the VIO.
# Gate before each run: SKIN < SKIN_MAX_START (default 36, user 2026-10-25 16:30) and AP < AP_MAX_START (default 50)
# ("Current temperatures from HAL"), screen not used by another app; abort at SKIN >= SKIN_ABORT (45) or AP >= AP_ABORT (50). Samples every 5 s: SKIN / AP, big-core caps, all clocks, battery current, screen,
# bridge thread CPU times.
cd /data/local/tmp/ovpipe || exit 1
R=/data/local/tmp/replay
C=/data/local/tmp/ovpipe/cfg
export LD_LIBRARY_PATH=/data/local/tmp/ovpipe/lib
ST=out/status.txt
mkdir -p out
SKIN_MAX_START=${SKIN_MAX_START:-36.0}
AP_MAX_START=${AP_MAX_START:-50.0}
SKIN_ABORT=${SKIN_ABORT:-45.0}
AP_ABORT=${AP_ABORT:-50.0}
skin() { dumpsys thermalservice | grep -A40 'Current temperatures from HAL' | grep -m1 'mName=SKIN' | sed 's/.*mValue=\([0-9.]*\).*/\1/'; }
ap() { dumpsys thermalservice | grep -A40 'Current temperatures from HAL' | grep -m1 'mName=AP,' | sed 's/.*mValue=\([0-9.]*\).*/\1/'; }
gt() { awk -v a="$1" -v b="$2" 'BEGIN{exit !(a>=b)}'; }
fg() { dumpsys window | grep -m1 mCurrentFocus | sed 's/.*u0 //; s/}.*//'; }
wake() { dumpsys power | grep -m1 mWakefulness= | sed 's/.*=//'; }
busy() { [ "$(wake)" = Awake ] || return 1; case "$(fg)" in *launcher*|*NotificationShade*|*null*|"") return 1;; esac; return 0; }
echo "start $(date +%T) list $1 skin_max_start $SKIN_MAX_START ap_max_start $AP_MAX_START" >> $ST
while read TAG BIN CFG SP TH SPD LOOPS SKIP STREAM EXTRA; do
  [ -z "$TAG" ] && continue
  case "$TAG" in \#*) continue;; esac
  [ "$STREAM" = raw80 ] && IN=$R/raw80.bin || IN=./$STREAM
  B=./$BIN
  O=out/$TAG; mkdir -p $O
  sleep ${MIN_COOL:-0}
  while gt "$(skin)" "$SKIN_MAX_START" || gt "$(ap)" "$AP_MAX_START" || busy; do
    echo "$(date +%T) $TAG wait SKIN $(skin) AP $(ap) screen $(wake) fg $(fg)" >> $ST; sleep 15
  done
  S0=$(skin); A0=$(ap); T0=$(date +%T)
  CAP0=$(cat /sys/devices/system/cpu/cpu9/cpufreq/scaling_max_freq)
  WK0=$(wake); FG0=$(fg)
  echo "$T0 $TAG run bin $BIN cfg $CFG spin $SP thr $TH speed $SPD loops $LOOPS skip $SKIP in $IN extra '$EXTRA' SKIN $S0 AP $A0 cpu9_cap $CAP0 screen $WK0 fg $FG0" >> $ST
  dumpsys battery > $O/battery_start.txt
  rm -f $C/$CFG/ov_timing.txt 2>/dev/null
  ( ./feeder4_ovpipe $IN 1.0 $O/progress.txt $SPD $LOOPS $SKIP | $B $C/$CFG/estimator_config.yaml --stdio --max-speed 8 \
      --lost-frames 1000000 --startup-lost-frames 1000000 --remap $R/remap_1920x1080.bin --timing-log $O/tl.csv \
      --threads $TH --idle-spin $SP $EXTRA > $O/po.jsonl 2> $O/pe.txt ) 2> $O/feeder.txt &
  SMAX=$S0; AB=0
  while pidof feeder4_ovpipe > /dev/null || pidof $BIN > /dev/null; do
    sleep 5
    P=$(pidof $BIN)
    { echo "t $(date +%s) skin $(skin) ap $(ap) cur $(dumpsys battery | grep -m1 'current now' | awk '{print $3}') \
bst $(dumpsys battery | grep -m1 ' status:' | awk '{print $2}') wake $(wake) \
cap9 $(cat /sys/devices/system/cpu/cpu9/cpufreq/scaling_max_freq) cap7 $(cat /sys/devices/system/cpu/cpu7/cpufreq/scaling_max_freq) \
cap4 $(cat /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq) freq $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq | xargs)"
      [ -n "$P" ] && cat /proc/$P/task/*/stat 2>/dev/null | awk '{print "  tid " $1 " ut " $14 " stt " $15 " cpu " $39}'; } >> $O/samp.txt
    s=$(skin); a=$(ap); gt "$s" "$SMAX" && SMAX=$s
    if gt "$s" "$SKIN_ABORT" || gt "$a" "$AP_ABORT"; then AB=1; echo "$(date +%T) $TAG ABORT SKIN $s AP $a" >> $ST; pkill feeder4_ovpipe; pkill $BIN; sleep 2; fi
  done
  wait
  [ -f $C/$CFG/ov_timing.txt ] && mv $C/$CFG/ov_timing.txt $O/ov_timing.txt
  dumpsys battery > $O/battery_end.txt
  echo "tag=$TAG bin=$BIN cfg=$CFG idle_spin=$SP threads=$TH speed=$SPD loops=$LOOPS skip=$SKIP in=$IN extra='$EXTRA' start=$T0 end=$(date +%T) skin_start=$S0 skin_end=$(skin) skin_max=$SMAX ap_start=$A0 ap_end=$(ap) cpu9_cap_start=$CAP0 screen_start=$WK0 fg_start=$FG0 screen_end=$(wake) fg_end=$(fg) aborted=$AB" > $O/run.txt
  echo "$(date +%T) $TAG done SKIN $(skin) max $SMAX aborted $AB" >> $ST
done < $1
echo "ALL DONE $(date +%T) $1" >> $ST
pidof feeder4_ovpipe $(ls ov_bridge_* 2>/dev/null) || echo "no bridge/feeder left" >> $ST
