# ON-PHONE runner for integ2-20261026 (copy of ovupd_20261026/phone/ovupd_run.sh; names -> integ2; FEED column added).
# Camera-free replay. LIST lines: TAG BIN CFG THREADS FEED [EXTRA bridge args...]
#   FEED = f4 (feeder4_integ2 raw80 speed 1.0, 1 loop) or f3v:MAXT:AT:MS:MODE (feeder3v_integ2 = droplatefix/verifier feeder:
#   MODE 2 = permanent shift: sleep MS once at stream time AT and move the schedule by MS; stop after MAXT s of stream).
#   @O@ in EXTRA = the run output dir. Gates as in the ovupd runner (start SKIN<36 / AP<50, abort SKIN>=45 / AP>=50, screen).
cd /data/local/tmp/integ2 || exit 1
R=/data/local/tmp/replay
C=/data/local/tmp/integ2/cfg
export LD_LIBRARY_PATH=/data/local/tmp/integ2/lib
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
while read TAG BIN CFG TH FEED EXTRA; do
  [ -z "$TAG" ] && continue
  case "$TAG" in \#*) continue;; esac
  IN=$R/raw80.bin
  B=./$BIN
  O=out/$TAG; mkdir -p $O
  EXTRA=$(echo "$EXTRA" | sed "s#@O@#$O#g")
  case "$FEED" in f3v:*) FD=feeder3v_integ2; FA="$(echo $FEED | cut -d: -f2-5 | tr : ' ')";; *) FD=feeder4_integ2; FA="";; esac
  sleep ${MIN_COOL:-0}
  while gt "$(skin)" "$SKIN_MAX_START" || gt "$(ap)" "$AP_MAX_START" || busy; do
    echo "$(date +%T) $TAG wait SKIN $(skin) AP $(ap) screen $(wake) fg $(fg)" >> $ST; sleep 15
  done
  S0=$(skin); A0=$(ap); T0=$(date +%T)
  CAP0=$(cat /sys/devices/system/cpu/cpu9/cpufreq/scaling_max_freq)
  WK0=$(wake); FG0=$(fg)
  echo "$T0 $TAG run bin $BIN cfg $CFG thr $TH feed $FEED in $IN extra '$EXTRA' SKIN $S0 AP $A0 cpu9_cap $CAP0 screen $WK0 fg $FG0" >> $ST
  dumpsys battery > $O/battery_start.txt
  rm -f $C/$CFG/ov_timing.txt 2>/dev/null
  if [ "$FD" = feeder3v_integ2 ]; then FCMD="./feeder3v_integ2 $IN 1.0 - 1.0 $FA"; else FCMD="./feeder4_integ2 $IN 1.0 $O/progress.txt 1.0 1 0"; fi
  ( $FCMD | $B $C/$CFG/estimator_config.yaml --stdio --max-speed 8 \
      --lost-frames 1000000 --startup-lost-frames 1000000 --remap $R/remap_1920x1080.bin --timing-log $O/tl.csv \
      --threads $TH $EXTRA > $O/po.jsonl 2> $O/pe.txt ) 2> $O/feeder.txt &
  SMAX=$S0; AB=0
  while pidof $FD > /dev/null || pidof $BIN > /dev/null; do
    sleep 5
    P=$(pidof $BIN)
    { echo "t $(date +%s) skin $(skin) ap $(ap) cur $(dumpsys battery | grep -m1 'current now' | awk '{print $3}') \
bst $(dumpsys battery | grep -m1 ' status:' | awk '{print $2}') wake $(wake) \
cap9 $(cat /sys/devices/system/cpu/cpu9/cpufreq/scaling_max_freq) cap7 $(cat /sys/devices/system/cpu/cpu7/cpufreq/scaling_max_freq) \
cap4 $(cat /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq) freq $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq | xargs)"
      [ -n "$P" ] && cat /proc/$P/task/*/stat 2>/dev/null | awk '{print "  tid " $1 " ut " $14 " stt " $15 " cpu " $39}'; } >> $O/samp.txt
    s=$(skin); a=$(ap); gt "$s" "$SMAX" && SMAX=$s
    if gt "$s" "$SKIN_ABORT" || gt "$a" "$AP_ABORT"; then AB=1; echo "$(date +%T) $TAG ABORT SKIN $s AP $a" >> $ST; pkill $FD; pkill $BIN; sleep 2; fi
  done
  wait
  [ -f $C/$CFG/ov_timing.txt ] && mv $C/$CFG/ov_timing.txt $O/ov_timing.txt
  dumpsys battery > $O/battery_end.txt
  echo "tag=$TAG bin=$BIN cfg=$CFG threads=$TH feed=$FEED in=$IN extra='$EXTRA' start=$T0 end=$(date +%T) skin_start=$S0 skin_end=$(skin) skin_max=$SMAX ap_start=$A0 ap_end=$(ap) cpu9_cap_start=$CAP0 screen_start=$WK0 fg_start=$FG0 screen_end=$(wake) fg_end=$(fg) aborted=$AB" > $O/run.txt
  echo "$(date +%T) $TAG done SKIN $(skin) max $SMAX aborted $AB" >> $ST
done < $1
echo "ALL DONE $(date +%T) $1" >> $ST
pidof feeder4_integ2 feeder3v_integ2 $(ls ov_bridge_* 2>/dev/null) || echo "no bridge/feeder left" >> $ST
