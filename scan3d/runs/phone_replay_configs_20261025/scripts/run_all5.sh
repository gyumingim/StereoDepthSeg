# phone side (adb shell, nohup): replay raw80.bin at real time into the 0.6.17 bridge for a list of runs.
# run_all5 (= run_all4 + screen guard: start only when the screen is off (user not using the phone) and SKIN < 36;
# screen_on_during=1 in run.txt if the screen came on during the run) (= run_all3 + optional 6th field EXTRA: extra bridge args, ':' for spaces, e.g. --cpus:7,8,9) (= run_all2 + cpu0/4/7/9 scaling_max_freq caps in samp.txt and run.txt): each line of $1: TAG CFG IDLE_SPIN THREADS [SPEED]; feeder3 (speed factor); samples cpu freqs + bridge thread
# cpu/utime every 5 s into $O/samp.txt (cheap /proc reads, no top)
cd /data/local/tmp/replay
export LD_LIBRARY_PATH=/data/local/tmp/kb0615
ST=status.txt
skin() { dumpsys thermalservice | grep -A40 'Current temperatures from HAL' | grep -m1 'mName=SKIN' | sed 's/.*mValue=\([0-9.]*\).*/\1/'; }
ap() { dumpsys thermalservice | grep -A40 'Current temperatures from HAL' | grep -m1 'mName=AP,' | sed 's/.*mValue=\([0-9.]*\).*/\1/'; }
awake() { dumpsys power | grep -q 'mWakefulness=Awake'; }
gt() { awk -v a="$1" -v b="$2" 'BEGIN{exit !(a>=b)}'; }
echo "start $(date +%T) list $1" >> $ST
while read TAG CFG SP TH SPD EXTRA; do
  XA=$(echo "$EXTRA" | tr ':' ' ')
  [ -z "$SPD" ] && SPD=1.0
  [ -z "$TAG" ] && continue
  case "$TAG" in \#*) continue;; esac
  O=out/$TAG; mkdir -p $O
  w0=$(date +%s)
  while gt "$(skin)" 36.0 || awake; do echo "$(date +%T) $TAG wait SKIN $(skin) screen $(dumpsys power | grep -o 'mWakefulness=[A-Za-z]*')" >> $ST; sleep 10; done
  S0=$(skin); A0=$(ap); T0=$(date +%T)
  C0=$(cat /sys/devices/system/cpu/cpu9/cpufreq/scaling_max_freq)
  echo "$T0 $TAG run cfg $CFG spin $SP thr $TH speed $SPD extra [$XA] SKIN $S0 AP $A0 waited $(( $(date +%s) - w0 )) s" >> $ST
  rm -f cfg/$CFG/ov_timing.txt
  ( ./feeder3 raw80.bin 1.0 $O/progress.txt $SPD | ./ov_bridge_0617 cfg/$CFG/estimator_config.yaml --stdio --max-speed 8 \
      --lost-frames 1000000 --startup-lost-frames 1000000 --remap remap_1920x1080.bin --timing-log $O/tl.csv \
      --threads $TH --idle-spin $SP $XA > $O/po.jsonl 2> $O/pe.txt ) 2> $O/feeder.txt &
  SMAX=$S0; AB=0; SCR=0
  k=0
  while pidof feeder3 > /dev/null || pidof ov_bridge_0617 > /dev/null; do
    sleep 5
    P=$(pidof ov_bridge_0617)
    { echo "t $(date +%T) freq $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq | xargs) cap $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq /sys/devices/system/cpu/cpu7/cpufreq/scaling_max_freq /sys/devices/system/cpu/cpu9/cpufreq/scaling_max_freq | xargs)"
      [ -n "$P" ] && cat /proc/$P/task/*/stat 2>/dev/null | awk '{print "  tid " $1 " st " $3 " ut " $14 " stt " $15 " cpu " $39}'; } >> $O/samp.txt
    k=$((k+1)); [ $((k % 2)) -eq 0 ] || continue
    awake && SCR=1
    s=$(skin); gt "$s" "$SMAX" && SMAX=$s
    if gt "$s" 38.0; then AB=1; echo "$(date +%T) $TAG ABORT SKIN $s" >> $ST; pkill feeder3; pkill ov_bridge_0617; sleep 2; fi
  done
  wait
  [ -f cfg/$CFG/ov_timing.txt ] && mv cfg/$CFG/ov_timing.txt $O/ov_timing.txt
  S1=$(skin); A1=$(ap)
  echo "tag=$TAG cfg=$CFG idle_spin=$SP threads=$TH speed=$SPD extra=$EXTRA start=$T0 end=$(date +%T) skin_start=$S0 skin_end=$S1 skin_max=$SMAX ap_start=$A0 ap_end=$A1 cap9_start=$C0 cap9_end=$(cat /sys/devices/system/cpu/cpu9/cpufreq/scaling_max_freq) screen_on_during=$SCR aborted=$AB" > $O/run.txt
  echo "$(date +%T) $TAG done SKIN $S1 max $SMAX aborted $AB screen_on $SCR frames $(grep -c . $O/tl.csv) $(cat $O/feeder.txt | tail -1)" >> $ST
done < $1
echo "ALL DONE $(date +%T) $1" >> $ST
pidof ov_bridge_0617 feeder3 || echo "no bridge/feeder left" >> $ST
