# phone side (adb shell, nohup): replay raw80.bin at real time into the 0.6.17 bridge for a list of runs.
# each line of $1: TAG CFG IDLE_SPIN THREADS
cd /data/local/tmp/replay
export LD_LIBRARY_PATH=/data/local/tmp/kb0615
ST=status.txt
skin() { dumpsys thermalservice | grep -A40 'Current temperatures from HAL' | grep -m1 'mName=SKIN' | sed 's/.*mValue=\([0-9.]*\).*/\1/'; }
ap() { dumpsys thermalservice | grep -A40 'Current temperatures from HAL' | grep -m1 'mName=AP,' | sed 's/.*mValue=\([0-9.]*\).*/\1/'; }
gt() { awk -v a="$1" -v b="$2" 'BEGIN{exit !(a>=b)}'; }
echo "start $(date +%T) list $1" >> $ST
while read TAG CFG SP TH; do
  [ -z "$TAG" ] && continue
  case "$TAG" in \#*) continue;; esac
  O=out/$TAG; mkdir -p $O
  w0=$(date +%s)
  while gt "$(skin)" 36.0; do echo "$(date +%T) $TAG wait SKIN $(skin)" >> $ST; sleep 10; done
  S0=$(skin); A0=$(ap); T0=$(date +%T)
  echo "$T0 $TAG run cfg $CFG spin $SP thr $TH SKIN $S0 AP $A0 waited $(( $(date +%s) - w0 )) s" >> $ST
  rm -f cfg/$CFG/ov_timing.txt
  ( ./feeder2 raw80.bin 1.0 $O/progress.txt | ./ov_bridge_0617 cfg/$CFG/estimator_config.yaml --stdio --max-speed 8 \
      --lost-frames 1000000 --startup-lost-frames 1000000 --remap remap_1920x1080.bin --timing-log $O/tl.csv \
      --threads $TH --idle-spin $SP > $O/po.jsonl 2> $O/pe.txt ) 2> $O/feeder.txt &
  SMAX=$S0; AB=0
  while pidof feeder2 > /dev/null || pidof ov_bridge_0617 > /dev/null; do
    sleep 10
    s=$(skin); gt "$s" "$SMAX" && SMAX=$s
    if gt "$s" 38.0; then AB=1; echo "$(date +%T) $TAG ABORT SKIN $s" >> $ST; pkill feeder2; pkill ov_bridge_0617; sleep 2; fi
  done
  wait
  [ -f cfg/$CFG/ov_timing.txt ] && mv cfg/$CFG/ov_timing.txt $O/ov_timing.txt
  S1=$(skin); A1=$(ap)
  echo "tag=$TAG cfg=$CFG idle_spin=$SP threads=$TH start=$T0 end=$(date +%T) skin_start=$S0 skin_end=$S1 skin_max=$SMAX ap_start=$A0 ap_end=$A1 aborted=$AB" > $O/run.txt
  echo "$(date +%T) $TAG done SKIN $S1 max $SMAX aborted $AB frames $(grep -c . $O/tl.csv) $(cat $O/feeder.txt | tail -1)" >> $ST
done < $1
echo "ALL DONE $(date +%T) $1" >> $ST
pidof ov_bridge_0617 feeder2 || echo "no bridge/feeder left" >> $ST
