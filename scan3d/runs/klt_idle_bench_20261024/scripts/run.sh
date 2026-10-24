# usage: run.sh OUTDIR PLANFILE ; plan lines: TAG|ARGS   (ARGS appended to klt_bench_idle f200.u8 960 540 200 --klt-only --overlap 0)
# Temperatures: dumpsys thermalservice "Current temperatures from HAL" SKIN and AP (the first SKIN line of dumpsys is the
# *cached* value, which stayed at 35.9 for the whole phase-1 run = stale). Run starts only if SKIN < 36; >= 38 -> cool 120 s.
export MSYS_NO_PATHCONV=1
A="/c/Users/a3162/platform-tools/adb.exe -s 100.79.156.20:5555"; adb() { $A "$@" < /dev/null; }
OUT=$1; PLAN=$2; mkdir -p $OUT; exec 3< $PLAN
PD=/data/local/tmp/kb0615/idle
adb shell "mkdir -p $PD"
hal() { adb shell "dumpsys thermalservice | sed -n '/Current temperatures from HAL/,\$p' | grep -m1 'mName=$1,'" | sed -E 's/.*mValue=([0-9.]+).*/\1/' | tr -d '\r'; }
while IFS="|" read -r TAG ARGS <&3; do
  [ -z "$TAG" ] && continue; case "$TAG" in \#*) continue;; esac
  while :; do s=$(hal SKIN); awk -v s="$s" 'BEGIN{exit !(s<36.0)}' && break; echo "$(date +%T) wait SKIN=$s" | tee -a $OUT/log.txt; sleep 20; done
  s0=$(hal SKIN); a0=$(hal AP); t0=$(date +%T)
  adb shell "cd /data/local/tmp/kb0615 && LD_LIBRARY_PATH=. ./klt_bench_idle f200.u8 960 540 200 --klt-only --overlap 0 $ARGS --freq-log $PD/${TAG}_freq.txt --frame-log $PD/${TAG}_frames.txt" > $OUT/${TAG}.txt 2>&1
  s1=$(hal SKIN); a1=$(hal AP)
  echo "start $t0 SKIN_before $s0 SKIN_after $s1 AP_before $a0 AP_after $a1 (HAL current) args: $ARGS" >> $OUT/${TAG}.txt
  adb pull $PD/${TAG}_freq.txt "$(cygpath -w $OUT)" >/dev/null; adb pull $PD/${TAG}_frames.txt "$(cygpath -w $OUT)" >/dev/null
  line=$(grep -m1 "^  T" $OUT/${TAG}.txt); h=$(grep -oE '[0-9a-f]{16}  \(get' $OUT/${TAG}.txt | cut -c1-16)
  echo "$t0 $TAG SKIN $s0->$s1 AP $a0->$a1 hash $h |$line" | tee -a $OUT/log.txt
  awk -v s="$s1" 'BEGIN{exit !(s>=38.0)}' && { echo "SKIN>=38, cooling" | tee -a $OUT/log.txt; sleep 120; }
done
echo DONE | tee -a $OUT/log.txt
