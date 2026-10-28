#!/bin/bash
# heat test monitor: every 2 s one adb shell -> epoch, AP, SKIN, battery level, current now (mA), AC/USB; writes CSV.
# Also writes STOP file when AP>=50 / SKIN>=45 / battery<20 (runner checks it).
export MSYS_NO_PATHCONV=1
A="adb -s 100.79.156.20:33841"; OUT=$1; STOPF=$2
echo "pc_epoch,ap,skin,level,current_mA,ac,usb" > $OUT
while [ ! -f ${STOPF}.end ]; do
  r=$($A shell 'dumpsys thermalservice | grep -A8 "Current temperatures from HAL" | grep -E "mName=(AP|SKIN)"; dumpsys battery | grep -E "^  (AC powered|USB powered|level|current now):"' 2>/dev/null | tr -d '\r')
  ap=$(echo "$r" | grep "mName=AP" | sed -E 's/.*mValue=([0-9.]+).*/\1/'); sk=$(echo "$r" | grep "mName=SKIN" | sed -E 's/.*mValue=([0-9.]+).*/\1/')
  lv=$(echo "$r" | grep "level:" | tr -dc '0-9'); cu=$(echo "$r" | grep "current now:" | sed -E 's/.*: *(-?[0-9]+).*/\1/')
  ac=$(echo "$r" | grep "AC powered" | awk '{print $3}'); us=$(echo "$r" | grep "USB powered" | awk '{print $3}')
  echo "$(date +%s.%N | cut -c1-14),$ap,$sk,$lv,$cu,$ac,$us" >> $OUT
  if [ -n "$ap" ] && awk "BEGIN{exit !($ap >= 50 || $sk >= 45 || $lv < 20)}"; then [ -f $STOPF ] || { touch $STOPF; $A shell am force-stop com.scan3d.nncprobe; echo "$(date +%s) LIMIT force-stop" >> ${STOPF}.log; }; fi
  sleep 2
done
