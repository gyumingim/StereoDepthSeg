#!/usr/bin/env bash
# Continue the static session: 2nd IMU logger (long), screen kept awake (doze gaps), temps every ~15 s, then slow stepwise
# heating with 240 fps hs recording (arlat, same intent as vio_bench_server hs_start), then cool-down.
# Safety: camera off at battery >= 42 C, thermal status >= 1, or app auto-stop. Always on exit: camera off, loggers killed,
# screen timeout restored. Deletes only hs_ folders created by this script.
# usage: run_thermal.sh HEAT_AT_EPOCH
HEAT_AT=$1; REC_S=${REC_S:-180}; PAUSE_S=${PAUSE_S:-120}; CYCLES=${CYCLES:-5}; COOL_S=${COOL_S:-900}; MAXDC=${MAXDC:-420}
cd /home/system/scanner/StereoDepthSeg/scan3d
A=/mnt/c/Users/a3162/platform-tools/adb.exe; S=100.79.156.20:35723; PKG=com.scan3d.arlat; BASE=/sdcard/Android/data/$PKG/files
R=$(cat runs/static_imu_current.txt); TS=${R##*_imu_}; LOG=runs/gyro_bias_progress.log
PH2=/data/local/tmp/imu_static2_$TS.csv
note(){ echo "$(date '+%F %T') $*" >> "$LOG"; }
CAM=0
TIMEOUT0=$($A -s $S shell settings get system screen_off_timeout | tr -d '\r')
echo "$TIMEOUT0" > "$R/screen_off_timeout_orig.txt"
cam_off(){ $A -s $S shell input keyevent KEYCODE_HOME >/dev/null 2>&1; sleep 3; $A -s $S shell am force-stop $PKG >/dev/null 2>&1; CAM=0
  note "camera off (arlat pid now: '$($A -s $S shell pidof $PKG | tr -d '\r')')"; }
cleanup(){ cam_off
  $A -s $S shell 'pkill -f ImuLog; sleep 1; ps -A -o ARGS | grep -c "[I]muLog"' > "$R/stop_check2.txt" 2>&1
  $A -s $S shell settings put system screen_off_timeout "${TIMEOUT0:-30000}"
  note "loggers stop check (0 = stopped): $(tr -d '\r' < "$R/stop_check2.txt"); screen_off_timeout restored to $($A -s $S shell settings get system screen_off_timeout | tr -d '\r')"; }
trap cleanup EXIT
$A -s $S shell "ls -d $BASE/hs_* 2>/dev/null" | tr -d '\r' | sort > "$R/hs_before.txt"
$A -s $S shell "nohup sh -c 'CLASSPATH=/data/local/tmp/imulog.dex app_process / ImuLog $PH2 5400' > /data/local/tmp/imulog2_$TS.out 2>&1 &"
$A -s $S shell settings put system screen_off_timeout 10800000
$A -s $S shell input keyevent KEYCODE_WAKEUP
sleep 2; note "logger2 started ($PH2); screen awake (timeout ${TIMEOUT0} -> 3 h); heating at $(date -d @$HEAT_AT '+%T'), rec ${REC_S}s pause ${PAUSE_S}s x$CYCLES, cool ${COOL_S}s, stop at ${MAXDC} dC"
echo "pc_unix,phone_uptime_s,battery_dC,thermal_status,cam_on,AP,BAT,CP,PA,SKIN,USB,wake" > "$R/temps_all.csv"
sample(){
  r=$($A -s $S shell 'cat /proc/uptime; dumpsys battery | grep "  temperature:"; dumpsys thermalservice | sed -n "/^Thermal Status/p;/Current temperatures from HAL/,/Current cooling/p"; dumpsys power | grep mWakefulness=' 2>/dev/null | tr -d '\r')
  UP=$(echo "$r" | head -1 | cut -d" " -f1); BT=$(echo "$r" | grep "temperature:" | head -1 | awk '{print $2}')
  TH=$(echo "$r" | grep "^Thermal Status" | awk '{print $3}')
  WK=$(echo "$r" | grep mWakefulness= | head -1 | sed 's/.*=//')
  g(){ echo "$r" | grep "mName=$1," | head -1 | sed 's/.*mValue=\([-0-9.]*\).*/\1/'; }
  echo "$(date +%s.%N | cut -c1-14),$UP,$BT,$TH,$CAM,$(g AP),$(g BAT),$(g CP),$(g PA),$(g SKIN),$(g USB),$WK" >> "$R/temps_all.csv"
  [ "$WK" != "Awake" ] && $A -s $S shell input keyevent KEYCODE_WAKEUP >/dev/null 2>&1
}
alive(){ $A -s $S shell 'ps -A -o ARGS | grep -c "[I]muLog"' 2>/dev/null | tr -d '\r'; }
wait_until(){ local n=0; while [ $(date +%s) -lt $1 ]; do sample; n=$((n+1)); [ $((n % 8)) -eq 0 ] && note "t $(date +%T) battery=$BT thermal=$TH cam=$CAM wake=$WK loggers=$(alive)"; sleep 13; done; }
wait_until $HEAT_AT
STOPPED=""
for c in $(seq 1 $CYCLES); do
  sample; [ "${BT:-0}" -ge $MAXDC ] && { STOPPED="battery $BT before cycle $c"; break; }
  [ "${TH:-0}" -ge 1 ] && { STOPPED="thermal status $TH before cycle $c"; break; }
  $A -s $S shell input keyevent KEYCODE_WAKEUP >/dev/null
  $A -s $S shell am start -n $PKG/.MainActivity --es mode hs --ei fps 240 --ef focus_dpt 2.2222 --ei max_temp_dc $MAXDC >/dev/null
  CAM=1; note "HEAT cycle $c: camera ON (240 fps hs), battery $BT"
  end=$(( $(date +%s) + REC_S ))
  while [ $(date +%s) -lt $end ]; do
    sleep 7; sample
    if [ "${BT:-0}" -ge $MAXDC ] || [ "${TH:-0}" -ge 1 ]; then STOPPED="battery $BT / thermal $TH in cycle $c"; break; fi
    if [ -z "$($A -s $S shell pidof $PKG | tr -d '\r')" ]; then STOPPED="app stopped by itself in cycle $c"; break; fi
  done
  cam_off; note "HEAT cycle $c: camera OFF, battery $BT thermal $TH ${STOPPED}"
  [ -n "$STOPPED" ] && break
  [ $c -lt $CYCLES ] && wait_until $(( $(date +%s) + PAUSE_S ))
done
[ -n "$STOPPED" ] && note "heating stopped early: $STOPPED"
note "COOL-DOWN start ${COOL_S}s, battery $BT"
wait_until $(( $(date +%s) + COOL_S ))
note "COOL-DOWN end, battery $BT, loggers alive $(alive)"
cleanup; trap - EXIT
$A -s $S pull $PH2 "$R/imu2.csv" >/dev/null; $A -s $S pull /data/local/tmp/imulog2_$TS.out "$R/logger2_out.txt" >/dev/null
P1=/data/local/tmp/imu_static_$TS.csv
[ -f "$R/imu.csv" ] || $A -s $S pull $P1 "$R/imu.csv" >/dev/null
[ -f "$R/logger_out.txt" ] || $A -s $S pull /data/local/tmp/imulog_$TS.out "$R/logger_out.txt" >/dev/null
note "pulled imu2.csv $(wc -l < "$R/imu2.csv") lines, imu.csv $(wc -l < "$R/imu.csv") lines"
$A -s $S shell "ls -d $BASE/hs_* 2>/dev/null" | tr -d '\r' | sort > "$R/hs_after.txt"
NEW=$(comm -13 "$R/hs_before.txt" "$R/hs_after.txt")
for d in $NEW; do $A -s $S shell rm -rf "$d"; done
note "deleted heating-only hs folders: $(echo $NEW) ; still present: '$( [ -n "$NEW" ] && $A -s $S shell "ls -d $NEW 2>/dev/null" | tr -d '\r' | tr '\n' ' ')'"
$A -s $S shell "rm -f $PH2 $P1 /data/local/tmp/imu_test.csv"
note "THERMAL SESSION DONE $R"
