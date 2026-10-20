#!/usr/bin/env bash
# 20-min static IMU-only recording (no camera, no app): shell-user app_process logger + temperature log every 30 s.
# usage: run_static.sh SECS   (run detached from WSL). Always kills the phone logger on exit.
SECS=${1:-1200}
cd /home/system/scanner/StereoDepthSeg/scan3d
A=/mnt/c/Users/a3162/platform-tools/adb.exe; S=100.79.156.20:35723
TS=$(date +%Y%m%d_%H%M%S); OUT=recordings/static_imu_$TS; mkdir -p "$OUT"
LOG=runs/gyro_bias_progress.log; PH=/data/local/tmp/imu_static_$TS.csv
echo "$OUT" > runs/static_imu_current.txt
note(){ echo "$(date '+%F %T') $*" >> "$LOG"; }
stop_phone(){
  $A -s $S shell 'pkill -f ImuLog; sleep 1; ps -A -o PID,ARGS | grep -c "[I]muLog"' > "$OUT/stop_check.txt" 2>&1
  note "phone logger stop check (0 = stopped): $(tr -d '\r' < "$OUT/stop_check.txt")"
}
trap 'stop_phone' EXIT
$A -s $S push tools/imu_shell/imulog.dex /data/local/tmp/imulog.dex >/dev/null
$A -s $S shell "nohup sh -c 'CLASSPATH=/data/local/tmp/imulog.dex app_process / ImuLog $PH $SECS' > /data/local/tmp/imulog_$TS.out 2>&1 &"
sleep 3
note "RECORDING STARTED $(date '+%F %T') -> $OUT (phone $PH, $SECS s)"
echo "pc_unix,phone_uptime_s,battery_dC,AP,BAT,CP,PA,SKIN,USB" > "$OUT/temps.csv"
t0=$(date +%s); i=0
while :; do
  r=$($A -s $S shell 'cat /proc/uptime; dumpsys battery | grep "  temperature:"; dumpsys thermalservice | sed -n "/Current temperatures from HAL/,/Current cooling/p"' 2>/dev/null | tr -d '\r')
  up=$(echo "$r" | head -1 | cut -d" " -f1); bt=$(echo "$r" | grep "temperature:" | head -1 | awk '{print $2}')
  g(){ echo "$r" | grep "mName=$1," | head -1 | sed 's/.*mValue=\([-0-9.]*\).*/\1/'; }
  echo "$(date +%s.%N | cut -c1-14),$up,$bt,$(g AP),$(g BAT),$(g CP),$(g PA),$(g SKIN),$(g USB)" >> "$OUT/temps.csv"
  alive=$($A -s $S shell 'ps -A -o ARGS | grep -c "[I]muLog"' 2>/dev/null | tr -d '\r')
  el=$(( $(date +%s) - t0 ))
  [ $((i % 4)) -eq 0 ] && note "t=${el}s battery=${bt} logger_alive=${alive} $( $A -s $S shell "wc -l < $PH" 2>/dev/null | tr -d '\r') lines"
  i=$((i+1))
  if [ "$alive" = "0" ] && [ $el -gt 10 ]; then break; fi
  if [ $el -gt $((SECS + 60)) ]; then note "timeout, stopping"; break; fi
  sleep 30
done
stop_phone; trap - EXIT
$A -s $S pull $PH "$OUT/imu.csv" >/dev/null && $A -s $S pull /data/local/tmp/imulog_$TS.out "$OUT/logger_out.txt" >/dev/null
note "pulled $(wc -l < "$OUT/imu.csv") lines to $OUT/imu.csv; logger: $(tail -2 "$OUT/logger_out.txt" | tr '\n' ' ')"
$A -s $S shell "rm -f $PH /data/local/tmp/imu_test.csv"
note "RECORDING DONE $OUT"
