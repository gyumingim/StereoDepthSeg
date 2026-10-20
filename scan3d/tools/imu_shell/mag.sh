#!/usr/bin/env bash
# 30 s magnetometer(2) + uncal mag(14) + rotation vector(11) + game rotvec(15) log at 50 Hz (heading / tilt for Earth-rate)
cd /home/system/scanner/StereoDepthSeg/scan3d
A=/mnt/c/Users/a3162/platform-tools/adb.exe; S=100.79.156.20:35723; R=$(cat runs/static_imu_current.txt)
$A -s $S push tools/imu_shell/imulog.dex /data/local/tmp/imulog2.dex >/dev/null
timeout 80 $A -s $S shell "CLASSPATH=/data/local/tmp/imulog2.dex app_process / ImuLog /data/local/tmp/mag.csv 30 2,14,11,15" | grep -v "^t "
$A -s $S pull /data/local/tmp/mag.csv "$R/mag.csv"
$A -s $S shell "rm /data/local/tmp/mag.csv /data/local/tmp/imulog2.dex; ps -A -o PID,ARGS | grep [I]muLog"
echo "$(date '+%F %T') mag/rotvec 30 s logged to $R/mag.csv" >> runs/gyro_bias_progress.log
