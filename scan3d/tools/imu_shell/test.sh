#!/usr/bin/env bash
cd "$(dirname "$0")"
A=/mnt/c/Users/a3162/platform-tools/adb.exe; S=100.79.156.20:35723
$A -s $S push imulog.dex /data/local/tmp/imulog.dex
timeout 60 $A -s $S shell 'CLASSPATH=/data/local/tmp/imulog.dex app_process / ImuLog /data/local/tmp/imu_test.csv 10' 2>&1 | tail -20
$A -s $S shell 'wc -l /data/local/tmp/imu_test.csv; cut -d, -f1 /data/local/tmp/imu_test.csv | sort | uniq -c; ps -A | grep app_process'
