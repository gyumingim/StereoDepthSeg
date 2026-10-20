cd /home/system/scanner/StereoDepthSeg/scan3d
A=/mnt/c/Users/a3162/platform-tools/adb.exe; S=100.79.156.20:35723; R=$(cat runs/static_imu_current.txt); TS=${R##*_imu_}
P=runs/static_imu_preview; mkdir -p $P; cp $R/temps.csv $R/temps_all.csv $R/mag.csv $P/ 2>/dev/null
$A -s $S pull /data/local/tmp/imu_static2_$TS.csv $P/imu2.csv >/dev/null 2>&1
$A -s $S pull /data/local/tmp/imu_static_$TS.csv $P/imu.csv >/dev/null; $A -s $S pull /data/local/tmp/imulog_$TS.out $P/logger_out.txt >/dev/null
tail -3 runs/gyro_bias_progress.log
../venv_ffs/bin/python tools/imu_shell/analyze_static.py $P --preview $EXTRA
