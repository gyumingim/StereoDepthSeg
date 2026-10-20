cd /home/system/scanner/StereoDepthSeg/scan3d
pkill -9 -f run_static.sh; sleep 1; echo "run_static left: $(pgrep -f run_static.sh)"
echo "$(date '+%F %T') run_static.sh killed (SIGKILL, no trap) so its pkill cannot stop logger2; logger1 ends itself at 1200 s" >> runs/gyro_bias_progress.log
HEAT=$(date -d "10:13:00" +%s)
nohup setsid bash tools/imu_shell/run_thermal.sh $HEAT > runs/static_imu_runner2.out 2>&1 < /dev/null &
sleep 12; tail -3 runs/gyro_bias_progress.log; cat runs/static_imu_runner2.out | tail -3
/mnt/c/Users/a3162/platform-tools/adb.exe -s 100.79.156.20:35723 shell "ps -A -o PID,ARGS | grep [I]muLog; dumpsys power | grep mWakefulness="
cat recordings/static_imu_*/temps_all.csv; date
