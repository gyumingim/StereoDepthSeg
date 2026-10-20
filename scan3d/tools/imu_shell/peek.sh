cd /home/system/scanner/StereoDepthSeg/scan3d
grep -n "hs\b\|\"hs\"\|'hs'\|mode.*hs\|force-stop\|am start\|max_temp\|rec_dir\|hs_" tools/vio_bench_server.py | head -50
A=/mnt/c/Users/a3162/platform-tools/adb.exe; S=100.79.156.20:35723
$A -s $S shell 'ls /sys/class/thermal/thermal_zone0/; cat /sys/class/thermal/thermal_zone0/temp; dumpsys activity activities | grep -E "mResumedActivity|topResumed" | head; pidof com.scan3d.arlat com.camera.dualstream'
tail -4 runs/gyro_bias_progress.log
