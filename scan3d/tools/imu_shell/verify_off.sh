A=/mnt/c/Users/a3162/platform-tools/adb.exe; S=100.79.156.20:35723
echo "ImuLog procs: $($A -s $S shell 'ps -A -o ARGS | grep -c [I]muLog' | tr -d '\r')"
echo "arlat pid: '$($A -s $S shell pidof com.scan3d.arlat | tr -d '\r')'"
echo "screen_off_timeout: $($A -s $S shell settings get system screen_off_timeout | tr -d '\r')"
echo "hs folders: $($A -s $S shell 'ls -d /sdcard/Android/data/com.scan3d.arlat/files/hs_* 2>/dev/null' | tr -d '\r' | tr '\n' ' ')"
