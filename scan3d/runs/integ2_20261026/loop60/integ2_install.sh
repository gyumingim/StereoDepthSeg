A=/c/Users/a3162/platform-tools/adb.exe; S=100.79.156.20:5555
SP=/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad
L=$SP/team/phone_lock.d
for i in $(seq 1 25); do
  st=$(MSYS_NO_PATHCONV=1 $A -s $S shell 'dumpsys power | grep -m1 "mWakefulness="; dumpsys window | grep -m1 mCurrentFocus' 2>/dev/null)
  echo "$(date +%T) $st" | tr '\n' ' '; echo
  if ! echo "$st" | grep -q "Awake" || echo "$st" | grep -qi "launcher\|NotificationShade\|null"; then
    if mkdir $L 2>/dev/null; then
      echo "integ2 $(date '+%F %T') install 0.6.28 (no runs)" > $L/owner.txt
      MSYS_NO_PATHCONV=1 $A -s $S install -r "C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\integ2\apk\arlat_0.6.28.apk"
      echo "installed at $(date '+%F %T')"
      MSYS_NO_PATHCONV=1 $A -s $S shell 'dumpsys package com.scan3d.arlat | grep -m2 "versionCode\|versionName"; dumpsys media.camera | grep -A1 "Active Camera Clients"; ps -A | grep -c "ov_bridge\|com.scan3d.arlat"' 2>/dev/null
      grep -q "^integ2 " $L/owner.txt && rm $L/owner.txt && rmdir $L && echo "lock released"
      exit 0
    else echo "lock busy: $(cat $L/owner.txt 2>/dev/null)"; fi
  fi
  sleep 60
done
echo "gave up (user still on phone)"
