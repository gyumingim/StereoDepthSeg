# lens05 phone step: (1) move the old f2.22 config set into files/ov/bak_f222_20261024/ (renamed, not deleted), (2) push the
# new 0.5 dpt set, (3) install 0.6.29. Only when: phone lock free, no camera client, SKIN < 36, no ov_bridge/app running,
# and the user is not using the phone (screen off, or launcher/notification shade in front). Never starts the camera/app.
A=/c/Users/a3162/platform-tools/adb.exe; S=100.79.156.20:5555
SP=/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad
L=$SP/team/phone_lock.d
NEW='C:/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/lens05/push'
APK='C:/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/lens05/apk/arlat_0.6.29.apk'
OV=/sdcard/Android/data/com.scan3d.arlat/files/ov
BAK=$OV/bak_f222_20261024
ITEMS="base60 c2 c4 c4c20 c4f60 c4n400 c4n400s100c20 c4s100 c5 c6 c7 c8 fps30 fps30n100c8 n100 n100c8 n150 remap_1920x1080.bin remap_640x360.bin variants.json"
adb_() { MSYS_NO_PATHCONV=1 $A -s $S "$@"; }
for i in $(seq 1 ${TRIES:-30}); do
  st=$(adb_ shell "dumpsys power | grep -m1 mWakefulness=; dumpsys window | grep -m1 mCurrentFocus; dumpsys media.camera | grep -A1 'Active Camera Clients' | tail -1; dumpsys thermalservice | grep -A40 'Current temperatures from HAL' | grep -m1 'mName=SKIN' | sed 's/.*mValue=\([0-9.]*\).*/SKIN=\1/'; echo procs=\$(ps -A | grep -c 'ov_bridge\|com.scan3d.arlat')" 2>/dev/null | tr -d '\r' | tr '\n' ' ')
  echo "$(date +%T) $st"
  skin=$(echo "$st" | sed -n 's/.*SKIN=\([0-9.]*\).*/\1/p')
  idle=0; if ! echo "$st" | grep -q "Awake" || echo "$st" | grep -qi "launcher\|NotificationShade\|mCurrentFocus=null"; then idle=1; fi
  cam_ok=0; echo "$st" | grep -q "Clients: *\[\] \|  *\[\] " && cam_ok=1
  echo "$st" | grep -q "\[\]" && cam_ok=1
  procs=$(echo "$st" | sed -n 's/.*procs=\([0-9]*\).*/\1/p')
  if [ "$idle" = 1 ] && [ "$cam_ok" = 1 ] && [ "${procs:-9}" = 0 ] && awk "BEGIN{exit !(${skin:-99} < 36)}"; then
    if mkdir $L 2>/dev/null; then
      echo "lens05 $(date '+%F %T') config push + install 0.6.29 (no runs, no camera)" > $L/owner.txt
      adb_ shell "ls -d $BAK" 2>/dev/null | grep -q bak_ && { echo "backup dir exists - stop"; rm $L/owner.txt; rmdir $L; exit 1; }
      adb_ shell "mkdir $BAK && cd $OV && for x in $ITEMS; do mv \$x $BAK/; done; ls $BAK | tr '\n' ' '"; echo
      adb_ push "$NEW/." $OV/ | tail -1
      adb_ shell "cd $OV && sha256sum remap_1920x1080.bin remap_640x360.bin c4/remap_1920x1080.bin c4f60/remap_1920x1080.bin c4f60/kalibr_imucam_chain.yaml c4/kalibr_imucam_chain.yaml variants.json; grep intrinsics c4f60/kalibr_imucam_chain.yaml c4/kalibr_imucam_chain.yaml base60/kalibr_imucam_chain.yaml"
      adb_ install -r "$APK"
      echo "installed at $(date '+%F %T')"
      adb_ shell "dumpsys package com.scan3d.arlat | grep -m2 'versionCode\|versionName'; dumpsys media.camera | grep -A1 'Active Camera Clients'; ps -A | grep -c 'ov_bridge\|com.scan3d.arlat'"
      grep -q "^lens05 " $L/owner.txt && rm $L/owner.txt && rmdir $L && echo "lock released"
      exit 0
    else echo "lock busy: $(cat $L/owner.txt 2>/dev/null)"; fi
  fi
  sleep 60
done
echo "gave up (phone busy / in use / warm)"; exit 2
