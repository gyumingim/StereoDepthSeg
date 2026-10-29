#!/usr/bin/env bash
# usbrelay 0.6.38 phone smoke (2026-10-29): lifecycle only, phone on the desk (VIO will not init).
# Safety: start only SKIN < 40; monitor every 3 s: stop at SKIN >= 45 or AP >= 60 or battery <= 5 %.
set -u
A=/mnt/c/Users/a3162/platform-tools/adb.exe; S=100.79.156.20:39323; PKG=com.scan3d.arlat
O=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/usbrelay/smoke
OV=/sdcard/Android/data/$PKG/files/ov
a() { "$A" -s "$S" "$@" | tr -d '\r'; }
temps() { a shell 'dumpsys thermalservice' | sed -n '/Current temperatures from HAL/,/^[^ \t]/p' | grep -oE "mValue=[0-9.]+, mType=[0-9]+, mName=(SKIN|AP)," | sed -E 's/mValue=([0-9.]+).*mName=([A-Z]+),/\2 \1/' | sort -u -k1,1 | tr '\n' ' '; echo "batt $(a shell dumpsys battery | grep -m1 level | grep -oE '[0-9]+')"; }
skin_ok() { local t; t=$(temps); echo "temps: $t"; local s; s=$(echo "$t" | grep -oE "SKIN [0-9.]+" | cut -d' ' -f2); [ -n "$s" ] && awk -v s="$s" 'BEGIN{exit !(s < 40)}'; }
latest() { a shell "ls -1d $OV/run_* 2>/dev/null | tail -1"; }
camstate() { a shell 'dumpsys media.camera' | grep -iE "^ *(Active Camera Clients|Device [0-9]+ is open)" -A1 | head -4 | tr '\n' ' '; echo; }
post() {   # $1 tag $2 run dir
  local T=$1 R=$2 D=$O/$1
  mkdir -p "$D"
  echo "== $T run $R"
  [ -n "$R" ] && { a pull "$R" "$(wslpath -w "$D")" | tail -1; }
  [ -n "${NOLOG:-}" ] || a logcat -d -v time > "$D/logcat.txt"
  grep -E "arlat|ENN|NPU|ov_bridge|xfnpu" "$D/logcat.txt" | grep -vE "chatty" | tail -60 > "$D/logcat_arlat.txt"
  echo "ANR/freeze: $(grep -cE "ANR in $PKG|Input dispatching timed out|Application Not Responding" "$D/logcat.txt") anr, skipped-frames max $(grep -oE "Skipped [0-9]+ frames" "$D/logcat.txt" | grep -oE "[0-9]+" | sort -n | tail -1)"
  echo "ENN hw4 lines: $(grep -c "hw4" "$D/logcat.txt"), ENN lines $(grep -c ENN "$D/logcat.txt")"
  echo "camera: $(camstate)"
  echo "ov_bridge procs: $(a shell 'ps -A -o PID,NAME | grep -c ov_bridge')"
  echo "app pid: $(a shell pidof $PKG)"
  local M=$D/$(basename "$R")/meta.json
  if [ -f "$M" ]; then python3 - "$M" <<'PY'
import json,sys
d=json.load(open(sys.argv[1]))
keys=["app_version","npu_loop","run_valid","stop_reason","bridge_exit","bridge_killed_by_stop","bridge_grace_ms","npu_failed","viop_udp","viop_usb_result","frames","poses","secs"]
for k in keys:
    if k in d: print(f"  meta {k}: {json.dumps(d[k])[:300]}")
PY
  else echo "  no meta.json"; fi
  ls "$D/$(basename "$R")" 2>/dev/null | tr '\n' ' '; echo
}
watch_run() {   # $1 max s: wait for meta.json in a new run dir or app gone; safety stop
  local before=$1 max=$2 t=0 r
  while [ $t -lt $max ]; do
    sleep 3; t=$((t + 3))
    r=$(latest)
    if [ -n "$r" ] && [ "$r" != "$before" ] && a shell "[ -f $r/meta.json ]" && [ -z "$(a shell 'ps -A -o NAME | grep ov_bridge')" ]; then echo "meta.json after ${t}s"; sleep 2; return 0; fi
    local tt; tt=$(temps)
    s=$(echo "$tt" | grep -oE "SKIN [0-9.]+" | cut -d' ' -f2); p=$(echo "$tt" | grep -oE "AP [0-9.]+" | cut -d' ' -f2); b=$(echo "$tt" | grep -oE "batt [0-9]+" | cut -d' ' -f2)
    if awk -v s="${s:-99}" -v p="${p:-0}" -v b="${b:-0}" 'BEGIN{exit !(s >= 45 || p >= 60 || b <= 5)}'; then
      echo "SAFETY STOP $tt"; a shell input keyevent 3; sleep 8; a shell am force-stop $PKG; return 1; fi
  done
  echo "timeout ${max}s"; return 1
}
# secs hard cap does not fire with the cue on screen (status view detached, see README) -> stop with HOME after N s
runfor() { local n=$1; shift; a shell input keyevent KEYCODE_WAKEUP; start "$@"; sleep "$n"; a shell input keyevent 3; }
start() { a shell am start -n $PKG/.MainActivity --es mode ov --es ov_cam main --es ov_cfg c4f60 --es profile rec60 "$@" > /dev/null; }

case "${1:-}" in
  t1) skin_ok || exit 2; a shell am force-stop $PKG; a logcat -c; b=$(latest); runfor 20 --ei secs 20; watch_run "$b" 30; post t1_npu0 "$(latest)" ;;
  t2) skin_ok || exit 2; a shell am force-stop $PKG; a logcat -c; b=$(latest); runfor 30 --ei secs 30 --ei npu_loop 1; watch_run "$b" 30; post t2_npu1 "$(latest)" ;;
  t3) skin_ok || exit 2; a shell am force-stop $PKG; a logcat -c; b=$(latest); a shell input keyevent KEYCODE_WAKEUP
      # HOME on the phone itself as soon as the app logs "status: NPU ..." (NPU start window is ~0.3 s)
      a shell 'logcat -s arlat:I | (grep -m1 "status: NPU" > /dev/null; input keyevent 3; echo HOME_SENT; pkill -x logcat)' > $O/t3_home_trigger.txt &
      trig=$!; sleep 1; start --ei secs 30 --ei npu_loop 1; wait $trig; cat $O/t3_home_trigger.txt
      sleep 10; post t3_home "$(latest)"
      # t4: relaunch at once (the old process is still alive after HOME)
      a logcat -c; echo "app pid before relaunch: $(a shell pidof $PKG)"; b=$(latest); runfor 15 -f 0x10008000 --ei secs 15 --ei npu_loop 1; watch_run "$b" 30; post t4_relaunch "$(latest)" ;;
  t4b) # same process, NEW activity instance (clear task) right after t3: server from t3 must be gone -> clean start
      skin_ok || exit 2; a logcat -c; echo "app pid before: $(a shell pidof $PKG)"; b=$(latest)
      runfor 15 -f 0x10008000 --ei secs 15 --ei npu_loop 1; watch_run "$b" 30; post t4_relaunch "$(latest)" ;;
  t6) # 0.6.38b: secs 20 with the cue on screen, NO HOME -> the app must stop itself at ~20 s
      skin_ok || exit 2; a shell am force-stop $PKG; a logcat -c; b=$(latest); a shell input keyevent KEYCODE_WAKEUP; start --ei secs 20; watch_run "$b" 45; post t6_secscap "$(latest)" ;;
  t5) skin_ok || exit 2; a shell am force-stop $PKG; a logcat -c; b=$(latest); runfor 20 --ei secs 20 --es viop_udp usb; watch_run "$b" 30; post t5_usb "$(latest)" ;;
  post) post "$2" "$3"; exit 0 ;;
  *) echo "usage: t1|t2|t3|t5"; exit 2 ;;
esac
a shell am force-stop $PKG
echo "after force-stop camera: $(camstate) temps: $(temps)"
