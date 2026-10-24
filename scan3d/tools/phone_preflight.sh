#!/usr/bin/env bash
# Pre-run cleanup + checks for every on-phone run (2026-10-24). Source it, then
#   phone_preflight <adb> <ip:port> [max_skin_c=36.0]      -> prints a log; returns 0 = ok, 2 = do not start
# One adb shell call (fast, ~1-2 s over Wi-Fi):
#   - am kill-all (background cached processes only; never the foreground app)
#   - force-stop the known heavy apps that are running (camera, gallery, photos, video, browsers, YouTube) -
#     force-stop only, never uninstall / disable; our own app is NOT touched here (the run scripts handle it)
#   - MemTotal / MemAvailable (/proc/meminfo), thermal status and SKIN (dumpsys thermalservice, HAL section)
# PREFLIGHT_DRY=1: no kill / force-stop, only report (for testing while another run may be using the phone).
# Refuses (returns 2) when SKIN cannot be read or SKIN >= max_skin_c (the existing 36 C start rule).
# Used by scan3d/tools/ov_phone_run.sh (WSL) and the std60 recording script rec60.sh (Git Bash): both shells work.
PREFLIGHT_HEAVY="com.sec.android.app.camera com.sec.android.gallery3d com.google.android.apps.photos \
com.samsung.android.video com.google.android.youtube com.android.chrome com.sec.android.app.sbrowser"

phone_preflight() {
  local A=$1 S=$2 lim=${3:-36.0} t0 t1 out skin ts
  t0=$(date +%s%3N)
  # device-side script (single quotes: expanded on the phone, not here)
  local kill='am kill-all;' stop='am force-stop $p;' verb=force-stopped
  [ -n "${PREFLIGHT_DRY:-}" ] && { kill=''; stop=''; verb='DRY would force-stop'; }   # test: report only
  local dev="$kill"' for p in '"$PREFLIGHT_HEAVY"'; do pid=$(pidof $p) && { '"$stop"' echo "'"$verb"' $p (pid $pid)"; }; done;
echo "--"; grep -E "^(MemTotal|MemAvailable):" /proc/meminfo;
dumpsys thermalservice > /data/local/tmp/pf_th.txt; grep -m1 "Thermal Status" /data/local/tmp/pf_th.txt;
sed -n "/Current temperatures from HAL/,\$p" /data/local/tmp/pf_th.txt | grep "mValue=.*mName=SKIN" | tail -1; rm -f /data/local/tmp/pf_th.txt'
  dev=${dev//$'\n'/ }            # one line for adb shell
  out=$(MSYS_NO_PATHCONV=1 "$A" -s "$S" shell "$dev" 2>&1 | tr -d '\r')
  t1=$(date +%s%3N)
  # only Temperature{mValue=..} lines: the static TemperatureThreshold{.. mName=SKIN ..} line has no value
  # (2026-10-24 bug: it let a 38.0 C phone pass); the value must be a number
  skin=$(echo "$out" | grep -E "mValue=[0-9.]+,.*mName=SKIN" | sed -E 's/.*mValue=([0-9.]+).*/\1/' | tail -1)
  [[ "$skin" =~ ^[0-9]+([.][0-9]+)?$ ]] || skin=""
  ts=$(echo "$out" | grep -m1 "Thermal Status" | grep -oE "[0-9]+" | head -1)
  echo "preflight $(date '+%Y-%m-%d %H:%M:%S') device $S ($((t1 - t0)) ms)"
  echo "kill-all: $([ -n "${PREFLIGHT_DRY:-}" ] && echo "skipped (DRY)" || echo done); heavy apps checked: $PREFLIGHT_HEAVY"
  echo "$out" | grep -E "force-stop|^Mem" | sed 's/^/  /'
  echo "$out" | grep -q "force-stop" || echo "  no heavy app was running"
  echo "thermal status: ${ts:-?}  SKIN: ${skin:-?} C (limit $lim C)"
  if [ -z "$skin" ]; then
    echo "PREFLIGHT REFUSED: cannot read SKIN from dumpsys thermalservice (raw output follows)"; echo "$out"; return 2
  fi
  if awk -v x="$skin" -v y="$lim" 'BEGIN{exit !(x + 0 >= y + 0)}'; then
    echo "PREFLIGHT REFUSED: phone too warm (SKIN $skin >= $lim C): cool it first"; return 2
  fi
  echo "PREFLIGHT OK"
  return 0
}
