# ovrobust phone plan (2026-10-25) - PC side. Brief: camera-free replay under the phone lock + stationary live runs.
#   bash phone_plan.sh prep          # PC only: staging dir (stripped binary, configs, lists)
#   bash phone_plan.sh push          # adb push staging -> /data/local/tmp/ovrobust (+ copies of feeder3 / libc++ from
#                                    #   /data/local/tmp/replay and /data/local/tmp/ovopt, read-only)
#   bash phone_plan.sh run LIST      # nohup phone_run.sh LIST.txt (returns)
#   bash phone_plan.sh status | pull LIST
# Configs: c4r = config/phone_ov_main/c4 with timeshift 0 (raw80 replay stream times are aligned, as the replay study),
#          c4l = c4 unchanged (timeshift 0.002172 = live app) for the stationary live dump (files/ov/live_stream.bin).
set -euo pipefail
H=~/scanner/StereoDepthSeg-ovrobust/scan3d
HERE=$H/runs/ovrobust_20261025/phone
ST=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/ovrobust/phone_stage
WST='C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\ovrobust\phone_stage'
ADB="/mnt/c/Users/a3162/platform-tools/adb.exe -s 100.79.156.20:5555"
NDK=~/android-ndk-r27c
P=/data/local/tmp/ovrobust
cmd=${1:?prep|push|run|status|pull}
case $cmd in
prep)
  rm -rf $ST; mkdir -p $ST/cfg
  $NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-strip -o $ST/ov_bridge_ovrobust $H/ov_bridge/android/build_ovrobust/ov_bridge
  for c in c4r c4l; do
    mkdir -p $ST/cfg/$c; cp $H/config/phone_ov_main/c4/*.yaml $ST/cfg/$c/
    sed -i "s#/storage/emulated/0/Android/data/com.scan3d.arlat/files/ov/c4/#$P/cfg/$c/#" $ST/cfg/$c/estimator_config.yaml
  done
  sed -i "s#timeshift_cam_imu: 0.002172#timeshift_cam_imu: 0.0#" $ST/cfg/c4r/kalibr_imucam_chain.yaml
  cp $HERE/phone_run.sh $HERE/P*.txt $ST/
  sha256sum $ST/ov_bridge_ovrobust > $ST/binaries.sha256
  grep -H timeshift $ST/cfg/*/kalibr_imucam_chain.yaml; ls -la $ST; cat $ST/binaries.sha256 ;;
push)
  $ADB shell mkdir -p $P/lib
  $ADB push "$WST\." $P/ | tail -1
  $ADB shell "cp /data/local/tmp/replay/feeder3 $P/feeder3_ovrobust && cp /data/local/tmp/ovopt/lib/libc++_shared.so $P/lib/ && chmod 755 $P/feeder3_ovrobust $P/ov_bridge_ovrobust && ls -la $P $P/lib" ;;
run)
  L=${2:?list name}
  $ADB shell "cd $P && mkdir -p out && SKIN_MAX_START=${SKIN_MAX_START:-36.0} AP_MAX_START=${AP_MAX_START:-200} MIN_COOL=${MIN_COOL:-0} nohup sh phone_run.sh $L.txt > out/nohup_$L.txt 2>&1 < /dev/null &"
  echo "started $L" ;;
status)
  $ADB shell "tail -5 $P/out/status.txt" ;;
pull)   # whole run dirs into the Windows stage dir (adb.exe), then copy without the raw po.jsonl (po.jsonl.gz kept)
  L=${2:?list name}; mkdir -p $ST/pull_$L $HERE/out_$L
  for t in $(awk '!/^#/ && NF {print $1}' $HERE/$L.txt); do
    $ADB shell "cd $P/out/$t && gzip -kf po.jsonl" || true
    $ADB pull $P/out/$t "$WST\pull_$L" > /dev/null || echo "pull $t failed"
    mkdir -p $HERE/out_$L/$t; (cd $ST/pull_$L/$t && cp $(ls | grep -v '^po.jsonl$') $HERE/out_$L/$t/)
  done
  $ADB pull $P/out/status.txt "$WST\pull_$L\status.txt" > /dev/null; cp $ST/pull_$L/status.txt $HERE/out_$L/
  ls $HERE/out_$L ;;
esac
