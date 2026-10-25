# ovupd 2026-10-26 phone replay plan (camera-free). Runner = copy of integ_20261025/phone/integ_run.sh (names -> ovupd).
#   bash ovupd_plan.sh prep | push | run LIST | status | pull
# Own dir /data/local/tmp/ovupd; reads /data/local/tmp/replay (raw80.bin, remap) read-only; feeder = copy of
# /data/local/tmp/integ/feeder4_integ as feeder4_ovupd. Phone lock (team/phone_lock.d) is taken by the caller.
set -euo pipefail
H=~/scanner/StereoDepthSeg-ovupd/scan3d
HERE=$H/runs/ovupd_20261026/phone
ST=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/ovupd/phone_stage
WST='C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\ovupd\phone_stage'
ADB="/mnt/c/Users/a3162/platform-tools/adb.exe -s 100.79.156.20:5555"
NDK=~/android-ndk-r27c
P=/data/local/tmp/ovupd
cmd=${1:?prep|push|run|status|pull}
case $cmd in
prep)
  rm -rf $ST; mkdir -p $ST/lib $ST/cfg
  $NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-strip -o $ST/ov_bridge_ovupd $H/ov_bridge/android/build_ovupd/ov_bridge
  cp $NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so $ST/lib/
  for s in 50 75 100; do
    c=c4s$s; [ $s = 50 ] && c=c4
    mkdir -p $ST/cfg/$c; cp $H/config/phone_ov_main/c4/*.yaml $ST/cfg/$c/
    sed -i "s#/storage/emulated/0/Android/data/com.scan3d.arlat/files/ov/c4/#$P/cfg/$c/#; s#^max_slam: 50#max_slam: $s#" $ST/cfg/$c/estimator_config.yaml
    sed -i "s#timeshift_cam_imu: 0.002172#timeshift_cam_imu: 0.0#" $ST/cfg/$c/kalibr_imucam_chain.yaml
    grep -H "^max_slam:" $ST/cfg/$c/estimator_config.yaml
  done
  cp $HERE/ovupd_run.sh $HERE/U*.txt $ST/
  sha256sum $ST/ov_bridge_ovupd $H/ov_bridge/android/build_ovupd/ov_bridge > $ST/binaries.sha256
  ls -la $ST; cat $ST/binaries.sha256 ;;
push)
  $ADB shell mkdir -p $P
  $ADB push "$WST\." $P/ | tail -1
  $ADB shell "cp /data/local/tmp/integ/feeder4_integ $P/feeder4_ovupd && chmod 755 $P/feeder4_ovupd $P/ov_bridge_* && ls -la $P" ;;
run)
  L=${2:?list name}
  $ADB shell "cd $P && mkdir -p out && nohup sh ovupd_run.sh $L.txt > out/nohup_$L.txt 2>&1 < /dev/null &"
  echo "started $L" ;;
status)
  $ADB shell "tail -5 $P/out/status.txt; pidof feeder4_ovupd ov_bridge_ovupd || echo idle" ;;
pull)
  L=${2:?list}; mkdir -p $HERE/out_$L
  for t in $(grep -v '^#' $HERE/$L.txt | awk 'NF{print $1}'); do
    mkdir -p $HERE/out_$L/$t
    for f in po.jsonl tl.csv prof.csv samp.txt run.txt pe.txt feeder.txt battery_start.txt battery_end.txt; do
      $ADB pull $P/out/$t/$f "$(wslpath -w $HERE/out_$L/$t)\$f" > /dev/null 2>&1 || true
    done
    [ -f $HERE/out_$L/$t/po.jsonl ] && gzip -f $HERE/out_$L/$t/po.jsonl
  done
  $ADB pull $P/out/status.txt "$(wslpath -w $HERE/out_$L)\status.txt" > /dev/null
  ls $HERE/out_$L ;;
esac
