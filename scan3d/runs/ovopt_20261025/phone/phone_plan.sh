# ovopt phone plan (2026-10-25) - PC side. NOT RUN: phone work needs the user's approval (coordinator, 2026-10-25).
#   bash phone_plan.sh prep                 # PC only: staging dir (binaries, libs, configs, lists) - safe to run any time
#   I_HAVE_USER_APPROVAL=yes bash phone_plan.sh push            # adb push the staging dir to /data/local/tmp/ovopt
#   I_HAVE_USER_APPROVAL=yes bash phone_plan.sh run L1_ident    # starts phone_run.sh with that list (nohup, returns)
#   bash phone_plan.sh status | pull L1_ident                   # progress / pull results to ./out_<list>/
# Needs: the replay study's files on the phone (read-only): /data/local/tmp/replay/{raw80.bin,feeder3,remap_1920x1080.bin}.
# Before any push/run: no other agent on the phone, screen off, phone on the fan, SKIN < 35 C (phone_run.sh waits).
set -euo pipefail
H=~/scanner/StereoDepthSeg-ovopt/scan3d
HERE=$H/runs/ovopt_20261025/phone
ST=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/ovopt/phone_stage
WST='C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\ovopt\phone_stage'
APK017=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/arlat_0.6.17.apk
ADB="/mnt/c/Users/a3162/platform-tools/adb.exe -s 100.79.156.20:5555"
NDK=~/android-ndk-r27c
P=/data/local/tmp/ovopt
need_ok() { [ "${I_HAVE_USER_APPROVAL:-}" = yes ] || { echo "phone step refused: set I_HAVE_USER_APPROVAL=yes only after the user approved phone work"; exit 3; }; }
cmd=${1:?prep|push|run|status|pull}
case $cmd in
prep)
  rm -rf $ST; mkdir -p $ST/lib $ST/cfg
  $NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-strip -o $ST/ov_bridge_ovopt $H/ov_bridge/android/build_ovopt/ov_bridge
  (cd $ST && unzip -o -q -j $APK017 lib/arm64-v8a/libov_bridge.so && mv libov_bridge.so ov_bridge_0617_ref)
  cp $NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so $ST/lib/
  for c in c4 c4s100 c4n400s100c20; do   # committed phone configs + replay-study change (timeshift 0: stream times are aligned)
    mkdir -p $ST/cfg/$c; cp $H/config/phone_ov_main/$c/*.yaml $ST/cfg/$c/
    sed -i "s#/storage/emulated/0/Android/data/com.scan3d.arlat/files/ov/$c/#$P/cfg/$c/#" $ST/cfg/$c/estimator_config.yaml
    sed -i "s#timeshift_cam_imu: 0.002172#timeshift_cam_imu: 0.0#" $ST/cfg/$c/kalibr_imucam_chain.yaml
  done
  mkdir -p $ST/cfg/c4s70; cp $ST/cfg/c4/*.yaml $ST/cfg/c4s70/                    # c4 with max_slam 70 (local variant)
  sed -i "s#^max_slam: 50#max_slam: 70#; s#$P/cfg/c4/#$P/cfg/c4s70/#" $ST/cfg/c4s70/estimator_config.yaml
  cp $HERE/phone_run.sh $HERE/L*.txt $ST/
  sha256sum $ST/ov_bridge_ovopt $ST/ov_bridge_0617_ref > $ST/binaries.sha256
  ls -la $ST $ST/cfg; cat $ST/binaries.sha256 ;;
push)
  need_ok
  $ADB shell mkdir -p $P
  $ADB push "$WST\\." $P/ | tail -1
  $ADB shell "cp /data/local/tmp/replay/feeder3 $P/feeder3_ovopt && chmod 755 $P/feeder3_ovopt $P/ov_bridge_* && ls -la $P" ;;
run)
  need_ok
  L=${2:?list name}
  $ADB shell "cd $P && mkdir -p out && SKIN_MAX_START=${SKIN_MAX_START:-36.0} AP_MAX_START=${AP_MAX_START:-200} MIN_COOL=${MIN_COOL:-0} nohup sh phone_run.sh $L.txt > out/nohup_$L.txt 2>&1 < /dev/null &"
  echo "started $L (start below SKIN ${SKIN_MAX_START:-36.0} / AP ${AP_MAX_START:-200} C, cool ${MIN_COOL:-0} s); poll: bash phone_plan.sh status" ;;
status)
  $ADB shell "tail -5 $P/out/status.txt" ;;
pull)   # adb.exe cannot write to \\wsl.localhost paths -> pull into the Windows stage dir, then copy into the worktree
  L=${2:?list name}; mkdir -p $ST/pull_$L $HERE/out_$L
  for t in $(awk '!/^#/ && NF {print $1}' $HERE/$L.txt); do
    $ADB pull $P/out/$t "$WST\\pull_$L" > /dev/null || echo "pull $t failed"
  done
  $ADB pull $P/out/status.txt "$WST\\pull_$L\\status.txt" > /dev/null
  cp -r $ST/pull_$L/. $HERE/out_$L/
  ls $HERE/out_$L ;;
esac
