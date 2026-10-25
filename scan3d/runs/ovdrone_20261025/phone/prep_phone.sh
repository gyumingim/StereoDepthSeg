# ovdrone phone staging (PC only): bash prep_phone.sh STAGE_DIR (bridge build android/build_ovdrone, c4 config, runner, lists)
set -euo pipefail
H=~/scanner/StereoDepthSeg-ovdrone/scan3d
RD=$H/runs/ovdrone_20261025
ST=$1
NDK=~/android-ndk-r27c
P=/data/local/tmp/ovdrone
rm -rf "$ST"; mkdir -p $ST/lib $ST/cfg/c4
$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-strip -o $ST/ov_bridge_ovdrone $H/ov_bridge/android/build_ovdrone/ov_bridge
cp $NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so $ST/lib/
cp $H/config/phone_ov_main/c4/*.yaml $ST/cfg/c4/
sed -i "s#/storage/emulated/0/Android/data/com.scan3d.arlat/files/ov/c4/#$P/cfg/c4/#" $ST/cfg/c4/estimator_config.yaml
sed -i "s#timeshift_cam_imu: 0.002172#timeshift_cam_imu: 0.0#" $ST/cfg/c4/kalibr_imucam_chain.yaml
sed -e "s#/data/local/tmp/ovopt#$P#g; s#feeder3_ovopt#feeder3_ovdrone#g" $H/runs/ovopt_20261025/phone/phone_run.sh > $ST/phone_run.sh
cp $RD/phone/L*.txt $ST/
sha256sum $ST/ov_bridge_ovdrone > $ST/binaries.sha256
grep -c ovdrone $ST/phone_run.sh; ls -la $ST; diff <(grep -v "^#" $H/runs/ovopt_20261025/phone/phone_run.sh) <(grep -v "^#" $ST/phone_run.sh) | head
