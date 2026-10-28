# appnpu: one short phone run, arlat 0.6.32, npu_loop 1 (user approved 10-28). Phone on the desk, screen not touched.
# ov_phone_run.sh does preflight (SKIN < 36), SKIN >= 40 / thermal >= 2 stop, pull. Extra monitor here: AP >= 50,
# SKIN >= 45, battery < 20 % -> force-stop. Logcat after the run, filtered to ENN / NPU / our tags only.
set -u
A=/mnt/c/Users/a3162/platform-tools/adb.exe; S=100.79.156.20:33841
D=~/scanner/StereoDepthSeg-appnpu/scan3d/runs/appnpu_20261028/phone; mkdir -p $D
FLAG=/tmp/appnpu_stopmon_$(date +%s)
T0=$($A -s $S shell date "+%m-%d\ %H:%M:%S.000" | tr -d "\r")
echo "logcat from $T0" > $D/run_info.txt
( while [ ! -f $FLAG ]; do
    o=$($A -s $S shell "dumpsys thermalservice | sed -n \"/Current temperatures from HAL/,/^[^ \t]/p\"; dumpsys battery | grep -E \"^  level\"" 2>/dev/null | tr -d "\r")
    sk=$(echo "$o" | grep -oE "mValue=[0-9.]+, mType=3, mName=SKIN" | grep -oE "[0-9.]+" | head -1)
    ap=$(echo "$o" | grep -oE "mValue=[0-9.]+, mType=0, mName=AP" | grep -oE "[0-9.]+" | head -1)
    bl=$(echo "$o" | grep -oE "level: [0-9]+" | grep -oE "[0-9]+")
    echo "$(date +%T) SKIN $sk AP $ap batt $bl" >> $D/temps_pc.txt
    if awk -v s="${sk:-0}" -v a="${ap:-0}" -v b="${bl:-100}" "BEGIN{exit !(s>=45||a>=50||b<20)}"; then
      echo "$(date +%T) SAFETY STOP SKIN $sk AP $ap batt $bl" | tee -a $D/temps_pc.txt; $A -s $S shell am force-stop com.scan3d.arlat; fi
    sleep 5; done ) &
MP=$!
OUT=$D bash ~/scanner/StereoDepthSeg-appnpu/scan3d/tools/ov_phone_run.sh run $S c4f60 120 --es profile rec60 --ei npu_loop 1 2>&1 | tee $D/ov_phone_run.log
touch $FLAG; wait $MP
$A -s $S logcat -d -v threadtime -T "$T0" > /tmp/appnpu_logcat_full.txt
grep -E "ENN|enn|NPU|Npu|npu|Accelerator|GpuUserDriver|UNIFIED|xfnpu|arlat|ov_bridge|libenn" /tmp/appnpu_logcat_full.txt > $D/logcat_npu_filtered.txt
wc -l /tmp/appnpu_logcat_full.txt $D/logcat_npu_filtered.txt
echo "== after: camera clients / processes"; $A -s $S shell "dumpsys media.camera | grep -A2 \"Active Camera Clients\"; pidof libov_bridge.so; ps -A | grep -c scan3d" | tee $D/after_check.txt
