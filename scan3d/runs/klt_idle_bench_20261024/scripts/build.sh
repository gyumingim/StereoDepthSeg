set -e
cd ~/scanner/StereoDepthSeg/scan3d/ov_bridge/android/build && make klt_bench -j8 2>&1 | grep -E "error|warning: unused|Built" | head -30
W=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/idle
~/android-ndk-r27c/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-strip -o $W/klt_bench_idle klt_bench
ls -la $W/klt_bench_idle
