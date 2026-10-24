set -u
ADB="/mnt/c/Users/a3162/platform-tools/adb.exe -s 100.79.156.20:5555"
SPI=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/idlespin
WSPI='C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\idlespin'
date
# first 12 s of the captured input stream (from 1 s before the first frame)
python3 - <<'EOF'
import struct
d = open("/tmp/idlespin/pose_ab/stream.bin", "rb").read()
i, t0, out = 0, None, []
while i + 12 <= len(d):
    m, t, n = struct.unpack_from("<III", d, i)
    if t in (1, 2):
        ts = struct.unpack_from("<q", d, i + 12)[0]
        t0 = ts if t0 is None else t0
        if ts - t0 > 12e9:
            break
    i += 12 + n
open("/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/idlespin/stream12.bin", "wb").write(d[:i])
print("stream12 bytes", i)
EOF
~/android-ndk-r27c/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android34-clang++ -O2 -static-libstdc++ -o $SPI/feeder $SPI/feeder.cpp || exit 1
$ADB push "$WSPI\\feeder" /data/local/tmp/idlespin/feeder | tail -1
$ADB push "$WSPI\\stream12.bin" /data/local/tmp/idlespin/stream12.bin | tail -1
cat > $SPI/pr.sh <<'EOS'
cd /data/local/tmp/idlespin
chmod 755 feeder ov_bridge_0616
export LD_LIBRARY_PATH=/data/local/tmp/kb0615
dumpsys thermalservice | grep -A30 "Current temperatures from HAL" | grep -E "mName=SKIN|mName=AP" | head -2
for m in 0 1 2; do
  echo "== phone paced mode $m $(date +%T)"
  ./feeder stream12.bin 1.0 | ./ov_bridge_0616 /data/local/tmp/kb0615/cfg/estimator_config.yaml --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --idle-spin $m --timing-log tl_$m.csv > po_$m.jsonl 2> pe_$m.txt
  grep -E "idle spin (end|:)" pe_$m.txt | tail -1
  sleep 5
done
dumpsys thermalservice | grep -A30 "Current temperatures from HAL" | grep -E "mName=SKIN|mName=AP" | head -2
pidof ov_bridge_0616 feeder || echo "no bridge/feeder running"
EOS
$ADB push "$WSPI\\pr.sh" /data/local/tmp/idlespin/pr.sh | tail -1
$ADB shell sh /data/local/tmp/idlespin/pr.sh
mkdir -p $SPI/phone_out
for m in 0 1 2; do $ADB pull /data/local/tmp/idlespin/tl_$m.csv "$WSPI\\phone_out\\tl_$m.csv" | tail -1; $ADB pull /data/local/tmp/idlespin/pe_$m.txt "$WSPI\\phone_out\\pe_$m.txt" | tail -1; done
for m in 0 1 2; do python3 - $SPI/phone_out/tl_$m.csv $m <<'EOF'
import csv, sys, numpy as np
r = list(csv.DictReader(open(sys.argv[1])))
f = lambda k: np.array([float(x[k]) for x in r])
fe, rm, ex, sw = f("feed_ms"), f("remap_ms"), f("spin_exit_us"), f("spin_wait_us")
pc = lambda a: f"p50 {np.percentile(a,50):.3f} p95 {np.percentile(a,95):.3f} max {a.max():.3f}" if len(a) else "-"
print(f"mode {sys.argv[2]} frames {len(r)} feed_ms {pc(fe)} | spin_wait_us(>0) n {int((sw>0).sum())} {pc(sw[sw>0])} | exit_us(before frames) n {int((ex>=0).sum())} {pc(ex[ex>=0])}")
EOF
done
date
