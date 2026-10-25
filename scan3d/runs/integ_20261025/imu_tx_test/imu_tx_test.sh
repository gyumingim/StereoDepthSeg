# PC test of the app's VIOP_T_IMU frame builder (integ-20261025): Java ViopImuSender.build vs C viop_proto.h
# (viop_build + viop_imu_t), decode with viop_imu_decode + stream parser, and the Python receiver viop_rx.py over UDP
# loopback (frames sent by a Python sender built from the same Java bytes). Output: imu_tx_test.log
set -eu
H=~/scanner/StereoDepthSeg-integ
T=$H/scan3d/runs/integ_20261025/imu_tx_test
JAR=~/scanner/StereoDepthSeg/tools/android-sdk/android-35/android.jar
W=/tmp/integ/imutx; rm -rf $W; mkdir -p $W/classes
javac -source 8 -target 8 -nowarn -classpath $JAR -d $W/classes $H/phone_stereo/arlat/src/com/scan3d/arlat/ViopImuSender.java $T/ImuFrameTest.java 2>&1 | grep -v "bootstrap\|^warning\|^1 warning\|^Note" || true
java -cp $W/classes:$JAR com.scan3d.arlat.ImuFrameTest > $W/java_frames.txt
wc -l < $W/java_frames.txt
gcc -std=c99 -O1 -Wall -Wextra -I$H/scan3d/ov_bridge -o $W/imu_frame_c $T/imu_frame_c.c -lm
$W/imu_frame_c < $W/java_frames.txt
# UDP loopback through viop_rx.py (IMU decode, rate / gaps / seq)
python3 $H/scan3d/runs/ovdrone_20261025/viop_rx.py listen --port 14698 --bind 127.0.0.1 --secs 4 --out $W/rx > $W/rx.log 2>&1 &
sleep 0.5
python3 - $W/java_frames.txt <<'PY'
import socket, sys, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
fr = [bytes.fromhex(l.split()[-1]) for l in open(sys.argv[1])][3:1003]  # 1000 frames with increasing time (2.003 ms)
t0 = time.perf_counter()
for i, f in enumerate(fr):
    while time.perf_counter() < t0 + i * 0.002:
        pass
    s.sendto(f, ("127.0.0.1", 14698))
PY
wait
python3 $H/scan3d/runs/ovdrone_20261025/viop_rx.py check $W/rx > /dev/null
python3 -c "import json;d=json.load(open('$W/rx/check.json'));print('viop_rx imu:', json.dumps(d.get('imu')))"
