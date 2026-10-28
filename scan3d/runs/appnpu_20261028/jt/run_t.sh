set -e
SDK=~/scanner/StereoDepthSeg/tools/android-sdk; J=$SDK/android-35/android.jar
S=~/scanner/StereoDepthSeg-appnpu/phone_stereo/arlat/src/com/scan3d/arlat
D=$(mktemp -d); javac -source 8 -target 8 -classpath $J -d $D $S/NpuLoop.java $(dirname $0)/TNpu.java 2>&1 | grep -v "warning" || true
java -cp $D:$J com.scan3d.arlat.TNpu
