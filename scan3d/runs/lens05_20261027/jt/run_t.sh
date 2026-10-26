set -e
J=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/lens05/jt
W=~/scanner/tmp_lens05/jt; rm -rf $W; mkdir -p $W/top
SRC=~/scanner/StereoDepthSeg-lens05/phone_stereo/arlat/src/com/scan3d/arlat/OvRunner.java
# method body exactly as in OvRunner.java (private -> static)
awk '/private void checkRemapSize/,/^    }$/' $SRC | sed 's/private void checkRemapSize/static void checkRemapSize/' > $W/body.txt
python3 - "$J/T.java" "$W/body.txt" "$W/T.java" <<'P'
import sys
t = open(sys.argv[1]).read(); b = open(sys.argv[2]).read()
open(sys.argv[3], "w").write(t.replace("    //BODY//\n", b))
P
F=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/lens05/phone_top/lens0u_remap.bin
echo "lens0u source: $F"
cp $F $W/top/; cp $(dirname $F)/kalibr_imucam_chain.yaml $W/top/
sha256sum $W/top/* | cut -c1-16
cd $W && javac -d . T.java && java -cp . T ~/scanner/tmp_lens05/new_gen $W/top ~/scanner/tmp_lens05/old_regen
