set -u
H=~/scanner/StereoDepthSeg-ovpipe/scan3d
O=/tmp/ovpipe/pace; mkdir -p $O
[ -x $O/feeder4 ] || g++ -O2 -o $O/feeder4 $H/runs/ovheat_20261025/feeder4.cpp -lpthread
B=$H/ov_bridge/build_ovpipe_full/ov_bridge
for tag in A60 B60; do
  X=""; [ $tag = B60 ] && X="--pipe 1"
  mkdir -p $O/$tag
  $O/feeder4 /tmp/ovopt/raw80.bin 1.0 $O/$tag.prog 2.0 1 0 2>$O/$tag.feeder | timeout 200 $B /tmp/ovopt/cfg/c4/estimator_config.yaml --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --remap /tmp/ovopt/remap_1920x1080.bin --timing-log $O/$tag/tl.csv --threads 2 --profile rec60 $X > $O/$tag/po.jsonl 2> $O/$tag/pe.txt
  echo "$tag rc $? poses $(grep -c '"pose"' $O/$tag/po.jsonl)"; grep "pipe: sub\|drop-late:" $O/$tag/pe.txt | cut -c1-300
done
python3 $H/runs/ovpipe_20261026/phone_an.py $O
