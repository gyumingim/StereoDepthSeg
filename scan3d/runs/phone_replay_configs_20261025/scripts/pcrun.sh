set -u
B=/home/system/scanner/StereoDepthSeg/scan3d/ov_bridge/build_par/ov_bridge
O=/tmp/replaycfg; mkdir -p $O
for c in ${CFGS:-c4 c4n400s100c20}; do
  mkdir -p $O/$c; cp /mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/replay/rcfg/$c/*.yaml $O/$c/
  sed -i "s#/data/local/tmp/replay/cfg/$c/#$O/$c/#" $O/$c/estimator_config.yaml
  echo "== $c $(date +%T)"
  $B $O/$c/estimator_config.yaml --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --remap /home/system/scanner/StereoDepthSeg/scan3d/config/phone_ov_main/c4/remap_1920x1080.bin --timing-log $O/$c/tl.csv --threads 2 < /mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/replay/raw80.bin > $O/$c/po.jsonl 2> $O/$c/pe.txt
  echo rc $? poses $(grep -c pose $O/$c/po.jsonl) tracking $(grep -c '"tracking"' $O/$c/po.jsonl)
done
