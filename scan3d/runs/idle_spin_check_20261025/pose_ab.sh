set -u
date
SP=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad
OUT=/tmp/idlespin/pose_ab; mkdir -p $OUT
cd ~/scanner/StereoDepthSeg/scan3d
BR=$PWD/ov_bridge/build_par/ov_bridge
mk() {  # name extra-args [tee]
  local pre=""
  [ "${3:-}" = tee ] && pre="tee $OUT/stream.bin | "
  cat > $OUT/br_$1.sh <<EOS
#!/bin/bash
$pre"$BR" "\$@" $2 --timing-log "$OUT/$1.tlog" 2>"$OUT/$1.err" | tee "$OUT/$1.jsonl"
EOS
  chmod +x $OUT/br_$1.sh
}
mk S0_t2 "--threads 2" tee
mk S1_t2 "--threads 2 --idle-spin 1"
mk S2_t2 "--threads 2 --idle-spin 2"
mk S2_t4 "--threads 4 --idle-spin 2"
mk S0_cfg ""
cat $OUT/br_S0_t2.sh $OUT/br_S2_t2.sh
for n in ${RUNS:-S0_t2 S1_t2 S2_t2 S2_t4 S0_cfg}; do
  echo "== $n $(date +%T)"
  ../venv_ffs/bin/python $SP/bench_wrap.py $OUT/br_$n.sh recordings/hs_20261024_165506_std60 --fps 30 --size 960x540 --clones 15 --npts 200 --imu-raw --frame-cache --out $OUT/$n.json > $OUT/$n.stdout 2> $OUT/$n.stderr
  echo "rc $? poses $(grep -c '"pose"' $OUT/$n.jsonl)"
  grep -E "idle spin" $OUT/$n.err | tail -2
done
echo DONE $(date +%T)
