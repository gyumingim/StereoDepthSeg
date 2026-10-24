set -u
SPI=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/idlespin
OUT=/tmp/idlespin/pose_ab
cd ~/scanner/StereoDepthSeg/scan3d
BR=$PWD/ov_bridge/build_par/ov_bridge
CFG=$(ls -td runs/vio_* | head -5 | while read d; do [ -f $d/estimator_config.yaml ] && echo $d && break; done)
echo "cfg dir $CFG ($(stat -c %y $CFG))"
echo "== fast-feed identity (bench_wrap, threads 2 ref; S0_cfg = config threads)"
python3 $SPI/cmp.py $OUT/S0_t2.jsonl $OUT/S1_t2.jsonl $OUT/S2_t2.jsonl $OUT/S2_t4.jsonl $OUT/S0_cfg.jsonl /tmp/kb0615/pose_ab/A_stock.jsonl
for m in 0 1 2; do
  echo "== paced mode $m $(date +%T)"
  python3 $SPI/paced_feed.py $OUT/stream.bin $OUT/P$m.jsonl -- $BR $PWD/$CFG/estimator_config.yaml --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --idle-spin $m --timing-log $OUT/P$m.tlog 2> $OUT/P$m.err
  grep "idle spin" $OUT/P$m.err | tail -1
  python3 - $OUT/P$m.tlog <<'EOF'
import csv, sys, numpy as np
r = list(csv.DictReader(open(sys.argv[1])))
f = lambda k: np.array([float(x[k]) for x in r])
fe, wt, ex, sw = f("feed_ms"), f("wait_ms"), f("spin_exit_us"), f("spin_wait_us")
pc = lambda a: f"p50 {np.percentile(a,50):.3f} p95 {np.percentile(a,95):.3f} max {a.max():.3f}" if len(a) else "-"
print(f"  frames {len(r)} feed_ms {pc(fe)} | spin_wait_us(>0) n {int((sw>0).sum())} {pc(sw[sw>0])} | exit_us n {int((ex>=0).sum())} {pc(ex[ex>=0])}")
EOF
done
echo "== paced identity vs fast-feed S0_t2"
python3 $SPI/cmp.py $OUT/S0_t2.jsonl $OUT/P0.jsonl $OUT/P1.jsonl $OUT/P2.jsonl
echo DONE $(date +%T)
