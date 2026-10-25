# Full-precision identity of the exact --ovopt options (2026-10-25, after the verifier note that 9-digit JSON is not a
# bit check): ov_bridge --pose-precision 17 (round-trip exact doubles in every pose-line number). Same inputs as
# ident_pc.sh (C4 harness, 845 frames) and prof_pc.sh (raw80 + phone remap, c4 / c4n400s100c20, 2327 frames).
# Reference = this branch's full-source build with no options (its 9-digit output = 0.6.17 build_par, ident_pc.sh).
set -u
H=~/scanner/StereoDepthSeg-ovopt/scan3d
RD=$H/runs/ovopt_20261025
OUT=/tmp/ovopt/ident17; mkdir -p $OUT
FULL=$H/ov_bridge/build_ovopt_full/ov_bridge
REMAP=/tmp/ovopt/remap_1920x1080.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
date
sha256sum $FULL
EX="skip_retri par_tri par_jac par_ma par_s cov_move givens_rm frame_nocopy all_exact"
# 1) C4 harness
for v in none $EX; do
  o=""; [ $v != none ] && o="--ovopt $v"
  $FULL $RD/cfg_identity/estimator_config.yaml $BASE $o < /tmp/idlespin/pose_ab/stream.bin > $OUT/h_$v.jsonl 2> $OUT/h_$v.err
  echo "== h_$v rc $? poses $(grep -c '"pose"' $OUT/h_$v.jsonl)"
done
python3 $RD/cmp.py $OUT/h_none.jsonl $(for v in $EX; do echo $OUT/h_$v.jsonl; done)
# 2) raw80 phone configs
for cfg in c4 c4n400s100c20; do
  C=/tmp/ovopt/cfg/$cfg
  vs="none $EX"; [ $cfg != c4 ] && vs="none all_exact"
  for v in $vs; do
    o=""; [ $v != none ] && o="--ovopt $v"
    $FULL $C/estimator_config.yaml $BASE --remap $REMAP $o < /tmp/ovopt/raw80.bin > $OUT/${cfg}_$v.jsonl 2> $OUT/${cfg}_$v.err
    echo "== ${cfg}_$v rc $? poses $(grep -c '"pose"' $OUT/${cfg}_$v.jsonl)"
    mv $C/ov_timing.txt $OUT/${cfg}_$v.ov_timing.txt 2>/dev/null
  done
  python3 $RD/cmp.py $OUT/${cfg}_none.jsonl $(for v in $vs; do [ $v != none ] && echo $OUT/${cfg}_$v.jsonl; done)
done
# 3) 9-digit cross-check: the 17-digit run rounded is not compared; the default precision reference stays ident_pc.sh
date
