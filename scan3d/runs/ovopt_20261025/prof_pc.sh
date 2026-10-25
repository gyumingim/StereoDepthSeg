# PC profile + identity of ov_bridge --ovopt on the phone replay stream (2026-10-25, branch ovopt-20261025).
# Input = raw80.bin (the other agent's camera-free phone replay stream: hs_20261021_213341, 80 s, 2327 raw 1920x1080
# frames + raw IMU, made by scratchpad/replay/make_raw_stream.py; copied to /tmp/ovopt/raw80.bin), phone configs
# config/phone_ov_main/<cfg> (paths -> /tmp), phone remap_1920x1080.bin, --threads 2 (= phone), unpaced feed (PC).
# Every run writes --prof-log (ovpatch stage timers) + --timing-log. Variants are interleaved (A B C A B C ...).
#   CFGS="c4 c4n400s100c20" VARS="A B C" REPS=3 bash prof_pc.sh
set -u
H=~/scanner/StereoDepthSeg-ovopt/scan3d
RD=$H/runs/ovopt_20261025
OUT=${OUT:-/tmp/ovopt/prof}; mkdir -p $OUT
STREAM=${STREAM:-/tmp/ovopt/raw80.bin}
REMAP=/tmp/ovopt/remap_1920x1080.bin
FULL=${FULL:-$H/ov_bridge/build_ovopt_full/ov_bridge}
[ -f $REMAP ] || cp ~/scanner/StereoDepthSeg/scan3d/config/phone_ov_main/c4/remap_1920x1080.bin $REMAP
declare -A OPT=([A]="" [B]="all_exact" [C]="all_exact,ekf_gemm,qr_compress" [D]="ekf_gemm" [E]="qr_compress"
                [S]="skip_retri" [T]="par_tri" [J]="par_jac" [M]="par_ma" [N]="frame_nocopy" [V]="cov_move"
                [R]="givens_rm" [G]="all_exact,ekf_gemm" [K]="all_exact,ekf_chol" [P]="all_exact,par_cov"
                [W]="all_exact,ekf_gemm,qr_compress,ekf_chol,par_cov")
# (2026-10-25 note: all_exact grew givens_rm after prof_pc_1; ekf_chol / par_cov exist from the Phase 2 build on)
date
echo "stream $STREAM ($(stat -c %s $STREAM) B) remap $(sha256sum $REMAP | cut -c1-16) bridge $(sha256sum $FULL | cut -c1-16)"
uptime
for cfg in ${CFGS:-c4 c4n400s100c20}; do
  C=/tmp/ovopt/cfg/$cfg; mkdir -p $C
  cp $H/config/phone_ov_main/$cfg/*.yaml $C/
  sed -i "s#/storage/emulated/0/Android/data/com.scan3d.arlat/files/ov/$cfg/#$C/#" $C/estimator_config.yaml
  grep -q "$C/ov_timing.txt" $C/estimator_config.yaml || { echo "path rewrite failed $cfg"; exit 1; }
  for r in $(seq 1 ${REPS:-3}); do
    for v in ${VARS:-A B C}; do
      n=${cfg}_${v}_r$r
      o=${OPT[$v]}
      t0=$(date +%s.%N)
      $FULL $C/estimator_config.yaml --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 \
        --remap $REMAP --threads 2 --timing-log $OUT/$n.tl.csv --prof-log $OUT/$n.prof.csv ${o:+--ovopt $o} \
        < $STREAM > $OUT/$n.jsonl 2> $OUT/$n.err
      echo "== $n rc $? wall $(echo "$(date +%s.%N) - $t0" | bc | cut -c1-6) s poses $(grep -c '"pose"' $OUT/$n.jsonl) ovopt '$o' load $(cut -d' ' -f1 /proc/loadavg)"
      mv $C/ov_timing.txt $OUT/$n.ov_timing.txt 2>/dev/null
    done
  done
  echo "== identity $cfg (ref = A r1)"
  ls $OUT/${cfg}_*_r*.jsonl | grep -v "_A_r1.jsonl" | xargs python3 $RD/cmp.py $OUT/${cfg}_A_r1.jsonl
done
date
