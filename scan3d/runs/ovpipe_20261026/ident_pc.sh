# ovpipe-20261026 PC identity: this build (--pipe 0 / 1) vs the integ-20261025 build (14de3b8 full-source Release), 17 digits.
# Inputs as integ ident_pc.sh: H = harness hs_20261024_165506_std60 (IMU lead 0.4 s, 845 frames, cfg_identity),
# R = raw80 phone replay stream (hs_20261021_213341, 2327 frames, c4 + remap). X = R through inject.py (a copy of the
# previous image after every 37th image + one RESET after image 1200) = drop / re-chain / tracker-reset paths.
#   bash ident_pc.sh [JOBS...]   outputs /tmp/ovpipe/ident/<job>.{jsonl,err}
set -u
H=~/scanner/StereoDepthSeg-ovpipe/scan3d
RD=$H/runs/ovpipe_20261026
OUT=/tmp/ovpipe/ident; mkdir -p $OUT
REF=~/scanner/StereoDepthSeg-integ/scan3d/ov_bridge/build_integ_full/ov_bridge
MINE=${MINE:-$H/ov_bridge/build_ovpipe_full/ov_bridge}
HC=$H/runs/ovopt_20261025/cfg_identity/estimator_config.yaml
HS=/tmp/imuprio/h_L0.4nc.bin
C4=/tmp/ovopt/cfg/c4/estimator_config.yaml
RM="--remap /tmp/ovopt/remap_1920x1080.bin"
R80=/tmp/ovopt/raw80.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
PT="--predict 2 --predict-hz 200 --pred-thread 1"
date; sha256sum $REF $MINE
run() { local n=$1 b=$2 cfg=$3 in=$4; shift 4
  $b $cfg $BASE "$@" < $in > $OUT/$n.jsonl 2> $OUT/$n.err; echo "== $n rc $? poses $(grep -c '"pose"' $OUT/$n.jsonl) | $*"; }
runx() { local n=$1 b=$2; shift 2
  : > $OUT/$n.err
  python3 $RD/inject.py $R80 37 1200 ${XMODE:-dup} 2>>$OUT/$n.err | $b $C4 $BASE $RM "$@" > $OUT/$n.jsonl 2>> $OUT/$n.err; echo "== $n rc $? poses $(grep -c '"pose"' $OUT/$n.jsonl) | inject $*"; }
job() {
  case $1 in
    h_ref) run $1 $REF $HC $HS ;;
    h_def) run $1 $MINE $HC $HS ;;
    h_pipe) run $1 $MINE $HC $HS --pipe 1 ;;
    h_pipe_pt) run $1 $MINE $HC $HS --pipe 1 $PT ;;
    h_pipe_log) run $1 $MINE $HC $HS --pipe 1 $PT --pipe-log $OUT/h_pipe_log.pipelog.csv --pipe-cpus 0-15 ;;
    r_ref) run $1 $REF $C4 $R80 $RM ;;
    r_def) run $1 $MINE $C4 $R80 $RM ;;
    r_pipe) run $1 $MINE $C4 $R80 $RM --pipe 1 ;;
    r60_ref) run $1 $REF $C4 $R80 $RM --profile rec60 --drop-late 0 ;;
    r60_pipe) run $1 $MINE $C4 $R80 $RM --profile rec60 --drop-late 0 --pipe 1 ;;
    x_def) runx $1 $MINE ;;
    x_pipe) runx $1 $MINE --pipe 1 ;;
    x_pipe_w3) runx $1 $MINE --pipe 1 --profile rec60 --drop-late 0 ;;
    x_w3) runx $1 $MINE --profile rec60 --drop-late 0 ;;
    s_def) XMODE=swap runx $1 $MINE ;;
    s_pipe) XMODE=swap runx $1 $MINE --pipe 1 ;;
  esac
}
JOBS=${@:-h_ref h_def h_pipe h_pipe_pt r_ref r_def r_pipe r60_ref r60_pipe x_def x_pipe x_w3 x_pipe_w3}
P=${P:-4}; n=0
for j in $JOBS; do job $j & n=$((n+1)); if [ $((n % P)) -eq 0 ]; then wait; fi; done; wait
C=$RD/cmp2.py
echo "== harness"; python3 $C $OUT/h_ref.jsonl $OUT/h_def.jsonl $OUT/h_pipe.jsonl $OUT/h_pipe_pt.jsonl $( [ -f $OUT/h_pipe_log.jsonl ] && echo $OUT/h_pipe_log.jsonl )
echo "== raw80"; python3 $C $OUT/r_ref.jsonl $OUT/r_def.jsonl $OUT/r_pipe.jsonl
echo "== raw80 rec60 profile"; python3 $C --ex h $OUT/r60_ref.jsonl $OUT/r60_pipe.jsonl
echo "== injected (drops + reset)"; python3 $C $OUT/x_def.jsonl $OUT/x_pipe.jsonl; python3 $C --ex h $OUT/x_w3.jsonl $OUT/x_pipe_w3.jsonl
[ -f $OUT/s_def.jsonl ] && { echo "== injected swapped-size copies (rejected -> drop) + reset"; python3 $C $OUT/s_def.jsonl $OUT/s_pipe.jsonl; }
grep -H "pipe:\|inject:\|exception" $OUT/*.err | head -60
date
