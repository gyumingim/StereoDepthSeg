# droplatefix PC tests: identity (drop-late off) + real-time paced drop-late repro (verifier feeder3v)
set -u
O=/tmp/droplatefix/t; mkdir -p $O
REF=/tmp/verify_ovgyro/build_integ/ov_bridge   # integ 14de3b8 build (verifier)
NEW=/tmp/droplatefix/build/ov_bridge
C4=/tmp/ovopt/cfg/c4/estimator_config.yaml
RM="--remap /tmp/ovopt/remap_1920x1080.bin"
R80=/tmp/ovopt/raw80.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
g++ -O2 -pthread /mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/vgy/feeder3v.cpp -o $O/feeder3v
date; uptime; sha256sum $REF $NEW $R80
idr() { local n=$1 b=$2; shift 2; nice -n 5 $b $C4 $BASE --pose-precision 17 $RM "$@" < $R80 > $O/$n.jsonl 2> $O/$n.err; echo "== $n rc $? poses $(grep -c '"type":"pose"' $O/$n.jsonl) | $*"; }
pac() { local n=$1 b=$2 mt=$3 m=$4; shift 4
  ( $O/feeder3v $R80 1.0 - 1.0 $mt 25 400 $m | nice -n 5 $b $C4 $BASE $RM --profile rec20261025 --timing-log $O/$n.tl.csv "$@" > $O/$n.jsonl 2> $O/$n.err ) 2> $O/$n.feeder
  echo "== $n m$m maxT $mt | $* | $(tail -1 $O/$n.feeder)"; }
if [ "${STAGE:-all}" = all ] || [ "${STAGE}" = ident ]; then
  idr i_ref $REF --profile rec20261025 --drop-late 0 & idr i_new $NEW --profile rec20261025 --drop-late 0 &
  idr p_ref $REF & idr p_new $NEW & wait
  C="python3 /mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/vgy/cmpv.py"
  $C $O/i_ref.jsonl $O/i_new.jsonl; $C $O/p_ref.jsonl $O/p_new.jsonl
  date
fi
if [ "${STAGE:-all}" = all ] || [ "${STAGE}" = pace ]; then
  for m in 2 1 0; do
    pac s${m}_ref $REF 45 $m --viop-udp 127.0.0.1:14990 --viop-log $O/s${m}_ref.viop.csv &
    pac s${m}_old $NEW 45 $m --drop-late-mode 0 --viop-udp 127.0.0.1:14991 --viop-log $O/s${m}_old.viop.csv &
    pac s${m}_new $NEW 45 $m --viop-udp 127.0.0.1:14992 --viop-log $O/s${m}_new.viop.csv &
    wait; date
  done
fi
if [ "${STAGE:-all}" = all ] || [ "${STAGE}" = full ]; then
  for r in 1 2; do
    pac f_ref_r$r $REF 0 0 & pac f_new_r$r $NEW 0 0 & wait; date
  done
fi
