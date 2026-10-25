# round 2 after verifier: rebuild check, identity, shift repro, overload (--test-feed-sleep 150:25 = ~4.5x frame period)
set -u
O=/tmp/droplatefix/t3; mkdir -p $O
REF=/tmp/verify_ovgyro/build_integ/ov_bridge; NEW=/tmp/droplatefix/build/ov_bridge; FD=/tmp/droplatefix/t/feeder3v
C4=/tmp/ovopt/cfg/c4/estimator_config.yaml; RM="--remap /tmp/ovopt/remap_1920x1080.bin"; R80=/tmp/ovopt/raw80.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
date; sha256sum $NEW
idr() { local n=$1 b=$2; shift 2; nice -n 5 $b $C4 $BASE --pose-precision 17 $RM "$@" < $R80 > $O/$n.jsonl 2> $O/$n.err; echo "== $n rc $? poses $(grep -c '"type":"pose"' $O/$n.jsonl) | $*"; }
pac() { local n=$1 b=$2 mt=$3 m=$4; shift 4
  ( $FD $R80 1.0 - 1.0 $mt 25 400 $m | nice -n 5 $b $C4 $BASE $RM --profile rec20261025 --timing-log $O/$n.tl.csv "$@" > $O/$n.jsonl 2> $O/$n.err ) 2> $O/$n.feeder
  echo "== $n m$m maxT $mt | $* | $(tail -1 $O/$n.feeder)"; }
idr i_ref $REF --profile rec20261025 --drop-late 0 & idr i_new $NEW --profile rec20261025 --drop-late 0 & idr p_new $NEW & wait
C="python3 /mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/vgy/cmpv.py"
$C $O/i_ref.jsonl $O/i_new.jsonl; $C /tmp/droplatefix/t/p_ref.jsonl $O/p_new.jsonl
pac s2_new $NEW 45 2 --viop-udp 127.0.0.1:14996 --viop-log $O/s2_new.viop.csv &
pac s1_new $NEW 45 1 &
pac s0_new $NEW 45 0 &
wait; date
pac o_old $NEW 55 0 --drop-late-mode 0 --test-feed-sleep 150:25 &
pac o_new $NEW 55 0 --test-feed-sleep 150:25 --viop-udp 127.0.0.1:14997 --viop-log $O/o_new.viop.csv &
pac o_new60 $NEW 55 0 --test-feed-sleep 60:25 &
wait; date
pac s2_old $NEW 45 2 --drop-late-mode 0 &
pac s2_ref $REF 45 2 &
wait; date
pac f_new_r3 $NEW 0 0 & wait; date
