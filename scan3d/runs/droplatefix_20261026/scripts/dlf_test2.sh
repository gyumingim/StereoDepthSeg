set -u
O=/tmp/droplatefix/t; T=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team
REF=/tmp/verify_ovgyro/build_integ/ov_bridge; NEW=/tmp/droplatefix/build/ov_bridge
C4=/tmp/ovopt/cfg/c4/estimator_config.yaml; RM="--remap /tmp/ovopt/remap_1920x1080.bin"; R80=/tmp/ovopt/raw80.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
pac() { local n=$1 b=$2 port=$3; shift 3
  python3 $T/dlf_udp.py $port 49 $O/$n.udp.json > $O/$n.udp.txt &
  sleep 0.5
  ( $O/feeder3v $R80 1.0 - 1.0 45 25 400 2 | nice -n 5 $b $C4 $BASE $RM --profile rec20261025 --viop-udp 127.0.0.1:$port --viop-log $O/$n.viop.csv "$@" > $O/$n.jsonl 2> $O/$n.err ) 2> $O/$n.feeder
  wait; echo "== $n | $*"; }
date
pac w2_ref $REF 14993 & pac w2_oldstale $NEW 14994 --drop-late-mode 0 --pred-stale-ms 500 & pac w2_new $NEW 14995 & wait
date
