#!/bin/bash
# run1.sh TAG "ARGS"  -> runs the instrument once, saves stream, app log, FULL logcat (unfiltered) since start, state before/after
export MSYS_NO_PATHCONV=1
A="adb -s 100.79.156.20:46821"; D=$(cd $(dirname $0) && pwd); O=$D/runs; mkdir -p $O; T=$1; shift
bash $D/st.sh > $O/$T.state_before.txt; rc=$?; cat $O/$T.state_before.txt
[ $rc -eq 3 ] && { echo "LIMIT -> stop"; exit 3; }
[ $rc -eq 2 ] && { echo "not under start gate -> skip"; exit 2; }
[ $rc -eq 4 ] && { echo "phone unreachable -> skip"; exit 2; }
st=$($A shell date '+%m-%d\ %H:%M:%S.000' | tr -d '\r')
$A shell am instrument -w -e args "\"$1\"" com.scan3d.nncprobe/.Probe > $O/$T.stream.txt 2>&1
sleep 1
$A logcat -d -v threadtime -T "$st" > $O/$T.logcat_full.txt 2>&1
$A exec-out run-as com.scan3d.nncprobe cat files/nncrun_log.txt > $O/$T.applog.txt
bash $D/st.sh > $O/$T.state_after.txt; cat $O/$T.state_after.txt
echo "logcat lines $(wc -l < $O/$T.logcat_full.txt)"
