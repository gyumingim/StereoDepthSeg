#!/bin/bash
# run a command, record peak RSS every 3 s to LOG.mem, kill it above LIMIT_MB
LIMIT=$1; LOG=$2; shift 2
"$@" > $LOG 2>&1 & P=$!; mx=0
while [ -d /proc/$P ]; do r=$(awk "/VmRSS/{print int(\$2/1024)}" /proc/$P/status 2>/dev/null); r=${r:-0}
  [ $r -gt $mx ] && mx=$r; echo $mx > $LOG.mem
  if [ $r -gt $LIMIT ]; then kill $P; echo "KILLED rss $r MB > $LIMIT" >> $LOG; fi; sleep 3; done
wait $P; echo "rc $? peak_rss_mb $mx" >> $LOG.mem
