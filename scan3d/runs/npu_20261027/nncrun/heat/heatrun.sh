#!/bin/bash
# heat test runner (pre-registered in README 11차): wait AP<40 & SKIN<36 (max 10 min) -> P0 60 s idle -> P1 180 s back-to-back
# x960p1mc NPU only -> P2 120 s idle -> if AP<45: P3 120 s at 250 ms period -> P4 60 s idle tail. Stop on STOP file.
export MSYS_NO_PATHCONV=1
D=$(cd $(dirname $0) && pwd); cd $D; mkdir -p heat
A="adb -s 100.79.156.20:33841"; C=heat/mon.csv; S=heat/STOP; rm -f $S ${S}.end
log() { echo "$(date +%s.%N | cut -c1-14) $(date +%T) $*" | tee -a heat/phases.log; }
cur() { $A shell 'dumpsys thermalservice | grep -A8 "Current temperatures from HAL" | grep -E "mName=(AP|SKIN)"' | sed -E 's/.*mValue=([0-9.]+).*mName=([A-Z]+).*/\2=\1/' | tr -d '\r' | tr '\n' ' '; }
: > heat/phases.log
bash heatmon.sh $C $S &
MP=$!
w=0
while :; do t=$(cur); ap=$(echo $t | grep -o 'AP=[0-9.]*' | cut -d= -f2); sk=$(echo $t | grep -o 'SKIN=[0-9.]*' | cut -d= -f2)
  awk "BEGIN{exit !($ap < 40 && $sk < 36)}" && break
  w=$((w+1)); [ $w -ge 60 ] && { log "WAIT TIMEOUT 10 min ($t) -> stop"; touch ${S}.end; wait $MP; exit 9; }
  sleep 10; done
log "P0 start idle 60 s ($t)"; sleep 60
[ -f $S ] && { log "STOP before P1"; touch ${S}.end; wait $MP; exit 3; }
log "P1 start back-to-back 180 s"
$A shell am instrument -w -e args '"model=p1.nnc in=in960.bin heat=1 secs=180 period=0 log=heat_p1.txt"' com.scan3d.nncprobe/.Probe > heat/p1.stream.txt 2>&1
log "P1 end"
[ -f $S ] && { log "STOP after P1"; }
log "P2 start idle 120 s"; sleep 120; log "P2 end ($(cur))"
ap=$(cur | grep -o 'AP=[0-9.]*' | cut -d= -f2)
if [ ! -f $S ] && awk "BEGIN{exit !($ap < 45)}"; then
  log "P3 start 4 Hz (250 ms) 120 s"
  $A shell am instrument -w -e args '"model=p1.nnc in=in960.bin heat=1 secs=120 period=250 log=heat_p3.txt"' com.scan3d.nncprobe/.Probe > heat/p3.stream.txt 2>&1
  log "P3 end"
  log "P4 start idle tail 60 s"; sleep 60; log "P4 end"
else log "P3 skipped (AP=$ap or STOP)"; fi
touch ${S}.end; wait $MP
$A exec-out run-as com.scan3d.nncprobe cat files/heat_p1.txt > heat/heat_p1.txt
$A exec-out run-as com.scan3d.nncprobe cat files/heat_p3.txt > heat/heat_p3.txt 2>/dev/null
log "DONE"
