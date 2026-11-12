#!/usr/bin/env bash
# mvs 11-12: run one command with /usr/bin/time -v and an nvidia-smi memory log (2 s) -> OUT/{time_v.txt,gpu_mem_mib.txt,done.txt,log.txt}
# usage: timed.sh NAME OUT cmd args...
set -u
NAME=$1; O=$2; shift 2
mkdir -p "$O"
nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits -l 2 > "$O/gpu_mem_mib.txt" 2>/dev/null &
SMI=$!
t0=$(date +%s)
/usr/bin/time -v -o "$O/time_v.txt" "$@" > "$O/log.txt" 2>&1
rc=$?
kill $SMI 2>/dev/null
echo "$NAME rc $rc wall $(( $(date +%s) - t0 )) s gpu_peak_mib $(cut -d, -f1 "$O/gpu_mem_mib.txt" | sort -n | tail -1) maxrss_kb $(grep 'Maximum resident' "$O/time_v.txt" | awk '{print $NF}')" > "$O/done.txt"
cat "$O/done.txt"
