s=open('run_src.sh').read()
s=s.replace('# ON-PHONE runner for integ2-20261026','# ON-PHONE runner timing0626 (copy of integ2_run.sh: own dir, keep-awake via KEYCODE_WAKEUP, wake/cap/uptime logging)\n# was: ON-PHONE runner for integ2-20261026')
s=s.replace('cd /data/local/tmp/integ2 ||','cd /data/local/tmp/timing0626 ||')
s=s.replace('C=/data/local/tmp/integ2/cfg','C=/data/local/tmp/timing0626/cfg')
s=s.replace('LD_LIBRARY_PATH=/data/local/tmp/integ2/lib','LD_LIBRARY_PATH=/data/local/tmp/timing0626/lib')
s=s.replace('  sleep ${MIN_COOL:-0}\n','  sleep ${MIN_COOL:-0}\n  [ "$(wake)" = Awake ] || { echo "$(date +%T) $TAG wake was $(wake) -> WAKEUP" >> $ST; input keyevent KEYCODE_WAKEUP; sleep 2; }\n')
s=s.replace('  S0=$(skin); A0=$(ap); T0=$(date +%T)\n','  [ "$(wake)" = Awake ] || { echo "$(date +%T) $TAG wake was $(wake) -> WAKEUP pre-run" >> $ST; input keyevent KEYCODE_WAKEUP; sleep 2; }\n  S0=$(skin); A0=$(ap); T0=$(date +%T); UP0=$(cut -d" " -f1 /proc/uptime); RT0=$(date +%s.%N)\n')
s=s.replace('    s=$(skin); a=$(ap); gt "$s" "$SMAX" && SMAX=$s\n','    W=$(wake); if [ "$W" != Awake ]; then NW=$((NW+1)); echo "$(date +%T) $TAG midrun wake $W -> WAKEUP" >> $ST; input keyevent KEYCODE_WAKEUP; fi\n    s=$(skin); a=$(ap); gt "$s" "$SMAX" && SMAX=$s\n')
s=s.replace('  SMAX=$S0; AB=0\n','  SMAX=$S0; AB=0; NW=0\n')
s=s.replace('aborted=$AB" > $O/run.txt','aborted=$AB uptime_start=$UP0 uptime_end=$(cut -d" " -f1 /proc/uptime) rt_start=$RT0 rt_end=$(date +%s.%N) cpu9_cap_end=$(cat /sys/devices/system/cpu/cpu9/cpufreq/scaling_max_freq) midrun_not_awake=$NW" > $O/run.txt')
s=s.replace('pidof feeder4_integ2 feeder3v_integ2 $(ls ov_bridge_* 2>/dev/null)','pidof feeder4_integ2 feeder3v_integ2 ov_bridge_integ2')
assert s.count('input keyevent KEYCODE_WAKEUP')==3 and 'midrun_not_awake' in s and 'NW=0' in s and 'UP0' in s and 'timing0626/lib' in s
open('timing0626_run.sh','w',newline='\n').write(s)
L1="""# timing0626 W1 (block 1): N = new rec20261025 profile (+sym_inplace,ekf_sparse), O = old ovopt; drop-late 0; ABBA
w_N_r1 ov_bridge_integ2 c4 2 f4 --profile rec20261025 --drop-late 0 --prof-log @O@/prof.csv
w_O_r1 ov_bridge_integ2 c4 2 f4 --profile rec20261025 --drop-late 0 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol --prof-log @O@/prof.csv
w_O_r2 ov_bridge_integ2 c4 2 f4 --profile rec20261025 --drop-late 0 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol --prof-log @O@/prof.csv
w_N_r2 ov_bridge_integ2 c4 2 f4 --profile rec20261025 --drop-late 0 --prof-log @O@/prof.csv
"""
L2=L1.replace("W1 (block 1)","W2 (block 2)").replace("_r1","_r3").replace("_r2","_r4")
open('W1.txt','w',newline='\n').write(L1); open('W2.txt','w',newline='\n').write(L2)
