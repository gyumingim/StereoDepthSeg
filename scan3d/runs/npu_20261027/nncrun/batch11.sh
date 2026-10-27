#!/bin/bash
# nncrun 9th batch: waits for the phone (wireless adb), then mc LG + XFeat builds, then the rest of the 8th (L4s256 sc),
# pulls outputs, uninstalls. Gate per item as usual (run1.sh/st.sh). Order = coordinator priority.
export MSYS_NO_PATHCONV=1
D=$(cd $(dirname $0) && pwd); cd $D
A="adb -s 100.79.156.20:46821"; P="run-as com.scan3d.nncprobe"; R=/c/Users/a3162/scan3d_handoff/npu_results
log() { echo "$(date +%T) $*"; }
n=0
until $A get-state >/dev/null 2>&1; do
  adb connect 100.79.156.20:46821 >/dev/null 2>&1
  n=$((n+1)); [ $n -ge 120 ] && { log "phone not back after ~4 h -> stop"; exit 9; }
  sleep 120
done
log "phone back"; bash st.sh
$A install -r nncprobe.apk || { log "install fail"; exit 5; }
push() { $A exec-in $P sh -c "cat > files/$2" < "$1"; }
for k in L6_a16 L4_a16 L4s256_a16; do push $R/lgnpu/${k}mc/${k}_simplify_O2_MultiCore.nnc ${k}mc.nnc; push $R/lgnpu/${k}mc/input_data_float32.bin g_${k}mc.bin; done
push $R/lab_xfeat_mc/xf1fpmc/xf1fp_simplify_O2_MultiCore.nnc xf1fpmc.nnc; push $R/lab_xfeat_mc/xf1fpmc/input_data_float32.bin g_xf1fpmc.bin
push $R/lab_xfeat_post/xpnmsr/xpnmsr_simplify_O2_MultiCore.nnc xpnmsr.nnc; push $R/lab_xfeat_post/xpnmsr/input_data_float32.bin g_xpnmsr.bin
push $R/lab_xfeat_fp16_variants/xfpmc/xfpmc_simplify_O2_MultiCore.nnc xfpmc.nnc
for f in in_a3.bin gray13.bin eval3ch.bin lg_x13.bin lg_pairs13.bin lg_x13_n512.bin lg_pairs13_n512.bin lg_wr.bin; do push $f $f; done
$A shell $P sh -c "'chmod 600 files/*; md5sum files/*.nnc files/*.bin'" > runs/md5_phone_batch11.txt
md5sum $R/lgnpu/*mc/*.nnc $R/lab_xfeat_mc/xf1fpmc/*.nnc $R/lab_xfeat_post/xpnmsr/*.nnc $R/lab_xfeat_fp16_variants/xfpmc/*.nnc > runs/md5_pc_batch11.txt
items=()
for k in L6_a16mc L4_a16mc L4s256_a16mc; do
  if [ $k = L4s256_a16mc ]; then x=lg_x13.bin; pr=lg_pairs13.bin; nn=256; else x=lg_x13_n512.bin; pr=lg_pairs13_n512.bin; nn=512; fi
  items+=("n_${k}_gold|model=$k.nnc in=g_$k.bin out=og_$k.bin warm=3 iters=10 reps=1")
  items+=("n_${k}_eval|model=$k.nnc in=$x out=oe_$k.bin warm=20 iters=200 reps=2")
  items+=("n_${k}_pipe_r1|model=$k.nnc in=$pr out=ml_${k}_r1.bin lgpipe=1 lgn=$nn passes=5 wr=lg_wr.bin")
  items+=("n_${k}_pipe_r2|model=$k.nnc in=$pr out=ml_${k}_r2.bin lgpipe=1 lgn=$nn passes=5 wr=lg_wr.bin")
  items+=("n_${k}_pipe_p2|model=$k.nnc in=$pr out=ml_${k}_p2.bin lgpipe=1 lgn=$nn passes=5 wr=lg_wr.bin lgpost=2")
done
items+=("n_xf1fpmc_gold|model=xf1fpmc.nnc in=g_xf1fpmc.bin out=og_xf1fpmc.bin warm=5 iters=10 reps=1")
items+=("n_xf1fpmc_eval|model=xf1fpmc.nnc in=in_a3.bin out=oe_xf1fpmc.bin warm=50 iters=200 reps=2")
items+=("n_xpnmsr_gold|model=xpnmsr.nnc in=g_xpnmsr.bin out=og_xpnmsr.bin warm=5 iters=10 reps=1")
items+=("n_xpnmsr_eval|model=xpnmsr.nnc in=eval3ch.bin out=oe_xpnmsr.bin warm=50 iters=200 reps=2")
for rep in r1 r2; do
  items+=("n_xf1fpmc_pipe_$rep|model=xf1fpmc.nnc in=gray13.bin out=kn_xf1fpmc_$rep.bin pipe=1 passes=10 post=2 threads=1")
  items+=("n_xpnmsr_pipe_$rep|model=xpnmsr.nnc in=gray13.bin out=kn_xpnmsr_$rep.bin pipe=1 passes=10 post=5 oc=128")
  items+=("n_xfpmc_pipe_$rep|model=xfpmc.nnc in=gray13.bin out=kn_xfpmc_$rep.bin pipe=1 passes=10 post=2 threads=1")
done
items+=("m_L4s256_a16_eval|model=L4s256_a16.nnc in=lg_x13.bin out=oe_L4s256_a16.bin warm=20 iters=200 reps=2")
items+=("m_L4s256_a16_pipe_r1|model=L4s256_a16.nnc in=lg_pairs13.bin out=ml_L4s256_a16_r1.bin lgpipe=1 lgn=256 passes=5 wr=lg_wr.bin")
items+=("m_L4s256_a16_pipe_r2|model=L4s256_a16.nnc in=lg_pairs13.bin out=ml_L4s256_a16_r2.bin lgpipe=1 lgn=256 passes=5 wr=lg_wr.bin")
DEADLINE=$(date -d "07:40" +%s)
for it in "${items[@]}"; do
  t=${it%%|*}; a=${it#*|}; n=0
  while :; do
    [ $(date +%s) -ge $DEADLINE ] && { log "DEADLINE 07:40 reached before $t -> stop measuring"; break 2; }
    bash run1.sh $t "$a" > /dev/null; rc=$?
    [ $rc -eq 0 ] && grep -q "nncrun end" runs/$t.applog.txt && break
    [ $rc -eq 3 ] && { log "LIMIT stop"; break 2; }
    n=$((n+1)); [ $n -ge 60 ] && { log "gate timeout at $t"; break 2; }
    sleep 60
  done
  log "ITEM $t done $(cat runs/$t.state_before.txt) | after $(cat runs/$t.state_after.txt)"
done
log "RUNS DONE, pulling"
for f in $($A shell $P ls files | tr -d '\r' | grep -E '^(o[eg]_|ml_|kn_)'); do $A exec-out $P cat files/$f > runs/$f; done
$A shell $P sh -c "'cd files; md5sum o?_* ml_* kn_*'" > runs/md5_out_batch11.txt
$A uninstall com.scan3d.nncprobe; log "pm=$($A shell pm list packages | grep -c nncprobe)"
echo BATCH_DONE
