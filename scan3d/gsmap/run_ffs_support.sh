#!/bin/bash
# gsmap 2026-11-26: FFS-support prune test on the ffsonly_np map (no retrain). usage: run_ffs_support.sh step [step ...]
# prune: gsm_ffs_support.py on newpose/carve.ply (pre-E3c) with fused/ (FFS Z,F,S = training depth) and the new BA poses -> support_N1/N2.ply
# then per N the same chain/params as run_ffsonly_np.sh: e3c --ceil-y -1.855, texture2 chunk 0.75 no-gain drop 0, atlas8k, eval2 (new-pose ruler).
# Output only in ffsonly_ba/support_test. One heavy job at a time, nice 19, 8 cores.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/ffsonly_ba; E=$O/evalrun_newcams; P=$O/newpose; Q=$O/support_test; mkdir -p $Q/logs
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 NUMEXPR_NUM_THREADS=8 PYTHONUNBUFFERED=1 GSMAP_RUN=$E
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
NEWDUMP=$N/ba_partners/run/dump_ba
wait_free() {
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|ffs[a-z0-9_]*\.py|kf_ba' | grep -c .)
    [ "$g" = 0 ] && [ "$p" = 0 ] && return
    echo "$(date '+%F %T') wait: gpu procs $g, heavy python $p" >> $Q/logs/wait.log; sleep 30
  done
}
run_step() {
  local st=$1; shift
  for try in 1 2; do
    wait_free
    echo "$(date '+%F %T') START $st try $try" >> $Q/run.log
    "$@" >> $Q/logs/$st.log 2>&1; rc=$?
    echo "$(date '+%F %T') END $st rc $rc" >> $Q/run.log
    [ $rc = 0 ] && return 0
  done
  echo "$(date '+%F %T') STOP: $st failed twice" >> $Q/run.log; exit 1
}
for st in "$@"; do
  case $st in
    prune) run_step prune $GS scan3d/gsmap/gsm_ffs_support.py $P/carve.ply $Q --fused $O/fused --cams $NEWDUMP/cams.npz --n 1,2 --island 0.01 ;;
    prunec) run_step prunec $GS scan3d/gsmap/gsm_ffs_support.py $P/carve.ply $Q --fused $O/fused --cams $NEWDUMP/cams.npz --votes $Q/support_votes.npz --comp-area 0.25 ;;
    N1|N2|NC) n=${st#N}; X=$Q/n$n; mkdir -p $X
      run_step e3c_$st $GS scan3d/gsmap/gsm_e3c.py $Q/support_N$n.ply $X/e3c.ply --ceil-y -1.855
      run_step tex_$st $GS scan3d/gsmap/gsm_texture2.py --dump $NEWDUMP --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 \
        $X/e3c.ply $X/tex_121801_ffssupport_N$n.glb
      run_step atlas_$st $GS scan3d/gsmap/gsm_atlas8k.py $X/tex_121801_ffssupport_N$n.glb
      run_step eval_$st $GS scan3d/gsmap/gsm_eval2.py $X/tex_121801_ffssupport_N$n.glb $X/eval_newcams_ffssupport_N$n.json --tex-json $X/tex_121801_ffssupport_N$n.json ;;
  esac
done
echo "$(date '+%F %T') DONE $*" >> $Q/run.log
