#!/bin/bash
# gsmap 2026-11-26 overnight W2: aisle / sheet cleanup on ffsonly2 (poses ba2, depth ffsonly_ba2/fused), from ffsonly2's carve output.
# usage: run_w2.sh step [step ...]. Output only in runs/gsmap_20261125_121801_v2/w2 (never overwrites other runs).
# One heavy step at a time (waits for free GPU / no heavy python), nice 19, 8 cores. Logs START / END per step in w2/run.log.
#   votes1            gsm_w2clean votes on ffsonly2 carve.ply                       -> w2/votes1.npz
#   votes2            second pass, z-buffer without the pass-1 candidates (CAND)    -> w2/votes2.npz
#   cfg NAME ARGS..   apply (NAME) -> mild NC prune -> E3c -> eval2 (ba2 ruler, ply) -> w2/cfg_NAME/
#   chain NAME        texture + atlas + seampad + eval2 (glb) for w2/cfg_NAME       -> w2/final/
cd "$HOME/scanner/StereoDepthSeg-gsmap"
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w2; B2=$N/ffsonly_ba2/b; E2=$N/ffsonly_ba2/evalrun_newcams2; E=$N/ffsonly_ba/evalrun_newcams
CAMS=$N/ba_partners2/run/dump_ba/cams.npz; NEWDUMP=$N/ba_partners2/run/dump_ba; FUSED=$N/ffsonly_ba2/fused
mkdir -p $O/logs
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 NUMEXPR_NUM_THREADS=8 PYTHONUNBUFFERED=1
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
W=scan3d/gsmap/gsm_w2clean.py; VT=${VOTES:-votes1c}   # votes1.npz had the inf-window bug, votes1b = round 1 (no backed fields)
log() { echo "$(date '+%F %T') $*" >> $O/run.log; }
wait_free() {
  local n=0
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|ffs[a-z0-9_]*\.py|kf_ba' | grep -c .)
    [ "$g" = 0 ] && [ "$p" = 0 ] && return
    sleep 30; n=$((n+1)); [ $n -gt 60 ] && { log "FAIL w2 wait_free > 30 min"; exit 1; }
  done
}
run_step() {
  local st=$1; shift
  wait_free; log "START $st"
  "$@" >> $O/logs/$st.log 2>&1; rc=$?
  log "END $st rc $rc"
  [ $rc = 0 ] || { log "FAIL w2 $st rc $rc"; exit 1; }
}
st=$1; shift
case $st in
  votes1) run_step votes_$VT $GS $W votes $B2/carve.ply $O/$VT.npz --fused $FUSED --cams $CAMS ;;
  votes2) run_step votes2 $GS $W votes $B2/carve.ply $O/votes2.npz --fused $FUSED --cams $CAMS --drop $O/drop1.npy ;;
  cfg) nm=$1; shift; C=$O/cfg_$nm; mkdir -p $C
       run_step apply_$nm $GS $W apply $B2/carve.ply $O/$VT.npz $C --base-votes $B2/support/support_votes.npz "$@"
       run_step prunec_$nm env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_ffs_support.py $C/w2.ply $C/support --fused $FUSED --cams $CAMS --votes $C/w2_votes.npz --comp-area 0.25
       run_step e3c_$nm env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_e3c.py $C/support/support_NC.ply $C/e3c.ply --ceil-y -1.855
       run_step eval_$nm env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_eval2.py $C/e3c.ply $C/eval2_ba2_ply.json ;;
  chain) nm=$1; C=$O/cfg_$nm; F=$O/final; mkdir -p $F
       cp -n $C/e3c.ply $F/e3c.ply
       run_step tex env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_texture2.py --dump $NEWDUMP --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 \
           $F/e3c.ply $F/tex_121801_w2.glb
       run_step atlas env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_atlas8k.py $F/tex_121801_w2.glb
       run_step pad $GS scan3d/gsmap/gsm_seampad.py $F/tex_121801_w2.glb $F/tex_121801_w2_pad.glb
       run_step eval_e2 env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_eval2.py $F/tex_121801_w2.glb $F/eval_newcams2_w2.json --tex-json $F/tex_121801_w2.json
       run_step eval_e env GSMAP_RUN=$E $GS scan3d/gsmap/gsm_eval2.py $F/tex_121801_w2.glb $F/eval_newcams_w2.json --tex-json $F/tex_121801_w2.json ;;
  baseeval) run_step baseeval env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_eval2.py $B2/e3c.ply $O/base_ffsonly2_e3c_eval2_ply.json ;;
  judge) mode=$1; sub=$2; shift 2; run_step judge_${mode}_$sub $GS scan3d/gsmap/gsm_w2judge.py $mode $O/$sub "$@" ;;
  aisle) # FFS support votes (gsm_ffs_support.py, unchanged) on the final e3c meshes of ffsonly2 and W2, then the aisle ROI share
       run_step aisle_sup_ffsonly2 env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_ffs_support.py $B2/e3c.ply $O/aisle/ffsonly2_e3c --fused $FUSED --cams $CAMS --n 2
       run_step aisle_sup_w2 env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_ffs_support.py $O/final/e3c.ply $O/aisle/w2_e3c --fused $FUSED --cams $CAMS --n 2
       run_step aisle_roi $GS scan3d/gsmap/gsm_relax_aisle.py $O/aisle/aisle_roi.json \
           ffsonly2_carve=$B2/carve.ply:$B2/support/support_votes.npz ffsonly2_e3c=$B2/e3c.ply:$O/aisle/ffsonly2_e3c/support_votes.npz \
           w2_e3c=$O/final/e3c.ply:$O/aisle/w2_e3c/support_votes.npz ;;
  *) log "unknown step $st"; exit 2 ;;
esac
