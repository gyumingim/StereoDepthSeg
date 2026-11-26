#!/bin/bash
# gsmap 2026-11-27 overnight W5 (L3): ONE 2DGS retrain of ffsonly2 with BA sparse depth targets (gsm_w5_sparse.py -> w5/sparse) added to the
# depth loss on non-FFS pixels (gsm_train.py --sparse-depth). Everything else = ffsonly2 (run_ffs_relax.sh train .. pad, eval), same params.
# Then rulers: eval2 (ba2 + bap evalruns), R ruler (gsm_w2judge geo + photo, ffsonly2 re-measured), F ruler (gsm_w5_ruler.py -> w5/fruler).
# usage: run_w5.sh step [step ...]. Output only in runs/gsmap_20261125_121801_v2/w5.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w5; B=$O/b; B2=$N/ffsonly_ba2/b; E2=$N/ffsonly_ba2/evalrun_newcams2; E=$N/ffsonly_ba/evalrun_newcams
NEWDUMP=$N/ba_partners2/run/dump_ba; FUSED=$N/ffsonly_ba2/fused
mkdir -p $O/logs
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 NUMEXPR_NUM_THREADS=8 PYTHONUNBUFFERED=1
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
log() { echo "$(date '+%F %T') $*" >> $O/run.log; }
wait_free() {
  local n=0
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|ffs[a-z0-9_]*\.py|kf_ba' | grep -c .)
    [ "$g" = 0 ] && [ "$p" = 0 ] && return
    sleep 30; n=$((n+1)); [ $n -gt 60 ] && { log "FAIL w5 wait_free > 30 min (gpu $g, heavy python $p)"; exit 1; }
  done
}
run_step() {
  local st=$1; shift
  wait_free; log "START $st"
  timeout 3600 "$@" >> $O/logs/$st.log 2>&1; rc=$?
  log "END $st rc $rc"
  [ $rc = 0 ] || { log "FAIL w5 $st rc $rc"; exit 1; }
}
for st in "$@"; do
case $st in
  train) source $HOME/scanner/gsplat_env/env.sh
         [ -e $B/gs/summary.json ] && { log "FAIL w5 train: $B/gs exists (never overwrite)"; exit 1; }
         run_step train $GS scan3d/gsmap/gsm_train.py $NEWDUMP --fused $FUSED --out $B/gs --surfel 1 --mask-border 1 --fill-w 0.1 --aniso-max 5 --aniso-w 1.0 \
             --max-scale 0.05 --voxel 0.025 --max-gs 2500000 --steps 30000 --exposure 1 --depth-sigma fused --hold 0 --sparse-depth $O/sparse ;;
  mesh) source $HOME/scanner/gsplat_env/env.sh
        run_step mesh env GSMAP_RUN=$N GSMAP_WELD=1 GSMAP_TRUNC=0.04 GSMAP_ZMAX=5.0 GSMAP_ZSIG=1 $GS scan3d/gsmap/gsm_mesh2d.py $B/gs $B/gs/mesh.ply ;;
  v3fix) run_step v3fix env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_v3fix.py $B/gs/mesh.ply $B/v3fix.ply ;;
  carve) run_step carve env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_carve.py $B/v3fix.ply $B/carve.ply --min-viol 5 --island-area 0 --min-tri 0 ;;
  prune) run_step prune env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_ffs_support.py $B/carve.ply $B/support --fused $FUSED --cams $NEWDUMP/cams.npz --n 1,2 --island 0.01
         run_step prunec env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_ffs_support.py $B/carve.ply $B/support --fused $FUSED --cams $NEWDUMP/cams.npz \
             --votes $B/support/support_votes.npz --comp-area 0.25 ;;
  e3c) run_step e3c env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_e3c.py $B/support/support_NC.ply $B/e3c.ply --ceil-y -1.855 ;;
  tex) run_step tex env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_texture2.py --dump $NEWDUMP --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 \
           $B/e3c.ply $B/tex_121801_w5.glb
       run_step atlas env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_atlas8k.py $B/tex_121801_w5.glb ;;
  pad) run_step pad $GS scan3d/gsmap/gsm_seampad.py $B/tex_121801_w5.glb $B/tex_121801_w5_pad.glb ;;
  eval) run_step eval_e2 env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_eval2.py $B/tex_121801_w5.glb $B/eval_newcams2_w5.json --tex-json $B/tex_121801_w5.json
        run_step eval_e env GSMAP_RUN=$E $GS scan3d/gsmap/gsm_eval2.py $B/tex_121801_w5.glb $B/eval_newcams_w5.json --tex-json $B/tex_121801_w5.json ;;
  judge) M="ffsonly2=$B2/tex_121801_ffsonly2.glb:ba2 W5=$B/tex_121801_w5.glb:ba2"
         run_step judge_geo $GS scan3d/gsmap/gsm_w2judge.py geo $O/rjudge $M
         run_step judge_photo $GS scan3d/gsmap/gsm_w2judge.py photo $O/rjudge $M ;;
  table) run_step table $GS scan3d/gsmap/gsm_w2table.py $O/table.json $O/rjudge/geo.json ffsonly2=$B2/eval_newcams2_ffsonly2.json W5=$B/eval_newcams2_w5.json ;;
  votes) run_step votes_e3c env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_ffs_support.py $B/e3c.ply $O/aisle/w5_e3c --fused $FUSED --cams $NEWDUMP/cams.npz --n 2 ;;
  fruler) run_step fruler $GS scan3d/gsmap/gsm_w5_ruler.py all ;;
  *) log "unknown step $st"; exit 2 ;;
esac
done
