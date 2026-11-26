#!/bin/bash
# gsmap 2026-11-26 overnight W3: vertical wall plane fill on ffsonly2 (ffsonly_ba2/b/e3c.ply, poses ba2, depth ffsonly_ba2/fused).
# Base switched from W2 to ffsonly2 by the coordinator (Fable judged W2 a wash). Output only in runs/gsmap_20261125_121801_v2/w3.
# Chain: gsm_w3wall (fill) -> gsm_texture2 (same args as ffsonly2 / W2) -> atlas8k -> seampad -> eval2 (ba2 + bap evalruns)
#        -> R ruler geo + photo (gsm_w2judge: ffsonly2 baseline, W2, W3) -> table -> picture.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w3; VAR=${VAR:-fill}; F=$O/$VAR;   # VAR = variant folder (fill = first run, fillB = --min-comp 0.25)
 B2=$N/ffsonly_ba2/b; E2=$N/ffsonly_ba2/evalrun_newcams2; E=$N/ffsonly_ba/evalrun_newcams
NEWDUMP=$N/ba_partners2/run/dump_ba; FUSED=$N/ffsonly_ba2/fused
mkdir -p $O/logs
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 NUMEXPR_NUM_THREADS=8 PYTHONUNBUFFERED=1
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
log() { echo "$(date '+%F %T') $*" >> $O/run.log; }
wait_free() {
  local n=0
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|ffs[a-z0-9_]*\.py|kf_ba' | grep -v gsm_rjudge | grep -c .)
    [ "$g" = 0 ] && [ "$p" = 0 ] && return
    sleep 30; n=$((n+1)); [ $n -gt 60 ] && { log "FAIL w3 wait_free > 30 min"; exit 1; }
  done
}
run_step() {
  local st=$1; shift
  wait_free; log "START $st"
  timeout 1800 "$@" >> $O/logs/$st.log 2>&1; rc=$?
  log "END $st rc $rc"
  [ $rc = 0 ] || { log "FAIL w3 $st rc $rc"; exit 1; }
}
for st in "$@"; do
case $st in
  fill) run_step fill_$VAR env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_w3wall.py $B2/e3c.ply $F --fused $FUSED --floor-y 0.847 --ceil-y -1.855 $FILLARGS ;;
  tex) run_step tex_$VAR env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_texture2.py --dump $NEWDUMP --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 \
           $F/w3.ply $F/tex_121801_w3.glb
       run_step atlas_$VAR env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_atlas8k.py $F/tex_121801_w3.glb
       run_step pad_$VAR $GS scan3d/gsmap/gsm_seampad.py $F/tex_121801_w3.glb $F/tex_121801_w3_pad.glb ;;
  eval) run_step eval_e2_$VAR env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_eval2.py $F/tex_121801_w3.glb $F/eval_newcams2_w3.json --tex-json $F/tex_121801_w3.json
        run_step eval_e_$VAR env GSMAP_RUN=$E $GS scan3d/gsmap/gsm_eval2.py $F/tex_121801_w3.glb $F/eval_newcams_w3.json --tex-json $F/tex_121801_w3.json ;;
  judge) M="ffsonly2=$B2/tex_121801_ffsonly2.glb:ba2 W2=$N/w2/final/tex_121801_w2.glb:ba2 W3=$F/tex_121801_w3.glb:ba2"
        run_step judge_geo_$VAR $GS scan3d/gsmap/gsm_w2judge.py geo $O/rjudge_$VAR $M
        run_step judge_photo_$VAR $GS scan3d/gsmap/gsm_w2judge.py photo $O/rjudge_$VAR $M ;;
  table) run_step table_$VAR $GS scan3d/gsmap/gsm_w2table.py $O/table_$VAR.json $O/rjudge_$VAR/geo.json ffsonly2=$B2/eval_newcams2_ffsonly2.json \
           W2=$N/w2/final/eval_newcams2_w2.json W3=$F/eval_newcams2_w3.json ;;
  pic) run_step pic_$VAR $GS scan3d/gsmap/gsm_w3pic.py $E2 $O/w3_121801_$VAR.jpg $F/w3.ply $F/w3_fill.npz \
           ffsonly2=$B2/tex_121801_ffsonly2_pad.glb W2=$N/w2/final/tex_121801_w2_pad.glb W3=$F/tex_121801_w3_pad.glb ;;
  *) log "unknown step $st"; exit 2 ;;
esac
done
