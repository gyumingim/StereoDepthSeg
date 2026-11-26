#!/bin/bash
# gsmap 2026-11-27 overnight W6: clean variants C1 / C2 / C3 of ffsonly2 (gsm_w6clean.py), each: rule -> NC (comp 0.25 m2) -> E3c (--ceil-y -1.855)
# -> texture -> atlas8k -> seampad (same args as ffsonly2 / run_w5.sh) -> eval2 (ba2 + bap evalruns) -> e3c FFS votes (--n 2).
# Then R ruler (gsm_w2judge geo + photo, ffsonly2 re-measured), table, F ruler (F3 / F2), picture.
# usage: run_w6.sh step [step ...]. Output only in runs/gsmap_20261125_121801_v2/w6 (never overwrites).
cd "$HOME/scanner/StereoDepthSeg-gsmap"
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w6; B2=$N/ffsonly_ba2/b; E2=$N/ffsonly_ba2/evalrun_newcams2; E=$N/ffsonly_ba/evalrun_newcams
NEWDUMP=$N/ba_partners2/run/dump_ba; CAMS=$NEWDUMP/cams.npz; FUSED=$N/ffsonly_ba2/fused
mkdir -p $O/logs
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 NUMEXPR_NUM_THREADS=8 PYTHONUNBUFFERED=1
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
W=scan3d/gsmap/gsm_w6clean.py
log() { echo "$(date '+%F %T') $*" >> $O/run.log; }
wait_free() {
  local n=0
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|ffs[a-z0-9_]*\.py|kf_ba' | grep -c .)
    [ "$g" = 0 ] && [ "$p" = 0 ] && return
    sleep 30; n=$((n+1)); [ $n -gt 60 ] && { log "FAIL w6 wait_free > 30 min (gpu $g, heavy python $p)"; exit 1; }
  done
}
run_step() {
  local st=$1; shift
  wait_free; log "START $st"
  timeout 3600 "$@" >> $O/logs/$st.log 2>&1; rc=$?
  log "END $st rc $rc"
  [ $rc = 0 ] || { log "FAIL w6 $st rc $rc"; exit 1; }
}
variant() {   # $1 = 1|2|3
  local i=$1 C=$O/c$1; local args="--variant C$i"
  [ $i = 2 ] && args="$args --far 0.35"; [ $i = 3 ] && args="$args --far 0.20"
  run_step apply_c$i $GS $W apply $B2/carve.ply $O/vis.npz $B2/support/support_votes.npz $C $args
  run_step prunec_c$i env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_ffs_support.py $C/w6.ply $C/support --fused $FUSED --cams $CAMS --votes $C/w6_votes.npz --comp-area 0.25
  run_step e3c_c$i env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_e3c.py $C/support/support_NC.ply $C/e3c.ply --ceil-y -1.855
  run_step tex_c$i env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_texture2.py --dump $NEWDUMP --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 \
      $C/e3c.ply $C/tex_121801_c$i.glb
  run_step atlas_c$i env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_atlas8k.py $C/tex_121801_c$i.glb
  run_step pad_c$i $GS scan3d/gsmap/gsm_seampad.py $C/tex_121801_c$i.glb $C/tex_121801_c${i}_pad.glb
  run_step eval_e2_c$i env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_eval2.py $C/tex_121801_c$i.glb $C/eval_newcams2_c$i.json --tex-json $C/tex_121801_c$i.json
  run_step eval_e_c$i env GSMAP_RUN=$E $GS scan3d/gsmap/gsm_eval2.py $C/tex_121801_c$i.glb $C/eval_newcams_c$i.json --tex-json $C/tex_121801_c$i.json
  run_step votes_e3c_c$i env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_ffs_support.py $C/e3c.ply $O/aisle/c${i}_e3c --fused $FUSED --cams $CAMS --n 2
}
for st in "$@"; do
case $st in
  vis) [ -e $O/vis.npz ] && { log "FAIL w6 vis exists"; exit 1; }
       run_step vis $GS $W vis $B2/carve.ply $O/vis.npz --cams $CAMS ;;
  c1) variant 1 ;;
  c2) variant 2 ;;
  c3) variant 3 ;;
  judge) M="ffsonly2=$B2/tex_121801_ffsonly2.glb:ba2 C1=$O/c1/tex_121801_c1.glb:ba2 C2=$O/c2/tex_121801_c2.glb:ba2 C3=$O/c3/tex_121801_c3.glb:ba2"
         run_step judge_geo $GS scan3d/gsmap/gsm_w2judge.py geo $O/rjudge $M
         run_step judge_photo $GS scan3d/gsmap/gsm_w2judge.py photo $O/rjudge $M ;;
  table) run_step table $GS scan3d/gsmap/gsm_w2table.py $O/table.json $O/rjudge/geo.json ffsonly2=$B2/eval_newcams2_ffsonly2.json \
             C1=$O/c1/eval_newcams2_c1.json C2=$O/c2/eval_newcams2_c2.json C3=$O/c3/eval_newcams2_c3.json ;;
  fruler) run_step fruler $GS $W fruler ;;
  pic) run_step pic $GS $W pic $O/w6_clean_121801.jpg ;;
  *) log "unknown step $st"; exit 2 ;;
esac
done
