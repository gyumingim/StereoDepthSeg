#!/bin/bash
# gsmap 2026-11-27 overnight W9: SM2 (W8 seam levelling soft 0.5) + ONE W9 fix at a time on the FIXED nC mesh (poses bap), then combos,
# then the final recipe on ffsonly2 (ffsonly_ba2/b/e3c.ply, poses ba2 = ba_partners2 dump, same args as its gsm_texture2 chain).
#   o1 = --seam-occl 1 | lu = --seam-mode luma | ga = --seam-mode gain | wt = --seam-target w | wr = --seam-target wr | fi = --seam-fill 1
#   cX = combo: run_w9.sh "combo NAME opts..." ; judge NAME... ; detail ; f2 OPTS (ffsonly2)
# Output only in runs/gsmap_20261125_121801_v2/w9 (never overwrites).
cd "$HOME/scanner/StereoDepthSeg-gsmap"
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w9; NC=$N/ffsonly_ba/support_test/nC; DUMP=$N/ba_partners/run/dump_ba
DUMP2=$N/ba_partners2/run/dump_ba; B2=$N/ffsonly_ba2/b; E2=$N/ffsonly_ba2/evalrun_newcams2
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
    sleep 30; n=$((n+1)); [ $n -gt 60 ] && { log "FAIL w9 wait_free > 30 min (gpu $g, heavy python $p)"; exit 1; }
  done
}
run_step() {
  local st=$1; shift
  wait_free; log "START $st"
  timeout 3600 "$@" >> $O/logs/$st.log 2>&1; rc=$?
  log "END $st rc $rc"
  [ $rc = 0 ] || { log "FAIL w9 $st rc $rc"; exit 1; }
}
SM2="--seam-level 1 --seam-soft 0.5"
TEX="scan3d/gsmap/gsm_texture3.py --dump $DUMP --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 $SM2"
variant() {   # $1 = name, rest = extra options (on top of SM2)
  local v=$1; shift; mkdir -p $O/$v; G=$O/$v/tex_121801_nC_$v.glb
  [ -e $G ] && { log "FAIL w9 $G exists"; exit 1; }
  run_step tex_$v $GS $TEX "$@" $NC/e3c.ply $G
  run_step atlas_$v $GS scan3d/gsmap/gsm_atlas8k.py $G
}
st=$1; shift
case $st in
  var) variant "$@" ;;
  judge) out=$1; shift; M="nC=$NC/tex_121801_ffssupport_NC.glb:bap SM2=$N/w8/sm2/tex_121801_nC_sm2.glb:bap"
         for v in "$@"; do M="$M $v=$O/$v/tex_121801_nC_$v.glb:bap"; done
         [ -e $O/$out/photo.json ] && { log "FAIL w9 $out/photo.json exists"; exit 1; }
         run_step judge_$out $GS scan3d/gsmap/gsm_w2judge.py photo $O/$out $M ;;
  detail) out=$1; shift; M="nC=$NC/tex_121801_ffssupport_NC.glb SM2=$N/w8/sm2/tex_121801_nC_sm2.glb"
         for v in "$@"; do M="$M $v=$O/$v/tex_121801_nC_$v.glb"; done
         run_step detail_$out $GS scan3d/gsmap/gsm_w9detail.py $O/$out.json bap $M ;;
  f2) mkdir -p $O/f2; G=$O/f2/tex_121801_ffsonly2_w9.glb; [ -e $G ] && { log "FAIL w9 $G exists"; exit 1; }
      run_step tex_f2 env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_texture3.py --dump $DUMP2 --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 $SM2 "$@" $B2/e3c.ply $G
      run_step atlas_f2 env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_atlas8k.py $G
      run_step pad_f2 $GS scan3d/gsmap/gsm_seampad.py $G $O/f2/tex_121801_ffsonly2_w9_pad.glb ;;
  f2judge) [ -e $O/rjudge_f2/photo.json ] && { log "FAIL w9 rjudge_f2 exists"; exit 1; }
      run_step judge_f2 $GS scan3d/gsmap/gsm_w2judge.py photo $O/rjudge_f2 ffsonly2=$B2/tex_121801_ffsonly2.glb:ba2 F2W9=$O/f2/tex_121801_ffsonly2_w9.glb:ba2
      run_step detail_f2 $GS scan3d/gsmap/gsm_w9detail.py $O/detail_f2.json ba2 ffsonly2=$B2/tex_121801_ffsonly2.glb F2W9=$O/f2/tex_121801_ffsonly2_w9.glb ;;
  *) log "unknown step $st"; exit 2 ;;
esac
log "STEP OK $st $*"
