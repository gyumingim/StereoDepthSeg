#!/bin/bash
# gsmap 2026-11-27 overnight W8: texture-quality variants on the FIXED nC mesh (support_test/nC/e3c.ply, poses bap = ba_partners).
# Each variant = gsm_texture3.py with the nC args (chunk 0.75, fill-all, drop view 0, no texture2 gain) + ONE option:
#   b0 = no option (reproducibility of nC), sl = --sharp-local, rg = --robust-gain, sm = --seam-level, hr = --atlas 12288 --cap 16384 (no 8k).
# b0/sl/rg/sm then gsm_atlas8k.py (as nC). judge = gsm_w2judge.py photo (R ruler, held-out partner frames) with the nC GLB re-measured.
# usage: run_w8.sh step [step ...]. Output only in runs/gsmap_20261125_121801_v2/w8 (never overwrites).
cd "$HOME/scanner/StereoDepthSeg-gsmap"
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w8; NC=$N/ffsonly_ba/support_test/nC; DUMP=$N/ba_partners/run/dump_ba
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
    sleep 30; n=$((n+1)); [ $n -gt 60 ] && { log "FAIL w8 wait_free > 30 min (gpu $g, heavy python $p)"; exit 1; }
  done
}
run_step() {
  local st=$1; shift
  wait_free; log "START $st"
  timeout 3600 "$@" >> $O/logs/$st.log 2>&1; rc=$?
  log "END $st rc $rc"
  [ $rc = 0 ] || { log "FAIL w8 $st rc $rc"; exit 1; }
}
TEX="scan3d/gsmap/gsm_texture3.py --dump $DUMP --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1"
variant() {   # $1 = name, rest = extra options
  local v=$1; shift; mkdir -p $O/$v; G=$O/$v/tex_121801_nC_$v.glb
  [ -e $G ] && { log "FAIL w8 $G exists"; exit 1; }
  run_step tex_$v $GS $TEX "$@" $NC/e3c.ply $G
}
for st in "$@"; do
case $st in
  b0) variant b0; run_step atlas_b0 $GS scan3d/gsmap/gsm_atlas8k.py $O/b0/tex_121801_nC_b0.glb ;;
  sl) variant sl --sharp-local 1; run_step atlas_sl $GS scan3d/gsmap/gsm_atlas8k.py $O/sl/tex_121801_nC_sl.glb ;;
  rg) variant rg --robust-gain 1; run_step atlas_rg $GS scan3d/gsmap/gsm_atlas8k.py $O/rg/tex_121801_nC_rg.glb ;;
  sm) variant sm --seam-level 1; run_step atlas_sm $GS scan3d/gsmap/gsm_atlas8k.py $O/sm/tex_121801_nC_sm.glb ;;
  hr) variant hr --atlas 12288 --cap 16384 ;;
  judge) M="nC=$NC/tex_121801_ffssupport_NC.glb:bap B0=$O/b0/tex_121801_nC_b0.glb:bap SL=$O/sl/tex_121801_nC_sl.glb:bap RG=$O/rg/tex_121801_nC_rg.glb:bap SM=$O/sm/tex_121801_nC_sm.glb:bap HR=$O/hr/tex_121801_nC_hr.glb:bap"
         [ -e $O/rjudge/photo.json ] && { log "FAIL w8 rjudge/photo.json exists"; exit 1; }
         run_step judge_photo $GS scan3d/gsmap/gsm_w2judge.py photo $O/rjudge $M ;;
  sm2) variant sm2 --seam-level 1 --seam-soft 0.5; run_step atlas_sm2 $GS scan3d/gsmap/gsm_atlas8k.py $O/sm2/tex_121801_nC_sm2.glb ;;
  judge2) M="nC=$NC/tex_121801_ffssupport_NC.glb:bap B0=$O/b0/tex_121801_nC_b0.glb:bap SM=$O/sm/tex_121801_nC_sm.glb:bap SM2=$O/sm2/tex_121801_nC_sm2.glb:bap"
         [ -e $O/rjudge2/photo.json ] && { log "FAIL w8 rjudge2/photo.json exists"; exit 1; }
         run_step judge2_photo $GS scan3d/gsmap/gsm_w2judge.py photo $O/rjudge2 $M ;;
  *) log "unknown step $st"; exit 2 ;;
esac
done
log "STEPS OK $*"
