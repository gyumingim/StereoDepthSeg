#!/bin/bash
# gsmap 2026-11-26: FFS partners for the 166 keyframes without a wide partner (gsm_ffs_relax.py) -> BA2 -> FFS -> fuse -> 2DGS retrain ->
# mesh chain mirrored from run_ffsonly_np.sh + mild prune NC (run_ffs_support.sh order: carve -> prune -> e3c) on the BA2 poses.
# usage: run_ffs_relax.sh step [step ...]. Outputs: ba_partners2/, ffs_relax/, ffsonly_ba2/ (never overwrites older runs).
# Power safety: before every step wait until no GPU compute process and no other heavy python (gsm_*.py, train, ffs, kf_ba) runs;
# one step at a time; CPU under nice 19 on 8 cores. A step that fails twice stops the chain.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/ffsonly_ba2; B=$O/b; E2=$O/evalrun_newcams2; E=$N/ffsonly_ba/evalrun_newcams; mkdir -p $O/logs
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 NUMEXPR_NUM_THREADS=8 PYTHONUNBUFFERED=1
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
FF="taskset -c 0-7 nice -n 19 env FFS_ROOT=$HOME/scanner/StereoDepthSeg $HOME/scanner/StereoDepthSeg/venv_ffs/bin/python -W ignore"
S=scan3d/gsmap/gsm_ffs_relax.py
NEWDUMP=$N/ba_partners2/run/dump_ba
D=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo
wait_free() {
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|ffs[a-z0-9_]*\.py|kf_ba' | grep -c .)
    [ "$g" = 0 ] && [ "$p" = 0 ] && return
    echo "$(date '+%F %T') wait: gpu procs $g, heavy python $p" >> $O/logs/wait.log; sleep 30
  done
}
run_step() {
  local st=$1; shift
  for try in 1 2; do
    wait_free
    echo "$(date '+%F %T') START $st try $try" >> $O/run.log
    "$@" >> $O/logs/$st.log 2>&1; rc=$?
    echo "$(date '+%F %T') END $st rc $rc" >> $O/run.log
    [ $rc = 0 ] && return 0
  done
  echo "$(date '+%F %T') STOP: $st failed twice" >> $O/run.log; exit 1
}
for st in "$@"; do
  case $st in
    plan|frames|post|fuse|combine) run_step $st $GS $S $st ;;
    ba) run_step ba $FF $S ba ;;
    ffs|measure) run_step $st $FF $S $st ;;
    fused) run_step fused $GS $S fused --name s4m_r ${DROP:+--drop $DROP} ${REVERT:+--revert-idx $REVERT} ;;
    evalrun) mkdir -p $E2/dump_ba $E2/eval2/ref
             ln -sfn $NEWDUMP/cams.npz $E2/dump_ba/cams.npz; ln -sfn $N/dump_ba/img $E2/dump_ba/img
             for f in $N/eval2/ref/*; do ln -sfn $f $E2/eval2/ref/$(basename $f); done ;;
    train) source $HOME/scanner/gsplat_env/env.sh
           run_step train $GS scan3d/gsmap/gsm_train.py $NEWDUMP --fused $O/fused --out $B/gs --surfel 1 --mask-border 1 --fill-w 0.1 --aniso-max 5 --aniso-w 1.0 \
             --max-scale 0.05 --voxel 0.025 --max-gs 2500000 --steps 30000 --exposure 1 --depth-sigma fused --hold 0 ;;
    mesh) source $HOME/scanner/gsplat_env/env.sh
          run_step mesh env GSMAP_RUN=$N GSMAP_WELD=1 GSMAP_TRUNC=0.04 GSMAP_ZMAX=5.0 GSMAP_ZSIG=1 $GS scan3d/gsmap/gsm_mesh2d.py $B/gs $B/gs/mesh.ply ;;
    v3fix) run_step v3fix env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_v3fix.py $B/gs/mesh.ply $B/v3fix.ply ;;
    carve) run_step carve env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_carve.py $B/v3fix.ply $B/carve.ply --min-viol 5 --island-area 0 --min-tri 0 ;;
    prune) run_step prune env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_ffs_support.py $B/carve.ply $B/support --fused $O/fused --cams $NEWDUMP/cams.npz --n 1,2 --island 0.01
           run_step prunec env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_ffs_support.py $B/carve.ply $B/support --fused $O/fused --cams $NEWDUMP/cams.npz \
             --votes $B/support/support_votes.npz --comp-area 0.25 ;;
    e3c) run_step e3c env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_e3c.py $B/support/support_NC.ply $B/e3c.ply --ceil-y -1.855 ;;
    tex) run_step tex env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_texture2.py --dump $NEWDUMP --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 \
           $B/e3c.ply $B/tex_121801_ffsonly2.glb
         run_step atlas env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_atlas8k.py $B/tex_121801_ffsonly2.glb ;;
    pad) run_step pad $GS scan3d/gsmap/gsm_seampad.py $B/tex_121801_ffsonly2.glb $B/tex_121801_ffsonly2_pad.glb ;;
    eval) run_step eval_e2 env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_eval2.py $B/tex_121801_ffsonly2.glb $B/eval_newcams2_ffsonly2.json --tex-json $B/tex_121801_ffsonly2.json
          run_step eval_e env GSMAP_RUN=$E $GS scan3d/gsmap/gsm_eval2.py $B/tex_121801_ffsonly2.glb $B/eval_newcams_ffsonly2.json --tex-json $B/tex_121801_ffsonly2.json ;;
  esac
done
echo "$(date '+%F %T') DONE $*" >> $O/run.log
