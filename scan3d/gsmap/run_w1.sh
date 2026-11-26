#!/bin/bash
# gsmap 2026-11-26 overnight W1: every stereo partner on one BA (BA3) -> FFS for the changed keyframe files -> fuse/gate/measure ->
# 2DGS retrain (ffsonly_ba settings) -> NC chain (run_ffs_relax.sh order) -> eval2 on the BA3 ruler and the ba_partners ruler.
# usage: run_w1.sh step [step ...]. Output only in runs/gsmap_20261125_121801_v2/w1 (never overwrites older runs).
# Power safety: before every step wait until no GPU compute process and no other heavy python runs; one step at a time;
# CPU under nice 19 on 8 cores. A step that fails twice stops the chain with FAIL w1.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w1; B=$O/b; E3=$O/evalrun_ba3; E=$N/ffsonly_ba/evalrun_newcams; mkdir -p $O/logs
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 NUMEXPR_NUM_THREADS=8 PYTHONUNBUFFERED=1
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
FF="taskset -c 0-7 nice -n 19 env FFS_ROOT=$HOME/scanner/StereoDepthSeg $HOME/scanner/StereoDepthSeg/venv_ffs/bin/python -W ignore"
S=scan3d/gsmap/gsm_w1.py
NEWDUMP=$O/ba3/run/dump_ba
D=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo
wait_free() {
  local n=0
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|ffs[a-z0-9_]*\.py|kf_ba' | grep -c .)
    [ "$g" = 0 ] && [ "$p" = 0 ] && return
    echo "$(date '+%F %T') wait: gpu procs $g, heavy python $p" >> $O/logs/wait.log; sleep 30; n=$((n+1))
    [ $n -gt 60 ] && { echo "$(date '+%F %T') FAIL w1 wait_free > 30 min" >> $O/run.log; exit 1; }
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
  echo "$(date '+%F %T') FAIL w1 $st failed twice" >> $O/run.log; exit 1
}
for st in "$@"; do
  case $st in
    inventory|frames|post|change|fuse23|prep|fuse|combine|fused) run_step $st $GS $S $st ;;
    ba) run_step ba $FF $S ba ;;
    ffs2|ffs3|wide|relax|gate) run_step $st $FF $S $st ;;
    measure) run_step measure $FF $S measure --name s4m,s4m_rg ;;
    evalrun) mkdir -p $E3/dump_ba $E3/eval2/ref
             ln -sfn $NEWDUMP/cams.npz $E3/dump_ba/cams.npz; ln -sfn $N/dump_ba/img $E3/dump_ba/img
             for f in $N/eval2/ref/*; do ln -sfn $f $E3/eval2/ref/$(basename $f); done
             echo "$(date '+%F %T') END evalrun rc 0" >> $O/run.log ;;
    train) source $HOME/scanner/gsplat_env/env.sh
           run_step train $GS scan3d/gsmap/gsm_train.py $NEWDUMP --fused $O/fused --out $B/gs --surfel 1 --mask-border 1 --fill-w 0.1 --aniso-max 5 --aniso-w 1.0 \
             --max-scale 0.05 --voxel 0.025 --max-gs 2500000 --steps 30000 --exposure 1 --depth-sigma fused --hold 0 ;;
    mesh) source $HOME/scanner/gsplat_env/env.sh
          run_step mesh env GSMAP_RUN=$N GSMAP_WELD=1 GSMAP_TRUNC=0.04 GSMAP_ZMAX=5.0 GSMAP_ZSIG=1 $GS scan3d/gsmap/gsm_mesh2d.py $B/gs $B/gs/mesh.ply ;;
    v3fix) run_step v3fix env GSMAP_RUN=$E3 $GS scan3d/gsmap/gsm_v3fix.py $B/gs/mesh.ply $B/v3fix.ply ;;
    carve) run_step carve env GSMAP_RUN=$E3 $GS scan3d/gsmap/gsm_carve.py $B/v3fix.ply $B/carve.ply --min-viol 5 --island-area 0 --min-tri 0 ;;
    prune) run_step prune env GSMAP_RUN=$E3 $GS scan3d/gsmap/gsm_ffs_support.py $B/carve.ply $B/support --fused $O/fused --cams $NEWDUMP/cams.npz --n 1,2 --island 0.01
           run_step prunec env GSMAP_RUN=$E3 $GS scan3d/gsmap/gsm_ffs_support.py $B/carve.ply $B/support --fused $O/fused --cams $NEWDUMP/cams.npz \
             --votes $B/support/support_votes.npz --comp-area 0.25 ;;
    e3c) run_step e3c env GSMAP_RUN=$E3 $GS scan3d/gsmap/gsm_e3c.py $B/support/support_NC.ply $B/e3c.ply --ceil-y -1.855 ;;
    tex) run_step tex env GSMAP_RUN=$E3 $GS scan3d/gsmap/gsm_texture2.py --dump $NEWDUMP --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 \
           $B/e3c.ply $B/tex_121801_w1.glb
         run_step atlas env GSMAP_RUN=$E3 $GS scan3d/gsmap/gsm_atlas8k.py $B/tex_121801_w1.glb ;;
    pad) run_step pad $GS scan3d/gsmap/gsm_seampad.py $B/tex_121801_w1.glb $B/tex_121801_w1_pad.glb ;;
    eval) run_step eval_e3 env GSMAP_RUN=$E3 $GS scan3d/gsmap/gsm_eval2.py $B/tex_121801_w1.glb $B/eval_ba3_w1.json --tex-json $B/tex_121801_w1.json
          run_step eval_e env GSMAP_RUN=$E $GS scan3d/gsmap/gsm_eval2.py $B/tex_121801_w1.glb $B/eval_newcams_w1.json --tex-json $B/tex_121801_w1.json ;;
    aisle) run_step aisle $GS scan3d/gsmap/gsm_relax_aisle.py $O/aisle_roi.json \
             np_nC_carve=$N/ffsonly_ba/newpose/carve.ply:$N/ffsonly_ba/support_test/support_votes.npz \
             ffsonly2_carve=$N/ffsonly_ba2/b/carve.ply:$N/ffsonly_ba2/b/support/support_votes.npz \
             w1_carve=$B/carve.ply:$B/support/support_votes.npz ;;
    sheet) run_step sheet env GSMAP_SPOTS="${SPOTS:?}" $GS scan3d/gsmap/gsm_crops.py $E3 $O/w1_121801.jpg \
             np=$N/ffsonly_ba/newpose/tex_121801_ffsonly_np.glb nC=$N/ffsonly_ba/support_test/nC/tex_121801_ffssupport_NC.glb w1=$B/tex_121801_w1.glb ;;
  esac
done
echo "$(date '+%F %T') DONE $*" >> $O/run.log
