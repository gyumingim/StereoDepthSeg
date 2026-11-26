#!/bin/bash
# gsmap 2026-11-26: FFS-only 121801 map on the new BA poses (gsm_ffsonly_ba.py). usage: run_ffsonly_ba.sh step [step ...]
# Power safety: one heavy job at a time (wait until no GPU compute process and no other heavy gsmap python); CPU under nice 19 on 8 cores.
# A step that fails twice stops the chain. Output only in runs/gsmap_20261125_121801_v2/ffsonly_ba.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/ffsonly_ba; mkdir -p $O/logs
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 PYTHONUNBUFFERED=1
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
FF="taskset -c 0-7 nice -n 19 env FFS_ROOT=$HOME/scanner/StereoDepthSeg $HOME/scanner/StereoDepthSeg/venv_ffs/bin/python -W ignore"
S=scan3d/gsmap/gsm_ffsonly_ba.py
NEWDUMP=$N/ba_partners/run/dump_ba
D=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo

wait_free() {
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|ffs[a-z0-9_]*\.py|kf_ba' | grep -v "$$" | grep -c .)
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
B="$O/b"; export GSMAP_RUN=$N
for st in "$@"; do
  case $st in
    prep) run_step prep $GS $S prep ;;
    ffs) run_step ffs $FF $S ffs ;;
    fuse) run_step fuse_s4m $GS $S fuse --name s4m ;;
    kfit) run_step kfit_odd $GS $S kfit --name s4m --parity odd --kout $O/kscale/k_odd.npz
          run_step kfit_all $GS $S kfit --name s4m --parity all --kout $O/kscale/k_all.npz ;;
    kfuse) run_step fuse_kodd $GS $S fuse --name s4m_kodd --kscale $O/kscale/k_odd.npz
           run_step fuse_kall $GS $S fuse --name s4m_kall --kscale $O/kscale/k_all.npz ;;
    judge) run_step judge $GS $S judge --name L,s4m,s4m_kodd,s4m_kall ;;
    measure) run_step measure $FF $S measure --name s4m,s4m_kodd,s4m_kall ;;
    # ---------------- stage B (SET = chosen stage-A set)
    fused) run_step fused $GS $S fused --name ${SET:?} ;;
    train) source $HOME/scanner/gsplat_env/env.sh
           run_step train $GS scan3d/gsmap/gsm_train.py $NEWDUMP --fused $O/fused --out $B/gs --surfel 1 --mask-border 1 --fill-w 0.1 --aniso-max 5 --aniso-w 1.0 \
             --max-scale 0.05 --voxel 0.025 --max-gs 2500000 --steps 30000 --exposure 1 --depth-sigma fused --hold 0 ;;
    mesh) source $HOME/scanner/gsplat_env/env.sh
          run_step mesh env GSMAP_WELD=1 GSMAP_TRUNC=0.04 GSMAP_ZMAX=5.0 GSMAP_ZSIG=1 $GS scan3d/gsmap/gsm_mesh2d.py $B/gs $B/gs/mesh.ply ;;
    v3fix) run_step v3fix $GS scan3d/gsmap/gsm_v3fix.py $B/gs/mesh.ply $B/v3fix.ply ;;
    carve) run_step carve $GS scan3d/gsmap/gsm_carve.py $B/v3fix.ply $B/carve.ply --min-viol 5 --island-area 0 --min-tri 0 ;;
    e3c) run_step e3c $GS scan3d/gsmap/gsm_e3c.py $B/carve.ply $B/e3c.ply --ceil-y -1.855 ;;
    tex) run_step tex $GS scan3d/gsmap/gsm_texture2.py --dump $NEWDUMP --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 \
           $B/e3c.ply $B/tex_121801_ffsonly.glb
         run_step atlas $GS scan3d/gsmap/gsm_atlas8k.py $B/tex_121801_ffsonly.glb ;;
    eval) run_step eval $GS scan3d/gsmap/gsm_eval2.py $B/tex_121801_ffsonly.glb $B/eval.json --tex-json $B/tex_121801_ffsonly.json ;;
    sheet) run_step sheet env GSMAP_SPOTS="${SPOTS:?}" $GS scan3d/gsmap/gsm_crops.py ${N%_v2} $B/ffsonly_121801.jpg \
             v4=$N/e1/tex_121801_v4.glb final=$N/e8/c1855/tex_121801_final.glb ffsonly=$B/tex_121801_ffsonly.glb
           cp $B/tex_121801_ffsonly.glb $B/ffsonly_121801.jpg $D/ ;;
  esac
done
echo "$(date '+%F %T') DONE $*" >> $O/run.log
