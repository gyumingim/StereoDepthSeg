#!/bin/bash
# noise_floor_20261128: ffsonly2 2DGS retrained with ONLY --seed changed (0 -> 1), same inputs/args as
# scan3d/gsmap/run_ffs_relax.sh "train" (ffsonly_ba2/b/gs, summary.json args), then the same mesh chain
# (run_ffs_relax.sh mesh v3fix carve prune e3c), the final texture recipe (run_w9.sh f2: texture3 SM2 + --seam-occl 1, atlas8k, seampad),
# eval2, the W9/audit10 photo judge (gsm_w2judge photo) and the plane_diag ruler (copy with GS/MESH/OUT overridable).
# Step-level checkpoint: a step whose marker logs/<step>.done exists is skipped. One step at a time, GPU alone, nice 10, cores 0-7.
# usage: run_nf.sh [step ...]   (default: all)
cd "$HOME/scanner/StereoDepthSeg-gsmap"
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O0=$N/ffsonly_ba2; B0=$O0/b; E2=$O0/evalrun_newcams2; E=$N/ffsonly_ba/evalrun_newcams
NEWDUMP=$N/ba_partners2/run/dump_ba; DUMP2=$NEWDUMP
R=$PWD/scan3d/runs/noise_floor_20261128; S=$R/seed1; mkdir -p $S $R/logs
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 NUMEXPR_NUM_THREADS=8 PYTHONUNBUFFERED=1
GS="taskset -c 0-7 nice -n 10 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
SEED=1
SM2="--seam-level 1 --seam-soft 0.5"
log() { echo "$(date '+%F %T') $*" >> $R/run.log; }
wait_free() {
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|ffs[a-z0-9_]*\.py|kf_ba|plane_diag' | grep -c .)
    [ "$g" = 0 ] && [ "$p" = 0 ] && return
    echo "$(date '+%F %T') wait: gpu procs $g, heavy python $p" >> $R/logs/wait.log; sleep 30
  done
}
run_step() {
  local st=$1; shift
  [ -e $R/logs/$st.done ] && { log "SKIP $st (done)"; return 0; }
  for try in 1 2; do
    wait_free; log "START $st try $try"
    "$@" >> $R/logs/$st.log 2>&1; rc=$?
    log "END $st rc $rc"
    [ $rc = 0 ] && { date '+%F %T' > $R/logs/$st.done; return 0; }
  done
  log "STOP: $st failed twice"; exit 1
}
steps="$*"; [ -z "$steps" ] && steps="train mesh v3fix carve prune e3c tex eval photo plane"
source $HOME/scanner/gsplat_env/env.sh
for st in $steps; do
  case $st in
    train) rm -rf $S/gs.partial; [ -e $R/logs/train.done ] || rm -rf $S/gs
           run_step train $GS scan3d/gsmap/gsm_train.py $NEWDUMP --fused $O0/fused --out $S/gs --surfel 1 --mask-border 1 --fill-w 0.1 --aniso-max 5 --aniso-w 1.0 \
             --max-scale 0.05 --voxel 0.025 --max-gs 2500000 --steps 30000 --exposure 1 --depth-sigma fused --hold 0 --seed $SEED ;;
    mesh) run_step mesh env GSMAP_RUN=$N GSMAP_WELD=1 GSMAP_TRUNC=0.04 GSMAP_ZMAX=5.0 GSMAP_ZSIG=1 $GS scan3d/gsmap/gsm_mesh2d.py $S/gs $S/gs/mesh.ply ;;
    v3fix) run_step v3fix env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_v3fix.py $S/gs/mesh.ply $S/v3fix.ply ;;
    carve) run_step carve env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_carve.py $S/v3fix.ply $S/carve.ply --min-viol 5 --island-area 0 --min-tri 0 ;;
    prune) run_step prune env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_ffs_support.py $S/carve.ply $S/support --fused $O0/fused --cams $NEWDUMP/cams.npz --n 1,2 --island 0.01
           run_step prunec env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_ffs_support.py $S/carve.ply $S/support --fused $O0/fused --cams $NEWDUMP/cams.npz \
             --votes $S/support/support_votes.npz --comp-area 0.25 ;;
    e3c) run_step e3c env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_e3c.py $S/support/support_NC.ply $S/e3c.ply --ceil-y -1.855 ;;
    tex) G=$S/tex_121801_ffsonly2_s1_w9.glb
         run_step tex env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_texture3.py --dump $DUMP2 --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 $SM2 --seam-occl 1 $S/e3c.ply $G
         run_step atlas env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_atlas8k.py $G
         run_step pad $GS scan3d/gsmap/gsm_seampad.py $G $S/tex_121801_ffsonly2_s1_w9_pad.glb ;;
    eval) for sd in 0 1; do
            if [ $sd = 0 ]; then G=$N/w9/f2/tex_121801_ffsonly2_w9.glb; J=$N/w9/f2/tex_121801_ffsonly2_w9.json; else G=$S/tex_121801_ffsonly2_s1_w9.glb; J=$S/tex_121801_ffsonly2_s1_w9.json; fi
            run_step eval_e2_s$sd env GSMAP_RUN=$E2 $GS scan3d/gsmap/gsm_eval2.py $G $R/eval_newcams2_s$sd.json --tex-json $J
            run_step eval_e_s$sd env GSMAP_RUN=$E $GS scan3d/gsmap/gsm_eval2.py $G $R/eval_newcams_s$sd.json --tex-json $J
          done ;;
    photo) [ -e $R/logs/photo.done ] || rm -rf $R/photo
           run_step photo $GS scan3d/gsmap/gsm_w2judge.py photo $R/photo nC_tex3=$N/w9/o1/tex_121801_nC_o1.glb:bap \
             ffsonly2_tex=$N/w9/f2/tex_121801_ffsonly2_w9.glb:ba2 ffsonly2_pad=$N/w9/f2/tex_121801_ffsonly2_w9_pad.glb:ba2 \
             s1_tex=$S/tex_121801_ffsonly2_s1_w9.glb:ba2 s1_pad=$S/tex_121801_ffsonly2_s1_w9_pad.glb:ba2 ;;
    plane) for sd in 0 1; do
             if [ $sd = 0 ]; then GG=$B0/gs/gaussians.pt; MM=$B0/e3c.ply; EE=$B0/e3c.json; else GG=$S/gs/gaussians.pt; MM=$S/e3c.ply; EE=$S/e3c.json; fi
             run_step plane_render_s$sd env NF_GS=$GG NF_MESH=$MM NF_E3C=$EE NF_OUT=$R/plane_s$sd $GS $R/scripts/plane_diag_nf.py render
             run_step plane_measure_s$sd env NF_GS=$GG NF_MESH=$MM NF_E3C=$EE NF_OUT=$R/plane_s$sd $GS $R/scripts/plane_diag_nf.py measure
           done ;;
    *) log "unknown step $st"; exit 2 ;;
  esac
done
log "DONE $steps"
