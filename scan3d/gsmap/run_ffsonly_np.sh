#!/bin/bash
# gsmap 2026-11-26: redo the post-training stages of ffsonly_ba on the NEW BA poses (no retrain). usage: run_ffsonly_np.sh step [step ...]
# Same commands/params as run_ffsonly_ba.sh (v3fix, carve mv5, e3c --ceil-y -1.855, texture2 chunk 0.75 no-gain drop 0, eval2),
# but GSMAP_RUN = ffsonly_ba/evalrun_newcams (dump_ba/cams.npz -> ba_partners/run/dump_ba = new BA keyframe poses; eval2/ref = v2 refs).
# Mesh: b/gs/mesh.ply reused (gsm_mesh2d renders with gaussians.pt T = training poses = new dump; checked in step 'chk').
cd "$HOME/scanner/StereoDepthSeg-gsmap"
N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/ffsonly_ba; B=$O/b; E=$O/evalrun_newcams; P=$O/newpose; mkdir -p $P/logs
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 NUMEXPR_NUM_THREADS=8 PYTHONUNBUFFERED=1 GSMAP_RUN=$E
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
NEWDUMP=$N/ba_partners/run/dump_ba
D=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/splat_demo
wait_free() {
  while true; do
    g=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
    p=$(pgrep -af python | grep -E 'gsm_[a-z0-9_]+\.py|train|ffs[a-z0-9_]*\.py|kf_ba' | grep -c .)
    [ "$g" = 0 ] && [ "$p" = 0 ] && return
    echo "$(date '+%F %T') wait: gpu procs $g, heavy python $p" >> $P/logs/wait.log; sleep 30
  done
}
run_step() {
  local st=$1; shift
  for try in 1 2; do
    wait_free
    echo "$(date '+%F %T') START $st try $try" >> $P/run.log
    "$@" >> $P/logs/$st.log 2>&1; rc=$?
    echo "$(date '+%F %T') END $st rc $rc" >> $P/run.log
    [ $rc = 0 ] && return 0
  done
  echo "$(date '+%F %T') STOP: $st failed twice" >> $P/run.log; exit 1
}
for st in "$@"; do
  case $st in
    chk) run_step chk $GS - <<PY
import json, time, numpy as np, torch
ck = torch.load("$B/gs/gaussians.pt", weights_only=False, map_location="cpu"); T = np.asarray(ck["T"])
n = np.load("$NEWDUMP/cams.npz"); o = np.load("$N/dump_ba/cams.npz"); e = np.load("$E/dump_ba/cams.npz")
r = dict(made=time.strftime("%F %T"), ck_vs_new_max_m=float(np.abs(T - n["T"]).max()), ck_vs_old_max_m=float(np.abs(T - o["T"]).max()),
         evalcams_vs_new_max_m=float(np.abs(e["T"] - n["T"]).max()), n=int(len(T)))
open("$P/chk_poses.json", "w").write(json.dumps(r, indent=1)); print(r)
assert r["ck_vs_new_max_m"] < 1e-5 and r["evalcams_vs_new_max_m"] < 1e-9
PY
    ;;
    v3fix) run_step v3fix $GS scan3d/gsmap/gsm_v3fix.py $B/gs/mesh.ply $P/v3fix.ply ;;
    carve) run_step carve $GS scan3d/gsmap/gsm_carve.py $P/v3fix.ply $P/carve.ply --min-viol 5 --island-area 0 --min-tri 0 ;;
    e3c) run_step e3c $GS scan3d/gsmap/gsm_e3c.py $P/carve.ply $P/e3c.ply --ceil-y -1.855 ;;
    tex) run_step tex $GS scan3d/gsmap/gsm_texture2.py --dump $NEWDUMP --rec $N/rec --gt lensonly --chunk 0.75 --fill-all 1 --drop-views 0 --no-gain 1 \
           $P/e3c.ply $P/tex_121801_ffsonly_np.glb
         run_step atlas $GS scan3d/gsmap/gsm_atlas8k.py $P/tex_121801_ffsonly_np.glb ;;
    eval) run_step eval_np $GS scan3d/gsmap/gsm_eval2.py $P/tex_121801_ffsonly_np.glb $P/eval_newcams_ffsonly_np.json --tex-json $P/tex_121801_ffsonly_np.json
          run_step eval_e8 $GS scan3d/gsmap/gsm_eval2.py $N/e8/c1855/tex_121801_final.glb $P/eval_newcams_e8final_rerun.json --tex-json $N/e8/c1855/tex_121801_final.json ;;
    sheet) run_step sheet env GSMAP_SPOTS="${SPOTS:?}" $GS scan3d/gsmap/gsm_crops.py ${N%_v2} $P/ffsonly_np_121801.jpg \
             ffsonly=$B/tex_121801_ffsonly.glb ffsonly_np=$P/tex_121801_ffsonly_np.glb
           cp $P/tex_121801_ffsonly_np.glb $P/ffsonly_np_121801.jpg $D/ ;;
  esac
done
echo "$(date '+%F %T') DONE $*" >> $P/run.log
