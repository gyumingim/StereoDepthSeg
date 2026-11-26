#!/bin/bash
# gsmap 2026-11-26 W2 final: chosen config AbBb (backed contradicted-front + backed free-space, every guard held in tuning rounds 2-4;
# every D (unobserved-far heuristic) variant broke the P2c guard). Full chain + both rulers + aisle share + picture.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
R=scan3d/gsmap/run_w2.sh; N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w2; E2=$N/ffsonly_ba2/evalrun_newcams2
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
export VOTES=votes1c
bash $R chain AbBb || exit 1
bash $R aisle || exit 1
M="ffsonly2=$N/ffsonly_ba2/b/tex_121801_ffsonly2.glb:ba2 W2=$O/final/tex_121801_w2.glb:ba2 nC=$N/ffsonly_ba/support_test/nC/tex_121801_ffssupport_NC.glb:bap \
np=$N/ffsonly_ba/newpose/tex_121801_ffsonly_np.glb:bap W1=$N/w1/b/tex_121801_w1.glb:ba3"
bash $R judge geo final_rjudge $M || exit 1
bash $R judge photo final_rjudge $M || exit 1
echo "$(date '+%F %T') START pic" >> $O/run.log
GSMAP_SPOTS="316:0:20:aisle kf316 down20;320:0:20:aisle kf320 down20;324:0:15:aisle kf324 down15;328:0:15:aisle kf328 down15;72:0:0:cabinet front (ROI kf774);60:0:15:kf60 cabinets down15;80:0:0:panel (ROI kf813)" \
  $GS scan3d/gsmap/gsm_w2pic.py $E2 $O/w2_121801.jpg $N/ffsonly_ba2/b/carve.ply $O/cfg_AbBb/removed.npz \
  ffsonly2=$N/ffsonly_ba2/b/tex_121801_ffsonly2.glb W2=$O/final/tex_121801_w2_pad.glb > $O/logs/pic.log 2>&1; rc=$?
echo "$(date '+%F %T') END pic rc $rc" >> $O/run.log
[ $rc = 0 ] || { echo "$(date '+%F %T') FAIL w2 pic rc $rc" >> $O/run.log; exit 1; }
echo "$(date '+%F %T') FINAL_OK" >> $O/run.log
