#!/bin/bash
# chain3: close-up picture (photo | nC | SM2 | o1) + final recipe (SM2 + --seam-occl 1) on ffsonly2 + its judge/detail
cd $HOME/scanner/StereoDepthSeg-gsmap
R=scan3d/gsmap/run_w9.sh; N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w9; L=$O/run.log
GS="taskset -c 0-7 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
echo "$(date '+%F %T') START w9 chain3 (pic + ffsonly2 SM2+occl)" >> $L
echo "$(date '+%F %T') START pic" >> $L
$GS scan3d/gsmap/gsm_w8pic.py $O/pic.json $O/w9_tex_121801.jpg nC=$N/ffsonly_ba/support_test/nC/tex_121801_ffssupport_NC.glb \
  SM2=$N/w8/sm2/tex_121801_nC_sm2.glb "SM2+occl(o1)"=$O/o1/tex_121801_nC_o1.glb > $O/logs/pic.log 2>&1
echo "$(date '+%F %T') END pic rc $?" >> $L
bash $R f2 --seam-occl 1 && bash $R f2judge
echo "$(date '+%F %T') END w9 chain3" >> $L
