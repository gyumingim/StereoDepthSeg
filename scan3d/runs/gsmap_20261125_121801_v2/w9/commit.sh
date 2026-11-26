#!/bin/bash
cd $HOME/scanner/StereoDepthSeg-gsmap
W=scan3d/runs/gsmap_20261125_121801_v2/w9
git add -f scan3d/gsmap/gsm_texture3.py scan3d/gsmap/gsm_w9detail.py scan3d/gsmap/gsm_w9table.py scan3d/gsmap/run_w9.sh \
  $W/chain1.sh $W/chain2.sh $W/chain3.sh $W/commit.sh $W/summary.json $W/table1.json $W/table2.json $W/table_f2.json \
  $W/detail1.json $W/detail2.json $W/detail_f2.json $W/rjudge1/photo.json $W/rjudge2/photo.json $W/rjudge_f2/photo.json $W/pic.json $W/run.log
git commit -q -m "gsmap W9: seam levelling fixes on SM2 (gsm_texture3 --seam-occl/--seam-mode/--seam-target/--seam-fill), full-res detail ruler gsm_w9detail; occlusion check o1 accepted (+0.21 dB vs SM2, detail +0.5 %); same recipe on ffsonly2 +0.60 dB, fake flat gradient -12.7 %

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
git log --oneline -1; git status --short | head
