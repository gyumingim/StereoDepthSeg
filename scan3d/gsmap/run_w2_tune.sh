#!/bin/bash
# gsmap 2026-11-26 W2 tuning round 1 (aisle ROI + guards): configs of gsm_w2clean apply -> NC prune -> E3c -> eval2 (ply, ba2 ruler),
# then the R ruler geo (own poses ba2) on ffsonly2 e3c.ply (baseline) + every config. Sequential, one job at a time.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
R=scan3d/gsmap/run_w2.sh; N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w2
bash $R baseeval || exit 1
bash $R cfg a     --a-min 2 --a-ratio 2 || exit 1
bash $R cfg ab    --a-min 2 --a-ratio 2 --b-min 2 --b-ratio 2 --b-field fs12 || exit 1
bash $R cfg abc   --a-min 2 --a-ratio 2 --b-min 2 --b-ratio 2 --b-field fs12 --frag-area 0.05 || exit 1
bash $R cfg abd30 --a-min 2 --a-ratio 2 --b-min 2 --b-ratio 2 --b-field fs12 --frag-area 0.05 --d-dist 0.30 || exit 1
bash $R cfg abd15 --a-min 2 --a-ratio 2 --b-min 2 --b-ratio 2 --b-field fs12 --frag-area 0.05 --d-dist 0.15 || exit 1
bash $R judge geo tune1_geo ffsonly2=$N/ffsonly_ba2/b/e3c.ply:ba2 a=$O/cfg_a/e3c.ply:ba2 ab=$O/cfg_ab/e3c.ply:ba2 abc=$O/cfg_abc/e3c.ply:ba2 \
    abd30=$O/cfg_abd30/e3c.ply:ba2 abd15=$O/cfg_abd15/e3c.ply:ba2 || exit 1
echo "$(date '+%F %T') TUNE1_OK" >> $O/run.log
