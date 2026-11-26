#!/bin/bash
# gsmap 2026-11-26 W2 tuning round 3: AbBb (backed A + B) passes every guard; AbBb + D30 fails P2c (+1.41). Try milder D / no fragment step.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
R=scan3d/gsmap/run_w2.sh; N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w2
export VOTES=votes1c
AB="--a-field front_backed --a-min 2 --a-ratio 2 --b-min 2 --b-ratio 2 --b-field fs12_backed"
bash $R cfg AbBbd30nf $AB --d-dist 0.30 || exit 1
bash $R cfg AbBbd40   $AB --frag-area 0.05 --d-dist 0.40 || exit 1
bash $R cfg AbBbd50nf $AB --d-dist 0.50 || exit 1
bash $R cfg AbBbc     $AB --frag-area 0.05 || exit 1
bash $R judge geo tune3_geo ffsonly2=$N/ffsonly_ba2/b/e3c.ply:ba2 AbBbd30nf=$O/cfg_AbBbd30nf/e3c.ply:ba2 AbBbd40=$O/cfg_AbBbd40/e3c.ply:ba2 \
    AbBbd50nf=$O/cfg_AbBbd50nf/e3c.ply:ba2 AbBbc=$O/cfg_AbBbc/e3c.ply:ba2 || exit 1
echo "$(date '+%F %T') TUNE3_OK" >> $O/run.log
