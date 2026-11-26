#!/bin/bash
# gsmap 2026-11-26 W2 tuning round 2: round 1 showed that removing contradicted / free-space faces without a surface behind them only turns
# 'front' into 'hole' (cabinet ROI hit -2..-4.5 pt, P2c +1.6..+2.2). Round 2 uses the 'backed' votes (votes1c) and tests D alone.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
R=scan3d/gsmap/run_w2.sh; N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w2
export VOTES=votes1c
bash $R votes1 || exit 1
bash $R cfg d30      --a-min 0 --frag-area 0.05 --d-dist 0.30 || exit 1
bash $R cfg Ab       --a-field front_backed --a-min 2 --a-ratio 2 || exit 1
bash $R cfg AbBb     --a-field front_backed --a-min 2 --a-ratio 2 --b-min 2 --b-ratio 2 --b-field fs12_backed || exit 1
bash $R cfg AbBbd30  --a-field front_backed --a-min 2 --a-ratio 2 --b-min 2 --b-ratio 2 --b-field fs12_backed --frag-area 0.05 --d-dist 0.30 || exit 1
bash $R cfg AbBbd20  --a-field front_backed --a-min 2 --a-ratio 2 --b-min 2 --b-ratio 2 --b-field fs12_backed --frag-area 0.05 --d-dist 0.20 || exit 1
bash $R judge geo tune2_geo ffsonly2=$N/ffsonly_ba2/b/e3c.ply:ba2 d30=$O/cfg_d30/e3c.ply:ba2 Ab=$O/cfg_Ab/e3c.ply:ba2 AbBb=$O/cfg_AbBb/e3c.ply:ba2 \
    AbBbd30=$O/cfg_AbBbd30/e3c.ply:ba2 AbBbd20=$O/cfg_AbBbd20/e3c.ply:ba2 || exit 1
echo "$(date '+%F %T') TUNE2_OK" >> $O/run.log
