#!/bin/bash
# gsmap 2026-11-26 W2 tuning round 4: every D variant fails P2c together with AbBb (+1.1..+1.7); try D only on faces visible in many keyframes.
cd "$HOME/scanner/StereoDepthSeg-gsmap"
R=scan3d/gsmap/run_w2.sh; N=$PWD/scan3d/runs/gsmap_20261125_121801_v2; O=$N/w2
export VOTES=votes1c
AB="--a-field front_backed --a-min 2 --a-ratio 2 --b-min 2 --b-ratio 2 --b-field fs12_backed"
bash $R cfg AbBbd30v10 $AB --d-dist 0.30 --d-vis 10 || exit 1
bash $R cfg AbBbd30v30 $AB --d-dist 0.30 --d-vis 30 || exit 1
bash $R judge geo tune4_geo ffsonly2=$N/ffsonly_ba2/b/e3c.ply:ba2 AbBbd30v10=$O/cfg_AbBbd30v10/e3c.ply:ba2 AbBbd30v30=$O/cfg_AbBbd30v30/e3c.ply:ba2 || exit 1
echo "$(date '+%F %T') TUNE4_OK" >> $O/run.log
