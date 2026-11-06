#!/bin/bash
# two stage instances, disjoint recordings, 1 job each; then pass 2 (x1.03 control, online calib) split the same way
cd "$(dirname "$0")"
A="20261021_214550 20261021_215348 20261020_221925 20261021_213341 20261020_223815 20261024_165506 20261022_134204 20261020_221052"
B="20261020_214138 20261020_222747 20261020_224323 20261022_133820 20261022_134550"
( VIBA_PAR=1 VARIANTS="viba ba_g" ./viba_stage.sh $A; VIBA_PAR=1 VARIANTS="viba_x103 viba_on" ./viba_stage.sh $A ) &
( VIBA_PAR=1 VARIANTS="viba ba_g" ./viba_stage.sh $B; VIBA_PAR=1 VARIANTS="viba_x103 viba_on" ./viba_stage.sh $B ) &
wait; echo STAGES2DONE $(date +%T)
