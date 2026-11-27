#!/bin/bash
# full newrec FULL smoke (to texture, 3000 steps, 20 kf) (30 fps 121801, --ffs-kf 20)
D=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/night_20261127/rehearsal30
REC=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/dryrun30_121801_20261127/rec_hs_20261112_121801_std30
rm -rf $D/c4_full
bash $HOME/scanner/StereoDepthSeg-gsmap/scan3d/gsmap/newrec/newrec.sh $REC $D/c4_full --ffs-kf 20 --stop-after tex --steps 3000 > $D/newrec_c4_full.out 2>&1
$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/night_20261127/log.sh "newrec FULL smoke (to texture, 3000 steps, 20 kf) (rehearsal30/c4_full) rc $?: $(grep -E 'CHECK|DONE|FAIL|partner BA' $D/c4_full/newrec.log | cut -c21-260 | tr '\n' ' ')"
