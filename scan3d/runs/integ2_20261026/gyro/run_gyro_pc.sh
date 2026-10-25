# integ2 gyro-on-profile PC checks (2026-10-26 morning): identity, then 42-cell GT ATE (G30 / GL30) + std60 rec60 (NL60 / GL60).
set -u
H=~/scanner/StereoDepthSeg-integ2/scan3d/runs/integ2_20261026
G=$H/gyro
bash $G/ident_gyro.sh > $G/ident_gyro.log 2>&1
echo "ident done $(date)"
export OV60_OUT=/tmp/integ2/ate_g
mkdir -p $OV60_OUT/N30 $OV60_OUT/NL30 $OV60_OUT/L60
cp -n /tmp/integ2/ate/N30/* $OV60_OUT/N30/
cp -n /tmp/integ2/ate/NL30/* $OV60_OUT/NL30/
cp -n /tmp/ov60eval/ate/L60/s* $OV60_OUT/L60/
cd $H
python3 integ2_ate.py freeze
python3 integ2_ate.py run "hs:G30,GL30;std:NL60,GL60" --par 6 > $OV60_OUT/sched.log 2>&1
tail -2 $OV60_OUT/sched.log
grep -c FAILED $OV60_OUT/sched.log
python3 integ2_ate.py score > $G/ate_score_gyro.txt 2>&1
cp $OV60_OUT/score.json $G/score_gyro.json
cp $OV60_OUT/bin/bridge.json $G/bridge_gyro.json
cp $OV60_OUT/sched.log $G/sched_gyro.log
echo "all done $(date)"
