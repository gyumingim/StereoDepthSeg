set -e
SRC=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/ovdrone_stage
R=/home/system/scanner/StereoDepthSeg-ovdrone/scan3d/runs/ovdrone_20261025
mkdir -p $R/phone/out_L_drone2 $R/phone_rx
cp -r $SRC/pull/out/. $R/phone/out_L_drone2/
for f in $R/phone/out_L_drone2/*/po.jsonl $R/phone/out_L_drone2/*/*_log.csv $R/phone/out_L_drone2/*/ov_timing.txt; do [ -f "$f" ] && gzip -f "$f"; done
cp -r $SRC/rx_viop_phone $SRC/rx_mav_phone $R/phone_rx/
gzip -f $R/phone_rx/*/raw.bin $R/phone_rx/*/*.csv
du -sh $R/phone $R/phone_rx
cp /tmp/ovdrone/ate/score_B0_R294_R294F.json /tmp/ovdrone/ate/seg_B0_R294_R294F_w2.json /tmp/ovdrone/ate/seg_B0_R294_R294F_w5.json $R/ 
cp /tmp/ovdrone/raw80_rs.log $R/
