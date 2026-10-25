D=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/esp32fix
R=~/scanner/StereoDepthSeg-esp32fix/scan3d/runs/esp32fix_20261026
bash $D/ident_pc.sh > $R/ident_pc_fix3.log 2>&1
MODE=1 bash $D/hb_pause_pc.sh > $R/hb_pause_pc_mode1_fix3.log 2>&1
grep -E "IDENTICAL|DIFFER|identical" $R/ident_pc_fix3.log | grep -v "^ref"
grep -E "pause_mode|heartbeat thread" $R/hb_pause_pc_mode1_fix3.log
