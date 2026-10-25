D=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/esp32fix
R=~/scanner/StereoDepthSeg-esp32fix/scan3d/runs/esp32fix_20261026
bash $D/hb_rate_pc.sh > $R/hb_rate_pc.log 2>&1
MODE=1 bash $D/hb_pause_pc.sh > $R/hb_pause_pc_mode1.log 2>&1
MODE=0 bash $D/hb_pause_pc.sh > $R/hb_pause_pc_mode0.log 2>&1
echo done
