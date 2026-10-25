# pull.sh TAG... : pull phone run outputs into results/TAG and analyze
W='C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\replay'
D=/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/replay
S=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/replay
ADB="/c/Users/a3162/platform-tools/adb.exe -s 100.79.156.20:5555"
L=""
for t in "$@"; do
  mkdir -p $D/results/$t
  for f in tl.csv po.jsonl pe.txt ov_timing.txt run.txt feeder.txt samp.txt; do MSYS_NO_PATHCONV=1 $ADB pull /data/local/tmp/replay/out/$t/$f "$W/results/$t/$f" > /dev/null; done
  L="$L $S/results/$t"
done
cd /c/; MSYS_NO_PATHCONV=1 wsl /home/system/scanner/StereoDepthSeg/venv_ffs/bin/python $S/analyze.py $L
