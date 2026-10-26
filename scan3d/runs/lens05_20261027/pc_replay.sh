# lens05 PC replay: phone c4f60 / c4 folders (old f2.22 vs new calcap 0.5 dpt) through the frozen integ2 PC bridge.
# "Does not break" check; ATE = reference only (never used to choose the lens).
set -u
S=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/lens05
OUT=~/scanner/tmp_lens05/replay; mkdir -p $OUT
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python
R=~/scanner/StereoDepthSeg/scan3d/recordings
date
sha256sum ~/scanner/tmp_integ2l/ate/bin/ov_bridge_81e547e9dafa ~/scanner/tmp_lens05/{old_regen,new_gen}/{c4,c4f60}/remap_1920x1080.bin | cut -c1-16
job() { local rec=$1 lens=$2 v=$3 fps=$4
  nice -n 5 $PY $S/phone_cfg_replay.py $R/$rec ~/scanner/tmp_lens05/$lens/$v $OUT/${rec}_${v}_${lens}.json --fps $fps > $OUT/${rec}_${v}_${lens}.log 2>&1
  echo "$rec $v $lens rc $? $(tail -1 $OUT/${rec}_${v}_${lens}.log)"; }
for rec in hs_20261024_165506_std60 hs_20261024_173432_std60; do
  job $rec old_regen c4f60 60 &
  job $rec new_gen c4f60 60 &
  wait
done
job hs_20261024_165506_std60 old_regen c4 30 &
job hs_20261024_165506_std60 new_gen c4 30 &
wait
grep -h "remap [0-9]" $OUT/*.bridge.err | sort | uniq -c
date
