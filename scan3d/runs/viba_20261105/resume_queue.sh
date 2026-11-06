#!/bin/bash
# viba resume after the 11-06 10:50 PC hang (README 2.7). ONE heavy job at a time, strictly serial.
# Before each job: wait until MemAvailable >= 12 GB + the job's expected peak (so >= 12 GB stays free).
# A job whose result file exists is skipped (finished runs are never redone). GPU jobs: gpu_guard lock, torch frac 0.6,
# torch.compile disabled (no inductor compile workers).
cd "$(dirname "$0")/../../.."
PV=$HOME/scanner/viba_work/venv/bin/python; P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
D=scan3d/runs/viba_20261105; BL=$HOME/scanner/build_livepose; MR=$HOME/scanner/viba_work/memrun.sh
export FFS_ROOT=$HOME/scanner/StereoDepthSeg SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 TORCHINDUCTOR_COMPILE_THREADS=1
memwait() { while [ $(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo) -lt $1 ]; do sleep 30; done; }
big() { case $1 in 20261022_*|20261020_221052|20261024_165506) return 1;; *) return 0;; esac; }
rec() { if [[ $1 == *165506 ]]; then echo scan3d/recordings/hs_${1}_std60; else echo scan3d/recordings/hs_$1; fi; }
cfg() { if [[ $1 == *165506 ]]; then echo $BL/cfg_gen/c4f60; else echo $BL/cfg_gen_hs222/c4f60; fi; }
viba() {  # rec variant
  r=$1; n=$2; [ -f $D/viba/$r/$n/viba.json ] && return
  case $n in ba_g) args="--vision-only";; viba) args="--calib yaml";; viba_on) args="--calib online";; viba_x103) args="--calib yaml --init-scale 1.03";; esac
  big $r && memwait 18 || memwait 14
  mkdir -p $D/viba/$r/$n; rm -f $D/viba/$r/$n.log.mem
  $MR 11000 $D/viba/$r/$n.log $PV scan3d/viba/viba.py $(rec $r) --dump $D/ba/$r/viba_input.npz --vio $D/vio/$r.poses.jsonl.gz \
      --cfg $(cfg $r) --out $D/viba/$r/$n $args
  echo "$r $n $(tail -1 $D/viba/$r/$n.log.mem) $(date +%T) $(grep -E '^\[lm\]' $D/viba/$r/$n.log)"
}
mesh() {  # rec ba|viba
  r=$1; n=$2; [ -f $D/mesh/${r}_$n/summary.json ] && return
  [ -f $D/viba/$r/viba/viba.json ] || return
  f=$D/ba/$r/poses_ba.jsonl.gz; [[ $n == viba ]] && f=$D/viba/$r/viba/poses_viba.jsonl.gz
  G=imu_gt_spl_cal1022; MX="--compress --kf-step 1"; [[ $r == *165506 ]] && { G=imu_gt_spl; MX=""; }
  memwait 16
  rm -rf $D/mesh/${r}_$n
  $P scan3d/viba/scan_offline.py $(rec $r) --out $D/mesh/${r}_$n --gt $G --vio $f $MX > $D/mesh/${r}_$n.log 2>&1
  echo "$r mesh $n rc $? $(date +%T)"
}
ALL="20261024_165506 20261022_134204 20261020_221052 20261022_133820 20261022_134550 20261021_214550 20261021_215348 20261020_221925 20261021_213341 20261020_223815 20261020_224323 20261020_222747 20261020_214138"
for r in 20261020_221925 20261021_213341 20261020_223815; do viba $r viba; done                      # 1. main VI-BA, missing 3
for r in $ALL; do big $r || viba $r viba_x103; done                                                 # 2. control, small ones
for r in $ALL; do mesh $r ba; mesh $r viba; done                                                     # 3. depth + TSDF (GPU)
for r in $ALL; do big $r && viba $r viba_x103; done                                                 # 4. control, big ones
for r in $ALL; do viba $r ba_g; done                                                                 # 5. same-solver reference
for r in $ALL; do viba $r viba_on; done                                                              # 6. online-calib sensitivity
echo RESUMEDONE $(date +%T)
