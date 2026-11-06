#!/bin/bash
# viba job queue (README 2.9): runs the jobs listed in JOBFILE one after another ("viba REC VARIANT" / "mesh REC ba|viba").
# Two of these run at once (2 CPU jobs max). Before each job: MemAvailable >= 8 GB + the job's expected peak.
# Finished results are skipped; a job already running elsewhere (same --out) is waited for, not duplicated.
# GPU (mesh): gpu_guard lock, torch frac 0.6, torch.compile disabled.
JOBS=$(readlink -f "$1"); cd "$(dirname "$0")/../../.."
PV=$HOME/scanner/viba_work/venv/bin/python; P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
D=scan3d/runs/viba_20261105; BL=$HOME/scanner/build_livepose; MR=$HOME/scanner/viba_work/memrun.sh
export FFS_ROOT=$HOME/scanner/StereoDepthSeg SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 TORCHINDUCTOR_COMPILE_THREADS=1
memwait() { while [ $(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo) -lt $1 ]; do sleep 30; done; }
big() { case $1 in 20261022_*|20261020_221052|20261024_165506) return 1;; *) return 0;; esac; }
rec() { if [[ $1 == *165506 ]]; then echo scan3d/recordings/hs_${1}_std60; else echo scan3d/recordings/hs_$1; fi; }
cfg() { if [[ $1 == *165506 ]]; then echo $BL/cfg_gen/c4f60; else echo $BL/cfg_gen_hs222/c4f60; fi; }
busy() { pgrep -f "[-]-out $1( |\$)" > /dev/null; }
viba() {
  r=$1; n=$2; o=$D/viba/$r/$n
  while busy $o; do sleep 30; done
  [ -f $o/viba.json ] && return
  case $n in ba_g) args="--vision-only";; viba) args="--calib yaml";; viba_on) args="--calib online";; viba_x103) args="--calib yaml --init-scale 1.03";; esac
  if big $r; then memwait 14; else memwait 10; fi
  busy $o && return; mkdir -p $o
  $MR 11000 $D/viba/$r/$n.log $PV scan3d/viba/viba.py $(rec $r) --dump $D/ba/$r/viba_input.npz --vio $D/vio/$r.poses.jsonl.gz \
      --cfg $(cfg $r) --out $o $args
  echo "$r $n $(tail -1 $D/viba/$r/$n.log.mem) $(date +%T) $(grep -E '^\[lm\]' $D/viba/$r/$n.log)"
}
mesh() {
  r=$1; n=$2; o=$D/mesh/${r}_$n
  while [ ! -f $D/viba/$r/viba/viba.json ] || busy $D/viba/$r/viba; do sleep 60; done
  [ -f $o/summary.json ] && return
  f=$D/ba/$r/poses_ba.jsonl.gz; [[ $n == viba ]] && f=$D/viba/$r/viba/poses_viba.jsonl.gz
  G=imu_gt_spl_cal1022; MX="--compress --kf-step 1"; [[ $r == *165506 ]] && { G=imu_gt_spl; MX=""; }
  memwait 13
  $P scan3d/viba/scan_offline.py $(rec $r) --out $o --gt $G --vio $f $MX > $o.log 2>&1
  echo "$r mesh $n rc $? $(date +%T)"
}
while read -r kind r n <&3; do [ -n "$kind" ] && $kind $r $n < /dev/null; done 3< "$JOBS"
echo "JOBQDONE $(basename $JOBS) $(date +%T)"
