#!/bin/bash
# gpuba 11-09: one full kf_ba S run (SIFT + matching + verification seed 0 + tracks + BA) for timing.
# usage: time_rec.sh ID DEV TAG [extra kf_ba args]   DEV = cpu|cuda ; out rec/ID/TAG ; CPU part under flock heavy.lock
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
D=scan3d/runs/gpuba_20261109; FL=scan3d/runs/featloop_20261105; CACHE=$HOME/scanner/featloop_ext/cache; HL=$HOME/scanner/heavy.lock
export SCAN3D_GPU_POLL_S=3 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
id=$1; dev=$2; tag=$3; shift 3
if [ "$id" = 165506 ]; then
  R=$W/scan3d/recordings/hs_20261024_165506_std60; G=imu_gt_spl; V=$W/../StereoDepthSeg-featloop/scan3d/runs/scanA_20261105/step1/B_o0.poses.jsonl.gz; KFO=""
else
  R=$W/scan3d/recordings/hs_$id; G=imu_gt_spl_cal1022; V=$W/../StereoDepthSeg-featloop/$FL/vio/$id.poses.jsonl.gz; KFO="--kf-from-vio --match-ms 1"
fi
O=$D/rec/$id; mkdir -p $O
while [ $(awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo) -lt 8 ]; do sleep 30; done
echo "== $id $tag ($dev) start $(date +%T)"
flock $HL $P scan3d/scanA/kf_ba.py $R --vio $V --out $O/$tag --gt $G $KFO --feat sift --loop none --cache-root $CACHE --seed 0 \
  --ba-device $dev "$@" > $O/$tag.log 2>&1 || echo "   failed rc $?"
grep -E "^\[(sim3|tri)\]" $O/$tag.log
echo "== $id $tag done $(date +%T)"
