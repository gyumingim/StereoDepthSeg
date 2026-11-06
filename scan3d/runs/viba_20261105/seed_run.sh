#!/bin/bash
# README 2.8: RANSAC seed spread. Per recording x seed: new kf_ba dump (CPU SIFT, --seed s) -> kf_ba BA from dump -> VI-BA (yaml).
# Runs after resume_queue (one heavy job at a time, RAM check like resume_queue). db.db files are left in place (small).
cd "$(dirname "$0")"; until grep -q RESUMEDONE resume_queue.out || [ "$1" == now ]; do sleep 60; done
cd ../../..
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python; PV=$HOME/scanner/viba_work/venv/bin/python; D=scan3d/runs/viba_20261105; BL=$HOME/scanner/build_livepose
memwait() { while [ $(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo) -lt $1 ]; do sleep 30; done; }
for r in 20261024_165506 20261022_134550; do for s in 0 1 2; do
  O=$D/seed/${r}_s$s; [ -f $O/viba/viba.json ] && continue; mkdir -p $O
  if [[ $r == *165506 ]]; then R=scan3d/recordings/hs_${r}_std60; G=imu_gt_spl; X=""; C=$BL/cfg_gen/c4f60; else R=scan3d/recordings/hs_$r; G=imu_gt_spl_cal1022; X="--kf-from-vio --match-ms 1"; C=$BL/cfg_gen_hs222/c4f60; fi
  memwait 14; [ -f $O/viba_input.npz ] || $P scan3d/viba/kf_ba.py $R --vio $D/vio/$r.poses.jsonl.gz --out $O --gt $G $X --cpu-features --dump --no-ba --seed $s > $O/kf_ba.log 2>&1
  memwait 14; [ -f $O/ba.json ] || $P scan3d/viba/kfba_from_dump.py $O > $O/kfd.out 2>&1
  memwait 14; $PV scan3d/viba/viba.py $R --dump $O/viba_input.npz --vio $D/vio/$r.poses.jsonl.gz --cfg $C --out $O/viba --calib yaml > $O/viba.log 2>&1
  echo "$r s$s done $(date +%T)"
done; done; echo SEEDDONE $(date +%T)
