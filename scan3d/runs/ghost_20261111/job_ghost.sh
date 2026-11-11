#!/bin/bash
# ghost 11-11 laptop job (cwd = worktree root, run by remote_laptop.sh 'cmd:'): job_ghost.sh VARIANTS [FUSE_TAG]
# 1) depth cache once (ghost_cache.py = scan_offline + npz per keyframe; kept on the laptop in $GC between jobs)  2) ghost_fuse.py variants
set -e
VARS=${1:-base,twide,cons,consx,icp,icp_cons}; TAGF=${2:-fuse}
D=scan3d/runs/ghost_20261111
GC=/home/system/scanner/ghost_cache/134550
df -h /home | tail -1
if [ ! -f $GC/done ]; then
  rm -rf $GC
  GHOST_CACHE=$GC /usr/bin/time -f "TIME cache %e s maxrss %M kB" $P $D/ghost_cache.py $R --out $O/cache_run --gt $G \
     --vio $WT/$D/A_poses_ba_134550.jsonl.gz --compress --kf-step 1 > $O/cache_run.log 2>&1
  touch $GC/done
fi
du -sh $GC; ls $GC | wc -l
/usr/bin/time -f "TIME fuse %e s maxrss %M kB" $P $D/ghost_fuse.py $GC $O/$TAGF --variants $VARS > $O/$TAGF.log 2>&1
echo done
