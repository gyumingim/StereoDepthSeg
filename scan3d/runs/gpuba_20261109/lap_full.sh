#!/bin/bash
# gpuba 11-11, runs ON THE LAPTOP: kf_ba from scratch (SIFT extraction + matching + verify seed 0 + tracks + BA) several times.
# usage: lap_full.sh R O G V "TAG:SIFTDEV:BADEV ..." [KFO...]   e.g. "old1:auto:cpu new1:auto:cuda fix1:cpu:cuda"
# BADEV cuda also gets --fast-tracks (= new version). DBs kept, reduced to keypoints/descriptors npz, then removed.
R=$1; O=$2; G=$3; V=$4; SPECS=$5; shift 5; KFO="$@"
P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python; CACHE=/home/system/scanner/laptop_cache
export SCAN3D_GPU_POLL_S=3 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
mkdir -p "${O:?}"
# image cache first (not part of any timing): a 16-feature run that stops after triangulation
$P scan3d/scanA/kf_ba.py $R --vio $V --out "${O:?}/warm" --gt $G $KFO --feat sift --loop none --cache-root $CACHE --seed 0 --dump-tri --max-features 16 > "${O:?}/warm.log" 2>&1
rm -rf "${O:?}/warm"
for sp in $SPECS; do
  IFS=: read tag sd bd ex <<< "$sp"
  F=""; [ "$bd" = cuda ] && F="--fast-tracks"
  case "$ex" in *s*) F="$F --sort-matches";; esac                 # 4th field: s = --sort-matches, m = --match-device cpu
  case "$ex" in *m*) F="$F --match-device cpu";; esac
  echo "$(date +%T) $tag sift $sd ba $bd"
  /usr/bin/time -f "TIME $tag %e s maxrss %M kB" $P scan3d/scanA/kf_ba.py $R --vio $V --out "${O:?}/${tag:?}" --gt $G $KFO --feat sift --loop none \
    --cache-root $CACHE --seed 0 --keep-db --sift-device $sd --ba-device $bd $F > "$O/$tag.log" 2>&1 || echo "$tag failed rc $?"
  grep -E "^\[sim3\]|^TIME" "$O/$tag.log"
  $P - "$O/$tag" <<"PY"
import sys, numpy as np, pycolmap
d = sys.argv[1]; db = pycolmap.Database.open(d + "/db.db"); kp = {}; de = {}
for im in db.read_all_images():
    kp[im.name] = np.asarray(db.read_keypoints(im.image_id)); de[im.name] = np.asarray(db.read_descriptors(im.image_id).data)
db.close()
np.savez_compressed(d + "/feats.npz", **{"kp_" + k: v for k, v in kp.items()}, **{"de_" + k: v for k, v in de.items()})
PY
  rm -f "${O:?}/${tag:?}/db.db"
done
echo "$(date +%T) LAPFULL DONE"
