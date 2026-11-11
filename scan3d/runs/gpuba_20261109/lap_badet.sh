#!/bin/bash
# gpuba 11-11, ON THE LAPTOP: is the GPU BA (--ba-device cuda --fast-tracks) bit-repeatable from the same db?
# 3x plain, 3x with CUBLAS_WORKSPACE_CONFIG=:4096:8, 2x CPU BA (control). usage: lap_badet.sh R O G V SRC [KFO...]
R=$1; O=$2; G=$3; V=$4; SRC=$5; shift 5; KFO="$@"
P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python; CACHE=/home/system/scanner/laptop_cache
export SCAN3D_GPU_POLL_S=3 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
mkdir -p "${O:?}"
for sp in g1:cuda: g2:cuda: g3:cuda: w1:cuda:w w2:cuda:w w3:cuda:w c1:cpu: c2:cpu:; do
  IFS=: read tag bd ws <<< "$sp"
  F="--no-fast-tracks"; [ "$bd" = cuda ] && F="--fast-tracks"
  E=""; [ "$ws" = w ] && E="CUBLAS_WORKSPACE_CONFIG=:4096:8"
  env $E $P scan3d/scanA/kf_ba.py $R --vio $V --out "$O/$tag" --gt $G $KFO --feat sift --loop none --cache-root $CACHE --seed 0 \
    --db-from $SRC --ba-device $bd --save-tri $F > "$O/$tag.log" 2>&1 || echo "$tag failed rc $?"
  grep -E "^\[sim3\]" "$O/$tag.log" | sed "s/^/$tag /"
done
$P - "$O" <<"PY"
import sys, gzip, json, numpy as np
o = sys.argv[1]
L = lambda t: np.array([json.loads(l)["T_world_cam0"] for l in gzip.open(f"{o}/{t}/poses_ba.jsonl.gz", "rt")])
T = lambda t: np.load(f"{o}/{t}/tri_dump.npz")
for a, bs in (("g1", ["g2", "g3"]), ("w1", ["w2", "w3"]), ("c1", ["c2"]), ("g1", ["w1", "c1"])):
    for b in bs:
        ta, tb = T(a), T(b)
        print("POSEDIFF", a, b, float(np.abs(L(a) - L(b)).max()), "tri_equal", all(np.array_equal(ta[k], tb[k]) for k in ta.files))
PY
rm -f "$O"/*/tri_dump.npz
rm -rf "${SRC:?}"
