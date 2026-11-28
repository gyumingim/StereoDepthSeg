#!/bin/bash
# gsmap newrec 11-28: used as P (python) for StereoDepthSeg-fixes scanA/run_scanner.sh. Same python binary, every argument unchanged,
# except the reduce step's script fixes scan3d/runs/kfonly_20261111/make_reduced.py -> the vendored copy next to this file, which
# imports the vendored kf_select.py (so newrec no longer reads keyframe-selection code from the fixes worktree).
PYBIN=${NEWREC_PY_REAL:-$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python}
D=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
args=()
for x in "$@"; do
  case $x in
    */scan3d/runs/kfonly_20261111/make_reduced.py) args+=("$D/make_reduced.py"); echo "[newrec kfsel] make_reduced.py -> $D/make_reduced.py (vendored kf_select)" >&2;;
    *) args+=("$x");;
  esac
done
exec "$PYBIN" "${args[@]}"
