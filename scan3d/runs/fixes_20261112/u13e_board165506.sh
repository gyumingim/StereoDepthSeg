#!/usr/bin/env bash
# step 5: 165506 board GT with the NEW DEFAULT (no --ffs-backend anywhere; host.env -> trt), same BA poses (fix_u13c_pt) as fix_u13d_pt/trt
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-fixes
cd "$WT"
export PATH=/usr/local/cuda/bin:$PATH PYTHONUNBUFFERED=1
unset SCAN3D_FFS_BACKEND
REC=/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261024_165506_std60
D=$WT/scan3d/runs/desktop/20261024_165506_std60
t0=$SECONDS
SCANNER_COMMIT=$(git rev-parse HEAD)-dirty bash scan3d/scanA/run_scanner.sh "$REC" "$D/fix_u13e_default" --reuse "$D/fix_u13c_pt" --board auto \
  --depth-sigma ba --mesh-args "--tsdf-max-blocks 250000" > "$D/fix_u13e_default.log" 2>&1
echo "board165506 default rc $? $((SECONDS - t0)) s"
/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python - <<'PY'
import json
D = "/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/desktop/20261024_165506_std60/"
for v in ("fix_u13d_pt", "fix_u13d_trt", "fix_u13e_default"):
    try:
        s = json.load(open(D + v + "/scan_summary.json"))
    except Exception as e:
        print(v, "no summary", e); continue
    for mk in ("mesh", "mesh_board"):
        m = s.get(mk) or {}
        print(v, mk, {k: m.get(k) for k in ("absdZ_p90_cm", "k_pct", "mesh_board_z_rms_cm", "board_cover_frac", "n_kf_depth",
                                          "mesh_vertices", "ffs_backend", "ffs_backend_source", "ffs_s_total")})
    print(v, "steps", s.get("step_s"))
PY
