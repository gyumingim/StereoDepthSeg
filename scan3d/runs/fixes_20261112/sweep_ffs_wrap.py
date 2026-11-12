#!/usr/bin/env python3
"""FFS settings sweep 11-12 (desktop): run a FROZEN copy of scan_offline.py with one FFS factor changed, without editing it.
Usage: SWEEP_MAX_DISP=256 sweep_ffs_wrap.py SNAPSHOT/scan_offline.py <scan_offline args...>
  * SWEEP_MAX_DISP (default 192 = unchanged): motion_stereo.MSConfig.ffs_max_disp after every MSConfig() (also dataclasses.replace),
    and lib_ffs.load(max_disp=...) for the PyTorch path. scan_offline has no CLI flag for it; TrtFFS refuses engines whose manifest
    max_disp differs from cfg.ffs_max_disp, so a TRT run that starts at all proves the override took effect.
  * the snapshot (git show HEAD:scan3d/scanA/scan_offline.py, taken at sweep start) keeps every config on the same code even if
    the worktree file is edited by another agent during the sweep. scan3d/ and scan3d/scanA/ of the worktree go on sys.path.
Writes OUT/sweep_meta.json (time, argv, env factors, sha of the snapshot and of the imported modules)."""
import hashlib
import json
import os
import runpy
import sys
import time
from pathlib import Path

WT = Path("/home/system/scanner/StereoDepthSeg-fixes")
S3 = WT / "scan3d"
snap = Path(sys.argv[1]).resolve()
sys.argv = [str(snap)] + sys.argv[2:]
for p in (str(S3 / "scanA"), str(S3)):
    sys.path.insert(0, p)
if os.environ.get("FFS_ROOT"):
    sys.path.insert(0, os.environ["FFS_ROOT"])
import motion_stereo as ms  # noqa: E402

MD = int(os.environ.get("SWEEP_MAX_DISP", "192"))
if MD != 192:
    _init = ms.MSConfig.__init__

    def _patched(self, *a, **k):
        _init(self, *a, **k)
        self.ffs_max_disp = MD
    ms.MSConfig.__init__ = _patched
    import lib_ffs  # noqa: E402
    _load = lib_ffs.load

    def _load_md(ckpt=lib_ffs.DEFAULT_CKPT, valid_iters=8, max_disp=192):
        return _load(ckpt, valid_iters=valid_iters, max_disp=MD)
    lib_ffs.load = _load_md
assert ms.MSConfig().ffs_max_disp == MD, "max_disp override failed"
print(f"[sweep] scan_offline snapshot {snap}, ffs_max_disp {MD}", flush=True)


def _sha(p):
    try:
        return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]
    except Exception as e:
        return repr(e)


out = None
for i, x in enumerate(sys.argv):
    if x == "--out" and i + 1 < len(sys.argv):
        out = Path(sys.argv[i + 1])
meta = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), argv=sys.argv, snapshot=str(snap), snapshot_sha=_sha(snap),
            ffs_max_disp=MD, env={k: os.environ.get(k) for k in ("SWEEP_CFG", "SWEEP_MAX_DISP", "FFS_ROOT", "SCAN3D_FFS_BACKEND")},
            module_sha={m: _sha(S3 / m) for m in ("motion_stereo.py", "tsdf_map.py", "ffs_trt.py", "scanA/kf_frames.py",
                                                  "scanA/gpu_guard.py", "scanA/scan_offline.py")})
if out is not None:
    out.mkdir(parents=True, exist_ok=True)
    (out / "sweep_meta.json").write_text(json.dumps(meta, indent=1))
t0 = time.time()
try:
    runpy.run_path(str(snap), run_name="__main__")
finally:
    if out is not None:
        meta["process_s"] = round(time.time() - t0, 1)
        meta["ended"] = time.strftime("%Y-%m-%d %H:%M:%S")
        (out / "sweep_meta.json").write_text(json.dumps(meta, indent=1))
