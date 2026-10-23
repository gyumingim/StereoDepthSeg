#!/usr/bin/env python3
"""Re-run phone60 cells with per-feature OpenVINS logging (2026-10-23, collapse review).

Same job as imu_learn/phone60_run.py job <name> (same args, same stock bridge binary ov_bridge/build/ov_bridge),
but the bridge loads the patched library ~/scanner/src/ovbuild_feat/libov_msckf_lib.so via LD_LIBRARY_PATH
(bridge RUNPATH is overridden by LD_LIBRARY_PATH). That library = ~/scanner/src/open_vins copy (open_vins_feat) +
tools/ov_featlog_patch.py: it only writes lines to $OV_FEATLOG.msckf / .slam (kind t featid A|R|T u v pred_u pred_v
t_lastobs chi2 thr). Outputs go to runs/collapse_feat/ (runs/phone60 is never written), then poses are compared to
runs/phone60/<name>.poses.jsonl.gz (all fields except per-frame wall time 'ms') -> identity report.

  ../venv_ffs/bin/python tools/collapse_feat_run.py 131326_o0_V4 224323_o0_V4 ...
"""
import gzip
import json
import os
import subprocess
import sys
from pathlib import Path

SCAN3D = Path(__file__).resolve().parents[1]
OUT = SCAN3D / "runs" / "collapse_feat"
LIB = Path.home() / "scanner" / "src" / "ovbuild_feat"


def compare(name):
    a = [json.loads(x) for x in gzip.open(SCAN3D / "runs" / "phone60" / f"{name}.poses.jsonl.gz", "rt")]
    b = [json.loads(x) for x in gzip.open(OUT / f"{name}.poses.jsonl.gz", "rt")]
    if len(a) != len(b):
        return dict(identical=False, why=f"rows {len(a)} vs {len(b)}")
    diff = 0
    for x, y in zip(a, b):
        x = {k: v for k, v in x.items() if k != "ms"}
        y = {k: v for k, v in y.items() if k != "ms"}
        diff += x != y
    return dict(identical=diff == 0, rows=len(a), rows_differ=diff)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    ij = OUT / "identity.json"
    rep = json.load(open(ij)) if ij.exists() else {}
    for name in sys.argv[1:]:
        env = dict(os.environ, LD_LIBRARY_PATH=f"{LIB}:" + os.environ.get("LD_LIBRARY_PATH", ""),
                   OV_FEATLOG=str(OUT / f"{name}.feat"), CONDS=name.rsplit("_", 1)[1])
        code = ("import sys; sys.path.insert(0,'imu_learn'); import phone60_run as p; from pathlib import Path; "
                f"p.OUT = Path('{OUT}'); p.job('{name}')")
        r = subprocess.run([sys.executable, "-c", code], cwd=SCAN3D, env=env, stderr=open(OUT / f"{name}.err", "w"))
        rep[name] = dict(rc=r.returncode, **(compare(name) if r.returncode == 0 else {}))
        print(name, rep[name], flush=True)
    json.dump(rep, open(ij, "w"), indent=1)


if __name__ == "__main__":
    main()
