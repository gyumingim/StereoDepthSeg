#!/usr/bin/env python3
"""integ-20261025: PC check of the c4f60 setting (C4 quality at 60 fps) on the two std60 live-camera-mode recordings
(hs_20261024_165506_std60, hs_20261024_173432_std60; 60 fps originals) x start offsets 0/1/2, vs the same at 30 fps.
Same bench + live-like protocol as ovacc acc_run.py L0 (histogram NONE, per-recording GT alignment, 960x540, 15 clones,
init window 1.0 s / disparity 22.5 px, single-threaded = deterministic), only fps changes: F30 = fps 30 (= ovacc L0 on
std60), F60 = fps 60 (= c4f60). Bridge = this branch's PC build (frozen copy). GT imu_gt_spl (= ovacc acc_std.py).
Purpose: does c4f60 initialise and track (not an accuracy decision: 2 recordings, 1 run per cell).

  python3 f60_pc.py run [--par 6];  python3 f60_pc.py score     (OUT /tmp/integ/f60)
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "ovacc_20261025"))
import acc_run as A  # noqa: E402

A.OUT = Path("/tmp/integ/f60")
A.WT = Path("/home/system/scanner/StereoDepthSeg-integ/scan3d")
SRC = A.WT / "ov_bridge" / "build_integ_full" / "ov_bridge"
FPS = {"F30": 30.0, "F60": 60.0}
for c in FPS:
    A.CONDS[c] = A.L()
A.cells = lambda: (A.cells_std(), None, set())
_cur = {}


class NS(argparse.Namespace):
    def __init__(self, **kw):
        kw["fps"] = FPS[_cur["cond"]]
        super().__init__(**kw)


A.argparse = types.SimpleNamespace(Namespace=NS)


def freeze():
    (A.OUT / "bin").mkdir(parents=True, exist_ok=True)
    h = hashlib.sha256(open(SRC, "rb").read()).hexdigest()
    dst = A.OUT / "bin" / f"ov_bridge_{h[:12]}"
    if not dst.exists():
        shutil.copy2(SRC, dst)
    head = subprocess.run(["git", "-C", str(A.WT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    json.dump(dict(src=str(SRC), sha256=h, copy=str(dst), frozen=time.strftime("%F %T"), worktree_head=head),
              open(A.OUT / "bin" / "bridge.json", "w"), indent=1)
    for c in FPS:
        w = A.wrapper(c)
        w.write_text(f"#!/bin/bash\nexec {dst} \"$@\"\n")
        w.chmod(0o755)


def run(par):
    freeze()
    q = [A.name_of(r, o, c) for c in FPS for r, o in A.cells_std()]
    q = [n for n in q if not (A.OUT / n.split("_")[2] / f"{n}.json").exists()]
    running = {}
    print(f"{time.strftime('%F %T')} start {len(q)} jobs", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (A.OUT / n.split("_")[2] / f"{n}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED'}", flush=True)
        while q and len(running) < par:
            n = q.pop(0)
            (A.OUT / n.split("_")[2]).mkdir(parents=True, exist_ok=True)
            err = open(A.OUT / n.split("_")[2] / f"{n}.err", "w")
            running[n] = (subprocess.Popen(["nice", "-n", "5", A.PY, str(Path(__file__).resolve()), "job", n],
                                           stdout=subprocess.DEVNULL, stderr=err), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end", flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "job", "score"])
    ap.add_argument("arg", nargs="?", default="")
    ap.add_argument("--par", type=int, default=6)
    a = ap.parse_args()
    if a.cmd == "run":
        run(a.par)
    elif a.cmd == "job":
        _cur["cond"] = a.arg.split("_")[2]
        A.job(a.arg)
    else:
        sys.path.insert(0, str(HERE.parent / "ovacc_20261025"))
        import acc_std as S
        S.score(list(FPS), "F30")
