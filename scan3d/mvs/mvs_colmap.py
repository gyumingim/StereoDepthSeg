#!/usr/bin/env python3
"""mvs 11-12 method A: COLMAP PatchMatch stereo + stereo fusion (pycolmap CUDA) on the mvs_prep.py keyframes / fixed BA poses.
Workspace (COLMAP format) built by hand: images = PREP/img{S} (already undistorted pinhole), sparse = PREP/sparse960/tri
(scaled x2 for 1920), patch-match.cfg = explicit source views from PREP/views.json (same lists as the MVSFormer++ run).
Depth ranges: COLMAP's own from the sparse points. Options = pycolmap defaults (geom consistency on, filter on) except max_image_size.
Usage: mvs_colmap.py PREP WS --size 960|1920 [--n-src 10] [--skip-pm] [--skip-fusion]"""
import argparse
import json
import os
import shutil
import time
from pathlib import Path

import numpy as np


def scale_model(src, dst, s):
    dst.mkdir(parents=True, exist_ok=True)
    cl = []
    for l in (src / "cameras.txt").read_text().splitlines():
        if l.startswith("#") or not l.strip():
            continue
        p = l.split()
        cl.append(" ".join(p[:2] + [str(int(p[2]) * s), str(int(p[3]) * s)] + [repr(float(x) * s) for x in p[4:]]))
    (dst / "cameras.txt").write_text("\n".join(cl) + "\n")
    lines = [l for l in (src / "images.txt").read_text().splitlines() if not l.startswith("#")]
    out = []
    for k in range(0, len(lines) - 1, 2):
        out.append(lines[k])
        p = lines[k + 1].split()
        q = []
        for j in range(0, len(p), 3):
            q += [repr(float(p[j]) * s), repr(float(p[j + 1]) * s), p[j + 2]]
        out.append(" ".join(q))
    (dst / "images.txt").write_text("\n".join(out) + "\n")
    shutil.copy(src / "points3D.txt", dst / "points3D.txt")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prep"); ap.add_argument("ws")
    ap.add_argument("--size", type=int, default=960)
    ap.add_argument("--n-src", type=int, default=10)
    ap.add_argument("--skip-pm", action="store_true"); ap.add_argument("--skip-fusion", action="store_true")
    a = ap.parse_args()
    import pycolmap
    P, W = Path(a.prep), Path(a.ws)
    W.mkdir(parents=True, exist_ok=True)
    info = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), prep=str(P), size=a.size, n_src=a.n_src, pycolmap=pycolmap.__version__,
                cuda=bool(getattr(pycolmap, "has_cuda", None)))
    if not (W / "images").exists():
        os.symlink(P / f"img{a.size}", W / "images")
    tri = P / "sparse960" / "tri"
    if a.size == 960:
        txt = tri
    else:
        txt = W / "sparse_txt"; scale_model(tri, txt, a.size // 960)
    sp = W / "sparse"; sp.mkdir(exist_ok=True)
    r = pycolmap.Reconstruction(str(txt)); r.write(str(sp))
    st = W / "stereo"
    for d in ("depth_maps", "normal_maps", "consistency_graphs"):
        (st / d).mkdir(parents=True, exist_ok=True)
    V = json.loads((P / "views.json").read_text())
    names = sorted(V)
    (st / "patch-match.cfg").write_text("".join(f"{n}\n{', '.join(V[n]['src'][:a.n_src])}\n" for n in names))
    (st / "fusion.cfg").write_text("".join(f"{n}\n" for n in names))
    po = pycolmap.PatchMatchOptions()
    po.max_image_size = a.size; po.gpu_index = "0"; po.geom_consistency = True; po.filter = True
    info["patch_match_options"] = {k: getattr(po, k) for k in dir(po) if not k.startswith("_") and not callable(getattr(po, k))}
    if not a.skip_pm:
        t0 = time.time()
        pycolmap.patch_match_stereo(str(W), workspace_format="COLMAP", options=po)
        info["patch_match_s"] = round(time.time() - t0, 1)
    if not a.skip_fusion:
        fo = pycolmap.StereoFusionOptions(); fo.max_image_size = a.size
        info["fusion_options"] = {k: (getattr(fo, k) if not hasattr(getattr(fo, k), "tolist") else None) for k in dir(fo)
                                  if not k.startswith("_") and not callable(getattr(fo, k)) and k not in ("bounding_box", "mask_path")}
        t0 = time.time()
        rf = pycolmap.stereo_fusion(str(W / "fused.ply"), str(W), workspace_format="COLMAP", input_type="geometric", options=fo,
                                    output_type="PLY")
        info["fusion_s"] = round(time.time() - t0, 1)
        info["fused_points"] = int(rf.num_points3D()) if hasattr(rf, "num_points3D") else None
    prev = json.loads((W / "colmap_run.json").read_text()) if (W / "colmap_run.json").exists() else {}
    prev.update(info)
    (W / "colmap_run.json").write_text(json.dumps(prev, indent=1, default=str))
    print(json.dumps({k: v for k, v in prev.items() if "options" not in k}, indent=1, default=str))


if __name__ == "__main__":
    main()
