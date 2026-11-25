#!/usr/bin/env python3
"""gsmap 2026-11-25 E1: mesh free-space carving + mid-air island removal (no retrain), CPU.

  GSMAP_RUN=<v2 run> python scan3d/gsmap/gsm_carve.py IN.ply OUT.ply [--min-viol 5] [--ratio 2] [--island-area 0.25] [--island-gap 0.05]
(a) per face: violated / agreeing keyframes vs the eval2 references (gold FFS else 2DGS depth; same rule as eval2 P1);
    delete faces with violated >= --min-viol and violated > --ratio x agreeing.
(b) delete welded components < --island-area m2 whose median distance to components >= 1 m2 is > --island-gap.
(c) delete any component < --min-tri triangles left over.
Replaces gsm_v3fix's "seen by <= 2 keyframes" island rule. Writes OUT.ply + OUT.json (area removed per step).
"""
import argparse, json, sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gsm_eval2 import load_mesh, cams, ref_depth, face_votes, components, islands  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("--min-viol", type=int, default=5); ap.add_argument("--ratio", type=float, default=2.0)
    ap.add_argument("--island-area", type=float, default=0.25); ap.add_argument("--island-gap", type=float, default=0.05); ap.add_argument("--min-tri", type=int, default=200)
    a = ap.parse_args()
    import open3d as o3d
    t0 = time.time(); V, F, _, _ = load_mesh(a.inp); kf, T, K, w, h = cams(); gold, ZM, AL = ref_depth()
    ar_f = lambda V, F: 0.5 * np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)  # noqa: E731
    A0 = ar_f(V, F).sum(); info = dict(faces_in=int(len(F)), area_in_m2=round(float(A0), 2))
    viol, agr, cen = face_votes(V, F, T, K, w, h, gold, ZM, AL)
    fl = (viol >= a.min_viol) & (viol > a.ratio * agr)
    info["a_floater_faces"] = int(fl.sum()); info["a_floater_area_pct"] = round(float(ar_f(V, F)[fl].sum() / A0 * 100), 3)
    F = F[~fl]; cen = cen[~fl]
    cl, nt, ar = components(V, F); isl = islands(V, F, cl, ar, cen, small=a.island_area, gap=a.island_gap)
    rm = isl[cl]; info["b_islands"] = int(isl.sum()); info["b_island_area_pct"] = round(float(ar_f(V, F)[rm].sum() / A0 * 100), 3)
    F = F[~rm]
    cl, nt, ar = components(V, F); sm = (nt < a.min_tri)[cl]
    info["c_small_components"] = int((nt < a.min_tri).sum()); info["c_small_area_pct"] = round(float(ar_f(V, F)[sm].sum() / A0 * 100), 3)
    F = F[~sm]
    used = np.zeros(len(V), bool); used[F.ravel()] = True; rmp = -np.ones(len(V), np.int64); rmp[used] = np.arange(used.sum()); V = V[used]; F = rmp[F]
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(F.astype(np.int32)))
    o3d.io.write_triangle_mesh(a.out, m, write_ascii=False, write_vertex_normals=False)
    info.update(made=time.strftime("%F %T"), inp=a.inp, args=vars(a), faces_out=int(len(F)), area_out_m2=round(float(ar_f(V, F).sum()), 2),
                removed_area_pct=round(float((1 - ar_f(V, F).sum() / A0) * 100), 3), runtime_s=round(time.time() - t0, 1))
    Path(a.out).with_suffix(".json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


if __name__ == "__main__":
    main()
