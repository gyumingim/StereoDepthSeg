#!/usr/bin/env python3
"""gsmap 2026-11-26: FFS-priority free-space gate. CPU.

  mesh mode : GSMAP_RUN=<run> python gsm_ffsgate.py mesh IN.ply OUT.ply [--fuse DIR]
  depth mode: GSMAP_RUN=<run> python gsm_ffsgate.py depth FUSE_IN FUSE_OUT

Reference = filtered FFS only (fuse F == 1; default fuse = <run>/e6/fuse). A point is "seen through" by an FFS keyframe if it projects
inside it, the FFS pixel is valid and the point lies IN FRONT of the FFS surface by more than max(3 cm, 1.5 %) (FFS saw free space
there). Violated = seen through by >= --min-views (2) FFS keyframes.
mesh : face centroids (non-grazing |cos| > 0.15) -> delete violated faces; report area share overall and per eval2 ROI (ray-cast hits).
depth: every AI-fill pixel (F == 2) back-projected -> violated pixels dropped (F -> 0); report share per keyframe.
"""
import json, os, sys, time
from pathlib import Path

import numpy as np

R = Path(os.environ["GSMAP_RUN"])


def load_ffs(fuse, kf):
    out = []
    for i in kf:
        d = np.load(Path(fuse) / f"{int(i):05d}.npz"); out.append(np.where(d["F"] == 1, d["Z"], np.nan).astype(np.float32))
    return out


def seen_through(X, nrm, T, K, W, H, FF, min_views, cen_cams=None):
    viol = np.zeros(len(X), np.int16); X32 = X.astype(np.float32)
    for j in range(len(T)):
        if not np.isfinite(FF[j]).any():
            continue
        Rm, t = T[j][:3, :3].astype(np.float32), T[j][:3, 3].astype(np.float32); Xc = (X32 - t) @ Rm; z = Xc[:, 2]; ok = z > 0.1
        u = np.round(K[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + K[0, 2]).astype(np.int32); v = np.round(K[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + K[1, 2]).astype(np.int32)
        ok &= (u >= 0) & (u < W) & (v >= 0) & (v < H)
        if nrm is not None:
            vd = t - X32; vd /= np.linalg.norm(vd, axis=1, keepdims=True) + 1e-9; ok &= np.abs((nrm * vd).sum(1)) > 0.15
        idx = np.nonzero(ok)[0]; zr = FF[j][v[idx], u[idx]]
        viol[idx] += (np.isfinite(zr) & (z[idx] < zr - np.maximum(0.03, 0.015 * zr))).astype(np.int16)
    return viol >= min_views, viol


def main():
    mode = sys.argv[1]; arg = lambda k, d=None: sys.argv[sys.argv.index(k) + 1] if k in sys.argv else d  # noqa: E731
    mv = int(arg("--min-views", 2)); t0 = time.time()
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, K = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]
    if mode == "mesh":
        import open3d as o3d
        inp, outp = sys.argv[2], sys.argv[3]; FF = load_ffs(arg("--fuse", R / "e6" / "fuse"), kf)
        m = o3d.io.read_triangle_mesh(inp); V = np.asarray(m.vertices, np.float64); F = np.asarray(m.triangles, np.int64)
        P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]; cen = (P0 + P1 + P2) / 3; nr = np.cross(P1 - P0, P2 - P0)
        ar = 0.5 * np.linalg.norm(nr, axis=1); nr = (nr / (2 * ar[:, None] + 1e-12)).astype(np.float32)
        bad, cnt = seen_through(cen, nr, T, K, W, H, FF, mv)
        info = dict(mode="mesh", inp=inp, faces=int(len(F)), violated_faces=int(bad.sum()), violated_area_pct=round(float(ar[bad].sum() / ar.sum() * 100), 3))
        sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
        rois = json.loads((R / "eval2" / "ref" / "rois.json").read_text()); info["roi_hit_violated_pct"] = {}
        for r in rois:
            x0, x1, y0, y1 = r["rect"]; uu, vv = np.meshgrid(np.arange(x0, x1, 2) + 0.5, np.arange(y0, y1, 2) + 0.5)
            d = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1).reshape(-1, 3); Tc = T[r["idx"]]
            pid = sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(Tc[:3, 3], d.shape), d @ Tc[:3, :3].T], 1).astype(np.float32)))["primitive_ids"].numpy()
            hit = pid != 4294967295
            info["roi_hit_violated_pct"][r["name"]] = round(float(bad[pid[hit]].mean() * 100), 2) if hit.any() else None
        F = F[~bad]
        mo = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(F.astype(np.int32))); mo.remove_unreferenced_vertices()
        o3d.io.write_triangle_mesh(outp, mo, write_ascii=False, write_vertex_normals=False)
    else:
        fin, fout = Path(sys.argv[2]), Path(sys.argv[3]); fout.mkdir(parents=True, exist_ok=True)
        FF = load_ffs(fin, kf); per = []; tot = drop = 0
        for ci, i in enumerate(kf):
            d = dict(np.load(fin / f"{int(i):05d}.npz")); Z, Fz = d["Z"], d["F"]; v, u = np.nonzero(Fz == 2)
            if len(v):
                z = Z[v, u]; X = np.stack([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z], 1) @ T[ci][:3, :3].T + T[ci][:3, 3]
                ffs_other = [FF[j] if j != ci else np.full_like(FF[j], np.nan) for j in range(len(kf))]
                bad, _ = seen_through(X, None, T, K, W, H, ffs_other, mv)
                Fz = Fz.copy(); Z = Z.copy(); Fz[v[bad], u[bad]] = 0; Z[v[bad], u[bad]] = np.nan
                per.append(dict(idx=ci, kf=int(i), fill=int(len(v)), dropped=int(bad.sum()), has_ffs=bool(np.isfinite(FF[ci]).any()))); tot += len(v); drop += int(bad.sum())
            np.savez_compressed(fout / f"{int(i):05d}.npz", Z=Z, F=Fz)
        nof = [p for p in per if not p["has_ffs"]]; wf = [p for p in per if p["has_ffs"]]
        info = dict(mode="depth", fuse_in=str(fin), fill_px=tot, dropped_px=drop, dropped_pct=round(drop / max(1, tot) * 100, 2),
                    dropped_pct_kf_without_ffs=round(sum(p["dropped"] for p in nof) / max(1, sum(p["fill"] for p in nof)) * 100, 2),
                    dropped_pct_kf_with_ffs=round(sum(p["dropped"] for p in wf) / max(1, sum(p["fill"] for p in wf)) * 100, 2),
                    top_kf=sorted(per, key=lambda p: -p["dropped"] / max(1, p["fill"]))[:10], per_kf=per)
        outp = str(fout / "gate.json")
    info.update(made=time.strftime("%F %T"), min_views=mv, tol="max(3 cm, 1.5 %)", runtime_s=round(time.time() - t0, 1))
    Path(outp).with_suffix(".json").write_text(json.dumps(info, indent=1)); print(json.dumps({k: v for k, v in info.items() if k != "per_kf"}), flush=True)


if __name__ == "__main__":
    main()
