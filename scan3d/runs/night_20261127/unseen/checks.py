#!/usr/bin/env python3
"""Night 11-27 item 2 checks (CPU, read-only on inputs): python checks.py all
1. hold-out: delete the MEASURED faces of the wall_kf538 ROI patch (within 10 cm of its plane and inside its hull + 10 cm), re-run
   detect + fill on that mesh -> held-out face centroids vs the regenerated wall plane (cm), and generated area inside the patch.
2. occlusion: from every 2nd keyframe (pixel stride 4), first hit of measured mesh vs generated faces: pixels where a generated face is
   hit > 2 cm in front of the measured hit (= generated face hides measured content) -> count (target 0).
3. FFS free-space gate (gsm_ffsgate.seen_through, the same rule as the 11-26 7.41 % check: >= 2 FFS keyframes see > max(3 cm, 1.5 %)
   beyond the face): violated share per wall (area). A wall with > 5 % is DROPPED (not tuned).
Writes checks.json; gen_final.ply = gen.ply minus dropped walls (if any).
"""
import json, sys, time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/gsmap"))
sys.path.insert(0, str(Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2/diag_plane_20261127/scripts"))
import unseen as U  # noqa: E402
import os  # noqa: E402
os.environ.setdefault("GSMAP_RUN", str(U.V2 / "ffsonly_ba2/evalrun_newcams2"))   # gsm_ffsgate reads it at import (only for its default fuse dir)


def holdout(res):
    import plane_diag as PD
    V, F, fin, fy = U.load_mesh(); C, N, A = U.face_geom(V, F[:fin])
    R = PD.roi_setup()["wall_kf538"]; n, d = R["n"], R["d"]
    from matplotlib.path import Path as MP
    hull = R["hull"]; cen = hull.mean(0); hull_big = cen + (hull - cen) * (1 + 0.10 / np.maximum(np.linalg.norm(hull - cen, axis=1, keepdims=True), 1e-6))
    q = np.c_[C @ R["e1"], C @ R["e2"]]; inside = MP(hull_big).contains_points(q) & (np.abs(C @ n + d) < 0.10)
    held = np.nonzero(inside)[0]
    keep = np.setdiff1d(np.arange(len(F)), held)
    Fr = F[keep]; fin_r = int((keep < fin).sum())
    out = HERE / "holdout"; out.mkdir(exist_ok=True)
    orig = U.load_mesh
    U.load_mesh = lambda: (V, Fr, fin_r, fy)
    try:
        class A_: pass
        a = A_(); a.out = str(out)
        if (out / "gen.ply").exists():
            (out / "gen.ply").rename(out / f"gen_old_{int(time.time())}.ply")
        U.cmd_detect(a); U.cmd_fill(a)
    finally:
        U.load_mesh = orig
    W = json.loads((out / "walls.json").read_text())
    best = None
    for w in W["walls"]:
        if not w["structure"]:
            continue
        ax = np.array(w["ax"]); cosang = abs(ax @ n)
        dist = np.abs(C[held] @ ax - w["off"])
        if cosang > np.cos(np.radians(10)) and (best is None or np.median(dist) < best[0]):
            best = (float(np.median(dist)), w, ax)
    r = dict(held_faces=int(len(held)), held_area_m2=float(A[held[held < fin]].sum()), n_struct_walls=sum(w["structure"] for w in W["walls"]))
    if best:
        _, w, ax = best; s = C[held] @ ax - w["off"]
        r.update(wall_off=w["off"], wall_theta=w["theta_deg"], held_vs_regen_plane_med_abs_cm=float(np.median(np.abs(s)) * 100),
                 held_vs_regen_plane_rms_cm=float(np.sqrt(np.mean(s ** 2)) * 100), held_vs_regen_signed_med_cm=float(np.median(s) * 100),
                 roi_ffs_plane_vs_regen_angle_deg=float(np.degrees(np.arccos(min(1.0, abs(ax @ n))))))
        import open3d as o3d
        g = o3d.io.read_triangle_mesh(str(out / "gen.ply")); Vg = np.asarray(g.vertices); Tg = np.asarray(g.triangles)
        if len(Tg):
            Cg = Vg[Tg].mean(1); qg = np.c_[Cg @ R["e1"], Cg @ R["e2"]]; ing = MP(hull_big).contains_points(qg) & (np.abs(Cg @ n + d) < 0.10)
            Ag = 0.5 * np.linalg.norm(np.cross(Vg[Tg[:, 1]] - Vg[Tg[:, 0]], Vg[Tg[:, 2]] - Vg[Tg[:, 0]]), axis=1)
            r["regen_area_in_patch_m2"] = float(Ag[ing].sum())
            from scipy.spatial import cKDTree
            dd, _ = cKDTree(Cg).query(C[held]); r["held_to_nearest_gen_face_med_cm"] = float(np.median(dd) * 100)
    else:
        r["note"] = "no structure wall within 10 deg of the wall538 plane was found after the hold-out"
    res["holdout_wall538"] = r; print("holdout", json.dumps(r), flush=True)


def occlusion_and_gate(res, gen_path):
    import open3d as o3d
    import gsm_ffsgate as FG
    V, F, fin, fy = U.load_mesh(); g = o3d.io.read_triangle_mesh(str(gen_path)); Vg = np.asarray(g.vertices); Tg = np.asarray(g.triangles)
    kf, T, Kr, (W, H) = U.cams()
    sm = o3d.t.geometry.RaycastingScene(); sm.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F[:fin].astype(np.uint32)))
    sg = o3d.t.geometry.RaycastingScene(); sg.add_triangles(o3d.core.Tensor(Vg.astype(np.float32)), o3d.core.Tensor(Tg.astype(np.uint32)))
    vv, uu = np.mgrid[0:H:4, 0:W:4]; Rc = np.stack([(uu - Kr[0, 2]) / Kr[0, 0], (vv - Kr[1, 2]) / Kr[1, 1], np.ones_like(uu, float)], -1).reshape(-1, 3)
    occ, both, genonly, tot = 0, 0, 0, 0
    for j in range(0, len(kf), 2):
        D = Rc @ T[j][:3, :3].T; o = np.broadcast_to(T[j][:3, 3], D.shape); rays = o3d.core.Tensor(np.concatenate([o, D], 1).astype(np.float32))
        tm = sm.cast_rays(rays)["t_hit"].numpy(); tg = sg.cast_rays(rays)["t_hit"].numpy(); nrm = np.linalg.norm(D, axis=1)
        fm, fg = np.isfinite(tm), np.isfinite(tg); tot += len(D)
        occ += int((fm & fg & ((tm - tg) * nrm > 0.02)).sum()); both += int((fm & fg).sum()); genonly += int((~fm & fg).sum())
    res["occlusion"] = dict(rays=tot, gen_in_front_of_measured_gt2cm=occ, share_of_rays=occ / max(tot, 1), rays_hit_both=both, rays_gen_only=genonly,
                            note="every 2nd keyframe, pixel stride 4, measured = ffsonly2 e3c faces < faces_in")
    print("occlusion", json.dumps(res["occlusion"]), flush=True)
    FF = FG.load_ffs(U.FUSED, kf)
    a, b, c = Vg[Tg[:, 0]], Vg[Tg[:, 1]], Vg[Tg[:, 2]]; Cg = (a + b + c) / 3; Ng = np.cross(b - a, c - a); Ag = 0.5 * np.linalg.norm(Ng, axis=1); Ng /= np.maximum(2 * Ag, 1e-12)[:, None]
    viol, nv = FG.seen_through(Cg, Ng.astype(np.float32), T, Kr, W, H, FF, 2)
    walls = json.loads((HERE / "unseen.json").read_text())["walls"]; WJ = json.loads((HERE / "walls.json").read_text())
    st = [w for w in WJ["walls"] if w["structure"]]; per = []; keep_face = np.ones(len(Tg), bool)
    for wi, w in enumerate(st):
        ax = np.array(w["ax"]); on = np.abs(Cg @ ax - w["off"]) < 0.01
        share = float(Ag[on & viol].sum() / max(Ag[on].sum(), 1e-9))
        per.append(dict(wall=wi, off=w["off"], theta_deg=w["theta_deg"], gen_m2=float(Ag[on].sum()), violated_m2=float(Ag[on & viol].sum()), violated_share=share,
                        dropped=share > 0.05))
        if share > 0.05:
            keep_face &= ~on
    res["ffsgate"] = dict(rule="gsm_ffsgate.seen_through: >= 2 FFS keyframes see > max(3 cm, 1.5 %) beyond the face centroid, |cos| > 0.15",
                          total_violated_share=float(Ag[viol].sum() / max(Ag.sum(), 1e-9)), per_wall=per)
    print("ffsgate", json.dumps(res["ffsgate"]), flush=True)
    g2 = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(Vg), o3d.utility.Vector3iVector(Tg[keep_face])); g2.remove_unreferenced_vertices()
    g2.paint_uniform_color([0.75, 0.55, 0.85]); g2.compute_vertex_normals(); o3d.io.write_triangle_mesh(str(HERE / "gen_final.ply"), g2)
    res["gen_final"] = dict(path=str(HERE / "gen_final.ply"), faces=int(keep_face.sum()), area_m2=float(Ag[keep_face].sum()), dropped_walls=[p["wall"] for p in per if p["dropped"]])


def main():
    t0 = time.time(); res = dict(made=time.strftime("%F %T"), script=str(Path(__file__).resolve()), mesh=str(U.MESH), gen=str(HERE / "gen.ply"))
    occlusion_and_gate(res, HERE / "gen.ply")
    holdout(res)
    res["scoring_note"] = "generated faces live only in gen*.ply; the measured mesh and its P2/P3/coverage/PSNR numbers are unchanged by construction (generated faces excluded)"
    res["runtime_s"] = round(time.time() - t0, 1)
    (HERE / "checks.json").write_text(json.dumps(res, indent=1, default=float)); print("done", res["runtime_s"])


if __name__ == "__main__":
    main()
