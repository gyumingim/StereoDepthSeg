#!/usr/bin/env python3
"""gsmap 2026-11-27 overnight W6: visibly CLEAN variants of ffsonly2 (remove evidence-free geometry), CPU, <= 8 threads.

  python gsm_w6clean.py vis   IN.ply OUT.npz --cams CAMS.npz            per-face keyframe visibility (face centre, mesh z-buffer)
  python gsm_w6clean.py apply IN.ply VIS.npz BASE_VOTES.npz OUTDIR --variant C1|C2|C3 [--far 0.35]
  python gsm_w6clean.py pic   OUT.jpg   (paths fixed below; ffsonly2 vs C1 vs C2 vs C3 + red removed-face rows)
  python gsm_w6clean.py fruler           (W4 F ruler, gsm_f_ruler.py unchanged + imported, into w6/fruler)

vis: per keyframe j of CAMS (training poses ba2): face centre -> camera, z > 0.1, pixel inside 960x540 (floor), visible when
     z <= zbuf(pixel centre) + max(2 cm, 1 % z) (same occlusion rule as gsm_ffs_support.py). vis_any = NO grazing cut (a face seen
     edge-on still counts as seen -> conservative removal); vis_cos = with |cos| > 0.15 (= the gsm_ffs_support / votes1c rule, sanity check).
apply (base = ffsonly2 carve.ply + its gsm_ffs_support votes 'agree'):
     cand = agree == 0 (no FFS view agrees)
     C1   cand & vis_any == 0                                   (never visible in any keyframe)
     C2   C1 | (cand & centre > 0.35 m from every face centre with agree >= 2)
     C3   C1 | (cand & centre > 0.20 m from every face centre with agree >= 2)
     Writes OUTDIR/w6.ply (kept faces, original order), OUTDIR/w6_votes.npz (base votes subset -> gsm_ffs_support.py --votes --comp-area 0.25
     = the NC rule, run by run_w6.sh), OUTDIR/removed.npz (per IN face: 0 kept, 1 never-seen, 2 far, 3 extra NC island (vs ffsonly2's NC),
     9 removed by the ffsonly2 NC rule anyway), OUTDIR/w6.json (areas whole / aisle ROI, area with agree >= 1 removed, inputs, time).
     The NC step is replicated here (same components() code) only to account the areas; the mesh that goes on is gsm_ffs_support.py's output.
"""
import json, sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gsm_ffs_support import components  # noqa: E402

ROI = dict(x=(-0.3, 3.3), z=(-5.0, -3.2))
HOME = Path.home() / "scanner/StereoDepthSeg-gsmap"
V2 = HOME / "scan3d/runs/gsmap_20261125_121801_v2"; W6 = V2 / "w6"; B2 = V2 / "ffsonly_ba2/b"


def load(p):
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(str(p)); V = np.asarray(m.vertices, np.float64); F = np.asarray(m.triangles, np.int64)
    P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]; n = np.cross(P1 - P0, P2 - P0); ar = 0.5 * np.linalg.norm(n, axis=1)
    return V, F, ((P0 + P1 + P2) / 3).astype(np.float32), ar, (n / (2 * ar[:, None] + 1e-12)).astype(np.float32)


def roi(c):
    return (c[:, 0] > ROI["x"][0]) & (c[:, 0] < ROI["x"][1]) & (c[:, 2] > ROI["z"][0]) & (c[:, 2] < ROI["z"][1])


def cmd_vis(inp, out, cams):
    import open3d as o3d
    t0 = time.time(); V, F, cen, ar, nrm = load(inp); nF = len(F)
    c = np.load(cams); kf, T, K = c["kf"], c["T"], c["Kr"]; w, h = [int(v) for v in c["size"]]
    sc = o3d.t.geometry.RaycastingScene(nthreads=8); sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
    uu, vv = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    ray = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
    vis_any = np.zeros(nF, np.int16); vis_cos = np.zeros(nF, np.int16); inview = np.zeros(nF, np.int16)
    for j in range(len(kf)):
        Rm, t = T[j][:3, :3], T[j][:3, 3]
        zb = sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(t, ray.shape), ray @ Rm.T], 1).astype(np.float32)), nthreads=8)["t_hit"].numpy().reshape(h, w)
        R32, t32 = Rm.astype(np.float32), t.astype(np.float32)
        Xc = (cen - t32) @ R32; z = Xc[:, 2]; ok = z > 0.1
        u = np.floor(K[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + K[0, 2]).astype(np.int32); v = np.floor(K[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + K[1, 2]).astype(np.int32)
        ok &= (u >= 0) & (u < w) & (v >= 0) & (v < h); idx = np.nonzero(ok)[0]
        if len(idx) == 0:
            continue
        inview[idx] += 1; zz = z[idx]; vis = zz <= zb[v[idx], u[idx]] + np.maximum(0.02, 0.01 * zz)
        vd = t32 - cen[idx]; vd /= np.linalg.norm(vd, axis=1, keepdims=True); cs = np.abs((nrm[idx] * vd).sum(1)) > 0.15
        vis_any[idx[vis]] += 1; vis_cos[idx[vis & cs]] += 1
        if j % 50 == 0:
            print(f"view {j}/{len(kf)} t={time.time() - t0:.0f}s", flush=True)
    np.savez_compressed(out, vis_any=vis_any, vis_cos=vis_cos, inview=inview, made=time.strftime("%F %T"), inp=str(inp), cams=str(cams), n_kf=len(kf))
    info = dict(made=time.strftime("%F %T"), inp=str(inp), cams=str(cams), n_kf=int(len(kf)), faces=nF, area_m2=round(float(ar.sum()), 2),
                never_vis_any_m2=round(float(ar[vis_any == 0].sum()), 3), never_vis_cos_m2=round(float(ar[vis_cos == 0].sum()), 3),
                never_inview_m2=round(float(ar[inview == 0].sum()), 3), runtime_s=round(time.time() - t0, 1))
    Path(str(out).replace(".npz", ".json")).write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


def masks(variant, far, ag, vis_any, cen):
    from scipy.spatial import cKDTree
    cand = ag == 0; typ = np.zeros(len(ag), np.uint8); c1 = cand & (vis_any == 0); typ[c1] = 1
    if variant in ("C2", "C3"):
        sup = ag >= 2; d, _ = cKDTree(cen[sup]).query(cen[cand & ~c1], k=1, workers=8)
        ci = np.nonzero(cand & ~c1)[0]; typ[ci[d > far]] = 2
    return typ


def nc_keep(V, F, ag, keep, comp_area=0.25):
    """the NC rule of gsm_ffs_support.py mild(): drop welded components < comp_area m2 with no agree > 0 face (on the kept faces)"""
    kidx = np.nonzero(keep)[0]; cl, nt, ar = components(V, F[kidx]); sup = np.bincount(cl, weights=(ag[kidx] > 0).astype(float), minlength=len(ar)) > 0
    rm = ((ar < comp_area) & ~sup)[cl]; out = keep.copy(); out[kidx[rm]] = False; return out


def cmd_apply(inp, visp, votesp, outdir, variant, far):
    import open3d as o3d
    t0 = time.time(); V, F, cen, ar, _ = load(inp); nF = len(F); base = np.load(votesp); ag = base["agree"]; vs = np.load(visp)
    assert len(ag) == nF and len(vs["vis_any"]) == nF
    typ = masks(variant, far, ag, vs["vis_any"], cen); keep = typ == 0
    nc_base = nc_keep(V, F, ag, np.ones(nF, bool)); nc_var = nc_keep(V, F, ag, keep)
    typ[keep & ~nc_var & nc_base] = 3; typ[(typ == 0) & ~nc_var] = 9   # 9: NC drops it in ffsonly2 too (not a W6 removal)
    od = Path(outdir); od.mkdir(parents=True, exist_ok=True)
    assert not (od / "w6.ply").exists(), "never overwrite"
    Fk = F[keep]; used = np.zeros(len(V), bool); used[Fk.ravel()] = True; rmp = -np.ones(len(V), np.int64); rmp[used] = np.arange(used.sum())
    o3d.io.write_triangle_mesh(str(od / "w6.ply"), o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V[used]), o3d.utility.Vector3iVector(rmp[Fk].astype(np.int32))),
                               write_ascii=False, write_vertex_normals=False)
    np.savez_compressed(od / "w6_votes.npz", **{k: base[k][keep] for k in ("agree", "front", "behind", "noffs", "occluded")})
    np.savez_compressed(od / "removed.npz", typ=typ, cen=cen, area=ar.astype(np.float32), nc_base_keep=nc_base, nc_var_keep=nc_var)
    R = roi(cen); A = lambda m: round(float(ar[m].sum()), 3)  # noqa: E731
    w6rm = (typ == 1) | (typ == 2) | (typ == 3)   # removed by W6 beyond what ffsonly2 removes anyway
    info = dict(made=time.strftime("%F %T"), worker="W6", variant=variant, far_m=far if variant != "C1" else None, inp=str(inp), vis=str(visp), base_votes=str(votesp),
                rule="cand = agree==0; C1 never visible (vis_any==0, no grazing cut); C2/C3 C1 | cand & dist to nearest agree>=2 face centre > far; then NC (comp < 0.25 m2, no agree>0)",
                faces_in=nF, area_in_m2=round(float(ar.sum()), 2), roi=ROI, roi_area_in_m2=A(R),
                ffsonly2_nc_area_m2=A(nc_base), variant_nc_area_m2=A(nc_var), ffsonly2_nc_roi_m2=A(nc_base & R), variant_nc_roi_m2=A(nc_var & R),
                removed_m2=dict(never_seen=dict(whole=A(typ == 1), roi=A(R & (typ == 1))), far=dict(whole=A(typ == 2), roi=A(R & (typ == 2))),
                                extra_nc_island=dict(whole=A(typ == 3), roi=A(R & (typ == 3)))),
                removed_vs_ffsonly2_m2=dict(whole=A(w6rm), roi=A(w6rm & R)),
                removed_vs_ffsonly2_pct=dict(whole=round(A(w6rm) / max(A(nc_base), 1e-9) * 100, 2), roi=round(A(w6rm & R) / max(A(nc_base & R), 1e-9) * 100, 2)),
                removed_with_ffs_agree_ge1_m2=A(w6rm & (ag >= 1)), removed_with_ffs_agree_ge2_m2=A(w6rm & (ag >= 2)),
                faces_out_rule=int(keep.sum()), out=str(od / "w6.ply"), runtime_s=round(time.time() - t0, 1))
    (od / "w6.json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


def look(eye, tgt):
    eye = np.asarray(eye, float); zc = np.asarray(tgt, float) - eye; zc /= np.linalg.norm(zc); xc = np.cross([0, 1.0, 0], zc); xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc); T = np.eye(4); T[:3, :3] = np.stack([xc, yc, zc], 1); T[:3, 3] = eye; return T


def cmd_pic(outp):
    import cv2
    import open3d as o3d
    from gsm_check import load_glb
    from gsm_crops import rot
    w, h, fl = 960, 540, 520.0
    c = np.load(V2 / "ba_partners2/run/dump_ba/cams.npz"); T, K, kf = c["T"], c["Kr"], c["kf"]; kfl = [int(x) for x in kf]
    V, F, cen, ar, nrm = load(B2 / "carve.ply"); ag = np.load(B2 / "support/support_votes.npz")["agree"]
    sup = ag >= 2; lo = np.percentile(cen[sup], 3, axis=0); hi = np.percentile(cen[sup], 97, axis=0)
    mid = (lo + hi) / 2; corner = np.array([lo[0] + 0.3, -1.45, hi[2] - 0.3]); tgt = np.array([mid[0], 0.4, mid[2]])
    j709 = kfl.index(709)
    views = [("aisle audit (F3 eye)", look((-0.9, -0.25, -3.75), (1.5, 0.55, -4.45)), fl),
             ("aisle kf2482 down20", rot(T[316], 0, 20), None), ("aisle kf2525 down15", rot(T[324], 0, 15), None),
             ("cabinet front kf774", T[72], None), ("white wall kf709", T[j709], None),
             ("overview from corner", look(corner, tgt), 360.0)]
    maps = [("ffsonly2", B2 / "tex_121801_ffsonly2_pad.glb")] + [(f"C{i}", W6 / f"c{i}/tex_121801_c{i}_pad.glb") for i in (1, 2, 3)]
    rems = [(f"C{i} removed (red)", W6 / f"c{i}/removed.npz") for i in (1, 2, 3)]
    uu, vv = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)

    def dirs(Tv, f):
        if f is None:
            d = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
        else:
            d = np.stack([(uu - w / 2) / f, (vv - h / 2) / f, np.ones_like(uu)], -1).reshape(-1, 3)
        return d @ Tv[:3, :3].T

    def cast(s, Tv, f):
        dw = dirs(Tv, f); return s.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(Tv[:3, 3], dw.shape), dw], 1).astype(np.float32)), nthreads=8), dw

    rows = []
    for nm, p in maps:
        GV, UV, GF, tex = load_glb(str(p)); s = o3d.t.geometry.RaycastingScene(); s.add_triangles(o3d.core.Tensor(GV), o3d.core.Tensor(GF)); tl = []
        for vn, Tv, f in views:
            r, _ = cast(s, Tv, f); pid = r["primitive_ids"].numpy(); bu = r["primitive_uvs"].numpy(); hit = pid != 4294967295
            img = np.zeros((h * w, 3), np.uint8); img[:] = (60, 0, 60); pp = pid[hit]; b1, b2 = bu[hit, 0], bu[hit, 1]; b0 = 1 - b1 - b2
            uv = b0[:, None] * UV[GF[pp, 0]] + b1[:, None] * UV[GF[pp, 1]] + b2[:, None] * UV[GF[pp, 2]]
            Ah, Aw = tex.shape[:2]; img[hit] = tex[np.clip((uv[:, 1] * Ah).astype(int), 0, Ah - 1), np.clip((uv[:, 0] * Aw).astype(int), 0, Aw - 1)]
            tl.append((img.reshape(h, w, 3), f"{nm} | {vn}"))
        rows.append(tl)
    # red rows: the pre-fill mesh of ffsonly2 (carve faces kept by ffsonly2's NC rule), grey shading, red = removed by the variant
    for nm, p in rems:
        R = np.load(p); typ = R["typ"]; base = R["nc_base_keep"]; assert len(typ) == len(F)
        idx = np.nonzero(base)[0]; s = o3d.t.geometry.RaycastingScene(); s.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F[idx].astype(np.uint32))); tl = []
        for vn, Tv, f in views:
            r, dw = cast(s, Tv, f); pid = r["primitive_ids"].numpy(); hit = pid != 4294967295; fi = idx[pid[hit]]
            dn = dw[hit] / np.linalg.norm(dw[hit], axis=1, keepdims=True); g = (60 + 170 * np.abs((nrm[fi] * dn).sum(1))).astype(np.uint8)
            img = np.zeros((h * w, 3), np.uint8); img[:] = (60, 0, 60); img[hit] = g[:, None]
            rm = np.isin(typ[fi], (1, 2, 3)); hh = np.nonzero(hit)[0][rm]; img[hh] = (0.3 * img[hh] + 0.7 * np.array((0, 0, 255))).astype(np.uint8)
            tl.append((img.reshape(h, w, 3), f"{nm} | {vn}"))
        rows.append(tl)
    out = []
    for tl in rows:
        t2 = []
        for x, nm in tl:
            x = cv2.resize(x, (480, 270), interpolation=cv2.INTER_AREA); cv2.putText(x, nm, (6, 18), 0, 0.5, (0, 0, 0), 3); cv2.putText(x, nm, (6, 18), 0, 0.5, (0, 255, 255), 1); t2.append(x)
        out.append(np.hstack(t2))
    leg = np.full((36, out[0].shape[1], 3), 20, np.uint8)
    cv2.putText(leg, "W6 121801: rows ffsonly2 | C1 never-seen | C2 +far>35cm | C3 +far>20cm (textured seampad GLB); last 3 rows = pre-fill mesh, red = removed by C1/C2/C3",
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.imwrite(str(outp), np.vstack([leg] + out), [cv2.IMWRITE_JPEG_QUALITY, 85])
    meta = dict(made=time.strftime("%F %T"), out=str(outp), maps={k: str(v) for k, v in maps}, removed={k: str(v) for k, v in rems},
                views=[dict(name=vn, T=Tv.tolist(), f=f) for vn, Tv, f in views], corner_eye=corner.tolist(), corner_tgt=tgt.tolist())
    (W6 / "pic.json").write_text(json.dumps(meta, indent=1)); print("wrote", outp, flush=True)


def cmd_fruler():
    import gsm_f_ruler as fr
    from argparse import Namespace
    fr.OUT = W6 / "fruler"; fr.OUT.mkdir(parents=True, exist_ok=True); assert not (fr.OUT / "f2f3.json").exists(), "never overwrite"
    M = {k: fr.MESHES[k] for k in ("ffsonly2_e3c", "W2_final", "W3A_fill")}   # W2/W3A only because the W4 compare-sheet code needs them
    for i in (1, 2, 3):
        M[f"C{i}_e3c"] = dict(ply=W6 / f"c{i}/e3c.ply", votes=W6 / f"aisle/c{i}_e3c/support_votes.npz", frame="ba2", glb=W6 / f"c{i}/tex_121801_c{i}.glb")
    fr.MESHES = M
    fr.cmd_f2f3(Namespace())
    f = json.loads((fr.OUT / "f2f3.json").read_text()); rows = {}
    for nm in ("ffsonly2_e3c", "C1_e3c", "C2_e3c", "C3_e3c"):
        o = f["meshes"][nm]
        rows[nm] = dict(area_m2=o["area_m2"], area_roi_m2=o["area_roi_m2"], unsupported_lt2_m2=o["unsupported_area_m2"], unsupported_lt2_roi_m2=o["unsupported_area_roi_m2"],
                        F3_red_px_9views=o["red_px_total_9views"], F3_audit_red_px=o["views"]["audit"]["red_px"], F3_audit_red_pct=o["views"]["audit"]["red_pct_of_hit"],
                        F3_audit_hit_px=o["views"]["audit"]["hit_px"], F3_tex_red_px_audit=o["tex_red_px_audit"], F2_tube_m2=o["f2_tube_area_m2"], sheet=o["sheet"])
    (fr.OUT / "w6_compare.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), source=str(fr.OUT / "f2f3.json"), n_recordings=1, rows=rows), indent=1))
    print(json.dumps(rows, indent=1))


def main():
    cmd = sys.argv[1]; a = sys.argv[2:]
    if cmd == "vis":
        cmd_vis(a[0], a[1], a[a.index("--cams") + 1])
    elif cmd == "apply":
        far = float(a[a.index("--far") + 1]) if "--far" in a else 0.35
        cmd_apply(a[0], a[1], a[2], a[3], a[a.index("--variant") + 1], far)
    elif cmd == "pic":
        cmd_pic(a[0])
    elif cmd == "fruler":
        cmd_fruler()


if __name__ == "__main__":
    main()
