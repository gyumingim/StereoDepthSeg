#!/usr/bin/env python3
"""Night 11-27 item 2: unseen fill = room structure walls behind objects (CPU, no AI, never alters measured faces).

  python unseen.py detect OUT          -> OUT/walls.json (+ OUT/walls_top.png top view)
  python unseen.py fill OUT            -> OUT/gen.ply (generated faces only), OUT/unseen.json

Input mesh = ffsonly2 pre-texture mesh (ffsonly_ba2/b/e3c.ply). Measured faces = id < faces_in (e3c.json); e3c floor / ceiling
faces (id >= faces_in) are already generated and are left as they are. Heights: floor y = e3c floor (+y is down), ceiling y = -1.855.

detect: vertical measured faces (|n_y| < 0.2, between ceiling + 0.15 and floor - 0.15) -> dominant (Manhattan) direction from the
  area-weighted histogram of horizontal normal angles mod 90 deg -> per axis direction, 2 cm histogram of face offsets (faces whose
  normal is within 20 deg of that axis) -> peaks with >= MIN_SUP m2 support. A peak is a STRUCTURE wall only if it is outermost:
  measured area farther from the camera path than the plane (beyond it by > 10 cm) inside the plane's span is <= 5 % of the slab area,
  and its supported height span >= 1.2 m. Wall extent = 2nd..98th percentile of its support along the wall, extended to meet
  a perpendicular wall line if that line is within CORNER m of the end (corner).
fill: 4 cm cells (u along wall, y from ceiling to floor) of each structure wall. Cell measured = a measured face sample within 8 cm of
  the plane projects into it. Generated = not measured AND not see-through: a cell is see-through (an opening, e.g. door / passage)
  if >= 2 keyframes see FFS depth beyond the cell by > max(5 cm, 4 %) at its pixel (camera on the room side). Morphological cleanup:
  drop generated components < 0.05 m2. Output faces face the room side. Tag: all faces of gen.ply are generated.
"""
import argparse, json, sys, time
from pathlib import Path

import numpy as np

HOME = Path.home()
V2 = HOME / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"
MESH = V2 / "ffsonly_ba2/b/e3c.ply"; E3C = V2 / "ffsonly_ba2/b/e3c.json"
DUMP = V2 / "ba_partners2/run/dump_ba"; FUSED = V2 / "ffsonly_ba2/fused"
CEIL_Y = -1.855
MIN_SUP, CORNER, CELL, NEAR = 0.6, 0.6, 0.04, 0.08
import os  # noqa: E402
# v2 (11-27 23:36, after the Fable check, set BEFORE running): fill only 0.48 m wall columns whose MEASURED wall cells (within 8 cm of
# the plane, before dilation) cover >= UNSEEN_COLMIN of the column; generated cells of other columns go to gen_lowconf.ply (not shown).
COLMIN = float(os.environ.get("UNSEEN_COLMIN", "0")); COLW = 12


def load_mesh():
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(str(MESH)); V = np.asarray(m.vertices, np.float64); F = np.asarray(m.triangles, np.int64)
    e = json.loads(E3C.read_text()); fin = e["faces_in"]
    return V, F, fin, e["floor"]["y_m"]


def face_geom(V, F):
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]; N = np.cross(b - a, c - a); A = 0.5 * np.linalg.norm(N, axis=1)
    N = N / np.maximum(2 * A, 1e-12)[:, None]; C = (a + b + c) / 3; return C, N, A


def cams():
    c = np.load(DUMP / "cams.npz"); return c["kf"], c["T"].astype(np.float64), c["Kr"].astype(np.float64), [int(v) for v in c["size"]]


def cmd_detect(a):
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True); t0 = time.time()
    V, F, fin, floor_y = load_mesh(); C, N, A = face_geom(V, F[:fin])
    kf, T, Kr, _ = cams(); P = T[:, :3, 3]
    vert = (np.abs(N[:, 1]) < 0.2) & (C[:, 1] > CEIL_Y + 0.15) & (C[:, 1] < floor_y - 0.15)
    ang = np.mod(np.degrees(np.arctan2(N[vert, 2], N[vert, 0])), 90.0)
    h, e = np.histogram(ang, bins=180, range=(0, 90), weights=A[vert]); hs = h + np.roll(h, 1) + np.roll(h, -1)
    th0 = float(e[np.argmax(hs)] + 0.25)
    walls = []
    for k in range(4):
        th = np.radians(th0 + 90 * k); ax = np.array([np.cos(th), 0.0, np.sin(th)])   # wall normal direction (outward candidate)
        sel = vert & (N @ ax > np.cos(np.radians(20)))   # faces whose normal points along +ax (either side ok, normals may flip)
        sel |= vert & (N @ ax < -np.cos(np.radians(20)))
        off = C[sel] @ ax; w = A[sel]
        lo, hi = np.percentile(off, 0.5), np.percentile(off, 99.5)
        hh, ee = np.histogram(off, bins=np.arange(lo, hi + 0.02, 0.02), weights=w)
        sm = np.convolve(hh, np.ones(3), "same")
        pk = [i for i in range(1, len(sm) - 1) if sm[i] >= sm[i - 1] and sm[i] >= sm[i + 1] and sm[i] >= MIN_SUP]
        cam_off = np.median(P @ ax)
        for i in pk:
            o = float(ee[i] + 0.01)
            if o <= cam_off:     # this direction: walls are on the +ax side of the camera path
                continue
            band = sel & (np.abs(C @ ax - o) < 0.04)
            t_ax = np.array([-ax[2], 0.0, ax[0]]); u = C[band] @ t_ax; ys = C[band, 1]
            u0, u1 = np.percentile(u, 2), np.percentile(u, 98); y0, y1 = np.percentile(ys, 2), np.percentile(ys, 98)
            span = (C @ t_ax >= u0) & (C @ t_ax <= u1) & (C[:, 1] > CEIL_Y + 0.05) & (C[:, 1] < floor_y - 0.05)
            slab = span & (np.abs(C @ ax - o) < 1.0) | (span & (C @ ax > o))
            beyond = span & (C @ ax > o + 0.10)
            a_beyond = float(A[:fin][beyond].sum()); a_slab = float(A[:fin][slab].sum())
            walls.append(dict(k=k, theta_deg=th0 + 90 * k, ax=ax.tolist(), t=t_ax.tolist(), off=o, support_m2=float(sm[i]),
                              u=[float(u0), float(u1)], y=[float(y0), float(y1)], height_span=float(y1 - y0),
                              beyond_m2=a_beyond, slab_m2=a_slab, beyond_share=a_beyond / max(a_slab, 1e-9)))
    for w_ in walls:
        w_["structure"] = bool(w_["beyond_share"] <= 0.05 and w_["height_span"] >= 1.2)
    # keep the outermost structure wall per direction band only if two are within 0.3 m (double surfaces)
    for w_ in walls:   # double surfaces: same direction, offsets < 0.3 m apart -> keep the larger support only
        for v_ in walls:
            if v_ is not w_ and v_["k"] == w_["k"] and v_["structure"] and w_["structure"] and abs(v_["off"] - w_["off"]) < 0.3 and v_["support_m2"] > w_["support_m2"]:
                w_["structure"] = False; w_["dropped"] = f"double of off {v_['off']:.2f}"
    st = [w_ for w_ in walls if w_["structure"]]
    for w_ in st:   # corners: extend ends to perpendicular structure walls within CORNER
        ax, t_ = np.array(w_["ax"]), np.array(w_["t"])
        for end in (0, 1):
            best = None
            for v_ in st:
                if v_ is w_ or abs(np.array(v_["ax"]) @ ax) > 0.2:
                    continue
                axv = np.array(v_["ax"]); den = t_ @ axv
                if abs(den) < 1e-6:
                    continue
                # point on w_ line: p = ax*off + t*u ; on v_ plane: axv.p = offv -> u = (offv - off*ax.axv)/(t.axv)
                uc = (v_["off"] - w_["off"] * (ax @ axv)) / den
                # the corner must lie on v_'s extent (+- CORNER)
                pc = ax * w_["off"] + t_ * uc; uv = pc @ np.array(v_["t"])
                if not (v_["u"][0] - CORNER <= uv <= v_["u"][1] + CORNER):
                    continue
                dist = abs(uc - w_["u"][end])
                if dist <= CORNER and (best is None or dist < best[0]):
                    best = (dist, uc)
            if best:
                w_.setdefault("corner_ext", {})[str(end)] = [w_["u"][end], best[1]]; w_["u"][end] = float(best[1])
    res = dict(made=time.strftime("%F %T"), script=str(Path(__file__).resolve()), mesh=str(MESH), floor_y=floor_y, ceil_y=CEIL_Y,
               manhattan_deg=th0, n_candidates=len(walls), n_structure=len(st), walls=walls, runtime_s=round(time.time() - t0, 1))
    (out / "walls.json").write_text(json.dumps(res, indent=1)); print(json.dumps({k: v for k, v in res.items() if k != "walls"}))
    for w_ in walls:
        print(f"k{w_['k']} off {w_['off']:.2f} sup {w_['support_m2']:.2f} m2 u {w_['u'][0]:.2f}..{w_['u'][1]:.2f} h {w_['height_span']:.2f} beyond {w_['beyond_share']:.3f} -> {'WALL' if w_['structure'] else '-'}")
    top(out, V, F, fin, P, walls)


def top(out, V, F, fin, P, walls, gen=None):
    import cv2
    C, N, A = face_geom(V, F[:fin]); s = 60.0
    x0, z0 = C[:, 0].min() - 0.5, C[:, 2].min() - 0.5; Wd = int((C[:, 0].max() + 0.5 - x0) * s); Hd = int((C[:, 2].max() + 0.5 - z0) * s)
    img = np.zeros((Hd, Wd, 3), np.uint8)
    px = ((C[:, 0] - x0) * s).astype(int).clip(0, Wd - 1); pz = ((C[:, 2] - z0) * s).astype(int).clip(0, Hd - 1)
    img[pz, px] = (90, 90, 90)
    if gen is not None:
        gx = ((gen[:, 0] - x0) * s).astype(int).clip(0, Wd - 1); gz = ((gen[:, 2] - z0) * s).astype(int).clip(0, Hd - 1); img[gz, gx] = (255, 0, 255)
    for p in P:
        cv2.circle(img, (int((p[0] - x0) * s), int((p[2] - z0) * s)), 1, (0, 200, 255), -1)
    for w_ in walls:
        ax, t_ = np.array(w_["ax"]), np.array(w_["t"]); p0 = ax * w_["off"] + t_ * w_["u"][0]; p1 = ax * w_["off"] + t_ * w_["u"][1]
        col = (0, 255, 0) if w_["structure"] else (0, 0, 200)
        cv2.line(img, (int((p0[0] - x0) * s), int((p0[2] - z0) * s)), (int((p1[0] - x0) * s), int((p1[2] - z0) * s)), col, 2)
    cv2.imwrite(str(out / ("walls_top.png" if gen is None else "gen_top.png")), img)


def cmd_fill(a):
    import cv2
    out = Path(a.out); t0 = time.time(); assert not (out / "gen.ply").exists(), "never overwrite"
    W_ = json.loads((out / "walls.json").read_text()); walls = [w for w in W_["walls"] if w["structure"]]
    V, F, fin, floor_y = load_mesh(); C, N, A = face_geom(V, F[:fin])
    # dense samples of measured faces (centroid + vertices + edge mids) for the "measured" test
    Fm = F[:fin]; a_, b_, c_ = V[Fm[:, 0]], V[Fm[:, 1]], V[Fm[:, 2]]
    S = np.concatenate([C, a_, b_, c_, (a_ + b_) / 2, (b_ + c_) / 2, (a_ + c_) / 2])
    kf, T, Kr, (Wi, Hi) = cams(); n = len(kf)
    ZF = np.zeros((n, Hi, Wi), np.float16)
    for j in range(n):
        d = np.load(FUSED / f"{int(kf[j]):05d}.npz"); ZF[j] = np.where(d["F"] == 1, d["Z"], 0)
    verts, tris, stats = [], [], []; gen_low_cells = []
    for wi, w in enumerate(walls):
        ax, t_ = np.array(w["ax"]), np.array(w["t"]); o = w["off"]
        us = np.arange(w["u"][0], w["u"][1], CELL); ys = np.arange(CEIL_Y, floor_y, CELL)
        nu, ny = len(us), len(ys)
        near = np.abs(S @ ax - o) < NEAR
        iu = np.floor((S[near] @ t_ - w["u"][0]) / CELL).astype(int); iy = np.floor((S[near][:, 1] - CEIL_Y) / CELL).astype(int)
        ok = (iu >= 0) & (iu < nu) & (iy >= 0) & (iy < ny); meas = np.zeros((ny, nu), bool); meas[iy[ok], iu[ok]] = True
        meas_raw = meas.copy()
        meas = cv2.dilate(meas.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)   # 1-cell tolerance around measured faces
        UU, YY = np.meshgrid(us + CELL / 2, ys + CELL / 2); X = ax[None, None] * o + t_[None, None] * UU[..., None]; X[..., 1] = YY
        Xf = X.reshape(-1, 3); see = np.zeros(len(Xf), np.int32); seen = np.zeros(len(Xf), np.int32)
        for j in range(n):
            if (T[j][:3, 3] @ ax) >= o:      # camera must be on the room side
                continue
            Xc = (Xf - T[j][:3, 3]) @ T[j][:3, :3]; z = Xc[:, 2]; okz = (z > 0.1) & (z < 6)
            u = np.round(Kr[0, 0] * Xc[:, 0] / np.where(okz, z, 1) + Kr[0, 2]).astype(int); v = np.round(Kr[1, 1] * Xc[:, 1] / np.where(okz, z, 1) + Kr[1, 2]).astype(int)
            okz &= (u >= 0) & (u < Wi) & (v >= 0) & (v < Hi)
            idx = np.nonzero(okz)[0]; zb = ZF[j][v[idx], u[idx]].astype(np.float64); has = zb > 0
            seen[idx[has]] += 1
            st_ = has & (zb > z[idx] + np.maximum(0.05, 0.04 * z[idx])); see[idx[st_]] += 1
        see = see.reshape(ny, nu); seen = seen.reshape(ny, nu)
        gen = (~meas) & (see < 2)
        nl, lab, sts, _ = cv2.connectedComponentsWithStats(gen.astype(np.uint8), connectivity=4)
        small = np.isin(lab, [l for l in range(1, nl) if sts[l, cv2.CC_STAT_AREA] * CELL * CELL < 0.05]); gen &= ~small
        low = np.zeros_like(gen)
        if COLMIN > 0:
            for c0 in range(0, nu, COLW):
                if meas_raw[:, c0:c0 + COLW].mean() < COLMIN:
                    low[:, c0:c0 + COLW] = gen[:, c0:c0 + COLW]
            gen_low_cells.append((w, low.copy(), us)); gen &= ~low
        stats.append(dict(wall=wi, off=o, theta_deg=w["theta_deg"], cells=int(nu * ny), measured_m2=float(meas.sum() * CELL * CELL),
                          generated_m2=float(gen.sum() * CELL * CELL), opening_m2=float(((~meas) & (see >= 2)).sum() * CELL * CELL),
                          gen_with_1_see_through_m2=float((gen & (see == 1)).sum() * CELL * CELL), lowconf_m2=float(low.sum() * CELL * CELL),
                          n_columns=int((nu + COLW - 1) // COLW), n_columns_supported=int(sum(meas_raw[:, c0:c0 + COLW].mean() >= COLMIN for c0 in range(0, nu, COLW)))))
        gy, gx = np.nonzero(gen)
        for y_, x_ in zip(gy, gx):
            u0, y0 = w["u"][0] + x_ * CELL, CEIL_Y + y_ * CELL
            q = [ax * o + t_ * u0 + np.array([0, y0, 0]), ax * o + t_ * (u0 + CELL) + np.array([0, y0, 0]),
                 ax * o + t_ * (u0 + CELL) + np.array([0, y0 + CELL, 0]), ax * o + t_ * u0 + np.array([0, y0 + CELL, 0])]
            b0 = len(verts); verts += q
            # normal should point to the room side (-ax)
            tri = [[b0, b0 + 1, b0 + 2], [b0, b0 + 2, b0 + 3]]
            nrm = np.cross(q[1] - q[0], q[2] - q[0])
            if nrm @ ax > 0:
                tri = [[b0, b0 + 2, b0 + 1], [b0, b0 + 3, b0 + 2]]
            tris += tri
        print(json.dumps(stats[-1]), flush=True)
    import open3d as o3d
    Vg = np.array(verts, np.float64).reshape(-1, 3); Tg = np.array(tris, np.int64).reshape(-1, 3)
    g = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(Vg), o3d.utility.Vector3iVector(Tg)); g.merge_close_vertices(1e-6)
    g.paint_uniform_color([0.75, 0.55, 0.85]); g.compute_vertex_normals(); o3d.io.write_triangle_mesh(str(out / "gen.ply"), g)
    lv, lt = [], []
    for w, low, us in gen_low_cells:
        ax, t_ = np.array(w["ax"]), np.array(w["t"]); o = w["off"]
        for y_, x_ in zip(*np.nonzero(low)):
            u0, y0 = w["u"][0] + x_ * CELL, CEIL_Y + y_ * CELL; b0 = len(lv)
            lv += [ax * o + t_ * u0 + np.array([0, y0, 0]), ax * o + t_ * (u0 + CELL) + np.array([0, y0, 0]),
                   ax * o + t_ * (u0 + CELL) + np.array([0, y0 + CELL, 0]), ax * o + t_ * u0 + np.array([0, y0 + CELL, 0])]
            lt += [[b0, b0 + 1, b0 + 2], [b0, b0 + 2, b0 + 3]]
    if lt:
        gl = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.array(lv)), o3d.utility.Vector3iVector(np.array(lt))); gl.merge_close_vertices(1e-6)
        o3d.io.write_triangle_mesh(str(out / "gen_lowconf.ply"), gl)
    res = dict(made=time.strftime("%F %T"), script=str(Path(__file__).resolve()), mesh=str(MESH), walls_json=str(out / "walls.json"), colmin=COLMIN,
               lowconf_m2=float(sum(s_["lowconf_m2"] for s_ in stats)),
               fused=str(FUSED), cams=str(DUMP / "cams.npz"), cell_m=CELL, near_m=NEAR, n_walls=len(walls), walls=stats,
               generated_m2=float(sum(s["generated_m2"] for s in stats)), generated_faces=int(len(np.asarray(g.triangles))),
               measured_faces_altered=0, note="gen.ply holds generated faces only; the measured mesh file is not modified",
               runtime_s=round(time.time() - t0, 1))
    (out / "unseen.json").write_text(json.dumps(res, indent=1)); print(json.dumps({k: v for k, v in res.items() if k != "walls"}))
    top(out, V, F, fin, T[:, :3, 3], [dict(w, structure=True) for w in walls], gen=Vg if len(Vg) else None)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("cmd"); ap.add_argument("out"); a = ap.parse_args()
    {"detect": cmd_detect, "fill": cmd_fill}[a.cmd](a)
