#!/usr/bin/env python3
"""FFS settings sweep 11-12: collect board 165506 numbers (from scan_offline summaries) and office 121801 mesh metrics vs base_a.
Office metric rules = runs/fixes_20261112/far_eval.py (unit 14, copied, no import):
  * camera depth of a vertex = nearest integrated keyframe camera of base_a (kf_poses.npz, 960x540 view test, no occlusion test)
  * top-view area = 5 cm cells (z up) holding vertices; also only vertices with camera depth > 1.6 m
  * near (< 1.2 m camera depth): nearest-vertex distance config -> base_a and base_a -> config, median / p90 mm
  * floaters = vertices in connected components < 500 / < 5000 welded vertices (a small real object also counts)
No ground truth on 121801: near distances = agreement with base, not accuracy. Accuracy only from the board (Z ~0.72 m).
Writes R/sweep_results.json, R/sweep_table.md, R/img_board_kf_p90.png, R/img_office_top_<cfg>.png
Usage: sweep_ffs_eval.py [--render CFG ...]"""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

R = Path("/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/desktop/sweep_ffs_20261112")
K = Path("/home/system/scanner/sweep_ffs/runs")
ORDER = ["base_a", "base_b", "base_rb", "it4", "it12", "it16", "md256", "r1280"]


def read_mesh(p):
    with open(p, "rb") as f:
        hdr = b""
        while not hdr.endswith(b"end_header\n"):
            hdr += f.readline()
        h = hdr.decode()
        nv = int(h.split("element vertex ")[1].split()[0])
        nf = int(h.split("element face ")[1].split()[0]) if "element face" in h else 0
        vb = h.split("element vertex")[1].split("element")[0].splitlines()
        props = [l.split()[-1] for l in vb if l.startswith("property")]
        types = [l.split()[1] for l in vb if l.startswith("property")]
        tmap = {"float": "<f4", "uchar": "u1", "double": "<f8", "int": "<i4"}
        dt = np.dtype([(n, tmap[t]) for n, t in zip(props, types)])
        v = np.frombuffer(f.read(nv * dt.itemsize), dt, nv)
        fr = np.frombuffer(f.read(nf * 13), np.dtype([("n", "u1"), ("i", "<i4", 3)]), nf)
    X = np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)
    C = np.stack([v["red"], v["green"], v["blue"]], 1)
    return X, C, fr["i"].astype(np.int64)


def cam_depth(X, T, f, cx, cy, W, H):
    best = np.full(len(X), np.inf, np.float32)
    Xf = X.astype(np.float32)
    for Tw in T:
        Rm = Tw[:3, :3].astype(np.float32); t = Tw[:3, 3].astype(np.float32)
        pc = (Xf - t) @ Rm
        z = pc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = pc[:, 0] / z * f + cx; v = pc[:, 1] / z * f + cy
        inn = (z > 0.05) & (u >= 0) & (u < W) & (v >= 0) & (v < H)
        best = np.where(inn, np.minimum(best, z), best)
    return best


def comp_size(X, F):
    _, wid = np.unique(np.round(X / 1e-4).astype(np.int64), axis=0, return_inverse=True)
    wid = wid.ravel()
    Fw = wid[F]
    nw = int(wid.max()) + 1
    e = np.concatenate([Fw[:, [0, 1]], Fw[:, [1, 2]]])
    g = coo_matrix((np.ones(len(e), np.int8), (e[:, 0], e[:, 1])), shape=(nw, nw))
    nc, lab_w = connected_components(g, directed=False)
    sz = np.bincount(lab_w, minlength=nc)
    return sz[lab_w[wid]], nc, sz


def cells(X):
    return set(map(tuple, np.unique(np.floor(X[:, :2] / 0.05).astype(np.int64), axis=0))) if len(X) else set()


def top(X, C, lo, hi, res, cams):
    W, H = int((hi[0] - lo[0]) / res) + 1, int((hi[1] - lo[1]) / res) + 1
    img = np.full((H, W, 3), 255, np.uint8)
    u = ((X[:, 0] - lo[0]) / res).astype(int); v = (H - 1 - (X[:, 1] - lo[1]) / res).astype(int)
    k = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    o = np.argsort(X[k, 2])
    img[v[k][o], u[k][o]] = C[k][o][:, ::-1]
    for cxy in cams:
        cv2.circle(img, (int((cxy[0] - lo[0]) / res), int(H - 1 - (cxy[1] - lo[1]) / res)), 2, (0, 0, 255), -1)
    return img


def js(p):
    try:
        return json.loads(Path(p).read_text())
    except Exception:
        return None


def board_row(c, name):
    s = js(R / c / f"{name}_summary.json")
    if not s:
        return None
    w = js(R / c / f"{name}_wall.json") or {}
    kf = js(R / c / f"{name}_keyframes.json") or []
    return dict(absdZ_p90_cm=s["pooled_absdZ_cm"]["p90"], absdZ_median_cm=s["pooled_absdZ_cm"]["median"],
                kf_absdZ_p90_cm_median=s.get("kf_board_absdZ_p90_cm_median"), kf_bias_cm_median=s.get("kf_board_dZ_bias_cm_median"),
                board_Z_m=s.get("kf_board_Z_m_median"), plane_rms_cm=s.get("mesh_board_z_rms_cm"), cover_frac=s.get("mesh_board_cover_frac"),
                n_kf_depth=s.get("n_kf_depth"), n_kf_board_eval=s.get("n_kf_board_eval"), vertices=s.get("mesh_vertices"),
                ffs_s=s.get("ffs_s_total"), runtime_s=s.get("runtime_s"), wall_s=w.get("wall_s"), rc=w.get("rc"),
                image_size=s.get("image_size"), f=(s.get("K_used") or [[None]])[0][0], ffs_backend=s.get("ffs_backend"),
                iters=s["args"]["iters"], scale=s["args"]["scale"], engines=s["args"]["ffs_engines"],
                kf_p90=[r["board_absdZ_p90_cm"] for r in kf if r.get("board_absdZ_p90_cm") is not None],
                ffs_scale_lt1=int(sum(1 for r in kf if r.get("scale") and min(r["scale"]) < 1)) if kf and "scale" in kf[0] else None)


def main():
    t0 = time.time()
    render = sys.argv[sys.argv.index("--render") + 1:] if "--render" in sys.argv else []
    cfgs = [c for c in ORDER if (R / c / "office_summary.json").exists() or (R / c / "board_mesh_summary.json").exists()]
    out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), inputs=dict(
        board="165506: fix_u13c_pt keyframes + ba/poses_ba (board_mesh) and board/poses_board (board_meshb)",
        office="121801: laptop fix_u8 keyframes + BA poses, fix_u8 mesh command, --tsdf-max-blocks 250000",
        snapshot=js(R / "snapshot.json"), note="1 run each, unverified (base_a / base_b = same engines twice, base_rb = same recipe rebuilt)"),
        configs={})
    P = np.load(K / "base_a" / "office" / "kf_poses.npz")
    T = P["T"]; f, cx, cy = float(P["f"]), float(P["cx"]), float(P["cy"]); W, H = (int(x) for x in P["size"])
    base = None
    M = {}
    for c in cfgs:
        row = dict(board=board_row(c, "board_mesh"), board_scaled=board_row(c, "board_meshb"))
        meta = js(R / c / "office_sweep_meta.json") or {}
        s = js(R / c / "office_summary.json")
        mp = K / c / "office" / "mesh.ply"
        if s and mp.exists():
            w = js(R / c / "office_wall.json") or {}
            X, C, F = read_mesh(mp)
            cd = cam_depth(X, T, f, cx, cy, W, H)
            fin = np.isfinite(cd)
            vsz, nc, sz = comp_size(X, F)
            g16 = fin & (cd > 1.6)
            o = dict(vertices=int(len(X)), faces=int(len(F)), top_area_m2=round(len(cells(X)) * 0.0025, 2),
                     top_area_gt_1p6_m2=round(len(cells(X[g16])) * 0.0025, 2), v_gt_1p6=int(g16.sum()),
                     cam_depth_p90_m=float(np.percentile(cd[fin], 90)), cam_depth_max_m=float(cd[fin].max()),
                     components=int(nc), floater_frac_lt500=float(np.mean(vsz < 500)), floater_frac_lt5000=float(np.mean(vsz < 5000)),
                     components_lt500=int((sz < 500).sum()), components_lt5000=int((sz < 5000).sum()),
                     ffs_s=s.get("ffs_s_total"), runtime_s=s.get("runtime_s"), wall_s=w.get("wall_s"), process_s=meta.get("process_s"),
                     gpu_peak_mib=w.get("gpu_peak_mib"), rc=w.get("rc"), tsdf_blocks=s.get("tsdf_blocks"), tsdf_full=s.get("tsdf_full"),
                     n_kf_depth=s.get("n_kf_depth"), image_size=s.get("image_size"), ffs_trt_shapes=(s.get("ffs_trt") or {}).get("shapes"),
                     ffs_max_disp=meta.get("ffs_max_disp"))
            if c == "base_a":
                base = (X, cd, cKDTree(X))
            if base is not None:
                Xb, cdb, tb = base
                nb, nn = np.isfinite(cdb) & (cdb < 1.2), fin & (cd < 1.2)
                d_cb, _ = tb.query(X[nn], k=1, workers=-1)
                d_bc, _ = cKDTree(X).query(Xb[nb], k=1, workers=-1)
                o["near_lt_1p2m"] = dict(cfg_to_base_median_mm=float(np.median(d_cb) * 1000), cfg_to_base_p90_mm=float(np.percentile(d_cb, 90) * 1000),
                                         base_to_cfg_median_mm=float(np.median(d_bc) * 1000), base_to_cfg_p90_mm=float(np.percentile(d_bc, 90) * 1000),
                                         cfg_to_base_frac_gt_1cm=float(np.mean(d_cb > 0.01)), base_covered_frac_1cm=float(np.mean(d_bc <= 0.01)),
                                         exact_frac_0p1mm=float(np.mean(d_cb < 1e-4)), n_cfg=int(nn.sum()), n_base=int(nb.sum()))
            row["office"] = o
            if c in render or c == "base_a":
                M[c] = (X, C)
            print(c, json.dumps(o)[:400], flush=True)
        row["meta"] = dict(office=meta, engine_manifest_made=(js(R / c / "engine_manifest.json") or {}).get("made"))
        out["configs"][c] = row
    (R / "sweep_results.json").write_text(json.dumps(out, indent=1))
    # table
    L = ["| cfg | board |dZ| p90 (cm) | kf p90 median (cm) | bias (cm) | plane RMS (cm) | cover | board_scaled |dZ| p90 / kf p90 (cm) | "
         "office near cfg->base med / p90 (mm) | vertices | top area / >1.6 m (m2) | floater <5000 frac | office FFS s | office runtime s | board FFS s |",
         "|" + "---|" * 15]
    for c, r in out["configs"].items():
        b, bs, o = r.get("board") or {}, r.get("board_scaled") or {}, r.get("office") or {}
        nr = o.get("near_lt_1p2m") or {}
        g = lambda d, k, fm="%.3f": (fm % d[k]) if d.get(k) is not None else "-"
        L.append(f"| {c} | {g(b,'absdZ_p90_cm')} | {g(b,'kf_absdZ_p90_cm_median')} | {g(b,'kf_bias_cm_median')} | {g(b,'plane_rms_cm')} | "
                 f"{g(b,'cover_frac')} | {g(bs,'absdZ_p90_cm')} / {g(bs,'kf_absdZ_p90_cm_median')} | "
                 f"{g(nr,'cfg_to_base_median_mm','%.2f')} / {g(nr,'cfg_to_base_p90_mm','%.2f')} | {o.get('vertices','-')} | "
                 f"{o.get('top_area_m2','-')} / {o.get('top_area_gt_1p6_m2','-')} | {g(o,'floater_frac_lt5000','%.4f')} | "
                 f"{o.get('ffs_s','-')} | {o.get('runtime_s','-')} | {b.get('ffs_s','-')} |")
    (R / "sweep_table.md").write_text(f"made {out['made']}, 1 run each, unverified\n\n" + "\n".join(L) + "\n")
    print("\n".join(L))
    # board image: per-keyframe board |dZ| p90 per config (box) + pooled p90 (dot)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cs = [c for c in out["configs"] if (out["configs"][c].get("board") or {}).get("kf_p90")]
    if cs:
        fig, ax = plt.subplots(figsize=(8, 3.6), dpi=130)
        data = [out["configs"][c]["board"]["kf_p90"] for c in cs]
        ax.boxplot(data, showfliers=False, whis=(10, 90))
        ax.plot(range(1, len(cs) + 1), [out["configs"][c]["board"]["absdZ_p90_cm"] for c in cs], "o", color="#c0392b",
                label="pooled mesh |dZ| p90")
        ax.set_xticks(range(1, len(cs) + 1)); ax.set_xticklabels(cs)
        ax.set_ylabel("cm"); ax.set_title("165506 board: per-keyframe |dZ| p90 (box p25-p75, whiskers p10-p90)", fontsize=9)
        ax.grid(axis="y", alpha=0.3); ax.legend(fontsize=8)
        fig.tight_layout(); fig.savefig(R / "img_board_kf_p90.png"); plt.close(fig)
    # office renders: top view (z up), colour, same window for all
    if M:
        Xb = M["base_a"][0]
        lo = np.percentile(Xb[:, :2], 0.5, 0) - 0.3; hi = np.percentile(Xb[:, :2], 99.5, 0) + 0.3
        ceil = np.percentile(Xb[:, 2], 97)
        camxy = T[:, :3, 3][:, :2]
        for c, (X, C) in M.items():
            m = X[:, 2] < ceil + 0.3
            t1 = top(X[m], C[m], lo, hi, 0.01, camxy)
            cv2.putText(t1, f"{c}: {len(X)} v", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 200), 2)
            sc = min(1.0, 1100 / t1.shape[1])
            cv2.imwrite(str(R / f"img_office_top_{c}.png"), cv2.resize(t1, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA))
    print(f"eval done {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
