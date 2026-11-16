#!/usr/bin/env python3
"""rec960 2026-11-16 (B): table of VIO 1920 vs 960 runs + scanner mesh comparison.  python b_summary.py OUT

OUT/vio_{1920,960}_r{1,2}/B_o0.{json,poses.jsonl.gz} (lp_replay.py, c4f60, identical settings) and OUT/scan_{1920,960}/ (run_scanner.sh).
VIO metrics = lp_score (camera-position ATE after SE3 alignment to the board GT imu_gt_spl, scale_sim3, init_s, wall_s) + per-frame
feature counts from the pose lines (n_slam, n_msckf, h.tracked, h.used). Mesh: vertex counts, scan_summary board metrics, and the
960 mesh aligned to the 1920 mesh (Umeyama SE3 on the integrated keyframe camera centres matched by t_ns, then open3d ICP
point-to-point 2 cm) -> nearest-vertex distances + a top/front view image (OUT/mesh_diff.png).
"""
import gzip, json, sys, time
from pathlib import Path
import numpy as np

OUT = Path(sys.argv[1])


def vio_row(d):
    j = json.load(open(d / "B_o0.json"))
    s = j["score"]
    tr = [json.loads(l) for l in gzip.open(d / "B_o0.poses.jsonl.gz", "rt")]
    tr = [m for m in tr if m.get("status") == "tracking"]
    def med(f):
        v = [f(m) for m in tr]
        v = [x for x in v if x is not None]
        return float(np.median(v)) if v else None
    tm = None
    for line in open(d / "vio.log"):
        if line.startswith("TIME"):
            tm = line.strip()
    return dict(run=d.name, status=s.get("status"), crashed=j.get("crashed"), ate_vio_cm=s.get("ate_vio_cm"), ate_map_cm=s.get("ate_map_cm"),
                scale_sim3=s.get("scale_sim3"), scale_err_pct=None if s.get("scale_sim3") is None else round(abs(s["scale_sim3"] - 1) * 100, 2),
                drift_cm_per_m=s.get("drift_cm_per_m"), rot_med_deg=s.get("rot_med_deg"), init_s=s.get("init_s"),
                tracked_frac=s.get("tracked_frac"), matched=s.get("matched"), n_loop=s.get("n_loop"),
                wall_s=j.get("wall_s"), pc_ms_med=s.get("pc_ms_med"), frames_fed=j.get("frames_fed"),
                n_slam_med=med(lambda m: m.get("n_slam")), n_msckf_med=med(lambda m: m.get("n_msckf")),
                tracked_med=med(lambda m: (m.get("h") or {}).get("tracked")), used_med=med(lambda m: (m.get("h") or {}).get("used")),
                remap=j.get("input_path", {}).get("remap"), time_line=tm)


def umeyama(A, B, scale=False):  # B ~ s R A + t
    ma, mb = A.mean(0), B.mean(0)
    H = (A - ma).T @ (B - mb) / len(A)
    U, S, Vt = np.linalg.svd(H)
    D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    s = (S * np.diag(D)).sum() / ((A - ma) ** 2).sum(1).mean() if scale else 1.0
    return s, R, mb - s * R @ ma


def mesh_cmp():
    import open3d as o3d
    from scipy.spatial import cKDTree
    res = {}
    m = {}
    for r in ("1920", "960"):
        d = OUT / f"scan_{r}"
        sm = json.load(open(d / "scan_summary.json"))
        ms = sm.get("mesh") or {}
        res[r] = dict(status=sm.get("status"), total_s=sm.get("total_s"), step_s=sm.get("step_s"), kf_res=sm.get("kf_res"),
                      n_kf=ms.get("n_kf"), mesh_vertices=ms.get("mesh_vertices"), absdZ_p90_cm=ms.get("absdZ_p90_cm"),
                      kf_absdZ_p90_cm_median=ms.get("kf_absdZ_p90_cm_median"), k_pct=ms.get("k_pct"),
                      mesh_board_z_rms_cm=ms.get("mesh_board_z_rms_cm"), board_cover_frac=ms.get("board_cover_frac"),
                      sent=sm.get("sent"), ba=sm.get("ba"))
        mesh = o3d.io.read_triangle_mesh(str(d / "mesh" / "mesh.ply"))
        kp = np.load(d / "mesh" / "kf_poses.npz")
        m[r] = (np.asarray(mesh.vertices), np.asarray(mesh.vertex_colors), kp["t_ns"], kp["T"][:, :3, 3])
    V1, C1, t1, c1 = m["1920"]; V2, C2, t2, c2 = m["960"]
    common, i1, i2 = np.intersect1d(t1, t2, return_indices=True)
    how = f"{len(common)} common keyframe times"
    if len(common) < 10:                                   # different keyframe sets: nearest time within 20 ms
        j = np.searchsorted(t1, t2).clip(1, len(t1) - 1)
        j = np.where(np.abs(t1[j - 1] - t2) < np.abs(t1[j] - t2), j - 1, j)
        ok = np.abs(t1[j] - t2) < 20e6
        i1, i2 = j[ok], np.flatnonzero(ok); how = f"{ok.sum()} keyframes matched by nearest time < 20 ms"
    s_sim, _, _ = umeyama(c2[i2], c1[i1], scale=True)
    _, R, t = umeyama(c2[i2], c1[i1], scale=False)
    V2a = V2 @ R.T + t
    cam_rms = float(np.sqrt(((c2[i2] @ R.T + t - c1[i1]) ** 2).sum(1).mean()))
    p1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(V1)); p2 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(V2a))
    icp = o3d.pipelines.registration.registration_icp(p2, p1, 0.02, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPoint())
    Ti = np.asarray(icp.transformation)
    V2b = V2a @ Ti[:3, :3].T + Ti[:3, 3]
    d21 = cKDTree(V1).query(V2b)[0]; d12 = cKDTree(V2b).query(V1)[0]
    pc = lambda x, q: round(float(np.percentile(x, q)) * 100, 3)
    res["align"] = dict(match=how, cam_rms_cm_after_se3=round(cam_rms * 100, 3), cam_sim3_scale_960_to_1920=round(float(s_sim), 5),
                        icp_fitness=round(icp.fitness, 4), icp_rmse_cm=round(icp.inlier_rmse * 100, 3),
                        icp_extra_shift_cm=round(float(np.linalg.norm(Ti[:3, 3])) * 100, 3),
                        d960_to_1920_cm=dict(p50=pc(d21, 50), p90=pc(d21, 90), p99=pc(d21, 99)),
                        d1920_to_960_cm=dict(p50=pc(d12, 50), p90=pc(d12, 90), p99=pc(d12, 99)),
                        frac_1920_without_960_within_1cm=round(float((d12 > 0.01).mean()), 4),
                        frac_960_without_1920_within_1cm=round(float((d21 > 0.01).mean()), 4))
    # picture: top (x-y) and front (x-z) views, 1920 grey, 960 coloured by distance to 1920
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 3, figsize=(18, 11))
    for row, (a0, a1, nm) in enumerate(((0, 1, "top x-y"), (0, 2, "front x-z"))):
        ax[row, 0].scatter(V1[:, a0], V1[:, a1], s=0.2, c=C1 if len(C1) else "k"); ax[row, 0].set_title(f"1920 input ({len(V1)} vertices) {nm}")
        ax[row, 1].scatter(V2b[:, a0], V2b[:, a1], s=0.2, c=C2 if len(C2) else "k"); ax[row, 1].set_title(f"960 input ({len(V2)} vertices), aligned")
        ax[row, 2].scatter(V1[:, a0], V1[:, a1], s=0.2, c="0.8")
        sc = ax[row, 2].scatter(V2b[:, a0], V2b[:, a1], s=0.3, c=np.clip(d21 * 100, 0, 2), cmap="viridis")
        ax[row, 2].set_title("960 vertices: distance to 1920 mesh (cm, clipped 2)")
        plt.colorbar(sc, ax=ax[row, 2])
        for k in range(3):
            ax[row, k].set_aspect("equal"); ax[row, k].set_xlabel("xyz"[a0] + " (m)"); ax[row, k].set_ylabel("xyz"[a1] + " (m)")
    fig.suptitle(f"rec960 B: scanner mesh 1920 vs 960 input (hs_20261024_165506_std60), made {time.strftime('%F %T')}")
    fig.tight_layout(); fig.savefig(OUT / "mesh_diff.png", dpi=110)
    return res


def main():
    rows = [vio_row(d) for d in sorted(OUT.glob("vio_*_r*")) if (d / "B_o0.json").exists()]
    out = dict(made=time.strftime("%F %T"), out=str(OUT), vio=rows)
    try:
        out["make960"] = json.loads((OUT / "make960.json").read_text().strip().splitlines()[-1])
    except Exception as e:
        out["make960"] = f"missing: {e}"
    try:
        out["mesh"] = mesh_cmp()
    except Exception as e:
        out["mesh"] = f"failed: {e!r}"
    json.dump(out, open(OUT / "b_summary.json", "w"), indent=1)
    keys = ["run", "status", "ate_vio_cm", "scale_err_pct", "scale_sim3", "init_s", "tracked_frac", "n_slam_med", "n_msckf_med",
            "tracked_med", "used_med", "wall_s", "pc_ms_med", "n_loop"]
    lines = ["| " + " | ".join(keys) + " |", "|" + "---|" * len(keys)]
    for r in rows:
        lines.append("| " + " | ".join(str(r.get(k)) for k in keys) + " |")
    if isinstance(out["mesh"], dict):
        mk = ["mesh_vertices", "n_kf", "k_pct", "absdZ_p90_cm", "kf_absdZ_p90_cm_median", "mesh_board_z_rms_cm", "total_s"]
        lines += ["", "| mesh | " + " | ".join(mk) + " |", "|---|" + "---|" * len(mk)]
        for r in ("1920", "960"):
            lines.append(f"| {r} | " + " | ".join(str(out['mesh'][r].get(k)) for k in mk) + " |")
        lines += ["", "align: " + json.dumps(out["mesh"]["align"])]
    else:
        lines += ["", f"mesh: {out['mesh']}"]
    (OUT / "b_table.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
