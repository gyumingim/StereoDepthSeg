#!/usr/bin/env python3
"""vioscale rulers (README 1).

  python vs_score.py views KF_DIR LENS OUT.pkl                  red-edge observations of the keyframe JPEGs (no poses), excl 0.15 thr 0.5
  python vs_score.py red VIEWS.pkl KFMAP POSES.jsonl.gz         s_VIO: measure_red estimator B on these poses (t_ns matched to KFMAP frames)
  python vs_score.py imu REC POSES.jsonl.gz IMUCAM_YAML WORKDIR  IMU ruler: imu_learn/scale_ls.py s_fixedlever on these poses
  python vs_score.py board REC POSES.jsonl.gz                   165506: k_fwd / k_rev vs imu_gt_spl (copy of scalehunt sh_score rulers)
All print one line 'RESULT <json>'.
"""
import gzip, json, pickle, re, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parents[1]
sys.path.insert(0, str(SCAN3D / "runs" / "redscreen_20261111")); sys.path.insert(0, str(SCAN3D / "imu_learn"))


def vio_poses(f):
    """t_ns -> 4x4 T_world_cam0 (tracking rows only)"""
    out = {}
    for line in gzip.open(f, "rt"):
        m = json.loads(line)
        if m.get("status") == "tracking" and m.get("T_world_cam0"):
            out[int(m["t_ns"])] = np.asarray(m["T_world_cam0"], float).reshape(4, 4)
    return out


def cmd_views(kf_dir, lens, out):
    import measure_red as M
    L = json.loads(Path(lens).read_text()); K = np.array(L["K"], float); dist = np.array(L["dist"], float)
    imgs = Path(kf_dir)
    frames = sorted(int(p.stem) for p in imgs.glob("*.jpg"))
    eye = {f: np.eye(4) for f in frames}
    V = M.views_for(frames, imgs, eye, K, dist, 0.15, 0.5, {})
    for v in V:
        v.pop("T")
    pickle.dump(dict(views=V, K=K, dist=dist, lens=lens, made=time.strftime("%F %T"), n_kf=len(frames)), open(out, "wb"))
    print("RESULT " + json.dumps(dict(n_kf=len(frames), n_views=len(V))))


def cmd_red(views_pkl, kfmap, poses):
    import measure_red as M
    D = pickle.load(open(views_pkl, "rb"))
    fr2t = {}
    for line in gzip.open(kfmap, "rt"):
        m = json.loads(line); fr2t[int(m["frame"])] = int(m["t_ns"])
    P = vio_poses(poses)
    V = []
    for v in D["views"]:
        t = fr2t.get(v["frame"])
        if t in P:
            V.append(dict(v, T=P[t]))
    fx = D["K"][0, 0]
    r = dict(poses=str(poses), n_views=len(V), n_views_all=len(D["views"]))
    if len(V) < 30:
        r["error"] = "too few views with a pose"
        print("RESULT " + json.dumps(r)); return
    A = M.est_A(V)
    B = M.est_B(V, fx, M.init_B(A))
    sA = M.summ(A["W"], A["H"]); sB = M.summ(B["W"], B["H"])
    r.update(s=round(sB["s_mean"], 5), sW=round(sB["sW"], 5), sH=round(sB["sH"], 5), W_mm=round(sB["W_mm"], 2), H_mm=round(sB["H_mm"], 2),
             rms_px=round(B["rms_px"], 3), med_abs_px=round(B["med_abs_px"], 3), sA=round(sA["s_mean"], 5), angle_A=round(A["angle_TL_deg"], 2))
    h = len(V) // 2
    for nm, Vs in (("s_first_half", V[:h]), ("s_second_half", V[h:])):
        bb = M.est_B(Vs, fx, B["x"]); r[nm] = round(M.summ(bb["W"], bb["H"])["s_mean"], 5)
    print("RESULT " + json.dumps(r))


def cmd_imu(rec, poses, imucam, work):
    """fake recording dir: imu.csv (symlink) + gt dir from the VIO poses + imu_gt/align.json (R_ci from the VIO yaml, dt from VIO)"""
    import scale_ls
    rec, work = Path(rec), Path(work)
    (work / "vgt").mkdir(parents=True, exist_ok=True); (work / "imu_gt").mkdir(exist_ok=True)
    if not (work / "imu.csv").exists():
        (work / "imu.csv").symlink_to(rec / "imu.csv")
    P = vio_poses(poses)
    dts = [json.loads(l).get("dt_cam_imu") for l in gzip.open(poses, "rt")]
    dts = [d for d in dts if d is not None]
    t = np.array(sorted(P), np.int64)
    T = np.stack([P[k] for k in t])
    p = T[:, :3, 3] - T[:, :3, 3].mean(0)          # centred: scale_ls drops |p| > 1.5 m; per-window p0 absorbs the shift
    np.savez(work / "vgt" / "gt.npz", t_ns=t, R_bc=T[:, :3, :3], p_bc=p, ok=np.ones(len(t), bool))
    y = Path(imucam).read_text()
    rows = re.findall(r"(?m)^\s*- \[([^\]]+)\]", y.split("cam_overlaps")[0])
    Tic = np.array([[float(v) for v in r.split(",")] for r in rows[:4]])     # OpenVINS T_imu_cam = T_CtoI
    R_ci = Tic[:3, :3].T                                                     # IMU -> camera
    out = {}
    for tag, dt in (("dtvio", -float(dts[-1]) if dts else 0.0), ("dt0", 0.0)):
        # OpenVINS: t_imu = t_cam + dt  ->  IMU sample on the camera clock = t_imu - dt
        json.dump(dict(dt_ns=int(round(dt * 1e9)), R_ci=R_ci.tolist(), imu_raw=True, note="vioscale: from VIO yaml/poses"),
                  open(work / "imu_gt" / "align.json", "w"))
        o = scale_ls.run(work, "vgt", 1.0, ref_s=None, raw=True)
        out[tag] = {k: o.get(k) for k in ("windows", "s", "s_ci95", "s_lever", "s_lever_ci95", "lever_fit_cm", "s_fixedlever",
                                          "s_fixedlever_ci95", "s_fixedlever_boot_std", "g_norm_fixedlever", "res_fixedlever_cm",
                                          "acc_dyn_rms_med")}
        out[tag]["dt_ms"] = round(dt * 1e3, 3)
    print("RESULT " + json.dumps(dict(poses=str(poses), **out)))


def sim3(A, B):
    ma, mb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma) / len(A))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / np.mean(np.sum((A - ma) ** 2, 1)))
    return s, R, mb - s * R @ ma


def cmd_board(rec, poses, gdir="imu_gt_spl"):
    """= scalehunt sh_score.match + rulers (k_fwd, k_rev, ATE)"""
    P = vio_poses(poses)
    vt = np.array(sorted(P), np.int64); Pp = np.stack([P[k][:3, 3] for k in vt])
    g = np.load(Path(rec) / gdir / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t, gt_p = g["t_ns"][ok], g["p_bc"][ok]
    j = np.clip(np.searchsorted(gt_t, vt), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - vt) < np.abs(gt_t[j] - vt), j - 1, j)
    mm = np.abs(gt_t[j] - vt) < 2_000_000
    A, G = Pp[mm], gt_p[j[mm]]
    s_f, R, _ = sim3(A, G); s_r, _, _ = sim3(G, A)
    Ac, Gc = A - A.mean(0), G - G.mean(0)
    U, _, Vt = np.linalg.svd(Gc.T @ Ac / len(A)); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]); R0 = U @ D @ Vt
    e = np.linalg.norm(Ac @ R0.T - Gc, axis=1)
    print("RESULT " + json.dumps(dict(poses=str(poses), gt=gdir, n=int(mm.sum()), k_fwd=round((1 / s_f - 1) * 100, 3),
                                      k_rev=round((s_r - 1) * 100, 3), ate_se3_cm=round(float(np.sqrt((e ** 2).mean())) * 100, 3))))


if __name__ == "__main__":
    c, a = sys.argv[1], sys.argv[2:]
    {"views": cmd_views, "red": cmd_red, "imu": cmd_imu, "board": cmd_board}[c](*a)
