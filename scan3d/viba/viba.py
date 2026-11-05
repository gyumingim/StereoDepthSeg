#!/usr/bin/env python3
"""viba 2026-11-05: full-batch visual-inertial bundle adjustment (GTSAM) on the kf_ba keyframes/observations.

  ~/scanner/viba_work/venv/bin/python scan3d/viba/viba.py REC --dump BA_DIR/viba_input.npz --vio VIO.poses.jsonl.gz \
      --cfg CFG_DIR(c4f60) --out DIR [--calib yaml|online] [--init-scale 1.0] [--check]

Variables per keyframe k: X(k) = T_world_imu (Pose3), V(k) velocity (world), B(k) IMU bias (acc, gyro); landmarks L(p).
Factors: reprojection (GenericProjectionFactorCal3_S2, body_P_sensor = T_imu_cam, Huber 1 px, sigma 1 px, K = kf_ba Kr),
CombinedImuFactor between consecutive keyframes (raw uncalibrated IMU, acc interpolated to gyro times, noise from
kalibr_imu_chain.yaml, gravity from estimator_config.yaml along -z of the VIO world), gauge prior on the first keyframe pose
(pos 1 mm, rot 0.05 rad) and bias (acc 0.1, gyro 0.01). Scale free: no Sim(3) to the VIO afterwards.
Writes OUT/poses_viba.jsonl.gz (T_world_cam0 = X(k) * T_imu_cam, t_ns = frame time), OUT/viba.json.
--check: physics checks only (gravity sign at the VIO states, IMU factor residuals with the VIO states), no optimisation.
"""
import argparse, gzip, json, re, sys, time
from pathlib import Path

import numpy as np
import gtsam
from gtsam.symbol_shorthand import B, L, V, X

SCAN3D = Path(__file__).resolve().parents[1]


def read_imu(path):
    """raw uncalibrated types 35 (acc) / 16 (gyro), values 0..2 (= scan3d/imu_learn/imu_gt.read_imu(raw=True))."""
    acc, gyr = [], []
    with open(path) as f:
        for line in f:
            r = line.rstrip("\n").split(",")
            if len(r) < 5 or r[0] == "type":
                continue
            try:
                t = int(r[0])
                if t == 35:
                    acc.append([int(r[1])] + [float(x) for x in r[2:5]])
                elif t == 16:
                    gyr.append([int(r[1])] + [float(x) for x in r[2:5]])
            except (ValueError, IndexError):
                pass
    acc, gyr = np.asarray(acc, float), np.asarray(gyr, float)
    acc = acc[np.argsort(acc[:, 0], kind="stable")]; gyr = gyr[np.argsort(gyr[:, 0], kind="stable")]
    ta = acc[:, 0].astype(np.int64); tg = gyr[:, 0].astype(np.int64)
    a_i = np.stack([np.interp(tg, ta, acc[:, 1 + j]) for j in range(3)], 1)
    return tg, a_i, gyr[:, 1:4].copy()


def yaml_mat(txt, key):
    m = re.search(rf"{key}:\s*\n((?:\s*- \[.*\]\s*\n)+)", txt)
    return np.array([[float(v) for v in row.split(",")] for row in re.findall(r"\[(.*?)\]", m.group(1))])


def yaml_num(txt, key):
    return float(re.search(rf"(?m)^\s*{key}:\s*([-0-9.eE+]+)", txt).group(1))


def load_vio_states(p):
    d = {}
    for line in gzip.open(p, "rt"):
        m = json.loads(line)
        if m.get("type") == "pose" and m.get("status") == "tracking" and m.get("T_world_cam0"):
            d[int(m["t_ns"])] = m
    return d


def pose3(T):
    return gtsam.Pose3(gtsam.Rot3(T[:3, :3]), gtsam.Point3(*T[:3, 3]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec"); ap.add_argument("--dump", required=True); ap.add_argument("--vio", required=True)
    ap.add_argument("--cfg", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--calib", choices=["yaml", "online"], default="yaml")
    ap.add_argument("--init-scale", type=float, default=1.0, help="positive control: stretch the initial positions/points/velocities")
    ap.add_argument("--iters", type=int, default=100)
    ap.add_argument("--huber", type=float, default=1.0)
    ap.add_argument("--diag-damp", type=int, default=1, help="LM diagonal (Ceres-like) damping")
    ap.add_argument("--split-tracks", type=int, default=15, help="max obs per landmark (README 2.6), 0 = off")
    ap.add_argument("--ordering", choices=["schur", "colamd"], default="schur", help="elimination ordering (README 2.6)")
    ap.add_argument("--rel-tol", type=float, default=1e-5, help="LM relative error decrease to stop (README 2.4)")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--vision-only", action="store_true",
                    help="baseline in the same solver: no IMU factors, kf_ba gauge (its fixed keyframes held), Sim(3) onto the VIO centres")
    a = ap.parse_args()
    t0 = time.time()
    rec, out = Path(a.rec), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    logf = open(out / ("check.log" if a.check else "viba.log"), "w")

    def log(s):
        print(s, flush=True); logf.write(s + "\n"); logf.flush()

    D = np.load(a.dump)
    kfs, tk, Kr, Tv = D["kfs"], D["t_ns"], D["Kr"], D["Tv"]
    cam, pt, uv, X0 = D["cam"], D["pt"], D["uv"], D["X0"]
    vio_t = D["vio_t_ns"]
    n_pt_orig = len(X0)
    if a.split_tracks > 0:        # README 2.6: split long tracks into sub-tracks of <= L obs (keyframe order), same observations
        o = np.lexsort((cam, pt))
        cam, pt, uv = cam[o], pt[o], uv[o]
        starts = np.r_[0, np.flatnonzero(np.diff(pt)) + 1]
        rank = np.arange(len(pt)) - np.repeat(starts, np.diff(np.r_[starts, len(pt)]))
        sub = rank // a.split_tracks
        key = pt.astype(np.int64) * 100000 + sub
        uk, newpt = np.unique(key, return_inverse=True)
        # a trailing sub-track with 1 obs cannot constrain a point: merge it into the previous sub-track
        cnt = np.bincount(newpt)
        lone = cnt[newpt] == 1
        sub[lone & (sub > 0)] -= 1
        key = pt.astype(np.int64) * 100000 + sub
        uk, newpt = np.unique(key, return_inverse=True)
        X0 = X0[uk // 100000]; pt = newpt
    N = len(kfs)
    cfg = Path(a.cfg)
    camy = (cfg / "kalibr_imucam_chain.yaml").read_text(); imuy = (cfg / "kalibr_imu_chain.yaml").read_text()
    esty = (cfg / "estimator_config.yaml").read_text()
    T_ic = yaml_mat(camy, "T_imu_cam"); shift = yaml_num(camy, "timeshift_cam_imu")
    sa, saw = yaml_num(imuy, "accelerometer_noise_density"), yaml_num(imuy, "accelerometer_random_walk")
    sg, sgw = yaml_num(imuy, "gyroscope_noise_density"), yaml_num(imuy, "gyroscope_random_walk")
    gmag = yaml_num(esty, "gravity_mag")
    S = load_vio_states(a.vio)
    st = [S[int(t)] for t in vio_t]
    if a.calib == "online":            # last online-calibrated values of the VIO run
        last = S[max(S)]
        R_CI = np.array(last["R_ItoC0"]).reshape(3, 3); p_IinC = np.array(last["p_IinC0"])
        T_ci = np.eye(4); T_ci[:3, :3] = R_CI; T_ci[:3, 3] = p_IinC
        T_ic = np.linalg.inv(T_ci); shift = float(last["dt_cam_imu"])
    calib = dict(mode=a.calib, T_imu_cam=T_ic.tolist(), timeshift_s=shift, acc_nd=sa, acc_rw=saw, gyr_nd=sg, gyr_rw=sgw, gravity=gmag)
    log(f"[in] {N} kf, {len(X0)} points, {len(cam)} obs; calib {a.calib}: shift {shift * 1e3:.3f} ms p_CinI {np.round(T_ic[:3, 3], 4).tolist()}, "
        f"noise a {sa:g}/{saw:g} g {sg:g}/{sgw:g}, g {gmag}")
    tg, acc, gyr = read_imu(rec / "imu.csv")
    ti = tk + int(round(shift * 1e9))                       # keyframe times on the IMU clock
    T_ci = np.linalg.inv(T_ic)
    Twi = np.array([T @ T_ci for T in Tv])                   # init IMU poses (camera poses = VIO keyframe poses)
    vel = np.array([s["v"] for s in st], float)
    ba0 = np.array([s["ba"] for s in st], float); bg0 = np.array([s["bg"] for s in st], float)
    Xl = X0.copy()
    if a.init_scale != 1.0:
        c0 = Tv[0][:3, 3]
        Twi[:, :3, 3] = c0 + a.init_scale * (Twi[:, :3, 3] - c0)
        Xl = c0 + a.init_scale * (Xl - c0); vel = vel * a.init_scale
    # ---- preintegration
    prm = gtsam.PreintegrationCombinedParams.MakeSharedU(gmag)
    I3 = np.eye(3)
    prm.setAccelerometerCovariance(I3 * sa ** 2); prm.setGyroscopeCovariance(I3 * sg ** 2)
    prm.setIntegrationCovariance(I3 * 1e-8); prm.setBiasAccCovariance(I3 * saw ** 2); prm.setBiasOmegaCovariance(I3 * sgw ** 2)
    if hasattr(prm, "setBiasAccOmegaInit"):
        prm.setBiasAccOmegaInit(np.eye(6) * 1e-5)
    elif hasattr(prm, "setBiasAccOmegaInt"):
        prm.setBiasAccOmegaInt(np.eye(6) * 1e-5)

    def preint(k, bias):
        pim = gtsam.PreintegratedCombinedMeasurements(prm, bias)
        t_a, t_b = ti[k], ti[k + 1]
        j0 = max(int(np.searchsorted(tg, t_a, "right")) - 1, 0)
        j = j0; n = 0
        while j < len(tg) and tg[j] < t_b:
            s0 = max(tg[j], t_a); s1 = min(tg[j + 1] if j + 1 < len(tg) else t_b, t_b)
            dt = (s1 - s0) * 1e-9
            if dt > 0:
                pim.integrateMeasurement(acc[j], gyr[j], dt); n += 1
            j += 1
        return pim, n

    gaps = np.diff(ti) * 1e-9
    log(f"[imu] {len(tg)} gyro rows {tg[0]}..{tg[-1]}, kf IMU span {ti[0]}..{ti[-1]}, kf gap median {np.median(gaps) * 1e3:.0f} ms max {gaps.max():.2f} s")
    # ---- physics check: gravity sign + IMU factor residuals at the VIO states
    Ri = Twi[:, :3, :3]
    jk = np.searchsorted(tg, ti).clip(0, len(tg) - 1)
    f_meas = acc[jk] - ba0
    f_pred_up = np.einsum("nji,j->ni", Ri, np.array([0, 0, gmag]))   # R^T (0,0,+g): world z up
    lin = np.array([np.linalg.norm(vel[min(k + 1, N - 1)] - vel[max(k - 1, 0)]) / max((ti[min(k + 1, N - 1)] - ti[max(k - 1, 0)]) * 1e-9, 1e-3) for k in range(N)])
    calm = lin < np.percentile(lin, 20)
    e_up = np.linalg.norm(f_meas - f_pred_up, axis=1); e_dn = np.linalg.norm(f_meas + f_pred_up, axis=1)
    chk = dict(acc_norm_med=float(np.median(np.linalg.norm(acc, axis=1))), grav_err_zup_med=float(np.median(e_up[calm])),
               grav_err_zdown_med=float(np.median(e_dn[calm])))
    rp, rv, rr = [], [], []
    for k in range(N - 1):
        bias = gtsam.imuBias.ConstantBias(ba0[k], bg0[k])
        pim, n = preint(k, bias)
        if n == 0:
            continue
        ns = pim.predict(gtsam.NavState(gtsam.Rot3(Ri[k]), Twi[k, :3, 3], vel[k]), bias)
        rp.append(np.linalg.norm(ns.position() - Twi[k + 1, :3, 3])); rv.append(np.linalg.norm(ns.velocity() - vel[k + 1]))
        dR = ns.attitude().matrix().T @ Ri[k + 1]
        rr.append(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
    chk.update(imu_res_pos_mm_med=float(np.median(rp) * 1e3), imu_res_pos_mm_p90=float(np.percentile(rp, 90) * 1e3),
               imu_res_vel_mms_med=float(np.median(rv) * 1e3), imu_res_rot_deg_med=float(np.median(rr)))
    log("[check] " + json.dumps({k: round(v, 4) for k, v in chk.items()}))
    if a.check:
        (out / "check.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), calib=calib, init_scale=a.init_scale, **chk), indent=1))
        return
    # ---- graph
    graph = gtsam.NonlinearFactorGraph(); vals = gtsam.Values()
    K = gtsam.Cal3_S2(Kr[0, 0], Kr[1, 1], 0.0, Kr[0, 2], Kr[1, 2])
    bPs = pose3(T_ic)
    rob = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(a.huber), gtsam.noiseModel.Isotropic.Sigma(2, 1.0))
    for k in range(N):
        vals.insert(X(k), pose3(Twi[k])); vals.insert(V(k), vel[k]); vals.insert(B(k), gtsam.imuBias.ConstantBias(ba0[k], bg0[k]))
    for p in range(len(Xl)):
        vals.insert(L(p), gtsam.Point3(*Xl[p]))
    for c, p_, q in zip(cam, pt, uv):
        graph.add(gtsam.GenericProjectionFactorCal3_S2(gtsam.Point2(*q), rob, X(int(c)), L(int(p_)), K, False, False, bPs))
    n_imu = 0
    fixed = [int(f) for f in D["fixed"]] if "fixed" in D.files else [0]
    for k in range(N - 1 if not a.vision_only else 0):
        pim, n = preint(k, gtsam.imuBias.ConstantBias(ba0[k], bg0[k]))
        if n == 0:
            graph.add(gtsam.BetweenFactorConstantBias(B(k), B(k + 1), gtsam.imuBias.ConstantBias(), gtsam.noiseModel.Isotropic.Sigma(6, 1e-3)))
            continue
        graph.add(gtsam.CombinedImuFactor(X(k), V(k), X(k + 1), V(k + 1), B(k), B(k + 1), pim)); n_imu += 1
    if a.vision_only:      # = kf_ba: fixed[0] (gauge) and keyframes with < 15 obs held at the VIO pose; V/B unused
        for k in fixed:
            graph.add(gtsam.PriorFactorPose3(X(k), pose3(Twi[k]), gtsam.noiseModel.Isotropic.Sigma(6, 1e-7)))
        for k in range(N):
            graph.add(gtsam.PriorFactorVector(V(k), vel[k], gtsam.noiseModel.Isotropic.Sigma(3, 1.0)))
            graph.add(gtsam.PriorFactorConstantBias(B(k), gtsam.imuBias.ConstantBias(ba0[k], bg0[k]), gtsam.noiseModel.Isotropic.Sigma(6, 1.0)))
    else:
        graph.add(gtsam.PriorFactorPose3(X(0), pose3(Twi[0]), gtsam.noiseModel.Diagonal.Sigmas(np.r_[[0.05] * 3, [0.001] * 3])))
        graph.add(gtsam.PriorFactorConstantBias(B(0), gtsam.imuBias.ConstantBias(ba0[0], bg0[0]),
                                                gtsam.noiseModel.Diagonal.Sigmas(np.r_[[0.1] * 3, [0.01] * 3])))
    log(f"[graph] factors {graph.size()} (imu {n_imu}), values {vals.size()} {time.time() - t0:.0f}s")

    def reproj(vs):
        Rw = np.array([vs.atPose3(X(k)).matrix() @ T_ic for k in range(N)])
        P = np.array([vs.atPoint3(L(p)) for p in range(len(Xl))])
        Rcw = np.transpose(Rw[:, :3, :3], (0, 2, 1)); tcw = -np.einsum("nij,nj->ni", Rcw, Rw[:, :3, 3])
        Xc = np.einsum("nij,nj->ni", Rcw[cam], P[pt]) + tcw[cam]
        q = np.stack([Kr[0, 0] * Xc[:, 0] / Xc[:, 2] + Kr[0, 2], Kr[1, 1] * Xc[:, 1] / Xc[:, 2] + Kr[1, 2]], 1)
        e = np.linalg.norm(q - uv, axis=1)
        return Rw, P, e

    _, _, e0 = reproj(vals)
    err0 = graph.error(vals)
    lp = gtsam.LevenbergMarquardtParams(); lp.setMaxIterations(a.iters); lp.setRelativeErrorTol(a.rel_tol); lp.setAbsoluteErrorTol(1e-9)
    if a.diag_damp:
        lp.setDiagonalDamping(True)
    if a.ordering == "schur":     # landmarks first, then V/B, then poses (Schur-like; COLAMD gave 25 GB fill-in on 1e6 obs)
        od = gtsam.Ordering()
        for p_ in range(len(Xl)):
            od.push_back(L(p_))
        for k in range(N):
            od.push_back(V(k)); od.push_back(B(k))
        for k in range(N):
            od.push_back(X(k))
        lp.setOrdering(od)
    opt = gtsam.LevenbergMarquardtOptimizer(graph, vals, lp)
    res = opt.optimize()
    err1 = graph.error(res)
    Rw, P, e1 = reproj(res)
    log(f"[lm] iters {opt.iterations()} error {err0:.1f} -> {err1:.1f}, reproj med {np.median(e0):.3f} -> {np.median(e1):.3f} px {time.time() - t0:.0f}s")
    # IMU residual (whitened error) of the result, bias summary
    imu_err = [graph.at(i).error(res) for i in range(graph.size()) if isinstance(graph.at(i), gtsam.CombinedImuFactor)]
    bas = np.array([res.atConstantBias(B(k)).accelerometer() for k in range(N)])
    bgs = np.array([res.atConstantBias(B(k)).gyroscope() for k in range(N)])
    sim3 = None
    if a.vision_only:      # Sim(3) of the BA centres onto the VIO centres (free keyframes + gauge kf), fixed kf keep the VIO pose
        fr = np.array([k not in set(fixed) or k == fixed[0] for k in range(N)])
        Cb, Cv0 = Rw[:, :3, 3], Tv[:, :3, 3]
        ma, mb = Cb[fr].mean(0), Cv0[fr].mean(0)
        U_, S_, Vt_ = np.linalg.svd((Cv0[fr] - mb).T @ (Cb[fr] - ma)); D_ = np.diag([1, 1, np.sign(np.linalg.det(U_ @ Vt_))])
        Ra = U_ @ D_ @ Vt_; sc = float(np.trace(np.diag(S_) @ D_) / ((Cb[fr] - ma) ** 2).sum()); ta = mb - sc * Ra @ ma
        Rn = Rw.copy()
        for k in range(N):
            if fr[k]:
                Rn[k, :3, :3] = Ra @ Rw[k, :3, :3]; Rn[k, :3, 3] = sc * Ra @ Rw[k, :3, 3] + ta
            else:
                Rn[k] = Tv[k]
        res_ = np.linalg.norm(Rn[fr, :3, 3] - Cv0[fr], axis=1)
        sim3 = dict(scale=sc, rms_cm=float(np.sqrt((res_ ** 2).mean()) * 100), n_fixed=len(fixed))
        log(f"[sim3] scale {sc:.5f}, centre rms {sim3['rms_cm']:.2f} cm")
        Rw = Rn
    with gzip.open(out / "poses_viba.jsonl.gz", "wt") as f:
        for k in range(N):
            f.write(json.dumps(dict(type="pose", status="tracking", t_ns=int(tk[k]), frame=int(kfs[k]), T_world_cam0=Rw[k].ravel().tolist(),
                                    v=res.atVector(V(k)).tolist(), ba=bas[k].tolist(), bg=bgs[k].tolist())) + "\n")
    # how far the camera centres moved vs the VIO init (scale of the move)
    Cv = Tv[:, :3, 3]; Cn = Rw[:, :3, 3]
    a_, b_ = Cv - Cv.mean(0), Cn - Cn.mean(0)
    s_rel = float(np.sqrt((b_ ** 2).sum() / (a_ ** 2).sum()))
    info = dict(made=time.strftime("%F %T"), rec=str(rec), dump=str(Path(a.dump).resolve()), vio=str(Path(a.vio).resolve()),
                cfg=str(cfg.resolve()), args=vars(a), calib=calib, check=chk, n_kf=int(N), points=int(len(Xl)), points_orig=int(n_pt_orig), obs=int(len(cam)),
                imu_factors=n_imu, kf_gap_max_s=float(gaps.max()), lm_iters=int(opt.iterations()), error0=float(err0), error1=float(err1),
                reproj_med_px0=float(np.median(e0)), reproj_med_px=float(np.median(e1)), reproj_rmse_px=float(np.sqrt((e1 ** 2).mean())),
                frac_inlier_1px=float((e1 <= 1).mean()), imu_err_med=float(np.median(imu_err)) if imu_err else None,
                imu_err_p90=float(np.percentile(imu_err, 90)) if imu_err else None,
                ba_med=np.median(bas, 0).tolist(), bg_med=np.median(bgs, 0).tolist(), size_ratio_vs_vio=s_rel,
                sim3=sim3, gtsam="4.2 (pip)", runtime_s=round(time.time() - t0, 1))
    (out / "viba.json").write_text(json.dumps(info, indent=1))
    log(json.dumps({k: v for k, v in info.items() if k not in ("calib", "args")}))


if __name__ == "__main__":
    main()
