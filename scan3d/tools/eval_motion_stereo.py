"""오프라인 평가: 두 렌즈 깊이 (움직임 보정 정류, scan3d 와 같은 경로) vs 모션 스테레오 (motion_stereo.py).

입력: 녹화 폴더 (cam2.mp4, cam5.mp4, frames_cam*.csv, imu.csv) + VIO 포즈 JSON (ovfeed.py 출력: {"poses": [pose msg ...]},
pose msg 는 ov_bridge 와 같은 키, cam0 = lens0u). replay.py·scan3d.py 는 쓰지 않는다.

  ./venv_ffs/bin/python scan3d/tools/eval_motion_stereo.py --poses /tmp/ovres_wide262.json --out /tmp/ms_eval

지표 (키프레임마다, rect0 격자 전체 화소)
- 거리 구간별 유효 화소 수: 두 렌즈 / 모션 (변형별) / 합친 결과
- 둘 다 유효한 화소의 상대 차이 (Zm−Z2)/Z2: 구간별 중앙값(치우침)·MAD
- anchor (두 렌즈로 모션 시차 맞추기) 값 c·e, 성공 비율
- 키프레임 간 일관성 (xkf): 앞 키프레임 깊이를 VIO 자세로 옮겨 같은 출처 깊이와 비교 — 두 렌즈를 정답으로 쓰지 않는 지표
- 평면 잔차: 두 깊이를 합친 점으로 RANSAC 가장 큰 평면 → 두 깊이 모두 그 평면 근처(0.15·Z)인 화소에서 깊이별로
  최소제곱 평면을 다시 맞춘 수직 잔차 RMS (두 렌즈 쪽은 0.15·Z 로 잘려 실제보다 좋게 나옴 = 보수적)
- 처리 시간
"""
import argparse
import csv
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
ROOT = SCAN3D.parent
sys.path.insert(0, str(SCAN3D))
sys.path.insert(1, str(ROOT))

import calib_scale          # noqa: E402
import frames               # noqa: E402
import motion_rect          # noqa: E402
import motion_stereo as MS  # noqa: E402

BINS = [0.3, 0.7, 1.0, 1.5, 2.0, 2.5, 3.5, 5.0]


def load_frames(rec, lens):
    fr = list(csv.DictReader(open(rec / f"frames_cam{lens}.csv")))
    t = np.array([int(r["t_mid_ns"]) for r in fr], np.int64)
    cap = cv2.VideoCapture(str(rec / f"cam{lens}.mp4"))
    ims = []
    while True:
        ok, im = cap.read()
        if not ok:
            break
        ims.append(im)
    n = min(len(ims), len(t))
    return t[:n], ims[:n]


def load_gyro(rec, gyro_type=4):
    g = []
    for r in csv.reader(open(rec / "imu.csv")):
        if r[0] == str(gyro_type):
            g.append((int(r[1]), (float(r[2]), float(r[3]), float(r[4]))))
    g.sort(key=lambda x: x[0])
    return g


def motion_of(msg, t_ns, dt_ns):
    """scan3d._motion_of 와 같은 내용 (복사: scan3d.py 를 import 하지 않으려고)."""
    mot = dict(t0_ns=int(t_ns), dt_ns=int(dt_ns), static=False)
    if msg.get("v") is not None and msg.get("q_GtoI_jpl") is not None:
        mot.update(v_G=np.asarray(msg["v"], float).reshape(3), R_GtoI=frames.jpl_quat_to_R(msg["q_GtoI_jpl"]))
    for k, n in (("bg", 3), ("R_ItoC0", 9), ("p_IinC0", 3)):
        if msg.get(k) is not None:
            mot[k] = np.asarray(msg[k], float).reshape(-1)
    return mot


def two_lens_rect(st, rp0, mot, gyro):
    """scan3d._pair_rect 와 같은 규칙 → (rp, R_fix, info) 또는 (None, None, info)."""
    dT, info = motion_rect.motion_delta(st, rp0, mot, gyro)
    if dT is None:
        return (rp0, None, dict(info, mode="fixed")) if abs(mot["dt_ns"] * 1e-6) <= motion_rect.FIXED_MAX_DT_MS else \
            (None, None, dict(info, mode="skipped"))
    if motion_rect.tiny(info):
        return rp0, None, dict(info, mode="fixed")
    ok, b = motion_rect.baseline_check(st, dT)
    if not ok:
        return None, None, dict(info, **b, mode="skipped")
    try:
        rpp = motion_rect.rectify_with_delta(st, dT)
    except ValueError as e:
        return None, None, dict(info, mode="skipped", why=str(e))
    return rpp, motion_rect.R_rect0_from(rp0, rpp), dict(info, mode="motion")


def bin_counts(Z, V):
    z = Z[V]
    return np.histogram(z, BINS)[0].tolist()


def plane_metrics(Z2, V2, Zm, Vm, rp, rng, step=4, Zf=None, Vf=None):
    """두 깊이 화소 공통 평면 영역의 잔차 RMS (m). 반환 dict 또는 None."""
    H, W = Z2.shape
    sub = np.zeros_like(V2)
    sub[::step, ::step] = True
    both = V2 & Vm & sub
    if both.sum() < 400:
        return None
    vs, us = np.nonzero(both)
    P2 = frames.rect0_points(Z2[vs, us], rp, us.astype(np.float32), vs.astype(np.float32)).astype(np.float64)
    Pm = frames.rect0_points(Zm[vs, us], rp, us.astype(np.float32), vs.astype(np.float32)).astype(np.float64)
    Pa = np.vstack([P2, Pm])
    za = Pa[:, 2]
    best, bn = None, 0
    for _ in range(300):
        i = rng.choice(len(Pa), 3, replace=False)
        n = np.cross(Pa[i[1]] - Pa[i[0]], Pa[i[2]] - Pa[i[0]])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        d = n @ Pa[i[0]]
        c = int((np.abs(Pa @ n - d) < 0.03 * za).sum())
        if c > bn:
            best, bn = (n, d), c
    if best is None:
        return None
    n, d = best
    z = 0.5 * (P2[:, 2] + Pm[:, 2])
    reg = (np.abs(P2 @ n - d) < 0.15 * z) & (np.abs(Pm @ n - d) < 0.15 * z)
    if reg.sum() < 300:
        return None

    def rms(P):
        Q = P[reg]
        c = Q.mean(0)
        _, _, vt = np.linalg.svd(Q - c, full_matrices=False)
        r = (Q - c) @ vt[2]
        return float(np.sqrt(np.mean(r ** 2)))
    res = dict(n=int(reg.sum()), z_med=float(np.median(z[reg])), rms2=rms(P2), rmsm=rms(Pm))
    if Zf is not None:
        okf = Vf[vs, us] & reg
        if okf.sum() > 100:
            Pf = frames.rect0_points(Zf[vs, us], rp, us.astype(np.float32), vs.astype(np.float32)).astype(np.float64)
            Q = Pf[okf]
            c = Q.mean(0)
            _, _, vt = np.linalg.svd(Q - c, full_matrices=False)
            res["rmsf"] = float(np.sqrt(np.mean(((Q - c) @ vt[2]) ** 2)))
            res["nf"] = int(okf.sum())
    return res


XBINS = [(0.3, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 5.0)]


def cross_kf(p, c, rp, step=4):
    """키프레임 간 일관성: 앞 키프레임 p 의 점을 VIO 자세로 지금 키프레임 c 에 투영해 같은 출처 깊이와 비교.
    공통 화소 (p 에서 세 출처 모두 유효, c 의 투영 화소에서 세 출처 모두 유효) 만 → 출처별 |상대 차이| 중앙값 (거리 구간별).
    자세 오차는 세 출처에 똑같이 들어간다."""
    H, W = p["two"][0].shape
    sub = np.zeros((H, W), bool)
    sub[::step, ::step] = True
    allp = sub & p["two"][1] & p["motion"][1] & p["fused"][1]
    if allp.sum() < 300:
        return None
    vs, us = np.nonzero(allp)
    T = np.linalg.inv(c["T"]) @ p["T"]
    out = {}
    proj = {}
    for s in ("two", "motion", "fused"):
        Z = p[s][0][vs, us]
        P = frames.rect0_points(Z, rp, us.astype(np.float32), vs.astype(np.float32))
        Q = frames.transform(T, P)
        z = Q[:, 2]
        u = np.round(rp["f"] * Q[:, 0] / z + rp["cx"]).astype(int)
        v = np.round(rp["f"] * Q[:, 1] / z + rp["cy"]).astype(int)
        proj[s] = (z, u, v)
    ok = np.ones(len(vs), bool)
    for s in proj:
        z, u, v = proj[s]
        ok &= (z > 0.1) & (u >= 0) & (u < W) & (v >= 0) & (v < H)
    for s in proj:
        z, u, v = proj[s]
        uu, vv = np.clip(u, 0, W - 1), np.clip(v, 0, H - 1)
        for s2 in ("two", "motion", "fused"):
            ok &= c[s2][1][vv, uu]
    if ok.sum() < 200:
        return None
    zref = p["two"][0][vs, us][ok]
    for s in proj:
        z, u, v = proj[s]
        Zc = c[s][0][v[ok], u[ok]]
        rel = np.abs(z[ok] - Zc) / Zc
        out[s] = {f"{lo}-{hi}": [int(((zref >= lo) & (zref < hi)).sum()),
                                 float(np.median(rel[(zref >= lo) & (zref < hi)])) if ((zref >= lo) & (zref < hi)).sum() > 50 else None]
                  for lo, hi in XBINS}
    out["base_m"] = float(np.linalg.norm(T[:3, 3]))
    return out


VARIANTS = {
    "auto": dict(),                                    # 기본: E/VIO 중 정류 검사 좋은 쪽 + affine anchor
    "scale": dict(anchor="scale"),
    "noanc": dict(anchor="none"),
    "vio": dict(pose="vio"),
    "e": dict(pose="e"),
    "pair2": dict(n_pairs=2),
    "lr": dict(lr_check=True),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rec", default=str(SCAN3D / "recordings/rec_20261016_143702"))
    ap.add_argument("--calib", default=str(ROOT / "calib_phone_pair.json"))
    ap.add_argument("--poses", default="/tmp/ovres_wide262.json")
    ap.add_argument("--out", default="/tmp/ms_eval")
    ap.add_argument("--iters", type=int, default=8)
    ap.add_argument("--kf-trans", type=float, default=0.03)
    ap.add_argument("--kf-rot", type=float, default=5.0)
    ap.add_argument("--max-kf", type=int, default=0)
    ap.add_argument("--variants", default="auto,scale,noanc,vio,pair2,lr")
    ap.add_argument("--png", type=int, default=8, help="키프레임 그림 몇 장 (첫 변형)")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rec = Path(a.rec)

    g = calib_scale.load_geometry(a.calib, ["2", "5"], (640, 480))
    st, rp0 = g["st"], g["rp"]
    assert not g["swap"]
    t2, im2 = load_frames(rec, "2")
    t5, im5 = load_frames(rec, "5")
    gyro = load_gyro(rec)
    poses = {}
    for p in json.load(open(a.poses))["poses"]:
        if p.get("type") == "pose" and p.get("init") and p.get("status") != "lost" and p.get("T_world_cam0") is not None:
            poses[int(p["t_ns"])] = frames.pose_cam0_to_rect0(p, rp0, frames.VIO_CAM0_LENS0U)
    print(f"frames {len(t2)}/{len(t5)}, gyro {len(gyro)}, tracked poses {len(poses)}", flush=True)

    import depth_seg
    import lib_ffs
    import torch
    ds = depth_seg.DepthSeg(rp0, iters=a.iters, scale=1.0, z_range=(0.3, 2.5), stride=3, no_yolo=True, log=print)
    ds.warmup()
    names = [v for v in a.variants.split(",") if v in VARIANTS]
    mss = {n: MS.MotionStereo(st, rp0, ds.model, cfg=MS.MSConfig(**VARIANTS[n]), iters=a.iters, infer=lib_ffs.infer)
           for n in names}

    rng = np.random.default_rng(0)
    prev = {n: [] for n in names}
    last_kf, rows, n_png = None, [], 0
    for k in range(len(t2)):
        tn = int(t2[k])
        msg = poses.get(tn)
        T = frames.T_world_rect0(msg, st, rp0) if msg is not None else None
        if T is None:
            continue
        want = last_kf is None or np.linalg.norm(T[:3, 3] - last_kf[:3, 3]) >= a.kf_trans or \
            frames.rot_deg(T[:3, :3], last_kf[:3, :3]) >= a.kf_rot
        j5 = int(np.argmin(np.abs(t5 - tn)))
        dt_ns = int(t5[j5] - tn)
        if want and abs(dt_ns) < 40e6 and (a.max_kf <= 0 or len(rows) < a.max_kf):
            last_kf = T
            row = dict(k=k, t=round((tn - t2[0]) * 1e-9, 3), dt_ms=round(dt_ns * 1e-6, 2))
            torch.cuda.synchronize()
            tA = time.perf_counter()
            rpp, R_fix, info = two_lens_rect(st, rp0, motion_of(msg, tn, dt_ns), gyro)
            row["two_mode"] = info.get("mode")
            r2 = ds(im2[k], im5[j5], rp=rpp, scale=1.0) if rpp is not None else None
            torch.cuda.synchronize()
            row["ms_two"] = round(1000 * (time.perf_counter() - tA), 1)
            for n in names:
                ms = mss[n]
                torch.cuda.synchronize()
                tB = time.perf_counter()
                o = ms.fuse_keyframe(tn, im2[k], T, r2=r2, rp_pair=rpp, R_fix=R_fix)
                torch.cuda.synchronize()
                inf = o["info"]
                R = dict(ms_total=round(1000 * (time.perf_counter() - tB), 1), pairs=inf.get("pairs", 0))
                Z2, V2 = o["Z2"], o["V2"]
                R["cnt_two"] = bin_counts(Z2, V2)
                R["cnt_fused"] = bin_counts(o["Z"], o["valid"])
                md = ms._last
                if md is not None:
                    Zm, Vm = md["Zm"], md["Vm"]
                    R.update({kk: inf[kk] for kk in ("B", "dt", "pose", "ffs_scale", "fuse", "anchored", "anchor")})
                    R.update(ms_ffs=round(inf["ms_ffs"], 1), ms_pose=round(inf["ms_pose"], 1),
                             roll=[round(p["roll_deg"], 1) for p in md["pairs"]],
                             perp=[round(p["perp_deg"], 1) for p in md["pairs"]],
                             rot=[round(p["rot_deg"], 2) for p in md["pairs"]],
                             gate=[p["gate"] for p in md["pairs"]], einfo=[p["e"] for p in md["pairs"]],
                             lr_frac=[p["lr_frac"] for p in md["pairs"]],
                             cnt_motion=bin_counts(Zm, Vm),
                             cnt_verified=bin_counts(Zm, Vm & md["verified"]) if ms.cfg.n_pairs > 1 else None)
                    both = V2 & Vm
                    diffs = {}
                    for lo, hi in zip(BINS[:-1], BINS[1:]):
                        m = both & (Z2 >= lo) & (Z2 < hi)
                        if m.sum() > 200:
                            rel = (Zm[m] - Z2[m]) / Z2[m]
                            diffs[f"{lo}-{hi}"] = [int(m.sum()), float(np.median(rel)),
                                                   float(np.median(np.abs(rel - np.median(rel))))]
                    R["diff"] = diffs
                    R["plane"] = plane_metrics(Z2, V2, Zm, Vm, rp0, rng, Zf=o["Z"], Vf=o["valid"])
                    cur = dict(T=T, two=(Z2, V2), motion=(Zm, Vm), fused=(o["Z"], o["valid"]))
                    R["xkf"] = [x for x in (cross_kf(p, cur, rp0) for p in prev[n]
                                            if np.linalg.norm(p["T"][:3, 3] - T[:3, 3]) >= 0.05) if x]
                    prev[n] = (prev[n] + [cur])[-4:]
                    if n == names[0] and n_png < a.png:
                        n_png += 1
                        save_png(out / f"kf_{k:03d}.png", o, Zm, Vm, md)
                row[n] = R
            rows.append(row)
            print(json.dumps(dict(k=k, t=row["t"], two=row["two_mode"],
                                  **{n: dict(p=row[n].get("pairs"), pose=row[n].get("pose"),
                                             anc=[(x.get("mode"), round(x.get("c", 0), 3), round(x.get("e", 0), 2),
                                                   x.get("mad_rel"), x.get("ok")) for x in (row[n].get("anchor") or [])],
                                             fuse=row[n].get("fuse")) for n in names}),
                             ensure_ascii=False), flush=True)
        for n in names:
            mss[n].add_frame(tn, im2[k], T)
    (out / "rows.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1, default=float))
    summarize(rows, names, out, {n: mss[n].stats for n in names})


def save_png(path, o, Zm, Vm, md):
    def col(Z, V, zmax=3.0):
        z = np.clip(np.nan_to_num(Z, nan=0) / zmax, 0, 1)
        c = cv2.applyColorMap((255 * (1 - z)).astype(np.uint8), cv2.COLORMAP_TURBO)
        c[~V] = 0
        return c
    p = md["pairs"][0]
    img = np.vstack([np.hstack([o["rectL"], col(o["Z2"], o["V2"])]), np.hstack([col(Zm, Vm), col(o["Z"], o["valid"])])])
    an = p["anchor"]
    lab = f"motion B={p['B'] * 100:.1f}cm {p['e'].get('used')} anc={an.get('mode') if an.get('ok') else 'no'}"
    for txt, xy in (("rect0", (5, 20)), ("two-lens (0-3 m)", (645, 20)), (lab, (5, 500)), ("fused", (645, 500))):
        cv2.putText(img, txt, xy, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imwrite(str(path), img)


def summarize(rows, names, out, stats=None):
    S = {"n_kf": len(rows), "two_mode": {}, "bins": [f"{lo}-{hi}" for lo, hi in zip(BINS[:-1], BINS[1:])]}
    for r in rows:
        S["two_mode"][r["two_mode"]] = S["two_mode"].get(r["two_mode"], 0) + 1
    S["ms_two_med"] = float(np.median([r["ms_two"] for r in rows]))
    for n in names:
        R = [r[n] for r in rows if n in r]
        W = [x for x in R if x.get("pairs")]
        s = dict(kf=len(R), kf_with_pair=len(W))
        if W:
            ancs = [a_ for x in W for a_ in x["anchor"]]
            s.update(B_med_cm=round(float(np.median([b for x in W for b in x["B"]])) * 100, 2),
                     dt_med_s=round(float(np.median([d for x in W for d in x["dt"]])), 3),
                     pose_E_frac=round(float(np.mean([p == "E" for x in W for p in x["pose"]])), 3),
                     roll_abs_med=round(float(np.median([abs(r) for x in W for r in x["roll"]])), 1),
                     gate_dy_med=round(float(np.median([g_["dy_med"] for x in W for g_ in x["gate"]])), 3),
                     anchored_frac=round(float(np.mean([a_.get("ok", False) for a_ in ancs])), 3),
                     anchor_affine_frac=round(float(np.mean([a_.get("mode") == "affine" for a_ in ancs if a_.get("ok")]))
                                              if any(a_.get("ok") for a_ in ancs) else 0.0, 3),
                     anchor_c_p10_50_90=[round(float(np.percentile([a_["c"] for a_ in ancs if a_.get("ok")], q)), 3)
                                         for q in (10, 50, 90)] if any(a_.get("ok") for a_ in ancs) else None,
                     anchor_e_abs_med_px=round(float(np.median([abs(a_["e"]) for a_ in ancs if a_.get("ok")])), 2)
                     if any(a_.get("ok") for a_ in ancs) else None,
                     ms_total_med=round(float(np.median([x["ms_total"] for x in W])), 1),
                     ms_ffs_med=round(float(np.median([x["ms_ffs"] for x in W])), 1),
                     ms_pose_med=round(float(np.median([x["ms_pose"] for x in W])), 1),
                     cnt_two_mean=np.mean([x["cnt_two"] for x in W], 0).round(0).tolist(),
                     cnt_motion_mean=np.mean([x["cnt_motion"] for x in W], 0).round(0).tolist(),
                     cnt_fused_mean=np.mean([x["cnt_fused"] for x in W], 0).round(0).tolist())
            if W[0].get("cnt_verified") is not None:
                s["cnt_verified_mean"] = np.mean([x["cnt_verified"] for x in W], 0).round(0).tolist()
            fz = {kk: int(np.sum([x["fuse"][kk] for x in W])) for kk in W[0]["fuse"]}
            s["fuse_sum"] = fz
            s["agree_frac"] = round(fz["agree"] / max(1, fz["both"]), 3)
            d = {}
            for b in S["bins"]:
                v = [x["diff"][b] for x in W if b in x["diff"]]
                if v:
                    d[b] = dict(kf=len(v), bias_med=round(float(np.median([q[1] for q in v])), 4),
                                mad_med=round(float(np.median([q[2] for q in v])), 4))
            s["rel_diff_motion_vs_two"] = d
            pl = [x["plane"] for x in W if x.get("plane")]
            pb = {}
            for lo, hi in XBINS:
                v = [q for q in pl if lo <= q["z_med"] < hi]
                if v:
                    pb[f"{lo}-{hi}"] = dict(n=len(v), rms_two_mm=round(1000 * float(np.median([q["rms2"] for q in v])), 1),
                                            rms_motion_mm=round(1000 * float(np.median([q["rmsm"] for q in v])), 1),
                                            rms_fused_mm=round(1000 * float(np.nanmedian([q.get("rmsf", np.nan) for q in v])), 1))
            s["plane"] = pb
            xs = [q for x in W for q in (x.get("xkf") or [])]
            xk = {}
            for lo, hi in XBINS:
                b = f"{lo}-{hi}"
                e = {src: [q[src][b][1] for q in xs if q[src][b][1] is not None] for src in ("two", "motion", "fused")}
                if e["two"]:
                    xk[b] = dict(n=len(e["two"]), **{f"{src}_med": round(float(np.median(v)), 4) for src, v in e.items() if v})
            s["xkf_relerr"] = xk
        if stats and n in stats:
            s["ms_stats"] = {kk: vv for kk, vv in stats[n].items() if kk != "rejects"}
            s["ms_stats"]["rejects_tail"] = stats[n].get("rejects", [])[-6:]
        S[n] = s
    (out / "summary.json").write_text(json.dumps(S, ensure_ascii=False, indent=1, default=float))
    print(json.dumps(S, ensure_ascii=False, indent=1, default=float))


if __name__ == "__main__":
    main()
