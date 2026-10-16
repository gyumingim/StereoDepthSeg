"""공장 값(Camera2 CameraCharacteristics)으로 렌즈 쌍 보정 파일 만들기 + 공장 값이 얼마나 맞는지 숫자로 보기.

왜
- 렌즈 2(초광각)·6(망원) 쌍은 체커보드 보정이 없다. 폰이 알려 주는 공장 값(intrinsics, distortion, pose_rotation,
  pose_translation, active_array — phone_stereo/cameras_s24.json)으로 calib_phone_pair.json 과 같은 형식의 파일을 만든다.
- 공장 값을 얼마나 믿을 수 있나: 렌즈 2·5 는 체커보드 보정(calib_phone_pair.json)이 있으니, 같은 방법으로 만든 2·5 공장
  보정과 비교해 R 각도 차·T 차·K 차를 숫자로 낸다 (--compare).

변환 (lib_phone_calib.factory_stereo 와 같은 식)
- K: intrinsics 는 active array 화소 기준 → 출력 W×H 로 sx = W/aw, sy = H/ah 배 (active array 가 4:3 이고 출력도 4:3 이라
  sx = sy; 폰이 4:3 스트림을 active array 전체에서 줄여 낸다는 전제 — calib_scale.py 와 같은 가정).
  렌즈 6(망원) active array 3648x2736 = 4:3, 640x480 이면 ×0.17544 → fx ≈ 1193.
- dist: Android [k1, k2, k3, p1, p2] → OpenCV [k1, k2, p1, p2, k3].
- R, T: X_B = R X_A + T,  R = R_B R_Aᵀ,  T = R_B (t_A − t_B)  (A = 왼쪽 렌즈 2, B = 오른쪽).

공장 값만으로는 부족 (2026-10-16 실측): 2·6 공장 회전 그대로 정류하면 세로 어긋남 dy 24~27 px (f 1248) → 장면 쌍으로 다듬는다.
- --refine DIR : 렌즈 2·6 쌍 (L<i>.jpg/R<i>.jpg, --grab 으로 받음) → 회전 x·z + 망원 초점 배율 (dy 로 보이는 값만).
  정류 영상끼리 SIFT (배율이 같아짐) → 정류맵으로 원영상 좌표 → lib_phone_calib.refine_pose (yaw·|T| 고정), 2번 반복.
- --yaw-ref DIR : 같은 장면을 폰을 안 움직이고 2·5 로 찍은 쌍. yaw 는 dy 로 안 보이고 시차에 f·δ 로 들어가므로,
  렌즈 2 같은 화소 광선의 거리를 2·5 (체커보드 보정, SGBM) 와 같게 맞춘다.

  ./venv_ffs/bin/python scan3d/tools/make_factory_pair_calib.py --grab phone_stereo/runs/pair26/p26   # 실시간 2+6 세션 중
  ./venv_ffs/bin/python scan3d/tools/make_factory_pair_calib.py --grab phone_stereo/runs/pair26/p25   # 같은 자리, 2+5 세션 중
  ./venv_ffs/bin/python scan3d/tools/make_factory_pair_calib.py --refine phone_stereo/runs/pair26/p26 --yaw-ref phone_stereo/runs/pair26/p25
  ./venv_ffs/bin/python scan3d/tools/make_factory_pair_calib.py                       # 공장 값만 (dy 큼, 비교용)
  ./venv_ffs/bin/python scan3d/tools/make_factory_pair_calib.py --compare-only         # 2·5 공장 vs 체커보드만 출력
"""
import argparse
import datetime as dt
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
ROOT = SCAN3D.parent
sys.path.insert(0, str(SCAN3D))
sys.path.insert(1, str(ROOT))

import calib_scale          # noqa: E402
import lib_phone_calib as pc  # noqa: E402
import lib_rectify as lr    # noqa: E402

FFS_MAX_DISP = 192          # depth_seg.py lib_ffs.load(max_disp=192)
SERVER = "http://localhost:8795"


def _np(st):
    return {k: (np.asarray(v).tolist() if isinstance(v, np.ndarray) else v) for k, v in st.items()}


def factory_pair(inv, left, right, size, logical="0"):
    """공장 값 보정 dict (np 배열). 오른쪽이 왼쪽에 있으면 ValueError (lib_phone_calib 규약: 오른쪽 T.x < 0)."""
    st = pc.factory_stereo(inv, left, right, size, logical)
    cL, cR = inv[str(left)], inv[str(right)]
    st["factory_lens"] = {str(left): dict(intrinsics=cL["intrinsics"], distortion=cL["distortion"],
                                          active_array=cL["active_array"], focal_mm=cL.get("focal_mm"),
                                          pose_translation=cL["pose_translation"], pose_rotation=cL["pose_rotation"]),
                          str(right): dict(intrinsics=cR["intrinsics"], distortion=cR["distortion"],
                                           active_array=cR["active_array"], focal_mm=cR.get("focal_mm"),
                                           pose_translation=cR["pose_translation"], pose_rotation=cR["pose_rotation"])}
    return st


def compare(board, fac):
    """체커보드 보정 vs 공장 보정 (같은 렌즈 쌍, 같은 크기) → dict 숫자."""
    Rb, Rf = np.asarray(board["R"], float), np.asarray(fac["R"], float)
    Tb, Tf = np.asarray(board["T"], float).reshape(3), np.asarray(fac["T"], float).reshape(3)
    dR = Rb @ Rf.T
    rv = np.degrees(cv2.Rodrigues(dR)[0].ravel())
    ang_t = float(np.degrees(np.arccos(np.clip(Tb @ Tf / (np.linalg.norm(Tb) * np.linalg.norm(Tf)), -1, 1))))
    out = dict(R_diff_deg=round(pc.rotation_deg(dR), 4), R_diff_xyz_deg=[round(float(v), 4) for v in rv],
               T_board_mm=[round(float(v) * 1000, 3) for v in Tb], T_factory_mm=[round(float(v) * 1000, 3) for v in Tf],
               T_diff_mm=round(float(np.linalg.norm(Tb - Tf)) * 1000, 3),
               baseline_ratio=round(float(np.linalg.norm(Tb) / np.linalg.norm(Tf)), 4), T_dir_diff_deg=round(ang_t, 3))
    for k in ("K1", "K2"):
        Kb, Kf = np.asarray(board[k], float), np.asarray(fac[k], float)
        out[k] = dict(fx_ratio=round(Kb[0, 0] / Kf[0, 0], 5), fy_ratio=round(Kb[1, 1] / Kf[1, 1], 5),
                      dcx_px=round(Kb[0, 2] - Kf[0, 2], 3), dcy_px=round(Kb[1, 2] - Kf[1, 2], 3))
    for k in ("dist1", "dist2"):
        out[k + "_same"] = bool(np.allclose(np.asarray(board[k], float), np.asarray(fac[k], float), atol=1e-9))
    # 이 R 차이가 정류 세로 어긋남(dy)으로 얼마나 되나: x·z 축 회전 성분 × f (작은 각, 영상 가운데 근사)
    out["note"] = ("체커보드 보정은 K1·dist 를 공장 값으로 고정하고 광각 fx 배율·R·T 만 맞춘 것 (board.fix_intrinsics) — "
                   "그래서 K1·dist 는 같고, 비교 의미가 있는 값은 R·T·K2 fx")
    return out


def rect_report(st, alpha=0.0):
    """정류 결과: f, 주점, 기선, 왼쪽(초광각) 원영상에서 쓰이는 영역, 오른쪽 유효 영역, 시차·깊이 표."""
    rp = calib_scale.rectify(st, alpha=alpha)
    W, H = st["image_size"]
    mx, my = rp["mapL"]
    inside = (mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)
    rx, ry = rp["mapR"]
    insideR = (rx >= 0) & (rx <= W - 1) & (ry >= 0) & (ry <= H - 1)
    f, B = float(rp["f"]), float(rp["baseline_m"])
    K1 = np.asarray(st["K1"], float)
    K2 = np.asarray(st["K2"], float)
    rows = []
    for Z in (0.3, 0.5, 1.0, 2.0, 3.0, 5.0):
        rows.append(dict(Z_m=Z, disp_px=round(f * B / Z, 2), sigma_Z_cm_at_0p5px=round(100 * Z * Z * 0.5 / (f * B), 2)))
    return rp, dict(
        alpha=alpha, f=round(f, 2), cx=round(float(rp["cx"]), 2), cy=round(float(rp["cy"]), 2),
        baseline_mm=round(B * 1000, 3), size=[int(W), int(H)],
        hfov_deg=round(float(np.degrees(2 * np.arctan(W / 2 / f))), 2),
        vfov_deg=round(float(np.degrees(2 * np.arctan(H / 2 / f))), 2),
        left_src_box=[round(float(mx.min()), 1), round(float(my.min()), 1), round(float(mx.max()), 1), round(float(my.max()), 1)],
        left_valid_frac=round(float(inside.mean()), 4), right_valid_frac=round(float(insideR.mean()), 4),
        right_src_box=[round(float(rx.min()), 1), round(float(ry.min()), 1), round(float(rx.max()), 1), round(float(ry.max()), 1)],
        zoom_left=round(f / K1[0, 0], 3), zoom_right=round(f / K2[0, 0], 3),
        ffs_max_disp=FFS_MAX_DISP, z_min_ffs_m=round(f * B / FFS_MAX_DISP, 3), depth_table=rows)


# ══════════════════════════════════════════════════════════════════════════════
# 장면 쌍으로 공장 값 다듬기 (2·6 실측: 공장 회전만으로는 정류 dy ≈ 23~27 px @ f 1248 → 쓸 수 없음)
# ══════════════════════════════════════════════════════════════════════════════
def grab_pairs(out, n=6, gap_s=1.0, server=SERVER):
    """실행 중인 scan3d 실시간 세션의 카메라 창 (/api/cam.jpg?lens=0|1, 원영상 640x480) 을 n 쌍 저장 → out/L<i>.jpg, R<i>.jpg.
    폰을 세워 둔 채로 (두 렌즈 시각차가 있어도 장면이 안 움직이면 같은 순간과 같다)."""
    import time
    import urllib.request
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for i in range(n):
        for lens, tag in ((0, "L"), (1, "R")):
            with urllib.request.urlopen(f"{server}/api/cam.jpg?lens={lens}", timeout=5) as r:
                (out / f"{tag}{i}.jpg").write_bytes(r.read())
        time.sleep(gap_s)
    print(f"저장: {out} ({n} 쌍)")


def load_frame_pairs(d):
    """폴더의 L<i>.jpg / R<i>.jpg (렌즈 0 = 2 초광각, 렌즈 1 = 오른쪽) → [(name, imL, imR)]."""
    d = Path(d)
    out = []
    for pL in sorted(d.glob("L*.jpg")):
        pR = d / ("R" + pL.name[1:])
        imL, imR = cv2.imread(str(pL)), cv2.imread(str(pR))
        if imL is not None and imR is not None:
            out.append((str(pL), imL, imR))
    if not out:
        raise FileNotFoundError(f"{d}: L<i>.jpg + R<i>.jpg 쌍이 없음 (--grab 으로 받기)")
    return out


def _bilin(m, pts):
    x = pts[:, 0].reshape(-1, 1).astype(np.float32)
    y = pts[:, 1].reshape(-1, 1).astype(np.float32)
    return cv2.remap(m, x, y, cv2.INTER_LINEAR).ravel().astype(np.float64)


def rect_matches(st, rp, imL, imR, ratio=0.75, n_max=2000):
    """정류 영상끼리 SIFT (초광각을 망원 배율로 확대한 뒤라 두 영상 배율이 같아 잘 맞는다) → 정류 좌표 qL, qR 와
    정류맵으로 되돌린 원영상 좌표 pL, pR."""
    rl, rr = lr.rectify_pair(imL, imR, rp)
    sift = cv2.SIFT_create(n_max)
    kL, dL = sift.detectAndCompute(cv2.cvtColor(rl, cv2.COLOR_BGR2GRAY), None)
    kR, dR = sift.detectAndCompute(cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY), None)
    if dL is None or dR is None or len(kL) < 2 or len(kR) < 2:
        z = np.zeros((0, 2))
        return z, z, z, z
    good = [p[0] for p in cv2.BFMatcher(cv2.NORM_L2).knnMatch(dL, dR, k=2)
            if len(p) == 2 and p[0].distance < ratio * p[1].distance]
    qL = np.array([kL[m.queryIdx].pt for m in good], np.float64).reshape(-1, 2)
    qR = np.array([kR[m.trainIdx].pt for m in good], np.float64).reshape(-1, 2)
    pL = np.column_stack([_bilin(rp["mapL"][0], qL), _bilin(rp["mapL"][1], qL)]) if len(qL) else qL
    pR = np.column_stack([_bilin(rp["mapR"][0], qR), _bilin(rp["mapR"][1], qR)]) if len(qR) else qR
    return qL, qR, pL, pR


def _norm(p, K, d):
    return cv2.undistortPoints(np.asarray(p, np.float64).reshape(-1, 1, 2), np.asarray(K, float),
                               np.asarray(d, float)).reshape(-1, 2)


def epi_rows(st, pairs):
    rp = calib_scale.rectify(st)
    rows = []
    for name, imL, imR in pairs:
        e = lr.epipolar_check(*lr.rectify_pair(imL, imR, rp))
        rows.append(dict(pair=Path(name).name, n=e["n"], dy_med=None if e["dy_med"] is None else round(e["dy_med"], 3),
                         inlier_frac=None if e["inlier_frac"] is None else round(e["inlier_frac"], 3),
                         pos_dx_frac=None if e["pos_dx_frac"] is None else round(e["pos_dx_frac"], 3)))
    return rows, float(rp["f"])


def refine_rotation(st, pairs, iters=2):
    """정류 세로 어긋남(dy)으로 보이는 값만 다듬는다: 오른쪽 렌즈 회전 x·z 축 + 오른쪽 초점 배율 (망원은 AF — 초점 1 m 에서
    공장 fx 와 다를 수 있음). yaw(y 축)·기선 길이는 dy 로 안 보여서 공장 값 그대로 (lib_phone_calib.refine_pose fix_yaw).
    정류 → SIFT → 원영상 좌표로 되돌려 Sampson 오차 최소화, 새 값으로 다시 정류해 한 번 더 (iters)."""
    st = dict(st)
    R0 = np.asarray(st["R"], float)
    infos = []
    s_tot = 1.0
    for _ in range(iters):
        rp = calib_scale.rectify(st)
        pLs, pRs = [], []
        for _, imL, imR in pairs:
            _, _, pL, pR = rect_matches(st, rp, imL, imR)
            pLs.append(pL)
            pRs.append(pR)
        pL, pR = np.vstack(pLs), np.vstack(pRs)
        if len(pL) < 40:
            raise ValueError(f"대응점 {len(pL)}개 — 질감 있는 장면을 망원 쪽에 두고 다시 찍을 것")
        nL, nR = _norm(pL, st["K1"], st["dist1"]), _norm(pR, st["K2"], st["dist2"])
        R, T, s, info = pc.refine_pose(nL, nR, np.asarray(st["R"], float), np.asarray(st["T"], float),
                                       float(rp["f"]), fit_t=False, fix_yaw=True, fit_scale=True, cauchy_px=1.0)
        st = pc.apply_scale(dict(st, R=R, T=T), s)
        s_tot *= s
        infos.append(dict(n=info["n"], f_rect=round(float(rp["f"]), 1), sampson_med_px=round(info["sampson_med_px"], 3),
                          inlier_frac_1px=round(info["inlier_frac_1px"], 3), delta_rot_deg_xyz=info["delta_rot_deg_xyz"],
                          focal_scale_right=round(s, 5)))
    dR = np.asarray(st["R"], float) @ R0.T
    total = dict(rot_change_deg=round(pc.rotation_deg(dR), 4),
                 rot_change_xyz_deg=[round(float(v), 4) for v in np.degrees(cv2.Rodrigues(dR)[0].ravel())],
                 focal_scale_right=round(s_tot, 5), iters=infos)
    st.pop("focal_scale_wide", None)
    st["focal_scale_right"] = s_tot
    return st, total


def _sgbm_ref(ref_st, ref_pairs):
    """기준 쌍 (2·5 체커보드 보정, 미터 맞음) 정류 + SGBM 시차 (쌍마다, 중앙값으로 합침) → (rp, D)."""
    rp = calib_scale.rectify(ref_st)
    sg = cv2.StereoSGBM_create(minDisparity=0, numDisparities=32, blockSize=7, P1=8 * 49, P2=32 * 49,
                               uniquenessRatio=10, speckleWindowSize=50, speckleRange=1, mode=cv2.STEREO_SGBM_MODE_HH)
    Ds = []
    for _, imL, imR in ref_pairs:
        rl, rr = lr.rectify_pair(imL, imR, rp)
        D = sg.compute(cv2.cvtColor(rl, cv2.COLOR_BGR2GRAY), cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)).astype(np.float32) / 16
        D[D <= 0.3] = np.nan
        Ds.append(D)
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)      # 모든 쌍에서 시차가 없는 화소 = NaN (정상)
        return rp, np.nanmedian(np.stack(Ds), axis=0)


def pin_yaw_from_ref(st, pairs, ref_st, ref_pairs):
    """yaw (오른쪽 렌즈 y 축 회전) 는 dy 로 안 보이고 시차에 f·δ 로 그대로 들어간다 (f 1300 이면 0.1° = 2.3 px).
    같은 장면을 (폰을 안 움직이고) 2·5 로도 찍어 두면, 렌즈 2 의 같은 화소 광선에서 2·5 거리 (체커보드 보정 = 미터 맞음)
    와 2·6 거리가 같아야 한다 → 거리 비 log 중앙값이 0 이 되는 δ 를 찾는다 (할선법)."""
    K1, d1 = np.asarray(st["K1"], float), np.asarray(st["dist1"], float)
    K2, d2 = np.asarray(st["K2"], float), np.asarray(st["dist2"], float)
    rp_ref, D = _sgbm_ref(ref_st, ref_pairs)
    rp = calib_scale.rectify(st)
    pLs, pRs = [], []
    for _, imL, imR in pairs:
        _, _, pL, pR = rect_matches(st, rp, imL, imR)
        pLs.append(pL)
        pRs.append(pR)
    pL, pR = np.vstack(pLs), np.vstack(pRs)
    # 기준 거리 (렌즈 2 광선 길이): 2·5 정류 좌표로 옮겨 5x5 중앙 시차
    Kr1, dr1 = np.asarray(ref_st["K1"], float), np.asarray(ref_st["dist1"], float)
    q = cv2.undistortPoints(pL.reshape(-1, 1, 2), Kr1, dr1, R=rp_ref["R1"], P=rp_ref["P1"]).reshape(-1, 2)
    H, W = D.shape
    ref = np.full(len(pL), np.nan)
    for i, (u, v) in enumerate(q):
        ui, vi = int(round(u)), int(round(v))
        if 2 <= ui < W - 2 and 2 <= vi < H - 2:
            w = D[vi - 2:vi + 3, ui - 2:ui + 3]
            w = w[np.isfinite(w)]
            if len(w) >= 8:
                x, y = (u - rp_ref["cx"]) / rp_ref["f"], (v - rp_ref["cy"]) / rp_ref["f"]
                ref[i] = rp_ref["f"] * rp_ref["baseline_m"] / float(np.median(w)) * np.sqrt(1 + x * x + y * y)
    cal = [dict(K=K1, dist=d1, image_size=st["image_size"]), dict(K=K2, dist=d2, image_size=st["image_size"])]
    R0, T0 = np.asarray(st["R"], float), np.asarray(st["T"], float)

    def logratio(delta):
        R = cv2.Rodrigues(np.array([0.0, delta, 0.0]))[0] @ R0
        r = lr.rectify_maps(*cal, R, T0)
        qL = cv2.undistortPoints(pL.reshape(-1, 1, 2), K1, d1, R=r["R1"], P=r["P1"]).reshape(-1, 2)
        qR = cv2.undistortPoints(pR.reshape(-1, 1, 2), K2, d2, R=r["R2"], P=r["P2"]).reshape(-1, 2)
        d = qL[:, 0] - qR[:, 0]
        ok = (np.abs(qL[:, 1] - qR[:, 1]) < 2.0) & (d > 0.5) & np.isfinite(ref)
        x, y = (qL[:, 0] - r["cx"]) / r["f"], (qL[:, 1] - r["cy"]) / r["f"]
        rng = r["f"] * r["baseline_m"] / np.maximum(d, 1e-6) * np.sqrt(1 + x * x + y * y)
        lg = np.log(rng[ok] / ref[ok])
        return float(np.median(lg)) if ok.sum() >= 20 else None, int(ok.sum()), float(np.median(rng[ok])) if ok.any() else None, lg

    g0, n0, rng0, lg0 = logratio(0.0)
    if g0 is None:
        raise ValueError(f"2·5 와 겹치는 대응점 {n0}개 — 기준 쌍(--yaw-ref)이 같은 장면인지 확인")
    x0, x1 = 0.0, np.radians(0.2)
    f0 = g0
    f1 = logratio(x1)[0]
    for _ in range(12):
        if f1 is None or abs(f1 - f0) < 1e-12:
            break
        x0, x1, f0 = x1, x1 - f1 * (x1 - x0) / (f1 - f0), f1
        f1 = logratio(x1)[0]
        if f1 is not None and abs(f1) < 1e-4:
            break
    g1, n1, rng1, lg1 = logratio(x1)
    R = cv2.Rodrigues(np.array([0.0, x1, 0.0]))[0] @ R0
    mad = float(np.median(np.abs(lg1 - np.median(lg1)))) if len(lg1) else None
    info = dict(n=n1, yaw_delta_deg=round(float(np.degrees(x1)), 4), logratio_before=round(g0, 4), logratio_after=round(g1, 4),
                range_ratio_before=round(float(np.exp(g0)), 4), range_med_m_before=round(rng0, 3),
                range_med_m_after=round(rng1, 3), logratio_mad_after=None if mad is None else round(mad, 4),
                ref_file="2·5 체커보드 보정 + SGBM", ref_valid=int(np.isfinite(ref).sum()), matches=int(len(pL)))
    return dict(st, R=R), info


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cameras", default=str(ROOT / "phone_stereo" / "cameras_s24.json"))
    ap.add_argument("--left", default="2")
    ap.add_argument("--right", default="6")
    ap.add_argument("--size", default="640x480", help="보정 크기 (4:3). scan3d 가 다른 크기로 비례 변환한다")
    ap.add_argument("--board", default=str(ROOT / "calib_phone_pair.json"), help="비교용 2·5 체커보드 보정")
    ap.add_argument("--out", default=None, help="기본 = 저장소 calib_phone_pair_<L>_<R>.json")
    ap.add_argument("--compare-only", action="store_true")
    ap.add_argument("--grab", default=None, metavar="DIR",
                    help="실행 중인 scan3d 실시간 세션에서 렌즈 0·1 원영상 n 쌍을 DIR 에 저장만 하고 끝 (폰은 세워 둔 채로)")
    ap.add_argument("--n", type=int, default=6, help="--grab 쌍 수")
    ap.add_argument("--refine", default=None, metavar="DIR",
                    help="이 쌍(L<i>.jpg/R<i>.jpg, 렌즈 2+오른쪽)으로 회전 x·z + 오른쪽 초점 배율 다듬기 (정류 dy 를 0 으로)")
    ap.add_argument("--yaw-ref", default=None, metavar="DIR",
                    help="같은 장면을 폰을 안 움직이고 2+5 로 찍은 쌍 → 2+5 체커보드 보정 거리에 맞춰 yaw 고정")
    a = ap.parse_args(argv)
    if a.grab:
        grab_pairs(a.grab, a.n)
        return 0
    size = calib_scale.check_43(*calib_scale.parse_size(a.size))
    inv = pc.load_inventory(a.cameras)

    # 1) 공장 값 신뢰도: 2·5 공장 vs 체커보드
    cmp = None
    if Path(a.board).is_file():
        board = pc.load(a.board)
        bL, bR = board["phone"]["physical"]
        board = calib_scale.scale_calib(_np(board), *size) if list(board["image_size"]) != list(size) else _np(board)
        fac_b = factory_pair(inv, bL, bR, size)
        cmp = compare(board, _np(fac_b))
        cmp.update(pair=f"{bL}+{bR}", board_file=str(a.board), board_rms_px=(board.get("board") or {}).get("rms_px"))
        f_b = float(calib_scale.rectify(board)["f"])
        cmp["dy_est_px_at_board_f"] = round(f_b * np.tan(np.radians(abs(cmp["R_diff_xyz_deg"][0]))), 2)
        print(f"[공장 vs 체커보드 {bL}+{bR} @ {size[0]}x{size[1]}]  R 차 {cmp['R_diff_deg']:.3f}° "
              f"(x {cmp['R_diff_xyz_deg'][0]:+.3f}, y {cmp['R_diff_xyz_deg'][1]:+.3f}, z {cmp['R_diff_xyz_deg'][2]:+.3f})  "
              f"T 차 {cmp['T_diff_mm']:.3f} mm (기선 비 {cmp['baseline_ratio']:.4f}, 방향 {cmp['T_dir_diff_deg']:.2f}°)  "
              f"K2 fx 비 {cmp['K2']['fx_ratio']:.4f} fy {cmp['K2']['fy_ratio']:.4f}  K1 fx 비 {cmp['K1']['fx_ratio']:.4f}  "
              f"dist 같음 {cmp['dist1_same']}/{cmp['dist2_same']}")
        print(f"    T board {cmp['T_board_mm']} mm  factory {cmp['T_factory_mm']} mm")
        print(f"    정류 세로 방향 (x축 회전 {cmp['R_diff_xyz_deg'][0]:+.3f}°) 만으로 f={f_b:.0f} 에서 dy ≈ {cmp['dy_est_px_at_board_f']} px")
    if a.compare_only:
        return 0

    # 2) 요청 쌍 공장 보정
    L, R = str(a.left), str(a.right)
    st = factory_pair(inv, L, R, size)
    cR = inv[R]
    aw, ah = pc.parse_rect(cR["active_array"])
    print(f"[렌즈 {R}] active array {aw}x{ah} (비 {aw / ah:.4f}), 출력 {size[0]}x{size[1]} 배율 {size[0] / aw:.5f} / {size[1] / ah:.5f}, "
          f"K2 fx {st['K2'][0, 0]:.2f} fy {st['K2'][1, 1]:.2f} cx {st['K2'][0, 2]:.2f} cy {st['K2'][1, 2]:.2f}, dist2 {np.round(st['dist2'], 4).tolist()}")
    print(f"[공장 {L}+{R}] T {np.round(st['T'] * 1000, 3).tolist()} mm (기선 {st['baseline_m'] * 1000:.2f} mm), "
          f"상대 회전 {st['factory_rotation_deg']:.3f}°")
    rp, rr = rect_report(_np(st))
    print(f"[정류 alpha 0] f {rr['f']} cx {rr['cx']} cy {rr['cy']} 기선 {rr['baseline_mm']} mm, 화각 {rr['hfov_deg']}x{rr['vfov_deg']}°, "
          f"초광각 원영상에서 쓰는 영역 {rr['left_src_box']} (확대 ×{rr['zoom_left']}), 망원 ×{rr['zoom_right']}, "
          f"오른쪽 유효 {rr['right_valid_frac']:.3f}, FFS 시차 192 → 최소 깊이 {rr['z_min_ffs_m']} m")
    for r in rr["depth_table"]:
        print(f"    Z {r['Z_m']:.1f} m → 시차 {r['disp_px']:7.2f} px, σZ(0.5 px) {r['sigma_Z_cm_at_0p5px']:.2f} cm")
    pose_source, scale_status = "factory:CameraCharacteristics", "factory_unverified"
    warnings = [f"렌즈 {L}·{R} 체커보드 보정 없음 — 공장 값에서 시작 (Camera2)"]
    history = [dict(pose_source=pose_source, scale_status=scale_status)]
    refine = yaw = None
    st_fac = dict(st)
    if a.refine:
        pairs = load_frame_pairs(a.refine)
        ep0, f0 = epi_rows(st, pairs)
        print(f"[공장 그대로 정류 검증 f {f0:.0f}] " + ", ".join(f"dy {r['dy_med']} (n {r['n']})" for r in ep0))
        st, refine = refine_rotation(st, pairs)
        ep1, f1 = epi_rows(st, pairs)
        refine.update(pairs_dir=str(a.refine), n_pairs=len(pairs), epi_factory=ep0, epi_refined=ep1,
                      f_rect_factory=round(f0, 1), f_rect_refined=round(f1, 1))
        print(f"[회전 다듬기] 회전 변화 {refine['rot_change_deg']}° (x,y,z {refine['rot_change_xyz_deg']}), "
              f"오른쪽 초점 배율 {refine['focal_scale_right']}, Sampson 중앙 {refine['iters'][-1]['sampson_med_px']} px")
        print(f"[다듬은 뒤 정류 검증 f {f1:.0f}] " + ", ".join(f"dy {r['dy_med']} inl {r['inlier_frac']} (n {r['n']})" for r in ep1))
        pose_source = "factory + scene_refined(rot_x,rot_z,right_focal_scale)"
        scale_status = "factory_baseline_refined"
        warnings = [f"렌즈 {L}·{R} 체커보드 보정 없음 — 공장 값 + 장면 쌍으로 회전(x·z)·망원 초점 배율만 다듬음",
                    "기선 길이 32.4 mm 는 공장 값 그대로 (검증 안 됨)"]
        history.append(dict(pose_source=pose_source, scale_status=scale_status))
        if a.yaw_ref:
            ref_pairs = load_frame_pairs(a.yaw_ref)
            ref_st = _np(pc.load(a.board))
            st, yaw = pin_yaw_from_ref(st, pairs, {k: np.asarray(v) if k in ("K1", "K2", "dist1", "dist2", "R", "T") else v
                                                   for k, v in ref_st.items()}, ref_pairs)
            yaw["ref_dir"] = str(a.yaw_ref)
            print(f"[yaw 고정 (2+5 거리 기준)] δ {yaw['yaw_delta_deg']:+.4f}°, 거리 비 2·6/2·5 {yaw['range_ratio_before']} → "
                  f"log {yaw['logratio_after']:+.4f}, 대응점 {yaw['n']}, 거리 중앙 {yaw['range_med_m_before']} → {yaw['range_med_m_after']} m, "
                  f"점별 흩어짐(MAD log) {yaw['logratio_mad_after']}")
            ep2, _ = epi_rows(st, pairs)
            yaw["epi_after"] = ep2
            print("[yaw 고정 뒤 정류 검증] " + ", ".join(f"dy {r['dy_med']} inl {r['inlier_frac']} dx>0 {r['pos_dx_frac']}" for r in ep2))
            pose_source += " + yaw_from_2_5_board_range"
            scale_status = "yaw_from_2_5"
            warnings[-1] = "기선 길이는 공장 값, yaw 는 같은 장면 2+5 (체커보드 보정) 거리에 맞춤 — 실측 거리 1개로 확인 권장"
            history.append(dict(pose_source=pose_source, scale_status=scale_status))
        else:
            warnings.append("yaw 는 공장 값 그대로 (dy 로 안 보임) → 깊이 치우침 가능. --yaw-ref 로 2+5 거리에 맞출 것")
        rp, rr = rect_report(_np(st))
        print(f"[다듬은 뒤 정류] f {rr['f']} cx {rr['cx']} cy {rr['cy']} 기선 {rr['baseline_mm']} mm, 화각 {rr['hfov_deg']}x{rr['vfov_deg']}°, "
              f"초광각 영역 {rr['left_src_box']} (×{rr['zoom_left']}), FFS 최소 깊이 {rr['z_min_ffs_m']} m")
    else:
        warnings += ["공장 회전은 렌즈 2·5 에서 체커보드와 약 " + (f"{cmp['R_diff_deg']:.2f}°" if cmp else "?")
                     + " 다름 (2·6 실측은 약 1.8°, 정류 dy 20 px 넘음) → --refine 필수", "기선 길이·yaw 는 실측 검증 전"]
    if cmp is not None:
        dR = np.asarray(st["R"], float) @ np.asarray(st_fac["R"], float).T
        cmp["this_pair_factory_vs_final_deg"] = round(pc.rotation_deg(dR), 4)
    st.update(
        pose_source=pose_source,
        scale_status=scale_status,
        created=dt.datetime.now().isoformat(timespec="seconds"),
        made_by="scan3d/tools/make_factory_pair_calib.py",
        inventory=str(a.cameras),
        rect_alpha0=rr,
        factory_check=cmp,
        scene_refine=refine,
        yaw_pin=yaw,
        factory_R=np.asarray(st_fac["R"]).tolist(),
        factory_K2=np.asarray(st_fac["K2"]).tolist(),
        warnings=warnings,
        history=history)
    out = Path(a.out) if a.out else ROOT / f"calib_phone_pair_{L}_{R}.json"
    pc.save(out, st)
    print(f"저장: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
