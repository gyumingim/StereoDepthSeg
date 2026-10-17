"""aruco_scale.py 시험: 합성 (보드를 실제 녹화 프레임에 원근 합성) + 실제 보드 녹화 처리.

합성 (FFS 없이, 몇 초):
  ./venv_ffs/bin/python scan3d/tools/eval_aruco_scale.py synth
  ./venv_ffs/bin/python scan3d/tools/eval_aruco_scale.py synth --rec scan3d/recordings/rec_20261016_165645 --trials 8
  ./venv_ffs/bin/python scan3d/tools/eval_aruco_scale.py synth --ffs          # + 실제 FFS 로 합성 쌍 시차 (GPU, 느림)

  배경 = 녹화의 렌즈 2·5 쌍을 기본 정류한 rect0/rect1 (녹화가 없으면 무늬 잡음). 보드 = board json 그림을 알려진 자세로
  rect0 와 rect1 (기선 B 만큼 옮김 + 주입한 외부 파라미터 오차) 에 원근 변환으로 붙이고, 흐림·잡음·JPEG 로 망가뜨림.
  시차 모형 (FFS 없이): d_ffs = a·d_true + b + 잡음(매끈한 상관 잡음 σ_d) + 이상치 2 %.
  시험 E1 한 장 정확도 (거리·각도·잡음별), E2 누적 (여러 거리 → s, Δd), E3 열 드리프트 추적, E4 외부 파라미터 잔차,
  E5 VIO 축척 비, E6 (--ffs) 실제 FFS 로 주입 오프셋 복원.

자 없이 칸 크기 재기 (사진) 합성 시험 (CPU):
  ./venv_ffs/bin/python scan3d/tools/eval_aruco_scale.py measure-synth --trials 3

실제 녹화 (내일 보드로 찍은 것):
  ./venv_ffs/bin/python scan3d/tools/eval_aruco_scale.py rec scan3d/recordings/rec_YYYYMMDD_HHMMSS \\
      --board scan3d/config/charuco/board_A4.json [--every 2] [--vio-poses auto|PATH|none] [--no-ffs]
  - 쌍마다 기본 정류로 보드 검출 → 보이면 움직임 보정 정류 (자이로 회전 + VIO 속도, VIO 없으면 회전만) → FFS → observe.
  - VIO 포즈: 녹화 폴더 안 가장 새 recon_*/vio_poses.json (replay 결과) 을 자동으로 씀 → VIO 축척 비·보드 위치 드리프트.
  - 결과: --out (기본 /tmp/aruco_eval_<녹화>) 에 frames.jsonl (키프레임별) + status.json (최종 상태) + 화면 요약.
"""
import argparse
import csv
import json
import math
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

import aruco_scale as AS    # noqa: E402
import calib_scale          # noqa: E402

DEF_REC = SCAN3D / "recordings" / "rec_20261016_165645"
DEF_BOARD = SCAN3D / "config" / "charuco" / "board_A4.json"


# ---------------------------------------------------------------- 공통

def geometry(calib, size, pair=("2", "5")):
    return calib_scale.load_geometry(calib, list(pair), size)


def rot(yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0):
    y, p, r = (math.radians(v) for v in (yaw_deg, pitch_deg, roll_deg))
    Ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(p), -math.sin(p)], [0, math.sin(p), math.cos(p)]])
    Rz = np.array([[math.cos(r), -math.sin(r), 0], [math.sin(r), math.cos(r), 0], [0, 0, 1]])
    return Ry @ Rx @ Rz


def load_backgrounds(rec, rp, swap, n=6):
    """녹화 쌍 n 개를 기본 정류 → [(rect0, rect1)]. 녹화가 없으면 무늬 잡음."""
    out = []
    if rec and Path(rec).is_dir():
        import recording
        R = recording.Recording(rec, log=lambda m: None, pair="2,5")
        fps = R.fps or 15
        tot = R.n_frames
        step = max(1, tot // (n + 1))
        k = 0
        for pr in R.iter_pairs(round(0.55 * 1000.0 / fps, 2)):
            k += 1
            if k % step:
                continue
            fL, fR = (pr[1], pr[0]) if swap else (pr[0], pr[1])
            out.append((cv2.remap(fL.bgr, *rp["mapL"], cv2.INTER_LINEAR), cv2.remap(fR.bgr, *rp["mapR"], cv2.INTER_LINEAR)))
            if len(out) >= n:
                break
    if not out:
        W, H = rp["size"]
        rng = np.random.default_rng(0)
        for _ in range(n):
            z = cv2.GaussianBlur(rng.integers(0, 255, (H, W), np.uint8), (0, 0), 3)
            z = cv2.cvtColor(cv2.normalize(z, None, 30, 220, cv2.NORM_MINMAX), cv2.COLOR_GRAY2BGR)
            out.append((z, np.roll(z, -12, axis=1)))
    return out


class BoardRenderer:
    """보드 그림 (흰 여백 포함) 과 보드 좌표(m) → 그림 화소 호모그래피 (검출로 맞춤: OpenCV 버전별 축 규약 무관)."""

    def __init__(self, board, px_per_sq=120):
        self.board = board
        img = board.image(board.sx * px_per_sq, board.sy * px_per_sq, 0)
        pad = px_per_sq // 2
        self.tex = cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
        c, ids, _ = board.detect(self.tex)
        if len(ids) < board.n_corners:
            raise RuntimeError("보드 그림에서 모서리 검출 실패")
        H, _ = cv2.findHomography(board.obj[ids][:, :2], c, 0)
        self.H_tex_board = H                        # 보드 (x, y) m → 그림 화소
        self.px_per_sq = px_per_sq

    def render(self, bg, R, t, K, px_sq_img):
        """bg (BGR) 위에 자세 (R, t) 의 보드를 붙인 사본 + 보드 마스크."""
        H_img_board = K @ np.column_stack([R[:, 0], R[:, 1], t])
        sc = float(np.clip(1.6 * px_sq_img / self.px_per_sq, 0.05, 1.0))   # 미리 줄여 앨리어싱 줄임
        tex = self.tex if sc >= 0.999 else cv2.resize(self.tex, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
        S = np.diag([sc, sc, 1.0])
        Hm = H_img_board @ np.linalg.inv(S @ self.H_tex_board)
        Wd, Hd = bg.shape[1], bg.shape[0]
        warped = cv2.warpPerspective(tex, Hm, (Wd, Hd), flags=cv2.INTER_LINEAR, borderValue=0)
        mask = cv2.warpPerspective(np.full(tex.shape, 255, np.uint8), Hm, (Wd, Hd), flags=cv2.INTER_LINEAR, borderValue=0)
        a = (mask.astype(np.float32) / 255.0)[..., None]
        # 종이 밝기·조명 (흰 = 210, 검정 = 25 쯤)
        paper = 25 + warped.astype(np.float32) * (185.0 / 255.0)
        out = bg.astype(np.float32) * (1 - a) + paper[..., None] * a
        return out, mask > 127


def degrade(img, rng, noise, blur, jpeg_q=85):
    if blur > 0:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    img = img + rng.normal(0, noise, img.shape)
    img = np.clip(img, 0, 255).astype(np.uint8)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, jpeg_q])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def plane_disp(R, t, rp, shape):
    """보드 평면의 화소별 참 시차 f·B/Z (보드 밖도 평면 연장값)."""
    H, W = shape
    n = R[:, 2]
    dist = float(n @ t)
    u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    nr = n[0] * (u - rp["cx"]) / rp["f"] + n[1] * (v - rp["cy"]) / rp["f"] + n[2]
    Z = dist / nr
    return (rp["f"] * rp["baseline_m"] / Z).astype(np.float32), Z


def smooth_noise(rng, shape, sigma_px, corr=6.0):
    z = cv2.GaussianBlur(rng.normal(0, 1, shape).astype(np.float32), (0, 0), corr)
    z /= max(float(z.std()), 1e-6)
    return z * sigma_px


def random_pose(rng, z, angle, board, rp):
    """보드 중심이 화면 가운데 근처, 거리 z, 법선이 시선과 angle 도. → (R, t) (보드 좌표 원점 = 보드 모서리)."""
    ax = rng.uniform(0, 2 * math.pi)
    R = rot(angle * math.cos(ax), angle * math.sin(ax), rng.uniform(-25, 25))
    W, H = rp["size"]
    cu = rp["cx"] + rng.uniform(-0.15, 0.15) * W
    cv_ = rp["cy"] + rng.uniform(-0.15, 0.15) * H
    c = np.array([(cu - rp["cx"]) / rp["f"] * z, (cv_ - rp["cy"]) / rp["f"] * z, z])
    half = np.array([board.size_m[0] / 2, board.size_m[1] / 2, 0.0])
    t = c - R @ half
    return R, t


def synth_frame(rnd, bgs, rng, R, t, rp, a=1.0, b=0.0, sigma_d=0.05, noise=3.0, blur=0.7, R_err=None,
                outlier=0.02):
    """합성 키프레임 → dict(rect0, rect1, disp, d_true, Z_true, mask)."""
    K = AS.K_of(rp)
    bg0, bg1 = bgs[rng.integers(len(bgs))]
    z = float((R @ np.array([rnd.board.size_m[0] / 2, rnd.board.size_m[1] / 2, 0]) + t)[2])
    px_sq = rp["f"] * rnd.board.square_m / z
    im0, m0 = rnd.render(bg0, R, t, K, px_sq)
    Re = np.eye(3) if R_err is None else R_err
    t1 = Re @ (t - np.array([rp["baseline_m"], 0, 0]))
    im1, _ = rnd.render(bg1, Re @ R, t1, K, px_sq)
    im0, im1 = degrade(im0, rng, noise, blur), degrade(im1, rng, noise, blur)
    dtrue, Z = plane_disp(R, t, rp, m0.shape)
    disp = a * dtrue + b + smooth_noise(rng, m0.shape, sigma_d)
    bad = rng.random(m0.shape) < outlier
    disp[bad] += rng.normal(0, 3.0, int(bad.sum())).astype(np.float32)
    bgd = 8.0 + smooth_noise(rng, m0.shape, 3.0, 30)
    disp = np.where(m0, disp, bgd).astype(np.float32)
    return dict(rect0=im0, rect1=im1, disp=disp, d_true=dtrue, Z=Z, mask=m0, z=z)


# ---------------------------------------------------------------- 합성 시험

def e1_single(A, rnd, bgs, rp, trials, rng):
    """한 장 정확도. 줄마다: 거리, 각도, 잡음 → 검출률, 사용률, PnP 거리 오차 %, 한 장 오프셋 오차 (px)."""
    rows = []
    for noise in (3.0, 8.0):
        for z in (0.5, 1.0, 1.5, 2.0):
            for ang in (0, 20, 40, 55):
                det = used = 0
                zerr, oerr, dpe = [], [], []
                for _ in range(trials):
                    R, t = random_pose(rng, z, ang, rnd.board, rp)
                    b = rng.uniform(-0.3, 0.3)
                    fr = synth_frame(rnd, bgs, rng, R, t, rp, b=b, noise=noise, blur=0.6 + noise / 10)
                    A.reset()
                    o = A.observe(fr["rect0"], fr["disp"], rp, 0.0)
                    det += o["n"] >= A.cfg.min_corners
                    if "z_m" in o:
                        zc = (R @ np.array([rnd.board.size_m[0] / 2, rnd.board.size_m[1] / 2, 0]) + t)[2]
                        zerr.append(100 * (o["z_m"] - zc) / zc)
                    if o["used"]:
                        used += 1
                        oerr.append(o["offset_px"] - b)
                        dpe.append(100 * (o["offset_px"] - b) / o["d_true"])
                rows.append(dict(noise=noise, z=z, angle=ang, det=det / trials, used=used / trials,
                                 z_err_pct_med=_r(np.median(np.abs(zerr))) if zerr else None,
                                 off_err_px_med=_r(np.median(np.abs(oerr))) if oerr else None,
                                 off_err_px_max=_r(np.max(np.abs(oerr))) if oerr else None,
                                 depth_err_pct_med=_r(np.median(np.abs(dpe))) if dpe else None))
    return rows


def _r(x, n=4):
    return None if x is None else round(float(x), n)


def e2_accum(A, rnd, bgs, rp, rng, a_true=1.01, b_true=0.15, dists=(0.5, 1.0, 1.5), per=8, sigma_d=0.05):
    A.reset()
    t = 0.0
    for z in dists:
        for _ in range(per):
            R, tv = random_pose(rng, z, rng.uniform(0, 35), rnd.board, rp)
            fr = synth_frame(rnd, bgs, rng, R, tv, rp, a=a_true, b=b_true, sigma_d=sigma_d)
            A.observe(fr["rect0"], fr["disp"], rp, t)
            t += 1.0
    st = A.status(t)
    s_true, dd_true = 1 / a_true, -b_true / a_true
    fB = rp["f"] * rp["baseline_m"]
    res = dict(a_true=a_true, b_true=b_true, s_true=_r(s_true, 5), dd_true=_r(dd_true), dists=list(dists),
               mode=st["mode"], s=st["s"], dd=st["delta_d_px"], n=st["n_obs"], ready=st["ready"])
    # 보정 후 남는 깊이 오차 (0.5 / 1 / 2 m)
    for zq in (0.5, 1.0, 2.0):
        d_meas = a_true * fB / zq + b_true
        d_corr = st["s"] * d_meas + st["delta_d_px"]
        res[f"resid_pct_{zq}m"] = _r(100 * (fB / d_corr - zq) / zq, 3)
        res[f"raw_pct_{zq}m"] = _r(100 * (fB / d_meas - zq) / zq, 3)
    return res


def e3_drift(A, rnd, bgs, rp, rng, total_s=180.0, every_s=5.0, b_end=0.2, z=1.0):
    """시차 오프셋이 0 → b_end 로 선형 증가 (열). 키프레임 every_s 마다 보드 관측 → 추적 오차."""
    A.reset()
    errs = []
    t = 0.0
    while t <= total_s:
        b = b_end * t / total_s
        R, tv = random_pose(rng, z * rng.uniform(0.8, 1.2), rng.uniform(0, 30), rnd.board, rp)
        fr = synth_frame(rnd, bgs, rng, R, tv, rp, b=b)
        A.observe(fr["rect0"], fr["disp"], rp, t)
        st = A.status(t)
        if st["n_obs"] >= 3:                       # 보정 후 1 m 시차 오차 (오프셋·affine 어느 쪽이든 같은 잣대)
            d1 = rp["f"] * rp["baseline_m"]
            errs.append(abs(st["s"] * (d1 + b) + st["delta_d_px"] - d1))
        t += every_s
    return dict(total_s=total_s, every_s=every_s, b_end=b_end, tau_s=A.cfg.tau_s,
                track_err_px_med=_r(np.median(errs)), track_err_px_end=_r(errs[-1]),
                final_dd=A.status(t)["delta_d_px"])


def e4_extrinsic(A, rnd, bgs, rp, rng, yaw_mrad=0.1, pitch_mrad=0.2, roll_mrad=0.5, n=12, z=0.8):
    A.reset()
    Re = rot(math.degrees(yaw_mrad / 1000), math.degrees(pitch_mrad / 1000), math.degrees(roll_mrad / 1000))
    rows = []
    for k in range(n):
        R, tv = random_pose(rng, z * rng.uniform(0.7, 1.4), rng.uniform(0, 30), rnd.board, rp)
        fr = synth_frame(rnd, bgs, rng, R, tv, rp, R_err=Re)
        o = A.observe(fr["rect0"], fr["disp"], rp, float(k), rect1=fr["rect1"])
        if o.get("ext"):
            rows.append(o["ext"])
    st = A.status()
    ex = st.get("extrinsic", {})
    return dict(inj=dict(yaw_mrad=yaw_mrad, pitch_mrad=pitch_mrad, roll_mrad=roll_mrad,
                         corner_offset_px=_r(-rp["f"] * yaw_mrad / 1000)),
                est=ex, n_frames=len(rows),
                yaw_sd_frame=_r(np.std([r["yaw_mrad"] for r in rows])) if rows else None,
                pnp_baseline_mm_med=_r(np.median([r["pnp_baseline_mm"] for r in rows if "pnp_baseline_mm" in r]), 2)
                if rows else None, B_mm=_r(1000 * rp["baseline_m"], 2))


def e5_vio(A, rnd, bgs, rp, rng, scale=1.03, n=14, noise_m=0.002):
    """보드 고정, 카메라가 보드 앞에서 움직임. VIO 이동을 scale 배 + 잡음 → 비 복원."""
    A.reset()
    T_world_board = np.eye(4)
    T_world_board[:3, 3] = [0.3, -0.2, 1.0]
    for k in range(n):
        z = rng.uniform(0.5, 1.4)
        R, tv = random_pose(rng, z, rng.uniform(0, 35), rnd.board, rp)
        T_cb = np.eye(4)
        T_cb[:3, :3], T_cb[:3, 3] = R, tv
        T_wc = T_world_board @ np.linalg.inv(T_cb)
        T_vio = T_wc.copy()
        T_vio[:3, 3] = T_vio[:3, 3] * scale + rng.normal(0, noise_m, 3)
        fr = synth_frame(rnd, bgs, rng, R, tv, rp)
        A.observe(fr["rect0"], fr["disp"], rp, float(k), T_world_cam=T_vio)
    st = A.status()
    return dict(scale_true=scale, est=st.get("vio_scale"), board_drift_m_last=(st.get("last") or {}).get("board_drift_m"))


def e6_ffs(A, rnd, bgs, rp, rng, iters, cases):
    """실제 FFS: rect1 의 보드를 yaw 로 돌려 그 만큼 시차 오프셋 주입 → FFS 시차로 observe → 복원 오프셋."""
    import lib_ffs
    model = lib_ffs.load(valid_iters=iters, max_disp=192)
    scale = calib_scale.auto_ffs_scale(*rp["size"])
    out = []
    for z, ang, dd_inj in cases:
        A.reset()
        yaw = dd_inj / rp["f"]          # rect1 을 +yaw 로 돌리면 u1 이 −f·yaw 이동 → 시차 +dd
        Re = rot(math.degrees(-yaw), 0, 0)
        ests, corner = [], []
        for k in range(3):
            R, tv = random_pose(rng, z, ang, rnd.board, rp)
            fr = synth_frame(rnd, bgs, rng, R, tv, rp, R_err=Re, noise=2.0, blur=0.5)
            t0 = time.perf_counter()
            disp = lib_ffs.infer(model, cv2.cvtColor(fr["rect0"], cv2.COLOR_BGR2RGB),
                                 cv2.cvtColor(fr["rect1"], cv2.COLOR_BGR2RGB), valid_iters=iters, scale=scale)
            ms = 1000 * (time.perf_counter() - t0)
            o = A.observe(fr["rect0"], disp, rp, float(k), rect1=fr["rect1"])
            if o["used"]:
                ests.append(o["offset_px"])
            if o.get("ext"):
                corner.append(o["ext"]["corner_offset_px"])
        out.append(dict(z=z, angle=ang, dd_inj=dd_inj, ffs_offset_px=[_r(e) for e in ests],
                        corner_offset_px=[_r(c) for c in corner], ffs_ms=round(ms), ffs_scale=scale))
    return out


def cmd_synth(a):
    g = geometry(a.calib, (a.width, a.height))
    rp = g["rp"]
    spec = AS.load_board_spec(a.board)
    A = AS.ArucoScale(spec, AS.ScaleConfig(tau_s=a.tau))
    rnd = BoardRenderer(A.board)
    rng = np.random.default_rng(a.seed)
    t0 = time.time()
    bgs = load_backgrounds(a.rec, rp, g["swap"])
    print(f"정류 f={rp['f']:.1f} px  B={1000 * rp['baseline_m']:.2f} mm  fB={rp['f'] * rp['baseline_m']:.2f} px·m  "
          f"크기 {rp['size']}  보드 칸 {1000 * spec['square_m']:.1f} mm  배경 {len(bgs)} 쌍 ({time.time() - t0:.1f}s)")
    R = dict(geometry=dict(f=rp["f"], B_mm=1000 * rp["baseline_m"], size=list(rp["size"])), board=spec["dict"],
             square_mm=1000 * spec["square_m"])
    if "1" in a.tests:
        R["E1"] = e1_single(A, rnd, bgs, rp, a.trials, rng)
        print("\nE1 한 장 정확도 (주입 오프셋 b ∈ ±0.3 px, 시차 잡음 σ 0.05 px 상관, 이상치 2 %)")
        print(" 잡음  거리m 각도  검출  사용  PnP거리오차%  오프셋오차px(중앙/최대)  깊이환산%")
        for r in R["E1"]:
            print(f" {r['noise']:4.0f}  {r['z']:4.1f}  {r['angle']:3d}  {r['det']:4.2f}  {r['used']:4.2f}  "
                  f"{_s(r['z_err_pct_med'])}  {_s(r['off_err_px_med'])} / {_s(r['off_err_px_max'])}  "
                  f"{_s(r['depth_err_pct_med'])}")
    if "2" in a.tests:
        R["E2"] = [e2_accum(A, rnd, bgs, rp, rng),
                   e2_accum(A, rnd, bgs, rp, rng, a_true=1.0, b_true=-0.12, dists=(1.0,)),
                   e2_accum(A, rnd, bgs, rp, rng, a_true=0.99, b_true=0.10, dists=(0.5, 1.0, 1.5), sigma_d=0.15)]
        print("\nE2 누적 (d_ffs = a·d_true + b)")
        for r in R["E2"]:
            print(f" 거리 {r['dists']}  주입 s={r['s_true']} Δd={r['dd_true']}  → {r['mode']} s={r['s']} Δd={r['dd']} "
                  f"(n={r['n']}, ready={r['ready']})  보정 전/후 깊이오차 % 0.5m {r['raw_pct_0.5m']}/{r['resid_pct_0.5m']} "
                  f"1m {r['raw_pct_1.0m']}/{r['resid_pct_1.0m']} 2m {r['raw_pct_2.0m']}/{r['resid_pct_2.0m']}")
    if "3" in a.tests:
        R["E3"] = e3_drift(A, rnd, bgs, rp, rng)
        print(f"\nE3 열 드리프트 0→{R['E3']['b_end']} px / {R['E3']['total_s']:.0f} s, 키프레임 {R['E3']['every_s']} s 마다, "
              f"τ={R['E3']['tau_s']} s → 보정 후 1 m 시차 오차 중앙 {R['E3']['track_err_px_med']} px, 끝 {R['E3']['track_err_px_end']} px")
    if "4" in a.tests:
        R["E4"] = e4_extrinsic(A, rnd, bgs, rp, rng)
        print(f"\nE4 외부 파라미터 주입 {R['E4']['inj']}\n   추정 {R['E4']['est']}  (한 장 yaw 표준편차 "
              f"{R['E4']['yaw_sd_frame']} mrad, PnP 기선 {R['E4']['pnp_baseline_mm_med']} mm vs B {R['E4']['B_mm']} mm)")
    if "5" in a.tests:
        R["E5"] = e5_vio(A, rnd, bgs, rp, rng)
        print(f"\nE5 VIO 축척 주입 {R['E5']['scale_true']} → {R['E5']['est']}")
    if a.ffs:
        cases = [(0.6, 15, 0.0), (0.6, 15, 0.3), (1.0, 20, 0.0), (1.0, 20, 0.3), (1.5, 20, 0.3)]
        R["E6"] = e6_ffs(A, rnd, bgs, rp, rng, a.iters, cases)
        print("\nE6 실제 FFS (rect1 보드 yaw 로 시차 오프셋 주입)")
        for r in R["E6"]:
            print(f" 거리 {r['z']} m 각도 {r['angle']}° 주입 {r['dd_inj']} px → FFS 오프셋 {r['ffs_offset_px']}  "
                  f"모서리 오프셋 {r['corner_offset_px']}  (FFS {r['ffs_ms']} ms, scale {r['ffs_scale']})")
    if a.out:
        Path(a.out).write_text(json.dumps(R, ensure_ascii=False, indent=1, default=_json) + "\n")
        print(f"\n저장 {a.out}")


def _s(x):
    return "   -  " if x is None else f"{x:6.3f}"


def _json(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer, np.bool_)):
        return o.item()
    return str(o)


# ---------------------------------------------------------------- 칸 크기 재기 (사진) 합성 시험

def measure_scene(rng, board, k_mm_per_px, ppm=6.0, card=True):
    """탁자 (어두운 무늬) 위 A4 세로 종이 (보드 그림을 k mm/px 로 가운데 인쇄) + 옆에 카드 → 평면 그림 (회색), 원점 mm."""
    x0, y0, x1, y1 = -60.0, -50.0, 380.0, 340.0
    W, H = int((x1 - x0) * ppm), int((y1 - y0) * ppm)
    tex = cv2.GaussianBlur(rng.integers(0, 255, (H // 8 + 1, W // 8 + 1), np.uint8), (0, 0), 1.5)
    scene = cv2.resize(cv2.normalize(tex, None, 35, 95, cv2.NORM_MINMAX), (W, H), interpolation=cv2.INTER_CUBIC)

    def px(xm, ym):
        return (xm - x0) * ppm, (ym - y0) * ppm
    # 종이
    a, b = px(0, 0)
    c, d = px(210, 297)
    paper = np.full((int(round(d - b)), int(round(c - a))), 238, np.uint8)
    # 보드 그림 (사용자 png 형식: 칸 200 px, 여백 80 px) 를 k mm/px 로
    sq = 200
    img = board.image(board.sx * sq, board.sy * sq, 0)
    img = cv2.copyMakeBorder(img, 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)
    bw, bh = img.shape[1] * k_mm_per_px, img.shape[0] * k_mm_per_px
    bi = cv2.resize(img, (int(round(bw * ppm)), int(round(bh * ppm))), interpolation=cv2.INTER_AREA)
    bi = (bi.astype(np.float32) * (210 / 255.0) + 25).astype(np.uint8)
    ox, oy = int(round((210 - bw) / 2 * ppm)), int(round((297 - bh) / 2 * ppm))
    paper[oy:oy + bi.shape[0], ox:ox + bi.shape[1]] = np.minimum(paper[oy:oy + bi.shape[0], ox:ox + bi.shape[1]], bi + 13)
    scene[int(round(b)):int(round(b)) + paper.shape[0], int(round(a)):int(round(a)) + paper.shape[1]] = paper
    if card:
        a, b = px(240, 30)
        cw, ch = int(round(53.98 * ppm)), int(round(85.60 * ppm))
        cd = np.full((ch, cw), 150, np.uint8)
        cv2.rectangle(cd, (cw // 6, ch // 8), (cw // 2, ch // 4), 200, -1)    # 카드 무늬
        scene[int(round(b)):int(round(b)) + ch, int(round(a)):int(round(a)) + cw] = cd
    return scene, (x0, y0), ppm


def photo_of(scene, origin, ppm, rng, tilt_deg, dist_m=0.45, f=3000.0, size=(4000, 3000), noise=3.0, k1=0.0):
    """평면 그림 → 폰 사진 흉내 (원근, 흐림, 잡음, JPEG, 선택: 방사 왜곡 k1)."""
    W, H = size
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1.0]])
    ax = rng.uniform(0, 2 * math.pi)
    R = rot(tilt_deg * math.cos(ax), tilt_deg * math.sin(ax), rng.uniform(-10, 10))
    cen = np.array([0.150, 0.150, 0.0])
    t = np.array([rng.uniform(-0.02, 0.02), rng.uniform(-0.02, 0.02), dist_m]) - R @ cen
    S = np.array([[1 / (ppm * 1000), 0, origin[0] / 1000], [0, 1 / (ppm * 1000), origin[1] / 1000], [0, 0, 1]])
    Hm = K @ np.column_stack([R[:, 0], R[:, 1], t]) @ S
    ph = cv2.warpPerspective(scene, Hm, (W, H), flags=cv2.INTER_LINEAR, borderValue=40)
    if k1:
        u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
        x, y = (u - W / 2) / f, (v - H / 2) / f
        r2 = x * x + y * y
        ph = cv2.remap(ph, (x * (1 + k1 * r2) * f + W / 2).astype(np.float32),
                       (y * (1 + k1 * r2) * f + H / 2).astype(np.float32), cv2.INTER_LINEAR)
    ph = cv2.GaussianBlur(ph, (0, 0), 0.8)
    ph = np.clip(ph + rng.normal(0, noise, ph.shape), 0, 255).astype(np.uint8)
    ok, buf = cv2.imencode(".jpg", ph, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)


def cmd_measure_synth(a):
    spec = AS.load_board_spec(a.board)
    board = AS.Board(spec)
    rng = np.random.default_rng(a.seed)
    print(f"보드 {spec['dict']} {spec['squares_x']}x{spec['squares_y']} 마커비 {spec['marker_ratio']} · 사진 4000x3000 f=3000 · "
          f"{a.trials} 회/조건")
    print(" 기울기°  왜곡k1   기준   성공  칸오차%(중앙/최대)  가로세로차%")
    rows = []
    for tilt in (0.0, 15.0, 30.0):
        for k1 in (0.0, -0.03):
            res = {"a4": [], "card": []}
            ani = {"a4": [], "card": []}
            for _ in range(a.trials):
                k = rng.uniform(0.165, 0.176)
                true_sq = 200 * k
                sc, org, ppm = measure_scene(rng, board, k)
                ph = photo_of(sc, org, ppm, rng, tilt, k1=k1)
                for ref in ("a4", "card"):
                    r = AS.measure_square_mm(ph, board, ref)
                    if r["ok"]:
                        res[ref].append(100 * (r["square_mm"] - true_sq) / true_sq)
                        ani[ref].append(100 * abs(r["sx_mm"] / r["sy_mm"] - 1))
            for ref in ("a4", "card"):
                e = np.abs(res[ref])
                row = dict(tilt=tilt, k1=k1, ref=ref, ok=len(e) / a.trials,
                           err_pct_med=_r(np.median(e)) if len(e) else None, err_pct_max=_r(np.max(e)) if len(e) else None,
                           aniso_pct_med=_r(np.median(ani[ref])) if len(e) else None)
                rows.append(row)
                print(f" {tilt:5.0f}  {k1:6.2f}  {ref:5s}  {row['ok']:4.2f}  {_s(row['err_pct_med'])} / {_s(row['err_pct_max'])}"
                      f"  {_s(row['aniso_pct_med'])}")
    if a.out:
        Path(a.out).write_text(json.dumps(rows, ensure_ascii=False, indent=1) + "\n")


# ---------------------------------------------------------------- 실제 녹화

def load_gyro(rec, gyro_type=4):
    g = []
    for r in csv.reader(open(Path(rec) / "imu.csv")):
        if r and r[0] == str(gyro_type):
            try:
                g.append((int(r[1]), (float(r[2]), float(r[3]), float(r[4]))))
            except (ValueError, IndexError):
                pass
    g.sort(key=lambda x: x[0])
    return g


def load_vio(rec, arg):
    """vio_poses.json → (t_ns 배열, msg 목록, vio_cam0) 또는 None."""
    if arg == "none":
        return None
    p = None
    if arg == "auto":
        c = sorted(Path(rec).glob("recon_*/vio_poses.json"), key=lambda q: q.stat().st_mtime)
        p = c[-1] if c else None
    else:
        p = Path(arg)
    if not p or not p.is_file():
        return None
    j = json.loads(p.read_text())
    ent = [e[2] for e in j.get("entries", []) if e and e[0] == "p" and isinstance(e[2], dict)]
    ent = [m for m in ent if m.get("t_ns") is not None]
    ent.sort(key=lambda m: m["t_ns"])
    return dict(path=str(p), t=np.array([m["t_ns"] for m in ent], np.int64), msgs=ent,
                cam0=j.get("vio_cam0", "lens0u"), pair=j.get("pair"))


def vio_at(V, t_ns, tol_ns=20_000_000):
    if V is None or not len(V["t"]):
        return None
    i = int(np.searchsorted(V["t"], t_ns))
    best = None
    for k in (i - 1, i):
        if 0 <= k < len(V["t"]) and abs(int(V["t"][k]) - t_ns) <= tol_ns:
            if best is None or abs(int(V["t"][k]) - t_ns) < abs(int(V["t"][best]) - t_ns):
                best = k
    return None if best is None else V["msgs"][best]


def motion_of(msg, t_ns, dt_ns):
    """scan3d._motion_of 와 같은 내용 (eval_motion_stereo.motion_of 복사). msg 없으면 회전만 (static)."""
    import frames
    mot = dict(t0_ns=int(t_ns), dt_ns=int(dt_ns), static=msg is None)
    if msg is None:
        return mot
    if msg.get("v") is not None and msg.get("q_GtoI_jpl") is not None:
        mot.update(v_G=np.asarray(msg["v"], float).reshape(3), R_GtoI=frames.jpl_quat_to_R(msg["q_GtoI_jpl"]))
    else:
        mot["static"] = True
    for k, n in (("bg", 3), ("R_ItoC0", 9), ("p_IinC0", 3)):
        if msg.get(k) is not None:
            mot[k] = np.asarray(msg[k], float).reshape(-1)
    return mot


def cmd_rec(a):
    import depth_seg
    import frames
    import motion_rect
    import recording
    rec = Path(a.rec)
    Rr = recording.Recording(rec, log=lambda m: None, pair=a.pair)
    g = geometry(a.calib, Rr.size, a.pair.replace("+", ",").split(","))
    st, rp0 = g["st"], g["rp"]
    A = AS.ArucoScale(AS.load_board_spec(a.board), AS.ScaleConfig(tau_s=a.tau))
    if Rr.size[0] < 1000:
        A.cfg.max_rms_px *= Rr.size[0] / 1440.0 * 1.5
        A.cfg.min_px *= Rr.size[0] / 1440.0
    gyro = load_gyro(rec)
    V = load_vio(rec, a.vio_poses)
    print(f"녹화 {rec.name} {Rr.size} {Rr.n_frames} 프레임 · 정류 f={rp0['f']:.1f} B={1000 * rp0['baseline_m']:.2f} mm · "
          f"자이로 {len(gyro)} · VIO {V['path'] if V else '없음 (회전만 보정, VIO 축척 비 없음)'}")
    if not A.spec["measured"]:
        print("경고: board json 에 자로 잰 값이 없음 — 의도 값으로 계산 (축척 확인 의미 없음)")
    model = None
    if not a.no_ffs:
        import lib_ffs
        model = lib_ffs.load(valid_iters=a.iters, max_disp=192)
        ffs_scale = a.ffs_scale or calib_scale.auto_ffs_scale(*Rr.size)
    out = Path(a.out or f"/tmp/aruco_eval_{rec.name}")
    out.mkdir(parents=True, exist_ok=True)
    fj = open(out / "frames.jsonl", "w")
    fps = Rr.fps or 15
    k = n_seen = n_used = 0
    t_start = time.time()
    for pr in Rr.iter_pairs(round(0.55 * 1000.0 / fps, 2)):
        k += 1
        if (k - 1) % a.every:
            continue
        if a.max_frames and k > a.max_frames:
            break
        fL, fR = (pr[1], pr[0]) if g["swap"] else (pr[0], pr[1])
        t = (fL.timestamp_ns - Rr.t_first) * 1e-9
        r0 = cv2.remap(fL.bgr, *rp0["mapL"], cv2.INTER_LINEAR)
        det = A.detect(r0)
        if len(det[1]) < A.cfg.min_corners:
            continue
        n_seen += 1
        msg = vio_at(V, fL.timestamp_ns)
        if msg is not None and V["cam0"] == frames.VIO_CAM0_LENS0U:
            msg = frames.pose_cam0_to_rect0(msg, rp0, frames.VIO_CAM0_LENS0U)
        mot = motion_of(msg, fL.timestamp_ns, fR.timestamp_ns - fL.timestamp_ns)
        dT, info = motion_rect.motion_delta(st, rp0, mot, gyro)
        rp, R_fix = rp0, None
        if dT is not None and not motion_rect.tiny(info):
            ok, binfo = motion_rect.baseline_check(st, dT)
            info.update(binfo)
            if not ok:
                continue
            rp = motion_rect.rectify_with_delta(st, dT)
            R_fix = motion_rect.R_rect0_from(rp0, rp)
        elif dT is None and abs(info.get("dt_ms", 0)) > motion_rect.FIXED_MAX_DT_MS:
            info["why_fixed"] = "자이로 창 없음 → 고정 정류 (움직이면 틀림)"
        rL, rR = (r0, cv2.remap(fR.bgr, *rp0["mapR"], cv2.INTER_LINEAR)) if rp is rp0 else \
            (cv2.remap(fL.bgr, *rp["mapL"], cv2.INTER_LINEAR), cv2.remap(fR.bgr, *rp["mapR"], cv2.INTER_LINEAR))
        disp = valid = None
        if model is not None:
            import lib_ffs
            disp = lib_ffs.infer(model, cv2.cvtColor(rL, cv2.COLOR_BGR2RGB), cv2.cvtColor(rR, cv2.COLOR_BGR2RGB),
                                 valid_iters=a.iters, scale=ffs_scale)
            valid, _ = depth_seg.valid_mask(disp, rp, (0.2, 4.0))
            valid &= ~depth_seg.uncertain_mask(disp)
        T_cam = None
        if msg is not None:
            T = frames.T_world_rect0(msg, st, rp0)
            if T is not None:
                T_cam = np.array(T, float)
                if R_fix is not None:
                    T_cam[:3, :3] = T_cam[:3, :3] @ R_fix
        o = A.observe(rL, disp, rp, t, valid=valid, T_world_cam=T_cam,
                      motion=dict(rot_deg=info.get("rot_deg", 0), trans_mm=info.get("trans_mm", 0)), rect1=rR,
                      det=None if rp is not rp0 else det)
        n_used += bool(o["used"])
        row = {kk: v for kk, v in o.items() if kk != "T_cam_board"}
        row.update(frame=k, mrect=info)
        fj.write(json.dumps(row, ensure_ascii=False, default=_json) + "\n")
        e = o.get("ext") or {}
        print(f" t={t:6.2f}s n={o['n']:2d} z={o.get('z_m', 0):.3f}m 각 {o.get('angle_deg', 0):4.1f}° rms {o.get('rms_px', 0):.2f} "
              f"{'오프셋 %+.3f px (깊이 %+.2f %%)' % (o['offset_px'], o['depth_err_pct']) if o['used'] else o['why']}"
              f"{'  yaw %.3f mrad dy %+.3f px' % (e['yaw_mrad'], e['dy_px']) if e else ''}"
              f"  회전 {info.get('rot_deg', '-')}° 이동 {info.get('trans_mm', '-')} mm")
    fj.close()
    S = A.status()
    S.update(recording=str(rec), n_pairs_seen_board=n_seen, n_used=n_used, vio=V["path"] if V else None,
             elapsed_s=round(time.time() - t_start, 1))
    (out / "status.json").write_text(json.dumps(S, ensure_ascii=False, indent=1, default=_json) + "\n")
    print(f"\n보드 보인 쌍 {n_seen}, 깊이 비교에 쓴 쌍 {n_used}")
    print(f"결과: {S['mode']} s={S['s']} Δd={S['delta_d_px']} px (ready={S['ready']}, 잔차 MAD {S['resid_mad_px']} px)")
    if S.get("depth_change_pct"):
        print(f"보정이 깊이에 주는 변화 %: {S['depth_change_pct']}")
    if S.get("extrinsic"):
        print(f"외부 파라미터 잔차 (중앙값): {S['extrinsic']}")
    if S.get("vio_scale"):
        print(f"VIO 축척 (VIO 이동 / 보드 기준 이동): {S['vio_scale']}")
    if S.get("warn"):
        print("경고:", S["warn"])
    print(f"저장 {out}/frames.jsonl, {out}/status.json")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("synth")
    s.add_argument("--rec", default=str(DEF_REC), help="배경용 녹화 (없으면 무늬 잡음)")
    s.add_argument("--board", default=str(DEF_BOARD) if DEF_BOARD.is_file() else None)
    s.add_argument("--calib", default=calib_scale.default_calib("2+5"))
    s.add_argument("--width", type=int, default=1440)
    s.add_argument("--height", type=int, default=1080)
    s.add_argument("--trials", type=int, default=6)
    s.add_argument("--tests", default="12345")
    s.add_argument("--ffs", action="store_true", help="E6 실제 FFS (GPU)")
    s.add_argument("--iters", type=int, default=4)
    s.add_argument("--seed", type=int, default=1)
    s.add_argument("--tau", type=float, default=60.0)
    s.add_argument("--out", default=None, help="결과 JSON")
    m = sub.add_parser("measure-synth", help="자 없이 칸 크기 재기 (사진) 합성 시험")
    m.add_argument("--board", default=str(SCAN3D / "config" / "charuco" / "board_user_A4_6x6.json"))
    m.add_argument("--trials", type=int, default=3)
    m.add_argument("--seed", type=int, default=2)
    m.add_argument("--out", default=None)
    r = sub.add_parser("rec")
    r.add_argument("rec")
    r.add_argument("--board", default=str(DEF_BOARD) if DEF_BOARD.is_file() else None)
    r.add_argument("--calib", default=calib_scale.default_calib("2+5"))
    r.add_argument("--pair", default="2,5")
    r.add_argument("--every", type=int, default=2, help="쌍 몇 개마다 하나")
    r.add_argument("--max-frames", type=int, default=0)
    r.add_argument("--vio-poses", default="auto", help="auto | none | vio_poses.json 경로")
    r.add_argument("--no-ffs", action="store_true", help="FFS 없이 자세·외부 파라미터·VIO 축척만")
    r.add_argument("--iters", type=int, default=4)
    r.add_argument("--ffs-scale", type=float, default=None)
    r.add_argument("--tau", type=float, default=60.0)
    r.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    if a.cmd == "synth":
        cmd_synth(a)
    elif a.cmd == "measure-synth":
        cmd_measure_synth(a)
    else:
        cmd_rec(a)


if __name__ == "__main__":
    main()
