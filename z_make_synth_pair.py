#!/usr/bin/env python3
"""
z_make_synth_pair.py — 정답을 아는 합성 스테레오 쌍을 만든다 (조밀/희소 경로 끝단 검증용)

  ./venv/bin/python z_make_synth_pair.py --out synth_pair

만드는 것: L.png, R.png(카메라2 = 다른 K + yaw 3° + baseline 150mm), calib_l/r.json, calib_stereo.json, gt.json
장면: 배경벽(4.0m, 사진 텍스처) / 옆판(1.1m) / 측정 대상 판(0.60x0.40m, 중심 (0.05,-0.02,1.50), 20° 기울임).
대상 판 텍스처는 shots/L.png 의 YOLO 'person' 크롭(있으면)이라 YOLO 가 검출한다. 없으면 노이즈 텍스처.
K 는 실제 캘리값(웹캠 fx 794 / 폰 초광각 fx 520)을 그대로 쓴다. 왜곡은 0 (호모그래피 렌더 한계).
"""
import argparse, json
from pathlib import Path
import cv2, numpy as np

HERE = Path(__file__).parent
SIZE = (1280, 720)
K1 = np.array([[794.31, 0, 651.0], [0, 794.70, 366.42], [0, 0, 1.]])     # 실제 웹캠 캘리
K2 = np.array([[519.74, 0, 656.66], [0, 518.47, 363.57], [0, 0, 1.]])    # 실제 폰 초광각 캘리
OBJ_C, OBJ_W, OBJ_H, OBJ_TILT = np.array([0.05, -0.02, 1.50]), 0.60, 0.40, 20.0


def tex(seed, n=(30, 45)):
    r = np.random.default_rng(seed)
    t = cv2.resize((r.random(n) * 255).astype(np.uint8), (600, 400), interpolation=cv2.INTER_CUBIC)
    return cv2.cvtColor(t, cv2.COLOR_GRAY2BGR)


def quad(c, w, h, tilt=0.0):
    th = np.radians(tilt); ex = np.array([np.cos(th), 0, np.sin(th)]); ey = np.array([0, 1., 0]); c = np.asarray(c, float)
    return np.array([c - ex*w/2 - ey*h/2, c + ex*w/2 - ey*h/2, c + ex*w/2 + ey*h/2, c - ex*w/2 + ey*h/2])


def render(planes, K, Rc, tc):
    """카메라 자세 X_cam = Rc X + tc. 먼 평면부터 그린다(페인터). 4배 슈퍼샘플 + 마스크 합성."""
    S = 4; hi = (SIZE[0]*S, SIZE[1]*S)
    img = np.full((SIZE[1], SIZE[0], 3), 25, np.uint8)
    for q, t in planes:
        p = (q @ Rc.T + tc) @ K.T; uv = (p[:, :2] / p[:, 2:3]).astype(np.float32)
        src = np.float32([[0, 0], [t.shape[1]-1, 0], [t.shape[1]-1, t.shape[0]-1], [0, t.shape[0]-1]])
        Hm = cv2.getPerspectiveTransform(src, uv * S)
        big = cv2.warpPerspective(t, Hm, hi, np.full((hi[1], hi[0], 3), 25, np.uint8), borderMode=cv2.BORDER_TRANSPARENT)
        mk = cv2.warpPerspective(np.full(t.shape[:2], 255, np.uint8), Hm, hi, np.zeros((hi[1], hi[0]), np.uint8))
        m = cv2.resize(mk, SIZE, interpolation=cv2.INTER_AREA) > 127
        img = np.where(m[..., None], cv2.resize(big, SIZE, interpolation=cv2.INTER_AREA), img)
    return img


def main(a):
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    photo = cv2.imread(str(HERE / "shots" / "L.png"))
    obj_tex, bg_tex = tex(4), tex(11)
    if photo is not None:
        bg_tex = cv2.resize(photo, (600, 400))
        try:
            import lib_detect as ld
            per = [d for d in ld.detect_yolo(photo) if d["label"] == "person"]
            if per:
                x, y, w, h = per[0]["box"]; obj_tex = cv2.resize(photo[max(0, y):y+h, max(0, x):x+w], (600, 400))
                print("  대상 텍스처: shots/L.png 의 person 크롭 (YOLO 검출 가능)")
        except Exception as e:
            print(f"  YOLO 크롭 실패({type(e).__name__}) — 노이즈 텍스처 사용")
    planes = [(quad([0, 0, 4.0], 3.2, 2.2), bg_tex), (quad([-0.55, 0.05, 1.1], 0.30, 0.30), tex(3)),
              (quad(OBJ_C, OBJ_W, OBJ_H, OBJ_TILT), obj_tex)]
    R = cv2.Rodrigues(np.array([0., np.radians(a.yaw_deg), 0.]))[0]; T = np.array([-a.baseline_mm / 1000.0, 0., 0.])
    cv2.imwrite(str(out / "L.png"), render(planes, K1, np.eye(3), np.zeros(3)))
    cv2.imwrite(str(out / "R.png"), render(planes, K2, R, T))
    for nm, K in (("calib_l.json", K1), ("calib_r.json", K2)):
        (out / nm).write_text(json.dumps(dict(K=K.tolist(), dist=[0]*5, image_size=list(SIZE), rms=0.1, per_view_err=[0.1],
                                             square_m=0.0082, n_views=20, fov_deg=[70., 45.]), indent=1))
    (out / "calib_stereo.json").write_text(json.dumps(dict(K1=K1.tolist(), dist1=[0]*5, K2=K2.tolist(), dist2=[0]*5, R=R.tolist(), T=T.tolist(),
                                                           image_size=list(SIZE), method="synthetic", scale_status="measured",
                                                           baseline_m=a.baseline_mm / 1000.0), indent=1))
    (out / "gt.json").write_text(json.dumps(dict(obj_center_cam1=OBJ_C.tolist(), obj_dist_m=float(np.linalg.norm(OBJ_C)),
                                                 obj_dims_m=[OBJ_W, OBJ_H, 0.0], obj_tilt_deg=OBJ_TILT, bg_z=4.0,
                                                 side_center=[-0.55, 0.05, 1.1], side_dims_m=[0.30, 0.30],
                                                 yaw_deg=a.yaw_deg, baseline_mm=a.baseline_mm), indent=1))
    print(f"  저장: {out}/  (L.png R.png calib_l/r.json calib_stereo.json gt.json)  대상 거리 {np.linalg.norm(OBJ_C):.3f} m")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(HERE / "synth_pair"))
    ap.add_argument("--yaw-deg", type=float, default=3.0)
    ap.add_argument("--baseline-mm", type=float, default=150.0)
    main(ap.parse_args())
