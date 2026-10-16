"""OpenVINS 설정 3개 생성 (estimator_config.yaml, kalibr_imu_chain.yaml, kalibr_imucam_chain.yaml).

입력: calib_phone_pair.json (스테레오 보정) + phone_stereo/cameras_s24.json (Camera2 렌즈 자세).
VIO 에 넣는 영상 (W×H gray, 기본 보정 크기 640x480, --size 로 다른 4:3 크기 — K 비례 변환):
  - stereo: **정류된** rect0 (왼쪽 렌즈) / rect1 (오른쪽 렌즈) (SPEC 9.4) -> intrinsics = 정류 P1 의 [f, f, cx, cy].
  - mono (기본 --mono-cam lens0u, 2026-10-16): 왼쪽 렌즈(렌즈 2) 를 **왜곡만 편** 핀홀 (R1 없음, f = K1, 초광각 화각 유지,
    calib_scale.vio_lens0_K). 정류 rect0 는 화각이 좁아 실제 손 회전 녹화에서 특징점이 끊겼다 (vio_lens0_K 주석).
    설정 파일에 `# scan3d_vio_cam0: lens0u` 표시 → scan3d 가 같은 맵으로 영상을 만들고 포즈를 rect0 로 바꿔 쓴다.
    --mono-cam rect0 = 옛 방식.
  distortion radtan [0,0,0,0], 내부 파라미터 보정 끔.

축 규약
- IMU 프레임 = Android 센서(기기) 좌표: 세로로 정면에서 볼 때 x 오른쪽, y 위, z 화면 밖.
  https://developer.android.com/develop/sensors-and-location/sensors/sensors_overview#sensors-coords
- 렌즈 카메라 좌표 = LENS_POSE_ROTATION 이 가리키는 좌표: x 센서 긴 변(영상 가로), y 짧은 변(영상 아래), z 광축
  (OpenCV 와 같음). v_cam = quat_to_R(pose_rotation) @ v_sensor.
  https://developer.android.com/reference/android/hardware/camera2/CameraCharacteristics#LENS_POSE_ROTATION
- rect0 = R1 @ (왼쪽 렌즈 카메라), rect1 = R2 @ (오른쪽 렌즈 카메라). stereoRectify 에서 R2 @ R == R1.
  R_I->C0 = R1 @ R_sensor_to_left,  R_I->C1 = R2 @ R @ R_sensor_to_left (= R_I->C0).
- kalibr T_imu_cam = cam -> IMU: [R_C->I | p_CinI].
- 평행이동: IMU 위치는 Camera2 가 알려 주지 않는다. --imu-origin lens5 (기본) = IMU 를 주 카메라(렌즈 5) 광학 중심에
  둔다고 가정 -> p_C0inI = 왼쪽 렌즈 pose_translation (LENS_POSE_REFERENCE=PRIMARY_CAMERA, 센서 축). cam0 이면 0.
  p_C1inI = p_C0inI + R_C0->I @ [baseline, 0, 0]. 둘 다 온라인 보정(calib_cam_extrinsics) 에 맡긴다.

실행 (WSL): venv_ffs/bin/python scan3d/make_ov_config.py [--fps 15 --imu-rate 400 --init static]
  -> scan3d/config/ (스테레오) + scan3d/config/mono/ (mono 설정, scan3d --vio-mode mono 용). --mono 면 mono 만 --out 에.
scan3d.py 는 세션 시작 때 ensure_config((W,H), fps, mono) 로 scan3d/config/<W>x<H>_<fps>/ 캐시를 쓴다 (없거나 낡으면 만듦).
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO))
sys.path.insert(1, str(HERE))
CONFIG_ROOT = HERE / "config"
# 설정 형식 버전 (ensure_config 캐시 판정). 2026-10-16b: mono cam0 = lens0u, 정지 판정 문턱 각도 기준
CFG_VERSION = "2026-10-16b"


def quat_to_R(q):
    """Android 쿼터니언 (x, y, z, w) -> 회전행렬 (lib_phone_calib.quat_to_R 과 같은 식)."""
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                     [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
                     [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]])


def rot_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))


def find_physical(cameras, logical, pid):
    for cam in cameras:
        if str(cam.get("id")) == str(logical):
            for ph in cam.get("physical", []):
                if str(ph.get("id")) == str(pid):
                    return ph
    raise ValueError(f"cameras json 에 logical {logical} / physical {pid} 없음")


def rectify(st):
    """lib_rectify.rectify_maps 와 같은 정류 (있으면 그것을 쓴다)."""
    cal = [dict(K=np.array(st[f"K{i}"], float), dist=np.array(st[f"dist{i}"], float),
                image_size=st["image_size"]) for i in (1, 2)]
    R, T = np.array(st["R"], float), np.array(st["T"], float)
    try:
        import lib_rectify as lr
        rp = lr.rectify_maps(cal[0], cal[1], R, T, alpha=0.0)
    except ImportError:
        import cv2
        R1, R2, P1, P2, *_ = cv2.stereoRectify(cal[0]["K"], cal[0]["dist"], cal[1]["K"], cal[1]["dist"],
                                               tuple(st["image_size"]), R, T.reshape(3, 1),
                                               flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.0)
        f = float(P1[0, 0])
        rp = dict(R1=R1, R2=R2, f=f, cx=float(P1[0, 2]), cy=float(P1[1, 2]), baseline_m=-float(P2[0, 3]) / f)
    return rp, R


def mat_rows(M, indent="    "):
    return "\n".join(f"{indent}- [" + ", ".join(f"{v:.12g}" for v in row) + "]" for row in np.asarray(M))


def T_block(R_CtoI, p_CinI):
    T = np.eye(4)
    T[:3, :3] = R_CtoI
    T[:3, 3] = p_CinI
    return mat_rows(T)


# ---------------------------------------------------------------------------------------------------
# IMU 잡음 기본값 (추정 — 실측 아님, 조정 대상)
# 폰 MEMS 데이터시트 수준 (예: ST LSM6DSO: 자이로 3.8 mdps/√Hz ≈ 6.6e-5 rad/s/√Hz, 가속 70 µg/√Hz ≈ 6.9e-4 m/s²/√Hz,
#   https://www.st.com/resource/en/datasheet/lsm6dso.pdf ; S24 의 실제 IMU 부품은 확인 안 됨)
# 에 VIO 관례대로 ~10~15배 부풀림 (모델링 안 된 진동·온도·양자화·타임스탬프 지터 흡수, 추정).
# 바이어스 random walk 는 데이터시트에 없어 경험적 값 (추정). SPEC 9.5 와 같은 값.
# 실측 (S24 LSM6DSV, rec_20261016_143702 처음 2.5 s 정지, 500 Hz): 가속 축별 std 0.008~0.024 m/s² → 밀도 5e-4~1.5e-3,
#   자이로 std 0.001~0.003 rad/s → 밀도 6.5e-5~1.8e-4, 자이로 평균(=바이어스) |0.0011| rad/s (TYPE_GYROSCOPE 는 이미 보정값).
#   → 아래 값은 실측의 약 7~15배. 오프라인 재생에서 0.5배로 줄이면 12 s 에 lost, 2배는 lost 없음 (1배와 비슷) → 그대로 둔다.
IMU_NOISE = dict(
    accelerometer_noise_density=1.0e-2,   # m/s^2/sqrt(Hz)
    accelerometer_random_walk=1.0e-3,     # m/s^3/sqrt(Hz)
    gyroscope_noise_density=1.0e-3,       # rad/s/sqrt(Hz)
    gyroscope_random_walk=1.0e-4,         # rad/s^2/sqrt(Hz)
)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=str(REPO / "calib_phone_pair.json"))
    ap.add_argument("--cameras", default=str(REPO / "phone_stereo" / "cameras_s24.json"))
    ap.add_argument("--out", default=str(REPO / "scan3d" / "config"))
    ap.add_argument("--fps", type=float, default=15.0, help="카메라 프레임률 = track_frequency")
    ap.add_argument("--imu-rate", type=float, default=400.0, help="실제 IMU 주기 Hz (sensors.json/수신 통계)")
    ap.add_argument("--init", choices=["static", "dynamic"], default="static",
                    help="static: 처음 1~2 s 가만히 든 뒤 움직이기. dynamic: 움직이면서 초기화")
    ap.add_argument("--imu-origin", choices=["lens5", "cam0"], default="lens5",
                    help="IMU 위치 가정 (lens5 = 주 카메라 광학 중심, cam0 = 왼쪽 렌즈 광학 중심)")
    ap.add_argument("--mono", action="store_true", help="cam0 만 쓰는 mono 설정 (max_cameras 1)")
    ap.add_argument("--mono-cam", choices=["lens0u", "rect0"], default="lens0u",
                    help="mono cam0 영상: lens0u = 렌즈 2 왜곡만 편 핀홀 (기본, 넓은 화각), rect0 = 정류 왼쪽 (옛 방식)")
    ap.add_argument("--exposure-ms", type=float, default=0.0,
                    help="카메라 시각이 첫 행 노출 시작 그대로일 때만: 노출 시간 (timeshift 초기값에 /2 반영)")
    ap.add_argument("--rs-skew-ms", type=float, default=0.0,
                    help="카메라 시각이 첫 행 시작 그대로일 때만: SENSOR_ROLLING_SHUTTER_SKEW ms (/2 반영). "
                         "클라이언트가 이미 노출 중앙으로 보정해 보내면 0 (기본, SPEC 9.4)")
    ap.add_argument("--gravity", type=float, default=9.80, help="중력 크기 m/s^2 (서울 ≈ 9.80)")
    ap.add_argument("--verbosity", default="INFO")
    ap.add_argument("--no-mono", action="store_true",
                    help="스테레오 설정만 (기본은 <out>/mono/ 에 mono 설정도 같이 만든다, scan3d --vio-mode mono 용)")
    ap.add_argument("--size", default=None,
                    help="영상 크기 WxH (4:3 만, 기본 = 보정 크기). 보정 크기와 다르면 K 를 비례 변환 (calib_scale.py)")
    a = ap.parse_args()
    write_configs(a)
    if not a.mono and not a.no_mono:
        b = argparse.Namespace(**vars(a))
        b.mono, b.out = True, str(Path(a.out) / "mono")
        write_configs(b)


def config_dir(size, fps, mono=False, root=CONFIG_ROOT):
    """크기·fps 별 캐시 폴더: config/<W>x<H>_<fps>/ (mono 는 그 아래 mono/)."""
    W, H = (int(v) for v in size)
    d = Path(root) / f"{W}x{H}_{int(round(float(fps)))}"
    return d / "mono" if mono else d


def ensure_config(size, fps, mono=False, calib=None, cameras=None, root=CONFIG_ROOT, log=print, **kw):
    """scan3d 세션 시작 때 호출: (W,H,fps) 설정이 캐시에 없거나 입력(보정·카메라 json·이 파일)보다 오래되면 만든다.
    반환 estimator_config.yaml 경로. kw 는 명령줄 인자 이름 그대로 (imu_rate, init, imu_origin …)."""
    calib = str(calib or REPO / "calib_phone_pair.json")
    cameras = str(cameras or REPO / "phone_stereo" / "cameras_s24.json")
    out = config_dir(size, fps, mono, root)
    est = out / "estimator_config.yaml"
    files = [est, out / "kalibr_imu_chain.yaml", out / "kalibr_imucam_chain.yaml"]
    srcs = [Path(calib), Path(cameras), Path(__file__), HERE / "calib_scale.py"]   # calib_scale.vio_lens0_K = mono intrinsics
    if all(f.is_file() for f in files):
        t_out = min(f.stat().st_mtime for f in files)
        try:
            same_ver = f"# scan3d_cfg_version: {CFG_VERSION}" in est.read_text(encoding="utf-8", errors="replace")
        except OSError:
            same_ver = False
        # 버전 표시까지 봐야 한다: 옛 코드로 돌던 프로세스가 mtime 만 보고 옛 형식으로 다시 써 둔 캐시를 새 코드가 그대로 쓰지 않게
        if same_ver and all(not s.is_file() or s.stat().st_mtime <= t_out for s in srcs):
            return est
    W, H = (int(v) for v in size)
    a = argparse.Namespace(calib=calib, cameras=cameras, out=str(out), fps=float(fps), imu_rate=400.0, init="static",
                           imu_origin="lens5", mono=bool(mono), exposure_ms=0.0, rs_skew_ms=0.0, gravity=9.80,
                           verbosity="INFO", no_mono=True, size=f"{W}x{H}", mono_cam="lens0u")
    for k, v in kw.items():
        setattr(a, k, v)
    log(f"[ov] OpenVINS 설정 만듦: {out} ({W}x{H}, {float(fps):g} fps, {'mono' if mono else 'stereo'})")
    write_configs(a)
    return est


def write_configs(a):
    """a.mono / a.out 기준으로 설정 3개를 쓴다."""
    st = json.loads(Path(a.calib).read_text(encoding="utf-8"))
    cams = json.loads(Path(a.cameras).read_text(encoding="utf-8"))
    logical = st["phone"]["logical"]
    left_id, right_id = [str(p) for p in st["phone"]["physical"][:2]]
    W0 = int(st["image_size"][0])
    if getattr(a, "size", None):
        import calib_scale
        st = calib_scale.scale_calib(st, *calib_scale.parse_size(a.size))   # K 만 비례 변환 (4:3 아니면 오류)
    W, H = st["image_size"]
    px = W / W0            # 화소 단위 문턱값(시차·특징 간격)도 같은 배율 (640x480 기준값 × px)

    phL = find_physical(cams, logical, left_id)
    phR = find_physical(cams, logical, right_id)
    R_s2l = quat_to_R(phL["pose_rotation"])          # 센서 -> 왼쪽 렌즈 카메라 (Camera2 공장값)
    R_s2r = quat_to_R(phR["pose_rotation"])
    if "R_sensor_to_left" in st:
        d = rot_deg(R_s2l, np.array(st["R_sensor_to_left"]))
        print(f"[make_ov_config] pose_rotation(lens {left_id}) vs calib R_sensor_to_left: {d:.4f} deg")
        if d > 1.0:
            print("[make_ov_config] WARNING: 1° 넘게 다름 — cameras json 과 보정 json 의 렌즈가 맞는지 확인")

    rp, R = rectify(st)
    R1, R2 = np.asarray(rp["R1"]), np.asarray(rp["R2"])
    f, cx, cy, b = float(rp["f"]), float(rp["cx"]), float(rp["cy"]), float(rp["baseline_m"])

    R_I_C0 = R1 @ R_s2l
    R_I_C1 = R2 @ R @ R_s2l
    print(f"[make_ov_config] rect f={f:.2f} cx={cx:.2f} cy={cy:.2f} baseline={b*1000:.2f} mm, "
          f"R_I->C1 vs R_I->C0 {rot_deg(R_I_C0, R_I_C1):.2e} deg, "
          f"board R vs factory (R_s2r) {rot_deg(R @ R_s2l, R_s2r):.3f} deg")
    R_C0_I, R_C1_I = R_I_C0.T, R_I_C1.T
    if a.imu_origin == "lens5":
        p_C0inI = np.asarray(phL.get("pose_translation", [0, 0, 0]), float)
        if int(phL.get("pose_reference", 0)) != 0:
            print("[make_ov_config] WARNING: pose_reference != PRIMARY_CAMERA — p_C0inI 를 0 으로")
            p_C0inI = np.zeros(3)
    else:
        p_C0inI = np.zeros(3)
    p_C1inI = p_C0inI + R_C0_I @ np.array([b, 0.0, 0.0])
    timeshift = 0.5 * (a.exposure_ms + a.rs_skew_ms) * 1e-3   # t_imu = t_cam + timeshift

    # mono cam0: lens0u (렌즈 2 왜곡만 편 핀홀, R1 없음 → R_I->C0 = 공장 pose_rotation 그대로) 또는 rect0
    # R_I->C·시간 검증 (rec_20261016_143702, 렌즈 2 원영상 프레임 간 essential 회전 vs 자이로 적분, 345 쌍):
    #   공장 pose_rotation 과 추정 R 의 차이 2.3° (프레임당 잔차 중앙값 0.33° — 영상 회전 추정 잡음 수준),
    #   최적 시간 오프셋 −0.5 ms (±5 ms 에서 잔차 +4%) → 축·부호·시각(t_mid = IMU 와 같은 시계) 모두 맞음, 고칠 것 없음.
    cam0_kind = "lens0u" if (a.mono and getattr(a, "mono_cam", "lens0u") == "lens0u") else "rect0"
    if cam0_kind == "lens0u":
        import calib_scale
        Kv = calib_scale.vio_lens0_K(st)
        f0, cx0, cy0 = float(Kv[0, 0]), float(Kv[0, 2]), float(Kv[1, 2])
        R_C0_I_cfg = R_s2l.T
        cam0_src = (f"lens0u = lens {left_id} undistorted pinhole (calib_scale.vio_lens0_K, no R1); "
                    f"R_ItoC = quat_to_R(pose_rotation lens {left_id})")
    else:
        f0, cx0, cy0 = f, cx, cy
        R_C0_I_cfg = R_C0_I
        cam0_src = f"rect0 = R1 @ lens {left_id} camera; R_ItoC = R1 @ quat_to_R(pose_rotation lens {left_id})"
    # 정지 판정 시차는 각도로 정해 cam0 초점거리로 화소 환산 (cam0 마다 f 가 달라서: rect0 463, lens0u 262 @640)
    #  init: 0.3° (전에는 15 px = rect0 에서 1.9°). 실시간 로그 runs/ov_bridge_live_20261016-143155.log 의 초기화 9번 중
    #   7번이 시차 2.6~14.8 px (= 천천히 도는 중) 에서 성공해 자이로 바이어스를 0.016~0.035 rad/s 로 잡았다 (실제 바이어스
    #   ≈0.001, 정지 구간 실측). 0.03 rad/s 바이어스 오차 = 1.7°/s 기울기 표류 → 중력이 새어 속도 ~0.3 m/s 로 흐름 (증상 그대로).
    #   진짜 정지일 때 시차는 0.04~0.09 px 라 0.3° 면 여유가 충분하다.
    #  zupt: 0.05°/프레임 (≈0.75°/s @15fps). ZUPT 갱신은 자이로 = 바이어스 로 보므로 느린 회전을 정지로 받으면 바이어스가 틀어진다.
    init_disp_px = f0 * np.tan(np.radians(0.3))
    zupt_disp_px = f0 * np.tan(np.radians(0.05))

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    runs = REPO / "scan3d" / "runs"
    stamp = dt.datetime.now().isoformat(timespec="seconds")
    head = (f"# generated {stamp} by scan3d/make_ov_config.py\n"
            f"# inputs: calib={a.calib} cameras={a.cameras} lenses L={left_id} R={right_id}\n"
            f"# rect: f={f:.4f} cx={cx:.4f} cy={cy:.4f} baseline_m={b:.6f} size={W}x{H}\n"
            f"# scan3d_cfg_version: {CFG_VERSION}\n"
            f"# scan3d_vio_cam0: {cam0_kind}\n"
            f"# cam0: f={f0:.4f} cx={cx0:.4f} cy={cy0:.4f} ({cam0_kind})\n"
            f"# args: fps={a.fps} imu_rate={a.imu_rate} init={a.init} imu_origin={a.imu_origin} mono={a.mono} "
            f"exposure_ms={a.exposure_ms} rs_skew_ms={a.rs_skew_ms}\n")
    n_cams = 1 if a.mono else 2
    dyn = a.init == "dynamic"

    est = f"""%YAML:1.0 # need to specify the file type at the top!
{head}
verbosity: "{a.verbosity}" # ALL, DEBUG, INFO, WARNING, ERROR, SILENT

use_fej: true
integration: "rk4"
use_stereo: {"false" if a.mono else "true"} # true = 정류 쌍 스테레오 KLT (클라이언트는 2장짜리만 보냄, 시각차 큰 쌍은 버림). false = cam0 만 (scan3d --vio-mode mono)
max_cameras: {n_cams}

calib_cam_extrinsics: true   # R_ItoC, p_CinI 온라인 보정 (p 초기값은 가정값)
calib_cam_intrinsics: false  # 정류 영상 -> 핀홀 고정
calib_cam_timeoffset: true   # 카메라-IMU 시간 오프셋 온라인 보정 (같은 elapsedRealtime 시계라 초기 0 근처)
calib_imu_intrinsics: false
calib_imu_g_sensitivity: false

max_clones: 11
max_slam: 50
max_slam_in_update: 25
max_msckf_in_update: 40
dt_slam_delay: 1

gravity_mag: {a.gravity:.4f}

feat_rep_msckf: "GLOBAL_3D"
feat_rep_slam: "ANCHORED_MSCKF_INVERSE_DEPTH"
feat_rep_aruco: "ANCHORED_MSCKF_INVERSE_DEPTH"

# 정지(손에 들고 멈춤) 구간에서 드리프트 억제: 시차 기반 ZUPT
try_zupt: true
zupt_chi2_multipler: 0 # 0 = 시차만 사용
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10
zupt_max_disparity: {zupt_disp_px:.3g} # 0.05° × cam0 f (make_ov_config.py 주석: 느린 회전을 정지로 받으면 바이어스가 틀어짐)
zupt_only_at_beginning: false

# ==================================================================
# 초기화. try_zupt=true 면 OpenVINS 는 "정지 판정"(두 반쪽 창 모두 시차·가속 흔들림이 문턱 아래) 순간 정적 초기화한다
# (VioManager: wait_for_jerk = (ZUPT 없음)). 시작 때 폰을 1.5 s 넘게 가만히 들고 있어야 한다.
# 문턱이 느슨하면 천천히 도는 중에 초기화돼 그 회전을 자이로 바이어스로 잡는다 → 속도 표류 (make_ov_config.py 주석, 실측).
init_window_time: {1.0 if dyn else 1.5}
init_imu_thresh: {0.5 if dyn else 0.3} # 가속도 표준편차 m/s^2. 실측: 정지 0.015~0.05, 살짝 건드림 0.1, 손 움직임 0.4~0.9 (전에는 1.0)
init_max_disparity: {init_disp_px:.2f} # 정지 판정 최대 평균 시차 px = 0.3° × cam0 f (전에는 15 px, 실측 근거는 make_ov_config.py)
init_max_features: 50

# dynamic: rec_20261016_143702 를 5 s 부터(움직이는 중) 재생하면 21 s 동안 한 번도 성공 못 함 (opt 반복 한도·공분산 실패 —
# 손 회전 위주라 이동이 작아 축척이 안 잡힘). 손에 든 폰은 정지 시작(static)이 현실적.
init_dyn_use: {"true" if dyn else "false"}
init_dyn_mle_opt_calib: false
init_dyn_mle_max_iter: 50
init_dyn_mle_max_time: 0.05
init_dyn_mle_max_threads: 4
init_dyn_num_pose: 6
init_dyn_min_deg: 10.0

init_dyn_inflation_ori: 10
init_dyn_inflation_vel: 100
init_dyn_inflation_bg: 10
init_dyn_inflation_ba: 100
init_dyn_min_rec_cond: 1e-12

init_dyn_bias_g: [ 0.0, 0.0, 0.0 ]
init_dyn_bias_a: [ 0.0, 0.0, 0.0 ]

# ==================================================================
record_timing_information: false
record_timing_filepath: "{runs / 'ov_timing.txt'}"

save_total_state: false
filepath_est: "{runs / 'ov_estimate.txt'}"
filepath_std: "{runs / 'ov_estimate_std.txt'}"
filepath_gt: "{runs / 'ov_groundtruth.txt'}"

# ==================================================================
use_klt: true
num_pts: 200
fast_threshold: 15 # 폰 소형 센서 영상 대비가 낮아 euroc(20) 보다 낮춤 (추정)
grid_x: 5
grid_y: 5
min_px_dist: {max(3, int(round(10 * px)))} # 640x480 기준 10 px × 크기 배율
knn_ratio: 0.70
track_frequency: {a.fps:.1f}
downsample_cameras: false
num_opencv_threads: 4
histogram_method: "HISTOGRAM"

use_aruco: false
num_aruco: 1024
downsize_aruco: true

# ==================================================================
up_msckf_sigma_px: 1
up_msckf_chi2_multipler: 1
up_slam_sigma_px: 1
up_slam_chi2_multipler: 1
up_aruco_sigma_px: 1
up_aruco_chi2_multipler: 1

use_mask: false

relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
"""

    n = IMU_NOISE
    imu = f"""%YAML:1.0
{head}
imu0:
  T_i_b:
{mat_rows(np.eye(4))}
  # 잡음: 폰 MEMS 데이터시트 값 x ~10-15 (추정, 조정 대상). 근거는 make_ov_config.py IMU_NOISE 주석.
  accelerometer_noise_density: {n['accelerometer_noise_density']:.4e}  # [ m / s^2 / sqrt(Hz) ]
  accelerometer_random_walk: {n['accelerometer_random_walk']:.4e}    # [ m / s^3 / sqrt(Hz) ]
  gyroscope_noise_density: {n['gyroscope_noise_density']:.4e}     # [ rad / s / sqrt(Hz) ]
  gyroscope_random_walk: {n['gyroscope_random_walk']:.4e}       # [ rad / s^2 / sqrt(Hz) ]
  rostopic: /imu0
  time_offset: 0.0
  update_rate: {a.imu_rate:.1f}
  # IMU 프레임 = Android 센서 좌표 (x 오른쪽, y 위, z 화면 밖). 가속도는 중력 포함 원시 비력 (TYPE_ACCELEROMETER).
  model: "kalibr"
  Tw:
{mat_rows(np.eye(3))}
  R_IMUtoGYRO:
{mat_rows(np.eye(3))}
  Ta:
{mat_rows(np.eye(3))}
  R_IMUtoACC:
{mat_rows(np.eye(3))}
  Tg:
{mat_rows(np.zeros((3, 3)))}
"""

    def cam_block(i, R_CtoI, p_CinI, overlaps, src, K=(f, cx, cy), what="정류 P1"):
        ts = f"  timeshift_cam_imu: {timeshift:.6f} # t_imu = t_cam + timeshift (초기값, 온라인 보정)\n" if i == 0 else ""
        return f"""cam{i}:
  # {src}
  T_imu_cam: # rotation from camera to IMU R_CtoI, position of camera in IMU p_CinI
{T_block(R_CtoI, p_CinI)}
  cam_overlaps: {overlaps}
  camera_model: pinhole
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0] # 왜곡 편 영상
  distortion_model: radtan
  intrinsics: [{K[0]:.6f}, {K[0]:.6f}, {K[1]:.6f}, {K[2]:.6f}] # fu, fv, cu, cv ({what})
  resolution: [{W}, {H}]
  rostopic: /cam{i}/image_raw
{ts}"""

    imucam = (f"%YAML:1.0\n{head}\n"
              + cam_block(0, R_C0_I_cfg, p_C0inI, "[1]" if not a.mono else "[]", cam0_src, K=(f0, cx0, cy0),
                          what="정류 P1" if cam0_kind == "rect0" else "calib_scale.vio_lens0_K"))
    if not a.mono:
        imucam += cam_block(1, R_C1_I, p_C1inI, "[0]",
                            f"rect1 = R2 @ lens {right_id} camera; origin = rect0 + (baseline, 0, 0)")

    (out / "estimator_config.yaml").write_text(est, encoding="utf-8")
    (out / "kalibr_imu_chain.yaml").write_text(imu, encoding="utf-8")
    (out / "kalibr_imucam_chain.yaml").write_text(imucam, encoding="utf-8")
    print(f"[make_ov_config] wrote {out}/estimator_config.yaml, kalibr_imu_chain.yaml, kalibr_imucam_chain.yaml")
    print(f"[make_ov_config] cam0={cam0_kind} f={f0:.2f} init_max_disparity={init_disp_px:.2f}px "
          f"zupt_max_disparity={zupt_disp_px:.3g}px R_I->C0 =\n{np.array2string(R_C0_I_cfg.T, precision=4)}\n"
          f" p_C0inI={p_C0inI}, p_C1inI={p_C1inI}")


def vio_cam0_kind(cfg):
    """설정 폴더(estimator_config.yaml 경로 또는 폴더)의 '# scan3d_vio_cam0: X' 표시 → 'lens0u' | 'rect0'.
    표시가 없으면 옛 설정 = 'rect0'."""
    import re
    cfg = Path(cfg)
    d = cfg if cfg.is_dir() else cfg.parent
    for name in ("kalibr_imucam_chain.yaml", "estimator_config.yaml"):
        try:
            m = re.search(r"#\s*scan3d_vio_cam0:\s*(\w+)", (d / name).read_text(encoding="utf-8", errors="replace"))
        except OSError:
            continue
        if m:
            return m.group(1)
    return "rect0"


if __name__ == "__main__":
    main()
