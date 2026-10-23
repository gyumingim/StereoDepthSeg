# phone60 설정 비교 — 계획 (2026-10-23, 실행 전 커밋)

사용자 결정: VIO 입력은 60 fps 유지, OpenVINS 설정을 60 fps 에 맞춘다.
설정 = `config/phone60/` (기존 `config/mono/` 는 그대로). 출발점 = OpenVINS 저자 안드로이드 앱 설정
(https://github.com/goldbattle/open_vins_mobile/blob/HEAD/app_device/config/estimator_config.yaml), 조사 = `docs/openvins_settings_survey.md`.

## base 와 다른 값 (이것만 다름)
| 키 | base | phone60 | 이유 | 출처 |
|---|---|---|---|---|
| `max_clones` | 11 | **20** | 60 fps × 20 = 0.33 s 창. 업스트림은 11 clone @ 21–31 Hz = 0.35–0.5 s | 조사 0-4, 업스트림 config 14 개 |
| `try_zupt` | true | true | 정지에서 바로 정적 초기화 (false 면 초기화 방식이 jerk 대기로 바뀜) | 조사 2, InertialInitializer.cpp |
| `zupt_only_at_beginning` | false | **true** | 손·폰·드론 설정은 전부 true, 계속 켜는 건 지상 차량뿐 | 조사 3.1, 폰 앱 |
| `zupt_max_disparity` | 0.5 px | **0.2 px** | 업스트림 0.5 px @ ~20 Hz 와 같은 px/s (60 fps 에서 0.5 px = 30 px/s 미만을 정지로 봄) | 조사 0-3 (환산은 계산) |
| `zupt_noise_multiplier` | 10 | **50** | 폰 앱 값 | 폰 앱 |
| `calib_cam_intrinsics` | false | **true** | 폰 앱: 온라인 보정 셋 다 켬 (extrinsics·timeoffset 은 base 도 켬) | 폰 앱 |
| `up_msckf/slam/aruco_sigma_px` | 1 | **2** | 폰 앱 값 (RS·압축 잔차 흡수, 이유는 추정) | 폰 앱 |
| `up_*_chi2_multipler` | 1 | **5** | 폰 앱 값 | 폰 앱 |
| `fast_threshold` | 15 | **20** | 폰 앱 값 (num_pts 200, grid 5×5, min_px_dist 10, KLT, HISTOGRAM 은 이미 같음) | 폰 앱 |
| IMU `update_rate` | 400 | **500** | 원시 IMU 실측 498.7–499.0 Hz (133820, 214550, 154320) | 측정 |

같게 둔 것: 초기화 (strict 정적 전용: `init_window_time 1.0`, `init_imu_thresh 0.3`, `init_max_disparity 15 px`, `init_dyn_use false`, `init_max_features 50`).
이유: OpenVINS 의 초기화 시차는 창의 앞 반·뒷 반 사이 시차(시간 기준)라 fps 와 무관하다. 영상 크기·f(≈430 px)도 같으므로 15 px = 같은 각도(≈2.0°).
폰 앱의 1.0 s / 0.4 / 2 px 는 jerk 방식 초기화(try_zupt false)용이라 가져오지 않았다. IMU 잡음도 base 그대로 (Allan × 15–25, 관례 범위).
`gravity_mag` 는 base 9.80 유지 (폰 앱 9.81, 차이 무시).

## 실행
- `imu_learn/phone60_run.py`: 15 녹화 × 오프셋 0/1/2 = 45 칸 × {base, phone60} = 90 run, 칸당 1 회.
  녹화 = 보드 3 (133820, 134550, 134204), i3 4 (214550, 213341, 213955, 215348), F 6 (214138, 221052, 221925, 222747, 223815, 224323), O 2 (131326, 154320).
- 같은 것: stock 다리 `ov_bridge/build/ov_bridge` (cam0 내부 파라미터 `intr0` 출력만 추가), 60 fps 640x360, `--frame-cache`, 원시 IMU (O 2 개는 원시 없음 → 보정 IMU, 두 조건 같음),
  `gyro_rebias False` 명시, `num_opencv_threads 2`, still-hold 꺼짐, IMU lead 2 s, 끝까지, 동시 ≤ 12.
- 섞인 변수: phone60 은 위 표 10 개 값을 한꺼번에 바꾼다 (어느 값이 효과를 냈는지는 이 시험으로 모름).

## 기준 (2 개)
채점 = `zupt_ob_score.run_metrics` (GT 중심 p_bc, 창 = 첫 추적 포즈 ~ 마지막 GT 표본).
- **(a) 발산 run 수**: 아무 5 s 창 상대 오차 > 30 cm 또는 창 ATE > 50 cm. 213955 (폰 낙하) 는 빼고 세고 따로 보고. phone60 ≤ base 이면 통과.
- **(b) 정상 run ATE 중앙값**: 두 조건 모두 발산 아닌 칸 (213955 제외) 의 창 ATE 중앙값. phone60 ≤ 1.2 × base + 0.3 cm 이면 통과.

판정: 둘 다 통과 → phone60 채택. 아니면 base 유지. 보고: 녹화별 ATE, 발산 수, run 당 시간, 전체 시간, 온라인 보정 값 (시간 오프셋, f).
