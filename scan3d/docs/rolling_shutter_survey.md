# S24 롤링셔터(RS) 줄이기·보정 조사

작성: 2026-10-23, 조사 에이전트 (웹 출처 + 우리 측정값 2개). 커밋 안 함.
입력: 우리 측정 `CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW`
- 고속 세션(constrained high-speed) 1920x1080 @240 fps → **2.94 ms**
- 일반 세션 1920x1080 @60 fps, 노출 2 ms → **6.54 ms**

## 결론 먼저

1. **스큐를 줄이는 가장 확실한 손잡이는 "센서 모드"이고, 앱에서 고를 수 있는 방법은 고속 세션(120/240 fps)뿐이다.** Android에는 읽기 속도를 직접 정하는 키가 없다. HAL이 출력 크기와 fps를 보고 센서 모드를 고른다. 이미 2.94 ms를 얻었으니, 남은 측정은 "더 작은 크기나 다른 fps에서 더 빠른 모드가 잡히는가"를 확인하는 것이다.
2. **스큐가 2.9 ms면 보정은 싸고 효과가 크다.** 1 rad/s, 640 px 폭(f≈430 px) 기준으로 프레임 위아래 어긋남은 약 **1.3 px**(60 fps 일반 세션은 **2.8 px**)이다. 자이로로 특징점 좌표만 행마다 회전 보정하면 회전 성분이 거의 사라진다(잔차 약 0.1 px, 추정).
3. 권장 구현: **특징점 단위 자이로 RS 보정**(이미지 전체 워핑은 하지 않음)과 기준 시각을 "가운데 행 + 노출/2"로 두는 것. 나중에 VIO 쪽에서 VINS-Mono 방식(특징 속도 × 행 시간) 항을 추가해 병진 성분도 줄인다.

---

## 0. S24 기본형 메인 센서 = ISOCELL GN3 (S5KGN3)

- S24/S24+는 센서를 바꾸지 않았고, 메인은 GN3다 ([Notebookcheck](https://www.notebookcheck.net/New-Samsung-Galaxy-S24-and-S24-do-not-feature-camera-sensor-upgrades.795131.0.html)). 1/1.57", 1.0 µm, S22·S23·S24 공용 ([GSMArena S24 FE 리뷰](https://m.gsmarena.com/samsung_galaxy_s24_fe-review-2755p5.php)). 삼성의 공식 S24 사양표로는 확인하지 못했다.
- 삼성 공식 GN3 페이지의 영상 모드는 **8K 30 fps, 4K 120 fps, FHD 480 fps**이고, 2x2 Tetrapixel 비닝으로 12.5 MP를 낸다 ([Samsung Semiconductor GN3](https://semiconductor.samsung.com/image-sensor/mobile-image-sensor/isocell-gn3/)). 이 페이지는 2025-07-31 이후 내려간다고 적혀 있다. 공개 데이터시트나 행 시간(line time) 값은 찾지 못했다.
- active array 4080x3060 = 50 MP(8160x6120)를 2x2 비닝한 12.5 MP다. 이 값은 위 비닝 설명과 맞는다(추정).
- **물리 상한(논리적 사실)**: 한 프레임 안에 노출을 시작하고 읽어야 하므로, 어떤 모드의 읽기 시간은 그 모드의 최대 fps 주기보다 짧다. 그래서 GN3의 FHD 480 fps 모드는 ≤2.08 ms, 4K 120 fps 모드는 ≤8.3 ms다. Android 문서도 스큐 범위를 "< 최소 프레임 간격"으로 둔다 ([CaptureResult 문서](https://developer.android.com/reference/android/hardware/camera2/CaptureResult#SENSOR_ROLLING_SHUTTER_SKEW)).
  - 우리 값 2.94 ms(240 fps, 주기 4.17 ms)와 6.54 ms(4K120급 모드의 상한 8.3 ms 안)는 이 상한과 모순이 없다.
  - 일반 60 fps 세션은 4K/12.5 MP 비닝 모드로, 고속 240 fps 세션은 FHD 고속 모드로 읽는 것으로 보인다(**추정**, 직접 확인 안 됨).
  - 센서에는 **FHD 480 fps 모드(≤2.08 ms)**가 있지만, S24가 서드파티 Camera2에 이 모드를 여는지는 모른다. `getHighSpeedVideoFpsRanges()`로 확인해야 한다.

## 1. Android Camera2에서 읽기 속도에 영향을 주는 것

| 손잡이 | 효과 | 근거 |
|---|---|---|
| 읽기 속도를 직접 정하는 표준 키 | **없음.** 스큐는 결과(Result) 값이고 요청할 수 없다 | [CaptureResult](https://developer.android.com/reference/android/hardware/camera2/CaptureResult#SENSOR_ROLLING_SHUTTER_SKEW) |
| 출력 크기 조합 (stream size) | HAL이 출력 크기에 맞춰 비닝/크롭 모드를 고른다. 문서에 "binned or cropped mode due to the current output target resolutions"라고 적혀 있다 | 같은 곳 |
| 고속 세션 (CONSTRAINED_HIGH_SPEED, ≥120 fps) | 별도 고속 센서 모드를 쓴다. 우리 측정 6.54 → 2.94 ms. **대신 AE/AWB/AF가 강제로 자동이 되어 수동 노출이 불가능하다**(aeLock, 노출보정, zoomRatio, cropRegion, OIS 모드는 동작). 출력 표면은 최대 2개(프리뷰·녹화)이고 `createHighSpeedRequestList`로 만든 요청만 받는다 | [CameraMetadata CONSTRAINED_HIGH_SPEED_VIDEO](https://developer.android.com/reference/android/hardware/camera2/CameraMetadata#REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO) |
| 일반 세션에서 고fps 범위 광고 | 기기가 aeAvailableTargetFpsRanges에 120 같은 값을 넣어 두면 일반 세션에서도 고fps가 된다. 이때는 수동 제어도 가능하다 | 같은 곳 |
| SENSOR_PIXEL_MODE = MAXIMUM_RESOLUTION | 비닝을 끈 50 MP 풀 해상도 모드다. 행 수가 2배라 **읽기가 더 느려질 가능성이 크다**(추정). VIO에는 쓰지 않는다 | [CaptureRequest SENSOR_PIXEL_MODE](https://developer.android.com/reference/android/hardware/camera2/CaptureRequest#SENSOR_PIXEL_MODE) |
| CONTROL_ZOOM_RATIO / SCALER_CROP_REGION | 대부분 ISP 디지털 크롭이라 센서 읽기는 그대로다. 2x에서 센서 내부 크롭(풀해상도 중앙 크롭) 모드로 바뀌는 기기도 있는데, 그러면 행 수와 행 시간이 함께 바뀐다(추정, 측정 필요) | 위 SENSOR_PIXEL_MODE 문서의 모드 구분을 근거로 한 추정 |
| 프레임레이트 (일반 세션 30/60) | 같은 센서 모드 안에서는 보통 blanking만 늘고 읽기 시간은 그대로다(추정) | 아래 2절 |
| 삼성 벤더 태그 (`samsung.android.*`) | **공개 문서를 찾지 못했다.** 삼성 포럼은 "서드파티 앱은 FPS가 제한된다"고 답했다. S24에서 60 fps가 되는 것은 우리가 확인했다 | [Samsung dev forum](https://forum.developer.samsung.com/t/high-speed-video-recording-issue-on-samsung-s23-series/25983) |
| 읽기 시작 타임스탬프 (SENSOR_READOUT_TIMESTAMP) | 읽기를 빠르게 하지는 않는다. 다만 지원 기기는 `onReadoutStarted`로 읽기 시작 시각을 준다. "readout − exposure start = 노출 + 상수 오프셋(센서 크롭 때문)"이라 시간 정렬 검증에 쓸 수 있다 | [CameraCharacteristics SENSOR_READOUT_TIMESTAMP](https://developer.android.com/reference/android/hardware/camera2/CameraCharacteristics#SENSOR_READOUT_TIMESTAMP) |

**SENSOR_ROLLING_SHUTTER_SKEW 정의 (문서 요약)** ([출처](https://developer.android.com/reference/android/hardware/camera2/CaptureResult#SENSOR_ROLLING_SHUTTER_SKEW)):
- active array의 첫 행 노출 시작부터 (마지막+1) 행 노출 시작까지의 시간이고, 일반 RS 센서에서는 프레임 읽기 시간과 같다.
- 비닝·크롭 모드에서는 **전체 배열 기준으로 보고될 수 있어 실제보다 클 수 있다.** 출력이 active 높이 H 중 N행을 덮으면 **N/H를 곱해** 쓰라고 한다.
- Android 11 이전에는 정의가 달랐다(첫 행 ~ 마지막 행). AOSP 커밋에서 수정되었다 ([AOSP commit](https://android.googlesource.com/platform/hardware/interfaces/+/34bb3064fe%5E%21)).
- **우리 경우에 미치는 영향**: 1080p(16:9)는 4:3 배열의 세로 75%만 쓴다(2295/3060). HAL이 전체 배열 기준으로 보고했다면 실제 뷰 안의 스큐는 2.94→**2.2 ms**, 6.54→**4.9 ms**다. 어느 쪽인지는 **측정으로만 알 수 있다**(4절 시험 A). 지금은 "보고값 그대로"를 보수적 가정으로 둔다.

## 2. 폰 센서의 실제 읽기 시간 (참고값)

| 대상 | 읽기 시간 | 출처·신뢰도 |
|---|---|---|
| **S24 / GN3, 우리 측정 (1080p240 고속)** | 2.94 ms (보고값) | 우리 측정 |
| **S24 / GN3, 우리 측정 (1080p60 일반)** | 6.54 ms (보고값) | 우리 측정 |
| GN3 모드 상한 | FHD480 ≤2.08, 4K120 ≤8.3, 8K30 ≤33 ms | [Samsung GN3](https://semiconductor.samsung.com/image-sensor/mobile-image-sensor/isocell-gn3/)의 fps에서 계산 |
| Huawei P30 Pro, 2336x1080@60 영상 | 행당 6.58 µs. 1080행이면 약 7.1 ms(추정) | [arXiv 2605.29118](https://arxiv.org/html/2605.29118) (200x200 영역 측정이라 전체 높이로 늘린 값은 추정) |
| iPhone 15 Pro | 약 5 ms | [wolfcrow](https://wolfcrow.com/the-essential-guide-to-rolling-shutter-what-you-need-to-know/) (글쓴이가 직접 측정하지 않았다고 명시, 낮은 신뢰) |
| 시네마 카메라 크롭 vs 풀 | ARRI Alexa 35 16:9 5.7 / Open Gate 7.9 ms, RED V-Raptor S35 6 / FF 8 ms | 같은 wolfcrow (크롭하면 행이 줄어 빨라진다는 예) |
| GH4: 4K 22.5 ms vs 1080p 13.7 ms / NX1: 1080p 7.9 ms vs 4K 30.9 ms | 모드에 따라 반대 방향도 있음 | [dvxuser 측정 스레드](https://www.dvxuser.com/threads/measuring-rolling-shutter-put-a-number-on-this-issue.307186/page-3) |
| TUM RS-VI 데이터셋(산업용 카메라) | 행당 29.47 µs. 1024행이면 약 30 ms(추정) | [TUM RS dataset](https://cvg.cit.tum.de/data/datasets/rolling-shutter-dataset) |
| 측정 방법 표준 | DxO LED Universal Timer: 행마다 LED 위치로 ERS를 잰다 | [DxO 2014 논문](https://corp.dxomark.com/wp-content/uploads/2017/11/2014_TimingMeasurements_2014-01-31_wCopyright.pdf) |

**"낮은 해상도 / 높은 fps = 빠른 읽기"인가?**
- **높은 fps 모드는 거의 항상 빠르다.** 읽기 시간 < 프레임 주기라는 물리 상한이 있기 때문이다(위 계산, 우리 측정도 그렇다).
- **해상도만 낮추는 것은 보장되지 않는다.** 같은 센서 모드를 ISP에서 줄이면 읽기 시간은 그대로다. 비닝·스키핑·크롭으로 센서 행 수가 줄 때만 빨라진다. GH4와 NX1이 반대 결과를 보인 것이 그 예다. 그래서 **크기·fps마다 직접 재야 한다.**

## 3. VIO/SLAM 쪽에서 RS를 다루는 방법

| 시스템/방법 | RS 처리 | 출처 |
|---|---|---|
| **OpenVINS** | RS 모델이 없다. 저자(goldbattle)는 "RS 읽기 시간이 작으면 글로벌 셔터와 (어느 정도) 비슷하게 동작할 것"이라고 답했다 | [issue #464 댓글](https://github.com/rpng/open_vins/issues/464#issuecomment-2495641577) |
| **VINS-Mono** | `rolling_shutter: 1`, `rolling_shutter_tr: <초, 프레임 읽기 시간>`. 투영 잔차에서 특징점 좌표를 `pts - (td - td_i + TR/ROW * (row - ROW/2)) * velocity`로 옮긴다. **이미지 평면의 특징 속도(광류)로 1차 보정**하는 방식이고, 가운데 행이 기준이다. 로그 문구는 "per line"이지만 수식상 프레임 전체 시간이다 | [euroc_config.yaml](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/config/euroc/euroc_config.yaml), [projection_td_factor.cpp](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/factor/projection_td_factor.cpp), [parameters.cpp](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/parameters.cpp) |
| **Basalt** | VIO 소스(미러)에 "rolling"/"readout" 문자열이 없다. RS 모델이 없는 것으로 본다(우리가 grep으로 확인, 하위 헤더 모듈은 미확인) | [basalt-mirror](https://github.com/VladyslavUsenko/basalt-mirror) |
| TUM 직접법 RS-VIO | 직접법 BA에 RS를 모델링했다. "RS를 무시하면 정확도와 강건성이 심하게 떨어진다." RS 데이터셋을 공개했다 | [Schubert et al. IROS 2019, arXiv 1911.01015](https://arxiv.org/abs/1911.01015) |
| Li & Mourikis (MSCKF-RS) | 궤적을 가정하지 않고 오차를 모델링한다. 폰 추적(ICRA 2013)에서는 속도·각속도를 상태에 넣는다 | [IJRR 2014 PDF](https://intra.ece.ucr.edu/~mourikis/papers/LiI2014JRR_RS.pdf), [ICRA 2013](https://intra.ece.ucr.edu/~mourikis/papers/Li2013ICRA_b.pdf) |
| Guo et al. (MARS) | 이웃 IMU 자세 보간 모델(상수속도 모델보다 빠름)과 OC-EKF를 쓴다. Galaxy S4로 실험했다 | [RSS 2014](https://roboticsproceedings.org/rss10/p57.html), [MARS 페이지](https://mars.cs.umn.edu/research/rollingshutter_vins.php) |
| Karpenko et al. 2011 | 자이로만으로 프레임 행마다 회전을 계산해 메시를 워핑한다. 읽기 시간과 자이로 지연은 한 번 녹화한 영상으로 최적화해 보정한다. 폰 GPU에서 실시간으로 돈다 | [Stanford tech report](https://graphics.stanford.edu/papers/stabilization/) |
| Google Pixel 2 Fused stabilization | 200 Hz 자이로로 "스캔라인마다 촘촘한 움직임 벡터"를 만들고 메시 비강체 워핑으로 RS를 지운다. "몇 ms만 어긋나도 떨림이 보인다" | [Google Research blog](https://research.google/blog/fused-video-stabilization-on-the-pixel-2-and-pixel-2-xl/) |
| ARCore | EIS가 자이로와 이미지 메타데이터로 RS와 흔들림을 함께 보정한다. 기기별 보정값은 모델 코드로 런타임에 받는다. 트래킹 내부 RS 처리는 비공개다 | [ARCore EIS](https://developers.google.com/ar/develop/electronic-image-stabilization), [ARCore 인증](https://developers.google.com/ar/develop/device-certification) |
| 보정 도구 | Kalibr `kalibr_calibrate_rs_cameras`(pinhole-radtan-rs, 카메라만, IMU 없음). Huai et al.은 연속시간 RS 카메라-IMU 보정으로 행 지연까지 추정한다 | [Kalibr wiki](https://github.com/ethz-asl/kalibr/wiki/Rolling-Shutter-Camera-calibration), [arXiv 2108.07200](https://arxiv.org/abs/2108.07200), [Oth et al. CVPR 2013](https://openaccess.thecvf.com/content_cvpr_2013/html/Oth_Rolling_Shutter_Camera_2013_CVPR_paper.html) |

## 4. 우리 경우에 대한 권장

### 4.1 크기 계산 (f ≈ 430 px, 폭 640, ω = 1 rad/s)

RS로 위아래 행 사이에 생기는 회전 어긋남은 대략 f·ω·t_r이다(소각 근사).

| 모드 | t_r (보고값) | 위↔아래 어긋남 | 가운데 행 기준 최대 | 뷰 크롭 보정 시(×0.75, 가정) |
|---|---|---|---|---|
| 고속 240 fps | 2.94 ms | **1.26 px** | 0.63 px | 0.95 px |
| 일반 60 fps | 6.54 ms | **2.81 px** | 1.41 px | 2.1 px |
| (참고) GN3 FHD480 상한 | ≤2.08 ms | ≤0.89 px | ≤0.45 px | – |

- 드론의 빠른 요(3 rad/s)에서는 위 값의 3배다. 240 fps에서 3.8 px, 60 fps에서 8.4 px. KLT 특징 잡음(약 1 px)보다 크다.
- 병진 성분은 자이로로 보정할 수 없다. 옆 속도 5 m/s, 깊이 5 m이면 f·v/Z = 430 px/s이고, 2.94 ms 동안 **1.26 px**다. 깊이 20 m면 0.3 px다(계산).
- **자이로 보정 후 회전 잔차(추정)**: t_r 오차 10%면 240 fps에서 0.13 px, 60 fps에서 0.28 px(1 rad/s, 위↔아래). 카메라-자이로 시간 오프셋 오차 δ는 RS와 상관없이 f·ω·δ로 남는다(1 ms → 0.43 px). 그래서 t_d 보정이 RS 보정만큼 중요하다.

### 4.2 폰에서 먼저 잴 모드 (순위)

매 모드마다 `SENSOR_ROLLING_SHUTTER_SKEW`, `SENSOR_FRAME_DURATION`, `SENSOR_EXPOSURE_TIME`, 그리고 가능하면 `onReadoutStarted` 차이를 기록한다. 측정 전에 `getHighSpeedVideoSizes()`와 `getHighSpeedVideoFpsRangesFor()` 목록을 덤프한다.

1. **고속 세션, 목록에서 가장 큰 fps(480이 있다면)와 1920x1080.** GN3에는 FHD480 모드가 있다.
2. **고속 세션 1280x720 @240 / @120.** 720p 전용 비닝·스키핑 모드가 잡히는지 본다.
3. **고속 세션 1920x1080 @120.** 240과 같은 모드인지 본다. 같으면 120 fps로 연산 부담을 줄이면서 2.9 ms를 유지할 수 있다.
4. **일반 세션에서 aeAvailableTargetFpsRanges에 120이 있으면 1920x1080·1280x720 @120.** 수동 노출이 가능해 VIO에 더 좋다.
5. 일반 세션 4:3 크기(1440x1080, 640x480) @60과 16:9 @60 비교. 보고값이 같으면 "전체 배열 기준 보고"를 뜻한다(N/H 스케일 필요).
6. 일반 세션 1080p @30 vs @60. 같으면 fps가 읽기 시간에 영향이 없다는 확인이 된다.
7. CONTROL_ZOOM_RATIO 2.0 (일반·고속 둘 다). 센서 크롭 모드로 바뀌는지 본다.
8. (참고만) 3840x2160 @30/60. SENSOR_PIXEL_MODE MAXIMUM_RESOLUTION은 생략해도 된다.

**보고값 자체를 검증하는 물리 시험 (먼저 할 것)**
- 시험 A: PWM LED 또는 형광등 깜빡임(주파수를 아는 것)을 찍는다. 띠 간격(행)을 재면 행당 시간이 나오고, 이를 보고값/H와 비교한다. 4:3과 16:9 크기에서 둘 다 해서 N/H 문제를 가린다(DxO 타이머와 같은 원리).
- 시험 B: 세로 직선 모서리를 보며 폰을 요축으로 빠르게 돌려 기울기(px)와 자이로 ω를 잰다. 기울기는 f·ω·t_r이어야 한다.
- 고속 세션은 **노출이 자동뿐**이다. aeLock과 노출보정(−)으로 짧은 노출을 유도하고, 실제 노출은 결과 값으로 기록한다. OIS는 `LENS_OPTICAL_STABILIZATION_MODE_OFF`로 끈다. OIS가 렌즈를 움직이면 내부 파라미터가 흔들린다(추정, 일반 관행).

### 4.3 스큐를 못 줄이면 구현할 보정

**1순위: 특징점 단위 자이로 회전 보정 (Karpenko식을 점에만 적용)**
- 각 특징 (u,v)의 노출 중심 시각: `t(v) = t_sensor + (v_full / H_active)·t_r + t_exp/2`. 여기서 v_full은 active array 좌표의 행이고, 크롭 오프셋을 반영한다.
- 기준 시각 `t_ref = t_sensor + t_r/2 + t_exp/2`(가운데 행)로 잡고, 자이로 적분 회전으로 베어링을 옮긴다: `b' = R(t_ref ← t(v)) · K⁻¹[u,v,1]`. 이 이미지의 VIO 타임스탬프를 t_ref로 넘긴다.
- 비용은 특징 수(수백 개) × 3x3 회전이라 거의 0이다. 이미지 리샘플이 없어 KLT 품질도 그대로다.
- t_r은 보고값에서 시작해 시험 A/B로 확정한다. t_d(카메라-IMU 지연)는 OpenVINS 온라인 추정이나 Kalibr로 따로 잡는다.
- 기대 효과: 1 rad/s에서 회전 RS 오차를 1.26 px(240 fps) 또는 2.81 px(60 fps)에서 약 0.1–0.3 px로 줄인다(추정, t_r 10% 오차 가정).

**2순위(나중): VINS-Mono식 잔차 내 보정.** 특징 광류 속도 × (행 − 가운데)·t_r/H를 쓰면 회전과 병진을 함께 1차로 다룬다. OpenVINS를 쓰면 업데이트 단계에 이 항을 넣어야 하므로 코드 수정이 필요하다.

**하지 않을 것**: 이미지 전체 메시 워핑(비용과 보간 블러가 있고 특징 보정과 효과가 같다), 50 MP 최대 해상도 모드.

## 확인 못 한 것 / 가정

- S24 HAL이 스큐를 전체 배열 기준으로 보고하는지, 실제 뷰 기준으로 보고하는지 모른다 → 시험 A.
- 일반 60 fps와 고속 240 fps가 각각 GN3의 어떤 모드인지 모른다(추정만 함).
- S24가 서드파티에 FHD 480 fps나 일반 세션 120 fps를 여는지 모른다.
- 삼성 벤더 태그 중 센서 모드를 고르는 키가 있는지 공개 자료가 없다. 기기에서 `CameraCharacteristics.getKeys()`/`getAvailableCaptureRequestKeys()`의 `samsung.*` 키를 덤프해 확인한다.
- Karpenko 논문 PDF는 접속 실패로 본문 수치(읽기 시간 값)를 확인하지 못했다. 프로젝트 페이지만 인용했다.
