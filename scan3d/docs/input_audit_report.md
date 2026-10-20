> **검증 결과 반영 (2026-10-20, docs/input_audit_verify.md)**: OIS 무시 의심 = **틀림** (고속 세션에서도 적용, 카메라/자이로 회전 비 0.986). EIS·노출/skew 상수 = **과장**. 시간차 이중 적용 = 맞지만 영향 0.17 mm (위험 낮음). 학습 1 ms 지연 = 맞음, 2.2 mm RMS (p95 4.8). 0.07 °/s = 1 LSB 는 맞지만 Android 보정판은 0~70 mdeg/s 톱니 오차를 남겨 온도 효과와 같은 크기 → 원본 사용 필요.

# 입력값 감사 보고서 (메타인지 감사) — 폰 VIO / IMU 보정 파이프라인

- 작성: 2026-10-20, 독립 감사 (파이프라인을 만들지 않은 사람). 코드·데이터는 읽기만 함, 폰은 읽기 전용 adb (dumpsys·getprop) 만 사용.
- 출발점: `docs/input_audit_checklist.md`. 그 표를 넘어 녹화 앱 → 파일 → PC 처리 → OpenVINS 까지 값 하나하나를 따라감.
- 실제 데이터 점검 녹화: `recordings/hs_20261019_131326` (1920x1080 @240 fps, 100 s, 과열로 정지).
- 폰 조회 (10:31, 열 시험 진행 중, 건드리지 않음): `dumpsys media.camera`, `dumpsys sensorservice`, `getprop`
  → SM-S921N (S24, Exynos 2400 `s5e9945`), Android 16 (SDK 36). IMU 칩 = **STM LSM6DSV** (`LSM6DSVTR`).
- 표의 "가정" = 이번에 확인하지 못한 것. 위험도 = 결과(정답·학습·VIO 점수)를 조용히 틀리게 할 가능성.

---

## 1. 폰 IMU 값 (HsRecorder → imu.csv)

| 값 | 어디서 만들어짐 | 숨은 처리/가정 | 확인 방법·근거 | 위험도 | 할 일 |
|---|---|---|---|---|---|
| 자이로 값 (type 4) | `HsRecorder.java:122` TYPE_GYROSCOPE, FASTEST | Android/삼성이 온라인 바이어스를 빼서 줌 (이미 알려짐). **새로**: 칩 분해능 1 LSB = 0.00122 rad/s = **0.07 °/s** (±2000 dps 범위). "0.07 °/s 계단" 은 바이어스 추정이 정확히 1 LSB 씩 바뀌는 것 | 녹화 자이로 최소 간격 0.00122 rad/s (세 축 모두); sensorservice 의 type 16 이벤트 `0.00122, 0.00244…` 와 바이어스 칸 `0.00122, 0.00244, 0.00122` | 높음 | 원본(16) 으로 바꿈 + 아래 "PC 가 여전히 1·4 만 읽음" 함께 고침. 정지 잡음 추정에 양자화(LSB/√12 ≈ 0.00035 rad/s) 를 고려 |
| 자이로 원본 (type 16), 가속도 원본 (35) | `HsRecorder.java:123-126` (2026-10-20 추가, 재설치 전) | 값 0-2 원본, 3-5 = Android 바이어스 추정. **PC 쪽은 아직 아무도 16·35 를 읽지 않음** | `imu_gt.py:31` `ACC, GYR = 1, 4`, `read_imu` (`imu_gt.py:156-167`) 가 1·4 만 고름. train_imu·batch_sync·spline_gt·vio_bench 모두 이 함수 사용 | 높음 | `read_imu` 에 원본 선택 옵션(16 의 v0-2, 35 의 v0-2). 새 녹화는 1·4·16·35 가 다 있으므로 어느 것을 썼는지 결과 파일에 기록 |
| 가속도 값 (type 1) | `HsRecorder.java:121` | 보정판. 원본 대비 바이어스 0 (sensorservice type 35 의 3-5 칸 = 0.00000 확인). 분해능 0.00479 m/s² (= 0.488 mg, ±16 g 범위) | 녹화 최소 간격 0.00479; dumpsys 이벤트 | 낮음 | — |
| IMU 표본 속도 | 요청 FASTEST, 칩 최대 500 Hz | 이 녹화는 가속도·자이로 모두 **498.9 Hz** (500 이 아님 — 칩 시계 0.2 % 느림 또는 허브 변환). 체크리스트의 "gyro 319 Hz" 와 다름 → 속도는 세션·동시 사용자에 따라 다르다고 봐야 함 | 시각 차이 2000.5~2008.3 µs (1~99 % 2002.1~2006.6), 빈틈·중복·역순 0. dumpsys: Gyro `selected = 2.00 ms`, `maxRate=500Hz` | 중간 | 녹화마다 실제 속도·빈틈을 meta 에 적고, 500 Hz 를 상수로 가정하는 곳(`train_imu.py:34 HZ=500`) 은 보간이므로 괜찮지만 잡음 환산엔 실측 dt 사용 |
| 가속도·자이로 시각 짝 | 칩/센서 허브 | **녹화 앱은 짝짓지 않음** (`HsRecorder.java:282-286` 은 event.timestamp 를 그대로 씀). 두 센서 시각이 표본마다 **완전히 같음** (49826/49826). dumpsys 의 원본 16·35 이벤트도 같은 시각 (`162056.207271015`) → 칩 FIFO 가 같은 ODR 틱으로 같이 찍는 것 (실제로 동시 표본으로 보임). 칩 안 필터 지연이 가속도·자이로에서 다를 수 있음 = 가정 | 위 수치, `sensorservice.txt` 378·920행 | 낮음 (지연 차이는 가정) | 가속도-자이로 상대 지연은 모델에 없음 → 회전+이동 녹화로 한 번 확인 |
| IMU 시각 (`event.timestamp`) | 센서 허브가 붙인 BOOTTIME 시각 | 카메라와 같은 시계(REALTIME) 인 것은 확인. 하지만 시각이 "표본 순간" 인지 "필터 출력 순간" 인지는 모름 → 카메라 대비 dt ≈ **−2.03 ms** 는 칩 필터 지연·허브 지연이 섞인 값. 녹화마다 −1.6~−2.9 ms 로 1.3 ms 흔들림 | `media.camera`: `android.sensor.info.timestampSource [REALTIME]`; `align.json` dt_ns −2028937 | 중간 | dt 를 녹화마다 다시 맞추는 지금 방식 유지. 흔들림 원인(설정·온도·추정 잡음) 분리 |
| 시작 첫 표본 | 칩 켜짐 직후 | 첫 32 표본(64 ms) 자이로 세 축이 모두 0.0012217305 로 같음 (가짜 값) | `audit` 출력 "rows with all 3 axes identical: 64" | 낮음 | 첫 0.1 s 버림 (지금은 카메라가 1.96 s 늦게 시작해서 영향 거의 없음) |
| 녹화 끝 | `stop()` 이 IMU 를 먼저 끔 | 마지막 84 ms 프레임에는 IMU 가 없음 | IMU 끝 − 마지막 프레임 = −84 ms | 낮음 | 끝 0.1 s 프레임 버림 |
| 이벤트 처리 스레드 | 센서·카메라 결과·인코더 콜백이 모두 같은 `hs` 스레드 (`HsRecorder.java:109,121-126,177,220`) | 스레드가 밀리면 소켓 버퍼(984 이벤트) 넘침 → 표본 유실 가능. 이 녹화에서는 유실 없음 | 빈틈 0; `Socket Buffer size = 984 events` | 낮음 | 센서를 별도 스레드로 (긴 녹화·과열 때 대비) |
| 중력 크기 | PC 상수 9.80665 | 정지 때 측정 |a| = 9.806 (이 녹화, 손에 든 "거의 정지") / 9.817 (오늘 dumpsys, 엎어 둔 폰). 서울 실제 g ≈ 9.80. 가속도 축척 오차 0.1~0.2 % 는 Calib 의 A 행렬만 흡수, BiasOnly·spline_gt 는 못 흡수 | `train_imu.py:149,373,379`, `spline_gt.py:42` | 낮음 | 정지 시험에서 6 면 |a| 측정 → 축척 확인 |
| 다른 녹화 앱들 | dualstream `MainActivity.java:44,288-298` (2500 µs = 400 Hz 요청, type 1·4), imu_shell `ImuLog.java:61-66` (1·4·16·35, 주기 0) | dualstream 은 원본을 기록하지 않음. imu_shell 은 shell 사용자로 돌아 다른 앱과 동시에 쓰면 HAL 이 가장 빠른 요청으로 맞춤 (서로의 속도에 영향) | 코드; dumpsys 에 ImuLog (uid 2000) 연결 확인 | 중간 | dualstream 에도 16·35 추가, 실제 속도 기록 |

## 2. 카메라 프레임 시각 (frames_cam5.csv, results_cam5.csv)

| 값 | 어디서 만들어짐 | 숨은 처리/가정 | 확인 방법·근거 | 위험도 | 할 일 |
|---|---|---|---|---|---|
| `physical_ts_ns` | 인코더 pts(MONOTONIC)+clockOffset 에서 2 ms 안의 가장 가까운 CaptureResult SENSOR_TIMESTAMP (`HsRecorder.java:158-162,268-276`), 나중에 `hs_fix_ts.py` 로 다시 짝지음 | pts 와 센서 시각 차이 중앙 +1 µs (−551~+138 µs) → 짝은 맞음. 못 찾은 34 장은 이전 차이로 추정 → 고친 뒤 7 장, 최대 22.6 µs 변경. **240 fps 에서 프레임 간격 4.17 ms 의 반이 2.08 ms** 라 2 ms 창은 여유가 거의 없음 (주석은 120 fps 기준 "8.3 ms") | `audit_rec.py` 출력; frames vs orig | 낮음 | 창을 프레임 간격의 1/3 로 |
| SENSOR_TIMESTAMP 의미 | HAL | REALTIME 시계, "첫 줄 노출 시작" (안드로이드 정의). 기기는 `android.sensor.readoutTimestamp [HARDWARE]` 지원 → 결과 키 `SENSOR_READOUT_TIMESTAMP` 로 실제 읽기 시작 시각을 받을 수 있는데 기록 안 함 | `media.camera` 1610행 | 중간 | 결과에 SENSOR_READOUT_TIMESTAMP 기록 → (readout − ts) = 노출 확인, skew 검증 |
| 결과 행이 프레임마다 2 개 | CaptureCallback | results_cam5.csv 42800 행 = 프레임 21400 의 정확히 2 배. 같은 프레임이 **17~20 µs 다른 시각으로 두 번** 보고됨 (첫 8 개 0…29166 µs, 다음 8 개 17…29184 µs). 인코딩된 것은 번갈아 한쪽. 어느 것이 "진짜" 인지 모름 (차이 20 µs 라 영향 작음) | `audit_rec2.py` 출력 | 낮음 | 원인 확인(미리보기 표면? HAL 버그?) — 가정 |
| `frame_number` | `r.getFrameNumber()` | 고속 세션은 버스트 8 장이 **같은 번호** (3,3,…,7,7,…). 체크리스트의 "frame_number 연속성으로 누락 확인" 은 **쓸 수 없음** | 결과 csv: 번호 5350 종류 × 8 행 | 중간 | 누락은 시각 간격으로만 판단 (체크리스트 고침) |
| `frame_idx` | 인코더 출력 순서 카운터 (`HsRecorder.java:164-165`) | 빠진 프레임이 있어도 **연속 번호** → 번호로는 누락이 안 보임 | 아래 "프레임 누락" | 중간 | 시각 간격 기반 누락 표시 열 추가 |
| **프레임 누락** | 카메라(열 제한) + 인코더 | 23492 장 중 **2092 장 (8.9 %) 빠짐**, 682 곳. 카메라 결과 자체가 1266 장 빠짐, 나머지 ~826 장은 카메라→파일 사이에서 빠짐. 앞 40 s 는 거의 없음 (처음 10 s 에 1 곳), **39.8 s 부터 급증** (마지막 10 s 에 129 곳), 한 번에 12.5~79 ms. 과열 정지(`thermal status 3`) 와 같은 흐름 | 프레임 간격 1·50·99 % = 4.15/4.17/20.9 ms, 최대 79.3 ms | 높음 (VIO 벤치) / 중간 (정답) | meta 에 누락 수 기록, VIO 벤치의 프레임 솎기를 번호 대신 시각으로 (아래 vio_bench), 열 시험 결과로 녹화 길이 제한 |
| `t_mid_ns` | `ts + exp/2 + skew/2` (`HsRecorder.java:164`, `hs_fix_ts.py:47`) | 가운데 줄 노출 가운데. 노출 4.002 ms 와 skew 2.9386 ms 가 **녹화 내내 상수** → HAL 이 실측이 아닌 고정값을 보고하는지 모름 (1080p 고속 모드는 센서 일부만 읽거나 묶어 읽음 — skew 가 그 모드 값인지 = 가정). 노출이 프레임 간격(4.17 ms) 의 96 % → 움직임 번짐 최대 | csv: 노출·skew 고유값 1 개씩 | 중간 | 위 READOUT 시각으로 skew 검증, 또는 깜박이는 LED 로 측정. evComp 로 노출 줄이기 시험 |
| clockOffset | `HsRecorder.java:119` 녹화 시작 때 한 번 (BOOTTIME − MONOTONIC) | 녹화 중 깊은 잠이 없다는 가정 (녹화 중엔 깨어 있음). 두 시계 읽기 사이 µs 오차 → 짝짓기 2 ms 창이 흡수 | meta `clock_offset_ns` 5123 s; pts 차이 중앙 1 µs | 낮음 | — |
| 초점 | `HsRecorder.java:199-202` AF OFF + 2.22 dpt | 결과 focus 2.222321 로 고정 확인. `lens_moving=1` 은 처음 8 행(0.03 s) 만 | results csv | 낮음 | — |

## 3. 카메라 영상(픽셀) · 렌즈

| 값 | 어디서 만들어짐 | 숨은 처리/가정 | 확인 방법·근거 | 위험도 | 할 일 |
|---|---|---|---|---|---|
| **OIS** | 요청 `LENS_OPTICAL_STABILIZATION_MODE_OFF` (`HsRecorder.java:196`) | **고속(constrained high-speed) 세션은 정해진 몇 가지 CONTROL_* 키만 따르고 나머지는 무시** 하도록 안드로이드 문서에 정해져 있음 (CameraDevice.createConstrainedHighSpeedCaptureSession 설명; LENS_OPTICAL_STABILIZATION_MODE 는 목록에 없음 — 기억 기반, 가정). 기기는 OIS 지원 `[0 1]` + 삼성 전용 `samsung.android.lens.info.availableOpticalStabilizationOperationMode [0 1]`. 결과 키 `opticalStabilizationMode` 는 있는데 **기록 안 함**. OIS 가 켜져 있으면 렌즈가 손떨림 반대로 움직여 영상이 자이로와 다르게 움직임 → 카메라-IMU 맞춤·RS 펴기에 직접 오차 | `media.camera` 436·466·483·1234행 | **높음** (가정) | 결과의 OIS 모드를 프레임마다 기록. 정지 보드 + 폰 손떨림 시험: 보드 자세 각속도 vs 자이로 크기 비 (OIS 면 카메라가 덜 돔) |
| EIS (영상 흔들림 보정) | 요청 VSTAB OFF (`:197`) | 기기에서 `videoStabilizationMode` 는 **세션 키** (`availableSessionKeys [65553]`) → 세션 만들 때 정해짐. 옛 API `createConstrainedHighSpeedCaptureSession` 은 세션 키를 못 넘김 → 기본값으로 세션이 만들어지고 요청 값은 나중에 적용(또는 무시) = 가정. 삼성 `VDIS Gyroscope`·`SuperSteady Gyroscope` 센서 존재 (EIS 용) | `media.camera` 515-516행; `sensorservice` 60·75행 | 중간 | SessionConfiguration(SESSION_HIGH_SPEED) + setSessionParameters 로 VSTAB OFF, 결과 VSTAB 모드 기록 |
| 왜곡 보정 | 요청 안 함 (기본값) | 표준 DISTORTION_CORRECTION 키 없음, 삼성 `lensDistortionCorrectionAvailableModes [0 1]` 존재 → 기본이 켜짐인지 모름. 자기 보정(rational) 이 흡수하지만 모드가 녹화마다 다르면 렌즈값 공유(`config/lens/hs_1920x1080_f2.22.json`) 가 틀림 | `media.camera` 1169행; HsRecorder 에 삼성 태그 없음 | 중간 | 삼성 태그를 0 으로 명시 + 결과 기록 |
| 잡음 제거·선명화·톤맵 | TEMPLATE_RECORD 기본값 (요청 안 함) | NR [0 1 2], EDGE [0 1 2] 지원. 고속 세션은 이 키를 무시할 가능성 (위와 같은 문서). 모서리 위치가 픽셀 아래 단위로 밀릴 수 있음 | `media.camera` 394·446행 | 낮음 | 결과에 NR/EDGE 모드 기록 |
| 어떤 물리 렌즈인가 | 논리 카메라 "0" (물리 5·6·2, 줌 0.6~10) | meta 의 `"physical": ["5"]` 는 **코드에 박힌 문자열** (`HsRecorder.java:312`), 실측 아님. 결과 키 `activePhysicalId` 기록 안 함 | `media.camera` 992행 `[5 6 2]`, 390행 zoomRatioRange | 낮음 (줌 1.0 기본) | 결과의 ACTIVE_PHYSICAL_ID 기록 |
| 잘라내기 (crop) | SCALER_CROP_REGION 기본, `croppingType CENTER_ONLY` | 4:3 센서(4080x3060) → 16:9 출력 = 가운데 자름. 공장 주점(2047.85, 1547.31) 을 자름·축척하면 (963.7, 548.2) → 자기 보정 (963.0, 546.9) 과 **잘 맞음**. 하지만 초점거리는 공장 2780×0.4706 = 1308 vs 자기 보정 **1267 (3 % 작음)** — 이유 모름 (초점 2.22 dpt, 왜곡보정 축척, rational 모델 상관 = 가정) | `media.camera` 424-425·944행; lens json | 낮음 | 공장값과 차이 원인 한 번 확인 (k1k2p 모델로 다시 보정해 f 비교) |
| 렌즈 왜곡 모델 | `lens_calib.py:30` rational 선택 | 계수가 매우 큼 (k1 27.9, k2 25.1, k3 −12.1, k4 27.8, k5 23.6, k6 −10.8) — 분자·분모가 서로 상쇄하는 모양. 보드가 덮지 않은 화면 가장자리에서는 값이 튈 수 있음. VIO 는 이 모델로 **전체 화면** 을 펴서 씀 (`vio_bench_hs.py:112-113`) | lens json `holdout_px` rational 0.2448 vs k1k2p_k3 0.2484 (차이 1.5 %) | 중간 | VIO 용은 k1k2p_k3 (차이 미미) 쓰거나, 펴진 영상 가장자리를 눈으로 확인 |
| IMU→카메라 축 (R_ci) | `imu_gt.align` + `batch_sync` (자이로-보드 맞춤) | **공장값으로 검산 가능**: `android.lens.poseRotation [0.7071 −0.7071 0 0]` (x,y,z,w) = [[0,−1,0],[−1,0,0],[0,0,−1]]. 측정 R_ci 와 같은 모양, 광축 둘레로 **1.25°** 차이 (0.022 rad) | `media.camera` 420-421행; `align.json` R_ci | 낮음 (좋은 소식) | 맞춤 결과를 공장값과 자동 비교 (2° 넘으면 경고) |
| 레버암 | 공장값 없음 | `poseTranslation [0 0 0]` + `poseReference PRIMARY_CAMERA` → 주 카메라 기준이라 0, IMU 위치 정보 없음. 학습(Calib.r) 으로만 추정 | `media.camera` 422-427행 | 낮음 | — |
| 영상 인코딩 | `HsRecorder.java:128-146` H.264 High, 160 Mbps 요청, I 프레임 1 s | 실제 평균 201 Mbps. **B 프레임 없음**, pts=dts, 순서 바뀜 없음 (확인). 색 범위 limited(16-235). cv2 의 `CAP_PROP_FPS` 는 218.4 (누락 때문에 평균) — 쓰는 곳 없음 | PyAV: `has_b_frames False`, 2000 패킷 pts≠dts 0, 키프레임 9 | 낮음 | — |
| 디코드 순서 = csv 순서 | `board_poses`, `vio_bench`, `spline_gt.load` 모두 "디코드 i 번째 = csv i 행" 가정 | 이 녹화에서는 **맞음**: 디코드 21400 = csv 21400, 컨테이너 시각 − csv pts = ±6 µs | `audit_vid.py` 출력 | 낮음 (확인됨) | 녹화마다 개수 비교를 자동 점검으로 |

## 4. PC 처리 단계

| 값 | 어디서 만들어짐 | 숨은 처리/가정 | 확인 방법·근거 | 위험도 | 할 일 |
|---|---|---|---|---|---|
| dt 부호 | `imu_gt.py:9` "IMU 시각 + dt = 카메라 시각" | batch_sync·train_imu·spline_gt 는 같은 부호로 일관 (`batch_sync.py:103`, `train_imu.py:112-113`, `spline_gt.py:294`) | 코드 | 낮음 | — |
| **OpenVINS 시간 오프셋 두 번 적용** | `vio_bench_hs.py:146,175` | 영상 시각을 이미 IMU 시계로 바꿔 보냄 (`tci = tc − dt`) **그리고** `timeshift_cam_imu = −dt` 도 넣음. OpenVINS 는 `t_imu = t_cam + timeshift` 로 한 번 더 더함 → 시작 오차 = −dt ≈ **+2.0 ms**. `calib_cam_timeoffset: true` (`config/mono/estimator_config.yaml:16`) 라 온라인으로 줄어들 수 있지만 처음 수 초와 짧은 녹화는 영향 | 코드 읽기 (ov_bridge 는 받은 t_ns 를 그대로 씀, `ov_bridge/main.cpp:303-356`). 효과 크기는 미측정 = 가정 | **높음** (VIO 벤치 비교) | 둘 중 하나만: 영상을 카메라 시각 그대로 보내고 timeshift=−dt, 또는 지금처럼 IMU 시각으로 보내고 timeshift=0. 고친 뒤 벤치 다시 |
| VIO 프레임 솎기 | `vio_bench_hs.py:143,171` `fi % k` | **프레임 번호** 로 솎음. 8.9 % 누락(뒤쪽에 몰림) 이면 "60 fps" 가 실제로는 고르지 않음 (최대 79 ms 빈틈) | 위 프레임 누락 | 중간 | 시각 격자(1/fps) 에 가장 가까운 프레임 고르기 |
| VIO 평가 정답 | `vio_bench_hs.py:200` imu_gt_rs 가 있으면 그것 | imu_gt_rs 는 **자이로로 롤링셔터를 편 정답** (`imu_gt.py:122-128`) + batch_sync 의 dt·R_ci·바이어스 = IMU 가 섞인 정답. VIO 는 같은 IMU 를 씀 → 조금 스스로 채점. 체크리스트 3 의 "평가용 정답은 카메라만" 과 어긋남 | 코드 | 중간 | 평가는 imu_gt(카메라만) 또는 imu_gt_spl (카메라만 스플라인) 로; rs 는 비교용 |
| VIO IMU 잡음 | `vio_bench_hs.py:51-62` (meas) / 127-129 (cfg) | 조용한 10 % 를 |ω| 로 고름 → 보정판 자이로(바이어스 계단 포함) 기준. 양자화 1 LSB 가 잡음의 큰 몫. random walk = 밀도/10 은 근거 없는 추정 | 코드 | 낮음 | 정지 시험(원본 16) 의 Allan 값으로 |
| VIO 영상 | `vio_bench_hs.py:112-113,182` | 왜곡 펴기 + 640x360 축소 (`getOptimalNewCameraMatrix(alpha=0)` → 새 K 를 그대로 OpenVINS 에), 회색 변환. 롤링셔터는 OpenVINS 에서 무시 (global shutter 가정, skew 2.9 ms) | 코드 | 중간 | OpenVINS 는 RS 모델 없음 → 빠른 움직임 오차의 일부임을 결과 해석에 명시 |
| 학습 IMU 격자 | `train_imu.py:111-113` | 498.9 Hz 표본을 500 Hz 격자로 **선형 보간** (빈틈 검사 없음 — 빈틈이 있으면 직선으로 메움). 가속도도 자이로와 같은 dt 로 옮김 (상대 지연 0 가정) | 코드 | 중간 | 50 ms 넘는 빈틈은 창에서 빼기 |
| **학습 적분의 반 표본 어긋남** | `train_imu.py:130-132, 261-264` | 표본 k 의 ω 를 구간 [t_k, t_k+1] 전체에 씀 (앞쪽 적분) → IMU 가 실제보다 **DT/2 = 1 ms 늦게** 들어간 것과 같음. batch_sync 는 구간 가운데 시각을 써서 (`batch_sync.py:103` `+0.5/HZ`) dt 를 맞췄으므로 학습 쪽만 1 ms 어긋남. 과거만 보는 CNN 은 늦음을 고칠 수 없음 (1 rad/s 에서 0.057°) | 코드 | 중간 | ω 를 (g[k]+g[k+1])/2 또는 격자를 반 칸 옮겨 쓰기 |
| 정답 프레임 시각 반올림 | `train_imu.py:164` | 프레임 시각을 2 ms 격자에 반올림 (±1 ms) | 코드 | 낮음 | 분수 위치 보간 (batch_sync 처럼) |
| 정답 거르기 | `train_imu.py:139` | 정답 회전과 **자이로** 적분이 3° 넘게 다르면 버림 → IMU 로 정답을 고름 (작은 자기 채점) | 코드 | 낮음 | 버린 수 보고 (이미 n_bad) |
| 중력 방향 | `train_imu.py:145-149` |ω|<0.05 rad/s 인 프레임 평균 → 크기 9.80665 로 고정, 그 뒤 `fit_gravity` 로 방향만 다시 맞춤 | 코드 | 낮음 | — |
| 자이로 바이어스 (RS 펴기) | `imu_gt.py:92` | batch_sync 가 20 s 마다 푼 바이어스의 **평균 하나** 를 뺌 (보정판 자이로라 Android 계단은 남음) | 코드 | 낮음 | 원본으로 바꾸면 절점 보간 사용 |
| 롤링셔터 방향 | `imu_gt.py:125`, `spline_gt.py:167` | 영상 위→아래 줄 = 읽는 순서, 영상은 센서 방향 그대로(회전 안 함, `sensor.orientation 90`) 가정. "부호 뒤집으면 더 나쁨" 으로 간접 확인 | `imu_gt.py:99-100` 주석 | 낮음 | READOUT 시각으로 직접 확인 |
| 보드 정의 | `imu_gt.py:30` 기본 `board_user_A4_6x6.json` / `spline_gt.py:41` **`board_monitor_6x6.json` 고정** | 두 파일 기본 보드가 다름. spline_gt 는 옵션이 없어 A4 보드 녹화에 쓰면 모서리 좌표가 틀린 채로 조용히 돌아감. 이 녹화의 렌즈 json 은 monitor 보드 | 코드; lens json `"board"` | 중간 | gt.npz 에 보드 이름 저장, spline_gt 가 그것을 읽게 |
| spline_gt IMU 항 | `spline_gt.py:289-298` | IMU 를 200 Hz 로 보간해 솎음 (흐림 거르기 없음 → 진동 에일리어싱, train_imu 주석이 경고한 그 문제). 중력 크기 9.80665 고정, 가속도 축척 없음 | 코드 | 낮음 (평가용 아님) | 500 Hz 그대로 또는 저역 거르기 후 솎기 |
| gt_smooth | `gt_smooth.py` | IMU 안 씀 (확인). 50 ms 빈틈에서 구간을 나눔 → 프레임 누락(최대 79 ms) 구간마다 잘림 | 코드 | 낮음 | — |
| recording.py | `recording.py:376-415` | type 을 그대로 보관, 정렬만. 새 녹화의 16·35 행도 같이 들어감 → 이걸 쓰는 실시간/재생 경로가 type 1·4 만 고르는지 확인 필요 = 가정 | 코드 | 낮음 | 재생 경로에서 type 필터 확인 |

## 5. 실제 데이터 점검 요약 (hs_20261019_131326)

```
IMU   type 1: 49826, type 4: 49826, 498.9 Hz, dt 2000.5~2008.3 us, 빈틈/중복/역순 0
      acc·gyro 시각 49826/49826 동일, 자이로 LSB 0.00122 rad/s, 가속도 LSB 0.00479 m/s²
      |a| (|ω|<0.02 rad/s, n 2427) 9.8061 ± 0.0916 m/s², 첫 64 ms 자이로 가짜 값
카메라 frames 21400 (meta 21400), 간격 중앙 4.1667 ms, 누락 2092 장 (8.9 %, 682 곳, 39.8 s 이후 몰림)
      노출 4.002 ms·skew 2.9386 ms 고정, pts−센서시각 중앙 +1 µs, fix 로 바뀐 프레임 7 (최대 22.6 µs)
      results 42800 행 = 프레임마다 2 번 (17~20 µs 차이), frame_number 버스트 8 장 같은 값
      초점 2.222321 고정, lens_moving 은 시작 0.03 s 만
영상  H.264 High, B 프레임 없음, 디코드 21400 = csv, 컨테이너 시각 − csv pts ±6 µs, 실제 201 Mbps
맞춤  dt −2.03 ms, R_ci ≈ 공장 poseRotation 과 1.25° 차이
```
명령: `venv_ffs/bin/python -I audit_rec.py|audit_rec2.py|audit_vid.py scan3d/recordings/hs_20261019_131326` (감사용 스크립트는 저장소 밖 임시 폴더), `adb shell dumpsys media.camera|sensorservice`, `getprop`.

---

## 6. 새로 발견한 함정 (위험 순)

1. **OIS 가 실제로 꺼졌는지 모름 (가정, 높음)** — 고속 세션은 CONTROL_* 일부 키만 따르는 것으로 문서화되어 있어 OIS OFF 요청이 무시될 수 있고, 결과 OIS 모드를 기록하지 않음. 켜져 있으면 카메라-자이로 일관성이 근본부터 깨짐. → 결과 키 기록 + 손떨림 시험.
2. **PC 가 원본 IMU 를 아직 안 읽음 (높음)** — HsRecorder 는 16·35 를 쓰기 시작했지만 `read_imu` 가 1·4 만 고름 (`imu_gt.py:31,156-167`). 모든 단계가 이 함수를 씀 → 녹화만 바꿔서는 오늘 고친 문제가 그대로.
3. **OpenVINS 시간 오프셋 두 번 적용 (높음)** — `vio_bench_hs.py:146,175`: IMU 시계로 바꾼 영상 시각 + timeshift=−dt. 시작 오차 ≈ 2 ms, 온라인 추정에 기댐.
4. **프레임 8.9 % 누락, 녹화 후반에 몰림 (높음/중간)** — frame_idx 는 연속이라 안 보임, frame_number 도 버스트마다 같아 못 씀. VIO 벤치는 번호로 솎아 실제 fps 가 고르지 않음. 과열과 같은 흐름.
5. **학습 적분이 1 ms 늦음 (중간)** — `train_imu.py` 앞쪽 적분 vs batch_sync 가운데 적분 → 같은 dt 를 쓰면서 반 표본 어긋남. 과거만 보는 CNN 이 못 고침.
6. **VIO 평가 정답에 IMU 가 섞임 (중간)** — imu_gt_rs (자이로 RS 펴기) 를 우선 사용 → 같은 IMU 를 쓰는 VIO 를 조금 스스로 채점.
7. **0.07 °/s 계단 = 자이로 1 LSB (중간, 해석)** — 칩(LSM6DSV) 분해능이 0.07 °/s 라서 바이어스 추정도 그 단위로 움직임. 정지 잡음·바이어스 안정성 해석에서 양자화를 따로 봐야 함.
8. **EIS 는 세션 키인데 옛 API 로 세션 생성 (중간, 가정)** — 요청의 VSTAB OFF 가 세션 설정에 반영되는지 모름. 삼성 EIS 전용 자이로 센서 존재.
9. **skew·노출이 상수로 보고됨 (중간, 가정)** — 실측인지 HAL 고정값인지 모름. 기기가 SENSOR_READOUT_TIMESTAMP 를 지원하므로 기록하면 검증 가능.
10. **spline_gt 보드 고정 (중간)** — `board_monitor_6x6.json` 고정, imu_gt 기본은 A4. 다른 보드 녹화에 조용히 틀림.
11. **rational 왜곡 계수가 매우 큼 (중간)** — 보드가 안 덮은 가장자리에서 VIO 용 펴기 지도가 튈 수 있음. holdout 차이는 1.5 % 뿐 → k1k2p_k3 로 충분.
12. **dt 가 녹화마다 1.3 ms 흔들림 (중간)** — IMU 시각이 칩 필터 지연을 포함; 원인 미분리.
13. 기타 (낮음): 결과 행 중복(17~20 µs), 첫 64 ms 자이로 가짜 값, 마지막 84 ms IMU 없음, meta 의 physical "5" 는 박힌 문자열, 공장 f 와 자기 보정 f 3 % 차이, 2 ms 짝짓기 창이 240 fps 반 간격과 거의 같음.

좋은 소식 (확인됨): IMU 빈틈·역순 없음, B 프레임 없음, 디코드 순서 = csv, pts↔센서 시각 짝 정확 (µs), 카메라·IMU 같은 REALTIME 시계, 측정 R_ci 가 공장 poseRotation 과 1.25° 안, 주점이 공장값과 1 px 안.

## 7. 체크리스트에 더할 것
- "frame_number 연속성" → **시각 간격 > 1.5 × 주기** 로 바꿈 (고속 세션은 frame_number 가 버스트 단위).
- 결과 키 프레임마다 기록: OIS 모드, VSTAB 모드, SENSOR_READOUT_TIMESTAMP, ACTIVE_PHYSICAL_ID, NR/EDGE 모드, 삼성 왜곡 보정 모드.
- 녹화마다: 실제 IMU 속도, 프레임 누락 수·위치, 디코드 개수 = csv 행 수.
- 측정 R_ci 와 `lens.poseRotation` 비교 (2° 넘으면 경고).
- 평가용 정답에 IMU 가 들어갔는지 (rs, 거르기, dt) 결과 파일에 표시.
