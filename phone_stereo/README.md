# 스마트폰 후면 카메라 → USB → PC

Galaxy S24 SM-S921N / Android 16에서 직접 확인한 구성이다. 휴대전화에 설치하는 `Phone Stereo` 앱은 Camera2의 logical camera 하나에 물리 출력 2~3개를 연결한다. PC는 ADB USB 포워딩으로 원본 YUV(NV21)를 받아 표시하고 NVIDIA GPU에서 YOLO를 실행한다.

## 실행

USB 디버깅을 허용한 폰을 연결하고 저장소 루트에서 실행한다. 카메라를 점유하는 기존 `tools/phonecam.sh`/scrcpy 실행은 먼저 종료한다. 기본 후면 렌즈 ID는 이번 S24에서 확인한 값이며, 다른 기기는 `--list`로 조회해야 한다.

```bash
# 최초 설치 또는 Android 앱 소스 변경 후 빌드·재설치 및 렌즈 조회
./venv/bin/python z_phone_stereo.py --install --list

# 기본: 광각 5 + 초광각 2, 640×480, 30 FPS 요청, PC에서 seg + bbox 표시
./venv/bin/python z_phone_stereo.py --fps 30 --yolo both

# bbox 전용 YOLO 모델
./venv/bin/python z_phone_stereo.py --fps 30 --yolo bbox

# 세 렌즈 동시 표시: 광각 5 + 초광각 2 + 망원 6
./venv/bin/python z_phone_stereo.py --physical 5 2 6 --yolo both

# 시간 제한 검증, 화면 표시 없이 프레임 통계 저장
./venv/bin/python z_phone_stereo.py --headless --duration 20
```

폰의 앱 화면은 켜두어야 한다. USB 전송용 포트는 폰과 PC의 loopback 8765이며 외부 네트워크에 카메라를 공개하지 않는다. `Q`/`Esc`/`Ctrl-C`로 종료하면 이 앱의 카메라와 ADB 포워딩을 해제한다. `S`는 타임스탬프가 맞는 원본 PNG 묶음을 `phone_stereo/runs/<실행시각>/captures/`에 저장한다. 종료 시 마지막 묶음과 `summary.json`도 실행 디렉터리에 저장한다.

기본 미리보기는 센서 출력 방향 그대로이며 회전·리사이즈하지 않는다. 추론 결과는 별도 창의 **추론에 사용된 프레임**에 그린다. 느린 추론 결과를 현재 라이브 프레임 위에 잘못 겹치지 않는다. `--yolo both`는 segmentation 모델 한 번에서 나온 mask와 bbox를 함께 표시하며, `--yolo bbox`는 detection 모델을 사용한다.

## 직접 확인한 결과

| 구성 | 조건 | 결과 |
|---|---|---|
| 광각 5 + 초광각 2 | 640×480, 15 FPS, 15초 | 각각 217프레임, 217쌍, 약 15 FPS |
| 광각 5 + 초광각 2 + 망원 6 | 640×480, 15 FPS, 12초 | 각각 171/172/171프레임, 171묶음, 약 15 FPS |
| 두 렌즈 + PC GUI + YOLO seg/bbox | 640×480, 30 FPS, 25초 | 736/737프레임, 735쌍, 약 30 FPS, `cuda:0` 추론 확인 |
| 두 렌즈 + bbox 전용 모델 | 640×480, 30 FPS, 20초, `venv_ffs` | 582쌍, GPU 추론 447회, 중앙값 11.26ms |

두 렌즈+YOLO 시험의 마지막 추론은 약 16.7ms였다. 전체 평균이나 FFS 포함 시간은 아니다. 수신 이미지들은 서로 다른 픽셀 배열이며 렌즈별 화각을 갖는다. 위 검증은 단기 실행이며 장시간 발열·배터리·고해상도 성능을 보증하지 않는다.

관측된 Image timestamp 차이는 0ms지만 HAL의 sensor sync 등급은 `APPROXIMATE`다. 동일한 보고 timestamp가 정밀한 물리 노출 동기화를 입증하지는 않는다. 다른 시점 프레임을 재사용하지 않으며 CLI 기본 35ms 허용차를 넘으면 묶음에서 제외한다. `--max-skew-ms`로 변경할 수 있다.

## FFS 객체 depth

`--calib-stereo`를 주면 PC FFS 추론을 활성화하고 기존 `lib_ffs`, `lib_rectify`, `z_object_depth`의 seg/bbox 집계 코드를 재사용한다. FFS는 앞의 두 렌즈만 사용한다. 실행 환경은 `venv_ffs`다.

```bash
./venv_ffs/bin/python z_phone_stereo.py --yolo both \
  --calib-stereo calib_phone_pair.json --iters 4
```

보정 파일 `calib_phone_pair.json` 은 `z_phone_calib.py` 가 만든다 (2026-09-22, Claude). 출처는 세 단계다.

| 단계 | 명령 | `scale_status` | 의미 |
|---|---|---|---|
| 공장값 | `./venv/bin/python z_phone_calib.py --factory` | `factory_unverified` | Camera2 공장 포즈 그대로. 실쌍에서 정류 dy 2~5px → 쓰지 말 것 |
| 회전 보정 | `./venv/bin/python z_phone_calib.py --refine phone_stereo/runs/<번들…>` | `factory_baseline_refined` | 장면 SIFT 대응점으로 roll/pitch + 광각 fx 스케일 보정(yaw 공장 고정). dy 는 맞지만 yaw 편향으로 거리가 11 % 짧았다 |
| 실측 1개 | `./venv/bin/python z_phone_calib.py --known <번들> --known-m <m> --roi x y w h` | `known_distance_pinned` | 거리를 아는 물체의 ROI 시차로 yaw 고정. 보드 없을 때 대안 |
| 체커보드 | `./venv/bin/python z_phone_calib.py --board phone_stereo/runs/<보드 번들> --screen laptop --ruler-mm <실측> --size 960 720 --target-size 640 480 --base calib_phone_pair.json` | `metric` (rms≤1px, 뷰≥8, baseline 공장값 ±15%) | **현재 저장소 파일**: RMS 0.26 px, 25뷰, baseline 15.53 mm. 보드 PnP 거리 대비 FFS 오차 −1.8 %(0.34~0.45 m). 촬영은 `--still --board-hud` 로 |

**초점은 고정해야 한다.** 광각 5 는 AF 렌즈라 자동초점이면 초점거리가 프레임마다 0.3~0.7% 변해 정류가 1~2px 씩 흔들린다. 앱은 기본으로 `CONTROL_AF_MODE_OFF` + `LENS_FOCUS_DISTANCE` 1 m 로 고정하며(`--focus-m`, 0 이면 자동초점), 캘리 번들의 초점값이 json 에 기록되고 실행값과 다르면 객체마다 `focus_mismatch` 경고가 붙는다. 초광각 2 는 고정초점이다.

공장값 변환 규약(NDK 문서 인용은 `lib_phone_calib.py` 머리): `X_B = R_B R_Aᵀ X_A + R_B(t_A − t_B)`, K 는 active array→출력 해상도 비례, 왜곡은 `[κ1,κ2,κ4,κ5,κ3]`. 초광각 2 가 왼쪽, 광각 5 가 오른쪽(Tx = −15.76mm)이라 파일의 `phone.physical` 은 `["2","5"]` 다. `--physical 5 2` 로 받아도 수신기가 같은 두 렌즈를 확인한 뒤 순서를 뒤집어 쓴다. 다른 렌즈(6)나 다른 해상도는 거부한다.

```bash
# 저장 번들로 depth 만 (폰 불필요): 번들 폴더에 depth_both.png / depth.json 생성
./venv_ffs/bin/python z_phone_stereo.py --physical 2 5 --calib-stereo calib_phone_pair.json --yolo both \
  --offline phone_stereo/runs/probe_now
# 라이브 FFS + seg/bbox
./venv_ffs/bin/python z_phone_stereo.py --physical 2 5 --calib-stereo calib_phone_pair.json --yolo both
```

직접 확인 (RTX 4060, venv_ffs, 640×480, 초점 1 m 고정): 오프라인 — 사람 seg 0.395/0.412 m (bbox 는 사람이 화면 대부분이라 배경 혼입 → seg 를 볼 것). 라이브 25초 — 367쌍 15.0 fps, FFS+YOLO(seg+bbox) 80회, 중앙값 258 ms, 첫 쌍 dy 0.32px·인라이어 0.95, 벽시계 1.80 m·183×60 mm.

**HAL의 동기 등급은 `APPROXIMATE`**이므로 동일한 논리 timestamp만으로 동시 노출을 보장할 수 없다. 앱은 `Image.getTimestamp()`와 **정확히 같은 논리 `SENSOR_TIMESTAMP`**의 촬영 결과를 찾아, 해당 렌즈의 물리 `SENSOR_TIMESTAMP`를 전송한다. 이미지/결과 콜백은 어느 쪽이 먼저 와도 연결하며, 결과 또는 물리 timestamp가 없으면 이전 오프셋을 재사용하지 않는다. 대기 항목은 24개/1초로 제한한다. Android `RESULT` 로그의 `matched`, `unmatched_dropped`로 연결 및 대기 이미지 폐기 수를 확인한다.

PC는 첫 렌즈 프레임 시각을 기준으로 다른 렌즈의 앞뒤 후보가 도착할 때까지 기다린 뒤 가까운 후보를 선택한다(보통 최대 한 프레임의 추가 대기). 선택한 묶음의 최대 시각차가 `--max-skew-ms 35`를 넘으면 제외하고 프레임을 재사용하지 않는다. `--raw-ts`는 논리 timestamp를 그대로 보내는 비교용 옵션이다. 이 변경은 메타데이터 연결과 프레임 선택을 바로잡으며, 센서 노출 자체를 동기화하지 않는다. 빠른 물체 운동과 롤링셔터 오차는 남을 수 있다.

이전 구현의 중앙값 6ms / p95 14ms는 **최신 오프셋을 재사용하던 코드의 과거 측정치**다. 새 구현의 결과로 해석하지 않는다. `frame_timing` 센서값(물리별 오프셋 ms, 노출, 프레임 주기, 롤링셔터 skew)은 계속 `summary.json`/`pair.json`에 기록된다. 캘리 촬영은 `--still --board-hud`로 정지 순간을 사용한다.

연결 로직 회귀 테스트: `./venv/bin/python -m unittest phone_stereo.test_receiver phone_stereo.test_timestamp_matcher` (Java 테스트에 JDK 필요).


**거리 정확도 한계**: baseline 15.76 mm 라 1 m 에서 시차가 7px 뿐이다. 시차 0.5px 오차 → 깊이 오차 ≈ Z²×0.070 (0.5 m 18 mm, 1 m 70 mm, 2 m 0.28 m). 캘리 yaw 0.1° 오차는 시차 0.76px 편향 = 1 m 에서 11%. `metric` 이 아닌 상태의 결과에는 객체마다 `scale_not_validated:<status>` 경고가 붙는다.

## 폰 센서 (IMU·기압)

앱이 자이로·중력·선가속도·게임 회전벡터·회전벡터(125 Hz), 기압(12.5 Hz)을 같은 소켓으로 보낸다(magic `SENS`, payload float32). 카메라와 같은 elapsedRealtimeNanos 시계라 프레임 시각의 센서값을 찍어 `pair.json`의 `imu` 와 depth 보고서 `imu`/`imu_raw` 에 넣는다. `summary.json` 의 `imu` 에 센서별 표본수·주기, `clock_gap_ms` 에 시계 일치 확인값이 있다.

- **중력 정렬**: 객체 `center_world_m` (x 카메라 오른쪽 수평, y 수평 전방, z 위), `height_rel_camera_m`, 보고서 `imu.pitch_deg/roll_deg`.
- **이동 스테레오** (`z_phone_motion.py`): GUI 에서 `S` 로 A 저장 → 폰을 10~20 cm 옆으로 옮겨 정지 → `S` 로 B 저장(10초 안에) → `./venv/bin/python z_phone_motion.py --a <A> --b <B>` (회전은 회전벡터, 이동방향은 영상, 크기는 15.76 mm 스테레오, 가속도 적분은 교차검증). `--dense` (venv_ffs) 면 이동 쌍을 정류해 FFS+seg 를 돌린다. 정지 상태 두 번들은 `baseline_unreliable` 로 거부된다.
- IMU 는 두 렌즈 사이 기하(yaw·baseline)를 관측하지 못한다 — 그건 `--known`/`--board` 의 몫이다.

## 파일과 검증

- `phone_stereo/android/src/com/camera/dualstream/MainActivity.java`: Camera2 촬영·물리 ID 조회·NV21 전송.
- `phone_stereo/build_android.py`: Google Android SDK 35 플랫폼과 build-tools 35.0.0의 고정 다운로드/체크섬 검증, Java APK 빌드. 다운로드 도구와 개발 서명 키는 Git에 저장하지 않는다.
- `lib_phone_stereo.py`: 분할 TCP 패킷 처리·타임스탬프 pairing·수신 통계.
- `z_phone_stereo.py`: 설치·시작·PC 표시·GPU 추론·원본 저장·종료.
- `lib_phone_depth.py`: 폰 쌍의 보정값 검증 및 기존 FFS depth 파이프라인 연결.
- `lib_phone_calib.py` / `z_phone_calib.py`: 공장 CameraCharacteristics → OpenCV 스테레오 캘리, 장면 회전 보정, 체커보드 stereoCalibrate, `--known`, `--check`.
- `lib_phone_motion.py` / `z_phone_motion.py`: 회전벡터 + 영상 + 스테레오 스케일로 폰 이동 두 시점을 넓은 baseline 스테레오로.
- `phone_stereo/cameras_s24.json`: 이 S24 의 `--list` 결과 사본 (runs/ 는 git 에 없으므로).
- `phone_stereo/test_receiver.py`: 잘린/손상된 패킷, 프레임 누락·재사용 방지, 다른 카메라 보정값 거부 테스트.

```bash
./venv/bin/python -m unittest discover -s phone_stereo -p 'test_*.py' -v
```

구현 근거: [Android Camera2 multi-camera](https://developer.android.com/media/camera/camera2/multi-camera), [Image.Plane stride](https://developer.android.com/reference/android/media/Image.Plane), [Image timestamp](https://developer.android.com/reference/android/media/Image#getTimestamp()), [FFS 공식 코드](https://github.com/NVlabs/Fast-FoundationStereo).

## RGB·깊이·3D·IMU 통합 뷰어

```bash
cd /home/karma/camera
./venv_ffs/bin/python z_phone_stereo.py \
  --physical 2 5 --calib-stereo calib_phone_pair.json --yolo seg \
  --rgbd-viewer --out phone_stereo/runs/rgbd_view
```

`--yolo seg`는 객체 인식을 수행하고 RGB 패널에 표시한다. 객체별 3D 모델 생성·맵 누적 기능은 제거했다. 현재 **전체 장면**의 RGB/깊이/컬러 점군/IMU를 표시한다. [RealSense D435i의 RGB-D·IMU 보기](https://github.com/realsenseai/librealsense/blob/master/doc/d435i.md)를 참고한 폰 전용 뷰어이며, 입력은 기존 폰 스테레오와 FFS 추정 깊이다. RealSense SDK 장치 에뮬레이션은 하지 않는다.

한 화면의 RGB·깊이·점군은 같은 완료된 추론 결과를 사용하고 IMU도 해당 왼쪽 렌즈 노출 시각에 대응한다. 카메라 수신 FPS와 별개로 추론이 끝날 때 갱신된다. 컬러 영상과 깊이는 정류된 왼쪽 영상 좌표로 정렬되어 있다.

- 상단 RGB/깊이 클릭: 해당 픽셀의 rectified Z, 렌즈에서의 거리, 원래 왼쪽 렌즈 좌표계 XYZ(m). 무효 깊이는 `invalid depth`.
- 왼쪽 아래 3D 패널: 드래그 회전, 휠 확대, Shift+드래그 이동, R 초기화.
- C: 점군 RGB/깊이 색 전환. F: 표시 프레임 고정/해제(카메라와 추론은 계속).
- IMU 패널: 자이로 rad/s, 중력 제거 선가속도 m/s², 중력 m/s², 센서-노출 시간차, 게임 회전벡터의 기기 방향 축.
- E: 현재 표시 프레임을 `rgbd_captures/<timestamp>/`에 저장. Q: 종료.

`rgbd/`에는 약 5초 간격과 종료 시 마지막 표시 프레임을 저장한다. `rgb.png`, `depth_color.png`, `viewer.png`, `depth_mm.png`(uint16, 1=1mm, 0=무효), `rgbd.npz`(float32 깊이 m·valid·원래 렌즈 좌표 점·K·R1), `scene.ply`, 회전 가능한 `scene.html`, `frame.json`(시각/단위/좌표계/IMU). PNG의 깊이는 정류 좌표 Z이고 PLY의 점은 원래 왼쪽 렌즈 좌표라 회전 R1이 메타데이터에 포함된다. IMU 벡터는 Android 기기 축이며 선가속도는 중력을 포함한 원시 가속도와 구분한다.

`--depth-range 0.3 3`은 깊이 색 범위 및 표시/PLY 점군의 Z 범위를 정한다. NPZ와 uint16 깊이 PNG에는 이 표시 범위로 잘라내지 않은 유효 깊이가 들어간다. `--headless`에서도 파일 생성 가능. `--offline <bundle>`과 함께 쓰면 `--out` 아래 `rgbd/<bundle 이름>/`으로 저장한다.
