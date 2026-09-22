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

관측된 Image timestamp 차이는 0ms지만 HAL의 sensor sync 등급은 `APPROXIMATE`다. 동일한 보고 timestamp가 정밀한 물리 노출 동기화를 입증하지는 않는다. 다른 시점 프레임을 재사용하지 않으며 기본 10ms 허용차를 넘으면 묶음에서 제외한다. `--max-skew-ms`로 변경할 수 있다.

## FFS 객체 depth

`--calib-stereo`를 주면 PC FFS 추론을 활성화하고 기존 `lib_ffs`, `lib_rectify`, `z_object_depth`의 seg/bbox 집계 코드를 재사용한다. FFS는 앞의 두 렌즈만 사용한다. 실행 환경은 `venv_ffs`다.

```bash
./venv_ffs/bin/python z_phone_stereo.py --yolo both \
  --calib-stereo calib_phone_pair.json --iters 4
```

**현재 폰 렌즈 쌍의 보정 파일은 아직 없다. 따라서 이 폰 입력으로 FFS 거리 추론을 실행·검증한 상태는 아니다.** 기존 노트북/폰 보정값을 대신 넣지 않는다. 파일에는 기존 stereo 형식의 `K1, dist1, K2, dist2, R, T, image_size`와 입력 ID를 검증하는 아래 필드가 필요하다.

```json
"phone": {"logical": "0", "physical": ["5", "2"]}
```

`R,T`는 `X2 = R·X1 + T`, T는 미터다. 렌즈 순서·수평 정류 부호가 맞지 않으면 기존 정류 코드가 거부한다. 보정에 맞춰 `--physical` 순서도 바꿔야 한다. 입력 해상도·렌즈 ID가 일치해야 하며 실제 거리 스케일을 검증한 경우에만 `scale_status: "metric"`로 기록한다. FFS 속도와 거리 정확도는 이 보정 완료 후 별도로 측정해야 한다.

## 파일과 검증

- `phone_stereo/android/src/com/camera/dualstream/MainActivity.java`: Camera2 촬영·물리 ID 조회·NV21 전송.
- `phone_stereo/build_android.py`: Google Android SDK 35 플랫폼과 build-tools 35.0.0의 고정 다운로드/체크섬 검증, Java APK 빌드. 다운로드 도구와 개발 서명 키는 Git에 저장하지 않는다.
- `lib_phone_stereo.py`: 분할 TCP 패킷 처리·타임스탬프 pairing·수신 통계.
- `z_phone_stereo.py`: 설치·시작·PC 표시·GPU 추론·원본 저장·종료.
- `lib_phone_depth.py`: 폰 쌍의 보정값 검증 및 기존 FFS depth 파이프라인 연결.
- `phone_stereo/test_receiver.py`: 잘린/손상된 패킷, 프레임 누락·재사용 방지, 다른 카메라 보정값 거부 테스트.

```bash
./venv/bin/python -m unittest discover -s phone_stereo -p 'test_*.py' -v
```

구현 근거: [Android Camera2 multi-camera](https://developer.android.com/media/camera/camera2/multi-camera), [Image.Plane stride](https://developer.android.com/reference/android/media/Image.Plane), [Image timestamp](https://developer.android.com/reference/android/media/Image#getTimestamp()), [FFS 공식 코드](https://github.com/NVlabs/Fast-FoundationStereo).
