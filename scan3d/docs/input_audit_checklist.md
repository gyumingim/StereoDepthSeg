# 입력 출처 점검표 — "이 값은 내 손에 오기 전에 누가 손댔나?"

새 데이터 출처(새 앱·새 녹화 형식·새 센서·새 기기)를 쓰기 전, 그리고 큰 단계(학습·정답 만들기)를 시작하기 전에 확인한다.
확인 방법과 날짜를 칸에 적는다. 확인 안 된 칸은 결과 보고에 "가정"으로 적는다.

## 1. IMU (Android)
| 항목 | 함정 | 확인 방법 | 상태 |
|---|---|---|---|
| 자이로 종류 | `TYPE_GYROSCOPE`(4) 는 Android 가 온라인 바이어스를 빼서 준다 — S24 에서 0.07 °/s 계단, 16 분에 436 번 (2026-10-20 실측) | imu.csv 첫 열 type: 16 = 원본, 값 3~5 = Android 추정 바이어스 | arlat HsRecorder 16·35 추가 (2026-10-20, 재설치 전) |
| 가속도 종류 | `TYPE_ACCELEROMETER`(1) 도 보정판. S24 는 추정값 0 (손대지 않음, 실측) | type 35 의 값 3~5 | 실측 |
| 표본 속도 | 요청(FASTEST)과 실제가 다름. S24 acc 499 Hz, gyro 319 Hz (imu_shell 기준) | 시각 차이 분포 | |
| 절전·화면 꺼짐 | 화면이 꺼지면 자이로가 약 1 s 씩 끊김 (시간의 36 %) | 시각 차이 > 10 ms 인 구간 비율 | 실측 |
| 시각 기준 | `event.timestamp` = elapsedRealtimeNanos 계열인지, 카메라 SENSOR_TIMESTAMP 와 같은 시계인지 | 기기 SENSOR_INFO_TIMESTAMP_SOURCE, batch_sync dt | dt −1.6~−2.9 ms |
| 공장 보정 | 원본(16)에도 칩 공장 보정은 들어 있음 (고정이라 괜찮음) | — | |
| 온도 | 바이어스가 온도로 변함 | 정지·가열 시험 (docs/gyro_bias_stability.html) | |

## 2. 카메라 (Camera2)
| 항목 | 함정 | 확인 방법 | 상태 |
|---|---|---|---|
| OIS | 렌즈가 움직여 주점이 흔들림. 요청 OFF 를 무시할 수 있음 | CaptureResult `LENS_OPTICAL_STABILIZATION_MODE` 를 프레임마다 기록 | 요청만 OFF |
| EIS | 영상을 잘라 흔들림 보정 → 프레임마다 위치가 바뀜 | CaptureResult `CONTROL_VIDEO_STABILIZATION_MODE` | 요청만 OFF |
| 왜곡 보정 | 표준 `DISTORTION_CORRECTION` 은 S24 미지원, 삼성 전용 `samsung.android.control.lensDistortionCorrectionMode` 가 있음 | dumpsys media.camera, 결과 태그 기록 | 마지막 요청 0 (추정 꺼짐) |
| 초점 | 자동 초점이면 f·왜곡이 바뀜 | LENS_FOCUS_DISTANCE 결과 기록 | 2.22 dpt 고정 |
| 노출·롤링셔터 | 프레임 시각 = 노출 가운데인지, skew 값 | results csv | 기록함 |
| 잡음 제거·선명화 | 모서리 위치가 조금 밀릴 수 있음 | NOISE_REDUCTION / EDGE 모드 기록 | 기본값 |
| 프레임 누락 | 고속 촬영에서 빠진 프레임 | frame_number 연속성 | |

## 3. 정답(GT)·처리 결과
| 항목 | 함정 | 확인 방법 |
|---|---|---|
| 렌즈 보정 | 초점·해상도·왜곡 보정 상태가 녹화와 같아야 함 | lens.json 의 focus_dpt, size |
| 정답이 학습 대상과 독립인지 | IMU 를 쓴 정답으로 IMU 를 채점하면 스스로 채점 | 평가용 정답은 카메라만 |
| 결과 파일이 입력보다 새로운지 | 앞 단계가 바뀌었는데 옛 캐시 사용 | 캐시 키·mtime |

## 4. 처음 쓰는 출처의 물리 상식 시험 (학습 전에)
- 가만히 두기 5~20 분: 자이로 ≈ 상수(바이어스), |가속도| ≈ 9.81, 계단·끊김 없음.
- 한 축으로만 천천히 돌리기: 다른 축 자이로 ≈ 0, 회전 각 적분 = 실제 각.
- 정지 → 이동 → 정지: 카메라 정답과 IMU 적분이 같은 방향·크기.
