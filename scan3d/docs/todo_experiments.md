# 나중에 할 폰 실험 (2026-10-20 정리)

근거: `docs/gyro_bias_stability.html`, `docs/temp_history_bias.html`, `docs/input_audit_report.md`.

## 1. 6면 놓기 — 가속도계 바이어스·축척 절댓값
- 왜: 한 방향 정지로는 바이어스와 중력(기울기)을 나눌 수 없다.
- 방법: 단단한 수평 책상에서 각 20 s, 손 뗀 채로. 화면 위 → 화면 아래 → 위 모서리 위 → 아래 모서리 위 → 왼쪽 옆면 위 → 오른쪽 옆면 위.
  세울 때는 책·블록에 기댄다. 면 바꿀 때 5 s 움직임. 한 번 더 반복하면 좋음. 카메라 끔, IMU 만 (`tools/imu_shell/`).
- 분석: `../venv_ffs/bin/python tools/imu_shell/analyze_static.py <녹화폴더> --six-face` → `runs/six_face_result.json`.
- 시간: 약 3~5 분.

## 2. 카메라 없이 데우기 — 온도 효과와 카메라 전원 효과 나누기
- 왜: 10-20 가열은 "데우기 = 240 fps 카메라 켜기" 라 카메라 켬 자체 효과(자이로 x +5.8, z −5.2 mdeg/s, 가속도 z +3 mg — 정정 15:20: **불확실**, 온도원을 PA 로 바꾸면
  z 가 +12.6 으로 부호가 바뀜, input_audit_verify.md 2절)와
  온도 효과가 섞였고, 가열 사건도 사실상 1 번뿐이라 시간 지연·계수가 추정.
- 방법: 폰 정지(화면 아래), IMU + 온도(CP/PA/AP/배터리) 기록, CPU 부하(또는 충전)로 천천히 데우고 15 분 식히기.
  배터리 42 °C·발열 경고·자동 정지에서 멈춤. 가능하면 한 번 더 반복(가열 속도 다르게).
- 확인할 것: 자이로 b = a + k·T_CP (지금 온도, 필터 없음), 가속도 x·y b = a + k·T_battery 계수가 10-20 값과 같은지.

## 3. 원본 IMU 새 녹화 — 앱 재설치 뒤
- 앱(arlat HsRecorder, **app_version hs-20261020-temps**, versionName 0.2-hs-20261020): 코드·빌드 끝, **설치·시험 남음**.
  - [x] type 16·35 원본 저장 (imu.csv 8 열), 센서는 전용 스레드(hs-imu)로 (카메라·인코더 스레드와 분리).
  - [x] temps.csv: 2 s 마다 확인, **10 s 마다 한 줄 + 값이 바뀌면 바로 한 줄** (reason = periodic / change / status).
    열: elapsed_ns, wall_ms, phone_uptime_s, pc_unix(폰 시계), battery_dC, thermal_status, headroom_10s, cam_on, AP, BAT, CP, PA,
    SKIN, USB, reason, other(name=값;…). `analyze_static.read_temps()` 가 그대로 읽음 (합성 파일로 확인).
    출처: 배터리 인텐트·thermal status·headroom 은 앱에서 됨. **thermal HAL(AP/CP/PA/SKIN…)은 IThermalService 리플렉션으로 시도 —
    DEVICE_POWER 권한 필요라 거부될 가능성 큼 (가정, 폰에서 확인 전)**, sysfs thermal_zone·HardwarePropertiesManager 도 시도.
    무엇이 됐는지는 meta.json `temp_sources`. HAL 이 거부되면 AP/CP/PA 칸은 비고, 그 온도는 adb `dumpsys thermalservice` (run_thermal.sh) 로만.
  - [x] results_cam5.csv 끝에 열 추가: ois_mode, eis_mode, nr_mode, edge_mode, readout_ts_ns (API 34+ onReadoutStarted, 하드웨어
    지원 때만), distortion_mode (AOSP), samsung_ldc_mode (삼성 vendor tag, 못 읽으면 null). 기존 8 열 그대로 (hs_fix_ts 확인).
  - [x] 요청: OIS·EIS OFF 유지, AOSP DISTORTION_CORRECTION_MODE OFF(제공될 때), `samsung.android.control.lensDistortionCorrectionMode = 0`
    (태그 없으면 건너뜀, 이 태그로 burst 가 실패하면 빼고 다시 시작). 결과는 meta.json `capture`.
- 설치 (PC, 폰 연결 후): `adb install -r C:\Users\a3162\AppData\Local\Temp\arlat\arlat_hs-20261020-temps.apk`
  (UNC 경로 금지, CLAUDE.md 9절. 서명 불일치면 `adb uninstall com.scan3d.arlat` 뒤 설치).
- 먼저 **10 초 시험 녹화** (vio_bench_server hs 시작, 끝나면 바로 정지 — 발열) 뒤 확인:
  - [ ] imu.csv 에 type 1·4·16·35 모두 있고, 16·35 행은 값 8 개(type, t_ns, v0..v5), 1·4 는 5 개.
  - [ ] temps.csv 행이 약 10 s 간격 (10 s 녹화면 2~3 줄 + change 줄), battery_dC 값 있음, meta.json `temp_sources` 에서 thermal_hal 결과 확인.
  - [ ] results_cam5.csv: ois_mode = 0, eis_mode = 0, samsung_ldc_mode = 0 (null 이면 태그 못 읽음 → `capture` 확인), distortion_mode 0 또는 null.
  - [ ] meta.json `app_version` = hs-20261020-temps, frames_unmatched 작음, 프레임 수 ≈ 240 × 초 (센서 스레드 분리 뒤 드롭 없는지).
- 본 녹화: 모니터 보드, 4~6 개 × 3~5 분, **시작과 끝에 5~10 s 폰 내려놓기** (녹화별 바이어스·녹화 중 변화 확인).
- 녹화 뒤 물리 보정에 온도 항 넣어 ~~0.99 cm 가 줄어드는지~~ **VIO 결과(ATE·초기화 성공)가 좋아지는지** 비교.
  (정정 15:20: 토론 합의로 3 초 IMU 점수 0.99 cm 는 판단 기준에서 뺌 — docs/debate_super_easy.html, critic_debate_20261020.html)

## 폰 VIO 메모
- histogram_method NONE since 2026-10-24 (c4, c4n400, c4s100, c4c20, c4n400s100c20; make_phone_ov_main.py HIST_NONE); if tracking gets worse (dark scenes, fewer features), HISTOGRAM is the fallback candidate.
