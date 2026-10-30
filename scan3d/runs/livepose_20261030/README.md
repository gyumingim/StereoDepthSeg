# livepose 2026-10-30 — 라이브 폰 위치 정확도: 같은 자 기준선 + 오차 예산

브랜치 `livepose-20261030` (appfix-20261029 b96c8bc 위), WSL `~/scanner/StereoDepthSeg-livepose`. PC 만 (폰·새 녹화 없음).
임시 출력 `~/scanner/tmp_livepose/` (커밋 안 함). 이 1–2 절은 **측정 전에 커밋** (사전 등록).

## 1. 라이브 설정 = 코드에서 읽은 값 (앱 0.6.38c, README 아님)
`phone_stereo/arlat/src/com/scan3d/arlat/MainActivity.java` 250–360, `OvRunner.java` 440–520, `scan3d/tools/ov_phone_run.sh` 154,
실제 폰 run `runs/appnpu_20261028/phone/run_20261028_200125_main60_c4f60/meta.json` 으로 대조.

| 항목 | 라이브 값 | 근거 |
|---|---|---|
| 카메라 | `ov_cam main` (논리 0 = 물리 5 광각), YUV Y 평면 1920×1080 @ 60 fps | ov_phone_run.sh 가 항상 `--es ov_cam main`; profile rec60 → fps 60, ov_cfg c4f60 |
| 노출·초점 | exp 2000 µs + auto ISO (시작 때 한 번 잠금), 초점 0.5 dpt 고정, OIS/왜곡보정 끔 | MainActivity main 기본값; meta iso 3200 (10-28 실내) |
| 프레임 시각 | SENSOR_TIMESTAMP + (노출 + RS skew)/2 = 노출 가운데 (skew 6.54 ms) | OvRunner midOffsetNs; 녹화 frames_cam5 `t_mid_ns` 와 같은 정의 (165506: +4.270 ms 확인) |
| IMU | **보정 안 된 자이로/가속 (types 16/35, 값 0..2), Android 바이어스 미적용, rebias 없음** | `ov.imuRaw = ... : main` → meta `imu uncalibrated types 35/16` |
| 설정 | `files/ov/c4f60/` = 960×540 정류, clones 15, pts 200, 렌즈 **calcap k12_notan 0.5 dpt** (lens05 10-27 설치) | lens05 README; PC 재생성 sha = 폰 sha (yaml 58b9a1af…, remap 483168c9…) |
| 시간차 | yaml `timeshift_cam_imu` 2.172 ms + 온라인 보정 (`calib_cam_timeoffset true`), 녹화별 이동 없음 | c4f60 yaml |
| 브리지 인자 | preset bench (`--max-speed 8 --lost-frames 1e6 --startup-lost-frames 1e6`) + `--remap` + rec60 플래그: ovopt `all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse,gyro_klt=5`, drop-late 50 ms (mode 1), loop 1 (BoW+BRIEF, 비동기 스레드), health 1, predict 2 @200 Hz + pred thread, idle spin 2, imu_prio 1 | MainActivity 329–352, meta.json bridge_cmd |
| ZUPT | 끔 (`try_zupt false`) | c4f60 yaml |

## 2. 사전 등록 (측정 전)
### 2.1 재생 = 라이브와 같게 (`lp_replay.py`)
- 같음: 폰 c4f60 폴더 그대로 (yaml + remap, 브리지가 remap), 같은 브리지 인자, raw IMU (rebias 없음), 프레임 = `t_mid` 그대로 (GT 정렬 dt 안 씀).
- 다름 (모든 결과 json `input_path` 에 기록): 영상 = 녹화 cam5.mp4 (std60 160 Mbit/s H.264) → OpenCV BGR → GRAY (폰 = 코덱 없는 Y 평면);
  오프라인이라 `--drop-late 0`, `--loop-sync 1` (지도 보정이 바로 적용 = 낙관적), `--idle-spin 0 --predict 0` (시간만 바뀌는 플래그).
  HS 녹화는 240 fps 에서 1/60 s 격자에 가장 가까운 프레임.
- 브리지 = PC 빌드 `~/scanner/tmp_appnpu/ident/ov_bridge_new` 사본 (sha 기록). 폰 네이티브 0.6.38c 와 소스 차이는 VIOP 쪽뿐 (60a7652, 포즈 무관 = **가정**).
- 재현 확인 (측정 전 스모크): 이 러너로 옛 하네스 조건 (보정 IMU + GT dt) 165506 → VIO ATE 1.63 cm = appnpu `ident/new.json` 1.63 cm.

### 2.2 녹화 묶음
- **S (라이브와 같음, GT)**: std60 0.5 dpt 60 fps — `165506`, `173432` (노트북 화면 ChArUco 5×3 GT, `imu_gt_spl`).
- **L (라이브와 같음, GT 없음)**: std60 손 루프 `142148`, `142610`, `151646` → 시작–끝 오차 (fi_limits_run.loop_eval 규칙).
- **H (같지 않음)**: HS 240 fps 세션, 초점 2.22 dpt (라이브 렌즈와 약 1.4 % 다른 초점 거리), GT 있음, ov60eval/phone60 RAW 12 개
  (`213955` 폰 끊김 제외, 19 일 `131326`·`154320` 은 raw IMU 없음 → 제외).
- 그 밖의 녹화 (목록만): 10-23/24 std60 짧은 녹화 70여 개 = 렌즈 보정용 (정렬·GT 없음), 그 밖 HS GT 있는 것 (`111605` 0.37 dpt 등) 은 표준 묶음 아님.

### 2.3 채점 (`lp_score.py`, 자 = 카메라 위치)
- ATE = VIO `T_world_cam0` 위치 vs GT `p_bc` (가장 가까운 GT < 2 ms, |p_bc| < 1.2 m), SE3 정렬 (축척 없음), 추적 포즈 전부. 지도 포즈 (`map.T_world_cam0`) 도 같은 방식.
- 경로 = GT 위치 10 Hz 표본 길이; ATE % = ATE / 경로. 표류/m = 0.5 s 마다 시작점, GT 경로 D (1 m, 경로 < 2 m 이면 0.5 m) 만큼 간 뒤 변위 오차 / D 의 중앙값.
- 초기화 시간 = 첫 tracking 포즈 − 첫 프레임 (s). 실패: 추적 < 50 포즈 = no_init, ATE ≥ 50 cm 또는 루프 오차 ≥ 50 cm = diverged, 브리지 종료 = crash.
- 반복: 재생은 결정적 → 같은 조건 반복 = 같은 값. 시작 오프셋 0/1/2 s 를 반복 대신 씀 (B, B_o1, B_o2).

### 2.4 조건 (기준 B 에서 한 가지씩만 바꿈, `lp_batch.py`)
| 이름 | 바꾼 것 | 오차 원천 |
|---|---|---|
| B / B_o1 / B_o2 | 라이브 그대로 / 시작 +1 s / +2 s | 기준선, 초기화 |
| L_d | 렌즈 = 다른 0.5 dpt 보드 보정 cal20261024d (보드 측정값, VIO 로 맞춘 값 아님) | 렌즈 보정 불확실성 |
| L_hs (H 만) | 렌즈 = 그 녹화의 2.22 dpt 보드 렌즈 | 렌즈 초점 불일치 (약 1.4 %) |
| I_cal | IMU = Android 보정값 (types 1/4) | IMU 바이어스 처리 |
| N_half / N_x2 | IMU 잡음 4 값 × 0.5 / × 2 | IMU 잡음 모수 |
| T_or | GT 정렬 dt 로 프레임 이동 + yaml 시간차 0 (오라클) | 카메라–IMU 시간차 |
| T_fix | 온라인 시간차 보정 끔 (2.172 ms 고정) | 〃 |
| T_p5 | 시작 시간차 +5 ms 틀림 (7.172 ms) | 〃 (민감도) |
| R_rs | + `rs_derot` (자이로 롤링셔터 보정, 6540 µs) | 롤링셔터 |
| K_off | gyro_klt 끔 | 특징 추적 |
| D10 | 넣는 프레임 10 개마다 1 개 버림 | drop-late / 시간 (스트레스) |
| Lp0 (S 만) | 루프 끔 → VIO 포즈가 B 와 비트 같아야 함 (확인용) | 루프 닫힘 |
- 루프 닫힘의 몫 = 같은 B run 의 지도 ATE − VIO ATE.

### 2.5 판정 규칙
- 짝 비교 (조건 − B, 같은 녹화·오프셋 0): H 12 개에서 |중앙 Δ ATE| ≥ 0.05 cm **그리고** 같은 부호 ≥ 10/12 (부호 검정 p ≈ 0.04) → "영향 있음".
  아니면 "잡음 수준". 발산 수 차이는 따로 적음 (짝 비교에서 둘 다 정상인 칸만 Δ).
- S·L 은 녹화 2–3 개 → 방향만 "추정". H 결과를 라이브에 옮기는 것도 "추정" (세션·초점·노출 다름).
- 오차 예산 = 원천마다 그럴듯한 불확실성 범위로 바꿨을 때 |Δ ATE| 중앙값, B 중앙 ATE 대비 %. 측정 안 한 원천 (디코드 손실, 실제 폰 타이밍, GT 자체 오차)은 "미측정".
- 기준을 맞추려고 판정 규칙·녹화를 바꾸지 않는다. 바꾸면 이 절 아래에 날짜와 이유를 덧붙인다.

### 2.6 덧붙임 (18:20, 기준선 B 일부를 본 뒤 — 탐색, 판정 근거 아님)
- 채점에 다른 GT 변형 (`imu_gt`, `imu_gt_rs`) 대비 VIO ATE 도 함께 적음 (자 자체의 흔들림 확인용; 1차 지표는 그대로 `imu_gt_spl`).
- 173432 가 B 에서 초기화 안 됨 (`[init]: not enough feats to compute disp: 0,67 < 15`, 추적 점 36–69 개). 탐색 조건 (S+L 만):
  `X_win05` init_window_time 0.5 s, `X_pts300` num_pts 300, `X_fast10` fast_threshold 10. 결과는 "탐색" 으로만 적는다.

### 2.7 독립 검증 (19:00, 읽기 전용 에이전트, `scratchpad/team/verify_livepose_baseline.md`) — 주장 8개 틀림 0, 덧붙인 차이
- c4f60 은 rec60 기본값이 아니라 launcher `<cfg>` 인자 (ov_phone_run.sh 121, 154) — 운영자가 c4f60 을 줄 때만 이 기준선이 라이브와 같다.
- 폰 첫 프레임 (idx 0) 은 capture result 전이라 시각 오프셋 0 으로 나감 (재생은 +4.27 ms). Image.getTimestamp = SENSOR_TIMESTAMP 는 가정.
- `--health 1` 은 gyro_klt mode 5 가드에 들어가고 (main.cpp 1959), drop-late mode 1 은 늦은 프레임이 이어지면 DEGRADED (2081) → 라이브에서는 늦은 프레임이 포즈를 바꿀 수 있음. 재생 (drop-late 0) 에는 이 경로 없음.
- 폰은 acquireLatestImage·pool 에서 프레임을 버릴 수 있음, 재생은 모두 넣음. 디코드 = H.264 (limited range) → BGR → GRAY. 폰 .so 와 PC 빌드 동일성은 확인 불가.
- 채점 세부: `scale_sim3` 는 반사 부호 미처리 (참고값), "10 Hz" 경로 = 매칭 포즈 6 개마다, 표류/m 은 실제 GT 경로 (≥ D) 로 나눔. `imu_gt` 시각은 t_mid 와 0.5–0.7 ms 어긋남 (참고 자).

## 3. 결과
(측정 뒤 채움)
