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

## 3. 결과 (21:20, 독립 검증 2회 반영, 모든 숫자 추정/미검증 표시는 HTML 참조)
- **라이브 설정 그대로 PC 재생 (미검증 숫자)**: 라이브와 같은 조건의 GT 녹화는 **1개뿐** (165506). 보드 근처 (카메라 위치 RMS 13 cm 범위) 에서 VIO ATE **1.5–1.7 cm** (시작 0/1/2 s), 경로 4.5 m 의 0.37 %, 표류 1.65 cm/m. 같은 조건의 다른 GT 녹화 173432 는 **초기화가 끝내 안 됨**.
- 방 안을 손으로 돈 루프 (GT 없음, 라이브와 같음): 시작–끝 오차 **6–15 cm / 2.5–3.8 m (2.5–4.5 %)**, 같은 루프도 시작 시각 1–2 s 차이로 12.3 → 7.5 → 6.4 cm (142148) 로 흔들림. 세 번째 루프 151646 은 초기화 안 됨. 루프 닫힘은 손 루프에서 **0 회** 발동 (흰 벽).
- 같지 않은 HS 녹화 12개 (2.22 dpt, 240 fps 세션): VIO ATE 0.33–1.65 cm (중앙 0.6), 실패 0. → 보드 앞 작은 움직임에서는 1 cm 안팎이지만, **방 크기 정확도는 GT 로 잰 적이 없다** (GT 는 보드 1.2 m 안에서만).
- **계통 오차 후보 하나** (사후 참고 지표): VIO 이동이 GT 보다 약 **1.5–3 % 크다** (Sim3 축척 중앙 0.978, GT 녹화 13개 중 11개 < 0.99; 165506 은 정방향 0.970 / 역방향 0.986). 어떤 절제 조건도 이것을 바꾸지 못함 → 원인 후보 = 가속도계 축척 또는 GT 자 (노트북 화면 보드 크기). HS 12개는 세션 3개에서 나와 독립 아님. 큰 공간에서 얼마가 될지는 외삽 (5 m 에 수 cm–10 cm, 추정).
- 오차 원천 절제 (한 가지씩, HS 12쌍): 판정 규칙을 넘은 것은 **IMU 잡음 ×2 (나빠짐 +0.22 cm, 11/12)** 하나뿐. 렌즈 (다른 보드 보정·초점 불일치), 시간차 (오라클·고정·+5 ms), IMU 보정값, 프레임 10 % 버림, gyro_klt 끔, 시작 시각은 모두 **잡음 수준** (|중앙 Δ| ≤ 0.07 cm). 루프 닫힘은 이 작은 공간에서 ATE 를 바꾸지 않음 (지도−VIO 13개 중앙 0.00, 좋아짐 5 / 나빠짐 6, 재생은 낙관적 동기 조건).
- 탐색 후보: **롤링셔터 보정 rs_derot** — 165506 1.70 → 1.20 cm (−0.50, 녹화 1개 · 시작 0 s 한 번; 다른 변화는 −0.24…+0.08). 손 루프 −3.3 cm 는 거의 모든 조건이 비슷하게 좋아져 근거 아님. HS 는 +0.11 (9/12) = 규칙상 잡음 수준. → **추정 후보일 뿐**, 새 60 fps GT 녹화로 판정.
- 가장 큰 불확실성은 **자(데이터)**다: 라이브와 같은 GT 녹화 1개, GT 변형에 따라 165506 이 1.70/1.75/2.08 cm (2.08 은 시각이 0.5–0.7 ms 어긋난 imu_gt 라 자 불확실성보다 그 어긋남 탓일 수 있음), 작은 공간. 다음 1순위 = 사용자가 **인쇄 보드 + 큰 공간 60 fps GT 녹화** (6절 절차).
- 표: summary.md, 보고: C:/Users/a3162/scan3d_handoff/changes_0620/livepose_20261030.html (사본 이 폴더)
- 다음 1. 라이브와 같은 60 fps GT 녹화 묶음 (std60 = 라이브 카메라 세션). 인쇄 보드 + 보드에서 시작·끝나는 큰 루프 — 사용자: A1 ChArUco (lenscal 절차의 보드) 를 벽에 고정. 녹화 10개 × 40–60 s: 바닥에 놓고 3 s 정지 → 들고 보드 앞 0.5–1 m 에서 10 s → 방을 3–10 m 돌아 (흰 벽·창 포함) → 보드 앞으로 돌아와 10 s → 3 s 정지. 빠르기 보통/빠름 반반. 2개는 복도 20 m 왕복.
- 다음 2. 줄자 직선 시험 (축척) — 사용자: 바닥에 줄자로 5.00 m 직선 표시. 끝점마다 폰을 같은 받침 위에 3 s 정지, 왕복 3 회 × 녹화 3개. GT 없이 끝점 거리만으로 채점
- 다음 3. 6 면 가속도계 보정 (보류 중인 실험) — 사용자: 폰을 6 면 (±x ±y ±z) 으로 각 30 s 정지, 카메라 끈 채 IMU 만 (imu_tx). 열 시험 없이
- 다음 4. 롤링셔터 보정 rs_derot 판정 — 사용자: 할 일 없음 — 1번 녹화가 생기면 PC 에서 같은 규칙으로 B vs R_rs. 좋으면 앱 플래그 하나
- 다음 5. 초기화 실패 줄이기 — 사용자: 녹화 시작 때 바닥 정지 3 s 후 '천천히 크게' 첫 움직임 (보드가 아닌 무늬 많은 곳을 보며). PC 에서 init 조건 탐색은 그 녹화로
- 다음 6. 인쇄 보드 렌즈 보정 (lenscal 2 절) — 사용자: 1번과 같은 보드로 정지사진 55 자세 × 2 세트. 값은 보드 보정으로만 정함
- 다음 7. IMU 잡음 ×0.5 실기 A/B — 사용자: 1번 녹화가 생기면 PC 판정 먼저; 폰은 설정 폴더 하나 (c4_nc05 처럼)
