# gyrocheck 2026-10-27 — PLAN (판정 규칙, 결과 보기 전에 적음)

작성 2026-10-26 23:04 (결과 0개 상태에서 커밋). 폰 안 씀, PC 재생만. 앱·프로필 기본값 안 바꿈.

## 질문
60 fps rec60 의 gyro_klt=5 (GL60) 가 없는 것 (NL60) 보다 나은가/나쁜가가 **계통 효과**인지 **초기화 시각 제비뽑기**인지.
(배경: scratchpad team/refute_metaAB_20261027.md C1–C4)

## 할 일 1 — NL60 vs GL60, 시작 밀기 5회
- 빌드: `~/scanner/tmp_dpvo/ov/bin/ov_bridge_c640f5344f35` (sha256 c640f5344f35c974…, 0.6.27 PC 빌드, dpvo ov_ref GL60 손 루프와 integ2 gyro 점수의 빌드). 두 조건 **같은 바이너리**.
- 조건 (integ2_ate.py CONDS 그대로):
  - NL60 = `--profile rec60 --drop-late 0 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse` (gyro_klt 없음), cfg LIVE
  - GL60 = `--profile rec60 --drop-late 0` (프로필 그대로 = gyro_klt=5 포함), cfg LIVE
- 러너: integ2_ate.py `job()` 과 같은 인자 (BASE_CFG, LIVE, clones 15, init strict 1.0 s / 22.5 px, lever, imu_raw 규칙, decode 2) 에서 **start_off 만** 바꿈, frame_cache=False (공유 캐시에 안 씀). 렌즈 = 벤치 기본 (기존 하네스와 같음).
- 녹화 6개 × 2조건 × k=0..4 = 60 작업:
  - 손 루프 142148, 142610, 151646 (60 fps 녹화): start_off = k/60 s (1 프레임씩)
  - std60 165506 (60 fps, GT imu_gt_spl): start_off = k/60 s
  - HS 131326, 154320 (240 fps, GT imu_gt_spl_cal1022): o0 + k/240 s (HS 1 프레임씩)
  - k=0 = 기존 칸 재현 확인 (예상: GL60 손 루프 9.14/16.54/31.83, NL60 4.48/11.2/발산; 131326_o0 GL60 발산·NL60 ~0.6; 154320_o0 NL60 ~2.5·GL60 ~1.65). 재현 안 되면 멈추고 원인부터.
- 채점기 (그대로): 손 루프 = `imu_learn/fi_limits_run.py loop_eval` 의 loop_err_cm; std60/HS = `lens_switch_gtscore.cam_metrics` 의 ate_w (cm) 와 divergent.
- 발산 정의: std/HS = cam_metrics divergent; 손 루프 = loop_err ≥ 100 cm 또는 추적 포즈 < 30.
- 동시 실행 ≤ 3, nice.

### 녹화별 판정 (짝 = 같은 k)
- div_N, div_G = 5회 중 발산 수.
- |div_G − div_N| ≥ 2 → 적은 쪽이 이김 (발산으로 결정).
- 아니면 둘 다 발산 안 한 짝 d_k = GL − NL (≥ 3 짝 필요, 아니면 무승부):
  - GYRO+ : GL 이 더 좋은 짝 ≥ 4 (짝 3개면 3/3) 이고 중앙 d ≤ −δ
  - GYRO− : 반대 (GL 이 더 나쁜 짝 ≥ 4 (3개면 3/3), 중앙 d ≥ +δ)
  - 그 외 무승부
  - δ: 손 루프 1.0 cm, std60/HS ATE 0.05 cm (기존 위상 흔들기 칸 표준편차 0.02–0.11 cm)
- 계통 / 제비뽑기: 녹화 승부가 났고 5개 위상 중 ≥ 4 에서 같은 방향 (발산은 가장 나쁨으로 셈) → "계통". k=0 의 방향이 나머지 4 위상 중 ≥ 2 에서 뒤집히면 → 그 녹화의 k=0 차이는 "제비뽑기". 초기화 시각 (t_init) 의 위상별 퍼짐도 같이 적음.

### 종합 (gyro_klt 유지/끄기 추천, 오프라인이라 항상 "추정")
- 유지: GYRO+ 녹화 ≥ 3, GYRO− ≤ 1, 총 발산 GL ≤ NL.
- 끄기: GYRO− 녹화 ≥ 3 이고 GYRO+ ≤ 1, 또는 총 발산 GL ≥ NL + 2.
- 그 외: 판정 불가 → 기본값 그대로 두고 라이브 60 fps GT 녹화 (켬/끔 짝) 로 결정.
- 손 루프는 GT 없음 (닫힘 오차 대용), 녹화 1개 = 1표로 같은 무게. 반복 = 위상 5개 (독립 녹화 아님).

## 할 일 2 — fc_safety arm 경로 (코드 수정 없음)
- `esp32fc/components/fc_safety/fc_safety.c` (이 worktree, integ2 4035d59) 를 그대로 링크한 최소 재현 프로그램 (runs/gyrocheck_20261027/arm_replay/, test 폴더 안 건드림) 에 기존 포즈·health 스트림 (할 일 1 의 GL60/NL60 손 루프 k=0 포즈 + 녹화 imu.csv) 을 넣음.
- 매핑 (ov_bridge mav_out.h emit_viop / hb_build_send 그대로):
  - pose (카메라 프레임마다): VALID = init, TRACKING = status "tracking", quality = lost 면 0 아니면 clamp(100(1−σ/0.2),1,100), pos_std_max = h.pos_std (= sqrt(trace), 실제 VIOP 의 축별 최대보다 크거나 같음 → 보수적; sqrt(trace/3) 하한도 같이 돌림), send_delay 0, reset_counter 0.
  - heartbeat 20 Hz: 초기화 전 STARTING, status lost → LOST, 아니면 TRACKING. flags 0.
  - fs_input (10 ms 마다): imu_ok = true, gyro_norm = 폰 자이로 |ω| (imu.csv type 16), acc_norm = |가속| (type 1), 기울기 = 끝 정지 자세 기준 중력 방향 각도 (가정: 끝 정지 = 수평 기체), thr 0, 배터리 16.0 V 유효, kill 스위치 끔, PC link 20 Hz 정상.
- 출력: 10 ms 마다 fs_t 복사본에 ARM 명령 → 거부 비트 (FS_B_*) 시계열, 처음 ARM 이 받아지는 시각, 구간 (처음 정지 / 흔들기 / 끝 정지) 별 막는 비트.
- 예상 (refute C5): 초기화 전 POSE|HB, 흔드는 중 MOVING, 끝 정지 중 0 (arm 가능). 발산한 NL60 151646 은 끝 정지에서도 POSE 로 막혀야 정상.

## 사후 수정 1 (2026-10-26 23:43, 결과 일부 = k0·k1 손 루프/std60, 131326 k0·k1 를 본 뒤; 이유: 독립성 — 감독 23:31 C3·C4·C5)
- 확인: l142610 k1 = k0 추적 포즈 해시 동일 (NL60·GL60 둘 다). l142148·s165506 은 첫 추적 시각 같지만 포즈 다름. 1 프레임 밀기는 독립 반복이 아닐 수 있음.
- 집계 추가 (원래 규칙도 그대로 같이 보고): 위상마다 추적 포즈 해시 (절대 t_ns + 포즈) 를 비교해 (NL60, GL60) 해시 쌍이 같은 위상은 1표로 묶음. 서로 다른 실행이 3개 미만인 녹화는 "판정 불가".
- 초기화 실패 (추적 0, 예: l151646 k1 두 조건) 는 "발산" 에서 빼고 따로 적음 (사후 집계에서만; 원래 규칙 표는 PLAN 정의 그대로).
- 새 실험 (사후): 60 fps 녹화 4개 (손 루프 3 + std60) × 2조건 × 시작 밀기 0.10/0.20/0.30/0.50 s = 32 작업 (gc_run.py runs). 사후 집계에만 넣음.
- arm 재현 (사후): 손 루프 k0 를 같은 빌드에 `--viop-udp 127.0.0.1:port` 만 더해 다시 돌려 (출력만, 포즈 해시 같음 확인) 실제 VIOP POSE/HEARTBEAT 프레임 (축별 pos_std_mm) 을 받아 fc_safety 에 넣음. h.pos_std 두 한계 대신 이 값으로 끝 정지 armable 률 확정.
