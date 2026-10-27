# ovrefine 2026-10-27 — OpenVINS 상태를 그대로 받아 최근 몇 장만 다듬기 (계획 · 결과 전 선언)

사용자 지시 (10-27 11:55): VINS 를 따로 돌리지 말고 "OpenVINS 상태를 그대로 받아서 최근 몇 장만 다듬기".
브랜치 `ovrefine-20261027` = `lens05-20261027` 9ef84a0 (integ2 4035d59 + 0.5 dpt 렌즈 세트) + ovswba-20261026 의 `ov_bridge/vins/` (요인 코드만 씀),
`main.cpp` · CMake (ovswba 의 `--swba` 는 그대로 둠, 기본 꺼짐) + 이 작업의 `vins/ovrefine.{h,cpp}` · `--refine`.
이 파일은 **결과를 보기 전에** 커밋한다. 러너는 이 파일이 커밋돼 있지 않으면 시작하지 않는다. 결과 json 마다 `plan_commit`.

## 0. 알고 시작하는 것 (오염 기록)
- ovswba (10-26, 옛 렌즈, GL60): VINS 를 독립 추정기로 돌리면 HS38 발산 8–12 vs 1, 짝 d_med +0.26..+0.87 cm. 사후 W60i (OpenVINS 상태로 VINS 창 초기화) 도
  개발 4칸 +0.23 cm. 이 합계에는 아래 시험 녹화가 섞여 있다 → 시험 녹화가 완전히 "안 본" 데이터는 아니다 (단, 이번 방식으로 돌린 결과는 아직 없음).
- xfopt (10-27) 가 같은 빌드 계열 (lens05) 로 B60 (루프 켬) 개발 결과를 냈다. 그 숫자는 쓰지 않는다 (기준선은 이 빌드로 다시).

## 1. 방식 (구현)
- `--refine 1` (기본 0 = 이 코드는 호출되지 않음, 출력 비트 동일). `--refine-cfg K=V,..`, `--refine-log FILE`. PC 평가는 VIO 스레드 안에서 바로 푼다 (결정적).
- 스냅샷: freq (기본 10 Hz) 마다 OpenVINS 가 초기화 · 추적 중인 프레임에서 복사 (읽기만): 상태 시각, R_ItoG, p, v, bg, ba, IMU 15×15 주변 공분산
  (`StateHelper::get_marginal_covariance`), 그 프레임의 KLT 트랙 (id, OpenVINS 렌즈로 왜곡 보정한 정규좌표 = OpenVINS 와 같은 관측), 카메라 외부 보정.
  필터에 넣은 IMU 샘플을 그대로 복사.
- 창: 최근 N 스냅샷. 매 새 스냅샷마다 창 전체를 **OpenVINS 값으로 새로 초기화**해서 한 번 푼다 (이전 창 결과를 이어 쓰지 않음 → 혼자 떠내려갈 상태가 없음, clearState 류 버그 없음).
  - IMU: 스냅샷 사이 VINS `IntegrationBase` 사전적분 (선형화 bias = 앞 스냅샷의 OpenVINS bias), `IMUFactor` 그대로. 잡음 = OpenVINS 연속 밀도 × √(IMU rate)
    (VINS 의 샘플당 표준편차 꼴, ovswba noise 2 와 같은 변환).
  - 재투영: VINS `ProjectionFactor` (첫 관측 스냅샷의 역깊이), sqrt_info = fx / up_msckf_sigma_px (OpenVINS 와 같은 1 px). 창 안에서 min_obs (기본 3) 스냅샷 이상
    보인 트랙만. 역깊이 초기값 = OpenVINS 포즈로 선형 삼각측량, 깊이 0.1–40 m, 초기 재투영 < 3 px 아닌 트랙은 뺌. Huber 1 (VINS 와 같음). 외부 보정 고정.
  - 사전 (앵커): 창 첫 상태 (자세 · 위치 · 속도 · bg · ba) 에 OpenVINS 15×15 공분산 × prior_scale 의 역을 정보로 (OpenVINS 오차 정의 = VINS 오른쪽 섭동과 같음).
- 출력: 포즈 줄의 `"ref"` 객체만 (`p`, `q`, `T_world_cam0` …). 제어 · VIO 필드 · 필터 상태는 안 바뀜.
  창이 받아들여지면 그 스냅샷 k 의 보정 ΔT = T_ref(k) T_ov(k)⁻¹ 를 다음 스냅샷까지 프레임마다 T_ref(t) = ΔT · T_ov(t) 로. 버린 창 뒤 · 첫 창 전 = OpenVINS 그대로.
- 안전장치 (창 버림, 이유별 개수 기록): Ceres 해 사용 불가, 비용 감소 없음 (최종 > 처음), 최종 비용/잔차 수 > cost_max, 쓸 트랙 < min_feat (20),
  |p_ref − p_ov| > dp_max (5 cm) 또는 회전 차 > dth_max (2°), 값이 유한하지 않음. 버리면 OpenVINS 그대로. OpenVINS 재초기화 · 추적 잃음 · 스냅샷 간격 > 0.5 s → 창 비움.
- 직접 시험 (판정 전): (a) `--refine 0` = 이 브랜치 이전 빌드 (lens05 9ef84a0 기반, vins 없음) 와 포즈 비트 동일, (b) `--refine 1` 의 VIO 필드 = `--refine 0`,
  (c) dp_max=0 (모든 창 버림) → ref 궤적 = VIO 궤적, 버린 수 = 창 수, 로그 집계 = 포즈 줄 집계 (ovswba 의 집계 버그 같은 것 확인).

## 2. 데이터 분할 (xfopt `split.json` 그대로 재사용, 녹화 단위)
- 개발 (튜닝, 15칸): hs_20261020_221052, hs_20261020_222747, hs_20261020_223815, hs_20261021_214550, hs_20261022_134550 × 오프셋 0–2.
- 시험 (판정, 27칸): hs_20261019_131326, hs_20261019_154320, hs_20261020_214138, hs_20261020_221925, hs_20261020_224323, hs_20261021_213341,
  hs_20261021_215348, hs_20261022_133820, hs_20261022_134204 × 0–2. std60 165506 o0–o2, 손 루프 142148 / 142610 = 시험 쪽 참고. (173432, 151646 = 참고만)
- 판정은 시험에서 **단 한 번**. 시험 조건 목록은 `test_conds.json` 으로 따로 커밋한 뒤에만 러너가 시험 칸을 돌린다.

## 3. 같은 빌드 · 같은 렌즈 · 같은 자
- VIO = 지금 앱 설정 GL60: `--profile rec60 --drop-late 0 --loop 0` (rec60 = gyro_klt=5, ekf_sparse …; 루프 닫기 끔 = 순수 OpenVINS), 960x540, pts 200,
  clones 15, vio_bench_hs 공급 (init strict 1.0 s / 22.5 px, cfg_over 같음, histogram NONE, frame cache).
- 렌즈 = lens05 (앱 0.6.29 와 같은 원본): std/손 루프 `config/lens/std_1920x1080_f0.50_calcap20261026.json`, HS `config/lens/hs_1920x1080_f2.22_from_calcap05_20261027.json`
  (xfopt cb746c4 와 같은 파일, sha256 0e44520c; 가정: 초점에 따라 배율만 바뀜).
- **기준선 B60 = 이 브랜치 빌드 `--refine 0`, 같은 칸 전부** (옛 GL60 숫자와 비교하지 않음). 후보 = 같은 빌드 `--refine 1` 의 `ref` 궤적.
- 채점: `lens_switch_gtscore.cam_metrics` (카메라 위치, GT 창 SE3 Umeyama, 발산 = ATE > 50 cm 또는 5 s 상대 > 30 cm), GT = `imu_gt_spl_cal1022` (HS), `imu_gt_spl` (std).
  참고 지표 (판정 아님): 1 s 상대 위치 오차 중앙 (rpe1), 손 루프 시작–끝 cm (`loop_eval`).

## 4. 개발 (튜닝 범위, 개발 15칸에서만)
- 조건 (각각 R10 대비 한 가지만 바꿈, 한 번씩): R10 = N 10, freq 10, prior_scale 1, min_obs 3, iters 10 (기본) | R5 = N 5 | R10p4 = prior_scale 4 (앵커 느슨)
  | R10q = prior_scale 0.25 (앵커 단단) | R10m2 = min_obs 2. 안전장치 문턱은 고정 (위 값).
- 개발 선택 규칙 (사전): 개발 15칸에서 발산 ≤ B60 개발값 이고 짝 d_med (ref − B60, 둘 다 정상인 칸) 가 가장 작은 것 하나. 그 d_med 가 0 이상이면 시험 안 함 → 불채택
  (이유 기록). 같은 값이면 버린 창이 적은 것. 조합 · 추가 조정은 하지 않는다 (개발 결과를 보고 조건을 더 만들면 "사후" 로 표시하고 판정에 안 씀).

- 추가 (14:10, 개발 격자 실행 전, 위 선택 규칙에 함께 들어감): R30f30 = N 30, freq 30 Hz (창 길이 1 s 그대로, 스냅샷 3배 — 10 Hz 는 60 fps 영상의 5/6 를 안 씀).
  오염 기록: 그 전에 스모크 1칸 (개발 221052_o0, R10) 을 봤다: VIO ATE 1.018 → ref 1.043 cm, rpe1 0.252 → 0.304 cm, 창 605 중 받아들임 601, 보정 중앙 2.1 mm / 0.17°.
  스모크 진단 (물리 시험): OpenVINS 값에서 IMU 사전적분 잔차 (보정 전, 0.1 s 구간 최대) 중앙 p 1.4 mm · q 0.0015 rad · v 1.0 cm/s → 규약 (부호 · 회전 방향 · 중력) 은 맞다고 본다
  (규약이 틀리면 q 잔차가 회전량의 2배 ≈ 0.05–0.1 rad). 초기 IMU 비용이 큰 것은 OpenVINS bias 가 갱신으로 움직여 random walk 잡음 (작음) 대비 큰 탓 (추정).
- 직접 시험 (c) 조건 = RX0 (dp_max 0), 동일성 (a) = BASE (integ2 4035d59 빌드) — 개발 3칸 (221052_o0, 222747_o0, 134550_o0).

## 5. 채택 규칙 (시험 27칸, 한 번, 모두 만족해야 "채택 가능")
- (1) 시험 HS27 발산 수 (ref) ≤ B60 발산 수.
- (2) 시험 HS27 짝 비교 (둘 다 정상인 칸) ref − B60 ATE 중앙값 < 0. 녹화 단위 (녹화 9개의 칸 평균 차) 좋아진 녹화 수도 보고.
- 참고 (판정 아님): std60 165506 o0–o2 중앙 (나빠지지 않는지), 손 루프 142148 / 142610 시작–끝 cm, rpe1, 프레임당 / 창당 PC 시간 p50/p95 (다른 에이전트와 같이 돈 PC = CPU 근거 아님),
  버린 창 수 · 이유.
- 반복: 결정적 실행, 칸당 1회. 오프셋 o0–o2 는 같은 녹화라 독립이 아니다 → 녹화별 합계 같이 보고, 칸마다 초기화 시각 · 첫 포즈 해시 기록 (같으면 중복 표시).
- 통과하면: 앱 반영은 사용자 확인 후 (이번에는 PC 평가 + 플래그까지). 폰은 쓰지 않는다 (사용자 지시).

## 6. 기록
진행 = scratchpad/team/ovrefine.progress.txt. 실행 원자료 = `~/scanner/tmp_ovrefine/` (저장소 밖). 결과 json 마다 plan_commit, 브리지 sha, 러너 sha, 렌즈, 만든 시각.
