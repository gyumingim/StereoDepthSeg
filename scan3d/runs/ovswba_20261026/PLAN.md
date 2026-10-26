# ovswba 2026-10-26 — VINS-Mono 슬라이딩 윈도우 BA 를 OpenVINS 옆에 (계획)

브랜치 `ovswba-20261026` (integ2-20261026 12bbf1c 에서). 60 fps (rec60) 만. 모든 기능은 `--swba 1` 일 때만 (기본 0 = 기존과 비트 동일).

## 사용자 결정 (10-26, 코디네이터 전달)
- VINS-Mono 의 estimator (슬라이딩 윈도우 최적화, IMU 사전적분 인자, 주변화, feature manager, 초기화) 를 **거의 그대로 이식**.
  ROS·시각화·루프/포즈그래프·설정 파일 배관만 잘라낸다. 수식·구조는 다시 만들지 않는다 (검증된 동작 유지). 고치는 건 동작 확인 뒤.
- 결합 방식은 (b) 느린 보정 먼저. 제어 포즈는 절대 튀지 않는다.

## 출처 · 라이선스
- VINS-Mono: https://github.com/HKUST-Aerial-Robotics/VINS-Mono  commit `90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d` (2024-05-23), 라이선스 **GPL-3.0** (LICENCE 파일).
- OpenVINS (이미 사용 중): GPL-3.0. 둘 다 GPL-3 → 같이 묶어 배포 가능 (앱 전체 GPL-3 조건은 이미 OpenVINS 때문에 성립). 출처·라이선스 문구는 이식 파일 머리에 남긴다.
- 이식 범위 (`scan3d/ov_bridge/vins/`): `estimator.{h,cpp}`, `feature_manager.{h,cpp}`, `parameters.{h,cpp}` (ROS 읽기 → 구조체 설정으로 교체),
  `factor/*` (imu_factor, integration_base, marginalization_factor, pose_local_parameterization, projection_factor, projection_td_factor),
  `initial/*` (SfM, 정렬, 5점, 외부 회전), `utility/{utility,tic_toc}`. 빼는 것: `estimator_node.cpp` 의 ROS 부분 (process() 의 IMU 보간 루프만 래퍼로 옮김),
  `visualization`, `CameraPoseVisualization`, `pose_graph/`, `feature_tracker/`, `camera_model/`.
- 꼭 필요한 최소 수정 (전부 파일에 `// ovswba:` 주석): ROS 매크로/Header → `vins/compat/` 의 작은 대체 헤더;
  Ceres 2.2 에는 `LocalParameterization` 이 없음 → `PoseLocalParameterization` 을 같은 Plus/Jacobian 의 `ceres::Manifold` 로 (수식 동일).

## 앞단: OpenVINS KLT 트랙 재사용 (VINS feature_tracker 안 씀)
- 이유: (1) 같은 프레임에서 이미 KLT (gyro_klt=5, RANSAC) 를 돌림 → 추가 추적 비용 0, (2) VINS feature_tracker 는 camodocal camera_model 까지 이식해야 함,
  (3) 비교가 깨끗함: 같은 관측으로 필터 vs 최적화만 다름. VINS 입력 형식 `[x y 1 u v vx vy]` = OpenVINS 왜곡 보정 정규좌표 + 픽셀, 속도 0 (td 추정 끔).
- 입력 빈도 = VINS 기본 `freq: 10` Hz → 60 fps 에서 6 프레임마다 1 프레임 (그 사이 IMU 는 전부 사전적분).

## 질문 1. 결합 방식
| | 방식 | 장점 | 위험 |
|---|---|---|---|
| (a) | 윈도우 BA 의 최신 포즈/속도/바이어스를 OpenVINS 에 EKF 갱신으로 되먹임 | 필터 자체가 좋아짐, 제어 포즈도 좋아짐 | **같은 관측 이중 사용**: BA 는 필터가 이미 쓴 같은 KLT 트랙·IMU 로 만든 값 → 독립 측정처럼 넣으면 공분산이 과신 (불일치). 막는 법: (i) Covariance Intersection (상관 모를 때도 일관), (ii) 공분산 크게 부풀린 위치/yaw 만 약하게, (iii) 필터가 안 쓴 정보만 (SLAM 아닌 장기 트랙 등). 필터 상태가 바뀌므로 제어 포즈가 한 프레임에 움직일 수 있음 → 갱신 크기 제한 필요 |
| (b) | BA 결과를 루프 닫기처럼 느린 지도/오돔 보정 `T_corr` (yaw+이동, 속도 제한) 로만 | 필터·제어 포즈 비트 동일 (끄고 켜도 VIO 필드 같음), 이중 사용 문제 없음 (출력만), ovloop MapOdom 과 같은 꼴 | 제어 루프는 이득 못 봄 (지도/위치 출력만 좋아짐). BA 가 필터보다 나쁘면 오히려 나빠짐 → 측정으로 확인 |
| (c) | 필터를 최적화로 교체 | VINS 단독 | 키프레임 지연 (10 Hz + 해 30–60 ms), 초기화 실패 위험, 지금 검증된 체계 전부 바꿈 → 이번엔 안 함 |

**권장: (b) 먼저** (출력 `"swba"` 객체 = `p_map = Rz(yaw) p_vio + t`, 속도 제한 0.10 m/s · 3 °/s 는 ovloop 와 같은 값). (b) 에서 VINS 궤적이 같은 자로
OpenVINS 보다 **확실히** 좋을 때만 (a) (CI 방식) 를 다음 단계로. 이중 사용을 피하는 (a) 의 순서: CI 가중 ω 를 trace 최소로, 위치·yaw 4-DoF 만, 한 번 갱신 ≤ 1 cm.
- (b) 의 보정 정의: VINS 가 NON_LINEAR 가 된 뒤 처음 몇 프레임으로 VINS 세계 ↔ OpenVINS 세계의 4-DoF 정렬 `A` (중력은 둘 다 정렬됨) 를 한 번 구하고,
  이후 해마다 최신 프레임에서 `T_target = A⁻¹·T_vins(t) · T_vio(t)⁻¹` 의 yaw+이동을 목표로, 출력은 속도 제한으로 따라감. VINS 재시작 (failure) → 새 정렬, 보정은 그 자리 유지 후 다시 따라감.

## 질문 2. 재사용
- 트랙·IMU: OpenVINS 트래커 `get_last_obs/ids` (ovloop 와 같은 읽기 전용 접근 `VioAcc`), IMU 는 pump() 에서 필터에 넣는 것과 같은 샘플을 복사.
- 수식: VINS-Mono 그대로 (사전적분 midpoint + 공분산·야코비안, ProjectionFactor (sqrt_info = FOCAL_LENGTH/1.5), Schur 주변화 + FEJ 사전, DOGLEG/DENSE_SCHUR, Huber 1.0).
- Ceres 는 이미 브리지 빌드 (PC 2.2, 안드로이드 2.2 정적) 에 있음.
- 잡음: VINS 의 acc_n 등은 샘플당 표준편차 꼴로 쓰임 (공분산 전파에 dt 가 곱해짐) → 두 후보를 **미리 선언**: N1 = VINS EuRoC 기본값 (0.08/0.004/4e-5/2e-6),
  N2 = OpenVINS 연속 밀도 × sqrt(IMU rate). 개발 칸 2개 (HS 133820_o0, 224323_o0) 로만 고르고, 고른 뒤 전체 평가 (개발 칸도 전체 표에 포함됨 — 보고에 적음).
- 외부 보정: RIC = R_ItoCᵀ, TIC = −R_ItoCᵀ p_IinC (OpenVINS 값 그대로, ESTIMATE_EXTRINSIC 0), 시간: VINS 프레임 시각 = OpenVINS 상태 시각 (카메라 t + 현재 dt), TD 0 고정.

## 질문 3. 윈도우 · 키프레임 · CPU
- WINDOW_SIZE 10 (VINS 기본) @ 10 Hz 입력 = 약 1 s+ (키프레임 아닌 프레임은 MARGIN_SECOND_NEW 로 버려져서 키프레임 간격만큼 길어짐), 키프레임 = 시차 10 px (FOCAL 460 기준) 그대로.
- 해: max_num_iterations 8, max_solver_time 0.04 s (VINS 기본). PC 평가는 결정성을 위해 시간 한도를 끄고 (1 s) 반복 수로만 제한 — 보고에 적음.
- 목표: 폰 백그라운드 코어 1개에서 프레임당 (해 + 주변화) ≤ 30–60 ms, 10 Hz → 코어의 30–60 % (추정, 폰 재생으로 측정). 주변화 스레드 4개 (VINS 기본 NUM_THREADS) 그대로.
- 메모리: 특징 ≤ 1000 (NUM_OF_F), 윈도우 11 프레임 × (IMU ~40 샘플), 주변화 사전 행렬 ~ (15·11 + 6)² double → 수 MB 이하 (추정).
- 스레드: VIO 스레드는 복사만 (10 Hz 특징 ~200 개 + IMU), 별도 스레드 (nice 10) 가 VINS 를 돌림. `--swba-sync 1` = PC 결정적 평가 (VIO 스레드가 기다림).

## 평가 (같은 자)
- 기준 = OpenVINS GL60 (dpvo ov_ref_run.py = integ2_ate 프로토콜, histogram NONE, rec60, gyro_klt=5, --drop-late 0): HS 42칸 div 1 / ate_w 중앙 0.570 cm,
  std60 165506 o0–o2 1.50/1.55/1.70 cm, 손 루프 142148/142610 3.27 % / 5.20 %.
- 같은 실행에서 VIO 필드 (기준과 동일해야 함) 와 `"swba"` 지도 포즈를 둘 다 같은 채점기 (`lens_switch_gtscore.cam_metrics`, `loop_eval`) 로. VINS 원 궤적 (10 Hz) 도 따로 채점.
- 동일성: `--swba 0` vs integ2 빌드, `--swba 1` 의 VIO 필드 = 끈 것. 키프레임당 CPU 시간 (해·주변화·전체) 로그.
- 폰 재생 타이밍은 시간 남으면 (잠금 규칙). 마지막에 독립 검증 에이전트 1개.

## 추가 (19:30, 결과 보기 전에 선언)
- 잡음 선택 결과 (개발 칸 2개): noise 1 (VINS EuRoC) swba 중앙 1.08 cm, noise 2 1.49 → **noise 1** (선언한 규칙대로). 이 두 칸에서 VINS 는 VIO 보다 나쁨 (raw 0.91/0.96 vs 같은 프레임 VIO 0.80/0.33).
- 한 번만 하는 개발 선별 (각각 W60 대비 한 가지만 바꿈), 개발 칸 4개 (133820_o0, 224323_o0, 214138_o0, 221052_o0; 이 칸들은 전체 표에도 들어감):
  W60e = estimate_extrinsic 1 (OpenVINS 는 외부 보정을 온라인으로 바꿈: p_IinC 약 1 cm, R 0.5°), W60f = FOCAL_LENGTH 661 (실제 fx; 시각 잔차 가중 1.5 px 실제),
  W60d = 정렬을 비선형 2 s 뒤에 (초기화 직후 VINS 값으로 정렬하면 일정한 오프셋 ~11 cm 가 남는 것 관찰), W60q = 입력 20 Hz.
  개발 칸 swba 중앙값이 W60 보다 좋은 것만 전체에서 다시 돌림 (같은 자). 그 밖의 조정은 하지 않음.

## 결정 (20:03) 과 채택 규칙 (20:40, W60f/d/q 전체 결과가 나오기 전에 적음)
- 20:03 결정: 결합 방식 (b) 를 지금 형태 (W60, VINS 자체 초기화) 로는 **채택 안 함**, 기본값 `--swba 0` 유지.
- 채택 규칙 (기준 = OpenVINS GL60, W60 아님): 주 표 = 개발 칸 4개를 뺀 HS 38칸. 다음을 **모두** 만족해야 채택 후보:
  (1) HS38 발산 수 ≤ GL60 (1), (2) HS38 짝 비교 (둘 다 정상인 칸) swba − GL60 중앙값 < 0, (3) 손 루프 142148 · 142610 둘 다 loop % 가 GL60 보다 나빠지지 않음,
  (4) std60 165506 o0–o2 중앙값이 GL60 보다 나빠지지 않음. 151646 은 VIO 자체가 발산하므로 근거로 쓰지 않음.
- 사후 (선언 안 된 것): W60i (OpenVINS 로 초기화), W60ifd (조합) 는 개발 칸 4개에서만, 참고용. W60i 도 GL60 보다 나쁨 (짝 d_med +0.23 cm, 1/4 좋아짐).
- 정정 (21:05, 감독 지적): 규칙 (3)(4) (손 루프 · std60) 는 W60f/d/q 의 std60 (20:15–20:17) · 손 루프 (20:17–20:18) 결과 파일이 이미 만들어진 뒤 (20:40) 에 적었다
  (그 파일은 보지 않았지만 "결과 전 선언" 이 아님). 그래서 **판정 근거는 HS38 규칙 (1)(2) 로만** 하고 (3)(4) 는 참고로 보고한다.
- 선택 규칙 (21:05, full2 끝나기 전): f/d/q 중 둘 이상이 (1)(2) 를 통과하면 HS38 짝 중앙값이 가장 작은 것 하나. 조합 (f+d 등) · W60i 는 이번 판정에서 제외.
- 불채택이면 원인 (관찰: VINS 자체 초기화의 gyro bias 오류, diag_W60.txt) 을 적고 이 작업을 닫는다 (실험 추가 없음).
- std60 165506 o0–o2 는 녹화 1개 × 시작 오프셋 3 = 독립 반복 아님.
- 정정 2 (21:35, 감독 지적, sched_full2.log 로 확인): 규칙 (1)(2) 도 W60f HS 가 대부분 끝난 뒤 (20:40) 적었고, 선택 규칙 (21:05) 은 W60f 전부 · W60d 일부가 끝난 뒤 적었다
  (결과를 보지는 않았으나 "결과 전 선언" 은 아님). 비교 기준 = HS38 (개발 칸 제외) GL60 중앙 0.503 cm. GL60 = 루프 닫기 없는 OpenVINS.
  W60f/d/q 빌드 8e0a6cd0 = 858f123 과 b90f451 사이의 미커밋 작업 트리 (W60 기본 동작은 세 빌드에서 4칸 비트 동일).

## 독립 검증 뒤 정정 (10-27, verify_ovswba_20261027.md 확인 후)
- 브리지 버그 (고치지 않음, 기본 꺼짐): VINS 실패 재시작을 `clearState()` 뒤에 읽어 늘 0 으로 집계, 재시작 뒤 정렬 A 를 다시 잡지 않음 → 위 설계 ("VINS 재시작 → 새 정렬") 와 다르게 동작했다.
  실제 재시작 W60 14회/10칸. 판정 (HS38 규칙 1·2) 은 그대로.
- 151646 을 근거에서 뺀 규칙 (3) 문구는 틀린 전제 (옛 ov60eval 결과) 에 기댔다: GL60 에선 추적되고 네 변형 모두 좋아졌다. 판정은 HS 로만 했으므로 영향 없음.
- 원인 진술은 "상관 (관찰)" 으로 낮춤. gyro bias 원인 가설은 개입 실험으로 시험되지 않았다 (W60i 개발 4칸은 W60 에서도 발산하지 않던 칸).
