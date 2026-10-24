# 특징점 초기화 한계 (fi_max_baseline · fi_max_cond_number) 시험 계획 (2026-10-24, 실행 전 등록)

## 배경
std60 방 크기 루프 감사 (scratchpad audit_std60): OpenVINS 특징점 삼각측량 한계 `fi_max_baseline` (기본 40) 와
`fi_max_cond_number` (기본 10000) 가 손 움직임 중 새 특징점을 전부 버렸다 (장면 깊이 > ~1 m, 60 fps × clones 11 ≈ 0.18 s 창).
둘 다 풀면 (1e8/1000 또는 1e6/200) 루프 오차 0.92 m → 0.04–0.05 m. HS 보드 녹화 2 개도 좋아졌지만 1 회씩이고 감사 설정은 최종 기준선과 다름.

## 준비 (실행 전에 끝냄, 결과 비교 아님)
- `config/mono`, `config/base_nozupt` 템플릿과 폰 설정 생성기 (`ov_bridge/android/make_phone_ov_main.py` → `config/phone_ov_main/*`)
  에 두 키를 **기본값 그대로** (40.0 / 10000.0) 넣음 → `--cfg-over` 로 바꿀 수 있음. 동작 변화 없음.
  - 키 이름: OpenVINS `ov_core/src/feat/FeatureInitializerOptions.h` 가 `fi_max_baseline`, `fi_max_cond_number` 를 읽음.
    DEBUG 로그로 확인: `--cfg-over fi_max_baseline=41,fi_max_cond_number=1.5e6` → `max_baseline: 41.000`, `max_cond_number: 1500000.000`.
  - 같음 확인: 새 템플릿 + 스레드 2 로 133820 o0, 214550 o1, 134204 o0 를 다시 돌린 포즈가 `runs/final_baseline` 과 3/3 비트 단위 같음.
  - 폰 설정: 다시 만든 7 변형의 estimator_config 는 두 줄 (+주석) 만 추가, kalibr 파일·remap bin 은 같음.
- 결정성: 같은 3 칸을 스레드 2 로 2 회, 스레드 1 로 2 회 → 4 회 모두 포즈가 비트 단위로 같고 final_baseline 과도 같음.
  (감사에서 133820 base 가 저장값과 달랐던 것은 설정 차이 — 감사는 프레임 캐시·창 1.0/시차 15 를 안 씀 — 로 보임, 추정.)
  → 이번 비교는 세 조건 모두 **num_opencv_threads=1**, base 도 같은 설정으로 **다시 돌림** (재사용 안 함; final_baseline 과 같은지 45 칸 모두 검사해 보고).

## 조건 (그 외 전부 최종 기준선 = `imu_learn/final_baseline_run.py` 와 같음)
| 이름 | cfg_over |
|---|---|
| base | num_opencv_threads=1, try_zupt=false |
| F1 | base + fi_max_cond_number=1e6, fi_max_baseline=200 |
| F2 | base + fi_max_cond_number=1e8, fi_max_baseline=1000 |

공통: 템플릿 config/mono, 원래 브리지, clones 11, strict init (창 1.0 s, 시차 15), pts 200, lever `config/imu_cam_lever.json`,
렌즈 = 벤치 기본 규칙, 프레임 캐시 켬, gyro rebias 끔, imu_calib 없음. 동시 실행 ≤ 12. 러너 `imu_learn/fi_limits_run.py`, 결과 `runs/fi_limits/`.

## 시험 1 — HS 15 녹화 × o0/o1/o2 (칸당 1 회)
- 채점: 새 K GT `imu_gt_spl_cal1022`, 카메라 위치 자 (final_baseline 과 같은 `lens_switch_gtscore.cam_metrics` / `bench_eval`).
  발산 = 창 ATE > 50 cm 또는 5 초 상대 오차 > 30 cm. 213955 (낙하) 제외 → 42 칸.
- 판정 (F1, F2 각각, base 대비):
  - (a) 발산 칸 수 ≤ base
  - (b) 짝 비교 (둘 다 정상인 칸) 창 ATE 차이 중앙값 ≤ +0.1 cm (0.1 cm 넘게 나빠지지 않음)
  - (c) 개선 보고: 짝 차이 중앙값·평균, 좋아진 칸 수, 오프셋별, Sim3 ATE·축척 중앙값
  - (a)(b) 둘 다 만족 = "HS 보드 녹화에서 해가 없음". 둘 다 만족하면 (b) 차이가 더 작은 쪽을 후보로 둔다 (같으면 덜 극단인 F1).
- 섞인 변수 없음 (한 조건당 두 키만 바뀜, 두 키는 함께 바뀜 — 따로의 몫은 이 시험으로 모름).

## 시험 2 — std60 루프 녹화 3 개 (GT 없음, `--gt none`)
`hs_20261024_142148_std60`, `_142610_std60`, `_151646_std60` × base/F1/F2, o0.
- 루프 오차 = 첫 추적 포즈 ↔ 움직임 끝 (자이로 |ω| − 바이어스 > 0.08 rad/s 의 마지막 시각) + 0.5 s 의 포즈 (±10 포즈 평균), scratchpad `loop_eval.py` 방식.
- 경로 길이 (그 시점까지, 6 포즈 간격), 움직임 구간 n_slam 중앙값·0 비율도 보고.
- 감사와 달리 최종 기준선 init (창 1.0/시차 15) 와 프레임 캐시를 씀 → 감사 수치와 직접 같지 않을 수 있음.
- 판정 기준 없음 (보고만). 렌즈는 std60 기본 규칙 파일 (작업 사본이 커밋과 다를 수 있음 → run JSON 의 lens_sha256 기록).

## 하지 않는 것
- 기본값 변경 (템플릿·폰 설정 모두 40/10000 유지). 채택은 결과 보고 후 사용자 결정.
