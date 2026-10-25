# ovacc 2026-10-25 — OpenVINS 설정 정확도 (오프라인 GT, PC)

## 결론 (먼저)
- **L0 (히스토그램 NONE = 현재 라이브 설정)** 이 HS 42칸에서 B0 (HISTOGRAM) 보다 발산 8→4, 짝지은 ATE 중앙값 -0.023 cm (33쌍 21/12, p=0.16; 이 비교 당시 L0 에 154320 3칸이 없어 쌍에서 빠짐. 녹화 단위 8/3, p=0.23).
  다만 std60 라이브 모드 녹화 1개(165506, 3칸)에서는 NONE 이 3/3 나쁨 (1.19–1.33 → 1.39–1.74 cm). 녹화 1개라 확정 아님 → 사용자 결정(NONE 유지, HISTOGRAM 예비)은 그대로.
- **NC05 (IMU 잡음 4값 모두 ×0.5)** 만 확인 라운드(42칸 전부)를 통과: 발산 4→3 (214550_o0 회복), 짝지은 d_med -0.031 cm / d_mean -0.041 cm,
  녹화 단위 10/13 개선 (부호검정 p≈0.09, 0.05 수준에서 유의하지 않음; 칸 단위 30/38 p≈0.0005 는 같은 녹화의 o0/o1/o2 가 서로 상관이라 과대). o0/o1/o2 각각 같은 방향 (9/3, 10/3, 11/2). 각 칸 1회 실행(결정적)이라 반복 잡음은 재표집 안 됨.
  이득 크기 d_med -0.03 cm 는 o0 잡음 바닥(|d_med| < ~0.04) 안쪽 → 근거는 '일관된 방향 + 발산 1칸 감소' 뿐.
  std60 165506 에서는 L0 대비 2/3 칸 약간 좋음(1.76/1.68/1.33) 이지만 B0(HISTOGRAM) 보다는 3/3 나쁨. 같은 셋에서 LC(1.32/1.37/1.21)는 B0 와 비슷. 처리 시간 비용 없음(설정값만). → **라이브 후보 1순위, 단 실기 A/B 1회 후 채택 권장 (추정 이득 ~0.03 cm, 작음)**.
- 나머지는 잡음 또는 나쁨: LC(라이브 R_ci/dt) ±0, DTp2/DTm2 ±0 (DTp2 는 134550 에서 크게 나빠짐), FXp/FXm(초점 ±0.3%) = 잡음 바닥,
  G85 확인 라운드에서 발산 4→5, 23/14 (p=0.19) → 잡음. F10/F20/M60/S70/MD15/N300M80/SG15 = o0 선별만(12쌍, p≥0.15, 또는 칸별 대폭 흔들림) → 결론 아님.
  NC2 (잡음 ×2) 와 SG07, CH2 는 나쁨 쪽.

## 잡음 바닥
FXp/FXm (fx ±0.3 %, 사실상 무의미한 변화) 가 o0 12쌍에서 d_med -0.011/-0.009, d_mean -0.048/-0.002, 칸별 최대 0.4–0.5 cm 흔들림.
→ o0 12쌍의 |d_med| < ~0.04 cm 는 잡음으로 본다. 단일 칸 |d| > 0.3 cm 도 흔함(혼돈적 궤적).

## 조건 (모두 L0 기준, 같은 고정 바이너리 sha256 71bb15d9…, 단일 스레드 = 결정적, 반복 1회/칸)
L0 = fps30 C4 프로토콜 + histogram NONE. 자세한 프로토콜은 `acc_run.py` 머리말.
| 조건 | 내용 |
|---|---|
| B0 | ovopt B0 (HISTOGRAM), B0R = 재실행 4칸 동일성 확인(4/4 동일) |
| LC | 녹화별 GT 정렬 대신 라이브 R_ci·dt (variants.json) |
| LS | std60 R_ci 평균 (std60 셋만) |
| DTp2/DTm2 | 카메라-IMU 시간차 사전값 ±2 ms |
| FXp/FXm | fx,fy ×1.003 / ×0.997 |
| F10/F20 | fast_threshold 10/20 (기본 15) |
| G85 | grid 8×5 (기본 5×5) |
| SG15/SG07 | up_msckf/slam_sigma_px 1.5/0.7 (기본 1) |
| CH2 | chi2 배수 2 |
| NC05/NC2 | IMU 잡음 밀도·랜덤워크 4값 ×0.5 / ×2 (기본 acc 1e-2/1e-3, gyro 1e-3/1e-4) |
| M60, S70, MD15, N300M80 | max_msckf_in_update 60, max_slam 70, min_px_dist 15, num_pts 300 + msckf 80 |

## 표
`summary.md` (= `python3 summarize.py`): 조건별 칸 수, 발산, ATE 중앙값, 짝지은 d_med/d_mean, 개선/악화 수, 부호검정 p, |d|>0.3 cm 칸.
- 42칸 = HS 녹화 14개 × 시작 오프셋 0/1/2 (213955 = 폰 낙하, 실행했지만 요약 제외). "o0" 표 = 선별(오프셋 0 만, 12쌍).
- std60 표 = 라이브 카메라 모드 녹화 2개 × 3 (173432 는 1.0 s 초기화 창으로 초기화 안 됨 — 알려진 문제).
- 표의 ms 값은 PC 부하(다른 에이전트, load ~30) 에 오염 → 시간 비교에 쓰지 말 것.

## 파일
- `acc_run.py` (HS 42칸 러너·채점), `acc_std.py` (std60 셋), `score_sub.py` (o0/o1/o2 부분 채점), `summarize.py`, `round*.sh` (실행 순서), `addcond.py`.
- `results/score_*.json` (채점 결과, 칸별 값 포함), `results/cell_json.tgz` (칸별 실행 결과 JSON, 적용된 설정 `ovacc_applied` 포함), `results/bridge.json` (바이너리 해시).
- `logs/*.log` (실행 로그). 포즈 원본(.poses.jsonl.gz, ~0.4 GB)은 git 밖: WSL `~/scanner/ovacc_raw_20261025/` (원래 `/tmp/ovacc/ate/`).

## 라이브에 넣는다면 (권장, 아직 안 함)
`config/phone_ov_main/c4/kalibr_imu_chain.yaml`: accelerometer_noise_density 5.0e-03, accelerometer_random_walk 5.0e-04,
gyroscope_noise_density 5.0e-04, gyroscope_random_walk 5.0e-05. 위험(추정): 잡음을 낮추면 필터가 IMU 를 더 믿음 → 손으로 든 HS 녹화보다 진동 큰 드론 비행에서는 효과가 다를 수 있음. 드론 진동 녹화로 다시 확인 필요.
