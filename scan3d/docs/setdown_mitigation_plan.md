# 내려놓기 실패 대책 시험 — 사전 등록 (2026-10-21, 실행 전)

배경: docs/setdown_failure.html — 224323 에서 내려놓은 뒤 손이 화면에 들어오자 ZUPT(시차만 봄)가 떨어지고,
SLAM 특징도 0 이라 IMU 만 적분 → 4 s 만에 오차 48–65 cm. 그 대책을 같은 녹화로 시험한다.

## 고정 조건 (모든 실행 같음)
- 녹화 hs_20261020_224323 (주 대상), hs_20261020_223815 (같은 약점, 덜 드러남). **전체 길이** (`--max-secs 0`).
- IMU 원본(유형 16/35, `--imu-raw`), 잡음 cfg, clones 11, 640x360 @60, IMU 앞당김 2 s (벤치 기본, endstill_vio 와 같음).
- 초기화 = init_window_result.html 1 등: 정적 전용 (`--init-cfg strict` = 동적 끔, IMU 문턱 0.3 m/s²) + `--init-window 1.0 --init-disp 15`.
- 시작 위치 `--start-off 0 / 1 / 2` s (반복 3 회). 실행기 imu_learn/c5_run.py (진단 브리지 tools/ov_bridge_diag, DEBUG 로그).
- 코드: 이 문서와 같은 커밋의 imu_learn/vio_bench_hs.py (기본값이면 출력 그대로인 옵션 2 개 추가), imu_learn/setdown_mitig.py.
  브리지·OpenVINS 라이브러리는 **안 고침** (해시는 결과 JSON binary_sha256).
- 동시 실행 3 개, nice 5. 2 녹화 × 3 위치 × 5 조건 = 30 실행 (약 55 분 예상).

## 조건 5 개 (한 번에 하나만 바꿈)
| 이름 | 바꾸는 것 | 뜻 |
|---|---|---|
| base | 없음 | 기준 (ZUPT = 시차만, zupt_chi2_multipler 0) |
| Z1 | zupt_chi2_multipler 1 | OpenVINS ZUPT 의 IMU 판정 켬: 시차가 떨어져도 IMU 잔차 chi² < 1×95 % 문턱이고 \|v\| < 0.1 m/s 면 정지 갱신 (UpdaterZeroVelocity.cpp: `!disparity_passed && (chi2 > mult*chi2_check \|\| \|v\| > max_vel)` 이면 거절) |
| Z2 | zupt_chi2_multipler 1 + zupt_noise_multiplier 1 | Z1 에서 ZUPT 잡음 배수 10 → 1 (IMU 판정이 더 깐깐, 갱신은 더 셈). 두 값 동시 변경임을 표시 |
| Z3 | zupt_chi2_multipler 5 | OpenVINS 기본값 (가장 느슨) |
| H | `--still-hold 1.0:0.05:0.05` | 바깥 IMU 정지 검출: 직전 1.0 s 동안 \|자이로\| 최대 < 0.05 rad/s 이고 \|가속도\| 표준편차 < 0.05 m/s² 이면 정지 시작 때 프레임을 붙잡아 계속 보냄 → 시차 0 → OpenVINS ZUPT 통과. 원인·결과 시각 그대로(인과), 브리지 코드 변경 없음 |

- H 의 문턱은 **IMU 만 보고** 정했다 (VIO 결과 보기 전): hs_20261020 녹화 5 개에서 W 0.5 s 는 222747 손 움직임 중 짧은 오검출 (84.6–85.0, 189.7–191.2 s 등),
  W 1.0 s · 0.05 · 0.05 는 시작 정지와 끝 정지만 잡음 (224323: 118.3 s → 끝, 223815: 133.8–140.6 s 등). 같은 녹화로 정한 문턱이라 **추정**.
- 속도 0 의사 갱신(velocity pseudo-update)은 OpenVINS 에 그 갱신이 없고 (explicitly_enforce_zero_motion 은 코드 상수 false) 라이브러리를 고쳐야 해서 이번엔 안 함.
  H 는 같은 효과(ZUPT 강제)를 브리지 입력 쪽에서 낸다.

## 지표 (실행마다, 시각 = 녹화 첫 IMU 표본부터 초, runs/endstill_tmp/endstill_check.json t0)
1. **마지막 정답 창 오차**: VIO 와 정답을 2 ms 안에서 짝 → **t ≤ 100 s 짝으로만** SE3 맞춤 → 마지막 짝 정답 시각부터 3 s 앞까지의 오차 중앙값·최대 (cm).
2. **끝 정지**: endstill_vio.analyse_one (창 = endstill_check.json used.end): VIO 속도 평균·최대 (cm/s), 위치 이동 (cm).
3. **전체 ATE**: 벤치 ate_cm (전 구간 SE3), 축척.
4. **첫 100 s 회귀**: t ≤ 100 s 짝의 SE3 ATE, Sim3 축척. 초기화 여부·초기화 시각(로그 'successful initialization').

## 판정 (녹화 단위, 위치 3 개의 중앙값; 결과 전 고정)
- **도움**: 224323 마지막 창 오차 중앙값 ≤ base 의 50 % **그리고** 끝 정지 이동 중앙값 ≤ base 의 50 %.
- **회귀 없음**: 두 녹화 모두 첫 100 s ATE 중앙값 ≤ base × 1.10 + 0.1 cm, 6 실행 모두 초기화되고 첫 100 s 축척 0.5–2·ATE ≤ 10 cm,
  223815 마지막 창 오차 중앙값 ≤ base × 1.5 + 0.5 cm.
- 권고: "도움 + 회귀 없음" 중 224323 마지막 창 오차 중앙값이 가장 작은 것. 같으면(±10 %) 설정만 바꾸는 쪽(Z) 우선.
- base 가 이번 초기화 설정에서 실패를 재현하지 않으면 (224323 마지막 창 오차 중앙값 < 10 cm) "재현 안 됨" 으로 적고 판정은 끝 정지 이동만으로.

## 감시 지표 (막지 않고 알리기만, 4번 요구)
- **특징 게이트 F_N**: 초기화 뒤 프레임마다 (ZUPT 통과 안 됨) 그리고 (MSCKF+SLAM 갱신 특징 0) 이 N 프레임 연속이면 경보. N = 5 (≈0.08 s), 30 (0.5 s).
  DEBUG 로그에서 셈 (endstill_setdown_check.py 와 같은 파싱). 보고: 첫 경보 시각, 그때 오차, t ≤ 100 s 의 경보 수(헛경보).
- **충격 표시 I**: 원본 가속도 \|a\|−g > 15 m/s² 뒤 3 s 안에 H 의 IMU 정지 판정이 시작되면 "착지". 보고: 시각, t ≤ 100 s 헛표시 수.
- 모든 조건의 로그에 적용 (조건마다 표).

## 한계 (미리 적음)
녹화 2 개, 같은 방, 손으로 내려놓음 (드론 착지 아님). 시작 위치 3 개는 독립 시행이 아님 → 모든 결론 **추정**.
