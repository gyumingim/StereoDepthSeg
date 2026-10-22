# 자이로 바이어스 보정 → OpenVINS — 사전 등록 (Part 2)

만든 때 2026-10-22, 실행 전에 커밋. Part 1 (`docs/imu_calib_gain.md`, 커밋 0a18a48) 에서 D 가 B 를 이김 (7/8) → 사전 등록대로 Part 2 진행.
질문: Part 1 의 자이로 적분 이득이 실제 VIO 궤적 (ATE·요 드리프트) 이득이 되나.

## 조건 (OpenVINS 는 그대로, IMU 입력만 바꿈)
- **U** (보정 안 함): `recordings/<rec>` 그대로, `--imu-raw` (raw type 16/35 v0..2).
- **D** (정지마다 보정): `runs/imu_calib_gain/recs/<rec>/` = 원래 녹화의 모든 파일 심볼릭 링크 + 새 `imu.csv`.
  새 `imu.csv` 는 type 16 줄의 v0..2 만 v − b_D(t) 로 바꾸고 나머지 줄·칸은 그대로 (가속도 type 35 그대로).
  b_D(t) = Part 1 의 D 와 같은 정의 (`imu_learn/calib_gain.py`: `still_runs` 정지, 길이 ≥ 4 s 면 앞 2 s 잘라 평균, 끝난 정지 중 가장 최근 것,
  첫 정지가 끝나기 전에는 첫 정지 평균). 사후 게이트 D2 는 쓰지 않음.
- 두 조건 모두 i3 confirm base 와 같은 설정: `z2_isolate.py job --cfg-over num_opencv_threads=2 --still-hold "" --`
  + `z2_regress.common(rec, off, 2.0, 0, True)` (strict static init 1.0 s / 15 px, clones 11, noise cfg, imu-lead 2.0, 전체 길이, decode threads 2, --imu-raw), 동시 실행 ≤ 5.
- 녹화·시작 위치: i3 confirm 과 같음 — `hs_20261021_213341`, `214550`, `215348` (213955 제외), 시작 오프셋 0 / 1 / 2 s (`runs/i3_confirm/inputs/*.json`). 3 × 3 × 2 = 18 런.
- 런마다 새로 돌림 (U 도 새로). 기존 i3 confirm base 런 (같은 설정) 과 새 U 의 차이 = 런 사이 잡음 참고값으로 보고.

## 지표
- 주 지표: **ATE** (cm, 벤치 `ate_cm`, GT = imu_gt_spl, SE3 정렬).
- 보조: **요 드리프트율** (deg/분) — 추적 포즈마다 M(t) = R_world_cam(t) · R_bc(t)ᵀ (GT 시각 = 포즈 시각 + dt, 2 ms 안 매칭),
  요(t) = rotvec(M(t) M(t_first)ᵀ) 의 world z 성분 (OpenVINS world z = 중력), 시간에 대한 최소제곱 기울기의 절댓값.
  그 밖에 scale, rot_med_deg, 붕괴 여부 (i3 규칙: 붕괴 표시 또는 scale 0.5–2 밖 또는 ATE > 10 cm).
- 쌍 = (녹화, 오프셋) 9 개.

## 판정 규칙
- **D 가 낫다** = ATE 가 D 에서 더 낮은 쌍 ≥ 6 / 9 **그리고** 9 쌍 ATE 중앙값이 U 보다 20 % 이상 낮음.
- **D 가 나쁘다** = 같은 규칙을 반대로. 둘 다 아니면 "뚜렷한 차이 없음".
- 붕괴 수 (U vs D) 와 요 드리프트 중앙값은 따로 보고 (판정에는 안 씀).

## 검사
- 독립 검사 스크립트 `imu_learn/calib_gain_p2_check.py`: (1) 새 imu.csv 가 type 16 v0..2 외에는 원본과 바이트 같음, v0..2 차이가 따로 다시 계산한 b_D 와 1e-9 안;
  (2) 포즈에서 ATE 를 따로 다시 계산 (자체 Umeyama) 해 벤치 값과 비교, 쌍별 승패 방향이 같은지; (3) 요 드리프트를 다른 방식 (scipy 쿼터니언) 으로 다시 계산; (4) 판정 다시 계산.

## 섞인 변수 · 한계 (미리 적음)
- 녹화 3 개 (반복 = 녹화 3 × 오프셋 3, 오프셋은 같은 녹화라 독립 아님). OpenVINS 는 스레드 때문에 런마다 조금 다름.
- 214550 은 i3 base 에서 3 런 모두 붕괴 (scale 0.23) — 이 녹화의 결과가 판정을 좌우할 수 있음.
- OpenVINS 는 정적 초기화 때 바이어스를 잡고 온라인으로 따라가므로, D 의 이득은 주로 나중 정지 (약 70 s) 의 바이어스 계단에서 나올 것으로 예상 (추정).
  D 입력에는 정지가 끝나는 순간 약 40–50 mdeg/s 계단이 생김 — 필터에는 바이어스 계단으로 보임.
