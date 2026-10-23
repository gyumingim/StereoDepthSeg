# phone60 나누기 — 계획 (2026-10-23, 실행 전 커밋)

배경: `docs/phone60_result.md` — phone60 (10 개 값 동시 변경) 발산 16 → 23. 온라인 초점거리가 +5 % 흐름 (원인 후보, 추정).
base 자체도 42 run 중 16 개 발산한다 → 결과는 녹화별 표로 같이 낸다.

## 변형 (각각 기준 설정에서 값 하나만 바꿈)
| 이름 | 기준 | 바꾼 것 | 묻는 것 |
|---|---|---|---|
| V1 | phone60 | `calib_cam_intrinsics false` | intrinsics 를 끄면 phone60 의 손해가 사라지나 |
| V2 | base | `zupt_only_at_beginning true` | 좋아진 녹화 (224323, 222747, 134550) 가 이 값 덕인가 |
| V3 | base | `calib_cam_intrinsics true` | intrinsics 만으로 base 가 나빠지나 (f 흐름 가설 확인) |

## 실행
- `imu_learn/phone60_run.py` (`CONDS=V1,V2,V3`), 변형은 `cfg_over` 한 개. 나머지는 phone60_plan 과 똑같다:
  15 녹화 × 오프셋 0/1/2, stock 다리, `--frame-cache`, 원시 IMU (O 2 개는 보정 IMU), gyro_rebias False, opencv 스레드 2, 동시 ≤ 12, 칸당 1 회.
- base·phone60 run 은 다시 돌리지 않고 `runs/phone60/` 것을 그대로 쓴다.

## 기준 (phone60_plan 과 같음, 각 변형 대 base)
채점 = `zupt_ob_score.run_metrics`.
- **(a)** 발산 run 수 (213955 제외) ≤ base (16).
- **(b)** 그 변형과 base 가 둘 다 정상인 칸의 창 ATE 중앙값 ≤ 1.2 × base + 0.3 cm.
둘 다 통과 = 그 변형 "통과". 보고: 기준, 녹화별 ATE 표, 발산 수, 시간, (V1·V3) 온라인 f 값.
