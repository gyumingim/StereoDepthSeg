# 최종 기준선 확인 (2026-10-23)

사용자 결정: **기준선 = base_nozupt + 카메라–IMU lever + 새 렌즈** (6 면 가속도 보정 없음, 자이로 rebias 없음).
러너 `imu_learn/final_baseline_run.py`, 결과 `runs/final_baseline/score.json` (VIO 45 run 의 json·로그도 같은 폴더).

## 결론 먼저 (새 K GT, 카메라 위치 자, 213955 낙하 제외 14 녹화 × o0/o1/o2 = 42 칸, 칸당 1 회)
| 설정 | 발산 | 정상 칸 | 창 ATE 중앙값 (cm) | Sim3 ATE 중앙값 | Sim3 축척 중앙값 | \|s−1\| 중앙값 |
|---|---|---|---|---|---|---|
| **최종** (nozupt + lever + 새 렌즈) | **5** | 37 | **0.675** | **0.67** | 0.997 | **0.0065** |
| 오늘 아침 (config/mono ZUPT 항상 켬, 옛 렌즈, lever 없음) | 16 | 26 | 1.088 | 1.02 | 1.006 | 0.021 |
| (참고) V4 = nozupt, 옛 렌즈, lever 없음 | 6 | 36 | 0.817 | 0.805 | 1.007 | 0.019 |

- 짝 비교 (둘 다 정상인 칸): 아침 → 최종 25 칸, 짝 차이 중앙값 **−0.28 cm**, 최종이 좋은 칸 22/25 (1.08 → 0.68 cm). V4 → 최종 35 칸, −0.17 cm, 29/35.
- 오프셋별 최종: o0 발산 1 / ATE 0.67, o1 발산 3 / 0.73, o2 발산 1 / 0.68 cm. 축척 중앙값 1.000 / 0.995 / 0.997.
- 최종에서 발산한 칸: **224323 o0·o1·o2** (아침·V4 에서도 발산하는 녹화), **214550 o1** (889 cm, 같은 녹화 o0·o2 는 0.48·0.69 cm), **131326 o1** (o0·o2 는 0.72·0.68 cm).
  아침 설정의 발산 16 칸에는 221925 o1·o2 (초기화를 못 해 추적 0) 가 들어 있다.
- 옛 K GT 로 채점해도 발산 수는 같다 (5 / 16 / 6). 최종의 축척 중앙값은 옛 K GT 에서 0.973, 새 K GT 에서 0.997 — GT 렌즈가 축척 판단을 바꾼다 (`docs/lens_switch_20261023.md` 7 절).

## 조건
- 최종: config/mono + try_zupt=false, 원래 브리지 (ov_bridge/build), clones 11, strict init, 창 1.0, 시차 15, 프레임 캐시 (새 렌즈 맵, 오프셋 0/1/2 미리 만듦), opencv 스레드 2,
  gyro_rebias 끔, imu_calib 없음, `lever = config/imu_cam_lever.json`, 렌즈 = 벤치 기본 규칙 (2.22 dpt → `hs_1920x1080_f2.22_cal20261022.json`; run JSON `lens_source = shared`, `lens_sha256` 22af64d1…).
  15 녹화 × o0/o1/o2, 동시 12, 벽시계 626 s.
- 비교 행은 **다시 돌리지 않고 다시 채점만** 함: 아침 = `runs/phone60/<key>_base` (config/mono, ZUPT 항상 켬, 옛 렌즈 fx 1267, lever 없음), V4 = `runs/phone60/<key>_V4`.
- 채점: 새 K GT `imu_gt_spl_cal1022` (옛 K GT `imu_gt_spl` 도 score.json 에 있음). 창 ATE·발산 = 카메라 위치 (T_world_cam0) 로 `zupt_ob_score.run_metrics` (발산 = 창 ATE > 50 cm 또는 5 초 상대 오차 > 30 cm), Sim3 ATE·축척 = 벤치 evaluate 를 저장된 포즈에 다시 돌린 값.
- 점검: 옛 K GT 로 다시 채점한 값이 저장값과 135/135 같음. 최종 o0 15 칸의 포즈가 렌즈 A/B 의 새 렌즈 run (`runs/lens_switch/<key>_o0_new`, `--lens-file` 로 명시) 과 15/15 같음 → 기본 규칙이 새 파일을 고른다는 것도 확인.

## 섞인 변수 · 한계
- 아침 → 최종은 세 가지 (ZUPT 끔, lever, 렌즈) 가 동시에 바뀜. 하나씩의 몫: ZUPT 끔 = V4 행 (`docs/phone60_ablation_result.md`), lever = `docs/imucal_result.md`, 렌즈 = `docs/lens_switch_20261023.md`.
- 칸당 1 회 (오프셋 3 개가 반복 역할). 발산은 오프셋만 바꿔도 갈린다 (214550, 131326 의 o1).
- 새 K GT: 13 개 새 녹화에서는 모서리 오차·IMU 축척 검사 모두 옛 K 보다 낫지만, 10-19 두 녹화(131326·154320)는 옛 K 가 조금 더 맞음 (옛 렌즈를 131326 으로 보정했기 때문으로 보임, 추정). 그 두 녹화의 새 GT 는 보드 칸 63.294 mm (옛 GT 63.403) 라 0.17 % 섞임.
- GT 의 dt·R_ci·자이로 바이어스 (align.json) 는 옛 K 로 구한 값 그대로.
- 213955 (낙하) 는 모든 설정에서 발산이라 표에서 뺌.
