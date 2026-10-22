# 렌즈 A/B (VIO) — 사전 등록 (2026-10-22, VIO 를 한 번도 돌리기 전에 커밋)

근거: `runs/lens_calib_round2_review/judge.md` "VIO: 기본값으로 바꾸기 전에 기존 렌즈 파일 대 새 2.22 노드 A/B 한 번 (같은 녹화·같은 채점). 나빠지지 않으면 기본으로."

## 무엇을 비교하나
| | 파일 | 모델 | fx / fy / cx / cy (1920×1080) | 640×360 VIO K (getOptimalNewCameraMatrix α=0) |
|---|---|---|---|---|
| A (지금 벤치가 2.22 녹화에 쓰는 것) | `config/lens/hs_1920x1080_f2.22.json` (10-19, 녹화 131326 하나, 119 장) | rational 8 | 1267.1 / 1266.8 / 963.0 / 546.9 | 428.06 / 427.91 / 321.15 / 181.76 |
| B (새 보정 2.22 노드) | `config/lens/hs_1920x1080_f2.22_cal20261022.json` (= `hs_1920x1080_focus_cal_20261022.json` 의 focus 2.22, 공동 맞춤) | k1 k2 p1 p2 k3 | 1303.9 / 1302.9 / 968.4 / 552.5 | 440.59 / 440.36 / 323.15 / 183.74 |

- 원시 화소 → 광선 방향 차이 (A 대 B): 가운데 0.40°, 중앙값 0.64°, 최대 1.06° (1300 px 기준 약 15 / 24 px). 큰 차이다.
- A 파일은 바꾸지 않는다 (B 는 새 파일). 벤치 코드 (`vio_bench_hs.lens_path`) 도 바꾸지 않고, 러너가 실행 안에서만 바꾼다.
- **섞인 변수**: B 는 K·왜곡 계수·왜곡 모델(rational → k5)이 한꺼번에 바뀐다. 그 결과 640×360 VIO K 와, 화소로 적힌 초기화·ZUPT 시차 문턱의 각도 의미도 약 3 % 바뀐다. 셋을 가르지 않는다 (이 시험의 질문은 "파일 교체가 나빠지지 않나").

## 같은 조건
- 브리지 `tools/ov_bridge_zbase` (고치지 않은 open_vins_diag 복사본, zupt_fixA 에서 옛 결과와 같은 바이트 출력 확인). ZUPT 수정 C 파일은 안 씀.
- 설정 = i3_confirm base: `zr.common(rec, off, 2.0, 0, raw=True, rebias=False)` → static init strict 1.0 s / 15 px, clones 11, noise cfg, 640×360, 60 fps, decode threads 2, `--imu-raw`, `--no-gyro-rebias`, 그리고 job 에서 `a.gyro_rebias = False` 명시, `num_opencv_threads=2`, still-hold 끔.
- 녹화 (초점 2.22, GT 있음): `hs_20261022_111836`, `133820`, `134204`, `134550`. 시작 오프셋 o0/o1/o2 = 0/1/2 s, 전체 길이. 칸 12 개 × A/B = 24 run. 칸마다 1 회 (VIO 는 같은 입력에 결정적; 오프셋 3 개가 반복 역할).
- 채점 = 벤치 그대로 (`--gt auto` = `imu_gt_spl`, SE3 `ate_cm`, Sim3 `ate_sim3_cm`, `scale`). **이 GT 는 렌즈 A 로 만든 것**이다 (`runs/board_rotation/gt/*_imu_gt.log`: "렌즈 값: config/lens/hs_1920x1080_f2.22.json, f 1267.1"; gt.npz 의 K 도 A). 그래서 아래 GT 민감도를 같이 한다.
- 동시 실행 ≤ 3 (GT 다시 만들기 포함), `nice -n 5`. 러너 `imu_learn/lens_ab.py`, 출력 `runs/lens_ab/`, 로그 `runs/lens_ab.log`.
- 동일성 관문: 첫 run `L_134204_o0_A` 의 포즈 스트림·`ate_cm` 이 `runs/zupt_fixA/BR_134204_o0_base` 와 **완전히 같아야** 나머지를 시작한다 (같은 브리지·인자·렌즈 A). 다르면 멈추고 보고.
- 각 결과 json 에 만든 시각, git HEAD, 코드·렌즈 파일 blob, 브리지·설정 sha256 (c5_run) 을 남긴다.

## 판정 기준 (돌리기 전에 고정)
- 칸별 짝 차이 d = ATE_B − ATE_A (SE3 `ate_cm`, GT A). 12 칸.
- 붕괴 정의 = `docs/zupt_fixA_plan.md` 와 같음: SLAM 전멸(일반 갱신 SLAM 수가 0.2 s 안에 ≥ 20 → ≤ 3) 뒤 5 s 안 최대 이동 > 0.5 m, 또는 run ATE > 50 cm. 결과 없음(오류·ATE 없음)도 붕괴로 센다.
- 새 붕괴 = 같은 칸의 A 는 붕괴가 아닌데 B 가 붕괴.
- **B 채택 ⇔ (1) 12 칸 d 의 중앙값 ≤ 0 그리고 (2) 새 붕괴 0.** 하나라도 어기면 채택 안 함 (A 유지).
- 같이 보고 (판정에 안 씀): Sim3 ATE 와 scale 의 A/B, 녹화별 d, 붕괴 칸 목록 (133820 은 기준선에서 붕괴 칸 — ZUPT 쪽 문제), d 의 부호 개수.

## GT 민감도 (GT 가 A 로 만들어진 문제)
- `imu_learn/lens_ab_gt.sh`: 134204 의 GT 를 렌즈 B 로 다시 만든다 — `imu_gt.py --rs --keep-corners --lens B --out-dir imu_gt_rs_lensB` → `spline_gt.py --knot 0.0125 --out-dir imu_gt_spl_lensB` (overnight_gt.sh 와 같은 단계, 새 폴더; 기존 imu_gt* 는 안 건드림). `imu_gt/align.json` (dt, R_ci, 자이로 바이어스) 은 A 때 것을 그대로 씀 (섞인 변수, 영향 작다고 봄 — 추정).
- 134204 o0–o2 의 A·B 를 GT B 로 다시 채점 (같은 VIO 를 다시 돌림, 포즈는 본 run 과 같아야 함 → 확인).
- `scale_ls.py` 로 GT A·GT B 의 IMU 축척 검사 (s, s_fixedlever) 를 나란히 보고.
- 해석 규칙: 134204 의 d 중앙값 부호가 GT A 와 GT B 에서 다르면, 판정 결과에 **"GT 의존"** 단서를 단다 (판정 규칙 자체는 위 그대로). 같으면 "GT 렌즈에 둔감 (134204 한 녹화 기준)".

## 결과
`docs/lens_ab_result.md` (+ `runs/lens_ab/summary.json`, 분석 `imu_learn/lens_ab_analyse.py`).
