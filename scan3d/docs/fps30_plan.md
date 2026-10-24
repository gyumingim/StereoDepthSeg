# 30 fps (무거운 설정) vs 60 fps — 사전 등록 계획 (2026-10-24)

질문: 폰 계산 예산을 프레임 수 대신 창 크기·해상도에 쓰면 (30 fps + clones ↑ / 해상도 ↑) 60 fps 기준선보다 나은가?
이 문서는 **실행 전에 커밋**한다. 러너 `imu_learn/fps30_run.py`, 결과 `runs/fps30/`, 결과 문서 `docs/fps30_result.md`.

## 조건 (그 외 전부 최종 기준선 = base_nozupt + lever + 새 렌즈 + F1)
공통: 템플릿 config/mono, `cfg_over = num_opencv_threads=1, try_zupt=false, fi_max_cond_number=1e6, fi_max_baseline=200`
(= `runs/fi_limits` F1 과 같은 문자열), 원래 브리지, strict init (창 1.0 s), pts 200, lever `config/imu_cam_lever.json`,
렌즈 = 벤치 기본 규칙, 프레임 캐시 켬, gyro rebias 끔, imu_calib 없음.

| 이름 | fps | clones | 크기 | init 시차 (px) | 비고 |
|---|---|---|---|---|---|
| C1 | 60 | 11 | 640x360 | 15 | 기준선. HS·std60 루프는 `runs/fi_limits/*_F1` 재사용 |
| C2 | 30 | 11 | 640x360 | 15 | fps 만 바뀜 |
| C3 | 30 | 20 | 640x360 | 15 | fps + clones (창 길이 0.33 s → 0.63 s) |
| C4 | 30 | 15 | 960x540 | **22.5** | fps + clones + 크기. 시차 문턱은 px 라 폭 비율 1.5 배 (15 × 1.5) |

- 30 fps = 벤치 `--fps 30` (시간 격자에서 가장 가까운 프레임; 240 fps 원본은 8 장마다 1 장, std60 60 fps 원본은 2 장마다 1 장).
- 섞인 변수: C3 은 fps·clones 두 개, C4 는 fps·clones·크기·시차 문턱 네 개가 함께 바뀜. C2 → C3 → C4 차이로 몫을 추정만 함.
- init 실패 칸 (추적 0): 같은 칸을 **감사 init (창 1.5 s, 시차 = K 기반 strict 기본 2.30 px @640, 960 에서는 약 3.45 px)** 로
  다시 돌려 `<조건>a` 로 **따로** 보고. 판정에는 넣지 않음. std60 151646 은 C1 에서도 기준선 init 이 실패하므로 C1a–C4a 모두 돌림.

## 재사용 점검
- C1 은 다시 돌리지 않고 `runs/fi_limits/<key>_F1` 를 씀 (그 뒤 템플릿 기본값이 F1 로 바뀌었지만 cfg_over 로 같은 값).
  133820 o0 C1 을 이번 러너로 다시 돌려 포즈가 비트 단위로 같은지 확인. 다르면 C1 42 칸 전부 다시 돌리고 그것을 기준으로 씀.

## 시험 1 — HS 15 녹화 × o0/o1/o2 (213955 낙하 제외 42 칸, 칸당 1 회)
- 채점: 새 K GT `imu_gt_spl_cal1022`, 카메라 위치 자 (`lens_switch_gtscore.cam_metrics` / `bench_eval`, fi_limits 와 같음).
  발산 = 창 ATE > 50 cm 또는 5 초 상대 오차 > 30 cm. 추적 0 (init 실패) 칸도 발산 수에 넣고 따로 표시.
- 보고: 발산 수·칸, 정상 칸 창 ATE·Sim3 ATE·축척 중앙값, C1 대비 짝 차이 중앙값·평균·좋아진 칸 수, 오프셋별.

## 시험 2 — std60 루프 3 개 (`--gt none`, o0)
`hs_20261024_142148_std60`, `_142610_std60`, `_151646_std60`. 루프 오차 = 첫 추적 포즈 ↔ 움직임 끝 + 0.5 s 포즈
(`fi_limits_run.loop_eval` = scratchpad loop_eval.py 방식), 경로 길이, 움직임 중 n_slam.

## 시험 3 — std60 보드 녹화 (조건부)
실행 때 `docs/std60_board_check_20261024.md` 가 있으면 `hs_20261024_165506_std60` o0/o1/o2 × C1–C4 를 그 GT (`imu_gt_spl`) 로 채점.
없으면 건너뛰고 적음. 판정에는 넣지 않음 (녹화 1 개).

## 계산 시간
- 각 포즈의 ov_bridge `ms` (OpenVINS 한 프레임 처리 시간, 데스크톱). 본 실행 (동시 ≤ 8 + 다른 에이전트 GT 작업) 의 값은 부하 영향을 받으므로
  **따로 순차 실행 (동시 1)**: 조건마다 133820 o0, 214550 o0, 222747 o0, std142610 → 중앙값·p95. 실행 때 load average 기록.
- 폰 예산: 30 fps 33 ms, 60 fps 16.7 ms. 데스크톱 ms 는 폰보다 빠름 → 폰 가능 여부는 비율로 **추정**만.

## 판정 (C2, C3, C4 각각, C1 대비) — 모두 만족하면 "C1 보다 낫다"
- (a) 발산 칸 수 ≤ C1 (42 칸)
- (b) 짝 비교 (둘 다 정상인 칸) 창 ATE 차이 중앙값 ≤ −0.05 cm (0.05 cm 이상 좋아짐)
- (c) std60 루프 오차 평균이 C1 보다 크지 않음 (기준선 init 으로 둘 다 추적된 녹화만; 151646 은 감사 init 변형이라 참고)
- 결과는 판정과 상관없이 모든 수치 보고. 칸당 1 회 (오프셋 3 개가 반복 역할).

## 하지 않는 것
- 기본값 변경 없음. 채택은 결과 보고 후 사용자 결정. 동시 실행 ≤ 8.
