# init 완화 시험 계획 (2026-10-24, 실행 전 커밋)

러너 `imu_learn/init_relax_run.py`, 결과 `runs/init_relax/` (`score.json`), 결과 문서 `docs/init_relax_result.md`.

## 질문
최종 기준선 (base_nozupt + lever + 새 렌즈 + F1) 의 init (창 1.0 s / 시차 15 px) 대신 감사 init (창 1.5 s / 시차 2.30 px,
= 벤치 strict 기본 `make_ov_config.init_params`, f ≈ 440 px × tan 0.3°) 을 기본으로 바꿔도 되는가?
근거: std60 보드 2 녹화에서 감사 init 이 6/6 칸 초기화 (기준선 3/6), 창 ATE 1.42 vs 2.25 cm, 축척 오차 1.3 vs 5.4 %
(`docs/std60_board_check_20261024.md`); std60 루프 151646 은 감사 init 으로만 초기화.

## 조건 (init 두 값만 다름)
- **base** = `runs/fi_limits/<key>_F1` 재사용 (config/mono 템플릿, try_zupt=false, lever, 벤치 기본 렌즈, strict init 창 1.0 / 시차 15,
  F1 = fi_max_cond_number 1e6 / fi_max_baseline 200, clones 11, 프레임 캐시, OpenCV 스레드 1).
  재사용 점검: 133820 o0 를 지금 코드로 다시 돌려 (`133820_o0_base__chk`) 포즈가 F1 저장본과 같은지 본다. 다르면 base 45 칸을 다시 돌린 뒤 비교.
- **aud** = base 와 같고 init 창 1.5 s / 시차 2.30 px (init_imu_thresh 0.3 은 둘 다 같음).
- 동시 작업 ≤ 8 (다른 에이전트가 돌고 있음).

## 데이터 · 채점
1. HS 15 녹화 × o0/o1/o2, 새 K GT `imu_gt_spl_cal1022`, 카메라 위치 자 (`lens_switch_gtscore.cam_metrics` / `bench_eval`), 213955 (낙하) 제외 → 42 칸.
   발산 = 창 ATE > 50 cm 또는 5 초 상대 오차 > 30 cm. 짝 비교 = 둘 다 정상인 칸의 창 ATE 차이 (aud − base).
2. std60 루프 3 녹화 (142148, 142610, 151646), `--gt none`, o0, 루프 오차 = `fi_limits_run.loop_eval` (스크래치패드 loop_eval.py 와 같은 계산).
   base = `runs/fi_limits/std<HHMMSS>_F1` 재사용, aud 는 새로 돌림.
3. std60 보드 2 녹화 (165506, 173432) × o0/o1/o2: `runs/std60_board_20261024/<HHMMSS>/o*_base|aud` 재사용 (같은 F1 템플릿, 스레드 2).
4. 칸마다 init 시각 (녹화 첫 프레임부터 초) 과 init 순간의 움직임:
   - 요청된 표시: init 순간 ±50 ms 의 GT 카메라 속도 > 2 cm/s 또는 자이로 (type 4) > 3 °/s → "init 중 움직임".
   - OpenVINS 정적 init 은 창의 앞 절반이 정지이고 뒤 절반에서 움직임 (jerk) 이 생기는 순간 초기화하므로 (소스 기억, 추정)
     위 표시는 대부분의 칸에서 켜질 수 있다. 그래서 **정지로 가정한 앞 절반** [t−W, t−W/2] 의 GT 속도 최대 / 자이로 최대도
     같은 문턱 (2 cm/s, 3 °/s) 으로 표시한다. std60 루프는 GT 가 없어 자이로만.

## 채택 기준 (미리 정함, HS 42 칸으로 판정)
- (a) aud 발산 수 ≤ base 발산 수.
- (b) 짝 창 ATE 차이 중앙값 (aud − base) ≤ +0.05 cm.
- (c) aud 에서만 새로 발산한 칸이 있고, 그 칸의 aud init 이 움직임 표시 (위 두 표시 중 하나라도) 를 받았으면 불합격.
- 세 개 모두 통과하면 채택: config/mono · config/base_nozupt 템플릿의 init 값, `imu_learn/final_baseline_run.py`,
  `ov_bridge/android/make_phone_ov_main.py` (→ `config/phone_ov_main/*` 다시 생성) 를 1.5 / 2.30 으로 바꾸고 `docs/final_baseline_20261023.md` 에 적는다.
  벤치 strict 기본 (`make_ov_config.init_params`) 은 이미 1.5 / 0.3° 라 그대로.
- std60 루프·보드는 판정에 넣지 않고 보고만 한다 (GT 없음 / 이미 본 데이터).

## 섞인 변수 · 한계 (미리)
- 창과 시차 두 값이 함께 바뀜 → 각 값의 몫은 모름.
- 칸당 1 회 (오프셋 3 개가 반복 역할). 보드 std60 은 스레드 2, 나머지는 스레드 1 (fi_limits 에서 base 45 칸 스레드 1/2 비트 동일 확인).
