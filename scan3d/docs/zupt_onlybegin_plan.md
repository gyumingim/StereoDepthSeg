# stock `zupt_only_at_beginning: true` 비교 — 계획 (2026-10-23, OB run 전에 커밋)

근거: `docs/metacog_audit_20261023.md` (하루에 결정 하나). A2b 와 T_cum 스윕은 중단했다 (A2b 는 셀 하나도 안 돌림, `runs/zupt_fixA2b.log`).
`docs/zupt_next_plan.md` 는 이 결정이 나올 때까지 보류한다.

## 질문
stock OpenVINS (패치 없는 zbase 빌드) 에 설정 한 줄 `zupt_only_at_beginning: true` ("OB") 를 주면, 기존 75 칸에서 A2 와 비기거나 낫나?
**그렇다면 ZUPT 패치 계열 (A, C, A', A2, A2b) 을 닫고 stock 옵션을 쓴다.** 아니면 A2 를 유지 후보로 남긴다.

## 실행
- `imu_learn/zupt_onlybegin.py` → `runs/zupt_onlybegin/*_OB`. 다리 = `tools/ov_bridge_zbase`, 설정은 base 와 같고 `zupt_only_at_beginning=true` 만 다름.
  gyro_rebias = False 명시, 스레드 2, still-hold 꺼짐, 동시 ≤ 5. base 는 `runs/zupt_fixA/*_base` 재사용, 먼저 base 2 칸 재실행이 비트 동일한지 확인.
- 75 칸은 개발용 데이터다 (이미 본 데이터). 이 결정은 "더 단순한 길로 갈지" 만 정한다.

## 채점 (`imu_learn/zupt_ob_score.py`, OB 와 A2 를 같은 방식으로)
- GT 중심 c = p_bc. 채점 창 = 첫 추적 포즈 ~ 마지막 유효 GT 표본.
- 발산 = 창 안 아무 5 s 창의 상대 오차 > 30 cm 또는 창 ATE > 50 cm (창 안 SE3 정렬).
- 진짜 정지 표류 = GT 속도 (±0.25 s) < 0.05 cm/s 인 구간 (≥ 1 s) 에서 VIO 위치 변화 합 / 정지 시간 (cm/s). 정지 ≥ 3 s 인 run 만. 녹화 값 = 반복 중앙값.

## 기준 (3 개)
1. **표적 발산 0**: 133820 o0–o2, 134550 o2, 222747 F o0–o2 (7 run) 모두 발산 아님.
2. **묶음별 발산 run 수 ≤ base**: 묶음 = board / i3 / F / T / O. 213955 (폰 낙하) 는 뺀다. 채점은 마지막 GT 표본까지.
3. **진짜 정지 표류**: 녹화별 OB ≤ 2.5 × base + 0.02 cm/s.
   - 여유는 A/A 에서 정했다 (`runs/zupt_onlybegin/aa_drift.json`, base o0–o2 끼리 36 쌍, 213955 제외): 모든 쌍이 통과하려면 k ≈ 2.0 (m = 0.02 cm/s) 이 필요 → 2.5 로 반올림. 표류 중앙값 0.013 cm/s, 쌍 차이 p90 0.15 cm/s.
   - 기준 1·2 는 base 대 base 가 정의상 통과한다 (같은 run). 예비 A/A (`docs/zupt_next_prep.md` 4) 처럼 오프셋끼리는 발산이 갈릴 수 있으므로 기준 2 의 1 run 차이는 잡음일 수 있다 (추정) — 결과에 그대로 적는다.

## 판정 규칙
- OB 가 세 기준을 모두 통과하고, A2 와 비교해 "기준 통과 수가 같거나 많고, 발산 run 수 (213955 제외) 가 A2 이하" 면 → **비기거나 낫다 → ZUPT 패치 계열을 닫고 stock 옵션을 쓴다.**
- 그 밖에는 → A2 를 후보로 남기고, OB 가 진 기준을 결과에 적는다.
- A2 는 같은 채점으로 보고만 한다 (A2 의 옛 판정 "불채택" 은 바꾸지 않는다).
- 반복 수: 칸당 1 회, 녹화당 오프셋 3 개. 섞인 변수 없음 (OB 는 설정 한 줄, A2 는 코드 패치).
