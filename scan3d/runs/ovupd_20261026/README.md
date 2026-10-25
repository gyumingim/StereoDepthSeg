# ovupd 2026-10-26 — SLAM/MSCKF 업데이트 계산 줄이기 (branch `ovupd-20261026`, integ 14de3b8 기준)

## 결론
- 새 옵션 3개 (전부 기본 꺼짐, `all_exact` 에도 안 넣음): `ekf_sparse`(근사-정확), `sym_inplace`(정확), `init_gemm`(근사-정확).
- 기본값 / `sym_inplace` = integ 기준과 포즈 17자리 동일 (하네스 845, raw80 2327, W3 위에서도). `ident_pc_1.log`, `ident_pc_2.log`.
- `ekf_sparse`: SLAM 묶음의 H 가 블록 희소 (특징점마다 자기 점 + 클론 1-2개 + 보정) → M = P H^T 를 0 아닌 블록만으로, S 는 M 행 재사용.
  raw80 c4/s75/s100 포즈 차 최대 < 1e-4 cm. GT 42칸 ATE: s50/75/100 모두 A(W3)=B(W3+sym+sparse), 짝 0.000 cm, 발산 8칸 같음 (`pc_ate/`).
- PC 단계 시간 (raw80, 2회, PC 공유·부하 ~17 → 참고용): feed p50 s100 19.1→15.2 ms, s75 16.5→13.7, s50 11.5→10.4;
  ekf_ma s100 3.64→1.30 ms, ekf_sym 0.53→0.29 (`pc_prof/prof_pc_1.md`). 남은 큰 비용 = 공분산 갱신 (N^2·m, s100 PC 4.3-4.5 ms).
- init_gemm: delay_init 평균 0.65→0.59 ms (PC 1회) — 효과 작음, 추천 안 함.
- s75/s100 은 GT ATE 에서 s50 보다 낫지 않음 (0.6047/0.6035 vs 0.5914, ovopt 결과와 같음) → 이득은 "여유 시간", SLAM 점 늘리기 근거 아님.
- **폰 재생 시간: 없음.** U1 은 사용자 폰 사용 중 (Twitter/Brave, 방전 -1~-2 A) AP 53.2 로 중단 → 버림. V1 (재시도) 은 시작 대기 중 멈춤.

## 재개
`phone/ovupd_plan.sh run V1` (s100/s75 ABBA) → `V2` (s50 ABBA + s100 B vs B+par_cov) → `pull V1` → `prof_an.py`.
