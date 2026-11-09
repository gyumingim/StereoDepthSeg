# gpuba 2026-11-09 — featloop kf_ba (S, seed 0) 를 GPU 로 빠르게 (결과는 그대로)

브랜치 `gpu-ba-20261109` (featloop-20261105 7384db5 위), 워크트리 `~/scanner/StereoDepthSeg-gpuba`. PC 만 (폰 없음, 창 안 띄움).
질문: `kf_ba.py --feat sift --loop none --seed 0` (featloop 표의 S) 에서 시간이 어디에 드는지 재고, 느린 부분을 GPU 로 옮겨
결과 (축척 k, 보드 깊이 |dZ| p90) 를 바꾸지 않고 빨라지나. 학습 특징 (XFeat 등), SALAD 루프, VI-BA 는 이미 기각 → 안 함.
이 0–3 절은 **11 개 녹화 비교를 돌리기 전에 커밋** (사전 등록). 바꾸면 아래에 날짜·이유를 덧붙인다.

## 0. 시작 전에 안 것 (11-09 20:00–20:11, 165506 만)
- SIFT 추출·매칭은 **이미 GPU** (pycolmap 4.2.1 cuda12 빌드, `has_cuda` True, `Device.auto`). 165506 139 장 추출 0.2–8 s.
  `matching_s` = GPU 매칭 + CPU 기하 검증 (RANSAC, 시드 0) 합.
- featloop 표의 BA 시간 (예: 165506 287 s) 은 다른 작업과 겹쳐 돈 값. 조용할 때 같은 CPU BA = 36 s (165506).
- CPU BA 안에서 시간 = 거의 모두 Schur 축소 행렬 만들기 (165506: 36 s 중 33.6 s; 관측 짝 66 만 개 × 6×6 블록, numpy einsum + bincount).
- **새로 특징을 뽑으면 결과가 달라짐**: 165506 을 지금 처음부터 다시 돌리면 검증 짝 1844 (featloop 때 1829), 점 8477 (7956).
  같은 패키지 (pycolmap 날짜 10-17) → GPU SIFT/매칭의 실행마다 흔들림으로 추정 (미확인). 그래서 결과 비교는 **같은 db** 로 한다 (아래 2).
- `--db-from` (featloop ba_S 의 특징 + 날 매칭) CPU 재실행은 featloop ba_S 자세를 재현 (최대 차 5e-8, BLAS 다중 스레드 반올림).

## 1. 바꾼 것
- `scan3d/scanA/ba_torch.py`: `kf_ba.ba_lm` 와 같은 LM·Schur 단계를 torch float64 로 GPU 에서 (같은 상수·같은 수락 규칙).
  다른 점 = 합 순서 (GPU 축약, GPU LU), Rodrigues 식 (cv2 대신 torch). `torch.use_deterministic_algorithms(True)` → 실행마다 같음.
- `kf_ba.py --ba-device cpu|cuda` (기본 cpu = 지금과 같음). cuda 일 때 GPU 잠금 (`gpu_guard`, 상한 0.6) 을 BA 끝까지 쥠.
- 시간 기록만 더함 (숫자 영향 없음): `timing.read_db_uf_s`, `tracks_s`, `ba_phases` (setup / jac / schur / solve / update_cost, 관측 짝 수, 풀이 횟수).

## 2. 비교 방법
- 녹화: featloop table.md 와 같은 H 11 개 (133820 134550 134204 214550 213341 215348 221052 221925 222747 223815 224323; 214138 뺌).
- 두 판 모두 `--db-from` featloop 의 S db (ba_S 또는 ba_S_gpu 의 db.db: 같은 특징·같은 날 매칭), `--seed 0`, 같은 VIO·GT·캐시.
  CPU 판 = `--ba-device cpu` (지금 코드), 새 판 = `--ba-device cuda`. 둘 다 이번에 새로 돌림 (같은 환경).
- 채점: `scan_offline.py` (featloop 와 같은 인자, `--compress --kf-step 1`, `TORCH_COMPILE_DISABLE=1`, GPU 상한 0.6) 를 두 판 자세에 똑같이 →
  축척 k (`sim3_scale_would_be`, %) 와 |dZ| p90 (cm). 녹화·판마다 1 회.
- 시간: 위 db-from 실행의 `runtime_s` (검증 + 트랙 + 삼각측량 + BA) 와 단계별 시간. 따로 처음부터 (특징 추출부터) 도는 시간은
  165506 + 20261022_133820 두 녹화에서 CPU 판 / 새 판 각 1 회 (조용한 때, 다른 무거운 작업 없음 확인).

## 3. 판정 (사전 등록)
새 판이 **통과** = 아래 셋 모두:
1. 축척 k: 11 개 **모든** 녹화에서 |k_new − k_cpu| ≤ 0.1 %p.
2. |dZ| p90: 11 개 **모든** 녹화에서 (new − cpu) / cpu ≤ +5 %.
3. 시간: 11 개 db-from 실행 `runtime_s` 합이 CPU 판보다 작음, 그리고 처음부터 도는 두 녹화 모두 새 판이 빠름.
하나라도 어기면 "실패" 로 그대로 적음 (문턱 안 바꿈). 반복 1 회, 섞인 변수 = 없음 (같은 db·같은 시드, BA 장치만 다름).
