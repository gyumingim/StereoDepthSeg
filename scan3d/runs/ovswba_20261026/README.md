# ovswba 2026-10-26 — VINS-Mono 슬라이딩 윈도우 BA 를 OpenVINS 옆에 (결과)

**결론: 이 입력 · 설정의 VINS 보정 (방식 b) 은 지금 형태로는 이득 없음 — 채택 안 함, 기본값 `--swba 0` 유지.**
관찰 (상관, 원인 미확인 — 개입 실험 없음): 발산한 칸은 거의 모두 VINS 자체 초기화 직후 폭주 (|v| 수십 m/s) 했고, 그때 gyro bias 도 컸다
(폭주 9칸 중 8칸 |bg| ≥ 0.03 rad/s; 예외 1칸, 발산 없이 bg 가 큰 반례 있음). 발산하지 않은 칸에서도 VINS 궤적이 60 fps OpenVINS 보다 덜 정확했다.
"VINS-Mono 원본이 나쁘다" 는 뜻은 아님: OpenVINS KLT 입력, 폭주 차단 없음, 아래 브리지 버그 조건에서의 결과. 원인 후보 (10 Hz 입력 · FOCAL 460 가중 · 초기화 · 특징 추적기) 는 **추정**.

**알려진 버그 (고치지 않음, `--swba` 기본 꺼짐)**: `vins_bridge.cpp` process() 가 실패 표시를 VINS `clearState()` (failure_occur = 0) 뒤에 읽어
실패 수가 늘 0 으로 찍히고, 재시작 뒤 4-DoF 정렬 A 를 다시 잡지 않음 (PLAN 설계와 다름). 실제 VINS 재시작: W60 14회 / 10칸 (HS 발산 9칸 전부 + l142610),
W60f 12/8, W60d 14/10, W60q 12/8 (독립 검증 verify_ovswba_20261027.md). 발산 9칸 중 7칸은 첫 재시작 전에 이미 0.3 m 이상 벗어나 판정은 그대로;
raw 발산 수 · diag 의 d_end · 손 루프 142610 은 재시작 뒤 구간이 섞인 값.

브랜치 `ovswba-20261026` (integ2-20261026 12bbf1c 에서). 계획·규칙·정정 기록 = `PLAN.md`.

## 한 것
- `scan3d/ov_bridge/vins/`: VINS-Mono vins_estimator (github.com/HKUST-Aerial-Robotics/VINS-Mono 90dabb5, **GPL-3**) 거의 그대로 이식.
  원본 그대로 커밋 518f950 → 수정 커밋에서 차이만 보임. 수정 (`ovswba` 주석): ROS 대체 헤더 `vins/compat/`, readParameters 삭제 (브리지가 전역값 설정),
  Ceres 2.2 Manifold (`PoseLocalParameterization`, `QuaternionManifold`, 수식 동일), 시간 측정 멤버, FOCAL_LENGTH 를 변수로 (기본 460 그대로), init=1 함수 (사후, 아래).
- `vins/vins_bridge.{h,cpp}`: estimator_node process() 루프 (ROS 없이), OpenVINS KLT 트랙 (gyro_klt=5) 을 10 Hz 로 입력 (VINS feature_tracker 안 씀: 추적 비용 0, camodocal 불필요, 같은 관측 비교).
  출력 = pose 줄의 `"swba"` 객체 (4-DoF 보정, ovloop 와 같은 속도 제한 0.10 m/s · 3 °/s) — **VIO / 제어 필드는 바뀌지 않음**.
- `main.cpp`: `--swba 0|1` (기본 0), `--swba-cfg K=V,..` (noise, freq, iters, solver_time, ext, focal, align_delay, init, ...), `--swba-log`, `--swba-sync` (PC 결정적 평가). PC·안드로이드 CMake 에 소스 추가 (**안드로이드 빌드는 시험 안 함**).
- 평가기 `swba_run.py` (같은 자), 표 `tables.py` → `tables.txt`, 발산 진단 `diag_W60.txt`.

## 같은 자
기준 = OpenVINS **GL60** (dpvo ov_ref_run.py 의 기존 실행 = integ2_ate 프로토콜: 960x540, pts 200, clones 15, histogram NONE, rec60 + gyro_klt=5, `--drop-late 0`) —
**루프 닫기 없는 OpenVINS** (이 브랜치 실행은 `--loop 0` 을 붙였고, GL60 빌드 c640f534 에는 루프 코드가 없음). 주 표 = 개발 (선별) 칸 4개 (133820_o0, 224323_o0, 214138_o0, 221052_o0) 를 뺀 **HS 38칸**, GL60 중앙 0.503 cm, 발산 1.
swba = 모든 프레임에서 보정 포즈 (VINS 첫 해 전에는 VIO 와 같음), raw = VINS 최신 프레임 포즈 (10 Hz 프레임만, 같은 프레임의 VIO 와 짝 비교).

## 결과 (반복 1회; 결정적 실행 — W60 은 빌드 3개에서 4칸 비트 동일)
| 조건 (빌드) | HS38 발산 | HS38 swba 중앙 cm | HS38 짝 d_med vs GL60 (좋아짐/n) | std60 165506 o0–o2 swba (GL60 1.50/1.55/1.70) | 손 루프 142148 / 142610 % (GL60 3.27 / 5.20) |
|---|---|---|---|---|---|
| W60 = VINS 그대로 (0fbfec36, 858f123) | 9 | 1.038 | +0.584 (3/29) | 1.21/1.25/1.32 | 7.9 / 21.1 |
| W60f focal 661 (8e0a6cd0*) | 9 | 1.021 | +0.548 (4/29) | 1.20/1.23/1.29 | 7.4 / 21.2 |
| W60d 정렬 2 s 지연 (8e0a6cd0*) | 8 | 0.894 | +0.261 (4/30) | 1.38/1.40/1.56 | 1.2 / 22.6 |
| W60q 입력 20 Hz (8e0a6cd0*) | 12 | 1.107 | +0.870 (5/26) | 1.54/1.63/1.62 | 3.2 / 5.4 |

\* 8e0a6cd0 = 858f123 과 b90f451 사이의 **미커밋 작업 트리** 빌드 (b90f451 은 init 옵션만 더한 cdd5bf11 을 만듦). W60 동작은 0fbfec36 · 8e0a6cd0 · cdd5bf11 에서 개발 칸 4개의 포즈 + swba 로그가 비트 동일.
- **판정 (PLAN 규칙, HS38 의 (1) 발산 ≤ 1 그리고 (2) 짝 d_med < 0)**: 넷 다 (1)(2) 모두 불통과 → 선택 없음, **불채택**.
  규칙 기록 시점: 규칙 (1)(2) 는 W60f HS 대부분이, (3)(4) 는 std60 · 손 루프 결과 파일이, 선택 규칙은 W60f 전부가 이미 만들어진 뒤에 적음 (결과는 안 봤지만 "사전 선언" 아님; PLAN.md 정정 줄).
- std60 은 **녹화 1개 × 시작 오프셋 3** (독립 반복 아님) 에서만 좋아짐 (W60 −0.30 cm). std60 173432 은 모든 조건에서 추적 0 (초기화 실패, 기존과 같음).
- 손 루프 (GT 없음, 약한 근거; % 는 각 궤적 자기 경로로 나눔, swba 경로가 14–18 % 짧음): 12개 (4조건 × 3) 중 6개 나빠짐 (하나는 거의 같음: W60q 142610 5.20 → 5.39 %),
  6개 좋아짐. 151646 (GL60 은 추적됨, 31.8 cm / 7.3 %) 은 네 조건 모두 좋아짐 (3.1 / 4.0 / 3.6 / 1.9 %). 앞선 판의 "151646 은 VIO 자체가 발산" 은 옛 산출물 (ov60eval L60) 근거라 틀림.
- HS42 (참고): GL60 0.570 / 발산 1, W60 swba 1.048 / 9, raw 0.951 / 11 (같은 프레임 VIO 0.572 / 0).
- W60d 의 raw · rawvio 는 정렬 뒤 프레임만이라 다른 프레임 묶음 (rawvio 0.496) — W60d raw 를 다른 조건과 직접 비교하지 말 것.

## 발산 진단 (관찰, `diag_W60.txt`)
- W60 swba 발산 9칸: 모두 VINS 자체 초기화 (SfM + 정렬) 직후 폭주 (|v| 6–50 m/s) + 재시작 있음; 나머지 33칸은 폭주 0. 첫 비선형 |bg| 는 9칸 중 8칸 ≥ 0.03 rad/s
  (예외 214138_o2 0.015). 발산 안 한 칸은 대부분 |bg| ≤ 0.01 이지만 0.048 (131326_o2), 0.022 (154320 ×3), 0.024 (214138_o0) 도 있음 → **상관**, bg 는 원인일 수도 SfM/정렬 실패의 증상일 수도 있음.
  (`diag_W60.txt` 의 failure 0 은 위 버그 때문에 틀린 값.) W60f · W60q 에는 초기화와 무관한 발산도 있음. 정렬 지연 (W60d) 은 일정 오프셋만 줄이고 발산은 못 막음.
- 사후 (선언 안 됨, 개발 칸 4개만, 참고): W60i = VINS 창을 OpenVINS 상태로 초기화 → swba 중앙 0.80 (W60 1.72) 이지만 여전히 GL60 보다 나쁨 (짝 d_med +0.23, 1/4). W60ifd (조합) 0.81. 이번 판정에서 제외.
- 잡음 선택 (미리 선언, 개발 칸 2개): noise 1 (VINS EuRoC 값) 1.08 vs noise 2 (OpenVINS 밀도 × √rate) 1.49 → noise 1.

## 동일성
- VIO 필드 (ms 류, map, swba 제외 해시) = GL60: `--swba 0` (Z60) 은 std 6 + 손 루프 3칸, `--swba 1` (각 W 조건) 은 HS 45/45 · std 6/6 · 손 루프 3/3.
- 기본값 0 이면 VINS 코드는 호출되지 않음 (Estimator 는 첫 사용 때 생성되는 함수 정적 객체).

## CPU (주장 아님)
PC 에서 VINS 프레임 (10 Hz) 당 전체 p50 약 30 ms / p95 약 50 ms (해 ~20, 주변화 ~1–12) — 다른 에이전트 (ovxfeat 등) 와 동시 실행, 부하 7–18 → **CPU 근거로 쓰지 않음**. 폰 재생 측정 안 함.

## 남은 것 / 다시 볼 때
- 다시 시도한다면 (추정 순서): 위 브리지 버그 (실패 집계 · 재정렬) 수정, OpenVINS 로 초기화 (init=1; 발산 칸에서는 아직 시험 안 됨) + 폭주 차단, 입력 빈도·가중 재검토,
  VINS 자체 특징 추적기로 바꿔 보기, 그 뒤에 (a) 방식 (CI EKF 갱신). 지금 데이터로는 VINS 가 필터보다 나빠 (a) 도 이득 기대 낮음.
- 안드로이드 빌드 · 폰 시간 측정 안 함. `score_W60.json` = 스모크 (옛 빌드 04b1e53c, std 1칸).

## 파일
`PLAN.md`, `swba_run.py`, `tables.py`, `tables.txt`, `diag_W60.txt`, `score_Z60_W60.json` (W60 전체), `score_W60f_W60d_W60q.json` (+ `.txt` 출력), `score_W60_W60n2.json` (잡음 선택),
`score_W60_W60e_W60f_W60d_W60q.json` (개발 선별), `score_W60_W60i_W60ifd.json` (사후). 실행 원자료 = `~/scanner/tmp_ovswba/{ate,ate_v,ate_v2}` (저장소 밖).
