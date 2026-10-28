# lgpost (10-28 저녁) — LighterGlue 짝 결정(후처리)을 NPU 모델 안으로

## 12차 사전 기준 (측정 전 작성, 시각 = 이 파일을 넣은 커밋). 폰 없음 - PC + Exynos Lab + 웹 NNC Profiler
### 무엇을 바꾸나
- 지금: 모델 L6_a16mc 출력 out [1,512,1026] = A | BT | z0 | z1, 폰 CPU 가 P = A*BT^T, sc = log P + logsig(z0)_i + logsig(z1)_j,
  행·열 argmax, 상호 최대, exp(max) > 0.1 (9차 lgpost 2, 폰 약 6.4 ms, 1 회차).
- 새 모델 (`lgpost.py`, 몸통 = lgnpu L6 와 같은 가중치·연산, LGLN=native, tanh, 6층):
  S = m0 m1^T, sc = LogSoftmax(S, 행) + LogSoftmax(S, 열) + E, E = [ls0, 1] @ [1, ls1]^T (K=2 MatMul, 브로드캐스트 없음 - g12/g13 실패 회피).
  출력 1개 out [1,N,3] = (행 argmax 열번호, 열 argmax 행번호(열 j 를 j 번째 줄에), 행 최대 sc). CPU 는 O(N): m1[m0[i]] == i 그리고 rmax_i > ln 0.1.
- 판: **L6p** (N=512, 입력 x [1,512,512] 그대로), **L6pm768** (N=768, 입력 x [1,768,513], 513번째 칸 v = 1 실제 점 / 0 채움).
  마스크 판: 모든 attention 을 [qX, vX] @ [kY, pbY]^T (pb = 30(v-1)) 로 -> 채움 점은 key 로 exp(-30) 무게, 최종 S·E 에도 pb ->
  채움 점의 행·열 점수 <= -30. CPU 는 i, m0[i] < Nvalid 도 확인 (이중 안전).
- 비교 기준판 **L6r** = 지금 모델(A|BT|z0|z1) 을 새 판과 **같은 보정**으로 다시 컴파일 (보정 차이와 후처리 차이를 떼어 보려고).

### 판정 규칙 1 - PC fp32 정확성 (13 평가 쌍 lgev512 / lgev1024, `pc_check.json`)
- 기준 = lgnpu L6 fp32 출력 + 지금 CPU 후처리 (lgnpu.post = nncrun lgpost 1/2 와 같은 계산) 의 짝 집합.
- L6p: torch fp32 와 ORT(ONNX) 둘 다 13/13 쌍에서 짝 집합 md5 동일이어야 합격 (하나라도 다르면 불합격, 다른 쌍 수를 적음).
- L6pm768: (a) 실제 512 + 채움 256 (0 채움 / 무작위 채움 둘 다) -> N=512 기준과 md5 동일, (b) 실제 674 + 채움 94 -> 674 점 기준과 동일,
  (c) 실제 768 -> 768 점 기준과 동일, (d) CPU 이중 안전 없이도 채움 점이 들어간 짝 0 개. 모두 13/13 이어야 합격.
- 주의: 이 PC 확인은 개발 중 한 번 돌렸다 (18:2x, 커밋 전 - 모두 일치). 이 커밋 뒤 같은 스크립트로 다시 돌린 결과를 공식으로 쓴다.

### 판정 규칙 2 - Lab 컴파일과 원격 시간
- Lab: a16w8 (activation 16, kernel 8, asymm, percentile), multicore, O2, Gen-6 = L6_a16mc 와 같은 설정.
  **보정 = 합성 무작위 입력 40 개** (kpts U[-1,1], 기술자 = 정규화된 가우스 64-d, 마스크 판은 실제 점 수 N/2~N 무작위, seed 1234).
  사용자 데이터는 올리지 않는다 (Lab 홈이 비었으므로 L6_a16mc 의 실보정 40 쌍은 쓸 수 없다). 그래서 L6r 를 같은 보정으로 같이 컴파일한다.
- 컴파일이 안 되는 연산(LogSoftmax/ArgMax/ReduceMax 등)이 나오면: 같은 계산의 다른 연산(Log(Softmax), 등)으로 바꾸고 다시 PC 규칙 1 을 돌린 뒤에만 컴파일.
- Lab 골든(eais 시뮬, 무작위 골든 입력 1 개) vs 같은 ONNX PC fp32: 짝 집합 일치율을 적음 (참고만, 무작위 입력이라 실제 정확도 아님).
- 원격: 웹 NNC Profiler, Exynos 2400, Duration 5 s, **같은 세션에서** L6_a16mc(기존), L6r, L6p, L6pm768 각 >= 2 회.
  올리는 것 = 우리 nnc + 그 Lab 골든 입·출력 bin(무작위) 만. 로그인 안 돼 있으면 못 잰다고 적고 멈춤.
- 판단: 이득 추정 = 6.4 ms (폰 CPU 후처리, 9차 1 회차) + 출력 복사 감소분 - (원격 L6p - 원격 L6r) - CPU 마무리(PC 에서 잼, O(N)).
  > 0 이면 "다음 폰 측정 후보", <= 0 이면 "이득 없음". L6pm768 는 원격 시간과 L6p 대비 배수만 적는다 (판정 없음).
  폰에서 재기 전 값은 모두 **미검증 / 추정**. 짝 품질(양자화 뒤)은 무작위 보정이라 폰 실데이터 측정 전에는 판정하지 않는다.

## 12차 결과 1 - PC (10-28 18:16~18:20)
- **규칙 1 합격** (`pc_check.json`, 18:16:39, 사전 등록 커밋 eabf624 18:16:30 뒤 재실행): L6p torch/ORT 13/13 쌍 짝 집합 md5 동일.
  L6pm768: 실제 512+채움 256 (0 / 무작위), 674+94 (0 / 무작위), 768 실제 모두 13/13 md5 동일, CPU 이중 안전 없이도 채움 짝 0, 채움 행 최대 점수 -60 ~ -84.
- 정정: 사전 기준의 "개발 중 18:2x" 는 실제 약 18:15 (커밋 직전). 같은 결과.
- 정정: 보정 생성은 lgpost.py(torch) 대신 `gen_cal.py` (numpy, seed 1234, 같은 분포) - Lab 에서 생성해 올림 양을 줄이려고. 사용자 데이터 없음.
- ONNX (npu_upload/lgpost/up, md5): L6r c37f236c, L6p a336b3b5, L6pm768 eaf1680d. 새 연산: LogSoftmax 2, ArgMax 2, Cast 2, ReduceMax 1, Sigmoid/Log 2.
- CPU 마무리 (O(N), PC x86 1 스레드, N=512): 약 3 us. 출력 512x1026 f32 (2.1 MB) -> 512x3 (6 KB).
## 컴파일·원격 시간 - 아직 못 함 (18:39 GUI 사용 중단 지시)
- Lab: zip 업로드 + 합성 보정 생성 + eais init 3판까지. 컴파일 안 함. Lab 서버 켜져 있음 (lgpost 가 18:36 켬).
- RESUME: (1) 각 판 dbgen_spec 입력 = cal<판> raw (1,N,512|513), Gen-6, multicore, a16w8, use_randomdb false, batch 40 -> conversion/compile.
  (2) 골든 vs ORT 짝 일치 (`gold_post.py`, 참고). (3) 원격 E2400 같은 세션 L6_a16mc/L6r/L6p/L6pm768 각 2회, 위 판정식.
  ArgMax/LogSoftmax 미지원이면 사전 기준대로 같은 계산 다른 연산으로 바꾸고 규칙 1 재실행.

## 12차 진행 2 (19:00~, CDP 크롬 + Jupyter API, pyautogui 안 씀)
- Lab 홈이 또 비어 있어 lgpost_up.zip 다시 올림 (md5 71d6864f 일치). 합성 보정 Lab 에서 생성 (L6r 과 L6p 보정 c000 md5 같음 25ff7d66; PC numpy 생성 md5 와는 다름 - numpy 판 차이 추정, 분포·seed 같음).
- **eais: "OPTYPE : LOG_SOFTMAX is not supported!"** (L6p, L6pm768 변환 실패). 사전 기준대로 같은 계산 다른 연산으로 바꿈:
  LogSoftmax -> Log(Softmax) (`LGLSM=logofsm`, 기본값). 규칙 1 다시 돌림 (`pc_check_logofsm.json`): L6p torch/ORT 13/13 md5 동일,
  마스크 판 5 경우 모두 13/13 동일, 채움 짝 0 -> 합격. 새 ONNX md5 L6p 3011be07, L6pm768 a7c7c5ae (옛 판은 npu_upload/lgpost/v1_logsm).

## 12차 결과 2 (19:00~19:55) - 모델 안 짝 결정: **eais 변환 못 함** / 대신 CPU 후처리 log 없는 판 (PC 에서 4.6 배)
### 원격 Exynos 2400 (웹 NNC Profiler, CDP 크롬 자기 탭, Duration 5 s, 같은 세션 19:10~19:13) - 미검증
| 판 | 입력·골든 | 평균 실행 (us) | SNR |
|---|---|---|---|
| L6_a16mc (기존, 실보정) | 1 회: 예전 골든(실보정 쌍 파생) / 2 회: L6r 합성 골든 | 13792 / 13841 | 49.1 / 12.4 (남의 골든) |
| L6r (같은 모델, 합성 무작위 보정 40) | 합성 | 13862 / 13861 | 61.1 / 61.1 |
- 보정을 합성으로 바꿔도 시간 같음 (13.8~13.9 ms). L6r 배치 = L6_a16mc 와 같음 (CU MUL96 Softmax52 CLN48 GeLU48, nnc 4,628,480 B).
- **규칙 위반 1 건**: L6_a16mc 1 회차에 예전 L6_a16mc 골든 입력(실보정 쌍 = 사용자 데이터 파생)을 프로파일러에 올림. 이후는 합성 입력만.
- L6r 골든(eais a16 시뮬, 합성 입력) vs PC fp32: 행 argmax 일치 82 %, 열 85 % (무작위 입력이라 짝 0 - 참고만).
### L6p / L6pm768: 변환 실패 6 판 (모든 판 PC 규칙 1 은 13/13 md5 동일 합격; 각 판 pc_check_<lsm>.json)
| 판 (LGLSM) | eais 결과 |
|---|---|
| logsm (LogSoftmax) | OPTYPE LOG_SOFTMAX is not supported |
| logofsm (Log(Softmax)) | 변환기가 다시 LOG_SOFTMAX 로 묶어 같은 실패 |
| pe (log(SM행 * SM열 + eps)) | "Add Constant Support" (행 softmax x 열 softmax 곱이 상수 입력으로 보임) |
| vec / bc / bc2 (마지막 축만, 벡터 붙이기) | Concat/브로드캐스트 모양 오류 ("Sizes of tensors must match") |
- 작은 시험 모델(N=64, Lab 무작위 보정, `npu_results/lgpost/lgpost_conv_logs.zip`)로 확인한 규칙:
  - softmax(S, 축1) 단독은 되지만, 같은 S 의 행·열 softmax 를 곱하면 실패 (c2~c5), 따로 만든 S 끼리도 모양 오류 (c7, c8).
  - ReduceMax 출력(1 칸 벡터)은 Linear 출력과 Concat 못 함 (p2, p6). [1,1,N] 행 벡터 더하기 + ArgMax/ReduceMax 는 됨 (q1).
    [1,N,1] 열 벡터 더하기, log(sigmoid) 행 더하기, 상수(0 가중치 Linear) 붙이기는 실패 (r1~r6, s5, s6, t1~t3).
  - **ArgMax 와 Log 는 ENN 의 CPU 에 배치됨** (CPU:ArgMax, CPU:Eltwise-LOG). N^2 개 원소의 ArgMax·Log 를 모델 안에 넣어도
    NPU 가 아니라 ENN CPU 가 돈다 -> 모델 안으로 옮겨도 이득이 작을 것 (추정).
- 판단 (사전 규칙): L6p 원격 시간이 없어 **판정 불가**. 남은 길: 비-NPU 연산 없이 짝을 정하는 다른 식이 있어야 하는데, 이번 시험으로는 못 찾음.
### 대신: CPU 후처리 log 없애기 (`cpu_prod.py`, `cpu_prod_bench.cpp`) - 폰 앱에 바로 넣을 수 있음
- sc_ij = log(A_ij BT_ji) + ls0_i + ls1_j 는 행마다 A_ij BT_ji sig(z1_j) 에, 열마다 A_ij BT_ji sig(z0_i) 에 단조 -> argmax 는 곱(float32)으로,
  log 는 행마다 한 번(문턱)만. 모델·출력은 지금(L6_a16mc) 그대로.
- PC fp32 13 쌍: 지금 후처리(lgpost 2 계산)와 짝 집합 md5 13/13 동일 (Python `cpu_prod_check.json`, C++ 13/13).
- PC x86 1 스레드 (13 쌍 x 10, 3 회, `cpu_prod_bench_pc.log`): lgpost 2 p50 5.30~5.48 ms -> 곱 판 **1.14~1.18 ms** (4.6 배).
  폰 추정: 6.4 ms x (1.15/5.3) = **약 1.4 ms (추정, 미검증)**. 폰 nncrun 에 lgpost=3 으로 넣고 같은 짝 md5 + 시간 잴 것 (폰 측정 필요).
- Lab 서버는 lgpost 가 19:58 Stop My Server 로 정지. Lab 홈의 올린 파일: lgpost_up.zip(모델 3 + 합성 보정 생성기), probe ONNX 들, 합성 보정 (사용자 데이터 없음).
