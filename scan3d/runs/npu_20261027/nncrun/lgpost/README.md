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
