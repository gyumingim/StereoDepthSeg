# 입력·과정 감사 3차 (2026-10-20 15:35, 읽기 전용)

대상: 정정 커밋 71021b9, 진행 중인 C5 (커밋 ead75fb·4ea51d6·178cfc4, runs/c5, runs/c5_progress.log), 리드가 오늘 한 주장.
수정한 파일 없음 (이 보고서만 씀). 실행 중인 C5 는 건드리지 않음 (ps·로그·결과 JSON 읽기만).
"가정" = 이 감사에서 직접 확인하지 못한 것.

## (a) 정정 내용 재계산

| 무엇 | 근거 | 판정 |
|---|---|---|
| 1290 번 / 2617 s (43.6 분) | runs/gyro_bias_stability.json android_cal.gyro.n_changes=1290, duration_s=2617.4 | 맞음 |
| CP 기울기 3.9/7.0/4.4 mdeg/s/°C | temp_correlation.CP slope 3.869/7.043/4.426 | 맞음 (단 새 발견 1) |
| 첫 카메라 구간 3.3 분, CP +16 °C, 배터리 +8 °C | camera_intervals_s[0]=1302.7~1498.7 s (3.27 분), T_blocks CP 28.68→44.54 (+15.9), battery 27.3→34.8 (+7.5); 원시 온도 +16.1 / +8.2 | 맞음 |
| x61/y112/z70 mdeg/s | 15.86 × slope = 61.4/111.7/70.2 | 산술은 맞음, 단 **외삽** (새 발견 1) |
| 레버암 x 4.8 / 크기 5.1, z −1.48±0.67 | recordings/hs_*/imu_gt_spl_imu/imu_params.json r: 11 개 평균 (4.82, 0.99, −1.48) cm, std (0.23, 0.48, 0.67), 크기 평균 5.19, 평균의 크기 5.14; 물리 보정 크기 4.57 | 맞음 ("약 5.1~5.2") |
| 스플라인 −3 % | spline_gt_result.html:17 1.018→0.988 | 숫자 맞음. today_20261020_easy.html:95 에는 p≈0.13 단서가 없음 (문서 간 불일치) |
| today_20261020_easy.html:98 "망가뜨렸나? — 아니다" | calib_vio_verify.md:73,124 (둘 다 성공한 5 짝 중 4 짝에서 보정 쪽 ATE 가 나쁨) | 제목 "아니다" 가 본문·calib_vio_check 의 "판정 불가" 보다 강함 → 문구 정정 필요 |
| 정정으로 새로 생긴 오류 | 수치 오류 없음. gyro_temp_table.py 가 숫자를 데이터에서 계산하게 바뀐 것은 좋음. "칩(CP)" 표기는 새 발견 2 | — |

## (b) C5 시험 과정

| 무엇 | 근거 | 위험도 | 할 일 |
|---|---|---|---|
| 판정 규칙 사전 등록 | 계획 ead75fb 15:11:53 → 시간 조각 15:13:59 → 변경 기록 커밋 4ea51d6 15:21:39 → 행렬 시작 15:21:43 / 재시작 15:24:58; runs/c5 결과 mtime 전부 15:26 이후 | 낮음 | 순서 맞음. 단 변경 기록 본문 시각 "15:30" 이 커밋(15:21)보다 늦음 → 오기, 15:21 로 |
| strict 값 단일 출처 | vio_bench_hs.py:83-91 이 make_ov_config.init_params (make_ov_config.py:125-131) 를 호출, 결과 config_used 에 실제 값 기록 (init_max_disparity 2.30 px) | 낮음 | 맞음 |
| 코드·설정 버전 기록 | 결과 JSON: git_head 178cfc4, bench_blob 25157b5, make_ov_config_blob, template_blobs, lens_blob, bench_dirty=false | 중간 | 빠진 것: ov_bridge_diag **바이너리 해시** (build/ 추적 안 됨), c5_run.py·collapse_signals.py blob (bench_dirty 가 이 둘을 안 봄). config_used 는 DEBUG 로 바꾸기 전 값 (c5_run.py:49-50) |
| 섞인 버전 방지 | c5_matrix.sh: `[ -f name.json ]` 이면 건너뜀; c5_analyse.py 는 bench_blobs 를 나열만, 거르지 않음 | 중간 | 분석에서 blob 이 2 개 이상이면 판정 중단 |
| 동시 실행 통제 | 4 개 병렬인데 c5_run python 이 각각 CPU ~300 % (영상 디코드) → loadavg 32~39 / 16 코어 (uptime, ps) | 중간~높음 | 동적 초기화는 벽시계 한도 (init_dyn_mle_max_time 0.05 s) 가 있어 loose·strict-dyn 결과가 부하에 흔들림. 끝의 반복 4 건(혼자 실행)과 꼭 비교, 다르면 병렬 2 개로 재실행 |
| 잡음 주/보조 | 결과 noise="cfg" (make_ov_config.IMU_NOISE), m10 은 보조로 뒤에 배치 | 낮음 | 맞음. 단 변경 기록 1 의 이유 "Allan 은 자이로만, 가속 없음" 은 **틀림**: gyro_bias_stability.json allan.accel_raw N≈3.7e-4 m/s²/√Hz (문서에 VRW 표 있음). cfg 값은 Allan 의 자이로 ~14 배, 가속 ~27 배 |
| **주 설정 (a) 가 시험이 안 됨** | 지금까지 strict 7/7 실행이 "too few tracked poses" (114000 ×6, 115620 o0); loose 는 초기화됨 (115620 o0 붕괴 재현: scale 0.0027) | **높음 (해석)** | init_dyn_use false + 2.3 px 정지 판정 → 손에 든 녹화에서 초기화 안 됨. c5_analyse.py 는 "untested" 를 "사라짐 아님"으로 세서 **"withdrawn"** 출력 → 이건 반증이 아니라 **시험 불가**. 보고는 "C5 판정 불가 (주 설정 시험 안 됨 n/7)". 실질 정보는 (d) strict-dyn (정지 문턱만 엄격, 동적 허용) — 사전 등록상 보조이므로 "사후 해석" 표시 |
| 누출·조용한 대체 | c5_analyse.py:84-86 예외를 analyse_error 로 삼키고 판정에서 뺌; JSON·poses 없는 실행(브리지 죽음)도 조용히 빠져 'untested' 로 감 | 중간 | 빠진 실행 수를 summary 에 따로 적기. 채점 정답 imu_gt_spl (gt_uses_imu=false) 의 매듭 간격은 평가 녹화+Android 자이로로 골랐음 (이미 문서화, 영향 작음 추정) |

## (c) 리드 주장 대조

| 주장 | 판정 | 근거 |
|---|---|---|
| 물리 보정 효과 판정 불가 | 맞음 | calib_vio_verify.md:73,124 |
| 벤치의 느슨한 초기화가 붕괴의 주된 후보 (시험 중) | 후보로는 타당. 그러나 지금 C5 (a) 로는 시험 불가 | 위 (b). critic_debate:93 "정지 초기화 붕괴 11/15, 동적 6/15" → 동적도 40 % 붕괴, 느슨한 정지 판정만으로는 다 설명 안 됨 |
| A 거꾸로 아님 | 코드 수준에서 맞음 (모든 곳 A·a+ba). 물리적 방향은 데이터로 판정 불가 (6면 시험 남음) | collapse_verify.md:18,104 |
| 자이로 3.9/7.0/4.4 mdeg/s/°C (CP) | 숫자 맞음, 적용 범위 한정 | 새 발견 1 |
| 스플라인 정답 약 3 % 좋음 | 숫자 맞음. 단 판단 기준에서 뺀 3 초 IMU 점수 기준, 1 ms 버그 고치기 전 코드, p≈0.13 | spline_gt_result.html:17-18 |
| 앱 변경: 컴파일됨, 온도·OIS 기록 없음 | 기록 없음 맞음 (HsRecorder.java:252-265 배터리 온도는 과열 중지 판단에만, :196 OIS OFF 요청만·결과 미기록). "컴파일됨" 은 확인 안 함 (가정) | — |

## 문서 정정 필요
1. today_20261020_easy.html:98 "망가뜨렸나? — 아니다" → "크게 해친다는 증거 없음, 이득·손해 판정 불가 (둘 다 성공한 5 짝 중 4 짝은 보정 쪽이 0.1 cm 나쁨)".
2. today_20261020_easy.html:95 스플라인 "4개 모두 같은 방향" 에 "p≈0.13, 145816 은 난수 폭 이하" 단서 (spline_gt_result 와 맞춤).
3. gyro_temp_table.html ④·spline_gt_result.html "칩(CP)" → "CP 온도 센서" (새 발견 2), x61/y112/z70 은 "맞춘 범위(27.8~37 °C) 밖 외삽, 실측 +69/+97/+70".
4. c5_test_plan.md 변경 기록 시각 15:30 → 15:21 (커밋 시각), 이유 "Allan 에 가속 없음" 삭제 (실제로 있음).
5. C5 결과 문서: (a) 가 초기화 못 하면 "C5 철회" 가 아니라 "C5 판정 불가 (주 설정 시험 안 됨)".

## 새로 발견
1. **CP 기울기는 거의 식는 구간에서만 맞춘 값**: temp_correlation.CP 범위 27.8~37.0 °C (카메라 켬 구간과 그 뒤 30 s 제외). heating n=14 블록은 정지 중 자연 흔들림, cooling n=31. 녹화 중 CP 는 44.5 °C 까지 감 → 녹화 안 바이어스 변화 61/112/70 은 7.5 °C 외삽, y 는 실측 97 보다 15 % 큼. CP 는 카메라 켠 30 s 만에 28.8→36.7 °C 로 뛰어 자이로 칩 온도 대리값으로는 지연·편향이 클 수 있음. 히스테리시스 "0~1.7" 도 실제 가열 없이 낸 값 (이미 '미결', 이유 보강).
2. 가정 (기억 기반): Samsung thermal HAL 의 "CP" 는 통신 프로세서(모뎀) 온도로 알려짐 — 자이로 칩(LSM6DSV) 온도가 아님. "칩(CP)" 표기는 오해 소지. dumpsys thermalservice 로 센서 이름 확인 필요.
3. 레버암 z 가 녹화 시각에 따라 두 무리: 114000·115620·125714 ≈ −0.5 cm, 131326 이후 −1.4~−2.5 cm (imu_params.json r). '±0.67' 은 무작위가 아니라 세션 차이 — 온도·가속 바이어스를 레버암이 떠안는 것으로 추정 (가정). 레버암을 고정값으로 쓰기 전 확인.
4. C5 (a) strict 는 손에 든 녹화에서 초기화 0 회 → 실제 앱 기본 (--init static, make_ov_config.py:141) 도 사용자가 1.5 s 가만히 들지 않으면 시작 못 함 (제품 위험; 앱이 같은 값을 쓴다는 것은 가정).
5. 정지 초기화 붕괴 11/15 vs 동적 6/15 → C5 의 핵심(느슨한 15 px 정지 판정의 오판)은 (d) strict-dyn 이 가장 직접 시험함.

## (d) 과정

| 무엇 | 근거 | 위험도 | 할 일 |
|---|---|---|---|
| runs/ 전체 추적 안 됨 (결과·요약 JSON) | git status | 낮음 (의도) | C5 summary 는 docs 에 요약해 커밋 |
| tools/ov_bridge_diag/build/ 추적 안 됨, 해시 미기록 | git status --ignored, c5_run.py:68 은 경로만 | 중간 | 결과에 sha256(ov_bridge) 기록 (지금은 binary 13:04:21 > main.cpp 13:03:40 이라 최신) |
| config/{1440x1080_15, …} 추적 안 됨 | git status | 낮음 | C5 는 안 씀 (config/mono, config/lens 만, 커밋됨) |
| 옛 결과 섞임 | runs/c5 는 재시작 뒤 파일만 (mtime ≥ 15:26:46), 첫 실행(bench 3a0b9dd)은 JSON 저장 전 죽음 | 낮음 | 분석 때 bench_blob 단일 확인 |
| "이전 붕괴 7 개" 는 옛 벤치·옛 브리지 결과 | c5_test_plan.md 2절 | 낮음 | (c) loose 재실행이 재현 확인 역할 — 설계 맞음 |
