# 입력값·주장 감사 2차 (메타인지 검사, 2026-10-20 오후)

- 작성: 독립 감사 (파이프라인·문서 작성자 아님). 코드·데이터는 읽기만 했다. 폰은 adb 연결이 안 돼(`device not found`) 조회하지 않았다.
- 기준: HEAD `ead75fb` (15:11). **감사 도중 다른 담당이 `vio_bench_hs.py`, `make_ov_config.py` 를 고치는 중** (커밋 안 됨, C5 시험 준비).
  아래 vio_bench 줄 번호는 HEAD 기준이고, 작업 중 변경은 항목 B9 에 따로 적었다.
- 1차 감사(`input_audit_report.md`)·검증(`input_audit_verify.md`)에서 이미 판정된 것은 다시 적지 않고, 오늘 새로 생긴 것과 아직 열린 것만 적었다.
- 제외 (다른 검증자 담당): `docs/vio_collapse_detect.html`, 가속도 행렬 A 방향.
- 재계산 스크립트 (저장소 밖): scratchpad `temps.py` (온도 상승), `h2t.py` (HTML 글자 추출). javac 시험 빌드 출력은 WSL `/tmp/audit2_javac`.
- 위험도 = 결과(정답·학습·VIO 점수·문서 결론)를 조용히 틀리게 할 가능성. "가정" = 이번에 확인 못 한 것.

---

## A. 정답(GT)·학습 단계

| # | 무엇 | 근거 (file:line / 명령) | 위험도 | 할 일 |
|---|---|---|---|---|
| A1 | **스플라인 매듭 0.0125 s 를 평가 녹화 + 자이로로 골랐다.** 고른 녹화 3 개(131326·145816·161035)가 VIO·보정 평가 녹화 4 개 중 3 개이고, 잣대 ② 가 Android 보정판 자이로와의 일치다. 그런데 결과 파일은 `imu_gt_spl` 을 "카메라만" (`gt_uses_imu=False`) 으로 표시 | `runs/spline_score_a.json`·`_b.json` 키 = 그 3 녹화; `spline_all.sh:3`; `spline_score.py:8`; `vio_bench_hs.py:289` | 중간 (영향 크기는 작을 것 — 0.0083~0.0125 사이 차이가 작음, 가정) | 매듭은 평가에 안 쓰는 녹화(114000·125714 등)로 다시 고르거나, 결과에 "매듭은 IMU 로 골랐음" 표시 |
| A2 | 카메라만 스플라인도 **초기값을 imu_gt_rs (자이로로 RS 편 정답) 에서** 가져온다. 최적화가 수렴하면 영향 없음 = 가정 | `spline_gt.py:175-183` (`load` 의 Rg·pg), `:383` 기본 `--gt-dir imu_gt_rs` | 낮음 | 한 녹화를 imu_gt 초기값으로 다시 맞춰 차이 확인 |
| A3 | spline_gt 보드 **여전히 `board_monitor_6x6.json` 고정** (1차 감사 지적, 안 고쳐짐). 지금 11 녹화는 모두 monitor 보드라 맞음 | `spline_gt.py:41`; lens json `"board"` | 중간 (다음 녹화에 A4 보드 쓰면 조용히 틀림) | gt.npz 에 보드 이름 저장 + spline_gt 가 읽기 |
| A4 | **공유 렌즈값이 평가 녹화 131326 하나로 맞춘 것**이고 9 개 녹화의 정답·VIO 가 모두 이 값 사용. `config/lens/` 는 git 에 없음 | `config/lens/hs_1920x1080_f2.22.json` `"recs": ["hs_20261019_131326"]`; `vio_bench_hs.py:50-54`; `git status` `?? config/lens/` | 중간 | 렌즈 파일 커밋, 결과에 렌즈 파일 해시 기록. 평가 녹화로 맞춘 값이라는 점을 문서에 표시 |
| A5 | **train_imu 의 중력 방향 맞춤이 평가(val) 녹화의 정답으로 이루어짐**, 그것도 보정 없는 IMU 로. 평가 녹화의 가속도 바이어스 일부를 중력 방향이 먼저 흡수 → "보정 없음" 쪽이 유리, 학습된 ba 의 이득이 작게 보일 수 있음 (가정). 난수(`randperm`) 라 seed 없으면 ±0.07 cm 흔들림 | `train_imu.py:377-396` `fit_gravity(tr + va)`; `cross_eval.sh` 주석 "±0.07 cm" | 중간 | 중력은 평가 녹화의 정지 구간 가속도만으로(정답 없이) 정하거나, 보정 적용 후 IMU 로 맞추기. 숫자 비교 때 이 점 명시 |
| A6 | `--calib-json` 평가 모드는 `--train` 에 평가 녹화 자신을 넣음 (`cross_eval.sh`). 학습은 안 하므로 누출은 아니지만 mu/sd·중력 모두 그 녹화에서 계산 | `cross_eval.sh` (`--train recordings/$v --val recordings/$v`), `train_imu.py:469-480` | 낮음 | — (주석으로 명시) |
| A7 | **seed 없는 학습 결과가 섞여 있음**: 2×2 표의 학습 난수 "seed 없음·1·2". GPU 결정성 설정(`torch.use_deterministic_algorithms`) 없음 → seed 를 줘도 GPU 에서는 비트 단위 재현 보장 없음 (가정) | `calib_folds.sh` (EXTRA 기본 빈칸), `train_imu.py:348-352`; spline_gt_result.html 1절 | 낮음 | 모든 학습에 seed 기록, 결과 json 에 seed·git 해시 저장 |
| A8 | **학습 결과 json 에 코드 버전이 없다** (git 해시·blob 없음). 1 ms 수정(11:21) 전후 결과가 같은 이름 규칙(`runs/on_*`)으로 섞여 있음. 구분은 이름(`on_mid_*` = 수정 후)뿐 | `train_imu.py:468-511` (res 에 code 키 없음); `ls runs/on_*` | 중간 | train_imu 결과에도 `code_version()` 추가 |
| A9 | 학습 캐시 키 = `CACHE_VERSION` + 입력 파일 mtime + 옵션. **코드 해시는 없음** → `_load_rec`·`imu_on_grid`·`read_imu` 를 고치고 버전 올리기를 잊으면 옛 캐시를 조용히 씀. `read_imu(raw=)` 선택도 키에 없음 (지금은 train 이 raw 를 안 쓰므로 괜찮음) | `train_imu.py:66,87-90`; `runs/train_cache` 48 파일 581 MB (v1·v2 섞임) | 중간 (다음 raw 학습 때 높음) | 키에 `_load_rec`/`imu_gt.py` 소스 해시와 raw 여부 추가 |
| A10 | 학습 IMU 격자: 빈틈 검사 없음, 498.9 Hz → 500 Hz 선형 보간 (1차 감사 지적, 그대로). 정답 거르기에 자이로 사용 (3° 넘게 다르면 버림) | `train_imu.py:122-123,140-149` | 낮음 | (그대로) |
| A11 | 스플라인 `--imu` 판(`imu_gt_spl_imu`)은 Android 보정판(1·4) IMU 로 맞춤. "평가 정답으로 안 쓴다" 는 지켜짐 (VIO 결과 45 개 모두 `imu_gt_spl`) | `spline_gt.py:292`; `runs/vio_calibcheck/*.json` `gt` 열 | 낮음 (확인됨) | — |

## B. VIO 벤치·지표

| # | 무엇 | 근거 | 위험도 | 할 일 |
|---|---|---|---|---|
| B1 | **IMU 잡음 ×10 을 평가 녹화에서 골랐다.** 131326·145816 두 녹화에서 ×1/×5/×10 을 비교해 "가장 좋았던 설정" 으로 고정, 그 뒤 같은 녹화로 보정 효과를 채점(calib_vio_check), C5 시험도 ×10. 고를 때는 **옛 벤치**(시간차 두 번, 번호 솎기, rs 정답)였고, 결과 json 에 `noise_scale: null` → 어떤 코드로 ×10 을 넣었는지 재현 불가 | `runs/vio/hs_*_noisemeas_x10.json` (`noise_scale` 없음, `gt: imu_gt_rs`); calib_vio_check.html 3절 "가장 좋았던 설정"; `docs/c5_test_plan.md` 3절 | **높음** (선택 누출 + 옛 코드) | 잡음 배율은 평가에 안 쓰는 녹화로 다시 고르거나 Allan(정지 시험, 원본 16) 값에서 정함. 지금 숫자는 "×10 은 평가 녹화에서 고른 값" 이라고 표시 |
| B2 | **OpenVINS 결과가 CPU 부하에 따라 달라질 수 있다.** 옛 벤치는 `init_dyn_use: true` 이고 동적 초기화 MLE 에 **벽시계 0.05 s 한도 + 4 스레드**. CPU 가 바쁘면 같은 입력도 반복 횟수가 줄어 다른 초기화가 나옴. calib 행렬은 최대 3 개 동시 + 같은 시각에 collapse 실행(13:27~14:00)도 돌았다. 래퍼 docstring 은 "같은 입력이면 결정적" 이라고 적음 | `config/mono/estimator_config.yaml:51-52,84`; OpenVINS `ov_init/src/dynamic/DynamicInitializer.cpp:629-631` (`max_solver_time_in_seconds = init_dyn_mle_max_time`); `vio_bench_hs.py:75`; `runs/vio_calibcheck/matrix5.sh` ("Max 3 at once"); `runs/collapse` 파일 시각; `calib_vio_ablation.py:9-10` | **높음** (초기화 성공 7/9 vs 5/9 같은 판정이 부하에 흔들림) | 동적 초기화를 쓰는 판은 한 번에 하나만, 또는 max_time 을 크게. 같은 입력 2 번 실행으로 결정성 직접 확인 (조용한 PC). C5 (c) loose 기준선도 같은 문제 |
| B3 | "같은 입력 두 번 → 3.6 cm vs 12.5 cm" 는 **두 판의 벤치 코드가 달랐다** (버린 판은 6253ee0 이전). 비결정성의 증거로 쓰기엔 약함 — B2 의 CPU 한도가 더 그럴듯한 설명 (가정) | `calib_vio_verify.md` 5절 "버린 판은 11:43, 벤치 6253ee0 이전" | 중간 | 같은 코드·같은 입력 2 회로 다시 확인 |
| B4 | **OpenVINS 온라인 시간차가 녹화별로 −0.4 ~ −0.57 ms 로 일정하게 남음** (145816·154320 전 판, 161035 는 ±2 ms 흔들림). 시간차를 한 번만 넣게 고친 뒤에도 align.json dt 가 ~0.5 ms 어긋났을 가능성 (가정; 131326 은 ~0). `calib_cam_timeoffset: true` 라 온라인 추정이 이를 가림 | `runs/vio_calibcheck/*.json` `dt_cam_imu_final_ms`; `config/mono/estimator_config.yaml:16` | 중간 | 녹화별 dt 를 batch_sync·스플라인(--imu) 두 방법으로 다시 대조, OpenVINS 온라인 시간 추정 끔/켬 비교 |
| B5 | VIO 의 R_ci·dt 는 **평가 녹화 자신의 자이로로 맞춘 align.json** (오라클 외부보정). 비교(보정 유무)는 공정하지만 절대 ATE 는 실제 비행보다 낙관적 | `vio_bench_hs.py:167-168,214` | 중간 (해석) | 문서에 "외부보정·시간차는 그 녹화로 맞춘 값" 명시. 공통 R_ci 로 한 번 돌려 차이 보기 |
| B6 | **지표 정의가 문서마다 다름**: 초기화 성공 = 첫 10 s 창 Sim3 축척 ∈ [0.5, 2] (summarize.py), 표의 빨간 칸 = 전체 축척 < 0.8 또는 > 1.25, C5 붕괴 = 축척 범위 + ATE > 10 cm. 성공 기준을 정한 `summarize.py` 는 **runs/ 에만 있고 git 에 없음** | `runs/vio_calibcheck/summarize.py:42`; calib_vio_check.html 3절; `c5_test_plan.md` 4절 | 중간 | 지표 코드를 imu_learn/ 로 옮겨 커밋, 문서마다 같은 정의 사용 |
| B7 | 전체 ATE·축척은 tracking 된 프레임만, 초기화 과도 포함으로 계산. ov_bridge 가 죽으면 그때까지만 채점 (`bridge_crash` 표시). summarize/make_html 은 `bridge_crash` 를 안 봄 → 일찍 죽은 판이 좋은 점수로 보일 수 있음. 이번 45 개에는 crash 없음 (확인) | `vio_bench_hs.py:263-271,297-319`; `grep bridge_crash` 0 건 | 낮음 (지금) | 요약에서 crash·짧은 클립 판을 따로 표시 |
| B8 | `--gt auto` 는 imu_gt_spl 이 없으면 **조용히 imu_gt 로** 대체 (stderr + 결과 `gt` 열에는 남음). summarize.py 는 **요약하는 시점의** auto 정답으로 다시 채점 → 실행 때와 요약 때 정답이 다를 수 있음 | `vio_bench_hs.py:144-154`; `summarize.py` `gt()` | 낮음 | 요약은 결과 json 의 `gt` 값을 그대로 쓰기 |
| B9 | **작업 중 (커밋 전) 벤치 기본값 변경**: `--init-cfg` 기본 = strict (정지 초기화, 동적 끔). 예전 결과(calib_vio_check 45 개, collapse)는 모두 loose. `calib_vio_ablation.py` 는 Namespace 에 init_cfg 를 안 넣어 **다시 돌리면 조용히 strict** 가 됨 → 옛 표와 섞일 위험 | `git diff scan3d/imu_learn/vio_bench_hs.py` (`getattr(a, "init_cfg", "strict")`); `calib_vio_ablation.py:65-66` | 중간 | 래퍼에 `--init-cfg` 추가, 결과에 `init_cfg` 기록(작업 중 diff 에 이미 있음). 옛 결과와 비교할 땐 loose 명시 |
| B10 | `--imu-raw` 와 `--imu-calib`(보정판 1·4 로 학습한 G·bg·A·ba) 를 같이 쓰면 막지 않음 → 원본 IMU 에 보정판용 바이어스를 빼는 실수 가능. 잡음 측정도 그때 IMU 기준 | `vio_bench_hs.py:179-193` | 중간 (원본 녹화가 생기면) | 보정 json 에 `imu_source` 기록, 다르면 거부 |
| B11 | 측정 잡음은 여전히 보정판 자이로(70 mdeg/s 톱니 포함) 의 조용한 10 % 에서. random walk = 밀도/10 근거 없음 (1차 지적 그대로) | `vio_bench_hs.py:57-68` | 낮음 | 정지 시험 Allan 값 |
| B12 | 임시 폴더 `runs/vio_XXXX` 가 지워지지 않고 236 개 쌓임 | `ls runs | grep vio_` | 낮음 | 실행 끝에 삭제 |
| B13 | PC 속도표(ms/프레임) 측정 때 다른 작업(학습·병렬 VIO)이 돌았는지 기록 없음 → 속도 숫자도 CPU 부하 가정 | report_20261020.html 속도표; `runs/vio/timing_*.json` | 낮음 | 측정 때 부하 기록 |

## C. 녹화·센서 (HsRecorder 변경, 2026-10-20)

| # | 무엇 | 근거 | 위험도 | 할 일 |
|---|---|---|---|---|
| C1 | **컴파일됨** (javac, android-35 + ARCore classes.jar, 오류 0, deprecation 경고만). `TYPE_GYROSCOPE_UNCALIBRATED` API 18, `TYPE_ACCELEROMETER_UNCALIBRATED` API 26, minSdk 28 → 문제 없음. `e.values[3..5]` 는 두 원본 센서 모두 6 개 값 | `javac ... -d /tmp/audit2_javac` rc 0; `AndroidManifest.xml` minSdk 28; `HsRecorder.java:123-126,280-286` | 낮음 | d8·설치는 아직 (APK 빌드 시각 10-19 12:08 < 커밋 10-20 10:54) |
| C2 | **아래쪽 읽는 코드는 안 깨짐**: `imu_gt.read_imu` (type 으로 고르고 `r[2:5]`), `recording.py:376-415` (모든 type 보관) → `phone_rx.ImuSync.add` 가 1·4 만 쓰고 `values[:3]`, `eval_motion_stereo`·`eval_aruco_scale` 는 type 4 만, `subsample_ana` 는 100 만, `analyze_static` 은 16·35 를 원래 다룸. 헤더는 예전부터 `type,t_ns,v0..v5` (8 칸) | 각 파일 줄 (본문 근거) | 낮음 (확인) | — |
| C3 | 센서 이벤트가 2 배 (1·4·16·35 각 ~500 Hz) 인데 카메라 결과·인코더와 **같은 `hs` 스레드** → 소켓 버퍼(984 이벤트) 넘침·표본 유실 가능성이 커짐. 16 을 같이 켜면 4 의 속도가 바뀌는지도 모름 (가정) | `HsRecorder.java:109,121-126`; 1차 감사 "Socket Buffer 984" | 중간 | 10 초 시험 녹화에서 type 별 개수·빈틈·속도 비교 (이미 todo 3절) |
| C4 | meta `imu_samples` 가 이제 네 종류 합계 (약 2 배). 읽는 코드는 없음 | `HsRecorder.java:287,313`; grep `imu_samples` 0 건 | 낮음 | meta 에 type 별 개수 |
| C5 | **온도 기록·CaptureResult(OIS·VSTAB·READOUT·물리 렌즈) 기록은 아직 코드에 없음**. 문서는 "다음 녹화부터 원본 + 온도" 라고 씀. 지금 앱을 깔면 온도 없이 녹화됨 | `HsRecorder.java` grep 결과 (온도·READOUT 없음, `:312` physical "5" 박힌 문자열 그대로); today_20261020_easy.html 한 줄 요약 | **높음** (내일 녹화가 계획과 다르게 나옴) | 설치 전에 온도(10 s)·결과 키 추가 또는 문서 정정 |
| C6 | 파일 머리 주석이 옛 형식 ("v0..v2 for 1 / 4") | `HsRecorder.java:45` | 낮음 | 주석 고침 |

## D. 저장소 상태 (재현성)

| # | 무엇 | 근거 | 위험도 | 할 일 |
|---|---|---|---|---|
| D1 | **모든 VIO 벤치가 읽는 OpenVINS 템플릿 `config/mono/` 가 git 에 없음** (estimator_config.yaml, kalibr_imu_chain.yaml). 잡음 기본값·초기화 값·max_time 이 여기 있음 | `vio_bench_hs.py:45,92`; `git status` `?? config/mono/` | **높음** | 커밋 (또는 make_ov_config 로 생성하고 결과에 해시) |
| D2 | `config/lens/` (공유 렌즈) 와 해상도별 `config/<WxH_fps>/` 도 git 에 없음 | `git status` | 중간 | 커밋 |
| D3 | **runs/ 전체 (2.1 GB) 가 추적 안 됨, 그 안에 결과를 만든 스크립트가 있음**: `vio_calibcheck/summarize.py`(성공 기준), `make_html.py`(calib_vio_check.html 생성기), `matrix*.sh`, `train_all.sh`, `table.py`, `fixtmp/*`, `verify_tmp/*`. 1차 검증 스크립트는 WSL `/tmp/v_*.py` (재부팅 때 사라짐) | `find runs -name '*.py' -o -name '*.sh'` | **높음** | 문서가 인용하는 스크립트는 imu_learn/ 또는 tools/ 로 옮겨 커밋 |
| D4 | 코드 버전이 남는 결과는 `calib_vio_ablation` 판(45 개)과 작업 중 벤치뿐. train_imu 결과, spline gt.npz, gyro_bias_stability.json, temp_history_bias.json, runs/vio/*.json 에는 코드 해시 없음 | `grep -l git imu_learn/*.py tools/imu_shell/*.py` | 중간 | 결과마다 `git_head`+blob 기록 (calib_vio_ablation.code_version 재사용) |
| D5 | 추적 안 되는 빌드물: `tools/imu_shell/cls/`, `tools/ov_bridge_diag/build/` | `git status` | 낮음 | .gitignore |
| D6 | 정답 폴더(imu_gt_spl 등)는 recordings/ (git 무시) 안에만 있고 만든 코드 버전 기록 없음 (knot 만 저장) | `spline_gt.py:368-376` | 중간 | gt.npz 에 코드 해시·인자 저장 |

---

## E. 오늘 문서 주장 대조

| 문서 | 주장 | 판정 | 근거 |
|---|---|---|---|
| today_20261020_easy | "손보는 양이 계단처럼 바뀐다 (**16분에** 1290번)" | **틀림** | 1290 번은 정지 시험 전체 43.6 분 (`runs/gyro_bias_stability.json` `android_cal.gyro.n_changes`, `duration_s` 2617) |
| today | "IMU 보정을 VIO 에 넣으니 녹화 하나에서 4.5배 나빠졌다 (2.6 → 11.8 cm)" (1절, '알아낸 것' 으로) | **낡음/과장** | 같은 문서 6절·calib_vio_check: 그 녹화는 보정 없이도 초기화 실패 3/3 → 보정 탓 아님. 1절 문장은 그대로 남아 서로 모순 |
| today | "VIO 시험 코드 버그 3개 … 다시 돌리는 중" / "학습 코드 1 ms … 아직 (지금 학습 끝나면)" | **낡음** | 1 ms 수정 659daff (11:21), 다시 돌리기 끝남 (14:48) |
| today | "OIS … 틀림. 꺼져 있었음 (데이터로 확인)" | **과장** | 검증은 131326 한 녹화에서 "OIS 흔적 없음" (회전비 0.986). 결과 키를 기록 안 하므로 다른 녹화는 미확인 |
| today | "칩이 약 15 °C 데워진다", "배터리 27 → 36 °C" | 맞음 | 첫 카메라 196 s: CP +16.1, PA +15.0, AP +21.7, 배터리 +7.2 °C; 배터리 최대 36.0, thermal 1 (`temps_all.csv`) |
| today | "레버암 … 녹화 11개에서 약 4.8 cm 로 일정" | **과장** | 4.8 은 x 성분만 (spline+IMU). 크기는 약 5.1 cm, z 는 −1.48 ± 0.67 로 흩어짐 큼. 물리 보정 값은 4.39 (크기 4.57). 두 방법이 0.4~0.7 cm 다름 |
| today | "검사의 지적 3개가 틀리거나 과장" | 대체로 맞음 | 검증표: A1 틀림, A8·A9 과장, A3·A5 일부 과장 (세는 방법에 따라 3~5) |
| report_20261020 정정 상자 | "학습 적분 1 ms 어긋남(**미수정**)" | **낡음** | 14:53 에 상자를 고치면서도 남음. 11:21 수정됨 |
| report 정정 상자 | "145816·161035 무너짐은 … 초기화 실패" | 부분 틀림 | 161035 = 초기화 뒤 속도 폭주 발산, 145816 보정 없음 +1 s 도 발산 (calib_vio_verify 6절) |
| report 본문 5절 | "OpenVINS 설정 확정: 잡음 ×10 … 시간차 −2 ms" | **낡음** (상자에서 철회했지만 본문 그대로) | ×10 은 평가 녹화 2개에서 고름 (B1). "시간차 −2 ms" 는 영상 시각을 IMU 시계로 옮긴 경우 0 이어야 함 (두 번 적용 버그) |
| report 3절 VIO 표 | 30/60 fps 등 숫자 | 낡음 (표시 필요) | 모두 옛 벤치 (시간차 두 번, 번호 솎기, `gt: imu_gt_rs`) |
| gyro_temp_table | "④ 240 fps 녹화 3분에 약 **8~10 °C** 오르므로 바이어스가 **40~70** mdeg/s 움직인다" | **틀림** | CP 는 3.3 분에 +16.1 °C → 표 자체 기울기로 x 62 · y 113 · z 71 mdeg/s. 검증(C3) 실측 +69/+97/+70. 같은 페이지 2절 표(15 °C → 58/106/66)와도 모순 |
| gyro_temp_table | "② 히스테리시스 CP·PA 기준 0~1.7 mdeg/s 로 거의 없다" | **문서끼리 모순** | temp_history_bias.html: CP/PA 현재 온도 냉각−가열 차 −4 ~ −11 mdeg/s (M1), 앞당김 넣어도 z PA −5.5 "0은 아님". 30 s 묶음·카메라 구간 제외 여부가 달라서로 보임 → 조건을 적어야 함 |
| gyro_temp_table | 기울기 x 3.87·y 7.04·z 4.43 (CP) | 문서끼리 다름 | temp_history_bias: CP 4.24 / 6.49 / 4.68. 자료 선택이 다름 (블록 길이·제외 구간) — 어느 것을 보정에 쓸지 미정 |
| gyro_temp_table | "가장 좋은 모델 2.81/3.13/2.55" | 과장 (낙관) | 검증: 시험 점수로 축마다 모델을 고른 값. 사전 고정 M1 은 3.41/3.63/6.06 |
| gyro_temp_table | 제목에 유닉스 시각 "1792457474.3579166 부터" | 표기 문제 | 2026-10-20 09:51 로 적기 |
| temp_history_bias | "온도 값보다 약 10 s 먼저 움직인다", 카메라 켬 효과 "x +5.8, z −5.2" | **검증 미반영** | 검증: 10 s 는 격자(10 s)·HAL 갱신 지연으로 설명, 0~8 s. 카메라 켬 효과는 온도원 따라 z 부호가 바뀜 (PA 기준 +12.6) = 불확실 |
| temp_history_bias | "앱이 **1 s마다** 온도 기록" 권장 | 문서끼리 다름 | todo_experiments: 10 s (사용자 결정, HAL 이 ~10 s 마다만 갱신) |
| spline_gt_result | "녹화 한 번에 폰이 약 **8~10 °C** 데워짐" | 틀림/모호 | 배터리 +7.2, 칩(CP) +16.1 °C. today 의 "15 °C" 와 모순 |
| spline_gt_result | "① train_imu 1 ms 버그 … 아직 안 고침" | 낡음 | 고쳐짐. 다만 이 문서 숫자는 **고치기 전** 코드 결과라는 표시는 필요 |
| spline_gt_result | "③ … 145816 에서 VIO 2.58 → 11.75 cm 로 나빠졌습니다" | 낡음 | 보정 탓 아님 (calib_vio_check) |
| spline_gt_result | "4개 모두 같은 방향이라 우연은 아닌 것 같지만" | 과장 | 4/4 부호 검정 p = 0.125 (양측). 145816 곡선 자 차이 0.003 cm < 학습 난수 폭 0.03 cm |
| spline_gt_result | 매듭 "남이 채점하는 두 잣대로 골랐다" | 누락 | 고른 녹화 3 개가 평가 녹화이고 잣대 하나가 자이로 (A1) |
| calib_vio_check | "물리 보정은 VIO 를 망가뜨리지 않는다" (결론 첫 줄), "자이로 행렬 방향이 맞았다 → 적용 버그 없음" | **검증 미반영** | calib_vio_verify: "해치지 않음" 과장 (짝 비교 4/5 에서 전부가 0.1 cm 나쁨), G 방향은 2/4 에서만 보임. 14:53 커밋은 report·today 만 고침 |
| calib_vio_check | "145816: 보정 없음 3 번 모두 초기화 실패 (과도)" | 부분 틀림 | +1 s 판은 과도가 아니라 속도 폭주 발산 (verify 6절) |
| calib_vio_check | "OpenVINS 는 같은 입력이면 대체로 같은 결과라 반복 대신 씀" ↔ 같은 문서 4절 "같은 입력 두 번 3.6 vs 12.5 cm" | **문서 안 모순** | 래퍼 docstring 도 "deterministic". 실제로는 동적 초기화 벽시계 한도로 CPU 부하 의존 가능 (B2), 두 판은 코드도 달랐음 (B3) |
| calib_vio_check | "IMU 잡음 = 녹화에서 잰 값 ×10 (가장 좋았던 설정)" | 누출 표시 없음 | 같은 평가 녹화에서 고른 값 (B1) |
| calib_vio_check | 빨간 칸 기준 0.8/1.25 vs 성공 기준 0.5/2 | 기준 둘 | B6 |
| debate_super_easy | "녹화 하나에서 크게 망가졌는데 이유를 아직 모름 … 보류" | 낡음 | 14:50 결론: 보정 무관 초기화 실패 |
| debate_super_easy | "배터리 8~10 °C, 칩 약 15 °C" | 거의 맞음 | 3 분 녹화 배터리 +7.2 (실험 전체 +9.1), CP +16 |
| todo_experiments | "카메라 켬 자체 효과(자이로 x +5.8, z −5.2 mdeg/s, 가속도 z +3 mg)" 를 사실처럼 | 과장 | 검증: 불확실 (온도원 따라 부호 바뀜) |
| todo_experiments | "3절: 물리 보정에 온도 항 넣어 **0.99 cm** 가 줄어드는지 비교" | 결정과 모순 | 토론 합의: 3 초 IMU 점수(0.99 cm) 는 기준에서 뺌 → VIO 로 채점해야 함 |
| todo_experiments | "앱: 16·35 원본 저장 (코드 수정 끝, 빌드·설치 남음)" + 온도·결과 키 "추가할 것" | 맞음 | today 문서만 "원본 + 온도를 기록하기로" 로 끝난 것처럼 읽힘 (C5) |

---

## 새로 발견 (위험 순)

1. **VIO 결과가 CPU 부하에 따라 바뀔 수 있음 (높음)** — 옛 벤치의 동적 초기화가 벽시계 0.05 s·4 스레드 한도 (`estimator_config.yaml:51-52`, OpenVINS `DynamicInitializer.cpp:630`). 보정 행렬은 3 개 동시 + collapse 실행과 겹쳐 돌았다. "초기화 성공 7/9 vs 5/9", "같은 입력 두 번 3.6 vs 12.5 cm" 해석이 이 영향을 받는다. 래퍼 docstring 의 "결정적" 은 확인되지 않은 말.
2. **잡음 ×10 을 평가 녹화(131326·145816)에서, 옛 벤치로 골랐고 재현 코드 기록도 없음 (높음)** — 이후 calib_vio_check, C5 가 모두 이 값을 고정 사용.
3. **결과를 만든 핵심 파일이 git 에 없음 (높음)** — `config/mono/` (OpenVINS 템플릿), `config/lens/`, `runs/vio_calibcheck/summarize.py`(성공 기준)·`make_html.py`(문서 생성기)·`matrix*.sh`.
4. **앱은 원본 IMU 만 추가, 온도·결과 키 기록은 아직 없음 (높음, 계획)** — 그대로 설치·녹화하면 "원본 + 온도" 계획과 다른 데이터가 나온다. 컴파일과 아래쪽 읽기 코드는 문제 없음 (확인).
5. **스플라인 매듭을 평가 녹화 3 개 + 자이로로 고름 (중간)** — "카메라만 정답" 표시와 다름. 공유 렌즈도 평가 녹화 131326 하나로 맞춤.
6. **OpenVINS 온라인 시간차가 녹화별 −0.4 ~ −0.57 ms 로 일정하게 남음 (중간, 가정)** — align.json dt 가 녹화마다 ~0.5 ms 틀렸을 가능성.
7. **train_imu 중력 맞춤이 평가 녹화 정답·보정 없는 IMU 로 이루어짐 (중간, 가정)** — 가속도 바이어스 보정의 이득이 작게 보일 수 있음.
8. **작업 중 벤치 기본값이 loose → strict 로 바뀜 (중간)** — 래퍼(calib_vio_ablation)는 init_cfg 를 안 넘겨 다시 돌리면 조용히 strict. 옛 결과와 섞이지 않게 표시 필요.
9. 학습 결과·정답·온도 분석 json 에 코드 해시 없음, 학습 캐시 키에 코드 해시 없음 (중간).
10. `--imu-raw` + 보정판용 `--imu-calib` 조합을 막지 않음 (중간, 원본 녹화가 생기면).
11. 1차 지적 중 안 고쳐진 것: spline_gt 보드 고정, 학습 격자 빈틈 검사, 측정 잡음이 보정판 기준, HsRecorder 2 ms 짝짓기 창·물리 렌즈 박힌 문자열 (낮음~중간).

## 문서 정정 필요

1. **today_20261020_easy.html** — "16분에 1290번" → "44분에 1290번". 1절 "VIO 에서 4.5배 나빠졌다" 를 6절 결론(보정 탓 아님)에 맞게 고치기. 2절 상태 "다시 돌리는 중"·"1 ms 아직" → 끝남. "OIS 꺼져 있었음 (확인)" → "131326 한 녹화에서 OIS 흔적 없음". 레버암 "4.8 cm" → x 성분임을 밝히고 물리 보정 4.4 cm 와 다름을 적기. "원본 + 온도 기록" → 온도는 아직 코드에 없음.
2. **report_20261020.html** — 정정 상자 "학습 적분 1 ms 어긋남(미수정)" → 수정됨 (11:21). "161035 = 초기화 실패" → 초기화 뒤 발산. 본문 5절 "설정 확정 (×10, 시간차 −2 ms)" 에 철회 표시, 3절 VIO 표가 옛 벤치 숫자임을 표시.
3. **gyro_temp_table.html** — ④ "8~10 °C, 40~70 mdeg/s" → "칩(CP) 약 16 °C(배터리 약 7 °C), 바이어스 약 60~110 mdeg/s (y 가 가장 큼)". ② 히스테리시스 "0~1.7" 은 30 s 묶음·카메라 구간 제외 조건임을 적고 temp_history 의 −4~−11 과 함께 표시. "가장 좋은 모델" 은 낙관적 값임을 표시. 제목의 유닉스 시각을 날짜로.
4. **temp_history_bias.html** — 검증 결과 반영: "10 s 앞섬" → "0~8 s, HAL 갱신 지연과 같은 크기, 물리적 앞섬 근거 없음". 카메라 켬 효과 → "불확실 (온도원 따라 부호 바뀜)". LOSO 최선값 → 낙관적, 사전 고정 M1 은 3.4/3.6/6.1. 온도 기록 간격 1 s → 10 s (사용자 결정).
5. **spline_gt_result.html** — "8~10 °C" → 배터리 7 / 칩 16 °C. "1 ms 버그 아직" → 고쳐졌으나 이 숫자는 고치기 전 코드. "145816 VIO 나빠짐" 문장 삭제/정정. "우연은 아닌 것 같다" → "4/4 같은 방향이지만 p≈0.13, 크기는 학습 난수 폭과 비슷". 매듭을 평가 녹화 3 개 + 자이로로 골랐다는 점 추가.
6. **calib_vio_check.html** — 검증(calib_vio_verify.md) 반영 상자 추가: "망가뜨리지 않는다" → "크게 해치는 증거 없음, 이득·손해 판정 불가". "자이로 행렬 방향 맞음" → 2/4 녹화에서만. 145816 +1 s 보정 없음 = 발산. "같은 입력이면 같은 결과" 삭제 (CPU 의존 가능). 잡음 ×10 이 같은 녹화에서 고른 값임을 표시. 성공 기준(0.5~2) 과 빨간 칸(0.8~1.25) 차이 설명.
7. **debate_super_easy.html** — "망가진 이유 아직 모름" → 14:50 결론 반영.
8. **todo_experiments.md** — 카메라 켬 효과 숫자에 "불확실" 표시. 3절 "0.99 cm 가 줄어드는지" → VIO 로 채점 (토론 합의).
9. **calib_vio_ablation.py docstring** (코드 주석) — "OpenVINS is deterministic for identical input" 는 확인 안 됨 → 주의 문구로.
