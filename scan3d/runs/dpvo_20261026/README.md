# dpvo 2026-10-26 — DPVO / DPV-SLAM vs OpenVINS (같은 데이터·같은 자)

## 결론 (60 fps 고정, 사용자 결정)
- **std60 165506 (보드, GT) 오프셋 3개 (녹화 1개, 각 1회)**: 세 오프셋 모두 DPVO 가 더 정확 (독립 장면은 1개). window ATE (DPVO final) 0.80/0.80/0.82 cm vs OpenVINS GL60 1.50/1.55/1.70 cm.
  처음 5 초만 GT 로 맞춘 축척 (s5, 이것도 GT 사용) 0.83/0.80/0.85 cm. DPVO 자체 축척은 GT 대비 1.10/0.84/1.16 (단안이라 10–16 % 틀림). 실시간 값 (causal) 0.82/0.81/1.12 cm.
  **단, DPVO 는 단안이라 축척을 GT 로 맞춰 줌** (OpenVINS 는 IMU 로 스스로 축척, 축척 오차 1–2 % 포함) → DPVO 에 유리한 자.
- **손 루프 (GT 없음, 시작-끝 오차 / 경로, 축척 무관)** 60 fps: 142148 DPVO final 0.83 % (실시간 causal 1.27 %) vs OV 3.27 %, 142610 0.67 % (causal 0.65 %) vs 5.20 %.
  경로/범위 비 (흔들림 부풀림 점검) DPVO 2.16/2.33 = OV 2.09/2.26 → 비교 공정.
- **DPV-SLAM (proximity loop closure)**: 루프 보정 1–3 번 일어남. final 은 std 0.76/0.78/0.92, 루프 0.74 %/1.25 % — base 보다 확실히 좋지 않음.
  causal (실시간 값) 은 보정 때 튀어서 std 2.7/1.5/3.1 cm 로 **나빠짐**.
- **속도**: 이 PC (WSL2, RTX 5080, torch 2.7.1+cu128) 에서 프레임당 중앙 46–81 ms (p95 53–120), 60 fps 예산 16.7 ms 의 3–5 배.
  두 작업 동시 실행 (par 2) 값이라 단독 속도 아님. GPU 사용률은 작업 중 nvidia-smi 표본 2번 12 %, 16 % (원자료 저장 안 함) → 계산이 아니라 커널 실행·동기화·Python 지연이 병목 (추정).
  GPU 메모리 할당 최대 0.82 GB (base) / 1.2 GB (slam), 예약 2.2–2.9 GB. 전용 깨끗한 타이밍 측정은 사용자 중지로 못 함 (부하 기록: load1 0.8–2.3).
- **추천**: 오프라인 기준 (참고용) 으로만 둔다. 폰 실시간 이식은 하지 않는다 (공개 모바일 포팅 없음, CUDA 연산 3개, PC 에서도 실시간 아님).
  OpenVINS 로 가져갈 아이디어 후보 (추정, 미검증): 학습 특징/대응 (패치 상관) 으로 손 루프 같은 저질감·빠른 회전 장면 추적 보강.
- VI 변형: 공식 DPVO 에 IMU 없음. 쓸 수 있는 것은 DBA-Fusion (DROID 기반, GPL-3, 무거움) 정도 → 돌리지 않음 (research_vi_mobile.md).

## 30 fps 곁가지 (취소 전 끝난 것만, 참고)
- HS 30 fps 20칸 (42 중): DPVO final 이 OV GL30 보다 짝 비교 중앙 +0.20 cm 나쁨, 19칸 중 4칸만 나음, 1칸 발산 (214138_o0).
- std 30 fps: 165506 o0/o2 DPVO 2.86/2.58 cm vs OV 1.78/1.36 (DPVO 나쁨), o1 0.79 vs 1.67. 173432 3칸은 OV 발산, DPVO 0.72–0.82 cm.
- 손 루프 30 fps: DPVO 0.40/0.44/0.95 % vs OV 1.52/2.63 %/초기화 실패 (151646).
- → DPVO 가 **어디서나** 낫지 않음. 일반 HS 칸은 OpenVINS 가 같거나 나음. DPVO 우세는 보드(std)·손 루프 같은 장면.

## 하지 않은 것 (사용자 중지)
- **HS 60 fps 12칸 부분집합 (PLAN.md 에 미리 정함) 은 실행 안 함** (사용자 중지, 17:47). 따라서 HS 장면의 60 fps 비교는 없음.
- 깨끗한 단독 GPU 타이밍·프로파일 (dpvo_prof.py 준비만 함), FLOP 수 (파라미터 3.53 M 은 dpvo_params.txt).

## 방법
- 입력: OpenVINS 벤치와 **바이트 같은** 960x540 회색 프레임 (FrameCache), 같은 시간 격자·시작 오프셋, t_ns = IMU 시계. DPVO 는 528 행으로 자름 (K 그대로).
- DPVO: princeton-vl/DPVO (dpvo_ext, MIT), dpvo.pth, 설정 base = default.yaml (96 패치, window 10), slam = + LOOP_CLOSURE True. 시드 0, 각 칸 1회.
  빌드 패치 dpvo_torch27.patch (torch 2.7 / sm_120).
- OpenVINS 기준: integ2 0.6.27 빌드 c640f534 (integ2 tip 0b469ac), rec20261025 (GL30) / rec60 (GL60) 프로필 + gyro_klt=5, histogram NONE, --drop-late 0.
  재실행이 integ2 발표값과 poses_hash 같음: HS GL30 45/45, std GL60 6/6 (ov_ref/ident_vs_integ2.txt).
- 점수 (dpvo_run.py score): sim3 = bench evaluate, ate_w/발산 = lens_switch_gtscore.cam_metrics (OpenVINS 와 같은 식) 을 전체 Sim3 축척으로 맞춘 DPVO 궤적에,
  ate_w_s5 = 처음 5 초 축척, ate_w_ow = OpenVINS 추적 창으로 자름, 루프 = fi_limits_run.loop_eval.
- 원자료: ~/scanner/tmp_dpvo (커밋 안 함). 결과: score.json, summary.txt/json, ov_ref/score_ov_ref.json, sched.log, loop_path_check.txt.

## 섞인 변수 / 주의
- 축척: DPVO = GT Sim3 축척 (s5 도 GT 사용), OpenVINS = 자기 축척 → DPVO 유리. std60 173432 는 OV 가 60 fps 에서 3칸 다 발산이라 60 fps 비교 없음 (DPVO 60 fps 안 돌림).
- DPVO 초기화 ~3–5 s vs OpenVINS ~4 s (std) / 9 s (HS) → 점수 창 다름 (ate_w_ow 로 일부 확인; std60 은 같음).
- 반복 1회 (오프셋 3개가 반복 역할), DPVO 는 CUDA 비결정 가능. 타이밍은 다른 역할(ovloop) 작업과 겹친 구간 있음 (load 기록).
- WSL 이 09:45, 17:34 두 번 멈춤 → 끊긴 작업은 다시 돌림 (끝난 결과만 사용).

## 독립 검증 (읽기 전용 에이전트, 2026-10-26)
수치 전부 원자료와 맞음. 과장 지적 4개 (3/3 칸 = 녹화 1개, s5 '현실적' 표현, 손 루프 causal 누락, GPU 사용률 원자료 없음) → 위에서 고침.
