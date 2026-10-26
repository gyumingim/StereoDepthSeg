# dpvofast 2026-10-26 — DPVO 속도 측정 (부분 결과, 18:53 사용자 결정으로 중단: DPVO 보류)

**상태: PARTIAL.** 계획(PLAN.md) 17개 설정 중 base 3시드 + p64 1칸만 끝남. 독립 검증 에이전트 안 돌림 (중단 지시) → 아래 수치는 미검증.

## 측정된 것 (PC 단독: 다른 무거운 작업 없음, load1 0.6–1.6, 가끔 4 = 이 작업 자신)
- 환경 점검 (코디네이터 요청): 확장 3개 (cuda_corr, cuda_ba, lietorch_backends) 모두 sm_120 SASS 로 빌드됨 (JIT 아님, strings 확인),
  viz=False, 설정 = config/default.yaml (96 패치, window 10, MIXED_PRECISION True = fp16 autocast 이미 켜짐). 데이터 로딩 = memmap uint8 + H2D (프레임 시간에 포함, 작음).
- **전체 실행 base 960x528 60 fps: 프레임당 중앙 45.4–46.9 ms (5칸 × 3시드 = 15 실행, p95 48–69)**. 이전 dpvo 보고의 46–81 ms 중 큰 값은 두 작업 동시 실행 탓.
  p64 (1칸만): 39.7 ms.
- 시간 분해 (split/*.json, 초기화 후 100–400 프레임, 동기 타이머; std60 165506_o0 / HS 221052_o0):
  960x528 벽시계 40–44 ms, 640x352 35–36 ms (해상도 영향 작음: 비용은 그래프 edge 수 ~36k 에 비례).
  구역 (960, ms): update 신경망 11–12, keyframe(+motionmag, 파이썬 루프·텐서 이동) 11–12, corr(cuda_corr) 8.5–9.5, patchify CNN 6, BA 1.8–1.9, reproject 0.9, 나머지 2.6.
  torch.profiler: 프레임당 커널 launch ~1170, cudaStreamSynchronize ~52, .item() 12, unique 4.
  corr_forward_kernel 4.8 ms × 2회/프레임 = 가장 큰 단일 커널. nvidia-smi: base 실행 중 GPU 55–60 %, 165 W, SM 2947 MHz (logs/gpu_util_sample_base_job.csv)
  → GPU 와 파이썬(CPU 1코어 100 %) 이 반반 병목. 이전 보고의 "GPU 12–16 %" 는 틀림 (동시 실행 중 표본).
  주의: split JSON 의 gpu_kernel_ms_per_frame / gpu_ms_by_family 는 op 행 + 커널 행 중복 합산이라 **과대** (코드는 고쳤으나 다시 안 돌림).
- 논문 ~16 ms (RTX 3090) 와 차이: 논문 EuRoC 입력은 0.5 배 축소 (stream.py), 그리고 WSL2 의 launch/sync 지연이 큰 것으로 추정 (미측정).

## 정확도 (같은 자, score.json / table.txt)
- OpenVINS GL60 을 **같은 GT Sim3 축척**으로 다시 채점: 1.49 / 1.54 / 1.68 cm (자기 축척 1.50/1.55/1.70) → 축척 차이 영향 거의 없음.
- DPVO base std ate_w final (o0/o1/o2): 시드0 0.78/0.78/**2.76**, 시드1 **1.19**/0.78/0.79, 시드2 **2.17**/0.79/0.79 cm.
  → 9 실행 중 2개는 OpenVINS(1.5–1.7) 보다 나쁨. **DPVO 는 시드(무작위 패치)에 민감** — 이전 보고의 0.80/0.80/0.82 는 운 좋은 1회.
- 손 루프 final 142148/142610: 0.90/0.67, 0.53/0.14, 0.56/0.25 % (OV 3.27/5.20 %), causal 시드0 142148 2.64 %.

## 안 한 것
나머지 knob (p48/p32, w7, fast, ba1, 해상도, stride 30/20/10 Hz, nosync, fp32, compile), Pareto 표, FLOP 측정 (dpvofast_flops.py 준비만),
모바일 이식 추정 (아래는 코드 읽기만 한 추정), 독립 검증.
- 모바일 (추정): 맞춤 연산 = cuda_corr (corr + patchify gather), cuda_ba (Schur + Cholesky), lietorch (SE3), torch_scatter (scatter_softmax/sum/max), unique,
  동적 크기 그래프 (프레임마다 edge 수 변함) → NPU(ENN) 에는 CNN/MLP 만, 나머지는 Vulkan/OpenCL 직접 작성 필요. update MLP ≈ edge 당 수 MFLOP × 36k edge → 프레임당 수십~수백 GFLOP (추정, 미측정).

## 파일
dpvofast.py (job/split), dpvofast_score.py, dpvofast_flops.py (미실행), knobs.txt, PLAN.md, split/*.json, score.json, table.txt, logs/.
원자료 ~/scanner/tmp_dpvofast/out (5.8 MB, 남김; cells → ~/scanner/tmp_dpvo/out/cells 심볼릭 링크).

## 다시 시작하려면 (RESUME)
`bash dpvofast_sched.sh (이 폴더) nosplit` — 끝난 실행은 건너뜀 (p64 o1 부터). 그 뒤 `venv_ffs/bin/python dpvofast_score.py`.
split 은 고친 profiler 로 다시 돌려야 GPU 커널 합이 맞음. 시드 민감도 때문에 knob 마다 최소 3시드 필요 (계획 수정).
