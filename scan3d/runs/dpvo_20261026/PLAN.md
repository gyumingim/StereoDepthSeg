# dpvo 2026-10-26 — DPVO / DPV-SLAM 도입·평가 계획 (사용자 결정 2026-10-26)

## 목표
DPVO (Deep Patch Visual Odometry, princeton-vl) 와 DPV-SLAM (loop closure) 을 우리 데이터·같은 자로 OpenVINS VIO 와 비교.
결론: 정확도 표 (같은 칸), 속도 (GPU ms/frame, 메모리), VI 융합 길, 폰 가능성, 추천.

## 규칙 (COMMON_BRIEF / OVERNIGHT 따름)
- 폰 안 씀 (이번 일은 PC 오프라인만). 다른 역할 worktree 안 건드림. 무거운 PC 작업 ≤ 6 병렬 (DPVO 는 GPU 1개 → GPU 작업 1~2개씩).
- 외부 코드: `~/scanner/dpvo_ext/DPVO` (저장소 밖, 커밋 안 함). 우리 쪽 코드·결과만 이 폴더에 커밋.
- 환경: WSL micromamba env `~/scanner/dpvo_ext/mamba/envs/dpvo` (python 3.10, cuda-nvcc 12.8, torch 2.7.1+cu128 = RTX 5080 sm_120 지원). sudo 없음.

## 단계
1. 코드·가중치: GitHub princeton-vl/DPVO (커밋 해시·MIT 라이선스 기록), models.zip (dpvo.pth). 확장 (cuda_corr, cuda_ba, lietorch_backends) 빌드.
   Blackwell (sm_120) 에서 실패하면 원인 기록 → torch/nvcc 버전 바꾸기 또는 가까운 fork. 스모크: 공식 데모 영상 1개.
2. 입력: `vio_bench_hs.py` 의 같은 함수 (undistort_maps, fed_selection, FrameCache) 로 **OpenVINS 와 바이트 같은 960x540 회색 프레임**
   (같은 렌즈 파일, 같은 시간 격자 30/60 fps, 같은 시작 오프셋 0/1/2 s) → 회색 3채널 복제 → DPVO (intrinsics = Knew).
   시각 = 카메라 시각을 IMU 시계로 (tc − dt), OpenVINS 포즈와 같은 t_ns.
3. 평가 (같은 자): HS 42칸 (GT imu_gt_spl_cal1022, lens_switch_gtscore.cam_metrics → zupt_ob_score.run_metrics),
   std60 165506 GT (imu_gt_spl, 3 오프셋), 손 루프 142148/142610/151646 (시작-끝 오차).
   DPVO 는 단안 = 축척 없음 → (a) Sim3 ATE (bench evaluate 와 같은 식, OpenVINS sim3 와 바로 비교),
   (b) 전체 Sim3 축척으로 맞춘 뒤 ATE_w·발산 판정 (같은 run_metrics), (c) 축척 흐름 = 앞 절반/뒤 절반 Sim3 축척 비,
   (d) 처음 몇 초로 정한 축척으로 나머지 ATE (현실적 축척), (e) 루프 오차 (Sim3 후, 경로 대비 %).
   궤적 두 종류: final (끝난 뒤 최적화된 keyframe + 보간, 논문 방식) 과 causal (각 프레임 처리 직후 그 프레임 포즈 = 실시간에 쓸 수 있는 값).
   OpenVINS 기준: integ2 GL30 (0.6.27 프로필 rec20261025 + gyro_klt=5, histogram NONE = 라이브), G30 (HISTOGRAM), ov60eval L60/P60,
   std60 GL60. 같은 칸·같은 창 (OpenVINS 추적 창으로 자른 DPVO 도 계산).
4. 속도: GPU ms/frame (cuda synchronize, 중앙/p95), 최대 메모리, 30 vs 60 fps. PC 다른 일 섞임 기록.
   설정: 기본 (96 patches, window 10) + 가벼운 설정 1개 참고.
5. DPV-SLAM: `LOOP_CLOSURE True` 로 루프 3개 + std60 GT 에 (가능하면).
6. VI 융합: 웹 조사 (DBA-Fusion 등) → 쓸 수 있으면 돌림, 아니면 느슨한 결합 설계 스케치 + 기대 이득 (추정).
7. 폰 가능성: 파라미터 수·연산·필요한 CUDA 연산 이식 목록, 모바일 GPU/NPU 지연 추정 (폰 작업 없음).
8. 독립 검증자 (읽기 전용) → 확인된 것만 고침 → README + 보고 (≤ 25줄). RESUME 노트는 progress 파일.

## 섞인 변수 / 주의 (미리)
- DPVO 는 컬러 TartanAir 로 학습 → 회색 3채널 입력은 분포 밖 (영향 작을 것으로 추정).
- HS 녹화 렌즈는 f2.22 (라이브 카메라 모드 아님). DPVO 는 비결정적일 수 있음 (CUDA, RANDOM 패치) → 시드 고정 + 일부 칸 반복.
- OpenVINS 는 IMU 로 축척을 앎 — Sim3 비교는 DPVO 에 유리한 자. 표에 명시.

## 변경 2026-10-26 17:25 (사용자 결정 via 코디네이터): 60 fps 고정 → 60 fps 부분집합만 (DPVO 결과 보기 전에 정함)
- 이유: DPVO 가 이 PC(WSL2, RTX 5080)에서 약 90 ms/프레임 → 60 fps HS 42칸 전체는 너무 김. 남은 30 fps 실행 취소 (끝난 30 fps 32칸은 참고로만).
- 비교 상대: OpenVINS GL60 (integ2 0.6.27 rec60 프로필, c4f60, gyro_klt=5, histogram NONE) 같은 칸.
- 60 fps 칸 (고정 규칙, DPVO 60 fps 결과 보기 전):
  - std60 165506 o0, o1, o2 (base + slam)
  - 손 루프 142148, 142610 (base + slam)
  - HS 12칸: 214550 o0/o1/o2, 131326 o0/o1/o2 (발산 잘 나는 녹화), 154320_o0, 222747_o0,
    보통 칸 4개 = 위 4녹화와 폰 낙하 213955 를 뺀 HS 녹화 이름 정렬 순서 앞 4개의 o0: 214138_o0, 221052_o0, 221925_o0, 223815_o0 (base)
- 순서: std base → 루프 base → std/루프 slam → HS 12칸. GPU 작업 2개 병렬.
