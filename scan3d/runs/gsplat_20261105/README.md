# gsplat 2026-11-05 — 가우시안 스플래팅 (2DGS) 메시 vs 지금 TSDF 메시

브랜치 `gsplat-20261105` (scanA-20261105 cce74a3 위), 워크트리 `~/scanner/StereoDepthSeg-gsplat`. PC 만 (폰 없음).
질문: 오프라인 스캐너에서 FFS 모션 스테레오 깊이 → TSDF 대신 (또는 그 위에) **2D 가우시안 스플래팅 (2DGS)** 으로 장면을 맞추고
메시를 뽑으면 보드 평면이 더 정확해지나 / 덮임이 늘어나나. 녹화 165506 (hs std60) 하나.
이 1–4 절은 **측정 전에 커밋** (사전 등록). 바꾸면 아래에 날짜·이유를 덧붙인다.

## 1. 방법 (측정 전)
- 구현: **gsplat** (nerfstudio, 오픈소스, `rasterization_2dgs` = 2DGS 래스터라이저) 위에 짧은 자체 학습 루프
  (`scan3d/gsplat/gs_train.py`). 손실 = gsplat 2DGS 예제 기본값: L1 0.8 + D-SSIM 0.2, 법선 일관성 0.05 (7000 걸음부터),
  깊이 왜곡 0.01 (3000 걸음부터), 기본 밀도 조절 (분할·복제·가지치기, 15000 걸음 중 500–7500). 15000 걸음.
  환경 = 자체 venv `~/scanner/gsplat_env/venv` (torch 2.11 cu128) + 자체 CUDA 12.8 툴킷 (`~/scanner/gsplat_env/cuda128`, gsplat 컴파일용).
  공유 venv (`venv_ffs` 등) 는 건드리지 않음.
- 입력 = scanA 와 **같은 것**: 녹화 `scanA/scan3d/recordings/hs_20261024_165506_std60` (읽기만), 자세 = scanA 단계 2 BA
  (`step2/ba_o0/poses_ba.jsonl.gz`), GT 월드 (보드 좌표) 정렬 = scan_offline.py 와 같은 코드 (VIO 중심 → GT SE(3); 오라클 축척 run 은
  Sim(3) 축척을 이동에 곱한 뒤 SE(3)). BA 재실행 없음.
- 영상 = 키프레임 (scan_offline 과 같은 고르기: 10 프레임마다, 자세 있는 것) 960×540, GT 렌즈 K·dist 로 왜곡 편 핀홀 격자 (scan_offline 과 같은 Kr).
- 변형 (둘 다 같은 자세·영상):
  - **GS-A** 순수 2DGS: 사진 손실만, 초기 점 = 카메라 시야 안 무작위 (깊이 0.3–1.5 m 균일, 10 만 개). 깊이 정보 안 씀.
  - **GS-B** 2DGS + FFS 깊이: 초기 점 = FFS 키프레임 깊이 (scan_offline 과 같은 짝·FFS·combine 결과를 저장해 씀) 에서 뽑은 점,
    손실에 FFS 깊이 L1 (유효 화소만, 가중 0.1, 렌더 깊이 vs FFS Z) 추가. 실제로 쓰일 법한 조합.
- 메시 뽑기 = 학습 뒤 각 키프레임에서 렌더 깊이 (2DGS median depth, 누적 알파 > 0.5 화소만) → **scanA 와 같은 tsdf_map.TSDFMap**
  (복셀 5 mm, z_max 2.0, sigma_max 0.08, sigma_ok 0.02, min_obs 2), 화소 σ = 0.01 m 고정 (GS 깊이에는 시차 σ 식이 없음).
  즉 바뀌는 것 = 깊이 출처 (FFS → GS 렌더) 하나. 2DGS 원 논문의 open3d TSDF 대신 이것을 써서 융합 코드를 같게 둠.

## 2. 지표 (scanA 6·9 절과 같은 채점 코드를 옮겨 씀 — `scan3d/gsplat/gs_score.py`)
- (a) **깊이 dZ** = Z − Z_plane(GT), 보드 안 (가장자리 1 cm 제외), 키프레임별 + 전체 화소 평균/중앙/p10/p90, |dZ| 평균/p90.
  TSDF 쪽 = FFS 키프레임 깊이 (scanA summary 그대로), GS 쪽 = GS 렌더 깊이 (알파 > 0.5). 두 깊이는 성격이 달라 (GS 는 학습 시점 렌더) 참고 지표.
- (b) **메시 깊이 dZ** (주 지표, 같은 자로 두 메시를 잼): 각 메시를 같은 키프레임 집합 (scanA TSDF run 의 보드 평가 키프레임) 에서
  광선 투사 (open3d RaycastingScene) → 위 (a) 와 같은 보드 안 dZ 통계. TSDF 메시와 GS 메시에 같은 스크립트.
- (c) **메시 평면**: 보드 위 꼭짓점 z 중앙·RMS·p5/p95 (scanA 와 같은 식), **덮임** = 보드 1 cm 칸 중 평면 3 cm 안 꼭짓점 있는 비율.
  자 확인: scanA ba / ba_fs 메시를 이 스크립트로 다시 재서 scanA summary 숫자 (z 1.36/0.09, 0.00/0.28, 덮임 0.90/0.92) 와 같아야 함.
- (d) 실행 시간 (학습 + 메시), GPU 최대 메모리 (torch.cuda.max_memory_allocated + nvidia-smi 2 s 표본 최대).
- 조건 2 개: 원래 축척 (BA, VIO 축척) / 오라클 전역 축척 (`_fs`, GT 사용 — 실제 상황 아님).
- 보기용: 메시 PLY (줄인 것 < 30 MB) → `C:\Users\a3162\scan3d_handoff\changes_0620\gsplat\`.

## 3. 판정 (측정 전)
- 오라클 축척 조건에서 GS 메시가 TSDF 메시보다 (b) 전체 |dZ| p90 **또는** (c) 메시 RMS 가 20 % 이상 줄고,
  다른 하나가 20 % 넘게 나빠지지 않고, 덮임이 0.05 넘게 줄지 않으면 → "좋아짐 (녹화 1개, 추정)".
  반대로 20 % 넘게 나빠지면 "나빠짐", 그 밖은 "잡음 수준". 원래 축척 조건도 같은 규칙으로 함께 적음.
- 실행 시간이 TSDF (약 100–140 s) 의 10 배를 넘으면 그 사실을 판정 옆에 적음 (오프라인이라 실격은 아님).

## 4. 한계 (측정 전)
- 녹화 1개, 보드 1장 (무늬 많은 쉬운 면 — GS 에 유리할 수 있음). 학습 시점 = 평가 시점 (따로 남긴 시점 없음; TSDF 도 같음).
- GS 는 무작위성 (밀도 조절) 이 있음 → 씨앗 고정 (0), 시간 되면 씨앗 1 로 한 번 더 (반복 횟수 적음).
- GPU 규칙: `~/scanner/gpu.lock` 을 잡고 (scanA gpu_guard 그대로), torch 메모리 몫 0.75, OOM 이면 실패로 끝냄 (재시도 없음), 960×540.
