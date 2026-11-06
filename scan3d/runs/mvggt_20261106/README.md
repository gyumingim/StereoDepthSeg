# mvggt 2026-11-06 — VGGT · MASt3R(-SfM) 가 지금 오프라인 스캐너보다 나은가

브랜치 `mvggt-20261106` (scanA-20261105 cef1ea2 위), 워크트리 `~/scanner/StereoDepthSeg-mvggt`. PC 만 (폰 없음).
사용자 승인 (11-06 "재봐라 ㄱㄱ"): 공식 출처만 — github naver/mast3r (+dust3r, croco), facebookresearch/vggt, HF 공식 가중치.
1–4 절은 **측정 전에 커밋** (사전 등록). 바꾸면 아래에 날짜·이유를 덧붙인다.

## 0. 설치 (결과 아님)
- 코드: `~/scanner/mvggt_env/src/` — mast3r f5209afc, dust3r 3cc8c88c, croco d7de0705, vggt a288dd0f (Windows git clone → 복사).
- 가중치 `~/scanner/mvggt_env/weights/` (git 에 안 넣음), sha256 = HF lfs 값과 같음 확인:
  - `facebook/VGGT-1B` rev 860abec7 `model.safetensors` 5.03 GB, f164acf6…467e (비상업 체크포인트, 연구 비교용)
  - `naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric` rev 06e7259f `model.safetensors` 2.75 GB, 0a615eb0…6feb
  - MASt3R-SfM 검색 모델(retrieval)은 받지 않음 → 장면 그래프는 swin (아래).
- 환경: `~/scanner/mvggt_env/venv` (python 3.12.15, venv_ffs 의 torch 2.11 cu128 등을 `.pth` 로 읽기만), 추가 설치 roma 1.8.0 ·
  trimesh 5.2.0 (`--no-deps`). dust3r curope (CUDA RoPE) 는 빌드 안 함 → 느린 pytorch RoPE (결과 같음, 속도만).
- 안전: 무거운 작업 전부 `flock ~/scanner/heavy.lock`, 스레드 8, GPU 는 `gpu_guard` (frac 0.6, gpu.lock, OOM 이면 실패), TORCH_COMPILE_DISABLE=1.

## 1. 질문과 비교 대상
지금 파이프라인 (B0): OpenVINS VIO 재생 → `kf_ba.py` (pycolmap SIFT + 자체 BA, `--seed 0`, featloop 판) → `scan_offline.py`
(FFS 모션 스테레오 960×540 → TSDF 5 mm). 같은 키프레임·같은 채점에서 바꾸는 것:

| 변형 | 자세 | 깊이 | 분리하는 것 |
|---|---|---|---|
| B0 | 우리 BA | FFS 모션 스테레오 (지금) | 기준선 (내가 다시 돌림) |
| V1 | VGGT (조각 이어 붙임) → Sim3 로 BA 틀 | VGGT depth head | VGGT 통째 |
| V2 | 우리 BA | VGGT depth (조각마다 Sim3 축척) | 깊이만 |
| M1 | MASt3R-SfM (sparse GA) → Sim3 로 BA 틀 | MASt3R dense depth (`get_dense_pts3d`) | MASt3R 통째 |
| M2 | 우리 BA | MASt3R depth (전역 Sim3 축척) | 깊이만 |

## 2. 사전 등록 — 데이터·입력
- 녹화 (차례대로): 165506 (`hs_20261024_165506_std60`, GT `imu_gt_spl`, 0.5 dpt 렌즈 fx 1286.18) →
  134204 (`hs_20261022_134204`, GT `imu_gt_spl_cal1022`, 2.22 dpt fx 1303.9) → 222747 (`hs_20261020_222747`, 같은 GT 종류).
- BA 자세: featloop `rec/<id>/ba_S/poses_ba.jsonl.gz` (SIFT, 루프 없음, `--seed 0`) 를 **읽기만 해서 복사** (sha256 기록).
  222747 은 featloop ba_S 가 없으면 (11-06 14:50 현재 없음) featloop 판 kf_ba 를 같은 인자 (`--kf-from-vio --match-ms 1 --feat sift
  --loop none --seed 0`) 로 내 폴더에서 돌린다 (반복 수가 다르면 적음).
- 키프레임 = BA 키프레임 전부 (165506 139, 134204 356, 222747 ≈1185). 모델에 넣는 집합이 400 장을 넘으면 k 장마다 하나 (k = 최소 정수,
  ≤ 400 장) → 222747 은 3 장마다 (≈395). **모든 변형 (B0 포함) 은 이 같은 집합의 키프레임만 융합·채점**한다
  (`--kf-subset`; B0 의 FFS 짝 후보는 지금처럼 BA 키프레임 전부에서 찾음). 222747 은 참고로 B0 전체 키프레임 run 도 적음.
- 영상: `scan_offline` 과 같은 왜곡 펴기 (gt.npz 의 K·dist, 960×540, INTER_AREA 축소 → `initUndistortRectifyMap`, 새 K = fx 하나로 정사각 화소,
  cx cy 그대로) 를 PNG 로 저장해 두 모델에 같은 파일을 넣음.
  - VGGT 입력: 공식 `load_and_preprocess_images(mode="crop")` → 518×294 (가로 0.5396 · 세로 0.5444 비율로 늘림, 공식 방식 그대로).
  - MASt3R 입력: 공식 `load_images(size=512)` → 512×288 (0.5333 같은 비율, 잘림 없음 — 확인해 적음).
  - 모델 깊이는 960×540 격자로 되돌림 (`cv2.resize` 선형, 마스크는 최근접). 깊이 Z 는 영상 크기와 무관.
  - 내부값: TSDF·채점은 **우리 K** (모든 변형 같음). VGGT 는 K 를 입력받지 않음 (예측 초점 거리를 기록만). MASt3R 는 우리 K 로
    초기화 (`init[img]['intrinsics']`, `shared_intrinsics=True`, `opt_pp=False`); 공식 코드에 초점 고정 스위치가 없어 초점은 다듬어짐 → 최종 초점 기록.

## 3. 사전 등록 — 모델 실행
- VGGT-1B: bf16 autocast, camera head + depth head 만 (point/track head 끔), 연속 키프레임 조각 C 장, 겹침 8 장.
  C = {96, 64, 48, 32} 중 165506 첫 조각이 GPU 상한 (0.6 × 16 GB) 안에 들어가는 가장 큰 값 (메모리 시험 = 결과 아님, 적어 둠).
  - 이어 붙이기 (V1): 다음 조각을 앞 사슬에 Sim3 — 축척 = 겹친 프레임들의 depth 비 (사슬/새 조각, 두 conf 모두 상위 50 % 화소) 중앙값,
    회전 = 겹친 프레임 R_사슬 R_새^T 의 평균 (SVD 투영), 이동 = 카메라 중심 차 평균. 겹친 프레임은 앞 조각 값 사용.
    그다음 사슬 전체를 BA 키프레임 카메라 중심에 Sim3 (Umeyama) 한 번 → 자세·깊이에 같은 축척.
  - V2: 자세 = BA. 깊이 = 각 조각의 VGGT depth × (그 조각 VGGT 카메라 중심 → BA 중심 Sim3 축척). 겹친 프레임은 앞 조각 깊이.
  - 깊이 마스크: 공식 데모 기본값 = 프레임마다 depth_conf 하위 50 % 버림.
- MASt3R-SfM: `sparse_global_alignment` 공식 데모 기본값 (lr1 0.07 / niter1 300, lr2 0.01 / niter2 300, optim `refine+depth`,
  matching_conf_thr 0, subsample 8), 장면 그래프 `swin` 창 8 비순환 (대칭 짝), `shared_intrinsics=True` (폰 렌즈 하나).
  깊이 = `get_dense_pts3d(clean_depth=True)` 의 depthmap, 마스크 = conf > 1.5 (데모 기본 min_conf_thr).
  - M1: MASt3R 자세 → BA 중심에 Sim3 한 번 → 자세·깊이에 같은 축척. M2: 자세 = BA, 깊이 × 같은 전역 Sim3 축척.
  - MASt3R 는 metric 모델 → 그 자체 축척 (BA 로 맞추기 전) 도 GT 대비로 적음 (참고).
- TSDF 깊이 잡음 (외부 깊이용, FFS 는 지금 그대로): σ = 0.003 + 0.01·Z² m (0.5 m 에서 5.5 mm), 같은 TSDF 설정 (5 mm, sigma_max 0.08).
- 메모리가 모자라면 (OOM) 다시 시도하지 않고 C 나 창을 줄인 값과 이유를 적는다. 모델 출력 (자세·깊이 원본 npz) 과 BA 틀로 바꾼 것 둘 다 저장.

## 4. 사전 등록 — 채점과 판정
- 채점 코드 = `scan_offline.py` 그대로 (`scan3d/mvggt/mv_offline.py` = 같은 코드 + 외부 깊이 `--ext-depth` + `--kf-subset`, FFS 경로는
  한 줄도 안 바꿈 → B0 가 featloop mesh_S 와 같은 숫자인지 먼저 확인 = 재현 관문).
  지표: 보드 전체 화소 dZ 평균·|dZ| p90, 키프레임별 통계, 메시 보드 평면 z 중앙·RMS, 덮임 (1 cm 칸), 평가 키프레임 수.
- 축척 두 조건:
  - **원래 축척 (native)** = 우리 파이프라인의 metric 축척 (BA = VIO 축척). 모델 자세는 BA 카메라 중심에 Sim3 → 그 뒤는 B0 와 같게
    (GT 에 SE3 강체 맞춤만).
  - **오라클 축척** = `--free-scale` (자세 위치를 GT 에 Sim3 한 축척을 이동에 곱함; 외부 깊이에도 같은 값을 곱함, FFS 는 기선 따라 저절로).
- 자세 정확도 (키프레임, GT ok 만): Sim3 맞춘 뒤 ATE RMSE (cm), 회전 오차 중앙·p90 (°), 축척 k = 1/s − 1 (%).
  VIO kf · BA · VGGT 사슬 · MASt3R 각각. 165506 은 보드 모서리 자 (`corner_eval.py`) 도 (시간 되면).
- **판정 (변형마다, 녹화마다)**: 오라클 축척에서 전체 화소 |dZ| p90 **또는** 메시 RMS 가 B0 보다 20 % 이상 낮고, 덮임이 0.05 넘게 줄지 않으면
  "좋아짐". 녹화 3 개 중 165506 포함 2 개 이상 "좋아짐" 이면 "추천 후보" (그래도 녹화 3 개·세션 독립 아님 = 추정).
  원래 축척 결과도 같이 적되 판정에는 안 씀 (축척 문제는 boardscale 에이전트 몫).
- 반복: 변형마다 1 회 (VGGT·MASt3R 추론은 결정적에 가깝다고 가정, 확인 안 함). 섞인 변수는 결과 절에 적는다.
- 한계 (미리): 녹화 3 개, 보드 1 장 장면, 모델 입력 해상도가 우리 깊이 (960) 의 약 0.54 배, VGGT 조각 이어 붙이기는 우리 방식 (공식 아님).
