# scanA 2026-11-05 — 첫 오프라인 폰 3D 스캐너 (A안)

브랜치 `scanA-20261105` (scanplan-20261105 f6b368e 위), 워크트리 `~/scanner/StereoDepthSeg-scanA`. PC 만 (폰 없음).
흐름: hs 녹화 (렌즈 5, 1920x1080@60, 0.5 dpt) → PC 에서 **라이브 VIO 설정 c4f60 재생** (자세) → 키프레임 BA (자세 다듬기)
→ 모션 스테레오 깊이 (FFS) → TSDF 메시.
빌드·설정은 `~/scanner/build_livepose/` (tmp_* 아님, 커밋 안 함, 아래 sha 로 다시 만들었는지 확인).
이 1–3 절은 **측정 전에 커밋** (사전 등록). 바꾸면 아래에 날짜·이유를 덧붙인다.

## 1. 사전 등록 — 단계 1 (라이브 설정 재생 복구)
- 브리지: `scan3d/ov_bridge` (이 브랜치 소스, 24a58c2 이후 변경 없음 = appfix-20261029 과 같은 소스) 를 `-DOV_FULL_SOURCE=ON`
  Release 로 빌드 → `~/scanner/build_livepose/bridge/ov_bridge`. OpenVINS = `~/scanner/src/open_vins` 6948812.
  vocab = `scan3d/ov_bridge/loop/voc_k10L4.voc`. 원래 bridge sha 기록은 지워져 **비트 동일 확인 불가** → 결과 기준으로만 판정.
- 설정: `make_phone_ov_main.py OUT --lens-file config/lens/std_1920x1080_f0.50_calcap20261026.json` → `OUT/c4f60/`.
  확인: `kalibr_imucam_chain.yaml` sha256 앞 16 = 58b9a1af46461eb3, `remap_1920x1080.bin` = 483168c9723c0ba1 (lens05 README 3절).
- 재생: `runs/livepose_20261030/lp_replay.py` 그대로 (인자 `--bridge`, `--vocab` 만 새 경로). 기준 B = 기본 인자, 시작 0/1/2 s.
- 통과 기준: 165506 B VIO ATE (lp_score, imu_gt_spl) 가 1.70 / 1.57 / 1.52 cm 와 각각 |Δ| ≤ 0.02 cm.
  스모크 (옛 하네스 조건 `--imu cal --time oracle`) 1.63 cm 도 같은 기준. 안 맞으면 숫자를 맞추려 바꾸지 않고 차이를 적는다.

## 2. 사전 등록 — 단계 2 (키프레임 BA)
- 입력: 단계 1 의 B o0 포즈 (c4f60 라이브 설정, `tracking` 포즈). 키프레임 = 10 프레임마다 (e2e_mono_ms 와 같음), 해상도 960x540
  (GT 렌즈 K·dist 로 왜곡 펴기 — e2e 와 같은 영상 격자).
- 방법: pycolmap — 키프레임 SIFT, 짝 = 시간 이웃 + VIO 자세로 시야 겹치는 짝, 기하 검증 → VIO 자세 고정 삼각측량 →
  번들 조정 (내부값 고정, 자세·점 풀림) → VIO 키프레임 중심에 Sim(3) 정렬 (축척·중력은 VIO 몫, 실제 상황과 같음).
  키프레임 사이 프레임은 쓰지 않음 (깊이 짝도 키프레임끼리).
- 1차 지표 = **짝 상대 자세 오차** vs GT (`imu_gt_spl`): 키프레임 짝 (i, j) 중 GT 기선 4–30 cm 이고 두 프레임 모두 GT ok 인 것 전부
  (짝 집합은 GT 로 한 번 정하고 모든 자세 출처에 같게). 이동 오차 |t_ij − t_ij_gt| (mm, 카메라 i 좌표), 회전 오차 (°).
  평균 · 중앙값 · p90 (짝 전체). 참고: 절대 ATE (SE3, cm).
- 자 자체의 흔들림 참고: 같은 짝에서 `imu_gt_spl` vs `imu_gt` (스플라인 없는 보드 자세) 차이 = GT 잡음 바닥 (참고).
- 판정: BA 뒤 이동 오차 중앙값이 20 % 이상 줄고 p90 이 늘지 않으면 "좋아짐 (녹화 1개, 추정)". 그 밖은 "잡음 수준/나빠짐".
- 순환 주의: BA 는 같은 영상 (보드 무늬 포함) 을 쓰고 GT 도 같은 영상의 보드 모서리에서 나옴 → 완전 독립 아님. 보드 영역 특징을 빼는
  시험 (`--mask-board`) 도 같이 돌려 차이를 적는다.

## 3. 사전 등록 — 단계 3 (깊이 + 융합, 보드 장면)
- 같은 녹화·같은 깊이 단계 (`e2e_mono_ms` 의 짝 고르기·FFS·combine·TSDF), 자세만 바꿈: GT (상한, **순환**), VIO c4f60 원래, BA 다듬은 것.
  (참고로 scanplan 의 C4/C1 결과도 표에 남김 — 다른 VIO 설정.)
- 키프레임 깊이 오차 dZ = Z − Z_plane(GT) (보드 안, 가장자리 1 cm 제외): 키프레임별 평균·중앙값·p10·p90 **와** 모든 키프레임 화소를 모은
  전체 평균·중앙값·p10·p90 (부호 있는 값과 |dZ|). 짝 못 찾은 키프레임 수도 적음.
- 메시 평면 오차: 복셀 5 mm (+ 꼭짓점이 sub-voxel 보간인지 확인) → 보드 위 꼭짓점 z 중앙값·RMS·p5/p95. 덮임 = 보드 사각형 1 cm 칸 중
  평면 3 cm 안 꼭짓점이 있는 칸 비율 (칸 수 반올림 버그 고침: floor 칸 번호, 분모 = 실제 칸 수).
- 모든 비교는 같은 녹화·같은 스크립트·같은 채점.
