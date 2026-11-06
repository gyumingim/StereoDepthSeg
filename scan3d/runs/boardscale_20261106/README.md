# boardscale 2026-11-06 — 크기 아는 보드로 BA 축척 자동 맞춤

브랜치 `boardscale-20261106` (scanA-20261105 cef1ea2 위), 워크트리 `~/scanner/StereoDepthSeg-boardscale`. PC 만 (폰 없음).
질문: scanA 의 BA 자세는 VIO 축척을 그대로 받아 약 +2 % 크다 (165506 Sim3 0.981, 보드 깊이 치우침 +1.35 cm).
장면에 보이는 **크기 아는 ChArUco 보드** 만으로 (GT 자세 안 씀) 축척 하나를 추정해 자세(·메시)를 줄이면 이 치우침이 없어지나.
이 1–3 절은 **측정 전에 커밋** (사전 등록). 바꾸면 아래에 날짜·이유를 덧붙인다.

## 1. 방법 (`scan3d/scanA/board_scale.py`)
입력 = BA 자세 (`poses_ba.jsonl.gz`, 키프레임만, 각 줄 `frame` = 영상 프레임 번호), 원본 영상 `cam5.mp4`, 보드 정의 json
(`scan3d/config/charuco/board_*.json` — 칸 크기·칸 수·사전·ids_start), 렌즈 K·dist (BA 가 쓴 것과 같은 값: 그 녹화 GT 폴더 `gt.npz` 의
`K`, `dist` 두 배열만 읽음 = 렌즈 보정값. GT 자세 `R_bc`·`p_bc`·`ok`·`t_ns`·`corners` 는 **읽지 않음**).
1. 검출: BA 키프레임마다 원본 1920×1080 회색 영상에서 ChArUco 모서리 검출 (`aruco_scale.Board.detect`, GT 만들 때와 같은 검출기).
   모서리 6 개 이상인 키프레임만.
2. 삼각측량: 모서리 id 마다, 그 모서리가 보인 모든 키프레임의 BA 자세로 DLT (왜곡 편 정규 좌표) → 재투영 오차 > max(3 px, 중앙값 + 4·MAD)
   관측을 빼고 다시 (최대 3 번). 보인 키프레임 3 개 미만 또는 남은 시선 사이 최대 각 < 2° 인 모서리는 버림.
3. 축척: 보드 모델 (`getChessboardCorners`, m) → 삼각측량 점에 닮음 맞춤 (Umeyama) X ≈ s·R·M + t.
   닮음 잔차 > max(2 mm·s, 3·중앙값) 인 모서리는 빼고 다시 맞춤 (1 번). 보정 축척 **c = 1/s** (자세 이동에 곱함).
4. 불확실성: 키프레임 **블록 부트스트랩** (연속 10 키프레임 묶음, 복원 추출, 200 번, 시드 0) → 각 표본마다 2–3 단계 전부 다시 →
   c 의 표준편차·2.5/97.5 %.
5. 출력: `poses_board.jsonl.gz` = BA 자세의 이동 × c (회전 그대로, 원점 기준 닮음 — scan_offline 은 GT 에 강체 정렬만 하므로 원점 무관),
   `board_scale.json` (c, 부트스트랩, 쓴 키프레임·모서리 수, 재투영 오차, 입력·만든 시각).
6. (보조 추정, 판정에 안 씀) 2시점 축척: BA 기선 4–30 cm, 공통 모서리 ≥ 6 인 키프레임 짝마다 삼각측량 → 닮음 축척 → 중앙값.
7. (검사, 추정에 안 씀) 내 검출 vs GT 폴더에 저장된 검출 (`imu_gt_rs*/gt.npz corners`) 의 같은 프레임·같은 id 화소 차 중앙값.

## 2. 평가 (`scan3d/runs/boardscale_20261106/bs_eval.sh`, `bs_table.py`)
- 녹화: 165506 (60 fps, 보드 302×181.2 mm = 5×3 칸 60.4 mm, `board_laptop_5x3.json`) + H 4 개 134204 (모니터 9×5 칸 63.294 mm),
  213341 (노트북 9×5 칸 32.03 mm), 222747 · 214138 (노트북 9×5 칸 42.86 mm, `square_estimated: true` = 자로 7 칸 30 cm 잰 값).
  BA = scanA 의 기존 결과 그대로 (165506 `step2/ba_o0`, H `more/ba/<rec>`; 다시 돌리지 않음).
  추가로 축척 표만 (깊이 run 없음): featloop 의 시드 0 SIFT BA (`featloop_20261105/rec/<rec>/ba_S`) 가 끝난 녹화 전부.
- "GT 축척" = BA 키프레임 중심을 GT 중심에 Sim(3) 맞춘 축척 (`scan_offline --vio` 가 적는 `sim3_scale_would_be`, 오라클 run 이 곱하는 값).
- 깊이: 같은 `scan_offline.py` (960×540, FFS 반복 8, TSDF 5 mm, 165506 kf-step 10 / H `--compress --kf-step 1`), 자세만 바꿈:
  (a) BA 원래 축척 = 기존 run (165506 `step3/ba`, H `more/mesh/<rec>_ba`), 165506 은 결정성 확인으로 한 번 다시 돌림;
  (b) **보드 축척** `--vio poses_board`; (c) 오라클 `--vio poses_ba --free-scale` (165506 은 기존 `step3/ba_fs`, H 는 새로 돌림).
- 지표: 축척 c vs GT 축척 (차 %, 부트스트랩 σ 대비), 전체 화소 보드 dZ 평균·중앙, |dZ| p90, 키프레임 치우침 중앙, 메시 보드 높이 중앙 — (a)(b)(c) 나란히.

## 3. 판정 규칙 (측정 전)
- 녹화마다 **축척 일치**: |c / c_GT − 1| ≤ 0.5 %. **깊이 일치**: 보드 축척 run 의 전체 dZ 평균이 오라클 run 과 0.3 cm 안,
  그리고 |dZ| p90 이 오라클의 1.1 배 + 0.1 cm 이하.
- 165506 + H 4 개 모두 두 조건 통과 → "보드 축척 = 오라클 수준 (녹화 5 개, 추정)". 하나라도 못 하면 그 녹화와 이유를 적고 "부분".
- 참고 (판정 아님): 부트스트랩 95 % 구간이 c_GT 를 덮는 비율 (불확실성이 정직한지), 원래 축척 대비 dZ 평균이 0 에 가까워졌는지.
- **순환 주의**: GT 자세도 같은 보드 모서리·같은 칸 크기로 만든 것 → "보드 축척 = GT 축척" 은 추정기가 맞게 동작한다는 뜻이지
  보드 크기 자체가 맞다는 뜻이 아님. 칸 크기가 1 % 틀리면 둘 다 1 % 틀림 (특히 42.86 mm 는 자로 잰 추정값). 깊이 dZ 의 GT 평면도 같은 칸 크기.
- 한계: 보드 1 장이 화면 가운데 근처, 165506 은 안쪽 모서리 8 개뿐. 녹화 세션 3 개 (서로 독립 아님). BA 반복 수·키프레임 간격이 녹화마다 다름 (scanA 11 절).

## 4. 실행 규칙 (PC 3 번 멈춤 뒤)
무거운 작업 (검출·scan_offline) 은 모두 `flock ~/scanner/heavy.lock` 안에서 하나씩, `OMP/MKL/OPENBLAS_NUM_THREADS=8`,
`SCAN3D_GPU_FRAC=0.6`, `TORCH_COMPILE_DISABLE=1`, gpu.lock (gpu_guard), 작업 전 RAM 여유 ≥ 8 GB.
