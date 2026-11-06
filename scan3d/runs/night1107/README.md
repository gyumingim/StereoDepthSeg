# night1107 2026-11-07 — 지금 스캐너 전체를 두 번째 std60 녹화 173432 에 돌리기

브랜치 `night-20261107` (boardscale-20261106 3bc14e4 위), 워크트리 `~/scanner/StereoDepthSeg-night1107`. PC 만 (폰 없음, 카메라 안 켬).
질문: 165506 에서 본 "VIO (60 fps, 0.5 dpt, c4f60) 축척 약 +2 %" 가 같은 날 같은 방식으로 찍은 **hs_20261024_173432_std60** 에서도 다시 나오나.
그리고 지금 스캐너 (VIO → 시드 0 SIFT BA → 보드 축척 → FFS + TSDF) 가 이 녹화에서도 165506 과 같은 수준인가.
이 1–3 절은 **측정 전에 커밋** (사전 등록). 바꾸면 아래에 날짜·이유를 덧붙인다.

## 1. 방법 (165506 과 같은 코드·같은 인자, 녹화만 바꿈)
- 코드: boardscale-20261106 (`board_scale.py`, `scan_offline.py`) + featloop-20261105 의 `scan3d/scanA/kf_ba.py`, `scan3d/featloop/feats.py`,
  `scan3d/scanA/gpu_guard.py` (`git checkout featloop-20261105 -- ...`, `--seed` 가 있는 판).
- VIO: `runs/livepose_20261030/lp_replay.py REC ~/scanner/build_livepose/cfg_gen/c4f60 OUT --bridge ~/scanner/build_livepose/bridge/ov_bridge
  --vocab scan3d/ov_bridge/loop/voc_k10L4.voc` (기본 인자 = 165506 의 B o0). sha: bridge 55e5d35b8ece3e29, imucam 58b9a1af46461eb3,
  remap 483168c9723c0ba1 (step1/build_shas.txt 와 같음, 11-06 22:43 확인). 렌즈 fx 1286.18 (GT `imu_gt_spl` K 와 같음).
- BA: `kf_ba.py REC --vio VIO --out ba_S --gt imu_gt_spl --feat sift --loop none --seed 0 --keep-db --cache-root ~/scanner/night1107_ext/cache`
  (165506 의 featloop `ba_S` 와 같음: 키프레임 10 프레임마다).
- 보드 축척: `board_scale.py REC --poses ba_S/poses_ba.jsonl.gz --board config/charuco/board_laptop_5x3.json --lens REC/imu_gt_spl/gt.npz
  --check-gt-corners REC/imu_gt_rs/gt.npz` (기본 문턱값).
- 깊이: `scan_offline.py REC --gt imu_gt_spl` (960×540, FFS 8 반복, TSDF 5 mm, kf-step 10) 세 가지 — (a) BA 원래 축척 `--vio ba_S/poses_ba`,
  (b) 보드 축척 `--vio poses_board`, (c) 오라클 `--vio ba_S/poses_ba --free-scale`.
- 165506 비교 열: featloop `rec/165506/ba_S` (시드 0) + `mesh_S` (a), boardscale `scale/165506_S` 의 c. 같은 자로 보려고 165506 ba_S 의
  (b) 보드 축척·(c) 오라클 깊이도 새로 돌린다 (지금까지 165506 깊이 보드/오라클은 scanA ba_o0 BA 로만 있었음).

## 2. 지표 (165506 과 같은 식)
- 축척 k (%, + = 너무 김), VIO 키프레임 (`poses_vio_kf`) 과 BA 각각: k_fwd = 1/s − 1 (자세 → GT 카메라 중심 Umeyama, viba_score 식),
  k_pair = 1 s 떨어진 키프레임 짝 (GT 이동 > 10 cm) 의 |dP|/|dGT| 중앙 − 1. GT = `imu_gt_spl` (ok, |p| < 1.2 m), 시각 차 < 2 ms.
- 보드 축척 c vs GT 축척 (bs_table `gt_scale`, 차 %), 부트스트랩 σ·95 %.
- 깊이 (scan_offline summary): 전체 화소 dZ 평균, |dZ| p90, kf 치우침 중앙; 메시 보드 높이 중앙·RMS, 덮임 비율. (a)(b)(c) 나란히.
- 참고: VIO ATE (lp_replay json), BA 재투영, 보드 키프레임 수, 내 검출 vs GT 검출 화소 차.

## 3. 판정 (측정 전)
- **"+2 % 다시 나옴"**: 173432 BA k_fwd 가 +1.0 ~ +3.0 % 그리고 k_pair 도 같은 부호 ≥ +0.5 %. **"안 나옴"**: |BA k_fwd| < 1.0 %.
  그 밖 (> +3 %, < −1 %) = "다른 크기" 로 숫자만 적는다. VIO 키프레임 k 도 같이 적는다 (165506: VIO +3.07, BA +2.06 %).
- 보드 축척: |c / c_GT − 1| ≤ 0.5 % 이면 boardscale 규칙 통과. 깊이: 보드 run dZ 평균이 오라클과 0.3 cm 안 그리고 |dZ| p90 ≤ 오라클 × 1.1 + 0.1 cm.
- 녹화 1 개 (165506 과 같은 날·같은 보드·같은 사람) — 두 녹화는 독립 표본이 아님. 결론은 "녹화 2 개, 추정".
- **순환 주의** (boardscale 과 같음): GT 자세도 같은 보드 모서리·같은 칸 크기 (60.4 mm, EDID) 로 만든 것. 보드 축척 = GT 축척 은 추정기
  동작 확인이지 보드 크기 확인이 아님.
- 조사 메모 (std60_survey): 173432 는 IMU 흔들림이 약하고 IMU 축척 점검 0.929 (−7 %) — VIO 축척이 165506 과 다를 수 있는 이유 후보 (미확인).

## 4. 실행 규칙
CPU 무거운 단계 (VIO 재생, kf_ba, board_scale) = `flock ~/scanner/heavy.lock`, GPU 단계 (scan_offline) = gpu.lock 만 (gpu_guard),
`SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 OMP/MKL/OPENBLAS_NUM_THREADS=8`, 시작 전 여유 RAM ≥ 12 GB, 자기 작업만 죽이는 감시 (< 4 GB).
featloop·viba 줄은 건드리지 않음.

## 5. 덧붙임 (11-07 00:20, 결과 일부를 본 뒤 — 사후 변경)
- **사전 등록 방법 그대로는 실패**: c4f60 VIO 재생 (o0) 이 173432 에서 **초기화를 한 번도 못 함** (1700 프레임 모두 `init`, bridge.err
  `[init]: not enough feats to compute disp`). 그래서 kf_ba 가 자세 0 개로 실패 (IndexError), 뒤 단계 없음. = **지금 스캐너는 이 녹화에서 자세를 못 만든다.**
  같은 현상이 10-24 문서 (`docs/std60_board_check_20261024.md`) 에 이미 있음: 기본 초기화 (창 1.0 s) 는 173432 o0/o1/o2 모두 초기화 못 함,
  감사 초기화 (창 1.5 s / init_max_disparity 2.30) 는 6/6 초기화.
- 추가 (사후, 판정 규칙은 3절 그대로 적용하되 "사전 등록 밖 변형" 으로 표시):
  (1) 기본 c4f60 o1·o2 (시작 1·2 s) 재생 — 초기화 실패가 시작 시점 탓인지 확인.
  (2) **c4f60 + 감사 초기화** `--yaml init_window_time=1.5,init_max_disparity=2.30` o0·o1·o2 재생 (나머지 설정 같음) → o0 로 BA (시드 0) → 보드 축척 → 깊이 3 개.
  섞인 변수: 초기화 설정 2 개가 165506 과 다름 (165506 은 기본 초기화로 됨). 165506 에도 같은 감사 초기화 VIO o0 를 돌려 k 를 같이 적는다 (축척 비교를 같은 설정으로).
- 실행: heavy.lock 을 CPU 단계 묶음 (재생 6 개 + BA + 보드 축척, 약 10 분) 동안 한 번만 잡음 (그 사이 다른 무거운 작업 없음 = 규칙 같음) — 단계마다 놓으면 다른 줄에 밀려 1 시간 넘게 기다림 (00:10 실제).

### 5.1 덧붙임 (11-07 00:58, 5절 결과를 본 뒤 — 탐색)
5절 결과 (미검증): 감사 초기화 VIO 의 축척이 시작 시점에 따라 크게 흔들림 (173432 o0/o1/o2 k_fwd +1.78 / −4.44 / −2.44 %,
165506 A o0 −1.56 % vs 기본 B o0 +3.11 %). 173432 A o0 BA 에서 보드 축척 c 가 GT 축척과 −2.41 % 다름 (boardscale 녹화 12 개 최대 0.57 %).
추가: 감사 초기화 VIO 173432 o1·o2, 165506 o0 에도 같은 BA (시드 0) → 보드 축척 → 깊이 (BA·보드) — "보드 축척이 VIO 축척 흔들림을 지우나" 와
"c vs GT 차가 o0 만의 일인가" 를 봄. 같은 nb_run 방식 (heavy.lock 한 번, 깊이는 gpu.lock).

## 6. 결과 (11-07 01:05; 숫자는 독립 검사 `verify_night1107.md` 에서 다시 계산해 같음, 해석은 그 지적 반영)
표 전체: `table.md` / `table.json` (`nb_score.py`). 165506 B_o1/B_o2 의 "tracking 0" 은 자세 파일이 없어서 (scanA 는 json 만 남김) 생긴 표시.

| run | VIO ATE cm | VIO k_fwd % | BA k_fwd / k_pair % | 보드 c vs GT 축척 | kf 치우침 BA → 보드 (오라클) cm |
|---|---|---|---|---|---|
| 165506 기본 c4f60 o0 (지금 스캐너) | 1.70 | +3.11 | +2.06 / +2.84 | −0.06 % | +1.55 → −0.04 (+0.01) |
| 173432 기본 c4f60 o0/o1/o2 | 초기화 못 함 | — | — | — | — |
| 173432 감사 초기화 o0 | 1.34 | +1.78 | +1.23 / +3.35 | **−2.41 %** | +2.82 → +0.07 (+1.84) |
| 173432 감사 초기화 o1 | 1.43 | −4.44 | −5.04 / −2.88 | −2.48 % | −2.08 → +0.01 |
| 173432 감사 초기화 o2 | 1.17 | −2.44 | −2.98 / −0.89 | −2.36 % | −0.42 → +0.06 |
| 165506 감사 초기화 o0 | 1.26 | −1.56 | −2.01 / −1.40 | −0.42 % | −1.23 → −0.08 |

- **+2 % 질문**: 3절 규칙으로는 173432 감사 o0 이 "다시 나옴" (+1.23 / +3.35) 이지만, 같은 녹화 o1·o2 는 −5.0 / −3.0 %, 165506 도 감사 초기화로는 −2.0 %.
  → **"+2 %" 는 60 fps / 0.5 dpt VIO 의 고정 치우침이 아니라 초기화에 따라 ±3–5 % 흔들리는 값** (BA 축척은 VIO 를 따라감, 녹화 2 개, 추정).
- **지금 스캐너 (기본 c4f60) 는 173432 에서 자세를 못 만듦** (초기화 실패, 10-24 문서와 같음). 감사 초기화는 두 녹화 모두 ATE 가 더 낮음 (1.26–1.43 vs 1.70 cm).
- **보드 축척**: 다섯 run 모두 kf 치우침을 ±0.08 cm 안으로 (BA 원래 −2.1 ~ +2.8 cm). 그러나 173432 는 c 가 GT Sim3 축척과 일관되게 −2.4 % 다름 → boardscale 축척 규칙 (≤0.5 %) 밖,
  사전 등록 깊이 규칙도 "못 함" (보드 dZ 평균 −0.36 vs 오라클 +1.34).
  독립 검사: 채점 버그 아님 (GT 폴더·프레임 시각 맞음, imu_gt_rs −2.6 %, imu_gt −3.1 %). 보드 c 는 1 s 짝 비율 (k_pair) 과 맞고 전역 축척과만 다름
  (173432 o0 짝 간격 0.5 s +3.1 % → 4 s +1.5 % → 전역 +1.2 %) → 궤적이 GT 와 순수 닮은꼴이 아님. BA 쪽 변형인지 GT 쪽 변형인지는 못 가림.
  **순환**: 보드 축척 깊이가 GT 평면에 앉는 것은 같은 60.4 mm 모서리·같은 렌즈에서 나오므로 거의 구조상 당연 — "보드 축척이 맞다" 의 증거가 아님.
  말할 수 있는 것: "전역 GT 축척 = 맞는 깊이 축척" (오라클 가정) 이 173432 에서 깨짐.
- 메시 RMS 가 보드 축척에서 커짐 (감사 run 0.20–0.30 vs 0.11–0.14 cm; 165506 기본은 반대 0.14 vs 0.18) — 원인 모름 (고른 2.5 % 축척으로 설명 안 됨).
- 위생: 코드·인자·렌즈·GT 폴더 165506 과 같음. 165506 BA 는 `--tracks` 인자 생기기 전 실행 (기록 None) — 지금 기본값 drop 이 옛 동작 (featloop README 6.2) 이라 같다고 봄 (가정).
  섞인 변수: 감사 run 은 초기화 설정 2 개가 다름. 반복 = 시작 시점 3 개 (173432) / 1 개 (165506). 녹화 2 개는 같은 날·같은 보드.
- 메시: `depth/173432A_board/mesh.ply` → `scan3d_handoff/changes_0620/overnight/night1107_173432_board.ply`, meshes.json 백업 `meshes.json.bak_night1107_20261107_0053`.
