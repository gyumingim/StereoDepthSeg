# scanA — 오프라인 폰 3D 스캐너 (키프레임 BA → FFS 깊이 → TSDF 메시)

브랜치 `scanner-defaults-20261111` (featloop-20261105 7384db5 + gpu-ba-20261109 + kfonly-20261111 + laptop-worker-20261111 + ghost-20261111 의 `tsdf_map.py` 고침 + ffsviews (6 절) + kfsel-20261111 (7 절)).
실험 기록은 `scan3d/runs/<이름>/README.md` (scanA_20261105, featloop_20261105, gpuba_20261109, kfonly_20261111). 이 파일은 "지금 기본값과 돌리는 법" 만.

## 1. 기본값 (사용자 결정 2026-11-11)
| 항목 | 기본값 | 옛 동작 (플래그) |
|---|---|---|
| 입력 | **키프레임만** (`REC/frames_kf/NNNNN.jpg`; `kf_frames.py` 가 옆에 `cam5.mp4` 가 있거나 키프레임이 빠지면 멈춤) | `--full-video` (cam5.mp4 를 복호) |
| 키프레임 화질 | **1920×1080 JPEG q90** 로 받아서 PC 가 960×540 (INTER_AREA) 으로 줄임. kf_ba · FFS · TSDF 계산은 전부 960 | `--kf-res 960` (kfonly B), `--kf-quality Q` |
| BA | **GPU** (`kf_ba.py` 기본 `--ba-device cuda`, `--sift-device cpu`, `--fast-tracks`; CUDA 없으면 cpu 로 바꾸고 `[device]` 줄에 찍음) | `--ba-device cpu`, `--sift-device auto` |
| 키프레임 고르기 | **움직임 기준** (11-11 사용자 결정, 7 절): 마지막 키프레임에서 3 cm 이동 또는 4° 회전하면, 그 VIO 프레임 + 뒤 3 프레임 중 노출 시간 자이로 |ω| 가 가장 작은 것 (`--kf-select motion --kf-dist 3 --kf-rot 4 --kf-win 4 --kf-blur gyro`, `kf_select.py`; kf_ba 는 manifest 목록 `--kf-list`) | `--kf-select interval` (VIO tracking 10 프레임마다), `--kf-blur lap` |
| 자세 | 폰 실시간 VIO 자세. **지금은 대용**: PC 재생 `lp_replay.py` + 폰 설정 c4f60 (H = `cfg_gen_hs222/c4f60`, 그 밖 `cfg_gen/c4f60`) | `--vio FILE`, `--cfg DIR` |
| 깊이·메시 | `scan_offline.py` FFS 960×540 + TSDF 5 mm, `--compress --kf-step 1` | `--mesh-args` |
| FFS 짝 | 깊이 지도 1 장 = 기준 키프레임 + 짝 1 개 (**`--ffs-views 2`**), 짝은 모든 키프레임에서 (**`--ffs-partners all`**: 시간차 ≤ 30 s, FFS 전 dy 정류 검사) (11-11 사용자 결정, 6·7 절) | `--ffs-partners near --ffs-views 3` (옛: 시간 이웃 0.15–2.5 s, 짝 최대 2 개 평균), `--ffs-views 3..5` |
| TSDF 블록 할당 | **촘촘히** (`scan_offline.py --tsdf-alloc dense` = `TSDFMap(alloc_dense=True)`: [−mu, +mu] 띠를 반 블록마다 할당 → 보드 앞 유령 두 번째 층 없앰, 11-11) | `--tsdf-alloc sparse` (옛 3 점 할당) |
| 축척 | 그대로 (VIO 축척). `--board auto|FILE` 이면 보드 축척 (`board_scale.py`, 1920 키프레임에서 ChArUco 검출) + 그 자세로 메시 한 번 더 | |

## 2. 돌리는 법
```bash
# 노트북 작업 PC 에서 (계산은 노트북만; 결과는 runs/laptop/<RID>/scan/ 로 돌아옴)
cd ~/scanner/StereoDepthSeg-defaults/scan3d
tools/remote_laptop.sh job 133820 scan                         # 기본값
tools/remote_laptop.sh job 133820 scan --args '--board auto'    # + 보드 축척
tools/remote_laptop.sh job 133820 scan --args '--full-video --ba-device cpu --sift-device auto --tsdf-alloc sparse' --tag old   # 옛 경로
tools/remote_laptop.sh job 133820 scan --args '--tsdf-alloc sparse' --tag sparse   # 유령층 고침만 끄기
tools/remote_laptop.sh job 133820 scan --args '--kf-select interval --ffs-partners near --ffs-views 3' --tag oldkf   # 7 절 전 기본값 (되돌리기)
# 앞 실행의 vio·reduce·kfba 를 다시 쓰고 mesh 만 (같은 BA 자세): --reuse <앞 OUT> (노트북에 그 폴더가 남아 있을 때)
tools/remote_laptop.sh job 133820 scan --tag fv_x --args '--reuse /home/system/scanner/StereoDepthSeg-defaults/scan3d/runs/laptop/20261022_133820/fv_n3/scan --ffs-views 4'
# 직접 (그 기계에서): scan3d/scanA/run_scanner.sh REC OUT [옵션]   (옵션 전체는 스크립트 머리말)
```
REC 가 원래 녹화면: vio (재생) → reduce (`runs/kfonly_20261111/make_reduced.py --full-res`, 폰이 보낼 폴더를 흉내: 키프레임 JPEG +
imu.csv + frames_cam5.csv + meta.json + `phone_live_poses.jsonl.gz`, 영상 없음) → kfba → mesh [→ board → mesh_board].
REC 가 이미 키프레임 폴더 (`frames_kf/` + `phone_live_poses.jsonl.gz`) 면 vio·reduce 를 건너뜀 = 폰 앱이 보낸 것을 그대로 처리.
결과: `OUT/scan_summary.json` (만든 시각, 커밋, 입력, 단계별 초, |dZ| p90, 축척 k, 보낸 JPEG MB/분), 단계별 로그 `OUT/*.log`,
줄인 폴더 `OUT/red/` (처리 전 입력), `OUT/ba/`, `OUT/mesh/` (mesh.ply, summary.json).
- GT 폴더 (`imu_gt_spl_cal1022` 있으면 그것, 없으면 `imu_gt_spl`) 는 렌즈 K·dist 와 채점에 씀. make_reduced 는 프레임 시각도 GT 의 t_ns 에서 읽음 (가정: = frames_cam5.csv 시각).
- **주의 (11-12 고침 M1, 동작은 그대로)**: GT 는 채점만이 아니다. `scan_offline.py` 는 GT 의 `ok` (그 프레임에서 보드 자세를 구했는가) 가
  꺼진 키프레임을 **메시 입력에서도 뺀다** (133820: 키프레임 203 개 중 7 개, 프레임 11533–11748). 그래서 보드가 안 보이는 구간의 물체는
  GT 가 있는 녹화에서 메시에 덜 들어간다. 빠진 수는 `mesh/summary.json` 의 `n_kf_gt_not_ok_dropped` (요약 `mesh.n_kf_gt_not_ok_dropped`).
  끄려면 `--mesh-args '--keep-gt-not-ok'` (그 키프레임도 메시에 넣고, 그 키프레임의 보드 깊이 채점만 건너뜀). 기본값은 아직 옛 동작.

## 3. 근거
- 키프레임만: kfonly README 10·11. 지금 파이프라인에서 영상 전체를 읽는 곳은 VIO 뿐이고 (BA·움직임 스테레오 짝은 키프레임끼리),
  **같은 자세** (A 의 BA 자세) 를 주면 960 q90 · 1080 q90 키프레임 모두 A 와 |dZ| p90 −0.1 / +0.3 % (11.1 절, 133820, 1 회) → 영상 화질 손실 없음.
  B·B1080 의 +6 / +10 % 는 **BA 자세 흔들림** (SIFT 특징이 조금 달라져 키프레임 자세 중앙 0.1–0.2 mm, 최대 2–3 mm, 0.1–0.3°) 몫. 축척 k 는 모든 변형에서 0.02 %p 안.
  데이터: 영상 전체 약 1400 MB/분 → 1080 q90 키프레임 약 112 MB/분 (960 q90 이면 약 35 MB/분).
- GPU BA: gpuba README 9. 같은 db 에서 CPU BA 와 자세 차 ~1e-15, k·|dZ| 같음, BA 단계 5.9× 빠름 (H 4 개, 판정 n 4/11 이라 미완).
  CPU SIFT = GPU SIFT 의 실행마다 다른 키포인트 (경쟁 조건 추정) 를 없앰, 녹화당 약 +4–6 s (추정).
- TSDF 촘촘한 블록 할당: ghost README 6 (`runs/ghost_20261111/`, 브랜치 ghost-20261111 e6cf7b4). 원인 = `integrate` 가 화소마다 z−mu, z, z+mu
  세 점만 블록을 할당해서, mu (7–10 cm) 가 블록 한 변 (8 복셀 = 4 cm) 보다 크면 z 와 z−mu 사이 블록을 대부분의 키프레임이 건너뜀 →
  처음 몇 키프레임이 남긴 앞쪽 면이 지워지지 않고 두 번째 층으로 남음. 134550 (A 자세): 이중층 0.336 → 0.000, |dZ| p90 같음, 융합 시간 같은 수준.
  `tsdf_map.py` 의 기본값은 False 그대로 (실시간 서버 등 다른 호출은 안 바뀜); 스캐너 경로 (run_scanner → scan_offline) 만 dense 를 넘김.

## 4. 확인 실행 (11-11, 133820 1 개, 노트북, 1 회, 미검증)
`tools/remote_laptop.sh job 133820 scan --args '--board auto'` (커밋 e2d8bfb, 결과 `runs/laptop/20261022_133820/scan/scan_summary.json`).
- 끝까지 돎 (rc 0). `[device] BA cuda, SIFT cpu, matcher auto`. 입력 = `frames_kf` 342 장 1920×1080 (124 MB, 112 MB/분), cam5.mp4 없음.
  키프레임 342 = kfonly B1080 과 같은 수. VIO 재생 결과 = featloop/kfonly 가 쓴 VIO 파일과 자세 행 3833 개 모두 같음 (t_ns·상태·자세).
| | |dZ| p90 cm | 축척 k % | 메시 RMS cm |
|---|---|---|---|
| 이번 (기본값: 1080 q90, GPU BA, CPU SIFT) | 0.735 | −1.374 | 0.690 |
| kfonly B1080 (GPU SIFT, CPU BA) | 0.780 | −1.373 | 0.783 |
| kfonly A (영상 전체) | 0.710 | −1.376 | 0.444 |
| 이번 + 보드 축척 (mesh_board) | 0.726 | −0.576 | 0.311 |
- |dZ| p90 은 B1080 보다 −5.7 % (BA 특징이 CPU SIFT 로 바뀐 몫 = BA 자세 흔들림 ±10 % 안, 좋아졌다고 읽지 않음). k 는 0.001 %p 차.
  메시 RMS 는 이중층에 흔들리는 지표 (kfonly 6.1) 라 판정에 안 씀.
- 단계별 시간 (노트북 RTX 4060): vio 88.7 s, reduce 17.7, kfba 68.5, mesh 50.3, board 39.0, mesh_board 47.7 = 312 s
  (+ 녹화 보내기 15 s, 전체 작업 341 s). vio·reduce 는 폰 앱이 하게 되면 PC 에서 빠짐 → PC 몫 약 120 s (보드 빼고).

## 4.1 유령층 고침 확인 (11-11, 133820 1 개, 노트북, 1 회, 미검증)
`tools/remote_laptop.sh job 133820 scan --args '--board auto' --tag ghost` (커밋 ff49d18, 결과 `runs/laptop/20261022_133820/ghost/scan/`).
4 절 실행과 다른 것은 TSDF 할당 하나 (vio·kfba 는 다시 돌렸지만 k 가 소수 4 자리까지 같음 = 같은 BA 자세로 봄). 끝까지 돎 (rc 0).
이중층 = `runs/kfonly_20261111/mesh_layers.py` (보드 안쪽 꼭짓점 중 중앙면에서 0.5 cm 넘게 떨어진 비율).
|dZ| p90 은 축척 k 가 섞인 값 (깊이 vs 보드 평면, VIO 축척 그대로).
| | 이중층 | |dZ| p90 cm (모은 화소) | kf 별 p90 중앙 cm | 축척 k % | 보드 덮임 | mesh 단계 s |
|---|---|---|---|---|---|---|
| 4 절 (sparse) | 0.451 | 0.735 | 0.575 | −1.374 | 0.55 | 50.3 |
| **이번 (dense)** | **0.069** | 0.735 | 0.575 | −1.374 | 0.96 | 46.2 |
| 4 절 + 보드 축척 (sparse) | 0.165 | 0.726 | 0.547 | −0.576 | 0.90 | 47.7 |
| 이번 + 보드 축척 (dense) | 0.061 | 0.725 | 0.547 | −0.576 | 0.96 | 50.1 |
- 깊이·자세는 안 바뀜 (|dZ| p90 차 < 0.001 cm). 이중층 0.451 → 0.069: 옛 메시의 앞쪽 층 (중앙면 −0.5…−1.5 cm, 812 꼭짓점) 은 0 개.
  남은 6.9 % (466 꼭짓점) 는 전부 +0.5…+1.5 cm 쪽, 64 % 가 보드 가장자리 5 cm 안 → 유령층이 아니라 가장자리·뒤쪽 두께로 추정 (미조사).
  134550 의 0.000 과 달리 0 은 아님. 주 층 RMS 0.35 → 0.16 cm, 보드 덮임 0.55 → 0.96.
- 시간: vio 85.0, reduce 20.9, kfba 64.2, mesh 46.2, board 41.1, mesh_board 50.1 = 307.5 s (4 절 311.9 s; 실행 1 회씩이라 차이는 잡음 수준).

## 5. 아직 없는 것 (폰 앱 몫)
- 폰에서 **실시간 VIO** 를 돌려 자세를 같이 보내기 (지금은 PC 재생 대용: H.264 복호 영상, `loop-sync 1`, `drop-late 0` — 실제 폰 자세는 조금 나쁠 수 있음, 미측정).
- 폰에서 **키프레임 고르기** (VIO tracking 10 프레임마다, 1 ms 안) 와 **1920×1080 JPEG q90 보내기** (카메라 원본에서 바로; 지금은 mp4 복호 → JPEG 라 한 번 더 압축된 비관적 입력).
- 보낸 폴더 형식 = `make_reduced.py` 가 만드는 것 (`frames_kf/NNNNN.jpg`, `phone_live_poses.jsonl.gz`, imu.csv, frames_cam5.csv, meta.json, manifest.json) + 렌즈 파일.
- 키프레임만 쓰는 경로의 판정은 n = 1–4 (kfonly 9 절 계획 변경) — 녹화를 늘린 확인은 안 함.

## 6. FFS 짝 수 (`--ffs-views`) 와 짝 찾기 (`--ffs-partners`) (11-11, 133820 1 개, 노트북, 각 1 회, 미검증)
실험 폴더 `runs/ffsviews_20261111/` (queue*.sh, table.py → table.json). 결과 `runs/laptop/20261022_133820/fv_<tag>/`.
**지금 기본값이 하는 일** (`scan_offline.py --n-pairs 2`, 옛 코드 그대로): 키프레임마다 후보 = 앞뒤 ±150 행 중 `score_pair` 통과
(시간차 0.15–2.5 s, 기선 4–30 cm, 기선–광축 ≥ 40°, 회전 ≤ 25°), 점수순으로 첫 짝 + 기선 방향이 25° 이상 다른 둘째 짝 → FFS 2 회 →
`combine_motion` (역분산 평균, 한 쌍이 3σ 넘게 어긋나면 그 화소 버림, 한 쌍만 유효한 화소는 그대로).
즉 **깊이 지도 1 장 = 최대 3 장 (기준 + 짝 2)**. 133820 에서는 336 키프레임 중 짝 0 개 262, 1 개 57, 2 개 17 (평균 1.2 쌍).
- `--ffs-views N` = 기준 + 짝 N−1 (기본 3 = 위 동작 그대로). 2 = 짝 1 개. 4·5 = 첫 두 짝은 옛 규칙 그대로, 더 붙는 짝 (`pick_extra`):
  ① 부호 있는 기선 방향이 고른 짝들과 25° 이상 다른 것 (같은 이동선의 반대쪽도 새 방향 → 가림이 반대쪽), ② 없으면 같은 방향이라도 기선 길이 ×1.5 이상 다른 것.
  이웃 키프레임 같은 거의 같은 짝은 안 붙임. 3 쌍 이상은 `fuse_median` (화소별 중앙값, 각 쌍 σ (깊이 잡음 모델, ∝ Z²) 의 3 배 안이면 인라이어,
  유효 쌍 ≥ 2 인 화소는 인라이어 2 개 이상일 때만 남김, 값 = 인라이어 역분산 평균). `--ffs-fuse mean|median` 으로 바꿀 수 있음.
- 짝이 없는 이유 (`pair_diag.py`, 자세만): 시간차 상한 2.5 s 안에서는 손이 천천히 움직여 기선이 4 cm 가 안 됨 (첫 탈락 이유: 기선 149, 회전 83, 전진 29 / 342).
  회전 ≤ 25°·기선 2–40 cm 인 쌍의 93 % 가 2.5 s 밖 (중앙 18 s). 시간 상한만 없애면 짝 있는 키프레임 81 → 313 / 342. `--compress` 는 원인 아님 (키프레임 행만 남기고 ±150 행 창은 시간 상한보다 넓음).
- `--ffs-partners all` = 모든 키프레임에서 짝 찾기 + 시간차 ≤ `--ffs-dt-max` 30 s + 짝 검사 `--pair-check dy` (FFS 전에 SIFT 대응을 이 쌍의 정류로 옮겨 |dy| 중앙 ≤ 1.2 px, motion_stereo gate).
  상한 없이 (a*, d*) 하면 녹화 처음 키프레임 (행 2·3·5) 이 85 개 키프레임의 짝으로 뽑혔는데 FFS 유효 0.10–0.14 이거나 보드 |dZ| p90 44 cm (확신 있게 틀림).
  행 2 는 BA 회전이 GT 대비 0.33° 어긋남 (다른 키프레임 0.06°); 행 3·5 는 dy 검사도 통과해서 원인 못 찾음 → 30 s 상한은 이 녹화에서 고른 값 (과적합 위험).
  FFS 뒤 유효 비율 문턱 (`--pair-min-valid 0.2`, g*) 은 더 나빠서 기본 꺼짐.
- 기본값 (`--ffs-views 3 --ffs-partners near`) 결과는 옛 코드 (bcc2c75) 와 같은 BA 자세에서 mesh.ply·mesh.obj md5 같음, keyframes.json (시간 칸 빼고) 같음 (코드 커밋 085109a·470ed3f·91f2a92·1964454 마다 확인, n3b–n3e).

같은 BA 자세 (fv_n3 의 ba/, vio·reduce·kfba 한 번), mesh 단계만 다시 (`--reuse`). 축척 k −1.374 % 는 모두 같음 → |dZ| 차이는 깊이 몫 (|dZ| 값 자체는 k 포함).
시간 = 노트북 RTX 4060, 실행 1 회 (기본값 경로 6 회 반복, 옛 코드 1 포함: mesh 41–47 s). 전체 = vio 84.9 + reduce 20.9 + kfba 68.6 + mesh; PC 몫 = kfba + mesh (vio·reduce 는 폰 몫).
kf 중앙 = 키프레임별 보드 |dZ| p90 의 중앙 (평가 키프레임 수가 달라서 모든 실행이 평가한 공통 68 개의 값도 적음). 이중층 = `runs/kfonly_20261111/mesh_layers.py`.

| tag | 짝 찾기 | N | 짝 있는 kf | 쌍 | FFS s | mesh s | 전체 s (기본 대비) | PC 몫 s (기본 대비) | |dZ| p90 모은 화소 cm | kf 중앙 cm (평가 kf) | 공통 68 kf 중앙 cm | 이중층 | 보드 덮임 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| n3 (기본) | near | 3 | 74 | 91 | 29.1 | 47.2 | 221.6 | 115.8 | 0.735 | 0.574 (70) | 0.569 | 0.079 | 0.960 |
| n2 | near | 2 | 74 | 74 | 21.9 | 38.3 | 212.7 (−4.0 %) | 106.9 (−7.7 %) | 0.775 | 0.575 (70) | 0.569 | 0.000 | 0.951 |
| n4 | near | 4 | 74 | 107 | 34.5 | 67.5 | 241.9 (+9.2 %) | 136.1 (+17.5 %) | 0.737 | 0.588 (70) | 0.575 | 0.012 | 0.972 |
| n5 | near | 5 | 74 | 111 | 34.9 | 67.0 | 241.4 (+8.9 %) | 135.6 (+17.1 %) | 0.737 | 0.588 (70) | 0.575 | 0.011 | 0.972 |
| n3med | near | 3 (median) | 74 | 91 | 27.8 | 59.7 | 234.1 (+5.6 %) | 128.3 (+10.8 %) | 0.732 | 0.574 (70) | 0.569 | 0.052 | 0.960 |
| c2 | all | 2 | 218 | 218 | 44.5 | 82.6 | 257.0 (+16.0 %) | 151.2 (+30.6 %) | 0.547 | 0.432 (218) | 0.499 | 0.000 | 1.000 |
| **c3** | all | 3 | 218 | 411 | 79.7 | 133.6 | 308.0 (+39.0 %) | 202.2 (+74.6 %) | 0.527 | 0.425 (218) | 0.428 | 0.000 | 1.000 |
| c4 | all | 4 | 218 | 605 | 119.3 | 218.1 | 392.5 (+77.1 %) | 286.7 (+147.6 %) | 0.516 | 0.431 (218) | 0.446 | 0.000 | 1.000 |
| c5 | all | 5 | 218 | 738 | 147.8 | 254.3 | 428.7 (+93.5 %) | 322.9 (+178.8 %) | 0.520 | 0.420 (218) | 0.440 | 0.000 | 1.000 |
| d3 | all, dt 상한 없음 | 3 | 301 | 584 | 105.8 | 173.9 | 348.3 (+57.2 %) | 242.5 (+109.4 %) | 0.787 | 0.492 (281) | 0.428 | 0.190 | 1.000 |
| a3 | all, 상한·dy 검사 없음 | 3 | 306 | 592 | 112.7 | 165.7 | 340.1 (+53.5 %) | 234.3 (+102.3 %) | 0.604 | 0.527 (294) | 0.433 | 0.081 | 1.000 |

(a2/a4/a5, d2/d4/d5, g2–g4 는 table.json. a5 는 모은 화소 p90 31 cm — 먼 짝 때문. mesh s 에는 FFS 말고 정류·합치기·TSDF·dy 검사 (c* 8–13 s) 가 들어 있음.)
- 시간 이웃만 쓰면 장수를 늘려도 (n4·n5) 후보가 없어 쌍 +16–20 개뿐, |dZ| 그대로, PC 몫 +17 %.
- 짝 찾기를 넓히면 (c*) 짝 있는 키프레임 22 % → 65 %, 모은 |dZ| p90 −26…−30 %, 이중층 0.079 → 0, 보드 덮임 1.0. 같은 키프레임 (공통 68) 에서는 c3 이 가장 좋음 (0.428, 기본 0.569);
  4·5 는 시간만 늘고 나아지지 않음. **추천 (미확정, 녹화 1 개·1 회)**: `--ffs-partners all --ffs-views 3` (c3), PC 몫 +86 s (+75 %), 전체 +39 %.
  싸게: c2 (PC +35 s, +31 %), 공통 키프레임 |dZ| 는 c3 보다 0.07 cm 나쁨. **기본값은 아직 안 바꿈** (다른 녹화에서 먼저 확인; 30 s 상한 근거가 이 녹화 하나).
- n3 의 이중층 0.079 (4.1 절 ghost 0.069) 는 평균 합치기 몫으로 보임: 짝 1 개 (n2) 0.000, 중앙값 합치기 (n3med) 0.052. 원인 미조사.

## 7. 움직임 키프레임 + 넓은 짝 = 새 기본값 (11-11, 133820 1 개, 노트북, 1 회, 미검증)
비교 실험: `runs/kfsel_20261111/README.md` (사전 등록 규칙, interval vs motion 4 변형; M2·M3g 합격, M3·M4 불합격; 판정 n = 1 이라 약함 —
같은 문턱에서 창 안 프레임만 달라도 k 가 0.5 %p 움직임). 사용자 결정: M3g (3 cm / 4°, win 4, gyro) + 6 절의 `--ffs-partners all --ffs-views 2`.
확인 실행: `tools/remote_laptop.sh job 133820 scan --tag newdefault` (기본값, 커밋 c13f22e, 결과 `runs/laptop/20261022_133820/newdefault/scan/`). 끝까지 돎 (rc 0).
| | 옛 기본값 (interval + near/3, 6 절 n3) | M3g (motion + near/3, kfsel) | **새 기본값 (motion + all/2)** | 참고 c2 (interval + all/2, 6 절) |
|---|---|---|---|---|
| 키프레임 (보낸 것) | 342 | 203 | **203** | 342 |
| 키프레임/분 · JPEG MB/분 | 297 · 112 | 176 · 66 | **176 · 66** | 297 · 112 |
| kfba s | 68.6 | 37.1 | **36.7** | 68.6 |
| mesh s | 47.2 | 42.6 | **63.1** | 82.6 |
| PC 몫 (kfba + mesh) s | 115.8 | 79.7 | **99.8** | 151.2 |
| 전체 s (vio + reduce + kfba + mesh) | 221.6 | (vio 생략) | **204.9** | 257.0 |
| 짝 있는 키프레임 | 74 / 336 | 82 / 196 | **160 / 196 (0.82)** | 218 / 336 |
| |dZ| p90 모은 화소 cm | 0.735 | 0.685 | **0.614** | 0.547 |
| kf 별 p90 중앙 cm (평가 kf) | 0.574 (70) | 0.535 (79) | **0.449 (160)** | 0.432 (218) |
| 축척 k % | −1.374 | −1.286 | **−1.286** | −1.374 |
| 이중층 | 0.079 | 0.004 | **0.000** | 0.000 |
| 보드 덮임 | 0.960 | 0.96 | **1.000** | 1.000 |
- 키프레임 선택은 M3g 를 그대로 재현: 키프레임 203 개, 자이로·창 위치 분포 같음, BA k −1.286 같음 (kfba 36.7 vs 37.1 s). 메시만 FFS 짝 규칙 (all/2) 만큼 바뀜.
- |dZ| 는 축척 k 가 섞인 값 (VIO 축척 그대로). 새 기본값은 옛 기본값보다 모은 p90 −16 %, kf 중앙 −22 %, PC 몫 −14 %, 데이터 −41 %.
  c2 (키프레임 342 개 + 넓은 짝) 보다는 |dZ| 가 0.07 / 0.02 cm 나쁘고 PC 몫 −34 %. 짝 없는 키프레임 36 개 (dy 검사 탈락 8).
- 6 절의 "공통 68 kf" 지표는 키프레임 집합이 달라서 이 실행엔 적용 안 됨.
- 메시는 뷰어에 "[새] 133820 · 새 기본값 (움직임 키프레임 + 넓은 짝)" (`runs/kfsel_20261111/publish.py`).
- 남은 것: 다른 녹화 (134550, 134204 등) 로 같은 비교 (motion 문턱·30 s 상한 모두 133820 하나에서 고름).

## 8. 빨간 화면으로 크기 맞춤 (`--scale-ref redscreen`, 11-11, 202906 1 개, 노트북, 1 회, 미검증)
쓰는 법: 노트북에 빨간 전체 화면 페이지를 띄워 (http://100.92.214.97:8797/red.html, F11 전체 화면) 스캔하는 동안 화면 네 변이
여러 번 다 보이게 찍는다. 그다음
`run_scanner.sh REC OUT --scale-ref redscreen [--red-size 302.0x188.75]` (노트북 작업: `tools/remote_laptop.sh job REC scan --args '--scale-ref redscreen'`).
- `--red-size` 기본값 302.0 x 188.75 mm = 14" Lenovo 83K9 화면 (1920x1200 px x 0.15729 mm, EDID 간격) — **가정** (자로 잰 값 아님). 다른 화면이면 꼭 바꾼다.
- 순서: BA 뒤 `redscreen_scale.py` (키프레임에서 빨간 사각형 네 변, 각 변 양끝 15 % 빼고 가운데만 직선 맞춤 — 모서리가 둥금,
  BA 자세로 직사각형 묶음 맞춤) → s = 잰 크기 / 기준 → BA 위치 / s → `OUT/mesh_red/` (메시 다시). 원래 `OUT/mesh/` 도 남는다.
- `scan_summary.json`: `redscreen` (s, sigma, W/H mm, 변 맞춤 rms, 모서리 각, 키프레임 수), `scale_status`, `scale_source: redscreen`, `scale_applied`, `final_mesh`.
- **조용히 넘어가지 않음**: 키프레임 < 20, 변 맞춤 rms 중앙 > 0.6 px, 모서리 각 90±1° 밖, W/H 가 기준과 3 % 넘게 다름, 묶음 잔차 중앙 > 2 px,
  문턱 0.3/0.7 차 > 1.5 %, s 가 0.8…1.25 밖 → `!!!! RED-SCREEN SCALE FAILED`, `scale_status: failed: ...`, 종료 코드 3, 크기 안 바꾼 결과만 남음
  (확인: `--red-size 300x225` 로 돌리면 "W/H 1.595 vs reference 1.333" 로 실패, 자세 파일 안 씀).
- GT 없는 녹화: GT 폴더가 없으면 `--lens FILE` 로 렌즈를 주거나, std + 초점 0.5 dpt 녹화는 자동으로 0.5 dpt 앱 렌즈
  (`config/lens/std_1920x1080_f0.50_calcap20261026.json`, c4f60 의 렌즈) 를 쓴다. 이때 `REC/lensonly/` (렌즈 + VIO 자세로 만든 대역, GT 아님)
  를 녹화 폴더 안에 쓴다. 그 밖의 녹화는 렌즈를 모르면 멈춘다. k_pct / 보드 숫자는 이때 의미 없음 (summary 에 note).
- 202906 확인 (커밋 cb7cc36, 노트북): 렌즈 자동 = 0.5 dpt calcap, VIO c4f60. s = 1.0489 ± 0.0034 (스캐너 +4.9 % 큼), 259/324 키프레임,
  변 맞춤 rms 0.21 px, 모서리 각 89.95°, W/H 1.595 (기준 1.600). 크기 맞춘 자세로 다시 재면 301.5 x 189.1 mm (s 1.0000) —
  **같은 측정으로 맞추고 같은 측정으로 확인한 것이라 순환 확인일 뿐** (독립 확인 아님).
- 늘어난 시간: redscreen 34.9 s + mesh_red 106.2 s = 141 s (전체 346.7 s, 원래 205.6 s 정도). 메시를 두 번 만들어서 —
  크기 맞춤 성공 시 원래 메시를 건너뛰면 105 s 줄일 수 있음 (안 함: 실패해도 결과가 남아야 해서).
- 뷰어: "202906 · 빨간 화면으로 크기 맞춤" (윤곽 = 기준 302 x 188.75 mm, 빨간 면이 윤곽에 맞아야 함), 비교: "202906 · 빨간 화면 크기 시험" (맞추기 전).

## 9. 이어 찍기 = 녹화 여러 개를 메시 하나로 (`run_merge.sh`, 11-12, 121801 을 둘로 나눠 시험, 노트북 1 회, 미검증)
한 장면을 녹화 두 개 (이상) 로 나눠 찍고 메시 하나를 얻는다. 기본 스캐너 (`run_scanner.sh`) 는 그대로다.
```bash
scan3d/scanA/run_merge.sh OUT REC_A REC_B [REC_C ...] [run_scanner 옵션, 예: --depth-sigma ba --mesh-args '--tsdf-max-blocks 250000']
#   --vio-args-of N 'ARGS'  N 번째 녹화 (1 = A) 의 VIO 재생 옵션 (시험용: '--max-secs 35', '--start-off 25 --yaml init_dyn_use=true')
#   --align-args '...'      merge_align.py 옵션 (--max-sub 80 --min-kf 5 --min-pnp 30)      --lens FILE (기본 0.5 dpt 앱 렌즈)
```
- 순서: 녹화마다 지금과 똑같이 VIO + 키프레임 (`run_scanner.sh --stop-after reduce`) → `merge_align.py` (A 의 키프레임 80 장 × B 의 80 장 SIFT 짝 전부 →
  A 의 VIO 자세로 A 쪽 점 삼각측량 → B 키프레임을 PnP 로 A 세계에 놓음 → **4 자유도** (방향각 + 이동; 두 VIO 모두 중력 z 위·미터 단위) 를 다수결로 맞춤.
  남은 기울기와 Sim(3) 축척비는 검사용으로만 기록) → 키프레임 폴더 하나 (B 자세는 A 세계로, 프레임 번호 이어 붙임, B 시각은 A 끝 + 100 s) →
  `run_scanner.sh` 그대로: kf_ba 한 번 (녹화 안 짝은 지금처럼 + 맞춘 VIO 자세로 생기는 녹화 사이 공시 짝 + merge_align 의 검증된 녹화 사이 짝
  `--extra-pairs`), FFS 깊이 짝은 **같은 녹화 안에서만** (시각 차 30 s 상한), TSDF 하나, 메시 하나 (A 의 세계 좌표).
- **안 겹치면 멈춤**: PnP 로 놓인 B 키프레임이 5 개 미만이거나 5 개 이상이 한 변환에 동의하지 않으면 `MERGE FAILED: ...` 와 종료 코드 6,
  메시 안 만듦. 결과 `OUT/merge_summary.json`, `OUT/merged/<A>_merged_kf_align/merge.json` (변환, 동의 수, 중심 오차, 기울기, 축척비).

### 이어 찍는 법 (두 번째 녹화)
1. 첫 녹화를 끝낼 때 **끝 장면을 기억** 한다 (끝 2–3 초는 천천히, 특징 많은 곳: 선반·공구판·포스터. 흰 벽·바닥만 보며 끝내지 않기).
2. 두 번째 녹화는 **폰을 바닥·책상에 내려놓고 가만히** 시작한다 (앱 안내의 처음 5 s 멈춤 = VIO 초기화. 손에 들고 움직이며 시작하면 VIO 가 늦게 켜지거나 축척이 틀어질 수 있음).
3. 들어 올린 뒤 **처음 5–10 초 동안 첫 녹화의 끝 장면을 다시 비춘다** (비슷한 거리·방향, 1–2 m). 이 겹친 부분으로 두 녹화를 잇는다. 겹침은 10 초 이상 권장.
4. 그다음 새로운 곳으로 이어 간다. 같은 폰·같은 앱 설정 (초점 0.5 dpt, 같은 렌즈) 이어야 한다 (렌즈가 다르면 멈춤).
5. 셋 이상도 같다: 각 녹화는 앞 녹화들 중 어딘가와 겹치게 시작.

### 시험 (121801 을 둘로 나눔, 노트북 1 회, 미검증; 자세한 표는 `runs/fixes_20261112/README.md` 10 절)
- A = 0–35 s (`--max-secs 35`, 처음 2 s 정지 → 정지 초기화), B = 25–60 s (`--start-off 25`). 이 녹화는 처음 약 2 s 말고는 멈춘 곳이 없어 (IMU 1 s 창 가속도 표준편차 ≥ 0.2)
  B 는 정지 초기화가 안 됨 → **B 만 동적 초기화** (`--yaml init_dyn_use=true`, 2.7 s 에 켜짐). 실제 두 번째 녹화는 정지로 시작하면 되므로 이 옵션은 시험용.
- 맞춤: B 키프레임 80 장 중 29 장 PnP, 25 장 동의, 중심 오차 중앙 3.0 cm, 기울기 0.5°, 축척비 1.026. 합친 BA 0.29 px, 정점 1 918 406 (한 번에 찍은 u8 1 905 456).
- 전체 녹화 BA (u8) 와 키프레임 중심 ATE 10.1 cm rms (SE3 맞춤; A 7.5 / B 12.1 cm). 위에서 본 모습은 u8 과 거의 같음.
- 안 겹치는 녹화 (121801 앞 35 s + 220612 다른 방): 검증된 녹화 사이 짝 0/6400 → `MERGE FAILED`, 종료 코드 6, 메시 안 만듦 (확인).
- 주의: 첫 실행은 mesh 단계에서 GPU 메모리 부족 (rc 3; u8 도 예약 5.95 / 상한 6.0 GB 로 빠듯했음). 같은 BA 자세로
  `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` 주고 mesh 만 다시 (`--reuse`) 하니 성공. run_merge 자체는 이 변수를 안 넣음 (기본 동작 그대로).
