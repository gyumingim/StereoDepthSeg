# scanA — 오프라인 폰 3D 스캐너 (키프레임 BA → FFS 깊이 → TSDF 메시)

브랜치 `scanner-defaults-20261111` (featloop-20261105 7384db5 + gpu-ba-20261109 + kfonly-20261111 + laptop-worker-20261111).
실험 기록은 `scan3d/runs/<이름>/README.md` (scanA_20261105, featloop_20261105, gpuba_20261109, kfonly_20261111). 이 파일은 "지금 기본값과 돌리는 법" 만.

## 1. 기본값 (사용자 결정 2026-11-11)
| 항목 | 기본값 | 옛 동작 (플래그) |
|---|---|---|
| 입력 | **키프레임만** (`REC/frames_kf/NNNNN.jpg`; `kf_frames.py` 가 옆에 `cam5.mp4` 가 있거나 키프레임이 빠지면 멈춤) | `--full-video` (cam5.mp4 를 복호) |
| 키프레임 화질 | **1920×1080 JPEG q90** 로 받아서 PC 가 960×540 (INTER_AREA) 으로 줄임. kf_ba · FFS · TSDF 계산은 전부 960 | `--kf-res 960` (kfonly B), `--kf-quality Q` |
| BA | **GPU** (`kf_ba.py` 기본 `--ba-device cuda`, `--sift-device cpu`, `--fast-tracks`; CUDA 없으면 cpu 로 바꾸고 `[device]` 줄에 찍음) | `--ba-device cpu`, `--sift-device auto` |
| 키프레임 고르기 | VIO tracking 프레임 10 개마다 (`--kf-from-vio --match-ms 1`), 짝 = 다른 키프레임 | `--kfba-args` |
| 자세 | 폰 실시간 VIO 자세. **지금은 대용**: PC 재생 `lp_replay.py` + 폰 설정 c4f60 (H = `cfg_gen_hs222/c4f60`, 그 밖 `cfg_gen/c4f60`) | `--vio FILE`, `--cfg DIR` |
| 깊이·메시 | `scan_offline.py` FFS 960×540 + TSDF 5 mm, `--compress --kf-step 1` | `--mesh-args` |
| 축척 | 그대로 (VIO 축척). `--board auto|FILE` 이면 보드 축척 (`board_scale.py`, 1920 키프레임에서 ChArUco 검출) + 그 자세로 메시 한 번 더 | |

## 2. 돌리는 법
```bash
# 노트북 작업 PC 에서 (계산은 노트북만; 결과는 runs/laptop/<RID>/scan/ 로 돌아옴)
cd ~/scanner/StereoDepthSeg-defaults/scan3d
tools/remote_laptop.sh job 133820 scan                         # 기본값
tools/remote_laptop.sh job 133820 scan --args '--board auto'    # + 보드 축척
tools/remote_laptop.sh job 133820 scan --args '--full-video --ba-device cpu --sift-device auto' --tag old   # 옛 경로
# 직접 (그 기계에서): scan3d/scanA/run_scanner.sh REC OUT [옵션]   (옵션 전체는 스크립트 머리말)
```
REC 가 원래 녹화면: vio (재생) → reduce (`runs/kfonly_20261111/make_reduced.py --full-res`, 폰이 보낼 폴더를 흉내: 키프레임 JPEG +
imu.csv + frames_cam5.csv + meta.json + `phone_live_poses.jsonl.gz`, 영상 없음) → kfba → mesh [→ board → mesh_board].
REC 가 이미 키프레임 폴더 (`frames_kf/` + `phone_live_poses.jsonl.gz`) 면 vio·reduce 를 건너뜀 = 폰 앱이 보낸 것을 그대로 처리.
결과: `OUT/scan_summary.json` (만든 시각, 커밋, 입력, 단계별 초, |dZ| p90, 축척 k, 보낸 JPEG MB/분), 단계별 로그 `OUT/*.log`,
줄인 폴더 `OUT/red/` (처리 전 입력), `OUT/ba/`, `OUT/mesh/` (mesh.ply, summary.json).
- GT 폴더 (`imu_gt_spl_cal1022` 있으면 그것, 없으면 `imu_gt_spl`) 는 렌즈 K·dist 와 채점에만 씀. make_reduced 는 프레임 시각도 GT 의 t_ns 에서 읽음 (가정: = frames_cam5.csv 시각).

## 3. 근거
- 키프레임만: kfonly README 10·11. 지금 파이프라인에서 영상 전체를 읽는 곳은 VIO 뿐이고 (BA·움직임 스테레오 짝은 키프레임끼리),
  **같은 자세** (A 의 BA 자세) 를 주면 960 q90 · 1080 q90 키프레임 모두 A 와 |dZ| p90 −0.1 / +0.3 % (11.1 절, 133820, 1 회) → 영상 화질 손실 없음.
  B·B1080 의 +6 / +10 % 는 **BA 자세 흔들림** (SIFT 특징이 조금 달라져 키프레임 자세 중앙 0.1–0.2 mm, 최대 2–3 mm, 0.1–0.3°) 몫. 축척 k 는 모든 변형에서 0.02 %p 안.
  데이터: 영상 전체 약 1400 MB/분 → 1080 q90 키프레임 약 112 MB/분 (960 q90 이면 약 35 MB/분).
- GPU BA: gpuba README 9. 같은 db 에서 CPU BA 와 자세 차 ~1e-15, k·|dZ| 같음, BA 단계 5.9× 빠름 (H 4 개, 판정 n 4/11 이라 미완).
  CPU SIFT = GPU SIFT 의 실행마다 다른 키포인트 (경쟁 조건 추정) 를 없앰, 녹화당 약 +4–6 s (추정).

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

## 5. 아직 없는 것 (폰 앱 몫)
- 폰에서 **실시간 VIO** 를 돌려 자세를 같이 보내기 (지금은 PC 재생 대용: H.264 복호 영상, `loop-sync 1`, `drop-late 0` — 실제 폰 자세는 조금 나쁠 수 있음, 미측정).
- 폰에서 **키프레임 고르기** (VIO tracking 10 프레임마다, 1 ms 안) 와 **1920×1080 JPEG q90 보내기** (카메라 원본에서 바로; 지금은 mp4 복호 → JPEG 라 한 번 더 압축된 비관적 입력).
- 보낸 폴더 형식 = `make_reduced.py` 가 만드는 것 (`frames_kf/NNNNN.jpg`, `phone_live_poses.jsonl.gz`, imu.csv, frames_cam5.csv, meta.json, manifest.json) + 렌즈 파일.
- 키프레임만 쓰는 경로의 판정은 n = 1–4 (kfonly 9 절 계획 변경) — 녹화를 늘린 확인은 안 함.
