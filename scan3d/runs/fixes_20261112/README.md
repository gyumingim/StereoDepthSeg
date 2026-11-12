# fixes_20261112 — 검토 지적 고침 (브랜치 fixes-20261112)

- 만든 시각: 2026-11-12. 입력: `C:\Users\a3162\scan3d_handoff\review_20261112\verify_code.md` (검증된 지적), `code_review.md`.
- 시작점: scanner-defaults-20261111 a4541d9. 계산은 전부 노트북 (`scan3d/tools/remote_laptop.sh`), BA 는 GPU.
- 모든 숫자는 녹화 133820 하나, 실행 1 회 (따로 적지 않으면 "1회, 미검증").

## 1. 단위 1 — 결과가 바뀌지 않는 고침 (커밋 b48e088)

### 고친 것
| 지적 | 고친 곳 | 내용 |
|---|---|---|
| H2 잠금·고아 작업 | `tools/remote_laptop.sh` | 잠금 주인 = 호스트:PID:시각:작업 id. 지시 셸이 죽어도 (SIGTERM/SIGHUP/링크 끊김) **노트북 작업이 살아 있으면** EXIT trap 이 녹화·캐시를 지우지 않고 잠금도 안 풂. 다음 작업은 "지시 셸 없음 + 노트북에 `laptop_jobs/*/run.sh` 없음 + 잠금 내용 그대로" 일 때만 노트북에서 한 명령으로 잠금을 지움 (그 작업의 캐시도 같이). 캐시는 작업마다 `laptop_cache/<작업 id>`. CLI 는 같음 |
| M8 잠금 경쟁 | 같은 파일, `scanA/gpu_guard.py` | PID 재사용은 `ps` 로 `remote_laptop.sh` 인지 확인. 폴링 최대 12 시간 (`POLL_MAX_S`). `unlock` 은 노트북 작업이 돌면 거부 (`FORCE=1` 로만). gpu_guard 는 살아 있는 python 작업을 나이(2 h)만으로 뺏지 않음 |
| H3 VIO 재생 실패가 rc 0 | `runs/livepose_20261030/lp_replay.py`, `scanA/run_scanner.sh` | 브리지 종료 코드를 json 에 기록 (`bridge_rc`), 입력 뒤에 죽어도 `crashed` 를 채움, 실패면 자세·json 을 쓴 뒤 **rc 2**. run_scanner 는 그 단계에서 멈추고 `scan_summary.json` 에 `status: failed: step vio rc 2`, `vio_crashed` 기록 |
| M2 옛 숫자 섞임 | `run_scanner.sh` | 시작할 때 옛 `mesh/ board/ mesh_board/ redscreen/ mesh_red/ scan_summary.json` 을 `OUT/_prev/` 로 옮김 (지우지 않음). 요약은 이번에 돈 단계 (`times.txt`) 만 읽음 |
| M3 품질 문턱 | `run_scanner.sh`, `scan_offline.py`, `kf_ba.py` | 메시 정점 < `--min-mesh-vertices` (기본 1000) 이면 rc 5 + 요약 `status failed`. 요약에 `ba_quality.n_kf_vio_pose` (관측 15 개 미만이라 VIO 자세로 남은 키프레임 수), `final_med_px`, `mesh.tsdf_full` / `tsdf_blocks` (블록 한도 60000), TSDF 블록 한도 메시지를 로그·요약에 남김 (동작은 그대로) |
| M1 GT ok 거르기 | `scan_offline.py`, `scanA/README.md` | 동작은 그대로. README 문장을 코드대로 고침. 빠진 수 `n_kf_gt_not_ok_dropped` 기록. 끄는 플래그 `--keep-gt-not-ok` (기본 꺼짐) |
| M5 | `run_scanner.sh` | `--gt 이름` 을 직접 줬는데 폴더가 없으면 멈춤. lens-only 에서 `--cfg` 없으면 경고 |
| M6 | `run_scanner.sh` | `--reuse` 와 무시되는 플래그를 같이 주면 경고. **단위 1 에서는 실제로 `--full-video` 하나에만 떴음 (검토 N1) → 단위 5 에서 고침** |
| M7 | `run_scanner.sh`, `remote_laptop.sh`, `scan_offline.py` | 요약 `provenance`: 커밋 + 작업 트리 전체 dirty, MAIN 커밋(+dirty), 렌즈 파일 sha, VIO 설정 yaml sha, 실제 import 된 `lib_ffs.py` 경로·sha |
| M9 | `run_scanner.sh` | `frames_kf` 첫 영상 크기가 `--kf-res` 와 안 맞으면 멈춤 |
| M10 | `kf_ba.py` | 영상 캐시 폴더 이름에 K·dist·크기·입력 해시 (`_L<hash>`) |
| (진단) | `scan_offline.py` | `mesh_cam_depth_m` (정점마다 그것을 시야에 둔 가장 가까운 키프레임 카메라 깊이; 가림 검사 없음), `kf_poses.npz` (TSDF 에 들어간 키프레임 자세). 메시에는 영향 없음 |

M4 (불확실 판정) 는 손대지 않음.

### 잠금 시험 (가짜 작업, 실제 작업 안 죽임)
작은 녹화 `calcap_20261025_205003` (20 KB) 로 `cmd:sleep` 작업만 씀.
1. A (`sleep 100`) 를 띄우고 내가 띄운 지시 셸에만 SIGTERM → trap: "laptop job ... is ALIVE: lock, cache ... kept". 노트북에 잠금·캐시·run.sh 남음 (확인).
2. A2 (`sleep 120`) 에 같은 SIGTERM 뒤 바로 B2 → B2 는 "has no orchestrator here but its laptop job still runs -> waiting" 으로 기다림,
   A2 가 끝난 뒤 다음 확인에서 낡은 잠금 + A2 캐시를 지우고 실행, rc 0, 정상 해제.
3. A 가 이미 끝난 뒤 B → 바로 "stale lock ... removed (+ its cache)", 실행, 해제.
로그: `~/scanner/fixes_tmp/logs/lock*.log` (WSL).

### 결과가 그대로인지 (133820, 노트북, 1 회)
`remote_laptop.sh job 133820 scan --tag fix_u1` (새 코드 b48e088) 대 저장된 `newdefault` (c13f22e, 11-11).

| 항목 | newdefault (11-11) | fix_u1 (11-12) |
|---|---|---|
| 키프레임 (보낸 것 / 메시 입력) | 203 / 196 | 203 / 196 |
| \|dZ\| p90 (보드, 모든 화소) | 0.6144 cm | 0.6144 cm (자릿수 전부 같음) |
| 키프레임별 \|dZ\| p90 중앙값 | 0.4494 cm | 0.4494 cm |
| 축척 k | −1.2864 % | −1.2864 % |
| 메시 정점 | 12 590 | 12 590 |
| `mesh.ply` | — | **바이트 단위로 같음** (`cmp` rc 0) |
| `keyframes.json` | — | `ffs_ms` (시간) 칸만 다름 |
| BA final_med_px | 0.3142 | 0.3142 |
| 단계 시간 vio / reduce / kfba / mesh | 84.3 / 20.8 / 36.7 / 63.1 s | 85.1 / 20.8 / 40.0 / 65.1 s |

- GPU BA 비결정성은 이번에는 보이지 않음 (BA 수치가 끝자리 1e-14 차이 수준).
- 새 기록: `vio.bridge_rc 0`, `crashed null`, `n_kf_vio_pose 0`, `tsdf_full false (3285 블록)`, `n_kf_gt_not_ok_dropped 7`,
  메시 정점 카메라 깊이 중앙 0.40 m / p99 0.53 m / 최대 **0.568 m** (0.65 m 넘는 정점 0 개) → 단위 2 의 "전" 값.
- 결과 파일: `scan3d/runs/laptop/20261022_133820/fix_u1/scan/` (scan_summary.json, mesh/, ba/ba.json, 로그).

## 2. 단위 2 — H1 깊이 범위 (메시가 바뀌는 고침, 기본값은 아직 그대로)

### 원인 (verify_code.md H1)
두 렌즈 기준(anchor) 이 없는 움직임 짝의 깊이 오차 모델이 "회전 0.5° + 기선 10 %" 를 가정한다 (`motion_stereo.pair_depth`).
σ = hypot(Z²·hypot(0.5 px, f·0.5°)/(f·B), 0.1·Z) 라서 σ ≤ 0.08 m (TSDF `sigma_max`) 는 기선 7.2 cm 에서 Z ≤ 0.63 m, 기선이 아무리 길어도 0.8 m 미만.
그리고 σ_ok 0.02 m 문턱 때문에 0.58 m 에서는 관측이 약 13 번 필요하다. 그래서 모니터 뒤 PC·선반·벽은 메시에 안 들어갔다.

### 새 값 고른 근거 (보드 점수에 맞춘 것 아님)
- BA 뒤 자세: 짝을 고를 때 dy 검사 (`--pair-check dy`) 가 SIFT 대응점의 세로 어긋남 중앙값 ≤ 1.2 px 인 짝만 통과시킨다.
  1.2 px / f 651.9 = **0.105°** → 통과한 짝의 상대 회전 오차는 대략 0.1° 이하 (세로 방향만 잰 것이라 가로 방향은 같다고 가정).
- BA 재투영 오차 중앙 0.31 px, 실행 사이 키프레임 자세 흔들림 중앙 0.1–0.2 mm · 최대 2–3 mm (scanA README 3 절) → 기선 7 cm 의 0.3 % 중앙, 최대 약 4 %.
  기선 오차는 **2 %** 로 잡음 (최대 쪽에 가깝게, 보수적). 전체 축척 오차는 모든 짝에 같이 걸려 융합 일관성에는 영향 없음.
- 확인: 133820 의 키프레임별 보드 오차 (FFS 깊이, TSDF 전) 에서 잰 σ ≈ |dZ| p90 / 1.645 = 중앙 0.27 cm.
  옛 모델 예측 중앙 7.17 cm (실측/예측 0.04), **새 모델 (0.1°, 2 %) 예측 중앙 1.48 cm (실측/예측 중앙 0.18, p90 0.34, 최대 0.45)** →
  새 모델도 여전히 보수적 (오차를 크게 봄). 회전 0°·기선 0 % 면 비율 최대 2.1 로 너무 낙관적.
- 새 모델의 잘림 거리 (σ = 0.08 m): 기선 4.7 / 7.2 / 8.4 cm 에서 1.36 / 1.66 / 1.78 m. `sigma_max`, `sigma_ok`, `z_max 2.0` 은 그대로.
- 플래그: `run_scanner.sh --depth-sigma old|ba` (기본 old) = `scan_offline.py --sigma-rot-deg 0.1 --sigma-b-rel 0.02`.
  `--sigma-max`, `--sigma-ok` 도 따로 있음 (안 바꿈).

### 실행 (133820, 노트북, 각 1 회, 미검증)
같은 BA 자세: 새 쪽은 `--reuse fix_u1/scan --depth-sigma ba` (vio·reduce·kfba 를 다시 안 돌림) → 바뀐 것은 깊이 잡음 모델 하나뿐.

| 항목 | old (0.5°, 10 %) = fix_u1 | ba (0.1°, 2 %) = fix_u2ba |
|---|---|---|
| 메시 정점 / 면 | 12 590 / 20 959 | **166 437 / 259 610** |
| 정점 카메라 깊이 중앙 / p90 / p99 / 최대 | 0.40 / 0.51 / 0.53 / **0.57 m** | 0.59 / 0.79 / 0.95 / **1.31 m** |
| 0.65 m 넘는 정점 / 1.0 m 넘는 정점 | 0 % / 0 % | 33.8 % / 0.4 % |
| \|dZ\| p90 보드 (모든 화소, TSDF 전) | 0.6144 cm | 0.6144 cm (같은 깊이 지도라 같음) |
| 키프레임별 \|dZ\| p90 중앙 (같은 160 키프레임) | 0.4494 cm | 0.4494 cm |
| 축척 k | −1.286 % | −1.286 % |
| 메시 보드 면 z RMS / 중앙 (TSDF 뒤) | 0.211 / −0.004 cm | 0.215 / −0.032 cm |
| 보드 덮임 | 1.00 | 1.00 |
| TSDF 블록 (한도 60 000) | 3 285 | 15 499 (한도 안) |
| mesh 단계 시간 | 65.1 s | 62.1 s (차이는 잡음 수준) |

- 보드 \|dZ\| 와 k 는 TSDF **앞** 깊이로 재므로 이 고침으로는 안 바뀐다 (예상대로). 보드 메시 면은 거의 같음 (RMS +0.004 cm).
- 그림 `u2_old_vs_ba_133820.png` (위 old, 아래 ba; 앞·왼위·오른아래). ba 는 보드 둘레 책상·모니터 받침·벽이 나온다.
  **번짐(줄무늬) 같은 조각도 보인다** — 먼 쪽 깊이의 품질은 정답이 없어 판정 못 함 (보드 밖 정답 없음).
- 최대 1.31 m: 1–2 m 의 PC 본체·선반은 일부만 들어옴 (관측 수 부족 또는 FFS 유효 화소 부족 추정; 가르지 않음).
- 보기: `C:\Users\a3162\scan3d_handoff\changes_0620\fixes_20261112\scanner_133820_sigma_{old,ba}.ply`, 뷰어 목록 `scanA/meshes.json` 에 2 개 추가
  (백업 `meshes.json.bak_fixes_20261112_103850`).

### 권고 (결정은 메인 세션)
- ba 모델은 실측 보드 오차보다 아직 2–5 배 보수적이고, 보드 결과는 그대로이며, 메시가 0.57 m → 1.3 m 까지 넓어진다 → **기본값 후보로 권고**.
- 다만 바꾸기 전에 (1) 다른 녹화 하나 (예: 222013 보드, 60 fps 0.5 dpt) 에서 같은 비교, (2) 뷰어로 먼 쪽 번짐 조각 눈 확인을 권함.
  먼 물체 치수 정답이 없으므로 "먼 쪽이 맞다" 는 주장은 아직 할 수 없다 (1회, 미검증).

## 3. 단위 3 — 줄자 녹화 220612 (와 222013)

### 녹화 옮김
- `C:\Users\a3162\scan3d_handoff\browser_upload\in\hs_20261106_220612_std60` → WSL `~/scanner/StereoDepthSeg/scan3d/recordings/hs_20261106_220612_std60`
  (cam5.mp4 954 780 273 B, `cmp` 로 같음 확인; sha 파일은 원래 없음). meta: 60 fps, 0.5 dpt, 2876 프레임, complete true, 51 s. 노트북에는 작업이 보냄.
- 222013 은 옮기지 않음 (보드 녹화, 아래).

### 줄자가 보이는가 → **안 보인다. 크기 측정은 하지 않고 멈춤.**
- 1 초마다 48 장 + 4 초마다 12 장을 봤다 (`ffmpeg` 없이 OpenCV, 창 안 띄움). 장면 = 방 둘러보기: 책상 (빨간 마우스패드, 노트북), 문, 스위치,
  옷장, 침대·옷, 실내 자전거, 운동 랙, 의자, 선풍기, 줄무늬 커튼. **줄자는 어느 장에도 없다.**
- 10 프레임마다 노란색 (줄자 몸통 색) 화소 비율도 봤다: 가장 높은 프레임 (2640–2670, 0–10) 은 의자 등받이·책상 위 물건이었음 (눈 확인).
- 사용자 계획 (메모리: "5 m 줄자 왕복 ×3, 처음·끝 멈춤") 과 다르다. "줄자 녹화" 라는 이름은 `MOVE_PC_20261111.md` 표의 설명뿐이고,
  줄자 길이 값 (cm) 도 기록에 없다. → 길이를 지어내지 않음. **빨간 화면 s = 1.0489 (202906) 와 비교할 독립 자는 이 녹화로는 만들 수 없다.**
- 222013 (보드 녹화, 3379 프레임) 은 16 장 확인: 노트북 화면의 ChArUco 보드 + 책상. 줄자 없음. 이 보드는 노트북 화면 크기 가정에 기대므로 독립 자가 아님 (보드 축척은 순환, 이전 기록).
- 그림: 확인용 장들은 임시 폴더에만 둠 (저장소에 안 넣음).

### 스캐너 실행 (기본값 = 단위 1 코드, 노트북, 1 회, 미검증)
`remote_laptop.sh job 220612 scan --tag fix_u3` → **rc 5 (새 메시 문턱이 잡음)**: `mesh.ply` 정점 **0 개**.
- GT 폴더가 없어 lens-only 대역 (`std_1920x1080_f0.50_calcap20261026.json`, VIO 설정 cfg_gen/c4f60) 으로 돎 → k·보드 숫자 없음.
- vio 39 s (브리지 rc 0, 2876 프레임 모두, 자세 2876), reduce 7 s (키프레임 285 장, 137 MB), kfba 32 s (재투영 중앙 0.30 px, VIO 자세로 남은 키프레임 0,
  BA-VIO 중심 rms 3.95 cm), mesh 89 s (깊이 지도 228/285 키프레임, FFS 유효 화소 중앙 57 %, 기선 중앙 8 cm).
- 정점 0 개 이유 (추정, 단위 2 와 같은 원인): 방 물체가 대부분 1–3 m 라 옛 깊이 잡음 모델의 잘림 (약 0.63–0.7 m) 밖. 이 녹화로 `--depth-sigma ba` 는
  돌리지 않았다 (실험은 녹화 하나 규칙, 단위 2 는 133820). 메인 세션이 원하면 `--reuse <fix_u3/scan> --depth-sigma ba` 로 mesh 만 약 1.5 분.
- 결과: `scan3d/runs/laptop/20261106_220612_std60/fix_u3/scan/` (scan_summary.json status `failed: mesh 0 vertices < 1000`).

## 4. 단위 4 — 220612 (방 둘러보기) 를 `--depth-sigma ba` 로 (노트북, 1 회, 미검증)

`remote_laptop.sh job 220612 scan --tag fix_u4ba --args '--reuse <fix_u3/scan> --depth-sigma ba'` (코드 217d404, dirty 아님).
- **재사용 됨**: vio·reduce·kfba 를 다시 안 돌림 (`steps_run` = lensonly 0.1 s + mesh). 같은 285 키프레임·같은 BA 자세 (`fix_u3/scan/ba`) →
  fix_u3 와 다른 것은 깊이 잡음 모델 하나뿐. lens-only 대역 (std 0.5 dpt 렌즈 json) 도 fix_u3 와 같음.

| 항목 | fix_u3 (old, 0.5°·10 %) | fix_u4ba (ba, 0.1°·2 %) |
|---|---|---|
| rc / status | 5 / failed: mesh 0 vertices | **0 / ok** |
| 메시 정점 / 면 | 0 / 0 | **880 571 / 1 546 274** |
| 정점 카메라 깊이 중앙 / p90 / p99 / 최대 | — | 1.22 / 1.42 / 1.51 / **1.59 m** |
| 0.65 m / 1.0 m 넘는 정점 | — | 98.3 % / 79.5 % |
| 깊이 지도 키프레임 | 228 / 285 | 228 / 285 (같음) |
| TSDF 블록 (한도 60 000) | — | **60 000 = 한도에 걸림** (`tsdf_full true`) |
| mesh 단계 / 전체 시간 | 88.9 s / 167 s (vio 포함) | 98.5 s / 98.6 s (재사용) |

- **품질 경고: TSDF 블록 한도.** mesh.log 에 `[tsdf] 블록 한도 60000 — 새 영역은 더 안 넣음` 이 키프레임 89 와 109 (285 중) 사이에 찍힘.
  그 뒤 키프레임은 이미 있는 블록만 고치고, 새 곳 (방 뒤쪽 절반: 침대·선풍기·책상 쪽) 은 메시에 없다. 동작은 단위 1 대로 그대로 둠 (한도 안 바꿈).
- 그림 `u4_220612_ba_kfviews.jpg` (왼쪽 키프레임 사진, 오른쪽 같은 카메라에서 본 메시 꼭짓점; 가림 처리는 z-버퍼 점만),
  `u4_220612_ba_top.png` (위에서 본 그림, 빨간 점 = 키프레임 카메라, 3.2 × 3.6 m).
- 눈으로 본 것: 앞 1/3 (kf 20) 은 벽·스위치·옷장 (손잡이까지)·실내 자전거 안장이 알아볼 수 있게 나옴. 위에서 보면 벽 두 면과 모서리, 옷장이 반듯함.
  kf 60 (운동 랙 앞 벽) 은 벽 위에 줄무늬·겹친 조각 (번짐) 이 많음. kf 150·240 (침대·옷·선풍기·책상) 은 거의 비어 있음 (위 블록 한도 때문).
  문·책상은 이 메시로는 확인 못 함. 떠 있는 작은 조각 몇 개 있음. 먼 쪽 치수 정답 없음.
- 메시 좌표 z 가 약 1000 m (lens-only 세계 좌표의 높이 값; 원인 안 봄). 뷰어 기본 보드 윤곽 (z=0) 은 1000 m 밖이라 안 보이지만,
  보드가 없는 녹화라 `meshes.json` 에 `board_mm: [0, 0]` (윤곽이 점 하나로 줄어듦) 을 넣음. viewer.html 은 안 고침.
- 보기: `changes_0620/fixes_20261112/scanner_220612_sigma_ba.ply` (33 MB), 목록 백업 `meshes.json.bak_fixes_u4_20261112_105408`.
- 결과: `scan3d/runs/laptop/20261106_220612_std60/fix_u4ba/scan/`. 그림 스크립트: WSL `~/scanner/fixes_tmp/render_kf.py`, `plyio.py`.
- 정리: ba 모델이면 방 녹화도 메시가 나온다 (0 → 88 만 정점). 다음 걸림돌은 **TSDF 블록 한도 60 000** (방 하나를 5 mm 로 다 못 담음) — 메인 세션 결정.

## 5. 단위 5 — 검토 지적 (review_fixes.md N1–N6) 작은 고침, 결과 안 바뀜

입력: `C:\Users\a3162\scan3d_handoff\review_20261112\review_fixes.md`. 줄 끝 (LF) 은 그대로, 다른 줄은 안 건드림.

| 지적 | 고친 곳 | 내용 |
|---|---|---|
| N2 실행 권한 | `tools/remote_laptop.sh` | git 모드 100644 → 100755, 파일도 `chmod +x` (queue 스크립트의 `tools/remote_laptop.sh job ...` 이 다시 됨) |
| N3 같은 초 작업 id | 같은 파일 | 작업 id = 시각 + 녹화 + 단계 + 태그 + 지시 셸 PID + 무작위 4 자리 16 진 (`..._cmd_tA_a_14680_b01b`). 같은 초·같은 녹화·같은 단계여도 작업 폴더·rc·캐시가 따로 |
| N6 trap 지우기 | 같은 파일 | 끝 정리를 노트북 한 명령으로: 작업 캐시는 늘 지우고, **잠금이 아직 내 것일 때만** 녹화를 지우고 잠금을 푼다. 아니면 "lock not ours ... recording and lock kept" |
| N5 `ps` 잘림 | 같은 파일 | 지시 셸 생존 확인을 `ps -o args=` 대신 `/proc/<pid>/cmdline` 전체로 |
| N4 멈춘 작업 | 같은 파일 | 환경 변수 `JOB_TIMEOUT_S` (기본 0 = 꺼짐, 예전과 같음). 주면 노트북 작업을 `timeout -k 60` 으로 감싸 그 시간 뒤 끝냄 (rc 124). 권장 예: 14400 (4 시간; 보통 scan 3–10 분). gpu_guard 는 안 고침 |
| N1 M6 경고 | `scanA/run_scanner.sh:79-92` | 둘째 `case` 가 값이 아니라 옵션 이름 (`opt=$1`, 첫 `case` 전에 잡음) 을 봄 |

### 시험 (가짜 작업만, 실제 작업 안 죽임; 로그 WSL `~/scanner/fixes_tmp/logs/testA_n1n5.log`, `tA_*.log`)
- N1 조각 (`run_scanner.sh` 75–92 줄만 떼어 실행): `--reuse /x --kf-select interval --kf-res 960 --ba-device cpu --full-video --kfba-args '--a b' --depth-sigma ba`
  → 옛 코드 `PRE_REUSE=[ --full-video]`, 새 코드 `[ --kf-select --kf-res --ba-device --full-video --kfba-args]`. 무시 플래그 없을 때 둘 다 빈 칸. 값 읽기는 같음.
- N5: 긴 경로 이름의 가짜 `remote_laptop.sh` 프로세스, `COLUMNS=40` → 옛 코드 "죽음" (틀림), 새 코드 "살아 있음". 죽은 PID → 새 코드 "죽음".
- N3: 같은 초 (10:58:54) 에 `job calcap_20261025_205003 'cmd:sleep 25' --tag tA_a &` 와 `--tag tA_b &` → id 가 다름 (`..._tA_a_14680_b01b`, `..._tA_b_14681_9020`),
  b 는 a 가 끝날 때까지 기다린 뒤 돎, 둘 다 rc 0, 각자 정상 해제.
- N4: `JOB_TIMEOUT_S=15 job ... 'cmd:sleep 120'` → 21 s 뒤 `job rc 124`, 정리·해제 정상, 노트북에 `sleep` 안 남음.
- N6: C (`sleep 90`) 도는 중 `FORCE=1 unlock` → D (`sleep 100; test -d $R`) 가 잠금을 잡음 → C 가 끝날 때 "lock not ours (… D …): recording … and lock kept",
  D 는 끝에 `REC_OK` (녹화가 안 지워짐), D 가 정상 해제. 뒤에 노트북 `laptop_cache` 비어 있음, 녹화 지워짐.
- 결과 숫자에 닿는 코드는 안 바뀜 (스캐너 계산 경로 변경 없음) → 메시 재실행 안 함.
- 안 한 것: SIGKILL·링크 끊김·재부팅 시험, gpu_guard 대기 시간 제한 (N4 의 다른 길), N7 (출처 기록) 은 손대지 않음.

## 6. 조사 — 220612 메시의 z 가 약 1000 m 인 이유 (보고만, 동작 안 바꿈)

**결론: 일부러 넣은 +1000 m 이동이다 (`scanA/lensonly_gt.py`). 숫자 (지표) 에는 영향 없음, 보기 (표시) 에만 보인다.**

근거 (저장된 출력만 읽음, 1회):
- `scanA/lensonly_gt.py` (GT 폴더 없는 녹화의 lens-only 대역) 가 `p_bc = VIO 자세 위치 + [0, 0, 1000.0]` 으로 쓴다. 머리말 이유:
  "scan_offline 정렬이 순수 평행 이동 (축척 1) 이 되고, 가짜 보드 평면 (z=0) 이 시야에 절대 안 들어오게 (보드 채점 없음)".
- `scan_offline.py:126-158`: `--vio` (여기서는 BA 자세) 를 GT 자세에 강체 (SE3, 축척 고정) 정렬 → BA 자세가 그 +1000 m 세계로 옮겨짐.
  `mesh/summary.json` poses: `se3_rmse_cm 3.95` (= BA 대 VIO 중심 rms, kfba 기록과 같음), `sim3_scale_would_be 1.0000`, `free_scale_applied false`.
- 값 대조: BA 자세 `fix_u3/scan/ba/poses_ba.jsonl.gz` z = 0.06–0.88 m → TSDF 에 들어간 자세 `fix_u4ba/scan/mesh/kf_poses.npz` z = 1000.06–1000.85 m.
  메시 정점 z = 999.48–1001.21 m (x −1.87…1.53, y −0.56…3.12). 세계 = VIO 세계 (중력 z 위) 그대로, z 만 +1000.
- GT 녹화 (133820 등) 는 진짜 GT (보드 기준, z≈0) 를 써서 해당 없음.

지표에 미치는 영향:
- lens-only 라 보드 지표 (|dZ|, k, 보드 RMS) 는 원래 없음 (`k_pct 1e-13` = 축척 1). `mesh_cam_depth_m`, 정점 수, TSDF 블록 수는 상대 거리라 그대로.
- TSDF 격자: 1000 m = 블록 (4 cm) 25 000 개 = 복셀 (5 mm) 200 000 개로 딱 나눠져 격자 위치가 안 바뀜.
- 유일한 실제 차이 = float32 정밀도: TSDF 적분 (`tsdf_map.py:268` 복셀 중심, 카메라 위치) 과 메시 정점이 float32 라 1000 m 에서 한 칸 6.1e-5 m (0.06 mm).
  저장된 메시 z 값의 가장 작은 간격도 정확히 6.1035e-5 m (확인). 복셀 5 mm 의 약 1 % → 메시 모양에는 사실상 영향 없음 (추정, 0 근처로 다시 돌려 비교는 안 함).
- 표시: 뷰어 기본 보드 윤곽 (z=0) 이 1000 m 아래라 안 보임 (그래서 `board_mm [0,0]`). 뷰어가 원점 기준으로 카메라를 두면 처음 화면이 비어 보일 수 있음 (추정).
- 고칠 방향 (안 함): 메시 저장 전에 −1000 m 되돌리기, 또는 lens-only 대역의 보드 평면을 다른 방법으로 끄기. 지금은 표시만의 문제라 그대로 둠.

## 7. 단위 7 — TSDF 블록 한도 (220612, 노트북 1 회, 미검증). 기본값은 안 바꿈

### 한도는 어디서, 메모리는 얼마나
- 한도: `scan3d/tsdf_map.py:49` `DEFAULTS max_blocks=60000`. `scan_offline.py` 는 이 값을 안 넘겨서 늘 60 000 이었다.
  → `scan_offline.py --tsdf-max-blocks N` 추가 (기본 60000 = 그대로, 커밋 bba07d9). run_scanner 에서는 `--mesh-args '--tsdf-max-blocks N'`.
- 블록 하나 = 8³ 복셀: sdf f32 2048 B + w f32 2048 B + rgb 1536 B + nobs 512 B + 키·판 32 B (+ 정렬 색인 16 B) ≈ **6.0 KiB (GPU)**.
- 저장소는 꽉 차면 **두 배로** 늘어난다 (`tsdf_map.py:_grow`, 1024 → 2048 → … → 65 536 → 131 072 → 262 144). 늘리는 순간 옛 것 + 새 것이 같이 있음.

| 한도 | 할당 칸 (2 배씩) | TSDF GPU | 늘리는 순간 최대 |
|---|---|---|---|
| 60 000 (지금) | 65 536 | 0.40 GB | 0.61 GB |
| 131 072 이하 | 131 072 | 0.81 GB | 1.22 GB |
| 250 000 (이번 시험) | 262 144 | 1.62 GB | 2.43 GB |
| 500 000 | 524 288 | 3.24 GB | 4.86 GB (+ FFS 약 1.3 GB → 8 GB 노트북 GPU 에 빠듯, 추정) |

노트북: RTX 4060 Laptop 8188 MiB, **WSL 이 보는 RAM 7 GB** (`free -g`). RAM 쪽이 더 좁을 수 있다 (아래 실측 4.46 GB).

### 시험: 한도만 250 000 으로 (fix_u4ba 와 같은 조건)
`remote_laptop.sh job 220612 'cmd:... run_scanner.sh $R $WT/$O/scan --reuse <fix_u3/scan> --depth-sigma ba --mesh-args "--tsdf-max-blocks 250000"'`
(`/usr/bin/time` 으로 RAM, `nvidia-smi -l 2` 로 GPU 를 재려고 `cmd:` 단계로 돌림; 코드 bba07d9, dirty 아님. `JOB_TIMEOUT_S=3600`).
같은 285 키프레임·같은 BA 자세 (`fix_u3/scan/ba`)·같은 렌즈 대역 → fix_u4ba 와 다른 것은 한도 하나뿐.

| 항목 | fix_u4ba (한도 60 000) | fix_u7cap250k (한도 250 000) |
|---|---|---|
| rc / status | 0 / ok | 0 / ok |
| TSDF 블록 | 60 000 (**걸림**, kf 89–109 쯤) | **138 632 (안 걸림)** = 60 000 의 2.3 배 |
| 메시 정점 / 면 | 880 571 / 1 546 274 | **1 394 583 / 2 418 089** |
| 정점 카메라 깊이 중앙 / p99 / 최대 | 1.22 / 1.51 / 1.59 m | 1.17 / 1.51 / 1.62 m |
| 키프레임 시야 덮임 중앙 (정점이 찍히는 화면 비율, 가림 무시) | 0.35 | **0.59** |
| 같은 값, 키프레임 순서 4 등분 평균 | 0.59 / 0.37 / 0.34 / 0.26 | 0.61 / 0.58 / 0.55 / 0.53 |
| 덮임 < 0.1 인 키프레임 | 6.1 % | 0 % |
| 위에서 본 넓이 (5 cm 칸) | 1.85 m² | **5.48 m²** |
| mesh 단계 시간 | 98.5 s | 111.7 s (+13 s) |
| GPU torch 최대 할당 / 예약 | 1.72 / 2.33 GB | 2.41 / 3.58 GB |
| GPU 전체 사용 최대 (nvidia-smi, 2 s 간격) | 안 잼 | 3 795 MiB / 8 188 |
| RAM 최대 (python maxrss) | 안 잼 | 4.46 GB (WSL 7 GB 중) |

- 덮임·넓이 계산: WSL `~/scanner/fixes_tmp/cover.py` (데스크톱에서 수 초, 저장된 메시만 읽음). 가림 검사 없음 → 절댓값보다 두 실행 비교용.
- 그림 `u7_220612_cap250k_kfviews.png` (u4 와 같은 kf 20 / 60 / 150 / 240; 왼쪽 사진, 오른쪽 메시 점), `u7_220612_cap250k_top.png` (위에서, 3.16 × 3.62 m).
  눈 확인: u4 에서 비어 있던 **침대·옷·선풍기·책상·바닥이 나옴** (kf 150·240). kf 60 운동 랙 앞 벽의 줄무늬 번짐은 그대로 (한도와 무관, 깊이 모델 쪽 문제).
  위에서 보면 방 윤곽 (벽 네 면 일부) 이 보이나 모서리 쪽 벽은 군데군데 비어 있음.
- 보기: `changes_0620/fixes_20261112/scanner_220612_sigma_ba_cap250k.ply` (52 MB), 목록 백업 `meshes.json.bak_fixes_u7_20261112_111232`.
- 결과: `scan3d/runs/laptop/20261106_220612_std60/fix_u7cap250k/` (mesh.ply·obj 는 커밋 안 함). provenance `main_commit` 은 `cmd:` 단계라 비어 있음 (`-dirty` 만 찍힘, N7 과 같은 계열).

### 고를 수 있는 길 (숫자는 위 표 + 계산, 결정은 메인 세션)
1. **한도만 올림 (250 000)**: 코드 변경 없음 (플래그). 이 방 (3.2 × 3.6 m, 51 s) 에 1.62 GB GPU·4.5 GB RAM·+13 s. 이 방의 1.9 배 블록 (262 144) 까지는 같은 메모리.
   그 이상은 할당이 524 288 칸 (3.2 GB, 늘리는 순간 4.9 GB) 으로 뛰어 8 GB GPU 에 빠듯 (추정). 가장 싸고 바로 쓸 수 있음.
2. **먼 곳만 1 cm 복셀 (두 단계)**: 블록 한 변 4 → 8 cm. 표면 넓이당 블록 수 1/4, 잘림 띠 (±μ, μ 최대 10 cm) 두께도 블록 5 → 2.5 개 → 먼 표면 블록 약 1/8 (추정).
   이 방의 먼 정점이 대부분 (>1.0 m 가 72 %) 이라 전체로 약 1/4–1/6 (추정). 대신 TSDF 두 개 (가까움 5 mm, 멀리 1 cm) 와 메시 잇기 코드가 필요 (큰 작업). 먼 쪽 깊이 σ 가 수 cm 라 1 cm 로 잃는 디테일은 적음 (추정).
3. **먼 곳 잘림 폭 줄이기** (`trunc_max` 10 → 5 cm): 먼 표면 띠 두께 절반 → 먼 블록 약 절반 (추정). 파라미터 하나지만 tsdf_map 기본값 근거 (10-17: 6 cm 에서 두께 나빠짐) 와 부딪힘 → 따로 시험 필요.
4. **블록 내보내기 (evict)**: 오래 안 본 블록을 CPU RAM 으로. WSL RAM 이 7 GB 이고 이번 실행이 이미 4.5 GB 라 여유 적음. 실시간 (폰 스캐너) 에는 필요하겠지만 오프라인 한 방에는 1 이 더 쌈.
5. (덤) 빈 블록 지우기: 무게가 끝까지 w_min 에 못 미친 블록을 주기적으로 지움. 몇 % 인지는 이번 출력으로는 모름 (안 잼).
- 권고: 오프라인 스캐너는 1 (`--tsdf-max-blocks 250000`) 을 기본값 후보로, 더 큰 공간이 필요해지면 2. 녹화 하나·실행 1 회라 미검증.

## 8. 단위 8 — 새 녹화 121801 (사무실 둘러보기, 60 s) 첫 스캔 (노트북, 1 회, 미검증)

### 녹화
- `hs_20261112_121801_std60` (폰 S24, 60 fps, 0.5 dpt, 3466 프레임, complete true, 60 s 안내: 5 s 멈춤 · 50 s 천천히 걷기 · 5 s 멈춤, 보드 없음·빨간 화면 없음).
- 폰에서 `adb pull` 만 → `C:\Users\a3162\scan3d_handoff\browser_upload\in\hs_20261112_121801_std60` (6 파일 크기 폰과 같음, cam5.mp4 1 150 329 822 B)
  → WSL `~/scanner/StereoDepthSeg/scan3d/recordings/hs_20261112_121801_std60` (`cmp` 6 개 같음, OpenCV 로 3466 프레임·끝 프레임 읽힘). 녹화는 커밋 안 함.

### 실행
`remote_laptop.sh job 121801 scan --tag fix_u8 --args '--depth-sigma ba --mesh-args "--tsdf-max-blocks 250000"'` (코드 d1dfefe, dirty 아님, 렌즈만 대역, 재사용 없음).

| 항목 | 값 |
|---|---|
| rc / status | **0 / ok** (job 368 s, 스캐너 342.7 s) |
| 단계 시간 vio / reduce / kfba / mesh | 45.6 / 10.3 / 99.5 / 187.1 s (FFS 102.7 s) |
| 키프레임 보냄 / 깊이 지도 있음 | 494 / 351 (짝 없음 143) |
| BA | 재투영 중앙 0.29 px, rmse 0.70 px, 1 px 안 91.7 %, VIO 자세로 남은 키프레임 0, BA-VIO 중심 rms **7.98 cm** (220612 는 3.95), GPU 최대 2.18 GB |
| 메시 정점 / 면 | **1 905 456 / 3 222 587** (ply 68 MB) |
| 정점 카메라 깊이 중앙 / p90 / p99 / 최대 | 1.04 / 1.29 / 1.44 / 1.59 m (0.65 m 넘음 92 %, 1.0 m 넘음 59 %) |
| TSDF 블록 | 154 556 / 한도 250 000 → **안 걸림** (`tsdf_full false`, tsdf_log 비어 있음) |
| 키프레임 시야 덮임 중앙 (cover.py, 가림 무시) | 0.48 (4 등분 0.50 / 0.50 / 0.41 / 0.36), 0.1 미만 4 % |
| 위에서 본 넓이 (5 cm 칸) | 11.8 m² (범위 약 8.7 × 6.8 m) |
| 품질 문턱 경고 | 없음 (정점 ≥ 1000, VIO 실패 없음, 블록 한도 안 걸림) |

- 그림 `u8_121801_kfviews.jpg` (kf 36 / 175 / 314 / 437; 왼쪽 사진, 오른쪽 같은 카메라에서 본 메시 점), `u8_121801_top.png` (위에서, 빨간 점 = 키프레임 카메라).
- 눈 확인: 공구판 (초록 타공판·펜치·드라이버), 선반·상자, 노란/파란 부품 상자, 회색 케이스, 시계·벽은 잘 알아봄.
  1.6 m 보다 먼 곳 (방 안쪽 책상줄·의자·문) 은 비거나 조각뿐이고, 검은 기계·바닥 일부는 번짐. 위에서 보면 카메라 길이 두 줄로 나란히 보임 (되돌아 걸었는지 VIO 드리프트인지 판정 못 함, 루프 없음).
- 보기: `changes_0620/fixes_20261112/scanner_121801_office_ba_cap250k.ply`, 목록 백업 `meshes.json.bak_fixes_u8_20261112_123013`.
- 결과: `scan3d/runs/laptop/20261112_121801_std60/fix_u8/` (mesh.ply·obj·키프레임 사진은 커밋 안 함).

## 9. 단위 9 — 121801 + BA 루프 짝 (`--loop salad`, 노트북 1 회, 미검증). 기본값은 안 바꿈

### 무엇을 바꿨나
- fix_u8 이 만든 키프레임 폴더 (`fix_u8/scan/red/hs_20261112_121801_std60_kf`: 같은 VIO 자세, 같은 494 키프레임) 를 REC 로 그대로 넣어
  vio·reduce 를 건너뛰고 **kfba 의 짝만** 바꿈: `--kfba-args "--loop salad"` (featloop 11-05 의 SALAD 전역 기술자로 키프레임마다 닮은 5 장을 더 짝지음).
  나머지 플래그는 u8 과 같음 (`--depth-sigma ba --mesh-args "--tsdf-max-blocks 250000"`, 시드 0, CPU SIFT, GPU BA).
- 명령: `remote_laptop.sh job 121801 'cmd:... run_scanner.sh <fix_u8 red 폴더> $WT/$O/scan --depth-sigma ba --mesh-args "--tsdf-max-blocks 250000" --kfba-args "--loop salad"' --tag fix_u9salad`
  (SALAD 무게 `dino_salad.ckpt`, `dinov2_vitb14_pretrain.pth`, `facebookresearch_dinov2_main`, `~/scanner/featloop_ext/salad` 를 노트북에 `--send`; 노트북에 처음). 코드 a616b26 (dirty 아님).

| 항목 | fix_u8 (루프 없음) | fix_u9salad (루프 SALAD) |
|---|---|---|
| rc / status | 0 / ok | 0 / ok |
| 짝 (기본 / 검증) | 5 461 / 5 419 | 같음 5 461 / 5 419 |
| 루프 짝 (제안 / 검증 ≥ 15 인라이어) | 0 | **1 881 / 1 593** (인라이어 중앙 272; VIO 거리 중앙 0.74 m, 각도 중앙 11°; 91.6 % 가 기존 공시 짝 조건 밖) |
| 루프 짝으로 이어진 3D 점 / 관측 | — | 22 139 / 289 658 |
| BA 재투영 중앙 / rmse / 1 px 안 | 0.292 / 0.702 px / 91.7 % | 0.294 / 0.709 px / 91.6 % |
| BA-VIO 중심 rms | 7.98 cm | **7.84 cm** |
| BA 자세 차 (u9 − u8, 같은 키프레임 494 개) | — | 중앙 0.24 cm, p90 0.53 cm, 최대 0.72 cm |
| 메시 정점 / TSDF 블록 | 1 905 456 / 154 556 | 1 903 622 / 152 750 |
| 정점 카메라 깊이 중앙 / p99 / 최대 | 1.04 / 1.44 / 1.59 m | 1.05 / 1.45 / 1.59 m |
| kfba / mesh 시간 | 99.5 / 187.1 s | 113.2 (SALAD 14.3 s 포함) / 184.5 s |

- 그림: `u9_121801_salad_kfviews.jpg` (kf 175 / 437, 왼쪽 사진·오른쪽 메시 점), `u9_121801_salad_kfviews_top.png` (위에서, 빨간 점 = 키프레임 카메라).
- **두 줄 길은 그대로**: 아래쪽 복도에서 카메라 길이 약 0.5 m 떨어진 두 줄 (가는 길·오는 길). 키프레임마다 "100 개 넘게 떨어진 키프레임 중 가장 가까운 것" 의 수평 거리 중앙 49.2 → 49.0 cm.
  루프 짝 1 593 개가 검증되고 2.2 만 점이 두 길을 이어 주는데도 BA 가 1 cm 도 안 움직였으므로, 드리프트가 아니라 **실제로 다른 줄로 걸어 돌아온 것** 으로 추정
  (메시의 선반·공구판도 두 겹이 아님, 눈 확인). 단, 삼각측량이 VIO 자세로 하고 4 px 넘는 관측을 버리므로 큰 드리프트였다면 루프 점이 덜 남았을 수 있음 (미검사).
- 반복성: featloop (11-05) 에서 SALAD 루프가 실행마다 달랐던 원인은 RANSAC 시드 (-1) 였음. 여기서는 시드 0 이지만 GPU 매처라 같은 결과가 또 나온다는 보장 없음 (한 번만 돌림).
- 정리: 이 녹화에서는 루프 짝이 결과를 거의 안 바꿈 (+14 s). 기본값 그대로 (`--loop none`).
- 보기: `changes_0620/fixes_20261112/scanner_121801_office_salad_loop.ply`, 목록 백업 `meshes.json.bak_fixes_u9_20261112_125424`.
- 결과: `scan3d/runs/laptop/20261112_121801_std60/fix_u9salad/` (mesh.ply·obj·salad_sim.npy 는 커밋 안 함).
