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
| M6 | `run_scanner.sh` | `--reuse` 와 무시되는 플래그를 같이 주면 경고 |
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
