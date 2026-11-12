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

## 10. 단위 10 — 이어 찍기: 녹화 둘을 메시 하나로 (`scanA/run_merge.sh`, 121801 을 둘로 나눠 시험, 노트북 1 회, 미검증)

### 만든 것 (사용법·찍는 법은 `scanA/README.md` 9 절)
- `scanA/run_merge.sh OUT REC_A REC_B [...]`: 녹화마다 `run_scanner.sh --stop-after reduce` (새 옵션, 기본 동작 그대로) → `scanA/merge_align.py` → 합친 키프레임 폴더에 `run_scanner.sh`.
- `merge_align.py`: 부분 표본 (녹화마다 키프레임 80 장) A×B SIFT 짝 6400 개 + A 안 이웃 짝 → A 의 VIO 자세로 삼각측량 → B 키프레임 PnP (OpenCV RANSAC 3 px, 시드 0) →
  4 자유도 (방향각 + 이동) 다수결 (10 cm / 3° 안), 동의 < 5 이면 `MERGE FAILED` 종료 코드 6. B 자세 = M·VIO, 프레임 번호 +A 프레임 수, 시각 = A 끝 + 100 s (FFS 짝은 30 s 안이라 녹화 사이 깊이 짝 없음).
- `kf_ba.py --extra-pairs FILE` (새 옵션, 기본 없음 = 그대로): merge_align 이 검증한 녹화 사이 짝을 매칭 목록에 더함 (pair_stats 의 loop 칸에 셈).

### 시험: 121801 을 A = 0–35 s, B = 25–60 s 로 (10 s 겹침), 각자 VIO
- B 는 걷는 중에 시작. IMU 를 1 s 창으로 보면 처음 2 s 뒤로는 멈춘 곳이 없음 (가속도 크기 표준편차 0.2–1.2 m/s², 자이로 평균 ≥ 0.19 rad/s) →
  정지 초기화 불가 → B 만 OpenVINS **동적 초기화** (`--vio-args-of 2 '--start-off 25 --yaml init_dyn_use=true'`). A 는 정지 초기화 그대로.
- 명령: `remote_laptop.sh job 121801 'cmd:... run_merge.sh $WT/$O/merge $R $R --vio-args-of 1 "--max-secs 35" --vio-args-of 2 "--start-off 25 --yaml init_dyn_use=true" --depth-sigma ba --mesh-args "--tsdf-max-blocks 250000"' --tag fix_u10split`
  (코드 a616b26 + 이 단위의 커밋 전 변경 = dirty).

| 항목 | 값 |
|---|---|
| VIO A / B | 2100 / 1966 자세, 초기화 4.13 s (정지) / 2.7 s (동적), 시간 28.6 / 47.2 s. (lp_score 의 "diverged" 는 시작·끝 거리 기준이라 조각에는 의미 없음) |
| 키프레임 A / B | 272 / 280 (합 552; 한 번에 찍은 u8 은 494) |
| 녹화 사이 짝 (A 80 × B 80) | 6400 중 552 검증 (≥ 15 인라이어) |
| B 쪽에서 보인 A 의 삼각측량 점 | 5 149 |
| PnP 성공 / 한 변환에 동의 | 29 / 25 키프레임 |
| 4 자유도 변환 | 방향각 95.15°, 이동 (3.63, −4.74, 0.70) m |
| 동의한 키프레임 중심 오차 중앙 / 최대 | 3.0 cm / 9.3 cm, 회전 오차 중앙 0.74° |
| 검사: 남은 기울기 중앙 / Sim(3) 축척비 | 0.51° / **1.026** (중심 퍼짐 0.75 m; 동적 초기화 축척 2.6 % 차이 추정) |
| kf_ba 에 더한 녹화 사이 짝 | 287 중 227 새 짝 (나머지는 이미 공시 짝), 227 모두 검증, 인라이어 중앙 131; 같은 영상 프레임끼리 짝 3 개 |
| 합친 BA | 0.291 px (rmse 0.697, 1 px 안 91.8 %), BA-VIO rms 6.19 cm, Sim3 축척 1.0033, **관측 < 15 라 VIO 자세로 남은 키프레임 12 개** (A 끝 33–34 s 4 개, B 끝쪽 8 개) |
| **ATE: 합친 BA vs 한 번에 찍은 u8 BA** (키프레임 중심 552 개, 487 개는 같은 프레임, 나머지 u8 키프레임 사이 보간) | SE3 맞춤 **10.1 cm rms** (중앙 7.6, 최대 22.5; A 7.5 / B 12.1), Sim3 9.8 cm (축척 0.991), 맞춤 없이 A 3.7 / B 15.2 cm |
| 같은 비교, BA 전 (합친 VIO 입력) | SE3 14.2 cm rms (A 10.9 / B 16.7) → BA 가 4 cm 줄임 |
| 메시 정점 / TSDF 블록 | **1 918 406** / 163 719 (u8 1 905 456 / 154 556) |
| 정점 카메라 깊이 중앙 / p99 / 최대 | 1.06 / 1.49 / 1.71 m |
| 시간 | part1 35.4 + part2 56.3 + align 49.6 + kfba 104.6 + mesh 187.9 s ≈ 434 s (u8 한 번에 343 s) |

- 참고 자 (u8) 도 정답이 아님: u8 BA 자체가 u8 VIO 와 7.98 cm rms 다름. 그래서 10 cm 는 "한 번에 찍은 것과 비슷한 정도" 로만 읽는다.
- **mesh 첫 실행 실패**: GPU 메모리 부족 (rc 3, 552 개 중 약 479 번째 FFS 에서 1.15 GiB 할당 실패, 상한 6.0 GB). u8·u9 도 예약 최대 5.95 / 5.93 GB 로 상한에 붙어 있었음 →
  합치기 탓이라기보다 원래 빠듯했던 것으로 추정. 같은 BA 자세로 `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` + `--reuse` 로 mesh 만 다시 → 성공 (`fix_u10split_mesh2`, 할당 4.69 / 예약 4.95 GB).
  run_merge·run_scanner 에는 이 변수를 아직 안 넣음 (메인 세션 결정).
- 그림: `u10_121801_merged_kfviews.jpg` (kf 175 = A 쪽, kf 437 = B 쪽 프레임 6254 = 원래 2788), `u10_121801_merged_kfviews_top.png`. 눈 확인: 위에서 본 모습이 u8 과 거의 같고,
  B 쪽 사진 (3D 프린터) 과 메시가 겹침. 두 줄 길도 u8 과 같은 모양.
- 쉬운 경우라는 점: 두 조각이 같은 영상에서 나와 겹침 10 s 의 사진이 거의 같다 (실제 두 번째 녹화보다 쉬움). 다만 kf_ba 에 들어간 녹화 사이 짝 중 같은 프레임끼리는 3 개뿐.

### 거부 시험: 안 겹치는 녹화 (121801 앞 35 s + 220612 다른 방)
- `run_merge.sh ... $R <220612> --vio-args-of 1 "--max-secs 35"` (`--tag fix_u10refuse`) → 녹화 사이 짝 6400 중 검증 0, 삼각측량 점 0, PnP 0 →
  `MERGE FAILED: part 1 (hs_20261106_220612_std60_kf): only 0 keyframes could be placed ... the recordings do not overlap`, **종료 코드 6**, scan 단계 안 돎. 시간 34 + 45 + 22 s.
- 노트북에 보낸 220612 는 작업 끝에 지움.

- 보기: `changes_0620/fixes_20261112/scanner_121801_merged_2parts.ply`, 목록 백업 `meshes.json.bak_fixes_u10_20261112_130731`.
- 결과: `scan3d/runs/laptop/20261112_121801_std60/fix_u10split/` (merge, BA), `fix_u10split_mesh2/` (메시), `fix_u10refuse/`, ATE `fix_u10split/ate_vs_fix_u8.json`
  (계산 스크립트 `runs/fixes_20261112/ate_merge.py`). 메시·키프레임 사진은 커밋 안 함.

## 11. 단위 11 — 실제 이어 찍기: 121801 + 새 녹화 131328 (`run_merge.sh`, 노트북 1 회, 미검증)

### 녹화
- `hs_20261112_131328_std60` (S24, 60 fps, 0.5 dpt, 3437 프레임, complete true, 60 s; 121801 과 같은 설정). 안내: 0–5 s 바닥에 두고 멈춤,
  5–15 s 121801 이 끝난 곳의 주황/노랑 부품 상자를 다시 비춤 (겹침), 15–55 s 새 곳, 55–60 s 멈춤.
- 폰에서 `adb pull` 만 → `C:\Users\a3162\scan3d_handoff\browser_upload\in\hs_20261112_131328_std60` (6 파일 크기 폰과 같음, cam5.mp4 1 140 920 348 B)
  → WSL `~/scanner/StereoDepthSeg/scan3d/recordings/hs_20261112_131328_std60` (`cmp` 6 개 같음, OpenCV 3437 프레임·끝 프레임 읽힘). 녹화는 커밋 안 함.

### 실행
`remote_laptop.sh job 121801 'cmd:SCANNER_COMMIT=7acaca7 bash scan3d/scanA/run_merge.sh $WT/$O/merge $R <131328> --depth-sigma ba --mesh-args "--tsdf-max-blocks 250000" ...' --tag fix_u11merge --send <131328>`
(코드 7acaca7, dirty 아님; run_merge 가 `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` 기본). 두 녹화 모두 **정지 초기화** (동적 초기화 안 씀), 121801 도 처음부터 다시 VIO (run_merge 는 u8 결과 재사용 기능 없음).
노트북에 보낸 131328 은 작업 끝에 지움.

| 항목 | 값 |
|---|---|
| rc / status | **0 / ok** (job 835 s: part1 55 + part2 55 + align 37 + scan 637 s (kfba 192, mesh 445)) |
| VIO A / B | 둘 다 정지 초기화 4.13 / 4.22 s, 3466 / 3437 자세 (B 의 lp_score "diverged" 는 시작·끝 거리 3.2 m 기준이라 의미 없음) |
| 키프레임 A / B | 494 / 498 (합 992) |
| 녹화 사이 짝 (A 80 × B 80) | 6400 중 **337 검증** (≥ 15 인라이어) |
| B 쪽에서 보인 A 의 삼각측량 점 | 2 774 |
| PnP 성공 / 한 변환에 동의 | 22 / **17** 키프레임 |
| 4 자유도 변환 | 방향각 −4.21°, 이동 (−0.010, −0.031, −0.016) m — 거의 그대로. 121801 이 시작한 책상 자리로 돌아와 끝났고 (시작·끝 16 cm), 131328 도 그 자리에서 시작해서 두 VIO 원점이 거의 같은 곳으로 추정 |
| 동의한 키프레임 중심 오차 중앙 / 회전 오차 중앙 | **3.6 cm** / 0.84° |
| 검사: 남은 기울기 중앙 / Sim(3) 축척비 | 0.50° / 1.020 (중심 퍼짐 1.66 m) |
| kf_ba 녹화 사이 짝 (extra) | 92 개 모두 검증, 인라이어 중앙 73, 이어진 점 172 (관측 1 761) |
| 합친 BA | 0.312 px (rmse 0.724, 1 px 안 91.2 %), BA-VIO rms 4.86 cm, VIO 자세로 남은 키프레임 1 |
| 메시 정점 | **3 791 137** (u8 한 녹화 1 905 456) |
| TSDF 블록 | **250 000 / 한도 250 000 → 걸림** (`tsdf_full true`): 일부가 빠졌을 수 있음 |
| 깊이 있는 키프레임 | 804 / 992 (짝 없음 188) |
| 정점 카메라 깊이 중앙 / p90 / p99 / 최대 | 0.96 / 1.26 / 1.45 / 1.67 m |
| 범위 (위에서) | 약 10.4 × 7.4 m |

- 그림 (`render_merge.py`, 창 없이 numpy): `u11_merge_121801_131328_top.png` (왼쪽 위에서 본 색, 오른쪽 바닥 위 0.4–1.2 m 단면; 빨강 = 121801 카메라, 파랑 = 131328),
  `u11_merge_121801_131328_overlap_zoom.png` (겹친 곳 확대 5 mm 칸), `u11_merge_121801_131328_kfviews.jpg` (131328 의 10 s·14 s 키프레임과 가장 비슷한 121801 키프레임; 왼쪽 사진, 오른쪽 같은 카메라에서 본 메시 점).
  메시는 VIO 세계 + z 1000 m (lensonly 대역 GT) 라 그림에서 1000 을 뺌.
- 눈 확인: 주황/노랑 부품 상자, 의자, 캐비닛, 선반의 테두리가 **한 겹** (위에서 본 단면에 두 겹 벽 안 보임), 131328 사진의 상자와 메시가 같은 자리에 겹침.
  카메라 사진 위 메시 그림의 방사형 줄무늬는 그림 방식 (z 버퍼 없이 점 찍기) 탓으로 추정.
- 할 일 후보: 두 녹화를 합치면 블록 한도 250 000 에 걸림 → 더 큰 한도 또는 먼 곳 1 cm 복셀 (7 절) 이 필요할 수 있음 (메인 세션 결정).
- 보기: `changes_0620/fixes_20261112/scanner_121801_131328_merged.ply`, 목록 백업 `meshes.json.bak_fixes_u11_20261112_133441`.
- 결과: `scan3d/runs/laptop/20261112_121801_std60/fix_u11merge/` (mesh·키프레임 사진·imu·db 는 커밋 안 함). 중간 보고 `review_20261112/interim_u11.txt`.

## 12. 단위 12 — TSDF 블록을 디스크로 내보내기 (`--tsdf-evict-gpu-blocks N`, **데스크톱 RTX 5080** 1 회, 미검증). 기본값은 안 바꿈

**주의: 이 절의 숫자는 데스크톱 (RTX 5080 16 GB, WSL RAM 30 GB) 에서 잰 것.** 노트북 (4060 8 GB, RAM 7 GB) 이 TensorRT 작업 중이라
사용자 허락으로 데스크톱에서 돌렸다. 시간·메모리는 앞 절 (노트북) 숫자와 비교하면 안 된다. 켬/끔 비교는 둘 다 데스크톱.

### 무엇
- 새 파일 `scan3d/tsdf_evict.py` (`EvictTSDFMap`, `TSDFMap` 을 물려받음). GPU 에는 최대 N 블록만 둔다.
  키프레임 하나가 GPU 칸을 더 원하면, **가장 오래 안 바뀐 블록** (블록 판 번호 bver 가 작은 것, 이번 키프레임이 쓰는 블록은 제외) 을
  디스크 폴더 (numpy memmap, 기본 `OUT/tsdf_spill/`) 로 보낸다 (모자란 만큼 + N 의 25 % 를 한꺼번에 → 매 키프레임 일어나지 않게).
  다시 보이면 같은 값 그대로 GPU 로 돌려와서 평소처럼 섞는다 (중복 블록 없음). `--tsdf-max-blocks` 는 이제 **GPU + 디스크 전체** 블록 한도.
- 메시: 끝에 모든 블록을 디스크로 보낸 뒤, 할당 순서대로 64 × 512 블록씩 작은 임시 GPU 지도 (그 블록 + 이웃 +{0,1}³ 블록) 에서 marching cubes.
  원래 `TSDFMap.update_mesh(force=True)` 와 같은 정육면체·같은 512 블록 묶음·같은 청크 순서 → **f32 면 메시가 비트 단위로 같게** 만듦. 블록 경계 이음매 없음.
- 디스크 쪽 페이지는 쓰고 읽은 뒤 `MADV_DONTNEED` 로 프로세스에서 뗌 (데이터는 파일/페이지 캐시에 남고 커널이 내보냄) → 프로세스 RAM 으로 안 셈.
- `--tsdf-evict-dtype f16`: 디스크에 sdf·w 를 float16 으로 (블록당 6.0 → 4.0 KiB, 값이 조금 바뀜). `--tsdf-evict-dir`, `--tsdf-evict-keep` (안 주면 성공 후 폴더 지움).
- summary.json 에 `tsdf_evict` (내보냄·되불러옴 횟수, 디스크 크기 `spill_gb`, 쓰기·읽기 시간 `store_write_s`/`store_read_s`, 최대 GPU 블록 수 …), `tsdf_blocks` = 전체.
- 끄면 (기본 0) 옛 코드 그대로. 조용한 대체 없음: 키프레임 하나가 N 보다 많은 블록을 원하면 RuntimeError.
  오프라인 경로만 지원 (실시간 증분 메시·merge·save_npz·ESDF 는 NotImplementedError). 실시간 폰 스캐너 (`voxel_map.py`) 에는 아직 안 붙임.
- 진단: `SCAN3D_TSDF_RECORD=DIR` (tsdf_map.integrate 입력을 손실 없이 저장) + `scan3d/tools/tsdf_evict_check.py DIR --gpu-blocks N --spill D`
  (같은 입력을 끔/켬 두 지도에 넣고 메시를 비교, FFS 없이 수십 초).

### 결과 (데스크톱 5080, 각 1 회)
같은 맞춤: `scan_offline.py --gt lensonly --compress --kf-step 1 --tsdf-alloc dense --ffs-views 2 --ffs-partners all --sigma-rot-deg 0.1 --sigma-b-rel 0.02`
(run_scanner `--depth-sigma ba` 와 같은 mesh 인자), 자세 = 220612 는 `fix_u3/scan/ba`, 합친 것은 `fix_u11merge/merge/scan/ba` + 그 키프레임 폴더 (BA 다시 안 돌림, mesh 단계만).

| 실행 | 블록 (전체) | tsdf_full | 정점 / 면 | 메시 같음? | mesh 단계 | torch 최대 할당 | nvidia-smi 최대 | 최대 RSS | 디스크 |
|---|---|---|---|---|---|---|---|---|---|
| 220612 끔 (한도 250 000) `fix_u12_off` | 138 688 | 아니오 | 1 393 276 / 2 416 360 | 기준 | 81 s | 2.38 GB | 3 054 MiB | 5.14 GB | — |
| 220612 켬 GPU 40 000 `fix_u12_evict40k_b` | 138 688 | 아니오 | 같음 | **mesh.ply 바이트 같음** (cmp) | 70 s | 1.24 GB | 1 814 MiB | 5.54 GB | 0.80 GB |
| 합침 끔, 한도 2 000 000 `fix_u12merge_off_nocap` | **305 348** | **아니오** | 4 485 785 / 7 602 695 | 기준 | 227 s | 5.79 GB | 6 462 MiB | 9.28 GB | — |
| 합침 켬 GPU 120 000 `fix_u12merge_evict120k` | 305 348 | 아니오 | 같음 | **mesh.ply 바이트 같음** | 223 s | 4.47 GB | 5 384 MiB | 9.34 GB | 1.75 GB |
| (참고) 노트북 u11 한도 250 000 | 250 000 | **예** | 3 791 137 | — | 445 s (노트북) | 5.79 GB | — | — | — |

- 220612 켬: 내보냄 19 번 (블록 244 103 개), 되불러옴 105 415 개, 디스크 쓰기 4.8 s / 읽기 2.0 s, 끝 메시 2.8 s. (첫 켬 실행 `fix_u12_evict40k` 는 MADV 고치기 전: RSS 5.87 GB, 메시는 역시 같음.)
- 합침 켬: 내보냄 9 번 (344 226 개), 되불러옴 38 878 개 (= 131328 이 121801 끝 쪽을 다시 봄), 디스크 쓰기 9.2 s / 읽기 1.9 s, 끝 메시 8.9 s (10 묶음). 디스크 1.75 GB 는 성공 후 지워짐.
- **한도 없이 합친 메시는 정점 4.49 M = 한도 걸린 노트북 u11 (3.79 M) 의 1.18 배.** 위에서 본 그림 비교 `u12_merge_cap_vs_evict_top.jpg` (위 = u11 한도 걸림, 아래 = u12):
  131328 이 새로 간 곳 (파랑 경로 둘레, 가운데 왼쪽) 이 u11 에는 비어 있고 u12 에는 채워짐. 다른 GPU 라 FFS 깊이가 아주 조금 다를 수 있어 정점 수 차이 전부가 한도 탓은 아닐 수 있음 (추정).
  그림 `u12_merge_evict_nocap_top.png`, `_overlap_zoom.png`, `_kfviews.jpg` (render_merge.py, 창 없이).
- 재생 시험 (`tsdf_evict_check.py`, 합침 입력 804 키프레임을 녹화해 둔 것, TSDF 만): GPU 60 000 블록 f32 → **비트 같음**, 내보냄 20 번·되불러옴 51 400 개, torch 최대 1.18 GB (끔 2.81 GB), TSDF 시간 27.6 s (끔 13.3 s).
  같은 것 f16 → 정점 4 486 232 (+447), 정점 이동 p99 0.003 mm, 최대 7.4 / 18.5 mm (정점의 0.0005 %; w 반올림으로 메시 기준 w_min 근처 복셀이 켜지고 꺼짐), 디스크 1.17 GB.
  220612 입력 재생 (GPU 40 000) 도 비트 같음.
- RAM: 합친 실행의 RSS 9.3 GB 는 켬/끔이 거의 같다 → TSDF 가 아니라 파이프라인 나머지 (키프레임 사진 992 장 등, 추정). **노트북 WSL 7 GB 에서는 이 합침이 TSDF 와 상관없이 빠듯** (노트북 u11 은 RSS 를 안 쟀음).

### 권고 (숫자는 위 표, 결정은 메인 세션, 기본값은 안 바꿈)
- 오프라인 스캐너 (run_scanner / run_merge) 기본값 후보: `--mesh-args '--tsdf-evict-gpu-blocks 120000 --tsdf-max-blocks 1000000'` (f32).
  근거: 메시가 끔과 바이트 같음 (2 경우), mesh 단계 시간 같음 (223 vs 227 s, 70 vs 81 s), TSDF GPU 저장소가 0.69 GB 로 고정 (끔은 블록 수에 따라 2 배씩 커져 30 만 블록이면 3.2 GB, 늘리는 순간 4.9 GB 추정),
  디스크는 블록당 6 KiB (30 만 블록 1.75 GB, 끝나면 지움). 한도 250 000 에 걸려 잘리던 합침이 안 잘림.
- 노트북에서는 아직 안 돌려 봄 (4060, 디스크 속도 다름) → 기본값으로 바꾸기 전에 노트북에서 합침 1 회 (RSS 같이 잼) 권장.
- f16 은 디스크 1/3 덜 쓰지만 메시가 조금 바뀜 → 디스크가 모자랄 때만.

### 파일
- 코드: `scan3d/tsdf_evict.py` (새), `scan3d/tsdf_map.py` (`SCAN3D_TSDF_RECORD` 진단만), `scan3d/scanA/scan_offline.py` (플래그 4 개 + 생성·summary 몇 줄), `scan3d/tools/tsdf_evict_check.py` (새).
- 결과: `scan3d/runs/desktop/20261106_220612_std60/fix_u12_*`, `scan3d/runs/desktop/20261112_121801_std60/fix_u12*` (mesh.ply·obj 는 커밋 안 함).
  실행 스크립트·녹화한 입력 (220612 1.4 GB, 합침 4.7 GB): 데스크톱 WSL `~/scanner/fixes_tmp/evict/` (커밋 안 함).
- 보기: `changes_0620/fixes_20261112/scanner_121801_131328_merged_evict_nocap.ply`, 목록 백업 `meshes.json.bak_fixes_u12_*`. 중간 보고 `review_20261112/interim_evict.txt`.

## 13. 단위 13 — 깊이(FFS)를 TensorRT 로 (`scan_offline --ffs-backend trt`, **노트북 RTX 4060** 1 회, 미검증). 기본값은 안 바꿈
(결과 폴더 이름은 `fix_u11trt` — 만들 때 붙인 이름. 위 11 절 합침과는 다른 작업.)

### 먼저 잰 것 (fix_u8 로그, 노트북)
- mesh 단계 187 s 중 FFS 102.7 s (55 %), 짝 검사(SIFT dy) 15.1 s, 나머지 (디코드·TSDF·메시 저장) 약 42 s, 시작(모델 로드·import) 약 27 s.
  FFS 가 가장 커서 진행. 한 쌍 약 200 ms (960x540), torch.compile 재컴파일 때 1 초짜리가 가끔.

### 만든 것
- `scan3d/ffs_trt.py` (새): 상류 `scripts/make_single_onnx.py` 의 ONNX 용 코스트 볼륨 함수로 **고정 크기** ONNX (opset 17) → TensorRT fp16 엔진.
  내보내기는 `lib_ffs.load` 와 같은 키 채우기 (normalize=True, iters 8, max_disp 192). ImageNet 정규화는 그래프 밖 (fp32) 에서.
  내보내기와 엔진 만들기는 **다른 프로세스** (같은 프로세스면 torch 캐시가 8 GB 중 7.35 GB 를 잡아 빌더가 메모리 부족, 첫 시도).
- 크기: 엔진 하나 = 입력 크기 하나 → **버킷 8 개** (HxW): 544x960, 448x768, 352x576, 256x448 과 세로판 960x544, 768x448, 576x352, 448x256.
  축소 배율 s 계산·resize·되돌리기 (`/ (W/W0)`) 는 lib_ffs 와 똑같음. 다른 점은 **패딩만**: lib_ffs 는 32 배수까지, TRT 는 그 크기를 담는 가장 작은 버킷까지
  (replicate, 같은 좌우·위아래 나눔). 화소 격자는 그대로라 시차 배율 계산은 안 바뀜. s=1 (540x960 / 960x540) 은 lib_ffs 와 패딩이 같음 (4 줄).
- `scan_offline.py`: `--ffs-backend {pytorch,trt}` (기본 pytorch), `--ffs-engines DIR`, `--dump-depth DIR` (진단: 키프레임 깊이 uint16 0.1 mm).
  엔진 폴더·manifest 없음, TensorRT 버전·GPU 이름·체크포인트(크기 + 앞 64 MB sha)·iters·max_disp·내보내기 버전이 다름, 엔진 sha 다름, 버킷보다 큰 입력 → **RuntimeError (PyTorch 로 안 돌아감)**.
  summary 에 `ffs_backend`, `ffs_trt` (버킷별 호출 수, 실제 크기 → 버킷).
- TensorRT 는 venv_ffs 를 건드리지 않고 `pip --target /home/system/scanner/ffs_trt/site` 에 설치 (노트북·데스크톱 같은 경로), TRT 모드에서만 sys.path 뒤에 붙임.
  버전: tensorrt-cu12 **10.16.1.11**, onnx 1.19.1 (중복 numpy 지움), torch 2.11.0+cu128 (CUDA 12.8). 노트북 드라이버 592.82, 데스크톱 610.88.
- 엔진 (`/home/system/scanner/ffs_trt/engines`, 각 40–63 MB) 은 GPU·TRT 버전마다 따로 만들어야 하고 git 에 안 넣음 (C-FFS 가중치 파생물).
- 실행: `runs/fixes_20261112/trt_job.sh` (엔진 없으면 만들기 → PyTorch → TRT → `trt_compare.py`). 같은 키프레임 (fix_u8 `red/`), 같은 BA 자세, fix_u8 과 같은 mesh 명령.

### 결과 (노트북 RTX 4060, 121801, 1 회, 미검증)
| | PyTorch | TensorRT fp16 |
|---|---|---|
| FFS 한 쌍 중앙 / p90 (ms) | 201 / 212 (평균 291, 컴파일 포함) | **136 / 141** |
| FFS 합계 (s) | 102.3 | **46.3** |
| mesh 단계 (s, 프로세스 전체) | 191 | **130** (−61 s) |
| scan_offline 내부 runtime (s) | 164.6 | 104.6 |
| GPU 최대 nvidia-smi (MiB) | 6223 | 6597 (TRT 공유 작업 메모리 2237 MB + torch TSDF) |
| torch max alloc / reserved (GB) | 4.83 / 5.95 | 3.77 / 5.90 |
| 메시 정점 / 면 | 1 905 456 / 3 222 587 (= fix_u8 똑같음) | 1 915 924 / 3 237 041 (+0.55 %) |
| 깊이 있는 키프레임 | 351 | 351 (짝 351/351 같음) |
| 유효 깊이 화소 | 76 603 165 | 76 559 557 (−0.06 %) |

- 전체 시간: fix_u8 343 s 기준 약 282 s 로 줄 것 (**추정**, 전체 파이프라인은 다시 안 돌림).
- 엔진 만들기: 노트북 8 개 5052 s (큰 버킷 약 12 분, 작은 것 약 9 분), 한 번만. 데스크톱 RTX 5080 8 개 787 s (따로 만들어 둠).
- 깊이 차이 (두 쪽 다 유효한 화소, 키프레임별 |dZ|):

  | 키프레임 | 수 | 중앙 (키프레임 중앙값) | p95 (키프레임 중앙값) | p95 (나쁜 쪽 10 %) |
  |---|---|---|---|---|
  | 전체 | 351 | 0.5 mm | 1.9 mm | 25.5 mm |
  | s = 1 (패딩 lib_ffs 와 같음) | 311 | 0.5 mm | 1.8 mm | 8.9 mm |
  | s < 1 (버킷 패딩 더 넓음) | 40 | 1.45 mm | 28 mm | 314 mm |

  상대 |dZ|/Z 중앙 0.04 %. 0.5 mm 는 덤프 단위 0.1 mm 의 반올림 포함.
- s < 1 키프레임 (11 %) 차이가 큰 원인은 가리지 못함: 넓은 replicate 패딩 vs fp16 (s<1 은 큰 시차 짝이기도 함). 어느 쪽이 맞는지도 모름 (이 녹화는 렌즈만, 보드 GT 없음).

### 권고 (결정은 메인 세션, 기본값은 안 바꿈)
- 아직 기본값으로 하지 않음. 이득은 mesh 단계 −61 s (전체의 약 18 %, 추정) 지만 축소 짝 11 % 에서 깊이가 수 cm 달라지고 원인 미확인.
- 다음 시험 (싼 순서): ① s<1 짝만 lib_ffs 와 같은 32 배수 크기 엔진으로 (버킷 패딩 영향 분리), ② 보드 GT 있는 녹화 (예 165506) 에서 두 백엔드의 보드 |dZ| 비교, ③ 그 뒤 기본값 결정.
- **데스크톱 비교는 보류** (사용자: 데스크톱 GPU 는 MVS 비교 먼저). 데스크톱 엔진은 지시가 바뀌기 전에 이미 다 만들어져 있음 (`scan3d/runs/desktop/20261112_121801_std60/fix_u11trt/build.log`); 비교 실행은 데스크톱 WSL `~/scanner/trt_tmp/desktop_run.sh` (지시가 오면).

### 파일
- 코드: `scan3d/ffs_trt.py` (새), `scan3d/scanA/scan_offline.py` (플래그 3 개), `runs/fixes_20261112/trt_job.sh`, `runs/fixes_20261112/trt_compare.py`.
- 결과: `scan3d/runs/laptop/20261112_121801_std60/fix_u11trt/` (trt_compare.json, times.txt, mesh_pt|mesh_trt/summary.json·keyframes.json, gpu_*.csv, build.log, engine_manifest.json),
  데스크톱 엔진 빌드 기록 `scan3d/runs/desktop/20261112_121801_std60/fix_u11trt/`.
  메시 ply/obj·깊이 덤프는 노트북 `/home/system/scanner/ffs_trt/{mesh_keep,dump_pt,dump_trt}` (커밋 안 함).
- 중간 보고 `review_20261112/interim_trt.txt`.

### 13b. 축소 짝 40 키프레임의 원인 = 버킷 패딩 (노트북 RTX 4060, 121801, 1 회, 미검증)
- `scan_offline --kf-only u13b_kf_s_lt1.json` (s<1 짝 40 키프레임만), 진단 백엔드 `pytorch_bucket` (PyTorch fp16 + TRT 와 같은 버킷 패딩), `pytorch_fp32`.
- PyTorch 기본 (u13 전체 덤프) 대비 |dZ| 키프레임 p95 중앙 / 나쁜 10 % / p95>10 mm 키프레임:
  PyTorch 재실행 0.0 / 0.2 mm / 0 %, **PyTorch + 버킷 패딩 27.5 / 313 mm / 68 %**, TRT 버킷 28.1 mm / 68 %, PyTorch fp32 1.0 / 4.3 mm / 0 %.
  → 원인은 넓은 replicate 패딩, fp16 은 작음. 결과 `scan3d/runs/laptop/20261112_121801_std60/fix_u13b/` (`trt13b_job.sh`).

### 13c/13d. 엔진 NaN 고침 + 정확한 /32 크기 (데스크톱 RTX 5080, 1 회, 미검증)
- **13c 데스크톱 보드 TRT rc 5 원인**: 빈 메시 막기 (`mesh 0 vertices < 1000`). 데스크톱 v1 엔진 8 개가 전부 NaN → 모든 키프레임 valid_frac 0.
  노트북 v1 고정 엔진은 정상이었고, 노트북 동적 엔진은 NaN + CUDA 오류 720 (`__myl_Inor`, TensorRT InstanceNorm 커널).
- 찾은 방법: 전부 fp32 엔진은 정상 (합성 12 px 시차 → 12.00) → 정밀도 문제. `trt_nan_bisect.py` (중간 텐서를 출력으로 표시) 로 처음 NaN 층 = gwc 비용 볼륨
  `F.normalize` 뒤 `/Mul_1` (9.8 % 비정상). 원인: eps 1e-12 가 fp16 에서 0 → 오른쪽 특징을 밀며 0 으로 채운 열 (노름 0) 에서 0/0.
- **내보내기 v3** (`ffs_trt.EXPORT_VERSION = 3`, 옛 엔진은 버전 검사로 거부):
  gwc `F.normalize` eps 1e-3 (0 벡터는 그대로 0, 실제 특징 노름은 훨씬 커서 값 같음), 모든 ReduceL2 층 fp32 (노름 최대 ~2000, 제곱합 fp16 넘침 방지),
  InstanceNorm 을 평균/분산/제곱근으로 풀어 fp32 (동적 엔진 CUDA 720 없앰). 엔진 폴더 기본값 `/home/system/scanner/ffs_trt/engines_v3`.
- **크기**: 고정 544x960·960x544 (전체 960x540 프레임 = PyTorch 와 같은 패딩) + 동적 엔진 2 개 (가로 128..544x960 opt 384x672, 세로 128..960x544 opt 672x384).
  고른 순서: /32 크기와 똑같은 고정 엔진 → 그 크기를 담는 동적 엔진 → (없으면) 버킷. 960x960 하나짜리 동적 엔진은 작업 메모리 4 GB 부족으로 실패.
  엔진마다 `*.engine.json` (만든 방식) — 같은 방식이면 다시 안 만듦. 만들기: 데스크톱 406 s (544x960 은 앞 시험 것 재사용, 한 개 약 100–145 s).
- 합성 시험 (7 크기, 12 px): 시차 중앙 12.00–12.02, 31 ms (540x960), 13 ms (247x440).

| 121801 축소 40 키프레임, 같은 기계 (데스크톱) PyTorch fp16 대비 | TRT 버킷 (v1, 노트북) | **TRT v3 정확한 /32 (데스크톱)** | 참고: PyTorch fp32 vs fp16 |
|---|---|---|---|
| |dZ| 키프레임 p95 중앙 (mm) | 28.1 | **2.1** | 1.0 |
| 나쁜 10 % 키프레임 p95 (mm) | 314 | **8.2** | 4.3 |
| p95 > 10 mm 키프레임 | 68 % | **5 %** (2 개) | 0 % |
| 모은 |dZ| 중앙 / p95 / p99 (mm) | — | 0.5 / 2.8 / 10.8 | — |
| FFS 합계 (s) | — | 1.5 (PyTorch 58.8, 크기마다 torch.compile) | — |
- 40 개 모두 엔진 39 가지 /32 크기로 (버킷 패딩 0). 결과 `scan3d/runs/desktop/20261112_121801_std60/fix_u13d/` (`trt13d_desk.sh`).
  (표의 `cmp_kf40_pt_vs_trtv2` 이름·로그의 "trt v2" 글자는 v3 엔진 결과임 — 이름만 옛것.)

| 165506 보드 GT, fix_u13c_pt 의 같은 BA 자세 (--reuse), 둘 다 새로 (데스크톱) | PyTorch | **TRT v3** |
|---|---|---|
| mesh: 보드 |dZ| p90 모음 (cm) | 2.942 | 2.938 |
| mesh: 키프레임별 |dZ| p90 중앙 (cm) | 1.203 | 1.186 |
| mesh: 보드 dZ 치우침 중앙 (cm, Z 약 0.72 m) | 0.895 | 0.875 |
| mesh_board (보드 배율 적용): |dZ| p90 모음 / 키프레임 중앙 (cm) | 2.422 / 0.599 | 2.424 / 0.599 |
| 배율 k (BA 자세 vs 보드, 깊이와 무관) | +1.355 % | +1.355 % (같은 자세라 같음) |
| 보드 평면 RMS mesh / mesh_board (cm) | 0.058 / 5.648 | 0.058 / 5.723 |
| FFS 합계 mesh (s) | 23.7 | **3.4** |
| mesh 단계 / scan_offline 내부 (s) | 38.5 / 35.0 | **18.1 / 15.6** |
| 메시 정점 | 140 609 | 140 674 |
- PyTorch 재실행은 16:15 결과 (fix_u13c_pt) 와 숫자 같음. 결과 `scan3d/runs/desktop/20261024_165506_std60/fix_u13d_{pt,trt}/` (메시 ply/obj 커밋 안 함).
