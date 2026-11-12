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
