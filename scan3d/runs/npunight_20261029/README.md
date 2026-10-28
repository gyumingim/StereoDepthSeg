# npunight 2026-10-28 밤 → 10-29 아침 — 카메라 없이 폰 재생(replay)으로 NPU 루프 A/B + 앱 속 느린 부분

브랜치 `npunight-20261029` (WSL `~/scanner/StereoDepthSeg-appnpu`, appnpu-20261028 93e53b2 위). **모든 숫자 미검증** (코디네이터가 검증).
카메라는 한 번도 켜지 않는다. 폰은 스스로 못 움직이므로 PC 에서 만든 녹화 입력 스트림을 앱 안에서 다시 넣는다.

## 0.6.35 (vc 41) 에서 바뀐 것
1. **intent `replay <파일>`** (외부 files 폴더 기준 상대 경로, 예 `replay/165506.bin`): 카메라·IMU 센서를 열지 않고, 재생 스레드가
   다리(브리지) 입력 레코드(12 B 머리 + 내용, 내용 앞 8 B = 시각; PC `tee` / live_stream.bin 과 같은 형식)를 레코드 시각대로
   다리 stdin 에 씀. 모든 시각에 상수 하나를 더해 첫 레코드 = 지금 + 0.5 s (elapsedRealtimeNanos 시계) → 다리는 실시간 스트림으로 봄.
   다리 인자·NPU 서버(앱 프로세스 안 ENN)·스레드는 live 와 같음. **다른 점**: 이미지를 쪼개 보내지 않음 (imu_prio 전송 안 씀), 카메라 스레드 없음,
   화면 안내(RunCue) 없음. 파일 끝 → 3 s → 자동 종료. 다리 pose 줄 전부 `poses_raw.jsonl` (시각은 이동된 값, meta `replay_offset_ns`).
   meta: `replay`, `replay_records`, `replay_images`, `replay_late_max_ms`, `replay_late_over5ms` (레코드가 예정보다 5 ms 넘게 늦은 수).
2. **intent `npu_opts`** (서버 CPU 후처리, 기본 = 0.6.34 와 같음): `nn_fast=1` 상호 NN 내적을 16 개 부분합 (NEON fma) 으로 (합 순서가 달라 cos 마지막 비트가 다를 수 있음),
   `lg_post=3` LighterGlue 후처리를 곱 형태로 (nncrun lgpost `cpu_prod_bench.cpp` postp, PC 13/13 쌍 같은 짝; N² log 없음). 코어 고정은 기존 `npu_cpus`.
3. 다리 바이너리 = 0.6.34 와 같음 (libov_bridge sha 2ed40e6b…). libxfnpu 새 빌드. Java 3 파일.

## PC 확인 (폰 아님)
- `xfnpu_pc` 시험 (`test_server.py`, 프레임 2 장, 같은 작성자): 기본 vs `nn_fast=1,lg_post=3` → 키포인트 512/512, 상호 NN 251/251, LG 175/175 둘 다 기준과 같음
  (쌍 1 개뿐이라 약한 확인). PC 시간은 x86 스칼라 흉내라 무관.

## 입력 (재생 스트림)
- PC `mkstream.py` (= appnpu_replay.py + 다리 stdin `tee`): c4f60 폰 설정 + PC 다리, `--profile rec60 --loop 1 --loop-sync 1`. 같은 실행의 PC 포즈/루프/ATE 도 남음 (`~/scanner/tmp_npunight/pc/`).
- GT 있는 std60 녹화는 2 개뿐: `hs_20261024_165506_std60`, `hs_20261024_173432_std60` (각 28 s). 세 번째·네 번째 = `hs_20261021_213341`, `hs_20261021_213955` (HS 240 fps 녹화를 60 fps 로 골라 먹임, 처음 40 s, 초점 2.22 dpt ≠ 설정 0.5 dpt → 내부 값 약 1.4 % 차이, **가정**). GT 없는 `142148`, `142610`, `151646` 은 여분. `173432` 는 PC 에서 VIO 가 초기화 안 됨 (추적 0) → 뺌, 213955 도 PC 에서 발산 (ATE 749 cm, 추적 603/2317) → 빼고 **214550** (PC ATE 0.49 cm) 을 씀. **GT 녹화 3 개 = 165506, 213341, 214550** (21:35 결정, 폰 측정 전).
- adb 가 느려서 (약 1 MB/s) 원본 스트림(28 s = 3.5 GB) 대신 H.264 crf 18 + 영상 없는 레코드 파일(side.bin)을 보내고 폰에서 `unpack` (ovheat 도구, MediaNDK 디코드 + LUT) 으로 다시 만듦 → **폰 입력 ≠ PC 입력 (손실 압축)**. 모든 폰 run 은 같은 파일.

### 폰 입력 만들기 (실제로 한 것, 21:35)
- 165506: 폰에 원본 cam5.mp4 가 있음 → `side.bin` + `idx.txt` (스트림 카메라 레코드 ↔ 원본 프레임 번호, `make_side_idx.py`, 시각 오차 0) 만 보내고
  `unpack2 ... nolut` (ovheat LUT 는 x264 재인코딩용이라 원본에는 안 맞음: LUT 쓰면 PC 대비 평균 4.75 회색값 차, 안 쓰면 **1.65**, 12 MB 표본 1 회). 크기 PC 와 같음 (3,505,463,192 B).
- 213341, 214550: 폰에 원본 없음 → `to_mp4.py --crf 23` (x264, 78 MB) + LUT (ovheat 방식). 213341 4,820,552,180 B = PC 와 같은 크기. adb 0.4 MB/s.
- 첫 run `T0_smoke` = 연습 (재생 경로 확인용, A/B 에 안 넣음).

## 사전 등록 (측정 전 커밋)
- 앱 0.6.35, `--es mode ov --es profile rec60 --es replay replay/<r>.bin` (+ `--ei npu_loop 1` 또는 없음 = npu 0). npu 0 = BoW+BRIEF 루프 (max_cand 3), npu 1 = XFeat+LG 문 (FLGc, max_cand 1).
- **A/B 순서 ABBA** (녹화마다): A1 = npu 1, B1 = npu 0, B2 = npu 0, A2 = npu 1. npu 1 은 기본 서버 설정 (0.6.34 와 같은 cpus 자동 0-3, nn_fast 0, lg_post 2).
- 시작 관문: SKIN < 38 그리고 AP < 40 (10 s 마다 확인, 최대 15 분 기다림; 못 내려가면 그 run 은 건너뛰고 기록). 정지: SKIN ≥ 45 또는 AP ≥ 60 또는 배터리 < 15 % (충전 중이 아니면). 화면은 앱이 켜 둠.
- 1 차 지표 = **지도(루프 보정) 포즈 ATE RMSE (cm)**: pose 줄 `map.T_world_cam0` 위치 vs GT (`gt.npz` 의 `p_bc`, 가장 가까운 GT < 2 ms, SE3 정렬, 추적 중 포즈만).
  보조: VIO 포즈 ATE, 끝점 오차 (마지막 1 s 평균, 정렬 뒤), 루프 채택 수, **맞는 루프** = 두 키프레임 시각의 GT 카메라 중심 거리 < 0.5 m 그리고 광축 각 < 30°
  (ovloop 재방문 자격; 상대 포즈 비교는 GT 축 규칙 가정이라 참고만), NPU/후처리 시간, VIO ms, 온도.
- 유효 run: meta.json 있음, bridge_exit 0, stop_reason "replay end", npu 1 은 run_valid true · xf_down 없음, 안전 정지 없음, `replay_late_over5ms` ≤ 레코드 1 %.
- 판정 (녹화마다, 네 run 모두 유효일 때만): 두 A 의 지도 ATE 가 두 B 보다 모두 작으면 "NPU 쪽이 낫다", 모두 크면 "BRIEF 쪽이 낫다", 겹치면 "차이 없음".
  전체: GT 녹화 3 개 중 2 개 이상 같은 쪽이고 반대쪽이 0 개면 그쪽, 아니면 "차이 보이지 않음". 4 회 × 3 녹화뿐 → 결과는 **추정**.
- 속도 (녹화 165506, ABBA 뒤): S1 = `npu_opts nn_fast=1,lg_post=3` (cpus 0-3), S2 = S1 + `npu_cpus 4-6`, S3 = S1 + `npu_cpus 7-8`, 각 2 회 (S1 S2 S3 S3 S2 S1).
  지표: npu.jsonl F (prep/npu/post_q) · M1 (상호 NN) · M2 (LG pack/npu/post) p50, 다리 VIO ms p50/p95 (poses_raw `ms`), 다리 큐 버림. 짝 수가 S0 (=A run) 과 같은지.
  좋아지면 0.6.35 기본값을 바꾸지 않고 (검증 전) 권장만 적는다.
- 한계: 같은 입력 재생이라 run 끼리 차이 = 스레드 타이밍·열 상태 (비결정). NPU 쪽과 BRIEF 쪽은 루프 문(max_cand 1 vs 3)도 다름 (섞인 변수).

## T0_smoke (0.6.35, 21:43, npu 1, 165506) = 연습 → 0.6.35b
- 재생 경로 동작: VIO 추적, NPU 열림 (cpus 0-3), 루프 8 (GT 맞음 8), 지도 ATE 1.84 cm (PC 1.86). 카메라 클라이언트 [] (카메라 안 켜짐).
- 문제 2 개: (1) PC 스트림 끝의 type 4 (끝 표시) 레코드를 "bad record" 로 보고 stop_reason 이 replay error. (2) 레코드 13 % 가 5 ms 넘게 늦음 (최대 49 ms) —
  원인 미측정 (FUSE 파일 읽기 추정). 둘 다 run 무효 규칙에 걸림 → **0.6.35b (vc 42, apk sha ca64df4d…)**: 파일을 따로 미리 읽는 스레드 (레코드 400 개 큐, 이미지 버퍼 24 개 재사용),
  type 4 = 끝. 네이티브 (다리 2ed40e6b, libxfnpu 5d10f2e9) 는 0.6.35 와 같음. A/B 는 0.6.35b 로 처음부터.
- rep_run.sh: 앱 프로세스는 finish 뒤에도 남으므로 새 run 폴더의 meta.json 이 생기면 끝으로 봄.
- T0 NPU 시간 (1 회, 작은 코어 0-3, 미검증): XFeat prep 11.2 · NPU 8.6 · post+질의 16.1 ms (p50), 상호 NN 88 ms, LG pack 2.3 · NPU 18.3 · post 33.9 ms. VIO ms p50 9.6 / p95 14.9 (PC 6.2 / 10.6). AP 최대 51.

## 0.6.35c (vc 43, apk sha 7defcdeb…, libxfnpu e6bee298…) — 빌드만, ABBA·S 묶음이 끝난 뒤 설치
- `npu_opts prep_fast=1`: XFeat 입력 평균·분산을 정확한 정수 합 (Σpx, Σpx² uint64) 으로 (T0b prep 11.9 ms = 픽셀마다 double 누적합, 작은 코어).
- PC (`pc_opts_c.sh`, 165506 전체, PC 대역 NPU): nn_fast=1,lg_post=3,prep_fast=1 vs 기본 → 포즈 1690 줄 같음, loop.jsonl 은 시간 칸 (ms, xf 서버 ms) 만 다름. `test_server` 키포인트·NN·LG 같음.
- 계획: 묶음 뒤 S4 = 세 옵션 모두 + S 결과에서 가장 빠른 코어, 2 회.

## 165506_A1 / B1 (21:58–21:59) 무효 → ABBA 처음부터
- 화면이 꺼져 있어 (T0b 뒤 화면 꺼짐, 보안 잠금) 액티비티가 1.2 s 만에 onPause → stop_reason "user", 레코드 0. B2 는 기다리는 중 멈춤.
- rep_run.sh: am start 전에 `input keyevent KEYCODE_WAKEUP` (화면만 켬, 소리 없음). 무효 폴더는 `phone_invalid/`.
