# timing_awake — 폰 깨운 상태 재생 시간 A/B (timing0626, 2026-10-26 06:30–06:53)

## 결론
- **계획 8회(ABBA×2) 중 깨끗한 실행은 2회(짝 1개)뿐.** 06:36 부터 **사용자가 폰을 직접 사용**(카카오톡·Brave·블로그 앱, 이후 충전기 뺌) → 나머지는 중단. 사용 중인 폰에 부하·발열을 주지 않으려고 06:53 에 러너를 멈추고 잠금 해제.
- 깨끗한 짝 (w_N_r1 vs w_O_r1, 둘 다 Awake·잠금화면·AC 연결·충전 안 함 st4, cpu9 상한 2304 MHz, 절전 틈 없음 = rx 지연 계단 2 ms):
  - 프레임 처리 p50 **7.78 vs 8.13 ms (−0.36, −4 %)**, p95 **11.49 vs 11.75 (−0.25)**, max 20.1 / 19.3, >33 ms 0 / 0.
  - SLAM 업데이트 (slam_jac+slam_ekf, SLAM 있는 프레임) p50 **0.92 vs 1.30 ms (−0.38, −29 %)**, p95 0.99 vs 1.47; slam_ekf 만 0.63 vs 1.01. 프레임 차이 ≈ SLAM EKF 차이 (slam_jac 은 같음 → 이치에 맞음).
  - 같은 프레임끼리 짝 비교 (2003 프레임): 중앙 차 −0.344 ms (부트스트랩 95 % [−0.358, −0.328]), 85 % 프레임에서 N 이 빠름. **이 구간은 실행 안 정밀도뿐** — 실행 간 흔들림은 짝 1개라 알 수 없음 → **프레임 단위 이득 ≈ −0.35 ms (p50) 는 "추정"** (방향은 SLAM 부분 및 integ2 I_2 의 slam_ekf 결과와 일치).
  - 섞인 변수: 순서 (N 먼저; SKIN 시작 31.1 vs 33.4 °C, O 가 더 따뜻한 상태; cpu9 상한은 둘 다 실행 중 2304 고정, O 는 끝난 직후 2112). ABBA 로 상쇄할 뒤 B·A 실행을 못 함.
- 절전 문제: Awake 로 돌리니 feeder 늦음 max 51 ms (I_2 389 ms), 벽시계 85.0 s (스트림 82.4 s), rx 지연 계단 없음 → 깨끗한 2회에서는 **화면이 Awake 인 동안 절전 틈이 없었음**. 단 06:51–52 에 Dozing 상태에서 KEYCODE_WAKEUP 이 6번 안 먹힘 → WAKEUP 만으로 늘 깨어 있게 된다고는 말 못 함.

## 실행 표 (전체 열은 `timing_awake.md`, `tw_an.py` 로 생성)
| run | 시작 | feed p50/p95 | SLAM upd p50/p95 | lag 계단 ms | wake | fg | cpu9 cap 시작/중앙/끝 | SKIN 시작/최대 | AP max | 전원 | 유효 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| w_N_r1 (새) | 06:31:55 | 7.78/11.49 | 0.92/0.99 | 2 | Awake | 잠금화면 | 2304/2304/2304 | 31.1/34.8 | 43.5 | AC st4 | 예 |
| w_O_r1 (옛) | 06:34:21 | 8.13/11.75 | 1.30/1.47 | 2 | Awake | 잠금화면 | 2304/2304/2112 | 33.4/36.2 | 45.0 | AC st4 | 예 |
| w_O_r2 (옛) | 06:51:50 | 8.77/13.29 (1110 프레임까지) | 1.29/1.72 | 1 | Dozing→Awake | 잠금화면→블로그 앱 | 2304/2304/1824 (최대 2592) | 35.0/37.5 | **50.9 → 중단** | 배터리 st3 (시작 때 이미 충전기 빠짐) | 아니오 |

## 방법
- 바이너리·입력: integ2 브리지 `ov_bridge_integ2` sha256 527b94eb (폰 `/data/local/tmp/integ2` 에서 내 폴더 `/data/local/tmp/timing0626` 로 복사), feeder4 (= integ feeder4_integ, sha256 ef067ff6), `raw80.bin` (`/data/local/tmp/replay`, 읽기만), c4 cfg (경로만 내 폴더로 바꿈). 카메라 없음, 앱 설치 없음, 설정 변경 없음.
- N = `--profile rec20261025` (ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse), O = 같은 프로필 + `--ovopt all_exact,ekf_gemm,qr_compress,ekf_chol`. 둘 다 `--drop-late 0`, threads 2, c4.
- 러너 `timing0626_run.sh` = integ2_run.sh 복사 + 실행 전 mWakefulness 가 Awake 가 아니면 `input keyevent KEYCODE_WAKEUP` (설정 안 바꿈; `stay_on_while_plugged_in=15` 는 이미 켜져 있었음), 실행 중 5 s 마다 wake·cap·SKIN·AP (W1b 부터 fg 도) 기록, run.txt 에 uptime/realtime 시작·끝과 cpu9 cap 끝. 문턱: 시작 SKIN<36·AP<50, 중단 SKIN≥45·AP≥50, 사용자 앱이 앞에 있으면 대기, 실행 사이 60 s.
- 유효 조건 (`tw_an.py`): rx 지연 계단 (rx_boot − 스트림 t 에서 실행 전체 최소값을 뺀 뒤, 1 s 간격 5 s 창 최소값의 최대) < 200 ms (검증자는 다른 정의로 10 ms 계산 — 둘 다 200 ms 보다 훨씬 작음), 모든 표본 Awake, 깨움 0회, 중단 아님, 앞 화면이 잠금화면/런처뿐.
- `framepair.py` = 같은 스트림 시각 프레임끼리 N−O 차 (부트스트랩 2000).

## 독립 검증 (읽기 전용 에이전트, 06:58)
- 수치 1–4·6 맞음 (p50/p95, SLAM upd, 프레임 짝 −0.344 ms, Awake·AC st4·cap 2304). 고친 것: O_r2 충전기는 시작 때 이미 빠져 있었음, cap 최대 2592 누락, "절전 틈 사라짐" 일반화는 과장 → 위 문구 수정. 스크립트 `scratchpad/timing0626/verif/an.py`.

## 사건 기록
- 06:36 사용자가 폰 사용 시작 (카카오톡→Brave) → 러너 대기. 06:38 adb 끊김 1회 → `adb connect` (PC 쪽 재연결만) 로 복구. 06:39 러너를 fg 기록 판으로 다시 시작 (W1b = O_r2, N_r2).
- 06:51:50 사용자가 블로그 앱(06:51:17 부터 앞)을 쓰다 잠깐 화면을 끈 사이 w_O_r2 시작 — **충전기는 시작 때 이미 빠져 있었음** (battery_start AC false, st3; 러너 시작 문턱이 충전 상태를 안 봄 = 러너 결함). Dozing 에서 WAKEUP 6번 안 먹힘, 사용자가 다시 블로그 앱 사용, cap9 2592 까지 오른 뒤 1824, AP 50.9 → 러너가 정상 중단. 실행 표에 없는 값: max 159.7 ms, >33 ms 7개. 06:53 러너 종료 (kill -9), 브리지·feeder 없음, 카메라 `[]`, 잠금 해제 (owner 확인).

## 남은 일
- ABBA×2 (8회) 다시: 사용자가 폰을 안 쓰는 시간 + 충전기 연결 상태에서 `W1.txt`/`W2.txt` 그대로. 판단 기준 = 짝 차의 실행 간 흔들림 (4짝). 러너에 시작 문턱 "AC 연결" 추가할 것.

## 파일
`out_W1/` (원시: tl.csv, prof.csv, samp.txt, run.txt, pe.txt, feeder.txt, battery_*.txt, po.jsonl.gz, status.txt), `timing_awake.md`, `framepair.txt`, `timing0626_run.sh`, `W1.txt`, `W1b.txt`, `W2.txt`, `tw_an.py`, `framepair.py`, `pull.sh`, `mk.py`, `watch.log`.
