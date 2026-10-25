# integ2 독립 검증 (읽기 전용, 2026-10-26 06:05–06:20 KST)

검증 대상: ~/scanner/StereoDepthSeg-integ2 (integ2-20261026, HEAD 2459241). 폰은 쓰지 않음(adb 0회). 내 재실행 출력: /tmp/vint2/.
내가 만든 스크립트: C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\vint2\ (cmpv.py = 독립 포즈 비교기, phan.py = 폰 결과 분석기)

## 결론
- 통합 담당의 주장은 거의 다 **맞음**. 머지에서 실제 위험은 찾지 못함.
- 고칠 점 4개(작음):
  1. `I_1_timing.md` 의 N/O 비교표는 쓰면 안 됨. t_O_r1 은 포즈 825개 중 tracking 이 2개뿐이라 시간 비교가 안 됨.
  2. `predchk_I_1.txt` 의 "age_ms>500 & status 1" 칸은 의미가 없음. 리플레이(replay_time 0)에서는 모든 행이 age ≈ 1.34e8 ms 라서 모든 행이 걸림.
  3. raw80 의 `--drop-late-mode 0` 동일성은 `--drop-late 0` 과 함께 돌렸음. 그래서 mode 0 코드는 실행되지 않았고, 그대로 같게 나오는 것이 당연함.
  4. ident 로그 앞부분(REF/MINE sha256 줄)이 덮어써져 없어짐. 영향은 없음: 내가 다시 돌려 같은 포즈를 얻었음.
- "폰 suspend" 는 **과장이 아님**. 서로 다른 시계 3가지가 같은 방향을 가리킴. 다만 커널 기록으로 직접 확인한 것은 아니고 추론임. t_O_r2 는 순수 suspend 만으로는 설명되지 않음(아래 4 참조).

## 1. PC 동일성
| 주장 | 판정 | 근거 |
|---|---|---|
| harness 845/845 IDENTICAL (plain, pred-thread+VIOP, +viop-hb-src 1) | 맞음 | 내 비교기(cmpv.py)로 /tmp/integ2/ident 원본을 다시 비교: 3쌍 모두 845/845, tracking 815/815. 내가 직접 다시 돌린 결과(/tmp/vint2/h_ref·h_mine·hv_ref·hv_mine1)도 IDENTICAL. 내 h_mine·hv_mine1 은 통합 담당의 출력과도 비트 단위로 같음 |
| raw80 2327/2327 IDENTICAL (프로파일 없음 / 옛 ovopt 를 명시한 rec20261025 / rec60 / dlm0) | 맞음 (단서 있음) | 4쌍 모두 2327/2327, tracking 2003. 단서: 전부 `--drop-late 0` 이라 mode 0/1 코드는 실행되지 않음. 그래서 dlm0 동일성은 당연한 결과이고, drop-late 50(프로파일 기본값) 경로는 PC 동일성 검사에 들어 있지 않음 |
| 새 프로파일은 포즈 333 부터 다름, max\|Δp\| 4.1e-6 cm | 맞음 | 첫 차이 index 333. IMU p 기준 4.13e-6 cm(카메라 위치 기준 3.88e-6 cm). status 차이 0 |
| 결정적(반복하면 같음) | 맞음 | r_mine_new 와 r_mine_new_rep 가 2327/2327 같음 |
| 바이너리가 현재 HEAD 로 빌드됨 | 맞음 | ident 는 7f5f6a7 에서 실행됨. 그 뒤 커밋 3개(71a5d45·004c0f8·2459241)는 scan3d/ov_bridge 를 건드리지 않음. main.cpp·ovopt.cpp·StateHelper·VioManager 의 blob 해시가 HEAD 와 같음. 바이너리(05:49:31)가 모든 소스(최신 main.cpp 05:46:52)와 .o 파일보다 새것. CMAKE_HOME = integ2 워크트리. 빌드 로그에 ovpatch md5 경고 없음 |
| (부수) integ2_ident.log | 결함(작음) | 로그 앞부분이 덮어써져 REF/MINE sha256 이 남지 않음. 지금 바이너리 sha: MINE a0311486…, REF 32cf9bb2… (/tmp/vint2 재실행 로그) |

## 2. 프로파일 · 앱 · APK · 브랜치
| 주장 | 판정 | 근거 |
|---|---|---|
| bridge rec20261025/rec60 = all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse + idle spin 2, 20/0 ms, drop-late 50 mode 1, health 1, predict 2 @200 Hz, pred-thread 1 | 맞음 | main.cpp:2702-2711. 바이너리를 직접 실행해 출력 확인: `drop-late 50 ms mode 1 cap 3 win 3.00 s \| pred-stale 500 ms`, idle_spin_ms rec60=0 / rec20261025=20 |
| 앱 프로파일 ov_opt 도 같음. par_cov·init_gemm·gyro_klt 는 어떤 프로파일에도 없음 | 맞음 | MainActivity.java:273, 275-276, 287 |
| kFields 에 ekf_sparse·sym_inplace·init_gemm·gyro_klt 가 있음. sym_inplace 는 all_exact 에 들어가지 않음 | 맞음 | ovopt.cpp:34-36. sym_inplace 의 exact 값 = false |
| `--pipe 1` 은 거부됨, gyro_klt 를 함께 주면 전용 오류 | 맞음 | 직접 실행: 둘 다 rc 2, 각각의 메시지 확인 |
| APK = vc 32 / 0.6.26-integ2-20261026 | 맞음 | AndroidManifest.xml:2. APK 안에 문자열 존재 |
| APK 의 libov_bridge.so sha 527b94eb… = strip 한 build_integ2 | 맞음 | android/build_integ2/ov_bridge 를 llvm-strip 해서 직접 재현 → 527b94eb…. 스테이징한 폰 바이너리와도 같음. 폰에 실제로 올라간 파일은 adb 를 쓰지 않아 미확인. 다만 폰 pe.txt 의 ovopt 줄에 새 필드가 있음 |
| ovpipe/ov60eval 은 HEAD 의 조상이 아님 | 맞음 | merge-base 14de3b8. `git cherry` 결과 전부 '+'(같은 패치가 HEAD 에 없음). ovpipe.h 가 HEAD 에 없음 |
| calcap 파일 변경 없음 | 맞음 | `git diff calcap-20261025 HEAD` 결과가 비어 있음(CalCapRunner/CalCapView/scan3d/tools/calcap) |

## 3. 머지 충돌 해결 (heartbeat)
- **맞음**: 두 경로 모두 `lost_ || late_lost_ || hb_stale()` 를 넘김.
  - 확인 위치: main.cpp:1289 (note_main, hb-src 1), :1292 (hb_tick, hb-src 0), 머지 커밋 06ee302 의 diff.
- 다른 상호작용(실제 위험 없음):
  - mav_out.h 는 esp32fix 와 diff 가 0.
  - note_feed / note_main 은 atomic 만 건드림. reader 스레드는 g_hb_feed_out 을 통해서만 접근함.
  - 종료 시 순서는 close() → release(누수, hb-src 1 일 때만). close 이후 detached reader 가 note_feed 를 불러도 atomic store 만 하므로 안전.
  - VIOP odometry emit 은 `lost_ || late_lost_ || stale` 를 받음 (main.cpp:1427).
  - hb_stale() 는 메인 루프(VIO 와 같은 스레드)에서만 불림.
  - 내 hv_mine1 재실행: heartbeat 간격 p50 50.00 ms, feed/main stale skip 0.

## 4. 폰 drop-late (ARM, out_I_1)
| 주장 | 판정 | 근거 (phan.py, out_I_1/*/po.jsonl.gz·pe.txt·tl.csv·feeder.txt·samp.txt·run.txt) |
|---|---|---|
| d_m0: 25 s 뒤 600/600 버림, 포즈 0, h>0.5 s 인데 "tracking" 인 예측 3599줄 (max 20.0 s) | 맞음 | 포즈 654개 전부 t≤24.96. 654+600=1254=feeder 프레임 수. 예측 h>0.5 s 가 3599줄이고 전부 tracking, max 20.032 s. VIOP 6115행 전부 status 1(오래된 예측이 정상값으로 나감) |
| d_m1: 69 버림 + 22 forced, t=28.030 에 re-anchor(+400.0 ms), 25 s 뒤 tracking 포즈 531, 예측 h max 0.168 s | 맞음 | pe.txt 요약 줄. 예측 h>0.5 s 는 0줄 |
| PC 결과(75+22, 3.03 s, +400.3, 포즈 526)를 재현 | 맞음 | 동작이 같음. 수는 조금 다름 |
| mode 1 실행 전부에서 h>0.5 s 인 "tracking" 예측 0 | 맞음 | t_N_r1 7857/0, t_O_r1 7810/0, t_O_r2 5055/0, t_N_r2 0/0. tracking 인 예측의 max h = 0.499 s |
| 지연이 평평하면 re-anchor, 움직이면 late-lost | 맞음 | re-anchor 때 window range 22–40 ms, late-lost 때 201–2660 ms |
| t_* 4개는 폰 suspend 를 맞음 | 맞음 (간접 근거, 아래) | 아래 3가지 참조 |
| BOOTTIME 수신 지연 1.5–11.7 s 증가, MONOTONIC feeder 지연 ≤ ~0.4 s | 맞음 (t_O_r2 는 확인 불가) | tl.csv 의 rx_hdr_boot_ns − t (reader 스레드가 header 를 읽는 순간 찍음, main.cpp:2550): t_N_r1 +9.3 s, t_O_r1 +11.7 s, t_N_r2 +1.5 s. t_O_r2 는 처리된 프레임이 t=49.5 까지뿐이라 ≤0.29 s 만 보임. feeder late_max 403/366/336/327 ms |
| 실행 시간 88–105 s (스트림 82 s) | 숫자는 맞음, t_N_r2 는 근거가 안 됨 | 날짜 기준 길이 105/101/92/88 s. d 실행에서 본 기본 오버헤드가 5–6 s(45 s 스트림이 50–51 s)이므로 t_N_r2 의 88 s 는 정상 범위 |
| 79–1600 프레임 버림 | 맞음 | 1600/1502/936/79 |
| 시간 측정용으로 못 씀 | 맞음 | 처리 포즈 727/825/1391/2248, tracking 387/2/1067/1924 로 조건마다 크게 다름. t_O_r1 은 사실상 추적을 못 함 → I_1_timing.md 의 "O 가 더 빠름" 표는 무효 |

"suspend" 판정의 근거 (세 가지 모두 MONOTONIC 은 멈추고 BOOTTIME/REALTIME 만 흐른 경우에만 같이 나옴):
1. reader 쪽 BOOTTIME 지연이 수 초 늘었는데 feeder 의 MONOTONIC 지연은 ≤0.4 s 였음.
   - 밀림(backlog)으로는 설명 안 됨: reader 가 image space 를 기다리면 pipe 가 차고, 그러면 feeder 가 막혀서 feeder 지연도 같이 늘어야 함.
2. 실행 시간(REALTIME)이 feeder 의 MONOTONIC 길이(82.4 s) + 오버헤드보다 길었음: t_N_r1 +17 s, t_O_r1 +13 s, t_O_r2 +4 s, t_N_r2 ≈0.
3. 5 s 주기 샘플러(samp.txt)의 REALTIME 간격이 늘어남.
   - d 실행과 t_N_r2 는 5–6 s.
   - t_N_r1 은 최대 9 s(합쳐서 약 18 s 초과), t_O_r1 은 최대 12 s(약 15 s), t_O_r2 는 최대 8 s(약 6 s).

단서:
- 커널 suspend 기록은 모으지 않았음(추론임).
- t_O_r2: t=49.56 이후 처음 버린 프레임들의 지연이 799→475 ms 로 프레임마다 약 30 ms 씩 줄어듦.
  - 이것은 메인 루프에 쌓인 밀림(약 0.3 s)이 빠지는 모양임. 순수 suspend 만으로는 설명되지 않음.
- 제안: tl.csv 에 reader 가 이미 찍고 있는 t_hdr_mono 를 같이 남기면, (hdr_boot − hdr_mono) 로 suspend 시간을 직접 잴 수 있음.
- 추정: I_2 의 "PC 가 2 s 마다 adb 를 폴링해 깨워 둠" 이 suspend 를 막는다는 보장은 없음. 위 방법으로 확인할 것.

I_2: out_I_2 가 없고 I_2_timing.md 도 없음(06:15 기준) → 검증하지 않음.
