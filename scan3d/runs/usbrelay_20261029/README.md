# usbrelay 2026-10-29 — 폰 앱 USB 중계 (ov_bridge VIOP <-> FC USB CDC)

## 0. 계획 (구현 전에 적음)
문제 (독립 검증 verify_arm_20261029 Re-verify): 앱의 USB 코드는 ViopImuSender (IMU bulk OUT) 하나뿐. ov_bridge 의
POSE / HEARTBEAT (UDP) 를 USB 로 FC 에 보내는 길도, FC -> 폰 (USB IN, 예: VIOP_T_VIO_RESET_REQ 0x23) 을 ov_bridge 로
넘기는 길도 없음 -> 실제 연결에서 FC 는 pose 를 못 받음.

할 일
1. fcarm d1b5779 (폰 쪽 SEEDED / F_RESET x50 / 0x23 처리) cherry-pick -> ov_bridge Android 바이너리 다시 빌드.
2. Java USB 중계 (양방향), USB 연결·끝점 주인은 하나 (ViopUsbLink) 를 ViopImuSender 와 같이 씀:
   - 쓰기 스레드 하나 + 큐 2개 (IMU 우선, 그다음 pose/heartbeat/기타). 한 프레임 = bulkTransfer 하나 (바이트 섞임 없음).
   - ov_bridge 는 지금처럼 UDP 로 보냄 (--viop-udp 127.0.0.1:<중계 포트>); 중계가 데이터그램 하나 = VIOP 프레임 하나를
     받아 USB OUT 큐로. 프레이밍은 viop_proto.h (A5 5A, ver 2, crc16 CCITT-FALSE) 그대로, 바이트 안 바꿈.
   - USB IN 읽기 스레드: 스트림에서 VIOP 프레임 찾기 (sync, len, crc) -> 0x23 은 ov_bridge UDP 소켓 (마지막 보낸 주소)
     으로 그대로 넘김 (ov_bridge 의 kReset 경로), 나머지는 종류별 개수만 기록 (FC 는 지금 USB 로 0x23 만 보냄).
   - 켜는 법: intent viop_udp usb (새 값). 기본 (intent 없음) = 명령줄·동작 그대로. USB 장치 없음 -> 중계는 받아서
     버리고 개수만 셈 (VIO 경로 영향 없음).
   - PC 시험: 가짜 USB 끝점으로 프레임 무결성, IMU 우선, IMU 지연 증가 (pose 트래픽 있음/없음), USB IN -> UDP, 장치 없음.
3. FC: F_RESET 50번 반복 구간에서 래치가 매 pose 마다 풀리는 문제 -> 새 W 하나에 한 번만 (fcarm 브랜치, PC 시험 추가).
4. APK 0.6.38 빌드 (설치 안 함) + SHA256SUMS, 이 README 에 시험한 것 (PC 만) / 보드+폰 시험 필요한 것 / 벤치 시험 순서.

규칙: gyro_klt=5 프로필 기본값 유지, NPU 의 BRIEF 대체 없음, 측정 안 한 것은 추정.

## 1. 한 일 (2026-10-29, PC 만 — 폰·보드·adb 안 씀)
| 항목 | 결과 |
|---|---|
| fcarm d1b5779 cherry-pick (SEEDED / F_RESET x50 / 0x23) | 충돌 0 → appfix 60a7652 |
| ov_bridge Android 빌드 | rc 0, 경고 0, stripped sha `3a012a39…` |
| 빌드 재현 확인 | 같은 소스·같은 경로 (appnpu) 다시 빌드 = 0.6.36 다리 `2ed40e6b` 와 **build-id 20 바이트만 다름** (코드 동일) → 플래그 같음 |
| Java USB 중계 (ViopRelayCore + ViopImuSender.UsbLink + OvRunner) | 아래 2절 |
| PC 시험 `test/run_test.sh 10` | **ALL PASS 62 / 0**, FC 파서 (esp32fc viop_stream) 로 OUT 바이트 다시 검사 **ALL OK** |
| FC: F_RESET 반복 구간 래치 문제 | fcarm 1af078e: `make run` **711 / 0** (새 시험 arm_hist_reset_run_once, 옛 규칙이면 3개 실패 확인), `make syntax` fail=0, `idf.py build` (v5.5.6) **경고 0** |
| APK 0.6.38 (vc 49) | 설치 안 함. `scratchpad/team/usbrelay/apk/arlat_0.6.38_usbrelay.apk` sha256 `0a298ac8…` (SHA256SUMS). 네이티브: libov_bridge 만 새 것, 나머지 + assets = 0.6.36 과 같음 (libarloop 는 appfix 때처럼 0.6.36 파일 그대로 넣음) |

## 2. 앱 중계 동작 (0.6.38)
- 켜기: intent `viop_udp usb` (새 값). ov_bridge 는 `--viop-udp 127.0.0.1:<중계 포트>` 로 지금처럼 UDP 로 보냄 → 앱이
  데이터그램 하나 (= VIOP 프레임 하나, viop_check 통과한 것만) 를 USB bulk OUT 으로 바이트 그대로 보냄.
  `viop_imu usb` 를 같이 주면 IMU 도 **같은 USB 연결** (UsbLink 하나) 을 씀.
- 쓰기 규칙: USB 전송은 잠금 하나 (한 번에 하나). IMU 는 빈 때 센서 스레드가 바로 씀 (= 0.6.37c 와 같은 지연),
  전송 중이면 IMU 큐 (POSE 보다 먼저 나감) 에 넣고 바로 돌아옴. 쓰기 스레드는 IMU 큐를 다 비운 뒤에만 POSE/HB 를
  묶어서 (≤ 256 B) 보냄. 큐가 차면 가장 오래된 것 버림 (IMU 64, 기타 32, 개수 기록).
  처음엔 "모든 IMU 를 큐로" 했는데 PC 시험에서 넘겨주기만으로 IMU p95 가 0.6–1.4 ms 늘어서 (아래 표 `_q`) 바꿈.
- 읽기: bulk IN 스트림을 viop_stream 규칙으로 파싱 → 0x23 은 ov_bridge 소켓 (마지막 데이터그램 보낸 주소) 으로 그대로
  보냄 (다리의 confirm 검사 → full_reset). 다른 종류는 개수·처음 200개 기록만 (지금 FC 는 USB 로 0x23 만 보냄.
  FC_STATUS / FC_EVENT 는 ESP-NOW 로 PC 에 감 → "폰에서 FC 상태 기록" 은 FC 가 USB 로 보내지 않아 할 것 없음).
- 끝낼 때: 다리가 끝난 뒤 (STOPPING 하트비트 3개도 나간 뒤) 중계 정지 → `<run>/viop_usb.json`, meta `viop_usb_result`.
  닫기 전 DTR 0 을 보냄 (FC link_task.c 가 "앱이 포트 닫음" 으로 읽음 → 빠른 폰 끊김 판정). **새 동작**.
- USB 장치 없음 / 권한 없음: 중계 소켓은 받아서 버림 (lo_no_port 로 셈), VIO 영향 없음. 권한 창이 뜨면 이번 실행은
  포트 없이 돎 → 허용 후 다시 시작 (0.6.37c 와 같음).
- intent 없음 (기본) / `viop_udp HOST:PORT` / `viop_imu udp:…`: 앱 명령줄은 0.6.37c 와 같음. **단 다리 바이너리가 새 것 (d1b5779)
  이라 UDP 로 나가는 VIOP 도 바뀜**: 새 W 의 처음 50 pose 에 F_RESET (예전 1번), 상태 주입 W 에 SEEDED → "USB 아닌 경로는 그대로"
  는 **틀림** (검토 지적). 이 폰 앱은 FC 펌웨어 **≥ fcarm 1af078e** 와 짝지어 쓴다 (그 전 펌웨어는 F_RESET 50번마다 래치를 다시 풂).
  npu_loop 경로 안 건드림.
  gyro_klt=5 프로필 기본값 그대로, NPU BRIEF 대체 없음.

## 3. PC 시험 숫자 (가짜 USB 끝점, 추정 조건)
가짜 전송 시간 = **가정** (S24 실측 아님): fast 0.125 ms + 0.02 ms/64B, slow 1 ms + 0.1 ms/64B. IMU 500 Hz, POSE 200 Hz + HB 20 Hz,
10 s, 실행 1번씩. **PC 부하 load average 29–49 (16 CPU, 다른 작업 중)** → max 값은 OS 선점이 섞임, p50/p95 만 비교.
| 경우 | IMU 들어옴→쓰기 끝 p50 / p95 / p99 ms | POSE UDP→쓰기 끝 p50 / p95 ms |
|---|---|---|
| fast IMU 만, 새 (d) | 0.147 / 0.158 / 0.297 | – |
| fast 옛 직접 쓰기 (0.6.37c) | 0.146 / 0.150 / 0.253 | – |
| fast IMU + POSE, 새 (d) | 0.147 / 0.167 / 0.357 | 0.196 / 1.620 |
| fast IMU + POSE, 모두 큐 (q, 안 씀) | 0.172 / 1.571 / 3.471 | 0.205 / 1.955 |
| slow IMU 만, 새 (d) | 1.101 / 1.116 / 2.148 | – |
| slow 옛 직접 쓰기 | 1.101 / 1.118 / 2.143 | – |
| slow IMU + POSE, 새 (d) | 1.102 / 2.125 / 2.752 | 1.924 / 5.126 |
- 결론 (PC 기준): POSE 가 없으면 IMU 지연 = 옛 경로와 같음. POSE 가 같이 가면 IMU p95 +0.01 ms (fast), slow 가정에서는
  **부하에 따라 다름** (이 표 +1.0 ms, 검토자 재실행 +0.36 ms — 반복 1번씩이라 확정 아님). slow 에선 POSE 가 IMU 에 밀려 p95 5 ms.
  **폰 실측 없음 → 추정**.
- 그 밖의 확인: 프레임 무결성 (8 경우, 바이트·순서 그대로, 버림 0), 쓰는 쪽 동시 2개 없음, 0x23 (조각 5 B 단위, 쓰레기·CRC 틀린
  프레임 섞임) → 맞는 2개만 다리 소켓에 그대로 + C 로 다리 검사식 통과, 다리 주소 모를 때 셈, 장치 없음, FC 멈춤 (모든 쓰기
  timeout) 에서 큐 한도·개수 맞음, 정지 때 STOPPING HB 3개 나감.
- 시험 안 한 것: 실제 Android UsbDeviceConnection (두 스레드가 OUT/IN 끝점을 동시에 bulkTransfer — 문서상 가능, **추정**),
  실제 ov_bridge 프로세스 → 중계 (다리 코드의 send_raw 는 프레임 하나 = 데이터그램 하나, 코드로만 확인), TinyUSB 수신 버퍼와 256 B 묶음.

## 4. 보드 + 폰이 있어야 확인되는 것
1. USB 권한·열기·DTR (FC 가 DTR 1 받음, 앱 끝날 때 0) — FC 로그 FC_MSG_PHONE_CONN.
2. FC 가 POSE / HB / IMU 를 실제로 받는지, 속도 (POSE ~200 Hz, HB 20 Hz, IMU ~500 Hz), CRC 오류 0.
3. viop_usb.json 의 imu_q_ms_* / lo_q_ms_* / max_write_ms (실제 전송 시간), imu_drop_full = 0, write_fail_calls = 0.
4. IMU 지연이 0.6.37c 대비 나빠지지 않았는지 (viop_imu.json lat_ms_p95, 0.6.37c 는 USB 로 시험된 적 없음 → 첫 실측).
5. FC → 폰 0x23: PC 에서 VIOP_CMD_VIO_RESET → FC → USB → 앱 → 다리 "VIO reset request from the FC" 로그 → reset_counter +1.
6. 케이블 뽑기 / 앱 강제 종료 때 FC 의 폰 끊김 판정.

## 5. 벤치 시험 순서 (프롭 없이 — 프롭을 반드시 뺀다)
독립 검토 (scratchpad team/review_usbrelay_20261029.md) 판정: 이 벤치는 **배터리와 ESC 신호선 둘 다 뺀 상태, 프롭 없음일 때만 GO**.
모터 전원이 들어가는 시험은 **NO-GO** — 하드웨어에서 USB 지연 · 부분 쓰기 (partial write) · OUT/IN 동시 bulkTransfer 를 먼저 확인할 것.
0. 준비 (먼저):
   - **배터리 뽑기 + ESC 신호선 뽑기 (둘 다)**. 프롭 없음. ESC 캘리브레이션은 아직 하지 않는다.
   - **킬 스위치 시험을 맨 처음**: PC 에서 KILL → FC 상태 KILLED 확인, CLEAR_KILL 까지 (모터 출력 0 인지 FC 로그로).
   - **ESP-NOW 짝 확인**: FC_PC_PEER_MAC 이 브로드캐스트 (짝 안 됨) 이면 **아무 MAC 의 명령이나 받음** (link_task.c pc_mac_ok, 벤치 전용)
     → 실제 PC 쪽 ESP32 MAC 을 넣고 빌드했는지 확인.
   - 폰 케이블은 ESP32-S3 의 **네이티브 USB 포트** (TinyUSB, GPIO19/20) 에 꽂는다 — UART 브리지 포트 아님.
   - **케이블을 뽑았다 꽂거나 FC 를 리셋하면 앱을 다시 시작**한다 (앱은 다시 열기를 안 함: 포트가 죽은 채로 돎).
   - 나중에 모터 전원을 넣는 단계가 오면 그 전에 **전류 제한 전원** (벤치 전원) 부터 쓴다. 이 README 의 순서에는 모터 전원 단계 없음.
1. FC 에 fcarm 1af078e 펌웨어 올리기 (`. ~/esp/esp-idf/export.sh && cd esp32fc && idf.py -p <포트> flash monitor`), PC 쪽 ESP32 와 짝.
2. 폰에 APK 0.6.38 설치 (SHA256SUMS 확인). 폰 ↔ FC 를 USB-C OTG 로 연결 → 처음엔 권한 창 → 허용.
3. 첫 시험 (카메라 없이 IMU 만): `am start ... --es mode imu_tx --es viop_imu usb --ei secs 20` →
   files/ov/imu_tx_*/viop_imu.json 의 usb_link: imu_written ≈ imu_in, imu_fail 0, max_write_ms; FC 모니터에 IMU 수신 수.
4. VIO + 중계: 평소 live 실행 intent 에 `--es viop_udp usb --es viop_imu usb` 추가 (predict 1/2 필요). 폰을 책상에 둔 채 60 s.
   확인: FC 모니터 pose 나이 (pose_age_ms) 작음, HB 끊김 없음, ARM 판정 (armguard 정지 1 s) 이 정상으로 바뀌는지 (실제 ARM 은
   모터 전원 없이만).
5. 0x23: FC 가 DISARMED 일 때 PC 에서 VIO_RESET 명령 → 다리 로그 (bridge_stderr.log "VIO reset request") + viop_usb.json
   reset_fwd 1, FC 에 새 reset_counter 와 F_RESET 50 pose → 래치 해제 (FS_B_VIO_HIST 사라짐).
6. 앱 정지 → FC 가 STOPPING HB 와 DTR 0 을 받고 "폰 끊김" 으로 가는지. 케이블 뽑기도 한 번.
7. 결과 파일 (viop_usb.json, viop_imu.json, meta.json, FC 로그) 을 PC 로 가져와 이 README 에 적기.

## 6. 위험한 곳 (독립 검토자에게)
- ViopRelayCore 잠금 순서: 쓰기 스레드는 wl → lk, 센서 스레드는 wl.tryLock → lk (교착 없음 의도). IMU 순서 보장은 "쓰기 스레드는
  wl 잡은 채로만 큐에서 꺼냄" 에 기댐.
- 빠른 경로는 센서 스레드가 USB 전송 동안 막힘 (0.6.37c 와 같음, timeout 2 ms). FC 가 멈추면 프레임마다 2 ms.
- 2개 스레드에서 같은 UsbDeviceConnection 에 bulkTransfer (OUT/IN) — 하드웨어 미시험.
- DTR 0 on close = 새 동작 (FC 가 폰 끊김으로 봄).
- prepare() 에서 링크를 연 뒤 시작 실패 → stop() 의 이른 return 두 곳에 closeUsbLinkEarly 추가 (Android 에서 미시험).
- FC 수정 + UDP: UDP 경로 (USB 아님) 에서 순서가 바뀐 pose (seq 뒤로) 는 DISARMED 에서 "보낸 쪽 재시작" 으로 읽혀 F_RESET 이 붙어
  있으면 래치를 잘못 풀 수 있음 — F_RESET 이 붙는 새 W 의 처음 50 pose (~0.25 s) 안에서만 (검토 지적, 불확실·작음). USB 는 순서 안 바뀜.
- FC 수정: F_RESET 상승 에지 또는 pose seq 가 뒤로 간 경우만 새 W. 같은 카운터로 앱이 다시 켜졌는데 첫 pose 들을 잃고 seq 도 앞으로만
  가면 (실제로는 seq 가 0 부터 다시 시작하므로 드묾) 해제 안 됨 → 안전 쪽.

## 7. phone smoke (2026-10-29 09:50–10:00, S24, APK 0.6.38 vc 49 sha 0a298ac8…, FC 없음, 폰은 책상 위 → VIO 초기화 안 됨)
수명 주기(시작·정지·NPU·카메라·USB 대기)만 본 시험이다. 정확도는 안 봤다. 스크립트: `smoke.sh` (이 폴더, Windows adb.exe 를 WSL 에서 씀).
안전 조건: 시작할 때 SKIN < 40, 실행 중 3 초마다 SKIN ≥ 45 / AP ≥ 60 / 배터리 ≤ 5 % 이면 멈춤 → 한 번도 걸리지 않음. SKIN 은 35.3–40.0, AP 는 최고 47 근처(측정 시점 기준), 충전 중 57→68 %.
| # | 조건 | 결과 (meta.json · logcat) |
|---|---|---|
| 1 | npu_loop 0, 프로필 rec60 | npu 칸 없음, bridge_exit 0, frames 3674. 화면이 꺼져 64 s 에 멈춤 (`secs 20` 이 안 걸림, 아래 참고) |
| 2 | npu_loop 1, 30 s 후 HOME | 앱 로그 "NPU 준비 중" → 0.29 s 뒤 "npu loop on"; ENN `Operator[0] (hw: 4) ENN_NPU`, warm-up XFeat 4.8 / LG 12.7 ms; 다리 "XFeat server … connected"; npu.jsonl 244 줄 (n_err 0); npu_loop "on", run_valid true, bridge_exit 0, killed_by_stop false, grace 5000 ms. HOME 입력부터 "ov stopped"(meta 다 씀)까지 0.6 s (시험 4·5 는 0.55 s·0.47 s) |
| 3 | npu_loop 1, "NPU 준비 중" 로그가 뜨자마자 폰에서 HOME | 0.11 s 뒤 "ov stopped while the NPU was starting"; meta npu_loop "cancelled: …", run_valid false; 0.28 s 뒤 늦게 뜬 서버를 끔 (npu_start.txt "late start after abandon -> server stopped"); 카메라·다리 안 켜짐 (같은 결과 2 회) |
| 4 | 3 바로 뒤 같은 프로세스(pid 같음)에서 새 액티비티 (`-f 0x10008000`) npu_loop 1 15 s | "already running" 거부 없이 정상 시작: connected, npu F 18 개, run_valid true, bridge_exit 0 |
| 5 | npu_loop 0 + `viop_udp usb`, FC 없음 | 문제 없이 대기: viop_usb.json `usb "no CDC device"`, udp_hb 383 → lo_no_port 383, writes 0, error ""; bridge_exit 0 |
매 실행 뒤 `dumpsys media.camera` Active Camera Clients `[]`, ov_bridge 프로세스 0. 다섯 번 모두 logcat 에 ANR·"Input dispatching timed out"·FATAL·Choreographer "Skipped frames" 없음.
- 플래그를 안 주고 다시 띄우면(`am start`) "task brought to front" 가 되어 같은 액티비티로 돌아온다. 그러면 mode ov 는 액티비티당 한 번만이라 새 실행이 없다 (0.6.34 설계).
- **발견 (0.6.38 이전부터, 고치지 않음)**: 큐(RunCue)가 화면에 있으면 `secs` 상한 타이머가 안 걸린다. `status.postDelayed` 를 부르는 status 뷰가 큐를 붙일 때 화면에서 떨어져서 그런 것으로 보인다 (추정, 코드 MainActivity afterStart). 시험 1 은 20 s 로 걸었지만 화면이 꺼질 때(64 s)까지 돌았다. 그래서 시험 2–5 는 N 초 뒤 HOME 으로 멈췄다. run_secs(VIO 초기화 기준)는 상관없다.
- 아직 안 본 것: FC 를 꽂은 실제 USB 중계, VIO 가 초기화된 상태의 루프 닫힘, 긴 실행·열.
