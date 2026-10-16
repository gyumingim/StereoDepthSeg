# REVIEW4 — JPEG 스트림 · VIO 안정화 · 원클릭 시작 · 웹 단계 표시 (독립 검토)

- 범위: 작업 트리의 변경 (git diff: MainActivity.java, lib_phone_stereo.py, phone_link.py, win_relay.py, scan3d.py,
  ov_client.py, ov_bridge/main.cpp, make_ov_config.py, web_server.py, web/app.js, index.html) 과 새 파일
  (serve_idle.sh, start_scanner.ps1/.bat, stop_scanner.bat, scan3d_server.vbs), 바탕화면 바로가기 2개.
- 방법: 코드 읽기만 했습니다. 실행·테스트·측정은 하지 않았습니다 (사용자 선호). 파일은 이 문서 말고는 고치지 않았습니다.
- 심각도: 상 = 기능이 안 됨 / 중 = 흔히 겪을 오동작 / 하 = 드문 경우·정보.

## 실제 버그 (심각도 순)

### B1. [상] 바탕화면 "3D 스캐너 시작" 이 서버를 못 켬 (서버가 꺼져 있을 때)
- 근거: start_scanner.ps1 의 서버 시작 줄
  `Start-Process -FilePath 'wsl.exe' -ArgumentList @('-d', $Distro, '-e', 'bash', '-lc', "bash $Repo/scan3d/serve_idle.sh")`.
  Windows PowerShell 5.1 의 Start-Process 는 배열 원소를 공백으로 이어 붙이기만 하고 따옴표를 넣지 않습니다
  (Microsoft 문서 Start-Process -ArgumentList: "parameter values contain a space ... need to be surrounded with escaped double quotes").
  그래서 실제 명령줄은 `wsl.exe -d Ubuntu-22.04 -e bash -lc bash /home/.../serve_idle.sh` 가 됩니다.
  `bash -lc bash` 는 serve_idle.sh 를 실행하지 않고, 경로는 `$0` 으로만 들어갑니다. 안쪽 bash 는 창 없는 stdin 을 읽다가 끝나거나 멈춥니다.
  결과: "서버가 60 초 안에 안 떴습니다" (창이 없어서 사용자에게는 브라우저도 안 열리는 것으로 보임).
- 언제: "3D 스캐너 끄기" 를 누른 뒤나 로그인 자동 시작이 실패했을 때. 이때가 바로 시작 아이콘이 필요한 경우입니다.
  로그인 때의 scan3d_server.vbs 는 `""...""` 로 따옴표를 바르게 넣었으므로 영향이 없습니다.
- 고치는 법: 공백 없는 원소만 넘깁니다. 예:
  `-ArgumentList @('-d', $Distro, '-e', 'bash', '-l', "$Repo/scan3d/serve_idle.sh")`.
  또는 문자열 하나로 넘기고 안쪽에 `\"…\"` 따옴표를 넣습니다.

### B2. [중] USB 를 다시 꽂아도 스트림이 영영 안 돌아올 수 있음 (adb forward 복구 누락)
- 근거: USB 가 끊기면 adb 가 그 기기의 forward(tcp:8765) 를 지웁니다. win_relay.py 는 forward 가 없어져도 죽지 않습니다.
  run_reverse 는 폰 포트 접속 실패를 로그로만 남기고 계속 돕니다. `--no-start-app` 이면 AppKeeper.after_session 이
  forward 를 다시 하기 전에 return 합니다. 그래서 forward 복구는 web_server.Helper 의 "다시 꽂음" 감지 하나에만 기대고 있습니다.
  그런데 그 감지는 순간값입니다.
  (a) 3 s 폴링 사이(끊김이 3 s 보다 짧음)에 다시 꽂히면 `prev` 가 "device" 라 감지되지 않습니다.
  (b) 다시 꽂은 순간이 마지막 중계 시작 8 s 안이면(`now >= self.relay_next` 거짓) 그 한 번의 기회가 사라집니다.
  `prev` 는 다음 루프에서 이미 "device" 이므로 다시 확인하지 않습니다.
  (c) `_relay_alive()` 는 중계가 살아 있으니 참이라 "dead" 로도 잡히지 않습니다.
  scan3d 의 quick_fails 도 발동하지 않습니다. 폰 접속 대기(connect_retry 5 s)가 3 s 를 넘어 매번 0 으로 돌아가기 때문입니다.
  P3 처럼 USB 가 자주 끊기는 폰에서는 특히 스캔 시작 직후(전송량이 큰 때)에 (b) 가 일어나기 쉽습니다.
- 고치는 법 (가장 튼튼한 쪽): win_relay.run_reverse 에서 폰 포트 접속이 실패하면(`TimeoutError`) `args.forward` 일 때
  `adb_forward` 를 다시 합니다(실패해도 SystemExit 를 잡고 계속). Helper 쪽에서는 replug 를 플래그로 래치해 두고 `relay_next` 가 지나면 처리합니다.

### B3. [중] 웹: 앱 시작 결과 ok=false 가 남으면 영상이 잘 들어와도 계속 "폰 카메라를 켜지 못했습니다"
- 근거: app.js stepFrom 은 `app.ok === false` 검사를 camOn 검사보다 먼저 합니다. app_status 는 start_app 의 결과로 한 번 정해집니다
  (scan3d 318, 493). status.txt 대기 시간을 넘기거나 adb shell 이 한 번 실패하면(이 폰은 USB 가 불안정) ok=false 가 됩니다.
  그 뒤 앱이 조금 늦게 떠서 프레임이 들어와도 app_status 는 바뀌지 않습니다. 그래서 빨간 오류 줄이 세션 내내 남습니다.
  `relay.state === '오류'` 검사도 같은 위치라 같은 문제가 있습니다.
- 고치는 법: 두 검사를 `if (!camOn)` 블록 안으로 옮깁니다(영상이 들어오면 무시).

### B4. [중] 웹: 녹화 복원(replay) 중에 단계 표시줄이 복원 진행 대신 "폰을 USB 로 연결하세요" 를 보여 줌
- 근거: stepFrom 은 usb `noadb` / `unauthorized` / `none` / `offline` 검사와 `temp_level === 'hot'` 검사를 replay 분기보다 먼저 합니다.
  복원은 PC 의 파일만 쓰므로 폰이 필요 없습니다. 그런데 폰을 뽑았거나 adb 가 없는 PC 에서는 진행률이 보이지 않습니다.
  noadb 문구는 "녹화 복원만 쓸 수 있습니다" 라고 안내하면서 정작 그 복원 진행은 가립니다.
- 고치는 법: `mode === 'replay'` 분기를 USB·온도 검사보다 위로 올립니다(복원 진행률 막대 renderProgress 는 이미 따로 그림).

### B5. [중하] 원클릭 경로가 adb 를 Claude 임시 폴더에서 찾음
- 근거: phone_link.find_adb 는 WSL 에서 `/mnt/c/Users/*/AppData/Local/Temp/claude/*/*/scratchpad/android/platform-tools/adb.exe` 를 찾습니다.
  start_scanner.ps1 Find-Adb 에는 같은 scratchpad 경로가 직접 적혀 있습니다. 실제로 찾히는 adb 는 이 Temp 폴더의 것뿐입니다.
  Temp 정리(저장소 센스, 디스크 정리)나 세션 폴더 삭제가 일어나면 웹에 "adb 를 찾지 못했습니다" 가 뜨고 스캔 시작도 앱 제어도 안 됩니다.
- 고치는 법 (사용자 작업): platform-tools 를 오래 남는 곳에 둡니다. 예를 들어 `%LOCALAPPDATA%\Android\Sdk\platform-tools`
  (이미 두 탐색 목록에 있음)나 사용자 환경변수 SCAN3D_ADB 를 씁니다. 그다음 코드의 scratchpad 경로는 지웁니다.

### B6. [중하] 항상 스테레오 + 시각차 기준 기본 16.67 ms → 사실상 거르지 않음 (정확도 위험, 충돌 아님)
- 근거: `--vio-stereo-skew-ms` 기본은 프레임 간격의 절반(30 fps → 16.67 ms)입니다. 그런데 쌍 묶기(`--pair-skew-ms`) 가 이미 10 ms 이하만 넘깁니다.
  그래서 이 거름은 하나도 버리지 않습니다(`skew_dropped` 는 늘 0). 첫 실행 로그에서는 1 ms 이하 쌍이 141 / 2350 (6 %) 이었습니다.
  지금은 두 렌즈가 최대 10 ms 어긋난 쌍이 OpenVINS 스테레오 KLT 와 삼각측량에 "동시 촬영" 으로 들어갑니다.
  예를 들어 1 rad/s 로 돌리면 10 ms 동안 약 10 mrad 돕니다. 640 px 영상(f ≈ 500 px)에서는 약 5 px 어긋남입니다(추정).
  이 어긋남은 깊이 오차와 스케일 흔들림으로 이어질 수 있습니다. 섞임 충돌(P2) 을 막는 것 자체는 맞게 고쳐졌습니다.
- 고치는 법: 첫 실행에서 stereo 와 `--vio-mode mono` 를 비교합니다(mono 는 동기화 안 된 두 렌즈에 더 안전한 기본값 후보).
  스테레오를 유지하면 기준을 쌍 묶기보다 작게(예 3~5 ms) 두되, 그때 생기는 프레임 공백이 KLT 를 끊는지 봐야 합니다.
  웹의 "렌즈 시각차" 줄이 판단 근거가 됩니다.

### B7. [하-중] 앱 기본 스트림이 jpeg 라서, 인자 없이 앱을 띄우는 옛 도구가 조용히 JPEG 을 받음
- 근거: MainActivity 는 `stream_fmt` 가 없으면 jpeg(q 90) 을 보냅니다. z_phone_stereo.py 의 am start 에는 이 인자가 없습니다.
  lib_phone_stereo 가 JPEG 도 풀어 주므로 오류 없이 돌지만, 보정 촬영(z_calibrate 의 스테레오 쌍 PNG)이 압축 영상이 됩니다.
  P4(세로 어긋남 1.16 px)를 다시 보정할 때 정확도를 떨어뜨릴 수 있습니다(추정).
- 고치는 법: 앱 기본값을 nv21 로 두고, scan3d·win_relay 처럼 PC 가 `--es stream_fmt jpeg` 를 명시적으로 요청하게 합니다(이미 그렇게 함).

### B8. [하] "3D 스캐너 끄기" 의 pkill 이 자기 부모 bash 를 죽여 뒤 단계가 안 돎
- 근거: `bash -lc "pkill -TERM -f 'scan3d/scan3d.py' ; sleep 2 ; pkill -TERM -f 'ov_bridge/build/ov_bridge' ; true"`.
  부모 bash 의 명령줄에도 'scan3d/scan3d.py' 가 들어 있어 pkill -f 가 그 bash 를 함께 죽입니다. 그래서 `sleep 2; pkill ov_bridge` 는 실행되지 않습니다.
  브리지는 scan3d 가 끝나면 stdin(pipe) EOF 로 스스로 끝나므로 실제 피해는 작습니다.
  (참고: 두 번째 패턴은 scan3d 명령줄의 `--ov-bridge .../ov_bridge/build/ov_bridge` 와도 맞습니다.)
- 고치는 법: `pkill -TERM -f 'scan3d/[s]can3d.py'` 처럼 대괄호 패턴을 써서 자기 명령줄과 맞지 않게 합니다.

### B9. [하] 브리지 "시작" 자체가 실패하면 자동 재시작 대상이 아니고, 화면은 엉뚱한 단계를 보여 줌
- 근거: `_ensure_live_vio` 가 예외를 내면(브리지 파일 없음, mono 설정 만들기 실패 등) live_vio=None 이고 `_session_main` 이 return 합니다.
  그러면 폰 스트림 접속도 하지 않습니다. `_vio_watchdog` 은 OVClient 일 때만 보므로 다시 시도하지 않습니다.
  웹은 camOn 이 거짓이라 "카메라 켜는 중… 10초쯤" 을 보여 주고, 진짜 원인(vio.error) 은 보이지 않습니다.
  vio.error 문구 "자동으로 다시 켭니다" 도 이 경우에는 사실이 아닙니다.
- 고치는 법: 단계 계산에서 `vio.error && !camOn` 이면 브리지 오류를 먼저 보여 줍니다. watchdog 이 live_vio None 도 재시도하게 하거나, VIO 없이라도 스트림은 받게 합니다.

### B10. [하] watchdog 재시작과 [처음부터 다시](start_session → _ensure_live_vio) 가 겹치면 브리지 하나가 고아로 남을 수 있음
- 근거: watchdog 은 교체를 session_lock 안에서 확인합니다. 그런데 `_session_main` 의 `_ensure_live_vio` 는 lock 없이 `self.live_vio` 를 바꿉니다.
  _ensure 가 죽은 v 를 본 뒤 watchdog 이 nv 로 바꾸고, 그다음 _ensure 가 자기 것으로 덮으면 nv 는 닫히지 않습니다.
  시간 창은 좁습니다. 고치는 법: `_ensure_live_vio` 의 교체도 session_lock 아래에서 `is` 확인 뒤 합니다.

### B11. [하] 끝없는 재시도에 backoff 가 없음
- 설정 오류 등으로 브리지가 시작 직후 계속 죽으면 약 2.5 s 마다 다시 띄우고, 그때마다 새 `runs/ov_bridge_live_<시각>.log` 가 생깁니다.
- 중계가 "오류"(Windows Python 없음 등)이면 Helper 가 8 s 마다 PowerShell 을 띄워 다시 시도합니다.
  평소에도 살아 있는지 확인하려고 약 6 s 마다 powershell.exe 를 띄웁니다(무겁지만 버그는 아님).
- 고치는 법: 연속 실패 수에 따라 2→4→8→30 s 로 늘립니다.

### B12. [하·정보] 확인 안 된 가정 두 가지
- 브리지 `--imu-hold 0.25`: 카메라 경로 지연이 0.25 s 를 넘으면 IMU 가 카메라보다 먼저 들어가 P2 의 "No IMU measurements (0 of 2)" 경고가 다시 날 수 있습니다.
  지연 요인은 폰 JPEG 인코드, PC 디코드, 정류, FFS 의 GPU 부하입니다(추정). 브리지 pose 줄의 `imu_held` 를 첫 실행 때 봅니다.
- JPEG 대역 "2~3 MB/s" 는 낮게 잡은 듯합니다. 640x480 q90 한 장은 대개 60~100 KB 이므로 60 장/s 이면 약 3.5~6 MB/s 입니다(추정, 원본의 1/5~1/8).

## 버그 아님으로 본 것 (확인한 내용)
- JPEG 헤더: STEJ 는 STER 와 같은 헤더를 씁니다. PC 는 스트림 번호, 크기, 길이(≤ 8 MB) 를 확인합니다.
  디코드에 실패하면 길이만큼 이미 읽었으므로 스트림 경계가 맞고, 건너뛴 뒤 개수만 셉니다. 옛 APK(NV21) 와 새 PC 도 맞습니다.
- 앱 스레드: 렌즈마다 단일 스레드 인코더라 렌즈별 순서가 유지됩니다. 밀린 작업은 렌즈당 최대 2개이고 넘치면 버립니다.
  nv21 배열은 프레임마다 새로 만들므로 녹화기와 인코더가 같은 배열을 써도 경합이 없습니다. stop() 의 shutdownNow 와 `running` 확인도 안전합니다.
- P2 원인: OpenVINS TrackKLT 소스에서 확인했습니다. mono 프레임은 img_pyramid_last[0] 만 바꾸고, 그 뒤 스테레오가 오면
  perform_detection_stereo/matching 이 빈 오른쪽 피라미드로 calcOpticalFlowPyrLK 를 부릅니다(levels2 단언).
  브리지(2장짜리만 받음)와 클라이언트(stereo 모드는 2장만 보냄) 양쪽에서 섞임을 막은 것이 맞습니다.
- 예외 처리 뒤 상태: make_system 이 cams_, last_imu_t_, lost_, zero_feat 를 초기화합니다.
  클라이언트는 `reason: exception` 줄을 auto 리셋으로 받아 inflight 를 비우고, scan3d 는 궤적·맵 규칙(on-vio-reset)을 탑니다.
  multi_threading_subs 기본값이 false 라 초기화 스레드는 join 됩니다. 다만 그 스레드 안의 예외는 잡히지 않으며, 이 경우는 watchdog 재시작이 맡습니다.
- `--live-bridge-args=...` 형식, run_scan.sh 와 serve_idle.sh 의 중복 실행 확인은 맞습니다.
  끄기 순서(녹화 정상 정지 → 중계 → scan3d → 앱)도 맞습니다(B8 제외).
- app.js 가 찾는 요소 id 중 index.html 에 없는 것(stepbar, cFps, chipTemp 등)은 모두 `#guide` / `#chips` / `#toast` 대체 경로로 처리됩니다.

## 10줄 요약
1. B1 [상]: 시작 아이콘의 Start-Process 인자에 따옴표가 없어 `bash -lc bash …` 가 됩니다. 끄기 뒤에는 서버가 안 켜집니다.
2. B1 고치는 법: `@('-d',$Distro,'-e','bash','-l',"$Repo/scan3d/serve_idle.sh")` 처럼 공백 없는 원소로 넘깁니다.
3. B2 [중]: USB 를 다시 꽂은 뒤 adb forward 복구가 3 s 폴링과 8 s 창에 걸리면 빠집니다. 중계는 죽지 않아 스트림이 영영 안 돌아옵니다.
4. B2 고치는 법: win_relay 가 폰 포트 접속 실패 때 스스로 adb forward 를 다시 합니다.
5. B3 [중]: 앱 시작 ok=false 와 중계 오류 검사가 camOn 보다 먼저라, 영상이 들어와도 빨간 오류가 남습니다.
6. B4 [중]: 복원 중에도 USB·온도 검사가 먼저라, 진행 대신 "폰을 연결하세요" 를 보여 줍니다.
7. B5 [중하]: adb 를 Claude Temp scratchpad 에서만 찾습니다. 정리되면 전부 멈추므로 고정 경로로 옮겨야 합니다.
8. B6 [중하]: 시각차 기준 16.67 ms 가 쌍 묶기 10 ms 보다 커서 거르지 않습니다. 최대 10 ms 어긋난 쌍이 스테레오로 들어가므로 mono 와 비교가 필요합니다.
9. B7–B11 [하]: 앱 기본 jpeg 가 옛 보정 도구에 영향, 끄기 pkill 이 자기 bash 를 죽임, 브리지 시작 실패는 재시도 안 함, 재시작 경합, backoff 없음.
10. JPEG 헤더·디코드 실패 처리, 앱 스레드, P2 원인과 섞임 차단, 예외 뒤 상태는 맞게 되어 있습니다. 실행·측정은 하지 않았습니다.
