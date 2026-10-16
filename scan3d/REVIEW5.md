# REVIEW5 — REVIEW4 수정 · 지연 로드 · 카메라 창 · 미리보기 서버 (독립 검토)

- 범위: scan3d.py, web_server.py, win_relay.py, phone_link.py, start_scanner.ps1, serve_idle.sh, web/app.js, web/index.html,
  preview_server.py, README.md, lib_phone_stereo.py(fr.jpeg 한 줄), replay.py(모델 대기 부분).
- 방법: 코드 읽기만 했습니다. 실행·테스트·측정은 하지 않았습니다. 이 문서 말고는 파일을 고치지 않았습니다.
- 심각도: 상 = 기능이 안 됨 / 중 = 흔히 겪을 오동작 / 하 = 드문 경우·정보.

## REVIEW4 항목 확인

| 항목 | 판정 | 근거 |
|---|---|---|
| B1 시작 스크립트 인자 | 고쳐짐 | `start_scanner.ps1:73` 원소에 공백 없음 → `wsl.exe -d Ubuntu-22.04 -e bash -l /home/.../serve_idle.sh` |
| B2 USB 다시 꽂음 | 고쳐짐 | `win_relay.py:344~353`(reverse), `:380~389`(listen) 폰 포트 접속 실패 → `try_forward` + 늘어나는 대기. 시작 때 forward 실패해도 계속(`:426`). `web_server.py:369` transport_id 변화, `:401` 래치, `:420` relay_next 뒤 처리 |
| B3 앱·중계 빨간 줄 | 고쳐짐 | `app.js:231` `!camOn && !recOn` 안으로 옮김 |
| B4 복원 단계 우선 | 고쳐짐 | `app.js:202` replay 분기가 USB·온도보다 위 |
| B5 adb 경로 | 코드는 고쳐짐 (문서·도움말 일부 남음, 아래 N4) | `phone_link.py:39,85`, `win_relay.py:56,61`, `start_scanner.ps1:19` |
| B6 시각차 기준 | 값은 맞춤 (새 위험 있음, 아래 N1) | `scan3d.py:84,144`, 화면 `app.js:377~379` |
| B7 앱 기본 jpeg | 안 고침 (APK 재빌드 필요, 보고대로) | — |
| B8 pkill 자기 bash | 고쳐짐 | `start_scanner.ps1:51` `[s]`·`[o]` 패턴은 자기 bash 명령줄과 안 맞음 |
| B9 브리지 시작 실패 | 고쳐짐 | `scan3d.py:351~353` 스트림은 받음, `:459~465` watchdog 이 `bridge_fails>0` 일 때 다시 시도, `app.js:246` 원인 먼저 |
| B10 브리지 고아 | 고쳐짐 | `scan3d.py:432~445` lock 안 `is` 확인 후 교체, 진 쪽은 닫음. watchdog(`:497~501`)과 둘 다 같은 규칙이라 어느 순서로 겹쳐도 하나만 남음 |
| B11 backoff | 고쳐짐 | 브리지 2→4→8→16→30 s(`:448~450`), 중계 8→16→32→60 s(`web_server.py:331`), 살아 있음 확인 12 s |

질문별 확인:
- 지연 로드 경로: live/both = `start_session:278`, replay = `start_replay:616`(가져와서 복원 `_pull_and_replay` → `start_replay` 포함), 명령줄 `--mode live`/`--replay` = `start()` → 같은 함수, [처음부터 다시] = `start_session`. 빠진 경로 없음.
  중복 로드 없음: `depth_lock` + `depth_thread.is_alive()` + `ready` 확인. 성공한 깊이 스레드는 끝나지 않으므로 다시 안 띄움. 초기화 실패 때만 다음 시작에서 다시 시도(의도대로).
  복원은 `replay.py:83` 에서 모델 준비를 기다리므로 키프레임을 잃지 않음. 실시간 첫 세션은 아래 N2.
- `/api/cam.jpg` 동시성: 문제 없음. `rx.latest[i]` 는 리스트 원소 하나 읽기이고, 프레임 객체·`bgr`·`jpeg` 버퍼는 프레임마다 새로 만들어 바뀌지 않음
  (`lib_phone_stereo.read_frame` 의 `recv_exact` 는 매번 새 bytearray). overlay 의 `cv2.remap` 은 읽기만 함. 렌즈 번호·swap 규칙도 `on_pair` 와 같음.
- 웹 갱신 멈춤: 카메라 창은 숨김·탭 숨김·idle/record 때 요청 안 함(`app.js:502`), 앞 요청이 안 끝나면 건너뜀. 깊이 화면도 숨김·탭 숨김·idle 때 멈춤.
  상태·복셀·궤적 폴링은 탭이 숨겨져도 계속 돎(이전부터, 가벼움 — 버그 아님).
- preview 서버 필드: 웹이 읽는 필드는 모두 있음(vio/phone/depth/session/adb/ui.relay/map/objects/classes/settings, 궤적 points·cam, 녹화 목록 형식). 추가 필드 `preview`, `ui.fake` 만 다름. 실제 서버엔 `preview` 가 없어 띠가 숨겨짐.
- 아이콘: `.lnk`·`CreateShortcut`·Desktop 을 만드는 코드 없음. 로그인 자동 시작 `scan3d_server.vbs` 만 남음(의도대로).

## 실제 버그·위험 (심각도 순)

### N1. [중] 쌍 묶기 기준 5 ms — 쌍이 크게 줄거나 0 이 될 수 있는데 화면이 원인을 말하지 않음
- 근거: `--pair-skew-ms` 기본이 10 → 5 ms 가 되면서 `FramePairer`(lib_phone_stereo) 가 5 ms 넘게 어긋난 프레임을 짝 없이 버립니다.
  두 렌즈 노출 동기는 APPROXIMATE 이고, 첫 실행에서 1 ms 이하 쌍이 6 % 였습니다(REVIEW4 B6). 어긋남이 프레임 간격(33 ms) 안에 고르게 퍼져 있으면
  5 ms 이하는 대략 30 % 안팎입니다(추정). 두 센서 위상차가 한동안 5 ms 넘게 고정되면 쌍이 거의 0 이 됩니다.
  쌍이 없으면 VIO 에 카메라가 안 들어가 `vio.init` 이 안 되고, 깊이·맵도 멈춥니다.
- 화면: camOn(fps>0) 은 참이고 `pairs>150` 검사도 안 걸리므로 단계 표시줄은 "폰을 2초 가만히 두세요" 에 계속 머뭅니다(`app.js:258`).
  원인(짝 없어 버림)은 고급 → "렌즈 시각차" 줄에만 보입니다.
- 범위: 녹화 복원도 `rec.iter_pairs(sc.a.pair_skew_ms)`(`replay.py:96`) 를 쓰므로 이미 찍은 녹화의 쌍 수도 줄어듭니다.
- 고치는 법: (1) stepFrom 에 "영상은 오는데 쌍 비율이 낮음"(예: `ph.pairs / ph.frames[0] < 0.3`, 또는 `ph.dropped` 급증) 경고를 넣어
  `--pair-skew-ms 10` 또는 `--vio-mode mono` 를 안내합니다. (2) 첫 실행 로그로 쌍 비율을 확인하기 전까지 복원 기본은 10 ms 로 둡니다.

### N2. [하-중] 첫 실시간 세션: 모델을 불러오는 약 30 s 동안의 키프레임은 하나만 남고 나머지는 버려짐
- 근거: `_ensure_depth` 는 [스캔 시작] 순간에 로드를 시작하지만, VIO 초기화(수 초)가 먼저 끝나므로 "첫 키프레임 전에 준비" 는 첫 세션에서는 안 됩니다.
  로딩 중 `_on_pose`(`scan3d.py:796`) 는 큐에 하나가 있으면 새 키프레임을 만들지 않습니다. 그래서 첫 키프레임 하나(이미 지난 장면)만 로드 뒤 처리되고,
  그동안 움직이며 비춘 부분은 맵에 안 들어갑니다. 웹 안내("그동안 위치만 추적합니다")는 맞게 나옵니다. 두 번째 세션부터는 해당 없음.
- 고치는 법(선택): 웹 첫 접속이나 USB 기기 감지 때 미리 로드하거나, 로딩 중에는 안내를 "모델 준비가 끝나면 움직이세요" 로 바꿔 사용자가 기다리게 합니다.

### N3. [하] 카메라 창·깊이 화면이 지난 세션(또는 복원)의 그림과 물체 상자를 계속 보여 줌
- 근거: `self.last_pair` 와 `self.panel` 은 `_reset_pipeline`(`scan3d.py:574~602`)에서 비우지 않습니다.
  - 새 실시간 세션에서 폰이 붙기 전, 또는 다시 접속 중(`self.receiver` 가 None)에는 `cam_jpeg`(`:1015~1018`) 가 `last_pair` 로 넘어가
    멈춘 옛 프레임을 200 으로 보냅니다. 그래서 창에 "영상 없음" 대신 옛 그림이 보이고, 끊김이 화면에서 감춰집니다.
  - "물체 표시" 는 첫 키프레임 전까지 지난 세션의 YOLO 마스크·상자를 새 영상 위에 겹칩니다(`:1022~1023`).
  - `poll2d` 는 `sessMode !== 'idle'` 이면(record 포함) 지난 세션의 깊이 그림을 계속 받아 옵니다(`app.js:474`).
- 고치는 법: `_reset_pipeline` 에서 `self.last_pair = None`, `with self.panel_lock: self.panel = {}`.
  실시간 모드에서는 `rx` 가 없으면 `last_pair` 로 넘어가지 말고 None(204)을 돌려줍니다. poll2d 는 live/both/replay 일 때만.

### N4. [하] adb 탐색 순서가 파일마다 다르고, 옛 안내·Temp 경로가 남아 있음
- `start_scanner.ps1:19` 는 고정 경로가 SCAN3D_ADB 보다 앞이고, `phone_link.find_adb`·`win_relay.adb_exe` 는 SCAN3D_ADB 가 앞입니다.
  SCAN3D_ADB 를 버전이 다른 adb 로 두면(WSL 은 WSLENV 로 넘겨야 보임) 두 adb 클라이언트가 서로의 adb 서버를 죽이고 다시 띄워
  ("adb server version doesn't match") 중계 forward 가 지워질 수 있습니다. 순서를 하나로 맞추는 것이 안전합니다.
- 옛 안내: `scan3d.py:70` `--adb` 도움말, `scan3d.py:19`, `pull_rec.py:9`, `run_scan.sh:19` 에 고정 경로 순서가 없습니다.
  `SPEC.md:290` 에는 아직 Temp scratchpad adb 경로가 적혀 있습니다(코드가 쓰지는 않음).

### N5. [하] 브리지 "다시 띄우기" 가 실패하면 화면에는 옛 오류가 보임
- 근거: watchdog 의 `_make_live_vio` 실패(`scan3d.py:490~496`) 때 `self.live_vio`·`self.vio` 는 닫힌 옛 OVClient 그대로입니다.
  `state()`(`:927`) 는 `self.vio.stats()["error"]`(옛 오류, 예: 프로세스 종료)를 `vio_error`("다시 띄우기 실패: …")보다 먼저 씁니다.
  그래서 `app.js:246` 의 "시작 실패|다시 띄우기 실패" 검사가 안 걸리고, 진짜 원인(예: 설정 파일 없음)이 상태 줄에 안 나옵니다.
  스트림이 오는 동안(camOn)에는 "다시 켜는 중" 안내가 나오므로 영향은 작습니다.
- 고치는 법: `state()` 에서 `vio_error` 를 우선하거나, 둘 다 보여 줍니다.

### N6. [하·기존] 실시간에서 키프레임 오류가 한 번 나면 그 뒤 복원이 바로 실패할 수 있음
- 근거: `_process_keyframe` 오류는 `dstats["error"]` 에 남고, 다음 성공 키프레임 때만 지워집니다.
  복원은 `replay.py:83~86` 에서 iters 가 다르면(실시간 4 ≠ 복원 8) 루프에 들어가 `error` 를 먼저 보고 "깊이 모델 오류" 로 끝납니다.
  `_ensure_depth` 는 모델이 이미 준비돼 있으면 일찍 돌아가므로 오류를 지우지 않습니다. 이번 변경으로 생긴 것은 아니지만 지연 로드 경로와 맞닿아 있습니다.
- 고치는 법: `start_replay` 에서 `ready` 이면 `dstats["error"] = None`, 또는 replay 대기에서 `init:` 으로 시작하는 오류만 보도록 합니다.

## 버그 아님으로 본 것
- `lib_phone_stereo.PhoneFrame` 은 slots 없는 dataclass 라 `fr.jpeg` 를 붙여도 됩니다. 메모리는 `recent` 90쌍 기준 약 +14 MB(추정).
- `/api/cam.jpg` overlay: YOLO 마스크는 `retina_masks=True` 로 정류 왼쪽 640x480 크기이고, `cam_jpeg` 도 `rp["mapL"]` 로 같은 정류를 하므로 좌표가 맞습니다.
- Helper: 래치가 `want_relay` 꺼지면 지워지고, "켜는 중" 동안은 유지됩니다. 곧 죽는 중계는 연속 실패로 세고 간격을 늘립니다.
- win_relay: forward 실패를 SystemExit 로 잡아 계속 돌고, 폰이 빠진 동안에는 1→8 s 간격으로 다시 시도합니다.
- preview_server: `--web` 경로, PNG 대체, 가짜 온도 "(가짜)", 띠 표시가 맞습니다. 실제 서버(8795)와 포트가 겹치지 않습니다.
- `web/OWNER.txt` 해시가 현재 index.html·app.js 와 같습니다.

## 10줄 요약
1. REVIEW4 의 B1·B2·B3·B4·B8·B9·B10·B11 은 코드상 고쳐졌습니다. B5 는 코드만, B7 은 보고대로 안 고쳤습니다.
2. 지연 로드는 live/both/replay/명령줄/[처음부터 다시] 모든 경로에서 불리고, lock 과 is_alive 확인으로 중복 로드가 없습니다.
3. /api/cam.jpg 는 바뀌지 않는 프레임 객체만 읽으므로 동시성 문제가 없고, 웹 갱신은 숨김·탭 숨김·정지 때 멈춥니다.
4. preview 서버는 웹이 읽는 필드를 모두 주고, 아이콘(.lnk)을 만드는 코드는 남아 있지 않습니다.
5. N1 [중]: 쌍 묶기 5 ms 로 쌍이 크게 줄거나 0 이 될 수 있습니다. 그때 화면은 "2초 가만히" 에 머물러 원인을 말하지 않습니다(복원에도 영향).
6. N2 [하-중]: 첫 실시간 세션은 모델 로딩(약 30 s) 동안 키프레임이 하나만 남습니다. 첫 키프레임 전에 준비되지 않습니다.
7. N3 [하]: last_pair·panel 을 안 비워 카메라 창·물체 상자·깊이 화면에 지난 세션 그림이 남고, 다시 접속 중 끊김이 감춰집니다.
8. N4 [하]: adb 순서가 ps1 과 Python 에서 다르고, 도움말·SPEC.md 에 옛 경로(Temp 포함)가 남았습니다.
9. N5·N6 [하]: 브리지 다시 띄우기 실패 원인이 옛 오류에 가려지고, 남은 키프레임 오류 때문에 복원이 바로 실패할 수 있습니다(기존).
10. 실행·측정은 하지 않았습니다. N1 은 첫 실행 로그의 쌍 비율(고급 → 렌즈 시각차)로 확인해야 합니다.

## 고친 기록 (2026-10-16, 검토 본문은 그대로)
- 고침: N1 — scan3d.state() 가 최근 10초 "짝 없어 버린 비율"(phone.pair_drop_pct)을 내고, 웹 단계 표시줄이 50 % 넘으면 "두 렌즈 시각 차이가 커서 짝이 안 맞음 — 기준 N ms, 버린 비율 X %" 를 보여 줌(추적 준비 전이거나 80 % 넘으면 빨간 줄). 기준은 자동으로 넓히지 않고 `--pair-skew-ms 10 --vio-stereo-skew-ms <지금 값>` 안내만(VIO 기준 유지). 고급 → 렌즈 시각차 줄에도 최근 비율 표시. 복원 기본값은 바꾸지 않음.
- 고침: N2 — dstats 에 load_t0·load_stage·load_s 추가, 웹이 첫 실시간 세션 로드 동안 "깊이 모델을 불러오는 중… N초 — 준비가 끝나면 움직이세요" 와 단계(모델 파일 읽는 중 / GPU 준비)를 보여 줌(복원 대기에도 경과 초).
- 고침: N3 — _reset_pipeline 에서 last_pair = None, panel = {}; cam_jpeg 는 복원 때만 last_pair 로 넘어가고 실시간에서 프레임이 없으면 204; poll2d 는 live/both/replay 때만.
- 고침: N5 — state() 가 실시간 모드에서 vio_error("다시 띄우기 실패: …")를 옛 브리지 오류보다 먼저 쓰고 옛 오류는 "(이전 오류: …)" 로 붙임.
- 고침: N6 — start_replay 가 모델이 준비돼 있으면 지난 키프레임 오류를 지우고, replay.py 대기는 "init:" 으로 시작하는 오류(모델 초기화 실패)에서만 멈춤.
- 안 고침: N4 (이번 범위 밖, adb 순서·옛 문구 그대로).
