# scan3d 독립 검토 3: REVIEW2 수정 확인 (2026-10-16)

범위: `phone_link.py`, `win_relay.py`, `scan3d.py`(세션·접속 부분), `recording.py`, `Recorder.java`, `MainActivity.java`, `README.md`, `RECORDING_FORMAT.md`.
코드는 읽기만 했고 실행·테스트·측정은 하지 않았습니다. 줄 번호는 현재 파일 기준입니다.

## REVIEW2 항목별 확인
| # | 판정 | 근거 |
|---|---|---|
| 1 | 고쳐짐 (아래 R2 예외) | phone_link 176·188-195: lock 안에서 녹화 중이면 `_stop_app_locked`를 먼저 부르고 나서 force-stop합니다. scan3d 278-295: 정지 스레드를 join합니다. win_relay 152-157도 같습니다. |
| 2 | 고쳐짐 | recording.py 232: `kk = last_row + (dec_i - last_dec)`. PTS를 못 쓰면 처음부터 순서 방식으로 갑니다(123-127). |
| 3 | 고쳐짐 (설계한 범위 안에서) | 113-161: 간격 무늬로 r(0..4)를 고릅니다. 간격이 일정하면 0으로 두고 `samples_written`과 행 수가 다를 때만 경고합니다. |
| 4 | 고쳐짐 | phone_link 59-63·235-239, win_relay 112-141. |
| 5 | 고쳐짐 | win_relay 266-270, scan3d 384-388. 재시작 루프는 없습니다(녹화 중이면 backoff 없이 return하고 접속만 다시 시도). |
| 6 | 고쳐짐 | Recorder 169-175. |
| 7 | meta 문제는 고쳐짐, 늦은 마무리는 실제로 못 씀 | 아래 R1. |
| 8 | 고쳐짐 (아래 R6 참고) | Recorder 193-231. NV12와 I420의 stride·slice 계산이 맞습니다. |
| 9 | 고쳐짐 | MainActivity 341, 517. |
| 10 | 고쳐짐 | MainActivity 283-286, 320-327. API 28 이상에서는 onSaveInstanceState가 onStop 뒤에 불리지만, recreate() 경로에서는 항상 저장됩니다. |
| 11 | 손대지 않음 (보고와 같음) | ov_client 256-258. |

## 남은 문제 (심각도 순)

### R1. [중] 정지 시간 초과 때 앱은 끝 상태를 쓰고 PC는 바로 force-stop함: 늦은 마무리와 meta 재기록이 실행될 기회가 없음
- 근거: `stopRecording`(MainActivity 246-266)은 `done=false`여도 status에 `RECORD_DONE rec=… reason=…`를 씁니다. onStop 경로(517)도 `STOPPED …`를 씁니다.
  PC 쪽 `is_done_state`는 둘 다 끝 상태로 봅니다(phone_link 235, win_relay 137). 그래서 곧바로 `am force-stop`이 실행됩니다(phone_link 246, win_relay 157).
  그 결과 아직 EOS·`muxer.stop()` 중인 Lens 스레드가 죽습니다. `whenFinished`(259) 콜백은 실행되지 않습니다. mp4에는 moov가 없고 meta는 `complete=false, finishing=true`로 남습니다.
  ImuWriter도 `finished()`에 들어가지 않으므로(85) 흘려 쓰지 못한 imu.csv 끝부분이 잃어집니다.
- 수정 7은 meta를 정직하게 만들었을 뿐이고, 시간 초과 상황(바로 수정 7이 노린 상황)에서 파일을 살리지는 못합니다.
- 고치는 법: `done=false`이면 status를 `RECORD_FINISHING rec=…`로 씁니다(끝 상태가 아님). `whenFinished`에서 `RECORD_DONE`을 다시 쓰고, `finished()`에 imu 스레드도 넣습니다.
  PC는 이미 빈 값이나 모르는 값이면 계속 기다리므로 wait_s만 늘리면 됩니다. 아니면 force-stop 전에 meta의 `finishing`을 확인합니다.

### R2. [중하] 앱 시작 도중 '정지'를 누르면 녹화가 PC 세션 없이 계속됨 (수정 1로 그 시간 창이 최대 8 s 늘어남)
- 근거: scan3d 268은 `self.link.app_mode`가 있을 때만 정지 스레드를 띄웁니다. 그런데 `app_mode`는 두 구간에서 None입니다.
  첫째, `_start_app_locked`가 앞 녹화를 정지하는 동안입니다(`_stop_app_locked` 247에서 None, 최대 8 s). 둘째, `am start`가 돌아오기 전입니다(203).
  이때 '정지'를 누르면 정지 스레드가 생기지 않습니다. 그 뒤 start_app은 계속 진행되어 새 record/both 앱을 띄웁니다. `_session_main` 299는 `stop.is_set()`을 보고 그냥 return하므로, 웹은 idle인데 폰은 녹화 중으로 남습니다.
- 고치는 법: `_session_main`에서 `start_app` 뒤 `stop.is_set()`이면 `link.stop_app()`을 부릅니다. 또는 268의 `app_mode` 조건을 빼고 `stop_app`에 맡깁니다(이제 lock으로 직렬화되어 있음).

### R3. [하-중] status.txt에 `RECORDING`이 남은 채 앱이 이미 죽어 있으면 시작할 때마다 8 s를 기다리고 BACK 키를 보냄
- 근거: 앱이 녹화 중에 crash하거나 `adb install -r`로 죽으면 status.txt에 `RECORDING …`이 그대로 남습니다.
  `_start_app_locked`는 is_recording이 참이니 `_stop_app_locked`를 부릅니다. 그런데 일찍 끝내는 조건이 `not raw.strip() and app_running() is False`(phone_link 237, win_relay 140)라서, status가 비어 있지 않으면 확인하지 않습니다.
  broadcast는 받을 쪽이 없고, 4 s 뒤 BACK 키가 지금 앞에 있는 다른 앱이나 런처로 갑니다. 8 s가 지나서야 진행됩니다. AppKeeper 재시작에서도 같습니다(backoff 때문에 루프는 아님).
- 고치는 법: status 값과 상관없이 `app_running() is False`이면 바로 끝냅니다(처음 한 번과 BACK 직전에 확인).

### R4. [하] both 모드에서 녹화가 끝난 뒤 스트림이 바로 끊기기를 반복하면 scan3d가 앱을 재시작해 새 녹화 폴더를 엶
- 근거: 외부 broadcast로 정지하면 status가 `RECORD_DONE … | STREAMING continues`가 됩니다. 화면이 꺼졌다 켜져 재생성된 뒤에는 `STREAMING … mode=both`가 됩니다(수정 10).
  두 경우 모두 `is_recording`이 거짓이므로, scan3d 389-391은 quick_fails가 3 이상이면 `start_app("both")`로 새 녹화를 시작합니다. 사용자는 이를 알지 못합니다.
  반대로 win_relay AppKeeper는 271에서 `RECORD_DONE`이면 계속 return하므로, 앱이 정말 죽어도 다시 띄우지 않습니다.
- 고치는 법: both 모드 재시작은 녹화를 다시 열지 않는 live로 하거나, 로그만 남기고 사용자 동작을 기다립니다.

### R5. [하] `OPEN mode=both|record`를 녹화 중으로 보는 규칙이 오판할 수 있음
- 수정 10으로 재생성될 때 both 모드는 녹화 없이 `OPEN mode=both`를 씁니다(MainActivity 354). 카메라 열기가 멈춰 OPEN에 머무르면 is_recording이 계속 참입니다.
  그러면 start_app은 매번 8 s 정지 대기를 하고, 재시작은 막힙니다. 오판은 안전한 쪽(재시작 안 함)이라 하로 둡니다.
- 고치는 법: OPEN 줄에 `rec=`를 넣고(녹화를 실제로 열었을 때만), is_recording은 `rec=`가 있을 때만 참으로 봅니다.

### R6. [하] NV12 대체 경로: Flexible로 요청했을 때 getInputFormat이 업체 전용 색 형식을 돌려주면 그 렌즈 녹화 전체가 실패함
- 근거: Recorder 199-209. `getInputImage`가 null인 코덱에서 `color-format`이 Qualcomm 전용 값(예: 0x7FA30C04) 등으로 오면 예외가 나고, Lens가 끝납니다(161). 전에는 색만 어긋났는데 이제 렌즈 영상이 통째로 없습니다.
  또 206은 Flexible이면 프레임마다 Log.w를 남깁니다(`error==null`이 늘 참). 대체 경로 자체가 드물어서(getInputImage null) 하로 둡니다. 기기에서는 확인하지 않았습니다(추정).

### R7. [하] 문서가 바뀐 동작과 다름
- README 6·185: 아직 "앱 1.1-rec (versionCode 2)"입니다. 매니페스트는 3 / 1.2-rec입니다.
- RECORDING_FORMAT 146: `dropped_encoder_busy`를 "20 ms"로 적었는데, 지금은 drain 뒤 10 ms를 최대 3번 기다립니다.

### R8. [하] REVIEW2 11(브리지 inflight 드롭 대상) 그대로입니다. 2 s 뒤 저절로 맞춰집니다.

## 버그 아님으로 본 것
- 재생 정렬: `_decide`와 `iter_frames`는 같은 기준(dec0, 행 r0의 pts)을 씁니다. 순서 방식으로 바꿀 때 `skipped_rows += kk - k`는 음수가 되지 않습니다. 드롭 간격이 있으면 r≠참값의 hit가 줄어서 바르게 가립니다.
- `whenFinished`와 `lensClosed`의 경합: 둘 다 같은 lock 안에서 개수를 확인하므로 콜백을 잃지 않습니다(R1처럼 프로세스가 죽는 경우는 별개).
- `fillBuffer` 크기: `need ≤ capacity`를 확인한 뒤 `max(min(cap, stride·slice·1.5), need)`를 돌려주므로 capacity를 넘지 않습니다.
- lock 순서: 정지 스레드와 세션 스레드가 lock을 잡는 순서가 join으로 정해집니다. 그래서 '정지' 직후 '시작'을 눌러도 force-stop이 마무리 대기를 앞지르지 못합니다.
- 재시작 루프: relay와 scan3d 모두 녹화 중이면 start_app을 부르지 않고, 다른 경로에는 backoff나 quick_fails 초기화가 있습니다.

## 10줄 요약
1. REVIEW2 1–10은 코드로 반영되었습니다. 11은 보고대로 그대로입니다.
2. 재생 정렬(2·3)은 맞게 고쳐졌고, 정렬 쪽에서 새 버그는 찾지 못했습니다.
3. R1 [중]: 정지 시간 초과에도 앱이 RECORD_DONE·STOPPED를 써서 PC가 바로 force-stop합니다. 늦은 마무리와 meta 재기록이 실행되지 못합니다.
4. R1 고치는 법: 시간 초과면 `RECORD_FINISHING`을 쓰고 whenFinished에서 RECORD_DONE으로 바꿉니다. imu 스레드도 finished에 넣습니다.
5. R2 [중하]: 앱 시작 중(app_mode=None 구간, 최대 8 s)에 '정지'를 누르면 폰 녹화가 고아로 남습니다.
6. R3 [하-중]: status에 RECORDING이 남은 채 앱이 죽어 있으면 시작마다 8 s를 기다리고 BACK 키가 다른 앱으로 갑니다.
7. R4 [하]: both 모드에서 녹화가 끝난 뒤 스트림 재시작이 새 녹화 폴더를 조용히 엽니다.
8. R5 [하]: `OPEN mode=both`를 녹화로 보는 규칙이 녹화 없는 재생성이나 멈춘 카메라에서 오판합니다(안전한 쪽).
9. R6 [하]: 대체 입력 경로가 업체 전용 색 형식이면 렌즈 녹화가 통째로 실패하고, 프레임마다 경고 로그가 남습니다.
10. R7 [하]: README 버전(1.1-rec)과 RECORDING_FORMAT의 "20 ms"가 낡았습니다. 기기에서의 동작은 확인하지 않았습니다.
