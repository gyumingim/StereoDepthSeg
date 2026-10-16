# scan3d 독립 검토 2 — 녹화·재생·모드 통합 (2026-10-16)

범위: `Recorder.java`, `MainActivity.java`, `CaptureTimestampMatcher.java`, `recording.py`, `replay.py`, `scan3d.py`(세션·재생 부분),
`phone_link.py`, `win_relay.py`, `ov_client.py`, `ov_bridge/main.cpp`, `phone_rx.py`(수신·FrameTimingIndex), `run_scan.sh`, `setup.sh`, `pull_rec.py`, `web/app.js`(모드 버튼).
코드를 읽기만 했습니다. 실행·테스트·측정은 하지 않았습니다. 줄 번호는 현재 파일 기준입니다.

## 실제 버그 (심각도 순)

### 1. [상] 녹화 중 웹에서 다른 모드를 시작하면 녹화가 마무리 없이 죽음 (mp4 moov 없음)
- 근거: app.js 289-292 확인창은 "지금 세션을 멈추고"라고 묻지만, scan3d.py 234 `_stop_session_locked(stop_app=False)`는 앱을 멈추지 않습니다.
  그 뒤 280 `link.start_app(mode)`가 phone_link.py 155 `am force-stop`와 157 `am start -S`를 바로 실행해서 녹화 중인 앱을 죽입니다.
  인코더 EOS와 `muxer.stop()`이 돌지 않으므로 mp4는 열리지 않고, meta.json은 `complete=false`로 남습니다.
- '정지' 직후 바로 '시작'을 눌러도 같은 일이 생깁니다. 정지는 scan3d.py 270-273에서 별도 스레드로 돌아 broadcast를 보내고 최대 8 s 기다리는데, 그동안 새 세션의 force-stop이 먼저 실행될 수 있습니다.
- win_relay를 `--no-start-app` 없이 다시 띄울 때도 같습니다(win_relay.py 99, 101). 웹에서 시작한 녹화를 force-stop합니다.
- 고치는 법: `PhoneLink.start_app` 처음에 status가 `RECORDING`으로 시작하면 `stop_app()`(broadcast 후 RECORD_DONE까지 대기)을 먼저 부릅니다. 정지 스레드가 아직 돌고 있으면 그 스레드를 join한 뒤 force-stop합니다.

### 2. [중상] 재생: PTS로 맞추다가 순서 맞추기로 바꾸면 모든 프레임 시각이 약 31프레임(약 1 s) 밀림
- 근거: recording.py 143-151. PTS가 맞지 않은 프레임은 `continue`로 버리는데, 이때 `k`(다음 CSV 행)는 그대로입니다.
  31번째에 `by_pts=False`로 바뀌면, 그 뒤 디코드 프레임 n에는 행 `k ≈ n−31`이 붙습니다. 카메라와 IMU 시각이 1 s 어긋난 채 VIO로 들어가고, 아무 경고도 나오지 않습니다.
- 이 대체 경로가 대비하려던 상황, 즉 PTS가 없어 OpenCV `CAP_PROP_POS_MSEC`가 0을 주는 경우(cap_ffmpeg는 NOPTS일 때 0을 돌려줌)에 바로 이 버그가 납니다. 첫 프레임만 맞고 그 뒤로 모두 miss가 됩니다.
- 고치는 법: 순서 맞추기로 바꿀 때 `k`를 지금 디코드 번호에 맞춥니다(`k = min(len(recs), n_dec)`; 마지막으로 맞은 행 + 그 뒤에 버린 프레임 수). 더 나은 방법은 PTS를 쓸지 처음 2~3프레임에서 미리 정하는 것입니다(두 번째 `pms`가 첫 번째보다 크지 않으면 처음부터 순서 맞추기).

### 3. [중] 재생: 첫 디코드 프레임을 무조건 첫 CSV 행에 맞춤 → 첫 프레임이 빠지면 전체가 한 칸(33 ms) 밀림
- 근거: recording.py 140-142. `dec0`가 첫 디코드 PTS이고, `_pts0`는 첫 행의 pts_us입니다(121).
  인코더가 첫 입력을 내보내지 않으면(rate control의 frame skip, 첫 sample을 쓰기 전에 muxer 시작) 디코드 0번은 실제로 행 1입니다. 그래도 간격이 일정하니 모든 프레임이 `tol` 안에서 바로 앞 행과 맞아 버립니다. 그러면 시각 전체가 한 프레임 일찍 찍히고, 로그에도 남지 않습니다(통합 보고 위험 14와 같은 문제지만 탐지도 안 됨).
- 고치는 법: 첫 N(≈10) 프레임 PTS 간격 열을 CSV pts 간격 열과 대조합니다(드롭으로 생긴 간격 무늬가 맞는 오프셋을 고름). 또는 `len(recs) − n_video_frames`를 meta `samples_written`와 비교해, 다르면 경고합니다. MP4 sample 수 = `samples_written`, 행 수 = `frames`입니다.

### 4. [중] `stop_app`: status를 못 읽으면 바로 force-stop → mp4 마무리 전에 죽음
- 근거: phone_link.py 141-143. `status_raw()`는 run-as가 실패하거나 6 s 시간 초과면 `""`를 돌려줍니다. 앱이 `openFileOutput("status.txt")`로 파일을 비운 직후에 읽어도 `""`입니다(MainActivity 173).
  이때 186-188 `not st.startswith("RECORDING")`이 참이 되어 바로 break하고, 193에서 force-stop합니다. broadcast로 시작한 인코더 마무리(최대 3 s)가 잘립니다.
- 고치는 법: `RECORD_DONE`, `STREAMING`, `STOPPED`, `ERROR`처럼 명시적인 끝 상태에서만 break합니다. 빈 값이나 모르는 값이면 계속 기다립니다(그래도 BACK 후 wait_s 동안은 기다림).

### 5. [중] both 모드(RECORDING+STREAMING)에서 스트림이 몇 번 바로 끊기면 앱을 재시작해 녹화를 죽임
- 근거:
  - win_relay.py 208: `first == "RECORDING"`만 거릅니다. `RECORDING+STREAMING`은 210을 지나 `fails ≥ 3`이면 218 `start_app`(force-stop)을 실행합니다.
  - scan3d.py 365-371 `_conn_loop`도 status를 보지 않고 `quick_fails ≥ 3`이면 `link.start_app(self.mode)`를 실행합니다.
  - 폰 앱 `serve()`는 클라이언트를 하나만 받습니다(MainActivity 451). 그래서 이전 연결이 남아 있거나 WSL 쪽이 재시작되면 adb forward가 받았다가 바로 닫는 일이 실제로 생깁니다.
- 고치는 법: 두 곳 모두 status 첫 낱말의 `+` 조각에 `RECORDING`이 있으면 재시작하지 않고 로그만 남깁니다.

### 6. [하-중] 인코더 입력 대기를 출력 비우기 전에 함 → 출력이 밀리면 프레임을 버림
- 근거: Recorder.java 146-148. `dequeueInputBuffer(20 ms)`가 `drain(0)`(136)보다 먼저 돕니다. 출력 버퍼가 다 차서 입력이 풀리지 않는 코덱이면, 20 ms를 기다린 뒤 `busy++`로 그 프레임을 버립니다. 행이 없으니 재생은 맞지만, 녹화 프레임이 빠집니다.
- 고치는 법: `feed` 앞에서 `drain(0)`을 부르고, 입력이 없으면 drain과 재시도를 2~3번 돌린 뒤에 버립니다.

### 7. [하] 정지 시간 초과 때 meta.json이 `complete=true`인데 mp4는 아직 마무리 중
- 근거: Recorder.java 71-76 join 시간 초과 뒤 MainActivity 247-254가 `lensStats()`(이때 `muxerOk=false`)와 `complete=true`를 씁니다. 그 뒤 Lens 스레드가 `close()`를 마쳐도 meta는 다시 쓰지 않습니다.
  recording.py 226-227이 `muxer_ok=false` 경고를 내서 정상 녹화를 의심하게 만들거나, 반대로 실제 실패를 `complete=true`로 가립니다.
- 고치는 법: 살아 있는 Lens가 있으면 `complete=false`와 `"finishing":true`로 씁니다. 각 Lens `close()` 끝에서 meta를 다시 쓰는 콜백을 둡니다(또는 join 시간을 늘림).

### 8. [하] NV12 대체 경로가 stride, slice-height, 색 형식을 가정함
- 근거: Recorder.java 156-160. `getInputImage`가 null일 때 너비 = stride, 높이 = slice-height, NV12라고 가정합니다. Qualcomm은 slice-height를 정렬된 값(예: 512)으로 줄 수 있고, I420 코덱도 있습니다. 그러면 색이 밀리거나 녹색 화면이 됩니다.
- 고치는 법: `codec.getInputFormat()`의 `stride`, `slice-height`, `color-format`을 읽어 써 넣습니다. 지원하지 않는 형식이면 error를 기록합니다.

### 9. [하] record 모드 broadcast 정지 뒤 status에서 녹화 이름이 사라짐
- 근거: MainActivity 302-303. `stopRecording`이 `RECORD_DONE rec=… reason=broadcast`를 쓴 직후 UI 스레드가 `RECORD_DONE (capture stopped)`로 덮어씁니다(`rec=` 없음). 그 뒤 onStop이면 `STOPPED …`가 됩니다.
  웹 rec_status와 pull 보호(scan3d.py 828-830)가 녹화 이름을 잃습니다.
- 고치는 법: 303 문구에 `rec=<이름>`을 유지합니다(예: 마지막 status에 " | capture stopped"를 덧붙임).

### 10. [하] record·both 모드에서 화면이 꺼졌다 켜지면 새 녹화가 저절로 시작됨
- 근거: MainActivity 477-478. onStop에서 녹화를 마무리한 뒤 onRestart가 `recreate()`를 부릅니다. 같은 intent(mode=record)로 onCreate 294-295가 새 `rec_…` 폴더를 엽니다. 사용자는 녹화 하나로 알지만 폴더가 둘로 나뉩니다.
- 고치는 법: recreate 때는 녹화를 다시 시작하지 않습니다(intent에 `rec_started` 표시를 남김). 또는 README에 "화면 꺼짐 = 녹화 끝, 켜면 새 폴더"라고 적습니다.

### 11. [하] 브리지 cam_dropped 증가분을 항상 가장 오래된 inflight로 처리함
- 근거: main.cpp 241-243의 역행 드롭은 가장 새 카메라를 버리는데, ov_client.py 256-258은 가장 오래된 inflight를 지웁니다. 그 결과 아직 브리지 큐에 있는 카메라가 inflight에서 빠져 `max_inflight`가 잠시 느슨해집니다. `INFLIGHT_STALE_S`(2 s) 뒤에는 저절로 맞춰집니다.
- 고치는 법: 브리지 pose 줄에 `dropped_t_ns`(버린 카메라 t_ns 목록)를 넣고, 클라이언트는 그 t_ns를 지웁니다.

## 이전 리뷰(REVIEW.md 1~7) 수정 확인
- 1: 고쳐짐. ov_client.py 198-204·265-266은 콜백 예외를 잡고, main.cpp 306은 `R_CtoG`·`T` 유한성을 봅니다. frames.py 89는 lost/null을 None으로 처리합니다.
- 2: 고쳐짐. phone_rx.py 165-171은 콜백을 감싸고, 209-214는 예상 못 한 예외와 스레드 끝을 `error`로 남겨 재접속하게 합니다.
- 3: 고쳐짐. phone_rx.py 46-58은 1 s timeout과 stop을 확인하고, scan3d.py 336-343은 접속 오류를 잡아 다시 시도합니다.
- 4: 고쳐짐. 개수 30과 나이 1 s로만 자릅니다(ov_client.py 109-111).
- 5: 고쳐짐. `--ov-images`는 rect만 받습니다(scan3d.py 84).
- 6: 대체로 고쳐짐. 프레임별 FrameTimingIndex를 쓰고, 녹화는 frames CSV 값으로 덮어씁니다. 노출 값이 아직 logical result에서 오는 점(MainActivity 104)은 그대로입니다.
- 7: 고쳐짐. inflight를 t_ns 집합으로 lock 아래에서 관리하고, RESET 때 비웁니다. 남은 작은 문제는 위 11입니다.
- 9: t_state 판정은 맞습니다. OpenVINS `propagate_and_clone`이 `state->_timestamp = 카메라 시각`을 쓰므로 `|t_state − t|`로 충분하고, 두 정밀도(9자리)도 같습니다.
- 10, 11: run_scan.sh 84 기본 `--auto-reset`, win_relay AppKeeper로 고쳐짐(위 5 예외).

## 버그 아님으로 본 것
- NV21에서 인코더 Image로 채울 때 V와 U 위치(Recorder 178-180)와 수신 측 NV21 조립(MainActivity 424-426)이 맞습니다.
- frameMeta를 넣고 찾는 일은 모두 카메라 핸들러 스레드에서 합니다. timing 콜백이 matcher.result보다 먼저 넣으므로 이미지가 먼저 와도 결과가 오면 찾습니다.
- 첫 프레임 PTS가 음수이면 `lastPts=-1` 조건 때문에 0으로 바뀝니다(151). CSV에도 같은 값이 남습니다.
- 재생에서 IMU는 시각순으로 정렬되어 카메라 시각 + 0.1 s까지 먼저 들어갑니다. `wait_pose`는 `_on_pose`가 `_replay_kf`를 넣은 뒤 `pose_seen_ns`를 올리므로 경합이 없습니다.
- swap일 때는 노출 보정이 그 프레임 자신의 index를 씁니다. 실시간과 재생이 같습니다.
- setup.sh, run_scan.sh: `set -u`에서 빈 배열 처리, `${OV_ARGS:-…}` 안의 따옴표, cmake 변수 이름(OV_SRC/OV_BUILD/OV_LOCAL)이 CMakeLists와 맞습니다. tools/는 git에 있어 `cd tools`가 실패하지 않습니다.
