# REVIEW7 — H.264 실시간 스트림 (앱 1.4-h264) 독립 검토

2026-10-16. 코드만 읽고 검토했습니다. 실행·측정은 하지 않았습니다. 파일은 이 문서 말고는 고치지 않았습니다.
읽은 파일: `H264.java`, `LiveEncoder.java`, `Recorder.java`, `MainActivity.java`(H.264·serve·명령 부분), `Caps.java`(autoKbps·findAvcEncoder),
`lib_phone_stereo.py`, `phone_rx.py`, `win_relay.py`(start_app·bridge·reverse/listen), `phone_link.py`, `scan3d.py`(set_stream·state·_pair_drop),
`web_server.py`(_apply_stream·부하 갱신), `load_model.py`(predict·combo·enc_need·live_kbps·LoadMonitor), `web/app.js`(sizeMismatch).

## 문제없다고 본 것
- **인코더 설정:** `H264.format`이 `KEY_MAX_B_FRAMES=0`(API 29+), `KEY_I_FRAME_INTERVAL=1`, `KEY_PRIORITY=0`을 넣습니다. `KEY_LATENCY=1`은 API 30 이상의 실시간 경로에서만 넣고, `KEY_LOW_LATENCY`는 `FEATURE_LowLatency`가 있을 때만 넣습니다. 프로파일은 High(B 0) → Baseline → 지연 키 없는 Baseline → 인코더 기본 순서로 시도하고, 시도 사이에 `reset()`합니다. 비트레이트 모드는 VBR이 되면 VBR, 아니면 CBR이고 그 이유를 주석에 적었습니다. 인코더 목록(`Caps.avcEncoders`)은 하드웨어를 먼저 둡니다. 비트레이트는 `4000 × 픽셀률 / (640·480·15)`이고 인코더 범위 안으로 자릅니다. 요구한 대로입니다.
- **센서 시각:** `LiveEncoder.feed`는 pts를 `(ts − t0)/1000`로 정하고 겹치면 +1 합니다. 출력에서는 `pending.remove(pts)`로 **원래 ns 값**을 다시 찾아 STEH 헤더에 넣습니다. 그래서 µs로 자를 때 생기는 오차가 없습니다. 녹화를 함께 쓰는 경로(`Recorder.ptsMeta`)도 같은 방식입니다.
- **SPS/PPS:** `frameBytes`가 모든 키프레임 앞에 csd를 붙입니다(이미 있으면 그대로). flags는 bit0 = 키프레임, bit1 = SPS 있음이고, PC의 `H264_FLAG_KEY`·`H264_FLAG_SPS` 해석과 같습니다. PC의 `_sps_pps` 저장과 "SPS가 빠지면 앞에 붙이기"도 맞습니다.
- **키프레임 요청:** PC가 보내는 `keyframe <렌즈>\n`은 앱 `command()`가 읽는 형식과 같습니다. 요청 간격은 렌즈별 0.5 s로 제한합니다. 새 클라이언트가 붙으면 앱이 `waitKey`를 모두 켜고 `requestKeyframe(-1)`을 부릅니다. `win_relay.bridge`는 양방향이고, reverse 모드의 `b"G"`는 `wait_cmd`가 먼저 읽어 버리므로 폰까지 가지 않습니다.
- **키프레임 전 P 버리기:** PC `need_key` 상태에서 키프레임이 아닌 패킷은 버리고 센 뒤 키프레임을 요청합니다. 키프레임이 오면 디코더 컨텍스트를 새로 만듭니다. 디코딩 예외·`is_corrupt`·4장 넘게 출력이 없음 → `_fail` → 키프레임 대기로 돌아갑니다.
- **쌍 묶기·OpenVINS 시각:** 디코딩해서 나온 `out_ts`는 센서 ns이고 PhoneFrame 시각으로 그대로 씁니다. 역행 폐기와 FramePairer는 그대로라 JPEG·NV21과 같은 시각 기준입니다.
- **부하 모델:** h264 USB = `2·live_kbps·여유`, PC 수신 = 디코딩 + 정류입니다. `enc_need`는 both + h264이고 `caps.live_h264.shared_with_record`(앱이 true로 씀)가 있으면 인코더 2개로 봅니다. 앱 필드 이름 `live_encoder`·`live_kbps_default`·`h264_kbps`·`h264_enc_ms_*`·`copy_ms_avg`도 맞습니다.
- **REVIEW5/6 고침 중 확인한 것:** R2(실측을 1/3~3배로 자름, 처음 a ≤ 0.5, load_only), R4(fps_req·STOPPED 상태 확인), R5(`dstats["size"]` 비교), R10(정규식이 `Invalid H.264 packet: stream=N WxH`와 맞음), N1(`_pair_drop`이 10 s 창으로 계산하고 수신기가 바뀌면 다시 셈)은 맞게 고쳐졌습니다. N2·N3·N5·N6·R1·R3·R6·R8·R9는 이번에 자세히 보지 않았습니다.

## 실제 버그·위험 (심각도 순)

### H1. [중] 참조 프레임을 잃은 P 프레임이 PC 디코더로 들어갈 수 있음 — PC는 seq가 건너뛰어도 세기만 함
- PC `H264Decoders.decode`는 seq가 건너뛰면 `seq_gaps`만 늘리고 그대로 디코딩합니다("앱이 이미 다음 키프레임까지 P를 안 보낸다"고 가정). 그런데 앱에서 이 가정이 깨지는 길이 두 개 있습니다.
  1. `MainActivity.offerFrame`: 큐가 차면 `old=q.poll(); q.offer(packet);`를 한 **다음에** `waitKey.set(old.index,1)`을 합니다. 그 사이에 `serve` 스레드가 같은 렌즈의 다음 P(`old` 바로 뒤 프레임)를 꺼내면, 그때는 `waitKey`가 아직 0이라 그대로 보냅니다. 그 P의 참조 프레임은 방금 버려진 `old`입니다.
  2. `LiveEncoder.drain`·`Recorder.drain`: `pending`/`ptsMeta`에서 pts를 못 찾으면(맵 32개가 넘쳐 밀려남, 인코더가 pts를 바꿈) 그 출력을 로그만 남기고 버립니다. 이때 **seq를 올리지 않고** `waitKey`도 켜지 않습니다. 그래서 PC는 빠진 것조차 모르고, 버려진 것이 P 프레임이면 다음 키프레임까지 참조가 틀린 채로 디코딩합니다.
- 결과: FFmpeg는 frame_num이 비면 숨김 처리를 하는데, `is_corrupt`가 늘 켜진다는 보장은 없습니다. 그러면 깨진 영상이 최대 1 s(다음 IDR까지) 동안 FFS·OpenVINS로 들어갑니다. 특징점 추적이 틀어지고 깊이에 줄무늬 같은 잡음이 생깁니다.
- 고칠 방향: (PC) seq가 건너뛰었고 그 패킷이 키프레임이 아니면 `reset(i)` 후 `request_key(i)`를 합니다. 앱이 정상적으로 키프레임까지 건너뛴 경우에는 다음 패킷이 키프레임이라 해가 없습니다. (앱) `waitKey.set`을 `poll` 앞으로 옮기거나 큐와 같은 lock 안에서 합니다. 인코더 출력을 버릴 때는 `seq++`를 하고 그 렌즈의 `waitKey`도 켭니다.

### H2. [중하] 실시간 인코더가 죽어도 폰 상태는 STREAMING이고 PC는 원인을 보여 주지 않음
- `LiveEncoder.Lens.run`은 configure나 codec에서 예외가 나면 `error`만 적고 스레드를 끝냅니다. 그 뒤 들어오는 프레임은 큐가 차서 `h264Drop`으로만 셉니다. 오류는 `metrics.json`의 `h264.error`에만 들어가고, `status.txt`는 그대로 `STREAMING`입니다.
- PC 쪽(`web_server`, `scan3d.state`, `app.js`)은 `perf.h264.error`를 읽지 않습니다. `phone_link`/`win_relay`의 `state_ok`도 통과합니다. 그래서 사용자는 "영상 없음"만 보고 원인은 알 수 없습니다. 다시 붙어도 같은 결과입니다.
- 고칠 방향: 앱에서 렌즈 오류가 나면 `status("ERROR LIVE_ENCODER …")`를 씁니다. 또는 PC가 `perf.h264.error`를 단계 표시줄에 보여 줍니다(예: "폰 영상 압축기가 멈춤 — 고급에서 JPEG로 바꿔 보세요").

### H3. [하] 녹화+실시간에서 녹화 정지가 시간 초과로 끝나면 두 인코더의 출력이 한 렌즈 스트림에 섞임
- `stopRecording`은 `rec.stop(3000)`이 `done=false`(인코더가 아직 마무리 중)여도 바로 `LiveEncoder`를 새로 만듭니다. 옛 Recorder의 lens 스레드는 계속 `endOfStream` → `drain` → `live.h264(...)`를 부르고, 공유 sink(`h264Sink`)는 출처를 가리지 않고 받습니다.
- 그러면 `waitKey`는 먼저 온 키프레임(새 인코더 IDR이든 옛 인코더 프레임이든)으로 풀립니다. 그 뒤로 옛 인코더의 P와 새 인코더의 프레임이 같은 렌즈에 번갈아 들어갑니다. PC 디코더는 다른 SPS·다른 참조 체인을 받게 되고, 시각이 뒤로 가는 프레임은 `reordered`로 버려집니다.
- 고칠 방향: 새 LiveEncoder를 만들기 전에 Recorder의 live sink를 떼어 냅니다(`volatile` 필드를 null로). 또는 sink 패킷에 출처 세대 번호를 붙이고 옛 세대는 버립니다. 덧붙여 both 모드에서는 앱이 웹의 `live_kbps`를 무시하고 `recKbps`를 씁니다. 그런데 부하 모델의 USB 예상치는 `live_kbps`로 계산합니다(두 값이 다르면 표시가 어긋납니다).

### H4. [하] 크기만 바꾸는 경로에서 지난 `live_kbps`가 그대로 남음
- `scan3d.set_stream`은 `stream_fmt`나 `live_kbps`가 들어올 때만 `link.live_kbps`를 바꿉니다. `start_session(size=…)`(scan3d 자체 API의 `body.size`, CLI 경로)은 둘 다 넘기지 않습니다. 그래서 전에 웹에서 1280x960(약 16 Mbps)을 고른 뒤 이 경로로 640x480을 시작하면, 앱이 640x480에 16000 kbps를 받습니다.
- 고칠 방향: 크기나 fps가 바뀌었는데 `live_kbps`가 주어지지 않았으면 `link.live_kbps=None`으로 둡니다(앱 자동값을 쓰게).

### H5. [하] B-frame이 정말 0인지 출력에서 확인하지 않음
- `configure`가 돌려주는 `max_b_frames: 0`은 **요청한 값**입니다. 일부 벤더 High 프로파일에서는 `KEY_MAX_B_FRAMES`가 힌트로만 다뤄집니다. PC는 `flags=low_delay`로 B가 없다고 가정하고, 녹화 복원의 PTS 맞추기도 같은 가정을 합니다. B가 섞이면 PC에서는 `pts_mismatch`만 늘고 아무 조치가 없습니다.
- 고칠 방향: LiveEncoder·Recorder 출력에서 `presentationTimeUs`가 바로 앞 출력보다 작으면 B-frame이 있다고 봅니다. 그때 `metrics`·`meta.json`에 `b_frames_seen=true`를 적고, 실시간이면 Baseline으로 다시 configure합니다. PC는 `pts_mismatch`가 늘어나면 경고를 띄웁니다.

### H6. [하·UX] H.264 디코더 상태가 웹에 하나도 안 나옴
- `PhoneReceiver.stats()["h264"]`(waiting_key, corrupt, decode_errors, seq_gaps, keyreq_sent)는 `scan3d.state()`의 `phone` dict에 들어가지 않습니다. 키프레임을 계속 기다리거나 깨진 프레임이 이어져도 화면에는 "영상 없음"이나 짝 버림 비율만 보입니다. 이때 N1 문구가 원인을 "두 렌즈 시각 차이"로 잘못 짚을 수 있습니다.
- 고칠 방향: `phone.h264 = {waiting_key, corrupt, seq_gaps, decode_ms}`를 넘기고, 고급 화면에 한 줄로 보여 줍니다.

(따로 확인했지만 버그가 아니었던 것: `H264Decoders.stats()`는 락 없이 deque를 읽습니다. 그래도 `sum`/`max`는 C 안에서 한 번에 돌기 때문에 "deque mutated" 예외는 나지 않습니다. 평균값이 한 장만큼 어긋날 수는 있습니다.)

## 10줄 요약
1. 인코더 설정(B 0, I 1 s, priority 0, latency 1, low-latency는 기능이 있을 때만, VBR 우선, 하드웨어 우선, 4 Mbps 비례)은 요구와 맞습니다.
2. 센서 ns 시각은 pts 맵으로 그대로 붙어서 오차가 없습니다. 쌍 묶기·OpenVINS 시각 기준도 JPEG 때와 같습니다.
3. SPS/PPS를 붙이고 flags를 해석하는 방식, `keyframe <렌즈>` 요청, 양방향 중계는 앱과 PC가 일치합니다.
4. [중] H1: 앱 큐에서 버리는 순서에 경쟁이 있고, 인코더 출력을 버려도 seq를 안 올립니다. 그래서 참조를 잃은 P가 PC로 갈 수 있는데, PC는 seq가 건너뛰어도 세기만 합니다.
5. → PC에서 "seq가 건너뛰고 키프레임이 아니면 키프레임 대기 + 요청" 한 줄이면 막을 수 있습니다.
6. [중하] H2: 실시간 인코더가 죽어도 상태가 STREAMING이고, `h264.error`를 PC가 읽지 않아서 원인이 안 보입니다.
7. [하] H3: 녹화+실시간에서 녹화 정지가 시간 초과로 끝나면 옛 인코더와 새 인코더의 출력이 섞입니다.
8. [하] H4: 크기만 바꾸는 경로에서 지난 `live_kbps`가 남습니다. H5: B 0을 출력에서 확인하지 않습니다.
9. [하·UX] H6: H.264 디코더 상태(키 대기·깨짐·seq 건너뜀)가 웹에 안 나옵니다.
10. REVIEW5/6 고침 중 R2·R4·R5·R10·N1은 맞게 고쳐졌습니다. 나머지는 자세히 보지 않았고, 실제 영상으로 해 본 확인은 아직 없습니다.

고침: H1 — PC H264Decoders 가 seq 가 건너뛴 패킷이 키프레임이 아니면(설정 패킷 포함) 그 렌즈를 키프레임 대기로 돌리고 바로 키프레임을 요청합니다(0.5 s 제한 유지, seq_gap_resync 로 셈). 앱은 큐에서 버릴 때 waitKey 를 큐와 같은 lock 안에서 켜고 serve 쪽 waitKey 확인도 같은 lock 으로 해서 틈을 없앴습니다. LiveEncoder·Recorder 가 인코더 출력을 버리면 seq 를 올리고 h264Lost 로 그 렌즈를 키프레임 대기 + 요청합니다(metrics h264_lost_output). H2 — 앱은 인코더 오류 때 status 를 'ERROR LIVE_ENCODER lens=… | try stream_fmt=jpeg' 로 쓰고 metrics h264.error 에도 남깁니다. PC web_server 가 metrics.json h264.error 를 읽어 ui.h264_error 로 넘기고, 웹 단계 표시줄이 '폰 영상 압축기 오류: … — 고급에서 JPEG 로 바꿔 보세요' 를 보여 줍니다. H3 — 앱 live sink 에 세대 번호(liveGen)를 붙여, 녹화 정지가 시간 초과로 끝나도 옛 Recorder 출력은 버립니다. H4 — scan3d.set_stream 이 크기·fps 만 바뀌면 live_kbps 를 새 크기 기준 기본값(load_model.live_kbps)으로 다시 계산하고, 사용자가 직접 준 값만 유지합니다(웹이 넘기는 값은 live_kbps_auto=True). H5 — PC 디코더가 슬라이스 헤더의 slice_type 으로 B 프레임을 세고(b_frames_seen) 웹 고급에 경고합니다. H6 — scan3d.state 의 phone.h264 에 렌즈별 키프레임 대기·깨진 프레임·seq 건너뜀·풀기 ms·kbps 를 넣고 웹 고급에 렌즈마다 한 줄로 보여 줍니다. 앱 1.4.1-h264 (versionCode 6) 빌드함, 폰이 연결돼 있지 않아 설치는 안 함. 실행·측정 확인은 아직 없음.


확인: (독립 확인, 코드 읽기와 py_compile만 함. 실제 영상으로 실행하거나 측정하지 않음)
- H1 PC: 맞게 고쳐졌습니다. seq가 건너뛴 P는 디코딩하지 않고 키프레임 대기로 돌린 뒤 요청합니다. 요청은 렌즈마다 0.5 s 제한을 지키고, 기다리는 동안 들어오는 P마다 다시 요청을 시도하므로 제한 때문에 요청이 빠져도 0.5 s 뒤에는 나갑니다. 대기가 안 풀리는 경우는 찾지 못했습니다. 키프레임은 seq가 건너뛰어도 그대로 받고, 인코더가 새로 만들어져 seq가 0부터 다시 시작해도 첫 패킷이 IDR이라 괜찮습니다. 앱은 설정만 든 패킷을 따로 보내지 않습니다(csd는 키프레임 앞에 붙임). 그래서 설정 패킷 앞이 빠진 경우를 처리하는 코드는 실제로 쓰일 일이 거의 없지만, 있어도 해는 없습니다.
- H1 앱: 큐에서 버릴 때의 틈은 막혔습니다. 버리는 것은 큐의 맨 앞(가장 오래된 것)이고, waitKey 설정과 serve 쪽 확인이 같은 lock 안에 있습니다. 출력 버림(h264Lost)에는 작은 틈이 남아 있습니다. 같은 렌즈의 키프레임이 아직 큐에 있을 때 그 뒤 출력이 버려지면, serve가 그 키프레임에서 waitKey를 풀고 참조를 잃은 P를 보냅니다. 이 경우는 PC의 seq 건너뜀 검사가 잡으므로 화면이 깨지지는 않습니다.
- [중하] 요청 폭주 (앱 쪽): 앱은 큐에서 하나를 버릴 때마다, 그리고 h264Lost가 올 때마다 바로 requestKeyframe을 부르고, 앱 쪽 시간 제한이 없습니다. LiveEncoder·Recorder의 keyReq는 boolean이라 프레임 하나에 한 번으로 합쳐지기는 합니다. 하지만 USB가 계속 막혀 큐(12)가 늘 차 있으면 거의 매 프레임이 키프레임(IDR)이 됩니다. 그러면 패킷이 커져 더 막히는 악순환이 생길 수 있습니다. 고칠 방향: 앱의 requestKeyframe에도 렌즈별로 0.3~0.5 s 제한을 둡니다(waitKey 설정은 그대로).
- H2: PC 경로(metrics h264.error → ui.h264_error → 단계 표시줄)는 맞습니다. 세션이 끝나면(대기 모드) 지워집니다. 새로 생긴 문제 두 가지가 있습니다.
  (1) [중하] mode=both(녹화 인코더를 같이 씀)에서 Recorder 렌즈 하나가 오류를 내면, status가 'RECORDING+STREAMING …'에서 'ERROR LIVE_ENCODER …'로 덮어써집니다. 그러면 phone_link와 win_relay의 is_recording()이 False가 되고, 정지 대기(is_done_state)는 'ERROR'를 보자마자 끝납니다. 그 결과 앱을 다시 시작하거나 정지할 때 다른 렌즈의 MP4를 마무리(moov)하기 전에 force-stop할 수 있습니다. 고칠 방향: 녹화 중이면 status를 덮지 말고 metrics(h264.error)에만 남깁니다. 또는 runningState()를 앞에 두고 '| LIVE_ENCODER_ERROR …'를 뒤에 붙입니다.
  (2) [하] liveError는 한 번 쓰이면 지워지지 않습니다. both에서 녹화 인코더 오류가 난 뒤 녹화를 멈추고 새 LiveEncoder가 정상으로 돌아도, metrics h264.error와 웹 빨간 줄이 세션 끝까지 남습니다(새 LiveEncoder를 만들 때 liveError=null로 지우면 됩니다).
- H3: 맞습니다. 새 실시간 소스를 만들 때마다 ++liveGen 하고, sink 입구에서 옛 세대의 h264/Drop/Lost/Error를 모두 버립니다. 세대를 올리기 전에 이미 큐에 들어간 옛 패킷은 그대로 나가지만, 순서가 맞고 새 인코더의 첫 패킷이 IDR이라 PC에서 문제가 없습니다.
- H4: 맞습니다. start_session(size=…)이나 fps만 바꾸는 경로에서는 새 크기 기준 기본값으로 다시 계산합니다. 부하 모델이 없으면 None이 되어 앱 자동값을 씁니다. 웹은 늘 live_kbps_auto=True로 넘깁니다. 다만 CLI나 API로 사용자가 직접 준 값은 크기를 바꿔도 남습니다(설계 의도대로이고, 버그 아님).
- H5: 일부만 고쳐졌습니다. PC가 slice_type%5==1로 B를 세고 웹 고급에 빨간 경고를 띄우는 것은 맞습니다. REVIEW7에 적은 나머지는 하지 않았습니다. 앱에서 Baseline으로 다시 configure하지 않고, 녹화 meta.json에 b_frames_seen을 적지 않습니다(녹화를 되살릴 때 PTS 가정은 그대로). 경고만 남겨도 되는지는 판단이 필요합니다.
- H6: 맞습니다. JPEG·NV21 세션에서는 h264.used가 False여서 phone.h264가 없고, h264Rows도 빈 목록을 돌려줍니다.
- JPEG/NV21 회귀는 보이지 않습니다. 바뀐 것은 offerFrame에 lock이 하나 더해진 것뿐입니다(KIND_H264일 때만 waitKey를 건드림). serve 쪽 lock도 H.264 패킷에만 걸립니다. PC의 read_frame JPEG·NV21 경로는 바뀌지 않았습니다. py_compile(lib_phone_stereo.py, scan3d.py, web_server.py)은 통과했습니다. WSL에 node가 없어서 app.js는 다시 검사하지 못했습니다.

고침2: (1) 앱 requestKeyframe 에 렌즈별 0.4 s 제한 — 큐 버림·출력 잃음(h264Lost)·PC keyframe 요청이 모두 같은 제한을 지나고, 제한 중이면 keyReqDeferred 만 켜 두었다가 제한이 끝난 뒤 그 렌즈의 다음 인코더 출력에서 한 번 요청합니다(그 사이 키프레임이 나오면 요청 취소, metrics h264.key_requests_limited). waitKey 는 그대로 바로 켭니다. 출력 버림 틈은 렌즈별 lossEpoch 로 막았습니다: 큐 버림·h264Lost 때 pendingLock 안에서 waitKey 와 lossEpoch 를 함께 올리고, 패킷은 offerFrame 에서 그때 epoch 를 받으며, serve 는 현재 epoch 의 키프레임만 대기를 풉니다(이미 큐에 있거나 serve 가 막 꺼낸 옛 키프레임은 대기를 못 풂). (2) 녹화 중(recorder 있음 또는 both 공유)에는 인코더 오류가 status 첫 낱말을 덮지 않고 'RECORDING… warn=LIVE_ENCODER lens=N' 만 덧붙입니다(오류 글은 metrics h264.error). 녹화 정지 중(recorder 이미 null)이면 status 를 건드리지 않습니다. 실시간 전용일 때만 'ERROR LIVE_ENCODER …'. phone_link(BAD_WORDS·is_recording·is_done_state·parse_status)와 win_relay(같은 규칙·줄 전체 BAD 검사)는 첫 낱말과 FAILED/ERROR/DISCONNECTED/STOPPED 문자열만 보므로 warn=LIVE_ENCODER 는 오류로 보지 않습니다. (3) 앱: liveError 에 세대(liveErrorGen)를 붙여 새 세대 소스가 정상 출력하면 지우고, metrics h264.gen 을 씁니다. PC web_server: 새 metrics 에 h264.error 가 없거나 h264.gen 이 오류 때와 다르면 ui.h264_error 를 지웁니다. (4) 녹화 meta.json 에 b_frames_seen(렌즈별 lenses.<id>.b_frames_seen·b_frames)을 Recorder 출력마다 slice_type%5==1 로 검사해 적습니다. Baseline 다시 configure 는 하지 않음 — 녹화 중인 MP4 는 도중에 프로파일을 바꿀 수 없고 PC 가 이미 B 를 세어 경고하므로 경고로 충분합니다. SPEC.md 1.2.2 metrics·meta 필드 갱신. 앱 1.4.2-h264 (versionCode 7) 빌드함, adb devices 에 기기가 없어 설치는 안 함. py_compile·pyflakes·node --check 통과, 실행·측정 확인은 아직 없음.


확인2: (독립 확인. 코드 읽기, py_compile(web_server·phone_link·win_relay), Windows node --check(app.js)만 했고 둘 다 통과했습니다. 실행이나 측정은 하지 않았습니다. APK 1.4.2-h264(versionCode 7)가 있는 것까지 확인했습니다.)
- (1) 요청 제한: 맞습니다. 큐 버림·h264Lost·PC `keyframe`·새 클라이언트가 모두 `requestKeyframe`을 지나고, 렌즈별 0.4 s 안이면 `keyReqDeferred`만 켭니다. `keyframeTick`은 sink의 `h264`에서 offerFrame보다 먼저 불립니다. 키프레임이 나오면 미뤄 둔 요청을 지우고, 제한이 끝난 뒤 P가 나오면 요청을 한 번 보냅니다. 락은 `keyReqLock`과 `pendingLock` 두 개인데 서로 겹쳐 잡지 않아 교착은 없습니다. waitKey는 제한과 상관없이 바로 켜지므로 끊긴 P가 나가는 일은 없습니다.
- (1) lossEpoch: 맞습니다. epoch는 offerFrame 안에서 lock을 잡은 채 찍습니다. 같은 렌즈를 버린 경우에는 버린 뒤에 다시 찍습니다. serve는 같은 lock 안에서 `KEY && epoch==현재`일 때만 대기를 풉니다. 그래서 확인에서 말한 "큐에 남은 옛 키프레임으로 대기가 풀림" 틈은 없어졌습니다.
  키프레임 대기가 안 풀리는 경우는 찾지 못했습니다. 요청이 제한에 걸려도 다음 출력에서 나가고, 늦어도 I 1 s 주기 IDR이 현재 epoch로 들어옵니다. stopRecording·serve는 epoch를 올리지 않고 waitKey만 켜는데, 이 경우에도 다음 키프레임이면 풀리므로 문제없습니다. 키프레임이 연달아 버려지면 대기가 길어질 수는 있습니다. 이것은 USB가 막힌 탓이고 버그가 아닙니다.
- (2) 녹화 중 status: 대체로 맞습니다. 첫 낱말은 RECORDING…으로 남고 뒤에 `warn=LIVE_ENCODER lens=N`만 붙습니다. phone_link(BAD_WORDS·is_recording·is_done_state·state_ok)와 win_relay(같은 규칙)를 다시 읽었습니다. 두 곳 모두 첫 낱말과 FAILED/ERROR/DISCONNECTED/STOPPED 부분 문자열만 봅니다. "LIVE_ENCODER"에는 이 문자열이 없어서 오류로 보지 않습니다. 녹화 중 판정과 정지 대기도 그대로 동작합니다.
  남은 틈 하나 [하]: "녹화 정지 중이면 status를 건드리지 않음"은 `lastStatus`가 RECORDING으로 시작하지 않을 때만 지켜집니다. stopRecording은 `recorder=null` 다음에 `rec.stop(3000)`을 부르고, RECORD_DONE은 그 뒤에 씁니다. 그 사이 `lastStatus`는 아직 "RECORDING…"입니다. Recorder 렌즈 스레드는 EOS drain 중 예외도 h264Error로 보냅니다(LiveEncoder와 달리 `stopping` 검사가 없음). 그러면 rec==null인데도 "RECORDING… warn=" 줄을 씁니다.
  rec.stop이 제때 끝나면(done=true) RECORD_DONE이 나중에 써지므로 해가 없습니다. 시간 초과(done=false)에서는 문제가 될 수 있습니다. 오류 스레드가 gen 검사를 지난 뒤, 정지 스레드가 `++liveGen`하고 RECORD_DONE을 쓰고, 그다음 오류 스레드가 status를 쓰면 RECORDING이 다시 덮어씁니다. 그러면 PC 정지 대기가 시간 초과까지 기다리고 force-stop합니다. 아주 드문 경우입니다. 고칠 방향: `if(rec==null) return;`을 base 검사 앞으로 옮깁니다(liveShared이면서 rec==null = 정지 중).
- (3) 오류 지우기: 맞습니다. 앱은 liveErrorGen과 다른 세대가 정상 출력하면 liveError를 지웁니다. both에서 다른 렌즈가 같은 세대로 계속 출력해도 지우지 않습니다. metrics는 `h264.gen`을 씁니다. web_server는 h264.error가 없으면 지우고, 있으면 그 세대로 바꿉니다. 세대를 비교하는 분기는 사실 이 규칙에 이미 포함되지만 해는 없습니다. 실시간 전용 LiveEncoder의 `le.error()`는 그 인코더가 살아 있는 동안 계속 남는데, 인코더가 죽은 상태이므로 맞는 동작입니다.
- (4) b_frames_seen: 맞습니다. Recorder가 config가 아닌 출력마다 `H264.isBSlice`로 첫 슬라이스(NAL 1/5)의 slice_type%5==1을 검사합니다. 이 검사는 emulation 03 제거와 4바이트 시작 코드도 처리합니다. meta.json에는 전체 `b_frames_seen`과 렌즈별 `lenses.<id>.b_frames_seen`·`b_frames`가 들어갑니다(writeFinalMeta, 늦게 끝난 경우 다시 쓸 때도 들어감). Baseline으로 다시 configure하지 않는 이유(녹화 MP4 중간에 프로파일을 바꿀 수 없음, PC가 경고함)는 받아들일 만합니다. 다만 실시간 전용 LiveEncoder 쪽은 앱이 B를 세지 않고 PC만 셉니다.
- 녹화 정지 흐름 회귀는 위 [하] 틈 하나 말고는 없습니다. 바뀐 순서는 stopRecording의 `++liveGen` 뒤에 새 LiveEncoder를 만드는 것인데, 이것은 이전과 같습니다. JPEG·NV21 경로는 epoch를 찍는 데에만 `KIND_H264` 조건을 걸었으므로 영향이 없습니다.
- 고침3 (메인 세션): h264Error 에서 rec==null 검사를 base 검사 앞으로 옮김 — 정지 중 RECORD_DONE 덮어쓰기 방지. 앱 1.4.3-h264.
