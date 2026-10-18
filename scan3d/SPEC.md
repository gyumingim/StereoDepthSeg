# scan3d — 실시간 3D 복셀 맵 스캐너 인터페이스 명세

작성 2026-10-16. 대상: scan3d 구현자. 근거는 저장소 코드(파일:행)와 WSL 설정을 직접 읽은 것이다.
"추정"이라고 쓴 값은 실측이 아니다. 테스트·측정은 실행하지 않았다(사용자 선호).

```
폰 Galaxy S24 (com.camera.dualstream)
  Camera2 렌즈 2(초광각)+5(광각) NV21 640x480 + SENS 센서 패킷 ── 127.0.0.1:8765 (폰 안)
        │ USB / adb forward tcp:8765 tcp:8765   (Windows adb.exe, Windows 127.0.0.1 에만 바인딩)
Windows ─ scan3d/win_relay.py (표준 라이브러리, "역방향" 릴레이: 두 쪽 모두 Windows 에서 바깥으로 connect)
        │ 127.0.0.1:8766 → WSL2 localhost forwarding
WSL Ubuntu-22.04 ─ scan3d/run_scanner.py (venv_ffs)
   phone_link ─┬─ IMU(accel+gyro 동기화) ─┐
               └─ 프레임 쌍 → 정류(gray) ─┴→ ov_bridge (C++, OpenVINS ROS-free, stdin/stdout 파이프) → T_world_cam0
                  프레임 쌍(키프레임) → FFS 시차 → 깊이 + YOLO-seg 마스크 ─→ voxel_map (클래스·인스턴스)
   web server 0.0.0.0:8080 ── Windows 브라우저 http://localhost:8080 (three.js 뷰어, 조작 전부 여기)
```

---------------------------------------------------------------------------------------------------

## 0. 파일 배치 (제안, 새 코드는 전부 scan3d/)

| 파일 | 실행 위치 | 역할 |
|---|---|---|
| `scan3d/win_relay.py` | Windows Python 3.13 (`py -3`) | adb forward 포트 ↔ WSL 포트 중계 (8절) |
| `scan3d/phone_link.py` | WSL | adb.exe(interop) 로 앱 시작, 8766 리스너, 패킷 수신, IMU 동기화 |
| `scan3d/calib_vio.py` | WSL | calib_phone_pair.json → 정류 파라미터 → OpenVINS yaml 3개 생성 |
| `scan3d/vio/CMakeLists.txt`, `scan3d/vio/ov_bridge.cpp` | WSL (C++) | VioManager 래퍼, 파이프 프로토콜 (9절) |
| `scan3d/vio_client.py` | WSL | ov_bridge 프로세스 띄우기·메시지 쓰기·포즈 읽기 |
| `scan3d/depth_seg.py` | WSL (venv_ffs) | FFS + YOLO-seg 워커 |
| `scan3d/voxel_map.py` | WSL | 희소 복셀 해시, 클래스/인스턴스 투표, 3D 박스 |
| `scan3d/server.py`, `scan3d/web/index.html`, `scan3d/web/app.js` | WSL | HTTP API + 뷰어 |
| `scan3d/run_scanner.py` | WSL | 진입점 (인자 파싱, 스레드 묶기) |

기존 파일 수정은 앱 2개만: `phone_stereo/android/src/com/camera/dualstream/MainActivity.java`, `phone_stereo/android/AndroidManifest.xml` (2절).
`lib_phone_stereo.py` 등 파이썬 라이브러리는 **수정하지 않고 import 만** 한다.

---------------------------------------------------------------------------------------------------

## 1. 폰 ↔ PC 프로토콜 (현재 코드 그대로)

### 1.1 전송 계층
- 폰 앱이 `ServerSocket(8765, backlog 1, 127.0.0.1)` 을 연다 (MainActivity.java:154). 클라이언트 1개만, 끊기면 다시 accept.
- PC 는 `adb forward tcp:8765 tcp:8765` 후 PC 의 127.0.0.1:8765 에 connect. 데이터는 폰→PC 단방향, PC 는 아무것도 보내지 않는다.
- **주의**: adb forward 포트는 폰 앱이 listen 하기 전에도 connect 를 받아 준다 → connect 성공 ≠ 준비 완료. 반드시 `status.txt` 가 `STREAMING` 으로 시작할 때 connect (z_phone_stereo.py:196-208).
- 앱이 `onStop`(화면 꺼짐/다른 앱) 되면 카메라·소켓을 닫고 `STOPPED (activity not visible)` 를 쓴다. 폰 화면을 켜 두어야 한다.

### 1.2 패킷 형식 (모두 big-endian, `java.io.DataOutputStream`)
헤더 = `struct.Struct(">IIIIQI")` 28 바이트 (lib_phone_stereo.py:20):

| 필드 | 형 | 프레임 패킷 | 센서 패킷 |
|---|---|---|---|
| magic | u32 | `0x53544552` "STER" | `0x53454E53` "SENS" |
| index | u32 | 스트림 번호 0..n-1 (`--physical` 순서) | Android `Sensor.getType()` 값 (앱 자체 100 = frame_timing, 101 = stream_info) |
| width | u32 | 640 | 0 |
| height | u32 | 480 | 0 |
| timestamp | u64 | ns, 그 렌즈 사진의 **노출 가운데 시각** (앱 1.7-mid, 아래 1.4) | ns, `SensorEvent.timestamp` |
| length | u32 | w*h*3/2 = 460800 | 4 × 값 개수 |
| payload | bytes | NV21 | float32 big-endian 값들 |

- NV21 (STER): Y 평면 w*h, 이어서 VU 교차(V 먼저) w*h/2. 앱이 plane stride 를 풀어 연속 배열로 만든다 (MainActivity.java:257-268).
  디코드: `np.frombuffer(buf, np.uint8).reshape(h*3//2, w)` → `cv2.cvtColor(..., cv2.COLOR_YUV2BGR_NV21)`.
- **이미지 회전 없음**: 센서 원래 방향(가로 640x480, SENSOR_ORIENTATION 90). 폰을 세로로 들면 영상이 90° 누워 있다. 보정·정류·OpenVINS 는 이 원 방향 그대로 쓴다. YOLO 정확도를 위해 회전이 필요하면 YOLO 입력만 `cv2.rotate(ROTATE_90_CLOCKWISE)` 하고 마스크를 되돌려 쓴다 (선택).
- 센서 패킷 검증 (read_frame, lib_phone_stereo.py:61): `width=height=0`, `length % 4 == 0`, `4 <= length <= 64` (값 1~16개). 새 센서(가속도 3값, uncalibrated 6값)도 이 범위 안이다.
- 프레임 패킷 검증: `index < stream_count`, `(w,h) == size`, 짝수 크기, `length == w*h*3/2`. 하나라도 어긋나면 `ValueError` → 수신 스레드 종료.
- 앱 큐: 프레임 큐 6 (가득 차면 오래된 것 버림), 센서 큐 1024 (가득 차면 오래된 것 버림). writer 스레드가 프레임 1개 + 센서 최대 1024개씩 번갈아 쓴다 (MainActivity.java:280-304).

#### 1.2.1 JPEG 프레임 패킷 "STEJ" (2026-10-16 추가, 앱 1.4-h264 부터는 고급 설정 — 기본은 1.2.2 H.264)
- 이유: NV21 원본 640x480 두 렌즈 30 fps = 약 28 MB/s. 첫 실행에서 78 초에 2.18 GB 를 보낸 뒤 폰이 adb 에서 사라졌다. JPEG q90 이면 약 2~3 MB/s (추정, 측정 안 함).
- 앱 인텐트 extra: `--es stream_fmt jpeg` (1.4-h264 부터 기본은 h264, 1.2.2), `--ei jpeg_q 90` (1~100). 녹화(record/both 의 MP4)는 이와 무관하게 H.264 그대로.
- 헤더는 STER 와 같고 magic 만 다르다:

| 필드 | 값 |
|---|---|
| magic | `0x5354454A` "STEJ" |
| index / width / height / timestamp | STER 와 같음 (렌즈 노출 가운데 시각 1.4, 640x480) |
| length | JPEG 바이트 수 (가변, PC 는 1 B ~ 8 MiB 만 받음) |
| payload | baseline JPEG 한 장 (`YuvImage(NV21).compressToJpeg`, 회전·크기 변경 없음) |

- 앱: 렌즈마다 단일 스레드 인코더(`jpeg-0`, `jpeg-1`)가 인코딩 → 렌즈별 순서 유지, 카메라 스레드를 막지 않음. 렌즈당 대기 2장 넘으면 새 프레임을 버린다 (logcat `jpeg dropped=`). 인코딩된 패킷은 기존 프레임 큐(6, 오래된 것 버림)로 들어간다.
- PC (`lib_phone_stereo.read_frame`): `cv2.imdecode(buf, cv2.IMREAD_COLOR)` → STER 와 똑같은 BGR `PhoneFrame`. 그 뒤 짝 맞추기·역행 폐기·IMU 처리는 바뀌지 않는다 (`scan3d/phone_rx.py` 도 그대로).
  디코드 실패나 크기 불일치 패킷은 건너뛰고 `stats()["jpeg_decode_errors"]` 로 센다 (길이로 경계가 맞으니 재접속 안 함). index/크기/길이 범위가 틀리면 STER 와 같이 `ValueError`.
- 한 스트림 안에 STER·STEJ·SENS 가 섞여도 된다 → 옛 앱(NV21 만)도 그대로 받는다. 옛 PC 코드는 STEJ 를 `ValueError` 로 거부하므로 옛 PC 와 쓰려면 `--es stream_fmt nv21`.
- JPEG 은 손실 압축이라 FFS 깊이·OpenVINS 특징점에 약간 영향이 있을 수 있다 (q90 에서 작다고 추정, 측정 안 함). 의심되면 `stream_fmt nv21` + `fps 15` 로 비교.

#### 1.2.2 H.264 프레임 패킷 "STEH" (2026-10-16 추가, 앱 1.4-h264 / versionCode 5, **실시간 기본**)
- 사용자 결정: 실시간 스트림도 녹화도 H.264. I+P 만 (B-frame 0), 키프레임 1 s 간격, 실시간 우선, 하드웨어 인코더 우선. JPEG·NV21 은 고급 설정.
- 기본을 H.264 로 한 이유: 압축을 폰의 하드웨어 인코더가 하므로 CPU 로 JPEG 을 만드는 것보다 폰 부하·발열이 작고, 같은 화질에서 USB 로 보내는 양이 훨씬 적다 (640x480@15 렌즈당 약 4 Mbps ≈ 0.5 MB/s, JPEG 은 2~3 MB/s 추정, NV21 은 약 14 MB/s). 측정 전 추정.
- B-frame 을 쓰지 않는 이유: B 는 뒤 프레임을 먼저 받아야 풀 수 있어 그만큼 지연이 생기고 (출력 순서 ≠ 입력 순서), 프레임 종류마다 화질이 출렁여 스테레오 깊이·VIO 특징점에 불리하다. I+P 면 패킷 하나 = 바로 풀리는 프레임 하나.
- 앱 인텐트 extra: `--es stream_fmt h264|jpeg|nv21` (기본 **h264**), `--ei live_kbps N` (렌즈당 kbps, 없으면 14.5 와 같은 자동값: 640x480@15 = 4000).
- 헤더는 1.2 의 28 바이트 공통 헤더 그대로 (`>IIIIQI`), payload 앞 8 바이트만 추가:

| 위치 | 필드 | 형 | 값 |
|---|---|---|---|
| 0 | magic | u32 | `0x53544548` "STEH" |
| 4 | index | u32 | 렌즈 번호 0..n-1 (`physical` 순서, 예 0 = 렌즈 2, 1 = 렌즈 5) |
| 8 / 12 | width / height | u32 | 인코딩 크기 (예 640 / 480) |
| 16 | timestamp | u64 | 그 프레임의 **노출 가운데 시각** ns (1.4, 앱 1.7-mid 부터; STER/STEJ 와 같은 값. 1.6 까지는 물리 SENSOR_TIMESTAMP) |
| 24 | length | u32 | 8 + NAL 바이트 수 |
| 28 | flags | u32 | bit0 = 키프레임 (IDR), bit1 = 이 payload 안에 SPS+PPS 있음 |
| 32 | seq | u32 | 렌즈별 인코더 출력 번호 (0 부터 +1). 건너뛰면 그 사이 프레임이 앱에서 버려진 것 |
| 36 | NAL | bytes | Annex-B (start code `00 00 00 01`) 접근 단위 1개 = 프레임 1장 |

- **키프레임마다 SPS/PPS 가 앞에 붙는다** (인코더가 다시 내지 않으면 앱이 첫 codec-config 버퍼를 붙임) → PC 디코더는 아무 키프레임에서나 시작 가능. 따로 오는 설정 전용 패킷은 없다.
- B-frame 0 이므로 출력 순서 = 입력 순서 = timestamp 증가 순서. 디코드한 프레임에 그 패킷의 timestamp 를 그대로 붙이면 된다.
- 렌즈마다 독립 스트림 (인코더 1개씩). PC 는 렌즈별 디코더 2개를 둔다: `lib_phone_stereo.H264Decoders` (PyAV `CodecContext.create("h264","r")`, SLICE 스레드, `flags=low_delay`; venv_ffs 에 `av==19.0.1`, 실제 스트림으로는 아직 확인 안 함). 키프레임 전 P 는 버리고 그 렌즈에 `keyframe <n>` 을 보낸다 (렌즈별 0.5 s 에 한 번까지). 디코딩 오류·깨진 프레임·출력 4 장 넘게 밀림 → 디코더 리셋 + 키프레임 요청. seq 가 건너뛰면 `seq_gaps` 로 센다. 결과는 BGR `PhoneFrame` 이라 쌍 묶기·OpenVINS·FFS·카메라 창은 그대로. 지표: `stats()["h264"]` (렌즈별) 와 `stats()["decode_ms"/"decode_ms_max"]` (렌즈 평균, 웹 부하 표시).
- 앱 쪽 버림 규칙: 프레임 큐 12 (가득 차면 가장 오래된 것 버림). H.264 패킷이 버려지면 그 렌즈는 다음 키프레임까지 P 프레임을 보내지 않고 (`h264_dropped_wait_key`) 즉시 키프레임을 요청한다. 새 클라이언트 접속 때도 모든 렌즈를 키프레임부터 보낸다.
- **PC → 폰 명령** (같은 TCP 소켓의 역방향, UTF-8 한 줄, `\n` 끝): `keyframe` (모든 렌즈) 또는 `keyframe 1` (렌즈 번호). 앱이 `PARAMETER_KEY_REQUEST_SYNC_FRAME` 으로 다음 프레임을 키프레임으로 만든다. 모르는 명령은 logcat 경고만. 아무것도 안 보내는 예전 PC 도 그대로 동작. win_relay 는 양방향 중계 (`pump` 두 방향).
- 인코더 설정 (`H264.java`): `KEY_MAX_B_FRAMES=0` (API 29+), 프로파일 High (목록에 있으면, B 0) → 실패하면 Baseline, `KEY_I_FRAME_INTERVAL=1`, `KEY_PRIORITY=0`, `KEY_LATENCY=1` (API 30+), `KEY_LOW_LATENCY=1` (코덱이 FEATURE_LowLatency 를 알릴 때만), 비트레이트 모드 VBR (지원 시, 아니면 CBR).
  VBR 을 고른 이유: CBR 은 초당 비트를 고정해서 손떨림·빠른 움직임 프레임의 화질이 떨어지고, VBR 은 같은 평균에서 그런 프레임에 비트를 더 줘 화질이 더 일정하다. USB 여유가 커서 VBR 의 짧은 순간 증가는 문제 없다 (추론, 이 폰에서 측정 안 함).
  설정이 거부되면 순서대로 낮춰 다시 시도 (High → Baseline → 지연 키 없이 → 인코더 기본). 실제 적용값은 metrics.json `h264.settings`.
- 입력 경로: ImageReader YUV → NV21 복사 → 인코더 입력 Image (YUV420Flexible). 인코더 입력 Surface 를 카메라 출력에 바로 붙이지 않은 이유: 논리 멀티카메라의 물리 스트림에서 Surface 로 들어가는 버퍼 timestamp 가 물리/논리 중 어느 것인지 문서에 없어 "정확한 센서 시각" 을 보장할 수 없고, 출력 4개(YUV 2 + 인코더 2)가 보장 스트림 조합을 넘을 수 있다 (LiveEncoder.java 주석).
- 녹화+실시간 (`mode=both`, h264): 실시간 스트림이 **녹화 인코더 출력을 공유** → 렌즈당 인코더 1개. 녹화만 먼저 끝나면 (STOP 방송) 그때 실시간 전용 인코더로 넘어간다 (첫 패킷은 키프레임). 녹화도 B 0 · I 1 s (meta.json `max_b_frames: 0` = 요청값, `b_frames_seen` = 인코더 출력 slice_type 검사 결과 (렌즈별 `lenses.<id>.b_frames_seen`), `live_shares_encoder`, `lenses.<id>.encoder_settings`).
- caps.json 추가: `live_h264` {default_stream_fmt, magic, input: "buffer", shared_with_record: true, b_frames: 0, i_interval_s: 1, priority: 0, encoders_per_lens, instances_needed, max_b_frames_key_api,
  encoder_640x480_15, hardware, profile, bitrate_mode, low_latency (그 인코더에서 처음 시도할 값)} (이름은 scan3d/load_model.py 의 가정 필드에 맞춤),
  `avc_encoders[]` 에 {low_latency_feature, profile_high, profile_baseline, b_frames_possible, max_b_frames_key, bitrate_modes {vbr,cbr,cq}, live_bitrate_mode}, `pairs[]` 에 {live_encoder, live_kbps_default}. (`hardware`, `max_instances` 는 원래 있음.)
- metrics.json 추가: `stream_fmt: "h264"`, `live_kbps` (목표, 렌즈 하나), `h264` {encoder, shared_with_recorder, kbps_target, settings (적용된 프로파일·모드·지연 키·하드웨어 여부), key_requests, key_requests_limited (렌즈별 0.4 s 제한에 걸린 요청), gen (실시간 소스 세대), error?},
  렌즈별 {h264_fps, h264_kbps (최근 2 s 인코더 출력), h264_enc_ms_avg/max (입력 넣고 → 출력 나올 때까지), h264_frames, h264_keyframes, h264_dropped (인코더 큐 가득·입력 버퍼 없음), h264_dropped_wait_key}.
- 지원 안 되는 조합: 실시간 h264 인데 쓸 인코더가 없으면 `ERROR UNSUPPORTED ... reason=no H.264 encoder for live ... (try stream_fmt=jpeg)` (14.4 와 같은 규칙).
- PC 쪽 기본도 h264: `phone_link.PhoneLink(stream_fmt="h264", live_kbps=None)` (→ `--es stream_fmt`, 값이 있을 때만 `--ei live_kbps`), `scan3d.py --app-stream h264|jpeg|nv21`, `win_relay.py --stream-fmt`, 웹 고급 설정 "영상 형식" (→ `scanner.set_stream(..., stream_fmt=, live_kbps=)`, 다음 세션부터).
- 녹화 복원(replay): 녹화 MP4 도 B 0 이라 디코드 순서 = PTS 순서. `recording.py` 의 PTS 맞추기는 그대로 쓴다.

#### 1.2.3 빠르게 찍고 1/N 만 저장 (앱 1.8-subsample / versionCode 12, 2026-10-18)
- 인텐트 `--ei fps <촬영 fps> --ei save_fps <저장 fps>` (save_fps 없거나 0 = fps, 예전 동작 그대로). fps 는 save_fps 의 정수배,
  `raw_ts=false` 만. 예: `--ei fps 60 --ei save_fps 15`. 카메라 AE 범위는 [fps,fps], 인코더(녹화·실시간)·MP4·스트림은 save_fps.
- 고르기 (`Subsample.java`, 카메라 스레드, 시각만 보고 고른 뒤 그 프레임만 복사·인코드; 버리는 프레임은 바로 close):
  기준 렌즈(물리 "2")는 1/save_fps 격자에 노출 가운데 시각이 가장 가까운 프레임, 다른 렌즈는 남긴 기준 프레임과 노출 가운데 시각이
  가장 가까운 한 장 (|dt| ≤ 촬영 주기 1 개, 그 렌즈의 기준 시각 + 0.6 주기 뒤 프레임까지 본 뒤 결정). 결정 전 이미지는 잠깐 붙잡는다
  (렌즈당 최대 5, ImageReader maxImages 8).
- 스트림 헤더·frames CSV `t_mid_ns` 는 그대로 노출 가운데. frames CSV 마지막 열 `pair_dt_ns` (기준 렌즈 행은 빈칸),
  meta.json `fps` = save_fps (파일 안 프레임률), `capture_fps`, `save_fps`, `subsample{pairs, unmatched, dt_ms_median/p95/max, kept, seen,
  forced_drops}`. metrics.json `fps` = save_fps, `capture_fps`, `save_fps`, `subsample` (최근 30 짝), 렌즈별 `cam_fps` = 촬영,
  `kept_fps`·`copy_ms_avg` = 남긴 프레임.
- PC: `phone_link.PhoneLink.capture_fps` (→ `--ei fps <capture> --ei save_fps <fps>`), `scan3d.py --app-capture-fps`, 웹 '촬영' 선택
  (`{"cmd":"mode", ..., "capture_fps": 60|0}`), `load_model.combo(..., capture_fps=)` (카메라 검사만 촬영 fps). 짝 묶기·VIO·복원은 저장 fps 그대로.
- S24 실측 (1440x1080, 두 렌즈 60 fps 요청): 카메라 결과는 약 50 /s (몇 프레임마다 HAL 이 하나씩 건너뜀), 렌즈 2 의 위상이 렌즈 5 에 대해
  계속 미끄러져 짝 시각차는 0~8.3 ms 에 고르게 퍼짐. 자세한 표는 `scan3d/research/subsample_60_15.md`.

### 1.3 파이썬 수신 API (lib_phone_stereo.py, import 해서 재사용)
- `read_frame(sock, stream_count, size, h264=None) -> PhoneFrame | SensorSample` — 블로킹, 패킷 1개. STEH 를 받으려면 `h264=H264Decoders(stream_count, sock)` (PhoneReceiver·phone_rx 는 알아서 만든다).
  - `PhoneFrame(index:int, timestamp_ns:int, bgr:np.ndarray HxWx3 uint8)`
  - `SensorSample(type:int, timestamp_ns:int, values:tuple[float])`
- `FramePairer(count=2, max_skew_ms=10)`: `.add(frame) -> tuple[PhoneFrame,...] | None`. 스트림 0 이 기준, 다른 스트림이 그 시각을 앞뒤로 감쌀 때까지 기다렸다 가장 가까운 것을 고름. 묶음 시각차 > max_skew 이면 기준 프레임 버림. 큐 길이 8.
- `PhoneReceiver(sock, count, size, max_skew_ms)`: 스레드로 위 둘을 돌리고 `snapshot()`(최신 쌍 폴링), `sensor_at`, `sensor_window`, `imu_for_pair`, `stats()` 제공. 센서 deque 길이 6000 (400 Hz 면 15 s).
  - **scan3d 는 PhoneReceiver 를 쓰지 말고** `read_frame` + `FramePairer` 로 자체 수신 스레드를 만든다. 이유: PhoneReceiver 는 최신 쌍만 폴링하게 되어 있어 IMU 를 시간 순서대로 빠짐없이 VIO 에 흘려보내는 콜백 경로가 없다. 동작(역행 timestamp 폐기, 쌍 skew 기록)은 PhoneReceiver._run 을 그대로 따라 한다.
  - 역행 처리: 같은 스트림에서 `timestamp_ns <= last_ts` 인 프레임은 버리고 센다 (lib_phone_stereo.py:138-140).

### 1.4 타임스탬프 기준
- 카메라: `SENSOR_INFO_TIMESTAMP_SOURCE = 1 (REALTIME)` (cameras_s24.json, 모든 물리 렌즈 ts=1) → `SystemClock.elapsedRealtimeNanos()` 시계.
- 센서: `SensorEvent.timestamp` 도 같은 elapsedRealtimeNanos 시계 (MainActivity.java:50-51 주석, `stats()["clock_gap_ms"]` 로 확인하는 구조).
- **프레임 timestamp (STER/STEJ/STEH 헤더, 녹화 frames CSV `t_mid_ns`) = 그 렌즈 사진 전체 노출의 가운데 시각** (앱 1.7-mid / versionCode 11, 2026-10-16):
  `t_mid = 물리 SENSOR_TIMESTAMP + SENSOR_EXPOSURE_TIME/2 + SENSOR_ROLLING_SHUTTER_SKEW/2`. 첫 줄은 [t, t+노출], 마지막 줄은 [t+skew, t+skew+노출]
  이므로 모든 줄 노출 구간의 가운데. 시계는 그대로 elapsedRealtimeNanos (IMU 와 같음) → PC 는 더할 것 없이 IMU 와 바로 비교.
  - 노출·skew 는 **렌즈별 물리 결과** (`getPhysicalCameraResults()`) 값. 그 렌즈 결과에 없으면 논리 결과 값, 그것도 없으면 그 렌즈의 마지막으로
    알던 값 (처음부터 모르면 0 → 노출 시작). metrics.json / meta.json `mid_ts` {ref, from_physical, from_logical, last_known, unknown, no_result}
    = 그렇게 계산한 프레임 수 (`no_result` = 결과를 못 찾은 프레임(raw_ts 에서 이미지가 결과보다 먼저 옴) → 그 렌즈 마지막 오프셋).
  - 기본 모드(`raw_ts=false`): t = **물리 렌즈별 SENSOR_TIMESTAMP** (CaptureTimestampMatcher 가 논리 결과의 `getPhysicalCameraResults()` 와 이미지 timestamp 를
    정확히 짝지은 뒤 물리 값으로 바꿈). `raw_ts=true` 면 논리 timestamp(두 렌즈 같음) 에 같은 오프셋.
  - 앱이 기준을 알리는 곳: status 꼬리 ` ts_ref=mid` (실행 상태 줄, 15.4), 센서 타입 101 "stream_info" (아래), caps.json / meta.json
    `timestamp_ref: "mid_exposure"`, metrics.json `timestamp_ref`. 1.6 까지(이 표시들이 없음)는 t = SENSOR_TIMESTAMP (첫 줄 노출 시작).
  - PC: `phone_rx.StreamReceiver` 가 101 을 받으면 (또는 scan3d 가 앱 status 의 `ts_ref=mid` 를 보면) `FrameTimingIndex.set_mid()` → 노출 보정 0.
    옛 앱(101 없음)만 예전처럼 frame_timing 으로 0.5·(노출 + skew) 를 더한다. 녹화 재생은 RECORDING_FORMAT.md (옛 녹화는 노출 열로 바꿈).
- SENSOR_TIMESTAMP 의 뜻 = 첫 행 노출 시작 (frame_timing 의 오프셋은 이 값 기준 그대로).
- 센서 타입 100 "frame_timing" (MainActivity.java:63-97): timestamp = 논리 SENSOR_TIMESTAMP, values = `[phys[0]-logical ms, phys[1]-logical ms, exposure ms, frame duration ms, rolling shutter skew ms]` (렌즈 2개 기준 5값, 노출·skew 는 논리 결과 값). 1.7-mid 에서도 바뀌지 않음 (물리 오프셋 = 노출 시작 기준).
- 센서 타입 101 "stream_info" (앱 1.7-mid): timestamp = 보낸 시각 (elapsedRealtimeNanos), values = `[ts_ref]` (1.0 = 프레임 timestamp 가 노출 가운데).
  접속 직후 한 번, 그 뒤 frame_timing 30 개마다 한 번. 녹화 imu.csv 에는 안 씀 (녹화는 meta `timestamp_ref`). 옛 PC 는 모르는 센서로 받아 넘김.
- 동기 등급 `LOGICAL_MULTI_CAMERA_SENSOR_SYNC_TYPE = 0 (APPROXIMATE)` → 두 렌즈 노출이 하드웨어로 묶였다는 보장은 없다. 과거 측정(raw_ts) skew 0 ms, 물리 timestamp 모드의 skew 분포는 이 명세 작성 시점에 기록 없음.

### 1.5 시작 절차 (z_phone_stereo.py:38-48, 186-208 그대로, adb 만 Windows adb.exe)
1. `adb shell pm enable com.camera.dualstream` (현재 폰에서 disabled — 최초 1회).
2. `adb shell am force-stop com.camera.dualstream`
3. `adb shell run-as com.camera.dualstream rm -f files/status.txt files/cameras.json`
4. `adb shell am start -S -n com.camera.dualstream/.MainActivity --es logical 0 --es physical 2,5 --ei width 640 --ei height 480 --ei fps 30 --ef focus_diopters 1.0 --ez raw_ts false` (+ 새 인자 `--ei imu_us 2500`, 2절)
   - 인텐트 extra: `logical`(str), `physical`(쉼표 목록 str, 2~3개, 서로 달라야 함), `width`/`height`(int, 기본 640/480), `fps`(int, 기본 15; `[fps,fps]` AE 범위가 있을 때만 적용), `focus_diopters`(float, 기본 1.0 = 1 m 고정 초점, 0 = AF), `raw_ts`(bool).
   - **scan3d 는 `--physical 2 5` 순서로 시작**한다 → 스트림 0 = 렌즈 2 = 보정의 왼쪽(cam1). 이러면 `load_phone_calibration` 의 `swap=False` 이고 FramePairer 기준 스트림이 왼쪽 렌즈가 된다. (z_phone_stereo 기본값 `5 2` 는 swap=True 경로.)
   - fps: VIO 용으로 30 권장 (README 의 30 FPS 실측 735쌍/25 s). 15 도 가능.
5. `adb forward tcp:8765 tcp:8765`
6. 12 s 안에 0.15 s 간격으로 `adb shell run-as com.camera.dualstream cat files/status.txt` 폴링:
   `STREAMING...` → 연결, `FAILED|ERROR|DISCONNECTED|STOPPED` 포함 → 오류. (`run-as` 는 manifest `debuggable=true` 덕분.)
   status 순서: `OPEN logical=.. physical=.. 640x480` → `STREAMING logical=0 physical=[2, 5] focus_diopters=1.0`.
7. 종료 시: `adb shell am force-stop com.camera.dualstream`, `adb forward --remove tcp:8765`.

---------------------------------------------------------------------------------------------------

## 2. 센서 — 현재와 추가할 것

### 2.1 현재 앱이 보내는 센서 (MainActivity.java:99-110)
`registerListener(imu, s, 10000 µs, handler)` — 모두 100 Hz 요청, **카메라 Handler 스레드**에서 콜백.

| Android 타입 | 상수 | 값 | 비고 |
|---|---|---|---|
| 4 | TYPE_GYROSCOPE | wx,wy,wz rad/s | 바이어스 보정된 자이로 |
| 9 | TYPE_GRAVITY | gx,gy,gz m/s² | 융합값 (위쪽을 가리킴, 화면 위로 눕히면 z=+9.81) |
| 10 | TYPE_LINEAR_ACCELERATION | m/s² | 중력 제거 융합값 — VIO 에 쓰면 안 됨 |
| 15 | TYPE_GAME_ROTATION_VECTOR | x,y,z,w(,…) | 융합 자세 |
| 11 | TYPE_ROTATION_VECTOR | | 지자기 포함 |
| 6 | TYPE_PRESSURE | hPa | |
| 100 | (앱) frame_timing | 1.4 참조 | 프레임마다 |
| 101 | (앱) stream_info | [ts_ref] 1.0 = 노출 가운데 (1.4) | 접속 때 + 약 2 s 마다 (1.7-mid) |

**원시 가속도계(TYPE_ACCELEROMETER = 1)는 지금 등록되어 있지 않다.** OpenVINS 는 중력 포함 원시 비력(specific force) + 자이로가 필요하다.

### 2.2 앱 변경 (최소)
`MainActivity.java`
1. `startSensors()` (99-110): 타입 목록에 `Sensor.TYPE_ACCELEROMETER`(1) 추가. 선택: `TYPE_GYROSCOPE_UNCALIBRATED`(16, 6값), `TYPE_ACCELEROMETER_UNCALIBRATED`(35, 6값).
   - accel/gyro 주기는 인텐트 extra `imu_us` (int, 기본 2500 = 400 Hz, 0 = `SensorManager.SENSOR_DELAY_FASTEST`) 로, 나머지 센서는 10000 µs 유지.
   - 등록 시 `s.getMinDelay()`(µs), `s.getMaxDelay()`, `s.getName()`, `s.getVendor()`, `s.getFifoMaxEventCount()` 를 로그와 `files/sensors.json` 에 남긴다 (PC 에서 `run-as cat` 으로 읽어 실제 가능 주기 확인).
   - 주석 106행("100 Hz: below the 200 Hz threshold")을 새 동작에 맞게 고친다.
2. 센서 콜백을 카메라 Handler 와 분리: `HandlerThread imuThread = new HandlerThread("imu", Process.THREAD_PRIORITY_URGENT_DISPLAY)` 를 만들어 `registerListener(..., imuHandler)`. 이유: `receive()` 의 화소 단위 NV21 복사 루프가 카메라 스레드를 점유한다. `stop()` 에서 `imuThread.quitSafely()`.
3. `imuPending` 용량 1024 → 4096 (400 Hz × 2 + 100 Hz × 6 + 30 Hz ≈ 1.4 k/s).
4. 패킷 형식은 바꾸지 않는다 (SENS, index = 타입, float32 값). PC `read_frame` 수정 불필요.

`AndroidManifest.xml`
- `<uses-permission android:name="android.permission.HIGH_SAMPLING_RATE_SENSORS" />` 추가. targetSdk 35 (Android 12+ 대상)에서 이 권한이 없으면 `registerListener` 주기가 200 Hz 로 제한된다. normal 권한이라 설치 시 자동 허용 (런타임 요청 불필요).
  출처: https://developer.android.com/develop/sensors-and-location/sensors/sensors_overview#sensors-rate-limiting

빌드·설치 (WSL 에 javac 11, keytool 있음, `build_android.py` 는 `javac -source 8 -target 8`):
- `python3 phone_stereo/build_android.py` → `phone_stereo/android/build/phone-stereo.apk`. SDK 를 `tools/android-sdk` 에 받는데, `~/android-sdk` 에 이미 `android-35/android.jar`, `android-15/d8` 가 있으므로 `ln -s ~/android-sdk tools/android-sdk` 로 다운로드를 건너뛸 수 있다 (존재 검사: build_android.py:28).
- 설치는 Windows adb: `adb.exe install -r -g $(wslpath -w .../phone-stereo.apk)`. 새 debug.keystore 로 서명되면 폰에 깔린 기존 앱과 서명이 달라 `INSTALL_FAILED_UPDATE_INCOMPATIBLE` 이 날 수 있다 → `adb uninstall com.camera.dualstream` 후 설치 (앱 데이터는 status/cameras.json 뿐). 설치 후 `pm enable` 이 다시 필요할 수 있다.

### 2.3 PC 쪽 IMU 동기화 (phone_link)
- 상수: `SENSOR_ACCEL=1, SENSOR_GYRO=4, SENSOR_GYRO_UNCAL=16, SENSOR_ACCEL_UNCAL=35, SENSOR_FRAME_TIMING=100` (scan3d 안에 정의, lib 는 안 고침).
- OpenVINS `ImuData` 는 같은 시각의 (wm, am) 쌍이다. Android 는 두 센서 시각이 다르다 → **자이로를 기준 시계로, 가속도를 그 시각에 선형 보간**. 앞뒤 가속도 샘플이 없으면 그 자이로 샘플은 대기, 50 ms 넘게 비면 버림. 출력 시각은 단조 증가해야 한다 (역행 샘플 폐기).
- 초기 시각 `t0_ns` = 첫 IMU 샘플. 이후 모든 시각을 `t = (t_ns - t0_ns) * 1e-9` (double 초)로 바꿔 VIO 에 보낸다.
- 단위: gyro rad/s, accel m/s² (중력 포함, 정지 시 크기 ≈ 9.81, 위쪽 방향 +). OpenVINS 규약과 같은 부호 (비력).

---------------------------------------------------------------------------------------------------

## 3. 스테레오 보정

### 3.1 `calib_phone_pair.json` (저장소 루트)
| 키 | 값/형 | 뜻 |
|---|---|---|
| `phone` | `{"logical":"0","physical":["2","5"]}` | **왼쪽(cam1)=렌즈 2 초광각, 오른쪽(cam2)=렌즈 5 광각** |
| `K1` | 3x3, fx 261.99, fy 262.29, cx 324.08, cy 238.67 | 렌즈 2, 640x480 기준 |
| `dist1` | `[k1,k2,p1,p2,k3]` OpenCV 순서 | 렌즈 2 |
| `K2` | 3x3, fx 432.12, fy 432.77, cx 321.23, cy 242.71 | 렌즈 5 |
| `dist2` | OpenCV 5계수 | 렌즈 5 |
| `R`, `T` | 3x3, 3 (m) | `X2 = R·X1 + T` (stereoCalibrate 규약). T ≈ [-0.01553, -0.00014, 0.00034] |
| `image_size` | `[640, 480]` | 이 크기 영상만 허용 (`scaled_from [960,720]`) |
| `baseline_m` | 0.015530 | 공장값 `factory_baseline_m` 0.015760 |
| `R_sensor_to_left` | 3x3 | Android 센서 좌표 → 렌즈 2 카메라 좌표 (렌즈 2 공장 pose_rotation 그대로, 차이 1.7e-6°) |
| `R_sensor_to_right` | 3x3 | → 렌즈 5 카메라 좌표 (공장값). 보드 R 과는 0.43° 어긋남: `R @ R_sensor_to_left` vs `R_sensor_to_right` |
| `focus_m` | 1.0 | 보정 때 고정 초점 1 m → 앱 `focus_diopters 1.0` 로 같게 실행해야 함 (다르면 광각 fx 가 변함) |
| `scale_status` | `"metric"` | 보드 stereoCalibrate, `board.rms_px` 0.26, 25 장, square 15.95 mm, pattern 15x6, 내부 파라미터 고정 |
| `pose_source`, `warnings`, `board.*`, `history` | 기록용 | |

### 3.2 불러오기·정류 API
```python
from lib_phone_depth import load_phone_calibration
import lib_rectify as lr
st, swap = load_phone_calibration("calib_phone_pair.json", logical="0", physical=["2","5"], size=(640,480))
# swap=False (physical 2 5 순서일 때). 렌즈 집합이나 image_size 가 다르면 ValueError.
cal = [dict(K=np.array(st[f"K{i}"]), dist=np.array(st[f"dist{i}"]), image_size=st["image_size"]) for i in (1, 2)]
rp = lr.rectify_maps(cal[0], cal[1], np.array(st["R"]), np.array(st["T"]), alpha=0.0)
rectL, rectR = lr.rectify_pair(bgr_lens2, bgr_lens5, rp)      # gray 도 됨 (2D 면 INTER_NEAREST 라 VIO 용은 BGR 정류 후 gray 변환 권장)
Z = lr.disparity_to_depth(disp, rp)                             # 정류 좌표 깊이 m, 무효 NaN
P_cam1 = lr.depth_to_points_cam1(Z, rp)                         # (H,W,3) 원래 렌즈 2 카메라 좌표
```
- `rp` 키: `R1, R2, P1, P2, Q, roi1, roi2, size, mapL, mapR, f, cx, cy, tx_m, baseline_m`.
- 이 보정의 값 (계산, 측정 아님): `f = 462.90 px`, `cx = 339.48`, `cy = 239.87`, `baseline_m = 0.015530`, `roi1 = roi2 = (0,0,640,480)`.
  정류 영상 가로 화각 ≈ 2·atan(320/462.9) ≈ 69°. 1 m 에서 시차 ≈ 7.2 px, 2 m ≈ 3.6 px → 시차 0.25 px 오차가 2 m 에서 깊이 ≈ 7% (추정). **맵 적분 깊이 범위 기본 0.3~2.5 m** 권장.
- 좌표계:
  - 정류 왼쪽(rect0) 좌표: OpenCV 카메라 (x 오른쪽, y 아래, z 앞). 깊이맵·FFS 시차·YOLO 마스크 모두 이 영상 기준.
  - `R1` = 원래 cam1(렌즈 2) → rect0 회전. `p_rect0 = R1 @ p_cam1`, `depth_to_points_cam1` 은 `R1ᵀ` 를 곱해 cam1 로 돌려준다.
  - rect1 (렌즈 5 정류): `R2` = cam2 → rect1. rect1 원점은 rect0 에서 `(+baseline, 0, 0)` (P2[0,3] = f·Tx, Tx<0).
  - **scan3d 맵 적분은 rect0 좌표 점을 쓴다** (`p_rect0 = [ (u-cx)/f·Z, (v-cy)/f·Z, Z ]`). VIO 의 cam0 도 rect0 로 정의하므로 (9절) 포즈와 바로 맞는다. `depth_to_points_cam1` 을 쓰면 `R1` 을 다시 곱해야 한다.
- 유효 화소: `z_object_depth.valid_mask(disp, rp, z_range, min_disp=0.5) -> (valid: bool HxW, Z)` — 시차 양수, 오른쪽 대응점이 roi2 안, 깊이 범위 안, roi1 안.
- 자이로 skew 보상: `lib_phone_depth.skew_rotation(st, imu)` 는 두 렌즈 노출 시각차 동안 회전으로 R 을 고쳐 정류표를 다시 만든다 (PhoneDepth.__call__ 이 0.02° 넘을 때 사용). scan3d 깊이 경로는 이를 그대로 따라 해도 되고, VIO 경로는 고정 `rp`(rp0) 를 쓴다.
- 첫 쌍 정류 검증: `lr.epipolar_check(rectL, rectR)` → `dy_med < 1.0 px` 이고 `inlier_frac >= 0.7` (z_object_depth `EPI_GATE_PX`, `EPI_INLIER_FRAC`). 실패면 시작 중단.

---------------------------------------------------------------------------------------------------

## 4. Camera2 렌즈 자세 → 카메라-IMU 회전 초기값

### 4.1 `phone_stereo/cameras_s24.json`
앱 `inventory()` 출력 (논리 카메라 배열, 각자 `physical` 배열). 후면 논리 0 (`facing 1`, sync_type 0, timestamp_source 1):

| 물리 | focal mm | pose_rotation (x,y,z,w) | pose_translation (m) | pose_reference | min_focus_diopters |
|---|---|---|---|---|---|
| 2 초광각 | 2.2 | [0.70368, -0.71051, 0.00052, -0.00107] | [0, 0.015760, 0] | 0 | 0 (고정 초점) |
| 5 광각 | 5.4 | [0.70710, -0.70710, 0, 0] | [0, 0, 0] | 0 | 10 |
| 6 망원 | 7.0 | [0.71052, -0.70365, -0.00078, -0.00476] | [0, -0.016596, 0] | 0 | 2 |

- 쿼터니언 → 행렬: `lib_phone_calib.quat_to_R(q)` (x,y,z,w; NDK 문서 식). `v_cam = quat_to_R(q) @ v_sensor` 즉 **Android 센서(기기) 좌표 → 카메라 좌표** 회전 (`R_sensor_to_*` 와 같은 뜻, lib_phone_calib.py:119).
  예: 렌즈 5 → `[[0,-1,0],[-1,0,0],[0,0,-1]]` : 기기 z(화면 밖) = 카메라 -z (후면), 기기 x(오른쪽) = 영상 -y(위).
- `LENS_POSE_REFERENCE = 0 (PRIMARY_CAMERA)`: translation 은 주 카메라(렌즈 5) 광학 중심 기준, 센서 좌표계 축으로 표시. IMU(자이로) 위치는 주어지지 않는다 → 카메라-IMU 평행이동은 모름.
  출처: https://developer.android.com/reference/android/hardware/camera2/CameraCharacteristics#LENS_POSE_REFERENCE , #LENS_POSE_ROTATION
- 카메라 좌표 축: x = 센서 긴 변(영상 가로), y = 짧은 변(영상 아래), z = 광축. OpenCV 규약과 같다 (보정 R_sensor_to_left 가 이 가정으로 gravity_frame 에 쓰이고 있음).
- Android 센서 좌표계 (SensorEvent): 기기를 세로로 정면에서 볼 때 **x 오른쪽, y 위, z 화면 밖(사용자 쪽)**. 출처: https://developer.android.com/develop/sensors-and-location/sensors/sensors_overview#sensors-coords

### 4.2 OpenVINS 카메라-IMU 외부 파라미터 초기값 (IMU 프레임 = Android 센서 프레임)
- rect0 (렌즈 2 정류): `R_I→C0 = R1 @ R_sensor_to_left` (계산값 ≈ `[[-0.0115,-0.9996,0.0263],[-0.9999,0.0115,-0.0024],[0.0021,-0.0264,-0.9997]]`).
- rect1 (렌즈 5 정류): `R_I→C1 = R2 @ R @ R_sensor_to_left` (이론상 R_I→C0 와 같음; 보드 R 을 쓰면 정류 정의와 일관).
- kalibr `T_imu_cam` 은 cam→IMU: 회전 `R_C→I = R_I→Cᵀ`, 평행이동 `p_CinI`.
  - `p_C0inI` 초기값 = [0,0,0] (모름, 폰 크기상 수 cm 이내 추정). `p_C1inI = p_C0inI + R_C0→I @ [baseline_m, 0, 0]`.
  - `calib_cam_extrinsics: true` 로 온라인 보정에 맡긴다.
- 시간 오프셋 초기값 0 (같은 시계), `calib_cam_timeoffset: true`.

---------------------------------------------------------------------------------------------------

## 5. FFS 호출 (`lib_ffs.py`)
```python
import lib_ffs
model = lib_ffs.load(ckpt=lib_ffs.DEFAULT_CKPT, valid_iters=8, max_disp=192)   # (ckpt, iters, max_disp) 키로 캐시
disp = lib_ffs.infer(model, rgbL, rgbR, valid_iters=8, scale=1.0, hiera=False) # float32 HxW px, <=0 무효
```
- 체크포인트: `tools/Fast-FoundationStereo/weights/c/model_best_bp2_serialize.pth` (+ 같은 폴더 `cfg.yaml`). 저장소 코드(`core/`)가 import 되어야 언피클됨 → lib_ffs 가 `tools/Fast-FoundationStereo` 를 sys.path 에 넣는다.
- 입력: **정류된 RGB uint8 HxWx3 두 장, 같은 크기** (BGR 이면 `cv2.cvtColor(..., COLOR_BGR2RGB)`), 왼쪽 = rect0 (렌즈 2), 양의 시차 규약. 정규화 없이 0~255 float 로 넣음, divis_by 32 패딩.
- 출력: 원본 해상도 시차 (scale<1 이면 내부 축소 후 되돌림, `/ (W/W0)`).
- 속도 옵션:
  - `valid_iters`: GRU 반복. 기존 라이브 기본 4 (`z_phone_stereo --iters 4`), lib 기본 8. **`load` 와 `infer` 에 같은 값**을 줄 것 (load 가 model.args.valid_iters 를 고정).
  - `scale` (0,1]: 0.5~0.75 로 줄이면 빠르고 VRAM 적게 (640x480 은 1.0 으로 충분할 것으로 추정).
  - `hiera=True`: `model.run_hierachical(..., small_ratio=0.5)` 고해상도용. 640x480 에선 불필요.
  - `max_disp=192`: 이 리그는 0.3 m 에서 시차 ≈ 24 px 이라 넉넉함.
- triton: `lib_ffs.triton_usable()` = 현재 파이썬 include 에 `Python.h` 가 있으면 triton/torch.compile 경로, 없으면 `sys.modules["triton"]=None` + dynamo 비활성 (순수 PyTorch, `optimize_build_volume='pytorch1'`). 새 venv_ffs 는 uv 의 python-build-standalone 3.12.15 라 헤더가 포함되어 triton 경로가 켜질 가능성이 높다 (미확인: `venv_ffs/bin/python -c "import lib_ffs;print(lib_ffs.triton_usable())"`). triton 경로는 첫 호출에 컴파일 시간이 길 수 있으니 시작 때 더미 1회 워밍업.
- lib_ffs docstring 의 "torch 2.6 cu124" 는 옛 venv 기준. 지금은 torch cu128 (RTX 5080 sm_120). `torch.OutOfMemoryError` 는 RuntimeError 로 다시 던져짐.
- GPU 공유: FFS·YOLO 는 같은 프로세스·같은 CUDA 장치. 동시에 부르지 말고 워커 스레드 1개에서 순서대로.

---------------------------------------------------------------------------------------------------

## 6. YOLO 분할 (`lib_detect.py`)
```python
import lib_detect as ld
dets = ld.detect_yolo(rect_left_bgr, model=ld.DEFAULT_SEG_MODEL, conf=0.25, classes=None, device="0", max_det=20)
# -> [{"box": (x, y, w, h) int, "label": str, "conf": float, "mask": HxW uint8 0/255 | None}, ...] conf 내림차순
```
- 기본 모델: `DEFAULT_SEG_MODEL = "yolo11m-seg.pt"` (마스크), `DEFAULT_MODEL = "yolo11m.pt"` (박스만). 첫 사용 때 ultralytics 가 내려받는다 (인터넷 필요, `*.pt` 는 .gitignore).
- `retina_masks=True` 라 마스크는 입력 영상 해상도 그대로 (구멍 보존). 해상도가 안 맞으면 mask=None.
- **반환에 클래스 id·트랙 id 가 없다** (label 문자열만). 클래스 id 는 `ld._MODEL_CACHE[model].names` (id→이름) 역매핑으로 얻는다.
  인스턴스를 프레임 사이에 이어 주려면 scan3d 에 자체 래퍼(`depth_seg.py`)를 두고 `YOLO(model).track(img, persist=True, retina_masks=True, conf=0.25, verbose=False)` 로 `boxes.id`(2D 트랙 id)와 `boxes.cls` 를 함께 받는다. 마스크 처리 규약(retina, 0/255, 크기 불일치면 None)은 detect_yolo 와 같게.
- 기존 사용처: `PhoneDepth.__call__` 이 **정류된 왼쪽 BGR** 에 detect_yolo 를 돌리고 `z_object_depth.aggregate(rec, region, valid, Z, points, rp, outline)` 로 물체별 깊이·중심·치수를 낸다 (lib_phone_depth.py:171-200). `z_object_depth.draw_overlay(rect_bgr, dets, rows, mode)` 는 2D 오버레이.
- 속도: README 실측 bbox 모델 GPU 중앙값 11.26 ms (640x480). seg 모델은 그보다 느림 (값 없음).

---------------------------------------------------------------------------------------------------

## 7. 재사용할 기존 뷰어·도구
| 모듈 | 내용 | scan3d 에서 |
|---|---|---|
| `lib_rgbd_viewer.RGBDViewer` | OpenCV 창 (cv2.imshow) RGB/깊이/점군/IMU 대시보드, snapshot dict | 직접 쓰지 않음 (PC 웹 뷰어 원칙). snapshot 키 구성(rgb, depth, valid, points, K_rect, R1, timestamp_ns)만 참고 |
| `lib_phone_depth.depth_colormap(depth, valid, zmin, zmax)` | 로그 스케일 JET 깊이 그림 + 눈금 | 웹 뷰어 2D 깊이 패널 PNG 로 재사용 |
| `z_object_depth.draw_overlay` / `ld.draw` | 마스크 윤곽 + 라벨 2D 오버레이 | 웹 뷰어 2D 카메라 패널 (JPEG 로 보냄) |
| `lib_pointcloud_view.write_viewer(path, points, colors, metadata)` | 의존성 없는 canvas 궤도 뷰어 HTML (객체 목록 토글, 궤적 `T_world_camera`) | 저장(Export) 기능: 복셀 중심점+색, `metadata={"objects":[{id,label,points,center_m}], "trajectory":[{T_world_camera}]}` |
| `lib_pointcloud_export.write_ply(path, points, colors)` | PLY 저장 | 저장 기능 |
| `lib_phone_depth.gravity_frame` | 중력 정렬 프레임 | 불필요 (OpenVINS 월드가 이미 중력 z-up) |

---------------------------------------------------------------------------------------------------

## 8. Windows adb ↔ WSL 네트워크

### 8.1 확인한 환경 (2026-10-16)
- WSL 2.6.3.0, 커널 6.6.87.2. `wslinfo --networking-mode` = **nat**.
- `/etc/wsl.conf` = `[boot] systemd=true` 만. `C:\Users\a3162\.wslconfig` **없음** → 기본값 `networkingMode=nat`, `localhostForwarding=true`.
- WSL eth0 172.29.129.154/20, 기본 게이트웨이 = Windows 쪽 `vEthernet (WSL (Hyper-V firewall))` 172.29.128.1/20.
- `/etc/resolv.conf` nameserver 10.255.255.254 (DNS 터널링) → **resolv.conf 로 호스트 IP 를 구하면 틀린다**. 호스트 IP 는 `ip route show default | awk '{print $3}'`.
- WSL interop 켜짐 (`/proc/sys/fs/binfmt_misc/WSLInterop` enabled) → WSL 에서 Windows exe 실행 가능.
  - adb: `/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/android/platform-tools/adb.exe`
  - Windows Python: `/mnt/c/Users/a3162/AppData/Local/Programs/Python/Launcher/py.exe` (`py -3`, 3.13.14), Python311/312 도 있음.
  - 경로는 환경변수 `SCAN3D_ADB`, `SCAN3D_WINPY` 로 덮어쓸 수 있게.

### 8.2 문제
`adb forward tcp:8765` 는 Windows 127.0.0.1:8765 에만 바인딩한다. WSL(NAT) 의 127.0.0.1 은 WSL 자신이라 접근 불가, 172.29.128.1:8765 에도 열려 있지 않다. WSL 안에서는 USB(폰) 가 보이지 않는다.

### 8.3 결정: 역방향 릴레이 `scan3d/win_relay.py` (Windows, 표준 라이브러리만)
WSL2 localhost forwarding 은 **Windows → WSL** 방향(WSL 에서 0.0.0.0/127.0.0.1 로 listen 한 포트가 Windows localhost 에 보임)으로 동작한다. 따라서 릴레이가 두 쪽 모두에 **바깥으로 connect** 하면 Windows 방화벽 인바운드 규칙·관리자 권한이 필요 없다.

절차·규약:
1. WSL `phone_link` 가 `0.0.0.0:8766` 에 listen (포트는 `--relay-port`, 기본 8766).
2. `phone_link` 가 interop 으로 릴레이를 띄운다: `py.exe -3 \\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg\scan3d\win_relay.py --wsl-port 8766 --phone-port 8765` (실패하면 사용자에게 Windows 에서 직접 실행하라고 안내). 릴레이 stdout/stderr 는 phone_link 가 로그로 받는다.
3. 릴레이: `127.0.0.1:8766` 에 connect (0.5 s 간격 재시도, 최대 30 s) — 이것이 WSL 로 가는 터널.
4. `phone_link` 가 1.5 절 1~6 (adb.exe 로 앱 시작, forward, status.txt 대기) 을 끝내고 `STREAMING` 을 보면, 릴레이 연결에 **1 바이트 `b"G"`** 를 보낸다.
5. 릴레이는 `G` 를 받으면 `127.0.0.1:8765` (adb forward) 에 connect 하고 폰→WSL 방향 바이트를 그대로 복사한다 (`recv_into` 1 MiB 버퍼, `TCP_NODELAY`, 해석·재조립 없음). WSL→폰 방향도 그대로 폰에 넘긴다 (H.264 키프레임 요청 `keyframe <n>`, 1.2.2).
6. 어느 쪽이든 EOF/오류 → 양쪽 소켓 닫고 3번으로 돌아감 (재시작 가능). `b"Q"` 를 받으면 릴레이 종료.
7. 릴레이 자체는 adb 를 부르지 않는다 (adb 제어는 전부 WSL 쪽 phone_link 가 interop 으로).

phone_link 에서 보이는 소켓 = 폰 소켓과 바이트 단위로 같으므로 `read_frame(sock, 2, (640,480))` 를 그대로 쓴다.

대안 (기본 아님, 실패 시): 정방향 — 릴레이가 Windows `172.29.128.1:8765` 에 listen, WSL 이 `$(ip route show default | awk '{print $3}'):8765` 로 connect. Windows Defender 방화벽 인바운드 허용(첫 실행 프롬프트 또는 관리자 `New-NetFirewallRule`)과 Hyper-V 방화벽 설정이 필요할 수 있어 기본에서 뺐다.

### 8.4 웹 뷰어 접근
WSL 서버를 `0.0.0.0:8080` 에 띄우면 Windows 브라우저에서 `http://localhost:8080` 으로 열린다 (NAT + localhostForwarding 기본값 true, .wslconfig 없음). 같은 이유로 3번의 `127.0.0.1:8766` 도 성립한다. 이 동작은 설정상 기본값으로 판단한 것이며 이 명세 작성 중 실제 연결로 확인하지는 않았다.
출처: https://learn.microsoft.com/en-us/windows/wsl/networking , https://learn.microsoft.com/en-us/windows/wsl/wsl-config#wslconfig

---------------------------------------------------------------------------------------------------

## 9. VIO: OpenVINS (ROS 없음) 브리지

### 9.1 빌드 상황
- 소스 `~/scanner/src/open_vins` (master, 6948812), 빌드 `~/scanner/src/ovbuild` (`cmake -S open_vins/ov_msckf -DENABLE_ROS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=~/scanner/local`). 결과물 `libov_msckf_lib.so` (ov_core, ov_init 소스 포함), `run_simulation` 등. ROS 실행 파일은 안 만들어짐.
- 의존: 시스템 OpenCV 4.5.4 (`/usr/lib/x86_64-linux-gnu/cmake/opencv4`), Eigen3 (`/usr/share/eigen3`), Boost 1.74, Ceres 2.2 (`~/scanner/local`, MINIGLOG, 공유 라이브러리 → 실행 시 `LD_LIBRARY_PATH=~/scanner/local/lib` 또는 RPATH).
- 설치 스크립트 `~/setup_scanner_wsl.sh`, 로그 `~/scanner_setup.log` 끝 `SETUP_DONE`. 그 전에는 C++ 빌드·venv 사용 금지.

### 9.2 `scan3d/vio/ov_bridge.cpp` (C++17)
`CMakeLists.txt`: include `~/scanner/src/open_vins/ov_msckf/src`, `.../ov_core/src`, `.../ov_init/src`, Eigen, OpenCV, Boost, Ceres; link `~/scanner/src/ovbuild/libov_msckf_lib.so` + OpenCV + Boost(filesystem, system) + Ceres; `BUILD_RPATH` 에 ovbuild, `~/scanner/local/lib`. 실행: `ov_bridge <estimator_config.yaml>`.

사용 API (VioManager.h, sensor_data.h):
```cpp
auto parser = std::make_shared<ov_core::YamlParser>(config_path);
ov_msckf::VioManagerOptions params; params.print_and_load(parser);
auto sys = std::make_shared<ov_msckf::VioManager>(params);
ov_core::ImuData imu; imu.timestamp=t; imu.wm=...; imu.am=...; sys->feed_measurement_imu(imu);
ov_core::CameraData cam; cam.timestamp=t; cam.sensor_ids={0,1}; cam.images={g0,g1};
cam.masks={cv::Mat::zeros(h,w,CV_8UC1), ...};   // 카메라마다 마스크 필수 (0 = 사용)
sys->feed_measurement_camera(cam);              // sensor_ids={0} 하나면 mono 추적 (TrackKLT::feed_new_camera num_images==1)
sys->initialized(); auto s = sys->get_state();  // s->_imu->quat() JPL q_GtoI, ->pos() p_IinG, ->vel(), ->bias_g(), ->bias_a()
// s->_calib_IMUtoCAM.at(i)->Rot() = R_ItoC, ->pos() = p_IinC ; s->_calib_dt_CAMtoIMU->value()(0)
sys->get_features_SLAM();                       // 월드 좌표 SLAM 특징점 (뷰어 표시용)
```
- 카메라 메시지는 **그 시각 이후의 IMU 가 이미 들어간 뒤**에 feed 한다 (ROS 노드의 camera_queue 규칙과 같음). 이 대기는 파이썬 쪽(vio_client)이 보장: 마지막으로 보낸 IMU 시각 ≥ t_cam 일 때만 카메라 전송.
- 카메라 처리가 밀리면(파이프 대기 2 프레임 초과) 카메라 프레임만 버린다. IMU 는 절대 버리지 않는다.

### 9.3 파이프 프로토콜 (vio_client.py ↔ ov_bridge, 같은 기계라 little-endian)
stdin 메시지 = 헤더 `<III` (magic `0x4F564252` "OVBR", type, payload_len) + payload:

| type | 이름 | payload |
|---|---|---|
| 1 | IMU | `<d3d3d` t(s), wx,wy,wz (rad/s), ax,ay,az (m/s²) — 56 B |
| 2 | CAM | `<dIIB3x` t(s), w, h, n_cams(1 또는 2) + n_cams × (w·h) gray uint8 (cam0=rect0 렌즈 2, cam1=rect1 렌즈 5) |
| 3 | RESET | 없음 — VioManager 재생성 |
| 4 | QUIT | 없음 |

stdout: CAM 1개 처리마다 JSON 한 줄 (stderr = OpenVINS 로그):
```json
{"t": 12.345, "init": true, "T_world_cam0": [16 floats, row-major, cam0 점 → 월드],
 "q_GtoI_jpl": [x,y,z,w], "p_IinG": [..], "v_IinG": [..], "bg": [..], "ba": [..],
 "R_ItoC0": [9], "p_IinC0": [3], "dt_cam_imu": 0.0, "n_slam": 37, "n_msckf": 80, "ms": 9.1}
```
- `T_world_cam0` 계산 (C++ 에서, JPL 혼동 방지): `R_GtoI = ov_core::quat_2_Rot(q)` (JPL), `R_ItoG = R_GtoIᵀ`; 추정된 `R_ItoC, p_IinC` 로 `R_C→G = R_ItoG · R_ItoCᵀ`, `p_CinG = p_IinG − R_C→G · p_IinC`. `init=false` 면 `T_world_cam0` 은 null.
- 월드 G: 중력 정렬 **z 위**, yaw·원점은 초기화 시점 기준. 맵·뷰어도 이 좌표 (z-up) 를 그대로 쓴다.

### 9.4 카메라 입력 규칙
- cam0/cam1 = **정류된** 영상 (`rectify_pair` 후 gray). 이유: 두 렌즈 fx 가 262 vs 432 로 1.65 배 달라 원영상끼리 KLT 스테레오 매칭이 어렵다. 정류 후 둘 다 f=462.9 핀홀, 왜곡 0.
- 스테레오 전송 조건: 쌍의 물리 timestamp 차 `|t5 − t2| <= 1 ms` (`--vio-stereo-skew-ms`, 기본 1.0). 넘으면 cam0 만 보냄 (n_cams=1). VIO 는 쌍마다 한 timestamp 만 받기 때문.
- 카메라 시각: `t_cam` = 왼쪽 렌즈 노출 가운데 시각 = 앱 1.7-mid 헤더 timestamp 그대로 (보정 0). 옛 앱만 `t_lens2_ns + 0.5·(exposure_ms + rolling_skew_ms)·1e6` (그 프레임 frame_timing 값, 없으면 최신값, 프레임 간격 안으로 자름) → `(t − t0_ns)·1e-9`. 나머지 상수 오프셋은 OpenVINS `calib_cam_timeoffset` 가 추정.

### 9.5 설정 파일 (calib_vio.py 가 생성, `scan3d/vio/config/phone_s24/`)
- `estimator_config.yaml`: config/euroc_mav 것을 바탕으로 `use_stereo: true`, `max_cameras: 2`, `calib_cam_extrinsics: true`, `calib_cam_intrinsics: false` (정류 영상이라 고정), `calib_cam_timeoffset: true`, `gravity_mag: 9.81`, `init_dyn_use: false` (정지 초기화: 시작 때 폰을 2 s 정지 후 움직이기, `init_window_time 2.0`, `init_imu_thresh` 는 폰 잡음에 맞춰 조정), `track_frequency` = 실제 fps (30), `num_pts 200`, `histogram_method HISTOGRAM`, `use_mask: false`, `relative_config_imu`, `relative_config_imucam`, `filepath_*` 는 `scan3d/runs/` 아래.
- `kalibr_imucam_chain.yaml`: cam0/cam1 각각 `T_imu_cam` (4.2), `cam_overlaps`, `camera_model: pinhole`, `distortion_model: radtan`, `distortion_coeffs: [0,0,0,0]`, `intrinsics: [f, f, cx, cy]` (rp 값), `resolution: [640, 480]`. rostopic 은 무시되지만 키는 둔다.
- `kalibr_imu_chain.yaml`: `update_rate` = 실제 IMU 주기 (sensors.json/수신 통계), 잡음 초기값 (폰 MEMS 추정, 조정 대상): `accelerometer_noise_density 1.0e-2`, `accelerometer_random_walk 1.0e-3`, `gyroscope_noise_density 1.0e-3`, `gyroscope_random_walk 1.0e-4`, `model: kalibr`, Tw/Ta/R_IMUtoGYRO/R_IMUtoACC 단위행렬, Tg 0.
- 생성 시각·입력(calib 경로, rp f/cx/cy/baseline) 을 yaml 주석에 기록.

---------------------------------------------------------------------------------------------------

## 10. 깊이 → 복셀 맵 → 물체

### 10.1 키프레임 깊이 (depth_seg 워커, 스레드 1개, GPU)
- 키프레임 조건: VIO `init=true` 이고 직전 키프레임 대비 이동 ≥ 5 cm 또는 회전 ≥ 5°, 또는 0.5 s 경과 (모두 인자). 워커가 바쁘면 건너뜀 (최신 쌍만).
- 처리: rectify(BGR) → FFS (iters 4, scale 1.0 기본) → `valid_mask(disp, rp, (0.3, 2.5))` → rect0 점 → YOLO-seg(track) on rectL.
- 포즈 매칭: 키프레임의 `t_cam` 과 같은 VIO 출력의 `T_world_cam0` (vio_client 가 t → pose 사전 유지; 정확히 같은 t 가 없으면 키프레임 취소).

### 10.2 `voxel_map.VoxelMap` 인터페이스
```python
vm = VoxelMap(voxel_m=0.02, max_points_per_frame=200_000)
vm.integrate(points_world: (N,3) f32, colors: (N,3) u8, class_id: (N,) i16 (-1=배경), inst_id: (N,) i32 (-1), cam_center_world: (3,), frame_id: int)
vm.delta(since_version: int) -> (version, keys(M,3) i32, rgb(M,3) u8, cls(M,) i16, inst(M,) i32, hits(M,) u16)   # 바뀐 복셀만
vm.objects() -> list[dict(id, class_id, label, color, n_voxels, box_center[3], box_size[3], yaw, conf, last_seen)]
vm.reset(); vm.save(dir)
```
- 키 = `floor(p / voxel_m)` int32×3. 저장: dict/해시 (numpy 패킹 int64 키, 또는 torch GPU `unique`). 복셀당: 색 누적 평균, hit 수, 클래스 투표 히스토그램(작은 고정 크기, 최다 득표), 인스턴스 투표.
- 표시 기준: `hits >= 2` (잡음 제거, 인자).
- 물체 인스턴스 3D 연결: 프레임의 마스크 점(이상치 제거: 깊이 중앙값 ±20%) → 월드 AABB/중심. 같은 클래스 기존 인스턴스와 중심 거리 < 0.3 m 또는 3D IoU > 0.2 면 같은 id, 아니면 새 id. 2D 트랙 id 가 있으면 우선 사용.
- 박스: 인스턴스 복셀 중심들의 xy PCA 로 yaw, 2~98 백분위로 크기 (z 는 축 정렬, 월드 z-up). 
- 클래스 색: COCO 80 클래스 고정 팔레트 (ultralytics `Colors()` 와 같은 순서 권장), 배경 = 관측 RGB.

---------------------------------------------------------------------------------------------------

## 11. 웹 서버·뷰어 (PC 쪽 전부, 폰 화면 UI 신경 안 씀)
- `server.py`: 표준 라이브러리 `http.server.ThreadingHTTPServer`, `0.0.0.0:8080` (`--port`). 정적 파일 `scan3d/web/`.
- API:
  - `GET /api/state` → JSON `{t, vio:{init, T_world_cam0, n_slam, ms}, phone:{fps, imu_rates, pairs, dropped, skew_ms, clock_gap_ms, error}, depth:{ms, keyframes}, map:{version, voxels}, objects:[...]}`
  - `GET /api/voxels?since=<version>` → `application/octet-stream`: 헤더 `<IIf` (version, M, voxel_m) + M × `<hhhBBBhiH` (ix,iy,iz int16, r,g,b, class int16, inst int32, hits uint16) 17 B. `since=0` 이면 전체.
  - `GET /api/trajectory` → `[[t, x, y, z], ...]` (cam0 중심, 간격 솎음)
  - `GET /api/frame.jpg?kind=rgb|overlay|depth` → 최근 키프레임 2D 패널 (overlay = draw_overlay, depth = depth_colormap)
  - `POST /api/cmd` JSON `{"cmd": "pause"|"resume"|"reset_map"|"reset_vio"|"save"|"set", ...}` (`set`: voxel_m, depth_max, color_mode "rgb"|"class", min_hits)
- `web/index.html` + `app.js`: three.js (cdnjs 고정 버전, 예 `https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js` + OrbitControls 는 직접 구현하거나 jsdelivr `three@0.128.0/examples/js/controls/OrbitControls.js`). 복셀은 `InstancedMesh` (상자) 또는 `Points`, 200 ms 간격 delta 폴링, 색 모드 토글(RGB/클래스), 물체 박스 `LineSegments` + 라벨(HTML 오버레이), 궤적 선, 현재 카메라 프러스텀, 상태 패널, 2D 패널, 버튼(일시정지/리셋/저장). 좌표: 월드 z-up → three.js 에서 `camera.up = (0,0,1)`.

---------------------------------------------------------------------------------------------------

## 12. 실행 진입점 (제안)
```bash
# WSL, SETUP_DONE 이후
cd /home/system/scanner/StereoDepthSeg
venv_ffs/bin/python -m scan3d.run_scanner --calib calib_phone_pair.json --fps 30 --imu-us 2500 \
    --voxel 0.02 --depth-range 0.3 2.5 --iters 4 --port 8080 --relay-port 8766
# → Windows 브라우저 http://localhost:8080
```
- 시작 순서: 설정 생성(calib_vio) → ov_bridge 실행 → 8766 listen → win_relay 실행(interop) → adb 시작 절차 → `G` → 수신 → 첫 쌍 epipolar_check → FFS/YOLO 워밍업 → 루프.
- 종료(Ctrl-C 또는 웹): 릴레이에 `Q`, `am force-stop`, `adb forward --remove tcp:8765`, ov_bridge QUIT, 맵 저장(선택).
- 산출물: `scan3d/runs/<YYYYmmdd-HHMMSS>/` (`voxels.ply`, `objects.json`, `trajectory.json`, `viewer.html`(write_viewer), `vio_config/` 사본, `summary.json`). 생성 시각·입력 인자를 함께 기록.

## 13. 알려진 위험
- 두 렌즈 노출 동기 APPROXIMATE, 물리 timestamp skew 분포 미측정 → 스테레오 VIO 가 mono 로 자주 떨어질 수 있음 (9.4 규칙으로 안전 쪽).
- baseline 15.5 mm 로 원거리 깊이가 약함 → 깊이 0.3~2.5 m 로 제한.
- 폰 IMU 잡음·바이어스, 원시 가속도 400 Hz 가능 여부(minDelay)는 기기 확인 필요.
- 정지 초기화가 손떨림에 실패할 수 있음 → `init_imu_thresh` 조정 또는 `init_dyn_use: true` 시도.
- 영상이 90° 누운 채라 YOLO 검출률이 떨어질 수 있음 (6절 회전 옵션).
- 앱 재서명 시 기존 설치본과 서명 충돌 가능 (2.2).

---------------------------------------------------------------------------------------------------

## 14. 폰 능력 조회·부하 지표 (`caps.json`, `metrics.json`, 앱 1.3-caps / versionCode 4, 2026-10-16)

소스: `phone_stereo/android/src/com/camera/dualstream/Caps.java`(조회·검사), `Metrics.java`(실행 지표), `MainActivity.java`, `Recorder.java`.
둘 다 **조회·자기 계측만** 한다 (별도 측정 부하 없음). 값은 실기기에서 아직 읽어 보지 않았다 (빌드만 확인).

### 14.1 파일 위치 (run-as 없이 읽기)
| 파일 | 외부 (adb 바로 cat) | 내부 (run-as) | 언제 |
|---|---|---|---|
| caps.json | `/sdcard/Android/data/com.camera.dualstream/files/caps.json` | `files/caps.json` | 앱 시작마다 1회 (인자 없이 시작해도), 배경 스레드 |
| metrics.json | `/sdcard/Android/data/com.camera.dualstream/files/metrics.json` | `files/metrics.json` | 카메라 동작 중 1 s 마다 |
| status.txt | (기존과 같음) | (기존과 같음) | 상태 바뀔 때 |

- 둘 다 `이름.tmp` 에 쓰고 rename → 반쯤 쓴 파일을 읽지 않는다. PC 는 `adb shell cat /sdcard/Android/data/com.camera.dualstream/files/metrics.json`.
- metrics.json 은 앱이 멈추면 더 이상 갱신되지 않는다 → `t_wall_ms` 가 3 s 넘게 안 바뀌면 낡은 값으로 본다.
- status.txt 형식은 바뀌지 않았다 (첫 낱말 규칙 그대로). 추가된 것: `OPEN ... 640x480 fps=15 ...` 에 `fps=` 가 붙고, 아래 14.4 의 `ERROR UNSUPPORTED` 줄.

### 14.2 caps.json (`format: "scan3d-phone-caps/1"`)
조회 대상 렌즈 = 인텐트 `physical` (없으면 `2,5`), 논리 카메라 = `logical` (없으면 `0`).
```
{ "format","written_wall_ms","written_wall","app_version","app_version_code",
  "device": {manufacturer, model, device, hardware, android, sdk, soc_manufacturer, soc_model (API31+), cpu_cores},
  "logical": {id, physical_ids[], logical_multi_camera, sync_type, hardware_level,
              ae_fps_ranges [[lo,hi],...], fixed_fps [15,20,24,30,60] (lo==hi 인 AE 범위)},
  "lenses": { "2": {active_array [l,t,r,b], pixel_array [w,h], active_array_4_3,
                    yuv: [{w,h, aspect_4_3, full_active_array, min_frame_ns, max_fps (=1e9/min_frame_ns, 모르면 null), stall_ns}, ...]},
              "5": {...} },
  "pair_lenses": ["2","5"],
  "pairs": [ {w,h, aspect_4_3, max_fps (두 렌즈 중 작은 값),        // 두 렌즈 모두 가진 YUV 크기, 픽셀 수 큰 것부터
              camera_fps [..]        // FPS 후보 {10,15,20,24,30,60} 중 앱이 실시간으로 받아 주는 값 (14.4 검사 통과)
              record_encoder {"10": "c2.xxx.avc.encoder"|null, "15": ..., ...}   // 녹화(record/both) 때 쓸 인코더, null = 거부
              rec_kbps_default {"15": 4000, ...} }, ... ],                  // 14.5
  "avc_encoders": [ {name, hardware, software_only, vendor (API29+), max_instances, yuv420_flexible,
                     bitrate_range [lo,hi] bps, widths, heights, frame_rates, width_align, height_align,
                     performance_points ["...", ...] (API29+, 하드웨어 인코더 처리 능력 문자열),
                     sizes: [{w,h, size_rate_supported {"10":bool,...,"60":bool}, achievable_fps [lo,hi]|null}]}, ... ],  // 하드웨어 먼저
  "session_support": {api, available, why?,                // API 35 CameraDeviceSetup, 카메라를 열지 않고 HAL 에 물음
                      results: [{w,h, supported (렌즈마다 YUV 출력 1개씩, 한 세션), max_fps_checked, supported_at_max_fps, error?}]} }
```
- S24 의 4:3 공통 크기 (cameras_s24.json 기준): 4000x3000, 1920x1440, 1440x1080, 960x720, 640x480, 320x240. 800x600·1280x960 은 폰이 제공하지 않는다.
  렌즈 2·5 의 active array 가 모두 4:3 (4000x3000, 4080x3060) → 4:3 출력은 센서 전체 영역 (`full_active_array=true`).
- `max_fps` 는 StreamConfigurationMap 의 크기별 최소 프레임 간격에서 나온 값 (출처: developer.android.com/reference/android/hardware/camera2/params/StreamConfigurationMap#getOutputMinFrameDuration(int,%20android.util.Size)).
  두 물리 출력이 한 요청에 묶이므로 쌍의 최대 fps = 1e9 / max(렌즈별 최소 간격). JPEG 압축·USB 한계는 여기 들어 있지 않다 (14.3 실측으로 PC 가 계산).
- 인코더: `areSizeAndRateSupported(w,h,fps)` 와 `getAchievableFrameRatesFor` 는 제조사가 적은 능력값 (developer.android.com/reference/android/media/MediaCodecInfo.VideoCapabilities). 렌즈 2개를 동시에 인코딩하므로 `max_instances ≥ 2` 도 요구한다. 두 인코더 합산 처리량은 `performance_points` 와 실행 중 `rec_encoder_busy` 로 판단.

### 14.3 metrics.json (`format: "scan3d-phone-metrics/1"`, 1 s 마다)
```
{ "format","t_wall_ms","elapsed_ns", "state" (status 첫 낱말), "mode","width","height","fps","physical":["2","5"],
  "stream_fmt" ("h264"|"jpeg"|"nv21"|"none"), "jpeg_q", "window_s": 2.0,
  "rec","rec_kbps","rec_encoder",                       // 녹화 중일 때만
  "lenses": { "2": {
      cam_fps            최근 2 s 카메라 실제 fps (렌즈별 이미지 timestamp 간격)
      cam_frames         누적 받은 이미지 수
      cam_gap_drops      누적 빠진 프레임 수 추정 (timestamp 간격 > 1.5 주기 → round(간격/주기)-1)
      cam_buffer_lost    누적 onCaptureBufferLost
      copy_ms_avg/max    카메라 스레드의 YUV→NV21 복사 시간 (최근 2 s)
      jpeg_fps           최근 2 s JPEG 완료 수 / 2
      jpeg_ms_avg/max    compressToJpeg 시간 (렌즈별 단일 스레드, 최근 2 s)
      jpeg_core_pct      최근 2 s 동안 JPEG 압축이 쓴 한 코어 시간 비율 % (= Σ압축시간 / 2 s × 100)
      jpeg_kb_avg        JPEG 한 장 평균 크기 KiB
      jpeg_frames, jpeg_dropped   누적 (대기 2장 초과·압축 실패로 버린 것)
      rec_frames, rec_fps, rec_dropped_queue, rec_encoder_busy   // 녹화 중: 누적 인코딩 수, 1 s 간 fps, 큐 가득 버림, 인코더 입력 버퍼 없음 버림
    }, "5": {...} },
  "jpeg_core_pct_total",                                // 두 렌즈 합 (100 = 한 코어 꽉 참)
  "capture_failed" (누적 onCaptureFailed), "matcher_unmatched_dropped",
  "usb": {client, bytes_total, MBps (지난 1 s 실제 전송, 헤더 포함), frame_queue_evicted (프레임 큐 6 가득 → 오래된 것 버림)},
  "phone": {proc_cpu_pct_one_core (앱 프로세스 전체 CPU, 한 코어=100), cpu_cores, battery_temp_c,
            thermal_status (PowerManager, 0 없음 … 6 종료), thermal_headroom_10s (1.0 = 스로틀 시작, 10 s 마다 갱신)} }
```
- PC 부하 모델(EWMA) 입력 권장: `jpeg_ms_avg / (w·h)` → 픽셀당 압축 시간, `jpeg_kb_avg·1024·8 / (w·h)` → 비트/픽셀, `usb.MBps`, `rec_encoder_busy` 증가율, `cam_fps` vs `fps`, `thermal_status`.
- 온도: 루트 없이 CPU 온도는 못 읽는다 → 배터리 온도와 thermal status/headroom 으로 대신.

### 14.4 지원 안 되는 조합 = ERROR (가장 가까운 크기로 바꾸지 않음)
시작 시 `Caps.checkCamera` (실시간·녹화 공통) + 녹화(record/both)면 `Caps.findAvcEncoder` 를 통과해야 카메라를 연다:
1. 크기가 짝수·양수, 선택한 모든 렌즈의 YUV_420_888 목록에 있음.
2. `fps ≤ 쌍 max_fps`.
3. 논리 카메라에 고정 AE 범위 `[fps,fps]` 가 있음 (S24: 15, 20, 24, 30, 60 → **10 은 거부**). 예전에는 없으면 조용히 기본 범위로 돌았다.
4. (녹화) H.264 인코더 중 하드웨어 먼저, `areSizeAndRateSupported(w,h,fps)` ∧ YUV420Flexible 입력 ∧ `max_instances ≥ 렌즈 수` 인 첫 인코더. 녹화는 그 인코더를 이름으로 만든다 (`meta.json` 의 `encoder`).

실패하면 카메라를 열지 않고 status.txt 에:
`ERROR UNSUPPORTED 1280x960@30 mode=live reason=lens 2 has no YUV 1280x960 | see caps.json`
(첫 낱말에 `ERROR` → win_relay / phone_link 의 기존 오류 처리 그대로 걸린다.)

### 14.5 녹화 비트레이트 기본
- `rec_kbps` 인자가 있으면 그 값 (렌즈마다). 없으면 `4000 × (w·h·fps) / (640·480·15)` kbps, 최소 500, 인코더 `bitrate_range` 안으로 자름.
  예: 640x480@15 = 4000, 640x480@30 = 8000, 960x720@15 = 9000, 1440x1080@30 = 40500 (범위 안이면). 이전 기본은 크기와 무관하게 8000.
- `meta.json` 에 `rec_kbps`, `rec_kbps_source` ("auto"|"extra"), `encoder` 가 남는다.
  `scan3d/RECORDING_FORMAT.md` 의 "기본 8000" 문구는 이 절로 대체된다 (그 파일은 이 작업에서 고치지 않음).

### 14.6 크기별 보정 (참고, PC 담당)
같은 4:3·센서 전체 영역(`full_active_array=true`)이면 640x480 보정을 비례 변환해 쓴다: fx, fy, cx, cy 에 (새 폭/640) 을 곱하고 왜곡 계수·두 렌즈 R,T 는 그대로.
엄밀히는 픽셀 중심 규약 때문에 cx' = (cx+0.5)·s − 0.5 (s = 크기 비율) 이 맞다 (단순 곱과의 차이 = 0.5·(s−1) px: 960x720 0.25 px, 1920x1440 1 px).

## 15. 렌즈 위상 맞추기 (앱 1.7-mid / versionCode 11: 위상 = 노출 가운데 차이; 1.6-sync / 10: 방법 C; 처음 1.5-sync / 9)

### 15.1 문제 (측정, 메인 세션 2026-10-16 13:40, 앱 1.4.3-h264, 640x480 live h264)
- 논리 카메라 `LOGICAL_MULTI_CAMERA_SENSOR_SYNC_TYPE = 0 (APPROXIMATE)` → 두 물리 렌즈(2, 5)는 하드웨어 동기 없이 따로 돈다.
- 렌즈 5 − 렌즈 2 물리 SENSOR_TIMESTAMP 차이(= 렌즈 위상)는 앱을 켤 때마다 무작위: 15 fps +26.2 ms, 20 fps −1.16 ms, 24 fps −10.26 ms,
  30 fps +5.4 ms. 30 fps 는 두 센서 주기가 조금 달라 약 4 ms/s 로 미끄러지고, 15·20·24 fps 도 0.05~0.08 ms/s.
- PC 쌍 묶기 기준 5 ms 밖이면 짝 0 → "짝 안 맞음 100 %". 기준을 넓히는 건 답이 아님 (기선 15.5 mm: 26 ms × 0.3 m/s = 8 mm 어긋남).
- **1.5-sync 실측 (메인 세션, 15 fps, 25 s)**: 방법 A — S24 는 물리 요청 키에 SENSOR_FRAME_DURATION·AE 키를 다 갖고 있지만 위상이
  따라오지 않음 ("requested −58.7 ms, moved 0.2 ms in 3 s"). 방법 B — 반복 요청 재시작 1번 + 세션 재생성 9번에도 위상 29.2 → 30.2 ms
  (자연 미끄러짐만). 반면 앱을 껐다 켜면 (CameraDevice close → open) 위상이 무작위로 바뀜. → 센서는 장치가 열려 있는 동안 계속 돌고,
  위상은 장치를 열 때 정해진다. 1.6-sync 부터 auto = 방법 C.

### 15.2 위상 재기 (`PhaseSync.java`, 카메라 스레드)
- 매 논리 결과의 `getPhysicalCameraResults()` SENSOR_TIMESTAMP: 대상 렌즈 = "5" (없으면 ids[1]), 기준 = 다른 하나.
- 주기 T = 기준 렌즈 연속 timestamp(노출 시작) 차이의 중앙값 (0.5~1.5 × 1/fps 만, 5개 전에는 1/fps).
- **1.7-mid: 위상 = wrap(t_mid5 − t_mid2)** ∈ [−T/2, T/2) — 두 렌즈 노출 가운데 시각 차이 (1.4 의 식, 렌즈별 물리 노출·skew).
  두 렌즈 노출 시간·skew 가 다르면 (렌즈 5 와 2 의 센서·AE 가 다름) 노출 시작이 같아도 찍힌 순간이 다르므로, PC 쌍 묶기·VIO 가 쓰는
  헤더 시각과 같은 기준으로 맞춘다. 1.6 까지의 위상 = wrap(t5 − t2) (노출 시작 차이) 도 계속 재서 `phase_start_ms` 로 보고.
  판단·표시·문턱(2.5 / 4 ms)은 가운데 위상. 표시·판단은 최근 9개 중앙값. 펼친(unwrap) 위상을 덮어쓰기 없는 구간 4 s 로 직선 맞춤 → 미끄러짐 ms/s.

### 15.3 맞추기
- **방법 C (auto 기본, 1.6-sync)**: |위상| ≥ 2.5 ms 이면 CameraDevice 를 닫고 다시 연다 (`MainActivity.doReopen`: 세션·장치 close →
  같은 논리 id 로 openCamera → 새 장치로 요청 다시 만들고 같은 ImageReader 로 세션). ImageReader·인코더·녹화기·소켓은 그대로.
  새 장치 첫 결과 2 개는 버리고 3~5 프레임으로 새 위상을 잰다: 3 개 중앙값이 6 ms 넘게 밖이면 바로 다음 시도, 5 개 중앙값 < 2.5 ms 면 locked.
  한 바퀴 최대 25 번. 위상이 열 때마다 [−T/2, T/2) 에 고르게 정해진다고 보면 한 번에 들어올 확률 5 ms / T: 15 fps 7.5 %
  (기대 ~13 번, 25 번 안에 ~86 %), 30 fps 15 % (기대 ~7 번, ~98 %). 25 번 실패 → `failed` 30 s 쉬고 다시 (가장 작았던 위상에서 멈추지 않음:
  다시 열면 어차피 바뀜). 한 번 걸린 시간 (요청 → 판정 `c_try_ms_*`, 요청 → 새 장치 첫 결과 `c_open_ms_*`), 지난 맞춤 걸린 시간
  `c_last_align_s` 기록. 열기 실패 → 0.7 s 뒤 다시 (3 번), 5 s 안에 장치·세션이 안 생기면 다시 → 그래도 안 되면 `ERROR CAMERA_REOPEN_FAILED`.
- locked 뒤 |위상| > 4 ms 로 미끄러지면 다시 C (잠근 뒤 20 s 안에는 안 함: 30 fps 의 4 ms/s 미끄러짐으로 끝없이 다시 여는 것 방지).
  **MP4 에 프레임이 들어가는 중이면 다시 열지 않음** → 상태 `drift` (경고만, `hold_reason`). 녹화가 끝나면 (both → 실시간 계속) 다시 C.
- **방법 A (`sync=a` 만)**: S24 에서 안 먹힘 (15.1). `Caps.methodA` 조건·제어는 1.5-sync 그대로 (렌즈 5 SENSOR_FRAME_DURATION ±2 %,
  P 0.15 + 미끄러짐 보정, 맞추는 동안 렌즈 5 AE 끔). 1.6: 물리 id 빌더가 렌즈별 값을 템플릿 기본값으로 채우므로 AF 끔·초점 거리·OIS 끔·
  영상 안정화 끔·AE fps 범위를 물리 키면 두 렌즈 모두에 복사 (REVIEW8 H1). 되돌리기 실패 → 다음 프레임마다 다시 (M5). A 가 안 되면 B 로
  가지 않고 `failed` (M4). "요청을 안 따름"만 영구 실패 (`method_a_failed`), 시간 초과는 그 회차만.
- **방법 B (`sync=b` 만)**: S24 에서 안 먹힘 (15.1). 반복 요청 재시작 → 세션 재생성, 최대 10 번. 녹화 중에는 안 함. 세션 재생성이
  CONFIGURE_FAILED 면 0.3 s 뒤 한 번 더, 그래도 실패면 `ERROR CONFIGURE_FAILED (phase sync)` (REVIEW8 L2).
- A·B 측정 결과는 caps.json `phase_sync.method_a_measured / method_b_measured`, metrics `sync.method_a_measured / method_b_measured`.
- 인자 (intent extra): `sync=auto|c|a|b|off` (기본 auto = c), `rec_wait_sync_s` (기본 40 = 15 fps 에서 C 약 25 번).
  PC: `phone_link.PhoneLink(sync=, rec_wait_sync_s=)` → `--es sync … --ei rec_wait_sync_s …`, scan3d `--app-sync`, `--app-rec-wait-sync-s`.

### 15.4 status.txt 꼬리 (첫 낱말 그대로)
실행 상태(STREAMING / RECORDING / RECORDING+STREAMING) 줄 끝에 붙음, 바뀔 때만 다시 씀 (맞추는 중에는 0.5 ms 넘게·1 s 넘게 지나야, C 는 시도마다):
`STREAMING logical=0 … mode=live ts_ref=mid sync=aligning phase=+12.3ms method=C try=7` / `… ts_ref=mid sync=locked phase=+0.4ms method=C|A|B|none` /
`sync=measuring` / `sync=failed phase=… method=C` / `sync=drift phase=… method=C`, 녹화기가 맞춤을 기다리는 동안 ` rec_wait=1`.
` ts_ref=mid` (1.7-mid) = 프레임 timestamp 가 노출 가운데 (1.4); sync=off 여도 실행 상태 줄에 붙음. `phase=` 는 가운데 위상.
`warn=` 이 있으면 ts_ref·sync 꼬리는 그 앞. 첫 낱말이 바뀌지 않으므로 phone_link / win_relay 의 상태 판단은 그대로. `method=none` = 처음부터 맞았음.
status 는 한 잠금 안에서 쓰고, sync 만 바뀌면 꼬리 없는 마지막 줄(base)에 새 꼬리를 붙여 다시 씀 (옛 줄이 RECORD_DONE 을 덮지 않음, REVIEW8 L1).
PhaseSync 는 자기 잠금을 쥔 채 status 를 부르지 않음 (카메라 스레드로 post).

### 15.5 녹화
- `mode=record`·`both`: 위상이 맞을 때까지 (최대 `rec_wait_sync_s`, 기본 40 s) 프레임을 녹화기에 안 넣음 → MP4 는 맞춘 뒤부터.
  `both` 는 실시간 스트림이 녹화 인코더를 같이 쓰므로 실시간 영상도 그때부터. IMU·frame_timing 은 처음부터.
  `meta.json`: `sync_mode`, `rec_wait_sync_s`, `sync_gate_wait_ms`, `sync_at_record_start` (아래 sync 객체), 끝날 때 `sync`.
- 녹화기에 프레임이 들어가기 시작하면 카메라를 다시 열지 않음 (C·B, MP4 끊김 방지) → 미끄러지면 `drift` 경고만. 매 프레임 가운데 위상은
  두 렌즈 frames CSV `t_mid_ns` 차이, 노출 시작 위상은 frame_timing (센서 type 100) 행의 두 렌즈 물리 오프셋 차이로 언제든 다시 계산할 수 있다.

### 15.6 metrics.json `sync` / 웹
```
"sync": {"state": "off|measuring|aligning|locked|failed|drift", "method": "C|A|B|none|null", "phase_ms" (마지막 값, 다시 여는 동안 유지; = phase_mid_ms),
         "phase_mid_ms" (1.7-mid: 노출 가운데 위상 = 판단 기준), "phase_start_ms" (노출 시작 위상, 1.6 까지의 정의), "phase_ref": "mid_exposure",
         "slip_ms_s" (1.6 에서 1000 배 오류 고침, REVIEW8 M1), "adjustments" (A 주기 요청 수), "restarts" (B 다시 시작 수), "episodes",
         "tries" (이번 회차 시도), "period_ms", "lens_ref", "lens_target", "mode", "auto_method": "C",
         "reopens", "reopen_fails", "c_try", "c_max_tries" 25, "c_rounds_failed", "c_locks", "c_lock_ms" 2.5, "c_realign_ms" 4,
         "c_expected_tries" (T / 5 ms), "c_try_ms_last|avg|max", "c_open_ms_last|avg|max", "c_last_align_s", "align_s" (맞추는 중),
         "retry_in_s" (failed), "hold_reason" (drift), "rec_wait", "rec_wait_sync_s",
         "method_a_possible", "method_a_ae_key", "method_a_reason", "method_a_failed", "method_a_fail_reason",
         "method_a_measured", "method_b_measured", "override_on", "target_dur_req_ms", "target_dur_ms", "session_recreate", "events" (처음 40 개)}
```
- caps.json: `logical.physical_request_keys` (이름 목록), `logical.request_keys_has`, `logical.session_keys`, `logical.manual_sensor`,
  렌즈별 `sync_type`, `manual_sensor`, `exposure_ns_range`, `sensitivity_range`, `max_frame_duration_ns`,
  `phase_sync{target_lens, sync_type, physical_request_keys_has{…}, method_a_possible, method_a_ae_key, method_a_reason, auto_method,
  method_a_measured, method_b_measured}`.
- web_server: metrics.json `sync` (events 뺀 전부) → `ui.sync`. 부하 모델이 꺼져 있어도 읽고, sync 가 없거나 metrics 를 못 읽으면 지움
  (REVIEW8 L3). scan3d: status 꼬리 → `session.sync` (녹화 모드만; 앱 시작 때 찍은 app_status 는 안 씀).
- 웹 단계 표시줄: 맞추는 중이면 "두 렌즈 시각 맞추는 중 — 카메라 다시 여는 중 (N 번째) · 지금 위상 X ms" (노랑). 그동안 영상이 끊기고
  짝이 0 이어도 빨간 경고 대신 이 안내. 25 번 실패 뒤 쉬는 중이면 "N 초 뒤 다시 시도". 녹화 대기(`rec_wait`)면 "맞추면 바로 녹화 시작".
  `drift` 면 녹화 중 어긋남 경고. 고급 → "렌즈 위상" 줄 (상태·위상·미끄러짐·방법·다시 열기 횟수·한 번 평균 시간·지난 맞춤 시간).
- 쌍 묶기 기준 `--pair-skew-ms 5` 는 그대로.
