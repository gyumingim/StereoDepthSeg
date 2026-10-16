# 폰 녹화 폴더 형식 (scan3d-phone-rec/1)

폰 앱 `com.camera.dualstream` (versionName `1.2-rec`)이 폰에 직접 저장하는 녹화 데이터 형식.
구현: `phone_stereo/android/src/com/camera/dualstream/Recorder.java`, `MainActivity.java`.

## 1. 모드와 실행

인텐트 인자 `--es mode live|record|both` (기본 `live`).

| mode | TCP 8765 스트림 | 폰 저장 | status.txt 머리말 |
|---|---|---|---|
| `live` | 켬 (지금과 같음) | 끔 | `STREAMING ...` |
| `record` | 끔 (서버 소켓도 안 엶, PC 없이 동작) | 켬 | `RECORDING rec=rec_... ...` |
| `both` | 켬 | 켬 | `RECORDING+STREAMING rec=rec_... ...` |

녹화 관련 추가 인자:

| 인자 | 기본 | 뜻 |
|---|---|---|
| `--ei rec_kbps N` | 자동 (앱 1.3-caps: 4000 × (w·h·fps)/(640·480·15), 640x480·15 fps = 4000, SPEC.md 14.5) | H.264 목표 비트레이트 (kbps), 렌즈마다 |
| `--ez rec_raw true` | false | mp4 와 함께 `camN.h264` (Annex-B 원시 스트림, 1 s 마다 flush) 도 저장 — 강제 종료 대비 백업 |

나머지 인자(`logical`, `physical`, `width`, `height`, `fps`, `focus_diopters`, `raw_ts`, `imu_us`)는 스트림과 같다.

예 (Git Bash, adb = 위 경로의 adb.exe):

```bash
adb shell am start -S -n com.camera.dualstream/.MainActivity \
  --es logical 0 --es physical 2,5 --ei width 640 --ei height 480 --ei fps 15 \
  --ef focus_diopters 1.0 --ez raw_ts false --ei imu_us 2500 \
  --es mode record --ei rec_kbps 8000
adb shell cat /sdcard/Android/data/com.camera.dualstream/files/status.txt   # RECORDING rec=rec_20261016_153012 ...
adb shell am broadcast -a com.camera.dualstream.STOP -p com.camera.dualstream   # 정지 (mp4 마무리)
adb shell cat /sdcard/Android/data/com.camera.dualstream/files/status.txt   # RECORD_DONE rec=... reason=broadcast
adb pull /sdcard/Android/data/com.camera.dualstream/files/rec_20261016_153012 .
```

### 시작·정지 규칙
- 녹화는 카메라 세션이 시작될 때(앱 시작 시) 자동 시작한다. 폴더는 앱 시작 시각으로 만든다.
- 정지 방법
  1. `am broadcast -a com.camera.dualstream.STOP` (권장). 인코더에 EOS 를 넣고 남은 출력을 비운 뒤 muxer 를 닫는다(최대 약 3 s).
     `record` 모드면 카메라·센서도 끈다(`RECORD_DONE rec=<이름> reason=broadcast | capture stopped`). `both` 모드면 녹화만 끝나고 스트림은 계속된다.
  2. 액티비티가 화면에서 사라질 때(`onStop`: 홈 버튼, 화면 꺼짐, `am start` 로 다른 앱, 정상 종료) 도 같은 마무리를 한다.
     그 뒤 status.txt 는 `STOPPED (activity not visible) rec=<이름>` 로 덮인다. 다시 앞으로 오면(`onRestart`) 액티비티가 재생성되지만
     녹화는 다시 시작하지 않는다(저장 상태 `rec_done_name`): `record` 는 `RECORD_DONE rec=<이름> reason=activity_stop | screen was off…` 로 멈춰 있고,
     `both` 는 스트림만 다시 연다. 새 녹화는 PC 에서 앱을 다시 시작한다 (앱 1.2 이후).
- **`am force-stop` 은 프로세스를 즉시 죽여 어떤 콜백도 오지 않는다** → mp4 의 moov 가 없어 재생 불가. CSV 는 1 s 마다 flush 하므로
  마지막 1 s 정도만 잃는다. force-stop 을 써야 한다면 먼저 STOP 브로드캐스트를 보내고 `RECORD_DONE` 을 확인하거나, `--ez rec_raw true` 로 `.h264` 백업을 남긴다.
  (`meta.json` 의 `"complete": false` 이면 정상 마무리가 안 된 녹화다.)
- status.txt 는 앱 내부(`run-as com.camera.dualstream cat files/status.txt`, win_relay 가 읽는 곳)와
  외부 `/sdcard/Android/data/com.camera.dualstream/files/status.txt` 두 곳에 같은 내용으로 쓴다.

참고: `both` 모드의 status 는 `RECORDING+STREAMING` 으로 시작한다. `win_relay.py`·`phone_link.py` 는 + 로 이은 상태를 받는다 (`state_ok`, 2026-10-16 통합 때 반영).

## 2. 폴더 구조

```
/sdcard/Android/data/com.camera.dualstream/files/        (= getExternalFilesDir(null), 권한 불필요, adb pull 가능)
  status.txt
  rec_YYYYMMDD_HHMMSS/          (폰 현지 시각)
    cam2.mp4                    렌즈(physical id) 2 영상
    cam5.mp4                    렌즈 5 영상
    frames_cam2.csv             cam2.mp4 의 프레임별 타임스탬프
    frames_cam5.csv
    imu.csv                     모든 센서 샘플 + frame_timing
    cameras.json                카메라 인벤토리 (스트림 경로와 같은 파일의 복사)
    sensors.json                등록된 센서 정보 (같은 파일의 복사)
    meta.json                   녹화 설정·통계
    cam2.h264, cam5.h264        (rec_raw=true 일 때만)
```

파일 이름의 숫자는 `--es physical` 의 physical camera id 그대로다 (`physical 2,5` → cam2, cam5; 스트림 index 0 = 첫 id).

## 3. 시계

모든 `*_ns` 시각은 `SystemClock.elapsedRealtimeNanos()` 기준이다 (이 기기의 카메라 `SENSOR_INFO_TIMESTAMP_SOURCE = REALTIME`,
`SensorEvent.timestamp` 도 같은 시계). 즉 프레임 시각과 IMU 시각을 바로 비교할 수 있다 — 스트림 경로와 동일.
벽시계 대응은 `meta.json` 의 `start_wall_ms` ↔ `start_elapsed_ns`.

## 4. 영상 `camN.mp4`

- H.264 (video/avc), 640x480 (인자 width/height), 인코더 입력 `COLOR_FormatYUV420Flexible`
  (NV21 → 인코더 `Image` 평면에 stride 를 지켜 복사; `getInputImage` 가 null 이면 NV12 = `YUV420SemiPlanar` 로 직접 씀).
- 비트레이트 `rec_kbps` (기본 = 크기·fps 비례 자동, SPEC.md 14.5; 기기 기본 비트레이트 모드), I-프레임 간격 1 s, `KEY_FRAME_RATE = fps`.
- 회전 메타데이터 없음: 센서 원래 방향(가로) 그대로. 안정화(OIS/EIS) 끔, 초점 고정 — 스트림과 같은 캡처 요청이다.
- PTS(µs) = (그 프레임의 physical SENSOR_TIMESTAMP − 녹화 첫 프레임의 physical timestamp(두 렌즈 공통)) / 1000.
  두 mp4 가 같은 시간축을 쓴다. 단조 증가를 보장하려고 같거나 작으면 이전 + 1 µs 로 민다.
- 인코더는 B-프레임 없이 입력 순서대로 출력한다고 가정한다. 디코드한 n 번째 프레임 = CSV 의 `frame_idx = n`.
  확실히 하려면 디코더가 주는 PTS 를 CSV 의 `pts_us` 와 맞춰라.

## 5. 프레임 CSV `frames_camN.csv`

```
frame_idx,physical_ts_ns,logical_ts_ns,exposure_ns,rolling_shutter_skew_ns,pts_us
0,123456789012345,123456789000000,9998000,25000000,0
```

| 열 | 뜻 |
|---|---|
| `frame_idx` | mp4 안의 프레임 순번 (0부터, 인코더에 넣은 순서). 버려진 프레임은 행이 없다. |
| `physical_ts_ns` | 이 렌즈의 physical `CaptureResult.SENSOR_TIMESTAMP` (`getPhysicalCameraResults()`) — **스트림 STER 패킷이 보내는 값과 같다** (`raw_ts=false`). `raw_ts=true` 면 logical 값. |
| `logical_ts_ns` | logical 결과의 `SENSOR_TIMESTAMP` (= ImageReader `Image.getTimestamp()`). 찾지 못하면 −1. |
| `exposure_ns` | `SENSOR_EXPOSURE_TIME` (logical 결과). 없으면 −1. |
| `rolling_shutter_skew_ns` | `SENSOR_ROLLING_SHUTTER_SKEW`. 없으면 −1. |
| `pts_us` | mp4 에 쓴 PTS (µs). |

스캐너와 같은 노출 중앙 시각: `t_mid = physical_ts_ns + 0.5·(exposure_ns + rolling_shutter_skew_ns)` (`phone_rx.camera_time_offset_ns` 와 같은 식).

스테레오 짝은 스트림 쪽과 같은 방식으로 `physical_ts_ns` 가 가장 가까운 프레임끼리 맞추거나, 같은 `logical_ts_ns` 로 맞춘다
(같은 캡처 요청의 두 렌즈 프레임은 logical 값이 같다).

## 6. 센서 CSV `imu.csv`

```
type,t_ns,v0,v1,v2,v3,v4,v5
1,123456789100000,0.12,9.79,0.33
4,123456789101250,0.001,-0.002,0.0005
```

- 한 행 = `SensorEvent` 하나 (값 개수는 센서마다 다름 → 행 길이가 가변; 헤더의 v0..v5 는 최대 개수).
- `type` = Android `Sensor.TYPE_*`: 1 ACCELEROMETER (m/s², 중력 포함 원시), 4 GYROSCOPE (rad/s, 원시) — `imu_us` 주기(기본 2500 µs = 400 Hz);
  9 GRAVITY, 10 LINEAR_ACCELERATION, 15 GAME_ROTATION_VECTOR, 11 ROTATION_VECTOR, 6 PRESSURE (hPa) — 10 ms.
- `type = 100` = frame_timing: 스트림 SENS 패킷 100 과 같은 값
  `[phys[0]−logical ms, phys[1]−logical ms, exposure ms, frame duration ms, rolling shutter skew ms]`, `t_ns` = logical timestamp.
- `t_ns` = `SensorEvent.timestamp` (elapsedRealtimeNanos).
- 쓰기 큐 16384 개; 넘치면 버리고 `meta.json` 의 `imu_dropped` 에 센다.

## 7. `meta.json`

시작할 때 `"complete": false` 로 한 번 쓰고, 정상 정지 때 통계를 채워 `"complete": true` 로 다시 쓴다.
정지 시간(3 s) 안에 인코더가 다 안 끝나면 `"complete": false, "finishing": true` 로 쓰고, 마지막 렌즈가 닫히면 실제 `muxer_ok` 로 다시 쓴다.

| 키 | 뜻 |
|---|---|
| `format` | `scan3d-phone-rec/1` |
| `mode`, `folder` | live/record/both, 폴더 이름 |
| `logical`, `physical` | 카메라 id |
| `width`, `height`, `fps`, `imu_us`, `focus_diopters`, `raw_ts` | 캡처 설정 (인텐트 값) |
| `rec_kbps`, `rec_raw_h264`, `i_frame_interval_s`, `codec` | 인코더 설정 |
| `timestamp_clock` | 시계 설명 |
| `start_wall_ms`, `start_wall`, `start_elapsed_ns` | 시작 시각 (벽시계 ms, ISO, elapsedRealtime ns) |
| `end_wall_ms`, `end_wall`, `end_elapsed_ns`, `stop_reason` | 끝 시각, `broadcast` 또는 `activity_stop` |
| `app_version`, `app_version_code` | 앱 버전 |
| `frames` | `{ "2": n, "5": n }` mp4 에 넣은 프레임 수 |
| `dropped` | `{ "2": n, ... }` 인코더 입력 큐(4 개)가 차서 버린 프레임 수 |
| `lenses` | 렌즈별 상세: `frames`, `dropped_queue`, `dropped_encoder_busy` (입력 버퍼 20 ms 안에 못 받음), `samples_written`, `muxer_ok`, `encoder` (코덱 이름), `error` |
| `matcher_unmatched_dropped` | physical 타임스탬프를 못 찾아 버린 프레임 (스트림과 공통 카운터) |
| `imu_samples`, `imu_dropped` | imu.csv 에 쓴 행 수, 버린 수 |
| `complete` | 정상 마무리 여부 |

## 8. 스레드·지연

- 카메라 스레드는 NV21 복사 뒤 같은 배열을 스트림 큐와 녹화 큐에 넣기만 한다(막지 않음).
- 렌즈마다 인코더 스레드 하나(`rec-cam2`, `rec-cam5`), 입력 큐 4 프레임. 꽉 차면 새 프레임을 버리고 센다 → 스트림 지연에 영향 없음.
- IMU 는 별도 스레드(`rec-imu`)가 CSV 로 쓴다.
- 대략 용량: 8 Mbps × 2 렌즈 ≈ 2 MB/s (+ imu.csv 약 0.1 MB/s).
