# ov_bridge 프로토콜

`ov_bridge` = OpenVINS `VioManager` (ROS 없이 빌드한 `libov_msckf_lib.so`) 를 감싼 실행 파일.
입력은 바이너리 레코드, 출력은 JSON 한 줄(`\n` 끝) 씩. 같은 기계 전용이라 **little-endian**.

## 빌드·실행 (WSL)
```bash
cd /home/system/scanner/StereoDepthSeg/scan3d/ov_bridge
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j8
# 설정 생성: venv_ffs/bin/python scan3d/make_ov_config.py  -> scan3d/config/*.yaml
build/ov_bridge ../config/estimator_config.yaml --stdio            # 파이프 (기본)
build/ov_bridge ../config/estimator_config.yaml --tcp 127.0.0.1:7600
```
**방어 동작 (2026-10-16)**
- OpenVINS 호출(`feed_measurement_imu/camera`) 예외(cv::Exception 포함)는 잡아서 VioManager 를 새로 만들고
  `{"type":"reset","auto":true,"reason":"exception","where":..,"msg":..}` 를 보낸다 (프로세스는 안 죽음). `lost` auto-reset 은 `"reason":"lost"`.
- `use_stereo: true` 설정이면 `n_cams=2` 레코드만 받는다 (1장짜리는 `cam_rejected` 로 셈). mono/stereo 를 섞으면 TrackKLT 가
  cam1 의 빈 이전 피라미드로 `calcOpticalFlowPyrLK` 를 불러 `levels2 >= 0` 단언으로 죽기 때문. 첫 레코드와 크기가 다른 영상도 버림.
- `--imu-hold S` (기본 0.25): 기다리는 카메라가 없으면 IMU 를 최대 S 초 붙잡아 두고, 카메라가 오면 그 카메라를 넘는 IMU 까지만 넣은 뒤
  카메라를 넣는다 (시간 순서 유지 → 초기화 직후 ZUPT "No IMU measurements to propagate with (0 of 2)" 방지).
- pose 줄 추가 필드: `cam_rejected`, `exceptions`, `imu_held`. ready 줄: `imu_hold_s`.
- scan3d 는 기본 `--ov auto` 로 이 브리지를 `--stdio` 자식 프로세스로 띄우고, 죽으면 `--bridge-restart-s`(2 s) 뒤 다시 띄운다.

옵션: `--max-cam-queue N` (기본 3, 넘으면 가장 오래된 카메라 레코드 버림, IMU 는 안 버림),
`--max-speed M_S` (기본 5 m/s, 넘으면 `lost`), `--lost-frames N` (기본 15, 특징점 0 이 N 프레임 연속이면 `lost`),
`--startup-lost-frames N` (기본 45, 2026-10-16: 초기화 뒤 **첫 특징 갱신 전**에는 이 프레임 수까지 기다림 — 정지 ZUPT 가
끝나고 움직이기 시작하면 clone 창이 차고 시차가 생길 때까지 10~14 프레임 동안 SLAM/MSCKF 갱신이 0 인 게 정상이라, 15 로
자르면 막 시작한 추적을 lost 로 끊고 다시 정지하기 전까지 초기화를 못 한다; rec_20261016_143702 재생에서 확인),
`--auto-reset` (`lost` 가 되면 VioManager 재생성),
`--reinit-state` (2026-10-17, scan3d 기본 켬: 특징 끊김 `lost` 때 새 VioManager 에 옛 상태 — 자세·위치·속도·바이어스(마지막 특징 갱신 뒤
IMU 로 전파한 값)·추정 외부 파라미터·시간 오프셋 — 를 `initialize_with_gt` 로 넣고 최근 IMU 를 다시 넣어 **정지 없이 같은 월드로 이어 감**.
reset 줄에 `"reinit":"state","gap":s,"t_state":..,"v_norm":..`. 마지막 특징 갱신이 `--reinit-max-gap`(1.5 s)보다 오래됐거나 속도 초과·NaN 이면
예전처럼 정지 초기화 (`"reinit":"static"` 또는 필드 없음). 넣은 상태의 공분산 `--reinit-sigma 3,0.10,0.30` (자세°, 위치 m, 속도 m/s, 추정).
주의: 영상 갱신이 계속 0 이면 주입 뒤에도 속도가 1.5 m/s 까지 표류했다 (옛 삼각측량 문턱일 때 실측) → scan3d 는 주입 뒤 조각도
재위치(reloc.py)로 확인한 뒤에만 옛 지도에 합친다), `--emit-features N` (pose 줄에 SLAM 특징점 최대 N 개).
RPATH 가 `~/scanner/src/ovbuild`, `~/scanner/local/lib` 로 박혀 있어 `LD_LIBRARY_PATH` 불필요.

## 채널
- `--stdio`: 입력 = stdin, 출력 = stdout. OpenVINS 로그(printf)는 **stderr** 로 돌려 놓았다 (stdout 에는 JSON 만).
- `--tcp HOST:PORT`: 서버. 클라이언트 1개씩, 같은 소켓으로 입출력. 연결이 끊겨도 VIO 상태는 유지 → 새 세션이면 RESET 을 먼저 보낼 것. QUIT 받으면 프로세스 종료.
- 연결(또는 시작) 직후 bridge 가 `{"type":"ready","num_cameras":2,"use_stereo":true}` 를 보낸다.

## 입력 레코드
공통 헤더 12 B: `struct.pack("<III", 0x4F564252, type, payload_len)` (magic = "OVBR"). magic 이 틀리거나 payload > 64 MiB 면 에러 줄을 보내고 연결을 닫는다.

| type | 이름 | payload (`struct` 형식) | 크기 |
|---|---|---|---|
| 1 | IMU | `<q3d3d`: t_ns (int64), gx gy gz (rad/s), ax ay az (m/s²) | 56 B |
| 2 | STEREO | `<qIIB3x`: t_ns, w, h, n_cams (1/2), 패딩 3 B → 이어서 n_cams × (w·h) 8bit 회색 (행 우선, 빈틈 없음) | 20 + n·w·h |
| 3 | RESET | 없음. VioManager 를 새로 만든다 (설정 그대로) | 0 |
| 4 | QUIT | 없음. 종료 | 0 |

- **시각**: `t_ns` 는 Android `elapsedRealtimeNanos` 시계 그대로(int64 ns). bridge 가 첫 레코드 시각 t0 을 빼서 `(t_ns − t0)·1e-9` 초로 OpenVINS 에 넣는다. IMU·카메라 모두 같은 시계여야 한다.
- **IMU**: Android 센서(기기) 축 (세로로 볼 때 x 오른쪽, y 위, z 화면 밖). 자이로 = `TYPE_GYROSCOPE`, 가속도 = 중력 포함 원시 `TYPE_ACCELEROMETER` (정지 시 |a| ≈ 9.8, 위쪽 +). 한 레코드 = 같은 시각의 (gyro, accel) 쌍 → 클라이언트가 자이로 시각에 가속도를 보간해 만든다. 시각이 직전 IMU 이하이면 버림(`imu_dropped`).
- **STEREO**: 이미지 0 = cam0 = **정류된 왼쪽(rect0, 렌즈 2)**, 이미지 1 = cam1 = 정류된 오른쪽(rect1, 렌즈 5). 640x480, `make_ov_config.py` 의 intrinsics(정류 P1)와 같은 정류표로 만든 영상이어야 한다. `n_cams=1` 이면 cam0 만 (mono 추적, 두 렌즈 시각차가 클 때). **mono 설정(2026-10-16 이후 기본)의 cam0 는 rect0 가 아니라
lens0u** = 렌즈 2 를 왜곡만 편 핀홀 (f = K1, 넓은 화각, `calib_scale.vio_lens0_K`; 설정 파일 `# scan3d_vio_cam0: lens0u`).
그때 pose 의 `T_world_cam0`·`R_ItoC0`·`p_IinC0` 도 lens0u 기준이고, scan3d 가 `frames.pose_cam0_to_rect0` 로 rect0 기준으로 바꿔 쓴다. `t_ns` = 카메라 시각 (권장: 왼쪽 렌즈 물리 SENSOR_TIMESTAMP + (노출 + 롤링셔터 skew)/2 = 노출 중앙. 앱 1.7-mid 는 헤더 timestamp 가 이미 이 값이라 scan3d 가 그대로 넣음). 남은 상수 오프셋은 OpenVINS 가 `dt_cam_imu` 로 추정. 직전 카메라 이하 시각이면 버림.
- **순서**: 클라이언트는 IMU 와 카메라를 시간 순서대로 섞어 보내면 된다. bridge 가 카메라를 큐에 넣고, ROS 노드(`ROS1Visualizer::callback_inertial`)와 같은 규칙 — `t_cam < t_last_imu − dt_cam_imu` 가 될 때(그 카메라보다 늦은 IMU 가 들어왔을 때) — 에만 `feed_measurement_camera` 한다. 따라서 카메라 레코드를 먼저 보내도 되지만 pose 는 그 뒤 IMU 가 도착해야 나온다.

## 출력 (JSON 한 줄씩)
| type | 언제 | 필드 |
|---|---|---|
| `ready` | 시작/연결 | `num_cameras`, `use_stereo` |
| `pose` | 카메라 레코드 하나 처리할 때마다 | 아래 |
| `reset` | RESET 처리 또는 auto-reset | `auto` (auto-reset 일 때 true) |
| `error` | 잘못된 레코드 | `msg` |

`pose` 필드:
- `t_ns` (int, 입력 카메라 레코드의 t_ns 그대로 → 키프레임 매칭 키), `t` (s, t0 기준), `mono` (bool)
- `t_state` (s, `t` 와 같은 기준): 이 줄을 쓸 때의 OpenVINS 상태 시각(`state->_timestamp`). 카메라를 처리하지 않고 넘겼으면 `t` 와 다르다 → 클라이언트는 `t` 와도 `t + dt_cam_imu` 와도 1 ms 넘게 다르면 stale 로 보고 키프레임에 안 쓴다 (REVIEW 9)
- `status`: `"init"` (초기화 전, 이때 포즈 필드 없음·`T_world_cam0: null`) / `"tracking"` / `"lost"` (속도 > max-speed, p·v·R_CtoG·T 에 NaN/inf, 또는 특징점 0 이 lost-frames 연속), `init` (bool)
- `p` [3]: IMU 위치 in 월드 (m) = p_IinG
- `q` [x,y,z,w]: **Hamilton** 쿼터니언, 월드 ← IMU 회전 (`v_world = R(q)·v_imu`). (OpenVINS 내부 JPL 값은 `q_GtoI_jpl` 로 따로 줌)
- `v` [3]: IMU 속도 in 월드 (m/s)
- `T_world_cam0` [16]: 행 우선 4x4, cam0(rect0, OpenCV 축) 점 → 월드. `R = R_ItoG·R_ItoC0ᵀ`, `p = p_IinG − R·p_IinC0` (C++ 에서 계산, 추정된 외부 파라미터 사용)
- `q_GtoI_jpl` [4], `bg` [3], `ba` [3], `R_ItoC0` [9 행 우선], `p_IinC0` [3]
- `dt_cam_imu` (s): 추정된 시간 오프셋, OpenVINS 규약 `t_imu = t_cam + dt_cam_imu`
- `n_slam`, `n_msckf`: 마지막 업데이트의 SLAM/MSCKF 특징 수, `ms`: 이 카메라 처리 시간
- `cam_dropped`, `imu_dropped`: 누적 버린 수
- `slam_pts` [[x,y,z],...]: `--emit-features N` 일 때만

월드 좌표 G: 중력 정렬 **z 위**, 원점·yaw 는 초기화 시점 기준.

## 파이썬 예 (stdio)
```python
import json, struct, subprocess
p = subprocess.Popen(["scan3d/ov_bridge/build/ov_bridge", "scan3d/config/estimator_config.yaml", "--stdio"],
                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
H = struct.Struct("<III"); MAGIC = 0x4F564252
def send_imu(t_ns, g, a):
    p.stdin.write(H.pack(MAGIC, 1, 56) + struct.pack("<q3d3d", t_ns, *g, *a))
def send_stereo(t_ns, g0, g1=None):            # g0/g1: (480,640) uint8 C-contiguous
    imgs = [g0] if g1 is None else [g0, g1]
    h, w = g0.shape
    body = struct.pack("<qIIB3x", t_ns, w, h, len(imgs)) + b"".join(i.tobytes() for i in imgs)
    p.stdin.write(H.pack(MAGIC, 2, len(body)) + body)
# 출력은 별도 스레드에서: for line in p.stdout: msg = json.loads(line)
```
stdout 을 읽는 스레드를 따로 두어야 파이프가 막히지 않는다 (bridge 는 입력 처리 중에 출력 쓰기를 기다린다).

## --init-gate MEAN,MAX,DISP_DEG (2026-10-18, scan3d 기본 꺼짐)

켜면 `init_max_disparity` 를 DISP_DEG 각도 (cam0 fx 로 화소 환산) 로 넓힌다. 대신 정지 초기화가 끝난 순간, 그 창 (넣은 IMU 중
초기화 시각까지 init_window_time + 0.05 s, 최소 0.8 s) 의 평균 각속도 벡터 크기가 MEAN 이하, 순간 크기가 MAX 이하이고 카메라
시작 뒤 0.8 s 가 지났는지 확인해, 아니면 그 초기화를 버리고 VioManager 를 새로 만든다 (최근 IMU 는 다시 넣음). 버린 프레임의
포즈 줄에 `"init_gate":"moving"|"short"|"cam_start"`. 프레임은 예전처럼 다 넣는다.
실측 (vio_track_eval / regress): rec_20261018_002349 초기화 못 함 → 추적 90 %; 참조 4 개 중 175446 재생 키프레임 142 → 111
(카메라 시작 0.18 s 에 넓힌 기준으로 초기화된 것을 버리고 1.18 s 에 다시) → 기본으로는 안 켬.
