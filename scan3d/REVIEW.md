# scan3d 독립 코드 검토 (2026-10-16)

검토 범위: `scan3d/` 전체(`ov_bridge/main.cpp`, `PROTOCOL.md`, `ov_client.py`, `frames.py`, `scan3d.py`, `phone_rx.py`,
`depth_seg.py`, `voxel_map.py`, `web_server.py`, `web/app.js`, `make_ov_config.py`, `config/*.yaml`, `run_scan.sh`, `win_relay.py`)와
바뀐 앱 코드(`MainActivity.java`, `AndroidManifest.xml`, `lib_phone_stereo.py` diff).
코드를 읽기만 했습니다. 실행, 테스트, 측정은 하지 않았습니다. 줄 번호는 현재 파일 기준입니다.

## 확인 결과 맞는 부분 (버그 아님)
- 프로토콜: 헤더 `<III`, IMU `<q3d3d` = 56 B, STEREO `<qIIB3x` = 20 B + n·w·h. 바이트 순서(LE)와 크기는 브리지(main.cpp 153, 213-222)와 클라이언트(ov_client.py 25-27, 95)가 같습니다.
- 쿼터니언:
  - `st->_imu->quat()`은 JPL q_GtoI입니다. 브리지는 `quat_2_Rot`로 R_GtoI를 구하고, 이를 전치해 R_ItoG를 만든 뒤 Eigen(Hamilton) `qh`로 바꿉니다(main.cpp 290-294). 맞습니다.
  - `frames.jpl_quat_to_R`(frames.py 28-34)는 OpenVINS `quat_2_Rot` 식과 같습니다.
  - `T_world_cam0`: `R_CtoG = R_ItoG·R_ItoCᵀ`, `p_CinG = p_IinG − R_CtoG·p_IinC`(main.cpp 295-302). 맞고, 행 우선 전송과 파이썬 `reshape(4,4)`, JS `T[0..11]`도 서로 맞습니다.
- 외부 파라미터: `kalibr_imucam_chain.yaml`의 T_imu_cam 열을 보면 광축은 기기 −z, 영상 x는 기기 −y, 영상 y는 기기 −x입니다(det +1). 이는 SENSOR_ORIENTATION 90인 후면 카메라와 맞습니다. 이 값은 `R1 @ R_s2l`이고, rect1 원점 ≈ IMU 원점(렌즈 5 가정)도 정합합니다.
- 타임스탬프:
  - 프레임 시각은 physical SENSOR_TIMESTAMP(CaptureTimestampMatcher)에 0.5·(노출 + skew)를 더한 값입니다.
  - 센서 시각은 SensorEvent.timestamp입니다. 둘 다 원래 ns 값이고, 브리지가 첫 레코드 t0를 뺍니다(main.cpp 182-188). 단위와 기준은 일관됩니다.
  - 카메라 feed 규칙 `t_cam < t_last_imu − dt`(main.cpp 257)는 ROS1Visualizer와 같습니다.
- 가속도 부호: TYPE_ACCELEROMETER(정지 시 위쪽 +g)는 OpenVINS/Kalibr의 비력 규약과 같습니다.
- 복셀:
  - `np.floor` 키라서 음수 좌표도 맞게 내림됩니다(voxel_map.py 287).
  - `pack_keys`의 오프셋 2^20과 21비트, `|k|<32000` 제한이 맞습니다.
  - JS 키 `((ix+32768)·65536+(iy+32768))·65536+(iz+32768)`은 2^48보다 작아서 정밀도가 유지됩니다.
  - 17 B 레코드 dtype(packed)과 JS 오프셋 6/9/11이 맞습니다.
- 증분 갱신: 바뀐 복셀은 `ver`가 갱신되고, `min_hits` 변경과 reset 때는 `epoch`가 올라 전체를 다시 보냅니다. JS는 epoch가 바뀌면 `clearVoxels`합니다. 빠진 경로를 찾지 못했습니다.
- `use_multi_threading_subs=false`이므로 초기화 스레드가 join되어, RESET 때 VioManager를 바꿔도 해제된 메모리를 쓰지 않습니다.

## 실제 버그 (심각도 순)

### 1. [중] 포즈 콜백에서 예외가 한 번 나면 OpenVINS 읽기 스레드가 영구히 죽음
- ov_client.py 184-227: `on_pose(msg)`(220-221)가 `for line in self._rfile` 루프 안의 같은 try에 있습니다. 그래서 `on_pose`에서 예외가 하나라도 나면 루프가 끝나고 이후 포즈가 다시 오지 않습니다.
- 실제로 이런 입력이 가능합니다. main.cpp 304의 `bad` 판정은 p와 v만 보고 쿼터니언/R은 보지 않습니다.
  - R이 NaN이면 `T_world_cam0`에 `null`이 들어가면서 status는 "tracking"이 됩니다(jnum, 98-103).
  - 그러면 `frames.T_world_rect0`의 `np.asarray(T, float)`(frames.py 347)가 TypeError를 냅니다.
  - 그 결과 reader 스레드가 종료되고, 맵·궤적이 멈춥니다(웹에는 error 문자열만 남음).
- 고치는 법: `on_pose` 호출을 try/except로 감싸서 로그만 남기고, 브리지 `bad`에 `R_CtoG.allFinite()`(또는 `q_GtoI.allFinite()`)를 추가합니다.

### 2. [중] 수신 스레드도 콜백 예외로 조용히 죽고 재접속되지 않음
- phone_rx.py 518-552: `on_sensor`/`on_pair`(528, 549)의 예외 중 OSError/EOFError/ValueError가 아닌 것(TypeError, IndexError, KeyError 등)은 잡히지 않습니다. 그러면 스레드는 끝나는데 `self.error`는 None으로 남습니다.
- scan3d.py 434의 재접속 조건이 `receiver.error`라서 스트림이 멈춘 채로 그대로 있습니다.
- 반대로 콜백에서 난 ValueError는 "소켓 오류"로 처리되어 쓸데없이 재접속합니다.
- 고치는 법: 콜백 호출을 따로 try/except로 감싸서 로그만 남깁니다. `_run` 바깥에 `except Exception as e: self.error = repr(e)`를 둡니다.

### 3. [중] 릴레이 대기(accept)와 재접속이 Ctrl+C를 막고, 재접속 실패 시 프로세스가 죽음
- phone_rx.py 483-486: `srv.accept()`는 타임아웃 없이 블록합니다. scan3d.py 429-430에서 SIGINT 핸들러를 바꿔서 KeyboardInterrupt가 나지 않습니다. PEP 475에 따라 accept가 EINTR 뒤에 다시 시작되므로, 릴레이가 붙기 전에는 Ctrl+C로 끝낼 수 없습니다. 시작할 때(scan3d.py 135)와 재접속 루프(437) 둘 다 해당합니다.
- connect 모드(`:18765`)에서는 60 s가 지나면 RuntimeError(phone_rx.py 500)가 납니다. 재접속 루프(scan3d.py 437)에서는 이 예외를 잡지 않아 `finally: sc.stop()`으로 스캐너 전체가 종료됩니다.
- 고치는 법: `srv.settimeout(1.0)` 루프 안에서 stop 이벤트를 확인하고, 재접속 블록은 try/except로 감싸서 다음 주기에 다시 시도합니다.

### 4. [중] 카메라 대기 큐 2장이 IMU 지연보다 짧으면 VIO 입력이 끊길 수 있음
- ov_client.py 96-100(`max_pending_cams=2`), 141-142: 카메라는 `t_cam ≤ last_imu_ns`가 될 때까지 cam_q에서 기다립니다. 그런데 큐가 2장을 넘으면 **가장 오래된(곧 보낼 수 있게 될) 카메라**부터 버립니다.
- IMU가 카메라보다 2프레임(약 66 ms) 넘게 늦게 도착하면(센서 FIFO 배치, 앱 `serve()`가 프레임 460 KB를 쓰는 동안 IMU를 비우지 못하는 경우 등), 큐 머리가 보낼 수 있게 되기 전에 매번 밀려납니다. 그러면 카메라가 하나도 브리지로 가지 않습니다.
- 지금 경로에서는 카메라 지연(ISP + USB)이 보통 더 커서 드물지만, 일어나면 아무 오류 없이 "초기화 대기"에 머뭅니다.
- 고치는 법: `max_pending_cams`를 10 이상으로 늘리고, 버리는 기준을 "개수"가 아니라 "`t_cam < last_imu_ns − 0.5 s`(너무 오래됨)"로 바꿉니다. 브리지 쪽 `--max-cam-queue`가 이미 밀림 방지를 맡고 있습니다.

### 5. [중] `--ov-images raw` 경로는 설정과 맞지 않아 VIO가 틀림
- scan3d.py 51-52, 190-192, frames.py 344: raw를 고르면 정류 전 원본 gray(렌즈 2 원래 K, 왜곡 있음, R1 회전 전 축)를 보냅니다.
- 그런데 `make_ov_config.py`(308, 306, 142)는 항상 정류 P1 핀홀, 왜곡 0, `R_ItoC = R1 @ R_s2l`로 설정을 만듭니다. 그래서 intrinsics와 extrinsics가 모두 틀리고, 투영이 어긋나 추적이 실패하거나 자세가 틀립니다.
- frames.py 344의 `T_cam1_rect0` 보정도 이 설정에서는 이중 보정이 됩니다.
- 고치는 법: 옵션을 지우거나, `make_ov_config.py --raw`로 K1/dist1(radtan)과 `R_s2l`을 쓰는 설정을 따로 만들고 둘이 짝이 맞는지 확인합니다.

### 6. [하-중] 노출 중앙 보정에 그 프레임의 값이 아니라 "최신" frame_timing을 씀
- phone_rx.py 526-527, 554-562, scan3d.py 174: `camera_time_offset_ns()`는 마지막으로 받은 frame_timing을 씁니다. frame_timing은 빠른 IMU 큐로 오고 프레임은 느린 큐로 오므로, 보통 몇 프레임 뒤의 값입니다.
- 자동 노출이 바뀌는 동안에는 오차가 Δexposure/2만큼 생깁니다(실내에서 10~30 ms 노출이면 수 ms). 이 오차는 OpenVINS dt 추정에 잡음으로 들어갑니다.
- 노출 값도 렌즈 2 physical result가 아니라 logical result에서 옵니다(MainActivity.java 87).
- 고치는 법: frame_timing 패킷의 timestamp(logical ts)와 물리 오프셋으로 렌즈 2 physical ts를 다시 계산해 dict에 넣고(`ts_phys → (exp, skew)`), 프레임 ts로 찾아 씁니다.

### 7. [하] RESET 직후 응답 대기 수 계산이 음수로 어긋나 흐름 제어가 꺼짐
- ov_client.py 152-155, 209: writer는 RESET을 보낸 직후 `_zero_counts()`를 합니다. 그런데 RESET 전에 보낸 카메라의 pose가 그 뒤에 도착하면 `poses_got > cams_sent`가 되고, 그 상태가 계속 남습니다. `max(n,0)`(139)이라 inflight가 줄어든 만큼 `max_inflight` 제한이 느슨해집니다.
- 두 스레드가 lock 없이 같은 카운터를 0으로 만드는 경합도 있습니다(130 vs 175/209).
- 고치는 법: 보낸 카메라의 t_ns 집합을 lock 아래에서 관리하고, pose나 reset 응답이 오면 해당 t_ns(또는 전부)를 지웁니다.

### 8. [하] VIO·맵 리셋과 키프레임 처리가 경합함
- scan3d.py 363-390과 267-283:
  - `reset_vio`/`reset_map`/`set voxel_m`은 HTTP 스레드에서 돌고, 깊이 워커는 리셋 전의 포즈 T로 계산한 점을 리셋 뒤의 새 맵에 적분할 수 있습니다(옛 월드 좌표가 섞임).
  - `voxel_map.integrate`는 키를 lock 밖에서 옛 `voxel_m`으로 계산합니다(287 vs 309).
  - `tracker.associate`는 lock 없이 `items`를 순회하는데, 그동안 `reset()`이 `__init__`을 다시 부르면 dict가 바뀝니다(InstanceTracker 192-193, 209).
- 고치는 법: 키프레임 job에 `vmap.epoch`를 같이 넣고, 적분 직전에 다르면 버립니다. 키 계산과 associate는 `vmap.lock` 안에서 합니다.

### 9. [하] 포즈 시각을 확인하지 않음
- main.cpp 272-303: 이 카메라에 대한 업데이트가 건너뛰어지면(예: OpenVINS 내부 `state->_timestamp > message.timestamp` 경고 후 return) 상태가 다른 시각에 있습니다. 그래도 `t_ns = c.t_ns`라는 라벨로 포즈를 내보내고, 키프레임이 그 영상과 다른 시각의 자세로 적분됩니다.
- 고치는 법: `state->_timestamp`를 JSON에 넣고, 클라이언트가 `|state_t − (t + dt_cam_imu)| > 1 ms`이면 그 포즈를 키프레임에 쓰지 않습니다.

### 10. [하] lost 뒤 복구 경로가 없음
- run_scan.sh 41: 브리지에 `--auto-reset`을 넘기지 않습니다. NaN 등으로 상태가 깨져 `lost`가 계속되면, 사용자가 "VIO 재시작"을 누르기 전까지 맵이 멈춥니다.
- 웹 문구 "추적 실패 (VIO 재시작)"(app.js 189)는 자동 재시작처럼 읽힙니다.
- 고치는 법: 기본 `OV_ARGS`에 `--auto-reset`을 넣고, auto reset 이벤트를 받으면 scan3d가 traj, last_kf와 (선택적으로) 맵 epoch를 바꿉니다. 지금은 world 원점이 바뀌어도 기존 맵에 계속 적분합니다.

### 11. [하] 릴레이가 앱이 죽어도 다시 시작하지 않음
- win_relay.py 221-243: 폰 쪽 연결이 끊긴 뒤에도 `adb forward` 포트에 connect 자체는 성공합니다(adb가 받고 바로 닫음). 앱이 죽었으면 scan3d 재접속(5 s 주기)과 즉시 끊김이 무한 반복되고, 앱은 다시 뜨지 않습니다.
- 고치는 법: 폰 쪽에서 0 바이트로 끊기면 `start_app()`과 `adb_forward()`를 다시 호출하고(백오프 포함), `run-as cat files/status.txt`를 확인합니다.

### 12. [하] 대체 경로가 설정과 다른 회전을 씀
- frames.py 299-307, 353: 대체 경로는 보정 json의 `R_sensor_to_left`를 쓰지만, 설정은 Camera2 `pose_rotation`을 씁니다(make_ov_config.py 130, 142). 두 값의 차이는 make_ov_config가 출력만 합니다.
- 대체 경로는 raw 모드 `fix`도 빠뜨립니다. 브리지가 항상 `T_world_cam0`를 주므로 지금은 실행되지 않는 경로입니다.
- 고치는 법: 대체 경로에서도 `fix`를 곱하고, 같은 회전 출처를 쓰거나 대체 경로를 지웁니다.

## 버그로 보지 않은 것 / 확인 못 한 것
- 1 ms 스테레오 판정(scan3d.py 185) 때문에 대부분 mono로 갈 수 있습니다(통합 보고 위험 3). 실측 skew를 보고 `--vio-stereo-skew-ms`를 조정할 일입니다.
- S24 `SENSOR_INFO_TIMESTAMP_SOURCE`가 REALTIME인지(앱이 cameras.json에 기록함)는 확인하지 않았습니다.
- OpenVINS의 `track_frequency` 제한은 ROS 노드에만 있어 브리지에는 적용되지 않습니다. 30 fps 입력이면 문제없습니다.
