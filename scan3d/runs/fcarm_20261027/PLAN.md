# fcarm 2026-10-27 — PLAN (판정 규칙, arm 재생 결과를 보기 전에 적음)

브랜치 `fcarm-20261027` (integ2-20261026 4035d59 에서 분기). PC 만 씀 (보드·폰 안 씀). 기본값은 바꾸지 않음 (사용자가 정함).

## 할 일 1 — ESP-IDF 실제 빌드
- ESP-IDF v5.5.6 (공식 git clone + `install.sh esp32s3`, WSL Ubuntu 22.04, `~/esp/esp-idf`).
- `idf.py set-target esp32s3 && idf.py build` (esp32fc/ 기본 설정 = PWM, 폰 IMU). 경고·오류를 모두 적고 펌웨어 쪽 원인은 고침
  (IDF 쪽 경고는 적기만). 바이너리 크기 (`idf.py size`) 기록. 다른 변형 (OneShot125 / DShot600, 로컬 IMU) 도 빌드해 봄.
- 기존 PC 검사: `make -C esp32fc/test run` (573 검사) 과 `make syntax` 가 그대로 통과해야 함.

## 할 일 2 — arm 보강 (플래그 뒤, 기본값 = 지금 동작)
새 설정 (fs_config_t), 기본값은 지금 코드와 같은 결과:
- `arm_gyro_max_rps` = 0.2 (지금 하드코딩 값). MOVING 규칙의 자이로 문턱 (arm + CLEAR_KILL 공용, 지금과 같음).
- `arm_acc_var_max`, `arm_acc_dev_max` = 음수 (= 지금처럼 touch_acc_var / touch_acc_dev 를 씀).
- `arm_still_ms` = 0 (끔). > 0 이면 IMU 정지 (자이로·가속 분산·|가속−g| 가 MOVING 문턱 안) 가 이 시간 연속이어야 arm. 새 거부 비트 FS_B_NOT_STILL.
- `arm_vio_speed_max` = 0 (끔). > 0 이면 마지막 pose 의 VIO |v| 가 이 값 이하여야 arm, 그리고 `arm_vio_speed_hold_ms` (기본 0)
  동안 이 값을 넘은 pose 가 없어야 함. 속도를 모르면 (NaN) 거부. 새 거부 비트 FS_B_VIO_SPEED. |v| 는 viop_pose_t.v_w 의 노름.

## 문턱 정하기 (사전 규칙)
### 데이터 (이미 있는 것만, 새 VIO 실행 0)
- gyrocheck 결과 `~/scanner/tmp_gyrocheck/ate/{NL60,GL60}/*.poses.jsonl.gz` (녹화 6개 × k0–k4 (+ 60 fps 4개 녹화 s10–s50) × 2조건 = 92 실행) 과 녹화 `imu.csv`.
  보조: 실제 VIOP 캡처 6개 (`ate/{NL60V,GL60V}/*.viop.bin.gz`, 축별 pos_std_mm).
- 재생 방식 = gyrocheck arm_replay 와 같음 (10 ms 주기, 기울기 = 끝 정지 기준, 배터리·링크 정상, heartbeat = 포즈 상태).
  pose std = h.pos_std/√3 (= √(trace/3), 실제 축별 최대보다 작거나 같음 → std 검사가 덜 막는 쪽 = 새 검사에 불리한 쪽).
  속도 = 포즈의 `v` 노름. 자이로 = imu.csv type 16 (없으면 type 4).
- 같은 실행 (gyrocheck 해시 같음) 은 1개로 셈. 초기화 실패 (noinit) 는 뺌.

### 꼬리표 (결과 보기 전 고정)
- 구간 (IMU 로만): 움직임 = 자이로 |ω| 가 처음 0.2 s 중앙값보다 0.08 rad/s 넘게 벗어난 순간 (0.2 s 번짐) — arm_prep.py 와 같음.
  H (손에 든 구간) = 첫 움직임 ~ 마지막 움직임. E (끝 정지) = 마지막 움직임 이후. E3 = 마지막 움직임 + 3.0 s 이후.
- 실행 종류: D (발산) = gyrocheck div 참, 또는 추적 중 VIO |v| 최대 > 2.0 m/s (일시 발산; 검증 N1 의 151646 NL60 k2·k3 를 넣기 위함 —
  정상 69개 최대 1.22 m/s 라는 검증 값을 알고 정한 문턱임). N (정상) = 나머지 (초기화 성공).

### 지표
- 정상 거부 FR = N 실행의 E3 주기 중 "지금 규칙으로 arm 가능한데 새 규칙으로 거부" 비율. **목표 0 %** (모든 N 실행 합).
- 손 허용 HA = 모든 실행 (N + D) 의 H 주기 중 새 규칙으로 arm 가능한 비율. **목표 0 %**.
- 발산 허용 DA = D 실행의 E 주기 중 새 규칙으로 arm 가능한 비율. **목표 0 %**. 두 번: (i) std 검사 그대로, (ii) std 검사 끔 (새 검사만으로 막는가).
- 같이 적음: 내려놓은 뒤 첫 arm 까지 시간 (N), 실행별 최악값.

### 격자
- arm_still_ms ∈ {0, 250, 500, 1000, 1500, 2000, 3000}
- arm_gyro_max_rps ∈ {0.03, 0.05, 0.08, 0.10, 0.15, 0.20}
- arm_acc_var_max ∈ {0.5 (지금), 0.1, 0.02}
- arm_vio_speed_max ∈ {끔, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0}, arm_vio_speed_hold_ms ∈ {0, 500}

### 추천 규칙
1. 가능 = FR 0 이고 HA 0 이고 DA(i) 0.
2. 튼튼 = 가능이고, 격자에서 한 매개변수씩 한 칸 옮긴 이웃이 모두 가능 (끝 칸은 있는 이웃만).
3. 추천 = 튼튼 중 덜 빡빡한 것: still_ms 작은 것 → gyro 큰 것 → acc_var 큰 것 → speed 큰 것 → hold 작은 것 순.
   튼튼이 없으면 가능 중 같은 순서, 가능도 없으면 "판정 불가" + 가장 가까운 것 (위반 합 최소).
4. DA(ii) (새 검사만으로 발산을 막는가) 는 따로 보고: 속도 문턱은 DA(ii) = 0 인 범위도 같이 적음.
5. 사례 수 (녹화 6개, D 실행 수, 서로 다른 실행 수) 를 적고, 결론은 모두 "추정" (같은 녹화의 시작 밀기는 독립 반복 아님,
   손 루프 3개 · 발산 사례 적음, 실기 USB 지연·실제 기체 진동 없음).
6. 기본값은 바꾸지 않음. 추천값만 보고.
