# armhist 2026-10-29 — VIO 이력 검사 (arm 금지 래치)

브랜치 `fcarm-20261027` (9233de1 위). PC 만 씀 (보드·폰 안 씀).
목적: armguard_20261029 에서 못 막은 151646 NL60 k2·k3 (손에 든 동안 VIO 가 튀었다가 내려놓으면 정상처럼 보임) 를 막기.

## 0. 사전 등록 (재생 결과 전에 커밋)
이 절은 재생 (`armhist_replay`) 을 돌리기 **전에** 커밋합니다. 결과는 아래 1절 이후에 따로 붙입니다. 이 절은 나중에 고치지 않습니다.

### 규칙 (fc_safety, 기본 켜짐)
- `arm_hist_speed_max` = **2.0 m/s** (0 = 끔). VIO 초기화된 (VIOP_F_VALID) pose 의 |v_w| 가 이 값을 넘거나 NaN 이면 → 래치.
- `arm_hist_lost` = **true**. 지금 VIO 세션에서 VALID+TRACKING pose 를 한 번 본 뒤, VALID 아님 또는 TRACKING 아님 pose 가 오면 → 래치.
- 래치가 켜져 있으면 ARM 거부, 새 거부 비트 **`FS_B_VIO_HIST` (1u<<16)**. CLEAR_KILL 은 영향 없음. 상태와 무관하게 (비행 중 포함) 래치되지만 막는 것은 ARM 뿐.
- 래치 해제는 VIO 재초기화 신호뿐: pose 의 `reset_counter` 가 바로 앞 pose 와 다름, 또는 pose flags 에 `VIOP_F_RESET` (0x0008, 새 W 의 첫 pose). 해제한 그 pose 자체는 다시 검사함. 그 밖에는 부팅 (`fs_init`) 뿐.
- 가속 (연속 pose 의 d|v|/dt) 은 **쓰지 않음**: 정상 최대 23.9 m/s² (bench) · 41 m/s² (VIOP 6 ms 간격), 발산 k3 NL60 22.4 m/s² 라 갈리지 않음 (흐름 통계, 아래 "정직하게").
- 위치 점프는 재생 흐름에 위치가 없어 이번에는 안 씀.

### 문턱 정하는 방법 (정상 실행만, 발산 실행 안 씀)
- 녹화 기준 나눔 (같은 녹화의 시작 밀기 k*/s* 는 독립 아님 → 녹화 단위로 나눔):
  - **고르기용 (train)** 정상: hs_131326 (5), hs_154320 (8, 걸으며 들고 다님), l142148 (bench 16 + VIOP 2), s165506 (18) = bench 47.
  - **확인용 (test)** 정상: l142610 (bench 12 + VIOP 2), l151646 k0_GL60 (bench 1 + VIOP 1) = bench 13.
- 식: `v_lim = max(2.0, 0.5 단위 올림(1.5 × train 정상 유효 pose |v| 최대))`. train 최대 1.224 m/s (154320) → 1.5× = 1.84 → **2.0**.
  하한 2.0 의 근거: 실내에서 들고 걷는 속도 (보통 걷기 약 1.3–1.4 m/s, 일반 상식, 출처 확인 안 함) 보다 위, 뛰는 속도 아래.
- 정직하게 적을 것 (블라인드 아님): 규칙을 정하기 전에 75개 흐름 **모두** (test 와 발산 포함) 의 실행별 |v| 최대·d|v|/dt 최대를 출력해 봤습니다.
  test 정상 최대 0.666 m/s, 발산 k2 NL60 7.07 / k3 NL60 2.755 m/s 를 이미 알고 정했습니다.
- **순환 주의**: fcarm 의 발산 라벨은 `div` (ATE) **또는** "tracking |v| 최대 > 2.0 m/s" 입니다. k2 NL60 · k3 NL60 은 `div=False` 라서 **라벨 자체가 |v| > 2.0 로 붙었습니다**.
  그래서 2.0 m/s 속도 래치가 이 둘을 막는 것은 일부 정의상 당연합니다. 따로 받치는 근거: 같은 녹화 정상 (k0 GL60) 의 경로 길이 4.4 m 대비 k2 NL60 42.3 m, k3 NL60 11.9 m (gyrocheck score.json `path_m`).
  k3 NL60 최대 2.755 는 문턱의 1.38 배라 여유가 크지 않습니다.

### 재생과 채점 (armguard_20261029 와 같음)
- 같은 75개 흐름 `~/scanner/tmp_fcarm/sweep1/ev/*.txt` (bench 69 + VIOP 6), 같은 구간: H = [첫 움직임, 마지막 움직임], E = (마지막 움직임, 끝], E3 = (마지막 움직임 + 3 s, 끝].
  라벨·구간 = `../fcarm_20261027/results.json`, D 9 = `posthoc.json` D, 정상 N 60 (bench), 끝 정지 ≥ 3 s 인 정상 NE 47.
- 이 흐름에는 재초기화가 없음 (reset_counter 항상 0, F_RESET 없음) → 재생에서 래치는 풀리지 않음. 해제는 PC 단위시험으로 확인.
- 설정 (재생 줄): 0 = 지금 기본 (armguard, 이력 끔) = FR·첫 arm 비교 기준, 1 = 새 기본 (손대지 않은 `fs_config_default()`), 2 = 같은 값 직접 넣음 (1 과 같아야 함),
  3 = 속도 래치만, 4 = lost 래치만, 5 = 예전 값 (즉시 arm) + 이력 켬. 민감도 (부차, **사후 라벨**): v_lim 1.5 / 2.5 / 3.0 / 5.0 (lost 켬·끔).

### 합격 기준 (설정 1, 모두 만족해야 합격)
- P1 발산 D 9개 (bench): E 구간 arm 가능 주기 합 **0** (실행마다 0).
- P2 정상 N 60 (bench): E3 에서 설정 0 대비 새로 거부된 주기 **0**; NE 47 실행의 내려놓은 뒤 첫 arm 시각이 설정 0 과 **모두 같음** (지연 증가 0);
  이력 래치가 켜진 정상 실행 **0** (train / test 따로 적음).
- P3 손에 든 채 arm 가능 HA: 0 그대로.
- P4 VIOP 6개: 발산 151646 k0 NL60V 의 E arm 가능 0; 정상 5개 첫 arm 시각이 설정 0 과 같음.
- P5 설정 1 = 설정 2 (75개 출력 모두 같음).
- 하나라도 못 맞추면 "불합격" 으로 그대로 적고, 같은 데이터로 문턱을 다시 맞추지 않습니다. 바꾼 값이 있으면 "사후" 로 적습니다.

## 1. 결과 (재생 2026-10-29 08:15, 사전 등록 커밋 e4cc5b0 뒤) — 합격 (P1–P5 모두), 숫자는 미검증
| | 지금 기본 (이력 끔, 설정 0) | 새 기본 (이력 켬, 설정 1) |
|---|---|---|
| P1 발산 D 9 끝 정지 arm 가능 (주기) | 1,524 (151646 NL60 k2 762 + k3 762) | **0** (실행마다 0) |
| P2 정상 60 내려놓고 3 s 뒤 새 거부 | 0 | **0** |
| P2 첫 arm 시각이 설정 0 과 다른 NE 실행 (47 중) | — | **0** (중앙 1.23 s, 최대 2.35 s 그대로) |
| P2 래치 켜진 정상 실행 train 47 / test 13 | — | **0 / 0** |
| P3 손에 든 채 arm 가능 HA | 0 | **0** |
| P4 VIOP: 발산 151646 k0 NL60V E arm / 정상 5 첫 arm | 0 / 0.93–2.35 s | **0** / 같음 |
| P5 설정 1 = 설정 2 (75개) | | **같음** |

- 발산 9개 모두 손에 든 동안 래치: 151646 6개는 10.995 s (내려놓기 23.67 s 보다 12.7 s 앞), 131326 3개는 11.8–12.8 s. k0·k2·k3 NL60 은 속도로만 (처음 넘은 pose |v| 2.009 m/s; k3 NL60 은 > 2.0 pose 45개), 나머지 (k2·k3 GL60, k4 NL60, 131326 3개) 는 속도+lost.
- 나눠 보기: 속도 래치만 (설정 3) 도 D 0. lost 래치만 (설정 4) 은 D 1,524 그대로 (k2·k3 NL60 은 lost 가 없음) → 이 데이터에서 lost 부분은 이득 없음 (다른 검사로 이미 막힌 실행에만 걸림).
- 예전 즉시 arm 값 + 이력 (설정 5): D 2,089 → 0, HA 17,262 → 16,963 (이력은 손 허용을 막는 검사가 아님).
- 민감도 (사후, 부차): v_lim 1.5 / 2.5 → D 0, 정상 래치 0. v_lim 3.0 / 5.0 → k3 NL60 762 주기 다시 arm 됨. 즉 이 데이터에서 통하는 범위는 정상 최대 1.224 초과 ~ k3 NL60 최대 2.755 미만, 2.0 은 그 안.

## 2. 한계 (추정으로 읽기)
- 순환: k2·k3 NL60 의 발산 라벨이 "|v| > 2.0" 이라 2.0 래치가 막는 것은 일부 정의상 당연 (0절). 독립 근거는 경로 길이뿐.
- 블라인드 아님: 정할 때 모든 흐름의 |v| 최대를 봤음. 발산 사례는 사실상 녹화 2개 (151646, 131326), k*/s* 는 같은 IMU 라 독립 아님.
- 2.0 m/s 넘게 들고 뛰거나 그보다 빨리 비행하면, 착륙 뒤 VIO 재초기화 전까지 arm 안 됨 (실패하면 안전 쪽). 실내 450 기체 비행 속도 ≤ 2 m/s 가정 (확인 안 함).
- 래치 해제는 폰이 재초기화 때 reset_counter 를 올리거나 VIOP_F_RESET 을 보내야 함. 실제 폰 앱 (ov_bridge) 이 그렇게 하는지는 이 작업에서 확인 안 함 (viop_proto.h 문서 기준).
- 재생 흐름에는 재초기화가 없어 해제 경로는 PC 단위시험으로만 확인. 위치 점프는 흐름에 위치가 없어 안 봄.

## 3. 검사 · 파일
- PC 단위시험 **678 통과 / 0 실패** (`test_safety_run.log`, 예전 644): 새 `t_arm_hist` (2.0 포함 통과 · 2.1 한 번이면 래치 · 5 s 정지 뒤에도 FS_B_VIO_HIST 만으로 거부 · 거부 이벤트에 비트 · reset_counter 변경 / VIOP_F_RESET 으로 해제 · 해제 pose 자체 검사 · lost 래치 · 추적 전 (초기화 중) 은 래치 안 됨 · 무효 pose 속도 무시 · 켜고 끄기 · 비행 중 래치는 arm 상태 유지 · CLEAR_KILL 영향 없음 · VIOP 변환).
  `t_arm_guard_defaults` 의 NaN 속도 경우는 이제 FS_B_VIO_SPEED|FS_B_VIO_HIST 로 거부되고 재초기화 뒤 arm (기대값 고침). `cfg_former_arm()` 은 이력도 끔.
- `make syntax` 6 변형 통과 (`syntax.log`). ESP-IDF v5.5.6 기본 변형 빌드 성공, 경고 0, 앱 0xbfe40 (예전 0xbfd40, +256 B) (`idf_build_default.log`, 증분 빌드: fc_safety.c · main/*.c 다시 컴파일).
- `armhist_replay.c` (fcarm_replay + 이력 설정 2개 + 래치 열), `armhist_eval.py` → `armhist_replay.json`, `armhist_replay_configs.txt`.
- 재현 (WSL, 이 폴더): `gcc -std=c99 -O2 -I../../../esp32fc/components/fc_safety/include armhist_replay.c ../../../esp32fc/components/fc_safety/fc_safety.c -lm -o /tmp/ah_fr && python3 armhist_eval.py /tmp/ah_fr ~/scanner/tmp_fcarm/sweep1/ev ../fcarm_20261027 armhist_replay.json`

## 4. v2 규칙 변경 (독립 검증 지적 반영) — 재생 다시 돌리기 전에 커밋
검증: `scratchpad/team/verify_arm_20261029.md` (읽기 전용 검증자). 지적 (A) 비행 중에도 래치 → 빠른 비행 / 착지 충격 튐이면 착지 뒤 arm 불가, 사용자가 VIO 를 재초기화할 수단 없음. (B) 앱 live 프리셋 `--reinit-state` (상태 주입) 가 새 W 를 만들어 래치를 풂 (옛 상태를 이어받는데도). (C) UDP 에서 첫 F_RESET pose 를 잃고 reset_counter 0 = 0 이면 안 풀림.
- **쌓이는 때**: 상태가 `FS_DISARMED` 일 때 받은 pose 만. ARMED_IDLE / FLYING / EMERG_LAND / KILLED 의 pose 는 래치를 켜지 않음. 착지 뒤 DISARM (또는 CLEAR_KILL) 하면 그때부터 다시 쌓임. 이미 켜진 래치는 상태와 무관하게 유지.
- **풀리는 때**: 새 W (reset_counter 바뀜 또는 VIOP_F_RESET) 이면서 **VIOP_F_SEEDED (0x0020) 가 없는** pose. SEEDED = 상태 주입으로 시작한 W 의 모든 pose 에 붙는 새 플래그 (폰 쪽 변경, 앱 다시 빌드 필요). 부팅 (`fs_init`).
- **F_RESET 반복**: 폰이 새 W 의 처음 `VIOP_RESET_REPEAT` = 50 pose 에 F_RESET 을 붙임 (예전 1 pose).
- **사용자 재초기화 경로**: PC `VIOP_T_CMD` cmd 6 `VIOP_CMD_VIO_RESET` → FC `FS_CMD_VIO_RESET` (DISARMED / KILLED 에서만, 이벤트 `FS_EV_VIO_RESET_REQ` 9) → link task 가 USB 로 폰에 `VIOP_T_VIO_RESET_REQ` 0x23 → ov_bridge 가 전체 재초기화 (kReset 과 같게, 상태 주입 아님).
- 재생 예측 (돌리기 전에 적음): 75개 흐름은 모두 DISARMED 이고 SEEDED 가 없어 결과가 1절과 **같아야 함**. 합격 기준은 0절 P1–P5 그대로.

## 5. v2 결과 (재생 다시 돌림, 4절 커밋 f522d2d 뒤) — 예측대로 1절과 같음, 숫자는 미검증
- `armhist_replay.json` 가 v1 과 `made`·binary 경로 말고 **완전히 같음**, P1–P5 합격. (재생 흐름은 모두 DISARMED, SEEDED 없음 → v2 차이는 재생으로는 시험 안 됨, PC 단위시험으로만 확인.)
- **효과 나누기 (정직하게)**:
  - armguard (순간 검사: 정지 1 s · 자이로 0.05 · 가속 분산 0.1 · |v| ≤ 0.2): 손에 든 채 arm 17,262 → 0, 발산 D 2,089 → 1,524, 첫 arm 지연 중앙 0.01 → 1.23 s (최대 0.49 → 2.35 s). 사후 선택이라 미검증.
  - 이력 래치 (이 작업): 발산 D 1,524 → 0 만. 이 1,524 는 전부 151646 NL60 k2·k3 두 실행이고 라벨이 |v| > 2.0 이라 **일부 정의상 당연** (독립 근거 = 경로 길이뿐). 늘어난 거부·지연 0. lost 부분은 이 데이터에서 이득 0.
  - 그러니 "발산을 막았다" 가 아니라 "이 두 순환 라벨 실행을 문턱 2.0 이 막는다" 로 읽어야 함.
- PC 단위시험 **700 통과 / 0 실패** (v1 678, +22): 무장·비행·KILLED 중 튐/lost 는 래치 안 함 → 착지·DISARM 뒤 바로 arm 가능, CLEAR_KILL 뒤 DISARMED 에서 다시 쌓임, 켜진 래치는 KILL/CLEAR_KILL 뒤에도 유지, SEEDED W 는 안 풂 (F_RESET·카운터 변경이 있어도), SEEDED W 뒤 전체 재초기화는 풂, 앞 5 프레임을 잃어도 반복된 F_RESET 으로 풂, FS_CMD_VIO_RESET 은 DISARMED·KILLED 에서만 (무장 중 거부 이벤트).
- `make syntax` 6 변형 통과 (link_task.c 의 USB 쓰기를 위해 host stub 2줄 추가). ESP-IDF 증분 빌드 성공, 경고 0, 앱 0xc0100 (`idf_build_default.log`).
- 폰 쪽 (별도 커밋, **앱 다시 빌드 필요**): `scan3d/ov_bridge/main.cpp`, `mav_out.h` — 상태 주입 W 의 모든 pose 에 VIOP_F_SEEDED, F_RESET 을 새 W 의 처음 50 pose 에, UDP 소켓으로 받은 0x23 (확인값 맞을 때) → 다음 입력에서 전체 재초기화 (kReset 과 같은 `full_reset()`). PC 에서 ov_bridge 빌드만 됨 (경고·오류 없음), **동작 시험 안 함**.
- 남은 것 (추정): 폰 앱 (Java) 이 FC 에서 USB 로 온 바이트를 ov_bridge UDP 소켓에 넘기는지 확인 안 함 → 안 넘기면 0x23 은 도착 안 하고, 사용자 재초기화는 앱 재시작뿐. FC 가 폰보다 늦게 켜지면 그 전 이력을 모름 (검증 지적, 안 고침). 위치 점프 검사 없음.
