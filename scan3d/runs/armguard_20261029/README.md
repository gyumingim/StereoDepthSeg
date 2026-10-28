# armguard 2026-10-29 — ESP32 FC arm 보호 기본 켜짐

브랜치 `fcarm-20261027`. 사용자 결정 "일단 켜셈". PC 만 씀 (보드·폰 안 씀).

## 결론 먼저
1. `fs_config_default()` 의 arm 보호가 **기본 켜짐**: `arm_still_ms 1000`, `arm_gyro_max_rps 0.05`, `arm_acc_var_max 0.1`, `arm_vio_speed_max 0.2`, `arm_vio_speed_hold_ms 0`.
   예전 값 (즉시 arm): still 0, gyro 0.2, acc_var -1 (= 0.5), speed 0, hold 0.
2. 근거는 fcarm 의 **사후 (사전 규칙 밖)** 선택이고, 아래 재생도 **같은 데이터** 라서 **미검증** 입니다.
3. 75개 재생 흐름 (bench 69 + VIOP 캡처 6), fcarm 과 같은 구간·채점:

| | 예전 값 | 새 기본값 |
|---|---|---|
| 정상 거부 FR (정상 60 실행, 내려놓고 3 s 뒤) | 0 / 29,757 | **0** / 29,757 |
| 손에 든 채 arm 가능 HA (주기) | 17,262 / 507,712 (3.4 %) | **0** |
| 발산 끝 정지 arm 가능, D 9개 | 2,089 | 1,524 (= 151646 NL60 k2 762 + k3 762) |
| 같은 것, D' 7개 (위 2개 뺌) | 347 (전부 151646 NL60 k0) | **0** (std 검사 꺼도 0) |
| 내려놓은 뒤 첫 arm (끝 정지 ≥ 3 s 인 정상 47 실행) | 중앙 0.01 s, 최대 0.49 s | **중앙 1.23 s, 90 % 2.35 s, 최대 2.35 s** |
| 실행별 늘어난 지연 | | 중앙 +1.22 s, 최대 +1.86 s |

   - 정상 arm 이 끝까지 거부된 실행: **0** (끝 정지가 있는 47 실행 모두 arm 됨). HS 131326·154320 의 13 실행은 녹화 끝까지 움직여서 예전 값으로도 끝 정지 arm 이 없음 (끝 정지 0 s, 비교 대상 아님).
   - VIOP 캡처 6개: 정상 5개 첫 arm 0.01–0.49 → 0.93–2.35 s, 발산 1개 (151646 k0 NL60V) 는 예전·새 모두 0.
   - `fs_config_default()` 그대로와 같은 값을 직접 넣은 설정의 출력이 75개 모두 같음 (`default_equals_explicit_set: true`).
   - fcarm README 의 예전 값 숫자 (17,262 / 2,089 / 347) 와 사후 튼튼 조합 {1000, 0.05, 0.1, 0.2, 0} 의 첫 arm (1.23 / 2.35 s) 을 그대로 다시 냄.
4. **풀리지 않은 것**: 151646 NL60 k2·k3 (손에 든 동안 VIO |v| 7.1 / 2.8 m/s 로 튀었다가 내려놓을 때 정상처럼 보임) 는 새 기본값으로도 arm 됩니다. "초기화 뒤 한 번이라도 크게 튀었으면 VIO 재초기화 전까지 arm 금지" 같은 **이력 검사가 필요 (안 만듦)**.

## 검사
- PC 단위시험 **644 통과 / 0 실패** (`test_safety_run.log`; 예전 610). 시나리오 검사 (착지 단계·끊김·kill·open loop 등) 는 `cfg_former_arm()` 으로 예전 arm 값을 직접 넣어 예전 시간 그대로 돌림.
- 새 검사 `t_arm_guard_defaults`: 기본값 확인, 1 s 정지 (990 ms 거부 / 1000 ms 통과), 손에 든 자이로 0.1 · 흔들리는 가속 (분산 ~0.3) 거부, 내려놓으면 1 s 다시 셈, 한 주기 0.06 rad/s 충격이면 다시 셈, |v| 0.25 거부 · 0.2 통과 · NaN 거부, 속도를 끝까지 모르면 거부, CLEAR_KILL 은 MOVING 규칙 (자이로 0.1 · 가속 분산 거부) 만 따르고 정지 시간·속도는 안 봄 (그 뒤 ARM 은 기다림).
- 새 검사 `t_arm_guard_sim` (1 kHz 폐루프 시뮬레이션, 기본값): 600 ms 에 arm 거부 (NOT_STILL), 1050 ms 에 arm, 이륙·착지, 착지 직후 재 arm 거부, 1.5 s 뒤 arm.
- `make syntax` 6 변형 통과 (`syntax.log`). ESP-IDF v5.5.6 실제 빌드 (기본 변형) 성공, 경고 0, 앱 0xbfd40 (`idf_build_default.log`).

## 파일 · 재현
- `armguard_eval.py` (재생·집계) → `armguard_replay.json`, `armguard_replay_configs.txt`.
- `../fcarm_20261027/fcarm_replay.c` 는 모든 설정 줄을 직접 적용하도록 고침 (기본값이 바뀌어서: 0번 줄 = 예전 값을 직접 넣음, `default` 줄 = 손대지 않은 기본값). 예전 configs.txt 의 0번 줄이 예전 기본값과 같아 fcarm 결과는 그대로 재현됨.
- 재현 (WSL, 이 폴더에서): `gcc -std=c99 -O2 -I../../../esp32fc/components/fc_safety/include ../fcarm_20261027/fcarm_replay.c ../../../esp32fc/components/fc_safety/fc_safety.c -lm -o /tmp/ag_fr && python3 armguard_eval.py /tmp/ag_fr ~/scanner/tmp_fcarm/sweep1/ev ../fcarm_20261027 armguard_replay.json`
- 입력: `~/scanner/tmp_fcarm/sweep1/ev/*.txt` (fcarm 이 만든 75개 흐름, 저장소 밖), `../fcarm_20261027/results.json` · `posthoc.json` (라벨·구간).

## 한계
- fcarm README 4절의 한계 그대로 (녹화 6개, 책상 위 끝 정지 4개, 10 ms 재생, 자이로 type 16 가정, 기체 진동·온도 없음). 첫 arm 시간은 약 0.1 s 짧게 잡혔을 수 있음 (추정).
- 기체 위에서 바람·진동으로 흔들리면 1 s 정지를 못 채울 수 있음 (추정, 미확인).
