# esp32fix 2026-10-26 — ESP32 비상착륙(open-loop) 모터 끄기 + VIOP heartbeat 20 Hz 고정

브랜치 `esp32fix-20261026` (integ-20261025 14de3b8 에서 분기). PC 만, 폰·하드웨어 없음. 앱 설치 안 함.

## 결론
1. **폰 IMU 끊김 → OPENLOOP 착륙에서 착지 뒤 모터가 ~1.7 s 도는 문제 고침** (기본값 변경 = 안전 버그 수정).
   - 예전: 1 m 에서 착지 1.86 s, 모터 끔 3.53 s → 땅에서 hover 98 % 추력으로 1.67 s. 또 기록값이 3 % 높으면 하강이 멈춰 **공중에서 모터가 꺼짐**(C 시뮬레이션: 1 m → 0.09 m 높이, 2 m → 1.62 m 높이에서 꺼져 5.6 m/s 낙하).
   - 지금 (`openloop_cut_mode = 1`): 마지막 VIO 높이에서 추력 −3 % / +3 % 두 경우를 미리 계산해 가장 이른/늦은 착지 시각 t_fast / t_slow. t_fast 부터 0.4 s 동안 추력을 hover 의 70 % 로 내리고, t_slow + 0.2 s 에 끔. 폰 pose 가 아직 오면, VIO 로 하강(vz < −0.3 m/s)을 본 뒤에 높이 < 0.10 m · |vz| < 0.2 m/s 가 0.15 s 이면 바로 끔.
   - 하강 속도 0.4 → 0.6 m/s, 순항 추력 0.99 → 0.95 (예전 값으로 되돌리기: `openloop_cut_mode = 0`, `openloop_vz_down = 0.4`, `openloop_cruise_ratio = 0.99`).
2. **VIOP heartbeat 를 고정 20 Hz 스레드로** (`ov_bridge --viop-hb-src 1`, 앱 intent `viop_hb_src 1`). **기본값 0 = 예전 그대로**(메인 루프).
   - 살아있음 검사 유지: IMU 수신이 100 ms 넘게 없거나 메인 루프가 200 ms 넘게 IMU 를 처리 못 하면 그 박자는 안 보냄 → ESP32 250 ms 제한이 그대로 동작.

## 근거 (PC, 1회씩, 결정적 시뮬레이션은 반복 불필요)
| 항목 | 예전 | 지금 | 파일 |
|---|---|---|---|
| PC 단위시험 | 389 통과 | **516 통과** (ASan/UBSan) | `test_fix2.log` (스윕 수치는 `test_m1b.log` 와 같음) (`test_base.log` = 시작 시점 389) |
| 펌웨어 문법 검사 6 조합 | 통과 | 통과 | `syntax_fix2.log` |
| 1 m, 폰 IMU+링크 끊김: 땅에서 도는 시간 | 1667 ms (98 % hover) | 603 ms (85 % 넘는 시간 29 ms) | `test_m1b.log` |
| 높이 0.3/1/2 m × 추력오차 −3/0/+3 %: 공중에서 꺼짐 | 2/9 (최대 1.62 m) | **0/9** | `test_m1b.log` (`t_openloop_cut`) |
| 같은 9 경우: 땅에서 85 % hover 넘게 도는 최대 시간 | 3693 ms | 181 ms | 〃 |
| 같은 9 경우: 최대 충돌 속도 | 5.64 m/s (공중 차단) | 2.19 m/s | 〃 |
| 1 m, 정상 기록: 충돌 속도 | 0.80 m/s | 1.24 m/s (대가) | 〃 |
| VIO 살아있을 때 (IMU 프레임만 끊김) | 타이머 | 착지 후 150 ms 에 끔 | 〃 |

Python 모델 (같은 식, 0.2–3 m × ±1.5/±3 %): `olsim3_out.txt` (`olsim*.py`). 시뮬레이션은 1-D 수직 모델(추력 ∝ 명령²) — 기울기·바람·지면효과 없음.

### heartbeat (PC, 실시간 속도 재생, C4 harness 845 프레임, --pred-thread 1 --predict 2 @200 Hz, 2회씩, 같은 바이너리)
| run | 박자 수 | 평균 Hz | 간격 p50 / p95 / max (ms, PC 수신) | > 75 ms |
|---|---|---|---|---|
| OLD_1 (src 0) | 518 | 17.09 | 54.3 / 78.5 / 127.2 | 36 |
| OLD_2 | 512 | 16.87 | 54.9 / 81.6 / 134.4 | 53 |
| NEW_1 (src 1) | 608 | 20.00 | 50.0 / 53.3 / 151.1* | 2 |
| NEW_2 | 609 | 20.07 | 50.0 / 53.1 / 59.8 | 0 |

\* NEW_1 의 151 ms 는 PC 수신 쪽(Python 수신기, 부하 20) 간격; 브리지가 보낸 간격의 max 는 77.2 ms (`[ov_bridge] viop heartbeat thread` 줄). PC 는 다른 에이전트와 공유(load 18–20) → 시간 값은 참고용. NEW 의 "feed stale" 건너뜀 18–20 회 = 재생 시작 전 1 s (아직 IMU 없음).
- 멈춤 시험 (raw80, feeder 를 25 s 에 2 s 멈춤): src 1 = 간격 max 2000 ms(멈춘 동안 안 보냄) 후 다시 20 Hz, src 0 = 2054 ms. `hb_pause_pc_mode1.log`, `hb_pause_pc_mode0.log`.
- 로그: `hb_rate_pc.log`, 스크립트 `hb_rate_pc.sh`, `hb_pause_pc.sh`, `hbstat.py`.

### 동일성 (자세 출력)
integ 14de3b8 빌드 vs esp32fix 빌드, 시간 키(ms, remap_ms, wait_ms) 뺀 pose 줄: harness 845/845 (기본, viop src 0, viop src 1), raw80 `--profile rec20261025 --drop-late 0` 2327/2327 (src 0/1) **모두 IDENTICAL**. `ident_pc.log`, `ident_pc.sh`. (drop-late 50 은 시간에 따라 프레임을 버리므로 비교에서 0.)

## 바뀐 파일
- `esp32fc/components/fc_safety/{fc_safety.c, include/fc_safety.h}`: `openloop_cut_mode`, `openloop_thr_err`, `openloop_end_ratio`, `openloop_end_ramp_s`, `openloop_hold_s`, `openloop_vio_touch(_h, _ms)`; 상태 `ol_fast_ms`, `ol_slow_ms`, `ol_ramp`.
- `esp32fc/test/test_safety.c`: `t_openloop_cut` (예전/지금 스윕 + VIO 착지), `t_phone_imu` 기대값 갱신 (폰 링크 전체 끊김으로 바꿈).
- `esp32fc/README.md`: OPENLOOP 설명·수치.
- `scan3d/ov_bridge/{mav_out.h, main.cpp}`: `--viop-hb-src 0|1`.
- `phone_stereo/arlat/.../{MainActivity,OvRunner}.java`: intent `viop_hb_src` (0 이면 명령줄 그대로; meta.json 에 `viop_hb_src` 키 추가).

## 독립 검증 (읽기 전용 에이전트, 22:45)
- 고친 것 (확인된 버그): VIO 착지 판정이 하강 확인 없이 걸려, 이륙 지점보다 높은 곳 위 호버 또는 VIO 높이 오차 때 공중(1.03 m)에서 150 ms 만에 끌 수 있었음 → 이 비행 단계에서 VIO 하강을 본 뒤에만 판정 (`openloop_vio_desc_vz` 0.3), 회귀 시험 `openloop_vio_touch_needs_descent` 추가. 숫자 불일치 2개(README 0.97 m, 코드 주석 2.98/0.12 s = Python 값) 고침.
- 남은 위험 (안 고침):
  - IMU 단계 착륙 중에 OPENLOOP 으로 넘어오면 높이가 낡았고 이미 내려가는 중 → t_fast 전에 닿아 0.95 추력으로 땅에서 돌 수 있음 (예전 코드도 같은 낡은 높이 사용).
  - 시험의 기체 모델과 예측 모델이 같은 식(추력 ∝ 명령², 공기저항 없음) → 실제 착지는 더 늦을 수 있음.
  - 띠 계산 비용(ESP32 제어 루프 안 2×1200 반복)은 추정 수백 µs–1 ms, 측정 안 함 (최악 루프 초과 1회).
  - heartbeat src 1: 메인 루프가 멈추면 ESP32 가 알아채는 데 약 450 ms (src 0 은 약 250 ms, 추정). IMU_STALE 기준이 수신 스레드 시각.
  - 종료 때 reader 스레드가 g_mav 를 볼 수 있는 경쟁 (출력 쓰기 실패 뒤 프로세스 종료 시에만, 낮음).

## 확인 안 된 것
- 실제 기체·ESP32 보드·IDF 빌드 없음 (스텁 문법 검사만). 착지 판정 수치(±3 %, 70 %, 0.6 m/s)는 시뮬레이션 근거만 — 실제 기록 오차 크기 모름.
- heartbeat 의 "메인 루프 200 ms 멈춤 → 박자 중단" 경로는 시험 못 함 (멈추게 할 방법 없음, 코드 검토만).
- 폰에서 heartbeat 스레드 실행 안 함 (앱 설치 안 함). APK 는 컴파일 확인만 (`apk_check.log`).

## 되돌리기
- 펌웨어: `openloop_cut_mode = 0`, `openloop_vz_down = 0.4f`, `openloop_cruise_ratio = 0.99f` → esp32fc 동작.
- 브리지/앱: `--viop-hb-src` 를 안 주면 예전과 같음. 브랜치 통째로 안 쓰면 integ 그대로.
