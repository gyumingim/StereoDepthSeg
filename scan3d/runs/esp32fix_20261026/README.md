# esp32fix 2026-10-26 — ESP32 비상착륙(open-loop) 모터 끄기 + VIOP heartbeat 20 Hz 고정

브랜치 `esp32fix-20261026` (integ-20261025 14de3b8 에서 분기). PC 만, 폰·하드웨어 없음. 앱 설치 안 함.

## 결론 (23:30 갱신: 코디네이터 지시로 **기본값은 예전 그대로**, 새 방식은 선택 사항 — 사용자가 정함)
1. **폰 IMU 끊김 → OPENLOOP 착륙의 새 모터 끄기 방식 `openloop_cut_mode = 1` (선택, 기본 꺼짐)**.
   - 기본값 = esp32fc 그대로: `openloop_cut_mode = 0`, `openloop_vz_down = 0.4`, `openloop_cruise_ratio = 0.99`.
   - 켜는 법: `openloop_cut_mode = 1`, `openloop_vz_down = 0.6`, `openloop_cruise_ratio = 0.95` (시험에서 쓴 조합).
   - 예전 방식의 문제 (시뮬레이션): 1 m 에서 착지 뒤 hover 98 % 추력으로 1.2–1.7 s 더 돌고, 기록값이 실제보다 3 % 크면 하강이 멈춰 **공중에서 모터가 꺼짐**.
   - mode 1:
     - 마지막 VIO 높이에서 추력 −3 % / +3 % 두 경우를 미리 계산해 가장 이른/늦은 착지 시각 t_fast / t_slow 를 구함.
     - t_fast 부터 0.4 s 동안 추력을 hover 의 70 % 로 내리고, t_slow + 0.2 s 에 끔.
     - 폰 pose 가 아직 오면: VIO 로 하강(vz < −0.3 m/s)을 본 뒤, 높이 < 0.10 m · |vz| < 0.2 m/s 가 0.15 s 이면 바로 끔.
     - 타이머 시각이 되어도 VIO 가 아직 하강(vz ≤ −0.2 m/s)을 보이면 끄지 않고 기다림 (최대 2 s 더; 땅에서 넘어져 VIO 가 계속 움직임을 보여도 상한에서 끔).
2. **VIOP heartbeat 를 고정 20 Hz 스레드로** (`ov_bridge --viop-hb-src 1`, 앱 intent `viop_hb_src 1`). **기본값 0 = 예전 그대로**(메인 루프).
   - 살아있음 검사 유지: IMU 수신이 100 ms 넘게 없거나 메인 루프가 200 ms 넘게 IMU 를 처리 못 하면 그 박자는 안 보냄 → ESP32 250 ms 제한이 그대로 동작.

## 근거 (PC, 결정적 시뮬레이션 1회씩)
추력 오차는 **실제 기록값 기준** (기록 평균 × 오차 = 실제 추력; 23:30 수정 — 그 전에는 기록이 1–1.5 % 낮게 잡혀 오차가 명목값과 달랐음).

| 항목 | mode 0 (기본) | mode 1 (선택) | 파일 |
|---|---|---|---|
| PC 단위시험 | — | **573 통과** (ASan/UBSan), 문법 검사 6 조합 통과 | `test_fix3.log`, `syntax_fix3.log` |
| 높이 0.3/1/2 m × 기록오차 −3/0/+3 %: 공중에서 꺼짐 | 3/9 (최대 3.37 m) | 0/9 — **같은 모델 안에서** (예측식 = 시험 기체식) | `test_fix3.log` (`t_openloop_cut`) |
| 같은 9 경우: 땅에서 85 % hover 넘게 도는 최대 시간 | 3506 ms | 118 ms | 〃 |
| 같은 9 경우: 최대 충돌 속도 | 8.17 m/s (공중 차단) | 2.35 m/s | 〃 |
| 1 m, 기록 정확: 착지 뒤 도는 시간 / 충돌 속도 | 1245 ms / 0.58 m/s | 482 ms / 1.23 m/s (대가) | 〃 |
| VIO 살아있음, 기록오차 −8 … +8 %, 1·2 m (12 경우) | — | 공중 차단 0, 최대 충돌 3.08 m/s | 〃 |
| VIO 높이 0.95 m 틀림 (1 m 호버가 0.05 m 로 보임) | 780 ms 에 0.76 m 에서 끔 | 착지 순간 끔 (z 0.00, 2.31 m/s) | 〃 |
| VIO 가 땅에서 계속 하강을 보임 (vz −0.5 편향) | — | 타이머 + 2000 ms 에 끔 (상한 동작) | 〃 |

### 같은 모델 밖에서의 실패 범위 (두 번째 독립 검증, `scratchpad\vx\out\`, 실제 fc_safety.c 에 다른 기체 모델; h 0.3–3 m × ±3 %, 15 경우)
| 바꾼 조건 | mode 1 공중 차단 | 예전(mode 0) 공중 차단 |
|---|---|---|
| 같은 식 | 0/15 | 5/15 |
| 지면효과·차체저항·전압 회복·추력지수 2.5·하강 중 시작·기울어짐 | 0/15 | (보고 없음) |
| 로터 수직감쇠 Zw 0.6/s (현실값 추정) | 2/15 (최고 0.70 m) | 8/15 |
| Zw 1.0/s | 5/15 (1.13 m) | 10/15 |
| 추력지수 1.6 | 3/15 (0.41 m) | 5/15 |
| 0.5 m/s 로 오르던 중 끊김 | 9/15 (1.15 m, 5.6 m/s) | 12/15 |
| 복합 모델 | 7/15 (1.23 m) | 15/15 |
| 기록오차 +5 % 이상 (35 경우) | 8/35 (2.32 m, 7.0 m/s) | 15/35 |

(이 표는 VIO 없는 경우. 위 "타이머가 VIO 하강을 기다림" 수정 전 숫자.) mode 1 은 모든 변형에서 예전보다 덜/낮게 끊지만, 실제 기체에서 공중 차단이 없다는 보장은 아니다.

Python 모델 (같은 식, 0.2–3 m × ±1.5/±3 %): `olsim3_out.txt` (`olsim*.py`). 시뮬레이션은 1-D 수직 모델(추력 ∝ 명령²) — 기울기·바람·지면효과 없음.

### heartbeat (PC, 실시간 속도 재생, C4 harness 845 프레임, --pred-thread 1 --predict 2 @200 Hz, 2회씩, 같은 바이너리)
| run | 박자 수 | 평균 Hz | 간격 p50 / p95 / max (ms, PC 수신) | > 75 ms |
|---|---|---|---|---|
| OLD_1 (src 0) | 518 | 17.09 | 54.3 / 78.5 / 127.2 | 36 |
| OLD_2 | 512 | 16.87 | 54.9 / 81.6 / 134.4 | 53 |
| NEW_1 (src 1) | 608 | 20.00 | 50.0 / 53.3 / 151.1* | 2 |
| NEW_2 | 609 | 20.07 | 50.0 / 53.1 / 59.8 | 0 |

\* NEW_1 의 151 ms 는 PC 수신 쪽(Python 수신기, 부하 20) 간격; 브리지가 보낸 간격의 max 는 77.2 ms. PC 는 다른 에이전트와 공유(load 18–20) → 시간 값은 참고용. NEW 의 "feed stale" 건너뜀 18–20 회 = 재생 시작 전 1 s.
- 멈춤 시험 (raw80, feeder 를 25 s 에 2 s 멈춤): src 1 = 간격 max 2000 ms 후 다시 20 Hz (23:30 수정 후 재실행도 같음: `hb_pause_pc_mode1_fix3.log`), src 0 = 2054 ms.
- 로그: `hb_rate_pc.log`, `hb_pause_pc_mode{0,1}.log`, 스크립트 `hb_rate_pc.sh`, `hb_pause_pc.sh`, `hbstat.py`.
- 23:30: reader 스레드는 `--viop-hb-src 1` 일 때만 링크 객체를 건드림 (전역 `g_hb_feed_out`), 그때는 종료 시 객체를 해제하지 않음 → 기본값에서 종료 경쟁 없음.

### 동일성 (자세 출력)
integ 14de3b8 빌드 vs esp32fix 빌드, 시간 키(ms, remap_ms, wait_ms) 뺀 pose 줄: harness 845/845 (기본, viop src 0, viop src 1), raw80 `--profile rec20261025 --drop-late 0` 2327/2327 (src 0/1) **모두 IDENTICAL** — 23:30 수정 후 다시 확인 (`ident_pc_fix3.log`; 처음 `ident_pc.log`).

## 바뀐 파일
- `esp32fc/components/fc_safety/{fc_safety.c, include/fc_safety.h}`: `openloop_cut_mode` (기본 0), `openloop_thr_err`, `openloop_end_ratio`, `openloop_end_ramp_s`, `openloop_hold_s`, `openloop_vio_touch(_h, _ms)`, `openloop_vio_desc_vz`, `openloop_vio_extend_ms`; 상태 `ol_fast_ms`, `ol_slow_ms`, `ol_ramp`, `ol_desc_seen`. mode 0 동작은 esp32fc 와 같음 (기존 시험 수치 그대로: 1 m 착지 1858 ms, 끔 3526 ms).
- `esp32fc/test/test_safety.c`: `t_openloop_cut` (mode 0/1 스윕, VIO 착지, VIO 높이 오차 착지까지, 타이머 대기, 대기 상한, mode 1 추력 단계). `t_phone_imu` 는 원래대로.
- `esp32fc/README.md`, `scan3d/ov_bridge/{mav_out.h, main.cpp}` (`--viop-hb-src 0|1`), `phone_stereo/arlat/.../{MainActivity,OvRunner}.java` (intent `viop_hb_src`; 0 이면 명령줄 그대로, meta.json 에 키 하나 추가).

## 열린 위험 (고치지 않음, 사용자 결정 필요)
- (c) IMU 가 10 ms 만 끊겨도 OPENLOOP 으로 들어가 되돌아오지 않음.
- (c) ESP32 가 pose horizon(예측 길이)을 확인하지 않음; `vio_usable` 이 heartbeat 시간초과를 보지 않음 → 브리지가 멈추면 추측 pose 로 VIO 착륙, src 1 이면 그 시작이 약 150 ms 늦음.
- IMU 단계 착륙 중 OPENLOOP 으로 넘어오면 높이가 낡았고 이미 내려가는 중 (mode 0·1 공통).
- mode 1 띠 계산 비용(ESP32 제어 루프 안 2×1200 반복)은 추정 수백 µs–1 ms, 측정 안 함.
- heartbeat src 1: 메인 루프가 멈추면 ESP32 시간초과까지 약 400–450 ms (src 0 약 250 ms, 계산). IMU_STALE 기준이 수신 스레드 시각.

## 확인 안 된 것
- 실제 기체·ESP32 보드·IDF 빌드 없음 (스텁 문법 검사만). mode 1 수치(±3 %, 70 %, 0.6 m/s, 2 s 상한)는 시뮬레이션 근거만.
- heartbeat 의 "메인 루프 200 ms 멈춤 → 박자 중단" 은 이 브랜치 시험에 없음 (두 번째 검증의 단위시험 `scratchpad\vx\hbtest.cpp` 에서 150 ms 확인).
- 폰에서 heartbeat 스레드 실행 안 함. APK 는 컴파일 확인만 (`apk_check.log`, 23:30 수정 전 빌드).

## 되돌리기
- 펌웨어: 기본값이 이미 esp32fc 동작. mode 1 은 설정으로만 켜짐.
- 브리지/앱: `--viop-hb-src` 를 안 주면 예전과 같음. 브랜치 통째로 안 쓰면 integ 그대로.
