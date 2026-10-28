# esp32fc — ESP32-S3 비행 제어 펌웨어 (모터 정지 · 비상착륙 포함)

브랜치 `esp32fc-20261025` (ovopt-20261025 0fcf53a에서 분기), 2026-10-25. **보드에 올려 본 적은 없습니다.** ESP-IDF v5.5.6 실제 빌드는 fcarm 2026-10-27 에 처음 했습니다 (4가지 변형 모두 성공, 경고 0; `scan3d/runs/fcarm_20261027/README.md`).

## 한국어 요약 (결론 먼저)

- **언어: C (ESP-IDF + FreeRTOS).** 사용자가 이렇게 정했습니다.
- **사용자 결정 반영 (2026-10-25 오후)**
  - ESP32에는 IMU가 없습니다. **폰 IMU(약 500 Hz)가 유일한 IMU**입니다. 폰이 USB로 IMU 샘플을 보내면, 샘플이 도착할 때마다 자세 루프가 돕니다.
  - ESC는 **일반 PWM**입니다(기본 400 Hz, 50–490 Hz 설정 가능). OneShot125와 DShot은 선택 사항으로 남겼습니다.
  - 프레임은 **쿼드 X 450 mm**입니다. 게인은 아직 임시값입니다.
  - ToF 센서는 쓰지 않고, 높이는 VIO에서 얻습니다.
- **모터 KILL (즉시 정지)**
  - 원인: PC 명령, 물리 스위치, 기울기 60° 초과(착륙 중에는 45°), 출력 NaN, 루프 멈춤.
  - **같은 제어 주기 안에** 정지 펄스(1000 µs)를 내고, KILLED 상태로 고정합니다.
  - 해제는 땅 위에서 CLEAR_KILL 명령으로만 됩니다.
- **비상착륙 원인**: 폰 pose 끊김, 폰 heartbeat 끊김(250 ms), PC 링크 끊김(500 ms), 저전압, 명령, 루프 과부하, **폰 IMU 끊김(10 ms)**.
- **착륙 단계**
  1. **LAND_VIO**: 제자리 x,y를 유지하며 0.3 m/s로 하강합니다.
  2. **LAND_IMU**: 수평을 유지하고, 폰 IMU로 하강 속도를 추정합니다.
  3. **LAND_OPENLOOP**: 자세 제어가 없습니다. 이번 비행 이륙 때 기록한 모터별 값에서 출력을 **역산**합니다. 추력 ∝ 명령² 모델을 쓰고 배터리 전압 차이를 보정합니다. 처음 0.4 s는 1 m/s²로 하강을 시작하고, 이후는 hover의 99 % 추력으로 내려갑니다. 마지막 VIO 높이로 예상 착지 시간을 계산해 그 1.2배 뒤에 모터를 끕니다 (기본 = `openloop_cut_mode 0`).
     - **선택 사항 `openloop_cut_mode 1` (esp32fix 2026-10-26, 기본 꺼짐 — 사용자가 정함; 함께 `openloop_vz_down 0.6`, `openloop_cruise_ratio 0.95`)**: 마지막 VIO 높이에서, 기록값이 ±3 % 틀린 두 경우를 미리 계산해 가장 이른 착지 시각(t_fast)과 가장 늦은 착지 시각(t_slow)을 구합니다. t_fast부터 0.4 s 동안 추력을 hover의 70 %로 내리고, t_slow + 0.2 s에 모터를 끕니다. 폰 pose가 아직 오면(IMU 프레임만 끊긴 경우) VIO로 하강을 본 뒤 높이 < 0.10 m · |vz| < 0.2 m/s가 0.15 s 이어질 때 바로 끕니다. 타이머 시각에 VIO가 아직 하강을 보이면 최대 2 s 기다립니다. 기본 방식은 시뮬레이션에서 착지 뒤 98 % 추력으로 1.2–1.7 s 돌고, 기록값이 3 % 크면 공중에서 끕니다. 근거와 실패 범위: `scan3d/runs/esp32fix_20261026/README.md`.
     - 예전 방식(`openloop_cut_mode = 0`): 착지 후 약 1.7 s 동안 hover의 98 % 추력으로 돌았고, 기록값이 3 % 높으면 하강이 멈춰 공중에서 모터가 꺼졌습니다(시뮬레이션 1.6 m 높이).
  - **폰 IMU가 끊기면 VIO·IMU 단계가 둘 다 불가능하므로 바로 OPENLOOP으로 갑니다.** `imu_fail_kill` 설정을 켜면 대신 KILL합니다.
  - 단계는 내려가기만 하고, 착륙 중 비행으로 돌아가지 않습니다.
- **PC 단위시험: 644개 검사 모두 통과** (armguard 2026-10-29 새 기본값 검사 +34; fcarm 610, esp32fix 573, 그 전 389) (gcc, ASan/UBSan, 시뮬레이션). 검사 대상: 폰 IMU 끊김 → OPENLOOP, IMU 지터 처리, PWM 출력 한계.
- **ESP-IDF 실제 빌드 (fcarm 2026-10-27)**: WSL 에 ESP-IDF v5.5.6 설치 → `idf.py build` 성공. 기본(PWM·폰 IMU) · OneShot125 · DShot600 · 로컬 IMU 4가지 변형, 펌웨어 경고 0 (로컬 IMU 변형의 미사용 변수 경고 1개는 고침). 앱 이미지 0xbfd40 = 785 KB (1 MB 앱 파티션의 75 %). 보드에는 아직 안 올렸습니다.
- **arm 보호 (armguard 2026-10-29, 기본 켜짐 — 사용자 결정 "일단 켜셈")**: 기본값 , , , ,  ( 그대로).
  예전 값 (10-27 까지의 기본, 즉시 arm):  (끔), ,  (= touch_acc_var 0.5),  (끔), hold 0.
  - 뜻: 1 s 동안 끊김 없이 정지 (자이로 ≤ 0.05 rad/s · 가속 분산 ≤ 0.1 · ~1 g) + 마지막 유효 pose |v| ≤ 0.2 m/s 여야 arm. 속도를 모르면 (NaN) 거부. CLEAR_KILL 은 MOVING 규칙 (자이로 0.05 · 가속 분산 0.1) 만 따르고 정지 시간·속도는 보지 않음.
  - 근거 = **사후 (사전 규칙 밖)** 선택: fcarm 사전 규칙으로는 가능한 조합 0개였고, 151646 NL60 k2·k3 두 발산 실행을 빼고 고른 "덜 아슬아슬한" 튼튼 조합입니다 (). 같은 데이터로 고르고 같은 데이터로 재생했으므로 **미검증**.
  - 75개 재생 흐름 (새 기본값, ): 손에 든 채 arm 가능 17,262 → 0 주기, 정상 실행 내려놓고 3 s 뒤 새 거부 0, **내려놓은 뒤 첫 arm 중앙 0.01 → 1.23 s, 최대 0.49 → 2.35 s**.
  - **풀리지 않은 것**: 위 두 발산 실행 (손에 든 동안 크게 튀었다가 내려놓을 때 정상처럼 보임) 은 새 기본값으로도 끝 정지에서 arm 됩니다 (각 762 주기). "초기화 뒤 한 번이라도 크게 튀었으면 VIO 재초기화 전까지 arm 금지" 같은 **이력 검사가 필요 (안 만듦)**.
  - 예전처럼 쓰려면 위 예전 값을  에 넣으면 됩니다 (PC 시나리오 검사들은  으로 그렇게 함).

### ⚠ 안전 위험: 비행 제어기에 독립 IMU가 없음 (사용자에게)

자세 제어가 **폰 + 앱 + USB 케이블**이 모두 정상일 때만 동작합니다.

- 셋 중 하나라도 10 ms 넘게 끊기면 남는 수단은 **자세 제어 없는 OPENLOOP 착륙**뿐입니다. 기체가 기울거나 흘러갈 수 있습니다.
- 끊김의 예: 앱 GC·스케줄링 지연, USB 순간 끊김, 앱 종료, 폰 과열 자체정지(40 °C).
- 폰 IMU 샘플은 USB와 PWM을 거치며 **약 2–8 ms(추정)** 늦게 반영됩니다. ESP32에 IMU가 직접 붙은 경우는 0.1–0.5 ms입니다. 그래서 자세 루프 게인을 낮춰야 하고, 외란에 덜 버팁니다.
- **먼저 할 일**: 실제 폰에서 IMU 샘플 간격(최대 gap) 분포를 재고, 10 ms 기준이 적절한지 정해야 합니다. 너무 빡빡하면 멀쩡한 비행 중에 OPENLOOP 착륙이 걸립니다.
- **값싼 보완책 (이번에는 추가하지 않음)**:
  - 3–5천 원짜리 IMU 모듈(예: ICM-42688 / MPU-6500 브레이크아웃)을 ESP32 SPI에 붙여 **비상용**으로만 쓰는 방법입니다.
  - 그러면 폰이 끊겨도 LAND_IMU(수평 유지 착륙)가 가능합니다.
  - 드라이버는 이미 선택 모듈(`FC_IMU_SOURCE=FC_IMU_LOCAL_ICM42688`)로 들어 있습니다. 다만 "평소엔 폰, 끊기면 로컬 IMU" 전환 로직은 아직 없습니다.

## 1. Language / framework (decision: C on ESP-IDF)

| option | verdict | why |
|---|---|---|
| **C on ESP-IDF (FreeRTOS)** | **chosen** (user decision) | Timing is deterministic and tasks are pinned to cores. Direct LEDC / RMT / USB (TinyUSB) drivers, ISRs in IRAM, `esp_timer`, task WDT. No hidden heap; vendor-supported. |
| Arduino-ESP32 | no | The `loop()` / `delay()` model and libraries allocate memory and log. Core and priority control is coarse. |
| Rust (esp-rs) | no (for now) | Drivers are less mature (TinyUSB CDC, LEDC/RMT), the team knows C, and the shared protocol header is C. |
| MicroPython | no | It is interpreted and has GC pauses (ms). A deterministic 500 Hz loop is not possible. |

How the firmware gets fast, deterministic control:

- **Core split.**
  - Core 1: `ctrl` task (priority 24) runs phone IMU samples → attitude → safety → rate PID → mixer → PWM. `pos` task (22) runs the position loop.
  - Core 0: Wi-Fi / ESP-NOW, TinyUSB, `link` (10), `supervisor` (24, above Wi-Fi 23).
- **Event-driven inner loop.** The USB CDC rx callback decodes `VIOP_T_IMU` frames, rotates each sample into the body frame, queues it, and wakes `ctrl` with `xTaskNotifyGive`.
  - `ctrl` integrates every new sample in order, each with its own **phone-timestamp dt**, then runs safety and control once.
  - It also wakes every 2 ms without samples, so the failsafe keeps running when the stream stops.
- **Jitter / staleness** are handled in `imu_stream_t` (fc_core, unit-tested):
  - duplicates and out-of-order samples are dropped;
  - a burst of samples uses phone time, not arrival time;
  - a long hole is never integrated in one step (dt clamped to 2× nominal);
  - a phone-time gap > 10 ms, or no sample for > 10 ms of local time, means IMU failure → fc_safety;
  - a phone/app restart resyncs and is flagged.
- **Motors.** Standard PWM 1000–2000 µs via LEDC, 14 bit, 400 Hz default (50–490). A new value takes effect at the next period (≤ 2.5 ms at 400 Hz). OneShot125 (LEDC, 2 kHz) and DShot600/300 (RMT) are compile-time options.
- **In the loops:** no `malloc`, no `ESP_LOG`, no blocking. Queues use timeout 0, shared structs are copied under a spinlock, and `fc_safety` has a single owner.
- **sdkconfig.defaults:** 240 MHz, no DFS, `-O2`, FreeRTOS 1 kHz, radio / timer / USB tasks on core 0, IRAM-safe ISRs, task WDT 1 s → panic → reboot (outputs stop), interrupt WDT, log level WARN.
- **No flash writes in flight.** The hover record goes to RTC RAM in flight, and to NVS after disarm.

## 2. Layout

```
esp32fc/
  CMakeLists.txt, sdkconfig.defaults
  main/            ESP-IDF app (built with ESP-IDF v5.5.6, fcarm 2026-10-27; not on hardware yet)
    fc_config.h      all hardware assumptions / user decisions (IMU source, PWM, frame, pins, battery, phone mount)
    main.c           init order (motors first = stop pulse), queues (fs inbox, events, phone IMU samples), tasks
    ctrl_task.c      event-driven inner loop on phone IMU samples -> Mahony -> fs_step -> control per mode -> outputs
    pos_task.c       position loop (woken per phone pose, <= 200 Hz), latency compensation, land-in-place xy hold
    link_task.c      USB CDC (TinyUSB; phone = USB host): VIOP pose / heartbeat / IMU; ESP-NOW to the PC
    supervisor_task.c fast SW watchdog (loop silent 20 ms -> stop pulses), 2nd kill-switch path, battery ADC 50 Hz
    motor_out.c/.h   PWM (default) / OneShot125 via LEDC, DShot600/300 via RMT; e-stop callable from any core
    imu_icm42688.c   OPTIONAL local IMU driver (FC_IMU_SOURCE = FC_IMU_LOCAL_ICM42688), not in the default build
    board.c/.h       battery ADC, kill switch ISR, LED, hover record in RTC no-init RAM + NVS
  components/
    fc_safety/       PURE C safety state machine (unit-tested on PC)
    fc_core/         PURE C Mahony, vertical-speed estimator, PID, attitude/position control, quad-X mixer,
                     phone IMU stream (jitter/gap), PWM pulse/duty helpers
    viop/            viop_proto.h (copy of scan3d/ov_bridge/viop_proto.h; FC + IMU message types merged in, integ-20261025)
  test/              `make run` (unit tests), `make syntax` (firmware -fsyntax-only, 6 build variants, IDF stubs)
```

## 3. Safety state machine (components/fc_safety)

States: `DISARMED, ARMED_IDLE, FLYING, EMERG_LAND, KILLED`. Anything not in this table is refused and logged as `FS_EV_ILLEGAL`; the tests assert none happen.

| from \ to | DISARMED | ARMED_IDLE | FLYING | EMERG_LAND | KILLED |
|---|---|---|---|---|---|
| DISARMED | | ARM cmd + all arm checks | | | kill cmd / switch |
| ARMED_IDLE | DISARM cmd, idle 5 s, any land trigger on the ground | | collective > 0.15 (after it was seen low) | | any kill |
| FLYING | | normal touchdown detected | | any land trigger | any kill |
| EMERG_LAND | touchdown / impact / open-loop cut | | **never (no auto-resume)** | | any kill |
| KILLED | CLEAR_KILL on ground (level, still, ~1 g, switch released, ≥ 1 s) | | | | |

**KILL** happens immediately: the stop pulse goes out in the same cycle, and the state latches.

| trigger | default |
|---|---|
| PC command | — |
| physical switch | ISR latch + level read, plus a supervisor path |
| tilt | > 60° (> 45° while landing) |
| gyro rate | > 1500 °/s |
| motor output NaN / out of range | — |
| control loop stall | 20 ms (supervisor) |
| phone IMU loss | only with `imu_fail_kill=true` (the default lands open-loop) |

**EMERGENCY LAND triggers:**

| trigger | default |
|---|---|
| no GOOD pose | 200 ms |
| pose lost flag / VIO reset in flight | immediate |
| phone heartbeat missing | 250 ms |
| heartbeat seq frozen | 250 ms |
| heartbeat STARTING / LOST / STOPPING / IMU_STALE / POSE_STALE | immediate |
| USB port closed (DTR 1→0) | immediate |
| PC link silent | 500 ms |
| battery < 3.45 V/cell | debounced 2 s |
| loop overruns | leaky bucket |
| position task dead | — |
| LAND command | — |
| **phone IMU stream stale** | > 10 ms (only when `imu_fail_kill` is off) |

**Tiers.** Only step down; every change is logged as `FS_EV_TIER`.

| tier | condition | control | end |
|---|---|---|---|
| LAND_VIO | pose fresh + heartbeat ok + phone IMU ok | pos loop holds land-start x,y; collective PI on VIO vz → −0.30 m/s | touchdown (accel quiet, \|f\|≈g, low collective or impact, \|vz\| small, 300 ms) |
| LAND_IMU | no usable VIO, phone IMU ok | level, yaw hold; vz from the IMU estimator, else a timed collective ramp | same touchdown test |
| LAND_OPENLOOP | **phone IMU lost** (→ VIO and IMU tiers impossible), or a VIO/IMU landing longer than 30 s | **no attitude control.** Per motor: `cmd_i = hover_i · r^(1/k) · V_rec/V_now`, with k = 2 (thrust ∝ cmd²). r = 1 − 1.0/g for 0.4 s (descend at ~0.4 m/s), then r = 0.99 (mode 1 as tested: 0.6 s / 0.6 m/s, then 0.95; from t_fast r ramps to 0.70 in 0.4 s). | **mode 1 (opt-in, esp32fix; default off):** at the tier start the profile is simulated from the last VIO height with the thrust off by −3 % / +3 % → earliest / latest touchdown t_fast / t_slow; cut at t_slow + 0.2 s (max 6 s). If poses + heartbeat still arrive and a VIO descent (vz < −0.3 m/s) was seen: cut when height < 0.10 m and \|vz\| < 0.2 m/s for 150 ms. While VIO is usable and still shows a descent (vz ≤ −0.2 m/s) the timer cut waits, at most 2 s. Impact cut if an accelerometer is still available. **mode 0 (default, esp32fc):** cut at (touchdown estimate with constant 0.4 m/s) × 1.2 + 0.3 s. |

**This flight's takeoff record** (the source for "back-calculate from the vertical takeoff"):

- Reset at every ARM. Starts when lift-off is proven (vz > 0.1 m/s), plus 300 ms.
- Averages per-motor commands and battery voltage over the stable climb and first hover. Valid after 0.5 s, frozen after 3 s.
- Fallbacks: previous flight (RTC RAM, else NVS) → `openloop_default_thr` 0.33.
- The OPENLOOP numbers are **provisional and will be tested in sim / real later**: thrust exponent, 1 m/s², 0.6 m/s, 0.95, ±3 % record error band, end ratio 0.70, ramp 0.4 s, hold 0.2 s (legacy: 0.4 m/s, 0.99, margin 1.2 with `openloop_cut_mode = 0`).

**Arming** requires all of: level, still on the ground (~1 g; since armguard 2026-10-29 by default gyro <= 0.05 rad/s and accel variance <= 0.1 continuously for 1 s, and last valid VIO |v| <= 0.2 m/s, unknown refuses), collective low, phone IMU ok, pose fresh, heartbeat fresh, USB open, PC link, battery known ≥ 3.70 V/cell, overrun bucket low, switch released. Auto-disarm after 5 s idle. Every transition, tier change and rejection is an event sent to the PC (VIOP 0x04), plus a 20 Hz status frame (0x05).

## 4. PC unit tests (all pass)

`make -C esp32fc/test` builds with gcc 11.4 in WSL, `-Wall -Wextra -Werror -fsanitize=address,undefined`. Result: **644 checks, 0 failed** (armguard 2026-10-29: arm guard ON by default, +34 checks for the new defaults -- , ; the other scenarios use the former arm values explicitly via ; fcarm 2026-10-27: 610, +37 arm-hardening checks; esp32fix 2026-10-26: 573; 389 before). Raw logs: `scan3d/runs/esp32fix_20261026/test_*.log` (earlier: `scan3d/runs/esp32fc_20261025/test_safety_run.log`).

The sim is a 1 kHz 1-D vertical model: thrust ∝ cmd² (matches the open-loop model), ground contact with an impact spike, per-motor trims. It feeds phone pose 200 Hz, heartbeat 20 Hz and PC heartbeat 10 Hz.

New this round:

- **Phone IMU stall → OPENLOOP:** this flight's record is used; the cut time follows from the last VIO height; motors are cut by the timer because there is no accelerometer.
- **Open-loop phases and voltage compensation.**
- `imu_fail_kill` option.
- **`imu_stream` jitter:** bursts, duplicates, out-of-order, 9.9 vs 10.1 ms silence, 12 ms phone gap, seq skip, restart resync, and stream → fs → OPENLOOP.
- **VIOP_T_IMU** encode/decode (n = 3 and n = 5 = 92 B frame).
- **PWM limits:** NaN/negative → 1000 µs, > 1 → 2000 µs, duty at 400 Hz 14 bit, 490 Hz ok / 500 Hz rejected, OneShot125.

Earlier scenarios (all still pass): arm rules, pose / heartbeat / both stalls, heartbeat resume (no auto-resume), seq frozen, LOST / STOPPING / IMU_STALE flags, USB DTR, link loss, low battery, tilt / NaN / switch / command KILL, overruns, VIO reset, hover record and persistence, tier downgrade with no upgrade, land timeout, review regressions.

Results printed by the tests (sim, 1 run each):

- **Phone-IMU loss from 1 m (default mode 0):** contact 1.86 s at 0.80 m/s, cut 3.53 s = **~1.7 s on the ground at ~98 % hover** (this test's record is ~1 % low). Opt-in mode 1, record exact: contact 1.57 s at 1.23 m/s, cut 2.05 s.
- **Sweep (`t_openloop_cut`, height 0.3 / 1 / 2 m × thrust error −3 / 0 / +3 %, 1 run each, deterministic sim):** error relative to the actual record. Mode 1 worst: impact 2.35 m/s, no cut in the air **within the same model** (predictor = test plant), ≤ 0.12 s on the ground above 85 % hover. Mode 0 worst: cut in the air 3/9 (up to 3.37 m), on-ground spin 3.5 s at > 85 % hover. Failure ranges with other plant models: run README. Python model of the same sweep (0.2–3 m, ±1.5 / ±3 %): `scan3d/runs/esp32fix_20261026/olsim3_out.txt`.
- **VIO still alive, IMU frames lost (mode 1):** cut 150 ms after ground contact (VIO touchdown).
- Trade-off (sim): impact speed at 1 m rises from 0.80 to 1.24 m/s (descent 0.6 m/s + end ramp) in exchange for ~1.1 s less spinning on the ground at near-hover thrust.
- **Takeoff record:** 0.7 % below the true hover.

`make -C esp32fc/test syntax`: all `main/*.c` compile `-fsyntax-only -Werror` for PWM / OneShot125 / DShot600 × phone / local IMU. **The stubs are written from memory of the IDF v5.2 API (추정); this is not an IDF build.** A real `idf.py build` (ESP-IDF v5.5.6) was done in fcarm 2026-10-27: all 4 variants build, 0 firmware warnings.

## 5. Timing and latency (ESP32-S3 @ 240 MHz — all 추정, not measured)

**Inner loop per IMU frame** (n = 1 sample):

| step | estimate (µs) |
|---|---|
| CDC rx + CRC + decode + rotate + queue (core 0) | 15–30 |
| notify → ctrl wake | 5–10 |
| Mahony + vz estimator per sample | 5–10 |
| `fs_step` | 3–8 |
| PID + mixer | 2–4 |
| 4× LEDC duty update | 10–20 |
| **total** | **≈ 40–80 µs per 2 ms sample**, i.e. 2–4 % of core 1 |

**Added latency of the inner loop, phone IMU vs a local IMU:**

| stage | estimate |
|---|---|
| phone sensor HAL → app callback | 1–3 ms (unknown; part of the team's 4.5 ms predicted-pose figure) |
| app write → USB full-speed bulk (1 ms frames) | 0.5–2 ms |
| TinyUSB → queue → ctrl | 0.1–0.3 ms |
| control | ~0.1 ms |
| PWM 400 Hz wait | 0–2.5 ms (avg 1.25) |
| **gyro → motor command total** | **≈ 2–8 ms (typ ~4 ms)** |

- A local IMU with DShot would be ≈ 0.1–0.5 ms. ESC / motor response comes on top in both cases.
- Phase lag of 4 ms is 14° at 10 Hz and 29° at 20 Hz. The rate loop therefore has to run at lower bandwidth (~8–12 Hz, 추정) with softer gains and weaker disturbance rejection. Probably acceptable indoors for a 450, but not proven.
- **Jitter is the bigger risk.** Android scheduling stalls of 5–20 ms happen. With the 10 ms max gap, such a stall in flight triggers an OPENLOOP landing. **Measure the real gap distribution** (phone timestamps in the IMU frames make that easy) before flying, then set `FC_IMU_MAX_GAP_US`.

**Pose path** (unchanged): the VIO predicted pose is 4.5 ms p50 / 6.9 ms p95 old (measured by the team) plus ~1–2.5 ms over USB, so about 6–9 ms old at the position loop. `pos_task` extrapolates with p += v·age.

## 6. Assumptions (fc_config.h) and what the user must decide

| item | default | status |
|---|---|---|
| IMU | **phone IMU only**, ~500 Hz, max gap 10 ms | user decision; gap threshold to be measured |
| ESC | **standard PWM 1000–2000 µs, 400 Hz** (50–490), GPIO 4,5,6,7; OneShot125 / DShot optional | user decision; ESC end-point calibration needed (1000 = stop) |
| frame | **quad X 450 mm**, Betaflight order 1 RR(CW) 2 FR(CCW) 3 RL(CCW) 4 FL(CW) | user decision; check motor order / spin |
| height | **VIO only, no ToF** | user decision |
| battery | 4S, ADC1 CH0 (GPIO1), divider 11:1, land 3.45 V/cell, arm ≥ 3.70 | decide |
| kill switch | GPIO15 active low | decide (normally-closed loop recommended) |
| phone mount | `viop_mount_cam_fwd`, phone at CG | decide |
| ESP-NOW | channel 1, accepts any MAC | **pair + encrypt before flying** |
| gains | provisional (450 class) | tune on the vehicle (lower rate-loop bandwidth because of the phone IMU latency) |
| open-loop landing | default (esp32fc): k = 2, 1 m/s², 0.4 m/s, 0.99, margin 1.2. Opt-in `openloop_cut_mode = 1` (+ 0.6 m/s, 0.95): band ±3 %, end 0.70 / 0.4 s, hold 0.2 s, VIO touchdown 0.10 m / 150 ms after a descent, timer waits ≤ 2 s while VIO descends | user decides mode 1; to be tested in sim / real |

## 7. Protocol / integration notes

- `viop_proto.h` is a copy of ovdrone's header (bc65b93 + their uncommitted HEARTBEAT 0x02). **Merge point:** replace it with ovdrone's committed version.
- **New proposed type `VIOP_T_IMU` 0x06** (now in `viop_proto.h`; viop_fc_ext.h merged and deleted by integ-20261025):
  - Layout: u64 t0_us, u8 n (1..5), u8 flags (bias-corrected / uncalibrated), u16 sample_seq0, then n × {u16 dt_us, i16 gyro[3] at 1e-3 rad/s, i16 acc[3] at 5e-3 m/s²}.
  - Axes are phone IMU axes (Android). n = 1 gives a 36 B frame; at 500 Hz that is 18 kB/s, far below USB full speed. **n = 1 is recommended**, because batching adds 2 ms per extra sample.
- **The phone-side IMU sender is NOT implemented** (ovdrone / integration). It should:
  - send one frame per sensor event, written immediately from the sensor callback thread;
  - use `SENSOR_DELAY_FASTEST` with `maxReportLatencyUs = 0` (no Android batching);
  - use CLOCK_BOOTTIME timestamps, the same timebase as POSE;
  - pair gyro and accel by timestamp;
  - subtract the VIO biases if available.
- The other proposed types are unchanged: 0x04 event, 0x05 status, 0x20 command, 0x21 setpoint, 0x2E PC heartbeat.
- Not done yet: PC-side ESP32 relay firmware and PC tool; phone clock sync; IMU source fallback (phone → local backup IMU); event-drop counter in status.

## 8. Review history

An independent read-only reviewer found 14 issues. The reproduced ones are fixed, with regression tests in `t_review_regressions`:

1. future timestamps
2. hover record polluted on the ground
3. NaN vz → KILL
4. re-takeoff after touchdown
5. accel-only validity
6. unknown battery allowed arming
7. land timeout cut in mid-air (now steps down to OPENLOOP)
8. kill-latch deadlock
9. frozen seq allowed arming
10. free-fall clear
11. supervisor kill lost in a full queue

## Build

- PC tests (works now): in WSL, `make -C esp32fc/test` and `make -C esp32fc/test syntax`.
- Firmware (ESP-IDF ≥ 5.2; v5.5.6 installed in WSL `~/esp/esp-idf` since fcarm 2026-10-27, builds with 0 warnings):

  ```
  . $IDF_PATH/export.sh
  cd esp32fc
  idf.py set-target esp32s3
  idf.py build
  idf.py -p <port> flash
  ```

  The USB OTG port is used by the phone link, so flash and use the console through UART0, or through download mode.
- Before the first flight, **without props**:
  1. Calibrate the ESC end points (1000 / 2000 µs).
  2. Check motor order and spin.
  3. Phone IMU axes: tilt the frame and watch roll / pitch in the PC status.
  4. Kill switch → stop.
  5. Unplug the phone → OPENLOOP event.
  6. Stop the PC heartbeat → EMERG_LAND.
  7. Measure the IMU gap distribution.
