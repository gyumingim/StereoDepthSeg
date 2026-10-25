# esp32fc — ESP32-S3 비행 제어 펌웨어 (모터 정지 · 비상착륙 포함)

브랜치 `esp32fc-20261025` (ovopt-20261025 0fcf53a 에서 분기), 2026-10-25. **보드에 올려 보거나 IDF로 빌드해 본 적은 없습니다.**

## 한국어 요약 (결론 먼저)

- **언어: C (ESP-IDF + FreeRTOS).** 사용자가 이렇게 정했습니다. 이유: 타이밍을 정해진 대로 맞출 수 있고, RMT(DShot)·SPI·gptimer 같은 하드웨어를 바로 다룰 수 있으며, 코어를 지정할 수 있습니다.
- **빠른 제어를 위한 설정**
  - 240 MHz, FreeRTOS 1 kHz 틱으로 둡니다.
  - 코어 1: 제어 루프(IMU → 자세 → 안전 → 모터)를 1 kHz로 돌립니다. 2–4 kHz도 가능합니다.
  - 코어 0: Wi-Fi / ESP-NOW / USB / 감시 태스크가 돕니다.
  - IMU data-ready 인터럽트가 제어 태스크를 바로 깨웁니다.
  - 모터는 DShot600(RMT)으로 출력합니다.
  - 루프 안에서는 malloc과 로그를 쓰지 않습니다.
- **모터 KILL (즉시 정지)**
  - 원인: PC 명령, 물리 스위치, 기울기 60° 초과(착륙 중에는 45°), 출력 NaN, 루프 멈춤.
  - **같은 제어 주기 안에** DShot 0을 보내고, KILLED 상태로 고정됩니다.
  - 해제하려면 땅 위에서 수평·정지 상태로 1 s 이상 지난 뒤 CLEAR_KILL 명령을 보내야 합니다.
- **비상착륙**
  - 원인: 폰 pose 끊김·불량, 폰 heartbeat 끊김(250 ms) / seq 멈춤 / LOST·STOPPING, PC 링크 끊김(500 ms), 저전압, 명령, 루프 과부하, IMU 고장.
  - 3단계로 내립니다:
    1. **LAND_VIO**: 시작 지점 x,y를 유지하며 0.3 m/s로 하강합니다.
    2. **LAND_IMU**: 수평을 유지하고 IMU로 추정한 속도로 하강합니다.
    3. **LAND_OPENLOOP**: 이번 비행 이륙 때 기록한 모터별 값 × 0.92에서 천천히 줄이고, 충돌을 감지하거나 6 s가 지나면 차단합니다. **자세 제어가 없는 최후 수단**입니다.
  - 단계는 **내려가기만 하고 올라가지 않습니다.** heartbeat가 돌아와도 비행으로 돌아가지 않으며, 착지 → DISARMED → 다시 ARM만 가능합니다.
- **PC 단위시험: 330개 검사 모두 통과** (gcc, ASan/UBSan). 기록은 `scan3d/runs/esp32fc_20261025/test_safety_run.log`에 있습니다.
- **독립 검토**: 버그 14개 보고. 재현된 위험 항목은 고쳤고 회귀시험도 추가했습니다 (아래 "Review").
- **ESP-IDF는 이 PC에 설치되어 있지 않습니다.** 그래서 `idf.py build`는 해 보지 못했습니다. 대신 IDF API 스텁으로 펌웨어 C 파일의 문법·타입만 검사했고 통과했습니다. 실제 API와 맞는지는 확인하지 못했습니다.
- **사용자가 정할 것**: IMU 칩, ESC 프로토콜, 프레임/모터 배치, 배터리 셀 수, 핀, 폰 장착 방향, 하향 거리센서(ToF) 추가 여부(권장), PC 쪽 ESP32 MAC.

## 1. Language / framework (decision: C on ESP-IDF)

| option | verdict | why |
|---|---|---|
| **C on ESP-IDF (FreeRTOS)** | **chosen** (user decision) | Timing is deterministic. Tasks are pinned to cores. You get direct RMT / SPI / gptimer drivers, ISRs in IRAM, `esp_timer`, and the task WDT. There is no hidden heap use, and it is the vendor-supported path. |
| Arduino-ESP32 | no | Same chip, but the `loop()` and `delay()` model and the libraries allocate memory and log. Core and priority control is coarse, and tuning sdkconfig is awkward. |
| Rust (esp-rs / esp-idf-hal) | no (for now) | Memory-safe and good long-term. But the drivers are less mature (RMT DShot, TinyUSB CDC), the team knows C, and the shared protocol header is C. |
| MicroPython | no | It is interpreted and has a GC that pauses (ms). A 1 kHz control loop is not possible. |

How the firmware gets fast, deterministic control:

- **Core split.**
  - Core 1: `ctrl` task at priority 24 runs IMU → attitude → safety → rate PID → mixer → DShot. `pos` task at priority 22 runs the position loop.
  - Core 0: Wi-Fi / ESP-NOW (IDF), TinyUSB, `link` (prio 10), `supervisor` (prio 24, above Wi-Fi 23).
- **Loop clock.** The ICM-42688 data-ready pin (INT1) fires an IRAM ISR, which calls `vTaskNotifyGiveFromISR` to wake `ctrl`. `FC_CLOCK_IMU_DRDY=0` uses a gptimer hardware alarm instead. `esp_timer` callbacks are not used for the loop (dispatch jitter).
- **IMU read.** One 15-byte polling SPI transaction at 20 MHz. For transfers this small, polling beats queued DMA. Use the FIFO later for 4–8 kHz.
- **Motors.** DShot600 via RMT. The ESP32-S3 has exactly 4 RMT TX channels, which covers a quad. Each frame takes 26.7 µs and is sent asynchronously. PWM400 (LEDC) is the fallback.
- **In the loop:** no `malloc`, no `ESP_LOG`, no blocking. Queues use timeout 0, shared structs are copied under a spinlock, and `fc_safety` has a single owner.
- **sdkconfig.defaults:**
  - 240 MHz, PM / DFS off, `-O2`, FreeRTOS 1 kHz.
  - The Wi-Fi, esp_timer and TinyUSB tasks are pinned to core 0.
  - SPI / RMT / GPTimer / GPIO ISRs are IRAM-safe.
  - Task WDT 1 s with panic → reboot (no DShot frames, so the ESCs stop). Interrupt WDT is on.
  - Log level is WARN.
- **No flash writes in flight.** An NVS write stalls both cores' caches for ms. The hover record goes to RTC RAM in flight, and to NVS only after disarm.

## 2. Layout

```
esp32fc/
  CMakeLists.txt, sdkconfig.defaults
  main/            ESP-IDF app (NOT compiled with IDF yet)
    fc_config.h      all hardware assumptions (pins, IMU, ESC, rates, cores, battery, phone mount)
    main.c           init order (motors first = DShot 0), queues, task creation, gptimer option
    ctrl_task.c      1 kHz loop: IMU -> Mahony -> drain inbox -> fs_step -> control per mode -> fs_check_outputs -> DShot
    pos_task.c       position loop (woken per phone pose, <= 200 Hz), latency compensation p += v*age, land-in-place xy hold
    link_task.c      USB CDC (TinyUSB; phone = USB host) VIOP parser + ESP-NOW (PC): events, 20 Hz status, commands, setpoints
    supervisor_task.c fast SW watchdog (loop silent 20 ms -> motor_out_emergency_stop), 2nd kill-switch path, battery ADC 50 Hz
    motor_out.c/.h   DShot600/300 (RMT bytes encoder) or PWM400 (LEDC); e-stop callable from any core
    imu_icm42688.c   ICM-42688-P SPI driver (2000 dps / 16 g / ODR 1 kHz, DRDY INT1, stuck / saturation checks)
    board.c/.h       battery ADC, kill switch ISR, LED, hover record in RTC no-init RAM + NVS
  components/
    fc_safety/       PURE C safety state machine (unit-tested on PC)   <- the core of this task
    fc_core/         PURE C Mahony attitude, IMU vertical-speed estimator, PID, attitude/position control, quad-X mixer
    viop/            viop_proto.h (copy from ovdrone) + viop_fc_ext.h (proposed FC message types)
  test/              PC tests: `make run` (unit tests), `make syntax` (firmware -fsyntax-only against IDF stubs)
```

## 3. Safety state machine (components/fc_safety)

States: `DISARMED, ARMED_IDLE, FLYING, EMERG_LAND, KILLED`. Allowed transitions (anything else is refused and logged as `FS_EV_ILLEGAL`; tests assert none happen):

| from \ to | DISARMED | ARMED_IDLE | FLYING | EMERG_LAND | KILLED |
|---|---|---|---|---|---|
| DISARMED | | ARM cmd + all arm checks | | | kill cmd / switch |
| ARMED_IDLE | DISARM cmd, idle 5 s, any land trigger on the ground | | collective > 0.15 (after it was seen low) | | any kill |
| FLYING | | normal touchdown detected | | any land trigger | any kill |
| EMERG_LAND | touchdown / impact / open-loop timeout | | **never (no auto-resume)** | | any kill |
| KILLED | CLEAR_KILL on ground (level, still, ~1 g, switch released, ≥ 1 s) | | | | |

**KILL** happens immediately: `motors_enabled=false`, and the control task writes DShot 0 in the same cycle. It latches.

| trigger | default |
|---|---|
| PC `VIOP_CMD_KILL` | — |
| physical switch | GPIO15 to GND, ISR latch + level read, plus a supervisor path |
| tilt | > 60° (> 45° during EMERG_LAND) |
| gyro rate | > 1500 °/s |
| motor output NaN / out of range | — |
| control loop stall | 20 ms (supervisor cuts the pins directly) |
| IMU failure | only if `imu_fail_kill=true`; the default goes to LAND_OPENLOOP, per the user's tier spec |

**EMERGENCY LAND triggers (defaults):**

| trigger | default |
|---|---|
| no GOOD pose | 200 ms (good = VALID+TRACKING, quality>0, std ≤ 0.15 m, send_delay ≤ 50 ms, seq increasing) |
| pose lost flag (VALID=0 / quality 0) | immediate |
| VIO reset_counter change in flight | immediate |
| phone heartbeat missing | 250 ms (5 periods at 20 Hz) |
| heartbeat seq not increasing | 250 ms, reason HB_SEQ |
| heartbeat state STARTING / LOST / STOPPING or flags STOPPING / IMU_STALE / POSE_STALE | immediate |
| USB port closed (DTR 1→0) | immediate |
| PC ESP-NOW link silent | 500 ms |
| battery < 3.45 V/cell | debounced 2 s, latched |
| loop overruns | leaky bucket: +10 per overrun, −1 per good cycle, > 500 means e.g. 50 consecutive overruns |
| position task dead | setpoint older than 50 ms |
| LAND command | — |

**Tiers.** The tier is chosen at the trigger and re-evaluated every cycle. It can only step down, and every change is logged as `FS_EV_TIER` with a reason.

| tier | condition | control | end |
|---|---|---|---|
| LAND_VIO | pose fresh + heartbeat not LOST + IMU ok | pos loop holds land-start x,y; collective PI on VIO vz → −0.30 m/s | touchdown (300 ms of: accel variance < 0.5, \|f\|≈g, collective ≤ 0.75·hover or impact, \|vz\| < 0.2) → DISARMED |
| LAND_IMU | no usable VIO, IMU ok | level attitude, yaw hold; vz from IMU estimator (bias learnt during VIO, leak 5 s, valid 8 s), else a timed collective ramp | same touchdown test (impact spike also counts) |
| LAND_OPENLOOP | IMU unusable, or land timeout 30 s in VIO/IMU | **no attitude control**: per-motor = hover record × (0.92 − 0.05/s, min 0.60) | impact (\|f\| > 2 g, accel alone) or 6 s → DISARMED |

**Hover record for tier 3.** This is per flight and reset at every ARM.

- Recording starts once lift-off is proven (vz > 0.10 m/s from VIO, else from the IMU estimate), plus 300 ms.
- It averages per-motor commands while tilt < 10°, rate < 30 °/s, filtered \|f\| within 0.4 m/s² of g, \|vz\| < 1 m/s and nothing saturated. It becomes valid after 0.5 s and freezes after 3 s.
- Sources, in order: this flight → previous flight (RTC RAM, else NVS, loaded at boot and range-checked) → `openloop_default_thr` 0.33.
- The tier event's `aux` says which source was used.
- In the sim, the record came out 1.4 % below true hover (the climb-to-hover deceleration enters the window).

**Arming.** All of these must hold (each failure is a bit in `FS_EV_ARM_REJECT`):

- level ≤ 10°
- still: gyro < 0.2 rad/s, accel variance low, \|f\|≈g
- collective request ≤ 0.05
- IMU ok
- pose fresh, heartbeat fresh (not STARTING / LOST), USB not closed, PC link fresh (each configurable)
- battery known and ≥ 3.70 V/cell
- overrun bucket low
- kill switch released

Auto-disarm after 5 s idle on the ground. **Every transition, tier change and rejection is an event**: `fs_event_t` → `viop_fc_event_t` (VIOP type 0x04) → ESP-NOW → PC. A 20 Hz status frame (0x05) carries state, tier, reasons, battery, attitude, loop max µs and overruns.

## 4. PC unit tests (all pass)

`make -C esp32fc/test` builds with gcc 11.4 in WSL, using `-Wall -Wextra -Werror -fsanitize=address,undefined`. Result: **330 checks, 0 failed**. The raw log is in `scan3d/runs/esp32fc_20261025/test_safety_run.log`.

The tests run a 1 kHz closed-loop 1-D vertical simulation: thrust, gravity, ground contact with an impact spike, and per-motor trims. It generates phone pose (200 Hz), heartbeat (20 Hz) and PC heartbeat (10 Hz).

Scenarios:

- **Transitions and arming:** transition table; arm rules (no input / no phone / VIO STARTING / tilt / low or unknown battery / kill switch / throttle / NaN / frozen seq).
- **Phone and link loss:**
  - pose stall with heartbeat alive → IMU tier → touchdown
  - heartbeat stall with pose alive → VIO tier → touchdown
  - both stalled
  - heartbeat returns during landing → stays in EMERG_LAND → DISARMED → re-ARM needed
  - heartbeat seq frozen; heartbeat LOST / STOPPING / IMU_STALE; USB DTR drop
  - link loss
- **Battery and loop:** low battery (and a short sag that must not trigger); loop overrun (sustained vs 5 %).
- **KILL:**
  - tilt kill (same cycle, latched, clear rules)
  - tilt > 45° during land → KILL
  - KILL command mid-flight and during land; kill switch; NaN output; loop-stall request
  - DISARM refused in flight
- **VIO reset in flight.**
- **Hover record:** values, freeze, reset at arm, previous-flight fallback, NaN-corrupted persisted value → default.
- **Tiers:** IMU fail → OPENLOOP (this flight's values, cut by impact); `imu_fail_kill` option; downgrade VIO → IMU → OPENLOOP with no upgrade; IMU tier without vz (ramp); land timeout → OPENLOOP.
- **Normal landing:** normal touchdown → idle disarm; no re-takeoff after touchdown; a land trigger on the ground disarms.
- **Review regressions:** future-stamped messages; ground spin-up not recorded; NaN vz during land does not KILL; CLEAR_KILL refused in free fall.
- **fc_core known-answer tests:** Mahony converges to a 20° / −10° static tilt, with gyro bias learnt in the observable plane; vz estimator learns accel bias and expires its horizon; mixer signs and saturation; VIOP glue.

Results printed by the tests (sim, 1 run each):

| scenario | result |
|---|---|
| pose stall (IMU tier) | touchdown after 3.5 s |
| heartbeat stall (VIO tier, 1 m at 0.3 m/s) | touchdown after 3.8 s |
| open-loop | cut by impact after 1.3 s |

`make -C esp32fc/test syntax` compiles every `main/*.c` with `-fsyntax-only -Werror` against `test/host_stubs`, for DShot and PWM and for DRDY and gptimer. Result: OK. **The stubs are written from memory of the IDF v5.2 API (추정). This is not an IDF build.**

## 5. Timing (ESP32-S3 @ 240 MHz — all 추정, not measured)

Control loop per cycle:

| step | estimate (µs) |
|---|---|
| ISR → task wake | 3–8 |
| SPI 15 B @ 20 MHz polling | 15–25 |
| Mahony + Euler + vz estimator (single-precision FPU) | 5–10 |
| queue drain | 2–10 |
| `fs_step` | 3–8 |
| attitude PID + mixer | 2–4 |
| spinlock copies | 1–3 |
| 4× `rmt_transmit` | 20–40 |
| **total** | **≈ 55–110 µs typical, ≈ 150 µs worst** (cache misses; flash writes are excluded by design) |

- At 1 kHz that is about 6–11 % of core 1. 2 kHz is comfortable. 4 kHz (250 µs) fits but is tight: use the IMU FIFO there. DShot600 (26.7 µs frame) is fine at 4 kHz; DShot300 (53 µs) still fits but with less margin.
- Overrun threshold = 80 % of the period.
- pos task: about 20–40 µs per run at ≤ 200 Hz.
- link: VIOP CRC + parse of a 92-byte frame is about 10–20 µs; an ESP-NOW send is about 50–100 µs, on core 0.

Phone → ESP32 pose latency (USB CDC, full-speed 12 Mbit/s, 1 ms frames):

| stage | estimate |
|---|---|
| bridge `write()` → Android usbfs bulk | 0.3–1 ms |
| USB bulk scheduling (1 ms frames) | 0–1 ms |
| TinyUSB task + parse + queue | 0.1–0.3 ms |
| **USB hop total** | **≈ 1–2.5 ms typical, ≈ 5 ms bad case** |

- The VIO predicted pose is already 4.5 ms p50 / 6.9 ms p95 old when sent (team measurement, includes IMU delivery). The pose is therefore about 6–9 ms old when the position loop uses it (추정).
- For comparison, the raw VIO pose is 54–62 ms old.
- `pos_task` compensates with p += v·age, where age = send_delay_us + local age. The USB transit (~1–2 ms) is not counted because there is no clock sync yet. At 1 m/s that leaves about 1–2 mm error.
- The attitude loop uses only the ESP32's own IMU, so phone latency affects only the position loop (bandwidth ~1–2 Hz; 8 ms ≈ 6° phase at 2 Hz).

## 6. Assumptions (fc_config.h) and what the user must decide

| item | assumed default | decide |
|---|---|---|
| IMU | TDK ICM-42688-P, SPI2 (SCLK 12, MOSI 11, MISO 13, CS 10, INT1 14), 2000 dps / 16 g, ODR 1 kHz, chip x fwd / z up | chip + mounting; verify the 3 static sign checks (imu_icm42688.c) before flight |
| ESC | DShot600 via RMT, motors GPIO 4,5,6,7 | DShot600/300 or PWM; ESC firmware (BLHeli_32 / AM32 / BLHeli_S) |
| frame | quad X, Betaflight order 1 RR(CW) 2 FR(CCW) 3 RL(CCW) 4 FL(CW), idle 0.05 | motor order / spin |
| battery | 4S, ADC1 CH0 (GPIO1), divider 11:1, land 3.45 V/cell, arm ≥ 3.70 | cells, divider, thresholds |
| kill switch | GPIO15 active low | wiring (NC loop recommended so that a broken wire = kill: TODO option) |
| phone mount | `viop_mount_cam_fwd`, phone at CG (lever arm 0) | real mounting |
| ESP-NOW | channel 1, PC MAC = broadcast (accept any) | **pair the PC-side ESP32 MAC and enable encryption before flying** (otherwise anyone can send ARM/KILL) |
| gains | placeholder (450 class guess) | tuning on the vehicle |
| sensors | no barometer / rangefinder | **a downward ToF (e.g. VL53L1X) is recommended**: LAND_IMU / LAND_OPENLOOP descend blind |

## 7. Protocol / integration notes

- `components/viop/include/viop_proto.h` is a copy of ovdrone's `scan3d/ov_bridge/viop_proto.h`: commit bc65b93 plus **ovdrone's uncommitted working-tree change (~16:50) adding `VIOP_T_HEARTBEAT 0x02` (18 B)**. ovdrone owns that header and the phone-side heartbeat sender (`--viop-hb-hz`, default 20 Hz, sent from the bridge main loop). **Merge point:** when ovdrone commits, replace this copy with theirs and diff it.
- `viop_fc_ext.h` holds the proposed FC types (to be moved into `viop_proto.h` by integration):

  | id | type | direction |
  |---|---|---|
  | 0x04 | FC_EVENT | drone → PC |
  | 0x05 | FC_STATUS | drone → PC |
  | 0x20 | CMD (ARM / DISARM / KILL / LAND / CLEAR_KILL with confirm word) | PC → drone |
  | 0x21 | SETPOINT | PC → drone |
  | 0x2E | PC_HB | PC → drone, 10 Hz |

  0x03 stays reserved for ovdrone's health / battery telemetry.
- Phone side: the app must open the ESP32's USB CDC-ACM port (the ESP32 is the USB device; the phone is the host via OTG) and write the VIOP frames, pose and heartbeat. If the app sets DTR, closing the port lands the drone immediately; otherwise the heartbeat timeout covers it.
- Not done yet: PC-side ESP32 relay firmware; a PC tool to send commands and show events; phone clock sync (TIME_REQ / RESP) for exact latency; NC kill-switch option; event-drop counter in status.

## 8. Review (independent read-only agent, 2026-10-25)

The reviewer reproduced these bugs in its own tests in /tmp. All are fixed and have regression tests in `t_review_regressions`:

1. A message stamped after the loop's `now` looked 49 days old, causing a false emergency land and a CLEAR_KILL bypass. Now `elapsed` saturates and the firmware stamps messages with the loop time.
2. The hover record was polluted by spinning up on the ground. Recording now starts only after lift-off is proven.
3. A NaN in vz during landing caused a KILL. Inputs are now sanitised.
4. The drone re-took off right after touchdown. Takeoff now needs the collective request to be seen low first.
5. In OPENLOOP, impact detection was dead because `acc_ok == imu_ok`. Accelerometer validity is now separate.
6. Unknown or NaN battery allowed arming, and a stale battery value stayed "good". The battery must now be known to arm, and the supervisor resets it after 5 failed ADC reads.
7. Land timeout cut the motors in mid-air in the VIO/IMU tiers. It now steps down to OPENLOOP, which ends with impact or a 6 s cut.
8. The kill-switch ISR latch caused a deadlock with CLEAR_KILL. The latch is now dropped once fs is KILLED and the pin is released.
9. A frozen seq still allowed arming. A repeated seq never refreshes now.
10. Clear / arm were possible in free fall. The ground check now requires \|f\|≈g.
11. A supervisor kill could be lost in a full queue. It now goes through a flag.

The DTR handling (12) was changed. Accepted as is: one IMU dropout keeps tier 3 (that is the spec), and the RMT outputs are not restored after a pin takeover (fails safe; needs a reboot).

## Build

- PC tests (works now): in WSL, `make -C esp32fc/test` and `make -C esp32fc/test syntax`.
- Firmware (needs ESP-IDF ≥ 5.2, **not installed on this PC**):

  ```
  . $IDF_PATH/export.sh
  cd esp32fc
  idf.py set-target esp32s3
  idf.py build
  idf.py -p <port> flash
  ```

  `esp_tinyusb` comes from the component manager. The USB OTG port is used by TinyUSB for the phone, so flash and use the console through UART0, or through download mode.
- Before the first flight, **without props**:
  1. Run the IMU sign checks.
  2. Check motor order and direction with low DShot.
  3. Kill switch → motors stop.
  4. Unplug the phone → EMERG_LAND event on the PC.
  5. Stop the PC heartbeat → EMERG_LAND.
  6. Command KILL → stop.
