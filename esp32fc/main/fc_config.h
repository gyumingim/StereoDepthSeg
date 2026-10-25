/* fc_config.h — compile-time configuration of the esp32fc flight firmware.
 * User decisions 2026-10-25: phone IMU is the ONLY IMU; ordinary PWM ESCs; quad X 450 mm; no ToF (VIO height).
 * Everything else here is an ASSUMPTION — see README "사용자가 정할 것". Change here, rebuild (most can also be
 * overridden with -D). Safety thresholds live in fs_config_default() (components/fc_safety), overridden in
 * fc_safety_config() (ctrl_task.c). */
#ifndef FC_CONFIG_H
#define FC_CONFIG_H

/* ---------------- IMU source ---------------- */
#define FC_IMU_PHONE 0            /* default: phone IMU samples over USB CDC (VIOP_T_IMU), event-driven inner loop */
#define FC_IMU_LOCAL_ICM42688 1   /* optional module: ICM-42688-P on the ESP32's SPI (imu_icm42688.c) */
#ifndef FC_IMU_SOURCE
#define FC_IMU_SOURCE FC_IMU_PHONE
#endif
#define FC_IMU_NOMINAL_HZ 500     /* phone accelerometer/gyro full rate (S24 ~500 Hz, 추정) */
#define FC_IMU_MAX_GAP_US 10000   /* no sample for > 10 ms (local) or a phone-time gap > 10 ms -> IMU failure (user) */
#define FC_CTRL_TIMEOUT_MS 2      /* control task runs at least every 2 ms even without samples (failsafe keeps going) */

/* ---------------- rates ---------------- */
#define FC_POS_HZ 200             /* position loop upper rate (also woken by every new phone pose) */
#define FC_STATUS_HZ 20           /* telemetry to the PC over ESP-NOW */
#define FC_SUPERVISOR_HZ 500      /* fast software watchdog of the control loop */
#define FC_CTRL_STALL_MS 20       /* control loop silent this long -> supervisor cuts motors directly */
#define FC_CTRL_OVERRUN_US (1000000 / FC_IMU_NOMINAL_HZ * 8 / 10) /* loop body > 80 % of a sample period = overrun */

/* ---------------- cores / priorities ----------------
 * core 1: control (highest), position. core 0: Wi-Fi / ESP-NOW (IDF), TinyUSB, link, supervisor. */
#define FC_CORE_CTRL 1
#define FC_CORE_LINK 0
#define FC_PRIO_CTRL (configMAX_PRIORITIES - 1)
#define FC_PRIO_POS (configMAX_PRIORITIES - 3)
#define FC_PRIO_SUPERVISOR (configMAX_PRIORITIES - 1) /* core 0, above Wi-Fi (23) so it can always cut motors */
#define FC_PRIO_LINK 10

/* ---------------- optional local IMU (FC_IMU_SOURCE == FC_IMU_LOCAL_ICM42688) ---------------- */
#define FC_IMU_SPI_HOST SPI2_HOST
#define FC_PIN_IMU_SCLK 12
#define FC_PIN_IMU_MOSI 11
#define FC_PIN_IMU_MISO 13
#define FC_PIN_IMU_CS 10
#define FC_PIN_IMU_INT1 14
#define FC_IMU_SPI_HZ (20 * 1000 * 1000) /* ICM-42688 max 24 MHz */
#define FC_IMU_ODR_HZ 1000
/* v_body(FRD) = R_BS * v_chip. Default diag(1,-1,-1): chip x forward, y left, z up. Verify the 3 static checks. */
#define FC_IMU_ROT {1, 0, 0, 0, -1, 0, 0, 0, -1}

/* ---------------- ESCs / motors ---------------- */
#define FC_ESC_PWM 0              /* default: standard PWM FC_PWM_MIN_US..FC_PWM_MAX_US at FC_PWM_HZ (LEDC) */
#define FC_ESC_ONESHOT125 1       /* optional: 125..250 us pulses at FC_ONESHOT_HZ (LEDC) */
#define FC_ESC_DSHOT600 2         /* optional: digital, RMT */
#define FC_ESC_DSHOT300 3
#ifndef FC_ESC_PROTOCOL
#define FC_ESC_PROTOCOL FC_ESC_PWM
#endif
#ifndef FC_PWM_HZ
#define FC_PWM_HZ 400             /* 50..490 Hz (490 max for 2000 us pulses); ESC must accept it */
#endif
#define FC_PWM_MIN_US 1000.0f     /* motor stop / disarmed (calibrate the ESC end points to 1000 / 2000) */
#define FC_PWM_MAX_US 2000.0f
#define FC_ONESHOT_HZ 2000
#define FC_ONESHOT_MIN_US 125.0f
#define FC_ONESHOT_MAX_US 250.0f
#if FC_ESC_PROTOCOL == FC_ESC_PWM && (FC_PWM_HZ < 50 || FC_PWM_HZ > 490)
#error "FC_PWM_HZ must be 50..490"
#endif
#define FC_N_MOTORS 4
#define FC_PIN_MOTOR {4, 5, 6, 7} /* motor 1 RR, 2 FR, 3 RL, 4 FL (Betaflight order) */
#define FC_MOTOR_IDLE 0.05f       /* armed idle spin (fraction of full throttle) */

/* ---------------- frame: quad X, 450 mm (motor-to-motor diagonal) ----------------
 * Motor positions in body FRD (m): 0.225 m arm at 45 deg -> +-0.159 m. 1 RR (-x,+y) CW, 2 FR (+x,+y) CCW,
 * 3 RL (-x,-y) CCW, 4 FL (+x,-y) CW (props-in, Betaflight default). The mixer (fc_core) is the symmetric quad-X table;
 * geometry is recorded here for tuning / future torque scaling. Gains in fc_core.c are PROVISIONAL. */
#define FC_FRAME_DIAG_M 0.450f
#define FC_FRAME_ARM_M (FC_FRAME_DIAG_M / 2.0f)
#define FC_FRAME_MOTOR_XY_M 0.159f

/* ---------------- battery / switches ---------------- */
#define FC_BATT_ADC_CHANNEL ADC_CHANNEL_0 /* ADC1 CH0 = GPIO1 */
#define FC_BATT_DIVIDER 11.0f     /* e.g. 100k / 10k divider: 16.8 V -> 1.53 V */
#define FC_BATT_CELLS 4
#define FC_PIN_KILL_SWITCH 15     /* physical kill switch, to GND when active (internal pull-up) */
#define FC_KILL_SWITCH_ACTIVE_LEVEL 0
#define FC_PIN_LED 48

/* ---------------- phone mount (phone IMU -> drone body FRD), see viop_proto.h ---------------- */
#define FC_PHONE_MOUNT viop_mount_cam_fwd
#define FC_PHONE_LEVER_ARM {0.0f, 0.0f, 0.0f} /* phone IMU -> vehicle CG in phone axes, m (phone at CG: zero) */

/* ---------------- ESP-NOW ---------------- */
#define FC_ESPNOW_CHANNEL 1
#define FC_PC_PEER_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} /* PC-side ESP32 MAC; broadcast until paired */
#define FC_ESPNOW_PMK "esp32fc_pmk_0001"                     /* 16 bytes; set the same on the PC-side ESP32 */

/* ---------------- hover / controller defaults ---------------- */
#define FC_HOVER_THR_DEFAULT 0.40f /* collective guess for a 450 class quad (추정) */

#endif /* FC_CONFIG_H */
