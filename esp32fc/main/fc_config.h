/* fc_config.h — compile-time configuration of the esp32fc flight firmware.
 * EVERY hardware value here is an ASSUMPTION (not decided by the user yet) — see README "사용자가 정할 것".
 * Change here, rebuild. Safety thresholds live in fs_config_default() (components/fc_safety) and are overridden in
 * fc_safety_config() below. */
#ifndef FC_CONFIG_H
#define FC_CONFIG_H

/* ---------------- rates ---------------- */
#define FC_CTRL_HZ 1000          /* IMU + attitude + rate loop (1000 / 2000 / 4000 possible, see README timing) */
#define FC_POS_HZ 200            /* position loop upper rate (also woken by every new phone pose) */
#define FC_STATUS_HZ 20          /* telemetry to the PC over ESP-NOW */
#define FC_SUPERVISOR_HZ 500     /* fast software watchdog of the control loop */
#define FC_CTRL_STALL_MS 20      /* control loop silent this long -> supervisor cuts motors directly */
#define FC_CTRL_OVERRUN_US (1000000 / FC_CTRL_HZ * 8 / 10) /* loop body longer than 80 % of the period = overrun */

/* ---------------- cores / priorities ----------------
 * core 1: control (highest), position. core 0: Wi-Fi / ESP-NOW (IDF), link (USB CDC + ESP-NOW), supervisor. */
#define FC_CORE_CTRL 1
#define FC_CORE_LINK 0
#define FC_PRIO_CTRL (configMAX_PRIORITIES - 1)
#define FC_PRIO_POS (configMAX_PRIORITIES - 3)
#define FC_PRIO_SUPERVISOR (configMAX_PRIORITIES - 1) /* core 0, above Wi-Fi (23) so it can always cut motors */
#define FC_PRIO_LINK 10

/* ---------------- loop clock source ---------------- */
#ifndef FC_CLOCK_IMU_DRDY
#define FC_CLOCK_IMU_DRDY 1      /* 1: IMU data-ready pin interrupt wakes the loop (lowest latency) */
                                 /* 0: gptimer hardware alarm at FC_CTRL_HZ (IMU polled) */
#endif

/* ---------------- IMU (assumed TDK ICM-42688-P on SPI) ---------------- */
#define FC_IMU_SPI_HOST SPI2_HOST
#define FC_PIN_IMU_SCLK 12
#define FC_PIN_IMU_MOSI 11
#define FC_PIN_IMU_MISO 13
#define FC_PIN_IMU_CS 10
#define FC_PIN_IMU_INT1 14
#define FC_IMU_SPI_HZ (20 * 1000 * 1000) /* ICM-42688 max 24 MHz */
#define FC_IMU_ODR_HZ 1000      /* must be >= FC_CTRL_HZ when FC_CLOCK_IMU_DRDY */
/* IMU mounting: v_body(FRD) = R_BS * v_chip. Default diag(1,-1,-1): chip x forward, chip y left, chip z up
 * (board component side up). Verify with the 3 static checks in imu_icm42688.c before flying. */
#define FC_IMU_ROT {1, 0, 0, 0, -1, 0, 0, 0, -1}

/* ---------------- ESCs / motors ---------------- */
#define FC_ESC_DSHOT600 0
#define FC_ESC_DSHOT300 1
#define FC_ESC_PWM400 2         /* standard PWM 1000-2000 us at 400 Hz (LEDC), for non-DShot ESCs */
#ifndef FC_ESC_PROTOCOL
#define FC_ESC_PROTOCOL FC_ESC_DSHOT600
#endif
#define FC_N_MOTORS 4           /* quad X; the ESP32-S3 RMT has exactly 4 TX channels */
#define FC_PIN_MOTOR {4, 5, 6, 7} /* motor 1 RR, 2 FR, 3 RL, 4 FL (Betaflight order) */
#define FC_MOTOR_IDLE 0.05f     /* armed idle spin (fraction of full throttle) */

/* ---------------- battery / switches ---------------- */
#define FC_BATT_ADC_CHANNEL ADC_CHANNEL_0 /* ADC1 CH0 = GPIO1 */
#define FC_BATT_DIVIDER 11.0f   /* e.g. 100k / 10k divider: 16.8 V -> 1.53 V */
#define FC_BATT_CELLS 4
#define FC_PIN_KILL_SWITCH 15   /* physical kill switch, to GND when active (internal pull-up) */
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
