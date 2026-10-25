/* viop_fc_ext.h — PROPOSED additions to viop_proto.h (VIOP v2) for the flight-controller link (esp32fc, 2026-10-25).
 *
 * MERGE POINT: viop_proto.h here is a verbatim copy of ovdrone-20261025 scan3d/ov_bridge/viop_proto.h (commit bc65b93
 * + ovdrone's UNCOMMITTED working-tree change of 2026-10-25 ~16:50 that adds VIOP_T_HEARTBEAT 0x02, 18 B). The phone
 * heartbeat is defined THERE (ovdrone owns it and implements the sender in ov_bridge, --viop-hb-hz, default 20 Hz).
 * The message types below use ids that file marks "reserved" (0x03-0x0F telemetry, 0x20-0x2F commands). When the
 * integration agent merges, move these definitions into viop_proto.h (same ids, same layouts) and delete this file.
 * Framing, CRC, seq rules: unchanged (see viop_proto.h). All little-endian, packed.
 *
 *   id    name                 dir                     payload  frame  rate
 *   0x06  VIOP_T_IMU           phone -> drone (USB)  12+14n B  36..92 B  RAW PHONE IMU at full rate (~500 Hz), n = 1..5
 *                                                                          samples per frame (n = 1 recommended: lowest
 *                                                                          latency; 36 B x 500 Hz = 18 kB/s << USB FS)
 *   0x04  VIOP_T_FC_EVENT      drone -> PC (ESP-NOW)    16 B    26 B   on every safety event (state / tier change...)
 *   0x05  VIOP_T_FC_STATUS     drone -> PC (ESP-NOW)    40 B    50 B   20 Hz
 *   0x20  VIOP_T_CMD           PC -> drone (ESP-NOW)     4 B    14 B   on demand (ARM, DISARM, KILL, LAND, CLEAR_KILL)
 *   0x21  VIOP_T_SETPOINT      PC -> drone (ESP-NOW)    16 B    26 B   on demand (local NED position + yaw)
 *   0x2E  VIOP_T_PC_HB         PC -> drone (ESP-NOW)     8 B    18 B   10 Hz (link-alive; missing -> EMERG_LAND)
 *
 * PHONE HEARTBEAT: see viop_proto.h (0x02). The drone lands (EMERG_LAND) if no valid heartbeat for hb_timeout_ms
 * (default 250 ms = 5 periods at 20 Hz), if its seq stops increasing, or if state is STARTING / LOST / STOPPING or a
 * flag STOPPING / IMU_STALE / POSE_STALE is set while flying. It does NOT resume flight when heartbeats come back.
 *
 * PHONE IMU STREAM (0x06, user decision 2026-10-25: the phone IMU is the ONLY IMU; the ESP32 attitude/rate loop runs on
 * it). The phone-side SENDER IS NOT IMPLEMENTED YET (ovdrone / integration): in the app / bridge, on every
 * accelerometer+gyro event (SENSOR_DELAY_FASTEST, maxReportLatencyUs = 0, i.e. no Android batching), write one frame
 * immediately to the USB CDC port, from the sensor callback thread (not via the VIO thread). Use the same CLOCK_BOOTTIME
 * timebase as POSE. Pair gyro and accel by nearest timestamp (gyro timestamp is the sample time). If the bridge has
 * bias estimates (VIO filter), subtract them and set VIOP_IMUF_BIAS_CORR. Frame seq +1 per frame; sample_seq0 counts
 * samples (drops visible even when frames are batched).
 * IMU PAYLOAD (12 + 14 n bytes):
 *   0  u64 t0_us        phone sensor time of sample 0 (CLOCK_BOOTTIME / 1000)
 *   8  u8  n            samples in this frame, 1..5
 *   9  u8  flags        VIOP_IMUF_*
 *  10  u16 sample_seq0  running sample counter of sample 0
 *  12  n x { u16 dt_us (offset from t0, sample 0 = 0); i16 gyro[3] (1e-3 rad/s, +-32.7 rad/s);
 *            i16 acc[3] (5e-3 m/s^2, +-163 m/s^2 = 16.7 g) }  — PHONE IMU axes I (Android sensor axes), specific force
 *            (at rest the axis pointing up reads +9.81), gyro right-handed rad/s (Android convention).
 */
#ifndef VIOP_FC_EXT_H
#define VIOP_FC_EXT_H

#include <stdint.h>
#include "viop_proto.h"

#define VIOP_T_IMU 0x06
#define VIOP_IMU_MAX_N 5
#define VIOP_IMUF_BIAS_CORR 0x01u /* bias-corrected by the phone */
#define VIOP_IMUF_UNCAL 0x02u     /* from *_UNCALIBRATED sensors */
#define VIOP_IMU_GYRO_LSB 1e-3f
#define VIOP_IMU_ACC_LSB 5e-3f
#define VIOP_T_FC_EVENT 0x04
#define VIOP_T_FC_STATUS 0x05
#define VIOP_T_CMD 0x20
#define VIOP_T_SETPOINT 0x21
#define VIOP_T_PC_HB 0x2E

/* commands (VIOP_T_CMD.cmd); confirm must equal VIOP_CMD_CONFIRM(cmd) (guards against a mis-parsed frame) */
#define VIOP_CMD_ARM 1
#define VIOP_CMD_DISARM 2
#define VIOP_CMD_KILL 3
#define VIOP_CMD_LAND 4
#define VIOP_CMD_CLEAR_KILL 5
#define VIOP_CMD_CONFIRM(c) ((uint16_t)(0xC0DEu ^ (uint16_t)(c)))

#if defined(_MSC_VER)
#pragma pack(push, 1)
#endif
typedef struct VIOP_PACKED {
  uint32_t t_ms;            /* drone ms since boot */
  uint16_t ev_seq;          /* +1 per event (gaps = lost events on the radio) */
  uint8_t code;             /* FS_EV_* (fc_safety.h) */
  uint8_t from;             /* state or tier before */
  uint8_t to;               /* state or tier after */
  uint8_t aux;              /* event specific (e.g. hover source for tier 3) */
  uint16_t pad;
  uint32_t reason;          /* FS_R_* / FS_B_* bits */
} viop_fc_event_t;

typedef struct VIOP_PACKED {
  uint32_t t_ms;
  uint8_t state;            /* fs_state_t */
  uint8_t land_tier;        /* fs_tier_t (0 when not landing) */
  uint16_t batt_mv;
  uint32_t land_reason;     /* FS_R_* bits of the active emergency land / kill */
  int16_t roll_cdeg, pitch_cdeg, yaw_cdeg;
  int16_t thr_permille;     /* collective 0..1000 */
  float p_l[3];             /* local NED position used by the controller, m */
  uint16_t loop_max_us;     /* worst control-loop time in the last status period */
  uint16_t overruns;        /* control-loop overruns since boot (saturated) */
  uint16_t pose_age_ms;     /* since last good pose */
  uint16_t hb_age_ms;       /* since last good phone heartbeat */
} viop_fc_status_t;

typedef struct VIOP_PACKED {
  uint8_t cmd;
  uint8_t arg;
  uint16_t confirm;
} viop_cmd_t;

typedef struct VIOP_PACKED {
  float x, y, z;            /* local NED, m */
  float yaw;                /* rad */
} viop_setpoint_t;

typedef struct VIOP_PACKED {
  uint32_t t_pc_ms;
  uint32_t flags;
} viop_pc_hb_t;

typedef struct VIOP_PACKED {
  uint16_t dt_us;
  int16_t gyro[3];
  int16_t acc[3];
} viop_imu_sample_t;

typedef struct VIOP_PACKED {
  uint64_t t0_us;
  uint8_t n;
  uint8_t flags;
  uint16_t sample_seq0;
  viop_imu_sample_t s[VIOP_IMU_MAX_N]; /* only n are sent: payload len = 12 + 14 n */
} viop_imu_t;
#if defined(_MSC_VER)
#pragma pack(pop)
#endif

typedef char viop_ev_size_check[(sizeof(viop_fc_event_t) == 16) ? 1 : -1];
typedef char viop_st_size_check[(sizeof(viop_fc_status_t) == 40) ? 1 : -1];
typedef char viop_cmd_size_check[(sizeof(viop_cmd_t) == 4) ? 1 : -1];
typedef char viop_sp_size_check[(sizeof(viop_setpoint_t) == 16) ? 1 : -1];
typedef char viop_pchb_size_check[(sizeof(viop_pc_hb_t) == 8) ? 1 : -1];
typedef char viop_imus_size_check[(sizeof(viop_imu_sample_t) == 14) ? 1 : -1];
typedef char viop_imu_size_check[(sizeof(viop_imu_t) == 12 + 14 * VIOP_IMU_MAX_N && 12 + 14 * VIOP_IMU_MAX_N <= VIOP_MAX_PAYLOAD) ? 1 : -1];

static inline uint8_t viop_imu_payload_len(int n) { return (uint8_t)(12 + 14 * n); }

/* decoded sample, still in PHONE IMU axes */
typedef struct {
  int64_t t_us;
  uint16_t seq;
  float gyro[3], acc[3];
} viop_imu_dec_t;

/* decode an IMU payload (len bytes). Returns number of samples (0 = malformed). */
static inline int viop_imu_decode(const uint8_t *payload, int len, viop_imu_dec_t *out) {
  viop_imu_t m;
  if (len < 12 + 14) return 0;
  memcpy(&m, payload, (size_t)(len < (int)sizeof(m) ? len : (int)sizeof(m)));
  if (m.n < 1 || m.n > VIOP_IMU_MAX_N || len != 12 + 14 * m.n) return 0;
  for (int i = 0; i < m.n; i++) {
    out[i].t_us = (int64_t)m.t0_us + m.s[i].dt_us;
    out[i].seq = (uint16_t)(m.sample_seq0 + i);
    for (int k = 0; k < 3; k++) {
      out[i].gyro[k] = (float)m.s[i].gyro[k] * VIOP_IMU_GYRO_LSB;
      out[i].acc[k] = (float)m.s[i].acc[k] * VIOP_IMU_ACC_LSB;
    }
  }
  return m.n;
}

#endif /* VIOP_FC_EXT_H */
