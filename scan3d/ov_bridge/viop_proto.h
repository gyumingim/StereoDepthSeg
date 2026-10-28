/* viop_proto.h — VIOP drone link framing + VIO pose message (2026-10-25, branch ovdrone-20261025, protocol version 2).
 * Plain C99, header-only, no dependencies. The SAME file is meant for: ov_bridge on the phone (sender), the drone
 * ESP32-S3 flight firmware, the PC-side ESP32 relay and PC tools (viop_rx.py mirrors it in Python).
 *
 * ASSUMED TOPOLOGY (coordinator's reading of the user, 2026-10-25; not confirmed):
 *   phone (VIO) --USB-OTG serial (CDC)--> drone ESP32-S3 (custom flight control) <==ESP-NOW==> PC-side ESP32 --USB--> PC
 *   (UI / logging). For bench tests now: phone --UDP--> PC (same bytes, one frame per datagram).
 * Design rules from that: one frame <= 96 bytes (ESP-NOW v1 payload limit 250) so the drone ESP32 can relay a pose
 * frame unchanged; a message-type byte so the same framing carries telemetry / commands later; seq + timestamps for
 * loss and latency statistics at every hop.
 *
 * FRAME (little-endian, packed):
 *   0  u8  sync0 0xA5
 *   1  u8  sync1 0x5A
 *   2  u8  version (VIOP_VERSION)
 *   3  u8  type    (VIOP_T_*)
 *   4  u8  len     payload bytes (frame size = 8 + len + 2)
 *   5  u8  src     sender id (VIOP_SRC_*)
 *   6  u16 seq     per sender and type, +1 per frame (gaps = loss)
 *   8  ... payload (len bytes)
 *   8+len u16 crc  CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over bytes [2, 8+len)  (sync bytes excluded)
 * Stream transports (USB CDC, UART, ESP-NOW concatenation): hunt for A5 5A, read len, check crc (viop_stream_feed).
 *
 * MESSAGE TYPES (payload layouts below; ids 0x03-0x0F telemetry, 0x20-0x2F commands are RESERVED for later, e.g.
 *   0x03 health / battery telemetry, 0x22 mode; flight-controller ids 0x04 0x05 0x06 0x20 0x21 0x2E are defined at the
 *   end of this file):
 *   0x01 VIOP_T_POSE      82 B  phone -> drone (-> PC relay): IMU-forward-predicted VIO pose      frame 92 B
 *   0x02 VIOP_T_HEARTBEAT 18 B  phone -> drone: liveness + VIO health, 10-50 Hz (default 20)        frame 28 B
 *        Sent from the bridge's MAIN input loop (on IMU records), NOT from a timer thread: when the app stops feeding,
 *        the bridge hangs or is killed, heartbeats stop -> the drone ESP32 must treat a missing heartbeat (e.g. > 3
 *        periods, timeout chosen by the firmware) as "phone dead" and land. On a clean stop the bridge sends 3
 *        heartbeats with state VIOP_HB_STOPPING first.
 *   0x10 VIOP_T_TIME_REQ  16 B  any -> phone: u64 t_req (requester clock, echoed), u64 0               frame 26 B
 *   0x11 VIOP_T_TIME_RESP 16 B  phone -> requester: u64 t_req (echo), u64 t_phone_rx_ns (CLOCK_BOOTTIME at receipt)
 *        offset(phone - requester) = t_phone_rx - (t_req + t_resp_arrival) / 2, take the smallest round trips.
 *
 * HEARTBEAT PAYLOAD (18 B):
 *   0  u64 t_phone_us       CLOCK_BOOTTIME at send, us
 *   8  u8  state            VIOP_HB_* (0 starting / not initialised, 1 tracking, 2 lost = IMU-only, 3 stopping)
 *   9  u8  flags            VIOP_HBF_* (bit0 app/bridge stopping, bit1 IMU stale > 100 ms, bit2 no pose sent > 200 ms)
 *  10  u16 imu_age_ms       now - receipt of the newest IMU record (saturated)
 *  12  u16 pose_age_ms      now - last POSE frame sent (65535 = none yet)
 *  14  u16 hb_period_ms     configured heartbeat period
 *  16  u8  reset_counter    as in POSE
 *  17  u8  quality          last POSE quality (0 if none)
 *
 * POSE PAYLOAD (82 B):
 *   0  u64 t_pred_us        predicted instant, phone sensor clock = Android CLOCK_BOOTTIME (elapsedRealtimeNanos)/1000
 *   8  u32 send_delay_us    t_send - t_pred (phone-internal age when handed to the transport)
 *  12  u16 horizon_100us    t_pred - filter state time (last camera update), 0.1 ms, saturated
 *  14  u16 flags            VIOP_F_*
 *  16  f32 p_w[3]           m      position of the phone IMU in W
 *  28  f32 v_w[3]           m/s    velocity in W
 *  40  i16 q_wi[4]          w x y z * 32767, rotation I -> W (Hamilton); renormalise after decoding
 *  48  f32 w_i[3]           rad/s  angular rate in I (gyro - estimated bias, newest IMU sample)
 *  60  u16 pos_std_mm[3]    1-sigma position on the W axes, mm (saturated 65535)
 *  66  u16 vel_std_mm_s[3]  1-sigma velocity on the W axes, mm/s
 *  72  u16 att_std_100urad[3] 1-sigma attitude, small angle about the I axes, 1e-4 rad
 *  78  u8  reset_counter    +1 per VIO (re)initialisation (W changes)
 *  79  u8  quality          0 lost / invalid, 1..100 (100 * (1 - pos_sigma / 0.20 m), provisional)
 *  80  u16 n_feat           SLAM features in the filter state (health)
 *   std values come from the filter covariance at the state time (not propagated over the horizon: optimistic).
 *
 * FRAMES:
 *   W = world, gravity aligned, Z UP (gravity -Z), x / y = arbitrary yaw fixed at VIO initialisation, origin = IMU
 *       position at initialisation; a NEW W after each reset (reset_counter changes, flag VIOP_F_RESET on the first
 *       VIOP_RESET_REPEAT poses of the new W - armhist 2026-10-29: repeated so that one lost frame does not hide it;
 *       was: once). A W started by STATE INJECTION (ov_bridge --reinit-state / --recover: the new filter is seeded
 *       with the old position / velocity / attitude) also sets VIOP_F_SEEDED on EVERY pose of that W: it is a new W
 *       for re-anchoring, but NOT a full VIO re-initialisation (a diverged state may have been carried over).
 *   I = body frame of the PHONE IMU = Android sensor axes: +x right edge of the screen, +y top edge, +z out of the
 *       screen (rear camera looks along -z). NOT the drone body.
 *   The drone ESP32 applies its mounting R_BI (phone IMU -> drone body, e.g. FRD) and z-down local frame
 *   R_LW = diag(1, -1, -1):  R_LB = R_LW R(q_WI) R_BI^T,  p_L = R_LW (p_W + R(q_WI) r_I)  (r_I = lever arm IMU ->
 *   vehicle reference point, in I), v_L = R_LW v_W, w_B = R_BI w_I.  viop_pose_to_local() does it without lever arm.
 */
#ifndef VIOP_PROTO_H
#define VIOP_PROTO_H

#include <stdint.h>
#include <string.h>

#define VIOP_SYNC0 0xA5
#define VIOP_SYNC1 0x5A
#define VIOP_VERSION 2
#define VIOP_HDR 8
#define VIOP_MAX_PAYLOAD 86 /* frame <= 96 bytes */
#define VIOP_MAX_FRAME (VIOP_HDR + VIOP_MAX_PAYLOAD + 2)

#define VIOP_T_POSE 0x01
#define VIOP_T_HEARTBEAT 0x02
#define VIOP_T_TIME_REQ 0x10
#define VIOP_T_TIME_RESP 0x11

#define VIOP_SRC_PHONE 1
#define VIOP_SRC_DRONE 2
#define VIOP_SRC_PC 3

#define VIOP_HB_STARTING 0
#define VIOP_HB_TRACKING 1
#define VIOP_HB_LOST 2
#define VIOP_HB_STOPPING 3
#define VIOP_HBF_STOPPING 0x01u
#define VIOP_HBF_IMU_STALE 0x02u
#define VIOP_HBF_POSE_STALE 0x04u

#define VIOP_F_VALID 0x0001u      /* VIO initialised */
#define VIOP_F_TRACKING 0x0002u   /* features tracked (not lost); lost = IMU-only drift */
#define VIOP_F_FROM_FRAME 0x0004u /* first prediction after a camera update (else between frames) */
#define VIOP_F_RESET 0x0008u      /* first pose of a new W: re-anchor */
#define VIOP_F_REPLAY 0x0010u     /* re-timed replay of an old stream (bench), not live */
#define VIOP_F_SEEDED 0x0020u     /* armhist 10-29: this W was seeded from the previous filter state (not a full
                                     re-init); set on every pose of that W */
#define VIOP_RESET_REPEAT 50      /* VIOP_F_RESET is set on the first 50 poses of a new W (armhist 10-29) */

#if defined(_MSC_VER)
#pragma pack(push, 1)
#define VIOP_PACKED
#else
#define VIOP_PACKED __attribute__((packed))
#endif
typedef struct VIOP_PACKED {
  uint64_t t_pred_us;
  uint32_t send_delay_us;
  uint16_t horizon_100us;
  uint16_t flags;
  float p_w[3];
  float v_w[3];
  int16_t q_wi[4];
  float w_i[3];
  uint16_t pos_std_mm[3];
  uint16_t vel_std_mm_s[3];
  uint16_t att_std_100urad[3];
  uint8_t reset_counter;
  uint8_t quality;
  uint16_t n_feat;
} viop_pose_t;
typedef struct VIOP_PACKED {
  uint64_t t_phone_us;
  uint8_t state;
  uint8_t flags;
  uint16_t imu_age_ms;
  uint16_t pose_age_ms;
  uint16_t hb_period_ms;
  uint8_t reset_counter;
  uint8_t quality;
} viop_heartbeat_t;
typedef struct VIOP_PACKED {
  uint64_t t_req;
  uint64_t t_phone_rx_ns;
} viop_time_t;
#if defined(_MSC_VER)
#pragma pack(pop)
#endif
typedef char viop_pose_size_check[(sizeof(viop_pose_t) == 82) ? 1 : -1];
typedef char viop_hb_size_check[(sizeof(viop_heartbeat_t) == 18) ? 1 : -1];
typedef char viop_time_size_check[(sizeof(viop_time_t) == 16) ? 1 : -1];

static inline uint16_t viop_crc16(const uint8_t *d, uint32_t n) {
  uint16_t crc = 0xFFFF;
  for (uint32_t i = 0; i < n; i++) {
    crc ^= (uint16_t)((uint16_t)d[i] << 8);
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

/* build a frame into out (>= VIOP_HDR + len + 2 bytes); returns the frame size */
static inline int viop_build(uint8_t *out, uint8_t type, uint8_t src, uint16_t seq, const void *payload, uint8_t len) {
  out[0] = VIOP_SYNC0;
  out[1] = VIOP_SYNC1;
  out[2] = VIOP_VERSION;
  out[3] = type;
  out[4] = len;
  out[5] = src;
  out[6] = (uint8_t)(seq & 0xff);
  out[7] = (uint8_t)(seq >> 8);
  memcpy(out + VIOP_HDR, payload, len);
  uint16_t c = viop_crc16(out + 2, (uint32_t)(VIOP_HDR - 2 + len));
  out[VIOP_HDR + len] = (uint8_t)(c & 0xff);
  out[VIOP_HDR + len + 1] = (uint8_t)(c >> 8);
  return VIOP_HDR + len + 2;
}

/* check one complete frame of n bytes (e.g. a UDP datagram / ESP-NOW packet). 0 ok, <0 error */
static inline int viop_check(const uint8_t *f, int n) {
  if (n < VIOP_HDR + 2 || f[0] != VIOP_SYNC0 || f[1] != VIOP_SYNC1) return -1;
  if (f[2] != VIOP_VERSION || f[4] > VIOP_MAX_PAYLOAD || n != VIOP_HDR + f[4] + 2) return -2;
  uint16_t c = (uint16_t)(f[VIOP_HDR + f[4]] | (f[VIOP_HDR + f[4] + 1] << 8));
  return viop_crc16(f + 2, (uint32_t)(VIOP_HDR - 2 + f[4])) == c ? 0 : -3;
}

/* byte-stream parser (USB CDC / UART / concatenated ESP-NOW): append received bytes with viop_stream_push(), then call
 * viop_stream_next() until it returns 0. Returns the frame size (frame at st->buf, valid until the next call). A bad
 * CRC or a false sync drops only ONE byte and rescans, so junk can never swallow a following real frame. */
typedef struct {
  uint8_t buf[4 * VIOP_MAX_FRAME];
  int n;          /* bytes in buf */
  int consumed;   /* size of the frame returned last (removed at the next call) */
  uint32_t ok, bad, dropped_bytes;
} viop_stream_t;
static inline int viop_stream_push(viop_stream_t *st, const uint8_t *d, int len) {
  int k = 0;
  while (k < len) {
    if (st->consumed) { memmove(st->buf, st->buf + st->consumed, (size_t)(st->n - st->consumed)); st->n -= st->consumed; st->consumed = 0; }
    int room = (int)sizeof(st->buf) - st->n;
    if (room == 0) { st->dropped_bytes++; memmove(st->buf, st->buf + 1, (size_t)(st->n - 1)); st->n--; room = 1; }
    int c = len - k < room ? len - k : room;
    memcpy(st->buf + st->n, d + k, (size_t)c);
    st->n += c;
    k += c;
  }
  return k;
}
static inline int viop_stream_next(viop_stream_t *st) {
  if (st->consumed) { memmove(st->buf, st->buf + st->consumed, (size_t)(st->n - st->consumed)); st->n -= st->consumed; st->consumed = 0; }
  for (;;) {
    int i = 0;
    while (i + 1 < st->n && !(st->buf[i] == VIOP_SYNC0 && st->buf[i + 1] == VIOP_SYNC1)) i++;
    if (i) { st->dropped_bytes += (uint32_t)i; memmove(st->buf, st->buf + i, (size_t)(st->n - i)); st->n -= i; }
    if (st->n < VIOP_HDR + 2) return 0;
    int len = st->buf[4];
    if (st->buf[2] != VIOP_VERSION || len > VIOP_MAX_PAYLOAD) {  /* false sync: drop one byte */
      st->bad++; st->dropped_bytes++; memmove(st->buf, st->buf + 1, (size_t)(st->n - 1)); st->n--; continue;
    }
    int fs = VIOP_HDR + len + 2;
    if (st->n < fs) return 0;
    if (viop_check(st->buf, fs) == 0) { st->ok++; st->consumed = fs; return fs; }
    st->bad++; st->dropped_bytes++; memmove(st->buf, st->buf + 1, (size_t)(st->n - 1)); st->n--;
  }
}

static inline void viop_quat_to_R(const float q[4], float R[9]) {
  float w = q[0], x = q[1], y = q[2], z = q[3];
  R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - w * z);     R[2] = 2 * (x * z + w * y);
  R[3] = 2 * (x * y + w * z);     R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - w * x);
  R[6] = 2 * (x * z - w * y);     R[7] = 2 * (y * z + w * x);     R[8] = 1 - 2 * (x * x + y * y);
}

/* decoded unit quaternion (w x y z) from the int16 field */
static inline void viop_pose_quat(const viop_pose_t *p, float q[4]) {
  float n = 0;
  for (int i = 0; i < 4; i++) { q[i] = p->q_wi[i] / 32767.0f; n += q[i] * q[i]; }
  if (n > 0) { float s = 1.0f / __builtin_sqrtf(n); for (int i = 0; i < 4; i++) q[i] *= s; }
}

/* example mounts R_BI (row-major, drone body FRD <- phone IMU), nominal axis permutations:
 *   cam_fwd : phone upright in landscape, rear camera looking forward, image upright (phone +x edge up)
 *   cam_down: phone flat, screen up, rear camera looking down, phone +x edge forward */
static const float viop_mount_cam_fwd[9] = {0, 0, -1, 0, -1, 0, -1, 0, 0};
static const float viop_mount_cam_down[9] = {1, 0, 0, 0, -1, 0, 0, 0, -1};

static inline void viop_pose_to_local(const viop_pose_t *p, const float R_BI[9], float p_l[3], float v_l[3], float R_LB[9],
                                      float w_b[3]) {
  float q[4], R_WI[9], pw[3], vw[3], wi[3];
  viop_pose_quat(p, q);
  viop_quat_to_R(q, R_WI);
  memcpy(pw, p->p_w, 12);
  memcpy(vw, p->v_w, 12);
  memcpy(wi, p->w_i, 12);
  const float s[3] = {1, -1, -1};
  for (int i = 0; i < 3; i++) {
    p_l[i] = s[i] * pw[i];
    v_l[i] = s[i] * vw[i];
    w_b[i] = R_BI[3 * i] * wi[0] + R_BI[3 * i + 1] * wi[1] + R_BI[3 * i + 2] * wi[2];
  }
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      float a = 0;
      for (int k = 0; k < 3; k++) a += R_WI[3 * i + k] * R_BI[3 * j + k];
      R_LB[3 * i + j] = s[i] * a;
    }
}

/* ===== Flight-controller link messages (esp32fc 2026-10-25; merged from esp32fc viop_fc_ext.h by integ-20261025, same ids /
 * layouts; viop_fc_ext.h deleted).
 *
 * Framing, CRC, seq rules: unchanged (see viop_proto.h). All little-endian, packed.
 *
 *   id    name                 dir                     payload  frame  rate
 *   0x06  VIOP_T_IMU           phone -> drone (USB)  12+14n B  36..92 B  RAW PHONE IMU at full rate (~500 Hz), n = 1..5
 *                                                                          samples per frame (n = 1 recommended: lowest
 *                                                                          latency; 36 B x 500 Hz = 18 kB/s << USB FS)
 *   0x04  VIOP_T_FC_EVENT      drone -> PC (ESP-NOW)    16 B    26 B   on every safety event (state / tier change...)
 *   0x05  VIOP_T_FC_STATUS     drone -> PC (ESP-NOW)    40 B    50 B   20 Hz
 *   0x20  VIOP_T_CMD           PC -> drone (ESP-NOW)     4 B    14 B   on demand (ARM, DISARM, KILL, LAND, CLEAR_KILL,
 *                                                                     VIO_RESET)
 *   0x23  VIOP_T_VIO_RESET_REQ drone -> phone (USB)     4 B    14 B   armhist 10-29: after an accepted VIOP_CMD_VIO_RESET
 *                                                                     (only DISARMED / KILLED): payload = viop_cmd_t
 *                                                                     {cmd VIOP_CMD_VIO_RESET, confirm}; the phone does a
 *                                                                     FULL VIO re-init (ov_bridge: as kReset, never state
 *                                                                     injection) -> new W without VIOP_F_SEEDED, which
 *                                                                     clears the FC VIO history latch (FS_B_VIO_HIST)
 *   0x21  VIOP_T_SETPOINT      PC -> drone (ESP-NOW)    16 B    26 B   on demand (local NED position + yaw)
 *   0x2E  VIOP_T_PC_HB         PC -> drone (ESP-NOW)     8 B    18 B   10 Hz (link-alive; missing -> EMERG_LAND)
 *
 * PHONE HEARTBEAT: see viop_proto.h (0x02). The drone lands (EMERG_LAND) if no valid heartbeat for hb_timeout_ms
 * (default 250 ms = 5 periods at 20 Hz), if its seq stops increasing, or if state is STARTING / LOST / STOPPING or a
 * flag STOPPING / IMU_STALE / POSE_STALE is set while flying. It does NOT resume flight when heartbeats come back.
 *
 * PHONE IMU STREAM (0x06, user decision 2026-10-25: the phone IMU is the ONLY IMU; the ESP32 attitude/rate loop runs on
 * it). Phone sender: arlat app ViopImuSender (integ-20261025, intent viop_imu, default off): on every
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
#define VIOP_T_VIO_RESET_REQ 0x23

/* commands (VIOP_T_CMD.cmd); confirm must equal VIOP_CMD_CONFIRM(cmd) (guards against a mis-parsed frame) */
#define VIOP_CMD_ARM 1
#define VIOP_CMD_DISARM 2
#define VIOP_CMD_KILL 3
#define VIOP_CMD_LAND 4
#define VIOP_CMD_CLEAR_KILL 5
#define VIOP_CMD_VIO_RESET 6 /* armhist 10-29: ask the phone for a full VIO re-init (FC forwards 0x23) */
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

/* ---- loop closure map frame (2026-10-26, branch ovloop-20261026; ov_bridge --loop 1 only) ----
 *   0x07 VIOP_T_MAP_ODOM  44 B  phone -> drone / PC: loop-closure correction T_map_odom, frame 54 B. Sent at
 *        --loop-out-hz (default 5 Hz) and at once after each new pose-graph optimisation. Receivers that do not know the
 *        type ignore it (esp32fc link_task.c handles only POSE / HEARTBEAT / IMU) -> the control pose (POSE) is unchanged.
 *        map position of the phone IMU: p_map = Rz(yaw_mo) p_w + t_mo (W axes, z up); attitude R_map = Rz(yaw_mo) R(q_wi).
 *        The published (t_mo, yaw_mo) moves toward the optimiser target (tgt_*) rate limited (no jump in the map pose).
 *   0  u64 t_us          frame time (phone sensor clock, us; + replay offset like POSE)
 *   8  f32 t_mo[3]       published translation (m)
 *  20  f32 yaw_mo        published yaw (rad)
 *  24  f32 tgt_t[3]      optimiser target translation (m)
 *  36  f32 tgt_yaw       target yaw (rad)
 *  40  u16 n_loops       accepted loop edges in this W
 *  42  u8  reset_counter as in POSE (a new W also starts a new map: T = identity)
 *  43  u8  flags         VIOP_MOF_*
 */
#define VIOP_T_MAP_ODOM 0x07
#define VIOP_MOF_VALID 0x01u      /* at least one optimisation in this W (else identity) */
#define VIOP_MOF_CONVERGING 0x02u /* published != target (still moving toward it) */
#if defined(_MSC_VER)
#pragma pack(push, 1)
#endif
typedef struct VIOP_PACKED {
  uint64_t t_us;
  float t_mo[3];
  float yaw_mo;
  float tgt_t[3];
  float tgt_yaw;
  uint16_t n_loops;
  uint8_t reset_counter;
  uint8_t flags;
} viop_map_odom_t;
#if defined(_MSC_VER)
#pragma pack(pop)
#endif
typedef char viop_map_odom_size_check[(sizeof(viop_map_odom_t) == 44) ? 1 : -1];

#endif /* VIOP_PROTO_H */
