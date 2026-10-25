/* viop_fc_ext.h — PROPOSED additions to viop_proto.h (VIOP v2) for the flight-controller link (esp32fc, 2026-10-25).
 *
 * MERGE POINT: viop_proto.h here is a verbatim copy of ovdrone-20261025 commit bc65b93 (scan3d/ov_bridge/viop_proto.h).
 * The message types below use ids that file marks "reserved" (0x02-0x0F telemetry, 0x20-0x2F commands). When the
 * integration agent merges, move these definitions into viop_proto.h (same ids, same layouts) and delete this file.
 * Framing, CRC, seq rules: unchanged (see viop_proto.h). All little-endian, packed.
 *
 *   id    name                 dir                     payload  frame  rate
 *   0x03  VIOP_T_HEARTBEAT     phone -> drone           16 B    26 B   20 Hz (10..50), from a thread SEPARATE from VIO
 *   0x04  VIOP_T_FC_EVENT      drone -> PC (ESP-NOW)    16 B    26 B   on every safety event (state / tier change...)
 *   0x05  VIOP_T_FC_STATUS     drone -> PC (ESP-NOW)    40 B    50 B   20 Hz
 *   0x20  VIOP_T_CMD           PC -> drone (ESP-NOW)     4 B    14 B   on demand (ARM, DISARM, KILL, LAND, CLEAR_KILL)
 *   0x21  VIOP_T_SETPOINT      PC -> drone (ESP-NOW)    16 B    26 B   on demand (local NED position + yaw)
 *   0x2E  VIOP_T_PC_HB         PC -> drone (ESP-NOW)     8 B    18 B   10 Hz (link-alive; missing -> EMERG_LAND)
 *
 * PHONE HEARTBEAT (the phone-side sender is NOT implemented yet: ovdrone / integration must add it to ov_bridge):
 *  - sent at 20 Hz on the same USB CDC stream as the pose frames, src = VIOP_SRC_PHONE, frame seq +1 per heartbeat
 *    (own counter, independent of the pose seq);
 *  - sent from its own thread (NOT the VIO thread), so a stuck VIO thread shows as vio_state LOST while the heartbeat
 *    keeps arriving, and a dead app / unplugged cable shows as heartbeat timeout;
 *  - vio_state: INIT until VIO initialised; OK; DEGRADED (tracking weak, still usable); LOST (no tracking or no VIO
 *    output for > 100 ms); STOPPING (app is shutting down, e.g. the 40 C thermal self-stop or user stop) — send STOPPING
 *    for >= 3 heartbeats before closing the port if possible;
 *  - the drone lands (EMERG_LAND) if no valid heartbeat for hb_timeout_ms (default 250 ms), if seq stops increasing,
 *    or if vio_state is INIT/LOST/STOPPING while flying. It does NOT resume flight when heartbeats come back.
 */
#ifndef VIOP_FC_EXT_H
#define VIOP_FC_EXT_H

#include <stdint.h>
#include "viop_proto.h"

#define VIOP_T_HEARTBEAT 0x03
#define VIOP_T_FC_EVENT 0x04
#define VIOP_T_FC_STATUS 0x05
#define VIOP_T_CMD 0x20
#define VIOP_T_SETPOINT 0x21
#define VIOP_T_PC_HB 0x2E

/* heartbeat vio_state */
#define VIOP_HB_INIT 0
#define VIOP_HB_OK 1
#define VIOP_HB_DEGRADED 2
#define VIOP_HB_LOST 3
#define VIOP_HB_STOPPING 4
/* heartbeat app_flags */
#define VIOP_HBF_STOPPING 0x01u   /* app shutting down */
#define VIOP_HBF_THERMAL 0x02u    /* thermal guard close to self-stop (warning only) */
#define VIOP_HBF_PREDICT 0x04u    /* IMU forward prediction active */

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
  uint64_t t_phone_us;      /* CLOCK_BOOTTIME / 1000, same clock as viop_pose_t.t_pred_us */
  uint16_t last_pose_seq;   /* seq of the newest pose frame sent */
  uint8_t vio_state;        /* VIOP_HB_* */
  uint8_t app_flags;        /* VIOP_HBF_* */
  uint8_t skin_c;           /* phone skin temperature, deg C (0 = unknown) */
  uint8_t cpu_load_pct;     /* 0 = unknown */
  uint16_t proc_ms_x10;     /* last VIO update processing time, 0.1 ms */
} viop_heartbeat_t;

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
#if defined(_MSC_VER)
#pragma pack(pop)
#endif

typedef char viop_hb_size_check[(sizeof(viop_heartbeat_t) == 16) ? 1 : -1];
typedef char viop_ev_size_check[(sizeof(viop_fc_event_t) == 16) ? 1 : -1];
typedef char viop_st_size_check[(sizeof(viop_fc_status_t) == 40) ? 1 : -1];
typedef char viop_cmd_size_check[(sizeof(viop_cmd_t) == 4) ? 1 : -1];
typedef char viop_sp_size_check[(sizeof(viop_setpoint_t) == 16) ? 1 : -1];
typedef char viop_pchb_size_check[(sizeof(viop_pc_hb_t) == 8) ? 1 : -1];

#endif /* VIOP_FC_EXT_H */
