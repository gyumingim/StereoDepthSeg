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
 *   0x04  VIOP_T_FC_EVENT      drone -> PC (ESP-NOW)    16 B    26 B   on every safety event (state / tier change...)
 *   0x05  VIOP_T_FC_STATUS     drone -> PC (ESP-NOW)    40 B    50 B   20 Hz
 *   0x20  VIOP_T_CMD           PC -> drone (ESP-NOW)     4 B    14 B   on demand (ARM, DISARM, KILL, LAND, CLEAR_KILL)
 *   0x21  VIOP_T_SETPOINT      PC -> drone (ESP-NOW)    16 B    26 B   on demand (local NED position + yaw)
 *   0x2E  VIOP_T_PC_HB         PC -> drone (ESP-NOW)     8 B    18 B   10 Hz (link-alive; missing -> EMERG_LAND)
 *
 * PHONE HEARTBEAT: see viop_proto.h (0x02). The drone lands (EMERG_LAND) if no valid heartbeat for hb_timeout_ms
 * (default 250 ms = 5 periods at 20 Hz), if its seq stops increasing, or if state is STARTING / LOST / STOPPING or a
 * flag STOPPING / IMU_STALE / POSE_STALE is set while flying. It does NOT resume flight when heartbeats come back.
 */
#ifndef VIOP_FC_EXT_H
#define VIOP_FC_EXT_H

#include <stdint.h>
#include "viop_proto.h"

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
#if defined(_MSC_VER)
#pragma pack(pop)
#endif

typedef char viop_ev_size_check[(sizeof(viop_fc_event_t) == 16) ? 1 : -1];
typedef char viop_st_size_check[(sizeof(viop_fc_status_t) == 40) ? 1 : -1];
typedef char viop_cmd_size_check[(sizeof(viop_cmd_t) == 4) ? 1 : -1];
typedef char viop_sp_size_check[(sizeof(viop_setpoint_t) == 16) ? 1 : -1];
typedef char viop_pchb_size_check[(sizeof(viop_pc_hb_t) == 8) ? 1 : -1];

#endif /* VIOP_FC_EXT_H */
