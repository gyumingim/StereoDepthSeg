/* fs_viop.h — glue between VIOP frames (viop_proto.h + viop_fc_ext.h) and fc_safety. Header-only, pure C. */
#ifndef FS_VIOP_H
#define FS_VIOP_H

#include "fc_safety.h"
#include "viop_fc_ext.h"

/* fc_safety.c hard-codes these values (to stay independent of the protocol header): keep them in sync */
typedef char fs_viop_chk1[(VIOP_F_VALID == 0x0001u && VIOP_F_TRACKING == 0x0002u) ? 1 : -1];
typedef char fs_viop_chk2[(VIOP_HB_INIT == 0 && VIOP_HB_LOST == 3 && VIOP_HB_STOPPING == 4 && VIOP_HBF_STOPPING == 1) ? 1 : -1];
typedef char fs_viop_chk3[(VIOP_CMD_ARM == FS_CMD_ARM && VIOP_CMD_DISARM == FS_CMD_DISARM && VIOP_CMD_KILL == FS_CMD_KILL &&
                           VIOP_CMD_LAND == FS_CMD_LAND && VIOP_CMD_CLEAR_KILL == FS_CMD_CLEAR_KILL) ? 1 : -1];

static inline void fs_pose_info_from_viop(const viop_pose_t *p, uint16_t frame_seq, fs_pose_info_t *o) {
  uint16_t m = p->pos_std_mm[0];
  if (p->pos_std_mm[1] > m) m = p->pos_std_mm[1];
  if (p->pos_std_mm[2] > m) m = p->pos_std_mm[2];
  o->seq = frame_seq;
  o->flags = p->flags;
  o->quality = p->quality;
  o->reset_counter = p->reset_counter;
  o->pos_std_max_m = (float)m * 1e-3f;
  o->send_delay_us = p->send_delay_us;
}

static inline void fs_event_to_viop(const fs_event_t *e, viop_fc_event_t *o) {
  o->t_ms = e->t_ms;
  o->ev_seq = e->seq;
  o->code = e->code;
  o->from = e->from;
  o->to = e->to;
  o->aux = e->aux;
  o->pad = 0;
  o->reason = e->reason;
}

#endif /* FS_VIOP_H */
