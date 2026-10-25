/* fc_safety.c — see fc_safety.h. Pure C99, single owner (control task), no malloc. */
#include "fc_safety.h"

#include <math.h>
#include <string.h>

#define G0 9.80665f
#define DEG2RAD 0.017453292519943295f

/* phone heartbeat state / flags, must match VIOP_HB_* / VIOP_HBF_* in viop_proto.h (checked in fs_viop.h) */
#define HB_STARTING 0
#define HB_LOST 2
#define HB_STOPPING 3
#define HBF_STOPPING 0x01u
#define HBF_IMU_STALE 0x02u
#define HBF_POSE_STALE 0x04u
/* pose flags, must match VIOP_F_* in viop_proto.h */
#define PF_VALID 0x0001u
#define PF_TRACKING 0x0002u

static const uint8_t k_allowed[FS_NUM_STATES][FS_NUM_STATES] = {
    /* to:          DISARMED ARMED_IDLE FLYING EMERG_LAND KILLED */
    /* DISARMED   */ {0, 1, 0, 0, 1},
    /* ARMED_IDLE */ {1, 0, 1, 0, 1},
    /* FLYING     */ {0, 1, 0, 1, 1},
    /* EMERG_LAND */ {1, 0, 0, 0, 1},
    /* KILLED     */ {1, 0, 0, 0, 0},
};

static inline uint32_t elapsed(uint32_t now, uint32_t t) { return (uint32_t)(now - t); }
static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline bool is_armed(fs_state_t s) { return s == FS_ARMED_IDLE || s == FS_FLYING || s == FS_EMERG_LAND; }
static inline float tilt_rad(const fs_input_t *in) {
  /* angle between body z and vertical: cos(tilt) = cos(roll) cos(pitch) */
  float c = cosf(in->roll_rad) * cosf(in->pitch_rad);
  return acosf(clampf(c, -1.0f, 1.0f));
}

void fs_config_default(fs_config_t *c) {
  memset(c, 0, sizeof(*c));
  c->n_motors = 4;
  c->tilt_kill_deg = 60.0f;
  c->land_tilt_kill_deg = 45.0f;
  c->rate_kill_dps = 1500.0f;
  c->imu_fail_kill = false;
  c->pose_timeout_ms = 200;
  c->pose_std_max_m = 0.15f;
  c->pose_max_delay_us = 50000;
  c->require_pose_in_flight = true;
  c->land_on_pose_reset = true;
  c->hb_timeout_ms = 250;
  c->require_hb_in_flight = true;
  c->link_timeout_ms = 500;
  c->require_link_in_flight = true;
  c->batt_cells = 4;
  c->batt_land_v_cell = 3.45f;
  c->batt_arm_min_v_cell = 3.70f;
  c->batt_debounce_ms = 2000;
  c->overrun_add = 10;
  c->overrun_land_level = 500;
  c->arm_tilt_max_deg = 10.0f;
  c->thr_arm_max = 0.05f;
  c->require_pose_to_arm = true;
  c->require_hb_to_arm = true;
  c->require_link_to_arm = true;
  c->idle_disarm_ms = 5000;
  c->kill_clear_min_ms = 1000;
  c->thr_takeoff = 0.15f;
  c->touch_acc_var = 0.5f;
  c->touch_acc_dev = 1.5f;
  c->touch_thr_frac = 0.75f;
  c->touch_vz_max = 0.2f;
  c->touch_hold_ms = 300;
  c->impact_acc = 2.0f * G0;
  c->acc_var_tau_s = 0.15f;
  c->land_rate_mps = 0.30f;
  c->land_kp = 0.20f;
  c->land_ki = 0.30f;
  c->hover_thr_default = 0.40f;
  c->land_thr_max_frac = 1.25f;
  c->land_max_ms = 30000;
  c->openloop_start_frac = 0.92f;
  c->openloop_ramp_per_s = 0.05f;
  c->openloop_min_frac = 0.60f;
  c->openloop_max_ms = 6000;
  c->openloop_default_thr = 0.33f;
  c->rec_skip_ms = 300;
  c->rec_min_ms = 500;
  c->rec_window_ms = 3000;
  c->rec_tilt_max_deg = 10.0f;
  c->rec_rate_max_dps = 30.0f;
  c->rec_acc_dev_max = 0.4f;
  c->rec_vz_max = 1.0f;
}

static void push_ev(fs_t *fs, uint8_t code, uint8_t from, uint8_t to, uint8_t aux, uint32_t reason) {
  if (fs->ev_count == FS_EV_RING) { /* full: drop the oldest */
    fs->ev_head = (uint16_t)((fs->ev_head + 1) % FS_EV_RING);
    fs->ev_count--;
    fs->ev_dropped++;
  }
  fs_event_t *e = &fs->ev[(fs->ev_head + fs->ev_count) % FS_EV_RING];
  e->t_ms = fs->now_ms;
  e->seq = fs->ev_seq++;
  e->code = code;
  e->from = from;
  e->to = to;
  e->aux = aux;
  e->reason = reason;
  fs->ev_count++;
}

bool fs_pop_event(fs_t *fs, fs_event_t *ev) {
  if (!fs->ev_count) return false;
  *ev = fs->ev[fs->ev_head];
  fs->ev_head = (uint16_t)((fs->ev_head + 1) % FS_EV_RING);
  fs->ev_count--;
  return true;
}

bool fs_transition_allowed(fs_state_t from, fs_state_t to) {
  if ((unsigned)from >= FS_NUM_STATES || (unsigned)to >= FS_NUM_STATES) return false;
  return k_allowed[from][to] != 0;
}

static void reset_land(fs_t *fs) {
  fs->tier = FS_TIER_NONE;
  fs->land_reason = 0;
  fs->land_i = 0;
  fs->land_thr = 0;
  fs->touch_timing = false;
  fs->ol_src = FS_HOVER_NONE;
  fs->impact_seen = false;
}

static bool go(fs_t *fs, fs_state_t to, uint32_t reason) {
  fs_state_t from = fs->state;
  if (!fs_transition_allowed(from, to)) {
    push_ev(fs, FS_EV_ILLEGAL, (uint8_t)from, (uint8_t)to, 0, reason);
    return false;
  }
  fs->state = to;
  fs->t_state_ms = fs->now_ms;
  push_ev(fs, FS_EV_TRANSITION, (uint8_t)from, (uint8_t)to, 0, reason);
  switch (to) {
    case FS_KILLED:
      fs->kill_reason = reason;
      fs->tier = FS_TIER_NONE;
      fs->pend_land = 0;
      break;
    case FS_DISARMED:
      reset_land(fs);
      fs->pose_reset_flag = false;
      fs->batt_low_active = false;
      fs->batt_low_timing = false;
      fs->pend_land = 0;
      break;
    case FS_FLYING:
      fs->t_takeoff_ms = fs->now_ms;
      fs->touch_timing = false;
      break;
    case FS_ARMED_IDLE:
      fs->touch_timing = false;
      break;
    default:
      break;
  }
  return true;
}

void fs_init(fs_t *fs, const fs_config_t *cfg, uint32_t now_ms) {
  memset(fs, 0, sizeof(*fs));
  if (cfg) fs->cfg = *cfg; else fs_config_default(&fs->cfg);
  if (fs->cfg.n_motors < 1) fs->cfg.n_motors = 1;
  if (fs->cfg.n_motors > FS_MAX_MOTORS) fs->cfg.n_motors = FS_MAX_MOTORS;
  fs->state = FS_DISARMED;
  fs->now_ms = now_ms;
  fs->t_state_ms = now_ms;
  fs->acc_mean = G0;
  fs->hover_ema = fs->cfg.hover_thr_default;
  fs->phone_disconnected = false;
}

/* ---------------- input messages ---------------- */

static bool pose_fresh(const fs_t *fs) {
  return fs->pose_seen && elapsed(fs->now_ms, fs->t_pose_ok_ms) <= fs->cfg.pose_timeout_ms && !fs->pose_lost_flag &&
         !fs->pose_reset_flag;
}
static bool hb_fresh(const fs_t *fs) {
  return fs->hb_seen && elapsed(fs->now_ms, fs->t_hb_ok_ms) <= fs->cfg.hb_timeout_ms && !fs->hb_lost_flag &&
         !fs->phone_disconnected;
}
static bool link_fresh(const fs_t *fs) { return fs->link_seen && elapsed(fs->now_ms, fs->t_link_ms) <= fs->cfg.link_timeout_ms; }

void fs_on_pose(fs_t *fs, uint32_t now_ms, const fs_pose_info_t *p) {
  fs->now_ms = now_ms;
  if (fs->pose_seq_seen && (int16_t)(uint16_t)(p->seq - fs->pose_seq) <= 0 && fs->state != FS_DISARMED)
    return; /* repeated / old frame: never refreshes the pose timer (resync allowed only when disarmed) */
  fs->pose_seq = p->seq;
  fs->pose_seq_seen = true;
  if (fs->rc_seen && p->reset_counter != fs->pose_reset_counter && is_armed(fs->state)) fs->pose_reset_flag = true;
  fs->pose_reset_counter = p->reset_counter;
  fs->rc_seen = true;
  bool valid = (p->flags & PF_VALID) != 0, tracking = (p->flags & PF_TRACKING) != 0;
  fs->pose_lost_flag = !valid || p->quality == 0;
  bool good = valid && tracking && p->quality > 0 && p->pos_std_max_m <= fs->cfg.pose_std_max_m &&
              p->pos_std_max_m == p->pos_std_max_m && p->send_delay_us <= fs->cfg.pose_max_delay_us;
  if (good) {
    fs->pose_seen = true;
    fs->t_pose_ok_ms = now_ms;
  }
}

void fs_on_heartbeat(fs_t *fs, uint32_t now_ms, uint16_t seq, uint8_t vio_state, uint8_t app_flags) {
  fs->now_ms = now_ms;
  if (fs->hb_seen && (int16_t)(uint16_t)(seq - fs->hb_seq) <= 0 && fs->state != FS_DISARMED) {
    fs->hb_seq_stalled = true; /* seq not increasing: frozen sender; does not refresh */
    return;
  }
  fs->hb_seen = true;
  fs->hb_seq = seq;
  fs->hb_seq_stalled = false;
  fs->t_hb_ok_ms = now_ms;
  fs->hb_lost_flag = vio_state == HB_STARTING || vio_state == HB_LOST || vio_state == HB_STOPPING || vio_state > HB_STOPPING ||
                     (app_flags & (HBF_STOPPING | HBF_IMU_STALE | HBF_POSE_STALE));
}

void fs_on_phone_connection(fs_t *fs, uint32_t now_ms, bool connected) {
  fs->now_ms = now_ms;
  fs->phone_disconnected = !connected;
}

void fs_on_link(fs_t *fs, uint32_t now_ms) {
  fs->now_ms = now_ms;
  fs->link_seen = true;
  fs->t_link_ms = now_ms;
}

/* ---------------- commands ---------------- */

static uint32_t ground_blocks(const fs_t *fs) {
  uint32_t b = 0;
  const fs_input_t *in = &fs->last;
  if (!fs->have_input) return FS_B_NO_INPUT;
  if (in->kill_switch) b |= FS_B_SWITCH;
  if (!in->imu_ok) b |= FS_B_IMU;
  else {
    if (tilt_rad(in) > fs->cfg.arm_tilt_max_deg * DEG2RAD) b |= FS_B_TILT;
    if (in->gyro_norm_rps > 0.2f) b |= FS_B_MOVING;
  }
  if (in->acc_ok && fs->acc_var > fs->cfg.touch_acc_var) b |= FS_B_MOVING;
  if (in->thr_cmd > fs->cfg.thr_arm_max) b |= FS_B_THR;
  return b;
}

static uint32_t arm_blocks(const fs_t *fs) {
  uint32_t b = ground_blocks(fs);
  if (fs->cfg.require_pose_to_arm && !pose_fresh(fs)) b |= FS_B_POSE;
  if (fs->cfg.require_hb_to_arm && !hb_fresh(fs)) b |= FS_B_HB;
  if (fs->phone_disconnected) b |= FS_B_PHONE_DISC;
  if (fs->cfg.require_link_to_arm && !link_fresh(fs)) b |= FS_B_LINK;
  if (fs->have_input && fs->last.batt_valid && fs->cfg.batt_cells > 0 &&
      fs->last.batt_v / (float)fs->cfg.batt_cells < fs->cfg.batt_arm_min_v_cell)
    b |= FS_B_BATT;
  if (fs->overrun_bucket > fs->cfg.overrun_land_level / 2) b |= FS_B_OVERRUN;
  return b;
}

static void reset_flight(fs_t *fs) {
  memset(fs->rec_sum, 0, sizeof(fs->rec_sum));
  fs->rec_n = 0;
  fs->rec_ms = 0;
  fs->rec_frozen = false;
  memset(&fs->rec, 0, sizeof(fs->rec));
  fs->rec.n = fs->cfg.n_motors;
  fs->hover_ema = fs->cfg.hover_thr_default;
  fs->pose_reset_flag = false;
  fs->batt_low_active = false;
  fs->batt_low_timing = false;
  fs->pend_land = 0;
  reset_land(fs);
}

static void enter_land(fs_t *fs, uint32_t reason);

bool fs_command(fs_t *fs, uint32_t now_ms, fs_cmd_t cmd) {
  fs->now_ms = now_ms;
  switch (cmd) {
    case FS_CMD_KILL:
      if (fs->state != FS_KILLED) go(fs, FS_KILLED, FS_R_CMD);
      return true;
    case FS_CMD_ARM: {
      if (fs->state != FS_DISARMED) {
        push_ev(fs, FS_EV_ARM_REJECT, (uint8_t)fs->state, (uint8_t)fs->state, (uint8_t)cmd, FS_B_STATE);
        return false;
      }
      uint32_t b = arm_blocks(fs);
      if (b) {
        push_ev(fs, FS_EV_ARM_REJECT, (uint8_t)fs->state, (uint8_t)fs->state, (uint8_t)cmd, b);
        return false;
      }
      reset_flight(fs);
      return go(fs, FS_ARMED_IDLE, FS_R_ARM | FS_R_CMD);
    }
    case FS_CMD_DISARM:
      if (fs->state == FS_ARMED_IDLE) return go(fs, FS_DISARMED, FS_R_DISARM | FS_R_CMD);
      if (fs->state == FS_DISARMED || fs->state == FS_KILLED) return true;
      /* in the air: refused (use KILL to stop motors, LAND to land) */
      push_ev(fs, FS_EV_CMD_REJECT, (uint8_t)fs->state, (uint8_t)fs->state, (uint8_t)cmd, FS_B_STATE);
      return false;
    case FS_CMD_LAND:
      if (fs->state == FS_FLYING) { enter_land(fs, FS_R_CMD); return true; }
      if (fs->state == FS_ARMED_IDLE) return go(fs, FS_DISARMED, FS_R_CMD);
      return true; /* already landing / on the ground */
    case FS_CMD_CLEAR_KILL: {
      if (fs->state != FS_KILLED) {
        push_ev(fs, FS_EV_CLEAR_REJECT, (uint8_t)fs->state, (uint8_t)fs->state, (uint8_t)cmd, FS_B_STATE);
        return false;
      }
      uint32_t b = ground_blocks(fs);
      if (elapsed(now_ms, fs->t_state_ms) < fs->cfg.kill_clear_min_ms) b |= FS_B_TOO_SOON;
      if (b) {
        push_ev(fs, FS_EV_CLEAR_REJECT, (uint8_t)fs->state, (uint8_t)fs->state, (uint8_t)cmd, b);
        return false;
      }
      return go(fs, FS_DISARMED, FS_R_CLEAR | FS_R_CMD);
    }
  }
  push_ev(fs, FS_EV_CMD_REJECT, (uint8_t)fs->state, (uint8_t)fs->state, (uint8_t)cmd, 0);
  return false;
}

void fs_request_kill(fs_t *fs, uint32_t now_ms, uint32_t reason) {
  fs->now_ms = now_ms;
  if (fs->state != FS_KILLED) go(fs, FS_KILLED, reason);
}

void fs_request_land(fs_t *fs, uint32_t now_ms, uint32_t reason) {
  fs->now_ms = now_ms;
  fs->pend_land |= reason;
}

/* ---------------- hover record ---------------- */

void fs_get_hover_record(const fs_t *fs, fs_hover_rec_t *out) { *out = fs->rec; }

void fs_set_prev_hover(fs_t *fs, const fs_hover_rec_t *prev) {
  fs->prev = *prev;
  if (fs->prev.n != fs->cfg.n_motors) fs->prev.valid = false;
  for (int i = 0; i < fs->cfg.n_motors && fs->prev.valid; i++)
    if (!(fs->prev.m[i] > 0.02f && fs->prev.m[i] < 0.98f)) fs->prev.valid = false; /* also rejects NaN */
}

static void record_hover(fs_t *fs, const fs_input_t *in) {
  if (fs->rec_frozen) return;
  const fs_config_t *c = &fs->cfg;
  if (elapsed(in->now_ms, fs->t_takeoff_ms) < c->rec_skip_ms) return;
  if (!in->imu_ok || !in->acc_ok) return;
  if (tilt_rad(in) > c->rec_tilt_max_deg * DEG2RAD) return;
  if (in->gyro_norm_rps > c->rec_rate_max_dps * DEG2RAD) return;
  /* filtered |f| (vibration-robust): near 1 g means thrust ~= weight, i.e. steady climb or hover */
  if (fabsf(fs->acc_mean - G0) > c->rec_acc_dev_max) return;
  if (in->vz_vio_valid && fabsf(in->vz_vio) > c->rec_vz_max) return;
  for (int i = 0; i < c->n_motors; i++)
    if (!(in->motor_prev[i] > 0.02f && in->motor_prev[i] < 0.98f)) return; /* saturated / NaN */
  for (int i = 0; i < c->n_motors; i++) fs->rec_sum[i] += in->motor_prev[i];
  fs->rec_n++;
  fs->rec_ms += in->dt_s * 1000.0f;
  if (fs->rec_ms >= (float)c->rec_min_ms) {
    float col = 0;
    for (int i = 0; i < c->n_motors; i++) {
      fs->rec.m[i] = fs->rec_sum[i] / (float)fs->rec_n;
      col += fs->rec.m[i];
    }
    fs->rec.n = c->n_motors;
    fs->rec.collective = col / (float)c->n_motors;
    fs->rec.samples = fs->rec_n;
    if (!fs->rec.valid) {
      fs->rec.valid = true;
      push_ev(fs, FS_EV_HOVER_REC, (uint8_t)fs->state, (uint8_t)fs->state, (uint8_t)(fs->rec_n / 100 > 255 ? 255 : fs->rec_n / 100), 0);
    }
    if (fs->rec_ms >= (float)c->rec_window_ms) fs->rec_frozen = true;
  }
}

static float hover_collective(const fs_t *fs) { return fs->rec.valid ? fs->rec.collective : fs->hover_ema; }

/* ---------------- emergency land ---------------- */

static bool vio_usable(const fs_t *fs, const fs_input_t *in) {
  return pose_fresh(fs) && !fs->hb_lost_flag && !fs->phone_disconnected && in->vz_vio_valid;
}

static uint32_t vio_bad_bits(const fs_t *fs, const fs_input_t *in) {
  uint32_t r = 0;
  if (!fs->pose_seen || elapsed(fs->now_ms, fs->t_pose_ok_ms) > fs->cfg.pose_timeout_ms) r |= FS_R_POSE_STALE;
  if (fs->pose_lost_flag || !in->vz_vio_valid) r |= FS_R_POSE_BAD;
  if (fs->pose_reset_flag) r |= FS_R_POSE_RESET;
  if (fs->hb_lost_flag) r |= FS_R_HB_LOST;
  if (fs->phone_disconnected) r |= FS_R_PHONE_DISC;
  return r;
}

static void set_tier(fs_t *fs, fs_tier_t t, uint32_t why) {
  fs_tier_t from = fs->tier;
  uint8_t aux = 0;
  if (t == FS_TIER_OPENLOOP) {
    const fs_hover_rec_t *src = NULL;
    if (fs->rec.valid) { src = &fs->rec; fs->ol_src = FS_HOVER_THIS_FLIGHT; }
    else if (fs->prev.valid) { src = &fs->prev; fs->ol_src = FS_HOVER_PREV_FLIGHT; }
    else fs->ol_src = FS_HOVER_DEFAULT;
    for (int i = 0; i < fs->cfg.n_motors; i++) fs->ol_base[i] = src ? src->m[i] : fs->cfg.openloop_default_thr;
    aux = (uint8_t)fs->ol_src;
  }
  fs->tier = t;
  fs->t_tier_ms = fs->now_ms;
  push_ev(fs, FS_EV_TIER, (uint8_t)from, (uint8_t)t, aux, why);
}

static void eval_tier(fs_t *fs, const fs_input_t *in) {
  fs_tier_t want;
  uint32_t why = 0;
  if (!in->imu_ok) { want = FS_TIER_OPENLOOP; why = FS_R_IMU | vio_bad_bits(fs, in); }
  else if (!vio_usable(fs, in)) { want = FS_TIER_IMU; why = vio_bad_bits(fs, in); }
  else want = FS_TIER_VIO;
  if (fs->tier == FS_TIER_NONE || want > fs->tier) set_tier(fs, want, why | (fs->tier == FS_TIER_NONE ? fs->land_reason : 0));
  /* want < tier (VIO came back): ignored on purpose, no step up during a landing */
}

static void enter_land(fs_t *fs, uint32_t reason) {
  if (!go(fs, FS_EMERG_LAND, reason)) return;
  reset_land(fs);
  fs->land_reason = reason;
  fs->land_thr = hover_collective(fs);
  if (fs->have_input) eval_tier(fs, &fs->last);
}

static bool touchdown_cond(const fs_t *fs, const fs_input_t *in, float thr, float hover, bool use_vio_vz) {
  if (!in->acc_ok) return false;
  if (fs->acc_var > fs->cfg.touch_acc_var) return false;
  if (fabsf(fs->acc_mean - G0) > fs->cfg.touch_acc_dev) return false; /* rules out free fall */
  bool impact_recent = fs->impact_seen && elapsed(in->now_ms, fs->t_impact_ms) < 1000;
  if (!(thr <= fs->cfg.touch_thr_frac * hover || impact_recent)) return false;
  if (use_vio_vz && in->vz_vio_valid && fabsf(in->vz_vio) > fs->cfg.touch_vz_max) return false;
  return true;
}

static void land_step(fs_t *fs, const fs_input_t *in, fs_output_t *out) {
  const fs_config_t *c = &fs->cfg;
  eval_tier(fs, in);
  float hover = hover_collective(fs);
  float t_tier = (float)elapsed(in->now_ms, fs->t_tier_ms) * 1e-3f;
  out->vz_target = -c->land_rate_mps;
  if (fs->tier == FS_TIER_OPENLOOP) {
    float frac = c->openloop_start_frac - c->openloop_ramp_per_s * t_tier;
    if (frac < c->openloop_min_frac) frac = c->openloop_min_frac;
    for (int i = 0; i < c->n_motors; i++) out->motor[i] = clampf(fs->ol_base[i] * frac, 0.0f, 1.0f);
    fs->land_thr = hover * frac;
    bool impact = in->acc_ok && fs->impact_seen && (int32_t)(fs->t_impact_ms - fs->t_tier_ms) >= 0;
    if (impact) { go(fs, FS_DISARMED, FS_R_IMPACT | FS_R_TOUCHDOWN); return; }
    if (elapsed(in->now_ms, fs->t_tier_ms) >= c->openloop_max_ms) { go(fs, FS_DISARMED, FS_R_LAND_TIMEOUT); return; }
  } else {
    bool have_vz = (fs->tier == FS_TIER_VIO) ? in->vz_vio_valid : in->vz_imu_valid;
    float vz = (fs->tier == FS_TIER_VIO) ? in->vz_vio : in->vz_imu;
    float thr;
    if (have_vz) {
      float err = -c->land_rate_mps - vz;
      fs->land_i = clampf(fs->land_i + c->land_ki * err * in->dt_s, -hover, 0.3f * hover);
      thr = hover + c->land_kp * err + fs->land_i;
    } else { /* no vertical speed at all: slow timed ramp of the collective */
      float frac = c->openloop_start_frac - c->openloop_ramp_per_s * t_tier;
      thr = hover * (frac < c->openloop_min_frac ? c->openloop_min_frac : frac);
    }
    thr = clampf(thr, 0.0f, c->land_thr_max_frac * hover);
    fs->land_thr = thr;
    if (touchdown_cond(fs, in, thr, hover, fs->tier == FS_TIER_VIO)) {
      if (!fs->touch_timing) { fs->touch_timing = true; fs->t_touch_ms = in->now_ms; }
      if (elapsed(in->now_ms, fs->t_touch_ms) >= c->touch_hold_ms) { go(fs, FS_DISARMED, FS_R_TOUCHDOWN); return; }
    } else fs->touch_timing = false;
  }
  if (elapsed(in->now_ms, fs->t_state_ms) >= c->land_max_ms) go(fs, FS_DISARMED, FS_R_LAND_TIMEOUT);
}

/* ---------------- main step ---------------- */

static uint32_t land_triggers(fs_t *fs, const fs_input_t *in) {
  const fs_config_t *c = &fs->cfg;
  uint32_t r = fs->pend_land;
  if (c->require_pose_in_flight) {
    if (!fs->pose_seen || elapsed(fs->now_ms, fs->t_pose_ok_ms) > c->pose_timeout_ms) r |= FS_R_POSE_STALE;
    if (fs->pose_lost_flag) r |= FS_R_POSE_BAD;
  }
  if (c->land_on_pose_reset && fs->pose_reset_flag) r |= FS_R_POSE_RESET;
  if (c->require_hb_in_flight) {
    if (!fs->hb_seen || elapsed(fs->now_ms, fs->t_hb_ok_ms) > c->hb_timeout_ms) r |= FS_R_HB_STALE | (fs->hb_seq_stalled ? FS_R_HB_SEQ : 0);
    if (fs->hb_lost_flag) r |= FS_R_HB_LOST;
    if (fs->phone_disconnected) r |= FS_R_PHONE_DISC;
  }
  if (c->require_link_in_flight && !link_fresh(fs)) r |= FS_R_LINK;
  if (fs->batt_low_active) r |= FS_R_BATT;
  if (fs->overrun_bucket > c->overrun_land_level) r |= FS_R_OVERRUN;
  if (!in->imu_ok && !c->imu_fail_kill) r |= FS_R_IMU;
  return r;
}

void fs_step(fs_t *fs, const fs_input_t *in, fs_output_t *out) {
  const fs_config_t *c = &fs->cfg;
  fs->now_ms = in->now_ms;
  float dt = in->dt_s;
  if (!(dt > 0.0f)) dt = 0.0f;
  if (dt > 0.1f) dt = 0.1f;

  /* accelerometer statistics (touchdown / impact) */
  if (in->acc_ok && in->acc_norm == in->acc_norm) {
    float a = dt / (c->acc_var_tau_s + dt);
    float d = in->acc_norm - fs->acc_mean;
    fs->acc_mean += a * d;
    fs->acc_var = (1.0f - a) * (fs->acc_var + a * d * d);
    if (in->acc_norm > c->impact_acc) { fs->impact_seen = true; fs->t_impact_ms = in->now_ms; }
  }
  /* loop overrun leaky bucket */
  if (in->loop_overrun) { fs->overrun_bucket += c->overrun_add; fs->overruns++; }
  else if (fs->overrun_bucket > 0) fs->overrun_bucket--;
  /* battery (debounced, latched until disarm) */
  if (in->batt_valid && c->batt_cells > 0 && in->batt_v / (float)c->batt_cells < c->batt_land_v_cell) {
    if (!fs->batt_low_timing) { fs->batt_low_timing = true; fs->t_batt_low_ms = in->now_ms; }
    if (elapsed(in->now_ms, fs->t_batt_low_ms) >= c->batt_debounce_ms && is_armed(fs->state)) fs->batt_low_active = true;
  } else fs->batt_low_timing = false;

  fs->last = *in;
  fs->last.dt_s = dt;
  fs->have_input = true;

  /* 1) KILL checks: override everything */
  if (fs->state != FS_KILLED) {
    uint32_t k = 0;
    if (in->kill_switch) k |= FS_R_SWITCH;
    if (is_armed(fs->state)) {
      if (in->imu_ok) {
        float lim = (fs->state == FS_EMERG_LAND ? c->land_tilt_kill_deg : c->tilt_kill_deg) * DEG2RAD;
        if (tilt_rad(in) > lim || in->roll_rad != in->roll_rad || in->pitch_rad != in->pitch_rad) k |= FS_R_TILT;
        if (in->gyro_norm_rps > c->rate_kill_dps * DEG2RAD) k |= FS_R_RATE;
      } else if (c->imu_fail_kill) k |= FS_R_IMU;
    }
    if (k) go(fs, FS_KILLED, k);
  }

  /* 2) land triggers */
  if (fs->state == FS_FLYING || fs->state == FS_ARMED_IDLE || fs->state == FS_EMERG_LAND) {
    uint32_t trig = land_triggers(fs, in);
    if (fs->state == FS_FLYING && trig) enter_land(fs, trig);
    else if (fs->state == FS_ARMED_IDLE && trig) go(fs, FS_DISARMED, trig);
    else if (fs->state == FS_EMERG_LAND) {
      uint32_t nw = trig & ~fs->land_reason;
      if (nw) { push_ev(fs, FS_EV_TRIGGER, (uint8_t)fs->state, (uint8_t)fs->state, 0, nw); fs->land_reason |= nw; }
    }
  }
  fs->pend_land = 0;

  /* 3) per-state behaviour */
  memset(out, 0, sizeof(*out));
  switch (fs->state) {
    case FS_ARMED_IDLE:
      if (in->thr_cmd > c->thr_takeoff) go(fs, FS_FLYING, FS_R_TAKEOFF);
      else if (elapsed(in->now_ms, fs->t_state_ms) >= c->idle_disarm_ms) go(fs, FS_DISARMED, FS_R_IDLE);
      break;
    case FS_FLYING: {
      record_hover(fs, in);
      bool stable = in->imu_ok && tilt_rad(in) < c->rec_tilt_max_deg * DEG2RAD && (!in->vz_vio_valid || fabsf(in->vz_vio) < 0.3f);
      if (stable && in->thr_cmd > 0.05f && in->thr_cmd < 0.95f) fs->hover_ema += dt / (2.0f + dt) * (in->thr_cmd - fs->hover_ema);
      /* normal (commanded) landing: low collective + quiet accelerometer, not right after takeoff */
      if (elapsed(in->now_ms, fs->t_takeoff_ms) > 2000 && in->thr_cmd <= c->touch_thr_frac * hover_collective(fs) &&
          touchdown_cond(fs, in, in->thr_cmd, hover_collective(fs), true)) {
        if (!fs->touch_timing) { fs->touch_timing = true; fs->t_touch_ms = in->now_ms; }
        if (elapsed(in->now_ms, fs->t_touch_ms) >= c->touch_hold_ms) go(fs, FS_ARMED_IDLE, FS_R_TOUCHDOWN);
      } else fs->touch_timing = false;
      break;
    }
    case FS_EMERG_LAND:
      land_step(fs, in, out);
      break;
    default:
      break;
  }

  /* 4) outputs */
  out->state = fs->state;
  out->tier = fs->state == FS_EMERG_LAND ? fs->tier : FS_TIER_NONE;
  out->motors_enabled = is_armed(fs->state);
  out->thr = fs->land_thr;
  switch (fs->state) {
    case FS_ARMED_IDLE: out->mode = FS_CTRL_IDLE; break;
    case FS_FLYING: out->mode = FS_CTRL_NORMAL; break;
    case FS_EMERG_LAND:
      out->mode = fs->tier == FS_TIER_VIO ? FS_CTRL_LAND_HOLD_XY : (fs->tier == FS_TIER_IMU ? FS_CTRL_LAND_LEVEL : FS_CTRL_OPENLOOP);
      break;
    default: out->mode = FS_CTRL_OFF; break;
  }
  if (!out->motors_enabled) {
    out->thr = 0;
    for (int i = 0; i < FS_MAX_MOTORS; i++) out->motor[i] = 0;
  }
}

bool fs_check_outputs(fs_t *fs, uint32_t now_ms, const float *m, int n) {
  fs->now_ms = now_ms;
  if (!is_armed(fs->state)) return false;
  for (int i = 0; i < n; i++) {
    if (!(m[i] >= -0.001f && m[i] <= 1.001f)) { /* NaN fails both comparisons */
      go(fs, FS_KILLED, FS_R_NAN);
      return false;
    }
  }
  return true;
}

const char *fs_state_name(fs_state_t s) {
  static const char *n[] = {"DISARMED", "ARMED_IDLE", "FLYING", "EMERG_LAND", "KILLED"};
  return (unsigned)s < FS_NUM_STATES ? n[s] : "?";
}
const char *fs_tier_name(fs_tier_t t) {
  static const char *n[] = {"NONE", "LAND_VIO", "LAND_IMU", "LAND_OPENLOOP"};
  return (unsigned)t <= FS_TIER_OPENLOOP ? n[t] : "?";
}
