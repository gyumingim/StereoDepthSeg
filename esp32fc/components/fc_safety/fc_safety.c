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

/* time since t; a t slightly in the FUTURE (message stamped after the loop's now) counts as 0, never as ~49 days */
static inline uint32_t elapsed(uint32_t now, uint32_t t) { return (int32_t)(now - t) < 0 ? 0u : (uint32_t)(now - t); }
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
  c->require_batt_to_arm = true;
  c->overrun_add = 10;
  c->overrun_land_level = 500;
  c->arm_tilt_max_deg = 10.0f;
  c->thr_arm_max = 0.05f;
  c->require_pose_to_arm = true;
  c->require_hb_to_arm = true;
  c->require_link_to_arm = true;
  c->arm_gyro_max_rps = 0.05f;     /* arm guard ON (user 10-29 "일단 켜셈"): fcarm post hoc less-marginal set; former 0.2 */
  c->arm_acc_var_max = 0.1f;       /* former -1 (= touch_acc_var) */
  c->arm_acc_dev_max = -1.0f;
  c->arm_still_ms = 1000;          /* former 0 (off) */
  c->arm_vio_speed_max = 0.2f;     /* former 0 (off) */
  c->arm_vio_speed_hold_ms = 0;
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
  c->openloop_thrust_exp = 2.0f;
  c->openloop_acc_down = 1.0f;
  /* esp32fix 2026-10-26: defaults = esp32fc (mode 0, 0.4, 0.99); the user decides. Mode 1 is opt-in and was evaluated
   * with openloop_cut_mode = 1, openloop_vz_down = 0.6, openloop_cruise_ratio = 0.95 (test t_openloop_cut, README) */
  c->openloop_vz_down = 0.4f;
  c->openloop_cruise_ratio = 0.99f;
  c->openloop_cut_margin = 1.2f;
  c->openloop_volt_comp = true;
  c->openloop_cut_mode = 0;
  c->openloop_thr_err = 0.03f;
  c->openloop_end_ratio = 0.70f;
  c->openloop_end_ramp_s = 0.4f;
  c->openloop_hold_s = 0.2f;
  c->openloop_vio_touch = true;
  c->openloop_vio_touch_h = 0.10f;
  c->openloop_vio_touch_ms = 150;
  c->openloop_vio_desc_vz = 0.3f;
  c->openloop_vio_extend_ms = 2000;
  c->rec_skip_ms = 300;
  c->rec_min_ms = 500;
  c->rec_window_ms = 3000;
  c->rec_tilt_max_deg = 10.0f;
  c->rec_rate_max_dps = 30.0f;
  c->rec_acc_dev_max = 0.4f;
  c->rec_vz_max = 1.0f;
  c->rec_climb_vz = 0.10f;
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
  fs->ol_ramp = false;
  fs->ol_touch_timing = false;
  fs->ol_desc_seen = false;
  fs->ol_fast_ms = fs->ol_slow_ms = 0;
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
      fs->airborne = false;
      break;
    case FS_ARMED_IDLE:
      fs->touch_timing = false;
      fs->takeoff_enabled = false; /* re-enabled when the collective request is seen low (no instant re-takeoff) */
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
  fs->pose_speed = NAN;
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
  if (fs->pose_seq_seen) {
    int16_t d = (int16_t)(uint16_t)(p->seq - fs->pose_seq);
    if (d == 0 || (d < 0 && fs->state != FS_DISARMED)) return; /* repeated / old frame: never refreshes the pose timer */
  }
  fs->pose_seq = p->seq;
  fs->pose_seq_seen = true;
  if (fs->rc_seen && p->reset_counter != fs->pose_reset_counter && is_armed(fs->state)) fs->pose_reset_flag = true;
  fs->pose_reset_counter = p->reset_counter;
  fs->rc_seen = true;
  bool valid = (p->flags & PF_VALID) != 0, tracking = (p->flags & PF_TRACKING) != 0;
  fs->pose_lost_flag = !valid || p->quality == 0;
  if (valid) { /* arm hardening: VIO speed of every valid pose (good or not) */
    fs->pose_speed = p->speed_mps;
    if (fs->cfg.arm_vio_speed_max > 0.0f && !(p->speed_mps <= fs->cfg.arm_vio_speed_max)) {
      fs->vio_fast_seen = true;
      fs->t_vio_fast_ms = now_ms;
    }
  }
  bool good = valid && tracking && p->quality > 0 && p->pos_std_max_m <= fs->cfg.pose_std_max_m &&
              p->pos_std_max_m == p->pos_std_max_m && p->send_delay_us <= fs->cfg.pose_max_delay_us;
  if (good) {
    fs->pose_seen = true;
    fs->t_pose_ok_ms = now_ms;
  }
}

void fs_on_heartbeat(fs_t *fs, uint32_t now_ms, uint16_t seq, uint8_t vio_state, uint8_t app_flags) {
  fs->now_ms = now_ms;
  if (fs->hb_seen) {
    int16_t d = (int16_t)(uint16_t)(seq - fs->hb_seq);
    if (d == 0 || (d < 0 && fs->state != FS_DISARMED)) { /* seq not increasing: frozen sender; does not refresh */
      fs->hb_seq_stalled = true;
      return;
    }
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

/* MOVING rule on the accelerometer: quiet AND ~1 g (rules out free fall / being carried); limits configurable (fcarm) */
static bool acc_quiet(const fs_t *fs, const fs_input_t *in) {
  float vmax = fs->cfg.arm_acc_var_max >= 0.0f ? fs->cfg.arm_acc_var_max : fs->cfg.touch_acc_var;
  float dmax = fs->cfg.arm_acc_dev_max >= 0.0f ? fs->cfg.arm_acc_dev_max : fs->cfg.touch_acc_dev;
  return in->acc_ok && fs->acc_var <= vmax && fabsf(fs->acc_mean - G0) <= dmax;
}
/* IMU still by the MOVING rule (gyro + accelerometer); used for arming and for the still-time requirement */
static bool imu_still(const fs_t *fs, const fs_input_t *in) {
  return in->imu_ok && in->gyro_norm_rps <= fs->cfg.arm_gyro_max_rps && acc_quiet(fs, in);
}

static uint32_t ground_blocks(const fs_t *fs) {
  uint32_t b = 0;
  const fs_input_t *in = &fs->last;
  if (!fs->have_input) return FS_B_NO_INPUT;
  if (in->kill_switch) b |= FS_B_SWITCH;
  if (!in->imu_ok) b |= FS_B_IMU;
  else {
    if (!(tilt_rad(in) <= fs->cfg.arm_tilt_max_deg * DEG2RAD)) b |= FS_B_TILT;
    if (!(in->gyro_norm_rps <= fs->cfg.arm_gyro_max_rps)) b |= FS_B_MOVING; /* NaN limit / rate blocks */
  }
  /* resting on the ground: quiet accelerometer AND ~1 g (rules out free fall / being carried) */
  if (!acc_quiet(fs, in)) b |= FS_B_MOVING;
  if (!(in->thr_cmd <= fs->cfg.thr_arm_max)) b |= FS_B_THR; /* NaN blocks too */
  return b;
}

static uint32_t arm_blocks(const fs_t *fs) {
  uint32_t b = ground_blocks(fs);
  if (fs->cfg.require_pose_to_arm && !pose_fresh(fs)) b |= FS_B_POSE;
  if (fs->cfg.require_hb_to_arm && !hb_fresh(fs)) b |= FS_B_HB;
  if (fs->phone_disconnected) b |= FS_B_PHONE_DISC;
  if (fs->cfg.require_link_to_arm && !link_fresh(fs)) b |= FS_B_LINK;
  if (fs->cfg.batt_cells > 0) {
    bool known = fs->have_input && fs->last.batt_valid;
    if ((fs->cfg.require_batt_to_arm && !known) ||
        (known && !(fs->last.batt_v / (float)fs->cfg.batt_cells >= fs->cfg.batt_arm_min_v_cell)))
      b |= FS_B_BATT;
  }
  if (fs->overrun_bucket > fs->cfg.overrun_land_level / 2) b |= FS_B_OVERRUN;
  /* fcarm 10-27, ON by default since armguard 10-29: still long enough, and VIO not moving while the IMU is still */
  if (fs->cfg.arm_still_ms > 0 && fs->have_input &&
      (!fs->still_run || elapsed(fs->now_ms, fs->t_still_ms) < fs->cfg.arm_still_ms))
    b |= FS_B_NOT_STILL;
  if (fs->cfg.arm_vio_speed_max > 0.0f &&
      (!(fs->pose_speed <= fs->cfg.arm_vio_speed_max) ||
       (fs->vio_fast_seen && elapsed(fs->now_ms, fs->t_vio_fast_ms) < fs->cfg.arm_vio_speed_hold_ms)))
    b |= FS_B_VIO_SPEED;
  return b;
}

static void reset_flight(fs_t *fs) {
  memset(fs->rec_sum, 0, sizeof(fs->rec_sum));
  fs->rec_batt_sum = 0;
  fs->rec_batt_n = 0;
  fs->last_height_valid = false;
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

static void update_airborne(fs_t *fs, const fs_input_t *in) {
  if (fs->airborne) return;
  /* lift-off evidence: climbing (VIO vz, else the IMU vz estimate). Spinning up on the ground never sets it. */
  bool climb = (in->vz_vio_valid && in->vz_vio > fs->cfg.rec_climb_vz) || (!in->vz_vio_valid && in->vz_imu_valid && in->vz_imu > fs->cfg.rec_climb_vz);
  if (climb) { fs->airborne = true; fs->t_airborne_ms = in->now_ms; }
}

static void record_hover(fs_t *fs, const fs_input_t *in) {
  if (fs->rec_frozen || !fs->airborne) return;
  const fs_config_t *c = &fs->cfg;
  if (elapsed(in->now_ms, fs->t_airborne_ms) < c->rec_skip_ms) return;
  if (!in->imu_ok || !in->acc_ok) return;
  if (tilt_rad(in) > c->rec_tilt_max_deg * DEG2RAD) return;
  if (in->gyro_norm_rps > c->rec_rate_max_dps * DEG2RAD) return;
  /* filtered |f| (vibration-robust): near 1 g means thrust ~= weight, i.e. steady climb or hover */
  if (fabsf(fs->acc_mean - G0) > c->rec_acc_dev_max) return;
  if (in->vz_vio_valid && fabsf(in->vz_vio) > c->rec_vz_max) return;
  for (int i = 0; i < c->n_motors; i++)
    if (!(in->motor_prev[i] > 0.02f && in->motor_prev[i] < 0.98f)) return; /* saturated / NaN */
  for (int i = 0; i < c->n_motors; i++) fs->rec_sum[i] += in->motor_prev[i];
  if (in->batt_valid) { fs->rec_batt_sum += in->batt_v; fs->rec_batt_n++; }
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
    fs->rec.batt_v = fs->rec_batt_n ? fs->rec_batt_sum / (float)fs->rec_batt_n : 0.0f;
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

/* open-loop thrust / weight ratio at t s after the tier start. ramp_s < 0: no end ramp (mode 0, or before t_fast) */
static float ol_ratio(const fs_config_t *c, float t, float ramp_s) {
  float t1 = c->openloop_vz_down / c->openloop_acc_down;
  float r = t < t1 ? 1.0f - c->openloop_acc_down / G0 : c->openloop_cruise_ratio;
  if (c->openloop_cut_mode == 1 && ramp_s >= 0.0f && t >= ramp_s) {
    float f = c->openloop_end_ramp_s > 0.0f ? (t - ramp_s) / c->openloop_end_ramp_s : 1.0f;
    if (f > 1.0f) f = 1.0f;
    r = r * (1.0f - f) + c->openloop_end_ratio * f;
  }
  return r;
}

/* 1-D fall of the open-loop profile from h0 with the thrust scaled by m; returns the touchdown time in ms (or max_ms).
 * 5 ms steps, <= openloop_max_ms / 5 iterations (1200 at 6 s): run twice at the tier start only. */
static uint32_t ol_sim_touch(const fs_config_t *c, float h0, float m, float ramp_s) {
  const float dt = 0.005f;
  float z = h0, v = 0.0f;
  uint32_t n = c->openloop_max_ms / 5u;
  for (uint32_t i = 0; i < n; i++) {
    float t = (float)i * dt;
    v += (ol_ratio(c, t, ramp_s) * m - 1.0f) * G0 * dt;
    z += v * dt;
    if (z <= 0.0f) return (i + 1u) * 5u;
  }
  return c->openloop_max_ms;
}

static void set_tier(fs_t *fs, fs_tier_t t, uint32_t why) {
  fs_tier_t from = fs->tier;
  uint8_t aux = 0;
  if (t == FS_TIER_OPENLOOP) {
    const fs_hover_rec_t *src = NULL;
    if (fs->rec.valid) { src = &fs->rec; fs->ol_src = FS_HOVER_THIS_FLIGHT; }
    else if (fs->prev.valid) { src = &fs->prev; fs->ol_src = FS_HOVER_PREV_FLIGHT; }
    else fs->ol_src = FS_HOVER_DEFAULT;
    const fs_config_t *c = &fs->cfg;
    for (int i = 0; i < c->n_motors; i++) fs->ol_base[i] = src ? src->m[i] : c->openloop_default_thr;
    aux = (uint8_t)fs->ol_src;
    /* battery sag since the record: the same thrust needs a larger command (thrust ~ (cmd * V)^k) */
    fs->ol_vcomp = 1.0f;
    if (c->openloop_volt_comp && src && src->batt_v > 1.0f && fs->have_input && fs->last.batt_valid && fs->last.batt_v > 1.0f)
      fs->ol_vcomp = clampf(src->batt_v / fs->last.batt_v, 0.85f, 1.2f);
    float t1 = c->openloop_vz_down / c->openloop_acc_down;
    fs->ol_cut_ms = c->openloop_max_ms;
    fs->ol_ramp = false;
    fs->ol_touch_timing = false;
    fs->ol_desc_seen = false;
    fs->ol_fast_ms = fs->ol_slow_ms = 0;
    if (c->openloop_cut_mode == 1) {
      /* band of touchdown times for a record that is off by -/+ thr_err. No height known: no end ramp, cut at
       * openloop_max_ms (as the legacy mode) */
      if (fs->last_height_valid && fs->last_height > 0.0f) {
        float h = fs->last_height;
        fs->ol_fast_ms = ol_sim_touch(c, h, 1.0f - c->openloop_thr_err, -1.0f);
        fs->ol_slow_ms = ol_sim_touch(c, h, 1.0f + c->openloop_thr_err, (float)fs->ol_fast_ms * 1e-3f);
        uint32_t cut = fs->ol_slow_ms + (uint32_t)(c->openloop_hold_s * 1000.0f);
        if (cut < fs->ol_cut_ms) fs->ol_cut_ms = cut;
        fs->ol_ramp = true;
      }
    } else if (fs->last_height_valid && fs->last_height > 0.0f) {
      /* legacy (mode 0) cut time from the last known height: phase 1 accelerates to vz_down, then ~constant descent */
      float d1 = 0.5f * c->openloop_acc_down * t1 * t1;
      float t_est = t1 + (fs->last_height > d1 ? (fs->last_height - d1) / c->openloop_vz_down : 0.0f);
      float ms = (t_est * c->openloop_cut_margin + 0.3f) * 1000.0f;
      if (ms < (float)fs->ol_cut_ms) fs->ol_cut_ms = (uint32_t)ms;
    }
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
    float ratio = ol_ratio(c, t_tier, fs->ol_ramp ? (float)fs->ol_fast_ms * 1e-3f : -1.0f);
    float frac = powf(ratio, 1.0f / c->openloop_thrust_exp) * fs->ol_vcomp;
    for (int i = 0; i < c->n_motors; i++) out->motor[i] = clampf(fs->ol_base[i] * frac, 0.0f, 1.0f);
    fs->land_thr = hover * frac;
    bool impact = in->acc_ok && fs->impact_seen && (int32_t)(fs->t_impact_ms - fs->t_tier_ms) >= 0;
    if (impact) { go(fs, FS_DISARMED, FS_R_IMPACT | FS_R_TOUCHDOWN); return; }
    /* mode 1: VIO still usable (only the IMU frames stopped): on the ground near the takeoff height, not moving --
     * only after a descent was seen on VIO in this tier (verifier 2026-10-26: at the tier start the vehicle hovers with
     * vz ~ 0, so a hover just above the takeoff height, or a low VIO height, would otherwise cut in the air) */
    const bool vio_ok = vio_usable(fs, in);
    if (vio_ok && in->vz_vio < -c->openloop_vio_desc_vz) fs->ol_desc_seen = true;
    if (c->openloop_cut_mode == 1 && c->openloop_vio_touch && vio_ok && fs->ol_desc_seen && in->height_valid &&
        in->height_m < c->openloop_vio_touch_h && fabsf(in->vz_vio) < c->touch_vz_max) {
      if (!fs->ol_touch_timing) { fs->ol_touch_timing = true; fs->t_ol_touch_ms = in->now_ms; }
      if (elapsed(in->now_ms, fs->t_ol_touch_ms) >= c->openloop_vio_touch_ms) { go(fs, FS_DISARMED, FS_R_TOUCHDOWN); return; }
    } else fs->ol_touch_timing = false;
    /* mode 1: never cut on the timer while VIO is usable and still shows a descent (vz <= -touch_vz_max), for at most
     * openloop_vio_extend_ms past the cut time (descent direction only + cap: a tip-over on the ground cannot keep the
     * motors on). Verifier 2026-10-26: the timer cut still descending vehicles in the air with VIO alive. */
    if (elapsed(in->now_ms, fs->t_tier_ms) >= fs->ol_cut_ms) {
      const bool extend = c->openloop_cut_mode == 1 && vio_ok && in->vz_vio <= -c->touch_vz_max &&
                          elapsed(in->now_ms, fs->t_tier_ms) < fs->ol_cut_ms + c->openloop_vio_extend_ms;
      if (!extend) { go(fs, FS_DISARMED, FS_R_LAND_TIMEOUT); return; }
    }
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
  /* VIO / IMU land taking too long (stuck, estimator wrong): step down to the open-loop ramp, which ends with a cut */
  if (fs->state == FS_EMERG_LAND && fs->tier != FS_TIER_OPENLOOP && elapsed(in->now_ms, fs->t_state_ms) >= c->land_max_ms)
    set_tier(fs, FS_TIER_OPENLOOP, FS_R_LAND_TIMEOUT);
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

static void sanitize(fs_input_t *s) {
  if (!isfinite(s->roll_rad) || !isfinite(s->pitch_rad) || !isfinite(s->gyro_norm_rps)) s->imu_ok = false;
  if (!isfinite(s->acc_norm)) { s->acc_ok = false; s->imu_ok = false; }
  if (!isfinite(s->vz_vio)) s->vz_vio_valid = false;
  if (!isfinite(s->vz_imu)) s->vz_imu_valid = false;
  if (!isfinite(s->height_m)) s->height_valid = false;
  if (!isfinite(s->batt_v) || s->batt_v <= 0.0f) s->batt_valid = false;
  /* thr_cmd NaN: comparisons below are written so that NaN blocks arming and never triggers takeoff */
}

void fs_step(fs_t *fs, const fs_input_t *in_raw, fs_output_t *out) {
  const fs_config_t *c = &fs->cfg;
  fs_input_t in_s = *in_raw;
  sanitize(&in_s);
  const fs_input_t *in = &in_s;
  fs->now_ms = in->now_ms;
  float dt = in->dt_s;
  if (!(dt > 0.0f)) dt = 0.0f;
  if (dt > 0.1f) dt = 0.1f;

  /* accelerometer statistics (touchdown / impact) */
  if (in->acc_ok) {
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
  if (in->batt_valid && c->batt_cells > 0 && in->batt_v / (float)c->batt_cells < c->batt_land_v_cell) { /* sanitised */
    if (!fs->batt_low_timing) { fs->batt_low_timing = true; fs->t_batt_low_ms = in->now_ms; }
    if (elapsed(in->now_ms, fs->t_batt_low_ms) >= c->batt_debounce_ms && is_armed(fs->state)) fs->batt_low_active = true;
  } else fs->batt_low_timing = false;

  fs->last = *in;
  fs->last.dt_s = dt;
  /* arm hardening: how long the IMU has been still without a break (MOVING rule, this cycle's statistics) */
  if (imu_still(fs, in)) {
    if (!fs->still_run) { fs->still_run = true; fs->t_still_ms = in->now_ms; }
  } else fs->still_run = false;
  /* last known height (for the open-loop cut time): while not landing, or while landing on VIO */
  if (in->height_valid && in->vz_vio_valid && (fs->state != FS_EMERG_LAND || fs->tier == FS_TIER_VIO)) {
    fs->last_height = in->height_m;
    fs->last_height_valid = true;
  }
  fs->have_input = true;

  /* 1) KILL checks: override everything */
  if (fs->state != FS_KILLED) {
    uint32_t k = 0;
    if (in->kill_switch) k |= FS_R_SWITCH;
    if (is_armed(fs->state)) {
      if (in->imu_ok) {
        float lim = (fs->state == FS_EMERG_LAND ? c->land_tilt_kill_deg : c->tilt_kill_deg) * DEG2RAD;
        if (!(tilt_rad(in) <= lim)) k |= FS_R_TILT;
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
      if (in->thr_cmd <= c->thr_arm_max) fs->takeoff_enabled = true;
      if (fs->takeoff_enabled && in->thr_cmd > c->thr_takeoff && in->thr_cmd <= 1.0f) go(fs, FS_FLYING, FS_R_TAKEOFF);
      else if (elapsed(in->now_ms, fs->t_state_ms) >= c->idle_disarm_ms) go(fs, FS_DISARMED, FS_R_IDLE);
      break;
    case FS_FLYING: {
      update_airborne(fs, in);
      record_hover(fs, in);
      bool stable = fs->airborne && in->imu_ok && tilt_rad(in) < c->rec_tilt_max_deg * DEG2RAD && (!in->vz_vio_valid || fabsf(in->vz_vio) < 0.3f);
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
