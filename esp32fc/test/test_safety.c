/* test_safety.c — PC unit tests for components/fc_safety (+ sanity tests for components/fc_core).
 * Build + run:  make -C esp32fc/test   (gcc, -Wall -Wextra -Werror, ASan + UBSan)
 * Scenarios run against a 1 kHz closed-loop vertical simulation (1-D: thrust, gravity, ground contact with an impact
 * spike), with phone pose (200 Hz), phone heartbeat (20 Hz) and PC link heartbeat (10 Hz) message generators. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fc_core.h"
#include "fc_safety.h"
#include "fs_viop.h"

static int g_fail = 0, g_checks = 0, g_illegal = 0;
static const char *g_test = "";
#define CHECK(c)                                                                         \
  do {                                                                                   \
    g_checks++;                                                                          \
    if (!(c)) {                                                                          \
      g_fail++;                                                                          \
      printf("  FAIL [%s] %s:%d: %s\n", g_test, __FILE__, __LINE__, #c);                 \
    }                                                                                    \
  } while (0)

/* ------------------------------------------------------------------ simulation */
#define NEV 512
typedef struct {
  fs_t fs;
  fs_input_t in;
  fs_output_t out;
  uint32_t t;                 /* ms */
  float z, vz;                /* up */
  float hover_true;           /* collective for 1 g */
  float z_target;
  bool want_fly;
  bool pose_on, hb_on, link_on, imu_ok, acc_ok, vio_vz_on, imu_vz_on;
  uint8_t hb_state, hb_flags, reset_counter;
  bool hb_freeze_seq;
  uint16_t pose_seq, hb_seq;
  float tilt_deg;             /* forced roll */
  float batt_v;
  bool kill_switch, overrun, nan_out;
  uint32_t rng;
  float applied;              /* collective applied this tick */
  float motor[4];
  fs_event_t ev[NEV];
  int nev;
  bool was_flying, saw_flying_after_land;
} sim_t;

static const float k_motor_ofs[4] = {-0.02f, 0.01f, 0.025f, -0.015f}; /* per-motor trim the controller "needs" */

static float frand(sim_t *s) {
  s->rng = s->rng * 1103515245u + 12345u;
  return ((float)((s->rng >> 9) & 0xffff) / 65535.0f) * 2.0f - 1.0f;
}

static void sim_init(sim_t *s, const fs_config_t *cfg) {
  memset(s, 0, sizeof(*s));
  fs_config_t c;
  if (cfg) c = *cfg; else fs_config_default(&c);
  s->t = 1000;
  fs_init(&s->fs, &c, s->t);
  s->hover_true = 0.42f;
  s->z_target = 1.0f;
  s->pose_on = s->hb_on = s->link_on = s->imu_ok = s->acc_ok = s->vio_vz_on = s->imu_vz_on = true;
  s->hb_state = VIOP_HB_TRACKING;
  s->batt_v = 16.4f;
  s->rng = 1;
}

static void drain(sim_t *s) {
  fs_event_t e;
  while (fs_pop_event(&s->fs, &e)) {
    if (e.code == FS_EV_ILLEGAL) g_illegal++;
    if (getenv("FS_VERBOSE"))
      printf("    ev t=%u code=%u from=%u to=%u aux=%u reason=0x%x\n", e.t_ms, e.code, e.from, e.to, e.aux, e.reason);
    if (s->nev < NEV) s->ev[s->nev++] = e;
  }
}

static void sim_tick(sim_t *s) {
  s->t++;
  uint32_t t = s->t;
  if (s->pose_on && t % 5 == 0) {
    fs_pose_info_t p = {.seq = s->pose_seq++, .flags = VIOP_F_VALID | VIOP_F_TRACKING, .quality = 90,
                        .reset_counter = s->reset_counter, .pos_std_max_m = 0.02f, .send_delay_us = 3000};
    fs_on_pose(&s->fs, t, &p);
  }
  if (s->hb_on && t % 50 == 0) {
    fs_on_heartbeat(&s->fs, t, s->hb_seq, s->hb_state, s->hb_flags);
    if (!s->hb_freeze_seq) s->hb_seq++;
  }
  if (s->link_on && t % 100 == 0) fs_on_link(&s->fs, t);

  /* normal controller (what the position loop would request) */
  float thr_cmd = 0;
  /* like pos_task: 0 in IDLE until a takeoff setpoint is processed (a few ms after arming) */
  if (s->fs.state == FS_ARMED_IDLE && s->want_fly && t - s->fs.t_state_ms > 10) thr_cmd = 0.5f;
  else if (s->fs.state == FS_FLYING || s->fs.state == FS_EMERG_LAND) {
    float h = 0.42f; /* the controller's own integrator converges on the true hover */
    thr_cmd = h * (1.0f + 0.6f * (s->z_target - s->z) - 0.6f * s->vz);
    if (thr_cmd < 0.05f) thr_cmd = 0.05f;
    if (thr_cmd > 0.9f) thr_cmd = 0.9f;
  }
  /* what actually reaches the motors, from last cycle's safety output */
  float col = 0;
  switch (s->out.mode) {
    case FS_CTRL_NORMAL: col = thr_cmd; break;
    case FS_CTRL_IDLE: col = 0.05f; break;
    case FS_CTRL_LAND_HOLD_XY:
    case FS_CTRL_LAND_LEVEL: col = s->out.thr; break;
    case FS_CTRL_OPENLOOP: col = 0.25f * (s->out.motor[0] + s->out.motor[1] + s->out.motor[2] + s->out.motor[3]); break;
    default: col = 0; break;
  }
  if (!s->out.motors_enabled) col = 0;
  for (int i = 0; i < 4; i++) {
    if (s->out.mode == FS_CTRL_OPENLOOP) s->motor[i] = s->out.motor[i];
    else s->motor[i] = col > 0 ? col * (1.0f + k_motor_ofs[i]) : 0;
  }
  s->applied = col;

  /* physics, dt = 1 ms */
  const float dt = 0.001f;
  float tr = (col / s->hover_true) * (col / s->hover_true); /* thrust / weight, thrust ~ cmd^2 */
  float a = tr * FC_G - FC_G;
  float acc_norm;
  bool impact = false;
  if (s->z <= 0 && a <= 0) {
    if (s->vz < -0.1f) impact = true;
    acc_norm = FC_G + 0.05f * frand(s);
    if (impact) acc_norm = FC_G + fabsf(s->vz) * 60.0f;
    s->z = 0;
    s->vz = 0;
  } else {
    s->vz += a * dt;
    s->z += s->vz * dt;
    acc_norm = tr * FC_G + 0.15f * frand(s);
    if (s->z < 0) { /* hit the ground this tick */
      if (s->vz < -0.1f) acc_norm = FC_G + fabsf(s->vz) * 60.0f;
      s->z = 0;
      s->vz = 0;
    }
  }

  fs_input_t *in = &s->in;
  memset(in, 0, sizeof(*in));
  in->now_ms = t;
  in->dt_s = dt;
  in->imu_ok = s->imu_ok;
  in->acc_ok = s->acc_ok;
  in->roll_rad = s->tilt_deg * 0.0174533f;
  in->pitch_rad = 0;
  in->gyro_norm_rps = 0.01f;
  in->acc_norm = acc_norm;
  in->thr_cmd = thr_cmd;
  in->vz_vio_valid = s->pose_on && s->vio_vz_on;
  in->vz_vio = s->vz;
  in->vz_imu_valid = s->imu_vz_on;
  in->vz_imu = s->vz + 0.03f; /* small estimator error */
  in->height_valid = s->pose_on;
  in->height_m = s->z;
  in->batt_valid = true;
  in->batt_v = s->batt_v;
  in->kill_switch = s->kill_switch;
  in->loop_overrun = s->overrun;
  for (int i = 0; i < 4; i++) in->motor_prev[i] = s->motor[i];
  fs_step(&s->fs, in, &s->out);

  if (s->out.motors_enabled) { /* mixer output check, as the firmware does */
    float m[4];
    for (int i = 0; i < 4; i++) m[i] = s->out.mode == FS_CTRL_OPENLOOP ? s->out.motor[i] : 0.4f;
    if (s->nan_out) m[2] = NAN;
    if (!fs_check_outputs(&s->fs, t, m, 4)) s->out.motors_enabled = false;
  }
  if (s->fs.state == FS_FLYING) s->was_flying = true;
  drain(s);
}

static void run_ms(sim_t *s, uint32_t ms) { for (uint32_t i = 0; i < ms; i++) sim_tick(s); }
/* run until state == st or timeout; returns ms taken (or -1) */
static int run_until(sim_t *s, fs_state_t st, uint32_t max_ms) {
  for (uint32_t i = 0; i < max_ms; i++) {
    if (s->fs.state == st) return (int)i;
    sim_tick(s);
  }
  return s->fs.state == st ? (int)max_ms : -1;
}
static const fs_event_t *find_ev(const sim_t *s, int code, int to) {
  for (int i = 0; i < s->nev; i++)
    if (s->ev[i].code == code && (to < 0 || s->ev[i].to == to)) return &s->ev[i];
  return NULL;
}
static int count_ev(const sim_t *s, int code, int to) {
  int n = 0;
  for (int i = 0; i < s->nev; i++) if (s->ev[i].code == code && (to < 0 || s->ev[i].to == to)) n++;
  return n;
}
static bool cmd(sim_t *s, fs_cmd_t c) {
  bool r = fs_command(&s->fs, s->t, c);
  drain(s);
  return r;
}

/* settle on the ground, arm, take off, hover at 1 m for hover_ms */
static void fly_to_hover(sim_t *s, uint32_t hover_ms) {
  run_ms(s, 600);
  CHECK(cmd(s, FS_CMD_ARM));
  CHECK(s->fs.state == FS_ARMED_IDLE);
  s->want_fly = true;
  run_ms(s, 3000 + hover_ms);
  CHECK(s->fs.state == FS_FLYING);
  CHECK(fabsf(s->z - 1.0f) < 0.1f);
}

/* ------------------------------------------------------------------ tests */

static void t_transition_table(void) {
  g_test = "transition_table";
  CHECK(fs_transition_allowed(FS_DISARMED, FS_ARMED_IDLE));
  CHECK(!fs_transition_allowed(FS_DISARMED, FS_FLYING));
  CHECK(!fs_transition_allowed(FS_EMERG_LAND, FS_FLYING)); /* no auto-resume */
  CHECK(!fs_transition_allowed(FS_KILLED, FS_ARMED_IDLE)); /* must clear first */
  CHECK(!fs_transition_allowed(FS_FLYING, FS_DISARMED));
  for (int f = 0; f < FS_NUM_STATES; f++) CHECK(fs_transition_allowed((fs_state_t)f, FS_KILLED) == (f != FS_KILLED));
}

static void t_arm_rules(void) {
  g_test = "arm_rules";
  sim_t s;
  sim_init(&s, NULL);
  CHECK(!cmd(&s, FS_CMD_ARM)); /* no input yet */
  CHECK(find_ev(&s, FS_EV_ARM_REJECT, -1) && (find_ev(&s, FS_EV_ARM_REJECT, -1)->reason & FS_B_NO_INPUT));
  s.pose_on = s.hb_on = s.link_on = false;
  run_ms(&s, 600);
  s.nev = 0;
  CHECK(!cmd(&s, FS_CMD_ARM));
  const fs_event_t *e = find_ev(&s, FS_EV_ARM_REJECT, -1);
  CHECK(e && (e->reason & FS_B_POSE) && (e->reason & FS_B_HB) && (e->reason & FS_B_LINK));
  s.pose_on = s.hb_on = s.link_on = true;
  s.hb_state = VIOP_HB_STARTING; /* VIO not initialised yet */
  run_ms(&s, 300);
  s.nev = 0;
  CHECK(!cmd(&s, FS_CMD_ARM));
  CHECK(find_ev(&s, FS_EV_ARM_REJECT, -1)->reason == FS_B_HB);
  s.hb_state = VIOP_HB_TRACKING;
  s.tilt_deg = 15;
  run_ms(&s, 300);
  s.nev = 0;
  CHECK(!cmd(&s, FS_CMD_ARM));
  CHECK(find_ev(&s, FS_EV_ARM_REJECT, -1)->reason & FS_B_TILT);
  s.tilt_deg = 0;
  s.batt_v = 14.0f; /* 3.5 V/cell < 3.7 */
  run_ms(&s, 300);
  s.nev = 0;
  CHECK(!cmd(&s, FS_CMD_ARM));
  CHECK(find_ev(&s, FS_EV_ARM_REJECT, -1)->reason == FS_B_BATT);
  s.batt_v = 16.4f;
  s.kill_switch = true; /* switch active while disarmed -> KILLED (latching) */
  run_ms(&s, 10);
  CHECK(s.fs.state == FS_KILLED);
  s.kill_switch = false;
  run_ms(&s, 1000);
  CHECK(cmd(&s, FS_CMD_CLEAR_KILL));
  CHECK(s.fs.state == FS_DISARMED);
  run_ms(&s, 100);
  CHECK(cmd(&s, FS_CMD_ARM));
  CHECK(s.fs.state == FS_ARMED_IDLE);
  CHECK(!cmd(&s, FS_CMD_ARM)); /* already armed */
  /* auto-disarm after idle on the ground */
  int ms = run_until(&s, FS_DISARMED, 6000);
  CHECK(ms >= 4900 && ms <= 5100);
  e = find_ev(&s, FS_EV_TRANSITION, FS_DISARMED);
  CHECK(e && (s.ev[s.nev - 1].reason & FS_R_IDLE));
  /* throttle high at arm */
  s.want_fly = false;
  run_ms(&s, 100);
  s.fs.cfg.thr_arm_max = -1.0f; /* force "throttle not low" */
  s.nev = 0;
  CHECK(!cmd(&s, FS_CMD_ARM));
  CHECK(find_ev(&s, FS_EV_ARM_REJECT, -1)->reason & FS_B_THR);
}

static void t_pose_stall(void) {
  g_test = "pose_stall_hb_alive";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 1000);
  s.nev = 0;
  s.pose_on = false; /* phone keeps sending heartbeats */
  int ms = run_until(&s, FS_EMERG_LAND, 1000);
  CHECK(ms >= 150 && ms <= 260);
  const fs_event_t *e = find_ev(&s, FS_EV_TRANSITION, FS_EMERG_LAND);
  CHECK(e && (e->reason & FS_R_POSE_STALE) && !(e->reason & FS_R_HB_STALE));
  e = find_ev(&s, FS_EV_TIER, FS_TIER_IMU);
  CHECK(e && e->from == FS_TIER_NONE);
  CHECK(s.out.mode == FS_CTRL_LAND_LEVEL);
  ms = run_until(&s, FS_DISARMED, 20000);
  CHECK(ms > 0);
  CHECK(s.z < 0.01f);
  e = &s.ev[s.nev - 1];
  CHECK(e->code == FS_EV_TRANSITION && e->to == FS_DISARMED && (e->reason & FS_R_TOUCHDOWN));
  CHECK(!find_ev(&s, FS_EV_TRANSITION, FS_KILLED));
  printf("  pose stall: land triggered, IMU tier, touchdown after %d ms\n", ms);
}

static void t_hb_stall(void) {
  g_test = "hb_stall_pose_alive";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 1000);
  s.nev = 0;
  s.hb_on = false; /* pose keeps arriving */
  int ms = run_until(&s, FS_EMERG_LAND, 1000);
  CHECK(ms >= 200 && ms <= 310);
  const fs_event_t *e = find_ev(&s, FS_EV_TRANSITION, FS_EMERG_LAND);
  CHECK(e && (e->reason & FS_R_HB_STALE) && !(e->reason & FS_R_POSE_STALE));
  CHECK(find_ev(&s, FS_EV_TIER, FS_TIER_VIO)); /* pose still good -> land in place on VIO */
  CHECK(s.out.mode == FS_CTRL_LAND_HOLD_XY);
  ms = run_until(&s, FS_DISARMED, 20000);
  CHECK(ms > 0 && s.z < 0.01f);
  CHECK(s.ev[s.nev - 1].reason & FS_R_TOUCHDOWN);
  CHECK(count_ev(&s, FS_EV_TIER, -1) == 1); /* stayed VIO the whole way */
  printf("  hb stall: VIO tier, touchdown after %d ms (descent 1 m at %.2f m/s)\n", ms, s.fs.cfg.land_rate_mps);
}

static void t_both_stall(void) {
  g_test = "pose_and_hb_stall";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 1000);
  s.nev = 0;
  s.hb_on = false;
  s.pose_on = false;
  int ms = run_until(&s, FS_EMERG_LAND, 1000);
  CHECK(ms >= 150 && ms <= 260);
  CHECK(find_ev(&s, FS_EV_TIER, FS_TIER_IMU));
  CHECK(run_until(&s, FS_DISARMED, 20000) > 0);
}

static void t_hb_resume_no_autoresume(void) {
  g_test = "hb_resume_during_land";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 1000);
  s.hb_on = false;
  CHECK(run_until(&s, FS_EMERG_LAND, 1000) > 0);
  run_ms(&s, 500);
  s.hb_on = true; /* heartbeat back during landing */
  int ms = run_until(&s, FS_DISARMED, 20000);
  CHECK(ms > 0);
  CHECK(!find_ev(&s, FS_EV_TRANSITION, FS_FLYING) || find_ev(&s, FS_EV_TRANSITION, FS_FLYING) < find_ev(&s, FS_EV_TRANSITION, FS_EMERG_LAND));
  /* the only way up again: explicit re-arm on the ground */
  s.want_fly = false;
  run_ms(&s, 600);
  CHECK(cmd(&s, FS_CMD_ARM));
  CHECK(s.fs.state == FS_ARMED_IDLE);
}

static void t_hb_seq_stall_and_lost(void) {
  g_test = "hb_seq_stall";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.nev = 0;
  s.hb_freeze_seq = true; /* heartbeats keep coming, seq frozen */
  int ms = run_until(&s, FS_EMERG_LAND, 1000);
  CHECK(ms > 200 && ms < 400);
  const fs_event_t *e = find_ev(&s, FS_EV_TRANSITION, FS_EMERG_LAND);
  CHECK(e && (e->reason & FS_R_HB_STALE) && (e->reason & FS_R_HB_SEQ));

  g_test = "hb_state_lost";
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.nev = 0;
  s.hb_state = VIOP_HB_LOST;
  ms = run_until(&s, FS_EMERG_LAND, 1000);
  CHECK(ms >= 0 && ms <= 51); /* next heartbeat (20 Hz) */
  e = find_ev(&s, FS_EV_TRANSITION, FS_EMERG_LAND);
  CHECK(e && (e->reason & FS_R_HB_LOST));
  CHECK(find_ev(&s, FS_EV_TIER, FS_TIER_IMU)); /* phone says VIO lost -> pose not trusted */
  CHECK(run_until(&s, FS_DISARMED, 20000) > 0);

  g_test = "hb_app_stopping";
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.nev = 0;
  s.hb_flags = VIOP_HBF_STOPPING;
  CHECK(run_until(&s, FS_EMERG_LAND, 100) >= 0);
  CHECK(find_ev(&s, FS_EV_TRANSITION, FS_EMERG_LAND)->reason & FS_R_HB_LOST);

  g_test = "hb_flag_imu_stale";
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.nev = 0;
  s.hb_flags = VIOP_HBF_IMU_STALE; /* phone IMU feed stalled: predicted pose not updated */
  CHECK(run_until(&s, FS_EMERG_LAND, 100) >= 0);
  CHECK(find_ev(&s, FS_EV_TIER, FS_TIER_IMU));

  g_test = "phone_usb_disconnect";
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.nev = 0;
  fs_on_phone_connection(&s.fs, s.t, false);
  sim_tick(&s);
  CHECK(s.fs.state == FS_EMERG_LAND);
  CHECK(find_ev(&s, FS_EV_TRANSITION, FS_EMERG_LAND)->reason & FS_R_PHONE_DISC);
}

static void t_link_loss(void) {
  g_test = "link_loss";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.nev = 0;
  s.link_on = false;
  int ms = run_until(&s, FS_EMERG_LAND, 2000);
  CHECK(ms >= 400 && ms <= 610);
  CHECK(find_ev(&s, FS_EV_TRANSITION, FS_EMERG_LAND)->reason & FS_R_LINK);
  CHECK(find_ev(&s, FS_EV_TIER, FS_TIER_VIO)); /* phone still fine -> VIO tier */
  CHECK(run_until(&s, FS_DISARMED, 20000) > 0);
}

static void t_low_battery(void) {
  g_test = "low_battery";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.nev = 0;
  s.batt_v = 13.6f; /* 3.40 V/cell */
  int ms = run_until(&s, FS_EMERG_LAND, 5000);
  CHECK(ms >= 1990 && ms <= 2010);
  CHECK(find_ev(&s, FS_EV_TRANSITION, FS_EMERG_LAND)->reason & FS_R_BATT);
  CHECK(run_until(&s, FS_DISARMED, 20000) > 0);
  /* a short sag does not trigger */
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.batt_v = 13.6f;
  run_ms(&s, 1500);
  s.batt_v = 15.5f;
  run_ms(&s, 3000);
  CHECK(s.fs.state == FS_FLYING);
}

static void t_tilt_kill_and_clear(void) {
  g_test = "tilt_kill";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.tilt_deg = 65;
  sim_tick(&s);
  CHECK(s.fs.state == FS_KILLED);
  CHECK(!s.out.motors_enabled); /* same cycle */
  CHECK(find_ev(&s, FS_EV_TRANSITION, FS_KILLED)->reason & FS_R_TILT);
  /* latching: still killed later, even with everything normal again */
  s.tilt_deg = 0;
  run_ms(&s, 300);
  CHECK(s.fs.state == FS_KILLED && !s.out.motors_enabled);
  CHECK(!cmd(&s, FS_CMD_ARM));
  /* clear rules */
  s.nev = 0;
  CHECK(!cmd(&s, FS_CMD_CLEAR_KILL)); /* too soon */
  CHECK(find_ev(&s, FS_EV_CLEAR_REJECT, -1)->reason & FS_B_TOO_SOON);
  run_ms(&s, 1500);
  s.tilt_deg = 20;
  run_ms(&s, 10);
  CHECK(!cmd(&s, FS_CMD_CLEAR_KILL)); /* not level */
  s.tilt_deg = 0;
  s.kill_switch = true;
  run_ms(&s, 10);
  CHECK(!cmd(&s, FS_CMD_CLEAR_KILL)); /* switch still active */
  s.kill_switch = false;
  run_ms(&s, 300);
  CHECK(cmd(&s, FS_CMD_CLEAR_KILL));
  CHECK(s.fs.state == FS_DISARMED);
  s.want_fly = false;
  run_ms(&s, 100);
  CHECK(cmd(&s, FS_CMD_ARM));

  g_test = "tilt_kill_during_land";
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  CHECK(cmd(&s, FS_CMD_LAND));
  CHECK(s.fs.state == FS_EMERG_LAND);
  run_ms(&s, 200);
  s.tilt_deg = 50; /* above land limit 45, below normal 60 */
  sim_tick(&s);
  CHECK(s.fs.state == FS_KILLED && !s.out.motors_enabled);
}

static void t_kill_cmd_switch_nan(void) {
  g_test = "kill_cmd_midflight";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  CHECK(cmd(&s, FS_CMD_KILL));
  CHECK(s.fs.state == FS_KILLED);
  CHECK(s.fs.state == FS_KILLED);
  sim_tick(&s); /* the very next fs_step (same firmware cycle: queue drained before fs_step) outputs OFF */
  CHECK(!s.out.motors_enabled && s.out.mode == FS_CTRL_OFF);
  sim_tick(&s);
  CHECK(s.applied == 0.0f);
  CHECK(find_ev(&s, FS_EV_TRANSITION, FS_KILLED)->reason & FS_R_CMD);

  g_test = "kill_cmd_during_emerg_land";
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.pose_on = false;
  CHECK(run_until(&s, FS_EMERG_LAND, 1000) > 0);
  CHECK(cmd(&s, FS_CMD_KILL));
  sim_tick(&s);
  CHECK(s.fs.state == FS_KILLED && !s.out.motors_enabled);

  g_test = "kill_switch_midflight";
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.kill_switch = true;
  sim_tick(&s);
  CHECK(s.fs.state == FS_KILLED && !s.out.motors_enabled);
  CHECK(find_ev(&s, FS_EV_TRANSITION, FS_KILLED)->reason & FS_R_SWITCH);

  g_test = "nan_output";
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.nan_out = true;
  sim_tick(&s);
  CHECK(s.fs.state == FS_KILLED && !s.out.motors_enabled);
  CHECK(find_ev(&s, FS_EV_TRANSITION, FS_KILLED)->reason & FS_R_NAN);

  g_test = "loop_stall_request";
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  fs_request_kill(&s.fs, s.t, FS_R_LOOP_STALL);
  sim_tick(&s);
  CHECK(s.fs.state == FS_KILLED && !s.out.motors_enabled);

  g_test = "disarm_cmd_in_flight_rejected";
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  CHECK(!cmd(&s, FS_CMD_DISARM));
  CHECK(s.fs.state == FS_FLYING);
  CHECK(find_ev(&s, FS_EV_CMD_REJECT, -1));
}

static void t_overrun(void) {
  g_test = "loop_overrun";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.overrun = true; /* every cycle late */
  int ms = run_until(&s, FS_EMERG_LAND, 1000);
  CHECK(ms >= 45 && ms <= 60);
  CHECK(find_ev(&s, FS_EV_TRANSITION, FS_EMERG_LAND)->reason & FS_R_OVERRUN);
  /* occasional overruns (5 %) do not trigger */
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  for (int i = 0; i < 5000; i++) { s.overrun = (i % 20) == 0; sim_tick(&s); }
  s.overrun = false;
  CHECK(s.fs.state == FS_FLYING);
}

static void t_pose_reset(void) {
  g_test = "pose_reset_in_flight";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  s.reset_counter++;
  int ms = run_until(&s, FS_EMERG_LAND, 100);
  CHECK(ms >= 0 && ms <= 6);
  CHECK(find_ev(&s, FS_EV_TRANSITION, FS_EMERG_LAND)->reason & FS_R_POSE_RESET);
  CHECK(find_ev(&s, FS_EV_TIER, FS_TIER_IMU)); /* new world frame: VIO position not used */
}

static void t_hover_record(void) {
  g_test = "hover_record";
  sim_t s;
  sim_init(&s, NULL);
  run_ms(&s, 600);
  CHECK(cmd(&s, FS_CMD_ARM));
  fs_hover_rec_t r;
  fs_get_hover_record(&s.fs, &r);
  CHECK(!r.valid);
  s.want_fly = true;
  run_ms(&s, 6000);
  fs_get_hover_record(&s.fs, &r);
  CHECK(r.valid && r.n == 4);
  CHECK(find_ev(&s, FS_EV_HOVER_REC, -1));
  for (int i = 0; i < 4; i++) {
    float expect = 0.42f * (1.0f + k_motor_ofs[i]);
    CHECK(fabsf(r.m[i] - expect) < 0.01f);
  }
  printf("  hover record: m = %.4f %.4f %.4f %.4f (true %.4f %.4f %.4f %.4f), %u samples, frozen=%d\n", r.m[0], r.m[1], r.m[2],
         r.m[3], 0.42f * (1 + k_motor_ofs[0]), 0.42f * (1 + k_motor_ofs[1]), 0.42f * (1 + k_motor_ofs[2]),
         0.42f * (1 + k_motor_ofs[3]), r.samples, s.fs.rec_frozen);
  CHECK(s.fs.rec_frozen);
  /* frozen: a later different hover (e.g. payload change) does not alter this flight's takeoff record */
  s.hover_true = 0.50f;
  run_ms(&s, 3000);
  fs_hover_rec_t r2;
  fs_get_hover_record(&s.fs, &r2);
  CHECK(memcmp(&r, &r2, sizeof(r)) == 0);
  /* reset at the next arming */
  s.hover_true = 0.42f;
  CHECK(cmd(&s, FS_CMD_LAND));
  CHECK(run_until(&s, FS_DISARMED, 20000) > 0);
  s.want_fly = false;
  run_ms(&s, 600);
  CHECK(cmd(&s, FS_CMD_ARM));
  fs_hover_rec_t r3;
  fs_get_hover_record(&s.fs, &r3);
  CHECK(!r3.valid);

  g_test = "hover_persist_prev_flight";
  sim_t s2;
  sim_init(&s2, NULL);
  fs_set_prev_hover(&s2.fs, &r); /* e.g. loaded from NVS at boot */
  run_ms(&s2, 600);
  CHECK(cmd(&s2, FS_CMD_ARM));
  s2.want_fly = true;
  run_ms(&s2, 150); /* emergency in the very first moments: no record of this flight yet */
  CHECK(s2.fs.state == FS_FLYING);
  s2.imu_ok = false;
  sim_tick(&s2);
  CHECK(s2.fs.state == FS_EMERG_LAND && s2.fs.tier == FS_TIER_OPENLOOP);
  const fs_event_t *e = find_ev(&s2, FS_EV_TIER, FS_TIER_OPENLOOP);
  CHECK(e && e->aux == FS_HOVER_PREV_FLIGHT);
  CHECK(fabsf(s2.out.motor[1] - r.m[1] * sqrtf(1.0f - 1.0f / FC_G)) < 0.005f); /* phase 1: thrust 1 - acc_down/g */

  g_test = "hover_missing_default";
  sim_init(&s2, NULL);
  fs_hover_rec_t bad = r;
  bad.m[2] = NAN; /* corrupted persisted value is rejected */
  fs_set_prev_hover(&s2.fs, &bad);
  run_ms(&s2, 600);
  CHECK(cmd(&s2, FS_CMD_ARM));
  s2.want_fly = true;
  run_ms(&s2, 150);
  s2.imu_ok = false;
  sim_tick(&s2);
  e = find_ev(&s2, FS_EV_TIER, FS_TIER_OPENLOOP);
  CHECK(e && e->aux == FS_HOVER_DEFAULT);
  for (int i = 0; i < 4; i++)
    CHECK(fabsf(s2.out.motor[i] - s2.fs.cfg.openloop_default_thr * sqrtf(1.0f - 1.0f / FC_G)) < 1e-4f);
}

static void t_openloop_land(void) {
  g_test = "imu_fail_openloop_this_flight";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 2000);
  fs_hover_rec_t r;
  fs_get_hover_record(&s.fs, &r);
  CHECK(r.valid);
  s.nev = 0;
  s.imu_ok = false; /* IMU dies: no attitude stabilisation possible */
  sim_tick(&s);
  CHECK(s.fs.state == FS_EMERG_LAND && s.fs.tier == FS_TIER_OPENLOOP && s.out.mode == FS_CTRL_OPENLOOP);
  const fs_event_t *e = find_ev(&s, FS_EV_TIER, FS_TIER_OPENLOOP);
  CHECK(e && e->from == FS_TIER_NONE && e->aux == FS_HOVER_THIS_FLIGHT && (e->reason & FS_R_IMU));
  for (int i = 0; i < 4; i++) CHECK(fabsf(s.out.motor[i] - r.m[i] * sqrtf(1.0f - 1.0f / FC_G)) < 1e-4f);
  /* per-motor ratios preserved */
  CHECK(s.out.motor[2] > s.out.motor[0]);
  int ms = run_until(&s, FS_DISARMED, 10000);
  CHECK(ms > 0);
  CHECK(s.ev[s.nev - 1].reason & (FS_R_IMPACT | FS_R_LAND_TIMEOUT));
  printf("  open-loop: motors cut after %d ms by %s, z=%.2f\n", ms, (s.ev[s.nev - 1].reason & FS_R_IMPACT) ? "impact" : "timeout", s.z);
  CHECK(s.z < 0.01f);

  g_test = "imu_fail_kill_option";
  fs_config_t c;
  fs_config_default(&c);
  c.imu_fail_kill = true;
  sim_init(&s, &c);
  fly_to_hover(&s, 500);
  s.imu_ok = false;
  sim_tick(&s);
  CHECK(s.fs.state == FS_KILLED);
}

static void t_tier_downgrade(void) {
  g_test = "tier_downgrade_no_upgrade";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 2000);
  s.nev = 0;
  CHECK(cmd(&s, FS_CMD_LAND));
  CHECK(s.fs.tier == FS_TIER_VIO);
  run_ms(&s, 500);
  s.pose_on = false; /* VIO lost mid-landing */
  run_ms(&s, 300);
  CHECK(s.fs.state == FS_EMERG_LAND && s.fs.tier == FS_TIER_IMU);
  s.pose_on = true; /* VIO back: must NOT step up */
  run_ms(&s, 500);
  CHECK(s.fs.tier == FS_TIER_IMU);
  s.imu_ok = false; /* IMU lost too */
  sim_tick(&s);
  CHECK(s.fs.tier == FS_TIER_OPENLOOP);
  s.imu_ok = true;
  run_ms(&s, 200);
  CHECK(s.fs.state != FS_EMERG_LAND || s.fs.tier == FS_TIER_OPENLOOP);
  CHECK(count_ev(&s, FS_EV_TIER, -1) == 3); /* NONE->VIO, VIO->IMU, IMU->OPENLOOP */
  const fs_event_t *e = find_ev(&s, FS_EV_TIER, FS_TIER_IMU);
  CHECK(e && e->from == FS_TIER_VIO && (e->reason & (FS_R_POSE_STALE | FS_R_POSE_BAD)));
  CHECK(run_until(&s, FS_DISARMED, 10000) >= 0);
}

static void t_imu_tier_no_vz(void) {
  g_test = "imu_tier_without_vz_ramp";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 2000);
  s.pose_on = false;
  s.imu_vz_on = false; /* estimator horizon expired too: timed collective ramp */
  CHECK(run_until(&s, FS_EMERG_LAND, 1000) > 0);
  CHECK(s.fs.tier == FS_TIER_IMU);
  int ms = run_until(&s, FS_DISARMED, 31000);
  CHECK(ms > 0 && s.z < 0.01f);
  printf("  IMU tier w/o vz: touchdown after %d ms (reason 0x%x)\n", ms, s.ev[s.nev - 1].reason);
}

static void t_normal_landing(void) {
  g_test = "normal_landing_then_idle_disarm";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 1000);
  s.z_target = -0.5f; /* controller lands normally */
  int ms = run_until(&s, FS_ARMED_IDLE, 10000);
  CHECK(ms > 0);
  CHECK(s.ev[s.nev - 1].reason & FS_R_TOUCHDOWN);
  s.want_fly = false;
  CHECK(run_until(&s, FS_DISARMED, 6000) > 0);
}

static void t_land_cmd_ground(void) {
  g_test = "land_trigger_on_ground_disarms";
  sim_t s;
  sim_init(&s, NULL);
  run_ms(&s, 600);
  CHECK(cmd(&s, FS_CMD_ARM));
  s.pose_on = false;
  CHECK(run_until(&s, FS_DISARMED, 1000) > 0);
  CHECK(!find_ev(&s, FS_EV_TRANSITION, FS_EMERG_LAND));
}

/* ---------------- regression tests for the independent review (2026-10-25) ---------------- */
static void t_review_regressions(void) {
  g_test = "future_timestamp_no_false_land";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  { /* messages stamped 1 ms after the loop's now (link task raced the control loop) */
    fs_pose_info_t p = {.seq = (uint16_t)(s.pose_seq + 100), .flags = VIOP_F_VALID | VIOP_F_TRACKING, .quality = 90,
                        .reset_counter = s.reset_counter, .pos_std_max_m = 0.02f, .send_delay_us = 3000};
    s.pose_seq = (uint16_t)(p.seq + 1);
    fs_on_pose(&s.fs, s.t + 1, &p);
    fs_on_heartbeat(&s.fs, s.t + 1, s.hb_seq++, VIOP_HB_TRACKING, 0);
    fs_on_link(&s.fs, s.t + 1);
  }
  run_ms(&s, 20);
  CHECK(s.fs.state == FS_FLYING);

  g_test = "clear_kill_cmd_time_before_kill";
  fs_request_kill(&s.fs, s.t, FS_R_CMD);
  CHECK(!fs_command(&s.fs, s.t - 5, FS_CMD_CLEAR_KILL)); /* earlier time must not bypass kill_clear_min_ms */
  drain(&s);

  g_test = "ground_spinup_not_recorded";
  sim_init(&s, NULL);
  s.hover_true = 0.42f;
  run_ms(&s, 600);
  CHECK(cmd(&s, FS_CMD_ARM));
  s.hover_true = 10.0f; /* motors cannot lift: sits on the ground at thr 0.5 (FLYING state, no climb) */
  s.want_fly = true;
  run_ms(&s, 2000);
  fs_hover_rec_t r;
  fs_get_hover_record(&s.fs, &r);
  CHECK(!r.valid && !s.fs.airborne);

  g_test = "nan_vz_during_land_no_kill";
  sim_init(&s, NULL);
  fly_to_hover(&s, 1000);
  CHECK(cmd(&s, FS_CMD_LAND));
  run_ms(&s, 100);
  s.in.vz_vio = NAN; /* inject directly: one NaN sample */
  {
    fs_input_t in = s.in;
    in.now_ms = ++s.t;
    in.vz_vio = NAN;
    in.vz_imu = NAN;
    fs_step(&s.fs, &in, &s.out);
    drain(&s);
  }
  CHECK(s.fs.state == FS_EMERG_LAND && s.out.thr == s.out.thr && s.fs.land_i == s.fs.land_i);
  CHECK(run_until(&s, FS_DISARMED, 20000) > 0);

  g_test = "no_retakeoff_after_touchdown";
  sim_init(&s, NULL);
  fly_to_hover(&s, 1000);
  s.z_target = -0.5f;
  CHECK(run_until(&s, FS_ARMED_IDLE, 10000) > 0);
  /* controller still asks 0.2 (stale setpoint): must NOT go FLYING again */
  for (int i = 0; i < 50; i++) {
    fs_input_t in = s.in;
    in.now_ms = ++s.t;
    in.thr_cmd = 0.2f;
    fs_step(&s.fs, &in, &s.out);
  }
  drain(&s);
  CHECK(s.fs.state == FS_ARMED_IDLE);

  g_test = "arm_needs_known_battery";
  sim_init(&s, NULL);
  run_ms(&s, 600);
  {
    fs_input_t in = s.in;
    in.now_ms = ++s.t;
    in.batt_valid = false;
    fs_step(&s.fs, &in, &s.out);
  }
  s.nev = 0;
  CHECK(!cmd(&s, FS_CMD_ARM));
  CHECK(find_ev(&s, FS_EV_ARM_REJECT, -1)->reason & FS_B_BATT);
  {
    fs_input_t in = s.in;
    in.now_ms = ++s.t;
    in.batt_v = NAN;
    fs_step(&s.fs, &in, &s.out);
  }
  CHECK(!cmd(&s, FS_CMD_ARM));

  g_test = "frozen_seq_blocks_arm";
  sim_init(&s, NULL);
  s.hb_freeze_seq = true;
  run_ms(&s, 600);
  s.nev = 0;
  CHECK(!cmd(&s, FS_CMD_ARM));
  CHECK(find_ev(&s, FS_EV_ARM_REJECT, -1)->reason & FS_B_HB);

  g_test = "nan_inputs_block_arm";
  sim_init(&s, NULL);
  run_ms(&s, 600);
  {
    fs_input_t in = s.in;
    in.now_ms = ++s.t;
    in.thr_cmd = NAN;
    fs_step(&s.fs, &in, &s.out);
  }
  CHECK(!cmd(&s, FS_CMD_ARM));
  {
    fs_input_t in = s.in;
    in.now_ms = ++s.t;
    in.gyro_norm_rps = NAN;
    fs_step(&s.fs, &in, &s.out);
  }
  CHECK(!cmd(&s, FS_CMD_ARM));

  g_test = "clear_kill_refused_in_free_fall";
  sim_init(&s, NULL);
  run_ms(&s, 600);
  CHECK(cmd(&s, FS_CMD_KILL));
  for (int i = 0; i < 1500; i++) {
    fs_input_t in = s.in;
    in.now_ms = ++s.t;
    in.acc_norm = 0.05f; /* falling */
    fs_step(&s.fs, &in, &s.out);
  }
  drain(&s);
  CHECK(!cmd(&s, FS_CMD_CLEAR_KILL));

  g_test = "land_timeout_steps_down_to_openloop";
  fs_config_t c;
  fs_config_default(&c);
  c.land_max_ms = 1000;
  sim_init(&s, &c);
  fly_to_hover(&s, 3000); /* 1 m at 0.3 m/s needs ~3.5 s > land_max_ms 1 s */
  CHECK(cmd(&s, FS_CMD_LAND));
  run_ms(&s, 1010);
  CHECK(s.fs.state == FS_EMERG_LAND && s.fs.tier == FS_TIER_OPENLOOP);
  const fs_event_t *e = find_ev(&s, FS_EV_TIER, FS_TIER_OPENLOOP);
  CHECK(e && (e->reason & FS_R_LAND_TIMEOUT));
  CHECK(run_until(&s, FS_DISARMED, 8000) >= 0);
}

/* ---------------- phone IMU = only IMU (user decision 2026-10-25) ---------------- */
/* esp32fix 2026-10-26: open-loop cut (phone IMU + VIO lost) over heights x hover-record thrust error.
 * Fly to z0, record hover, then change the true hover so that the recorded commands give m x weight thrust. */
typedef struct { int t_ground, t_cut, ms_hi; float vimp, z_cut; uint32_t reason; } ol_res_t;
static ol_res_t ol_case(const fs_config_t *c, float z0, float m, bool keep_pose) {
  sim_t s;
  sim_init(&s, c);
  s.z_target = z0;
  run_ms(&s, 600);
  CHECK(cmd(&s, FS_CMD_ARM));
  s.want_fly = true;
  run_ms(&s, 5000);
  CHECK(s.fs.state == FS_FLYING && fabsf(s.z - z0) < 0.1f && s.fs.rec.valid);
  /* sim thrust = (col / hover_true)^2: hover_true / sqrt(m) scales the thrust of every command by m */
  s.hover_true = 0.42f / sqrtf(m);
  s.nev = 0;
  s.imu_ok = s.acc_ok = false;
  if (!keep_pose) s.pose_on = s.hb_on = false;
  ol_res_t r = {-1, -1, 0, 0.0f, -1.0f, 0};
  for (int i = 0; i < 9000; i++) {
    float vz_before = s.vz;
    sim_tick(&s);
    if (r.t_ground < 0 && s.z <= 0.0f) { r.t_ground = i; r.vimp = -vz_before; }
    if (r.t_ground >= 0 && s.out.motors_enabled) {
      float tr = (s.applied / s.hover_true) * (s.applied / s.hover_true);
      if (tr > 0.85f) r.ms_hi++;
    }
    if (s.fs.state == FS_DISARMED) { r.t_cut = i; r.z_cut = s.z; r.reason = s.ev[s.nev - 1].reason; break; }
  }
  if (r.t_ground < 0 && r.z_cut > 0) r.vimp = sqrtf(s.vz * s.vz + 2.0f * FC_G * r.z_cut); /* falls after the cut */
  CHECK(g_illegal == 0);
  return r;
}

static void t_openloop_cut(void) {
  static const float hs[] = {0.3f, 1.0f, 2.0f};
  static const float ms[] = {0.97f, 1.0f, 1.03f};
  for (int legacy = 1; legacy >= 0; legacy--) {
    fs_config_t c;
    fs_config_default(&c);
    if (legacy) { c.openloop_cut_mode = 0; c.openloop_vz_down = 0.4f; c.openloop_cruise_ratio = 0.99f; }
    float w_v = 0, w_zcut = 0;
    int w_hi = 0, w_spin = 0;
    for (int a = 0; a < 3; a++)
      for (int b = 0; b < 3; b++) {
        g_test = legacy ? "openloop_cut_legacy_sweep" : "openloop_cut_mode1_sweep";
        ol_res_t r = ol_case(&c, hs[a], ms[b], false);
        int spin = r.t_ground >= 0 ? r.t_cut - r.t_ground : 0;
        printf("  %s h %.1f m thrust x%.2f: ground %d ms, cut %d ms (%s), on-ground spin %d ms (>85%% hover %d ms), "
               "impact %.2f m/s%s\n", legacy ? "legacy" : "mode1 ", (double)hs[a], (double)ms[b], r.t_ground, r.t_cut,
               (r.reason & FS_R_TOUCHDOWN) ? "touch" : "timer", spin, r.ms_hi, (double)r.vimp,
               r.z_cut > 0.01f ? " CUT IN THE AIR" : "");
        CHECK(r.t_cut > 0);
        if (r.vimp > w_v) w_v = r.vimp;
        if (r.z_cut > w_zcut) w_zcut = r.z_cut;
        if (r.ms_hi > w_hi) w_hi = r.ms_hi;
        if (spin > w_spin) w_spin = spin;
        if (!legacy) {
          CHECK(r.t_ground >= 0 && r.z_cut < 0.01f); /* never cut in the air within +-3 % record error */
          CHECK(r.ms_hi <= 250);                     /* <= 0.25 s on the ground above 85 % hover thrust */
          CHECK(spin <= 1500);
          CHECK(r.vimp < 3.0f);
        }
      }
    printf("  %s worst: impact %.2f m/s, cut height %.2f m, on-ground spin %d ms, >85%% hover on ground %d ms\n",
           legacy ? "legacy" : "mode1 ", (double)w_v, (double)w_zcut, w_spin, w_hi);
    if (legacy) CHECK(w_zcut > 0.5f && w_hi > 1500); /* documents the legacy failure modes */
  }

  g_test = "openloop_vio_touchdown";
  fs_config_t c;
  fs_config_default(&c);
  ol_res_t r = ol_case(&c, 1.0f, 1.0f, true); /* IMU frames stop, poses + heartbeat continue */
  printf("  mode1 + VIO: ground %d ms, cut %d ms (%s), impact %.2f m/s\n", r.t_ground, r.t_cut,
         (r.reason & FS_R_TOUCHDOWN) ? "touch" : "timer", (double)r.vimp);
  CHECK(r.t_ground > 0 && (r.reason & FS_R_TOUCHDOWN) && r.t_cut - r.t_ground >= 150 && r.t_cut - r.t_ground <= 300);
  c.openloop_vio_touch = false;
  r = ol_case(&c, 1.0f, 1.0f, true);
  CHECK(r.t_ground > 0 && (r.reason & FS_R_LAND_TIMEOUT) && !(r.reason & FS_R_TOUCHDOWN));
}

static bool elapsed_ms_lt(const sim_t *s, uint32_t ms) { return s->t - s->fs.t_tier_ms < ms; }
static bool elapsed_ms_ge(const sim_t *s, uint32_t ms) { return s->t - s->fs.t_tier_ms >= ms; }

static void t_phone_imu(void) {
  g_test = "phone_imu_stall_openloop_backcalc";
  sim_t s;
  sim_init(&s, NULL);
  fly_to_hover(&s, 2000);
  fs_hover_rec_t r;
  fs_get_hover_record(&s.fs, &r);
  CHECK(r.valid && fabsf(r.batt_v - 16.4f) < 0.01f);
  s.nev = 0;
  s.imu_ok = false; /* phone IMU stream lost (USB / phone): no gyro, no accel, and (USB) no pose / heartbeat either */
  s.acc_ok = false;
  s.pose_on = s.hb_on = false; /* esp32fix: whole phone link gone (with poses on, mode 1 would cut on VIO touchdown) */
  sim_tick(&s);
  CHECK(s.fs.state == FS_EMERG_LAND && s.fs.tier == FS_TIER_OPENLOOP);
  const fs_event_t *e = find_ev(&s, FS_EV_TIER, FS_TIER_OPENLOOP);
  CHECK(e && e->from == FS_TIER_NONE && e->aux == FS_HOVER_THIS_FLIGHT);
  /* mode 1 (default): cut at the slow (+3 % thrust) touchdown time from the last VIO height (~1 m) + 0.2 s */
  CHECK(s.fs.ol_ramp && s.fs.ol_fast_ms > 1000 && s.fs.ol_fast_ms < s.fs.ol_slow_ms);
  CHECK(s.fs.ol_cut_ms == s.fs.ol_slow_ms + 200 && s.fs.ol_cut_ms < 3000);
  int ms = -1, t_ground = -1;
  float vz_min = 0;
  for (int i = 0; i < 8000; i++) {
    if (s.fs.state == FS_DISARMED) { ms = i; break; }
    sim_tick(&s);
    if (s.vz < vz_min) vz_min = s.vz;
    if (t_ground < 0 && s.z <= 0.0f) t_ground = i;
  }
  CHECK(ms > 0 && (s.ev[s.nev - 1].reason & FS_R_LAND_TIMEOUT)); /* no accelerometer: no impact detection */
  CHECK(s.z < 0.01f && t_ground > 0 && t_ground < ms);
  CHECK(ms - t_ground < 1000); /* esp32fix: legacy spun ~1670 ms on the ground at ~98 % hover thrust */
  printf("  phone-IMU loss (1 m): ground contact after %d ms at up to %.2f m/s, motors cut after %d ms (cut time %u ms)\n",
         t_ground, (double)-vz_min, ms, s.fs.ol_cut_ms);

  g_test = "openloop_phase2_and_volt_comp";
  sim_init(&s, NULL);
  fly_to_hover(&s, 2000);
  fs_get_hover_record(&s.fs, &r);
  s.batt_v = 15.0f; /* battery sagged since the takeoff record (16.4 V) */
  run_ms(&s, 5);
  s.imu_ok = s.acc_ok = false;
  sim_tick(&s);
  float vc = 16.4f / 15.0f;
  CHECK(fabsf(s.fs.ol_vcomp - vc) < 0.01f);
  CHECK(fabsf(s.out.motor[0] - r.m[0] * sqrtf(1.0f - 1.0f / FC_G) * vc) < 0.002f);
  /* after vz_down / acc_down (0.6 s with the esp32fix defaults): cruise just under hover, before the end ramp (t_fast) */
  run_ms(&s, (uint32_t)(1000.0f * s.fs.cfg.openloop_vz_down / s.fs.cfg.openloop_acc_down) + 100);
  CHECK(s.fs.state == FS_EMERG_LAND && s.fs.ol_ramp && elapsed_ms_lt(&s, s.fs.ol_fast_ms));
  if (s.fs.state == FS_EMERG_LAND) CHECK(fabsf(s.out.motor[0] - r.m[0] * sqrtf(s.fs.cfg.openloop_cruise_ratio) * vc) < 0.002f);
  /* after the end ramp: openloop_end_ratio */
  while (s.fs.state == FS_EMERG_LAND && !elapsed_ms_ge(&s, s.fs.ol_fast_ms + 400 + 10)) sim_tick(&s);
  if (s.fs.state == FS_EMERG_LAND) CHECK(fabsf(s.out.motor[0] - r.m[0] * sqrtf(s.fs.cfg.openloop_end_ratio) * vc) < 0.002f);

  g_test = "imu_fail_kill_option_still_works";
  fs_config_t c;
  fs_config_default(&c);
  c.imu_fail_kill = true;
  sim_init(&s, &c);
  fly_to_hover(&s, 500);
  s.imu_ok = s.acc_ok = false;
  sim_tick(&s);
  CHECK(s.fs.state == FS_KILLED);

  g_test = "imu_stream_jitter";
  imu_stream_t st;
  imu_stream_init(&st, 10000, 0.002f);
  float dt;
  int64_t rx = 1000000;
  CHECK(imu_stream_accept(&st, 5000000, 0, rx, &dt));
  /* burst: 3 samples (2 ms apart on the phone) arrive in the same USB transfer -> dt from phone time */
  CHECK(imu_stream_accept(&st, 5002000, 1, rx + 6000, &dt) && fabsf(dt - 0.002f) < 1e-6f);
  CHECK(imu_stream_accept(&st, 5004000, 2, rx + 6000, &dt) && fabsf(dt - 0.002f) < 1e-6f);
  CHECK(imu_stream_accept(&st, 5006000, 3, rx + 6000, &dt) && fabsf(dt - 0.002f) < 1e-6f);
  CHECK(imu_stream_ok(&st, rx + 6000));
  CHECK(!imu_stream_accept(&st, 5006000, 3, rx + 6100, &dt)); /* duplicate */
  CHECK(!imu_stream_accept(&st, 5004000, 2, rx + 6100, &dt)); /* out of order */
  CHECK(st.n_dup == 1 && st.n_old == 1);
  CHECK(imu_stream_ok(&st, rx + 15900));  /* 9.9 ms of local silence: still ok */
  CHECK(!imu_stream_ok(&st, rx + 16100)); /* > 10 ms: stale -> failsafe */
  /* phone-time gap of 12 ms (sample drop on the phone) */
  CHECK(imu_stream_accept(&st, 5018000, 4, rx + 18000, &dt));
  CHECK(!imu_stream_ok(&st, rx + 18000) && st.n_gap == 1);
  CHECK(fabsf(dt - 0.004f) < 1e-6f); /* long hole never integrated in one step (clamped to 2 x nominal) */
  CHECK(imu_stream_accept(&st, 5020000, 6, rx + 20000, &dt) && imu_stream_ok(&st, rx + 20000) && st.n_seq_skip == 1);
  CHECK(imu_stream_accept(&st, 1000, 0, rx + 22000, &dt) && st.gap_flag); /* app restart: resync, flagged */

  g_test = "imu_stream_drives_fs";
  sim_init(&s, NULL);
  fly_to_hover(&s, 500);
  imu_stream_init(&st, 10000, 0.002f);
  int64_t tp = 0, tl = 0;
  for (int i = 0; i < 40; i++) { tp += 2000; tl += 2000; imu_stream_accept(&st, tp, (uint16_t)i, tl, &dt); }
  s.imu_ok = imu_stream_ok(&st, tl + 1000);
  run_ms(&s, 1);
  CHECK(s.fs.state == FS_FLYING);
  s.imu_ok = imu_stream_ok(&st, tl + 11000); /* USB stalled 11 ms */
  s.acc_ok = s.imu_ok;
  sim_tick(&s);
  CHECK(s.fs.state == FS_EMERG_LAND && s.fs.tier == FS_TIER_OPENLOOP);

  g_test = "viop_imu_decode";
  viop_imu_t m;
  memset(&m, 0, sizeof(m));
  m.t0_us = 123456789;
  m.n = 3;
  m.flags = VIOP_IMUF_BIAS_CORR;
  m.sample_seq0 = 65535;
  for (int i = 0; i < 3; i++) {
    m.s[i].dt_us = (uint16_t)(2000 * i);
    m.s[i].gyro[0] = 1000; m.s[i].gyro[1] = -500; m.s[i].gyro[2] = (int16_t)(10 * i);
    m.s[i].acc[0] = 0; m.s[i].acc[1] = 1961; m.s[i].acc[2] = -200;
  }
  uint8_t fr[VIOP_MAX_FRAME];
  int n = viop_build(fr, VIOP_T_IMU, VIOP_SRC_PHONE, 1, &m, viop_imu_payload_len(3));
  CHECK(n == 8 + 12 + 42 + 2 && viop_check(fr, n) == 0);
  viop_imu_dec_t d[VIOP_IMU_MAX_N];
  CHECK(viop_imu_decode(fr + VIOP_HDR, fr[4], d) == 3);
  CHECK(d[2].t_us == 123456789 + 4000 && d[1].seq == 0 && fabsf(d[0].gyro[0] - 1.0f) < 1e-6f && fabsf(d[0].acc[1] - 9.805f) < 1e-3f);
  CHECK(viop_imu_decode(fr + VIOP_HDR, fr[4] - 1, d) == 0); /* length mismatch */
  int n5 = viop_build(fr, VIOP_T_IMU, VIOP_SRC_PHONE, 2, &m, viop_imu_payload_len(5));
  CHECK(n5 == 92 && n5 <= VIOP_MAX_FRAME); /* 5 samples still fit one ESP-NOW-relayable frame */

  g_test = "pwm_output_limits";
  CHECK(fc_pwm_pulse_us(0.0f, 1000, 2000) == 1000.0f);
  CHECK(fc_pwm_pulse_us(-0.3f, 1000, 2000) == 1000.0f);
  CHECK(fc_pwm_pulse_us(NAN, 1000, 2000) == 1000.0f); /* NaN -> motor stop */
  CHECK(fc_pwm_pulse_us(1.7f, 1000, 2000) == 2000.0f);
  CHECK(fabsf(fc_pwm_pulse_us(0.42f, 1000, 2000) - 1420.0f) < 0.01f);
  CHECK(fc_pwm_duty(1000, 400, 14) == 6553); /* 1000 us of a 2500 us period, 14 bit */
  CHECK(fc_pwm_duty(2000, 400, 14) == 13106);
  CHECK(fc_pwm_duty(5000, 400, 14) == fc_pwm_duty(2480, 400, 14)); /* clamped: >= 20 us low time */
  CHECK(fc_pwm_config_ok(50, 2000) && fc_pwm_config_ok(400, 2000) && fc_pwm_config_ok(490, 2000));
  CHECK(!fc_pwm_config_ok(500, 2000)); /* 2000 us pulse does not fit a 2000 us period */
  CHECK(fc_pwm_config_ok(2000, 250) && fabsf(fc_pwm_pulse_us(1.0f, 125, 250) - 250.0f) < 1e-3f); /* OneShot125 */
}

/* ---------------- fc_core sanity ("known answer" tests) ---------------- */
static void t_core(void) {
  g_test = "att_est_static_tilt";
  att_est_t e;
  att_est_init(&e, 1.0f, 0.2f, 0.5f);
  float roll = 20.0f * 0.0174533f, pitch = -10.0f * 0.0174533f;
  /* specific force at rest, FRD: acc = -g * (-sin p, sin r cos p, cos r cos p) */
  float acc[3] = {FC_G * sinf(pitch), -FC_G * sinf(roll) * cosf(pitch), -FC_G * cosf(roll) * cosf(pitch)};
  float gyro[3] = {0.01f, -0.02f, 0.005f}; /* constant bias to be learnt */
  float lvl[3] = {0, 0, -FC_G};
  att_est_update(&e, gyro, lvl, 0.001f); /* init level (wrong on purpose) */
  for (int i = 0; i < 20000; i++) att_est_update(&e, gyro, acc, 0.001f);
  float r, p, y;
  att_est_euler(&e, &r, &p, &y);
  CHECK(fabsf(r - roll) < 0.01f && fabsf(p - pitch) < 0.01f);
  { /* bias is observable only perpendicular to gravity (the part along "down" = yaw rate is not) */
    float Rm[9];
    att_est_rot(&e, Rm);
    float dv[3] = {Rm[6], Rm[7], Rm[8]}, be[3] = {e.bias[0] - gyro[0], e.bias[1] - gyro[1], e.bias[2] - gyro[2]};
    float d = be[0] * dv[0] + be[1] * dv[1] + be[2] * dv[2];
    float perp = sqrtf((be[0] - d * dv[0]) * (be[0] - d * dv[0]) + (be[1] - d * dv[1]) * (be[1] - d * dv[1]) +
                       (be[2] - d * dv[2]) * (be[2] - d * dv[2]));
    CHECK(perp < 0.002f);
  }
  printf("  att_est: roll %.2f deg (20), pitch %.2f deg (-10), bias %.4f %.4f\n", r / 0.0174533f, p / 0.0174533f, e.bias[0], e.bias[1]);

  g_test = "vz_est_bias_and_horizon";
  vz_est_t v;
  vz_est_init(&v, 2.0f, 0.5f, 5.0f, 8.0f);
  float R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float a_b[3] = {0, 0, -FC_G - 0.3f}; /* accelerometer bias 0.3 m/s^2 (reads "up accel") while hovering */
  for (int i = 0; i < 30000; i++) { vz_est_predict(&v, R, a_b, 0.001f); if (i % 5 == 0) vz_est_correct(&v, 0.0f, 0.005f); }
  CHECK(fabsf(v.bias - 0.3f) < 0.02f && fabsf(v.vz) < 0.02f && vz_est_valid(&v));
  for (int i = 0; i < 3000; i++) vz_est_predict(&v, R, a_b, 0.001f); /* VIO gone 3 s */
  CHECK(fabsf(v.vz) < 0.1f && vz_est_valid(&v));
  for (int i = 0; i < 6000; i++) vz_est_predict(&v, R, a_b, 0.001f);
  CHECK(!vz_est_valid(&v));

  g_test = "mixer";
  float m[4], tq0[3] = {0, 0, 0}, tqr[3] = {0.1f, 0, 0}, tqbig[3] = {0.8f, 0.8f, 0.5f};
  mixer_quad_x(0.4f, tq0, 0.05f, m);
  CHECK(m[0] == 0.4f && m[1] == 0.4f && m[2] == 0.4f && m[3] == 0.4f);
  mixer_quad_x(0.4f, tqr, 0.05f, m);
  CHECK(m[0] < 0.4f && m[1] < 0.4f && m[2] > 0.4f && m[3] > 0.4f); /* roll right: right motors slower */
  mixer_quad_x(0.9f, tqbig, 0.05f, m);
  for (int i = 0; i < 4; i++) CHECK(m[i] >= 0.05f && m[i] <= 1.0f && m[i] == m[i]);

  g_test = "viop_glue";
  viop_pose_t vp;
  memset(&vp, 0, sizeof(vp));
  vp.pos_std_mm[0] = 10; vp.pos_std_mm[1] = 250; vp.pos_std_mm[2] = 30;
  vp.flags = VIOP_F_VALID | VIOP_F_TRACKING; vp.quality = 50; vp.send_delay_us = 1234;
  fs_pose_info_t pi;
  fs_pose_info_from_viop(&vp, 77, &pi);
  CHECK(pi.seq == 77 && fabsf(pi.pos_std_max_m - 0.25f) < 1e-6f && pi.send_delay_us == 1234);
  uint8_t fr[VIOP_MAX_FRAME];
  viop_heartbeat_t hb = {.t_phone_us = 1, .state = VIOP_HB_TRACKING, .hb_period_ms = 50};
  int n = viop_build(fr, VIOP_T_HEARTBEAT, VIOP_SRC_PHONE, 5, &hb, sizeof(hb));
  CHECK(n == 28 && viop_check(fr, n) == 0);
}

int main(void) {
  t_transition_table();
  t_arm_rules();
  t_pose_stall();
  t_hb_stall();
  t_both_stall();
  t_hb_resume_no_autoresume();
  t_hb_seq_stall_and_lost();
  t_link_loss();
  t_low_battery();
  t_tilt_kill_and_clear();
  t_kill_cmd_switch_nan();
  t_overrun();
  t_pose_reset();
  t_hover_record();
  t_openloop_land();
  t_tier_downgrade();
  t_imu_tier_no_vz();
  t_normal_landing();
  t_land_cmd_ground();
  t_review_regressions();
  t_phone_imu();
  t_openloop_cut();
  t_core();
  g_test = "no_illegal_transitions";
  CHECK(g_illegal == 0);
  printf("%s: %d checks, %d failed\n", g_fail ? "FAILED" : "ALL PASS", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
