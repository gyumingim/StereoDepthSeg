/* ctrl_task.c — the 1 kHz (FC_CTRL_HZ) loop on core 1: IMU -> attitude estimate -> safety -> rate control -> mixer ->
 * motors, all in one task so a KILL decided in a cycle reaches the ESCs in the same cycle.
 * Rules inside the loop: no malloc, no logging, no blocking calls (queues with timeout 0), float only.
 * NOT COMPILED (ESP-IDF not installed). */
#include <math.h>
#include <string.h>

#include "board.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "fc_config.h"
#include "fc_core.h"
#include "fc_safety.h"
#include "fc_shared.h"
#include "imu.h"
#include "motor_out.h"

void fc_safety_config(fs_config_t *c) {
  fs_config_default(c);
  c->n_motors = FC_N_MOTORS;
  c->batt_cells = FC_BATT_CELLS;
  c->hover_thr_default = FC_HOVER_THR_DEFAULT;
  /* everything else: defaults in fc_safety.c (tilt 60 deg kill, pose 200 ms, heartbeat 250 ms, link 500 ms, ...) */
}

static fs_t s_fs; /* static: never on the stack, owned by this task only */

static void dispatch(const fc_msg_t *m) {
  switch (m->type) {
    case FC_MSG_POSE: fs_on_pose(&s_fs, m->t_ms, &m->u.pose); break;
    case FC_MSG_HB: fs_on_heartbeat(&s_fs, m->t_ms, m->u.hb.seq, m->u.hb.vio_state, m->u.hb.app_flags); break;
    case FC_MSG_LINK: fs_on_link(&s_fs, m->t_ms); break;
    case FC_MSG_CMD: fs_command(&s_fs, m->t_ms, (fs_cmd_t)m->u.cmd); break;
    case FC_MSG_PHONE_CONN: fs_on_phone_connection(&s_fs, m->t_ms, m->u.connected); break;
    case FC_MSG_KILL_REQ: fs_request_kill(&s_fs, m->t_ms, m->u.reason); break;
    case FC_MSG_LAND_REQ: fs_request_land(&s_fs, m->t_ms, m->u.reason); break;
    default: break;
  }
}

void ctrl_task(void *arg) {
  (void)arg;
  fs_config_t cfg;
  fc_safety_config(&cfg);
  fs_init(&s_fs, &cfg, fc_ms());
  fs_hover_rec_t prev;
  hover_store_load(&prev);
  fs_set_prev_hover(&s_fs, &prev);

  bool imu_present = imu_init(FC_CLOCK_IMU_DRDY ? (void *)xTaskGetCurrentTaskHandle() : NULL) == ESP_OK;

  att_est_t att;
  att_est_init(&att, 1.0f, 0.2f, 0.5f);
  att_ctrl_t actl;
  att_ctrl_init(&actl);
  vz_est_t vz;
  vz_est_init(&vz, 2.0f, 0.5f, 5.0f, 8.0f);

  esp_task_wdt_add(NULL);
  int64_t t_prev = esp_timer_get_time();
  float m_prev[FS_MAX_MOTORS] = {0};
  float yaw_hold = 0;
  uint32_t pose_seq_seen = 0, n = 0, imu_fail_run = 0, overruns = 0;
  uint16_t loop_max_us = 0;
  bool overrun = false;
  imu_sample_t im = {0};
  fs_state_t prev_state = FS_DISARMED;
  fs_ctrl_mode_t prev_mode = FS_CTRL_OFF;

  for (;;) {
    uint32_t woke = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3)); /* IMU DRDY / gptimer; timeout = clock missing */
    int64_t t0 = esp_timer_get_time();
    float dt = (float)(t0 - t_prev) * 1e-6f;
    t_prev = t0;
    if (dt <= 0.0f || dt > 0.02f) dt = 1.0f / FC_CTRL_HZ;
    uint32_t now = (uint32_t)(t0 / 1000);

    /* 1) IMU (a single bad read reuses the last sample; 3 in a row = IMU failure) */
    imu_sample_t s;
    bool rd = imu_present && woke && imu_read(&s);
    if (rd) { im = s; imu_fail_run = 0; } else imu_fail_run++;
    bool imu_ok = imu_present && imu_fail_run < 3;
    float R[9], roll = 0, pitch = 0, yaw = 0;
    if (imu_ok) {
      att_est_update(&att, im.gyro, im.acc, dt);
      att_est_rot(&att, R);
      att_est_euler(&att, &roll, &pitch, &yaw);
      vz_est_predict(&vz, R, im.acc, dt);
    }
    float gyro_n = sqrtf(im.gyro[0] * im.gyro[0] + im.gyro[1] * im.gyro[1] + im.gyro[2] * im.gyro[2]);
    float acc_n = sqrtf(im.acc[0] * im.acc[0] + im.acc[1] * im.acc[1] + im.acc[2] * im.acc[2]);

    /* 2) newest phone pose: vertical speed + yaw fusion */
    fc_pose_local_t pl;
    shared_get_pose(&pl);
    float pose_age_s = (float)(t0 - pl.t_rx_us) * 1e-6f + (float)pl.age_us * 1e-6f;
    bool vio_fresh = pl.seq != 0 && pl.good && pose_age_s < 0.1f;
    if (vio_fresh && pl.seq != pose_seq_seen) {
      vz_est_correct(&vz, -pl.v[2], 0.005f);
      if (imu_ok) att_est_fuse_yaw(&att, pl.yaw, 0.005f);
      pose_seq_seen = pl.seq;
    }

    /* 3) messages from the other tasks (pose / heartbeat / link / commands / requests) */
    fc_msg_t msg;
    while (xQueueReceive(g_fs_inbox, &msg, 0) == pdTRUE) dispatch(&msg);

    /* 4) safety */
    fc_att_sp_t sp;
    shared_get_att_sp(&sp);
    bool sp_fresh = sp.valid && (uint32_t)(now - sp.t_ms) < 50;
    fs_input_t in = {
        .now_ms = now, .dt_s = dt, .imu_ok = imu_ok, .acc_ok = imu_present && imu_fail_run < 3,
        .roll_rad = roll, .pitch_rad = pitch, .gyro_norm_rps = gyro_n, .acc_norm = acc_n,
        .thr_cmd = sp_fresh ? sp.thr : 0.0f,
        .vz_vio_valid = vio_fresh, .vz_vio = -pl.v[2],
        .vz_imu_valid = vz_est_valid(&vz), .vz_imu = vz.vz,
        .batt_valid = false, .batt_v = 0,
        .kill_switch = board_kill_switch() || g_kill_switch_isr,
        .loop_overrun = overrun,
    };
    float bv = g_batt_v; /* measured by the supervisor at 50 Hz */
    if (bv > 1.0f) { in.batt_valid = true; in.batt_v = bv; }
    for (int i = 0; i < FC_N_MOTORS; i++) in.motor_prev[i] = m_prev[i];
    if (s_fs.state == FS_FLYING && !sp_fresh) fs_request_land(&s_fs, now, FS_R_LOOP_STALL); /* position task dead */
    fs_output_t out;
    fs_step(&s_fs, &in, &out);

    /* 5) control per safety mode */
    float m[FS_MAX_MOTORS] = {0};
    if (out.mode != prev_mode && (out.mode == FS_CTRL_LAND_LEVEL || out.mode == FS_CTRL_LAND_HOLD_XY)) yaw_hold = yaw;
    switch (out.mode) {
      case FS_CTRL_NORMAL: {
        float t[3], rpy[3] = {sp.roll, sp.pitch, sp.yaw};
        att_ctrl_step(&actl, rpy, roll, pitch, yaw, im.gyro, dt, true, t);
        mixer_quad_x(sp.thr, t, FC_MOTOR_IDLE, m);
        break;
      }
      case FS_CTRL_LAND_HOLD_XY: { /* roll/pitch from the position loop holding land_xy; collective from fc_safety */
        float t[3], rpy[3] = {sp_fresh ? sp.roll : 0.0f, sp_fresh ? sp.pitch : 0.0f, yaw_hold};
        att_ctrl_step(&actl, rpy, roll, pitch, yaw, im.gyro, dt, true, t);
        mixer_quad_x(out.thr, t, FC_MOTOR_IDLE, m);
        break;
      }
      case FS_CTRL_LAND_LEVEL: {
        float t[3], rpy[3] = {0.0f, 0.0f, yaw_hold};
        att_ctrl_step(&actl, rpy, roll, pitch, yaw, im.gyro, dt, true, t);
        mixer_quad_x(out.thr, t, FC_MOTOR_IDLE, m);
        break;
      }
      case FS_CTRL_OPENLOOP: /* no attitude control at all: recorded per-motor hover * ramp (last resort) */
        for (int i = 0; i < FC_N_MOTORS; i++) m[i] = out.motor[i];
        break;
      case FS_CTRL_IDLE:
        for (int i = 0; i < FC_N_MOTORS; i++) m[i] = FC_MOTOR_IDLE;
        att_ctrl_reset(&actl);
        break;
      default:
        att_ctrl_reset(&actl);
        break;
    }

    /* 6) outputs: KILL / DISARM -> DShot 0 in this very cycle */
    if (out.motors_enabled && fs_check_outputs(&s_fs, now, m, FC_N_MOTORS)) {
      motor_out_write(m);
    } else {
      motor_out_disarm();
      memset(m, 0, sizeof(m));
    }
    if (s_fs.state == FS_DISARMED && motor_out_estopped() && !in.kill_switch) motor_out_clear_estop();
    if (s_fs.state == FS_DISARMED) g_kill_switch_isr = false; /* latch consumed (fs is KILLED -> cleared by command) */
    memcpy(m_prev, m, sizeof(m_prev));

    /* 7) events to the link task; hover record to RTC RAM when it becomes valid and at every disarm */
    fs_event_t ev;
    while (fs_pop_event(&s_fs, &ev)) {
      xQueueSend(g_ev_queue, &ev, 0);
      if (ev.code == FS_EV_HOVER_REC || (ev.code == FS_EV_TRANSITION && ev.to == FS_DISARMED)) {
        fs_hover_rec_t r;
        fs_get_hover_record(&s_fs, &r);
        if (r.valid) hover_store_save_rtc(&r);
      }
    }
    prev_state = s_fs.state;
    prev_mode = out.mode;

    /* 8) publish state (every 5th cycle = 200 Hz) */
    if ((n % 5) == 0) {
      fc_ctrl_state_t cs; /* written only by this task */
      memset(&cs, 0, sizeof(cs));
      cs.out = out;
      cs.batt_v = bv;
      cs.land_reason = s_fs.state == FS_KILLED ? s_fs.kill_reason : s_fs.land_reason;
      uint32_t hba = s_fs.hb_seen ? now - s_fs.t_hb_ok_ms : 65535u;
      cs.hb_age_ms = (uint16_t)(hba > 65535u ? 65535u : hba);
      cs.roll = roll; cs.pitch = pitch; cs.yaw = yaw;
      cs.vz_imu = vz.vz; cs.vz_imu_valid = vz_est_valid(&vz);
      cs.loop_max_us = loop_max_us;
      cs.overruns = overruns;
      shared_put_ctrl(&cs);
      if ((n % 1000) == 0) loop_max_us = 0;
    }

    /* 9) timing + watchdogs */
    uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
    if (us > loop_max_us) loop_max_us = (uint16_t)(us > 65535 ? 65535 : us);
    overrun = us > FC_CTRL_OVERRUN_US || !woke;
    if (overrun) overruns++;
    g_ctrl_alive++;
    if ((++n & 63) == 0) esp_task_wdt_reset();
    (void)prev_state;
  }
}
