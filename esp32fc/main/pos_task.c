/* pos_task.c — position loop (core 1, below the control task): woken by every new phone pose (link task notify) or at
 * FC_POS_HZ. Phone pose (IMU-forward-predicted VIO) -> NED position/velocity, extrapolated to "now" with the velocity
 * over the measured age -> cascaded P / PI controller -> roll, pitch, collective setpoint for the control task.
 * During LAND_VIO (FS_CTRL_LAND_HOLD_XY) it holds the x,y of the land start; the collective then comes from fc_safety.
 * NOT COMPILED (ESP-IDF not installed). */
#include <math.h>
#include <string.h>

#include "esp_timer.h"
#include "fc_config.h"
#include "fc_core.h"
#include "fc_shared.h"

void pos_task(void *arg) {
  (void)arg;
  pos_ctrl_t pc;
  pos_ctrl_init(&pc, FC_HOVER_THR_DEFAULT);
  int64_t t_prev = esp_timer_get_time();
  fs_ctrl_mode_t prev_mode = FS_CTRL_OFF;
  float hold[3] = {0}, land_xy[2] = {0}, yaw_sp = 0;
  bool have_hold = false;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000 / FC_POS_HZ));
    int64_t t = esp_timer_get_time();
    float dt = (float)(t - t_prev) * 1e-6f;
    t_prev = t;
    if (dt <= 0 || dt > 0.1f) dt = 1.0f / FC_POS_HZ;

    fc_pose_local_t pl;
    fc_ctrl_state_t cs;
    fc_setpoint_t sp;
    shared_get_pose(&pl);
    shared_get_ctrl(&cs);
    shared_get_setpoint(&sp);

    float age = (float)(t - pl.t_rx_us) * 1e-6f + (float)pl.age_us * 1e-6f;
    bool fresh = pl.seq != 0 && pl.good && age < 0.1f;
    float p[3], v[3];
    for (int i = 0; i < 3; i++) { v[i] = pl.v[i]; p[i] = pl.p[i] + pl.v[i] * age; } /* latency compensation */

    fs_ctrl_mode_t mode = cs.out.mode;
    fc_att_sp_t out = {.roll = 0, .pitch = 0, .yaw = cs.yaw, .thr = 0, .t_ms = (uint32_t)(t / 1000), .valid = true};

    if (mode == FS_CTRL_IDLE || mode == FS_CTRL_OFF) {
      pos_ctrl_reset(&pc);
      have_hold = false;
      yaw_sp = cs.yaw;
      /* takeoff request: a PC setpoint clearly above the current height (NED: smaller z) */
      if (mode == FS_CTRL_IDLE && fresh && sp.valid && sp.p[2] < p[2] - 0.2f) {
        float r, pt, th;
        pos_ctrl_step(&pc, p, v, sp.p, cs.yaw, NAN, dt, false, &r, &pt, &th);
        out.thr = th > 0.2f ? th : 0.2f; /* > thr_takeoff -> fc_safety goes FLYING */
      }
    } else if (mode == FS_CTRL_NORMAL && fresh) {
      if (!have_hold) { memcpy(hold, p, sizeof(hold)); have_hold = true; }
      const float *target = sp.valid ? sp.p : hold;
      if (sp.valid) yaw_sp = sp.yaw;
      pos_ctrl_step(&pc, p, v, target, cs.yaw, NAN, dt, true, &out.roll, &out.pitch, &out.thr);
      out.yaw = yaw_sp;
    } else if (mode == FS_CTRL_LAND_HOLD_XY && fresh) {
      if (prev_mode != FS_CTRL_LAND_HOLD_XY) { land_xy[0] = p[0]; land_xy[1] = p[1]; }
      float target[3] = {land_xy[0], land_xy[1], p[2]};
      float thr_unused;
      pos_ctrl_step(&pc, p, v, target, cs.yaw, cs.out.vz_target < 0 ? -cs.out.vz_target : 0.3f, dt, true, &out.roll,
                    &out.pitch, &thr_unused);
      out.thr = cs.out.thr;
    } else {
      /* no fresh pose: level, hover collective (fc_safety lands / has landed) */
      out.thr = mode == FS_CTRL_NORMAL ? pc.hover_thr : 0.0f;
    }
    prev_mode = mode;
    shared_put_att_sp(&out);
  }
}
